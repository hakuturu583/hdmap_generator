//! Lookups across traces: from a written element to the IR, from the IR to a written
//! element, and from one format to another through the IR.

use std::collections::{BTreeMap, BTreeSet, HashMap, HashSet};
use std::path::{Path, PathBuf};

use quick_xml::events::Event;
use quick_xml::Reader;
use serde::Deserialize;

use roadgen_core::trace::{IrRef, Relation, Trace};

use crate::error::TraceError;
use crate::file::{parent_of, read_ir, read_ir_str, TraceFile};
use crate::ir::{IrCatalog, IrDocument};

/// One link, as a lookup returns it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Link {
    /// The IR element, printed: `lane/north/0`.
    pub ir: String,
    /// The written element: `lanelet:1000123`.
    pub local: String,
    pub relation: Relation,
    pub role: Option<String>,
}

/// One answer to [`TraceIndex::translate`].
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub struct Translation {
    /// The element of the target format.
    pub local: String,
    /// The IR element the answer went through.
    pub ir: String,
    /// How the target element relates to that IR element.
    pub relation: Relation,
    pub role: Option<String>,
    /// Set when the IR element the source came from had no counterpart in the target
    /// format, and the answer is for a neighbour of it instead — the road of a lane,
    /// the lanes of a connection. Names the element the lookup started from.
    pub via: Option<String>,
}

/// What reading a built SUMO network added to the SUMO trace.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct SumoNetReport {
    /// Internal lanes traced back to the IR.
    pub internal_lanes: usize,
    /// Pedestrian crossings traced back to the crosswalks they were written for.
    pub crossings: usize,
    /// Connections netconvert made that the export did not ask for, and that
    /// therefore have no IR counterpart. None for a network built with the export's
    /// own configuration; a U-turn at every dead end for one built without it, since
    /// netconvert adds those unless told not to.
    pub untraced: usize,
}

/// Everything loaded so far, indexed both ways.
#[derive(Debug, Default)]
pub struct TraceIndex {
    ir: Option<IrCatalog>,
    /// Every element of the dump's neighbours, worked out once when the dump is
    /// loaded: a translation steps to them for every element it cannot answer, and
    /// finding them in the dump each time is a scan of the whole of it.
    neighbours: HashMap<String, Vec<String>>,
    fingerprint: Option<String>,
    formats: BTreeMap<String, FormatLinks>,
    check_files: bool,
}

#[derive(Debug, Default)]
struct FormatLinks {
    links: Vec<Link>,
    by_local: HashMap<String, Vec<usize>>,
    by_ir: HashMap<String, Vec<usize>>,
    /// The kinds of written element this format has: `lanelet`, `linestring`.
    kinds: BTreeSet<String>,
    /// Where the files the trace describes are, when they are known: a SUMO
    /// network built from them is checked against its `.nod.xml`.
    files: Vec<PathBuf>,
}

impl FormatLinks {
    fn push(&mut self, link: Link) {
        if let Some((kind, _)) = link.local.split_once(':') {
            self.kinds.insert(kind.to_owned());
        }
        let index = self.links.len();
        self.by_local
            .entry(link.local.clone())
            .or_default()
            .push(index);
        self.by_ir.entry(link.ir.clone()).or_default().push(index);
        self.links.push(link);
    }

    fn with_local(&self, local: &str) -> impl Iterator<Item = &Link> {
        self.by_local
            .get(local)
            .into_iter()
            .flatten()
            .map(|&index| &self.links[index])
    }

    fn with_ir(&self, ir: &str) -> impl Iterator<Item = &Link> {
        self.by_ir
            .get(ir)
            .into_iter()
            .flatten()
            .map(|&index| &self.links[index])
    }
}

/// The kind a bare element is taken to be, for the formats whose elements are most
/// often named without one: a lanelet id, a SUMO lane id, an OSM way id.
fn default_kind(format: &str) -> Option<&'static str> {
    Some(match format {
        "lanelet2" => "lanelet",
        "osm" => "way",
        "sumo" => "lane",
        "opendrive" => "road",
        "clipgt" => "lane",
        "gpudrive" => "road",
        "carla" => "actor",
        _ => return None,
    })
}

impl TraceIndex {
    /// An empty index that checks, as each trace is loaded, that the files it
    /// describes are unchanged.
    pub fn new() -> Self {
        TraceIndex {
            check_files: true,
            ..TraceIndex::default()
        }
    }

    /// Whether loading a trace checks the digests of the files it describes. On by
    /// default; off is for a trace whose files are not at hand.
    pub fn check_files(mut self, check: bool) -> Self {
        self.check_files = check;
        self
    }

    /// Loads an IR dump or a trace file, telling which by its schema, or a SUMO
    /// `.net.xml` by its name.
    ///
    /// The file is read once, and the text that told which it is is the text parsed.
    pub fn load(&mut self, path: impl AsRef<Path>) -> Result<(), TraceError> {
        /// All of a file that is needed to tell which it is.
        #[derive(Deserialize)]
        struct Head {
            schema: String,
        }
        let path = path.as_ref();
        if path.to_string_lossy().ends_with(".net.xml") {
            return self.load_sumo_net(path).map(|_| ());
        }
        let text = crate::file::read_text(path)?;
        let schema = serde_json::from_str::<Head>(&text)
            .map(|head| head.schema)
            .unwrap_or_default();
        let name = path.display().to_string();
        if schema.starts_with("roadgen-ir/") {
            self.add_ir_as(&name, read_ir_str(&text, &name)?)
        } else {
            self.add_trace_read(path, TraceFile::parse(&text, &name)?)
        }
    }

    pub fn load_ir(&mut self, path: impl AsRef<Path>) -> Result<(), TraceError> {
        let path = path.as_ref();
        self.add_ir_as(&path.display().to_string(), read_ir(path)?)
    }

    /// Adds an IR dump already in memory.
    pub fn add_ir(&mut self, document: IrDocument) -> Result<(), TraceError> {
        document.check("the IR dump")?;
        self.add_ir_as("the IR dump", document)
    }

    /// Adds a checked dump, called `name` in any complaint.
    fn add_ir_as(&mut self, name: &str, document: IrDocument) -> Result<(), TraceError> {
        self.agree(name, &document.fingerprint)?;
        self.neighbours = neighbour_table(&document.body);
        self.ir = Some(document.body);
        Ok(())
    }

    pub fn load_trace(&mut self, path: impl AsRef<Path>) -> Result<(), TraceError> {
        let path = path.as_ref();
        self.add_trace_read(path, TraceFile::read(path)?)
    }

    /// Adds `file`, as read from `path`.
    fn add_trace_read(&mut self, path: &Path, file: TraceFile) -> Result<(), TraceError> {
        if self.check_files {
            file.verify_files(path)?;
        }
        self.add_trace_file(&path.display().to_string(), file, &parent_of(path))
    }

    /// Adds a trace already in memory, of the map `ir_fingerprint` identifies.
    pub fn add_trace(&mut self, trace: &Trace, ir_fingerprint: &str) -> Result<(), TraceError> {
        self.agree(&format!("the {} trace", trace.format), ir_fingerprint)?;
        let mut links = FormatLinks::default();
        for link in &trace.links {
            links.push(Link {
                ir: link.ir.to_string(),
                local: link.local.clone(),
                relation: link.relation,
                role: link.role.clone(),
            });
        }
        links.files = trace.files.clone();
        self.insert(&trace.format, links)
    }

    fn add_trace_file(
        &mut self,
        name: &str,
        file: TraceFile,
        base: &Path,
    ) -> Result<(), TraceError> {
        self.agree(name, &file.ir_fingerprint)?;
        let mut links = FormatLinks {
            files: file
                .files
                .iter()
                .map(|described| base.join(&described.path))
                .collect(),
            ..FormatLinks::default()
        };
        for link in file.links {
            links.push(Link {
                relation: Relation::parse(&link.rel).expect("checked when read"),
                ir: link.ir,
                local: link.local,
                role: link.role,
            });
        }
        self.insert(&file.format, links)
    }

    fn insert(&mut self, format: &str, links: FormatLinks) -> Result<(), TraceError> {
        if self.formats.contains_key(format) {
            return Err(TraceError::Duplicate(format.to_owned()));
        }
        self.formats.insert(format.to_owned(), links);
        Ok(())
    }

    /// Every file loaded has to come from the same map as the first.
    fn agree(&mut self, name: &str, fingerprint: &str) -> Result<(), TraceError> {
        match &self.fingerprint {
            Some(expected) if expected != fingerprint => Err(TraceError::Mismatch {
                path: name.to_owned(),
                found: fingerprint.to_owned(),
                expected: expected.clone(),
            }),
            Some(_) => Ok(()),
            None => {
                self.fingerprint = Some(fingerprint.to_owned());
                Ok(())
            }
        }
    }

    /// The formats loaded, in name order.
    pub fn formats(&self) -> Vec<&str> {
        self.formats.keys().map(String::as_str).collect()
    }

    /// The IR dump, when one was loaded.
    pub fn ir(&self) -> Option<&IrCatalog> {
        self.ir.as_ref()
    }

    /// `element` with its kind, as the traces of `format` write it.
    ///
    /// `1000123` in Lanelet2 is `lanelet:1000123`; `north.fwd_0` in SUMO is
    /// `lane:north.fwd_0`, and so is `:j_x_0_0`, whose leading colon is SUMO's own
    /// and not a kind. An element that already starts with a kind the format has is
    /// left alone.
    pub fn qualify(&self, format: &str, element: &str) -> String {
        if let Some((kind, _)) = element.split_once(':') {
            let known = self
                .formats
                .get(format)
                .is_some_and(|links| links.kinds.contains(kind));
            if known {
                return element.to_owned();
            }
        }
        match default_kind(format) {
            Some(kind) => format!("{kind}:{element}"),
            None => element.to_owned(),
        }
    }

    /// The IR elements `element` of `format` was written from.
    pub fn to_ir(&self, format: &str, element: &str) -> Result<Vec<&Link>, TraceError> {
        let local = self.qualify(format, element);
        Ok(self.format(format)?.with_local(&local).collect())
    }

    /// What `ir` was written as in `format`.
    pub fn from_ir(&self, ir: &str, format: &str) -> Result<Vec<&Link>, TraceError> {
        Ok(self.format(format)?.with_ir(ir).collect())
    }

    /// The elements of `to` that `element` of `from` corresponds to.
    ///
    /// Through the IR: `element` back to the IR elements it came from, and each of
    /// those forward into `to`. When none of them has a counterpart in `to` — a lane
    /// in OpenStreetMap, which writes roads — they are replaced by their neighbours
    /// in the IR dump, one step out, and the answers say so in
    /// [`via`](Translation::via). Only then: an element written from several IR
    /// elements, some of which `to` has, is answered by those alone, rather than
    /// padded with the surroundings of the rest. Without a dump there is no such
    /// step.
    ///
    /// Answers come as a set, never as the one answer: a lanelet boundary is two
    /// lanes, and a road is two SUMO edges.
    pub fn translate(
        &self,
        from: &str,
        element: &str,
        to: &str,
    ) -> Result<Vec<Translation>, TraceError> {
        let target = self.format(to)?;
        let sources = self.to_ir(from, element)?;
        // (target element, IR element it came from, the element stepped from)
        let mut found: Vec<(&Link, Option<&str>)> = sources
            .iter()
            .flat_map(|source| target.with_ir(&source.ir).map(|link| (link, None)))
            .collect();
        if found.is_empty() {
            for source in &sources {
                for neighbour in self.neighbours(&source.ir) {
                    found.extend(
                        target
                            .with_ir(&neighbour)
                            .map(|link| (link, Some(source.ir.as_str()))),
                    );
                }
            }
        }
        let mut answers: Vec<Translation> = found
            .into_iter()
            .map(|(link, via)| Translation {
                local: link.local.clone(),
                ir: link.ir.clone(),
                relation: link.relation,
                role: link.role.clone(),
                via: via.map(str::to_owned),
            })
            .collect();
        // An answer is dropped only when identical to an earlier one: one element in
        // two roles — the lane an agent drives and the lane its goal is on — or
        // reached from two IR elements through one neighbour is two answers.
        let mut seen = HashSet::new();
        answers.retain(|answer| seen.insert(answer.clone()));
        Ok(answers)
    }

    /// The IR elements one step from `ir`, by the dump: what a lane belongs to and
    /// what joins it, what a connection joins, what a rule governs.
    pub fn neighbours(&self, ir: &str) -> Vec<String> {
        self.neighbours.get(ir).cloned().unwrap_or_default()
    }

    fn format(&self, format: &str) -> Result<&FormatLinks, TraceError> {
        self.formats
            .get(format)
            .ok_or_else(|| TraceError::Missing(format.to_owned()))
    }

    /// Reads the network netconvert built from a SUMO export, and traces the internal
    /// lanes and the pedestrian crossings it made.
    ///
    /// The export cannot name them: an internal lane is netconvert's, drawn across a
    /// junction when the network is built. But the built network says which
    /// connection each one carries — `<connection from to fromLane toLane via>` —
    /// and `from`, `to` and the lane indices are the export's own, so the connection
    /// is in the SUMO trace, and the internal lane is linked to whatever that
    /// connection was written from. A movement that stops inside the junction, as a
    /// turn across oncoming traffic does, gets a second internal lane, which the
    /// built network reaches from the first; both are linked.
    ///
    /// Needs the SUMO trace to be loaded first, and refuses a network that was not
    /// built from the export it describes. A built network records nothing of the
    /// files it was built from, and a network of another map can share every name —
    /// `north.fwd`, `j_x` — so the check is on what the two must agree on: the same
    /// edges and lanes, every connection the export wrote, and every node of the
    /// export's `.nod.xml` where the network put its junction.
    ///
    /// A crossing is netconvert's too: the export writes a `<crossing node edges>`,
    /// which has no id, and netconvert builds it as an edge `:<node>_c<n>` numbered in
    /// its own order. The built edge records the edges it crosses (`crossingEdges`),
    /// so the node and those edges find the crossing the export wrote, and the
    /// crossing's lane is linked to the crosswalk behind it with the role
    /// `crossing`. A crossing the export wrote that the network does not have —
    /// netconvert discards one it cannot build — makes the network not the export's.
    pub fn load_sumo_net(&mut self, path: impl AsRef<Path>) -> Result<SumoNetReport, TraceError> {
        let path = path.as_ref();
        let text = crate::file::read_text(path)?;
        let net = parse_net(&text)
            .map_err(|detail| TraceError::Parse(path.display().to_string(), detail))?;
        let sumo = self
            .formats
            .get_mut("sumo")
            .ok_or_else(|| TraceError::Missing("sumo".to_owned()))?;
        built_from(sumo, &net).map_err(|reason| TraceError::Foreign {
            path: path.display().to_string(),
            reason,
        })?;
        let connections = net.connections;
        let crossings = net.crossings;

        let mut report = SumoNetReport::default();
        // The IR elements each internal lane carries, filled from the connections that
        // enter a junction and then along the chains inside it.
        let mut carried: BTreeMap<String, Vec<Link>> = BTreeMap::new();
        for connection in connections.iter().filter(|c| !c.from.starts_with(':')) {
            let Some(via) = &connection.via else { continue };
            let links: Vec<Link> = sumo.with_local(&connection.key()).cloned().collect();
            if links.is_empty() {
                report.untraced += 1;
            } else {
                carried.insert(via.clone(), links);
            }
        }
        loop {
            let mut grew = false;
            for connection in connections.iter().filter(|c| c.from.starts_with(':')) {
                let Some(via) = &connection.via else { continue };
                let lane = format!("{}_{}", connection.from, connection.from_lane);
                if carried.contains_key(via) {
                    continue;
                }
                if let Some(links) = carried.get(&lane).cloned() {
                    carried.insert(via.clone(), links);
                    grew = true;
                }
            }
            if !grew {
                break;
            }
        }
        for (lane, links) in carried {
            report.internal_lanes += 1;
            for link in links {
                sumo.push(Link {
                    ir: link.ir,
                    local: format!("lane:{lane}"),
                    relation: Relation::Part,
                    role: Some("internal".to_owned()),
                });
            }
        }
        for crossing in &crossings {
            let links: Vec<Link> = sumo.with_local(&crossing.key()).cloned().collect();
            if !links.is_empty() {
                report.crossings += 1;
            }
            for lane in &crossing.lanes {
                for link in &links {
                    sumo.push(Link {
                        ir: link.ir.clone(),
                        local: format!("lane:{lane}"),
                        relation: link.relation,
                        role: Some("crossing".to_owned()),
                    });
                }
            }
        }
        Ok(report)
    }
}

/// The IR elements one step from each element of `catalog`, for
/// [`TraceIndex::neighbours`]: a lane's road and the connections that join it, a
/// road's lanes and its junction, a connection's two lanes and its junction, a
/// junction's connecting roads, an object's lanes, a rule's objects and then its
/// lanes, a building's parts, a part's building.
///
/// An element is taken by the kind its id names, as [`IrRef::parse`] reads it, and
/// where two entries share an id the first is the one described — as a search of
/// the dump for that id would find.
fn neighbour_table(catalog: &IrCatalog) -> HashMap<String, Vec<String>> {
    let is = |id: &str, kind: fn(&IrRef) -> bool| IrRef::parse(id).as_ref().is_some_and(kind);
    let lane = |id: &str| is(id, |r| matches!(r, IrRef::Lane(_)));
    let road = |id: &str| is(id, |r| matches!(r, IrRef::Road(_)));

    let mut table: HashMap<String, Vec<String>> = HashMap::new();
    for entry in &catalog.lanes {
        if lane(&entry.id) {
            table
                .entry(entry.id.clone())
                .or_insert_with(|| vec![entry.road.clone()]);
        }
        if road(&entry.road) {
            table
                .entry(entry.road.clone())
                .or_default()
                .push(entry.id.clone());
        }
    }
    for connection in &catalog.connections {
        let (from, to) = (&connection.from.lane, &connection.to.lane);
        if lane(from) {
            table
                .entry(from.clone())
                .or_default()
                .push(connection.id.clone());
        }
        if to != from && lane(to) {
            table
                .entry(to.clone())
                .or_default()
                .push(connection.id.clone());
        }
        if is(&connection.id, |r| matches!(r, IrRef::Connection(_))) {
            table.entry(connection.id.clone()).or_insert_with(|| {
                let mut out = vec![from.clone(), to.clone()];
                out.extend(connection.junction.clone());
                out
            });
        }
    }
    // After the lanes, whose entries a road's junction follows.
    let mut roads = HashSet::new();
    for entry in &catalog.roads {
        if road(&entry.id) && roads.insert(entry.id.as_str()) {
            table
                .entry(entry.id.clone())
                .or_default()
                .extend(entry.junction.clone());
        }
    }
    for junction in &catalog.junctions {
        if is(&junction.id, |r| matches!(r, IrRef::Junction(_))) {
            table
                .entry(junction.id.clone())
                .or_insert_with(|| junction.connecting.clone());
        }
    }
    for object in &catalog.objects {
        if is(&object.id, |r| matches!(r, IrRef::Object(_))) {
            table
                .entry(object.id.clone())
                .or_insert_with(|| object.lanes.clone());
        }
    }
    for rule in &catalog.rules {
        if is(&rule.id, |r| matches!(r, IrRef::Rule(_))) {
            table
                .entry(rule.id.clone())
                .or_insert_with(|| [rule.objects.clone(), rule.lanes.clone()].concat());
        }
    }
    for building in &catalog.buildings {
        if is(&building.id, |r| matches!(r, IrRef::Building(_))) {
            table
                .entry(building.id.clone())
                .or_insert_with(|| building.parts.clone());
        }
        for part in &building.parts {
            if is(part, |r| matches!(r, IrRef::BuildingPart(_))) {
                table
                    .entry(part.clone())
                    .or_insert_with(|| vec![building.id.clone()]);
            }
        }
    }
    table
}

/// Whether `net` is the network netconvert built from the export `sumo` traces.
fn built_from(sumo: &FormatLinks, net: &Net) -> Result<(), String> {
    let written = |kind: &str| -> BTreeSet<&str> {
        sumo.links
            .iter()
            .filter_map(|link| link.local.strip_prefix(kind))
            // An internal lane traced from an earlier network is not the export's.
            .filter(|local| !local.starts_with(':'))
            .collect()
    };
    let edges: BTreeSet<&str> = net.edges.iter().map(String::as_str).collect();
    if written("edge:") != edges {
        return Err("its edges are not the ones the export wrote".into());
    }
    let lanes: BTreeSet<&str> = net.lanes.iter().map(String::as_str).collect();
    if written("lane:") != lanes {
        return Err("its lanes are not the ones the export wrote".into());
    }
    let connections: BTreeSet<String> = net
        .connections
        .iter()
        .filter(|c| !c.from.starts_with(':'))
        .map(NetConnection::key)
        .collect();
    if let Some(missing) = written("connection:")
        .into_iter()
        .find(|connection| !connections.contains(&format!("connection:{connection}")))
    {
        return Err(format!(
            "it has no connection {missing}, which the export wrote"
        ));
    }
    let crossings: BTreeSet<String> = net.crossings.iter().map(NetCrossing::key).collect();
    if let Some(missing) = written("crossing:")
        .into_iter()
        .find(|crossing| !crossings.contains(&format!("crossing:{crossing}")))
    {
        return Err(format!(
            "it has no crossing {missing}, which the export wrote"
        ));
    }
    let nodes = sumo
        .files
        .iter()
        .find(|file| file.to_string_lossy().ends_with(".nod.xml"));
    if let Some(nodes) = nodes {
        let text = std::fs::read_to_string(nodes)
            .map_err(|error| format!("{}: {error}", nodes.display()))?;
        let (declared, nodes) = parse_nodes(&text)?;
        // The network's coordinates are the export's moved by however far netconvert
        // shifted them, and rounded to the centimetre. The shift is the difference
        // between the network's offset and the one the node file's own `<location>`
        // declared — that part of the offset is the geo-reference, which netconvert
        // carries through without moving anything — so it is zero for an export
        // built with normalisation off, whatever its projection.
        let shift = (net.offset.0 - declared.0, net.offset.1 - declared.1);
        for (id, (x, y)) in nodes {
            let Some(&(net_x, net_y)) = net.junctions.get(&id) else {
                return Err(format!("it has no junction {id}, which the export wrote"));
            };
            let (net_x, net_y) = (net_x - shift.0, net_y - shift.1);
            if (net_x - x).hypot(net_y - y) > NODE_TOLERANCE {
                return Err(format!(
                    "its junction {id} is at ({net_x:.2}, {net_y:.2}), not at ({x:.2}, {y:.2}) \
                     where the export put it"
                ));
            }
        }
    }
    Ok(())
}

/// How far a built junction may sit from the node it was built from: netconvert
/// writes coordinates to the centimetre.
const NODE_TOLERANCE: f64 = 0.02;

/// What a built network says that the trace can be checked against.
#[derive(Default)]
struct Net {
    connections: Vec<NetConnection>,
    /// The pedestrian crossings netconvert built.
    crossings: Vec<NetCrossing>,
    /// The edges that are not internal: the export's.
    edges: BTreeSet<String>,
    /// Their lanes.
    lanes: BTreeSet<String>,
    junctions: HashMap<String, Position>,
    offset: Position,
}

struct NetConnection {
    from: String,
    to: String,
    from_lane: String,
    to_lane: String,
    via: Option<String>,
}

impl NetConnection {
    /// The connection as a trace names it, `connection:<from>_<fromLane>><to>_<toLane>`:
    /// the form the SUMO exporter writes, so a built connection is found in the trace
    /// by its key.
    fn key(&self) -> String {
        format!(
            "connection:{}_{}>{}_{}",
            self.from, self.from_lane, self.to, self.to_lane
        )
    }
}

/// A pedestrian crossing of a built network: the node it is at, the edges it
/// crosses, and its lanes.
struct NetCrossing {
    node: String,
    edges: Vec<String>,
    lanes: Vec<String>,
}

impl NetCrossing {
    /// The crossing as a trace names it, `crossing:<node>/<edge>+<edge>` with the
    /// edges sorted: the form the SUMO exporter writes, where netconvert lists the
    /// same edges in an order of its own.
    fn key(&self) -> String {
        let mut edges = self.edges.clone();
        edges.sort();
        format!("crossing:{}/{}", self.node, edges.join("+"))
    }
}

/// A plan-view position, metres.
type Position = (f64, f64);

/// An XML element's attributes, by name.
type Attributes = HashMap<Vec<u8>, String>;

fn attributes(element: &quick_xml::events::BytesStart) -> Result<Attributes, String> {
    let mut attributes = HashMap::new();
    for attribute in element.attributes() {
        let attribute = attribute.map_err(|error| error.to_string())?;
        let value = attribute
            .normalized_value(quick_xml::XmlVersion::Implicit1_0)
            .map_err(|error| error.to_string())?;
        attributes.insert(attribute.key.as_ref().to_vec(), value.into_owned());
    }
    Ok(attributes)
}

fn coordinate(attributes: &Attributes, key: &[u8]) -> Result<f64, String> {
    attributes
        .get(key)
        .and_then(|value| value.parse().ok())
        .ok_or_else(|| format!("a missing or unreadable {}", String::from_utf8_lossy(key)))
}

fn parse_net(text: &str) -> Result<Net, String> {
    let mut reader = Reader::from_str(text);
    let mut net = Net::default();
    // The edge whose lanes are being read, when it is one of the export's.
    let mut edge: Option<String> = None;
    // Whether the edge being read is a crossing, whose lanes go to the last of
    // `net.crossings`.
    let mut crossing = false;
    loop {
        let (element, empty) = match reader.read_event().map_err(|error| error.to_string())? {
            Event::Eof => break,
            Event::End(end) => {
                if end.name().as_ref() == b"edge" {
                    edge = None;
                    crossing = false;
                }
                continue;
            }
            Event::Start(element) => (element, false),
            Event::Empty(element) => (element, true),
            _ => continue,
        };
        let mut attributes = attributes(&element)?;
        match element.name().as_ref() {
            b"location" => {
                if let Some(offset) = attributes.get(b"netOffset".as_slice()) {
                    net.offset = net_offset(offset)?;
                }
            }
            b"edge" => {
                // Any edge with a `function` is netconvert's own: an `internal` lane
                // across a junction, and — where the export has footways — the
                // `walkingarea` joining them at a node, or a `crossing`. The export's
                // edges are the ones without.
                let generated = attributes.contains_key(b"function".as_slice());
                let id = attributes.remove(b"id".as_slice()).unwrap_or_default();
                if attributes.get(b"function".as_slice()).map(String::as_str) == Some("crossing") {
                    // netconvert names a crossing `:<node>_c<n>`.
                    let node = id
                        .strip_prefix(':')
                        .and_then(|rest| rest.rsplit_once("_c"))
                        .map(|(node, _)| node.to_owned())
                        .unwrap_or_default();
                    let edges = attributes
                        .get(b"crossingEdges".as_slice())
                        .map(|edges| edges.split_whitespace().map(str::to_owned).collect())
                        .unwrap_or_default();
                    net.crossings.push(NetCrossing {
                        node,
                        edges,
                        lanes: Vec::new(),
                    });
                    crossing = !empty;
                }
                if !generated {
                    net.edges.insert(id.clone());
                    if !empty {
                        edge = Some(id);
                    }
                }
            }
            b"lane" if edge.is_some() => {
                if let Some(id) = attributes.remove(b"id".as_slice()) {
                    net.lanes.insert(id);
                }
            }
            b"lane" if crossing => {
                if let (Some(id), Some(last)) = (
                    attributes.remove(b"id".as_slice()),
                    net.crossings.last_mut(),
                ) {
                    last.lanes.push(id);
                }
            }
            b"junction" => {
                if attributes.get(b"type".as_slice()).map(String::as_str) != Some("internal") {
                    let position = (
                        coordinate(&attributes, b"x")?,
                        coordinate(&attributes, b"y")?,
                    );
                    if let Some(id) = attributes.remove(b"id".as_slice()) {
                        net.junctions.insert(id, position);
                    }
                }
            }
            b"connection" => {
                let mut take = |key: &[u8]| attributes.remove(key);
                if let (Some(from), Some(to), Some(from_lane), Some(to_lane)) = (
                    take(b"from"),
                    take(b"to"),
                    take(b"fromLane"),
                    take(b"toLane"),
                ) {
                    net.connections.push(NetConnection {
                        from,
                        to,
                        from_lane,
                        to_lane,
                        via: take(b"via"),
                    });
                }
            }
            _ => {}
        }
    }
    Ok(net)
}

/// The offset a plain `.nod.xml` declares in its `<location>` — zero when it has
/// none — and its nodes, by id.
fn parse_nodes(text: &str) -> Result<(Position, Vec<(String, Position)>), String> {
    let mut reader = Reader::from_str(text);
    let mut offset = (0.0, 0.0);
    let mut nodes = Vec::new();
    loop {
        match reader.read_event().map_err(|error| error.to_string())? {
            Event::Eof => break,
            Event::Start(element) | Event::Empty(element)
                if element.name().as_ref() == b"location" =>
            {
                if let Some(declared) = attributes(&element)?.get(b"netOffset".as_slice()) {
                    offset = net_offset(declared)?;
                }
            }
            Event::Start(element) | Event::Empty(element) if element.name().as_ref() == b"node" => {
                let attributes = attributes(&element)?;
                let position = (
                    coordinate(&attributes, b"x")?,
                    coordinate(&attributes, b"y")?,
                );
                if let Some(id) = attributes.get(b"id".as_slice()) {
                    nodes.push((id.clone(), position));
                }
            }
            _ => {}
        }
    }
    Ok((offset, nodes))
}

/// A `netOffset`: `x,y`, or `x,y,z` with the height ignored.
fn net_offset(value: &str) -> Result<Position, String> {
    let unreadable = || format!("an unreadable netOffset {value}");
    let mut parts = value.split(',').map(|part| part.trim().parse::<f64>());
    match (parts.next(), parts.next()) {
        (Some(Ok(x)), Some(Ok(y))) => Ok((x, y)),
        _ => Err(unreadable()),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_node_file_declares_the_offset_of_its_location_and_no_other() {
        let georeferenced = r#"<nodes>
            <location netOffset="-386000.25,-3950000.50" convBoundary="0,0,1,1"
                      origBoundary="0,0,1,1" projParameter="+proj=utm +zone=54"/>
            <node id="a" x="1.5" y="-2.0"/>
        </nodes>"#;
        let (offset, nodes) = parse_nodes(georeferenced).unwrap();
        assert_eq!(offset, (-386000.25, -3950000.5));
        assert_eq!(nodes, vec![("a".to_owned(), (1.5, -2.0))]);

        let plain = r#"<nodes><node id="a" x="1.5" y="-2.0"/></nodes>"#;
        assert_eq!(parse_nodes(plain).unwrap().0, (0.0, 0.0));
    }

    #[test]
    fn a_net_offset_may_carry_a_height() {
        assert_eq!(net_offset("1.5,-2").unwrap(), (1.5, -2.0));
        assert_eq!(net_offset("1.5, -2, 30").unwrap(), (1.5, -2.0));
        assert!(net_offset("1.5").is_err());
    }
}
