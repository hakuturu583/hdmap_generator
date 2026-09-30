//! Lookups across traces: from a written element to the IR, from the IR to a written
//! element, and from one format to another through the IR.

use std::collections::{BTreeMap, BTreeSet, HashMap};
use std::path::Path;

use quick_xml::events::Event;
use quick_xml::Reader;

use roadgen_core::trace::{IrRef, Relation, Trace};

use crate::error::TraceError;
use crate::file::{read_ir, TraceFile};
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
#[derive(Debug, Clone, PartialEq, Eq)]
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
    pub fn load(&mut self, path: impl AsRef<Path>) -> Result<(), TraceError> {
        let path = path.as_ref();
        if path.to_string_lossy().ends_with(".net.xml") {
            return self.load_sumo_net(path).map(|_| ());
        }
        let text = std::fs::read_to_string(path).map_err(|error| TraceError::io(path, error))?;
        let schema = serde_json::from_str::<serde_json::Value>(&text)
            .ok()
            .and_then(|value| value.get("schema")?.as_str().map(str::to_owned))
            .unwrap_or_default();
        if schema.starts_with("roadgen-ir/") {
            self.load_ir(path)
        } else {
            self.load_trace(path)
        }
    }

    pub fn load_ir(&mut self, path: impl AsRef<Path>) -> Result<(), TraceError> {
        let path = path.as_ref();
        let document = read_ir(path)?;
        self.agree(&path.display().to_string(), &document.fingerprint)?;
        self.ir = Some(document.body);
        Ok(())
    }

    /// Adds an IR dump already in memory.
    pub fn add_ir(&mut self, document: IrDocument) -> Result<(), TraceError> {
        document.check("the IR dump")?;
        self.agree("the IR dump", &document.fingerprint)?;
        self.ir = Some(document.body);
        Ok(())
    }

    pub fn load_trace(&mut self, path: impl AsRef<Path>) -> Result<(), TraceError> {
        let path = path.as_ref();
        let file = TraceFile::read(path)?;
        if self.check_files {
            file.verify_files(path)?;
        }
        self.add_trace_file(&path.display().to_string(), file)
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
        self.insert(&trace.format, links)
    }

    fn add_trace_file(&mut self, name: &str, file: TraceFile) -> Result<(), TraceError> {
        self.agree(name, &file.ir_fingerprint)?;
        let mut links = FormatLinks::default();
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
        let mut seen = BTreeSet::new();
        let answers = found
            .into_iter()
            .filter(|(link, _)| seen.insert((link.local.as_str(), link.ir.as_str())))
            .map(|(link, via)| Translation {
                local: link.local.clone(),
                ir: link.ir.clone(),
                relation: link.relation,
                role: link.role.clone(),
                via: via.map(str::to_owned),
            })
            .collect();
        Ok(answers)
    }

    /// The IR elements one step from `ir`, by the dump: what a lane belongs to and
    /// what joins it, what a connection joins, what a rule governs.
    pub fn neighbours(&self, ir: &str) -> Vec<String> {
        let Some(catalog) = &self.ir else {
            return Vec::new();
        };
        let mut out = Vec::new();
        match IrRef::parse(ir) {
            Some(IrRef::Lane(_)) => {
                if let Some(lane) = catalog.lanes.iter().find(|lane| lane.id == ir) {
                    out.push(lane.road.clone());
                }
                for connection in &catalog.connections {
                    if connection.from.lane == ir || connection.to.lane == ir {
                        out.push(connection.id.clone());
                    }
                }
            }
            Some(IrRef::Road(_)) => {
                out.extend(
                    catalog
                        .lanes
                        .iter()
                        .filter(|lane| lane.road == ir)
                        .map(|lane| lane.id.clone()),
                );
                if let Some(road) = catalog.roads.iter().find(|road| road.id == ir) {
                    out.extend(road.junction.clone());
                }
            }
            Some(IrRef::Connection(_)) => {
                if let Some(connection) = catalog.connections.iter().find(|c| c.id == ir) {
                    out.push(connection.from.lane.clone());
                    out.push(connection.to.lane.clone());
                    out.extend(connection.junction.clone());
                }
            }
            Some(IrRef::Junction(_)) => {
                if let Some(junction) = catalog.junctions.iter().find(|j| j.id == ir) {
                    out.extend(junction.connecting.iter().cloned());
                }
            }
            Some(IrRef::Object(_)) => {
                if let Some(object) = catalog.objects.iter().find(|o| o.id == ir) {
                    out.extend(object.lanes.iter().cloned());
                }
            }
            Some(IrRef::Rule(_)) => {
                if let Some(rule) = catalog.rules.iter().find(|r| r.id == ir) {
                    out.extend(rule.objects.iter().cloned());
                    out.extend(rule.lanes.iter().cloned());
                }
            }
            Some(IrRef::Building(_)) => {
                if let Some(building) = catalog.buildings.iter().find(|b| b.id == ir) {
                    out.extend(building.parts.iter().cloned());
                }
            }
            Some(IrRef::BuildingPart(_)) => {
                if let Some(building) = catalog
                    .buildings
                    .iter()
                    .find(|b| b.parts.iter().any(|part| part == ir))
                {
                    out.push(building.id.clone());
                }
            }
            None => {}
        }
        out
    }

    fn format(&self, format: &str) -> Result<&FormatLinks, TraceError> {
        self.formats
            .get(format)
            .ok_or_else(|| TraceError::Missing(format.to_owned()))
    }

    /// Reads the network netconvert built from a SUMO export, and traces the internal
    /// lanes it made.
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
    /// Needs the SUMO trace to be loaded first.
    pub fn load_sumo_net(&mut self, path: impl AsRef<Path>) -> Result<SumoNetReport, TraceError> {
        let path = path.as_ref();
        let text = std::fs::read_to_string(path).map_err(|error| TraceError::io(path, error))?;
        let connections = net_connections(&text)
            .map_err(|detail| TraceError::Parse(path.display().to_string(), detail))?;
        let sumo = self
            .formats
            .get_mut("sumo")
            .ok_or_else(|| TraceError::Missing("sumo".to_owned()))?;

        let mut report = SumoNetReport::default();
        // The IR elements each internal lane carries, filled from the connections that
        // enter a junction and then along the chains inside it.
        let mut carried: BTreeMap<String, Vec<Link>> = BTreeMap::new();
        for connection in connections.iter().filter(|c| !c.from.starts_with(':')) {
            let Some(via) = &connection.via else { continue };
            let local = format!(
                "connection:{}_{}>{}_{}",
                connection.from, connection.from_lane, connection.to, connection.to_lane
            );
            let links: Vec<Link> = sumo.with_local(&local).cloned().collect();
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
        Ok(report)
    }
}

struct NetConnection {
    from: String,
    to: String,
    from_lane: String,
    to_lane: String,
    via: Option<String>,
}

fn net_connections(text: &str) -> Result<Vec<NetConnection>, String> {
    let mut reader = Reader::from_str(text);
    let mut out = Vec::new();
    loop {
        match reader.read_event().map_err(|error| error.to_string())? {
            Event::Eof => break,
            Event::Start(element) | Event::Empty(element)
                if element.name().as_ref() == b"connection" =>
            {
                let mut attributes: HashMap<Vec<u8>, String> = HashMap::new();
                for attribute in element.attributes() {
                    let attribute = attribute.map_err(|error| error.to_string())?;
                    let value = attribute
                        .normalized_value(quick_xml::XmlVersion::Implicit1_0)
                        .map_err(|error| error.to_string())?;
                    attributes.insert(attribute.key.as_ref().to_vec(), value.into_owned());
                }
                let mut take = |key: &[u8]| attributes.remove(key);
                if let (Some(from), Some(to), Some(from_lane), Some(to_lane)) = (
                    take(b"from"),
                    take(b"to"),
                    take(b"fromLane"),
                    take(b"toLane"),
                ) {
                    out.push(NetConnection {
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
    Ok(out)
}
