//! `roadgen-sumo` — writes the canonical IR as a SUMO network.
//!
//! # Which SUMO format this is
//!
//! SUMO's simulator reads a `.net.xml`, and a `.net.xml` is a *build product*: it
//! carries the shape of every junction, the internal lane through every movement and
//! the right-of-way matrix that decides who waits for whom, all of it computed by
//! `netconvert`. Writing one directly would mean reimplementing netconvert, and
//! getting it subtly wrong.
//!
//! So what is written here is netconvert's own input — the **plain XML** network:
//!
//! | file | what is in it |
//! | --- | --- |
//! | `<name>.nod.xml` | junctions and road ends, as points |
//! | `<name>.edg.xml` | one edge per direction of travel, with its lanes |
//! | `<name>.con.xml` | which lane may be left for which lane |
//! | `<name>.netccfg` | the netconvert run that turns the three into a `.net.xml` |
//!
//! ```text
//! netconvert -c <name>.netccfg
//! ```
//!
//! That is the format a network *generator* is expected to produce, and it is the
//! one SUMO documents for the purpose.
//!
//! # What a SUMO edge is
//!
//! A one-way bundle of lanes. A road carrying traffic both ways is therefore two
//! edges, pointing at each other, and the IR's reference line — which has no
//! direction as far as traffic is concerned — becomes the thing they are both
//! measured against. Lanes within an edge are numbered from the **right** in the
//! direction of travel, which is index 0, so the numbering of the same physical lane
//! differs between the two carriageways. That is SUMO's convention and not a choice
//! made here.
//!
//! A road whose cross-section changes becomes one edge per cross-section, joined at
//! an internal node: a SUMO edge has one lane count from end to end.
//!
//! Every lane is written with its own shape, so the geometry the generator computed
//! is the geometry SUMO gets — not a centreline with a width, which is what a
//! network imported from OpenStreetMap has to make do with.
//!
//! # Junctions
//!
//! The IR draws a junction as a set of connector roads, one per movement. SUMO draws
//! it as a *node*: the arms stop at its edge, and netconvert generates an internal
//! lane for every connection across it. The two models agree about what matters —
//! the enumerated movements — so the connectors are not written as edges. Each one
//! becomes the `<connection>` that says its approach lane may be left for its exit
//! lane, and netconvert draws the path.
//!
//! This is why the arms are left exactly where the IR puts them, short of the
//! junction: the gap is the junction, and netconvert fills it.
//!
//! # The trace
//!
//! [`PlainNetwork::trace`] records where each element of the IR went, naming the
//! written elements as follows:
//!
//! | written element | IR element | relation |
//! | --- | --- | --- |
//! | `node:<id>` | a junction | exact |
//! | `node:<id>` | a traffic light, role `traffic_light` | merged |
//! | `edge:<id>` | a road — one edge per direction and cross-section | part |
//! | `lane:<edge>_<index>` | a lane | exact |
//! | `connection:<from edge>_<from lane>><to edge>_<to lane>` | a lane connection | exact, or merged |
//! | the same connection | a connector lane it runs over | collapsed |
//! | `lane:<edge>_<index>` | a stop line, role `stopOffset` | merged |
//!
//! A connection is named by its two lanes as the built network names them, so the
//! `.net.xml` `<connection from fromLane to toLane via>` that netconvert writes for it
//! is found from the trace by the same `<edge>_<index>` pair — and with it the
//! internal lane (`via`) netconvert generated in place of the connector. A movement
//! through a junction is two IR connections and one connector lane, all of them
//! written as the one connection, so each is `merged` or `collapsed` into it.
//!
//! # Stop lines
//!
//! SUMO has no stop line as a thing of its own. What it has is a lane's
//! `<stopOffset>`: how far short of the end of the lane a vehicle that has to wait at
//! the junction ahead comes to a halt. Without one it halts at the very end of the
//! lane, which for an arm that stops where the IR's junction begins is the junction's
//! mouth — not the line painted some metres before it.
//!
//! So each stop line is measured along every written lane it crosses, from where it
//! crosses the lane's centreline to the lane's end in the direction of travel, and
//! that distance is written as the lane's stop offset. It is only meaningful where
//! the lane runs into a junction — the end of an edge at a cross-section boundary or
//! at a joint with the next road is not a place anyone waits — so a lane split over
//! several edges carries it on the edge that reaches the junction, and only there. A
//! stop line SUMO has no place for — on a lane that is not written, short of the end
//! of a lane that does not meet a junction, further back than the lane is long — is
//! named by [`check`] rather than dropped without a word, lane by lane, so a line
//! written on some of its lanes is reported only for the ones it misses.

pub mod classes;
pub mod error;
mod xml;

use std::collections::{BTreeMap, BTreeSet, HashMap, HashSet};
use std::path::Path;

use roadgen_core::geometry::{Point3, Polyline3, SamplingConfig};
use roadgen_core::map::{Lane, Road};
use roadgen_core::semantics::{MapObjectKind, ObjectGeometry, TrafficRule};
use roadgen_core::topology::{Direction, LaneEnd, RoadEnd, RoadLinkTarget};
use roadgen_core::trace::{IrRef, Relation, Trace};
use roadgen_core::{ConnectionId, JunctionId, LaneId, ObjectId, RoadId, ValidatedMap};

pub use classes::Permission;
pub use error::ExportError;

/// The name a map with none of its own is written under.
const DEFAULT_NAME: &str = "network";

/// The shortest stop offset worth writing, metres.
///
/// A stop line this close to the end of its lane is, to any precision a simulation
/// cares about, at the end of it — which is where SUMO stops a vehicle with no
/// offset at all. It is also the margin kept below an edge's length, which
/// netconvert refuses an offset to reach.
const MIN_STOP_OFFSET: f64 = 0.1;

/// A SUMO plain-XML network: the three input files, and the netconvert run that
/// turns them into a `.net.xml`.
///
/// The file names are derived from [`PlainNetwork::prefix`], and the configuration
/// refers to them, so the four belong together in one directory.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PlainNetwork {
    /// What the files are named before their `.nod.xml`, `.edg.xml`, `.con.xml` and
    /// `.netccfg` suffixes.
    pub prefix: String,
    pub nodes: String,
    pub edges: String,
    pub connections: String,
    pub config: String,
    /// Where each lane of the IR ended up, as the `<edge>_<index>` identifier the
    /// built network gives it.
    ///
    /// A SUMO lane's identity is positional, so this is the only way back from a
    /// lane of the map to the lane of the network — and the numbering is not the
    /// IR's: SUMO counts from the right of the direction of travel.
    pub lanes: BTreeMap<LaneId, String>,
    /// Where each element of the IR ended up in the four files: nodes, edges, lanes
    /// and connections, by the identifiers written for them. See the crate
    /// documentation for how each is named.
    ///
    /// Its `files` are empty: the network is in memory until [`write_traced`] puts it
    /// somewhere.
    pub trace: Trace,
}

impl PlainNetwork {
    /// Each file's name and its contents, in the order netconvert reads them.
    pub fn files(&self) -> Vec<(String, &str)> {
        vec![
            (format!("{}.nod.xml", self.prefix), self.nodes.as_str()),
            (format!("{}.edg.xml", self.prefix), self.edges.as_str()),
            (
                format!("{}.con.xml", self.prefix),
                self.connections.as_str(),
            ),
            (format!("{}.netccfg", self.prefix), self.config.as_str()),
        ]
    }

    /// Writes the four files into `directory`, creating it if it is not there.
    pub fn write_to(&self, directory: impl AsRef<Path>) -> Result<(), ExportError> {
        let directory = directory.as_ref();
        std::fs::create_dir_all(directory)
            .map_err(|error| ExportError::Io(format!("{}: {error}", directory.display())))?;
        for (name, contents) in self.files() {
            let path = directory.join(&name);
            std::fs::write(&path, contents)
                .map_err(|error| ExportError::Io(format!("{}: {error}", path.display())))?;
        }
        Ok(())
    }
}

/// What the map's files are named: its own name, reduced to something a file name
/// and a SUMO identifier can both hold.
pub fn network_name(map: &ValidatedMap) -> String {
    match map.metadata.name.as_deref().map(identifier) {
        Some(name) if !name.is_empty() => name,
        _ => DEFAULT_NAME.to_owned(),
    }
}

/// Renders `map` as a SUMO plain-XML network.
pub fn to_plain_xml(map: &ValidatedMap) -> Result<PlainNetwork, ExportError> {
    let mut exporter = Exporter::new(map);
    exporter.build()?;
    Ok(exporter.render(network_name(map)))
}

/// Writes `map` into `directory` as a SUMO plain-XML network, and hands back the
/// prefix its files were named with.
pub fn write(map: &ValidatedMap, directory: impl AsRef<Path>) -> Result<String, ExportError> {
    let (prefix, _) = write_traced(map, directory)?;
    Ok(prefix)
}

/// Writes `map` into `directory` as [`write`] does, and hands back the trace of what
/// was written alongside the prefix, with the four files' paths in it.
pub fn write_traced(
    map: &ValidatedMap,
    directory: impl AsRef<Path>,
) -> Result<(String, Trace), ExportError> {
    let directory = directory.as_ref();
    let network = to_plain_xml(map)?;
    network.write_to(directory)?;
    let files = network
        .files()
        .into_iter()
        .map(|(name, _)| directory.join(name))
        .collect();
    let mut trace = network.trace;
    trace.files = files;
    Ok((network.prefix, trace))
}

/// What this map loses on the way into a SUMO network.
///
/// All of these are properties of the format rather than faults in the map. SUMO is
/// a description of a road network for a traffic simulator: it cares what may drive
/// where and how fast, and not at all what the road surface looks like.
pub fn check(map: &ValidatedMap) -> Vec<String> {
    let mut problems = vec![
        "SUMO has no lane markings: which line is painted between two lanes, and in \
         what colour, is not written"
            .to_owned(),
    ];

    if !map.buildings.is_empty() {
        problems.push(format!(
            "a SUMO network is a traffic network and holds no scenery, so the map's \
             {} buildings ({} parts) are not written",
            map.buildings.len(),
            map.building_parts.len()
        ));
    }

    let dropped: BTreeSet<&str> = map
        .lanes
        .iter()
        .filter(|lane| classes::permission(lane.lane_type).is_none())
        .map(|lane| lane.lane_type.as_str())
        .collect();
    if !dropped.is_empty() {
        problems.push(format!(
            "a SUMO lane is a strip traffic runs along, so the map's {} lanes are not \
             written",
            dropped.into_iter().collect::<Vec<_>>().join(", ")
        ));
    }

    let tapered: Vec<&str> = map
        .lanes
        .iter()
        .filter(|lane| !lane.width.is_constant())
        .map(|lane| lane.id.as_str())
        .collect();
    if !tapered.is_empty() {
        problems.push(format!(
            "a SUMO lane has one width from end to end, so the {} tapering lanes are \
             written at their mean width along: {}",
            tapered.len(),
            tapered.join(", ")
        ));
    }

    if map.roads.iter().any(|road| !road.superelevation.is_zero()) {
        problems.push(
            "a SUMO lane is flat across, so the banking is dropped — the heights \
             along it survive"
                .to_owned(),
        );
    }

    let sectioned: Vec<&str> = map
        .roads
        .iter()
        .filter(|road| !road.is_connector() && road.sections.len() > 1)
        .map(|road| road.id.as_str())
        .collect();
    if !sectioned.is_empty() {
        problems.push(format!(
            "an edge has one lane count, so the changing cross-section of {} becomes \
             a chain of edges with a node between them, and which lane continues into \
             which across that node is netconvert's lane-matching — the IR states no \
             movement there: {}",
            sectioned.len(),
            sectioned.join(", ")
        ));
    }

    let signals = map
        .objects
        .iter()
        .filter(|object| object.kind.is_traffic_light())
        .count();
    if signals > 0 {
        problems.push(format!(
            "the IR holds no signal timing, so the {signals} traffic lights make their \
             junctions `traffic_light` nodes and netconvert generates the phases"
        ));
    }

    let unwritten: BTreeSet<&str> = map
        .objects
        .iter()
        .filter(|object| {
            matches!(
                object.kind,
                MapObjectKind::Crosswalk | MapObjectKind::TrafficSign { .. }
            )
        })
        .map(|object| object.kind.as_str())
        .collect();
    if !unwritten.is_empty() {
        problems.push(format!(
            "a SUMO crossing belongs to a node and a sign is an additional file, not \
             part of the network, so the map's {} are not written",
            unwritten.into_iter().collect::<Vec<_>>().join(" and ")
        ));
    }

    if map
        .rules
        .iter()
        .any(|rule| matches!(rule, TrafficRule::RightOfWay { .. }))
    {
        problems.push(
            "SUMO's right of way is a matrix over pairs of movements, and a plain XML \
             file cannot state one: a right-of-way rule is written as the edge \
             priorities of the approaches it names, with the junction marked \
             `rightOfWay=\"edgePriority\"` so that they decide it. Which approach holds \
             right of way survives; which of its own movements must still give way to \
             an oncoming one is netconvert's"
                .to_owned(),
        );
    }

    problems.extend(stop_line_problems(map));

    if !map.junctions.is_empty() {
        problems.push(format!(
            "the connector roads of the {} junctions are not written as edges: SUMO \
             builds an internal lane per movement from the connections instead, so \
             the path across a junction is netconvert's and not the IR's",
            map.junctions.len()
        ));
    }

    problems.push(
        "the network is in the map's own metres about its origin; SUMO carries no \
         geo-reference for it"
            .to_owned(),
    );
    problems
}

/// What becomes of the map's stop lines: how the ones that are written are written,
/// and which ones are not, and why.
///
/// Where a stop line can go depends on the network as it is laid out — which lanes
/// are written, on which edge, ending at which node — so this lays it out, exactly as
/// the export does, and reads the answer off that. A map that cannot be laid out at
/// all fails the export with its own error, and there is nothing to say here about
/// its stop lines.
fn stop_line_problems(map: &ValidatedMap) -> Vec<String> {
    let total = map
        .objects
        .iter()
        .filter(|object| object.kind == MapObjectKind::StopLine)
        .count();
    if total == 0 {
        return Vec::new();
    }
    let mut exporter = Exporter::new(map);
    if exporter.build().is_err() {
        return Vec::new();
    }

    let mut problems = vec![format!(
        "a SUMO network has no stop lines, only a lane's stop offset back from the \
         junction it runs into: the {total} stop lines are written as the stopOffset \
         of the lanes they cross, and the paint itself is not"
    )];
    // Named lane by lane: a line drawn across both carriageways may well be written
    // on the one that runs into the junction and not on the one that leaves it, and
    // only what is missing is missing.
    for (reason, objects) in &exporter.unplaced {
        let why = match reason {
            Unplaced::NotALine => "it is not drawn as a line across the road",
            Unplaced::LaneNotWritten => {
                "no SUMO lane is written there — a junction's connector, or a lane of a \
                 type SUMO has no place for"
            }
            Unplaced::NoJunctionAhead => {
                "the edge does not end at a junction, where a stop offset stops nothing"
            }
            Unplaced::AtLaneStart => {
                "it is at the lane's start, not before the junction it runs into"
            }
            Unplaced::NotAcross => "it does not cross the lane's centreline",
            Unplaced::BeyondLane => {
                "it is further back from the junction than the edge is long, which \
                 netconvert refuses as an offset"
            }
            Unplaced::Superseded => {
                "a stop line nearer the junction holds the lane's one stop offset"
            }
        };
        for (object, lanes) in objects {
            let lanes: Vec<String> = lanes.iter().map(LaneId::local_name).collect();
            problems.push(match lanes.as_slice() {
                [] => format!("stop line {} is not written: {why}", object.as_str()),
                [lane] => format!(
                    "stop line {} is not written on lane {lane}: {why}",
                    object.as_str()
                ),
                _ => format!(
                    "stop line {} is not written on lanes {}: {why}",
                    object.as_str(),
                    lanes.join(", ")
                ),
            });
        }
    }
    problems
}

// --------------------------------------------------------------------------- //
// The network being built
// --------------------------------------------------------------------------- //

/// How netconvert should treat a node.
///
/// Anything not named here is left for netconvert to work out, which it does from
/// the edges that meet there and their priorities. Saying `priority` at a node that
/// should be a `dead_end` is worse than saying nothing.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum NodeKind {
    /// netconvert decides.
    Unstated,
    /// Nothing continues past this point.
    DeadEnd,
    /// A signalised junction, whose phases netconvert generates.
    TrafficLight,
}

impl NodeKind {
    fn as_str(self) -> Option<&'static str> {
        match self {
            NodeKind::Unstated => None,
            NodeKind::DeadEnd => Some("dead_end"),
            NodeKind::TrafficLight => Some("traffic_light"),
        }
    }
}

struct Node {
    point: Point3,
    kind: NodeKind,
    /// Set where the IR states a right of way, and it makes the edge priorities
    /// *decide* the junction rather than being one input among several. See
    /// [`Exporter::priority_of`].
    edge_priority: bool,
    /// The junction this node is, when it is one rather than a road end.
    junction: Option<JunctionId>,
}

struct Edge {
    id: String,
    road: RoadId,
    from: String,
    to: String,
    name: Option<String>,
    priority: i32,
    speed: f64,
    shape: Polyline3,
    lanes: Vec<EdgeLane>,
}

struct EdgeLane {
    lane: LaneId,
    width: f64,
    speed: Option<f64>,
    permission: Option<Permission>,
    shape: Polyline3,
}

/// Where a lane of the IR ended up: which edge, and which index within it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Slot {
    edge: usize,
    index: usize,
}

/// The IR behind one written connection: the lane connections it was followed along,
/// and the connector lanes it crossed a junction on.
#[derive(Default)]
struct Movement {
    connections: BTreeSet<ConnectionId>,
    connectors: BTreeSet<LaneId>,
}

struct Exporter<'a> {
    map: &'a ValidatedMap,
    sampling: SamplingConfig,
    nodes: BTreeMap<String, Node>,
    edges: Vec<Edge>,
    /// Each written connection, by (from edge, from lane, to edge, to lane), and what
    /// of the IR it stands for. Ordered, and a map, because a junction with several
    /// connectors between one pair of lanes would otherwise write the same movement
    /// twice.
    movements: BTreeMap<(usize, usize, usize, usize), Movement>,
    slots: HashMap<LaneId, Slot>,
    /// Junctions a traffic light controls an approach to.
    /// The signalised junctions, each with the lights that govern it.
    signalised: BTreeMap<JunctionId, BTreeSet<ObjectId>>,
    /// Lanes that keep right of way where another yields to them, and the lanes that
    /// yield. Held as lanes, not roads: a rule about one carriageway's approach must
    /// not move the opposing carriageway's priority with it.
    right_of_way: HashSet<LaneId>,
    yielding: HashSet<LaneId>,
    /// Junctions a right-of-way rule speaks about.
    ruled: HashSet<JunctionId>,
    /// The stop offset of each written lane that has one, by (edge, lane index): how
    /// far back from the lane's end its stop line is, and which stop line that is.
    stop_offsets: BTreeMap<(usize, usize), (ObjectId, f64)>,
    /// Where stop lines could not be written as a stop offset, for [`check`] to
    /// name: by reason, each stop line with the lanes it misses. A line that is not
    /// a line at all misses every lane, and is held with none.
    unplaced: BTreeMap<Unplaced, BTreeMap<ObjectId, BTreeSet<LaneId>>>,
}

/// Why a stop line did not become a lane's stop offset.
///
/// Ordered, so that [`check`] names the reasons in a fixed order and each once, with
/// every stop line and lane it applies to.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
enum Unplaced {
    /// The line is drawn as something other than a line across the road.
    NotALine,
    /// It is on a lane no SUMO lane was written for: a junction's connector, or a
    /// lane of a type SUMO has no place for.
    LaneNotWritten,
    /// Its lane does not run into a junction, so there is nothing at its end to wait
    /// for.
    NoJunctionAhead,
    /// It crosses the lane nearer where the lane starts than where it runs into its
    /// junction: a line across both carriageways, seen from the one leaving the
    /// junction it stands before.
    AtLaneStart,
    /// It does not cross the centreline of the lane it names.
    NotAcross,
    /// It is further back from the junction than its edge is long, which netconvert
    /// refuses as an offset.
    BeyondLane,
    /// Another stop line, nearer the junction, already holds the lane's offset.
    Superseded,
}

impl<'a> Exporter<'a> {
    fn new(map: &'a ValidatedMap) -> Self {
        let mut exporter = Exporter {
            map,
            sampling: map.metadata.sampling,
            nodes: BTreeMap::new(),
            edges: Vec::new(),
            movements: BTreeMap::new(),
            slots: HashMap::new(),
            signalised: BTreeMap::new(),
            right_of_way: HashSet::new(),
            yielding: HashSet::new(),
            ruled: HashSet::new(),
            stop_offsets: BTreeMap::new(),
            unplaced: BTreeMap::new(),
        };
        exporter.read_rules();
        exporter
    }

    /// The two things about a map that are decided before any edge is written: which
    /// junctions are signalised, and which arms hold right of way over which.
    fn read_rules(&mut self) {
        for object in self.map.objects.iter() {
            if !object.kind.is_traffic_light() {
                continue;
            }
            for lane in &object.lanes {
                if let Some(junction) = self.junction_ahead_of(lane) {
                    self.signalised
                        .entry(junction)
                        .or_default()
                        .insert(object.id.clone());
                }
            }
        }

        for rule in &self.map.rules {
            let TrafficRule::RightOfWay {
                right_of_way,
                yielding,
                ..
            } = rule
            else {
                continue;
            };
            for lane in right_of_way.iter().chain(yielding) {
                if let Some(junction) = self.junction_ahead_of(lane) {
                    self.ruled.insert(junction);
                }
            }
            self.right_of_way.extend(right_of_way.iter().cloned());
            self.yielding.extend(yielding.iter().cloned());
        }
    }

    /// The junction a lane runs into, which is the one a control on that lane governs.
    ///
    /// Read from the end traffic *leaves* by rather than from either end of the road,
    /// so a light on the northbound carriageway does not also signalise the junction
    /// behind it.
    fn junction_ahead_of(&self, lane: &LaneId) -> Option<JunctionId> {
        let lane = self.map.lane(lane)?;
        let road = self.map.road(&lane.road)?;
        if let Some(junction) = &road.junction {
            // A control on a connector is a control inside the junction itself.
            return Some(junction.clone());
        }
        let end = match lane.direction.exit_end() {
            LaneEnd::Start => RoadEnd::Start,
            LaneEnd::End => RoadEnd::End,
        };
        match road.link.at(end) {
            Some(RoadLinkTarget::Junction(junction)) => Some(junction.clone()),
            _ => None,
        }
    }

    fn build(&mut self) -> Result<(), ExportError> {
        for road in self.map.roads.iter() {
            if road.is_connector() {
                // A connector is the IR's drawing of one movement through a junction.
                // SUMO draws that itself, from the connection, as an internal lane.
                continue;
            }
            for section in 0..road.sections.len() {
                self.section_edges(road, section)?;
            }
        }
        self.build_connections();
        self.place_stop_lines()?;
        Ok(())
    }

    // ----------------------------------------------------------------------- //
    // Nodes
    // ----------------------------------------------------------------------- //

    fn node(
        &mut self,
        id: String,
        point: Point3,
        kind: NodeKind,
        edge_priority: bool,
        junction: Option<JunctionId>,
    ) -> String {
        self.nodes.entry(id.clone()).or_insert(Node {
            point,
            kind,
            edge_priority,
            junction,
        });
        id
    }

    /// The node at one end of a road: its junction, the joint it shares with the road
    /// it continues into, or a dead end of its own.
    fn road_end_node(&mut self, road: &Road, end: RoadEnd) -> String {
        match road.link.at(end).cloned() {
            Some(RoadLinkTarget::Junction(junction)) => {
                let kind = if self.signalised.contains_key(&junction) {
                    NodeKind::TrafficLight
                } else {
                    NodeKind::Unstated
                };
                let point = self
                    .map
                    .junction_centre(&junction)
                    .unwrap_or_else(|| road.endpoint(end));
                self.node(
                    format!("j_{}", identifier(junction.local_name())),
                    point,
                    kind,
                    self.ruled.contains(&junction),
                    Some(junction),
                )
            }
            Some(RoadLinkTarget::Road(other)) => {
                // Both roads have to name the joint the same way, so the pair is put
                // in a fixed order and the first of the two names it.
                let here = (identifier(road.id.local_name()), end);
                let there = (identifier(other.road.local_name()), other.end);
                let first =
                    if (here.0.as_str(), here.1.as_str()) <= (there.0.as_str(), there.1.as_str()) {
                        &here
                    } else {
                        &there
                    };
                // The two endpoints are the same place to within the validation
                // tolerance; the midpoint splits whatever is left of the difference.
                let point = match self.map.road(&other.road) {
                    Some(neighbour) => road.endpoint(end).lerp(neighbour.endpoint(other.end), 0.5),
                    None => road.endpoint(end),
                };
                self.node(
                    format!("n_{}_{}", first.0, first.1.as_str()),
                    point,
                    NodeKind::Unstated,
                    false,
                    None,
                )
            }
            None => self.node(
                format!("n_{}_{}", identifier(road.id.local_name()), end.as_str()),
                road.endpoint(end),
                NodeKind::DeadEnd,
                false,
                None,
            ),
        }
    }

    /// The node a road's cross-section boundary sits on, which exists only because a
    /// SUMO edge cannot change its lane count partway along.
    fn section_node(&mut self, road: &Road, section: usize) -> Result<String, ExportError> {
        let station = road.sections[section].station;
        let point = road.reference_line.sample_at(station, self.sampling)?.point;
        Ok(self.node(
            format!("n_{}_s{section}", identifier(road.id.local_name())),
            point,
            NodeKind::Unstated,
            false,
            None,
        ))
    }

    // ----------------------------------------------------------------------- //
    // Edges
    // ----------------------------------------------------------------------- //

    /// The one or two edges of one cross-section: one per direction traffic runs in.
    ///
    /// A cross-section none of whose lanes SUMO has a place for — all of them
    /// borders, parking or the like — writes no edge, and then it writes no node
    /// either. The nodes at its two ends exist only for edges to run between, and a
    /// node no edge reaches is not a dead end or a joint but a stray point netconvert
    /// has to throw away. So the carriageways are read first, and the ends are made
    /// only once there is something to hang on them. A node another road does reach
    /// is still written, by that road.
    fn section_edges(&mut self, road: &Road, section: usize) -> Result<(), ExportError> {
        let carriageways: Vec<(Direction, Vec<&'a Lane>)> =
            [Direction::Forward, Direction::Backward]
                .into_iter()
                .map(|direction| (direction, self.carriageway(road, section, direction)))
                .filter(|(_, lanes)| !lanes.is_empty())
                .collect();
        if carriageways.is_empty() {
            return Ok(());
        }

        let at_start = if section == 0 {
            self.road_end_node(road, RoadEnd::Start)
        } else {
            self.section_node(road, section)?
        };
        let at_end = if section + 1 == road.sections.len() {
            self.road_end_node(road, RoadEnd::End)
        } else {
            self.section_node(road, section + 1)?
        };

        for (direction, lanes) in carriageways {
            let (from, to) = match direction {
                Direction::Forward => (at_start.clone(), at_end.clone()),
                Direction::Backward => (at_end.clone(), at_start.clone()),
            };
            self.add_edge(road, section, direction, from, to, &lanes)?;
        }
        Ok(())
    }

    /// The lanes of one cross-section that run in one direction, ordered as SUMO
    /// numbers them: index 0 is the rightmost in the direction of travel.
    ///
    /// Ordered by where the lanes actually are rather than by which side of the
    /// reference line they sit on, because which side is the driver's right depends
    /// on the direction of travel — and, for the reference line itself, on whether
    /// the map drives on the left.
    fn carriageway(&self, road: &Road, section: usize, direction: Direction) -> Vec<&'a Lane> {
        let mut lanes: Vec<&Lane> = self
            .map
            .lanes_of_section(&road.id, section)
            .into_iter()
            .filter(|lane| lane.direction == direction)
            .filter(|lane| classes::permission(lane.lane_type).is_some())
            .collect();
        // A lateral offset counts to the left of the reference line, so a driver
        // running forward sees the smallest offset on the right and one running
        // backward sees the largest.
        lanes.sort_by(|a, b| {
            let rightwards = |lane: &Lane| match direction {
                Direction::Forward => lane.center_offset(),
                Direction::Backward => -lane.center_offset(),
            };
            rightwards(a)
                .total_cmp(&rightwards(b))
                .then(a.index.cmp(&b.index))
        });
        lanes
    }

    fn add_edge(
        &mut self,
        road: &Road,
        section: usize,
        direction: Direction,
        from: String,
        to: String,
        lanes: &[&Lane],
    ) -> Result<(), ExportError> {
        let index = self.edges.len();
        let mut written = Vec::with_capacity(lanes.len());
        for (position, lane) in lanes.iter().enumerate() {
            let travel = lane.travel_geometry(self.sampling)?;
            written.push(EdgeLane {
                lane: lane.id.clone(),
                width: self.mean_width(lane)?,
                speed: lane.speed_limit.map(|limit| limit.mps()),
                permission: classes::permission(lane.lane_type),
                shape: travel.centerline.to_polyline(self.sampling)?,
            });
            self.slots.insert(
                lane.id.clone(),
                Slot {
                    edge: index,
                    index: position,
                },
            );
        }

        let shape = self.carriageway_shape(lanes, &written)?;
        self.edges.push(Edge {
            id: edge_id(road, section, direction),
            road: road.id.clone(),
            from,
            to,
            name: road.name.clone(),
            priority: self.priority_of(road, lanes),
            speed: road
                .speed_limit
                .map(|limit| limit.mps())
                .unwrap_or_else(|| classes::default_speed(road.road_type)),
            shape,
            lanes: written,
        });
        Ok(())
    }

    /// The one width SUMO lets a lane have: the area of the lane divided by its
    /// length.
    ///
    /// A lane that tapers has no single width, and which number to write is a real
    /// choice. The width halfway along would say 3.5 m for a lane that runs full
    /// width for half its length and then closes to nothing, which is the width it
    /// has almost nowhere. The mean along it is the width a lane of constant width
    /// would need to cover the same ground, and it is what [`crate::check`] reports.
    fn mean_width(&self, lane: &Lane) -> Result<f64, ExportError> {
        let (start, end) = lane.station_range;
        if lane.width.is_constant() || end - start <= 0.0 {
            return Ok(lane.width_at(start));
        }
        // At the stations the lane's own geometry was generated at, so the taper is
        // measured where it was drawn.
        let stations: Vec<f64> = self
            .map
            .vertex_stations(&lane.road)?
            .into_iter()
            .filter(|station| *station >= start - 1e-9 && *station <= end + 1e-9)
            .collect();
        if stations.len() < 2 {
            return Ok(lane.width_at((start + end) / 2.0));
        }
        let area: f64 = stations
            .windows(2)
            .map(|pair| {
                (lane.width_at(pair[0]) + lane.width_at(pair[1])) / 2.0 * (pair[1] - pair[0])
            })
            .sum();
        Ok(area / (stations[stations.len() - 1] - stations[0]))
    }

    /// The line an edge is drawn down: the middle of the carriageway.
    ///
    /// Written as the mean of the two outer kerbs rather than of the lane centres, so
    /// that a carriageway with lanes of different widths still has its geometric
    /// middle. It is only ever the *edge's* line — every lane carries its own shape,
    /// so nothing is laid out from this.
    fn carriageway_shape(
        &self,
        lanes: &[&Lane],
        written: &[EdgeLane],
    ) -> Result<Polyline3, ExportError> {
        let (Some(rightmost), Some(leftmost)) = (lanes.first(), lanes.last()) else {
            return Err(ExportError::Unknown("an edge with no lanes".to_owned()));
        };
        let right = rightmost.travel_geometry(self.sampling)?.right;
        let left = leftmost.travel_geometry(self.sampling)?.left;
        let (right, left) = (
            right.to_polyline(self.sampling)?,
            left.to_polyline(self.sampling)?,
        );
        if right.len() != left.len() {
            // Two boundaries of one cross-section are sampled at the same stations,
            // so this does not happen; if it ever did, a lane's own centreline is a
            // truthful line down the edge.
            return Ok(written[0].shape.clone());
        }
        Ok(Polyline3::new(
            right
                .points()
                .iter()
                .zip(left.points())
                .map(|(a, b)| a.lerp(*b, 0.5)),
        )?)
    }

    /// Where an edge sits in the priority order.
    ///
    /// The road type sets the rung, and a right-of-way rule moves this edge off it:
    /// up if it carries a lane that keeps right of way, down if it carries one that
    /// yields.
    ///
    /// Judged from *this edge's own lanes* rather than from its road, because the
    /// rule is about lanes. A road is two edges pointing at each other, and a rule
    /// naming the northbound approach says nothing whatever about the southbound one.
    ///
    /// This is the whole of what the format can carry. SUMO's right of way is the
    /// `<request>` matrix — which movement gives way to which other movement, pair by
    /// pair — and a plain XML file has no way to state it: netconvert computes it,
    /// and an edge priority is one of the things it computes it from. So the junction
    /// this edge runs into is marked `rightOfWay="edgePriority"` (see
    /// [`Exporter::road_end_node`]), which makes these numbers *decide* who yields
    /// instead of being weighed against netconvert's own reading of the geometry. The
    /// IR's statement then survives as far as the format allows: which approach holds
    /// right of way. Which of its movements must still give way to an oncoming one is
    /// netconvert's, and [`crate::check`] says so.
    ///
    /// The whole ladder is lifted one rung before a rule moves anything. The bottom
    /// of [`classes::priority`] is 1, a footway's, and one rung below it is 0. This
    /// export used to clamp that back up to 1, which put a yielding footway level
    /// with the footway it yields to, and the rule was gone without a word. Lifted,
    /// the lowest a yielding edge can land is still 1, and every edge keeps its
    /// place relative to every other; since only the *order* of the numbers decides
    /// anything, the shift changes nothing else.
    fn priority_of(&self, road: &Road, lanes: &[&Lane]) -> i32 {
        let base = classes::priority(road.road_type) + 1;
        let carries = |named: &HashSet<LaneId>| lanes.iter().any(|lane| named.contains(&lane.id));
        let raise = i32::from(carries(&self.right_of_way));
        let lower = i32::from(carries(&self.yielding));
        base + raise - lower
    }

    // ----------------------------------------------------------------------- //
    // Connections
    // ----------------------------------------------------------------------- //

    /// Every movement the IR permits, as a connection between two written lanes.
    ///
    /// A movement through a junction is stated in the IR as two connections — into a
    /// connector lane and out of it again — because the connector is a road there. In
    /// SUMO it is one connection from the approach to the exit, and that connection is
    /// what makes netconvert draw the internal lane. So a movement is followed from an
    /// approach through however many connectors lie between it and the lane it comes
    /// out on, and only the two ends are written.
    ///
    /// Every IR connection walked along the way is kept with the connection it
    /// became, as is every connector lane crossed, so the trace can say what each
    /// written connection stands for.
    fn build_connections(&mut self) {
        let mut successors: BTreeMap<LaneId, Vec<(ConnectionId, LaneId)>> = BTreeMap::new();
        for connection in self.map.connections.iter() {
            successors
                .entry(connection.from.lane.clone())
                .or_default()
                .push((connection.id.clone(), connection.to.lane.clone()));
        }
        let step = |lane: &LaneId| {
            successors
                .get(lane)
                .into_iter()
                .flatten()
                .map(|(id, to)| (lane.clone(), id.clone(), to.clone()))
                .collect::<Vec<_>>()
        };

        let mut sources: Vec<LaneId> = self.slots.keys().cloned().collect();
        sources.sort();
        for source in sources {
            let mut reached: Vec<LaneId> = Vec::new();
            let mut seen: HashSet<LaneId> = HashSet::from([source.clone()]);
            // Every connection followed, whether or not it led anywhere new.
            let mut walked: Vec<(LaneId, ConnectionId, LaneId)> = Vec::new();
            let mut frontier = step(&source);
            while let Some((from, connection, next)) = frontier.pop() {
                walked.push((from, connection, next.clone()));
                if !seen.insert(next.clone()) {
                    continue;
                }
                if self.slots.contains_key(&next) {
                    reached.push(next);
                } else if self.on_connector(&next) {
                    // A connector is not a lane of the network: carry on to whatever
                    // it leads to. A lane that is merely of a type SUMO has no place
                    // for is a different matter — the movement ends there, because
                    // nothing was written for traffic to arrive on.
                    frontier.extend(step(&next));
                }
            }
            for target in reached {
                let movement = self.movement_between(&source, &target, &walked);
                self.connect(&source, &target, movement);
            }
        }
    }

    /// The part of what was walked from `source` that lies on a way to `target`: the
    /// connections into the target, and back from there through connectors to the
    /// source.
    fn movement_between(
        &self,
        source: &LaneId,
        target: &LaneId,
        walked: &[(LaneId, ConnectionId, LaneId)],
    ) -> Movement {
        let mut movement = Movement::default();
        let mut ahead: HashSet<&LaneId> = HashSet::from([target]);
        let mut pending: Vec<&LaneId> = vec![target];
        while let Some(lane) = pending.pop() {
            for (from, connection, _) in walked.iter().filter(|(_, _, to)| to == lane) {
                movement.connections.insert(connection.clone());
                if from != source && self.on_connector(from) && ahead.insert(from) {
                    movement.connectors.insert(from.clone());
                    pending.push(from);
                }
            }
        }
        movement
    }

    fn on_connector(&self, lane: &LaneId) -> bool {
        self.map
            .lane(lane)
            .and_then(|lane| self.map.road(&lane.road))
            .is_some_and(Road::is_connector)
    }

    fn connect(&mut self, from: &LaneId, to: &LaneId, movement: Movement) {
        let (Some(from), Some(to)) = (self.slots.get(from), self.slots.get(to)) else {
            return;
        };
        if from.edge == to.edge {
            // Two lanes of one edge; SUMO says that with a lane change, not a
            // connection.
            return;
        }
        let key = (from.edge, from.index, to.edge, to.index);
        let merged = self.movements.entry(key).or_default();
        merged.connections.extend(movement.connections);
        merged.connectors.extend(movement.connectors);
    }

    // ----------------------------------------------------------------------- //
    // Stop lines
    // ----------------------------------------------------------------------- //

    /// Every stop line of the map, as the stop offset of the written lanes it
    /// crosses.
    ///
    /// The lanes a stop line is measured against are the ones it names, and the ones
    /// a rule that names it governs: a right-of-way rule's yielding lanes, a traffic
    /// light rule's lanes. The first are the stop line's own word, so a line that
    /// does not actually cross one of them is reported. The second are the rule's,
    /// and a rule over several lanes may well carry a stop line drawn across only
    /// some of them — so a lane the line does not cross is simply not one it stops.
    ///
    /// The distance is measured along the lane's written shape, the line SUMO
    /// measures its own offsets along, from where the stop line crosses it to its
    /// far end. netconvert keeps a lane's custom shape as it was given rather than
    /// cutting it back to the junction, so the end measured from here is the end the
    /// built network has.
    fn place_stop_lines(&mut self) -> Result<(), ExportError> {
        let map = self.map;
        let mut governed: HashMap<&ObjectId, BTreeSet<&LaneId>> = HashMap::new();
        for rule in &map.rules {
            let (stop_line, lanes) = match rule {
                TrafficRule::RightOfWay {
                    stop_line,
                    yielding,
                    ..
                } => (stop_line, yielding),
                TrafficRule::TrafficLight {
                    stop_line, lanes, ..
                } => (stop_line, lanes),
                TrafficRule::SpeedLimit { .. } => continue,
            };
            if let Some(stop_line) = stop_line {
                governed.entry(stop_line).or_default().extend(lanes);
            }
        }

        // Every placement each lane could take, to be sorted nearest the junction
        // first, so that the line that governs the junction is the one written.
        let mut candidates: BTreeMap<(usize, usize), Vec<(f64, ObjectId)>> = BTreeMap::new();
        for object in map.objects.iter() {
            if object.kind != MapObjectKind::StopLine {
                continue;
            }
            let ObjectGeometry::Line(curve) = &object.geometry else {
                self.unplace(Unplaced::NotALine, &object.id, None);
                continue;
            };
            let line = curve.to_polyline(self.sampling)?;
            let named: BTreeSet<&LaneId> = object.lanes.iter().collect();
            let lanes: BTreeSet<&LaneId> = named
                .iter()
                .copied()
                .chain(governed.get(&object.id).into_iter().flatten().copied())
                .collect();
            for lane in lanes {
                let own = named.contains(lane);
                let Some(&slot) = self.slots.get(lane) else {
                    if own {
                        self.unplace(Unplaced::LaneNotWritten, &object.id, Some(lane));
                    }
                    continue;
                };
                let edge = &self.edges[slot.edge];
                let shape = &edge.lanes[slot.index].shape;
                let Some(distance) = crossing_from_end(shape, &line) else {
                    if own {
                        self.unplace(Unplaced::NotAcross, &object.id, Some(lane));
                    }
                    continue;
                };
                let at_junction = self
                    .nodes
                    .get(&edge.to)
                    .is_some_and(|node| node.junction.is_some());
                if !at_junction {
                    self.unplace(Unplaced::NoJunctionAhead, &object.id, Some(lane));
                    continue;
                }
                // Nothing says which way a line runs, and one drawn across the whole
                // road names the lane leaving a junction as well as the one entering
                // it. It stands before the junction ahead only where it is in the
                // half of the lane that leads into it, nearer its end than its start,
                // and nearer the edge's own mouth at that end than at the other (the
                // nodes themselves will not do: a junction's sits at its middle).
                let from_start = shape.length() - distance;
                let start_nearer = match edge.shape.points() {
                    [first, .., last] => {
                        distance_to_line(*first, &line) < distance_to_line(*last, &line)
                    }
                    _ => false,
                };
                if distance >= from_start || start_nearer {
                    self.unplace(Unplaced::AtLaneStart, &object.id, Some(lane));
                    continue;
                }
                // netconvert measures the limit against the edge, whose line is the
                // middle of the carriageway; the lane's own length is the other bound
                // that matters, on the inside of a bend.
                let limit = shape.length().min(edge.shape.length());
                if distance >= limit - MIN_STOP_OFFSET {
                    self.unplace(Unplaced::BeyondLane, &object.id, Some(lane));
                    continue;
                }
                candidates
                    .entry((slot.edge, slot.index))
                    .or_default()
                    .push((distance, object.id.clone()));
            }
        }

        for (key, mut placements) in candidates {
            placements.sort_by(|a, b| a.0.total_cmp(&b.0).then_with(|| a.1.cmp(&b.1)));
            let mut placements = placements.into_iter();
            let Some((distance, object)) = placements.next() else {
                continue;
            };
            for (_, other) in placements {
                if other != object {
                    let lane = self.edges[key.0].lanes[key.1].lane.clone();
                    self.unplace(Unplaced::Superseded, &other, Some(&lane));
                }
            }
            // A line at the very end of the lane is where SUMO stops a vehicle
            // anyway; writing an offset of nothing would only add noise.
            if distance > MIN_STOP_OFFSET {
                self.stop_offsets.insert(key, (object, distance));
            }
        }
        Ok(())
    }

    fn unplace(&mut self, reason: Unplaced, object: &ObjectId, lane: Option<&LaneId>) {
        let lanes = self
            .unplaced
            .entry(reason)
            .or_default()
            .entry(object.clone())
            .or_default();
        lanes.extend(lane.cloned());
    }

    // ----------------------------------------------------------------------- //
    // Rendering
    // ----------------------------------------------------------------------- //

    fn render(&self, prefix: String) -> PlainNetwork {
        PlainNetwork {
            nodes: self.render_nodes(),
            edges: self.render_edges(),
            connections: self.render_connections(),
            config: render_config(&prefix),
            trace: self.trace(),
            lanes: self
                .slots
                .iter()
                .map(|(lane, slot)| {
                    (
                        lane.clone(),
                        format!("{}_{}", self.edges[slot.edge].id, slot.index),
                    )
                })
                .collect(),
            prefix,
        }
    }

    /// Where each element of the IR went, read off the nodes, edges and connections
    /// as they are rendered, in the order they are written.
    fn trace(&self) -> Trace {
        let mut trace = Trace::new("sumo");

        let mut junction_nodes: HashMap<&JunctionId, &str> = HashMap::new();
        for (id, node) in &self.nodes {
            if let Some(junction) = &node.junction {
                trace.link(junction.clone(), format!("node:{id}"), Relation::Exact);
                junction_nodes.insert(junction, id);
            }
        }
        // A light is part of the signal of the node it made one, as `read_rules`
        // decided when it chose which nodes are signalised.
        for (junction, lights) in &self.signalised {
            let Some(node) = junction_nodes.get(junction) else {
                continue;
            };
            for light in lights {
                trace.link_as(
                    light.clone(),
                    format!("node:{node}"),
                    Relation::Merged,
                    "traffic_light",
                );
            }
        }

        // A right-of-way rule is carried by the priority of the edges whose lanes it
        // names — raised for those that hold right of way, lowered for those that
        // yield — which is all of it the format can state.
        for (index, rule) in self.map.rules.iter().enumerate() {
            let TrafficRule::RightOfWay {
                right_of_way,
                yielding,
                ..
            } = rule
            else {
                continue;
            };
            let named: BTreeSet<usize> = right_of_way
                .iter()
                .chain(yielding.iter())
                .filter_map(|lane| self.slots.get(lane))
                .map(|slot| slot.edge)
                .collect();
            for edge in named {
                trace.link_as(
                    IrRef::Rule(index),
                    format!("edge:{}", self.edges[edge].id),
                    Relation::Merged,
                    "priority",
                );
            }
        }

        for edge in &self.edges {
            trace.link(
                edge.road.clone(),
                format!("edge:{}", edge.id),
                Relation::Part,
            );
            for (index, lane) in edge.lanes.iter().enumerate() {
                trace.link(
                    lane.lane.clone(),
                    format!("lane:{}_{index}", edge.id),
                    Relation::Exact,
                );
            }
        }

        // A stop line is carried by the stop offset of each lane it was measured
        // against: one attribute of a lane that is otherwise the IR's lane.
        for (&(edge, index), (object, _)) in &self.stop_offsets {
            trace.link_as(
                object.clone(),
                format!("lane:{}_{index}", self.edges[edge].id),
                Relation::Merged,
                "stopOffset",
            );
        }

        for (&(from_edge, from_lane, to_edge, to_lane), movement) in &self.movements {
            let local = format!(
                "connection:{}_{from_lane}>{}_{to_lane}",
                self.edges[from_edge].id, self.edges[to_edge].id
            );
            // One IR connection straight from lane to lane is this connection; any
            // more, or any connector between them, and it is one of several.
            let relation = if movement.connectors.is_empty() && movement.connections.len() == 1 {
                Relation::Exact
            } else {
                Relation::Merged
            };
            for connection in &movement.connections {
                trace.link(connection.clone(), local.clone(), relation);
            }
            for connector in &movement.connectors {
                trace.link(connector.clone(), local.clone(), Relation::Collapsed);
            }
        }
        trace
    }

    fn render_nodes(&self) -> String {
        let mut document = xml::Document::new("nodes", "http://sumo.dlr.de/xsd/nodes_file.xsd");
        for (id, node) in &self.nodes {
            let mut attributes = vec![
                ("id", id.clone()),
                ("x", metres(node.point.x)),
                ("y", metres(node.point.y)),
                ("z", metres(node.point.z)),
            ];
            if let Some(kind) = node.kind.as_str() {
                attributes.push(("type", kind.to_owned()));
            }
            if node.edge_priority {
                // The map said who yields here, so the priorities decide it rather
                // than netconvert's reading of the geometry.
                attributes.push(("rightOfWay", "edgePriority".to_owned()));
            }
            document.leaf("node", &attributes);
        }
        document.finish()
    }

    fn render_edges(&self) -> String {
        let mut document = xml::Document::new("edges", "http://sumo.dlr.de/xsd/edges_file.xsd");
        for (edge_index, edge) in self.edges.iter().enumerate() {
            let mut attributes = vec![
                ("id", edge.id.clone()),
                ("from", edge.from.clone()),
                ("to", edge.to.clone()),
                ("priority", edge.priority.to_string()),
                ("numLanes", edge.lanes.len().to_string()),
                ("speed", metres(edge.speed)),
                // The lanes carry their own shapes, so this one is the middle of the
                // carriageway and is spread from as such.
                ("spreadType", "center".to_owned()),
                ("shape", shape(&edge.shape)),
            ];
            if let Some(name) = &edge.name {
                attributes.push(("name", name.clone()));
            }
            document.open("edge", &attributes);
            for (index, lane) in edge.lanes.iter().enumerate() {
                let mut attributes = vec![
                    ("index", index.to_string()),
                    ("width", metres(lane.width)),
                    ("shape", shape(&lane.shape)),
                ];
                if let Some(speed) = lane.speed {
                    attributes.push(("speed", metres(speed)));
                }
                if let Some(permission) = lane.permission {
                    let (key, value) = permission.attribute();
                    attributes.push((key, value.to_owned()));
                }
                match self.stop_offsets.get(&(edge_index, index)) {
                    // No `vClasses`: a stop line binds everything that drives up to
                    // it, which is SUMO's default of `all`.
                    Some((_, distance)) => {
                        document.open("lane", &attributes);
                        document.leaf("stopOffset", &[("value", metres(*distance))]);
                        document.close("lane");
                    }
                    None => document.leaf("lane", &attributes),
                }
            }
            document.close("edge");
        }
        document.finish()
    }

    fn render_connections(&self) -> String {
        let mut document =
            xml::Document::new("connections", "http://sumo.dlr.de/xsd/connections_file.xsd");
        for (from_edge, from_lane, to_edge, to_lane) in self.movements.keys() {
            document.leaf(
                "connection",
                &[
                    ("from", self.edges[*from_edge].id.clone()),
                    ("to", self.edges[*to_edge].id.clone()),
                    ("fromLane", from_lane.to_string()),
                    ("toLane", to_lane.to_string()),
                ],
            );
        }
        document.finish()
    }
}

/// The netconvert run that builds the `.net.xml`.
///
/// Normalising the offset is turned off because the map's metres are about its own
/// geographic origin: a network shifted so that its lowest corner is at zero would no
/// longer line up with the OpenDRIVE, the Lanelet2 map or the clip written from the
/// same IR.
///
/// Turnarounds are turned off because the IR has none. Left to itself netconvert adds
/// a U-turn at every dead end — the far end of each arm — so a vehicle in SUMO could
/// turn back where, in the OpenDRIVE and the Lanelet2 map written from the same IR,
/// the lane simply ends; and the internal lane it draws for one would be the only
/// lane in the network the trace could not follow back to the map.
fn render_config(prefix: &str) -> String {
    let mut document = xml::Document::new(
        "configuration",
        "http://sumo.dlr.de/xsd/netconvertConfiguration.xsd",
    );
    document.open("input", &[]);
    document.leaf("node-files", &[("value", format!("{prefix}.nod.xml"))]);
    document.leaf("edge-files", &[("value", format!("{prefix}.edg.xml"))]);
    document.leaf(
        "connection-files",
        &[("value", format!("{prefix}.con.xml"))],
    );
    document.close("input");
    document.open("output", &[]);
    document.leaf("output-file", &[("value", format!("{prefix}.net.xml"))]);
    document.close("output");
    document.open("processing", &[]);
    document.leaf("offset.disable-normalization", &[("value", "true".into())]);
    document.leaf("no-turnarounds", &[("value", "true".into())]);
    document.close("processing");
    document.finish()
}

/// What an edge is called: the road, the cross-section when there is more than one,
/// and which way traffic runs along it.
fn edge_id(road: &Road, section: usize, direction: Direction) -> String {
    let name = identifier(road.id.local_name());
    let sense = match direction {
        Direction::Forward => "fwd",
        Direction::Backward => "bwd",
    };
    if road.sections.len() > 1 {
        format!("{name}.{section}.{sense}")
    } else {
        format!("{name}.{sense}")
    }
}

/// How far back from the far end of `path` the line `line` crosses it, measured
/// along `path`; `None` if the two never meet.
///
/// The crossing is found in plan, because a stop line is paint on the road and the
/// lane's centreline is on the same surface: what matters is where one passes over
/// the other, not whether two floating-point heights agree. The distance is then
/// taken along the path in three dimensions, which is how SUMO measures a lane's
/// length and so its offsets.
///
/// A line drawn exactly to the end of the path — a stop line at the very mouth of the
/// junction — counts as crossing it there, with a little tolerance on both, so that
/// the rounding in where the line was put does not lose it. Where the line crosses
/// more than once, the crossing nearest the end is the one that stops a vehicle.
fn crossing_from_end(path: &Polyline3, line: &Polyline3) -> Option<f64> {
    const TOLERANCE: f64 = 1e-6;
    let cross = |a: (f64, f64), b: (f64, f64)| a.0 * b.1 - a.1 * b.0;
    let total = path.length();
    let mut travelled = 0.0;
    let mut best: Option<f64> = None;
    for segment in path.points().windows(2) {
        let (a, b) = (segment[0], segment[1]);
        let along = (b.x - a.x, b.y - a.y);
        let length = a.distance_to(b);
        for piece in line.points().windows(2) {
            let (c, d) = (piece[0], piece[1]);
            let across = (d.x - c.x, d.y - c.y);
            let denominator = cross(along, across);
            if denominator.abs() < 1e-12 {
                // Parallel: a stop line running along a lane stops nothing on it.
                continue;
            }
            let offset = (c.x - a.x, c.y - a.y);
            let t = cross(offset, across) / denominator;
            let u = cross(offset, along) / denominator;
            if (-TOLERANCE..=1.0 + TOLERANCE).contains(&t)
                && (-TOLERANCE..=1.0 + TOLERANCE).contains(&u)
            {
                let remaining = (total - travelled - t.clamp(0.0, 1.0) * length).max(0.0);
                if best.is_none_or(|previous| remaining < previous) {
                    best = Some(remaining);
                }
            }
        }
        travelled += length;
    }
    best
}

/// How far `point` is from the nearest piece of `line`, in plan.
fn distance_to_line(point: Point3, line: &Polyline3) -> f64 {
    line.points()
        .windows(2)
        .map(|piece| {
            let (c, d) = (piece[0], piece[1]);
            let across = (d.x - c.x, d.y - c.y);
            let squared = across.0 * across.0 + across.1 * across.1;
            let t = if squared > 0.0 {
                (((point.x - c.x) * across.0 + (point.y - c.y) * across.1) / squared)
                    .clamp(0.0, 1.0)
            } else {
                0.0
            };
            (point.x - c.x - t * across.0).hypot(point.y - c.y - t * across.1)
        })
        .fold(f64::INFINITY, f64::min)
}

/// A name reduced to something a SUMO identifier can hold.
///
/// SUMO splits lists of ids on whitespace and reserves a leading colon for the
/// internal edges it generates itself, so neither can survive in a name the caller
/// chose.
fn identifier(name: &str) -> String {
    name.chars()
        .map(|character| {
            if character.is_whitespace() || character == ':' {
                '_'
            } else {
                character
            }
        })
        .collect()
}

/// A length, written to the millimetre.
///
/// Finer than netconvert's own output, which rounds to the centimetre, so nothing is
/// lost on the way in.
fn metres(value: f64) -> String {
    format!("{value:.3}")
}

/// A polyline, as SUMO writes one: `x,y,z` triples separated by spaces.
fn shape(line: &Polyline3) -> String {
    line.points()
        .iter()
        .map(|point| {
            format!(
                "{},{},{}",
                metres(point.x),
                metres(point.y),
                metres(point.z)
            )
        })
        .collect::<Vec<_>>()
        .join(" ")
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn an_identifier_loses_what_sumo_cannot_hold() {
        assert_eq!(identifier("north arm"), "north_arm");
        assert_eq!(identifier("a:b"), "a_b");
        assert_eq!(identifier("plain"), "plain");
    }

    #[test]
    fn a_shape_is_written_as_sumo_reads_one() {
        let line =
            Polyline3::new([Point3::new(0.0, 1.0, 2.0), Point3::new(3.0, 4.0, 5.0)]).unwrap();
        assert_eq!(shape(&line), "0.000,1.000,2.000 3.000,4.000,5.000");
    }

    // ----------------------------------------------------------------------- //
    // The trace
    // ----------------------------------------------------------------------- //

    use quick_xml::events::Event;
    use roadgen_core::prelude::*;
    use roadgen_core::trace::{IrRef, TraceLink};

    fn two_way() -> Vec<LaneSpec> {
        let width = PositiveWidth::new(3.5).unwrap();
        vec![
            LaneSpec::new(width, Direction::Forward),
            LaneSpec::new(width, Direction::Backward),
        ]
    }

    fn metadata(name: &str) -> MapMetadata {
        MapMetadata {
            name: Some(name.to_owned()),
            ..MapMetadata::default()
        }
    }

    /// Four arms meeting at one junction, every pair of them connected, with a light
    /// on the northern approach.
    fn crossroads() -> ValidatedMap {
        let mut builder = MapBuilder::new(metadata("crossroads"));
        let arms = [
            (
                "north",
                Point3::new(0.0, 70.0, 0.0),
                Point3::new(0.0, 14.0, 0.0),
            ),
            (
                "east",
                Point3::new(70.0, 0.0, 0.0),
                Point3::new(14.0, 0.0, 0.0),
            ),
            (
                "south",
                Point3::new(0.0, -70.0, 0.0),
                Point3::new(0.0, -14.0, 0.0),
            ),
            (
                "west",
                Point3::new(-70.0, 0.0, 0.0),
                Point3::new(-14.0, 0.0, 0.0),
            ),
        ];
        let roads: Vec<RoadId> = arms
            .into_iter()
            .map(|(name, start, end)| {
                builder
                    .add_road(
                        RoadSpec::line(start, end, two_way())
                            .unwrap()
                            .with_name(name),
                    )
                    .unwrap()
            })
            .collect();
        let junction = builder.add_junction(Some("x"));
        for (index, from) in roads.iter().enumerate() {
            for to in roads.iter().skip(index + 1) {
                builder
                    .connect_ends(from, RoadEnd::End, to, RoadEnd::End, Some(&junction))
                    .unwrap();
            }
        }
        builder
            .add_traffic_light(&LaneRef::new(roads[0].clone(), 0), LaneEnd::End, 5.0)
            .unwrap();
        builder.finish().unwrap().validate().unwrap()
    }

    /// Two roads joined end to end, with no junction between them.
    fn in_line() -> ValidatedMap {
        let mut builder = MapBuilder::new(metadata("in-line"));
        let a = builder
            .add_road(
                RoadSpec::line(
                    Point3::new(0.0, 0.0, 0.0),
                    Point3::new(100.0, 0.0, 0.0),
                    two_way(),
                )
                .unwrap()
                .with_name("a"),
            )
            .unwrap();
        let b = builder
            .add_road(
                RoadSpec::line(
                    Point3::new(100.0, 0.0, 0.0),
                    Point3::new(200.0, 0.0, 0.0),
                    two_way(),
                )
                .unwrap()
                .with_name("b"),
            )
            .unwrap();
        builder.connect(&a, &b).unwrap();
        builder.finish().unwrap().validate().unwrap()
    }

    /// Every `<kind>:<local>` the rendered files actually contain, read back from the
    /// XML rather than from the exporter's state.
    fn written(network: &PlainNetwork) -> BTreeSet<String> {
        let mut found = BTreeSet::new();
        let mut edge = String::new();
        for (_, contents) in network.files() {
            let mut reader = quick_xml::Reader::from_str(contents);
            loop {
                let event = reader.read_event().unwrap();
                let element = match &event {
                    Event::Eof => break,
                    Event::Start(element) | Event::Empty(element) => element,
                    _ => continue,
                };
                let attributes: HashMap<String, String> = element
                    .attributes()
                    .map(|attribute| {
                        let attribute = attribute.unwrap();
                        (
                            String::from_utf8_lossy(attribute.key.as_ref()).into_owned(),
                            String::from_utf8_lossy(&attribute.value).into_owned(),
                        )
                    })
                    .collect();
                match element.name().as_ref() {
                    b"node" => {
                        found.insert(format!("node:{}", attributes["id"]));
                    }
                    b"edge" => {
                        edge = attributes["id"].clone();
                        found.insert(format!("edge:{edge}"));
                    }
                    b"lane" => {
                        found.insert(format!("lane:{edge}_{}", attributes["index"]));
                    }
                    b"connection" => {
                        found.insert(format!(
                            "connection:{}_{}>{}_{}",
                            attributes["from"],
                            attributes["fromLane"],
                            attributes["to"],
                            attributes["toLane"]
                        ));
                    }
                    _ => {}
                }
            }
        }
        found
    }

    #[test]
    fn everything_the_trace_names_is_in_the_files() {
        for map in [crossroads(), in_line()] {
            let network = to_plain_xml(&map).unwrap();
            let written = written(&network);
            assert_eq!(network.trace.format, "sumo");
            assert!(network.trace.files.is_empty());
            for link in &network.trace.links {
                assert!(
                    written.contains(&link.local),
                    "{} is traced to {}, which was not written",
                    link.ir,
                    link.local
                );
            }
            // And every connection written is accounted for by the IR.
            for local in written
                .iter()
                .filter(|local| local.starts_with("connection:"))
            {
                assert!(
                    network.trace.links_to(local).next().is_some(),
                    "{local} has no IR behind it"
                );
            }
        }
    }

    #[test]
    fn every_written_lane_has_one_exact_link_matching_its_id() {
        for map in [crossroads(), in_line()] {
            let network = to_plain_xml(&map).unwrap();
            assert!(!network.lanes.is_empty());
            for (lane, id) in &network.lanes {
                let exact: Vec<&str> = network
                    .trace
                    .links_of(&IrRef::Lane(lane.clone()))
                    .filter(|link| link.relation == Relation::Exact)
                    .map(|link| link.local.as_str())
                    .collect();
                assert_eq!(exact, vec![format!("lane:{id}")], "{lane}");
            }
        }
    }

    #[test]
    fn a_movement_through_a_junction_is_one_connection() {
        let map = crossroads();
        let network = to_plain_xml(&map).unwrap();
        let trace = &network.trace;

        let junction = &map.junctions.iter().next().unwrap().id;
        let exact: Vec<_> = trace.links_of(&IrRef::Junction(junction.clone())).collect();
        assert_eq!(exact.len(), 1);
        assert_eq!(exact[0].local, "node:j_x");
        assert_eq!(exact[0].relation, Relation::Exact);

        let light = map
            .objects
            .iter()
            .find(|object| object.kind.is_traffic_light())
            .unwrap();
        let signal: Vec<_> = trace.links_of(&IrRef::Object(light.id.clone())).collect();
        assert_eq!(signal.len(), 1);
        assert_eq!(signal[0].local, "node:j_x");
        assert_eq!(signal[0].relation, Relation::Merged);
        assert_eq!(signal[0].role.as_deref(), Some("traffic_light"));

        // A two-way arm is two edges.
        let north = map
            .roads
            .iter()
            .find(|road| road.name.as_deref() == Some("north"))
            .unwrap();
        let edges: Vec<&str> = trace
            .links_of(&IrRef::Road(north.id.clone()))
            .map(|link| link.local.as_str())
            .collect();
        assert_eq!(edges, ["edge:north.fwd", "edge:north.bwd"]);

        // Every connector lane is collapsed into exactly one written connection, and
        // every IR connection in the junction is merged into one.
        for road in map.roads.iter().filter(|road| road.is_connector()) {
            for lane in map.lanes.iter().filter(|lane| lane.road == road.id) {
                let links: Vec<_> = trace.links_of(&IrRef::Lane(lane.id.clone())).collect();
                assert_eq!(links.len(), 1, "{}", lane.id);
                assert_eq!(links[0].relation, Relation::Collapsed);
                assert!(links[0].local.starts_with("connection:"));
            }
        }
        for connection in map.connections.iter() {
            let links: Vec<_> = trace
                .links_of(&IrRef::Connection(connection.id.clone()))
                .collect();
            assert_eq!(links.len(), 1, "{}", connection.id);
            assert_eq!(links[0].relation, Relation::Merged);
        }
        assert!(trace
            .links
            .iter()
            .any(|link| link.local == "connection:north.fwd_0>west.bwd_0"));
    }

    #[test]
    fn a_connection_between_two_roads_is_exact() {
        let map = in_line();
        let network = to_plain_xml(&map).unwrap();
        let connections: Vec<&TraceLink> = network
            .trace
            .links
            .iter()
            .filter(|link| link.local.starts_with("connection:"))
            .collect();
        assert!(!connections.is_empty());
        for link in connections {
            assert_eq!(link.relation, Relation::Exact, "{}", link.local);
            assert!(matches!(link.ir, IrRef::Connection(_)));
        }
    }

    // ----------------------------------------------------------------------- //
    // Stop lines
    // ----------------------------------------------------------------------- //

    /// A tee whose northern arm changes cross-section partway along, with a stop line
    /// eight metres back from the junction on its approach, another at the end of
    /// the approach's first section, and one right at the mouth of the eastern arm.
    fn stop_lines() -> (ValidatedMap, [ObjectId; 3]) {
        let mut builder = MapBuilder::new(metadata("stops"));
        let north = builder
            .add_road(
                RoadSpec::line(
                    Point3::new(0.0, 70.0, 0.0),
                    Point3::new(0.0, 14.0, 0.0),
                    two_way(),
                )
                .unwrap()
                .with_name("north")
                .with_cross_section(30.0, two_way()),
            )
            .unwrap();
        let others: Vec<RoadId> = [
            (
                Point3::new(70.0, 0.0, 0.0),
                Point3::new(14.0, 0.0, 0.0),
                "east",
            ),
            (
                Point3::new(-70.0, 0.0, 0.0),
                Point3::new(-14.0, 0.0, 0.0),
                "west",
            ),
        ]
        .into_iter()
        .map(|(start, end, name)| {
            builder
                .add_road(
                    RoadSpec::line(start, end, two_way())
                        .unwrap()
                        .with_name(name),
                )
                .unwrap()
        })
        .collect();
        let junction = builder.add_junction(Some("t"));
        for other in &others {
            builder
                .connect_ends(&north, RoadEnd::End, other, RoadEnd::End, Some(&junction))
                .unwrap();
        }
        // The northern road's lanes are counted across both its cross-sections: 0
        // and 1 are the first, 2 and 3 the second, the one that meets the junction.
        let approach = LaneRef::new(north.clone(), 2);
        let set_back = builder
            .add_stop_line_at(&approach, LaneEnd::End, 8.0)
            .unwrap();
        let mid_road = builder
            .add_stop_line(&LaneRef::new(north.clone(), 0), LaneEnd::End)
            .unwrap();
        let at_mouth = builder
            .add_stop_line(&LaneRef::new(others[0].clone(), 0), LaneEnd::End)
            .unwrap();
        builder.add_right_of_way(
            vec![LaneRef::new(others[0].clone(), 0)],
            vec![approach],
            Some(set_back.clone()),
        );
        let map = builder.finish().unwrap().validate().unwrap();
        (map, [set_back, mid_road, at_mouth])
    }

    /// Every `<stopOffset>` in the edge file, by the `<edge>_<index>` of its lane.
    fn stop_offsets(network: &PlainNetwork) -> BTreeMap<String, f64> {
        let mut found = BTreeMap::new();
        let mut reader = quick_xml::Reader::from_str(&network.edges);
        let (mut edge, mut lane) = (String::new(), String::new());
        loop {
            let event = reader.read_event().unwrap();
            let element = match &event {
                Event::Eof => break,
                Event::Start(element) | Event::Empty(element) => element,
                _ => continue,
            };
            let attribute = |key: &str| {
                element
                    .try_get_attribute(key)
                    .unwrap()
                    .map(|attribute| String::from_utf8_lossy(&attribute.value).into_owned())
            };
            match element.name().as_ref() {
                b"edge" => edge = attribute("id").unwrap(),
                b"lane" => lane = format!("{edge}_{}", attribute("index").unwrap()),
                b"stopOffset" => {
                    assert!(
                        attribute("vClasses").is_none(),
                        "a stop line binds every class"
                    );
                    found.insert(lane.clone(), attribute("value").unwrap().parse().unwrap());
                }
                _ => {}
            }
        }
        found
    }

    #[test]
    fn a_stop_line_is_the_stop_offset_of_the_lane_it_crosses_into_the_junction() {
        let (map, [set_back, mid_road, at_mouth]) = stop_lines();
        let network = to_plain_xml(&map).unwrap();

        // Only the edge that reaches the junction carries it, and only the stop line
        // set back from the mouth says anything SUMO would not assume anyway.
        let offsets = stop_offsets(&network);
        assert_eq!(offsets.keys().collect::<Vec<_>>(), ["north.1.fwd_0"]);
        assert!((offsets["north.1.fwd_0"] - 8.0).abs() < 1e-3, "{offsets:?}");
        let approach = LaneId::of_road(&RoadId::new("north"), 2);
        assert_eq!(network.lanes[&approach], "north.1.fwd_0");

        let links: Vec<&TraceLink> = network
            .trace
            .links_of(&IrRef::Object(set_back.clone()))
            .collect();
        assert_eq!(links.len(), 1, "{links:?}");
        assert_eq!(links[0].local, "lane:north.1.fwd_0");
        assert_eq!(links[0].relation, Relation::Merged);
        assert_eq!(links[0].role.as_deref(), Some("stopOffset"));
        for object in [&mid_road, &at_mouth] {
            assert!(network
                .trace
                .links_of(&IrRef::Object(object.clone()))
                .next()
                .is_none());
        }

        // The line at the end of the first section is short of a node that is not a
        // junction, and is said to be lost; the one at the mouth loses nothing.
        let report = check(&map).join("\n");
        assert!(report.contains("written as the stopOffset"), "{report}");
        assert!(
            report.contains("does not end at a junction") && report.contains(mid_road.as_str()),
            "{report}"
        );
        assert!(!report.contains(at_mouth.as_str()), "{report}");
        assert!(!report.contains(set_back.as_str()), "{report}");
    }

    #[test]
    fn a_stop_line_across_both_carriageways_is_missing_only_where_it_is_not_written() {
        // The set-back line redrawn across the whole road, naming the approach and
        // the lane beside it that leaves the junction, as an imported line would.
        let (map, [set_back, ..]) = stop_lines();
        let mut map = UnvalidatedMap::from_map(map.into_map());
        let approach = LaneId::of_road(&RoadId::new("north"), 2);
        let leaving = LaneId::of_road(&RoadId::new("north"), 3);
        let object = map
            .as_map_mut()
            .objects
            .get_mut(&set_back)
            .expect("the set-back stop line");
        object.geometry = ObjectGeometry::Line(
            Curve3::line(Point3::new(-7.0, 22.0, 0.0), Point3::new(7.0, 22.0, 0.0)).unwrap(),
        );
        object.lanes = vec![approach.clone(), leaving.clone()];
        let map = map.validate().unwrap();
        let network = to_plain_xml(&map).unwrap();

        // Written where it stops traffic into the junction...
        let offsets = stop_offsets(&network);
        assert_eq!(offsets.keys().collect::<Vec<_>>(), ["north.1.fwd_0"]);
        assert!((offsets["north.1.fwd_0"] - 8.0).abs() < 1e-3, "{offsets:?}");
        assert_eq!(network.lanes[&approach], "north.1.fwd_0");

        // ...and said to be missing only on the lane that leaves it.
        let report = check(&map);
        let about: Vec<&String> = report
            .iter()
            .filter(|line| line.contains(set_back.as_str()))
            .collect();
        assert_eq!(about.len(), 1, "{report:#?}");
        assert!(
            about[0].contains(&format!("is not written on lane {}:", leaving.local_name()))
                && about[0].contains("does not end at a junction"),
            "{report:#?}"
        );
        // The line's own name carries its lane's, so look for the approach as a lane.
        assert!(
            !about[0].contains("on lanes ")
                && !about[0].contains(&format!("lane {}", approach.local_name())),
            "{report:#?}"
        );
    }

    #[test]
    fn a_stop_line_across_a_road_between_junctions_stops_only_the_lane_approaching_it() {
        // A road from one junction to another, with a line eight metres short of the
        // second drawn across both carriageways, as an imported line with no
        // orientation is: its lane leaving the first junction must not take an
        // offset of nearly the whole road.
        let mut builder = MapBuilder::new(metadata("between"));
        let road = |builder: &mut MapBuilder, name: &str, from: f64, to: f64| {
            builder
                .add_road(
                    RoadSpec::line(
                        Point3::new(from, 0.0, 0.0),
                        Point3::new(to, 0.0, 0.0),
                        two_way(),
                    )
                    .unwrap()
                    .with_name(name),
                )
                .unwrap()
        };
        let west = road(&mut builder, "west", -100.0, -10.0);
        let mid = road(&mut builder, "mid", 0.0, 100.0);
        let east = road(&mut builder, "east", 110.0, 200.0);
        let ja = builder.add_junction(Some("ja"));
        let jb = builder.add_junction(Some("jb"));
        builder
            .connect_ends(&west, RoadEnd::End, &mid, RoadEnd::Start, Some(&ja))
            .unwrap();
        builder
            .connect_ends(&mid, RoadEnd::End, &east, RoadEnd::Start, Some(&jb))
            .unwrap();
        let line = builder
            .add_stop_line_at(&LaneRef::new(mid.clone(), 0), LaneEnd::End, 8.0)
            .unwrap();
        let mut map =
            UnvalidatedMap::from_map(builder.finish().unwrap().validate().unwrap().into_map());
        let approaching = LaneId::of_road(&mid, 0);
        let leaving = LaneId::of_road(&mid, 1);
        let object = map
            .as_map_mut()
            .objects
            .get_mut(&line)
            .expect("the stop line");
        object.geometry = ObjectGeometry::Line(
            Curve3::line(Point3::new(92.0, -7.0, 0.0), Point3::new(92.0, 7.0, 0.0)).unwrap(),
        );
        object.lanes = vec![approaching.clone(), leaving.clone()];
        let map = map.validate().unwrap();
        let network = to_plain_xml(&map).unwrap();

        let offsets = stop_offsets(&network);
        let written = network.lanes[&approaching].clone();
        assert_eq!(
            offsets.keys().collect::<Vec<_>>(),
            [&written],
            "{offsets:?}"
        );
        assert!((offsets[&written] - 8.0).abs() < 1e-3, "{offsets:?}");

        let report = check(&map);
        let about: Vec<&String> = report
            .iter()
            .filter(|line_| line_.contains(line.as_str()) && line_.contains("is not written"))
            .collect();
        assert_eq!(about.len(), 1, "{report:#?}");
        assert!(
            about[0].contains(&format!("is not written on lane {}:", leaving.local_name()))
                && about[0]
                    .contains("it is at the lane's start, not before the junction it runs into"),
            "{report:#?}"
        );
    }

    #[test]
    fn a_map_without_stop_lines_says_nothing_about_them() {
        let report = check(&in_line()).join("\n");
        assert!(!report.contains("stop line"), "{report}");
        assert!(stop_offsets(&to_plain_xml(&in_line()).unwrap()).is_empty());
    }

    #[test]
    fn a_crossing_is_measured_back_from_the_end_of_the_path() {
        let path = Polyline3::new([
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(10.0, 0.0, 0.0),
            Point3::new(10.0, 10.0, 0.0),
        ])
        .unwrap();
        let across = |x: f64, y: f64, dx: f64, dy: f64| {
            Polyline3::new([
                Point3::new(x - dx, y - dy, 0.0),
                Point3::new(x + dx, y + dy, 0.0),
            ])
            .unwrap()
        };
        let near = |value: Option<f64>, expected: f64| {
            value.is_some_and(|value| (value - expected).abs() < 1e-9)
        };
        assert!(near(
            crossing_from_end(&path, &across(4.0, 0.0, 0.0, 2.0)),
            16.0
        ));
        assert!(near(
            crossing_from_end(&path, &across(10.0, 7.0, 2.0, 0.0)),
            3.0
        ));
        // Drawn exactly to the end, it is at the end.
        assert!(near(
            crossing_from_end(&path, &across(10.0, 10.0, 2.0, 0.0)),
            0.0
        ));
        // Beside the path, or along it, it crosses nothing.
        assert!(crossing_from_end(&path, &across(4.0, 5.0, 0.0, 2.0)).is_none());
        assert!(crossing_from_end(&path, &across(4.0, 0.0, 2.0, 0.0)).is_none());
    }

    /// A road none of whose lanes SUMO can carry leaves nothing behind: no edge, and
    /// no node for an edge that was never written to run between.
    #[test]
    fn a_road_with_no_sumo_lanes_writes_no_nodes() {
        let mut builder = MapBuilder::new(metadata("verge"));
        builder
            .add_road(
                RoadSpec::line(
                    Point3::new(0.0, 0.0, 0.0),
                    Point3::new(100.0, 0.0, 0.0),
                    two_way(),
                )
                .unwrap()
                .with_name("street"),
            )
            .unwrap();
        let width = PositiveWidth::new(2.0).unwrap();
        builder
            .add_road(
                RoadSpec::line(
                    Point3::new(0.0, 50.0, 0.0),
                    Point3::new(100.0, 50.0, 0.0),
                    vec![
                        LaneSpec::new(width, Direction::Forward).with_type(LaneType::Border),
                        LaneSpec::new(width, Direction::Backward).with_type(LaneType::Parking),
                    ],
                )
                .unwrap()
                .with_name("verge"),
            )
            .unwrap();
        let map = builder.finish().unwrap().validate().unwrap();

        let mut exporter = Exporter::new(&map);
        exporter.build().unwrap();
        assert!(exporter
            .edges
            .iter()
            .all(|edge| edge.id.starts_with("street")));
        let reached: BTreeSet<&str> = exporter
            .edges
            .iter()
            .flat_map(|edge| [edge.from.as_str(), edge.to.as_str()])
            .collect();
        let nodes: BTreeSet<&str> = exporter.nodes.keys().map(String::as_str).collect();
        assert_eq!(nodes, reached, "every node written should be an edge's end");
    }

    /// A right-of-way rule between footways must still move their edges apart.
    ///
    /// A footway is the bottom of the priority ladder, so its yielding approach is the
    /// one edge that could fall off it. It has to land below the approach it yields
    /// to, and below the carriageways the rule leaves alone, or the rule is lost.
    #[test]
    fn a_yielding_footway_still_ranks_below_the_one_it_yields_to() {
        let mut builder = MapBuilder::new(metadata("footways"));
        let arms: Vec<RoadId> = [
            (
                "north",
                Point3::new(0.0, 70.0, 0.0),
                Point3::new(0.0, 14.0, 0.0),
            ),
            (
                "east",
                Point3::new(70.0, 0.0, 0.0),
                Point3::new(14.0, 0.0, 0.0),
            ),
        ]
        .into_iter()
        .map(|(name, start, end)| {
            builder
                .add_road(
                    RoadSpec::line(start, end, two_way())
                        .unwrap()
                        .with_name(name)
                        .with_type(RoadType::Pedestrian),
                )
                .unwrap()
        })
        .collect();
        let junction = builder.add_junction(Some("x"));
        builder
            .connect_ends(
                &arms[0],
                RoadEnd::End,
                &arms[1],
                RoadEnd::End,
                Some(&junction),
            )
            .unwrap();
        builder.add_right_of_way(
            vec![LaneRef::new(arms[0].clone(), 0)],
            vec![LaneRef::new(arms[1].clone(), 0)],
            None,
        );
        let map = builder.finish().unwrap().validate().unwrap();

        let mut exporter = Exporter::new(&map);
        exporter.build().unwrap();
        let priority = |id: &str| {
            exporter
                .edges
                .iter()
                .find(|edge| edge.id == id)
                .unwrap_or_else(|| panic!("no edge {id}"))
                .priority
        };
        let untouched = priority("north.bwd");
        assert_eq!(priority("east.bwd"), untouched);
        assert!(priority("north.fwd") > untouched);
        assert!(
            priority("east.fwd") < untouched,
            "the footway that yields should rank below the ones the rule left alone"
        );
        assert!(priority("east.fwd") >= 1);
    }

    #[test]
    fn the_trace_is_the_same_every_time() {
        let map = crossroads();
        let first = to_plain_xml(&map).unwrap();
        for _ in 0..3 {
            assert_eq!(to_plain_xml(&map).unwrap().trace, first.trace);
        }
    }

    #[test]
    fn a_written_trace_names_the_files() {
        let map = in_line();
        let directory =
            std::env::temp_dir().join(format!("roadgen-sumo-trace-{}", std::process::id()));
        let (prefix, trace) = write_traced(&map, &directory).unwrap();
        let names: Vec<String> = ["nod.xml", "edg.xml", "con.xml", "netccfg"]
            .iter()
            .map(|suffix| format!("{prefix}.{suffix}"))
            .collect();
        assert_eq!(
            trace.files,
            names
                .iter()
                .map(|name| directory.join(name))
                .collect::<Vec<_>>()
        );
        assert!(trace.files.iter().all(|path| path.is_file()));
        assert_eq!(trace.links, to_plain_xml(&map).unwrap().trace.links);
        std::fs::remove_dir_all(&directory).unwrap();
    }

    // ----------------------------------------------------------------------- //
    // What the edges file says about each lane
    // ----------------------------------------------------------------------- //

    /// One `<edge>` of the edges file as it was written: its own attributes, and
    /// each of its `<lane>`s' in document order.
    struct WrittenEdge {
        attributes: HashMap<String, String>,
        lanes: Vec<HashMap<String, String>>,
    }

    impl WrittenEdge {
        fn number(&self, key: &str) -> f64 {
            self.attributes[key].parse().unwrap()
        }
    }

    /// The edges file, read back from the XML rather than from the exporter's state,
    /// so that what is checked is what netconvert would be handed.
    fn edges_file(network: &PlainNetwork) -> BTreeMap<String, WrittenEdge> {
        let mut edges = BTreeMap::new();
        let mut current: Option<String> = None;
        let mut reader = quick_xml::Reader::from_str(&network.edges);
        loop {
            let event = reader.read_event().unwrap();
            let element = match &event {
                Event::Eof => break,
                Event::Start(element) | Event::Empty(element) => element,
                _ => continue,
            };
            let attributes: HashMap<String, String> = element
                .attributes()
                .map(|attribute| {
                    let attribute = attribute.unwrap();
                    (
                        String::from_utf8_lossy(attribute.key.as_ref()).into_owned(),
                        quick_xml::escape::unescape(&String::from_utf8_lossy(&attribute.value))
                            .unwrap()
                            .into_owned(),
                    )
                })
                .collect();
            match element.name().as_ref() {
                b"edge" => {
                    let id = attributes["id"].clone();
                    edges.insert(
                        id.clone(),
                        WrittenEdge {
                            attributes,
                            lanes: Vec::new(),
                        },
                    );
                    current = Some(id);
                }
                b"lane" => {
                    let edge = current.as_ref().expect("a lane inside an edge");
                    edges.get_mut(edge).unwrap().lanes.push(attributes);
                }
                _ => {}
            }
        }
        edges
    }

    fn one_road(spec: RoadSpec) -> ValidatedMap {
        let mut builder = MapBuilder::new(metadata("one-road"));
        builder.add_road(spec).unwrap();
        builder.finish().unwrap().validate().unwrap()
    }

    fn lane(width: f64) -> LaneSpec {
        LaneSpec::new(PositiveWidth::new(width).unwrap(), Direction::Forward)
    }

    /// A cycle lane and a hard shoulder each carry the one class they exist for, as
    /// an `allow` on the lane — the lowering [`classes::permission`] states, checked
    /// here where it lands, in the file.
    #[test]
    fn a_cycle_lane_and_a_shoulder_are_written_with_their_own_class() {
        let map = one_road(
            RoadSpec::line(
                Point3::ORIGIN,
                Point3::new(120.0, 0.0, 0.0),
                vec![
                    lane(3.5),
                    lane(1.5).with_type(LaneType::Biking),
                    lane(2.5).with_type(LaneType::Shoulder),
                ],
            )
            .unwrap()
            .with_name("street"),
        );
        let network = to_plain_xml(&map).unwrap();
        let edges = edges_file(&network);
        let street = &edges["street.fwd"];
        assert_eq!(street.attributes["numLanes"], "3");

        // Outermost first: the shoulder is the kerbside lane 0, the cycle lane
        // inside it and the driving lane innermost.
        let class = |index: usize| {
            let lane = &street.lanes[index];
            (
                lane.get("allow").map(String::as_str),
                lane.get("disallow").map(String::as_str),
            )
        };
        assert_eq!(class(0), (Some("emergency"), None));
        assert_eq!(class(1), (Some("bicycle"), None));
        assert_eq!(class(2), (None, Some("pedestrian")));
    }

    /// A lane SUMO has no place for leaves no hole behind it. The edge's lane count
    /// is the lanes actually written, they are numbered 0, 1, 2 … with nothing
    /// skipped, and the dropped lanes are nowhere in the files or in the lane map —
    /// even when they sit *between* lanes that are written, which is where a
    /// numbering taken from the IR's lane index would leave a gap.
    #[test]
    fn a_dropped_lane_leaves_no_gap_in_the_numbering() {
        let map = one_road(
            RoadSpec::line(
                Point3::ORIGIN,
                Point3::new(120.0, 0.0, 0.0),
                vec![
                    lane(3.5),
                    lane(2.5).with_type(LaneType::Restricted),
                    lane(3.5),
                    lane(2.5).with_type(LaneType::Parking),
                    lane(0.5).with_type(LaneType::Border),
                    lane(2.0).with_type(LaneType::Sidewalk),
                ],
            )
            .unwrap()
            .with_name("street"),
        );
        let network = to_plain_xml(&map).unwrap();
        let edges = edges_file(&network);
        assert_eq!(edges.len(), 1, "every lane runs forward, so one edge");
        let street = &edges["street.fwd"];

        assert_eq!(street.attributes["numLanes"], "3");
        let indices: Vec<&str> = street
            .lanes
            .iter()
            .map(|lane| lane["index"].as_str())
            .collect();
        assert_eq!(indices, ["0", "1", "2"]);
        // The sidewalk outermost, the two driving lanes inside it.
        assert_eq!(
            street.lanes[0].get("allow").map(String::as_str),
            Some("pedestrian")
        );
        for driving in &street.lanes[1..] {
            assert_eq!(
                driving.get("disallow").map(String::as_str),
                Some("pedestrian")
            );
        }

        // The lane map and the trace know only the written lanes, and name them by
        // the contiguous indices.
        let written: BTreeSet<&str> = network.lanes.values().map(String::as_str).collect();
        assert_eq!(
            written,
            BTreeSet::from(["street.fwd_0", "street.fwd_1", "street.fwd_2"])
        );
        for dropped in map.lanes.iter().filter(|lane| {
            matches!(
                lane.lane_type,
                LaneType::Restricted | LaneType::Parking | LaneType::Border
            )
        }) {
            assert!(!network.lanes.contains_key(&dropped.id), "{}", dropped.id);
            assert!(
                network
                    .trace
                    .links_of(&IrRef::Lane(dropped.id.clone()))
                    .next()
                    .is_none(),
                "{} was dropped but is traced",
                dropped.id
            );
        }
    }

    /// A lane on a road 260 m long that runs at 2 m, widens to 5 m between 60 m and
    /// 100 m, holds 5 m to 160 m and narrows back to 2 m by 200 m.
    fn lay_by(taper: Taper) -> ValidatedMap {
        let width = |metres: f64| PositiveWidth::new(metres).unwrap();
        let profile = WidthProfile::new(
            [
                (60.0, width(2.0)),
                (100.0, width(5.0)),
                (160.0, width(5.0)),
                (200.0, width(2.0)),
            ],
            taper,
        )
        .unwrap();
        one_road(
            RoadSpec::line(
                Point3::ORIGIN,
                Point3::new(260.0, 0.0, 0.0),
                vec![lane(3.5), lane(2.0).with_width_profile(profile)],
            )
            .unwrap()
            .with_name("layby"),
        )
    }

    /// The width written for a tapering lane is its mean along — the area of the lane
    /// over its length — and not the width at either end or halfway along.
    ///
    /// The lay-by's area is a sum of trapezoids that can be done by hand: 2 m over
    /// the 120 m outside the bay, 5 m over the 60 m of it, and each 40 m ramp
    /// averaging 3.5 m, so 820 m² over 260 m. A smooth taper eases in and out
    /// symmetrically, `3t² − 2t³` having a mean of exactly ½ over the ramp, so it
    /// covers the same ground as the straight one and has the same mean; it is
    /// measured from samples along the curve rather than exactly, so it is held to
    /// the centimetre rather than to the millimetre the file is written to.
    #[test]
    fn a_tapering_lane_is_written_at_its_mean_width_along() {
        let analytic = (2.0 * 120.0 + 5.0 * 60.0 + 3.5 * 80.0) / 260.0;
        for (taper, tolerance) in [(Taper::Linear, 0.0006), (Taper::Smooth, 0.01)] {
            let map = lay_by(taper);
            let network = to_plain_xml(&map).unwrap();
            let edges = edges_file(&network);
            let layby = &edges["layby.fwd"];
            // The bay is the outer lane, so the kerbside lane 0.
            let written: f64 = layby.lanes[0]["width"].parse().unwrap();
            assert!(
                (written - analytic).abs() < tolerance,
                "{taper:?}: written at {written} m, but the mean along is {analytic} m"
            );
            // Not any of the widths the lane actually has at a landmark.
            for landmark in [2.0, 3.5, 5.0] {
                assert!((written - landmark).abs() > 0.1, "{taper:?}: {written}");
            }
            // And the lane that does not taper is written at its own width.
            assert_eq!(layby.lanes[1]["width"], "3.500");
        }
    }

    /// A lane with a limit of its own carries it as the lane's `speed`, which in
    /// SUMO overrides the edge's — here the road's own limit. (A lane on a
    /// road that states no limit at all writes no `speed`, and runs at its road
    /// type's fallback: see the test after this one.)
    #[test]
    fn a_lane_speed_limit_is_written_on_the_lane() {
        let map = one_road(
            RoadSpec::line(
                Point3::ORIGIN,
                Point3::new(120.0, 0.0, 0.0),
                vec![
                    lane(3.5).with_speed_limit(SpeedLimit::from_kph(30.0).unwrap()),
                    lane(3.5),
                ],
            )
            .unwrap()
            .with_name("street")
            .with_speed_limit(SpeedLimit::from_kph(50.0).unwrap()),
        );
        let network = to_plain_xml(&map).unwrap();
        let edges = edges_file(&network);
        let street = &edges["street.fwd"];
        assert_eq!(street.attributes["speed"], metres(50.0 / 3.6));

        let limited = map
            .lanes
            .iter()
            .find(|lane| lane.speed_limit.is_some())
            .unwrap();
        let (_, index) = network.lanes[&limited.id].rsplit_once('_').unwrap();
        let index: usize = index.parse().unwrap();
        assert_eq!(
            street.lanes[index].get("speed").map(String::as_str),
            Some(metres(30.0 / 3.6).as_str())
        );
        // The builder hands the road's limit down to a lane that states none, so the
        // other lane carries the road's 50 km/h as a limit of its own.
        assert_eq!(
            street.lanes[1 - index].get("speed").map(String::as_str),
            Some(metres(50.0 / 3.6).as_str())
        );
    }

    const LADDER: [RoadType; 5] = [
        RoadType::Motorway,
        RoadType::Rural,
        RoadType::Town,
        RoadType::LowSpeed,
        RoadType::Pedestrian,
    ];

    /// One unconnected road of each type, named after it.
    fn one_road_of_each_type() -> ValidatedMap {
        let mut builder = MapBuilder::new(metadata("ladder"));
        for (row, road_type) in LADDER.into_iter().enumerate() {
            let y = 50.0 * row as f64;
            builder
                .add_road(
                    RoadSpec::line(
                        Point3::new(0.0, y, 0.0),
                        Point3::new(120.0, y, 0.0),
                        vec![lane(3.5)],
                    )
                    .unwrap()
                    .with_name(road_type.as_str())
                    .with_type(road_type),
                )
                .unwrap();
        }
        builder.finish().unwrap().validate().unwrap()
    }

    /// A road that states no limit runs at its type's fallback speed, and the edges
    /// of a map rank by type the way [`classes::priority`] ladders them. Only the
    /// order is checked, not the numbers: it is the order netconvert reads to decide
    /// who yields, and the numbers are free to move as long as it holds.
    #[test]
    fn a_road_type_sets_the_edge_speed_and_its_place_in_the_priority_order() {
        let map = one_road_of_each_type();
        let network = to_plain_xml(&map).unwrap();
        let edges = edges_file(&network);
        let edge = |road_type: RoadType| &edges[&format!("{}.fwd", road_type.as_str())];

        for road_type in LADDER {
            assert_eq!(
                edge(road_type).attributes["speed"],
                metres(classes::default_speed(road_type)),
                "{road_type:?}"
            );
            // No lane states a limit, so none writes a speed of its own.
            assert!(!edge(road_type).lanes[0].contains_key("speed"));
        }
        for pair in LADDER.windows(2) {
            assert!(
                edge(pair[0]).number("priority") > edge(pair[1]).number("priority"),
                "a {:?} road should outrank a {:?} one",
                pair[0],
                pair[1]
            );
            assert!(
                edge(pair[0]).number("speed") > edge(pair[1]).number("speed"),
                "a {:?} road should be faster than a {:?} one",
                pair[0],
                pair[1]
            );
        }
    }

    /// What [`identifier`] does to each character SUMO cannot take in a name it did
    /// not generate: whitespace of every kind and every colon, wherever it is, become
    /// an underscore. Everything else is passed through as it is — including `#`,
    /// which SUMO's own OpenStreetMap import writes into the edges it splits
    /// (`123#0`, `123#1`) and so takes in an id, and `.`, `-` and `/`, which it
    /// takes as well.
    #[test]
    fn an_identifier_replaces_every_whitespace_and_colon_and_keeps_the_rest() {
        assert_eq!(identifier("a\tb\nc\r d"), "a_b_c__d");
        assert_eq!(identifier(":internal"), "_internal");
        assert_eq!(identifier("a::b:"), "a__b_");
        assert_eq!(identifier("a\u{a0}b"), "a_b");
        assert_eq!(identifier("route#2"), "route#2");
        assert_eq!(identifier("a.b-c/d"), "a.b-c/d");
        assert_eq!(identifier("Ginza-dōri"), "Ginza-dōri");
        assert_eq!(identifier(""), "");
    }

    /// The identifier is what every written id is made of, so a road named with a
    /// space and a colon is written without either — in its edges, its nodes and the
    /// lane map — while its `name` keeps what the map called it.
    #[test]
    fn a_road_name_sumo_cannot_hold_is_made_into_one_it_can() {
        let map = one_road(
            RoadSpec::line(
                Point3::ORIGIN,
                Point3::new(120.0, 0.0, 0.0),
                vec![
                    LaneSpec::new(PositiveWidth::new(3.5).unwrap(), Direction::Forward),
                    LaneSpec::new(PositiveWidth::new(3.5).unwrap(), Direction::Backward),
                ],
            )
            .unwrap()
            .with_name("north arm:2"),
        );
        let network = to_plain_xml(&map).unwrap();
        let edges = edges_file(&network);
        assert_eq!(
            edges.keys().map(String::as_str).collect::<Vec<_>>(),
            ["north_arm_2.bwd", "north_arm_2.fwd"]
        );
        let forward = &edges["north_arm_2.fwd"];
        assert_eq!(forward.attributes["name"], "north arm:2");
        assert_eq!(forward.attributes["from"], "n_north_arm_2_start");
        assert_eq!(forward.attributes["to"], "n_north_arm_2_end");
        for id in network.lanes.values() {
            assert!(id.starts_with("north_arm_2."), "{id}");
        }
        for local in written(&network) {
            assert!(
                !local.contains(' ') && !local.contains("arm:"),
                "{local} still holds a character SUMO cannot"
            );
        }
    }
}
