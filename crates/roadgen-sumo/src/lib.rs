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
//! | `<name>.tll.xml` | the program of each signalised junction, when there is one |
//! | `<name>.netccfg` | the netconvert run that turns the rest into a `.net.xml` |
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
//! # Traffic lights
//!
//! A junction a light governs an approach to is a `traffic_light` node, and its
//! traffic light is named after it — the node `j_x` is controlled by the light `j_x`,
//! written as the node's `tl` — so that the name does not depend on how netconvert
//! would have chosen one. The IR holds no timing, so the program is a fixed-time one
//! decided here (see [`signals`]) and written to `<name>.tll.xml`: the phases as a
//! `<tlLogic>`, and beside them every connection into the node again, with that
//! light's `tl` and the `linkIndex` it is given. netconvert then has nothing to
//! generate, and the position of each movement in each state string is the export's,
//! which is what lets the trace say which slot a light of the map controls.
//!
//! # The trace
//!
//! [`PlainNetwork::trace`] records where each element of the IR went, naming the
//! written elements as follows:
//!
//! | written element | IR element | relation |
//! | --- | --- | --- |
//! | `node:<id>` | a junction | exact |
//! | `tls:<id>` | a traffic light, or a traffic-light rule, role `traffic_light` | merged |
//! | `tls:<id>/<link index>` | a traffic light, for each controlled connection leaving a lane it governs, role `link` | merged |
//! | `edge:<id>` | a road — one edge per direction and cross-section | part |
//! | `lane:<edge>_<index>` | a lane | exact |
//! | `connection:<from edge>_<from lane>><to edge>_<to lane>` | a lane connection | exact, or merged |
//! | the same connection | a connector lane it runs over | collapsed |
//!
//! A light's program is named by its SUMO id, the `id` of its `<tlLogic>`, and each
//! slot in it by that id and the `linkIndex` of the connection the slot controls — the
//! position of that connection's signal in every `state` of the program. A light
//! governs the lanes it stands on and the lanes any rule naming it lists, and so the
//! slots of every movement off those lanes; the connection itself is the one written
//! with that `tl` and `linkIndex` in `<name>.tll.xml`.
//!
//! A connection is named by its two lanes as the built network names them, so the
//! `.net.xml` `<connection from fromLane to toLane via>` that netconvert writes for it
//! is found from the trace by the same `<edge>_<index>` pair — and with it the
//! internal lane (`via`) netconvert generated in place of the connector. A movement
//! through a junction is two IR connections and one connector lane, all of them
//! written as the one connection, so each is `merged` or `collapsed` into it.

pub mod classes;
pub mod error;
pub mod signals;
mod xml;

use std::collections::{BTreeMap, BTreeSet, HashMap, HashSet};
use std::path::Path;

use roadgen_core::geometry::{Point3, Polyline3, SamplingConfig};
use roadgen_core::map::{Lane, Road};
use roadgen_core::semantics::{MapObjectKind, TrafficRule};
use roadgen_core::topology::{Direction, LaneEnd, RoadEnd, RoadLinkTarget};
use roadgen_core::trace::{IrRef, Relation, Trace};
use roadgen_core::{ConnectionId, JunctionId, LaneId, ObjectId, RoadId, ValidatedMap};

pub use classes::Permission;
pub use error::ExportError;

/// The name a map with none of its own is written under.
const DEFAULT_NAME: &str = "network";

/// A SUMO plain-XML network: the input files, and the netconvert run that turns
/// them into a `.net.xml`.
///
/// The file names are derived from [`PlainNetwork::prefix`], and the configuration
/// refers to them, so they belong together in one directory.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PlainNetwork {
    /// What the files are named before their `.nod.xml`, `.edg.xml`, `.con.xml`,
    /// `.tll.xml` and `.netccfg` suffixes.
    pub prefix: String,
    pub nodes: String,
    pub edges: String,
    pub connections: String,
    /// The program of every signalised junction, as a `.tll.xml`; `None` for a map
    /// with no traffic light, which is written without one.
    pub traffic_lights: Option<String>,
    pub config: String,
    /// Where each lane of the IR ended up, as the `<edge>_<index>` identifier the
    /// built network gives it.
    ///
    /// A SUMO lane's identity is positional, so this is the only way back from a
    /// lane of the map to the lane of the network — and the numbering is not the
    /// IR's: SUMO counts from the right of the direction of travel.
    pub lanes: BTreeMap<LaneId, String>,
    /// Where each element of the IR ended up in the files: nodes, edges, lanes,
    /// connections and traffic lights, by the identifiers written for them. See the
    /// crate documentation for how each is named.
    ///
    /// Its `files` are empty: the network is in memory until [`write_traced`] puts it
    /// somewhere.
    pub trace: Trace,
}

impl PlainNetwork {
    /// Each file's name and its contents, in the order netconvert reads them.
    pub fn files(&self) -> Vec<(String, &str)> {
        let mut files = vec![
            (format!("{}.nod.xml", self.prefix), self.nodes.as_str()),
            (format!("{}.edg.xml", self.prefix), self.edges.as_str()),
            (
                format!("{}.con.xml", self.prefix),
                self.connections.as_str(),
            ),
        ];
        if let Some(traffic_lights) = &self.traffic_lights {
            files.push((format!("{}.tll.xml", self.prefix), traffic_lights.as_str()));
        }
        files.push((format!("{}.netccfg", self.prefix), self.config.as_str()));
        files
    }

    /// Writes the files into `directory`, creating it if it is not there.
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
/// was written alongside the prefix, with the files' paths in it.
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

/// Whether `id` names a traffic light of the map — not a missing object, and not an
/// object of another kind.
fn is_light(map: &ValidatedMap, id: &ObjectId) -> bool {
    map.objects
        .get(id)
        .is_some_and(|object| object.kind.is_traffic_light())
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

    let lights = map
        .objects
        .iter()
        .filter(|object| object.kind.is_traffic_light())
        .count();
    if lights > 0 {
        problems.push(format!(
            "the IR holds no signal timing, so the junctions of the {lights} traffic \
             lights get a fixed-time program of the export's own: opposing approaches \
             green together for {few} s ({many} s where a junction has more than two \
             groups of them), then {yellow} s yellow and {red} s all-red, with turns \
             across oncoming traffic permissive",
            few = signals::GREEN_FEW_SECONDS,
            many = signals::GREEN_MANY_SECONDS,
            yellow = signals::YELLOW_SECONDS,
            red = signals::ALL_RED_SECONDS,
        ));
    }

    let unresolved: Vec<String> = map
        .rules
        .iter()
        .filter_map(|rule| match rule {
            TrafficRule::TrafficLight { lights, .. } => Some(lights),
            _ => None,
        })
        .flatten()
        .filter(|light| !is_light(map, light))
        .map(|light| match map.objects.get(light) {
            Some(object) => format!("{light} (a {})", object.kind.as_str()),
            None => format!("{light} (missing)"),
        })
        .collect::<BTreeSet<_>>()
        .into_iter()
        .collect();
    if !unresolved.is_empty() {
        problems.push(format!(
            "a traffic-light rule names {} objects that are not traffic lights, which \
             govern nothing: a rule with no light left signalises no junction: {}",
            unresolved.len(),
            unresolved.join(", ")
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
    /// A signalised junction, whose program is written alongside it (see
    /// [`Signal`]).
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

/// The traffic light of one signalised node, and its program.
struct Signal {
    /// What the light is called in SUMO, which is the node's own id: a name that
    /// follows from the IR junction rather than from netconvert's choice of one.
    id: String,
    /// The connections it controls, by their key in [`Exporter::movements`], in
    /// link-index order: the n-th is the n-th signal of every state.
    links: Vec<(usize, usize, usize, usize)>,
    phases: Vec<signals::Phase>,
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
    /// The signalised junctions, each with the lights that govern it.
    signalised: BTreeMap<JunctionId, BTreeSet<ObjectId>>,
    /// The lanes each light governs: the ones it stands on, and the ones any
    /// traffic-light rule naming it lists.
    governed: BTreeMap<ObjectId, BTreeSet<LaneId>>,
    /// The traffic-light rules, by their index among the map's rules, each with the
    /// junctions its lanes run into.
    light_rules: BTreeMap<usize, BTreeSet<JunctionId>>,
    /// The light of each signalised node with something to control, by node id.
    signals: BTreeMap<String, Signal>,
    /// Lanes that keep right of way where another yields to them, and the lanes that
    /// yield. Held as lanes, not roads: a rule about one carriageway's approach must
    /// not move the opposing carriageway's priority with it.
    right_of_way: HashSet<LaneId>,
    yielding: HashSet<LaneId>,
    /// Junctions a right-of-way rule speaks about.
    ruled: HashSet<JunctionId>,
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
            governed: BTreeMap::new(),
            light_rules: BTreeMap::new(),
            signals: BTreeMap::new(),
            right_of_way: HashSet::new(),
            yielding: HashSet::new(),
            ruled: HashSet::new(),
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
                self.govern(&object.id, lane);
            }
        }
        // A rule says which lanes its lights govern in so many words, and they need
        // not be the lanes the lights stand on: a light on a gantry over one lane can
        // govern the whole approach.
        for (index, rule) in self.map.rules.iter().enumerate() {
            let TrafficRule::TrafficLight { lights, lanes, .. } = rule else {
                continue;
            };
            // Only a name that resolves to a traffic light governs anything. A rule
            // naming a missing object, or a sign or a stop line, passes validation —
            // which checks a rule's lanes, not its lights — but signalising a junction
            // on its word would write a program, and trace it, for a light the map
            // does not have. `check` reports the names skipped.
            let lights: Vec<&ObjectId> = lights
                .iter()
                .filter(|light| is_light(self.map, light))
                .collect();
            if lights.is_empty() {
                continue;
            }
            for lane in lanes {
                for light in &lights {
                    self.govern(light, lane);
                }
                if let Some(junction) = self.junction_ahead_of(lane) {
                    self.light_rules.entry(index).or_default().insert(junction);
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

    /// Records that `light` governs `lane`, and so signalises the junction it runs
    /// into.
    fn govern(&mut self, light: &ObjectId, lane: &LaneId) {
        self.governed
            .entry(light.clone())
            .or_default()
            .insert(lane.clone());
        if let Some(junction) = self.junction_ahead_of(lane) {
            self.signalised
                .entry(junction)
                .or_default()
                .insert(light.clone());
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
        self.build_signals();
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
    // Traffic lights
    // ----------------------------------------------------------------------- //

    /// The light and the program of every signalised node.
    ///
    /// A light controls *every* connection into its node, not only those off the
    /// approaches a light of the map stands on: SUMO's traffic light is a property of
    /// the junction, and a junction where one approach is signalled and the others
    /// merely give way is not a thing it models. Which approaches the map's lights do
    /// govern is what the trace records.
    ///
    /// The links are numbered in the order the connections are written, which is
    /// fixed, and the program is decided from the geometry the export wrote: each
    /// approach's heading where its edge meets the node, and each movement's swing
    /// from the end of the lane it leaves to the start of the lane it joins. See
    /// [`signals`] for what is made of them.
    fn build_signals(&mut self) {
        let handedness = self.map.metadata.handedness;
        for (id, node) in &self.nodes {
            if node.kind != NodeKind::TrafficLight {
                continue;
            }
            let keys: Vec<(usize, usize, usize, usize)> = self
                .movements
                .keys()
                .filter(|(from, ..)| self.edges[*from].to == *id)
                .copied()
                .collect();
            if keys.is_empty() {
                // Nothing to control, so nothing to program: the node stays a
                // `traffic_light`, and what netconvert makes of it is its own.
                continue;
            }
            let links: Vec<signals::Link> = keys
                .iter()
                .map(|&(from_edge, from_lane, to_edge, to_lane)| {
                    let leaving = &self.edges[from_edge].lanes[from_lane].shape;
                    let joining = &self.edges[to_edge].lanes[to_lane].shape;
                    signals::Link {
                        approach: from_edge,
                        heading: final_heading(&self.edges[from_edge].shape),
                        turn: signals::swing(final_heading(leaving), initial_heading(joining)),
                        target: (to_edge, to_lane),
                    }
                })
                .collect();
            let phases = signals::program(&links, handedness);
            self.signals.insert(
                id.clone(),
                Signal {
                    id: id.clone(),
                    links: keys,
                    phases,
                },
            );
        }
    }

    /// The light controlling each written connection that one does, and the link
    /// index it has there.
    fn link_indices(&self) -> HashMap<(usize, usize, usize, usize), (&str, usize)> {
        self.signals
            .values()
            .flat_map(|signal| {
                signal
                    .links
                    .iter()
                    .enumerate()
                    .map(|(index, key)| (*key, (signal.id.as_str(), index)))
            })
            .collect()
    }

    // ----------------------------------------------------------------------- //
    // Rendering
    // ----------------------------------------------------------------------- //

    fn render(&self, prefix: String) -> PlainNetwork {
        PlainNetwork {
            nodes: self.render_nodes(),
            edges: self.render_edges(),
            connections: self.render_connections(),
            traffic_lights: self.render_traffic_lights(),
            config: render_config(&prefix, !self.signals.is_empty()),
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
        // A light is part of the traffic light of the node it made one, as
        // `read_rules` decided when it chose which nodes are signalised — or, at a
        // node with nothing to control and so no program, part of the node.
        let link_indices = self.link_indices();
        for (junction, lights) in &self.signalised {
            let Some(node) = junction_nodes.get(junction) else {
                continue;
            };
            let signal = self.signals.get(*node);
            for light in lights {
                let local = match signal {
                    Some(signal) => format!("tls:{}", signal.id),
                    None => format!("node:{node}"),
                };
                trace.link_as(light.clone(), local, Relation::Merged, "traffic_light");
                // And of the slot of every movement off a lane it governs. A light on
                // a connector governs the movements that run over it.
                let Some(signal) = signal else { continue };
                let governed = self.governed.get(light);
                let governs = |lane: &LaneId| governed.is_some_and(|lanes| lanes.contains(lane));
                for key in &signal.links {
                    let from = &self.edges[key.0].lanes[key.1].lane;
                    let movement = &self.movements[key];
                    if governs(from) || movement.connectors.iter().any(governs) {
                        let (id, index) = link_indices[key];
                        trace.link_as(
                            light.clone(),
                            format!("tls:{id}/{index}"),
                            Relation::Merged,
                            "link",
                        );
                    }
                }
            }
        }
        // A traffic-light rule is carried by the program of each junction its lanes
        // run into, alongside the lights it names.
        for (index, junctions) in &self.light_rules {
            for junction in junctions {
                let Some(signal) = junction_nodes
                    .get(junction)
                    .and_then(|node| self.signals.get(*node))
                else {
                    continue;
                };
                trace.link_as(
                    IrRef::Rule(*index),
                    format!("tls:{}", signal.id),
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
            if let Some(signal) = self.signals.get(id) {
                // Named, so that netconvert does not name it.
                attributes.push(("tl", signal.id.clone()));
            }
            document.leaf("node", &attributes);
        }
        document.finish()
    }

    fn render_edges(&self) -> String {
        let mut document = xml::Document::new("edges", "http://sumo.dlr.de/xsd/edges_file.xsd");
        for edge in &self.edges {
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
                document.leaf("lane", &attributes);
            }
            document.close("edge");
        }
        document.finish()
    }

    fn render_connections(&self) -> String {
        let mut document =
            xml::Document::new("connections", "http://sumo.dlr.de/xsd/connections_file.xsd");
        for key in self.movements.keys() {
            document.leaf("connection", &self.connection_attributes(key));
        }
        document.finish()
    }

    /// The four attributes that name a written connection.
    fn connection_attributes(
        &self,
        &(from_edge, from_lane, to_edge, to_lane): &(usize, usize, usize, usize),
    ) -> Vec<(&'static str, String)> {
        vec![
            ("from", self.edges[from_edge].id.clone()),
            ("to", self.edges[to_edge].id.clone()),
            ("fromLane", from_lane.to_string()),
            ("toLane", to_lane.to_string()),
        ]
    }

    /// The `.tll.xml`: one static program per signalised node, and the link index
    /// of every connection it controls — or nothing at all for a map with no traffic
    /// light.
    ///
    /// The link indices go here rather than on the connections of the `.con.xml`,
    /// because that is where SUMO's schema has them: a connection file says which
    /// movements exist, and a traffic-light file says which signal each obeys. A
    /// connection named here is matched to the one the `.con.xml` declared by its
    /// two lanes. Stating the index rather than leaving netconvert to number the
    /// links is what makes the program's state strings mean what the export says they
    /// mean.
    fn render_traffic_lights(&self) -> Option<String> {
        if self.signals.is_empty() {
            return None;
        }
        let mut document =
            xml::Document::new("tlLogics", "http://sumo.dlr.de/xsd/tllogic_file.xsd");
        for signal in self.signals.values() {
            document.open(
                "tlLogic",
                &[
                    ("id", signal.id.clone()),
                    ("type", "static".to_owned()),
                    ("programID", signals::PROGRAM_ID.to_owned()),
                    ("offset", "0".to_owned()),
                ],
            );
            for phase in &signal.phases {
                document.leaf(
                    "phase",
                    &[
                        ("duration", phase.duration.to_string()),
                        ("state", phase.state.clone()),
                    ],
                );
            }
            document.close("tlLogic");
        }
        for signal in self.signals.values() {
            for (index, key) in signal.links.iter().enumerate() {
                let mut attributes = self.connection_attributes(key);
                attributes.push(("tl", signal.id.clone()));
                attributes.push(("linkIndex", index.to_string()));
                document.leaf("connection", &attributes);
            }
        }
        Some(document.finish())
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
///
/// The traffic-light file is named only when there is one, which is when the map has
/// a signalised junction with something to control.
fn render_config(prefix: &str, traffic_lights: bool) -> String {
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
    if traffic_lights {
        document.leaf("tllogic-files", &[("value", format!("{prefix}.tll.xml"))]);
    }
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

/// The heading a line leaves by: the direction of its last segment, radians
/// anticlockwise from +x, in the horizontal plane.
fn final_heading(line: &Polyline3) -> f64 {
    match line.points() {
        [.., a, b] => (b.y - a.y).atan2(b.x - a.x),
        _ => 0.0,
    }
}

/// The heading a line sets off on: the direction of its first segment.
fn initial_heading(line: &Polyline3) -> f64 {
    match line.points() {
        [a, b, ..] => (b.y - a.y).atan2(b.x - a.x),
        _ => 0.0,
    }
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
        crossroads_driving(TrafficHandedness::RightHand)
    }

    /// The same crossroads, on whichever side of the road traffic keeps to.
    fn crossroads_driving(handedness: TrafficHandedness) -> ValidatedMap {
        crossroads_controlled(handedness, |builder, roads| {
            builder
                .add_traffic_light(&LaneRef::new(roads[0].clone(), 0), LaneEnd::End, 5.0)
                .unwrap();
        })
    }

    /// The same crossroads, with its controls — lights, signs, rules — added by
    /// `control`, which is handed the four arms north, east, south, west.
    fn crossroads_controlled(
        handedness: TrafficHandedness,
        control: impl FnOnce(&mut MapBuilder, &[RoadId]),
    ) -> ValidatedMap {
        let mut builder = MapBuilder::new(MapMetadata {
            handedness,
            ..metadata("crossroads")
        });
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
        control(&mut builder, &roads);
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
                        if let (Some(tl), Some(index)) =
                            (attributes.get("tl"), attributes.get("linkIndex"))
                        {
                            found.insert(format!("tls:{tl}/{index}"));
                        }
                    }
                    b"tlLogic" => {
                        found.insert(format!("tls:{}", attributes["id"]));
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
        let signal: Vec<_> = trace
            .links_of(&IrRef::Object(light.id.clone()))
            .filter(|link| link.role.as_deref() == Some("traffic_light"))
            .collect();
        assert_eq!(signal.len(), 1);
        assert_eq!(signal[0].local, "tls:j_x");
        assert_eq!(signal[0].relation, Relation::Merged);

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

    // ----------------------------------------------------------------------- //
    // Traffic lights
    // ----------------------------------------------------------------------- //

    /// The attributes of every `element` in `xml`, in document order.
    fn elements(xml: &str, element: &str) -> Vec<HashMap<String, String>> {
        let mut reader = quick_xml::Reader::from_str(xml);
        let mut found = Vec::new();
        loop {
            match reader.read_event().unwrap() {
                Event::Eof => break,
                Event::Start(tag) | Event::Empty(tag)
                    if tag.name().as_ref() == element.as_bytes() =>
                {
                    found.push(
                        tag.attributes()
                            .map(|attribute| {
                                let attribute = attribute.unwrap();
                                (
                                    String::from_utf8_lossy(attribute.key.as_ref()).into_owned(),
                                    String::from_utf8_lossy(&attribute.value).into_owned(),
                                )
                            })
                            .collect(),
                    );
                }
                _ => {}
            }
        }
        found
    }

    #[test]
    fn a_signalised_node_names_its_light_after_itself() {
        let network = to_plain_xml(&crossroads()).unwrap();
        let nodes = elements(&network.nodes, "node");
        let junction = nodes.iter().find(|node| node["id"] == "j_x").unwrap();
        assert_eq!(junction["type"], "traffic_light");
        assert_eq!(junction["tl"], "j_x");
        // And no other node has a light.
        assert_eq!(
            nodes.iter().filter(|node| node.contains_key("tl")).count(),
            1
        );
    }

    /// The state of the movement `from` → `to` in each phase of the one program in
    /// `network`, read through the link index the export gave the movement.
    fn states(network: &PlainNetwork, from: &str, to: &str) -> String {
        let tll = network.traffic_lights.as_deref().expect("a .tll.xml");
        let link = elements(tll, "connection")
            .into_iter()
            .find(|connection| connection["from"] == from && connection["to"] == to)
            .unwrap_or_else(|| panic!("no controlled connection {from} → {to}"));
        let index: usize = link["linkIndex"].parse().unwrap();
        elements(tll, "phase")
            .iter()
            .map(|phase| phase["state"].as_bytes()[index] as char)
            .collect()
    }

    #[test]
    fn every_connection_into_a_signalised_node_has_its_own_link_index() {
        let network = to_plain_xml(&crossroads()).unwrap();
        let declared = elements(&network.connections, "connection");
        // Four arms, each turning into the other three.
        assert_eq!(declared.len(), 12);
        // The connection file stays SUMO's plain format: it says which movements
        // exist and nothing about signals.
        assert!(declared
            .iter()
            .all(|connection| !connection.contains_key("tl")));

        let tll = network.traffic_lights.as_deref().unwrap();
        let controlled = elements(tll, "connection");
        let name = |connection: &HashMap<String, String>| {
            ["from", "fromLane", "to", "toLane"].map(|key| connection[key].clone())
        };
        assert_eq!(
            controlled.iter().map(name).collect::<BTreeSet<_>>(),
            declared.iter().map(name).collect::<BTreeSet<_>>(),
            "every movement into the node is controlled, and only those"
        );
        let mut indices: Vec<usize> = controlled
            .iter()
            .map(|connection| {
                assert_eq!(connection["tl"], "j_x");
                connection["linkIndex"].parse().unwrap()
            })
            .collect();
        indices.sort();
        assert_eq!(indices, (0..12).collect::<Vec<_>>());
    }

    #[test]
    fn the_program_is_written_and_named_by_the_configuration() {
        let network = to_plain_xml(&crossroads()).unwrap();
        let tll = network.traffic_lights.as_deref().expect("a .tll.xml");
        let logics = elements(tll, "tlLogic");
        assert_eq!(logics.len(), 1);
        assert_eq!(logics[0]["id"], "j_x");
        assert_eq!(logics[0]["programID"], "0");
        assert_eq!(logics[0]["type"], "static");
        assert_eq!(logics[0]["offset"], "0");

        // Two groups of approaches, north-south and east-west, each a green, a
        // yellow and an all-red.
        let phases = elements(tll, "phase");
        let durations: Vec<&str> = phases
            .iter()
            .map(|phase| phase["duration"].as_str())
            .collect();
        assert_eq!(durations, ["35", "3", "2", "35", "3", "2"]);
        assert!(phases.iter().all(|phase| phase["state"].len() == 12));
        assert!(phases[2]["state"].chars().all(|state| state == 'r'));

        // The northern approach is written first, so its group is released first.
        // Straight on is protected. Every turn runs into a one-lane exit that the
        // opposing approach turns into too, and so gives way.
        assert_eq!(states(&network, "north.fwd", "south.bwd"), "Gyrrrr");
        assert_eq!(states(&network, "south.fwd", "north.bwd"), "Gyrrrr");
        assert_eq!(states(&network, "north.fwd", "east.bwd"), "gyrrrr");
        assert_eq!(states(&network, "north.fwd", "west.bwd"), "gyrrrr");
        assert_eq!(states(&network, "east.fwd", "west.bwd"), "rrrGyr");
        assert_eq!(states(&network, "west.fwd", "south.bwd"), "rrrgyr");

        assert!(network
            .config
            .contains(r#"<tllogic-files value="crossroads.tll.xml"/>"#));
        assert!(network
            .files()
            .iter()
            .any(|(name, _)| name == "crossroads.tll.xml"));
    }

    /// A tee with no movement between its two side arms: north runs straight on to
    /// south and turns into west, and nothing else turns into west from the north–
    /// south road. So the turn into west gives way only if it crosses the stream
    /// coming the other way — which depends on the side traffic keeps to.
    fn tee(handedness: TrafficHandedness) -> ValidatedMap {
        let mut builder = MapBuilder::new(MapMetadata {
            handedness,
            ..metadata("tee")
        });
        let mut arm = |name: &str, start: Point3, end: Point3| {
            builder
                .add_road(
                    RoadSpec::line(start, end, two_way())
                        .unwrap()
                        .with_name(name),
                )
                .unwrap()
        };
        let north = arm(
            "north",
            Point3::new(0.0, 70.0, 0.0),
            Point3::new(0.0, 14.0, 0.0),
        );
        let south = arm(
            "south",
            Point3::new(0.0, -70.0, 0.0),
            Point3::new(0.0, -14.0, 0.0),
        );
        let west = arm(
            "west",
            Point3::new(-70.0, 0.0, 0.0),
            Point3::new(-14.0, 0.0, 0.0),
        );
        let junction = builder.add_junction(Some("t"));
        for other in [&south, &west] {
            builder
                .connect_ends(&north, RoadEnd::End, other, RoadEnd::End, Some(&junction))
                .unwrap();
        }
        builder
            .add_traffic_light(&LaneRef::new(north, 0), LaneEnd::End, 5.0)
            .unwrap();
        builder.finish().unwrap().validate().unwrap()
    }

    #[test]
    fn the_turn_across_oncoming_traffic_follows_the_handedness() {
        // Heading south, west is on the right: a turn across nothing where traffic
        // keeps right, and across the northbound stream where it keeps left.
        let right = to_plain_xml(&tee(TrafficHandedness::RightHand)).unwrap();
        assert_eq!(states(&right, "north.fwd", "west.bwd"), "Gyrrrr");
        let left = to_plain_xml(&tee(TrafficHandedness::LeftHand)).unwrap();
        assert_eq!(states(&left, "north.fwd", "west.bwd"), "gyrrrr");

        // Straight on is protected either way, and the side arm runs on its own.
        for network in [&right, &left] {
            assert_eq!(states(network, "north.fwd", "south.bwd"), "Gyrrrr");
            assert_eq!(states(network, "west.fwd", "north.bwd"), "rrrGyr");
        }
    }

    #[test]
    fn a_map_without_lights_has_no_program() {
        let network = to_plain_xml(&in_line()).unwrap();
        assert!(network.traffic_lights.is_none());
        assert!(!network.config.contains("tllogic-files"));
        assert!(network
            .files()
            .iter()
            .all(|(name, _)| !name.ends_with(".tll.xml")));
    }

    #[test]
    fn a_light_is_traced_to_the_slots_of_the_lane_it_governs() {
        let map = crossroads();
        let network = to_plain_xml(&map).unwrap();
        let light = map
            .objects
            .iter()
            .find(|object| object.kind.is_traffic_light())
            .unwrap();
        let slots: BTreeSet<String> = network
            .trace
            .links_of(&IrRef::Object(light.id.clone()))
            .filter(|link| link.role.as_deref() == Some("link"))
            .map(|link| {
                assert_eq!(link.relation, Relation::Merged);
                link.local.clone()
            })
            .collect();
        // The light stands on the northern approach's lane 0, which turns into the
        // three other arms.
        let expected: BTreeSet<String> =
            elements(network.traffic_lights.as_deref().unwrap(), "connection")
                .iter()
                .filter(|connection| connection["from"] == "north.fwd")
                .map(|connection| format!("tls:j_x/{}", connection["linkIndex"]))
                .collect();
        assert_eq!(expected.len(), 3);
        assert_eq!(slots, expected);
    }

    #[test]
    fn a_rule_naming_no_light_signalises_nothing() {
        // A rule over the eastern approach that names a sign and an object the map
        // does not have: validation accepts it, but neither is a light.
        let map = crossroads_controlled(TrafficHandedness::RightHand, |builder, roads| {
            let sign = builder
                .add_traffic_sign(
                    &LaneRef::new(roads[1].clone(), 0),
                    LaneEnd::End,
                    "stop",
                    2.0,
                )
                .unwrap();
            builder.add_traffic_light_rule(
                vec![sign, ObjectId::new("nowhere")],
                None,
                vec![LaneRef::new(roads[1].clone(), 0)],
            );
        });
        let network = to_plain_xml(&map).unwrap();
        assert!(network.traffic_lights.is_none());
        assert!(!network.nodes.contains("traffic_light"));
        assert!(network
            .trace
            .links
            .iter()
            .all(|link| !link.local.starts_with("tls:")));
        let problems = check(&map);
        let skipped = problems
            .iter()
            .find(|problem| problem.contains("not traffic lights"))
            .expect("the skipped names are reported");
        assert!(skipped.contains("object/nowhere (missing)"), "{skipped}");
        assert!(skipped.contains("(a traffic_sign)"), "{skipped}");
    }

    #[test]
    fn a_rule_naming_a_light_and_a_stranger_signalises_with_the_light_only() {
        let map = crossroads_controlled(TrafficHandedness::RightHand, |builder, roads| {
            let light = builder
                .add_traffic_light(&LaneRef::new(roads[0].clone(), 0), LaneEnd::End, 5.0)
                .unwrap();
            builder.add_traffic_light_rule(
                vec![light, ObjectId::new("nowhere")],
                None,
                vec![LaneRef::new(roads[1].clone(), 0)],
            );
        });
        let network = to_plain_xml(&map).unwrap();
        assert!(network.traffic_lights.is_some());
        assert_eq!(
            network
                .trace
                .links_of(&IrRef::Object(ObjectId::new("nowhere")))
                .count(),
            0
        );
        assert!(network.trace.links_of(&IrRef::Rule(0)).count() > 0);
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
}
