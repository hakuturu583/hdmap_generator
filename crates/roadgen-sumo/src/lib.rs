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
//! measured against. Lanes within an edge are numbered from the **outside** of the
//! carriageway — the kerb side, which is the right in the direction of travel where
//! traffic drives on the right — so the numbering of the same physical lane differs
//! between the two carriageways. That is SUMO's convention and not a choice made
//! here.
//!
//! # Left-hand traffic
//!
//! SUMO does not read handedness off the geometry: a network is right-hand unless
//! netconvert is told otherwise, and what it is told decides far more than which side
//! the lanes are drawn on. It decides which movement crosses oncoming traffic and so
//! must give way to it, which side the internal lanes of a junction are laid out on,
//! and which lane is the slow one. So a map whose metadata says it drives on the left
//! is written with `lefthand` in its configuration, and netconvert records it in the
//! `.net.xml` for the simulator.
//!
//! The lane numbering follows. In a left-hand network SUMO's index 0 is still the
//! outer lane, and the outer lane is now the **leftmost** in the direction of travel
//! — which is what netconvert itself lays out when it spreads an edge's lanes with
//! `lefthand` set, and what a vehicle keeping to its side drives in.
//!
//! A road whose cross-section changes becomes one edge per cross-section, joined at
//! an internal node: a SUMO edge has one lane count from end to end. Which lane
//! carries on across that node is the IR's connections, written as the movements
//! through it; for an edge the IR carries nothing on from, its movements onto the
//! edges beyond are listed as deleted, so netconvert guesses none.
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
//! lane, and the connector's centreline goes with it as the connection's `shape`.
//! netconvert lays the internal lane along that shape, so the path across the
//! junction is the IR's — the same curve the OpenDRIVE and Lanelet2 exports carry —
//! even though the lane that follows it is netconvert's.
//!
//! This is why the arms are left exactly where the IR puts them, short of the
//! junction: the gap is the junction, and netconvert fills it along the connectors.
//!
//! Footways are the exception. SUMO's pedestrians do not walk along connections:
//! they cross a node on a *walking area* that netconvert builds there, joining every
//! footway that meets at the node, walked either way. So the pavement the IR lays
//! round a junction corner — a connector between two sidewalks — is not written as a
//! connection, and neither is any other movement from one footway to another; the
//! configuration asks netconvert for walking areas instead.
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
//! | `edge:<id>` | a right-of-way rule naming its lanes, role `priority` | merged |
//! | `lane:<edge>_<index>` | a lane | exact |
//! | `lane:<edge>_<index>` | a speed-limit rule naming the lane, role `speed` | merged |
//! | `connection:<from edge>_<from lane>><to edge>_<to lane>` | a lane connection | exact, or merged |
//! | the same connection | a connector lane it runs over | collapsed |
//! | `node:<id>`, role `walkingarea` | a connection between two footways, and a pavement lane round a corner | collapsed |
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
//!
//! # Where on the globe
//!
//! The node file opens with a `<location>`: the PROJ definition of the frame the
//! map's metres are read against, and the offset that takes a projected position to
//! the network's own. netconvert carries it into the `.net.xml` unchanged, so the
//! built network is georeferenced — `sumolib`'s `convertXY2LonLat` turns a junction
//! back into the latitude and longitude the IR puts it at — while its coordinates
//! stay the IR's metres, the same as every other export of the map. See
//! [`location`] for which projection each of the map's projections becomes.

pub mod classes;
pub mod error;
pub mod location;
pub mod signals;
mod xml;

use std::collections::{BTreeMap, BTreeSet, HashMap, HashSet};
use std::path::Path;

use roadgen_core::geometry::{Point3, Polyline3, SamplingConfig};
use roadgen_core::map::{Lane, Projection, Road, TrafficHandedness};
use roadgen_core::semantics::{LaneType, MapObjectKind, TrafficRule};
use roadgen_core::topology::{
    Direction, LaneConnection, LaneEnd, LaneEndpoint, RoadEnd, RoadLinkTarget,
};
use roadgen_core::trace::{IrRef, Relation, Trace};
use roadgen_core::{ConnectionId, JunctionId, LaneId, ObjectId, RoadId, ValidatedMap};

pub use classes::Permission;
pub use error::ExportError;
pub use location::{geo_reference, height_error, GeoReference};

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
    /// IR's: SUMO counts from the outside of the carriageway, which is the right of
    /// the direction of travel under right-hand traffic and the left under left-hand.
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

    let footway = |lane: &LaneId| {
        map.lane(lane)
            .is_some_and(|lane| lane.lane_type == LaneType::Sidewalk)
    };
    let walks = map
        .connections
        .iter()
        .filter(|connection| footway(&connection.from.lane) && footway(&connection.to.lane))
        .count();
    if walks > 0 {
        problems.push(format!(
            "SUMO's pedestrians cross a node on the walking area netconvert builds \
             there, not along connections, so the {walks} connections between \
             footways — the pavements round junction corners among them — are not \
             written: at each node, a pedestrian may walk between any of the footways \
             that meet there, in either direction"
        ));
    }

    problems.extend(speed_limit_problems(map));

    if !map.junctions.is_empty() {
        problems.push(format!(
            "the connector roads of the {} junctions are not written as edges: SUMO \
             builds an internal lane per movement from the connections instead. Each \
             connection carries its connectors' centreline as its shape, so the path \
             across a junction is the IR's, but the junction's outline and where a \
             turn waits inside it are netconvert's",
            map.junctions.len()
        ));
    }

    let unshaped = parallel_connectors(map);
    if !unshaped.is_empty() {
        problems.push(format!(
            "a SUMO connection has one shape, but the IR draws more than one way across \
             a junction between some pairs of lanes; each is written as one connection \
             along the first of its ways, and the rest are not drawn: {}",
            unshaped.join(", ")
        ));
    }
    problems.extend(Ids::new(map).renamed);

    let altitude = map.metadata.origin.altitude();
    if altitude != 0.0 {
        problems.push(match map.metadata.projection {
            Projection::LocalCartesian | Projection::Mgrs => format!(
                "a SUMO `<location>` ties the network to the globe horizontally only: \
                 the heights are the map's own z, and the origin's altitude of \
                 {altitude} m is not written as a height — it is in the transverse \
                 Mercator's scale, which keeps the horizontal positions those of the \
                 east/north/up frame at that altitude"
            ),
            Projection::Utm => format!(
                "a SUMO `<location>` ties the network to the globe horizontally only: \
                 the heights are the map's own z, and the origin's altitude of \
                 {altitude} m is not written"
            ),
        });
    }

    // What a 2D `<location>` cannot carry: each point's own height above the
    // origin's plane. Measured along the reference lines, which is where the heights
    // are; a lane's edge is never more than a few metres from its road's.
    let worst = map
        .roads
        .iter()
        .filter_map(|road| road.reference_line.samples(map.metadata.sampling).ok())
        .flatten()
        .map(|sample| height_error(map, &sample.point))
        .fold(0.0, f64::max);
    if worst >= 0.01 {
        problems.push(format!(
            "a SUMO `<location>` has no height, so `convertXY2LonLat` places a point \
             above or below the origin's plane as if it were on it: up to {worst:.3} m \
             from the latitude and longitude the Lanelet2 export gives it (the point's \
             distance from the origin times its height, over the earth's radius)"
        ));
    }
    problems
}

/// What of the map's `SpeedLimit` rules does not reach a lane `speed`.
///
/// A rule names lanes, and an IR lane runs the whole length of one cross-section of
/// its road — which is exactly what one lane of one edge is — so wherever the lane it
/// names is written, the rule is written exactly. What is lost is the rest:
///
/// - a lane on a **connector road**, because the connectors are not written as edges
///   and the internal lane netconvert builds in their place takes the speed
///   netconvert works out for the turn;
/// - a lane of a type SUMO has **no lane for**, because nothing is written for it to
///   go on;
/// - the **higher** of two rules naming the same lane, because a lane has one speed
///   and the lower of them is the one a driver has to keep to.
fn speed_limit_problems(map: &ValidatedMap) -> Vec<String> {
    let mut on_connectors = BTreeSet::new();
    let mut on_unwritten = BTreeSet::new();
    let mut limits: BTreeMap<&LaneId, BTreeSet<u64>> = BTreeMap::new();
    for rule in &map.rules {
        let TrafficRule::SpeedLimit { limit, lanes } = rule else {
            continue;
        };
        for id in lanes {
            let Some(lane) = map.lane(id) else {
                continue;
            };
            if map.road(&lane.road).is_some_and(Road::is_connector) {
                on_connectors.insert(id.as_str());
            } else if classes::permission(lane.lane_type).is_none() {
                on_unwritten.insert(id.as_str());
            } else {
                // Compared as bit patterns so a set can hold them; two limits that
                // are the same number are the same rule as far as the lane goes.
                limits.entry(id).or_default().insert(limit.mps().to_bits());
            }
        }
    }
    let contested: Vec<&str> = limits
        .into_iter()
        .filter(|(_, limits)| limits.len() > 1)
        .map(|(lane, _)| lane.as_str())
        .collect();

    let mut problems = Vec::new();
    if !on_connectors.is_empty() {
        problems.push(format!(
            "the connector roads are not written as edges, so a speed-limit rule on \
             the {} connector lanes it names is not written: netconvert sets the speed \
             of the internal lane it builds for each movement: {}",
            on_connectors.len(),
            on_connectors.into_iter().collect::<Vec<_>>().join(", ")
        ));
    }
    if !on_unwritten.is_empty() {
        problems.push(format!(
            "a speed-limit rule on the {} lanes SUMO has no lane for is dropped with \
             them: {}",
            on_unwritten.len(),
            on_unwritten.into_iter().collect::<Vec<_>>().join(", ")
        ));
    }
    if !contested.is_empty() {
        problems.push(format!(
            "a SUMO lane has one speed, so where speed-limit rules disagree about a \
             lane the lowest of them is written and the others are dropped: {}",
            contested.join(", ")
        ));
    }
    problems
}

/// Pairs of written lanes the IR joins by more than one way through connectors, as
/// `from > to`. The exporter writes one connection per pair, and a connection carries
/// one shape, so every way but the first goes unwritten; `check()` names them.
///
/// The walk is the one `build_connections` makes: out of a lane SUMO has a place for,
/// through any number of connector lanes, to the next lane SUMO has a place for. Ways
/// are counted rather than lanes reached, so two connectors side by side between the
/// same lanes count twice where one connector reached twice does not.
fn parallel_connectors(map: &ValidatedMap) -> Vec<String> {
    let mut successors: BTreeMap<&LaneId, Vec<&LaneId>> = BTreeMap::new();
    for connection in map.connections.iter() {
        successors
            .entry(&connection.from.lane)
            .or_default()
            .push(&connection.to.lane);
    }
    let on_connector = |lane: &LaneId| {
        map.lane(lane)
            .and_then(|lane| map.road(&lane.road))
            .is_some_and(Road::is_connector)
    };
    let written = |lane: &LaneId| {
        !on_connector(lane)
            && map
                .lane(lane)
                .is_some_and(|lane| classes::permission(lane.lane_type).is_some())
    };

    let mut found = Vec::new();
    for source in map
        .lanes
        .iter()
        .map(|lane| &lane.id)
        .filter(|id| written(id))
    {
        // Every way out of the source, each one a stack entry with the connector
        // lanes it has crossed so far, so a way that loops back on itself stops.
        let mut ways: BTreeMap<&LaneId, usize> = BTreeMap::new();
        let mut pending: Vec<(&LaneId, Vec<&LaneId>)> = vec![(source, Vec::new())];
        while let Some((lane, crossed)) = pending.pop() {
            for &next in successors.get(lane).into_iter().flatten() {
                if written(next) {
                    if !crossed.is_empty() {
                        *ways.entry(next).or_default() += 1;
                    }
                } else if on_connector(next) && !crossed.contains(&next) {
                    let mut further = crossed.clone();
                    further.push(next);
                    pending.push((next, further));
                }
            }
        }
        found.extend(
            ways.into_iter()
                .filter(|&(_, count)| count > 1)
                .map(|(target, _)| format!("{source} > {target}")),
        );
    }
    found
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
    /// The path across the junction, as the IR drew it: the centrelines of the
    /// connector lanes one way through, in travel order and joined end to end. Absent
    /// for a movement that crosses no connector — two roads that meet head on — where
    /// there is no drawing of the IR's to pass on and netconvert's own short internal
    /// lane is the right one.
    ///
    /// Only one way through is drawn, even where a junction holds several connectors
    /// between the same pair of lanes: a connection has one shape. The first one the
    /// walk found is kept, and `check()` names the pairs of lanes where that drops one.
    shape: Option<Polyline3>,
}

struct Exporter<'a> {
    map: &'a ValidatedMap,
    sampling: SamplingConfig,
    /// The name every id derived from a road or a junction is built from.
    ids: Ids,
    nodes: BTreeMap<String, Node>,
    edges: Vec<Edge>,
    /// Each written connection, by (from edge, from lane, to edge, to lane), and what
    /// of the IR it stands for. Ordered, and a map, because a junction with several
    /// connectors between one pair of lanes would otherwise write the same movement
    /// twice.
    movements: BTreeMap<(usize, usize, usize, usize), Movement>,
    /// What the IR says about walking from one footway to another, by the node it
    /// happens at. None of it is written as a connection: netconvert builds a
    /// walking area at every node footways meet at, and that is how SUMO's
    /// pedestrians get from one to the next. See [`Exporter::connect`].
    walking: BTreeMap<String, Movement>,
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
    /// The speed each lane named by a `SpeedLimit` rule is held to, in m/s, and the
    /// rules that hold it there. See [`Exporter::lane_speed`].
    rule_speeds: HashMap<LaneId, (f64, Vec<usize>)>,
    /// The attributes of the node file's `<location>`, worked out once everything
    /// it has to bound has been built.
    location: Vec<(&'static str, String)>,
}

impl<'a> Exporter<'a> {
    fn new(map: &'a ValidatedMap) -> Self {
        let mut exporter = Exporter {
            map,
            sampling: map.metadata.sampling,
            ids: Ids::new(map),
            nodes: BTreeMap::new(),
            edges: Vec::new(),
            movements: BTreeMap::new(),
            walking: BTreeMap::new(),
            slots: HashMap::new(),
            signalised: BTreeMap::new(),
            governed: BTreeMap::new(),
            light_rules: BTreeMap::new(),
            signals: BTreeMap::new(),
            right_of_way: HashSet::new(),
            yielding: HashSet::new(),
            ruled: HashSet::new(),
            rule_speeds: HashMap::new(),
            location: Vec::new(),
        };
        exporter.read_rules();
        exporter
    }

    /// The things about a map that are decided before any edge is written: which
    /// junctions are signalised, which arms hold right of way over which, and which
    /// lanes a speed-limit rule holds to a speed.
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

        for (index, rule) in self.map.rules.iter().enumerate() {
            let TrafficRule::SpeedLimit { limit, lanes } = rule else {
                continue;
            };
            let limit = limit.mps();
            for lane in lanes {
                let held = self
                    .rule_speeds
                    .entry(lane.clone())
                    .or_insert((limit, Vec::new()));
                if limit < held.0 {
                    *held = (limit, Vec::new());
                }
                // A rule that names one lane twice is still one rule over it.
                if limit == held.0 && held.1.last() != Some(&index) {
                    held.1.push(index);
                }
            }
        }
    }

    /// The speed a lane is written with, in m/s, or `None` to leave it the edge's.
    ///
    /// A `SpeedLimit` rule is the IR saying how fast the lanes it names may be driven,
    /// and where one names this lane it *is* the lane's limit: it replaces whatever
    /// limit the lane carried of its own, as it does in the Lanelet2 export, where the
    /// rule overwrites the lanelet's `speed_limit` tag. Where several rules name the
    /// same lane the lowest of them is the one a driver has to keep to, so that is the
    /// one written, and [`crate::check`] names the lane.
    ///
    /// Nothing is lost along the lane: an IR lane runs the whole of one cross-section
    /// of its road, and one cross-section is one edge, so the rule covers the SUMO
    /// lane from end to end exactly as it covers the IR lane.
    fn lane_speed(&self, lane: &Lane) -> Option<f64> {
        match self.rule_speeds.get(&lane.id) {
            Some((limit, _)) => Some(*limit),
            None => lane.speed_limit.map(|limit| limit.mps()),
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
                // SUMO draws that itself, from the connection, as an internal lane —
                // along the connector's centreline, which the connection carries.
                continue;
            }
            for section in 0..road.sections.len() {
                self.section_edges(road, section)?;
            }
        }
        self.build_connections()?;
        self.build_signals();
        let points = self
            .nodes
            .values()
            .map(|node| &node.point)
            .chain(self.edges.iter().flat_map(|edge| {
                edge.shape.points().iter().chain(
                    edge.lanes
                        .iter()
                        .flat_map(|lane| lane.shape.points().iter()),
                )
            }))
            .chain(
                self.movements
                    .values()
                    .filter_map(|movement| movement.shape.as_ref())
                    .flat_map(|shape| shape.points().iter()),
            );
        self.location = location::location(self.map, points)?;
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
                    format!("j_{}", self.ids.junction(&junction)),
                    point,
                    kind,
                    self.ruled.contains(&junction),
                    Some(junction),
                )
            }
            Some(RoadLinkTarget::Road(other)) => {
                // Both roads have to name the joint the same way, so the pair is put
                // in a fixed order and the first of the two names it.
                let here = (self.ids.road(&road.id), end);
                let there = (self.ids.road(&other.road), other.end);
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
                format!("n_{}_{}", self.ids.road(&road.id), end.as_str()),
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
            format!("n_{}_s{section}", self.ids.road(&road.id)),
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
    /// numbers them: index 0 is the outer lane — the rightmost in the direction of
    /// travel where traffic drives on the right, and the leftmost where it drives on
    /// the left.
    ///
    /// Ordered by where the lanes actually are rather than by which side of the
    /// reference line they sit on, because which side is the driver's right depends
    /// on the direction of travel — and, for the reference line itself, on whether
    /// the map drives on the left.
    ///
    /// The handedness matters a second time for the order itself. netconvert is told
    /// a left-hand map is one (see [`render_config`]), and a left-hand SUMO network
    /// counts its lanes from the left: lane 0 is the kerb lane a vehicle keeps to and
    /// the highest index the one beside the oncoming traffic. Numbering a left-hand
    /// carriageway from the right would hand SUMO its overtaking lane as its slow one,
    /// and every lane-change and turn-lane decision would be taken from the wrong
    /// side.
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
        if self.map.metadata.handedness == TrafficHandedness::LeftHand {
            lanes.reverse();
        }
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
                speed: self.lane_speed(lane),
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
            id: edge_id(&self.ids.road(&road.id), road, section, direction),
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
    ///
    /// `lanes` is in SUMO's order, outer lane first, so which end of it is the
    /// rightmost lane depends on the map's handedness.
    fn carriageway_shape(
        &self,
        lanes: &[&Lane],
        written: &[EdgeLane],
    ) -> Result<Polyline3, ExportError> {
        let (Some(outer), Some(inner)) = (lanes.first(), lanes.last()) else {
            return Err(ExportError::Unknown("an edge with no lanes".to_owned()));
        };
        let (rightmost, leftmost) = match self.map.metadata.handedness {
            TrafficHandedness::RightHand => (outer, inner),
            TrafficHandedness::LeftHand => (inner, outer),
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
    ///
    /// The connectors are not lost on the way, though: their centrelines, in the order
    /// the movement crosses them, become the connection's `shape`. netconvert takes a
    /// connection's shape as the shape of the internal lane it draws for it — clipped
    /// or stretched only to meet the junction's border, which here is where the arms
    /// stop and the connector starts anyway — so the path across the junction in SUMO
    /// is the one the IR drew, and the one the OpenDRIVE and the Lanelet2 map written
    /// from the same IR carry.
    fn build_connections(&mut self) -> Result<(), ExportError> {
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
            // The lane each lane was first reached from, which is one way back from
            // anywhere the walk got to: the path a connection's shape is drawn along.
            let mut reached_from: HashMap<LaneId, LaneId> = HashMap::new();
            let mut frontier = step(&source);
            while let Some((from, connection, next)) = frontier.pop() {
                walked.push((from.clone(), connection, next.clone()));
                if !seen.insert(next.clone()) {
                    continue;
                }
                reached_from.insert(next.clone(), from);
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
                let mut movement = self.movement_between(&source, &target, &walked);
                movement.shape = self.path_across(&source, &target, &reached_from)?;
                self.connect(&source, &target, movement);
            }
        }
        Ok(())
    }

    /// The connector centrelines between `source` and `target`, one after the other in
    /// travel order: the shape of the connection between the two.
    ///
    /// The way back is read from `reached_from`, so it is the path the walk first
    /// found. Where one connector runs into the next, the first one's last point and
    /// the second one's first are the same joint, and only one of them is kept — a
    /// repeated point is a zero-length segment, which netconvert would have to make a
    /// direction out of.
    fn path_across(
        &self,
        source: &LaneId,
        target: &LaneId,
        reached_from: &HashMap<LaneId, LaneId>,
    ) -> Result<Option<Polyline3>, ExportError> {
        let mut connectors: Vec<&LaneId> = Vec::new();
        let mut lane = target;
        while let Some(previous) = reached_from.get(lane) {
            if previous == source {
                break;
            }
            connectors.push(previous);
            lane = previous;
        }
        connectors.reverse();

        let mut points: Vec<Point3> = Vec::new();
        for connector in connectors {
            let lane = self
                .map
                .lane(connector)
                .ok_or_else(|| ExportError::Unknown(format!("connector lane {connector}")))?;
            let centreline = lane
                .travel_geometry(self.sampling)?
                .centerline
                .to_polyline(self.sampling)?;
            for &point in centreline.points() {
                // The joint between two connectors, or a sample on top of its
                // neighbour: a point within a millimetre of the last is the same point
                // once written.
                if points
                    .last()
                    .is_some_and(|last| last.distance_to(point) < 1e-3)
                {
                    continue;
                }
                points.push(point);
            }
        }
        if points.len() < 2 {
            return Ok(None);
        }
        Ok(Some(Polyline3::new(points)?))
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

    /// Records one movement between two written lanes — as a connection, unless both
    /// are footways.
    ///
    /// SUMO does not walk its pedestrians along connections. Where footways meet,
    /// netconvert builds a *walking area* — a patch of pavement joining every
    /// footway that ends or starts at the node, walked in any direction — and a
    /// pedestrian crosses the node on it. That is the model every SUMO network with
    /// pedestrians uses, and it is the one asked for here: the configuration sets
    /// `walkingareas` (see [`render_config`]).
    ///
    /// The IR's model is not that. A pavement round a junction corner is a connector
    /// road from one arm's sidewalk to the next arm's, and it is joined to them by a
    /// connection at each end, read in whatever direction the two sidewalks happen
    /// to run. Which sidewalk faces a corner depends on which end of each arm meets
    /// the junction and on the side traffic keeps to, so the pavement can run *from*
    /// a sidewalk that leaves the junction *to* one that arrives at it. Written as a
    /// SUMO connection, that is a connection from an edge that starts at the node to
    /// one that ends there, and netconvert refuses the network: "could not insert
    /// connection … after build". Even where the directions happen to line up, the
    /// connection would only say a pedestrian may walk round the corner one way, and
    /// it would draw an internal lane across the walking area that netconvert builds
    /// anyway.
    ///
    /// So a movement from footway to footway is not written. Where it happens — the
    /// node it crosses, see [`Exporter::walking_nodes`] — is kept instead, and the
    /// trace says that the IR's connections and pavement lanes became part of that
    /// node's walking area. A joint between two roads is treated the same way, for
    /// the same reason: the walking area netconvert builds there is what joins the
    /// two footways, in both directions.
    fn connect(&mut self, source: &LaneId, target: &LaneId, movement: Movement) {
        let (Some(&from), Some(&to)) = (self.slots.get(source), self.slots.get(target)) else {
            return;
        };
        if from.edge == to.edge {
            // Two lanes of one edge; SUMO says that with a lane change, not a
            // connection.
            return;
        }
        if self.is_footway(from) && self.is_footway(to) {
            for (node, part) in self.walking_nodes(source, target, movement) {
                let walked = self.walking.entry(node).or_default();
                walked.connections.extend(part.connections);
                walked.connectors.extend(part.connectors);
            }
            return;
        }
        let key = (from.edge, from.index, to.edge, to.index);
        let merged = self.movements.entry(key).or_default();
        merged.connections.extend(movement.connections);
        merged.connectors.extend(movement.connectors);
        if merged.shape.is_none() {
            merged.shape = movement.shape;
        }
    }

    /// Whether a written lane is one only pedestrians may use.
    fn is_footway(&self, slot: Slot) -> bool {
        self.edges[slot.edge].lanes[slot.index].permission
            == classes::permission(LaneType::Sidewalk)
    }

    /// The nodes a walk from footway `source` to footway `target` crosses, each with
    /// the part of `movement` that crosses it.
    ///
    /// Read from the ends the IR's connections name, not from the edges: the two
    /// sidewalks of one road are carried by its two edges, which share *both* their
    /// nodes, so the edges alone cannot say which end of the road a pavement joining
    /// them turns at. A connection out of the source names the end of the source it
    /// leaves by, and one into the target the end of the target it arrives at; each
    /// is a node of that lane's edge. A pavement lane between them — and the
    /// connections on from it — crosses the node of the connection it was reached
    /// by. The walk may cross more than one node, when the two sidewalks are joined
    /// round both ends of a road. Anything no connection places falls back to the
    /// node the two edges share (see [`Exporter::shared_node`]).
    fn walking_nodes(
        &self,
        source: &LaneId,
        target: &LaneId,
        movement: Movement,
    ) -> BTreeMap<String, Movement> {
        let connections: Vec<&LaneConnection> = movement
            .connections
            .iter()
            .filter_map(|id| self.map.connections.get(id))
            .collect();
        let mut connection_node: HashMap<&ConnectionId, String> = HashMap::new();
        let mut lane_node: HashMap<&LaneId, String> = HashMap::new();
        for connection in &connections {
            let placed = if &connection.from.lane == source {
                self.node_at(&connection.from)
                    .map(|node| (node, &connection.to.lane))
            } else if &connection.to.lane == target {
                self.node_at(&connection.to)
                    .map(|node| (node, &connection.from.lane))
            } else {
                None
            };
            if let Some((node, other)) = placed {
                lane_node.entry(other).or_insert_with(|| node.clone());
                connection_node.insert(&connection.id, node);
            }
        }
        // Carry each node on through the pavement lanes it was reached by.
        loop {
            let mut changed = false;
            for connection in &connections {
                if connection_node.contains_key(&connection.id) {
                    continue;
                }
                let Some(node) = lane_node
                    .get(&connection.from.lane)
                    .or_else(|| lane_node.get(&connection.to.lane))
                    .cloned()
                else {
                    continue;
                };
                for lane in [&connection.from.lane, &connection.to.lane] {
                    lane_node.entry(lane).or_insert_with(|| node.clone());
                }
                connection_node.insert(&connection.id, node);
                changed = true;
            }
            if !changed {
                break;
            }
        }

        let fallback = || self.shared_node(self.slots[source].edge, self.slots[target].edge);
        let mut parts: BTreeMap<String, Movement> = BTreeMap::new();
        for connection in movement.connections {
            let node = connection_node
                .get(&connection)
                .cloned()
                .unwrap_or_else(fallback);
            parts
                .entry(node)
                .or_default()
                .connections
                .insert(connection);
        }
        for connector in movement.connectors {
            let node = lane_node.get(&connector).cloned().unwrap_or_else(fallback);
            parts.entry(node).or_default().connectors.insert(connector);
        }
        parts
    }

    /// The node at one end of a written lane: the end of its edge that the lane's
    /// end, counted along the reference line, lies on.
    fn node_at(&self, endpoint: &LaneEndpoint) -> Option<String> {
        let slot = self.slots.get(&endpoint.lane)?;
        let direction = self.map.lane(&endpoint.lane)?.direction;
        let edge = &self.edges[slot.edge];
        // A forward edge runs from the section's start to its end; a backward edge
        // the other way.
        Some(match (direction, endpoint.end) {
            (Direction::Forward, LaneEnd::Start) | (Direction::Backward, LaneEnd::End) => {
                edge.from.clone()
            }
            (Direction::Forward, LaneEnd::End) | (Direction::Backward, LaneEnd::Start) => {
                edge.to.clone()
            }
        })
    }

    /// The node two edges meet at, for a walk no connection places.
    ///
    /// Read from the edges' own ends rather than from the direction of travel,
    /// because a movement between two footways need not follow it: the pavement
    /// round a corner can start on a sidewalk that leaves the junction. Where the
    /// first edge's far end is one of the second's ends, that is the node; otherwise
    /// it is the first edge's near end. Two edges that share both their nodes — the
    /// two directions of one road — make this a guess, which is why
    /// [`Exporter::walking_nodes`] asks the connections first.
    fn shared_node(&self, from: usize, to: usize) -> String {
        let (from, to) = (&self.edges[from], &self.edges[to]);
        if from.to == to.from || from.to == to.to {
            from.to.clone()
        } else {
            from.from.clone()
        }
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
            config: render_config(
                &prefix,
                self.map.metadata.handedness,
                !self.signals.is_empty(),
            ),
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

        // A speed-limit rule is the `speed` of each lane it names — merged into a lane
        // that is otherwise the IR lane's, as the right-of-way rule is merged into an
        // edge's priority. Only the rules whose limit was the one written are linked:
        // a higher rule over the same lane left nothing in the file.
        for edge in &self.edges {
            for (index, lane) in edge.lanes.iter().enumerate() {
                let Some((_, rules)) = self.rule_speeds.get(&lane.lane) else {
                    continue;
                };
                for rule in rules {
                    trace.link_as(
                        IrRef::Rule(*rule),
                        format!("lane:{}_{index}", edge.id),
                        Relation::Merged,
                        "speed",
                    );
                }
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

        // A walk from one footway to another is the walking area netconvert builds
        // at the node, which the export cannot name — so it is traced to the node,
        // as part of what that node became.
        for (node, walked) in &self.walking {
            let local = format!("node:{node}");
            for connection in &walked.connections {
                trace.link_as(
                    connection.clone(),
                    local.clone(),
                    Relation::Collapsed,
                    "walkingarea",
                );
            }
            for connector in &walked.connectors {
                trace.link_as(
                    connector.clone(),
                    local.clone(),
                    Relation::Collapsed,
                    "walkingarea",
                );
            }
        }
        trace
    }

    fn render_nodes(&self) -> String {
        let mut document = xml::Document::new("nodes", "http://sumo.dlr.de/xsd/nodes_file.xsd");
        // First, as the schema requires: netconvert takes the nodes that follow as
        // positions in the frame it describes, and writes it into the network.
        document.leaf("location", &self.location);
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
        for (key, movement) in &self.movements {
            let mut attributes = self.connection_attributes(key);
            if let Some(path) = &movement.shape {
                // What becomes the internal lane: the IR's own path across the
                // junction rather than one netconvert would invent.
                attributes.push(("shape", shape(path)));
            }
            document.leaf("connection", &attributes);
        }
        // An edge none of whose lanes the IR carries on from, leading anywhere but a
        // dead end. With nothing listed for it netconvert would take its movements
        // as unspecified and guess some — a lane dropped where the type changes, say
        // driving into cycling, would be carried on into the cycle lane — so the
        // edge's movements onto each edge leaving the node it runs into are listed
        // as deleted. A `<connection>` with no target says the same thing to newer
        // versions, but netconvert 1.18 still guesses past it; a `<delete>` is
        // honoured by both, and deleting movements netconvert would not have built
        // (the turnaround, which `no-turnarounds` already rules out) is silent.
        // netconvert warns that the edge goes nowhere, and that is what the IR says.
        for unconnected in self.unconnected_edges() {
            let edge = &self.edges[unconnected];
            for next in self.edges.iter().filter(|next| next.from == edge.to) {
                document.leaf(
                    "delete",
                    &[("from", edge.id.clone()), ("to", next.id.clone())],
                );
            }
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

    /// The edges that run into a node something could continue from, but from which
    /// the IR states no movement at all.
    ///
    /// At a signalised node this matters twice over: a movement netconvert guessed
    /// there would be in no program — no `tl`, state `m` — so vehicles on it would
    /// drive through the junction ignoring the signal. Deleting them leaves every
    /// movement into the node one the `.tll.xml` controls.
    ///
    /// A footway is not one of them: pedestrians do not follow connections but cross
    /// a node on its walking area, so there is no movement for netconvert to guess.
    fn unconnected_edges(&self) -> Vec<usize> {
        let connected: BTreeSet<usize> = self.movements.keys().map(|key| key.0).collect();
        let footway = |edge: &Edge| {
            edge.lanes
                .iter()
                .all(|lane| lane.permission == Some(Permission::Allow("pedestrian")))
        };
        (0..self.edges.len())
            .filter(|edge| !connected.contains(edge))
            .filter(|&edge| !footway(&self.edges[edge]))
            .filter(|&edge| {
                self.nodes
                    .get(&self.edges[edge].to)
                    .is_some_and(|node| node.kind != NodeKind::DeadEnd)
            })
            .collect()
    }
}

/// The netconvert run that builds the `.net.xml`.
///
/// Normalising the offset is turned off because the map's metres are about its own
/// geographic origin: a network shifted so that its lowest corner is at zero would no
/// longer line up with the OpenDRIVE, the Lanelet2 map or the clip written from the
/// same IR. The geo-reference does not need it either way — netconvert would fold the
/// shift into the `netOffset` of the `<location>` the node file carries — but the
/// coordinates would no longer be the IR's.
///
/// Turnarounds are turned off because the IR has none. Left to itself netconvert adds
/// a U-turn at every dead end — the far end of each arm — so a vehicle in SUMO could
/// turn back where, in the OpenDRIVE and the Lanelet2 map written from the same IR,
/// the lane simply ends; and the internal lane it draws for one would be the only
/// lane in the network the trace could not follow back to the map.
///
/// Walking areas are turned on because they are how SUMO's pedestrians cross a node,
/// and the export relies on them: no connection is written between two footways (see
/// [`Exporter::connect`]). Left to itself netconvert builds walking areas only at a
/// node that also has a pedestrian crossing, which the export never writes, so a
/// sidewalk would end at every junction. A network with no footways gets none, and
/// nothing else about it changes.
///
/// Left-hand traffic is stated when the map drives on the left, because netconvert
/// cannot infer it and assumes the right. Left unsaid, a left-hand map is built as a
/// right-hand one drawn on the wrong side of the road: the turn that crosses oncoming
/// traffic is taken to be the left one and made to give way, the right turn across
/// the oncoming carriageway is given priority it does not have, and lane 0 — which
/// the export has made the left, kerb-side lane, see [`Exporter::carriageway`] — is
/// taken for the right. Nothing is written for a right-hand map, which is
/// netconvert's default, so its configuration is the same as it always was.
///
/// The traffic-light file is named only when there is one, which is when the map has
/// a signalised junction with something to control.
fn render_config(prefix: &str, handedness: TrafficHandedness, traffic_lights: bool) -> String {
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
    if handedness == TrafficHandedness::LeftHand {
        document.leaf("lefthand", &[("value", "true".into())]);
    }
    document.close("processing");
    document.open("pedestrian", &[]);
    document.leaf("walkingareas", &[("value", "true".into())]);
    document.close("pedestrian");
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
fn edge_id(name: &str, road: &Road, section: usize, direction: Direction) -> String {
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

/// The name each road and junction is written under: its own, reduced by
/// [`identifier`], and made unique where that reduction made two of them one.
///
/// Replacing the characters SUMO refuses loses information: `A;B`, `A B` and `A_B`
/// are three roads to the IR and one name after [`identifier`], and the edges and
/// nodes built from it would then share ids — which netconvert refuses, or which
/// would silently merge two nodes into one. An edge id also adds to the name, so
/// road `a` with two cross-sections and a road called `a.0` would both write
/// `a.0.fwd`.
///
/// So the names are settled once, before anything is written, over the whole map:
/// every road and junction whose reduced name — and, for a road, every edge id it
/// will write — is not already someone else's keeps it, in the IR's order. Only the
/// ones left over are changed, by appending `~1`, `~2` and so on until nothing they
/// would write is taken; `~` is a character SUMO accepts in an id and [`identifier`]
/// leaves alone, so it reads as a mark of the export rather than of the map. Every
/// id that does not collide is exactly what [`identifier`] makes of its name, and
/// the same map is always given the same names. [`crate::check`] lists the changes.
///
/// Node ids need no table of their own: a node is named `j_` and a junction's name,
/// or `n_` and a road's name followed by `_start`, `_end` or `_s` and a number, so
/// distinct names give distinct nodes.
#[derive(Default)]
struct Ids {
    roads: HashMap<RoadId, String>,
    junctions: HashMap<JunctionId, String>,
    /// One sentence per name that was changed, for [`crate::check`].
    renamed: Vec<String>,
}

impl Ids {
    fn new(map: &ValidatedMap) -> Self {
        let mut ids = Ids::default();

        let roads = map
            .roads
            .iter()
            .filter(|road| !road.is_connector())
            .map(|road| {
                let senses = edge_suffixes(map, road);
                (road.id.clone(), road.id.local_name().to_owned(), senses)
            });
        let roads: Vec<_> = roads.collect();
        let written = settle(
            roads
                .iter()
                .map(|(id, name, senses)| (id.as_str(), name.as_str(), senses.as_slice())),
            &mut ids.renamed,
        );
        ids.roads = roads.into_iter().map(|(id, ..)| id).zip(written).collect();

        let junctions: Vec<_> = map
            .junctions
            .iter()
            .map(|junction| (junction.id.clone(), junction.id.local_name().to_owned()))
            .collect();
        let written = settle(
            junctions
                .iter()
                .map(|(id, name)| (id.as_str(), name.as_str(), &[][..])),
            &mut ids.renamed,
        );
        ids.junctions = junctions
            .into_iter()
            .map(|(id, _)| id)
            .zip(written)
            .collect();
        ids
    }

    /// The name a road's edges and nodes are built from.
    fn road(&self, road: &RoadId) -> String {
        self.roads
            .get(road)
            .cloned()
            .unwrap_or_else(|| identifier(road.local_name()))
    }

    /// The name a junction's node is built from.
    fn junction(&self, junction: &JunctionId) -> String {
        self.junctions
            .get(junction)
            .cloned()
            .unwrap_or_else(|| identifier(junction.local_name()))
    }
}

/// What a road's edge ids add to its name, one per edge it will write: `.fwd`,
/// `.bwd`, or `.<section>.fwd` and so on when it has several cross-sections. Only
/// the directions that carry a lane SUMO has a place for are an edge.
fn edge_suffixes(map: &ValidatedMap, road: &Road) -> Vec<String> {
    let mut suffixes = Vec::new();
    for section in 0..road.sections.len() {
        let lanes = map.lanes_of_section(&road.id, section);
        for direction in [Direction::Forward, Direction::Backward] {
            let carries = lanes.iter().any(|lane| {
                lane.direction == direction && classes::permission(lane.lane_type).is_some()
            });
            if carries {
                // The name is left empty: this is the part of the id after it.
                suffixes.push(edge_id("", road, section, direction));
            }
        }
    }
    suffixes
}

/// Gives each of a list of `(IR id, name, edge suffixes)` the name it is written
/// under, in the same order: see [`Ids`].
fn settle<'a>(
    elements: impl Iterator<Item = (&'a str, &'a str, &'a [String])>,
    renamed: &mut Vec<String>,
) -> Vec<String> {
    // Everything one element will write, each tagged with what kind of id it is so
    // a name never clashes with an edge id it merely spells the same as.
    let claims = |name: &str, suffixes: &[String]| -> Vec<(String, String)> {
        std::iter::once((format!("name\0{name}"), name.to_owned()))
            .chain(suffixes.iter().map(|suffix| {
                let edge = format!("{name}{suffix}");
                (format!("edge\0{edge}"), edge)
            }))
            .collect()
    };
    let mut owners: HashMap<String, &str> = HashMap::new();
    let mut written: Vec<Option<String>> = Vec::new();
    // The ones whose plain name was taken, and by whom, settled once every plain
    // name is known.
    struct Clash<'a> {
        index: usize,
        id: &'a str,
        plain: String,
        suffixes: &'a [String],
        owner: &'a str,
        what: String,
    }
    let mut left: Vec<Clash> = Vec::new();
    for (index, (id, name, suffixes)) in elements.enumerate() {
        let plain = identifier(name);
        let wanted = claims(&plain, suffixes);
        if let Some((owner, what)) = wanted
            .iter()
            .find_map(|(key, what)| owners.get(key).map(|owner| (*owner, what.clone())))
        {
            written.push(None);
            left.push(Clash {
                index,
                id,
                plain,
                suffixes,
                owner,
                what,
            });
            continue;
        }
        for (key, _) in wanted {
            owners.insert(key, id);
        }
        written.push(Some(plain));
    }
    for Clash {
        index,
        id,
        plain,
        suffixes,
        owner,
        what,
    } in left
    {
        let (name, wanted) = (1..)
            .map(|n| {
                let name = format!("{plain}~{n}");
                let wanted = claims(&name, suffixes);
                (name, wanted)
            })
            .find(|(_, wanted)| wanted.iter().all(|(key, _)| !owners.contains_key(key)))
            .expect("some suffix is free");
        for (key, _) in wanted {
            owners.insert(key, id);
        }
        renamed.push(format!(
            "a SUMO id is unique, but {owner} and {id} would both be written as \
             `{what}`, so {id} is written as `{name}` instead of `{plain}`"
        ));
        written[index] = Some(name);
    }
    written
        .into_iter()
        .map(|name| name.expect("every element is named"))
        .collect()
}

/// A name reduced to something a SUMO identifier can hold.
///
/// SUMO splits lists of ids on whitespace and reserves a leading colon for the
/// internal edges it generates itself, so neither can survive in a name the caller
/// chose.
///
/// Beyond those, SUMO refuses an id outright if it holds any of
/// [`FORBIDDEN_IN_IDS`]: the set its own validity check
/// (`SUMOXMLDefinitions::isValidNetID`) rejects, and which netconvert enforces on
/// every node and edge it reads, stopping with "Invalid edge id". They are the
/// characters that would be ambiguous in SUMO's own lists and route strings (`;`,
/// `,`, `|`) or awkward to carry through XML and the shells its tools are driven
/// from (the quotes, `&`, the angle brackets, the backslash). A road called "Smith's
/// Lane" or "A; B" is an ordinary name, so each of them becomes an underscore here
/// rather than failing the build. Everything else — `#`, `.`, `/`, letters in any
/// script — SUMO accepts, and it is kept, so an id stays as close to the caller's
/// name as it can.
///
/// Every id the export writes — edges, nodes, and the prefix the files are named
/// after — is built from names that pass through here, so none of them can hold a
/// character netconvert would refuse.
fn identifier(name: &str) -> String {
    name.chars()
        .map(|character| {
            if character.is_whitespace()
                || character == ':'
                || FORBIDDEN_IN_IDS.contains(&character)
            {
                '_'
            } else {
                character
            }
        })
        .collect()
}

/// The characters SUMO will not accept anywhere in a network id, besides whitespace.
///
/// Mirrors the set in `SUMOXMLDefinitions::isValidNetID`, and checked against
/// netconvert 1.26, which rejects a node or an edge id holding any one of them.
const FORBIDDEN_IN_IDS: [char; 9] = [';', ',', '|', '\'', '"', '&', '<', '>', '\\'];

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
    fn an_identifier_loses_what_netconvert_refuses_and_keeps_the_rest() {
        assert_eq!(identifier("A; B"), "A__B");
        assert_eq!(identifier("Smith's Lane"), "Smith_s_Lane");
        assert_eq!(identifier("x|y,z"), "x_y_z");
        assert_eq!(identifier(r#"a"b&c<d>e\f"#), "a_b_c_d_e_f");
        // What SUMO is happy with stays as the caller wrote it.
        assert_eq!(identifier("route#7/a.b-c"), "route#7/a.b-c");
        assert_eq!(identifier("銀座通り"), "銀座通り");
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

    /// A two-way road whose forward lane becomes a cycle lane halfway along, while the
    /// backward lane stays a driving lane throughout.
    fn type_change() -> ValidatedMap {
        let width = PositiveWidth::new(3.5).unwrap();
        let mut builder = MapBuilder::new(metadata("type-change"));
        builder
            .add_road(
                RoadSpec::line(
                    Point3::new(0.0, 0.0, 0.0),
                    Point3::new(200.0, 0.0, 0.0),
                    two_way(),
                )
                .unwrap()
                .with_name("r")
                .with_cross_section(
                    100.0,
                    vec![
                        LaneSpec::new(width, Direction::Forward).with_type(LaneType::Biking),
                        LaneSpec::new(width, Direction::Backward),
                    ],
                ),
            )
            .unwrap();
        builder.finish().unwrap().validate().unwrap()
    }

    /// A map whose names collide once SUMO's refused characters are replaced: a chain
    /// of roads called `A;B`, `A_B`, `A B` and `A_B~1`, two junctions called `J;1` and
    /// `J 1` with three arms each, and a road `c` with two cross-sections next to a
    /// road `c.0`, whose edge ids would meet at `c.0.fwd`.
    fn colliding() -> ValidatedMap {
        let mut builder = MapBuilder::new(metadata("colliding"));
        let mut previous: Option<RoadId> = None;
        for (index, name) in ["A;B", "A_B", "A B", "A_B~1"].into_iter().enumerate() {
            let x = index as f64 * 100.0;
            let road = builder
                .add_road(
                    RoadSpec::line(
                        Point3::new(x, 0.0, 0.0),
                        Point3::new(x + 100.0, 0.0, 0.0),
                        two_way(),
                    )
                    .unwrap()
                    .with_name(name),
                )
                .unwrap();
            if let Some(previous) = &previous {
                builder.connect(previous, &road).unwrap();
            }
            previous = Some(road);
        }

        for (index, junction) in ["J;1", "J 1"].into_iter().enumerate() {
            let centre = Point3::new(index as f64 * 300.0, 300.0, 0.0);
            let arms: Vec<RoadId> = [(0.0, 1.0), (1.0, 0.0), (0.0, -1.0)]
                .into_iter()
                .enumerate()
                .map(|(arm, (dx, dy))| {
                    let at =
                        |reach: f64| Point3::new(centre.x + dx * reach, centre.y + dy * reach, 0.0);
                    builder
                        .add_road(
                            RoadSpec::line(at(100.0), at(14.0), two_way())
                                .unwrap()
                                .with_name(format!("t{index}{arm}")),
                        )
                        .unwrap()
                })
                .collect();
            let junction = builder.add_junction(Some(junction));
            for (index, from) in arms.iter().enumerate() {
                for to in arms.iter().skip(index + 1) {
                    builder
                        .connect_ends(from, RoadEnd::End, to, RoadEnd::End, Some(&junction))
                        .unwrap();
                }
            }
        }

        let width = PositiveWidth::new(3.5).unwrap();
        let mut wider = two_way();
        wider.push(LaneSpec::new(width, Direction::Forward));
        builder
            .add_road(
                RoadSpec::line(
                    Point3::new(0.0, -300.0, 0.0),
                    Point3::new(100.0, -300.0, 0.0),
                    two_way(),
                )
                .unwrap()
                .with_cross_section(50.0, wider)
                .with_name("c"),
            )
            .unwrap();
        builder
            .add_road(
                RoadSpec::line(
                    Point3::new(0.0, -400.0, 0.0),
                    Point3::new(100.0, -400.0, 0.0),
                    two_way(),
                )
                .unwrap()
                .with_name("c.0"),
            )
            .unwrap();
        builder.finish().unwrap().validate().unwrap()
    }

    fn traced(network: &PlainNetwork, element: IrRef) -> Vec<String> {
        network
            .trace
            .links_of(&element)
            .map(|link| link.local.clone())
            .collect()
    }

    #[test]
    fn names_that_reduce_to_one_are_told_apart() {
        let map = colliding();
        let network = to_plain_xml(&map).unwrap();
        let edges = |road: &str| traced(&network, IrRef::Road(RoadId::new(road)));

        // The first to claim a name keeps it, a road whose own name already ends in
        // `~1` keeps that, and the rest are numbered past every name in use.
        assert_eq!(edges("A;B"), ["edge:A_B.fwd", "edge:A_B.bwd"]);
        assert_eq!(edges("A_B~1"), ["edge:A_B~1.fwd", "edge:A_B~1.bwd"]);
        assert_eq!(edges("A_B"), ["edge:A_B~2.fwd", "edge:A_B~2.bwd"]);
        assert_eq!(edges("A B"), ["edge:A_B~3.fwd", "edge:A_B~3.bwd"]);
        // An edge id is a name and more, and a collision there counts too.
        assert_eq!(
            edges("c"),
            [
                "edge:c.0.fwd",
                "edge:c.0.bwd",
                "edge:c.1.fwd",
                "edge:c.1.bwd"
            ]
        );
        assert_eq!(edges("c.0"), ["edge:c.0~1.fwd", "edge:c.0~1.bwd"]);
        // A name that collides with nothing is exactly what it was.
        assert_eq!(edges("t00"), ["edge:t00.fwd", "edge:t00.bwd"]);

        let node = |junction: &str| traced(&network, IrRef::Junction(JunctionId::new(junction)));
        assert_eq!(node("J;1"), ["node:j_J_1"]);
        assert_eq!(node("J 1"), ["node:j_J_1~1"]);

        // Every edge and every node written is its own.
        let written: Vec<&str> = network
            .trace
            .links
            .iter()
            .filter(|link| matches!(link.ir, IrRef::Road(_)))
            .map(|link| link.local.as_str())
            .collect();
        let distinct: BTreeSet<&str> = written.iter().copied().collect();
        assert_eq!(distinct.len(), written.len(), "{written:?}");
        let nodes: Vec<&str> = network
            .nodes
            .split("<node id=\"")
            .skip(1)
            .map(|rest| rest.split('"').next().unwrap())
            .collect();
        let distinct: BTreeSet<&str> = nodes.iter().copied().collect();
        assert_eq!(distinct.len(), nodes.len(), "{nodes:?}");
        // The chain's ends and joints are five nodes, none merged into another.
        for id in [
            "n_A_B_start",
            "n_A_B_end",
            "n_A_B~2_end",
            "n_A_B~1_start",
            "n_A_B~1_end",
        ] {
            assert!(distinct.contains(id), "{id} missing from {distinct:?}");
        }

        // The same map is always given the same names.
        for _ in 0..3 {
            assert_eq!(to_plain_xml(&map).unwrap(), network);
        }
    }

    #[test]
    fn check_names_every_id_it_changed() {
        let problems = check(&colliding());
        let renamed: Vec<&String> = problems
            .iter()
            .filter(|problem| problem.starts_with("a SUMO id is unique"))
            .collect();
        assert_eq!(renamed.len(), 4, "{renamed:#?}");
        assert!(renamed.iter().any(|problem| problem.contains("road/A;B")
            && problem.contains("road/A_B would")
            && problem.contains("`A_B~2`")));
        assert!(renamed
            .iter()
            .any(|problem| problem.contains("`c.0.fwd`") && problem.contains("`c.0~1`")));
        assert!(renamed
            .iter()
            .any(|problem| problem.contains("junction/J 1") && problem.contains("`J_1~1`")));
        assert!(check(&in_line())
            .iter()
            .all(|problem| !problem.starts_with("a SUMO id is unique")));
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
                    b"delete" => {
                        found.insert(format!(
                            "delete:{}>{}",
                            attributes["from"], attributes["to"]
                        ));
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
        for map in [
            crossroads(),
            in_line(),
            colliding(),
            ruled_street(),
            type_change(),
        ] {
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
        for map in [crossroads(), in_line(), colliding(), type_change()] {
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
    fn one_connector_per_movement_loses_no_way_across() {
        // The builder draws one connector per movement, so on a crossroads every pair
        // of lanes has one way across and every connection's shape is the whole of
        // what the IR drew: nothing to report.
        let map = crossroads();
        assert!(parallel_connectors(&map).is_empty());
        assert!(!check(&map).iter().any(|line| line.contains("one shape")));
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

    /// Where no lane of an edge carries on into the next cross-section, every movement
    /// netconvert could guess from the edge is deleted; left out, netconvert would
    /// guess a movement the IR does not have.
    #[test]
    fn an_edge_the_map_carries_nothing_on_from_connects_to_nothing() {
        let map = type_change();
        let network = to_plain_xml(&map).unwrap();
        let written = written(&network);

        // The driving lane ends where the cycle lane begins: nothing joins them, and
        // nothing turns it back either.
        assert!(written.contains("delete:r.0.fwd>r.1.fwd"), "{written:?}");
        assert!(
            !written
                .iter()
                .any(|element| element.starts_with("connection:r.0.fwd_")),
            "{written:?}"
        );
        // The other carriageway carries on, so it is written as the movement it is.
        assert!(
            written.contains("connection:r.1.bwd_0>r.0.bwd_0"),
            "{written:?}"
        );
        // Only the edge that goes nowhere has anything deleted: the far end of either
        // carriageway is a dead end, where nothing continues for netconvert to guess.
        assert!(
            written
                .iter()
                .filter(|element| element.starts_with("delete:"))
                .all(|element| element.starts_with("delete:r.0.fwd>")),
            "{written:?}"
        );
    }

    /// A footway the IR carries nothing on from is left alone: pedestrians cross a
    /// node on its walking area, not along connections, so there is nothing for
    /// netconvert to guess and nothing to delete.
    #[test]
    fn a_footway_the_map_carries_nothing_on_from_has_nothing_deleted() {
        let width = PositiveWidth::new(3.5).unwrap();
        let mut builder = MapBuilder::new(metadata("footway-ends"));
        builder
            .add_road(
                RoadSpec::line(
                    Point3::new(0.0, 0.0, 0.0),
                    Point3::new(200.0, 0.0, 0.0),
                    vec![LaneSpec::new(width, Direction::Forward).with_type(LaneType::Sidewalk)],
                )
                .unwrap()
                .with_name("r")
                .with_cross_section(100.0, vec![LaneSpec::new(width, Direction::Forward)]),
            )
            .unwrap();
        let map = builder.finish().unwrap().validate().unwrap();
        let network = to_plain_xml(&map).unwrap();
        let written = written(&network);

        assert!(written.contains("edge:r.0.fwd"), "{written:?}");
        assert!(
            !written.iter().any(|element| element.starts_with("delete:")),
            "{written:?}"
        );
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

    /// A road whose two sidewalks — one on each of its two edges, which share both
    /// their nodes — are joined by a pavement turning back round each end, through a
    /// junction there of its own: `start` and `end`. Both walks are read from the
    /// forward sidewalk to the backward one, so the exporter follows them as one
    /// movement.
    ///
    /// The builder lays a footway the way traffic would run, which round the start is
    /// from the backward sidewalk to the forward one; that walk is reversed by hand
    /// afterwards. A corner pavement is read in whatever direction its two sidewalks
    /// happen to run, and a footway connection may leave by either end.
    fn turn_back() -> ValidatedMap {
        let width = |metres| PositiveWidth::new(metres).unwrap();
        let street = vec![
            LaneSpec::new(width(3.5), Direction::Backward),
            LaneSpec::new(width(2.0), Direction::Backward).with_type(LaneType::Sidewalk),
            LaneSpec::new(width(3.5), Direction::Forward),
            LaneSpec::new(width(2.0), Direction::Forward).with_type(LaneType::Sidewalk),
        ];
        let mut builder = MapBuilder::new(metadata("turn-back"));
        let road = builder
            .add_road(
                RoadSpec::line(
                    Point3::new(0.0, 0.0, 0.0),
                    Point3::new(100.0, 0.0, 0.0),
                    street,
                )
                .unwrap()
                .with_name("street"),
            )
            .unwrap();
        let (backward, forward) = (LaneRef::new(road.clone(), 1), LaneRef::new(road, 3));
        // The backward sidewalk leaves by the road's start and the forward one by its
        // end, so these are pavements round the start and round the end.
        let start = builder.add_junction(Some("start"));
        builder
            .connect_lanes(&backward, &forward, Some(&start))
            .unwrap();
        let end = builder.add_junction(Some("end"));
        builder
            .connect_lanes(&forward, &backward, Some(&end))
            .unwrap();
        let mut map = builder.finish().unwrap().validate().unwrap().into_map();
        for connection in map.connections.iter_mut() {
            if connection.junction.as_ref() == Some(&start) {
                std::mem::swap(&mut connection.from, &mut connection.to);
            }
        }
        UnvalidatedMap::from_map(map).validate().unwrap()
    }

    #[test]
    fn a_walk_between_the_two_edges_of_a_road_is_traced_to_the_end_it_happens_at() {
        let map = turn_back();
        let street = map
            .roads
            .iter()
            .find(|road| road.name.as_deref() == Some("street"))
            .unwrap();
        let forward = map
            .lanes
            .iter()
            .find(|lane| {
                lane.road == street.id
                    && lane.lane_type == LaneType::Sidewalk
                    && lane.direction == Direction::Forward
            })
            .unwrap();
        // Both walks leave the forward sidewalk: one by its start, the other by its
        // end. The forward edge's far end is a node the backward edge starts at, so
        // the edges alone would put both at the road's end.
        let leaving: Vec<LaneEnd> = map
            .connections
            .iter()
            .filter(|connection| connection.from.lane == forward.id)
            .map(|connection| connection.from.end)
            .collect();
        assert!(leaving.contains(&LaneEnd::Start) && leaving.contains(&LaneEnd::End));

        let network = to_plain_xml(&map).unwrap();
        let trace = &network.trace;
        let written = written(&network);
        assert_eq!(map.connections.len(), 4);
        for connection in map.connections.iter() {
            let junction = connection.junction.as_ref().unwrap();
            let node = format!("node:j_{}", junction.local_name());
            assert!(written.contains(&node), "{node}");
            let links: Vec<_> = trace
                .links_of(&IrRef::Connection(connection.id.clone()))
                .collect();
            assert_eq!(links.len(), 1, "{}", connection.id);
            assert_eq!(links[0].local, node, "{}", connection.id);
            assert_eq!(links[0].role.as_deref(), Some("walkingarea"));
        }
        for pavement in map.roads.iter().filter(|road| road.is_connector()) {
            let junction = pavement.junction.as_ref().unwrap();
            let node = format!("node:j_{}", junction.local_name());
            for lane in map.lanes.iter().filter(|lane| lane.road == pavement.id) {
                let links: Vec<_> = trace.links_of(&IrRef::Lane(lane.id.clone())).collect();
                assert_eq!(links.len(), 1, "{}", lane.id);
                assert_eq!(links[0].local, node, "{}", lane.id);
            }
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

    // ----------------------------------------------------------------------- //
    // Speed-limit rules
    // ----------------------------------------------------------------------- //

    /// One road with three rules over its lanes: one that overrides a lane's own
    /// limit and also names a lane SUMO has no place for, and two that disagree about
    /// the same lane.
    fn ruled_street() -> ValidatedMap {
        let width = PositiveWidth::new(3.5).unwrap();
        let mut builder = MapBuilder::new(metadata("ruled"));
        let road = builder
            .add_road(
                RoadSpec::line(
                    Point3::ORIGIN,
                    Point3::new(120.0, 0.0, 0.0),
                    vec![
                        LaneSpec::new(width, Direction::Forward)
                            .with_speed_limit(SpeedLimit::from_kph(60.0).unwrap()),
                        LaneSpec::new(width, Direction::Forward),
                        LaneSpec::new(width, Direction::Backward),
                        LaneSpec::new(width, Direction::Forward).with_type(LaneType::Border),
                    ],
                )
                .unwrap()
                .with_name("main"),
            )
            .unwrap();
        let lane = |index| LaneRef::new(road.clone(), index);
        let kph = |value| SpeedLimit::from_kph(value).unwrap();
        builder.add_speed_limit_rule(kph(30.0), vec![lane(0), lane(3)]);
        builder.add_speed_limit_rule(kph(40.0), vec![lane(1)]);
        builder.add_speed_limit_rule(kph(50.0), vec![lane(1)]);
        builder.finish().unwrap().validate().unwrap()
    }

    /// The IR lane at `index` in the road's cross-section.
    fn lane_at(map: &ValidatedMap, index: usize) -> &Lane {
        map.lanes.iter().find(|lane| lane.index == index).unwrap()
    }

    /// Each written lane's `speed` attribute, by `<edge>_<index>`, read back from the
    /// `.edg.xml` rather than from the exporter's state.
    fn lane_speeds(network: &PlainNetwork) -> HashMap<String, Option<String>> {
        let mut speeds = HashMap::new();
        let mut edge = String::new();
        let mut reader = quick_xml::Reader::from_str(&network.edges);
        loop {
            let event = reader.read_event().unwrap();
            let element = match &event {
                Event::Eof => break,
                Event::Start(element) | Event::Empty(element) => element,
                _ => continue,
            };
            let attribute = |name: &str| {
                element
                    .attributes()
                    .map(Result::unwrap)
                    .find(|attribute| attribute.key.as_ref() == name.as_bytes())
                    .map(|attribute| String::from_utf8_lossy(&attribute.value).into_owned())
            };
            match element.name().as_ref() {
                b"edge" => edge = attribute("id").unwrap(),
                b"lane" => {
                    speeds.insert(
                        format!("{edge}_{}", attribute("index").unwrap()),
                        attribute("speed"),
                    );
                }
                _ => {}
            }
        }
        speeds
    }

    #[test]
    fn a_speed_limit_rule_is_the_speed_of_the_lanes_it_names() {
        let map = ruled_street();
        let network = to_plain_xml(&map).unwrap();
        let speeds = lane_speeds(&network);
        let written = |index| &network.lanes[&lane_at(&map, index).id];

        // The rule replaces the lane's own 60 km/h, as it does in Lanelet2.
        assert_eq!(speeds[written(0)].as_deref(), Some("8.333"));
        // Of two rules over one lane, the lower is the one written.
        assert_eq!(speeds[written(1)].as_deref(), Some("11.111"));
        // A lane no rule names, with no limit of its own, keeps the edge's speed.
        assert_eq!(speeds[written(2)], None);
        // And the border lane the first rule also names is not written at all.
        assert!(!network.lanes.contains_key(&lane_at(&map, 3).id));
    }

    #[test]
    fn a_speed_limit_rule_is_merged_into_the_lanes_it_set() {
        let map = ruled_street();
        let network = to_plain_xml(&map).unwrap();
        let rule_links = |rule: usize| -> Vec<&TraceLink> {
            network.trace.links_of(&IrRef::Rule(rule)).collect()
        };

        for (rule, index) in [(0, 0), (1, 1)] {
            let links = rule_links(rule);
            assert_eq!(links.len(), 1, "rule {rule}");
            assert_eq!(
                links[0].local,
                format!("lane:{}", network.lanes[&lane_at(&map, index).id])
            );
            assert_eq!(links[0].relation, Relation::Merged);
            assert_eq!(links[0].role.as_deref(), Some("speed"));
        }
        // The 50 km/h rule lost to the 40 km/h one and left nothing in the file.
        assert!(rule_links(2).is_empty());
    }

    #[test]
    fn what_a_speed_limit_rule_cannot_reach_is_reported() {
        let map = ruled_street();
        let report = check(&map).join("\n");
        assert!(
            report.contains(&format!(
                "no lane for is dropped with them: {}",
                lane_at(&map, 3).id
            )),
            "{report}"
        );
        assert!(
            report.contains(&format!("the others are dropped: {}", lane_at(&map, 1).id)),
            "{report}"
        );
        // A map with no such rules says nothing about them.
        assert!(!check(&in_line()).join("\n").contains("speed-limit"));
    }

    // ----------------------------------------------------------------------- //
    // Handedness
    // ----------------------------------------------------------------------- //

    /// One road, two lanes each way, under the given handedness.
    fn dual(handedness: TrafficHandedness) -> ValidatedMap {
        let mut builder = MapBuilder::new(MapMetadata {
            handedness,
            ..metadata("dual")
        });
        let width = PositiveWidth::new(3.5).unwrap();
        builder
            .add_road(
                RoadSpec::line(
                    Point3::new(0.0, 0.0, 0.0),
                    Point3::new(200.0, 0.0, 0.0),
                    vec![
                        LaneSpec::new(width, Direction::Forward),
                        LaneSpec::new(width, Direction::Forward),
                        LaneSpec::new(width, Direction::Backward),
                        LaneSpec::new(width, Direction::Backward),
                    ],
                )
                .unwrap()
                .with_name("dual"),
            )
            .unwrap();
        builder.finish().unwrap().validate().unwrap()
    }

    #[test]
    fn a_left_hand_map_tells_netconvert_so() {
        let config = to_plain_xml(&dual(TrafficHandedness::LeftHand))
            .unwrap()
            .config;
        // In the processing section, beside the other options that shape the build.
        let processing =
            &config[config.find("<processing>").unwrap()..config.find("</processing>").unwrap()];
        assert!(
            processing.contains(r#"<lefthand value="true"/>"#),
            "{config}"
        );
    }

    #[test]
    fn a_right_hand_map_leaves_netconvert_at_its_default() {
        let config = to_plain_xml(&dual(TrafficHandedness::RightHand))
            .unwrap()
            .config;
        assert!(!config.contains("lefthand"), "{config}");
    }

    /// Lane 0 is the outer lane either way, so the two handednesses number each
    /// carriageway from opposite sides — and the edge's own line is the middle of the
    /// carriageway whichever end of the list its kerb is at.
    #[test]
    fn lane_zero_is_the_outer_lane_under_either_handedness() {
        let y = |network: &PlainNetwork, map: &ValidatedMap, id: &str| {
            let (lane, _) = network
                .lanes
                .iter()
                .find(|(_, written)| written.as_str() == id)
                .unwrap();
            map.lane(lane).unwrap().center_offset()
        };
        for (handedness, outward) in [
            (TrafficHandedness::RightHand, -1.0),
            (TrafficHandedness::LeftHand, 1.0),
        ] {
            let map = dual(handedness);
            let network = to_plain_xml(&map).unwrap();
            // Forward traffic runs along +x, on the side its handedness puts it.
            let (outer, inner) = (
                y(&network, &map, "dual.fwd_0"),
                y(&network, &map, "dual.fwd_1"),
            );
            assert!(outer * outward > inner * outward, "{handedness:?}");
            assert!(outer.abs() > inner.abs(), "{handedness:?}");
            let (outer, inner) = (
                y(&network, &map, "dual.bwd_0"),
                y(&network, &map, "dual.bwd_1"),
            );
            assert!(outer.abs() > inner.abs(), "{handedness:?}");
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
