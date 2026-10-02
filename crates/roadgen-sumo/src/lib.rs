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
//! | `<name>.safe.{src,dst,via}.xml` | where `randomTrips.py` may start, end and route trips |
//!
//! ```text
//! netconvert -c <name>.netccfg
//! randomTrips.py -n <name>.net.xml --weights-prefix <name>.safe --validate
//! ```
//!
//! That is the format a network *generator* is expected to produce, and it is the
//! one SUMO documents for the purpose.
//!
//! The last three are not part of the network but of the demand put on it: edge
//! weights that keep `randomTrips.py` from starting a car trip where no car can leave,
//! or ending one where no car can arrive. A generated network has many such places,
//! since its unlinked road ends are dead ends with no U-turn; see [`weights`] for what
//! is weighted and why.
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
//! # Lane changes
//!
//! SUMO has no paint, but it has the thing paint between two lanes is *for*: whether
//! a vehicle may change across it. Each lane says so with `changeLeft` and
//! `changeRight`, and the exporter reads both off the IR's marking on the boundary
//! the lane shares with its neighbour in the edge. A solid line, a double solid line
//! or a kerb leaves the change to `emergency` vehicles; a broken line or no paint at
//! all says nothing, which SUMO reads as open to everyone. A line that is solid on
//! one side and broken on the other binds each vehicle by the half nearer it. See
//! [`lane_change`].
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
//! A crosswalk becomes a pedestrian `<crossing>` in the connections file. SUMO puts a
//! crossing at a node, across the mouth of the edges it names, between the walking
//! areas either side of them; so a crosswalk near the end of a road — where the IR
//! puts one at a junction — is written at that end's node, across the road's edges
//! there that have vehicle lanes on them (an edge that is all footway, like the far
//! pavement of a one-way street, is not crossed). A crosswalk further along the
//! road, across one without a footway at both kerbs or without traffic, or across a
//! one-way road at a dead end, has nothing to be in SUMO and is reported by
//! [`check`] instead.
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
//! | `edge:<id>` | a right-of-way rule naming its lanes, role `priority` | merged |
//! | `lane:<edge>_<index>` | a lane | exact |
//! | `lane:<edge>_<index>` | a speed-limit rule naming the lane, role `speed` | merged |
//! | `connection:<from edge>_<from lane>><to edge>_<to lane>` | a lane connection | exact, or merged |
//! | the same connection | a connector lane it runs over | collapsed |
//! | `node:<id>`, role `walkingarea` | a connection between two footways, and a pavement lane round a corner | collapsed |
//! | `crossing:<node>/<edge>+<edge>` | a crosswalk | exact, or merged |
//!
//! A connection is named by its two lanes as the built network names them, so the
//! `.net.xml` `<connection from fromLane to toLane via>` that netconvert writes for it
//! is found from the trace by the same `<edge>_<index>` pair — and with it the
//! internal lane (`via`) netconvert generated in place of the connector. A movement
//! through a junction is two IR connections and one connector lane, all of them
//! written as the one connection, so each is `merged` or `collapsed` into it.
//!
//! A crossing has no id in the plain format, so it is named by its node and the
//! edges it crosses, sorted, which is also how the built network can be matched to
//! it: netconvert names the crossing `:<node>_c<n>` and lists the same edges as its
//! `crossingEdges`.
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
pub mod weights;
mod xml;

use std::collections::{BTreeMap, BTreeSet, HashMap, HashSet};
use std::path::Path;

use roadgen_core::geometry::{Curve3, Point3, Polyline3, SamplingConfig};
use roadgen_core::map::{Lane, Projection, Road, TrafficHandedness};
use roadgen_core::semantics::{LaneType, MapObject, MapObjectKind, ObjectGeometry, TrafficRule};
use roadgen_core::topology::{
    Direction, LaneConnection, LaneEnd, LaneEndpoint, LateralSide, RoadEnd, RoadLinkTarget,
};
use roadgen_core::trace::{IrRef, Relation, Trace};
use roadgen_core::{ConnectionId, JunctionId, LaneId, ObjectId, RoadId, ValidatedMap};

pub use classes::Permission;
pub use error::ExportError;
pub use location::{geo_reference, height_error, GeoReference};
pub use weights::TripWeights;

/// The name a map with none of its own is written under.
const DEFAULT_NAME: &str = "network";

/// A SUMO plain-XML network: the three input files, the netconvert run that turns
/// them into a `.net.xml`, and the `randomTrips.py` edge weights for the network it
/// builds.
///
/// The file names are derived from [`PlainNetwork::prefix`], and the configuration
/// refers to them, so the seven belong together in one directory.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PlainNetwork {
    /// What the files are named before their `.nod.xml`, `.edg.xml`, `.con.xml` and
    /// `.netccfg` suffixes.
    pub prefix: String,
    pub nodes: String,
    pub edges: String,
    pub connections: String,
    pub config: String,
    /// The `randomTrips.py` weights, written as `<prefix>.safe.src.xml`,
    /// `<prefix>.safe.dst.xml` and `<prefix>.safe.via.xml` and read with
    /// `--weights-prefix <prefix>.safe`. They name the edges as the built network
    /// does, so they are used with the `.net.xml` netconvert makes of the rest.
    pub weights: TripWeights,
    /// Where each lane of the IR ended up, as the `<edge>_<index>` identifier the
    /// built network gives it.
    ///
    /// A SUMO lane's identity is positional, so this is the only way back from a
    /// lane of the map to the lane of the network — and the numbering is not the
    /// IR's: SUMO counts from the outside of the carriageway, which is the right of
    /// the direction of travel under right-hand traffic and the left under left-hand.
    pub lanes: BTreeMap<LaneId, String>,
    /// Where each element of the IR ended up in the network files: nodes, edges, lanes
    /// and connections, by the identifiers written for them. See the crate
    /// documentation for how each is named.
    ///
    /// Its `files` are empty: the network is in memory until [`write_traced`] puts it
    /// somewhere.
    pub trace: Trace,
}

impl PlainNetwork {
    /// Each file's name and its contents: the network in the order netconvert reads
    /// it, then the trip weights in the order `src`, `dst`, `via`.
    pub fn files(&self) -> Vec<(String, &str)> {
        vec![
            (format!("{}.nod.xml", self.prefix), self.nodes.as_str()),
            (format!("{}.edg.xml", self.prefix), self.edges.as_str()),
            (
                format!("{}.con.xml", self.prefix),
                self.connections.as_str(),
            ),
            (format!("{}.netccfg", self.prefix), self.config.as_str()),
            (
                format!("{}.{}", self.prefix, TripWeights::SUFFIXES[0]),
                self.weights.src.as_str(),
            ),
            (
                format!("{}.{}", self.prefix, TripWeights::SUFFIXES[1]),
                self.weights.dst.as_str(),
            ),
            (
                format!("{}.{}", self.prefix, TripWeights::SUFFIXES[2]),
                self.weights.via.as_str(),
            ),
        ]
    }

    /// Writes the seven files into `directory`, creating it if it is not there.
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
/// was written alongside the prefix, with the seven files' paths in it.
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
         what colour, is not written — only whether it may be crossed, which becomes \
         each lane's `changeLeft` and `changeRight`"
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

    if map
        .objects
        .iter()
        .any(|object| matches!(object.kind, MapObjectKind::TrafficSign { .. }))
    {
        problems.push(
            "a SUMO sign is an additional file, not part of the network, so the map's \
             traffic signs are not written"
                .to_owned(),
        );
    }
    problems.extend(crosswalk_problems(map));

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
// Crosswalks
// --------------------------------------------------------------------------- //

/// What [`check`] says about the map's crosswalks: how far the ones written as
/// crossings were moved to reach their node, and which were not written and why.
fn crosswalk_problems(map: &ValidatedMap) -> Vec<String> {
    let mut moved: Vec<f64> = Vec::new();
    let mut mid_block: Vec<String> = Vec::new();
    let mut no_footway: Vec<&str> = Vec::new();
    let mut nothing_to_cross: Vec<&str> = Vec::new();
    let mut dead_end: Vec<&str> = Vec::new();
    let mut unplaced: Vec<&str> = Vec::new();
    for object in map
        .objects
        .iter()
        .filter(|object| object.kind == MapObjectKind::Crosswalk)
    {
        match place_crosswalk(map, object) {
            Placement::AtEnd { offset, .. } => moved.push(offset),
            Placement::MidBlock { offset } => {
                mid_block.push(format!("{} ({offset:.1} m)", object.id.as_str()))
            }
            Placement::NoFootway => no_footway.push(object.id.as_str()),
            Placement::NothingToCross => nothing_to_cross.push(object.id.as_str()),
            Placement::DeadEnd => dead_end.push(object.id.as_str()),
            Placement::Unplaced => unplaced.push(object.id.as_str()),
        }
    }

    let mut problems = Vec::new();
    if !moved.is_empty() {
        let furthest = moved.iter().copied().fold(0.0_f64, f64::max);
        problems.push(format!(
            "a SUMO crossing belongs to a node and lies across the mouth of the edges it \
             crosses, so the {} crosswalks near the end of their road are written as \
             crossings at the node there, moved up to {furthest:.1} m from where the IR \
             draws them; pedestrians have priority on each, or the signal decides at a \
             signalised node",
            moved.len()
        ));
    }
    if !mid_block.is_empty() {
        problems.push(format!(
            "a SUMO crossing belongs to a node, so the {} crosswalks more than \
             {CROSSING_REACH} m from either end of their road are not written — splitting \
             the road with a node of its own there would be needed: {}",
            mid_block.len(),
            mid_block.join(", ")
        ));
    }
    if !no_footway.is_empty() {
        problems.push(format!(
            "a SUMO crossing joins the footways either side of the road, so the {} \
             crosswalks across a road without a footway at both kerbs are not written: \
             {}",
            no_footway.len(),
            no_footway.join(", ")
        ));
    }
    if !nothing_to_cross.is_empty() {
        problems.push(format!(
            "a SUMO crossing lies across vehicle lanes, so the {} crosswalks across a \
             road with nothing but footway at the end they are near are not written: {}",
            nothing_to_cross.len(),
            nothing_to_cross.join(", ")
        ));
    }
    if !dead_end.is_empty() {
        problems.push(format!(
            "a SUMO crossing joins two walking areas, and at a dead end one walking \
             area wraps round the end of the road, so the {} crosswalks at a dead end \
             of a road with traffic one way only are not written: {}",
            dead_end.len(),
            dead_end.join(", ")
        ));
    }
    if !unplaced.is_empty() {
        problems.push(format!(
            "a SUMO crossing lies across the edges at a node, and the junction itself \
             has none, so the {} crosswalks inside a junction are not written: {}",
            unplaced.len(),
            unplaced.join(", ")
        ));
    }
    problems
}

/// Whether a lane written with `permission` is one only pedestrians may use.
fn is_footway(permission: Option<Permission>) -> bool {
    permission == classes::permission(LaneType::Sidewalk)
}

/// How far from the end of its road a crosswalk may lie and still be written as a
/// crossing at the node there, measured from the road end to the crosswalk's nearer
/// edge.
///
/// A SUMO crossing has no position of its own: it belongs to a node, and netconvert
/// lays it across the mouth of the edges it names, right where they meet the
/// junction. The IR draws a crosswalk wherever it was put, and at a junction that is
/// usually a little way back from the mouth — far enough for a turning vehicle to
/// wait clear of the junction before it, a car length or two. Fifteen metres takes
/// in that setback and no more: a crosswalk further along is a mid-block crossing,
/// and writing it at the node would move the place pedestrians cross by half a
/// block. [`check`] says how far each written crossing was moved.
const CROSSING_REACH: f64 = 15.0;

/// Where a crosswalk of the IR can go in a SUMO network.
///
/// Decided from the IR alone, so that [`check`] says exactly what the export does.
#[derive(Debug, Clone, PartialEq)]
enum Placement {
    /// A crossing at the node at `end` of `road`, across that end's edges. `offset`
    /// is how far the crosswalk's nearer edge lies from the road end — how far the
    /// crossing moves — and `width` is the crosswalk's, along the road.
    AtEnd {
        road: RoadId,
        end: RoadEnd,
        offset: f64,
        width: f64,
    },
    /// Too far from either end of its road to belong to a node; `offset` is the
    /// distance to the nearer one.
    MidBlock { offset: f64 },
    /// At the end of a road with no footway on one side or the other, so the
    /// crossing would have nowhere to land.
    NoFootway,
    /// At the end of a road whose lanes there are all footway, so there is no
    /// vehicle lane for the crossing to cross.
    NothingToCross,
    /// At a dead end, across a single edge with vehicle lanes on it: the one walking
    /// area round the end of the road is at both ends of it.
    DeadEnd,
    /// Across a connector road — inside a junction, where SUMO has no edge to lay a
    /// crossing over — or across no road at all.
    Unplaced,
}

/// Where `object`, a crosswalk, can go: see [`Placement`].
///
/// The road is the one its lanes are on — the builder lists every lane of the road
/// it crosses — and where along that road it lies is read from its outline, the two
/// lines across the road that are its edges. Each is taken at its middle and found
/// on the road's reference line, which gives the station of each edge; the nearer
/// one to a road end is how far the crossing would have to move to reach that end's
/// node, and the distance between the two is the crosswalk's width.
///
/// A crossing joins the footways either side of the road, so the road's cross-section
/// at that end has to be footway at both kerbs: netconvert builds a crossing with no
/// footway at one end of it, but nothing leads off it there, and it says so ("has no
/// target"). Only the crossed road's own footways are looked for, not those of the
/// arms beside it, which is what the generator lays out.
fn place_crosswalk(map: &ValidatedMap, object: &MapObject) -> Placement {
    let Some(road) = object
        .lanes
        .iter()
        .find_map(|lane| map.lane(lane))
        .and_then(|lane| map.road(&lane.road))
    else {
        return Placement::Unplaced;
    };
    if road.is_connector() {
        return Placement::Unplaced;
    }
    let ObjectGeometry::Band { left, right } = &object.geometry else {
        return Placement::Unplaced;
    };
    let middle = |edge: &Curve3| edge.start_point().lerp(edge.end_point(), 0.5);
    let (left, right) = (middle(left), middle(right));
    let (Some(a), Some(b), Ok(length)) = (
        station_on(map, road, left),
        station_on(map, road, right),
        road.horizontal_length(),
    ) else {
        return Placement::Unplaced;
    };
    let (near, far) = (a.min(b), a.max(b));
    let (end, offset) = if near <= length - far {
        (RoadEnd::Start, near.max(0.0))
    } else {
        (RoadEnd::End, (length - far).max(0.0))
    };
    if offset > CROSSING_REACH {
        return Placement::MidBlock { offset };
    }
    let section = match end {
        RoadEnd::Start => 0,
        RoadEnd::End => road.sections.len() - 1,
    };
    let mut kerb_to_kerb: Vec<&Lane> = map
        .lanes_of_section(&road.id, section)
        .into_iter()
        .filter(|lane| classes::permission(lane.lane_type).is_some())
        .collect();
    kerb_to_kerb.sort_by(|a, b| a.center_offset().total_cmp(&b.center_offset()));
    let footway = |lane: &&Lane| is_footway(classes::permission(lane.lane_type));
    let kerbs = kerb_to_kerb.first().zip(kerb_to_kerb.last());
    if !kerbs.is_some_and(|(right, left)| footway(right) && footway(left) && right.id != left.id) {
        return Placement::NoFootway;
    }
    // The crossing lies across the edges with something on them a pedestrian has to
    // cross: one direction's lanes are an edge, and where that is all footway — the
    // pavement on the far kerb of a one-way street, say — the edge is left out of
    // the crossing, since netconvert discards a crossing over no vehicle lane ("no
    // vehicle lanes to cross") rather than build it.
    let crossed: Vec<Direction> = [Direction::Forward, Direction::Backward]
        .into_iter()
        .filter(|direction| {
            kerb_to_kerb
                .iter()
                .any(|lane| lane.direction == *direction && !footway(lane))
        })
        .collect();
    if crossed.is_empty() {
        return Placement::NothingToCross;
    }
    // At a dead end netconvert builds one walking area round the end of the road
    // unless the crossing has two edges to lie between, so a crossing over one edge
    // there starts and ends on the same walking area, and netconvert says so ("starts
    // and ends at walkingarea") and leaves it out of the pedestrian network. That is
    // the case whether the edge holds both footways or the far one is an edge of its
    // own beside it.
    if road.link.at(end).is_none() && crossed.len() == 1 {
        return Placement::DeadEnd;
    }
    Placement::AtEnd {
        road: road.id.clone(),
        end,
        offset,
        width: left.horizontal_distance_to(right),
    }
}

/// The station on `road`'s reference line nearest to `point`, read off the
/// polyline through the road's vertices.
fn station_on(map: &ValidatedMap, road: &Road, point: Point3) -> Option<f64> {
    let samples = map.vertex_samples(&road.id).ok()?;
    let mut best: Option<(f64, f64)> = None;
    for pair in samples.windows(2) {
        let (a, b) = (pair[0].point, pair[1].point);
        let (dx, dy) = (b.x - a.x, b.y - a.y);
        let span = dx * dx + dy * dy;
        let t = if span > 0.0 {
            (((point.x - a.x) * dx + (point.y - a.y) * dy) / span).clamp(0.0, 1.0)
        } else {
            0.0
        };
        let distance = a.lerp(b, t).horizontal_distance_to(point);
        let station = pair[0].station + t * (pair[1].station - pair[0].station);
        if best.is_none_or(|(nearest, _)| distance < nearest) {
            best = Some((distance, station));
        }
    }
    best.map(|(_, station)| station)
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
    /// Which cross-section of the road, and which way along it: what tells two edges
    /// of one road that netconvert will join across a section node from the two
    /// carriageways of it.
    section: usize,
    direction: Direction,
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
    /// Whether the boundary on each side of the direction of travel is one only
    /// emergency vehicles may change across. Never set on the outer side of the
    /// outermost lanes: there is no lane there to change into.
    restricted_left: bool,
    restricted_right: bool,
    shape: Polyline3,
}

/// Where a lane of the IR ended up: which edge, and which index within it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Slot {
    edge: usize,
    index: usize,
}

/// One written pedestrian crossing, and the crosswalks of the IR it stands for.
#[derive(Default)]
struct Crossing {
    /// Its width along the road: the widest of its crosswalks.
    width: f64,
    crosswalks: BTreeSet<ObjectId>,
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
    /// The pedestrian crossings, by the node each is at and the edges it crosses
    /// there, in the order they are written; see [`Exporter::build_crossings`].
    crossings: BTreeMap<(String, Vec<String>), Crossing>,
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
            crossings: BTreeMap::new(),
            slots: HashMap::new(),
            signalised: BTreeMap::new(),
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
        self.build_crossings();
        self.open_dead_ends();
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

    /// Lifts the paint's restriction from every lane that leads nowhere while the
    /// rest of its edge carries on: a lane that drops, say, past the end of a taper.
    ///
    /// The only way out of such a lane is sideways, so the topology of the IR says a
    /// vehicle in it must change lanes whatever is painted beside it — and the
    /// builder's default marking is a solid line, so a lane drop drawn without
    /// thought for its paint has one. netconvert agrees: it refuses a prohibition
    /// that would trap a vehicle in a dead-end lane, discards it and complains. So
    /// it is not written, and the change out of the lane is left open to everyone.
    /// An edge none of whose lanes lead anywhere is the end of the road rather than
    /// a lane drop, and its paint is written as it is.
    fn open_dead_ends(&mut self) {
        let leaving: HashSet<(usize, usize)> = self
            .movements
            .keys()
            .map(|&(edge, lane, _, _)| (edge, lane))
            .collect();
        for (index, edge) in self.edges.iter_mut().enumerate() {
            let continues = |position: usize| leaving.contains(&(index, position));
            if !(0..edge.lanes.len()).any(continues) {
                continue;
            }
            for (position, lane) in edge.lanes.iter_mut().enumerate() {
                if !continues(position) {
                    lane.restricted_left = false;
                    lane.restricted_right = false;
                }
            }
        }
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
            // SUMO counts from the kerb, so the next lane up is the one further from
            // it: to the driver's left where traffic keeps right, and to the right
            // where it keeps left. `changeLeft` and `changeRight` name the driver's
            // sides in travel under either handedness, not a direction in index.
            let above = lanes.get(position + 1);
            let below = position.checked_sub(1).and_then(|below| lanes.get(below));
            let (to_left, to_right) = match self.map.metadata.handedness {
                TrafficHandedness::RightHand => (above, below),
                TrafficHandedness::LeftHand => (below, above),
            };
            written.push(EdgeLane {
                lane: lane.id.clone(),
                width: self.mean_width(lane)?,
                speed: self.lane_speed(lane),
                permission: classes::permission(lane.lane_type),
                restricted_left: to_left
                    .is_some_and(|other| !lane_change(lane, other, LateralSide::Left)),
                restricted_right: to_right
                    .is_some_and(|other| !lane_change(lane, other, LateralSide::Right)),
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
            section,
            direction,
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
        is_footway(self.edges[slot.edge].lanes[slot.index].permission)
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
    // Crossings
    // ----------------------------------------------------------------------- //

    /// Every crosswalk that can be a SUMO crossing, as one: at the node at the end of
    /// the road it crosses, across the edges of that end.
    ///
    /// A SUMO crossing is not a strip of its own with a place along the road. It is a
    /// `<crossing node edges>` in the connections file, and netconvert lays it across
    /// the named edges where they meet the node, joining the walking areas either
    /// side of them. So a crosswalk near the end of a road — which is where the IR
    /// puts one at a junction, a little short of the mouth — becomes the crossing
    /// at that end's node, over both carriageways of the road if it has two; one
    /// further along the road has no node to belong to and is left out (see
    /// [`place_crosswalk`] for what decides it, and [`check`] for what is reported).
    ///
    /// The edges are written in a fixed order, sorted by name, because netconvert
    /// reads them as a set — it writes them back in an order of its own — and two
    /// crosswalks across the same end of the same road would be the same crossing.
    /// They are merged into one, at the wider of the two widths, rather than written
    /// twice, which netconvert would refuse.
    fn build_crossings(&mut self) {
        for object in self.map.objects.iter() {
            if object.kind != MapObjectKind::Crosswalk {
                continue;
            }
            let Placement::AtEnd {
                road, end, width, ..
            } = place_crosswalk(self.map, object)
            else {
                continue;
            };
            let Some((node, edges)) = self.edges_at(&road, end) else {
                continue;
            };
            let crossing = self.crossings.entry((node, edges)).or_default();
            crossing.width = crossing.width.max(width);
            crossing.crosswalks.insert(object.id.clone());
        }
    }

    /// The node at one end of a road, and the road's edges that meet it there.
    ///
    /// Found from the written edges rather than worked out again: the first
    /// cross-section's edges at the start of the road, the last one's at its end, of
    /// which a forward edge leaves the start and arrives at the end, and a backward
    /// edge the other way round.
    fn edges_at(&self, road: &RoadId, end: RoadEnd) -> Option<(String, Vec<String>)> {
        let road = self.map.road(road)?;
        let section = match end {
            RoadEnd::Start => 0,
            RoadEnd::End => road.sections.len() - 1,
        };
        let mut node = None;
        let mut edges = Vec::new();
        for direction in [Direction::Forward, Direction::Backward] {
            let id = edge_id(&self.ids.road(&road.id), road, section, direction);
            let Some(edge) = self.edges.iter().find(|edge| edge.id == id) else {
                continue;
            };
            if edge.lanes.iter().all(|lane| is_footway(lane.permission)) {
                // Nothing on it to cross; see [`place_crosswalk`].
                continue;
            }
            let at = match (end, direction) {
                (RoadEnd::Start, Direction::Forward) | (RoadEnd::End, Direction::Backward) => {
                    &edge.from
                }
                _ => &edge.to,
            };
            node.get_or_insert_with(|| at.clone());
            edges.push(id);
        }
        edges.sort();
        Some((node?, edges)).filter(|(_, edges)| !edges.is_empty())
    }

    // ----------------------------------------------------------------------- //
    // Rendering
    // ----------------------------------------------------------------------- //

    fn render(&self, prefix: String) -> PlainNetwork {
        PlainNetwork {
            nodes: self.render_nodes(),
            edges: self.render_edges(),
            connections: self.render_connections(),
            config: render_config(&prefix, self.map.metadata.handedness),
            weights: self.render_weights(),
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

        // A crosswalk is the crossing written for it, and one of several where two
        // crosswalks cross the same end of the same road.
        for ((node, edges), crossing) in &self.crossings {
            let local = format!("crossing:{}", crossing_name(node, edges));
            let relation = if crossing.crosswalks.len() == 1 {
                Relation::Exact
            } else {
                Relation::Merged
            };
            for crosswalk in &crossing.crosswalks {
                trace.link(crosswalk.clone(), local.clone(), relation);
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
                // Left out where the paint allows it: SUMO's default is that anyone
                // may change.
                if lane.restricted_left {
                    attributes.push(("changeLeft", classes::CHANGE_ACROSS_SOLID.to_owned()));
                }
                if lane.restricted_right {
                    attributes.push(("changeRight", classes::CHANGE_ACROSS_SOLID.to_owned()));
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
        for ((from_edge, from_lane, to_edge, to_lane), movement) in &self.movements {
            let mut attributes = vec![
                ("from", self.edges[*from_edge].id.clone()),
                ("to", self.edges[*to_edge].id.clone()),
                ("fromLane", from_lane.to_string()),
                ("toLane", to_lane.to_string()),
            ];
            if let Some(path) = &movement.shape {
                // What becomes the internal lane: the IR's own path across the
                // junction rather than one netconvert would invent.
                attributes.push(("shape", shape(path)));
            }
            document.leaf("connection", &attributes);
        }
        // `priority` is written as a number because that is what SUMO's schema for
        // the file says it is, though netconvert reads it as a yes or no: with
        // `SUMO_HOME` set, netconvert validates the file and refuses `true`.
        //
        // It is always yes. A crosswalk in the IR is a painted one — it has an
        // outline, the stripes the other exports draw — and at a marked crossing
        // traffic gives way to pedestrians on it. At a signalised node the signal
        // decides instead, and netconvert gives the crossing a phase of its own.
        for ((node, edges), crossing) in &self.crossings {
            document.leaf(
                "crossing",
                &[
                    ("node", node.clone()),
                    ("edges", edges.join(" ")),
                    ("priority", "1".to_owned()),
                    ("width", metres(crossing.width)),
                ],
            );
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

    /// The edges that run into a node something could continue from, but from which
    /// the IR states no movement at all.
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

/// What the trace calls a crossing: the node it is at and the edges it crosses
/// there, as `<node>/<edge>+<edge>`.
///
/// A `<crossing>` has no id in the plain format. netconvert names the one it builds
/// `:<node>_c<n>`, numbered in an order of its own, and records the edges it crosses
/// beside it (`crossingEdges`), so the node and the edges are what both ends can
/// agree on.
fn crossing_name(node: &str, edges: &[String]) -> String {
    format!("{node}/{}", edges.join("+"))
}

/// Whether the paint lets a vehicle in `lane` change into `neighbour`, the lane
/// beside it in the same edge on the `towards` side of its direction of travel.
///
/// The IR keeps a lane's markings by the side of the *reference line* they are on,
/// and which side that is depends on which way the lane runs: a backward lane's
/// left in travel is the reference line's right. Only the direction matters here,
/// not the map's handedness: [`Exporter::add_edge`] has already picked the
/// neighbour on the driver's `towards` side by where the lanes physically are —
/// the next index up under right-hand traffic, the next one down under left-hand,
/// since SUMO counts from the kerb either way and its `changeLeft` and
/// `changeRight` name the driver's sides in travel.
///
/// The marking read is `lane`'s own on that side. Two lanes sharing a boundary
/// normally agree about it — the OpenDRIVE reader gives each the same one, and a
/// caller describing a road should too — and where they do not, each vehicle obeys
/// the paint its own lane describes. A vehicle sits on the opposite side of the
/// boundary from where the boundary sits relative to its lane, and that is the side
/// [`classes::may_cross`] wants: it picks the nearer half of a line like
/// `solid broken`.
///
/// Two lanes that are neighbours in SUMO but do not share a boundary in the IR have
/// something between them SUMO has no lane for — a painted island, a border, a
/// lane the other way — and crossing it is no lane change the map offers, so it is
/// restricted like a solid line.
fn lane_change(lane: &Lane, neighbour: &Lane, towards: LateralSide) -> bool {
    let across = match lane.direction {
        Direction::Forward => towards,
        Direction::Backward => towards.opposite(),
    };
    let (marking, own, theirs) = match across {
        LateralSide::Left => (lane.left_marking, lane.left_edge, neighbour.right_edge),
        LateralSide::Right => (lane.right_marking, lane.right_edge, neighbour.left_edge),
    };
    own == theirs && classes::may_cross(marking.marking, across.opposite())
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
/// node that also has a pedestrian crossing, which the export writes only where the
/// IR has a crosswalk, so a sidewalk would end at every other junction. A network
/// with no footways gets none, and nothing else about it changes.
///
/// Left-hand traffic is stated when the map drives on the left, because netconvert
/// cannot infer it and assumes the right. Left unsaid, a left-hand map is built as a
/// right-hand one drawn on the wrong side of the road: the turn that crosses oncoming
/// traffic is taken to be the left one and made to give way, the right turn across
/// the oncoming carriageway is given priority it does not have, and lane 0 — which
/// the export has made the left, kerb-side lane, see [`Exporter::carriageway`] — is
/// taken for the right. Nothing is written for a right-hand map, which is
/// netconvert's default, so its configuration is the same as it always was.
fn render_config(prefix: &str, handedness: TrafficHandedness) -> String {
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
    if handedness == TrafficHandedness::LeftHand {
        document.leaf("lefthand", &[("value", "true".into())]);
    }
    document.close("processing");
    document.open("pedestrian", &[]);
    document.leaf("walkingareas", &[("value", "true".into())]);
    document.close("pedestrian");
    document.finish()
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

    /// Two streets with crosswalks on them: `paved`, with a footway at both kerbs and
    /// three crosswalks — two near its start, one in the middle — and `bare`, with
    /// no footway and one crosswalk near its end.
    fn crossing_street() -> ValidatedMap {
        let mut builder = MapBuilder::new(metadata("crossings"));
        let width = PositiveWidth::new(3.5).unwrap();
        let footway = PositiveWidth::new(2.0).unwrap();
        let paved = builder
            .add_road(
                RoadSpec::line(
                    Point3::new(0.0, 0.0, 0.0),
                    Point3::new(100.0, 0.0, 0.0),
                    vec![
                        LaneSpec::new(width, Direction::Backward),
                        LaneSpec::new(footway, Direction::Backward).with_type(LaneType::Sidewalk),
                        LaneSpec::new(width, Direction::Forward),
                        LaneSpec::new(footway, Direction::Forward).with_type(LaneType::Sidewalk),
                    ],
                )
                .unwrap()
                .with_name("paved"),
            )
            .unwrap();
        let bare = builder
            .add_road(
                RoadSpec::line(
                    Point3::new(0.0, 50.0, 0.0),
                    Point3::new(100.0, 50.0, 0.0),
                    two_way(),
                )
                .unwrap()
                .with_name("bare"),
            )
            .unwrap();
        builder.add_crosswalk(&paved, 0.05, 4.0).unwrap();
        builder.add_crosswalk(&paved, 0.06, 3.0).unwrap();
        builder.add_crosswalk(&paved, 0.5, 4.0).unwrap();
        builder.add_crosswalk(&bare, 0.95, 4.0).unwrap();
        builder.finish().unwrap().validate().unwrap()
    }

    #[test]
    fn a_crosswalk_near_the_end_of_its_road_is_a_crossing_at_the_node_there() {
        let map = crossing_street();
        let network = to_plain_xml(&map).unwrap();
        // The two near the start are one crossing, as wide as the wider; the one in
        // the middle and the one with no footway to land on are not written.
        let crossings: Vec<&str> = network
            .connections
            .lines()
            .filter(|line| line.contains("<crossing"))
            .collect();
        assert_eq!(
            crossings,
            [
                r#"    <crossing node="n_paved_start" edges="paved.bwd paved.fwd" priority="1" width="4.000"/>"#
            ],
            "{}",
            network.connections
        );

        let local = "crossing:n_paved_start/paved.bwd+paved.fwd";
        let near: Vec<&ObjectId> = map
            .objects
            .iter()
            .filter(|object| {
                ["object/crosswalk/paved/0.05", "object/crosswalk/paved/0.06"]
                    .contains(&object.id.as_str())
            })
            .map(|object| &object.id)
            .collect();
        assert_eq!(near.len(), 2);
        for id in near {
            let links: Vec<&TraceLink> =
                network.trace.links_of(&IrRef::Object(id.clone())).collect();
            assert_eq!(links.len(), 1, "{id}");
            assert_eq!(links[0].local, local);
            assert_eq!(links[0].relation, Relation::Merged);
        }
        for id in ["object/crosswalk/paved/0.5", "object/crosswalk/bare/0.95"] {
            let id = &map
                .objects
                .iter()
                .find(|object| object.id.as_str() == id)
                .unwrap()
                .id;
            assert!(
                network
                    .trace
                    .links_of(&IrRef::Object(id.clone()))
                    .next()
                    .is_none(),
                "{id} was not written, so it should not be traced"
            );
        }
    }

    #[test]
    fn what_becomes_of_each_crosswalk_is_reported() {
        let report = check(&crossing_street()).join("\n");
        assert!(
            report.contains("the 2 crosswalks near the end of their road are written"),
            "{report}"
        );
        assert!(
            report.contains("object/crosswalk/paved/0.5 (48.0 m)"),
            "{report}"
        );
        assert!(
            report.contains(
                "without a footway at both kerbs are not written: object/crosswalk/bare/0.95"
            ),
            "{report}"
        );
        // Nothing about crosswalks for a map with none.
        assert!(!check(&in_line()).join("\n").contains("crosswalk"));
    }

    /// One-way streets with a footway at each kerb, the far one running against the
    /// traffic: `through` joined end to end to `onward`, with a crosswalk at the joint;
    /// `dead_end`, ending in nothing, with a crosswalk at that end; and `footpath`,
    /// two footways and nothing else, with a crosswalk across it.
    fn one_way_streets() -> ValidatedMap {
        let mut builder = MapBuilder::new(metadata("one_way_crossings"));
        let width = PositiveWidth::new(3.5).unwrap();
        let footway = PositiveWidth::new(2.0).unwrap();
        let sidewalk = |direction| LaneSpec::new(footway, direction).with_type(LaneType::Sidewalk);
        let one_way = || {
            vec![
                sidewalk(Direction::Backward),
                LaneSpec::new(width, Direction::Forward),
                sidewalk(Direction::Forward),
            ]
        };
        let mut road = |name: &str, y: f64, x: (f64, f64), lanes: Vec<LaneSpec>| {
            builder
                .add_road(
                    RoadSpec::line(Point3::new(x.0, y, 0.0), Point3::new(x.1, y, 0.0), lanes)
                        .unwrap()
                        .with_name(name),
                )
                .unwrap()
        };
        let through = road("through", 0.0, (0.0, 100.0), one_way());
        let onward = road("onward", 0.0, (100.0, 200.0), one_way());
        let dead_end = road("dead_end", 50.0, (0.0, 100.0), one_way());
        let footpath = road(
            "footpath",
            100.0,
            (0.0, 100.0),
            vec![sidewalk(Direction::Backward), sidewalk(Direction::Forward)],
        );
        builder.connect(&through, &onward).unwrap();
        builder.add_crosswalk(&through, 0.95, 4.0).unwrap();
        builder.add_crosswalk(&dead_end, 0.05, 4.0).unwrap();
        builder.add_crosswalk(&footpath, 0.05, 4.0).unwrap();
        builder.finish().unwrap().validate().unwrap()
    }

    /// A crossing lies across the edges with vehicle lanes on them, never across an
    /// edge that is all footway — netconvert discards a crossing with "no vehicle lanes
    /// to cross" — and none is written where that leaves nothing to cross, or one edge
    /// at a dead end, where netconvert would find the crossing "starts and ends at" the
    /// one walking area there. The report says which crosswalks those are.
    #[test]
    fn a_crossing_crosses_only_the_edges_with_traffic_on_them() {
        let map = one_way_streets();
        let network = to_plain_xml(&map).unwrap();
        let crossings: Vec<&str> = network
            .connections
            .lines()
            .filter(|line| line.contains("<crossing"))
            .collect();
        assert_eq!(
            crossings,
            [
                r#"    <crossing node="n_onward_start" edges="through.fwd" priority="1" width="4.000"/>"#
            ],
            "{}",
            network.connections
        );
        let traced: Vec<&TraceLink> = map
            .objects
            .iter()
            .flat_map(|object| network.trace.links_of(&IrRef::Object(object.id.clone())))
            .collect();
        assert_eq!(traced.len(), 1, "{traced:?}");
        assert_eq!(traced[0].local, "crossing:n_onward_start/through.fwd");

        let report = check(&map).join("\n");
        assert!(
            report.contains("the 1 crosswalks near the end of their road are written"),
            "{report}"
        );
        assert!(
            report.contains(
                "nothing but footway at the end they are near are not written: \
                 object/crosswalk/footpath/0.05"
            ),
            "{report}"
        );
        assert!(
            report.contains(
                "at a dead end of a road with traffic one way only are not written: \
                 object/crosswalk/dead_end/0.05"
            ),
            "{report}"
        );
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
                    }
                    b"crossing" => {
                        let mut edges: Vec<String> = attributes["edges"]
                            .split_whitespace()
                            .map(str::to_owned)
                            .collect();
                        edges.sort();
                        found.insert(format!(
                            "crossing:{}",
                            crossing_name(&attributes["node"], &edges)
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
        for map in [
            crossroads(),
            in_line(),
            colliding(),
            ruled_street(),
            type_change(),
            crossing_street(),
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
    // Lane changes
    // ----------------------------------------------------------------------- //

    /// A straight road whose lanes, listed from the reference line outwards, all run
    /// `direction` and all carry `separator` on both sides — so the line between
    /// every pair of them is `separator`, whichever lane it is read off.
    fn separated(direction: Direction, separator: RoadMarking) -> ValidatedMap {
        separated_in(TrafficHandedness::RightHand, direction, separator)
    }

    fn separated_in(
        handedness: TrafficHandedness,
        direction: Direction,
        separator: RoadMarking,
    ) -> ValidatedMap {
        let width = PositiveWidth::new(3.5).unwrap();
        let marking = BoundaryMarking::new(separator, MarkingColor::White);
        let lanes = (0..2)
            .map(|_| LaneSpec::new(width, direction).with_markings(marking, marking))
            .collect();
        let mut builder = MapBuilder::new(MapMetadata {
            handedness,
            ..metadata("separated")
        });
        builder
            .add_road(
                RoadSpec::line(Point3::ORIGIN, Point3::new(100.0, 0.0, 0.0), lanes)
                    .unwrap()
                    .with_name("road"),
            )
            .unwrap();
        builder.finish().unwrap().validate().unwrap()
    }

    /// Each written lane's `changeLeft` and `changeRight`, by `<edge>_<index>`, read
    /// back from the `.edg.xml`.
    fn lane_changes(network: &PlainNetwork) -> BTreeMap<String, (Option<String>, Option<String>)> {
        let mut found = BTreeMap::new();
        let mut edge = String::new();
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
                        String::from_utf8_lossy(&attribute.value).into_owned(),
                    )
                })
                .collect();
            match element.name().as_ref() {
                b"edge" => edge = attributes["id"].clone(),
                b"lane" => {
                    found.insert(
                        format!("{edge}_{}", attributes["index"]),
                        (
                            attributes.get("changeLeft").cloned(),
                            attributes.get("changeRight").cloned(),
                        ),
                    );
                }
                _ => {}
            }
        }
        found
    }

    fn emergency() -> Option<String> {
        Some("emergency".to_owned())
    }

    #[test]
    fn a_solid_line_between_two_lanes_keeps_all_but_emergency_vehicles_from_crossing() {
        let network = to_plain_xml(&separated(Direction::Forward, RoadMarking::Solid)).unwrap();
        let changes = lane_changes(&network);
        // Lane 0 is the right-hand lane, so its neighbour is on its left and lane 1's
        // is on its right. The outer sides have no neighbour and say nothing.
        assert_eq!(changes["road.fwd_0"], (emergency(), None));
        assert_eq!(changes["road.fwd_1"], (None, emergency()));
    }

    #[test]
    fn a_broken_line_or_none_lets_anyone_change() {
        for separator in [RoadMarking::Broken, RoadMarking::None] {
            let network = to_plain_xml(&separated(Direction::Forward, separator)).unwrap();
            for (lane, change) in lane_changes(&network) {
                assert_eq!(change, (None, None), "{lane} across {separator:?}");
            }
        }
    }

    #[test]
    fn every_line_no_one_may_cross_restricts_the_change() {
        for separator in [
            RoadMarking::Solid,
            RoadMarking::SolidSolid,
            RoadMarking::Curbstone,
        ] {
            let network = to_plain_xml(&separated(Direction::Forward, separator)).unwrap();
            let changes = lane_changes(&network);
            assert_eq!(changes["road.fwd_0"].0, emergency(), "{separator:?}");
            assert_eq!(changes["road.fwd_1"].1, emergency(), "{separator:?}");
        }
    }

    #[test]
    fn half_of_a_solid_broken_line_binds_the_vehicle_on_its_side() {
        // Forward lanes on the right of the reference line, looking along it: lane 1
        // is the inner one, on the left of the line between them, and so faces the
        // solid half of `solid broken`; lane 0 faces the broken half.
        let network =
            to_plain_xml(&separated(Direction::Forward, RoadMarking::SolidBroken)).unwrap();
        let changes = lane_changes(&network);
        assert_eq!(changes["road.fwd_0"], (None, None));
        assert_eq!(changes["road.fwd_1"], (None, emergency()));

        let network =
            to_plain_xml(&separated(Direction::Forward, RoadMarking::BrokenSolid)).unwrap();
        let changes = lane_changes(&network);
        assert_eq!(changes["road.fwd_0"], (emergency(), None));
        assert_eq!(changes["road.fwd_1"], (None, None));
    }

    #[test]
    fn a_backward_lane_reads_its_markings_the_other_way_round() {
        // Backward lanes sit left of the reference line and run against it. Lane 0
        // is the outer one, which is on the left of the line between the two looking
        // along the reference line — so it faces the solid half of `solid broken`,
        // and that half is on its left in travel.
        let network =
            to_plain_xml(&separated(Direction::Backward, RoadMarking::SolidBroken)).unwrap();
        let changes = lane_changes(&network);
        assert_eq!(changes["road.bwd_0"], (emergency(), None));
        assert_eq!(changes["road.bwd_1"], (None, None));
    }

    #[test]
    fn handedness_moves_the_lanes_but_not_which_half_of_the_line_binds_whom() {
        // Under left-hand traffic the forward lanes sit left of the reference line
        // and SUMO counts them from the kerb, so lane 0 is the outer one — the
        // leftmost in travel — and lane 1 is on its right. Lane 0 is on the left of
        // the line between the two looking along the reference line, so it faces
        // the solid half of `solid broken`, which is on its right in travel; lane 1
        // faces the broken half.
        let map = separated_in(
            TrafficHandedness::LeftHand,
            Direction::Forward,
            RoadMarking::SolidBroken,
        );
        let changes = lane_changes(&to_plain_xml(&map).unwrap());
        assert_eq!(changes["road.fwd_0"], (None, emergency()));
        assert_eq!(changes["road.fwd_1"], (None, None));
    }

    #[test]
    fn under_left_hand_traffic_the_kerb_lane_changes_to_its_right() {
        // `changeLeft` and `changeRight` are the driver's sides in travel, and in a
        // left-hand network the next lane up from the kerb is on the right — so a
        // solid line closes lane 0's change to the right and lane 1's to the left,
        // the mirror of a right-hand road. The outer sides have no neighbour.
        for direction in [Direction::Forward, Direction::Backward] {
            let map = separated_in(TrafficHandedness::LeftHand, direction, RoadMarking::Solid);
            let changes = lane_changes(&to_plain_xml(&map).unwrap());
            let edge = match direction {
                Direction::Forward => "road.fwd",
                Direction::Backward => "road.bwd",
            };
            assert_eq!(changes[&format!("{edge}_0")], (None, emergency()), "{edge}");
            assert_eq!(changes[&format!("{edge}_1")], (emergency(), None), "{edge}");
        }
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
        let names: Vec<String> = [
            "nod.xml",
            "edg.xml",
            "con.xml",
            "netccfg",
            "safe.src.xml",
            "safe.dst.xml",
            "safe.via.xml",
        ]
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
    // The randomTrips weights
    // ----------------------------------------------------------------------- //

    /// Each edge's weight in one of the `edgedata` files, read back from the XML.
    fn weights(xml: &str) -> BTreeMap<String, f64> {
        let mut found = BTreeMap::new();
        let mut reader = quick_xml::Reader::from_str(xml);
        loop {
            let event = reader.read_event().unwrap();
            let element = match &event {
                Event::Eof => break,
                Event::Start(element) | Event::Empty(element) => element,
                _ => continue,
            };
            if element.name().as_ref() != b"edge" {
                continue;
            }
            let attribute = |key: &[u8]| {
                let value = element
                    .attributes()
                    .map(Result::unwrap)
                    .find(|attribute| attribute.key.as_ref() == key)
                    .unwrap()
                    .value;
                String::from_utf8_lossy(&value).into_owned()
            };
            found.insert(attribute(b"id"), attribute(b"value").parse().unwrap());
        }
        found
    }

    /// The edges given a weight in one file.
    fn weighted(xml: &str) -> BTreeSet<String> {
        weights(xml)
            .into_iter()
            .filter(|(_, value)| *value > 0.0)
            .map(|(edge, _)| edge)
            .collect()
    }

    fn set(edges: &[&str]) -> BTreeSet<String> {
        edges.iter().map(|edge| (*edge).to_owned()).collect()
    }

    /// Each arm of the crossroads runs out to a dead end with no U-turn, so the edge
    /// approaching the junction can only be left and the edge leaving it can only be
    /// arrived on: a trip may start on the one and end on the other, and a way point
    /// may be neither.
    #[test]
    fn a_road_into_a_dead_end_is_no_place_to_start_a_trip() {
        let network = to_plain_xml(&crossroads()).unwrap();
        let every = weights(&network.weights.src);
        assert_eq!(every.len(), 8, "every edge is named: {every:?}");
        assert_eq!(
            weighted(&network.weights.src),
            set(&["north.fwd", "east.fwd", "south.fwd", "west.fwd"])
        );
        assert_eq!(
            weighted(&network.weights.dst),
            set(&["north.bwd", "east.bwd", "south.bwd", "west.bwd"])
        );
        assert!(weighted(&network.weights.via).is_empty());

        // A weighted edge carries its length, which is the length of its lane: the
        // arm runs from 70 m out to 14 m short of the centre.
        for (edge, value) in every {
            if value > 0.0 {
                assert!((value - 56.0).abs() < 0.01, "{edge}: {value}");
            }
        }
    }

    /// The two carriageways of a road whose ends link nowhere are two one-way strips
    /// once U-turns are gone, and no trip joins the one to the other. Only one of them
    /// is weighted, so every trip drawn has a route.
    #[test]
    fn only_the_largest_connected_part_of_the_network_is_weighted() {
        let network = to_plain_xml(&in_line()).unwrap();
        let src = weighted(&network.weights.src);
        let dst = weighted(&network.weights.dst);
        assert!(
            (src == set(&["a.fwd"]) && dst == set(&["b.fwd"]))
                || (src == set(&["b.bwd"]) && dst == set(&["a.bwd"])),
            "src {src:?}, dst {dst:?}"
        );
        assert!(weighted(&network.weights.via).is_empty());
        // And the choice is the same every time.
        assert_eq!(to_plain_xml(&in_line()).unwrap().weights, network.weights);
    }

    /// Three one-way roads end to end. The middle one has a way in and a way out, so
    /// it is the one place a way point may be.
    fn chain(builder: &mut MapBuilder, name: &str, y: f64, lanes: Vec<LaneSpec>) {
        let roads: Vec<RoadId> = (0..3)
            .map(|index| {
                let x = 100.0 * f64::from(index);
                builder
                    .add_road(
                        RoadSpec::line(
                            Point3::new(x, y, 0.0),
                            Point3::new(x + 100.0, y, 0.0),
                            lanes.clone(),
                        )
                        .unwrap()
                        .with_name(format!("{name}{index}")),
                    )
                    .unwrap()
            })
            .collect();
        builder.connect(&roads[0], &roads[1]).unwrap();
        builder.connect(&roads[1], &roads[2]).unwrap();
    }

    #[test]
    fn an_edge_with_a_way_in_and_a_way_out_may_be_passed_through() {
        let width = PositiveWidth::new(3.5).unwrap();
        let mut builder = MapBuilder::new(metadata("chain"));
        chain(
            &mut builder,
            "road",
            0.0,
            vec![LaneSpec::new(width, Direction::Forward)],
        );
        let map = builder.finish().unwrap().validate().unwrap();
        let network = to_plain_xml(&map).unwrap();
        assert_eq!(
            weighted(&network.weights.src),
            set(&["road0.fwd", "road1.fwd"])
        );
        assert_eq!(
            weighted(&network.weights.dst),
            set(&["road1.fwd", "road2.fwd"])
        );
        assert_eq!(weighted(&network.weights.via), set(&["road1.fwd"]));
    }

    /// The weights are for cars, so a footway gets none even where it has a way in
    /// and a way out — and a street carrying a footway alongside its driving lane is
    /// still a street.
    #[test]
    fn a_footway_is_no_place_for_a_car_trip() {
        let width = PositiveWidth::new(3.0).unwrap();
        let mut builder = MapBuilder::new(metadata("footways"));
        chain(
            &mut builder,
            "path",
            50.0,
            vec![LaneSpec::new(width, Direction::Forward).with_type(LaneType::Sidewalk)],
        );
        chain(
            &mut builder,
            "street",
            0.0,
            vec![
                LaneSpec::new(width, Direction::Forward),
                LaneSpec::new(width, Direction::Forward).with_type(LaneType::Sidewalk),
            ],
        );
        let map = builder.finish().unwrap().validate().unwrap();
        let network = to_plain_xml(&map).unwrap();

        // The footway is named, with nothing.
        let every = weights(&network.weights.via);
        assert_eq!(every.get("path1.fwd"), Some(&0.0), "{every:?}");
        assert_eq!(weighted(&network.weights.via), set(&["street1.fwd"]));
        assert_eq!(
            weighted(&network.weights.src),
            set(&["street0.fwd", "street1.fwd"])
        );
        assert_eq!(
            weighted(&network.weights.dst),
            set(&["street1.fwd", "street2.fwd"])
        );
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
