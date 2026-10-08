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
//! which is what lets the trace say which slot a light of the map controls. At a
//! node with crossings netconvert extends the program: it appends the crossings'
//! links after the export's and splits each green to end it with a pedestrian
//! clearance, leaving the vehicle links and their indices as they were written.
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
//! | `lane:<edge>_<index>` | a stop line, role `stopOffset` | merged |
//! | `node:<id>`, role `walkingarea` | a connection between two footways, and a pavement lane round a corner | collapsed |
//! | `crossing:<node>/<edge>+<edge>` | a crosswalk | exact, or merged |
//!
//! With [`Options::curve_lateral_acceleration`] an edge may be cut into pieces at its
//! bends (see [`curves`]): the first keeps the edge's id and the rest are
//! `<edge>.p1`, `<edge>.p2`, …, joined by nodes `n_<edge>_p1`, …. Each piece is `part`
//! of the road, each of its lanes `part` of the IR lane, and so is each
//! `connection:<edge>_<i>><edge>.p1_<i>` straight on between them. Connections and
//! stop offsets leaving an edge are on its last piece, connections arriving on it on
//! its first, and [`PlainNetwork::lanes`] names the first.
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
//! A crossing has no id in the plain format, so it is named by its node and the
//! edges it crosses, sorted, which is also how the built network can be matched to
//! it: netconvert names the crossing `:<node>_c<n>` and lists the same edges as its
//! `crossingEdges`.
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
//! line drawn across lanes running both ways stops only the lanes it is nearer the
//! junction ahead of than the junction behind, measured on the IR along the road and
//! the roads joined to it end to end, rather than edge by edge. A
//! stop line SUMO has no place for — on a lane that is not written, short of the end
//! of a lane that does not meet a junction, further back than the lane is long — is
//! named by [`check`] rather than dropped without a word, lane by lane, so a line
//! written on some of its lanes is reported only for the ones it misses.
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
pub mod curves;
pub mod error;
pub mod location;
pub mod signals;
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

/// The shortest stop offset worth writing, metres.
///
/// A stop line this close to the end of its lane is, to any precision a simulation
/// cares about, at the end of it — which is where SUMO stops a vehicle with no
/// offset at all. It is also the margin kept below an edge's length, which
/// netconvert refuses an offset to reach.
const MIN_STOP_OFFSET: f64 = 0.1;

/// A SUMO plain-XML network: the input files, the netconvert run that turns them
/// into a `.net.xml`, and the `randomTrips.py` edge weights for the network it
/// builds.
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
    ///
    /// Where an edge is cut at curves (see [`Options`]) this is the lane of its first
    /// piece, where traffic enters it; the trace's `part` links name the rest.
    pub lanes: BTreeMap<LaneId, String>,
    /// Where each element of the IR ended up in the network files: nodes, edges, lanes,
    /// connections and traffic lights, by the identifiers written for them. See the
    /// crate documentation for how each is named.
    ///
    /// Its `files` are empty: the network is in memory until [`write_traced`] puts it
    /// somewhere.
    pub trace: Trace,
}

impl PlainNetwork {
    /// Each file's name and its contents: the network in the order netconvert reads
    /// it, then the trip weights in the order `src`, `dst`, `via`.
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
        files.extend(
            [&self.weights.src, &self.weights.dst, &self.weights.via]
                .into_iter()
                .zip(TripWeights::SUFFIXES)
                .map(|(weights, suffix)| (format!("{}.{suffix}", self.prefix), weights.as_str())),
        );
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

/// Choices about the network that the map does not make.
///
/// The default writes the map as it is.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Options {
    /// Hold traffic to the speed a curve allows at this lateral acceleration, m/s²:
    /// the edges are cut where their curvature changes and each piece is given the
    /// speed `√(a / κ)` of its sharpest point, where that is below the limit, and
    /// netconvert is asked to do the same for the internal lanes of junctions with
    /// `junctions.limit-turn-speed`. See [`curves`].
    ///
    /// `None` writes every edge whole at its limit, and leaves the internal lanes to
    /// netconvert's default.
    pub curve_lateral_acceleration: Option<f64>,
}

/// Renders `map` as a SUMO plain-XML network.
pub fn to_plain_xml(map: &ValidatedMap) -> Result<PlainNetwork, ExportError> {
    to_plain_xml_with(map, &Options::default())
}

/// Renders `map` as a SUMO plain-XML network, as `options` say.
pub fn to_plain_xml_with(
    map: &ValidatedMap,
    options: &Options,
) -> Result<PlainNetwork, ExportError> {
    if let Some(acceleration) = options.curve_lateral_acceleration {
        if !(acceleration.is_finite() && acceleration > 0.0) {
            return Err(ExportError::InvalidCurveAcceleration(acceleration));
        }
    }
    let mut exporter = Exporter::new(map, *options);
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
    write_traced_with(map, directory, &Options::default())
}

/// Writes `map` into `directory` as [`write_traced`] does, as `options` say.
pub fn write_traced_with(
    map: &ValidatedMap,
    directory: impl AsRef<Path>,
    options: &Options,
) -> Result<(String, Trace), ExportError> {
    let directory = directory.as_ref();
    let network = to_plain_xml_with(map, options)?;
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

    let lights = map
        .objects
        .iter()
        .filter(|object| object.kind.is_traffic_light())
        .count();
    if lights > 0 {
        let mut problem = format!(
            "the IR holds no signal timing, so the junctions of the {lights} traffic \
             lights get a fixed-time program of the export's own: opposing approaches \
             green together for {few} s ({many} s where a junction has more than two \
             groups of them), then {yellow} s yellow and {red} s all-red, with turns \
             across oncoming traffic permissive",
            few = signals::GREEN_FEW_SECONDS,
            many = signals::GREEN_MANY_SECONDS,
            yellow = signals::YELLOW_SECONDS,
            red = signals::ALL_RED_SECONDS,
        );
        if map
            .objects
            .iter()
            .any(|object| object.kind == MapObjectKind::Crosswalk)
        {
            problem.push_str(
                "; at a junction with pedestrian crossings netconvert appends the \
                 crossings' links after the vehicle links and splits each green to add \
                 a pedestrian clearance — the vehicle links and their indices stay the \
                 export's",
            );
        }
        problems.push(problem);
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

    problems.extend(stop_line_problems(map));

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
    let mut exporter = Exporter::new(map, Options::default());
    if exporter.build().is_err() {
        return Vec::new();
    }

    // Only the lines that hold some lane's stop are written; the rest are named
    // below. One at the very end of each lane it holds needs no offset, since that
    // is where SUMO stops a vehicle anyway, and is counted apart: it leaves no
    // `<stopOffset>` and no trace link behind.
    let offset: BTreeSet<&ObjectId> = exporter
        .stop_offsets
        .values()
        .map(|(object, _)| object)
        .collect();
    let mut how = match offset.len() {
        0 => "none of the map's stop lines is written as a stopOffset".to_owned(),
        1 => "1 stop line is written as the stopOffset of the lanes it crosses".to_owned(),
        count => {
            format!("{count} stop lines are written as the stopOffset of the lanes they cross")
        }
    };
    match exporter
        .stopping
        .iter()
        .filter(|object| !offset.contains(object))
        .count()
    {
        0 => {}
        1 => how.push_str(
            "; 1 is at the very end of its lanes, where SUMO stops a vehicle without \
             one, and needs none",
        ),
        count => how.push_str(&format!(
            "; {count} are at the very end of their lanes, where SUMO stops a vehicle \
             without one, and need none"
        )),
    }
    let mut problems = vec![format!(
        "a SUMO network has no stop lines, only a lane's stop offset back from the \
         junction it runs into: {how}, and the paint itself is not"
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
            Unplaced::BeyondLastEdge => {
                "it is further back from the junction than the last edge before the \
                 junction is long, and the edge it is on ends short of the junction, \
                 where a stop offset stops nothing"
            }
            Unplaced::AtLaneStart => {
                "it is at the lane's start: drawn across lanes running both ways, it \
                 is nearer the junction the lane leaves than the first junction the \
                 lane runs into (or no nearer either), and belongs to the lanes running \
                 the other way"
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
    /// What the edge is written as, in travel order: itself whole, or the pieces
    /// [`curves`] cut it into. Filled in once everything else is built, by
    /// [`Exporter::cut_curves`]; the first piece keeps the edge's id.
    pieces: Vec<Piece>,
}

/// One written edge of an [`Edge`]: the whole of it, or a stretch between two cuts.
/// The first piece keeps the edge's id, so connections arriving on the edge name
/// it as they always did.
struct Piece {
    id: String,
    from: String,
    to: String,
    shape: Polyline3,
    /// Each lane's shape over the stretch, in the edge's lane order.
    lanes: Vec<Polyline3>,
    /// Each lane's speed over the stretch where its curvature holds traffic below
    /// the lane's limit, m/s; see [`curves::cap`].
    speeds: Vec<Option<f64>>,
    /// The longest of its lanes, which is what its trip weight is.
    length: f64,
}

impl Edge {
    /// The piece traffic leaves the edge from, which connections, signals and stop
    /// offsets belong to.
    fn exit(&self) -> &Piece {
        self.pieces
            .last()
            .expect("cut_curves gives every edge a piece")
    }
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
    options: Options,
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
    crossings: BTreeMap<(String, Vec<usize>), Crossing>,
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
    /// The stop offset of each written lane that has one, by (edge, lane index): how
    /// far back from the lane's end its stop line is, and which stop line that is.
    stop_offsets: BTreeMap<(usize, usize), (ObjectId, f64)>,
    /// The stop lines that hold some written lane's stop: as its offset, or — at the
    /// very end of the lane, where SUMO stops a vehicle anyway — as no offset at all.
    stopping: BTreeSet<ObjectId>,
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
    /// It is set back further from the junction than the edge that reaches the
    /// junction is long: on an earlier edge of the approach, which ends at a mere
    /// change of cross-section or joint, where a stop offset stops nothing.
    BeyondLastEdge,
    /// A line that also crosses lanes running the other way crosses this one nearer
    /// the junction the lane leaves than the first junction it runs into — measured
    /// along the lane's travel through plain joints, a dead end counting as never
    /// reaching one — or no nearer either: a line across both carriageways, seen from
    /// the one it does not stop.
    AtLaneStart,
    /// It does not cross the centreline of the lane it names.
    NotAcross,
    /// It is further back from the junction than its edge is long, which netconvert
    /// refuses as an offset.
    BeyondLane,
    /// Another stop line, nearer the junction, already holds the lane's offset.
    Superseded,
}

/// Which junction a lane is nearer where a stop line crosses it. See
/// [`Exporter::line_end`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum LineEnd {
    /// The junction the lane runs into.
    Exit,
    /// The junction the lane leaves.
    Entry,
    /// Exactly halfway.
    Middle,
}

impl<'a> Exporter<'a> {
    fn new(map: &'a ValidatedMap, options: Options) -> Self {
        let mut exporter = Exporter {
            map,
            options,
            sampling: map.metadata.sampling,
            ids: Ids::new(map),
            nodes: BTreeMap::new(),
            edges: Vec::new(),
            movements: BTreeMap::new(),
            walking: BTreeMap::new(),
            crossings: BTreeMap::new(),
            slots: HashMap::new(),
            signalised: BTreeMap::new(),
            governed: BTreeMap::new(),
            light_rules: BTreeMap::new(),
            signals: BTreeMap::new(),
            right_of_way: HashSet::new(),
            yielding: HashSet::new(),
            ruled: HashSet::new(),
            stop_offsets: BTreeMap::new(),
            stopping: BTreeSet::new(),
            unplaced: BTreeMap::new(),
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
        let end = lane.direction.exit_end().as_road_end();
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
        self.build_crossings();
        self.open_dead_ends();
        self.place_stop_lines()?;
        self.cut_curves()?;
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
            pieces: Vec::new(),
        });
        Ok(())
    }

    /// Cuts every edge into the pieces [`curves`] finds along it, or writes it whole
    /// when the options ask for no curve speeds; see [`Options`].
    ///
    /// Done last, once the stop offsets are placed: the last piece is kept long
    /// enough to carry the edge's.
    fn cut_curves(&mut self) -> Result<(), ExportError> {
        for index in 0..self.edges.len() {
            let pieces = self.pieces_of(index)?;
            for piece in &pieces[1..] {
                let point = piece.shape.first();
                self.node(piece.from.clone(), point, NodeKind::Unstated, false, None);
            }
            self.edges[index].pieces = pieces;
        }
        Ok(())
    }

    fn pieces_of(&self, index: usize) -> Result<Vec<Piece>, ExportError> {
        let edge = &self.edges[index];
        let longest = |lanes: &[Polyline3]| lanes.iter().map(Polyline3::length).fold(0.0, f64::max);
        let Some(acceleration) = self.options.curve_lateral_acceleration else {
            let lanes: Vec<Polyline3> = edge.lanes.iter().map(|lane| lane.shape.clone()).collect();
            return Ok(vec![Piece {
                id: edge.id.clone(),
                from: edge.from.clone(),
                to: edge.to.clone(),
                shape: edge.shape.clone(),
                length: longest(&lanes),
                speeds: vec![None; lanes.len()],
                lanes,
            }]);
        };
        let limit = |lane: &EdgeLane| lane.speed.unwrap_or(edge.speed);
        let centre = curves::Plan::new(&edge.shape);
        let plans: Vec<curves::Plan> = edge
            .lanes
            .iter()
            .map(|lane| curves::Plan::new(&lane.shape))
            .collect();
        // No cut after the furthest-back stop line, as each lane measures it, so
        // every stop offset lands on the last piece and fits on its lane.
        let latest_cut = self
            .stop_offsets
            .range((index, 0)..(index + 1, 0))
            .map(|(&(_, lane), (_, distance))| {
                let plan = &plans[lane];
                let line = plan.point_at(plan.length() - distance - MIN_STOP_OFFSET);
                station_along(&edge.shape, line).0
            })
            .fold(f64::INFINITY, f64::min);
        let fastest = edge.lanes.iter().map(limit).fold(edge.speed, f64::max);
        let stretches = curves::stretches(
            &centre.curve_speeds(acceleration),
            centre.length(),
            fastest,
            latest_cut,
        );
        let whole = stretches.len() == 1;
        // Each lane is cut where the centre line is, across the carriageway.
        let lane_stations: Vec<Vec<f64>> = edge
            .lanes
            .iter()
            .zip(&plans)
            .map(|(lane, plan)| {
                std::iter::once(0.0)
                    .chain(stretches[1..].iter().map(|stretch| {
                        station_along(&lane.shape, centre.point_at(stretch.start)).0
                    }))
                    .chain(std::iter::once(plan.length()))
                    .collect()
            })
            .collect();
        let lane_speeds: Vec<Vec<f64>> = plans
            .iter()
            .map(|plan| plan.curve_speeds(acceleration))
            .collect();
        let node = |k: usize| format!("n_{}_p{k}", edge.id);
        let mut pieces = Vec::with_capacity(stretches.len());
        for (k, stretch) in stretches.iter().enumerate() {
            let lanes: Vec<Polyline3> = if whole {
                edge.lanes.iter().map(|lane| lane.shape.clone()).collect()
            } else {
                plans
                    .iter()
                    .zip(&lane_stations)
                    .map(|(plan, stations)| plan.slice(stations[k], stations[k + 1]))
                    .collect::<Result<_, _>>()?
            };
            pieces.push(Piece {
                id: if k == 0 {
                    edge.id.clone()
                } else {
                    format!("{}.p{k}", edge.id)
                },
                from: if k == 0 { edge.from.clone() } else { node(k) },
                to: if k + 1 == stretches.len() {
                    edge.to.clone()
                } else {
                    node(k + 1)
                },
                shape: if whole {
                    edge.shape.clone()
                } else {
                    centre.slice(stretch.start, stretch.end)?
                },
                speeds: edge
                    .lanes
                    .iter()
                    .zip(&lane_speeds)
                    .zip(&lane_stations)
                    .map(|((lane, speeds), stations)| {
                        curves::cap(speeds, stations[k], stations[k + 1], limit(lane))
                    })
                    .collect(),
                length: longest(&lanes),
                lanes,
            });
        }
        Ok(pieces)
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
            // A line is measured against the lanes of its rule that stop at it —
            // all of them for a rule with one line, the line's own for a rule
            // with one per mouth (`Map::traffic_light_stops`, as Lanelet2 has it)
            // — and which of those it actually crosses is found below.
            match rule {
                TrafficRule::RightOfWay {
                    stop_line: Some(stop_line),
                    yielding,
                    ..
                } => governed.entry(stop_line).or_default().extend(yielding),
                TrafficRule::TrafficLight {
                    stop_lines, lanes, ..
                } => {
                    for (stop_line, stopped) in map.traffic_light_stops(stop_lines, lanes) {
                        let Some(id) = stop_lines.iter().find(|id| Some(*id) == stop_line.as_ref())
                        else {
                            continue;
                        };
                        governed
                            .entry(id)
                            .or_default()
                            .extend(lanes.iter().filter(|lane| stopped.contains(lane)));
                    }
                }
                _ => {}
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
            // Where the line crosses each written lane it is measured against, before
            // anything is decided about any of them: whether the line is in doubt
            // about its direction depends on all of them together.
            let mut crossed = Vec::new();
            for lane in lanes {
                let own = named.contains(lane);
                let Some(&slot) = self.slots.get(lane) else {
                    if own {
                        self.unplace(Unplaced::LaneNotWritten, &object.id, Some(lane));
                    }
                    continue;
                };
                let shape = &self.edges[slot.edge].lanes[slot.index].shape;
                let Some(crossing) = crossing(shape, &line) else {
                    if own {
                        self.unplace(Unplaced::NotAcross, &object.id, Some(lane));
                    }
                    continue;
                };
                crossed.push((lane, slot, crossing));
            }
            // A line across both carriageways: some lane it crosses passes over it
            // one way and another the other way.
            let both_ways = crossed
                .iter()
                .any(|(_, _, a)| crossed.iter().any(|(_, _, b)| a.sense != b.sense));
            let ruled = governed.get(&object.id);
            for (lane, slot, crossing) in crossed {
                // Nothing says which way a line runs, and one drawn across the whole
                // road names the lanes leaving a junction as well as the ones
                // entering it. That is only in doubt where the line does cross lanes
                // running both ways — one whose lanes all run the same way stands
                // before wherever they run to, however short the edge. Where it is in
                // doubt, it is settled on the IR, not on the SUMO edges the roads are
                // cut into: the line belongs to the carriageway whose lanes it is
                // nearer the junction ahead of than the one behind, measured along
                // the road and the roads joined to it. See [`Exporter::line_end`].
                if both_ways {
                    let belongs = match self.line_end(lane, crossing.point)? {
                        LineEnd::Exit => true,
                        LineEnd::Entry => false,
                        LineEnd::Middle => ruled.is_some_and(|lanes| lanes.contains(lane)),
                    };
                    if !belongs {
                        self.unplace(Unplaced::AtLaneStart, &object.id, Some(lane));
                        continue;
                    }
                }
                let distance = crossing.from_end;
                let edge = &self.edges[slot.edge];
                let shape = &edge.lanes[slot.index].shape;
                let at_junction = self
                    .nodes
                    .get(&edge.to)
                    .is_some_and(|node| node.junction.is_some());
                if !at_junction {
                    // Set back further than the edge that reaches the junction is
                    // long, the line falls on an earlier edge of the same approach;
                    // only one exactly at that edge's end is at a mere boundary.
                    let reason = if distance > MIN_STOP_OFFSET && self.junction_beyond(slot) {
                        Unplaced::BeyondLastEdge
                    } else {
                        Unplaced::NoJunctionAhead
                    };
                    self.unplace(reason, &object.id, Some(lane));
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
            self.stopping.insert(object.clone());
            // A line at the very end of the lane is where SUMO stops a vehicle
            // anyway; writing an offset of nothing would only add noise.
            if distance > MIN_STOP_OFFSET {
                self.stop_offsets.insert(key, (object, distance));
            }
        }
        Ok(())
    }

    /// Whether the lane in `slot`, on an edge that does not end at a junction, runs
    /// on into one: through the one written lane it continues into at each node on
    /// the way, as across a change of cross-section or a joint between two roads.
    fn junction_beyond(&self, slot: Slot) -> bool {
        let (mut edge, mut index) = (slot.edge, slot.index);
        for _ in 0..self.edges.len() {
            let mut next = self
                .movements
                .range((edge, index, 0, 0)..=(edge, index, usize::MAX, usize::MAX))
                .map(|(&(_, _, to_edge, to_lane), _)| (to_edge, to_lane));
            let (Some(following), None) = (next.next(), next.next()) else {
                return false;
            };
            (edge, index) = following;
            let ends_at_junction = self
                .nodes
                .get(&self.edges[edge].to)
                .is_some_and(|node| node.junction.is_some());
            if ends_at_junction {
                return true;
            }
        }
        false
    }

    /// Which junction the lane `lane` is nearer at `point`, where a stop line crosses
    /// it: the one it runs into, or the one it leaves.
    ///
    /// Measured on the IR, along the lane's road and on through the roads it is
    /// joined to end to end — across a plain joint, not through a junction — to the
    /// first junction reached each way, so the answer is the same however the roads
    /// are cut into SUMO edges. It is the junctions that matter, not the road's own
    /// ends: a road joined to the next by a plain joint at one end and running into
    /// a junction at the other is approached from far beyond its start, and a stub
    /// with a dead end has a junction only one way. A line nearer the junction ahead
    /// is before it; one nearer the junction behind is at the start of the lane, and
    /// belongs to the lanes running the other way. With no junction either way, or
    /// the line exactly halfway between them, it is in the middle.
    fn line_end(&self, lane: &LaneId, point: Point3) -> Result<LineEnd, ExportError> {
        let Some((lane, road)) = self
            .map
            .lane(lane)
            .and_then(|lane| Some((lane, self.map.road(&lane.road)?)))
        else {
            // A written lane is always on a road of the map.
            return Ok(LineEnd::Exit);
        };
        let reference = road.reference_line.to_polyline(self.sampling)?;
        let (station, length) = station_along(&reference, point);
        let to_start = station + self.junction_beyond_end(road, RoadEnd::Start)?;
        let to_end = length - station + self.junction_beyond_end(road, RoadEnd::End)?;
        let (ahead, behind) = match lane.direction {
            Direction::Forward => (to_end, to_start),
            Direction::Backward => (to_start, to_end),
        };
        Ok(if ahead == behind || (ahead - behind).abs() <= 1e-6 {
            LineEnd::Middle
        } else if ahead < behind {
            LineEnd::Exit
        } else {
            LineEnd::Entry
        })
    }

    /// How far beyond the end `end` of `road` the first junction is: nothing if the
    /// road runs into one there, the length of every road joined end to end on the
    /// way to one otherwise, and infinitely far at a dead end — or after going round
    /// a loop of plain joints without ever meeting one.
    fn junction_beyond_end(&self, road: &Road, end: RoadEnd) -> Result<f64, ExportError> {
        let mut distance = 0.0;
        let mut seen: HashSet<&RoadId> = HashSet::from([&road.id]);
        let mut at = road.link.at(end);
        loop {
            let next = match at {
                None => return Ok(f64::INFINITY),
                Some(RoadLinkTarget::Junction(_)) => return Ok(distance),
                Some(RoadLinkTarget::Road(endpoint)) => endpoint,
            };
            let Some(joined) = self.map.road(&next.road) else {
                return Ok(f64::INFINITY);
            };
            if joined.is_connector() {
                // The road beyond is a junction's own.
                return Ok(distance);
            }
            if !seen.insert(&joined.id) {
                return Ok(f64::INFINITY);
            }
            distance += joined.horizontal_length()?;
            at = joined.link.at(next.end.opposite());
        }
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
    fn edges_at(&self, road: &RoadId, end: RoadEnd) -> Option<(String, Vec<usize>)> {
        let road = self.map.road(road)?;
        let section = match end {
            RoadEnd::Start => 0,
            RoadEnd::End => road.sections.len() - 1,
        };
        let mut node = None;
        let mut edges = Vec::new();
        for direction in [Direction::Forward, Direction::Backward] {
            let id = edge_id(&self.ids.road(&road.id), road, section, direction);
            let Some((index, edge)) = self
                .edges
                .iter()
                .enumerate()
                .find(|(_, edge)| edge.id == id)
            else {
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
            edges.push(index);
        }
        edges.sort();
        Some((node?, edges)).filter(|(_, edges)| !edges.is_empty())
    }

    /// The crossings as they are written: each by its node and the edges it crosses
    /// there, sorted by name. An edge cut at curves is crossed on the piece at the
    /// node — the last of one arriving, the first (which keeps the edge's id) of one
    /// leaving.
    fn written_crossings(&self) -> Vec<(String, Vec<String>, &Crossing)> {
        let mut written: Vec<_> = self
            .crossings
            .iter()
            .map(|((node, edges), crossing)| {
                let mut names: Vec<String> = edges
                    .iter()
                    .map(|&index| {
                        let edge = &self.edges[index];
                        let piece = if &edge.to == node {
                            edge.exit()
                        } else {
                            &edge.pieces[0]
                        };
                        piece.id.clone()
                    })
                    .collect();
                names.sort();
                (node.clone(), names, crossing)
            })
            .collect();
        written.sort_by(|a, b| (&a.0, &a.1).cmp(&(&b.0, &b.1)));
        written
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
                self.options.curve_lateral_acceleration,
            ),
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
        // A light is part of the traffic light of the node it made one, as
        // `read_rules` decided when it chose which nodes are signalised — or, at a
        // node with nothing to control and so no program, part of the node.
        let link_indices = self.link_indices();
        for (junction, lights) in &self.signalised {
            let Some(node) = junction_nodes.get(junction) else {
                continue;
            };
            let signal = self.signals.get(*node);
            // The movements of the program a light governs: off the lanes it stands
            // over, or off every lane its rules give it. None without a program.
            let reach = |light: &ObjectId, own: bool| -> BTreeSet<&(usize, usize, usize, usize)> {
                let Some(signal) = signal else {
                    return BTreeSet::new();
                };
                let lanes = if own {
                    self.map
                        .objects
                        .get(light)
                        .map(|object| object.lanes.iter().collect())
                } else {
                    self.governed.get(light).map(|lanes| lanes.iter().collect())
                };
                let lanes: BTreeSet<&LaneId> = lanes.unwrap_or_default();
                signal
                    .links
                    .iter()
                    .filter(|key| {
                        let from = &self.edges[key.0].lanes[key.1].lane;
                        lanes.contains(from)
                            || self.movements[*key]
                                .connectors
                                .iter()
                                .any(|lane| lanes.contains(lane))
                    })
                    .collect()
            };
            let own: BTreeMap<&ObjectId, BTreeSet<_>> = lights
                .iter()
                .map(|light| (light, reach(light, true)))
                .collect();
            let claimed: BTreeSet<_> = own.values().flatten().copied().collect();
            for light in lights {
                let local = match signal {
                    Some(signal) => format!("tls:{}", signal.id),
                    None => format!("node:{node}"),
                };
                trace.link_as(light.clone(), local, Relation::Merged, "traffic_light");
                // And of the slot of every movement off a lane it governs. A light on
                // a connector governs the movements that run over it.
                //
                // The lanes the light stands over come first. A rule hands each of its
                // lights every lane of the rule — an OpenDRIVE controller lists the
                // signals that switch together, and the reader makes it one rule over
                // all their lanes — which is right for the program, but traced that way
                // two lights of one controller would each claim the other's movements,
                // and a simulator that does switch them apart could not tell which
                // light a movement follows. So a light answers for the movements off
                // its own lanes, and for those its rules give it that no light here
                // stands over.
                let unclaimed = reach(light, false)
                    .into_iter()
                    .filter(|key| !claimed.contains(key));
                let keys: BTreeSet<_> = own[light].iter().copied().chain(unclaimed).collect();
                for key in keys {
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
                for piece in &self.edges[edge].pieces {
                    trace.link_as(
                        IrRef::Rule(index),
                        format!("edge:{}", piece.id),
                        Relation::Merged,
                        "priority",
                    );
                }
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
                    for piece in &edge.pieces {
                        trace.link_as(
                            IrRef::Rule(*rule),
                            format!("lane:{}_{index}", piece.id),
                            Relation::Merged,
                            "speed",
                        );
                    }
                }
            }
        }

        // An edge cut at curves is several edges, and each of its lanes several lanes
        // and the connections straight on between them: all of them part of the IR's.
        for edge in &self.edges {
            let lane_relation = if edge.pieces.len() == 1 {
                Relation::Exact
            } else {
                Relation::Part
            };
            for piece in &edge.pieces {
                trace.link(
                    edge.road.clone(),
                    format!("edge:{}", piece.id),
                    Relation::Part,
                );
                for (index, lane) in edge.lanes.iter().enumerate() {
                    trace.link(
                        lane.lane.clone(),
                        format!("lane:{}_{index}", piece.id),
                        lane_relation,
                    );
                }
            }
        }
        for (lane, from, to, index) in self.piece_joints() {
            trace.link(
                lane.lane.clone(),
                format!("connection:{}_{index}>{}_{index}", from.id, to.id),
                Relation::Part,
            );
        }

        // A stop line is carried by the stop offset of each lane it was measured
        // against: one attribute of a lane that is otherwise the IR's lane.
        for (&(edge, index), (object, _)) in &self.stop_offsets {
            trace.link_as(
                object.clone(),
                format!("lane:{}_{index}", self.edges[edge].exit().id),
                Relation::Merged,
                "stopOffset",
            );
        }

        for (&(from_edge, from_lane, to_edge, to_lane), movement) in &self.movements {
            let local = format!(
                "connection:{}_{from_lane}>{}_{to_lane}",
                self.edges[from_edge].exit().id,
                self.edges[to_edge].id
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
        for (node, edges, crossing) in self.written_crossings() {
            let local = format!("crossing:{}", crossing_name(&node, &edges));
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
        for (edge_index, edge) in self.edges.iter().enumerate() {
            for piece_index in 0..edge.pieces.len() {
                self.render_piece(&mut document, edge_index, piece_index);
            }
        }
        document.finish()
    }

    /// One `<edge>`: piece `piece_index` of edge `edge_index`. The last piece
    /// carries the edge's stop offsets.
    fn render_piece(&self, document: &mut xml::Document, edge_index: usize, piece_index: usize) {
        let edge = &self.edges[edge_index];
        let piece = &edge.pieces[piece_index];
        let last = piece_index + 1 == edge.pieces.len();
        let mut attributes = vec![
            ("id", piece.id.clone()),
            ("from", piece.from.clone()),
            ("to", piece.to.clone()),
            ("priority", edge.priority.to_string()),
            ("numLanes", edge.lanes.len().to_string()),
            ("speed", metres(edge.speed)),
            // The lanes carry their own shapes, so this one is the middle of the
            // carriageway and is spread from as such.
            ("spreadType", "center".to_owned()),
            ("shape", shape(&piece.shape)),
        ];
        if let Some(name) = &edge.name {
            attributes.push(("name", name.clone()));
        }
        document.open("edge", &attributes);
        for (index, ((lane, lane_shape), curve_speed)) in edge
            .lanes
            .iter()
            .zip(&piece.lanes)
            .zip(&piece.speeds)
            .enumerate()
        {
            let mut attributes = vec![
                ("index", index.to_string()),
                ("width", metres(lane.width)),
                ("shape", shape(lane_shape)),
            ];
            if let Some(speed) = curve_speed.or(lane.speed) {
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
            match self.stop_offsets.get(&(edge_index, index)).filter(|_| last) {
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

    /// The lane-to-lane connections that carry an edge on from each of its pieces
    /// to the next, as (lane, from piece, to piece, lane index).
    fn piece_joints(&self) -> impl Iterator<Item = (&EdgeLane, &Piece, &Piece, usize)> {
        self.edges.iter().flat_map(|edge| {
            edge.pieces.windows(2).flat_map(move |pair| {
                edge.lanes
                    .iter()
                    .enumerate()
                    .map(move |(index, lane)| (lane, &pair[0], &pair[1], index))
            })
        })
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
                if let Some(speed) = self.turn_speed(key, path) {
                    attributes.push(("speed", metres(speed)));
                }
            }
            document.leaf("connection", &attributes);
        }
        // Each lane carries straight on from one piece of its edge to the next.
        for (_, from, to, lane) in self.piece_joints() {
            document.leaf(
                "connection",
                &[
                    ("from", from.id.clone()),
                    ("to", to.id.clone()),
                    ("fromLane", lane.to_string()),
                    ("toLane", lane.to_string()),
                ],
            );
        }
        // `priority` is written as a number because that is what SUMO's schema for
        // the file says it is, though netconvert reads it as a yes or no: with
        // `SUMO_HOME` set, netconvert validates the file and refuses `true`.
        //
        // It is always yes. A crosswalk in the IR is a painted one — it has an
        // outline, the stripes the other exports draw — and at a marked crossing
        // traffic gives way to pedestrians on it. At a signalised node the signal
        // decides instead: netconvert takes the program written to `.tll.xml` and
        // appends the crossing's links after the export's, numbered on from the last
        // of them, splitting each green to end it with a pedestrian clearance. The
        // vehicle links and their indices stay the export's.
        for (node, edges, crossing) in self.written_crossings() {
            document.leaf(
                "crossing",
                &[
                    ("node", node),
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
                    &[("from", edge.exit().id.clone()), ("to", next.id.clone())],
                );
            }
        }
        document.finish()
    }

    /// The speed a movement's path across a junction holds traffic to, where the
    /// options ask for curve speeds and that is below what either lane allows.
    ///
    /// netconvert's own `junctions.limit-turn-speed` reads a radius off the whole of
    /// the internal lane — its length over how far it turns — which is the mean of a
    /// turn drawn tighter in the middle than at its ends, as the connectors of real
    /// junctions are. So the export measures the IR's path itself, as it does the
    /// edges, and states the speed.
    fn turn_speed(
        &self,
        &(from_edge, from_lane, to_edge, to_lane): &(usize, usize, usize, usize),
        path: &Polyline3,
    ) -> Option<f64> {
        let acceleration = self.options.curve_lateral_acceleration?;
        let lane_speed = |edge: usize, lane: usize| {
            let edge = &self.edges[edge];
            edge.lanes[lane].speed.unwrap_or(edge.speed)
        };
        let limit = lane_speed(from_edge, from_lane).min(lane_speed(to_edge, to_lane));
        curves::slowest(path, acceleration).filter(|&speed| speed < limit - curves::MARGIN)
    }

    /// The four attributes that name a written connection.
    fn connection_attributes(
        &self,
        &(from_edge, from_lane, to_edge, to_lane): &(usize, usize, usize, usize),
    ) -> Vec<(&'static str, String)> {
        vec![
            ("from", self.edges[from_edge].exit().id.clone()),
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
///
/// The traffic-light file is named only when there is one, which is when the map has
/// a signalised junction with something to control.
fn render_config(
    prefix: &str,
    handedness: TrafficHandedness,
    traffic_lights: bool,
    turn_acceleration: Option<f64>,
) -> String {
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
    if let Some(acceleration) = turn_acceleration {
        // The internal lanes' counterpart of the edges' curve speeds; see `curves`.
        document.open("junctions", &[]);
        document.leaf(
            "junctions.limit-turn-speed",
            &[("value", metres(acceleration))],
        );
        document.close("junctions");
    }
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

/// Where a path crosses a line: see [`crossing`].
#[derive(Debug, Clone, Copy)]
struct LineCrossing {
    /// How far back from the far end of the path, measured along it.
    from_end: f64,
    /// Which way the path passes over the line. Two paths cross it the same way
    /// exactly where this agrees, whichever way the line itself was drawn.
    sense: bool,
    /// Where on the path the line crosses it.
    point: Point3,
}

/// How far back from the far end of `path` the line `line` crosses it, measured
/// along `path`, and which way; `None` if the two never meet.
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
fn crossing(path: &Polyline3, line: &Polyline3) -> Option<LineCrossing> {
    const TOLERANCE: f64 = 1e-6;
    let cross = |a: (f64, f64), b: (f64, f64)| a.0 * b.1 - a.1 * b.0;
    let total = path.length();
    let mut travelled = 0.0;
    let mut best: Option<LineCrossing> = None;
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
                if best.is_none_or(|previous| remaining < previous.from_end) {
                    best = Some(LineCrossing {
                        from_end: remaining,
                        sense: denominator > 0.0,
                        point: a.lerp(b, t.clamp(0.0, 1.0)),
                    });
                }
            }
        }
        travelled += length;
    }
    best
}

/// How far along `path` the point of it nearest `point` is, in plan, and how long
/// `path` is in plan: the station of `point` on a road whose reference line `path`
/// samples, and the road's length measured the same way.
fn station_along(path: &Polyline3, point: Point3) -> (f64, f64) {
    let mut travelled = 0.0;
    let mut best = (f64::INFINITY, 0.0);
    for piece in path.points().windows(2) {
        let (c, d) = (piece[0], piece[1]);
        let along = (d.x - c.x, d.y - c.y);
        let length = along.0.hypot(along.1);
        let t = if length > 0.0 {
            (((point.x - c.x) * along.0 + (point.y - c.y) * along.1) / (length * length))
                .clamp(0.0, 1.0)
        } else {
            0.0
        };
        let off = (point.x - c.x - t * along.0).hypot(point.y - c.y - t * along.1);
        if off < best.0 {
            best = (off, travelled + t * length);
        }
        travelled += length;
    }
    (best.1, travelled)
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
                        if let (Some(tl), Some(index)) =
                            (attributes.get("tl"), attributes.get("linkIndex"))
                        {
                            found.insert(format!("tls:{tl}/{index}"));
                        }
                    }
                    b"tlLogic" => {
                        found.insert(format!("tls:{}", attributes["id"]));
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
                && about[0].contains("it is at the lane's start"),
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
                    .contains("it is at the lane's start: drawn across lanes running both ways"),
            "{report:#?}"
        );
    }

    /// The road between two junctions of the test above, 100 m long, its
    /// cross-section changing at `change`, with a line at x = 92 — 8 m short of the
    /// second junction — drawn across both carriageways and naming the lane of each
    /// that it crosses. Returns the map, the line, and the eastbound lane approaching
    /// the line's junction and the westbound one leaving it.
    fn line_across_a_split_road(change: f64) -> (ValidatedMap, ObjectId, LaneId, LaneId) {
        let mut builder = MapBuilder::new(metadata("split"));
        let road = |builder: &mut MapBuilder, name: &str, from: f64, to: f64, change| {
            let spec = RoadSpec::line(
                Point3::new(from, 0.0, 0.0),
                Point3::new(to, 0.0, 0.0),
                two_way(),
            )
            .unwrap()
            .with_name(name);
            let spec = match change {
                Some(station) => spec.with_cross_section(station, two_way()),
                None => spec,
            };
            builder.add_road(spec).unwrap()
        };
        let west = road(&mut builder, "west", -100.0, -10.0, None);
        let mid = road(&mut builder, "mid", 0.0, 100.0, Some(change));
        let east = road(&mut builder, "east", 110.0, 200.0, None);
        let ja = builder.add_junction(Some("ja"));
        let jb = builder.add_junction(Some("jb"));
        builder
            .connect_ends(&west, RoadEnd::End, &mid, RoadEnd::Start, Some(&ja))
            .unwrap();
        builder
            .connect_ends(&mid, RoadEnd::End, &east, RoadEnd::Start, Some(&jb))
            .unwrap();
        // Lanes 0 and 1 are the first cross-section's, 2 and 3 the second's.
        let section = usize::from(change < 92.0) * 2;
        let line = builder
            .add_stop_line(&LaneRef::new(mid.clone(), section), LaneEnd::End)
            .unwrap();
        let mut map =
            UnvalidatedMap::from_map(builder.finish().unwrap().validate().unwrap().into_map());
        let approaching = LaneId::of_road(&mid, section);
        let leaving = LaneId::of_road(&mid, section + 1);
        let object = map
            .as_map_mut()
            .objects
            .get_mut(&line)
            .expect("the stop line");
        object.geometry = ObjectGeometry::Line(
            Curve3::line(Point3::new(92.0, -7.0, 0.0), Point3::new(92.0, 7.0, 0.0)).unwrap(),
        );
        object.lanes = vec![approaching.clone(), leaving.clone()];
        (map.validate().unwrap(), line, approaching, leaving)
    }

    /// What `check` says `line` is not written on, one entry per line of the report.
    fn said_about(map: &ValidatedMap, line: &ObjectId) -> Vec<String> {
        check(map)
            .into_iter()
            .filter(|said| said.contains(line.as_str()) && said.contains("is not written"))
            .collect()
    }

    #[test]
    fn a_stop_line_across_both_carriageways_of_a_road_split_behind_it_stops_nothing_far_back() {
        // The cross-section changes 5 m short of the second junction, behind the
        // line: both lanes the line crosses are on the first section's edges, and
        // neither of those edges reaches the junction ahead of the eastbound lane.
        // The westbound lane leaves that junction, so the line is at its start — not
        // a stop 92 m short of the first junction.
        let (map, line, approaching, leaving) = line_across_a_split_road(95.0);
        let network = to_plain_xml(&map).unwrap();
        let offsets = stop_offsets(&network);
        assert!(offsets.is_empty(), "{offsets:?}");

        let said = said_about(&map, &line);
        assert_eq!(said.len(), 2, "{said:#?}");
        assert!(
            said.iter().any(|line_| line_
                .contains(&format!("is not written on lane {}:", leaving.local_name()))
                && line_.contains("it is at the lane's start")),
            "{said:#?}"
        );
        assert!(
            said.iter().any(|line_| line_.contains(&format!(
                "is not written on lane {}:",
                approaching.local_name()
            )) && line_
                .contains("further back from the junction than the last edge")),
            "{said:#?}"
        );
    }

    #[test]
    fn a_stop_line_across_both_carriageways_of_a_road_split_midway_is_at_the_leaving_start() {
        // The cross-section changes halfway, so the eastbound lane's last edge
        // reaches the junction 8 m past the line and takes the offset; the westbound
        // lane's edge there runs from that junction to the change of cross-section,
        // and the line is at its start, not on an earlier edge of some approach.
        let (map, line, approaching, leaving) = line_across_a_split_road(50.0);
        let network = to_plain_xml(&map).unwrap();
        let offsets = stop_offsets(&network);
        let written = network.lanes[&approaching].clone();
        assert_eq!(
            offsets.keys().collect::<Vec<_>>(),
            [&written],
            "{offsets:?}"
        );
        assert!((offsets[&written] - 8.0).abs() < 1e-3, "{offsets:?}");

        let said = said_about(&map, &line);
        assert_eq!(said.len(), 1, "{said:#?}");
        assert!(
            said[0].contains(&format!("is not written on lane {}:", leaving.local_name()))
                && said[0].contains("it is at the lane's start")
                && !said[0].contains("further back"),
            "{said:#?}"
        );
    }

    #[test]
    fn a_stop_line_across_both_carriageways_is_judged_by_the_junctions_not_the_road_ends() {
        // A road `b` running east into junction `jb`, with a line `setback` metres
        // short of it drawn across both carriageways. Behind `b` is either a 200 m
        // road `a` joined to it by a plain joint, no junction, or nothing — a dead
        // end. Either way the only junction near the line is the one ahead of the
        // eastbound lane, however far past the middle of `b` itself the line is.
        for (length, setback, joined) in [(12.0, 8.0, true), (30.0, 20.0, true), (12.0, 8.0, false)]
        {
            let context = format!("{length} m, {setback} m back, joined: {joined}");
            let mut builder = MapBuilder::new(metadata("joint"));
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
            let b = road(&mut builder, "b", 0.0, length);
            let c = road(&mut builder, "c", length + 10.0, length + 100.0);
            if joined {
                let a = road(&mut builder, "a", -200.0, 0.0);
                builder
                    .connect_ends(&a, RoadEnd::End, &b, RoadEnd::Start, None)
                    .unwrap();
            }
            let jb = builder.add_junction(Some("jb"));
            builder
                .connect_ends(&b, RoadEnd::End, &c, RoadEnd::Start, Some(&jb))
                .unwrap();
            let line = builder
                .add_stop_line_at(&LaneRef::new(b.clone(), 0), LaneEnd::End, setback)
                .unwrap();
            let mut map =
                UnvalidatedMap::from_map(builder.finish().unwrap().validate().unwrap().into_map());
            let approaching = LaneId::of_road(&b, 0);
            let leaving = LaneId::of_road(&b, 1);
            let x = length - setback;
            let object = map
                .as_map_mut()
                .objects
                .get_mut(&line)
                .expect("the stop line");
            object.geometry = ObjectGeometry::Line(
                Curve3::line(Point3::new(x, -7.0, 0.0), Point3::new(x, 7.0, 0.0)).unwrap(),
            );
            object.lanes = vec![approaching.clone(), leaving.clone()];
            let map = map.validate().unwrap();
            let network = to_plain_xml(&map).unwrap();

            let offsets = stop_offsets(&network);
            let written = network.lanes[&approaching].clone();
            assert_eq!(
                offsets.keys().collect::<Vec<_>>(),
                [&written],
                "{context}: {offsets:?}"
            );
            assert!(
                (offsets[&written] - setback).abs() < 1e-3,
                "{context}: {offsets:?}"
            );

            let said = said_about(&map, &line);
            assert_eq!(said.len(), 1, "{context}: {said:#?}");
            assert!(
                said[0].contains(&format!("is not written on lane {}:", leaving.local_name()))
                    && said[0].contains("it is at the lane's start")
                    && said[0].contains("nearer the junction the lane leaves")
                    && !said[0].contains("does not end at a junction"),
                "{context}: {said:#?}"
            );
        }
    }

    #[test]
    fn a_stop_line_on_one_lane_of_a_short_road_between_junctions_is_before_its_junction() {
        // The same three roads with the middle one only 12 m long, and a line on
        // each of its lanes 7 m back from the junction that lane runs into: past the
        // middle of the road, but each line names only the lane it stands on, so
        // there is no doubt which junction it stands before.
        for (length, setback) in [(12.0, 7.0), (16.0, 9.0), (100.0, 8.0)] {
            let mut builder = MapBuilder::new(metadata("short_link"));
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
            let mid = road(&mut builder, "mid", 0.0, length);
            let east = road(&mut builder, "east", length + 10.0, length + 100.0);
            let ja = builder.add_junction(Some("ja"));
            let jb = builder.add_junction(Some("jb"));
            builder
                .connect_ends(&west, RoadEnd::End, &mid, RoadEnd::Start, Some(&ja))
                .unwrap();
            builder
                .connect_ends(&mid, RoadEnd::End, &east, RoadEnd::Start, Some(&jb))
                .unwrap();
            let eastbound = builder
                .add_stop_line_at(&LaneRef::new(mid.clone(), 0), LaneEnd::End, setback)
                .unwrap();
            let westbound = builder
                .add_stop_line_at(&LaneRef::new(mid.clone(), 1), LaneEnd::Start, setback)
                .unwrap();
            let map = builder.finish().unwrap().validate().unwrap();
            let network = to_plain_xml(&map).unwrap();

            let offsets = stop_offsets(&network);
            for lane in [0, 1] {
                let written = &network.lanes[&LaneId::of_road(&mid, lane)];
                assert!(
                    offsets
                        .get(written)
                        .is_some_and(|offset| (offset - setback).abs() < 1e-3),
                    "{length} m, {setback} m: {offsets:?}"
                );
            }
            let report = check(&map);
            for line in [&eastbound, &westbound] {
                assert!(
                    !report.iter().any(|line_| line_.contains(line.as_str())),
                    "{length} m, {setback} m: {report:#?}"
                );
            }
        }
    }

    #[test]
    fn a_stop_line_further_back_than_the_last_section_is_beyond_the_last_edge() {
        // The tee's approach, named on its first section's lane 30 m back from the
        // junction: further back than the 26 m section that reaches it.
        let mut builder = MapBuilder::new(metadata("beyond"));
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
        let junction = builder.add_junction(Some("t"));
        for (x, name) in [(70.0, "east"), (-70.0, "west")] {
            let other = builder
                .add_road(
                    RoadSpec::line(
                        Point3::new(x, 0.0, 0.0),
                        Point3::new(x / 5.0, 0.0, 0.0),
                        two_way(),
                    )
                    .unwrap()
                    .with_name(name),
                )
                .unwrap();
            builder
                .connect_ends(&north, RoadEnd::End, &other, RoadEnd::End, Some(&junction))
                .unwrap();
        }
        let line = builder
            .add_stop_line_at(&LaneRef::new(north.clone(), 0), LaneEnd::End, 4.0)
            .unwrap();
        let map = builder.finish().unwrap().validate().unwrap();
        assert!(stop_offsets(&to_plain_xml(&map).unwrap()).is_empty());

        let report = check(&map);
        let about: Vec<&String> = report
            .iter()
            .filter(|line_| line_.contains(line.as_str()))
            .collect();
        assert_eq!(about.len(), 1, "{report:#?}");
        assert!(
            about[0].contains("further back from the junction than the last edge")
                && !about[0].contains("does not end at a junction"),
            "{report:#?}"
        );
    }

    #[test]
    fn a_stop_line_on_a_short_last_section_is_still_before_its_junction() {
        // The tee again, its northern road's cross-section changing only 6 m short
        // of the junction, with the stop line 4 m back: more than half of that last
        // edge, but well within the lane, which leaves no junction to be at the
        // start of.
        let mut builder = MapBuilder::new(metadata("short"));
        let north = builder
            .add_road(
                RoadSpec::line(
                    Point3::new(0.0, 70.0, 0.0),
                    Point3::new(0.0, 14.0, 0.0),
                    two_way(),
                )
                .unwrap()
                .with_name("north")
                .with_cross_section(50.0, two_way()),
            )
            .unwrap();
        let junction = builder.add_junction(Some("t"));
        for (x, name) in [(70.0, "east"), (-70.0, "west")] {
            let other = builder
                .add_road(
                    RoadSpec::line(
                        Point3::new(x, 0.0, 0.0),
                        Point3::new(x / 5.0, 0.0, 0.0),
                        two_way(),
                    )
                    .unwrap()
                    .with_name(name),
                )
                .unwrap();
            builder
                .connect_ends(&north, RoadEnd::End, &other, RoadEnd::End, Some(&junction))
                .unwrap();
        }
        let line = builder
            .add_stop_line_at(&LaneRef::new(north.clone(), 2), LaneEnd::End, 4.0)
            .unwrap();
        let map = builder.finish().unwrap().validate().unwrap();
        let network = to_plain_xml(&map).unwrap();

        let approach = LaneId::of_road(&north, 2);
        assert_eq!(network.lanes[&approach], "north.1.fwd_0");
        let offsets = stop_offsets(&network);
        assert_eq!(offsets.keys().collect::<Vec<_>>(), ["north.1.fwd_0"]);
        assert!((offsets["north.1.fwd_0"] - 4.0).abs() < 1e-3, "{offsets:?}");

        let report = check(&map);
        assert!(
            !report.iter().any(|line_| line_.contains(line.as_str())),
            "{report:#?}"
        );
        assert!(
            report.iter().any(|line_| line_.contains(
                "1 stop line is written as the stopOffset of the \
                                             lanes it crosses"
            )),
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
        let near = |value: Option<LineCrossing>, expected: f64| {
            value.is_some_and(|value| (value.from_end - expected).abs() < 1e-9)
        };
        assert!(near(crossing(&path, &across(4.0, 0.0, 0.0, 2.0)), 16.0));
        assert!(near(crossing(&path, &across(10.0, 7.0, 2.0, 0.0)), 3.0));
        // Drawn exactly to the end, it is at the end.
        assert!(near(crossing(&path, &across(10.0, 10.0, 2.0, 0.0)), 0.0));
        // The path back the other way crosses the same line the other way.
        let back =
            Polyline3::new([Point3::new(10.0, 0.0, 0.0), Point3::new(0.0, 0.0, 0.0)]).unwrap();
        let line = across(4.0, 0.0, 0.0, 2.0);
        assert_ne!(
            crossing(&path, &line).unwrap().sense,
            crossing(&back, &line).unwrap().sense
        );
        // Beside the path, or along it, it crosses nothing.
        assert!(crossing(&path, &across(4.0, 5.0, 0.0, 2.0)).is_none());
        assert!(crossing(&path, &across(4.0, 0.0, 2.0, 0.0)).is_none());
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

        let mut exporter = Exporter::new(&map, Options::default());
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

        let mut exporter = Exporter::new(&map, Options::default());
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
    fn lights_switched_together_are_traced_to_their_own_approaches() {
        // An OpenDRIVE controller over two lights, read as one rule over both their
        // approaches: each light still answers for the movements off its own.
        let map = crossroads_controlled(TrafficHandedness::RightHand, |builder, roads| {
            let north = builder
                .add_traffic_light(&LaneRef::new(roads[0].clone(), 0), LaneEnd::End, 5.0)
                .unwrap();
            let east = builder
                .add_traffic_light(&LaneRef::new(roads[1].clone(), 0), LaneEnd::End, 5.0)
                .unwrap();
            builder.add_traffic_light_rule(
                vec![north, east],
                None,
                vec![
                    LaneRef::new(roads[0].clone(), 0),
                    LaneRef::new(roads[1].clone(), 0),
                ],
            );
        });
        let network = to_plain_xml(&map).unwrap();
        let connections = elements(network.traffic_lights.as_deref().unwrap(), "connection");
        let off = |edge: &str| -> BTreeSet<String> {
            connections
                .iter()
                .filter(|connection| connection["from"] == edge)
                .map(|connection| format!("tls:j_x/{}", connection["linkIndex"]))
                .collect()
        };
        let lights: Vec<&ObjectId> = map
            .objects
            .iter()
            .filter(|object| object.kind.is_traffic_light())
            .map(|object| &object.id)
            .collect();
        let traced = |light: &ObjectId| -> BTreeSet<String> {
            network
                .trace
                .links_of(&IrRef::Object(light.clone()))
                .filter(|link| link.role.as_deref() == Some("link"))
                .map(|link| link.local.clone())
                .collect()
        };
        assert_eq!(lights.len(), 2);
        assert_eq!(traced(lights[0]), off("north.fwd"));
        assert_eq!(traced(lights[1]), off("east.fwd"));
        assert!(!off("north.fwd").is_empty() && !off("east.fwd").is_empty());
    }

    #[test]
    fn a_light_whose_own_lane_reaches_no_movement_is_traced_by_its_rule() {
        // A light standing on the far side of the junction, on a lane that leaves it,
        // governs the approach only through its rule.
        let map = crossroads_controlled(TrafficHandedness::RightHand, |builder, roads| {
            let light = builder
                .add_traffic_light(&LaneRef::new(roads[2].clone(), 1), LaneEnd::Start, 5.0)
                .unwrap();
            builder.add_traffic_light_rule(
                vec![light],
                None,
                vec![LaneRef::new(roads[0].clone(), 0)],
            );
        });
        let network = to_plain_xml(&map).unwrap();
        let light = map
            .objects
            .iter()
            .find(|object| object.kind.is_traffic_light())
            .unwrap();
        let traced: BTreeSet<String> = network
            .trace
            .links_of(&IrRef::Object(light.id.clone()))
            .filter(|link| link.role.as_deref() == Some("link"))
            .map(|link| link.local.clone())
            .collect();
        let expected: BTreeSet<String> =
            elements(network.traffic_lights.as_deref().unwrap(), "connection")
                .iter()
                .filter(|connection| connection["from"] == "north.fwd")
                .map(|connection| format!("tls:j_x/{}", connection["linkIndex"]))
                .collect();
        assert!(!expected.is_empty());
        assert_eq!(traced, expected);
    }

    #[test]
    fn a_light_answers_for_the_approaches_its_rule_gives_it_that_no_light_stands_over() {
        // One light on the northern approach, its rule over the eastern one too.
        let map = crossroads_controlled(TrafficHandedness::RightHand, |builder, roads| {
            let north = builder
                .add_traffic_light(&LaneRef::new(roads[0].clone(), 0), LaneEnd::End, 5.0)
                .unwrap();
            builder.add_traffic_light_rule(
                vec![north],
                None,
                vec![
                    LaneRef::new(roads[0].clone(), 0),
                    LaneRef::new(roads[1].clone(), 0),
                ],
            );
        });
        let network = to_plain_xml(&map).unwrap();
        let light = map
            .objects
            .iter()
            .find(|object| object.kind.is_traffic_light())
            .unwrap();
        let traced: BTreeSet<String> = network
            .trace
            .links_of(&IrRef::Object(light.id.clone()))
            .filter(|link| link.role.as_deref() == Some("link"))
            .map(|link| link.local.clone())
            .collect();
        let expected: BTreeSet<String> =
            elements(network.traffic_lights.as_deref().unwrap(), "connection")
                .iter()
                .filter(|connection| {
                    ["north.fwd", "east.fwd"].contains(&connection["from"].as_str())
                })
                .map(|connection| format!("tls:j_x/{}", connection["linkIndex"]))
                .collect();
        assert_eq!(expected.len(), 6);
        assert_eq!(traced, expected);
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
