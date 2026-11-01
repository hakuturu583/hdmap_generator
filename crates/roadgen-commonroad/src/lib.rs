//! `roadgen-commonroad` — writes the canonical IR as a
//! [CommonRoad](https://commonroad.in.tum.de/) scenario.
//!
//! A CommonRoad scenario is one XML document, of the 2020a schema that
//! `commonroad-io` ships and reads: a lanelet network, the traffic signs and lights
//! on it, its intersections, and the planning problems posed on it — where a
//! motion planner starts and where it has to get to. This crate writes all four,
//! so a scenario written here is opened by `commonroad-io`'s `CommonRoadFileReader`
//! and handed to a planner, a route planner or the CommonRoad drivability checker
//! with no conversion in between.
//!
//! Like the other exporters this is a thin lowering layer: it takes a
//! [`ValidatedMap`] and pushes nothing back into the IR.
//!
//! # Coordinates
//!
//! A CommonRoad point is metres in a plane, and the IR's own metres about the map
//! origin are written as they are; the origin's latitude and longitude go in the
//! scenario's `location`, with the PROJ string those metres are in — the OpenDRIVE
//! export's `<geoReference>` — as its `geoReference`. The schema allows a `z` but CommonRoad's tools work in
//! the plane, so heights are left out unless [`ScenarioConfig::elevation`] asks
//! for them.
//!
//! # What a scenario carries
//!
//! - **Lanelets**, built in [`network`]: one per driving lane, bicycle lane and
//!   footway, with its bounds in travel order and their paint, its predecessors
//!   and successors, the lanelets beside it, what it is and who uses it; and one
//!   per crossing.
//! - **Traffic signs**: the IR's own signs CommonRoad's catalogue has, a virtual
//!   speed-limit sign per distinct limit and a virtual give-way sign per
//!   right-of-way rule, each referred to by the lanelets it governs, with the stop
//!   line a stop or give-way puts across them.
//! - **Traffic lights**, referred to by the lanes that stop for them. The IR has no
//!   signal timing, so each light's cycle is one `inactive` phase unless
//!   [`ScenarioConfig::light_cycle`] gives them one to run, crossing directions
//!   half a cycle apart.
//! - **Intersections**: one per at-grade junction, with its approaches and the
//!   movements each one makes.
//! - **A planning problem**, which the schema requires: from rest at the start of
//!   the first lane a vehicle can set off from to the last lane of the route the
//!   map's own policy takes from there ([`Map::route_from`]).
//!
//!
//! # The map alone
//!
//! The XML is a scenario. [`protobuf`] writes the map alone, as a CommonRoad 2024
//! `CommonRoadMap` protocol buffer, built from the same network.
//!
//! [`Map::route_from`]: roadgen_core::map::Map::route_from

pub mod error;
pub mod model;
mod network;
pub mod protobuf;
mod signs;
mod xml;

use std::path::{Path, PathBuf};
use std::time::{SystemTime, UNIX_EPOCH};

use roadgen_core::map::Map;
use roadgen_core::semantics::{MapObjectKind, TrafficRule};
use roadgen_core::{LaneType, Trace, TrafficHandedness, ValidatedMap};

pub use error::ExportError;
pub use model::{Scenario, COMMONROAD_VERSION};

/// The format name a CommonRoad [`Trace`] is recorded under.
pub const TRACE_FORMAT: &str = "commonroad";

/// Seconds per time step: CommonRoad's own scenarios use a tenth of a second.
pub const TIME_STEP: f64 = 0.1;

/// The country a scenario is set in: Zamunda, CommonRoad's own for scenarios that
/// are nowhere real, whose traffic signs are Germany's.
pub const COUNTRY: &str = "ZAM";

/// What a traffic light shows during one phase of its cycle.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum LightPhase {
    Red,
    /// Red and amber together, before green, as German lights show it.
    RedYellow,
    Green,
    Yellow,
    /// Dark, or flashing: the light governs nothing.
    Inactive,
}

impl LightPhase {
    /// The phase's name in the schema, which is also what [`LightPhase::parse`]
    /// reads.
    pub fn as_str(self) -> &'static str {
        match self {
            LightPhase::Red => "red",
            LightPhase::RedYellow => "redYellow",
            LightPhase::Green => "green",
            LightPhase::Yellow => "yellow",
            LightPhase::Inactive => "inactive",
        }
    }

    pub fn parse(value: &str) -> Option<LightPhase> {
        Some(
            match value
                .to_ascii_lowercase()
                .replace(['_', '-', ' '], "")
                .as_str()
            {
                "red" => LightPhase::Red,
                "redyellow" | "redamber" => LightPhase::RedYellow,
                "green" => LightPhase::Green,
                "yellow" | "amber" => LightPhase::Yellow,
                "inactive" | "off" => LightPhase::Inactive,
                _ => return None,
            },
        )
    }
}

/// The cycle a light runs when the caller asks for the default one: the Scenario
/// Designer's own, 60, 10, 60 and 10 steps of 0.1 s, in seconds.
///
/// Red and red-and-amber together last as long as green and amber, so that two
/// groups of lights half a cycle apart take turns and are never green together.
pub const DEFAULT_LIGHT_CYCLE: [(LightPhase, f64); 4] = [
    (LightPhase::Red, 6.0),
    (LightPhase::RedYellow, 1.0),
    (LightPhase::Green, 6.0),
    (LightPhase::Yellow, 1.0),
];

/// How long a cycle runs before its first green — red and red-and-amber, in a
/// German cycle — in whatever unit its phases are given. The lights across a
/// junction's first approach run this far behind, which is half the cycle when the
/// cycle is laid out for two directions to take turns.
pub(crate) fn before_green<T: Copy + std::iter::Sum<T>>(cycle: &[(LightPhase, T)]) -> T {
    cycle
        .iter()
        .take_while(|(phase, _)| *phase != LightPhase::Green)
        .map(|&(_, duration)| duration)
        .sum()
}

/// How a map is turned into a scenario.
#[derive(Debug, Clone, PartialEq)]
pub struct ScenarioConfig {
    /// CommonRoad's scenario id: `ZAM_<map>-1_1_T-1` by default.
    pub benchmark_id: String,
    pub author: String,
    pub affiliation: String,
    pub source: String,
    /// `YYYY-MM-DD`; today, in UTC, when `None`.
    pub date: Option<String>,
    /// Seconds per time step.
    pub time_step: f64,
    /// Whether to pose a planning problem. The schema requires one, so a scenario
    /// written without is a map that `commonroad-io` reads but a validator refuses.
    pub planning_problem: bool,
    /// The last time step the planning problem's goal may be reached at; enough to
    /// drive the route at 5 m/s when `None`.
    pub goal_steps: Option<u64>,
    /// Whether points carry their `z`.
    pub elevation: bool,
    /// The cycle every traffic light runs, as phases and how many seconds each
    /// lasts; `None` writes each light dark, as one `inactive` phase, since the map
    /// holds no timing. At each junction the lights on the approaches across the
    /// first ones run half a cycle behind them — by the time from the cycle's
    /// start to its first green — so that crossing directions take turns.
    pub light_cycle: Option<Vec<(LightPhase, f64)>>,
}

impl Default for ScenarioConfig {
    fn default() -> Self {
        ScenarioConfig {
            benchmark_id: benchmark_id(""),
            author: "roadgen".into(),
            affiliation: String::new(),
            source: "roadgen".into(),
            date: None,
            time_step: TIME_STEP,
            planning_problem: true,
            goal_steps: None,
            elevation: false,
            light_cycle: None,
        }
    }
}

impl ScenarioConfig {
    /// The configuration a map exports under when the caller names none: a
    /// scenario id made from the map's own name.
    pub fn for_map(map: &Map) -> Self {
        ScenarioConfig {
            benchmark_id: benchmark_id(map.metadata.name.as_deref().unwrap_or_default()),
            ..ScenarioConfig::default()
        }
    }

    pub fn with_benchmark_id(mut self, id: impl Into<String>) -> Self {
        self.benchmark_id = id.into();
        self
    }

    pub fn with_date(mut self, date: impl Into<String>) -> Self {
        self.date = Some(date.into());
        self
    }

    pub fn with_time_step(mut self, time_step: f64) -> Self {
        self.time_step = time_step;
        self
    }

    pub fn with_planning_problem(mut self, planning_problem: bool) -> Self {
        self.planning_problem = planning_problem;
        self
    }

    pub fn with_goal_steps(mut self, steps: u64) -> Self {
        self.goal_steps = Some(steps);
        self
    }

    pub fn with_elevation(mut self, elevation: bool) -> Self {
        self.elevation = elevation;
        self
    }

    pub fn with_light_cycle(mut self, cycle: impl IntoIterator<Item = (LightPhase, f64)>) -> Self {
        self.light_cycle = Some(cycle.into_iter().collect());
        self
    }

    /// Every light runs [`DEFAULT_LIGHT_CYCLE`].
    pub fn with_default_light_cycle(self) -> Self {
        self.with_light_cycle(DEFAULT_LIGHT_CYCLE)
    }

    /// The light cycle in time steps, each phase at least one long.
    fn light_steps(&self) -> Result<Option<Vec<(LightPhase, u64)>>, ExportError> {
        let Some(cycle) = &self.light_cycle else {
            return Ok(None);
        };
        if cycle.is_empty() {
            return Err(ExportError::Config(
                "a light cycle needs one phase at least".into(),
            ));
        }
        cycle
            .iter()
            .map(|&(phase, seconds)| {
                let steps = (seconds / self.time_step).round();
                if !(seconds.is_finite() && steps >= 1.0) {
                    return Err(ExportError::Config(format!(
                        "the {} phase lasts {seconds} s, and has to last one time step \
                         ({} s) at least",
                        phase.as_str(),
                        self.time_step
                    )));
                }
                Ok((phase, steps as u64))
            })
            .collect::<Result<Vec<_>, _>>()
            .map(Some)
    }

    /// Refuses a configuration CommonRoad cannot describe, and hands back the light
    /// cycle in time steps, which checking it has worked out.
    fn validate(&self) -> Result<Option<Vec<(LightPhase, u64)>>, ExportError> {
        if !(self.time_step.is_finite() && self.time_step > 0.0) {
            return Err(ExportError::Config(format!(
                "the time step is {} s, and has to be a positive number of seconds",
                self.time_step
            )));
        }
        if self.goal_steps == Some(0) {
            return Err(ExportError::Config(
                "the goal has to be reachable after one time step at least".into(),
            ));
        }
        let light_steps = self.light_steps()?;
        if self.benchmark_id.is_empty() {
            return Err(ExportError::Config("the benchmark id is empty".into()));
        }
        if let Some(date) = &self.date {
            if !is_date(date) {
                return Err(ExportError::Config(format!(
                    "the date is {date:?}, and has to be written YYYY-MM-DD"
                )));
            }
        }
        Ok(light_steps)
    }
}

/// The scenario id a map named `name` is written under: `ZAM_<Name>-1_1_T-1`, the
/// name's letters and digits with each word capitalised, or `Roadgen`.
///
/// CommonRoad's map name is letters and digits only, so anything else in the name
/// is dropped; `T` says the scenario's other traffic is given as trajectories,
/// which, there being none, it trivially is.
pub fn benchmark_id(name: &str) -> String {
    let mut map_name = String::new();
    for word in name.split(|c: char| !c.is_ascii_alphanumeric()) {
        let mut chars = word.chars();
        if let Some(first) = chars.next() {
            map_name.push(first.to_ascii_uppercase());
            map_name.extend(chars);
        }
    }
    if map_name.is_empty() {
        map_name = "Roadgen".into();
    }
    format!("{COUNTRY}_{map_name}-1_1_T-1")
}

/// Builds the scenario `map` and `config` describe.
pub fn to_scenario(map: &ValidatedMap, config: &ScenarioConfig) -> Result<Scenario, ExportError> {
    to_scenario_traced(map, config).map(|(scenario, _)| scenario)
}

/// [`to_scenario`], with the [`Trace`] of which IR elements each written element
/// came from.
///
/// A lane is `lanelet:<id>`, a crossing a `lanelet:<id>` too (role `crosswalk`), a
/// sign `traffic_sign:<id>` and a light `traffic_light:<id>`. A speed-limit sign is
/// linked from the rules and lanes it holds the limit of, a give-way sign from its
/// rule; a traffic-light rule is a `part` of each light it names. A junction is
/// `intersection:<id>` and a `part` of each `incoming:<id>`; the planning problem
/// is a `part` of the lanes it starts and ends on. Nothing is written, so the
/// trace names no files.
pub fn to_scenario_traced(
    map: &ValidatedMap,
    config: &ScenarioConfig,
) -> Result<(Scenario, Trace), ExportError> {
    let light_cycle = config.validate()?;
    let mut trace = Trace::new(TRACE_FORMAT);
    let planning = network::Planning {
        enabled: config.planning_problem,
        time_step: config.time_step,
        goal_steps: config.goal_steps,
        light_cycle,
    };
    let network = network::build(map, &planning, &mut trace)?;
    let origin = map.metadata.origin;
    let scenario = Scenario {
        benchmark_id: config.benchmark_id.clone(),
        date: config.date.clone().unwrap_or_else(today),
        author: config.author.clone(),
        affiliation: config.affiliation.clone(),
        source: config.source.clone(),
        time_step: config.time_step,
        location: model::Location {
            geo_name_id: -999,
            latitude: origin.latitude(),
            longitude: origin.longitude(),
            geo_reference: roadgen_opendrive::geo_reference(map)
                .map_err(|error| ExportError::Projection(error.to_string()))?,
        },
        tags: network.tags,
        lanelets: network.lanelets,
        traffic_signs: network.traffic_signs,
        traffic_lights: network.traffic_lights,
        intersections: network.intersections,
        planning_problems: network.planning_problems,
        elevation: config.elevation,
    };
    Ok((scenario, trace))
}

/// The scenario as a CommonRoad XML document.
pub fn to_xml(map: &ValidatedMap, config: &ScenarioConfig) -> Result<String, ExportError> {
    to_xml_traced(map, config).map(|(xml, _)| xml)
}

/// [`to_xml`], with the [`Trace`] of the scenario it renders.
pub fn to_xml_traced(
    map: &ValidatedMap,
    config: &ScenarioConfig,
) -> Result<(String, Trace), ExportError> {
    let (scenario, trace) = to_scenario_traced(map, config)?;
    Ok((scenario.to_xml(), trace))
}

/// Writes `map` to `path` as a CommonRoad XML scenario.
pub fn write(
    map: &ValidatedMap,
    path: impl AsRef<Path>,
    config: &ScenarioConfig,
) -> Result<(), ExportError> {
    write_traced(map, path, config).map(|_| ())
}

/// [`write`], with the [`Trace`] of the scenario it wrote. The trace's one file is
/// `path`.
pub fn write_traced(
    map: &ValidatedMap,
    path: impl AsRef<Path>,
    config: &ScenarioConfig,
) -> Result<Trace, ExportError> {
    let path = path.as_ref();
    let (xml, mut trace) = to_xml_traced(map, config)?;
    std::fs::write(path, xml)
        .map_err(|error| ExportError::Io(format!("{}: {error}", path.display())))?;
    trace.files = vec![PathBuf::from(path)];
    Ok(trace)
}

/// What this map loses on the way into a CommonRoad scenario.
///
/// Most of these are properties of the format rather than faults in the map. Pass
/// the configuration to have it checked as well.
pub fn check(map: &ValidatedMap, config: Option<&ScenarioConfig>) -> Vec<String> {
    problems(map, config, Target::Scenario)
}

/// What this map loses on the way into a CommonRoad 2024 map ([`protobuf`]).
///
/// The same as [`check`] but for what only a scenario has — a planning problem,
/// light timing — and for the buildings, which a map holds as environment
/// obstacles.
pub fn check_map(map: &ValidatedMap, config: Option<&ScenarioConfig>) -> Vec<String> {
    problems(map, config, Target::Map)
}

/// Which CommonRoad file the problems are for.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Target {
    /// The 2020a XML scenario.
    Scenario,
    /// The 2024 protobuf map.
    Map,
}

fn problems(map: &ValidatedMap, config: Option<&ScenarioConfig>, target: Target) -> Vec<String> {
    let elevation = config.is_some_and(|config| config.elevation);
    let mut problems = vec![
        "CommonRoad ids are numbered as they are written, in one id space for \
         lanelets, signs, lights, intersections and the rest, so the map's own \
         identifiers do not survive the crossing"
            .to_owned(),
    ];
    if target == Target::Scenario {
        problems.push(
            "a CommonRoad scenario written from a map has no obstacles: there is no \
             other traffic, only the planning problem"
                .to_owned(),
        );
    }
    if !elevation {
        problems.push(
            "a CommonRoad lanelet lies in the plane, so heights are not written; ask for \
             elevation to have each point keep its z, which the schema allows and most \
             CommonRoad tools ignore"
                .to_owned(),
        );
    }

    if !map.tin_surfaces.is_empty() || !map.land_covers.is_empty() {
        problems.push(format!(
            "a CommonRoad scenario has no terrain and no land cover, so the map's {} TIN \
             surfaces and {} land covers are not written",
            map.tin_surfaces.len(),
            map.land_covers.len()
        ));
    }
    if !map.buildings.is_empty() {
        problems.push(match target {
            Target::Scenario => format!(
                "the map's {} buildings ({} parts) are not written: the 2020a scenario \
                 has no building, and the CommonRoad map does",
                map.buildings.len(),
                map.building_parts.len()
            ),
            Target::Map => format!(
                "the map's {} building parts are environment obstacles of their outline \
                 in plan: their heights and roofs are not written",
                map.building_parts.len()
            ),
        });
    }
    if !map.structures.is_empty() {
        problems.push(format!(
            "CommonRoad has no bridges or tunnels, so the map's {} structures are not \
             written{}",
            map.structures.len(),
            if elevation {
                ""
            } else {
                ", and in the plane a road over another crosses it: the lanelets overlap \
                 but are not connected"
            }
        ));
    }

    let dropped = map
        .lanes
        .iter()
        .filter(|lane| !network::is_written(lane.lane_type))
        .count();
    if dropped > 0 {
        problems.push(format!(
            "a lanelet is a lane somebody travels along, so the map's {dropped} \
             shoulders, borders, parking strips and other lanes nobody does are not \
             written"
        ));
    }

    let orientation = network::Orientation::of(map);
    let unlinked = map
        .connections
        .iter()
        .filter(|connection| {
            [&connection.from.lane, &connection.to.lane]
                .into_iter()
                .any(|lane| {
                    map.lane(lane)
                        .is_some_and(|lane| lane.lane_type == LaneType::Sidewalk)
                })
        })
        .filter(|connection| orientation.link(map, connection).is_none())
        .count();
    if unlinked > 0 {
        problems.push(format!(
            "{unlinked} links between footways join two of their ends that both lead \
             out or both lead in, which CommonRoad's successor relation cannot say, so \
             they are not written; the footways are still walked both ways"
        ));
    }

    let objects = |wanted: fn(&MapObjectKind) -> bool| {
        map.objects
            .iter()
            .filter(|object| wanted(&object.kind))
            .count()
    };
    if objects(MapObjectKind::is_traffic_light) > 0 && target == Target::Map {
        problems.push(
            "a CommonRoad map holds where a traffic light stands and which lanes stop for \
             it, not its timing, which belongs to a dynamic file: the lights are written \
             with no cycle, whatever light cycle is asked for"
                .to_owned(),
        );
    } else if objects(MapObjectKind::is_traffic_light) > 0 {
        match config.and_then(|config| config.light_cycle.as_ref()) {
            None => problems.push(
                "the map has no signal timing, so every traffic light's cycle is a \
                 single inactive phase — a planner sees no signal; ask for a light \
                 cycle to have them run one"
                    .to_owned(),
            ),
            Some(cycle) => {
                problems.push(
                    "the map has no signal timing, so every traffic light runs the cycle \
                     asked for, the lights across each junction's first approaches half \
                     a cycle behind; a junction with more than two directions of approach \
                     still has two groups"
                        .to_owned(),
                );
                let before = before_green(cycle);
                let total: f64 = cycle.iter().map(|(_, seconds)| seconds).sum();
                if !cycle.iter().any(|(phase, _)| *phase == LightPhase::Green) {
                    problems.push("the light cycle has no green phase".to_owned());
                } else if before * 2.0 < total - 1e-9 {
                    problems.push(format!(
                        "the light cycle reaches green {before} s into its {total} s: \
                         the two groups of lights at a junction take turns only when that \
                         is half the cycle, so crossing directions are green together"
                    ));
                }
            }
        }
    }
    let unsigned = map
        .objects
        .iter()
        .filter(|object| matches!(object.kind, MapObjectKind::TrafficSign { .. }))
        .filter(|object| network::writable_sign(map, object).is_none())
        .count();
    if unsigned > 0 {
        problems.push(format!(
            "{unsigned} traffic signs are not written: CommonRoad holds a sign by its \
             number in the German catalogue, and needs a lanelet to refer to it, and \
             these have a code it does not know or govern no written lane"
        ));
    }
    let barriers = objects(|kind| matches!(kind, MapObjectKind::Barrier { .. }));
    if barriers > 0 {
        problems.push(format!(
            "the map's {barriers} barriers are not written: a lanelet's bound is where \
             the road ends, whatever stands along it"
        ));
    }
    let yields = map
        .rules
        .iter()
        .filter(|rule| matches!(rule, TrafficRule::RightOfWay { .. }))
        .count();
    if yields > 0 {
        problems.push(format!(
            "CommonRoad has no right-of-way relation, so each of the map's {yields} \
             right-of-way rules is a virtual give-way sign on the lanes that yield; \
             which lanes they yield to is not written"
        ));
    }
    if map
        .rules
        .iter()
        .any(|rule| matches!(rule, TrafficRule::SpeedLimit { .. }))
        || map.lanes.iter().any(|lane| lane.speed_limit.is_some())
    {
        problems.push(
            "a speed limit is a virtual sign the lanelets it holds on refer to, and \
             CommonRoad carries a sign's limit on along the lanelets after it, so a \
             lane with no limit of its own takes the one before it"
                .to_owned(),
        );
    }
    let merges = map
        .junctions
        .iter()
        .filter(|junction| !network::is_intersection(map, &junction.id))
        .count();
    if merges > 0 {
        problems.push(format!(
            "the map's {merges} junctions of motorways and slip roads alone are merges \
             and diverges, not intersections, and are written as lanelets alone"
        ));
    }
    if map.metadata.handedness == TrafficHandedness::LeftHand {
        problems.push(format!(
            "the scenario is set in {COUNTRY}, where traffic keeps right, and a reader \
             puts a sign with no position on the right of the road; the map drives on \
             the left"
        ));
    }
    let drivable = map.lanes.iter().any(|lane| lane.lane_type.is_drivable());
    if !drivable && target == Target::Scenario {
        problems.push(
            "the map has no lane a vehicle drives, so there is no planning problem and \
             the scenario is not valid under the CommonRoad schema"
                .to_owned(),
        );
    }

    if let Some(config) = config {
        if let Err(error) = config.validate() {
            problems.push(error.to_string());
        }
        if !config.planning_problem && drivable && target == Target::Scenario {
            problems.push(
                "no planning problem is posed, which the CommonRoad schema requires: \
                 commonroad-io reads the scenario, but a validator refuses it"
                    .to_owned(),
            );
        }
        if !is_benchmark_id(&config.benchmark_id) {
            problems.push(format!(
                "the benchmark id {:?} is not of the form COUNTRY_MapName-1_1_T-1, so \
                 commonroad-io reads it as the name of the map and nothing else{}",
                config.benchmark_id,
                if target == Target::Map {
                    ", and the map is filed under ZAM, that name and map 1"
                } else {
                    ""
                }
            ));
        }
    }
    problems
}

/// Whether `id` reads as a CommonRoad scenario id: an optional `C-`, a three-letter
/// country, `_`, a map name of letters and digits, `-` and a map number, then
/// optionally `_` and a configuration number and `_`, a prediction type and its
/// numbers.
fn is_benchmark_id(id: &str) -> bool {
    let id = id.strip_prefix("C-").unwrap_or(id);
    let mut parts = id.split('_');
    let country = parts.next().unwrap_or_default();
    if country.len() != 3 || !country.chars().all(|c| c.is_ascii_uppercase()) {
        return false;
    }
    let Some((name, map_id)) = parts.next().and_then(|map| map.split_once('-')) else {
        return false;
    };
    let number = |text: &str| {
        !text.is_empty() && !text.starts_with('0') && text.chars().all(|c| c.is_ascii_digit())
    };
    if name.is_empty() || !name.chars().all(|c| c.is_ascii_alphanumeric()) || !number(map_id) {
        return false;
    }
    match parts.next() {
        None => return true,
        Some(configuration) if number(configuration) => {}
        Some(_) => return false,
    }
    match parts.next() {
        None => true,
        Some(prediction) => {
            let mut pieces = prediction.split('-');
            let kind = pieces.next().unwrap_or_default();
            let ids: Vec<&str> = pieces.collect();
            matches!(kind, "S" | "T" | "P" | "I")
                && !ids.is_empty()
                && ids.iter().all(|id| number(id))
                && parts.next().is_none()
        }
    }
}

fn is_date(date: &str) -> bool {
    let bytes = date.as_bytes();
    if !(bytes.len() == 10
        && bytes[4] == b'-'
        && bytes[7] == b'-'
        && bytes
            .iter()
            .enumerate()
            .all(|(index, byte)| index == 4 || index == 7 || byte.is_ascii_digit()))
    {
        return false;
    }
    let (Ok(year), Ok(month), Ok(day)) = (
        date[..4].parse::<u32>(),
        date[5..7].parse::<u32>(),
        date[8..10].parse::<u32>(),
    ) else {
        return false;
    };
    // A reader parses the date as a calendar day, so 31 February is no date.
    let leap = year % 4 == 0 && (year % 100 != 0 || year % 400 == 0);
    let days = match month {
        1 | 3 | 5 | 7 | 8 | 10 | 12 => 31,
        4 | 6 | 9 | 11 => 30,
        2 if leap => 29,
        2 => 28,
        _ => return false,
    };
    (1..=days).contains(&day)
}

/// Today's date in UTC, `YYYY-MM-DD`.
fn today() -> String {
    let days = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|elapsed| elapsed.as_secs() / 86_400)
        .unwrap_or_default() as i64;
    let (year, month, day) = civil_from_days(days);
    format!("{year:04}-{month:02}-{day:02}")
}

/// The calendar date `days` after 1970-01-01, in the proleptic Gregorian calendar
/// (Howard Hinnant's `civil_from_days`).
fn civil_from_days(days: i64) -> (i64, u32, u32) {
    let z = days + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z.rem_euclid(146_097);
    let yoe = (doe - doe / 1_460 + doe / 36_524 - doe / 146_096) / 365;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let day = (doy - (153 * mp + 2) / 5 + 1) as u32;
    let month = if mp < 10 { mp + 3 } else { mp - 9 } as u32;
    let year = yoe + era * 400 + i64::from(month <= 2);
    (year, month, day)
}

#[cfg(test)]
mod tests {
    use super::*;
    use roadgen_core::prelude::*;

    fn straight_map() -> ValidatedMap {
        let mut builder = MapBuilder::new(MapMetadata {
            name: Some("straight road".into()),
            ..MapMetadata::default()
        });
        builder
            .add_road(
                RoadSpec::line(
                    Point3::new(0.0, 0.0, 0.0),
                    Point3::new(100.0, 0.0, 5.0),
                    vec![
                        LaneSpec::new(PositiveWidth::new(3.5).unwrap(), Direction::Forward),
                        LaneSpec::new(PositiveWidth::new(3.5).unwrap(), Direction::Backward),
                    ],
                )
                .unwrap(),
            )
            .unwrap();
        builder.finish().unwrap().validate().unwrap()
    }

    #[test]
    fn a_maps_name_becomes_the_benchmark_id() {
        assert_eq!(
            ScenarioConfig::for_map(&straight_map()).benchmark_id,
            "ZAM_StraightRoad-1_1_T-1"
        );
        assert_eq!(benchmark_id(""), "ZAM_Roadgen-1_1_T-1");
        assert!(is_benchmark_id(&benchmark_id("town 1")));
        assert!(is_benchmark_id("DEU_Ffb-1"));
        assert!(!is_benchmark_id("town"));
        assert!(!is_benchmark_id("ZAM_Town-0_1_T-1"));
    }

    #[test]
    fn dates_are_counted_from_the_epoch() {
        assert_eq!(civil_from_days(0), (1970, 1, 1));
        assert_eq!(civil_from_days(20_758), (2026, 11, 1));
        assert!(is_date(&today()));
        assert!(!is_date("2026-13-01"));
        assert!(!is_date("2026-02-31"));
        assert!(is_date("2024-02-29"));
        assert!(!is_date("2100-02-29"));
    }

    #[test]
    fn two_lanes_are_two_opposite_neighbours() {
        let scenario = to_scenario(&straight_map(), &ScenarioConfig::default()).unwrap();
        let [a, b] = &scenario.lanelets[..] else {
            panic!("two lanelets");
        };
        assert_eq!(a.left.points.len(), a.right.points.len());
        // Each lanelet's left is the other one's left, run the other way.
        assert_eq!(
            a.adjacent_left.map(|n| (n.lanelet, n.same_direction)),
            Some((b.id, false))
        );
        assert_eq!(
            b.adjacent_left.map(|n| (n.lanelet, n.same_direction)),
            Some((a.id, false))
        );
        assert_eq!(a.left.points.first(), b.left.points.last());
    }

    #[test]
    fn the_planning_problem_starts_at_rest_on_the_first_lane() {
        let scenario = to_scenario(&straight_map(), &ScenarioConfig::default()).unwrap();
        let [problem] = &scenario.planning_problems[..] else {
            panic!("one planning problem");
        };
        assert_eq!(problem.initial_state.velocity, 0.0);
        assert!(problem.initial_state.position.x.abs() < 1e-9);
        // 100 m at 5 m/s in tenths of a second.
        assert_eq!(problem.goal_states[0].time, (0, 201));
        assert_eq!(problem.goal_states[0].lanelets, vec![1]);
    }

    #[test]
    fn the_height_is_written_only_when_asked_for() {
        let map = straight_map();
        let config = ScenarioConfig::default().with_date("2026-01-01");
        assert!(!to_xml(&map, &config).unwrap().contains("<z>"));
        assert!(to_xml(&map, &config.with_elevation(true))
            .unwrap()
            .contains("<z>"));
    }

    #[test]
    fn the_scenario_is_georeferenced_as_the_opendrive_export_is() {
        let map = straight_map();
        let xml = to_xml(&map, &ScenarioConfig::default()).unwrap();
        let reference = roadgen_opendrive::geo_reference(&map).unwrap();
        assert!(xml.contains(&format!("<geoReference>{reference}</geoReference>")));
    }

    #[test]
    fn a_bad_configuration_is_refused() {
        let map = straight_map();
        assert!(to_xml(&map, &ScenarioConfig::default().with_time_step(0.0)).is_err());
        assert!(to_xml(&map, &ScenarioConfig::default().with_date("1 May")).is_err());
        assert!(
            check(&map, Some(&ScenarioConfig::default().with_goal_steps(0)))
                .iter()
                .any(|problem| problem.contains("one time step"))
        );
    }

    #[test]
    fn a_written_file_is_the_document() {
        let map = straight_map();
        let config = ScenarioConfig::for_map(&map).with_date("2026-01-01");
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("map.xml");
        let trace = write_traced(&map, &path, &config).unwrap();
        assert_eq!(trace.files, vec![path.clone()]);
        assert_eq!(
            std::fs::read_to_string(&path).unwrap(),
            to_xml(&map, &config).unwrap()
        );
    }
}
