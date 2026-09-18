//! `roadgen-gpudrive` — writes the canonical IR as a GPUDrive scene.
//!
//! GPUDrive is a GPU-accelerated driving simulator that loads scenes from JSON: one
//! file per scene, holding a map as polylines and the agents driving it as logged
//! tracks. There is no published schema. What there is is the simulator's reader,
//! `src/json_serialization.hpp`, and the data model in [`scene`] is shaped by it field
//! for field — every key it insists on, every key it will do without, and the strings
//! it compares types against. Nothing from that file is reproduced here; only the
//! names two programs have to agree on to exchange data.
//!
//! Like the other exporters this is a thin lowering layer: it takes a [`ValidatedMap`]
//! and pushes nothing back into the IR.
//!
//! # Coordinates
//!
//! **GPUDrive is a plane.** A position is an `(x, y)` in metres and there is no z
//! anywhere in the document, so the elevation, the grade and the superelevation the
//! generator computed are dropped at the boundary. Distances along an agent's route
//! are measured in plan view for the same reason: an agent asked for 10 m/s should
//! cover ten metres of ground a second, not ten metres of a climbing road.
//!
//! The x and y written are the IR's own metres about the map origin. The simulator
//! re-centres a scene on the mean of everything in it when it loads, so no offset
//! needs applying here.
//!
//! # What a scene is
//!
//! A map and the agents driving it. The map is a set of polylines — lane centrelines,
//! painted lines, the edges of the drivable surface, crossings, stop signs — with no
//! widths and no topology: a lane does not say what it leads to. The agents are
//! tracks: a position, a heading and a velocity at each of the scene's timesteps, plus
//! the goal the track ended at, which is what the simulator scores a policy against.
//!
//! A generated map has no logged traffic, so [`objects`] drives agents along it: a
//! route followed lane by lane at a constant speed. Which routes, at what speed and in
//! what vehicle is a [`SceneConfig`], and a [`scenario`] file is that configuration
//! written down.

pub mod error;
pub mod objects;
pub mod roads;
pub mod scenario;
pub mod scene;

use std::path::Path;

use roadgen_core::map::Map;
use roadgen_core::semantics::{MapObjectKind, TrafficRule};
use roadgen_core::{LaneId, ValidatedMap};

pub use error::ExportError;
// The route is the IR's own: it names lanes of the map and follows the map's
// successors, so it is said the same way here as in every other exporter that drives
// a generated map.
pub use roadgen_core::Route;
pub use scene::{
    MapElement, Metadata, Object, ObjectKind, Road, RoadKind, Scene, TrackToPredict, Vector2,
};

/// One agent of the scene.
#[derive(Debug, Clone, PartialEq)]
pub struct Agent {
    pub kind: ObjectKind,
    /// Where it drives. When `None`, a start is found and followed.
    pub route: Option<Route>,
    /// How fast it travels, metres per second. Zero is an agent that stands still.
    pub speed: f64,
    /// Metres, along the agent's own forward axis.
    pub length: f64,
    pub width: f64,
    pub height: f64,
    /// Makes the simulator replay the track rather than hand the agent to a policy.
    pub mark_as_expert: bool,
    /// Lists the agent in the scene's `tracks_to_predict`.
    pub track_to_predict: bool,
    /// How hard that prediction is held to be. Only read when `track_to_predict` is
    /// set, and only meaningful against a dataset that defines a scale.
    pub difficulty: i32,
    /// Lists the agent in the scene's `objects_of_interest`.
    pub of_interest: bool,
}

impl Default for Agent {
    fn default() -> Self {
        Agent::new(ObjectKind::Vehicle)
    }
}

impl Agent {
    /// An agent of `kind` at that kind's ordinary size, driving a route that will be
    /// found for it.
    pub fn new(kind: ObjectKind) -> Self {
        let (length, width, height) = objects::default_size(kind);
        Agent {
            kind,
            route: None,
            speed: 10.0,
            length,
            width,
            height,
            mark_as_expert: false,
            track_to_predict: true,
            difficulty: 0,
            of_interest: false,
        }
    }

    pub fn with_speed(mut self, speed: f64) -> Self {
        self.speed = speed;
        self
    }
}

/// How a map is turned into a scene.
#[derive(Debug, Clone, PartialEq)]
pub struct SceneConfig {
    /// The scene's name. The reader keeps 31 characters of it.
    pub name: String,
    /// The scene's identifier, kept the same way.
    pub scenario_id: String,
    /// Seconds between timesteps. GPUDrive's datasets are 0.1.
    pub time_step: f64,
    /// How many timesteps the scene runs for. The simulator reads at most 91, which
    /// is its episode length.
    pub steps: usize,
    /// The agents, in the order they are written. The first is the scene's
    /// self-driving car.
    pub agents: Vec<Agent>,
}

impl Default for SceneConfig {
    fn default() -> Self {
        SceneConfig {
            name: "roadgen".into(),
            scenario_id: "roadgen".into(),
            time_step: scene::TIME_STEP,
            steps: scene::MAX_POSITIONS,
            agents: vec![Agent::default()],
        }
    }
}

impl SceneConfig {
    pub fn new(name: impl Into<String>) -> Self {
        let name = name.into();
        SceneConfig {
            scenario_id: name.clone(),
            name,
            ..SceneConfig::default()
        }
    }

    /// The configuration a map exports under when the caller names none: the map's own
    /// name, or `roadgen` for a map that has none either.
    pub fn for_map(map: &Map) -> Self {
        match map.metadata.name.as_deref() {
            Some(name) if !name.is_empty() => SceneConfig::new(name),
            _ => SceneConfig::default(),
        }
    }

    pub fn with_steps(mut self, steps: usize) -> Self {
        self.steps = steps;
        self
    }

    pub fn with_time_step(mut self, time_step: f64) -> Self {
        self.time_step = time_step;
        self
    }

    pub fn with_agents(mut self, agents: Vec<Agent>) -> Self {
        self.agents = agents;
        self
    }

    /// The scene's own vehicle, to nudge: the first agent, because that is the one
    /// `sdc_track_index` points at. A configuration whose agents were all taken away
    /// gets one back, since a caller asking for the ego vehicle means to have one.
    pub fn sdc_mut(&mut self) -> &mut Agent {
        if self.agents.is_empty() {
            self.agents.push(Agent::default());
        }
        &mut self.agents[0]
    }

    /// Reads a scenario file over this configuration: anything the file does not
    /// mention keeps the value it already has.
    pub fn with_scenario_file(self, path: impl AsRef<Path>) -> Result<Self, ExportError> {
        scenario::from_yaml_file(path, self)
    }

    /// The same, from YAML text already in hand.
    pub fn with_scenario_str(self, text: &str) -> Result<Self, ExportError> {
        scenario::from_yaml_str(text, self)
    }
}

/// Builds the scene `map` and `config` describe.
pub fn to_scene(map: &ValidatedMap, config: &SceneConfig) -> Result<Scene, ExportError> {
    let mut agents = Vec::with_capacity(config.agents.len());
    for (index, agent) in config.agents.iter().enumerate() {
        // Ids count from one so that a reader which treats zero as "no object" — and
        // the padding the simulator fills unused slots with — cannot be confused for
        // the first agent.
        let id = index as u32 + 1;
        agents.push(objects::track(
            map,
            agent,
            id,
            config.steps,
            config.time_step,
        )?);
    }

    let tracks_to_predict = config
        .agents
        .iter()
        .enumerate()
        .filter(|(_, agent)| agent.track_to_predict)
        .map(|(index, agent)| TrackToPredict {
            track_index: index as i32,
            difficulty: agent.difficulty,
        })
        .collect();
    let objects_of_interest = config
        .agents
        .iter()
        .zip(&agents)
        .filter(|(agent, _)| agent.of_interest)
        .map(|(_, object)| object.id)
        .collect();

    Ok(Scene {
        name: scene::truncate_name(&config.name),
        scenario_id: scene::truncate_name(&config.scenario_id),
        metadata: Metadata {
            // The first agent is the scene's own vehicle; a scene with no agents has
            // none, which is what -1 says.
            sdc_track_index: if agents.is_empty() { -1 } else { 0 },
            tracks_to_predict,
            objects_of_interest,
        },
        objects: agents,
        roads: roads::all(map)?,
    })
}

/// Renders `map` as a GPUDrive scene document.
pub fn to_json(map: &ValidatedMap, config: &SceneConfig) -> Result<String, ExportError> {
    let scene = to_scene(map, config)?;
    serde_json::to_string(&scene).map_err(|error| ExportError::Json(error.to_string()))
}

/// Writes `map` to `path` as a GPUDrive scene.
///
/// Rendered straight into the file rather than into a string first: a dense map's
/// scene is megabytes of JSON, and there is no reason to hold all of it in memory to
/// hand it to the kernel a moment later.
pub fn write(
    map: &ValidatedMap,
    path: impl AsRef<Path>,
    config: &SceneConfig,
) -> Result<(), ExportError> {
    let path = path.as_ref();
    let scene = to_scene(map, config)?;
    let io = |error: std::io::Error| ExportError::Io(format!("{}: {error}", path.display()));
    let file = std::fs::File::create(path).map_err(io)?;
    let mut writer = std::io::BufWriter::new(file);
    serde_json::to_writer(&mut writer, &scene)
        .map_err(|error| ExportError::Json(error.to_string()))?;
    std::io::Write::flush(&mut writer).map_err(io)
}

/// What this map loses on the way into GPUDrive, and what the scene runs up against.
///
/// Most of these are properties of the format rather than faults in the map — a
/// perfectly good map still sheds its heights and its topology — which is the point.
/// Pass the configuration to have the agents, the routes and the simulator's own
/// limits checked as well.
pub fn check(map: &ValidatedMap, config: Option<&SceneConfig>) -> Vec<String> {
    let mut problems = vec![
        "GPUDrive is a plane: positions are (x, y) metres and the document has no z, \
         so the elevation, grade and superelevation of the map are dropped"
            .to_owned(),
        "a GPUDrive lane is a centreline, so lane widths and the boundaries' \
         relationship to them are not written; the painted lines and the edges of the \
         drivable surface go out as polylines of their own"
            .to_owned(),
    ];

    if !map.connections.is_empty() {
        problems.push(format!(
            "a GPUDrive scene holds no lane topology, so the map's {} lane connections \
             are not written; an agent's route is baked into its track instead",
            map.connections.len()
        ));
    }
    problems.push(
        "road element ids are the row index in the written scene, so the map's own \
         identifiers do not survive the crossing"
            .to_owned(),
    );

    let lights = roads::objects_of(map, |kind| matches!(kind, MapObjectKind::TrafficLight)).count();
    if lights > 0 {
        problems.push(format!(
            "the scene format has no traffic-light element, so the {lights} lights and \
             the phases they govern are not written"
        ));
    }
    let stop_lines = roads::objects_of(map, |kind| matches!(kind, MapObjectKind::StopLine)).count();
    if stop_lines > 0 {
        problems.push(format!(
            "there is no stop-line element either: the {stop_lines} stop lines are \
             dropped, and a junction that stops traffic says so through the stop signs \
             beside it"
        ));
    }
    let other_signs = roads::objects_of(map, |kind| match kind {
        MapObjectKind::TrafficSign { code } => !roads::is_stop_sign(code),
        _ => false,
    })
    .count();
    if other_signs > 0 {
        problems.push(format!(
            "GPUDrive's map vocabulary has one sign in it — the stop sign — so the \
             {other_signs} signs of other kinds are not written"
        ));
    }

    let limited = map.roads.iter().any(|road| road.speed_limit.is_some())
        || map.lanes.iter().any(|lane| lane.speed_limit.is_some())
        || map
            .rules
            .iter()
            .any(|rule| matches!(rule, TrafficRule::SpeedLimit { .. }));
    if limited {
        problems.push(
            "a scene carries no speed limits: what an agent may do is the simulator's \
             dynamics and its policy, not the map's"
                .to_owned(),
        );
    }

    if let Some(config) = config {
        problems.extend(check_config(map, config));
    }
    problems
}

/// The half of the check that needs the configuration: the agents, their routes, and
/// the fixed sizes the reader's buffers impose.
fn check_config(map: &ValidatedMap, config: &SceneConfig) -> Vec<String> {
    let mut problems = Vec::new();
    if config.agents.is_empty() {
        problems.push(
            "the scenario configures no agents, so the scene is a map with nothing in \
             it: it loads, but there is nothing to control and no goal to score"
                .to_owned(),
        );
    }
    if config.name.chars().count() > scene::MAX_NAME {
        problems.push(format!(
            "the scene name is {} characters and the reader keeps {}, so it is written \
             truncated",
            config.name.chars().count(),
            scene::MAX_NAME
        ));
    }
    if config.scenario_id.chars().count() > scene::MAX_NAME {
        problems.push(format!(
            "the scenario id is {} characters and the reader keeps {}, so it is \
             written truncated",
            config.scenario_id.chars().count(),
            scene::MAX_NAME
        ));
    }
    for route in config
        .agents
        .iter()
        .filter_map(|agent| agent.route.as_ref())
    {
        let named: &[LaneId] = match route {
            Route::Lanes(lanes) => lanes,
            Route::From(start) => std::slice::from_ref(start),
        };
        for lane in named.iter().filter(|lane| map.lane(lane).is_none()) {
            problems.push(format!(
                "an agent's route names {lane}, which is not a lane of this map"
            ));
        }
    }

    // The reader's fixed buffers are properties of the scene rather than of the
    // configuration, so they are the scene's own to report.
    match to_scene(map, config) {
        Ok(scene) => problems.extend(scene.over_limits()),
        Err(error) => problems.push(format!("this scenario cannot be exported: {error}")),
    }
    problems
}

#[cfg(test)]
mod tests {
    use super::*;
    use roadgen_core::prelude::*;

    fn straight_map() -> ValidatedMap {
        let mut builder = MapBuilder::new(MapMetadata {
            name: Some("straight".into()),
            ..MapMetadata::default()
        });
        builder
            .add_road(
                RoadSpec::line(
                    Point3::new(0.0, 0.0, 0.0),
                    Point3::new(100.0, 0.0, 0.0),
                    vec![
                        LaneSpec::new(PositiveWidth::new(3.5).unwrap(), Direction::Forward),
                        LaneSpec::new(PositiveWidth::new(3.5).unwrap(), Direction::Backward),
                    ],
                )
                .unwrap()
                .with_name("main"),
            )
            .unwrap();
        builder.finish().unwrap().validate().unwrap()
    }

    #[test]
    fn a_maps_name_becomes_the_scenes() {
        let config = SceneConfig::for_map(&straight_map());
        assert_eq!(config.name, "straight");
        assert_eq!(config.scenario_id, "straight");
    }

    #[test]
    fn a_name_longer_than_the_readers_buffer_is_truncated_and_reported() {
        let map = straight_map();
        let config = SceneConfig::new("x".repeat(40));
        let scene = to_scene(&map, &config).unwrap();
        assert_eq!(scene.name.chars().count(), scene::MAX_NAME);
        assert!(check(&map, Some(&config))
            .iter()
            .any(|problem| problem.contains("scene name is 40 characters")));
    }

    #[test]
    fn the_first_agent_is_the_scenes_own_vehicle() {
        let scene = to_scene(&straight_map(), &SceneConfig::default()).unwrap();
        assert_eq!(scene.metadata.sdc_track_index, 0);
        assert_eq!(scene.objects.len(), 1);
        assert_eq!(scene.objects[0].id, 1);
        assert_eq!(
            scene.metadata.tracks_to_predict,
            vec![TrackToPredict {
                track_index: 0,
                difficulty: 0
            }]
        );
    }

    #[test]
    fn a_scene_with_no_agents_says_it_has_no_vehicle() {
        let map = straight_map();
        let config = SceneConfig::default().with_agents(Vec::new());
        let scene = to_scene(&map, &config).unwrap();
        assert_eq!(scene.metadata.sdc_track_index, -1);
        assert!(scene.objects.is_empty());
        assert!(check(&map, Some(&config))
            .iter()
            .any(|problem| problem.contains("no agents")));
    }
}
