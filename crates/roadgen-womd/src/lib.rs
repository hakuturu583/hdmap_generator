//! `roadgen-womd` — writes the canonical IR as the map of a Waymo Open Motion Dataset
//! scenario.
//!
//! WOMD is distributed as TFRecord files of `Scenario` protocol buffers: one scenario
//! is a map, the signal states on it at every timestep, and the agents' logged tracks.
//! This crate writes the first two. The schema is Waymo's own `scenario.proto` and
//! `map.proto`, compiled at build time from `proto/` — the files are Apache-2.0 and
//! carried unchanged but for two lidar and camera fields a map has no use for — so a
//! scenario written here is read by the dataset's own tooling, by Waymax, and by
//! anything else that reads WOMD, with no conversion in between.
//!
//! Like the other exporters this is a thin lowering layer: it takes a
//! [`ValidatedMap`] and pushes nothing back into the IR.
//!
//! # Coordinates
//!
//! A WOMD point is `(x, y, z)` in metres about "an arbitrary location", and the IR's
//! own metres about the map origin are written as they are. Unlike GPUDrive, the z is
//! kept: the elevation of every lane, line and edge reaches the file.
//!
//! # What a scenario carries
//!
//! - **Map features**, built in [`features`]: every drivable lane as its centreline
//!   in travel order, with the lanes it is entered from and exits to, its
//!   same-direction neighbours, and the line or edge on each side; the painted lines
//!   and the edges of the drivable surface; crossings; stop signs, with the lanes
//!   they control.
//! - **Dynamic map states**: one per timestep, each holding the lanes a traffic
//!   light governs with the point they stop at. The IR has no signal timing, so every
//!   state is `LANE_STATE_UNKNOWN`.
//! - **No tracks.** A generated map has no logged traffic; [`check`] says so.

pub mod error;
pub mod features;
pub mod tfrecord;

use std::path::{Path, PathBuf};

use prost::Message;
use roadgen_core::map::Map;
use roadgen_core::semantics::{MapObjectKind, TrafficRule};
use roadgen_core::{Trace, ValidatedMap};

pub use error::ExportError;

/// The generated message types: Waymo's `waymo.open_dataset` package.
pub mod pb {
    include!(concat!(env!("OUT_DIR"), "/waymo.open_dataset.rs"));
}

pub use pb::Scenario;

/// The format name a WOMD [`Trace`] is recorded under.
pub const TRACE_FORMAT: &str = "womd";

/// Seconds between timesteps in WOMD.
pub const TIME_STEP: f64 = 0.1;
/// Timesteps in a WOMD training scenario: one second of history, the current step
/// and eight seconds of future.
pub const STEPS: usize = 91;
/// The step a WOMD training scenario calls "now".
pub const CURRENT_TIME_INDEX: usize = 10;

/// How a map is turned into a scenario.
#[derive(Debug, Clone, PartialEq)]
pub struct ScenarioConfig {
    pub scenario_id: String,
    /// Seconds between timesteps.
    pub time_step: f64,
    /// How many timesteps the scenario has: the length of `timestamps_seconds` and
    /// of `dynamic_map_states`.
    pub steps: usize,
    /// The step that is "now"; the ones before it are history.
    pub current_time_index: usize,
}

impl Default for ScenarioConfig {
    fn default() -> Self {
        ScenarioConfig {
            scenario_id: "roadgen".into(),
            time_step: TIME_STEP,
            steps: STEPS,
            current_time_index: CURRENT_TIME_INDEX,
        }
    }
}

impl ScenarioConfig {
    pub fn new(scenario_id: impl Into<String>) -> Self {
        ScenarioConfig {
            scenario_id: scenario_id.into(),
            ..ScenarioConfig::default()
        }
    }

    /// The configuration a map exports under when the caller names none: the map's
    /// own name, or `roadgen` for a map that has none either.
    pub fn for_map(map: &Map) -> Self {
        match map.metadata.name.as_deref() {
            Some(name) if !name.is_empty() => ScenarioConfig::new(name),
            _ => ScenarioConfig::default(),
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

    pub fn with_current_time_index(mut self, index: usize) -> Self {
        self.current_time_index = index;
        self
    }

    fn validate(&self) -> Result<(), ExportError> {
        if self.steps == 0 {
            return Err(ExportError::Config(
                "a scenario needs one timestep at least".into(),
            ));
        }
        if !(self.time_step.is_finite() && self.time_step > 0.0) {
            return Err(ExportError::Config(format!(
                "the time step is {} s, and has to be a positive number of seconds",
                self.time_step
            )));
        }
        if self.current_time_index >= self.steps {
            return Err(ExportError::Config(format!(
                "the current time index is {} but the scenario has {} timesteps",
                self.current_time_index, self.steps
            )));
        }
        Ok(())
    }
}

/// Builds the scenario `map` and `config` describe.
pub fn to_scenario(map: &ValidatedMap, config: &ScenarioConfig) -> Result<Scenario, ExportError> {
    to_scenario_traced(map, config).map(|(scenario, _)| scenario)
}

/// [`to_scenario`], with the [`Trace`] of which IR elements each written element
/// came from.
///
/// A map feature is `feature:<id>`, with the id the scenario gives it. A lane, a
/// crossing and a stop sign are each one IR element; a line or edge two lanes share
/// is `merged` from both. A signal's lane state is `signal:<lane id>`, a `part` of
/// the traffic-light rule it was written from. Nothing is written, so the trace
/// names no files.
pub fn to_scenario_traced(
    map: &ValidatedMap,
    config: &ScenarioConfig,
) -> Result<(Scenario, Trace), ExportError> {
    config.validate()?;
    let mut trace = Trace::new(TRACE_FORMAT);
    let written = features::all_traced(map, &mut trace)?;
    let lane_states = features::signal_states(map, &written, &mut trace);
    let scenario = Scenario {
        scenario_id: Some(config.scenario_id.clone()),
        timestamps_seconds: (0..config.steps)
            .map(|step| step as f64 * config.time_step)
            .collect(),
        current_time_index: Some(config.current_time_index as i32),
        tracks: Vec::new(),
        dynamic_map_states: vec![
            pb::DynamicMapState {
                lane_states: lane_states.clone()
            };
            config.steps
        ],
        map_features: written.features,
        // No tracks, so there is no self-driving car to point at and nothing to
        // predict.
        sdc_track_index: None,
        objects_of_interest: Vec::new(),
        tracks_to_predict: Vec::new(),
    };
    Ok((scenario, trace))
}

/// The scenario as one serialized `Scenario` message.
pub fn to_bytes(map: &ValidatedMap, config: &ScenarioConfig) -> Result<Vec<u8>, ExportError> {
    to_bytes_traced(map, config).map(|(bytes, _)| bytes)
}

/// [`to_bytes`], with the [`Trace`] of the scenario it encodes.
pub fn to_bytes_traced(
    map: &ValidatedMap,
    config: &ScenarioConfig,
) -> Result<(Vec<u8>, Trace), ExportError> {
    let (scenario, trace) = to_scenario_traced(map, config)?;
    Ok((scenario.encode_to_vec(), trace))
}

/// Writes `map` to `path` as a TFRecord file holding the one scenario, the way WOMD
/// ships its own.
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
    let (bytes, mut trace) = to_bytes_traced(map, config)?;
    std::fs::write(path, tfrecord::record(&bytes))
        .map_err(|error| ExportError::Io(format!("{}: {error}", path.display())))?;
    trace.files = vec![PathBuf::from(path)];
    Ok(trace)
}

/// Every scenario in a TFRecord file, the way a consumer reads one.
pub fn read(path: impl AsRef<Path>) -> Result<Vec<Scenario>, ExportError> {
    let path = path.as_ref();
    let bytes = std::fs::read(path)
        .map_err(|error| ExportError::Io(format!("{}: {error}", path.display())))?;
    from_tfrecord(&bytes)
}

/// Every scenario in TFRecord bytes already in hand.
pub fn from_tfrecord(bytes: &[u8]) -> Result<Vec<Scenario>, ExportError> {
    tfrecord::records(bytes)?
        .iter()
        .map(|record| from_bytes(record))
        .collect()
}

/// One serialized `Scenario`.
pub fn from_bytes(bytes: &[u8]) -> Result<Scenario, ExportError> {
    Scenario::decode(bytes).map_err(|error| ExportError::Decode(error.to_string()))
}

/// What this map loses on the way into a WOMD scenario.
///
/// Most of these are properties of the format rather than faults in the map, which
/// is the point. Pass the configuration to have it checked as well.
pub fn check(map: &ValidatedMap, config: Option<&ScenarioConfig>) -> Vec<String> {
    let mut problems = vec![
        "a WOMD scenario written from a map has no tracks: there is no logged traffic, \
         so there is no self-driving car, nothing to predict, and a consumer that \
         expects agents has none"
            .to_owned(),
        "a WOMD lane is a centreline, so lane widths are not written; the painted lines \
         and the edges of the drivable surface go out as features of their own, which \
         each lane names as its boundaries"
            .to_owned(),
        "map feature ids are numbered as they are written, so the map's own identifiers \
         do not survive the crossing"
            .to_owned(),
    ];

    if !map.tin_surfaces.is_empty() || !map.land_covers.is_empty() {
        problems.push(format!(
            "a WOMD map has no terrain and no land cover, so the map's {} TIN surfaces \
             and {} land covers are not written",
            map.tin_surfaces.len(),
            map.land_covers.len()
        ));
    }
    if !map.buildings.is_empty() {
        problems.push(format!(
            "a WOMD map is roads, so the map's {} buildings ({} parts) are not written",
            map.buildings.len(),
            map.building_parts.len()
        ));
    }
    if !map.structures.is_empty() {
        problems.push(format!(
            "a WOMD map has no structures, so the map's {} bridges and tunnels are not \
             written; the roads they carry keep their heights",
            map.structures.len()
        ));
    }
    let non_drivable = map
        .lanes
        .iter()
        .filter(|lane| !lane.lane_type.is_drivable())
        .count();
    if non_drivable > 0 {
        problems.push(format!(
            "WOMD's lanes are lanes a vehicle or a bicycle travels, so the map's \
             {non_drivable} footways, verges and other lanes nobody drives are not \
             written"
        ));
    }

    let lights = features::objects_of(map, |kind| kind.is_traffic_light()).count();
    let ruled = map
        .rules
        .iter()
        .any(|rule| matches!(rule, TrafficRule::TrafficLight { .. }));
    if lights > 0 || ruled {
        problems.push(format!(
            "WOMD holds a traffic light as the state of the lanes it governs rather \
             than as a thing: the {lights} light heads are not written, and as the map \
             has no signal timing every lane state is LANE_STATE_UNKNOWN"
        ));
    }
    let stop_lines =
        features::objects_of(map, |kind| matches!(kind, MapObjectKind::StopLine)).count();
    if stop_lines > 0 {
        problems.push(format!(
            "there is no stop-line feature: the {stop_lines} stop lines are dropped, \
             and a signalled lane stops at the start of its way through the junction"
        ));
    }
    let other_signs = features::objects_of(map, |kind| match kind {
        MapObjectKind::TrafficSign { code } => !features::is_stop_sign(code),
        _ => false,
    })
    .count();
    if other_signs > 0 {
        problems.push(format!(
            "WOMD's map has one sign in it — the stop sign — so the {other_signs} signs \
             of other kinds are not written"
        ));
    }
    let yields = map
        .rules
        .iter()
        .filter(|rule| matches!(rule, TrafficRule::RightOfWay { .. }))
        .count();
    if yields > 0 {
        problems.push(format!(
            "WOMD has no right-of-way relation, so the map's {yields} right-of-way \
             rules are not written"
        ));
    }

    if let Some(config) = config {
        if let Err(error) = config.validate() {
            problems.push(error.to_string());
        }
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
    fn a_maps_name_becomes_the_scenario_id() {
        assert_eq!(
            ScenarioConfig::for_map(&straight_map()).scenario_id,
            "straight"
        );
    }

    #[test]
    fn the_timeline_is_womds_unless_told_otherwise() {
        let scenario = to_scenario(&straight_map(), &ScenarioConfig::default()).unwrap();
        assert_eq!(scenario.timestamps_seconds.len(), STEPS);
        assert_eq!(scenario.dynamic_map_states.len(), STEPS);
        assert_eq!(scenario.current_time_index, Some(10));
        assert!((scenario.timestamps_seconds[90] - 9.0).abs() < 1e-9);
    }

    #[test]
    fn a_current_step_past_the_end_is_refused() {
        let config = ScenarioConfig::default()
            .with_steps(5)
            .with_current_time_index(5);
        assert!(to_scenario(&straight_map(), &config).is_err());
        assert!(check(&straight_map(), Some(&config))
            .iter()
            .any(|problem| problem.contains("current time index")));
    }

    #[test]
    fn the_height_reaches_the_file() {
        let scenario = to_scenario(&straight_map(), &ScenarioConfig::default()).unwrap();
        let Some(pb::map_feature::FeatureData::Lane(lane)) = &scenario.map_features[0].feature_data
        else {
            panic!("the first feature is a lane");
        };
        let z: Vec<f64> = lane.polyline.iter().map(|p| p.z.unwrap()).collect();
        assert!(z.iter().any(|z| *z > 1.0), "{z:?}");
    }

    #[test]
    fn a_written_file_reads_back_as_the_scenario() {
        let map = straight_map();
        let config = ScenarioConfig::for_map(&map);
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("map.tfrecord");
        let trace = write_traced(&map, &path, &config).unwrap();
        assert_eq!(trace.files, vec![path.clone()]);
        assert_eq!(
            read(&path).unwrap(),
            vec![to_scenario(&map, &config).unwrap()]
        );
    }
}
