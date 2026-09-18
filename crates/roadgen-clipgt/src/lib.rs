//! `roadgen-clipgt` — writes the canonical IR as a ClipGT clip.
//!
//! ClipGT is the scene format NVIDIA's Cosmos world-scenario tooling reads: a
//! directory of Parquet files named `{clip_id}.{layer}.parquet`, one row per map
//! element, each row a nested struct. There is no published schema; the field names
//! used here are the ones the public `clipgt_loader.py` reads, and nothing from that
//! file is reproduced — only the names two programs must agree on to interoperate.
//!
//! Like the other exporters this is a thin lowering layer: it takes a
//! [`ValidatedMap`] and pushes nothing back into the IR.
//!
//! # Coordinates
//!
//! ClipGT is **FLU** — x forward, y left, z up. That is the same shape of frame as
//! the IR's east-north-up: right-handed with z upwards. So the IR's coordinates are
//! written through unchanged, heights and all. Nothing here projects to the
//! horizontal plane; a rail's vertices carry the elevation and the superelevation the
//! generator computed for them, because a lane on a graded, banked road is not a flat
//! ribbon and writing it as one would be a different map.
//!
//! # What the format cannot hold
//!
//! ClipGT has no topology. A lane is two rails; there is no successor, no
//! predecessor, and no junction movement, so a map written here and read back is a
//! picture of the roads rather than a network you can route on. Element identifiers
//! are the Parquet row index, so the IR's stable ids do not survive either.
//! [`check`] says so rather than letting a caller assume otherwise.

pub mod columns;
pub mod ego;
pub mod error;
pub mod layers;
pub mod scenario;
mod table;

use std::path::Path;

use roadgen_core::map::Map;
use roadgen_core::{LaneId, ValidatedMap};

pub use ego::Pose;
pub use error::ExportError;
pub use layers::Layer;
// The route is the IR's own: `Route::From` names a lane of the map and the
// successors are the map's, so both exporters that drive a map say it the same way.
pub use roadgen_core::Route;
pub use scenario::{PolynomialType, Sensor};

/// How a map is turned into a clip.
#[derive(Debug, Clone, PartialEq)]
pub struct ClipConfig {
    /// Names every file in the directory, so it has to be one path component.
    pub clip_id: String,
    /// Frames per second of the egomotion track.
    pub frame_rate: f64,
    /// How fast the ego vehicle travels, metres per second.
    pub speed: f64,
    /// Where the ego vehicle drives. When `None`, a start is found and followed.
    pub route: Option<Route>,
    /// The rig. An empty one is a clip with no cameras, which loads but cannot be
    /// rendered from.
    pub sensors: Vec<Sensor>,
}

impl Default for ClipConfig {
    fn default() -> Self {
        ClipConfig {
            clip_id: "clip".into(),
            frame_rate: 30.0,
            speed: 10.0,
            route: None,
            sensors: Vec::new(),
        }
    }
}

impl ClipConfig {
    pub fn new(clip_id: impl Into<String>) -> Self {
        ClipConfig {
            clip_id: clip_id.into(),
            ..ClipConfig::default()
        }
    }

    pub fn with_frame_rate(mut self, frame_rate: f64) -> Self {
        self.frame_rate = frame_rate;
        self
    }

    pub fn with_speed(mut self, speed: f64) -> Self {
        self.speed = speed;
        self
    }

    /// Drives exactly these lanes, in this order.
    pub fn with_route(mut self, route: Vec<LaneId>) -> Self {
        self.route = Some(Route::Lanes(route));
        self
    }

    /// Sets off at `start` and follows successors from there.
    pub fn starting_at(mut self, start: LaneId) -> Self {
        self.route = Some(Route::From(start));
        self
    }

    pub fn with_sensors(mut self, sensors: Vec<Sensor>) -> Self {
        self.sensors = sensors;
        self
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

    /// The clip id a map exports under when the caller names none: its own name,
    /// reduced to something that can sit in a file name.
    pub fn for_map(map: &Map) -> Self {
        let name = map.metadata.name.as_deref().unwrap_or("clip");
        let cleaned: String = name
            .chars()
            .map(|character| {
                if character.is_ascii_alphanumeric() || character == '-' || character == '_' {
                    character
                } else {
                    '_'
                }
            })
            .collect();
        ClipConfig::new(if cleaned.is_empty() {
            "clip".to_owned()
        } else {
            cleaned
        })
    }
}

/// Renders `map` as the clip's tables, without writing anything.
///
/// The other exporters each hand back the document they built — an `OpenDrive`, a
/// `LaneletMap`, a string of XML — and this is ClipGT's: one `RecordBatch` per layer,
/// in the order they are written. A caller that wants to look at a clip rather than
/// keep it does not have to go through the file system to do it.
pub fn to_layers(map: &ValidatedMap, config: &ClipConfig) -> Result<Vec<Layer>, ExportError> {
    layers::all(map, config)
}

/// Writes `map` into `directory` as a ClipGT clip, creating the directory if needed.
///
/// Returns the clip id the files were named with.
pub fn write(
    map: &ValidatedMap,
    directory: impl AsRef<Path>,
    config: &ClipConfig,
) -> Result<String, ExportError> {
    let clip = validate_clip_id(&config.clip_id)?;
    let directory = directory.as_ref();
    std::fs::create_dir_all(directory)
        .map_err(|error| ExportError::Io(format!("{}: {error}", directory.display())))?;

    for layer in to_layers(map, config)? {
        let path = directory.join(format!("{clip}.{}.parquet", layer.name));
        table::write(&path, &layer.batch)?;
    }
    write_camera_timestamps(map, directory, clip, config)?;
    Ok(clip.to_owned())
}

/// Writes `{clip_id}.{camera}.json` for each configured camera: the frames it saw, as
/// the timestamps of the ego poses.
///
/// A reader uses this to line poses up with frames instead of resampling to a nominal
/// rate. Here the two are the same thing — the track was generated at the clip's own
/// frame rate — so the file says so rather than leaving the reader to assume it.
fn write_camera_timestamps(
    map: &ValidatedMap,
    directory: &Path,
    clip: &str,
    config: &ClipConfig,
) -> Result<(), ExportError> {
    if config.sensors.is_empty() {
        return Ok(());
    }
    let frames: Vec<serde_json::Value> = layers::poses(map, config)?
        .iter()
        .map(|pose| serde_json::json!({ "timestamp": pose.timestamp_micros }))
        .collect();
    let text = serde_json::Value::Array(frames).to_string();
    for sensor in &config.sensors {
        let path = directory.join(format!("{clip}.{}.json", sensor.canonical_name()));
        std::fs::write(&path, &text)
            .map_err(|error| ExportError::Io(format!("{}: {error}", path.display())))?;
    }
    Ok(())
}

/// What this map loses on the way into ClipGT, and what the scenario leaves out.
///
/// Most of these are properties of the format rather than of the map, so a perfectly
/// clean map still reports the topology it is about to shed; that is the point. Pass
/// the configuration to have the route and the rig checked as well.
pub fn check(map: &ValidatedMap, config: Option<&ClipConfig>) -> Vec<String> {
    let mut problems = Vec::new();
    if !map.buildings.is_empty() {
        problems.push(format!(
            "ClipGT's layers describe the road surface and its markings, so the map's \
             {} buildings are not written",
            map.buildings.len()
        ));
    }
    if !map.connections.is_empty() {
        problems.push(format!(
            "ClipGT holds no lane topology, so the map's {} lane connections are not \
             written; only the rails are",
            map.connections.len()
        ));
    }
    if !map.junctions.is_empty() {
        problems.push(format!(
            "the {} junctions become intersection areas — an outline each, with no \
             movements through them",
            map.junctions.len()
        ));
    }
    let signs = map
        .objects
        .iter()
        .filter(|object| {
            matches!(
                object.kind,
                roadgen_core::semantics::MapObjectKind::TrafficSign { .. }
            )
        })
        .count();
    if signs > 0 {
        problems.push(format!(
            "ClipGT recognises only STOP, YIELD and SPEED_LIMIT sign categories; the \
             {signs} signs' own catalogue codes go through verbatim and anything else \
             reads back as UNKNOWN"
        ));
    }
    if map.roads.iter().all(|road| road.is_connector()) && !map.roads.is_empty() {
        problems.push(
            "every road is a junction connector, so there is nowhere for the ego \
             vehicle to start"
                .into(),
        );
    }

    if let Some(config) = config {
        if config.sensors.is_empty() {
            problems.push(
                "the scenario configures no sensors, so the clip carries a rig with no \
                 cameras: it will load, but there is nothing to render from"
                    .into(),
            );
        }
        for lane in route_lanes(config) {
            if map.lane(lane).is_none() {
                problems.push(format!(
                    "the scenario's route names {lane}, which is not a lane of this map"
                ));
            }
        }
    }
    problems
}

/// The lanes a configured route names, whether it lists them or starts at one.
fn route_lanes(config: &ClipConfig) -> &[LaneId] {
    match &config.route {
        Some(Route::Lanes(lanes)) => lanes,
        Some(Route::From(start)) => std::slice::from_ref(start),
        None => &[],
    }
}

/// A clip id has to be a single path component, because it is the prefix of every
/// file name in the directory.
fn validate_clip_id(id: &str) -> Result<&str, ExportError> {
    let usable = !id.is_empty()
        && id.chars().all(|character| {
            character.is_ascii_alphanumeric() || character == '-' || character == '_'
        });
    if usable {
        Ok(id)
    } else {
        Err(ExportError::InvalidClipId(id.to_owned()))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use roadgen_core::prelude::*;

    #[test]
    fn a_clip_id_that_would_escape_the_directory_is_refused() {
        for id in ["", "../escape", "a/b", "with space", "dotted.name"] {
            assert!(validate_clip_id(id).is_err(), "{id:?}");
        }
        assert!(validate_clip_id("clip-01_x").is_ok());
    }

    #[test]
    fn a_maps_name_becomes_a_usable_clip_id() {
        let mut builder = MapBuilder::new(MapMetadata {
            name: Some("Tokyo / bay route".into()),
            ..MapMetadata::default()
        });
        builder
            .add_road(
                RoadSpec::line(
                    Point3::ORIGIN,
                    Point3::new(50.0, 0.0, 0.0),
                    vec![LaneSpec::new(
                        PositiveWidth::new(3.5).unwrap(),
                        Direction::Forward,
                    )],
                )
                .unwrap(),
            )
            .unwrap();
        let map = builder.finish().unwrap().validate().unwrap();
        let config = ClipConfig::for_map(&map);
        assert_eq!(config.clip_id, "Tokyo___bay_route");
        assert!(validate_clip_id(&config.clip_id).is_ok());
    }
}
