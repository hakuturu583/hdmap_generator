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
mod layers;
mod table;

use std::path::Path;

use roadgen_core::map::Map;
use roadgen_core::{LaneId, ValidatedMap};

pub use ego::Pose;
pub use error::ExportError;

/// How a map is turned into a clip.
#[derive(Debug, Clone, PartialEq)]
pub struct ClipConfig {
    /// Names every file in the directory, so it has to be one path component.
    pub clip_id: String,
    /// Frames per second of the egomotion track.
    pub frame_rate: f64,
    /// How fast the ego vehicle travels, metres per second.
    pub speed: f64,
    /// Lanes to drive, in order. Found automatically when `None`.
    pub route: Option<Vec<LaneId>>,
}

impl Default for ClipConfig {
    fn default() -> Self {
        ClipConfig {
            clip_id: "clip".into(),
            frame_rate: 30.0,
            speed: 10.0,
            route: None,
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

    pub fn with_route(mut self, route: Vec<LaneId>) -> Self {
        self.route = Some(route);
        self
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

    for layer in layers::all(map, config)? {
        let path = directory.join(format!("{clip}.{}.parquet", layer.name));
        table::write(&path, &layer.batch)?;
    }
    Ok(clip.to_owned())
}

/// What this map loses on the way into ClipGT.
///
/// Every one of these is a property of the format rather than of the map, so a clean
/// map still reports the topology it is about to shed; that is the point.
pub fn check(map: &ValidatedMap) -> Vec<String> {
    let mut problems = Vec::new();
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
    problems
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
