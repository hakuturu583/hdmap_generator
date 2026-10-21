//! `roadgen-rdshq` — writes the canonical IR as an RDS-HQ clip.
//!
//! RDS-HQ is the dataset layout NVIDIA's Cosmos-Drive-Dreams toolkit renders its
//! HD-map, LiDAR and world-scenario control videos from, and the one the
//! PhysicalAI-Autonomous-Vehicle-Cosmos-Drive-Dreams dataset is published in. A clip
//! is a directory of folders, each holding one `{clip_id}.tar` per clip:
//!
//! ```text
//! <root>/
//!   3d_lanes/<clip>.tar            3d_crosswalks/<clip>.tar
//!   3d_lanelines/<clip>.tar        3d_road_markings/<clip>.tar
//!   3d_road_boundaries/<clip>.tar  3d_traffic_signs/<clip>.tar
//!   3d_wait_lines/<clip>.tar       3d_traffic_lights/<clip>.tar
//!   3d_poles/<clip>.tar
//!   pose/<clip>.tar                ftheta_intrinsic/<clip>.tar
//!   vehicle_pose/<clip>.tar        pinhole_intrinsic/<clip>.tar
//!   timestamp/<clip>.tar           all_object_info/<clip>.tar
//! ```
//!
//! Each tar is a single WebDataset sample (see [`wds`]). There is no published
//! schema; the entry names and the shape of each document are the ones the public
//! toolkit's renderer reads and its Waymo converter writes, and nothing from either
//! is reproduced — only the names two programs must agree on to interoperate.
//! Several clips can share a root: each writes its own `{clip_id}.tar` in each
//! folder.
//!
//! Like the other exporters this is a thin lowering layer: it takes a
//! [`ValidatedMap`] and pushes nothing back into the IR. The clip is configured with
//! the same [`ClipConfig`], read from the same scenario file, as a ClipGT clip, and
//! the two formats share their ego track and their boxes for lights and signs, so a
//! map exported both ways is the same drive in both.
//!
//! # Coordinates
//!
//! Map layers are written in the IR's own frame, east-north-up, heights and all:
//! RDS-HQ's world frame is any right-handed frame with z up, which this is. Poses
//! take each frame's coordinates into that one; see [`scene`].
//!
//! # What the format cannot hold
//!
//! RDS-HQ is a picture of the road, like ClipGT: no successors, no junction
//! movements, no identifiers but a label's position in its list. [`check`] says so,
//! and says what the toolkit will make of the clip that a caller may not expect.

pub mod error;
pub mod labels;
pub mod scene;
pub mod wds;

use std::path::{Path, PathBuf};

use roadgen_core::{Trace, ValidatedMap};

pub use error::ExportError;
pub use labels::{MapLayer, TRACE_FORMAT};
pub use roadgen_clipgt::{ClipConfig, PolynomialType, Route, Sensor};
pub use wds::Sample;

/// The frame rate the toolkit reads every pose at, whatever the clip was made at.
pub const POSE_FRAME_RATE: f64 = 30.0;
/// How many frames the toolkit cuts each rendered video into; a shorter tail is
/// dropped.
pub const CHUNK_FRAMES: usize = 121;

/// One file of the clip: the folder it goes in, and the sample it holds.
///
/// A shard with no entries is a file the clip does not have — a pinhole for a
/// lens with no focal length. A tar with nothing in it is not a sample, and a
/// reader that opens one fails; a folder with no file for the clip is how the
/// toolkit already spells "not available".
#[derive(Debug, Clone, PartialEq)]
pub struct Shard {
    pub folder: String,
    pub sample: Sample,
}

/// Renders `map` as the clip's shards, without writing anything.
pub fn to_shards(map: &ValidatedMap, config: &ClipConfig) -> Result<Vec<Shard>, ExportError> {
    to_shards_traced(map, config).map(|(shards, _)| shards)
}

/// [`to_shards`], with the [`Trace`] of which IR element each label came from.
///
/// A label is named `<layer>:<index>` — `lanes:4`, `lanelines:0` — with the layer as
/// [`MapLayer::name`] gives it. The scene's shards describe the drive rather than the
/// map, so they leave nothing in the trace.
pub fn to_shards_traced(
    map: &ValidatedMap,
    config: &ClipConfig,
) -> Result<(Vec<Shard>, Trace), ExportError> {
    let clip = clip_id(config)?;
    let mut trace = Trace::new(TRACE_FORMAT);
    let mut shards = Vec::new();
    for layer in labels::all_traced(map, &mut trace)? {
        let mut sample = Sample::new(clip);
        let folder = format!("3d_{}", layer.name);
        let entry = format!("{}.json", layer.name);
        sample.push(entry, layer.into_document().to_string().into_bytes());
        shards.push(Shard { folder, sample });
    }
    shards.extend(scene::all(map, config, clip)?);
    Ok((shards, trace))
}

/// Writes `map` under `root` as an RDS-HQ clip, creating the folders as needed.
///
/// Returns the clip id the files were named with.
pub fn write(
    map: &ValidatedMap,
    root: impl AsRef<Path>,
    config: &ClipConfig,
) -> Result<String, ExportError> {
    write_traced(map, root, config).map(|(clip, _)| clip)
}

/// [`write`], with the [`Trace`] of which IR element each label came from. The
/// trace's files are the tars written, in the order they were written.
pub fn write_traced(
    map: &ValidatedMap,
    root: impl AsRef<Path>,
    config: &ClipConfig,
) -> Result<(String, Trace), ExportError> {
    let root = root.as_ref();
    let clip = config.clip_id.clone();
    let (shards, mut trace) = to_shards_traced(map, config)?;
    let mut files: Vec<PathBuf> = Vec::new();
    for shard in shards {
        let folder = root.join(&shard.folder);
        let path = folder.join(format!("{clip}.tar"));
        if shard.sample.entries.is_empty() {
            // An earlier export of the same clip may have had this file; left
            // behind, it would describe cameras this clip no longer has.
            match std::fs::remove_file(&path) {
                Ok(()) => {}
                Err(error) if error.kind() == std::io::ErrorKind::NotFound => {}
                Err(error) => return Err(ExportError::Io(format!("{}: {error}", path.display()))),
            }
            continue;
        }
        std::fs::create_dir_all(&folder)
            .map_err(|error| ExportError::Io(format!("{}: {error}", folder.display())))?;
        shard.sample.write(&path)?;
        files.push(path);
    }
    trace.files = files;
    Ok((clip, trace))
}

/// What this map loses on the way into RDS-HQ, and what the toolkit will make of
/// the clip.
///
/// Like ClipGT's, most of these are properties of the format, so a clean map still
/// reports the topology it is about to shed. Pass the configuration to have the
/// drive and the cameras checked against what the renderer expects.
pub fn check(map: &ValidatedMap, config: Option<&ClipConfig>) -> Vec<String> {
    let mut problems = Vec::new();
    if !map.tin_surfaces.is_empty() || !map.land_covers.is_empty() {
        problems.push(format!(
            "RDS-HQ's layers have no terrain and no land cover, so the map's {} TIN \
             surfaces and {} land covers are not written",
            map.tin_surfaces.len(),
            map.land_covers.len()
        ));
    }
    if !map.buildings.is_empty() {
        problems.push(format!(
            "RDS-HQ's layers describe the road surface and its furniture, so the map's \
             {} buildings ({} parts) are not written",
            map.buildings.len(),
            map.building_parts.len()
        ));
    }
    if !map.structures.is_empty() {
        problems.push(format!(
            "RDS-HQ's layers have no bridges or tunnels, so the map's {} structures are \
             not written; the lanes they carry keep their heights in their points",
            map.structures.len()
        ));
    }
    if !map.connections.is_empty() {
        problems.push(format!(
            "RDS-HQ holds no lane topology, so the map's {} lane connections are not \
             written; only the lanes' lines are",
            map.connections.len()
        ));
    }
    if !map.junctions.is_empty() {
        problems.push(format!(
            "RDS-HQ has no layer for junctions, so the map's {} junctions are not \
             written; the lanes through them are",
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
            "an RDS-HQ sign is a box with no type, so the {signs} signs' catalogue \
             codes are not written"
        ));
    }
    let lights = roadgen_clipgt::layers::device_frames(map, |kind| kind.is_traffic_light())
        .map(|frames| frames.len())
        .unwrap_or(0);
    for (layer, count) in [
        ("lanelines", labels::painted_edges(map).len()),
        ("traffic_lights", lights),
    ] {
        if count == 1 {
            problems.push(format!(
                "the map has exactly one of {layer}: the toolkit's world-scenario \
                 renderer reads a {layer} layer with one label as a placeholder for \
                 none, so it will not draw it"
            ));
        }
    }

    let Some(config) = config else {
        return problems;
    };
    if let Err(error) = clip_id(config) {
        problems.push(error.to_string());
    }
    if (config.frame_rate - POSE_FRAME_RATE).abs() > 1e-9 {
        problems.push(format!(
            "the clip is made at {} fps, and the toolkit reads every RDS-HQ pose as \
             {POSE_FRAME_RATE} fps, so its videos will play the drive at {:.2}× speed",
            config.frame_rate,
            POSE_FRAME_RATE / config.frame_rate
        ));
    }
    match roadgen_clipgt::layers::poses(map, config) {
        Ok(poses) if poses.len() < CHUNK_FRAMES => problems.push(format!(
            "the drive is {} frames long, and the toolkit renders {CHUNK_FRAMES}-frame \
             videos and drops a shorter tail, so it will render nothing: drive \
             further or more slowly",
            poses.len()
        )),
        Ok(_) => {}
        Err(error) => problems.push(error.to_string()),
    }
    let sensors = scene::sensors(config);
    if config.sensors.is_empty() {
        let camera = &sensors[0];
        let [ahead, _, up] = camera.position;
        problems.push(format!(
            "the scenario configures no sensors, so the clip is given one: {}, an \
             equidistant 120° lens {ahead} m ahead of the rig and {up} m up",
            camera.canonical_name()
        ));
    } else if !sensors
        .iter()
        .any(|sensor| sensor.name == scene::DEFAULT_CAMERA)
    {
        problems.push(format!(
            "no camera is named {}, which is the one camera the toolkit's single-view \
             dataset configuration renders",
            scene::DEFAULT_CAMERA
        ));
    }
    for sensor in &sensors {
        if let Err(error) = scene::ftheta_intrinsic(sensor) {
            problems.push(error.to_string());
        }
        if scene::pinhole_intrinsic(sensor).is_none() {
            problems.push(format!(
                "{}'s polynomial has no linear term, so it has no focal length to give \
                 a pinhole: it is written with f-theta intrinsics only",
                sensor.name
            ));
        }
        // The renderer resizes every frame to 1280 × 720 and refuses an f-theta
        // camera whose two axes would be scaled differently.
        let (ratio_w, ratio_h) = (1280.0 / sensor.width as f64, 720.0 / sensor.height as f64);
        if (ratio_w - ratio_h).abs() >= 0.02 {
            problems.push(format!(
                "{} is {} × {}, and the toolkit renders f-theta cameras only at 16:9",
                sensor.name, sensor.width, sensor.height
            ));
        }
    }
    problems
}

/// A clip id has to be a single path component with no dots: it is the file name in
/// every folder, and a WebDataset reader takes everything before a member's first
/// dot as the sample's key. ClipGT's rule is exactly that, so it is the one used.
fn clip_id(config: &ClipConfig) -> Result<&str, ExportError> {
    roadgen_clipgt::validate_clip_id(&config.clip_id)
        .map_err(|_| ExportError::InvalidClipId(config.clip_id.clone()))
}
