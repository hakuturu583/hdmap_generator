//! Exporting to RDS-HQ.
//!
//! Like ClipGT, RDS-HQ is lists of points with no topology to check against. What
//! these tests check is that every file the Cosmos-Drive-Dreams renderer opens is
//! there and holds one sample under the clip's key, that the map layers are the IR's
//! geometry in three dimensions, and that the poses are the drive a ClipGT clip of
//! the same map holds — read back out of the archives rather than the exporter.

use roadgen_core::prelude::*;
use roadgen_core::Vector3;
use roadgen_integration_tests::rdshq_read::{sample, vertices};
use roadgen_integration_tests::scenarios;
use roadgen_rdshq::ClipConfig;

/// Every folder the renderer opens for an HD-map video, and the ones beside them.
const FOLDERS: [&str; 15] = [
    "3d_lanes",
    "3d_lanelines",
    "3d_road_boundaries",
    "3d_wait_lines",
    "3d_poles",
    "3d_crosswalks",
    "3d_road_markings",
    "3d_traffic_signs",
    "3d_traffic_lights",
    "pose",
    "vehicle_pose",
    "timestamp",
    "all_object_info",
    "ftheta_intrinsic",
    "pinhole_intrinsic",
];

const CAMERA: &str = "camera_front_wide_120fov";

fn write(map: &ValidatedMap, config: &ClipConfig) -> (tempfile::TempDir, String) {
    let root = tempfile::tempdir().expect("a temporary directory");
    let clip = roadgen_rdshq::write(map, root.path(), config).expect("the map should export");
    (root, clip)
}

fn labels(root: &std::path::Path, clip: &str, layer: &str) -> Vec<serde_json::Value> {
    sample(root, &format!("3d_{layer}"), clip).json(&format!("{layer}.json"))["labels"]
        .as_array()
        .expect("a list of labels")
        .clone()
}

fn matrices(root: &std::path::Path, folder: &str, clip: &str, suffix: &str) -> Vec<[[f64; 4]; 4]> {
    let sample = sample(root, folder, clip);
    sample
        .ending_in(suffix)
        .map(|(name, bytes)| {
            let (shape, values) = roadgen_integration_tests::rdshq_read::npy(bytes);
            assert_eq!(shape, [4, 4], "{name}");
            let mut matrix = [[0.0; 4]; 4];
            for (i, value) in values.into_iter().enumerate() {
                matrix[i / 4][i % 4] = value;
            }
            matrix
        })
        .collect()
}

#[test]
fn a_clip_holds_every_file_the_renderer_opens() {
    let map = scenarios::controlled_crossroads();
    let (root, clip) = write(&map, &ClipConfig::new("crossroads"));
    assert_eq!(clip, "crossroads");
    for folder in FOLDERS {
        let read = sample(root.path(), folder, &clip);
        assert_eq!(read.key, clip, "{folder}");
        assert!(!read.entries.is_empty(), "{folder} is empty");
    }
    // Every map layer is a `{layer}.json` holding a list, empty or not.
    assert!(labels(root.path(), &clip, "poles").is_empty());
    assert!(labels(root.path(), &clip, "road_markings").is_empty());
}

#[test]
fn the_map_layers_are_the_irs_geometry() {
    let map = scenarios::graded_road();
    let (root, clip) = write(&map, &ClipConfig::new("graded"));
    let config = map.metadata.sampling;

    let lanes = labels(root.path(), &clip, "lanes");
    let drivable: Vec<_> = map
        .lanes
        .iter()
        .filter(|lane| lane.lane_type.is_drivable())
        .collect();
    assert_eq!(lanes.len(), drivable.len());
    for (label, lane) in lanes.iter().zip(&drivable) {
        let points = vertices(&label["labelData"]["shape3d"]["polyline3d"]);
        let travel = lane.travel_geometry(config).unwrap();
        let expected = travel.centerline.to_polyline(config).unwrap();
        assert_eq!(points.len(), expected.points().len());
        for (written, ir) in points.iter().zip(expected.points()) {
            assert!(written.distance_to(*ir) < 1e-9, "{written:?} vs {ir:?}");
        }
    }

    // A graded road climbs, and a layer that came out flat would have lost it.
    let heights: Vec<f64> = labels(root.path(), &clip, "road_boundaries")
        .iter()
        .flat_map(|label| vertices(&label["labelData"]["shape3d"]["polyline3d"]))
        .map(|point| point.z)
        .collect();
    let spread = heights.iter().cloned().fold(f64::MIN, f64::max)
        - heights.iter().cloned().fold(f64::MAX, f64::min);
    assert!(spread > 1.0, "the boundaries span {spread} m of height");
}

#[test]
fn a_lane_line_says_its_colour_and_style() {
    let map = scenarios::controlled_crossroads();
    let (root, clip) = write(&map, &ClipConfig::new("lines"));
    let lines = labels(root.path(), &clip, "lanelines");
    assert!(!lines.is_empty());
    for line in &lines {
        let attributes = line["labelData"]["shape3d"]["attributes"]
            .as_array()
            .expect("attributes");
        let value = |name: &str| -> String {
            attributes
                .iter()
                .find(|attribute| attribute["name"] == name)
                .and_then(|attribute| attribute["enumsList"]["enumsList"][0].as_str())
                .unwrap_or_else(|| panic!("no {name} in {attributes:?}"))
                .to_owned()
        };
        assert!(["WHITE", "YELLOW"].contains(&value("colors").as_str()));
        assert!(!value("styles").is_empty());
    }
}

#[test]
fn lights_and_signs_are_boxes_with_eight_corners() {
    let map = scenarios::controlled_crossroads();
    let (root, clip) = write(&map, &ClipConfig::new("devices"));

    let lights = labels(root.path(), &clip, "traffic_lights");
    assert_eq!(lights.len(), 1);
    let shape = &lights[0]["labelData"]["shape3d"];
    let corners = vertices(&shape["cuboid3d"]);
    assert_eq!(corners.len(), 8);
    // The box is the ClipGT one, centred where the IR hangs the light.
    let center = corners
        .iter()
        .fold(Vector3::new(0.0, 0.0, 0.0), |sum, corner| {
            sum + (*corner - Point3::ORIGIN)
        })
        * (1.0 / 8.0);
    assert!(
        center.z > 4.9,
        "a light hangs above the road, not at {center:?}"
    );
    let light = map
        .objects
        .iter()
        .find(|object| object.kind.is_traffic_light())
        .unwrap();
    assert_eq!(shape["attributes"][0]["name"], "feature_id");
    assert_eq!(shape["attributes"][0]["text"], light.id.as_str());

    // Opposite corners are the box's diagonal: 0.6 × 0.6 × 1.0.
    let diagonal = corners[0].distance_to(corners[6]);
    assert!((diagonal - (0.36_f64 + 0.36 + 1.0).sqrt()).abs() < 1e-9);

    assert_eq!(labels(root.path(), &clip, "wait_lines").len(), 1);
    let crossing = labels(root.path(), &clip, "crosswalks");
    assert_eq!(crossing.len(), 1);
    assert!(vertices(&crossing[0]["labelData"]["shape3d"]["surface"]).len() >= 4);
}

#[test]
fn the_poses_are_the_drive_a_clipgt_clip_holds() {
    let map = scenarios::controlled_crossroads();
    let config = ClipConfig::new("drive").with_speed(8.0);
    let (root, clip) = write(&map, &config);

    let rigs = matrices(root.path(), "vehicle_pose", &clip, ".vehicle_pose.npy");
    let cameras = matrices(root.path(), "pose", &clip, &format!(".pose.{CAMERA}.npy"));
    let track = roadgen_clipgt::layers::poses(&map, &config).unwrap();
    assert_eq!(rigs.len(), track.len());
    assert_eq!(cameras.len(), track.len());

    let timestamps = sample(root.path(), "timestamp", &clip);
    let objects = sample(root.path(), "all_object_info", &clip);
    assert_eq!(
        timestamps.ending_in(".timestamp_micros.txt").count(),
        track.len()
    );
    assert_eq!(
        objects.ending_in(".all_object_info.json").count(),
        track.len()
    );
    assert_eq!(
        std::str::from_utf8(&timestamps.entries["000001.timestamp_micros.txt"]).unwrap(),
        track[1].timestamp_micros.to_string()
    );

    for ((rig, camera), pose) in rigs.iter().zip(&cameras).zip(&track) {
        let column = |m: &[[f64; 4]; 4], j: usize| Vector3::new(m[0][j], m[1][j], m[2][j]);
        // The rig is where the track is, and its rotation is a rotation.
        assert!((column(rig, 3) - (pose.position - Point3::ORIGIN)).norm() < 1e-9);
        for j in 0..3 {
            assert!((column(rig, j).norm() - 1.0).abs() < 1e-9);
        }
        assert!((column(rig, 0).cross(column(rig, 1)) - column(rig, 2)).norm() < 1e-9);

        // The default camera looks the way the vehicle drives, with OpenCV's z, and
        // its y points down.
        assert!((column(camera, 2) - column(rig, 0)).norm() < 1e-9);
        assert!((column(camera, 1) + column(rig, 2)).norm() < 1e-9);
        let offset = column(camera, 3) - column(rig, 3);
        assert!((offset.dot(column(rig, 0)) - 1.7).abs() < 1e-9);
        assert!((offset.dot(column(rig, 2)) - 1.45).abs() < 1e-9);
    }
}

#[test]
fn the_intrinsics_are_the_cameras_lens() {
    let map = scenarios::controlled_crossroads();
    let (root, clip) = write(&map, &ClipConfig::new("lens"));
    let ftheta = sample(root.path(), "ftheta_intrinsic", &clip)
        .array(&format!("ftheta_intrinsic.{CAMERA}.npy"));
    assert_eq!(ftheta.0, [14]);
    assert_eq!(&ftheta.1[..4], &[960.0, 540.0, 1920.0, 1080.0]);
    // An equidistant 120° lens: the half-angle lands on the edge of the frame.
    assert!((ftheta.1[5] * 60.0_f64.to_radians() - 960.0).abs() < 1e-9);

    let pinhole = sample(root.path(), "pinhole_intrinsic", &clip)
        .array(&format!("pinhole_intrinsic.{CAMERA}.npy"));
    assert_eq!(pinhole.0, [6]);
    assert_eq!(
        pinhole.1[0], ftheta.1[5],
        "the same focal length at the centre"
    );
}

#[test]
fn every_lane_and_object_is_traced_to_a_label() {
    let map = scenarios::controlled_crossroads();
    let (_, trace) = roadgen_rdshq::to_shards_traced(&map, &ClipConfig::new("traced")).unwrap();
    assert_eq!(trace.format, "rdshq");
    for lane in map.lanes.iter().filter(|lane| lane.lane_type.is_drivable()) {
        assert!(
            trace
                .links
                .iter()
                .any(|link| link.ir == lane.id.clone().into() && link.local.starts_with("lanes:")),
            "{} has no label",
            lane.id
        );
    }
    for object in &map.objects {
        assert!(
            trace
                .links
                .iter()
                .any(|link| link.ir == object.id.clone().into()),
            "{} has no label",
            object.id
        );
    }
}

#[test]
fn the_same_map_writes_the_same_bytes() {
    let map = scenarios::controlled_crossroads();
    let config = ClipConfig::new("same");
    let first = roadgen_rdshq::to_shards(&map, &config).unwrap();
    let second = roadgen_rdshq::to_shards(&map, &config).unwrap();
    for (a, b) in first.iter().zip(&second) {
        assert_eq!(
            a.sample.to_tar().unwrap(),
            b.sample.to_tar().unwrap(),
            "{}",
            a.folder
        );
    }
}

#[test]
fn check_says_what_the_toolkit_will_do_with_the_clip() {
    let map = scenarios::controlled_crossroads();
    let general = roadgen_rdshq::check(&map, None);
    assert!(
        general.iter().any(|problem| problem.contains("topology")),
        "{general:?}"
    );
    // One light is the toolkit's placeholder for none.
    assert!(
        general
            .iter()
            .any(|problem| problem.contains("traffic_lights")),
        "{general:?}"
    );

    let slow = ClipConfig::new("x").with_frame_rate(10.0).with_speed(30.0);
    let problems = roadgen_rdshq::check(&map, Some(&slow));
    assert!(
        problems.iter().any(|problem| problem.contains("30 fps")),
        "{problems:?}"
    );
    assert!(
        problems
            .iter()
            .any(|problem| problem.contains("render nothing")),
        "{problems:?}"
    );
    assert!(
        problems
            .iter()
            .any(|problem| problem.contains("no sensors")),
        "{problems:?}"
    );

    let dotted = ClipConfig::new("a.b");
    assert!(roadgen_rdshq::write(&map, tempfile::tempdir().unwrap().path(), &dotted).is_err());
}

#[test]
fn a_file_the_clip_no_longer_has_does_not_outlive_a_re_export() {
    let map = scenarios::controlled_crossroads();
    let root = tempfile::tempdir().unwrap();
    let config = ClipConfig::new("again");
    roadgen_rdshq::write(&map, root.path(), &config).unwrap();
    let pinhole = root.path().join("pinhole_intrinsic").join("again.tar");
    assert!(pinhole.is_file());

    // A lens with no linear term has no focal length to give a pinhole.
    let mut sensor = roadgen_rdshq::scene::default_sensors().remove(0);
    sensor.polynomial = vec![0.0, 0.0, 1000.0];
    roadgen_rdshq::write(&map, root.path(), &config.with_sensors(vec![sensor])).unwrap();
    assert!(!pinhole.exists(), "the first export's pinhole is gone");
    assert!(root
        .path()
        .join("ftheta_intrinsic")
        .join("again.tar")
        .is_file());
}
