//! Driving a generated map from a scenario file.
//!
//! The route and the rig are the two parts of a clip that are not the map, and both
//! come out of a YAML file so the same network can be driven several ways without
//! touching it. These check that what the file says is what lands in the clip —
//! through the written parquet, not through the config object, since a scenario that
//! parses but never reaches the file is no use to anyone.

use std::fs;

use roadgen_clipgt::{ClipConfig, Route};
use roadgen_core::prelude::*;
use roadgen_integration_tests::clipgt_read::layer;
use roadgen_integration_tests::{scenarios, write_clip};

/// The rig of a written clip, parsed back out of `rig_json`.
fn rig(directory: &std::path::Path, clip: &str) -> serde_json::Value {
    let row = &layer(directory, clip, "calibration_estimate")[0];
    serde_json::from_str(&row.strings["rig_json"]).expect("rig_json should be JSON")
}

fn config_from(text: &str) -> ClipConfig {
    ClipConfig::new("x")
        .with_scenario_str(text)
        .expect("the scenario should load")
}

#[test]
fn a_scenario_sets_the_route_the_vehicle_actually_drives() {
    let map = scenarios::crossroads();
    // The south arm, driven inwards. Nothing else in the map starts there, so a clip
    // that ignored the file would begin somewhere else entirely.
    let config = config_from("route:\n  start: lane/south/0\nspeed: 8.0\n");
    assert_eq!(
        config.route,
        Some(Route::From(LaneId::from_raw("lane/south/0")))
    );

    let (directory, clip) = write_clip(&map, &config);
    let poses = layer(directory.path(), &clip, "egomotion_estimate");
    let first = poses[0].point("location");

    let lane = map.lane(&LaneId::from_raw("lane/south/0")).unwrap();
    assert!(
        first.distance_to(lane.entry_point()) < 1.0,
        "the clip should start where the scenario says: {first:?}"
    );

    // And at the speed it asked for: 8 m/s at the default 30 Hz is 0.267 m a frame.
    let step = poses[0]
        .point("location")
        .distance_to(poses[1].point("location"));
    assert!((step - 8.0 / 30.0).abs() < 1e-6, "{step} m a frame");
}

#[test]
fn a_listed_route_is_driven_exactly_as_written() {
    let map = scenarios::two_roads_joined();
    // Backwards down the second road and then the first: an order no successor
    // search would ever produce.
    let config = config_from("route:\n  lanes: [lane/b/1, lane/a/1]\n");
    let (directory, clip) = write_clip(&map, &config);
    let poses = layer(directory.path(), &clip, "egomotion_estimate");

    let start = map.lane(&LaneId::from_raw("lane/b/1")).unwrap();
    let finish = map.lane(&LaneId::from_raw("lane/a/1")).unwrap();
    assert!(poses[0].point("location").distance_to(start.entry_point()) < 1.0);
    assert!(
        poses
            .last()
            .unwrap()
            .point("location")
            .distance_to(finish.exit_point())
            < 1.0
    );
}

#[test]
fn a_route_naming_a_lane_the_map_does_not_have_is_refused() {
    let map = scenarios::straight_road();
    let config = config_from("route:\n  start: lane/nowhere/0\n");

    // Reported before the export, and refused by it.
    let problems = roadgen_clipgt::check(&map, Some(&config));
    assert!(
        problems
            .iter()
            .any(|problem| problem.contains("lane/nowhere/0")),
        "{problems:?}"
    );
    let directory = tempfile::tempdir().unwrap();
    let error = roadgen_clipgt::write(&map, directory.path(), &config).unwrap_err();
    assert!(format!("{error}").contains("lane/nowhere/0"), "{error}");
}

#[test]
fn the_rig_reaches_the_clip_with_the_poses_it_was_given() {
    let map = scenarios::straight_road();
    let config = config_from(
        "sensors:\n\
         \x20 - name: camera:front_wide_120fov\n\
         \x20   position: [1.7, 0.0, 1.45]\n\
         \x20   width: 1920\n\
         \x20   height: 1080\n\
         \x20   fov_degrees: 120.0\n\
         \x20 - name: camera:rear_tele_30fov\n\
         \x20   position: [-0.9, 0.0, 1.3]\n\
         \x20   roll_pitch_yaw: [0.0, -1.5, 180.0]\n\
         \x20   width: 1280\n\
         \x20   height: 960\n\
         \x20   cx: 630.0\n\
         \x20   polynomial: [0.0, 1830.0, 0.0, 0.0, 0.0, 0.0]\n",
    );
    let (directory, clip) = write_clip(&map, &config);
    let rig = rig(directory.path(), &clip);
    let sensors = rig["rig"]["sensors"].as_array().unwrap();
    assert_eq!(sensors.len(), 2);

    let front = &sensors[0];
    assert_eq!(front["name"], "camera:front_wide_120fov");
    assert_eq!(front["properties"]["width"], "1920");
    // cx defaults to the middle of the frame; cy likewise.
    assert_eq!(front["properties"]["cx"], "960");
    assert_eq!(front["properties"]["cy"], "540");
    assert_eq!(front["nominalSensor2Rig_FLU"]["t"][0], 1.7);
    assert_eq!(front["nominalSensor2Rig_FLU"]["roll-pitch-yaw"][2], 0.0);

    let rear = &sensors[1];
    assert_eq!(rear["properties"]["cx"], "630", "given, so not the default");
    assert_eq!(rear["properties"]["cy"], "480", "not given, so the default");
    assert_eq!(rear["nominalSensor2Rig_FLU"]["roll-pitch-yaw"][2], 180.0);
    // The coefficients are space-separated, which is how they are read back.
    let coefficients: Vec<f64> = rear["properties"]["polynomial"]
        .as_str()
        .unwrap()
        .split_whitespace()
        .map(|value| value.parse().unwrap())
        .collect();
    assert_eq!(coefficients[1], 1830.0);
}

#[test]
fn each_camera_gets_the_frames_the_ego_track_has() {
    let map = scenarios::straight_road();
    let config = config_from(
        "frame_rate: 10.0\n\
         sensors:\n\
         \x20 - name: camera:front_wide_120fov\n\
         \x20   position: [1.7, 0.0, 1.45]\n\
         \x20   width: 100\n\
         \x20   height: 100\n\
         \x20   fov_degrees: 90.0\n",
    );
    let (directory, clip) = write_clip(&map, &config);

    let path = directory
        .path()
        .join(format!("{clip}.camera_front_wide_120fov.json"));
    let frames: Vec<serde_json::Value> =
        serde_json::from_str(&fs::read_to_string(&path).expect("the timestamps file")).unwrap();

    let poses = layer(directory.path(), &clip, "egomotion_estimate");
    assert_eq!(
        frames.len(),
        poses.len(),
        "one frame per pose, not a resample"
    );
    for (frame, pose) in frames.iter().zip(&poses) {
        assert_eq!(
            frame["timestamp"].as_i64().unwrap(),
            pose.integers["timestamp_micros"]
        );
    }
}

#[test]
fn a_clip_with_no_sensors_writes_no_timestamps_and_says_why() {
    let map = scenarios::straight_road();
    let config = ClipConfig::new("x");
    let (directory, clip) = write_clip(&map, &config);

    assert!(
        fs::read_dir(directory.path())
            .unwrap()
            .filter_map(Result::ok)
            .all(|entry| entry
                .path()
                .extension()
                .is_some_and(|kind| kind == "parquet")),
        "with no cameras there are no camera files"
    );
    let rig = rig(directory.path(), &clip);
    assert!(rig["rig"]["sensors"].as_array().unwrap().is_empty());

    let problems = roadgen_clipgt::check(&map, Some(&config));
    assert!(
        problems
            .iter()
            .any(|problem| problem.contains("no sensors")),
        "{problems:?}"
    );
}

#[test]
fn a_scenario_file_is_read_off_disk_and_named_in_its_errors() {
    let directory = tempfile::tempdir().unwrap();
    let path = directory.path().join("scenario.yaml");
    fs::write(&path, "speed: 14.0\nframe_rate: 20.0\n").unwrap();

    let config = ClipConfig::new("x").with_scenario_file(&path).unwrap();
    assert_eq!((config.speed, config.frame_rate), (14.0, 20.0));

    // A file that does not parse says which file, because a line and column on their
    // own are no help when several scenarios are in play.
    fs::write(&path, "speed: [this is not a number]\n").unwrap();
    let error = ClipConfig::new("x").with_scenario_file(&path).unwrap_err();
    assert!(format!("{error}").contains("scenario.yaml"), "{error}");

    let missing = ClipConfig::new("x")
        .with_scenario_file(directory.path().join("absent.yaml"))
        .unwrap_err();
    assert!(format!("{missing}").contains("absent.yaml"), "{missing}");
}

#[test]
fn the_same_scenario_exports_identically_twice() {
    let map = scenarios::controlled_crossroads();
    let scenario = "clip_id: twice\nspeed: 9.0\n\
         sensors:\n\
         \x20 - name: camera:front\n\
         \x20   position: [1.5, 0.0, 1.4]\n\
         \x20   width: 640\n\
         \x20   height: 480\n\
         \x20   fov_degrees: 100.0\n";
    let (first, clip) = write_clip(&map, &config_from(scenario));
    let (second, _) = write_clip(&map, &config_from(scenario));
    for entry in fs::read_dir(first.path()).unwrap() {
        let name = entry.unwrap().file_name();
        assert_eq!(
            fs::read(first.path().join(&name)).unwrap(),
            fs::read(second.path().join(&name)).unwrap(),
            "{name:?} differs between two exports of one scenario"
        );
    }
    assert_eq!(clip, "twice", "the file names it");
}

#[test]
fn the_example_scenario_in_the_repository_still_works() {
    // The file the README points at. If it stops parsing, the documentation is
    // wrong, and a test that only ever reads inline YAML would not notice.
    let path = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../examples/clipgt-scenario.yaml"
    );
    let config = ClipConfig::new("x")
        .with_scenario_file(path)
        .expect("the example scenario should load");

    assert_eq!(config.clip_id, "town");
    assert_eq!(config.speed, 12.0);
    assert_eq!(config.sensors.len(), 2);
    assert_eq!(
        config.sensors[0].canonical_name(),
        "camera_front_wide_120fov"
    );

    // And it drives the map it was written against.
    let map = scenarios::crossroads();
    assert!(roadgen_clipgt::check(&map, Some(&config))
        .iter()
        .all(|problem| !problem.contains("route")));
    let (directory, clip) = write_clip(&map, &config);
    assert_eq!(clip, "town");
    assert!(!layer(directory.path(), &clip, "egomotion_estimate").is_empty());
}
