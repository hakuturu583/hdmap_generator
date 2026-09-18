//! Driving a generated map from a GPUDrive scenario file.
//!
//! The agents are the part of a scene that is not the map, and they come out of a YAML
//! file so the same network can be populated several ways without touching it. These
//! check that what the file says is what lands in the written scene, rather than only
//! in the configuration object — a scenario that parses but never reaches the file is
//! no use to anyone.

use roadgen_core::prelude::*;
use roadgen_gpudrive::scene::Scene;
use roadgen_gpudrive::{ObjectKind, SceneConfig};
use roadgen_integration_tests::scenarios;

fn written(map: &ValidatedMap, config: &SceneConfig) -> Scene {
    let text = roadgen_gpudrive::to_json(map, config).expect("the map should export");
    serde_json::from_str(&text).expect("the scene should read back")
}

#[test]
fn a_scenario_sets_the_route_the_agent_actually_drives() {
    let map = scenarios::crossroads();
    // The south arm, driven inwards. Nothing else in the map starts there, so a scene
    // that ignored the file would begin somewhere else entirely.
    let config = SceneConfig::new("x")
        .with_scenario_str("agents:\n  - speed: 8.0\n    route:\n      start: lane/south/0\n")
        .expect("the scenario should load");
    let scene = written(&map, &config);

    let lane = map.lane(&LaneId::from_raw("lane/south/0")).unwrap();
    let entry = lane.entry_point();
    let first = scene.objects[0].position[0];
    assert!(
        (first.x - entry.x).hypot(first.y - entry.y) < 1e-6,
        "the track should start where the scenario says: {first:?}"
    );

    // And at the speed it asked for: 8 m/s at the default 0.1 s timestep is 0.8 m a
    // step.
    let second = scene.objects[0].position[1];
    let step = (second.x - first.x).hypot(second.y - first.y);
    assert!((step - 0.8).abs() < 1e-6, "{step} m a step");
}

#[test]
fn a_listed_route_is_driven_exactly_as_written() {
    let map = scenarios::two_roads_joined();
    // Backwards down the second road and then the first: an order no successor search
    // would produce on its own.
    let config = SceneConfig::new("x")
        .with_scenario_str("agents:\n  - route:\n      lanes: [b/1, a/1]\n")
        .expect("the scenario should load");
    let scene = written(&map, &config);

    let track = &scene.objects[0];
    let start = map
        .lane(&LaneId::from_raw("lane/b/1"))
        .unwrap()
        .entry_point();
    assert!((track.position[0].x - start.x).hypot(track.position[0].y - start.y) < 1e-6);

    let goal = map
        .lane(&LaneId::from_raw("lane/a/1"))
        .unwrap()
        .exit_point();
    assert!(
        (track.goal_position.x - goal.x).hypot(track.goal_position.y - goal.y) < 1e-6,
        "the goal is where the route ends"
    );
}

#[test]
fn the_example_scenario_in_the_repository_still_works() {
    // The file the README points at. It names lanes of the crossroads fixture, so it
    // is checked against that map rather than only parsed.
    let map = scenarios::crossroads();
    let config = SceneConfig::for_map(&map)
        .with_scenario_file("../../examples/gpudrive-scenario.yaml")
        .expect("the example scenario should load");

    assert_eq!(config.name, "town");
    assert_eq!(config.scenario_id, "town-0001");
    assert_eq!(config.steps, 91);
    assert_eq!(config.agents.len(), 2);

    let scene = written(&map, &config);
    assert_eq!(scene.objects[0].kind, ObjectKind::Vehicle);
    assert_eq!(scene.objects[1].kind, ObjectKind::Cyclist);
    assert!(scene.objects[1].mark_as_expert);
    assert_eq!(
        scene.metadata.objects_of_interest,
        vec![scene.objects[1].id]
    );
    assert!(roadgen_gpudrive::check(&map, Some(&config))
        .iter()
        .all(|problem| !problem.contains("is not a lane of this map")));
}

#[test]
fn a_route_that_names_a_lane_the_map_does_not_have_is_refused() {
    let map = scenarios::crossroads();
    let config = SceneConfig::new("x")
        .with_scenario_str("agents:\n  - route:\n      start: lane/nowhere/0\n")
        .expect("the scenario itself parses");
    let error = roadgen_gpudrive::to_json(&map, &config).unwrap_err();
    assert!(error.to_string().contains("not a lane of this map"));
    assert!(roadgen_gpudrive::check(&map, Some(&config))
        .iter()
        .any(|problem| problem.contains("not a lane of this map")));
}
