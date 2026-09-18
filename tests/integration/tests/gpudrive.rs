//! Exporting to GPUDrive.
//!
//! A GPUDrive scene is JSON with no schema: what makes a file a scene is that the
//! simulator's reader accepts it. So these tests read the written scene back the way
//! the reader does — every key it insists on, the array lengths it steps through in
//! parallel, the type strings it compares against — and then check that what those
//! fields hold is the map the IR described and the agents the scenario asked for.

use std::collections::{BTreeSet, HashSet};

use serde_json::Value;

use roadgen_core::prelude::*;
use roadgen_core::Lane;
use roadgen_gpudrive::scene::{MapElement, RoadKind, Scene};
use roadgen_gpudrive::{Agent, ObjectKind, SceneConfig};
use roadgen_integration_tests::scenarios;

/// The scene as JSON, and as the model, from one export.
fn export(map: &ValidatedMap, config: &SceneConfig) -> (Value, Scene) {
    let text = roadgen_gpudrive::to_json(map, config).expect("the map should export as a scene");
    (
        serde_json::from_str(&text).expect("the scene should be JSON"),
        serde_json::from_str(&text).expect("the scene should read back as a scene"),
    )
}

fn scene_of(map: &ValidatedMap) -> Scene {
    export(map, &SceneConfig::for_map(map)).1
}

fn roads_of(scene: &Scene, kind: RoadKind) -> Vec<&roadgen_gpudrive::Road> {
    scene
        .roads
        .iter()
        .filter(|road| road.kind == kind)
        .collect()
}

#[test]
fn a_scene_holds_every_key_the_reader_insists_on() {
    let map = scenarios::controlled_crossroads();
    let (json, _) = export(&map, &SceneConfig::new("x"));

    for key in ["name", "scenario_id", "objects", "roads", "metadata"] {
        assert!(json.get(key).is_some(), "the scene has no {key}");
    }
    let metadata = &json["metadata"];
    for key in [
        "sdc_track_index",
        "tracks_to_predict",
        "objects_of_interest",
    ] {
        assert!(metadata.get(key).is_some(), "the metadata has no {key}");
    }

    for object in json["objects"].as_array().expect("objects is an array") {
        for key in [
            "position",
            "width",
            "length",
            "height",
            "id",
            "heading",
            "velocity",
            "valid",
            "goalPosition",
            "type",
        ] {
            assert!(object.get(key).is_some(), "an object has no {key}");
        }
        // The reader walks these four in step, so a scene whose arrays disagree is one
        // it reads past the end of.
        let steps = object["position"].as_array().unwrap().len();
        for key in ["heading", "velocity", "valid"] {
            assert_eq!(
                object[key].as_array().unwrap().len(),
                steps,
                "{key} is not as long as the positions"
            );
        }
        for point in object["position"].as_array().unwrap() {
            assert!(point.get("x").is_some() && point.get("y").is_some());
        }
    }

    for road in json["roads"].as_array().expect("roads is an array") {
        for key in ["type", "geometry", "id", "map_element_id"] {
            assert!(road.get(key).is_some(), "a road element has no {key}");
        }
        // A geometry of no points sizes as -1 segments in the reader's arithmetic.
        assert!(!road["geometry"].as_array().unwrap().is_empty());
    }
}

#[test]
fn the_types_written_are_the_strings_the_reader_compares_against() {
    let map = scenarios::controlled_crossroads();
    let (json, _) = export(&map, &SceneConfig::default());

    let object_types: BTreeSet<&str> = json["objects"]
        .as_array()
        .unwrap()
        .iter()
        .map(|object| object["type"].as_str().unwrap())
        .collect();
    for written in &object_types {
        assert!(
            ["vehicle", "pedestrian", "cyclist"].contains(written),
            "{written} is not an entity type the reader knows"
        );
    }

    let road_types: BTreeSet<&str> = json["roads"]
        .as_array()
        .unwrap()
        .iter()
        .map(|road| road["type"].as_str().unwrap())
        .collect();
    for written in &road_types {
        assert!(
            [
                "road_edge",
                "road_line",
                "lane",
                "crosswalk",
                "speed_bump",
                "stop_sign"
            ]
            .contains(written),
            "{written} is not a road type the reader knows"
        );
    }
    assert!(road_types.contains("lane"));
    assert!(road_types.contains("road_edge"));
    assert!(road_types.contains("crosswalk"));
}

#[test]
fn a_lane_is_its_centreline_in_travel_order() {
    let map = scenarios::bidirectional_road();
    let scene = scene_of(&map);
    let drivable: Vec<&Lane> = map
        .lanes
        .iter()
        .filter(|lane| lane.lane_type.is_drivable())
        .collect();
    let lanes = roads_of(&scene, RoadKind::Lane);
    assert_eq!(lanes.len(), drivable.len());

    // The lanes are written in the map's own order, so the n-th row is the n-th lane —
    // and a backward lane's row starts where traffic enters it, not where the
    // reference line does.
    for (row, lane) in lanes.iter().zip(&drivable) {
        let entry = lane.entry_point();
        let exit = lane.exit_point();
        let first = row.geometry.first().unwrap();
        let last = row.geometry.last().unwrap();
        assert!(
            (first.x - entry.x).hypot(first.y - entry.y) < 1e-6,
            "{} does not start where traffic enters it",
            lane.id
        );
        assert!(
            (last.x - exit.x).hypot(last.y - exit.y) < 1e-6,
            "{} does not end where traffic leaves it",
            lane.id
        );
    }
}

#[test]
fn the_markings_of_a_road_become_waymo_line_codes() {
    let scene = scene_of(&scenarios::multi_lane_road());
    let codes: HashSet<MapElement> = roads_of(&scene, RoadKind::RoadLine)
        .iter()
        .map(|road| road.map_element_id)
        .collect();
    assert!(codes.contains(&MapElement::RoadLineSolidSingleYellow));
    assert!(codes.contains(&MapElement::RoadLineBrokenSingleWhite));

    // The outermost edge of the carriageway is an edge, whatever is painted on it: one
    // element per cross-section boundary, never two stacked on each other.
    let edges = roads_of(&scene, RoadKind::RoadEdge);
    assert_eq!(edges.len(), 2, "a road has two sides");
    for edge in edges {
        assert_eq!(edge.map_element_id, MapElement::RoadEdgeBoundary);
    }
}

#[test]
fn a_junction_has_lanes_through_it_but_no_edges_across_it() {
    let map = scenarios::crossroads();
    let scene = scene_of(&map);

    let connectors: Vec<&Lane> = map
        .lanes
        .iter()
        .filter(|lane| map.road(&lane.road).is_some_and(|road| road.is_connector()))
        .collect();
    assert!(!connectors.is_empty(), "the fixture has connectors");
    assert_eq!(
        roads_of(&scene, RoadKind::Lane).len(),
        map.lanes
            .iter()
            .filter(|lane| lane.lane_type.is_drivable())
            .count(),
        "every drivable lane, connectors included, is written as a centreline"
    );

    // A road edge is something an agent collides with, and there is no wall down the
    // middle of an intersection: the arms stop 14 m short of the centre, so nothing
    // nearer than that may be an edge.
    for edge in roads_of(&scene, RoadKind::RoadEdge) {
        for point in &edge.geometry {
            assert!(
                point.x.hypot(point.y) > 10.0,
                "a road edge runs through the junction at ({}, {})",
                point.x,
                point.y
            );
        }
    }
}

#[test]
fn a_stop_sign_is_the_one_sign_the_vocabulary_has() {
    let mut builder = MapBuilder::new(scenarios::metadata("signed"));
    let road = builder
        .add_road(
            RoadSpec::line(
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(80.0, 0.0, 0.0),
                scenarios::two_way(),
            )
            .unwrap()
            .with_name("main"),
        )
        .unwrap();
    let lane = LaneRef::new(road, 0);
    builder
        .add_traffic_sign(&lane, LaneEnd::End, "stop", 2.5)
        .unwrap();
    builder
        .add_traffic_sign(&lane, LaneEnd::Start, "speed_limit_50", 2.5)
        .unwrap();
    let map = builder.finish().unwrap().validate().unwrap();

    let scene = scene_of(&map);
    let signs = roads_of(&scene, RoadKind::StopSign);
    assert_eq!(signs.len(), 1, "only the stop sign has anywhere to go");
    assert_eq!(signs[0].map_element_id, MapElement::StopSign);
    assert_eq!(signs[0].geometry.len(), 1, "a stop sign is a point");
    assert!(
        (signs[0].geometry[0].x - 80.0).abs() < 1.0,
        "the sign sits at the end of the lane it governs"
    );

    assert!(roadgen_gpudrive::check(&map, None)
        .iter()
        .any(|problem| problem.contains("signs of other kinds")));
}

#[test]
fn an_agent_drives_its_route_at_the_speed_it_was_asked_for() {
    let map = scenarios::two_roads_joined();
    let config = SceneConfig::new("driven")
        .with_steps(20)
        .with_time_step(0.1)
        .with_agents(vec![Agent::new(ObjectKind::Vehicle).with_speed(12.0)]);
    let scene = export(&map, &config).1;

    let agent = &scene.objects[0];
    assert_eq!(agent.steps(), 20);
    assert!(agent.is_consistent());
    assert!(agent.valid.iter().all(|valid| *valid));

    // Twelve metres a second at a tenth of a second is 1.2 m a step, measured on the
    // ground: the fixture climbs, and a track that paced itself along the slope would
    // fall short here.
    for pair in agent.position.windows(2) {
        let step = (pair[1].x - pair[0].x).hypot(pair[1].y - pair[0].y);
        assert!((step - 1.2).abs() < 1e-2, "a step of {step} m");
    }
    for (position, heading) in agent.position.windows(2).zip(&agent.heading) {
        let along = (position[1].y - position[0].y).atan2(position[1].x - position[0].x);
        assert!(
            (along - heading).abs() < 1e-6,
            "the heading is the direction"
        );
    }
    for (velocity, heading) in agent.velocity.iter().zip(&agent.heading) {
        assert!((velocity.x.hypot(velocity.y) - 12.0).abs() < 1e-9);
        assert!((velocity.y.atan2(velocity.x) - heading).abs() < 1e-9);
    }
}

#[test]
fn an_agent_stands_at_its_goal_once_the_route_runs_out() {
    let map = scenarios::straight_road();
    // Ninety-one steps at 20 m/s is 180 m of driving and the fixture is 120 m long, so
    // the last steps are the agent waiting at the end of it.
    let config = SceneConfig::new("short").with_agents(vec![Agent::default().with_speed(20.0)]);
    let scene = export(&map, &config).1;
    let agent = &scene.objects[0];

    let last = agent.position.last().unwrap();
    assert!((last.x - agent.goal_position.x).abs() < 1e-9);
    assert!((last.y - agent.goal_position.y).abs() < 1e-9);

    let stopped = agent.velocity.last().unwrap();
    assert_eq!((stopped.x, stopped.y), (0.0, 0.0));
    assert!(
        agent.valid.iter().all(|valid| *valid),
        "an agent that has arrived is still in the scene"
    );
}

#[test]
fn a_scenario_file_says_who_drives_and_where() {
    let map = scenarios::crossroads();
    let start = map
        .lanes
        .iter()
        .find(|lane| lane.lane_type.is_drivable())
        .unwrap()
        .id
        .clone();
    let text = format!(
        "name: crossing\nscenario_id: crossing-1\nsteps: 30\nagents:\n  \
         - type: vehicle\n    speed: 8.0\n    route:\n      start: {start}\n  \
         - type: cyclist\n    speed: 4.0\n    mark_as_expert: true\n    of_interest: true\n"
    );
    let config = SceneConfig::for_map(&map)
        .with_scenario_str(&text)
        .expect("the scenario should read");
    let scene = export(&map, &config).1;

    assert_eq!(scene.name, "crossing");
    assert_eq!(scene.scenario_id, "crossing-1");
    assert_eq!(scene.objects.len(), 2);
    assert_eq!(scene.objects[0].kind, ObjectKind::Vehicle);
    assert_eq!(scene.objects[1].kind, ObjectKind::Cyclist);
    assert!(scene.objects[1].mark_as_expert);
    assert_eq!(scene.objects[0].steps(), 30);
    assert_eq!(scene.metadata.sdc_track_index, 0);
    assert_eq!(
        scene.metadata.objects_of_interest,
        vec![scene.objects[1].id]
    );
    assert_eq!(scene.metadata.tracks_to_predict.len(), 2);
}

#[test]
fn what_the_format_cannot_hold_is_reported_rather_than_assumed() {
    let map = scenarios::controlled_crossroads();
    let problems = roadgen_gpudrive::check(&map, Some(&SceneConfig::for_map(&map)));
    let said = |text: &str| problems.iter().any(|problem| problem.contains(text));

    assert!(said("no z"), "the heights are dropped");
    assert!(said("centreline"), "the lane widths are dropped");
    assert!(said("no lane topology"), "the connections are dropped");
    assert!(said("traffic-light element"), "the lights are dropped");
    assert!(said("stop lines are"), "the stop lines are dropped");

    // The limit belongs to a road rather than to a junction, so it takes a map that
    // has one to report.
    let limited = scenarios::multi_lane_road();
    assert!(roadgen_gpudrive::check(&limited, None)
        .iter()
        .any(|problem| problem.contains("speed limits")));
}

#[test]
fn a_written_scene_is_the_scene_that_was_built() {
    let map = scenarios::crossroads();
    let config = SceneConfig::for_map(&map);
    let built = roadgen_gpudrive::to_scene(&map, &config).unwrap();

    let directory = tempfile::tempdir().unwrap();
    let path = directory.path().join("scene.json");
    roadgen_gpudrive::write(&map, &path, &config).expect("the scene should write");

    let text = std::fs::read_to_string(&path).unwrap();
    let read: Scene = serde_json::from_str(&text).expect("the file should read back");
    assert_eq!(read, built);
    assert!(read.road_segments() > 0);
}
