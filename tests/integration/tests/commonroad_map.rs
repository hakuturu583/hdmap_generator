//! Exporting the map alone, as a CommonRoad 2024 `CommonRoadMap` protocol buffer.
//!
//! The map is the scenario's lanelet network re-cut into the 2024 format's
//! elements — bounds and stop lines of their own, named signs that all stand
//! somewhere — with the buildings beside it. These tests read the written bytes
//! back with the schema they were written with, and check that every reference
//! lands and that the map is the scenario's.

use std::collections::{HashMap, HashSet};

use roadgen_buildings::Rules;
use roadgen_commonroad::protobuf::pb::commonroad_common::traffic_sign_id_enum::TrafficSignId;
use roadgen_commonroad::protobuf::pb::commonroad_map::environment_obstacle_type_enum::EnvironmentObstacleType;
use roadgen_commonroad::protobuf::{self, CommonRoadMap};
use roadgen_commonroad::ScenarioConfig;
use roadgen_core::prelude::*;
use roadgen_core::IrRef;
use roadgen_integration_tests::{highway, scenarios};

fn config(map: &ValidatedMap) -> ScenarioConfig {
    ScenarioConfig::for_map(map).with_date("2026-01-01")
}

/// The map written to a file and read back.
fn reread(map: &ValidatedMap) -> CommonRoadMap {
    let directory = tempfile::tempdir().unwrap();
    let path = directory.path().join("map.pb");
    protobuf::write_map(map, &path, &config(map)).unwrap();
    protobuf::read_map(&path).unwrap()
}

#[test]
fn every_reference_lands_and_no_bound_is_shared() {
    for map in [
        scenarios::controlled_crossroads(),
        scenarios::sidewalk_crossroads(TrafficHandedness::RightHand),
        scenarios::crosswalk_crossroads(TrafficHandedness::LeftHand).0,
        highway::highway(),
    ] {
        let written = reread(&map);
        let lanelets: HashSet<u32> = written.lanelets.iter().map(|l| l.lanelet_id).collect();
        let bounds: HashMap<i32, usize> = written
            .boundaries
            .iter()
            .map(|bound| (bound.boundary_id, bound.points.len()))
            .collect();
        let stop_lines: HashSet<u32> = written.stop_lines.iter().map(|s| s.stop_line_id).collect();
        let signs: HashSet<u32> = written
            .traffic_signs
            .iter()
            .map(|s| s.traffic_sign_id)
            .collect();
        let lights: HashSet<u32> = written
            .traffic_lights
            .iter()
            .map(|l| l.traffic_light_id)
            .collect();
        assert_eq!(
            bounds.len(),
            written.boundaries.len(),
            "bound ids are unique"
        );
        assert_eq!(
            bounds.len(),
            2 * written.lanelets.len(),
            "two bounds a lanelet"
        );

        let mut used = HashSet::new();
        for lanelet in &written.lanelets {
            for bound in [lanelet.left_bound, lanelet.right_bound] {
                assert!(bounds.contains_key(&bound), "bound {bound} names nothing");
                assert!(used.insert(bound), "bound {bound} is shared");
            }
            assert_eq!(bounds[&lanelet.left_bound], bounds[&lanelet.right_bound]);
            for next in lanelet.successors.iter().chain(&lanelet.predecessors) {
                assert!(lanelets.contains(next));
            }
            for adjacent in lanelet.adjacent_left.iter().chain(&lanelet.adjacent_right) {
                assert!(lanelets.contains(adjacent));
            }
            if let Some(stop) = lanelet.stop_line {
                assert!(stop_lines.contains(&stop));
            }
            assert!(lanelet
                .traffic_sign_refs
                .iter()
                .all(|id| signs.contains(id)));
            assert!(lanelet
                .traffic_light_refs
                .iter()
                .all(|id| lights.contains(id)));
            assert!(!lanelet.lanelet_types.is_empty());
        }
        for intersection in &written.intersections {
            for incoming in &intersection.incomings {
                for id in incoming
                    .incoming_lanelets
                    .iter()
                    .chain(&incoming.outgoing_left)
                    .chain(&incoming.outgoing_straight)
                    .chain(&incoming.outgoing_right)
                {
                    assert!(lanelets.contains(id));
                }
            }
        }
    }
}

#[test]
fn the_map_is_the_scenarios_network() {
    let map = scenarios::controlled_crossroads();
    let scenario = roadgen_commonroad::to_scenario(&map, &config(&map)).unwrap();
    let written = reread(&map);
    let bounds: HashMap<i32, &_> = written
        .boundaries
        .iter()
        .map(|bound| (bound.boundary_id, bound))
        .collect();
    assert_eq!(written.lanelets.len(), scenario.lanelets.len());
    for (lanelet, expected) in written.lanelets.iter().zip(&scenario.lanelets) {
        assert_eq!(u64::from(lanelet.lanelet_id), expected.id);
        let left = &bounds[&lanelet.left_bound].points;
        assert_eq!(left.len(), expected.left.points.len());
        assert!((left[0].x - expected.left.points[0].x).abs() < 1e-12);
        assert!(left[0].z.is_none(), "in the plane unless asked");
        let successors: Vec<u64> = lanelet.successors.iter().map(|&id| id.into()).collect();
        assert_eq!(successors, expected.successors);
    }
    assert!(
        !written.stop_lines.is_empty(),
        "the approach stops for its light"
    );
}

#[test]
fn every_sign_is_named_and_stands_somewhere() {
    let mut builder = MapBuilder::new(scenarios::metadata("signed"));
    let road = builder
        .add_road(
            RoadSpec::line(
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(80.0, 0.0, 0.0),
                scenarios::two_way(),
            )
            .unwrap(),
        )
        .unwrap();
    let forward = LaneRef::new(road.clone(), 0);
    builder
        .add_traffic_sign(&forward, LaneEnd::End, "stop", 2.5)
        .unwrap();
    builder.add_speed_limit_rule(
        SpeedLimit::from_kph(30.0).unwrap(),
        vec![LaneRef::new(road, 1)],
    );
    let map = builder.finish().unwrap().validate().unwrap();
    let written = reread(&map);

    let named: Vec<(TrafficSignId, Option<bool>)> = written
        .traffic_signs
        .iter()
        .map(|sign| (sign.traffic_sign_elements[0].element_id(), sign.r#virtual))
        .collect();
    assert!(named.contains(&(TrafficSignId::Stop, Some(false))));
    assert!(named.contains(&(TrafficSignId::MaxSpeed, Some(true))));
    // The virtual limit stands where the lanelet it holds on begins.
    let limit = written
        .traffic_signs
        .iter()
        .find(|sign| sign.r#virtual == Some(true))
        .unwrap();
    let lanelet = written
        .lanelets
        .iter()
        .find(|lanelet| lanelet.traffic_sign_refs.contains(&limit.traffic_sign_id))
        .unwrap();
    let right = written
        .boundaries
        .iter()
        .find(|bound| bound.boundary_id == lanelet.right_bound)
        .unwrap();
    assert_eq!(limit.position, right.points[0]);
}

#[test]
fn the_buildings_are_environment_obstacles() {
    let mut map = scenarios::crossroads_builder("town", 260.0)
        .finish()
        .unwrap();
    roadgen_buildings::generate(&mut map, &Rules::default()).unwrap();
    let map = map.validate().unwrap();
    let (written, trace) = protobuf::to_map_traced(&map, &config(&map)).unwrap();
    assert!(!map.building_parts.is_empty());
    assert_eq!(
        written.environment_obstacles.len(),
        map.building_parts.len()
    );
    for obstacle in &written.environment_obstacles {
        assert_eq!(obstacle.obstacle_type(), EnvironmentObstacleType::Building);
    }
    let part = map.building_parts.iter().next().unwrap();
    assert!(trace
        .links_of(&IrRef::BuildingPart(part.id.clone()))
        .any(|link| link.local.starts_with("environment_obstacle:")));
    assert!(roadgen_commonroad::check_map(&map, None)
        .iter()
        .any(|problem| problem.contains("environment obstacles")));
}
