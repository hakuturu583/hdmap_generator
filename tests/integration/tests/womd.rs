//! Exporting to the Waymo Open Motion Dataset.
//!
//! A WOMD scenario is a protocol buffer of Waymo's own schema in a TFRecord file, so
//! these tests write the file, read it back the way a consumer does — framing,
//! checksums and all — and check that the map in it is the one the IR described:
//! the lanes and the graph between them, the lines either side of each, the signals
//! and the signs.

use std::collections::HashMap;

use roadgen_core::prelude::*;
use roadgen_core::{IrRef, Relation};
use roadgen_integration_tests::scenarios;
use roadgen_womd::pb::{self, lane_center, map_feature::FeatureData, road_line};
use roadgen_womd::{Scenario, ScenarioConfig};

/// The map written to a TFRecord file and read back.
fn reread(map: &ValidatedMap) -> Scenario {
    let directory = tempfile::tempdir().unwrap();
    let path = directory.path().join("map.tfrecord");
    roadgen_womd::write(map, &path, &ScenarioConfig::for_map(map)).unwrap();
    let mut scenarios = roadgen_womd::read(&path).unwrap();
    assert_eq!(scenarios.len(), 1, "one scenario per file");
    scenarios.remove(0)
}

fn by_id(scenario: &Scenario) -> HashMap<i64, &FeatureData> {
    scenario
        .map_features
        .iter()
        .map(|feature| (feature.id.unwrap(), feature.feature_data.as_ref().unwrap()))
        .collect()
}

fn lanes(scenario: &Scenario) -> Vec<(i64, &pb::LaneCenter)> {
    scenario
        .map_features
        .iter()
        .filter_map(|feature| match &feature.feature_data {
            Some(FeatureData::Lane(lane)) => Some((feature.id.unwrap(), lane)),
            _ => None,
        })
        .collect()
}

#[test]
fn every_feature_has_a_distinct_id_and_geometry() {
    let scenario = reread(&scenarios::controlled_crossroads());
    let features = by_id(&scenario);
    assert_eq!(
        features.len(),
        scenario.map_features.len(),
        "ids are unique"
    );
    for feature in &scenario.map_features {
        let points = match feature.feature_data.as_ref().unwrap() {
            FeatureData::Lane(lane) => lane.polyline.len(),
            FeatureData::RoadLine(line) => line.polyline.len(),
            FeatureData::RoadEdge(edge) => edge.polyline.len(),
            FeatureData::Crosswalk(crosswalk) => crosswalk.polygon.len(),
            FeatureData::StopSign(sign) => usize::from(sign.position.is_some()),
            other => panic!("nothing writes {other:?}"),
        };
        assert!(points > 0, "feature {:?} has no geometry", feature.id);
    }
}

#[test]
fn the_lane_graph_is_the_maps() {
    let map = scenarios::controlled_crossroads();
    let scenario = reread(&map);
    let (_, trace) =
        roadgen_womd::to_scenario_traced(&map, &ScenarioConfig::for_map(&map)).unwrap();
    let id_of = |lane: &LaneId| -> i64 {
        let link = trace
            .links_of(&IrRef::Lane(lane.clone()))
            .find(|link| link.relation == Relation::Exact)
            .unwrap();
        link.local
            .strip_prefix("feature:")
            .unwrap()
            .parse()
            .unwrap()
    };
    let features = by_id(&scenario);
    for lane in map.lanes.iter().filter(|lane| lane.lane_type.is_drivable()) {
        let Some(FeatureData::Lane(center)) = features.get(&id_of(&lane.id)) else {
            panic!("{} is not written as a lane", lane.id);
        };
        let exits: Vec<i64> = map.successors(&lane.id).iter().map(id_of).collect();
        let entries: Vec<i64> = map.predecessors(&lane.id).iter().map(id_of).collect();
        assert_eq!(center.exit_lanes, exits, "{}", lane.id);
        assert_eq!(center.entry_lanes, entries, "{}", lane.id);
        let connector = map.road(&lane.road).unwrap().is_connector();
        assert_eq!(center.interpolating, Some(connector), "{}", lane.id);
        if connector {
            assert!(center.left_boundaries.is_empty() && center.right_boundaries.is_empty());
        }
        // Each exit begins where this lane ends.
        for exit in &center.exit_lanes {
            let Some(FeatureData::Lane(next)) = features.get(exit) else {
                panic!("an exit is not a lane");
            };
            let (a, b) = (
                center.polyline.last().unwrap(),
                next.polyline.first().unwrap(),
            );
            let gap = (a.x() - b.x()).hypot(a.y() - b.y());
            assert!(gap < 0.5, "{} is {gap} m from its exit", lane.id);
        }
    }
}

#[test]
fn a_lane_names_the_lines_beside_it_and_its_neighbours_going_its_way() {
    let map = scenarios::multi_lane_road();
    let scenario = reread(&map);
    let features = by_id(&scenario);
    let lanes = lanes(&scenario);
    assert_eq!(lanes.len(), 4);

    // The 60 km/h limit, in WOMD's miles per hour.
    for (_, lane) in &lanes {
        let mph = lane.speed_limit_mph.unwrap();
        assert!((mph - 60.0 / 1.609_344).abs() < 1e-6, "{mph}");
        assert_eq!(lane.r#type(), lane_center::LaneType::TypeSurfaceStreet);
    }

    // The three forward lanes, innermost first: a yellow line on the left of the
    // first, broken white lines between them, a solid white edge line on the right
    // of the third — which is the road's edge, since nothing drives beyond it.
    let forward = &lanes[..3];
    let left_type = |lane: &pb::LaneCenter| {
        let segment = &lane.left_boundaries[0];
        (
            segment.boundary_type(),
            features[&segment.boundary_feature_id()],
        )
    };
    let (kind, _) = left_type(forward[0].1);
    assert_eq!(kind, road_line::RoadLineType::TypeSolidSingleYellow);
    for (_, lane) in &forward[1..] {
        let (kind, feature) = left_type(lane);
        assert_eq!(kind, road_line::RoadLineType::TypeBrokenSingleWhite);
        assert!(matches!(feature, FeatureData::RoadLine(_)));
    }
    let right = &forward[2].1.right_boundaries[0];
    assert!(matches!(
        features[&right.boundary_feature_id()],
        FeatureData::RoadEdge(_)
    ));
    assert_eq!(right.boundary_type(), road_line::RoadLineType::TypeUnknown);
    for (_, lane) in forward {
        let segment = &lane.left_boundaries[0];
        assert_eq!(segment.lane_start_index(), 0);
        assert_eq!(segment.lane_end_index() as usize, lane.polyline.len() - 1);
    }

    // Neighbours are the adjacent lanes going the same way, and only those.
    let neighbour_ids =
        |list: &[pb::LaneNeighbor]| list.iter().map(|n| n.feature_id()).collect::<Vec<_>>();
    assert_eq!(
        neighbour_ids(&forward[0].1.left_neighbors),
        Vec::<i64>::new()
    );
    assert_eq!(
        neighbour_ids(&forward[0].1.right_neighbors),
        vec![forward[1].0]
    );
    assert_eq!(
        neighbour_ids(&forward[1].1.left_neighbors),
        vec![forward[0].0]
    );
    assert_eq!(
        neighbour_ids(&forward[1].1.right_neighbors),
        vec![forward[2].0]
    );
    assert!(lanes[3].1.left_neighbors.is_empty() && lanes[3].1.right_neighbors.is_empty());
}

#[test]
fn a_signalled_approach_hands_its_state_to_the_lanes_through_the_junction() {
    let map = scenarios::controlled_crossroads();
    let scenario = reread(&map);
    let features = by_id(&scenario);
    assert_eq!(scenario.dynamic_map_states.len(), roadgen_womd::STEPS);
    let states = &scenario.dynamic_map_states[0].lane_states;
    assert!(!states.is_empty(), "the light governs something");
    for state in states {
        let Some(FeatureData::Lane(lane)) = features.get(&state.lane()) else {
            panic!("a signal names a feature that is not a lane");
        };
        assert_eq!(
            lane.interpolating,
            Some(true),
            "the state is on a lane through"
        );
        assert_eq!(
            state.state(),
            pb::traffic_signal_lane_state::State::LaneStateUnknown
        );
        assert_eq!(state.stop_point.as_ref(), lane.polyline.first());
    }
    for step in &scenario.dynamic_map_states {
        assert_eq!(&step.lane_states, states, "the IR has no timing to vary");
    }
}

#[test]
fn a_stop_sign_names_the_lane_it_stops() {
    let mut builder = MapBuilder::new(scenarios::metadata("signed"));
    let road = builder
        .add_road(
            RoadSpec::line(
                Point3::new(0.0, 0.0, 2.0),
                Point3::new(80.0, 0.0, 2.0),
                scenarios::two_way(),
            )
            .unwrap(),
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

    let scenario = reread(&map);
    let signs: Vec<&pb::StopSign> = scenario
        .map_features
        .iter()
        .filter_map(|feature| match &feature.feature_data {
            Some(FeatureData::StopSign(sign)) => Some(sign),
            _ => None,
        })
        .collect();
    assert_eq!(signs.len(), 1, "only the stop sign has anywhere to go");
    let position = signs[0].position.as_ref().unwrap();
    assert!((position.x() - 80.0).abs() < 1.0);
    // The road is 2 m up and the sign hangs 2.5 m above it: a WOMD point has a z.
    assert!((position.z() - 4.5).abs() < 1e-6, "{}", position.z());
    let features = by_id(&scenario);
    assert_eq!(signs[0].lane.len(), 1);
    assert!(matches!(features[&signs[0].lane[0]], FeatureData::Lane(_)));
    assert!(roadgen_womd::check(&map, None)
        .iter()
        .any(|problem| problem.contains("1 signs of other kinds")));
}

#[test]
fn the_check_says_what_the_format_cannot_hold() {
    let map = scenarios::controlled_crossroads();
    let problems = roadgen_womd::check(&map, Some(&ScenarioConfig::default()));
    let said = |needle: &str| problems.iter().any(|problem| problem.contains(needle));
    assert!(said("no tracks"));
    assert!(said("LANE_STATE_UNKNOWN"));
    assert!(said("stop lines are dropped"));
    assert!(said("right-of-way"));
}
