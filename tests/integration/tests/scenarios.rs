//! The ten scenarios the design notes ask for, each generated once and then checked
//! in the IR and in both exports.

use ll2_core::map::as_lanelet;
use roadgen_core::prelude::*;
use roadgen_integration_tests::scenarios;
use roadgen_integration_tests::{reload_lanelet2, reparse_opendrive, routing_edges};

/// Every scenario has to come out of the builder valid and be acceptable to both
/// exporters — that is the floor the rest of the tests stand on.
fn assert_exports_cleanly(map: &ValidatedMap) {
    assert!(
        roadgen_opendrive::check(map).is_empty(),
        "OpenDRIVE: {:?}",
        roadgen_opendrive::check(map)
    );
    assert!(
        roadgen_lanelet2::check(map).is_empty(),
        "Lanelet2: {:?}",
        roadgen_lanelet2::check(map)
    );
    reparse_opendrive(map);
    reload_lanelet2(map);
}

// 1 -------------------------------------------------------------------------

#[test]
fn a_straight_road_in_three_dimensions() {
    let map = scenarios::straight_road();
    assert_exports_cleanly(&map);

    assert_eq!(map.roads.len(), 1);
    assert_eq!(map.lanes.len(), 1);
    let lane = map.lanes.iter().next().unwrap();

    // One forward lane under right-hand traffic sits to the right of the reference
    // line, so its centre is half a lane width below zero.
    assert_eq!(lane.side, LateralSide::Right);
    assert!((lane.center_offset() + 1.75).abs() < 1e-9);
    assert!((lane.centerline.start_point().y + 1.75).abs() < 1e-9);
    // The road is level: every vertex keeps the height it was given.
    assert!((lane.centerline.start_point().z - 5.0).abs() < 1e-9);
    assert!((lane.centerline.end_point().z - 5.0).abs() < 1e-9);
}

// 2 -------------------------------------------------------------------------

#[test]
fn a_road_with_a_carriageway_each_way() {
    let map = scenarios::bidirectional_road();
    assert_exports_cleanly(&map);

    let lanes = map.lanes_of(&RoadId::new("main"));
    assert_eq!(lanes.len(), 2);
    assert_eq!(lanes[0].direction, Direction::Forward);
    assert_eq!(lanes[1].direction, Direction::Backward);
    // They sit on opposite sides of the reference line and share the boundary at
    // zero between them.
    assert_eq!(lanes[0].side, LateralSide::Right);
    assert_eq!(lanes[1].side, LateralSide::Left);
    assert!((lanes[0].left_offset).abs() < 1e-9);
    assert!((lanes[1].right_offset).abs() < 1e-9);

    // Both become lanelets, each running the way its traffic does.
    let loaded = reload_lanelet2(&map);
    assert_eq!(loaded.lanelets.len(), 2);
    let forward = &lanes[0];
    let backward = &lanes[1];
    assert!(forward
        .travel_geometry(map.metadata.sampling)
        .unwrap()
        .centerline
        .start_point()
        .is_close(Point3::new(0.0, -1.75, 0.0), 1e-9));
    assert!(backward
        .travel_geometry(map.metadata.sampling)
        .unwrap()
        .centerline
        .start_point()
        .is_close(Point3::new(150.0, 1.75, 0.0), 1e-9));
}

// 3 -------------------------------------------------------------------------

#[test]
fn a_road_with_several_lanes() {
    let map = scenarios::multi_lane_road();
    assert_exports_cleanly(&map);

    let lanes = map.lanes_of(&RoadId::new("wide"));
    assert_eq!(lanes.len(), 4);

    // Three lanes one way, ranked outwards from the reference line, then one the
    // other way on the far side.
    let forward: Vec<_> = lanes
        .iter()
        .filter(|lane| lane.direction == Direction::Forward)
        .collect();
    assert_eq!(
        forward.iter().map(|lane| lane.ordinal).collect::<Vec<_>>(),
        [1, 2, 3]
    );
    // Each lane starts where the one before it ended: no gaps and no overlap.
    for pair in forward.windows(2) {
        assert!((pair[0].right_offset - pair[1].left_offset).abs() < 1e-9);
    }
    // The outermost forward lane ends 3.5 + 3.5 + 3.25 m right of the reference line.
    assert!((forward[2].right_offset + 10.25).abs() < 1e-9);

    // Neighbouring lanes share the boundary between them, which is what makes them
    // laterally adjacent in Lanelet2.
    let exported = roadgen_lanelet2::to_lanelet_map(&map).unwrap();
    let lanelets: Vec<_> = exported
        .lanelets
        .all()
        .into_iter()
        .filter_map(|primitive| as_lanelet(&primitive).cloned())
        .collect();
    assert_eq!(lanelets.len(), 4);
    let neighbours = lanelets
        .iter()
        .flat_map(|a| {
            lanelets
                .iter()
                .filter(move |b| !a.is_same_view(b) && ll2_core::geometry::lanelet::left_of(a, b))
        })
        .count();
    // Three boundaries are shared between same-direction neighbours.
    assert_eq!(neighbours, 2);

    // The speed limit set on the road reaches every lane.
    assert!(lanes
        .iter()
        .all(|lane| lane.speed_limit.map(|limit| limit.kph()).unwrap_or(0.0) > 59.9));
}

// 4 -------------------------------------------------------------------------

#[test]
fn two_roads_joined_end_to_start() {
    let map = scenarios::two_roads_joined();
    assert_exports_cleanly(&map);

    assert_eq!(map.connections.len(), 2);
    assert_eq!(
        map.successors(&LaneId::of_road(&RoadId::new("a"), 0)),
        [LaneId::of_road(&RoadId::new("b"), 0)]
    );
    // The opposing carriageway runs the other way, from b back into a.
    assert_eq!(
        map.successors(&LaneId::of_road(&RoadId::new("b"), 1)),
        [LaneId::of_road(&RoadId::new("a"), 1)]
    );

    // Both movements survive into a Lanelet2 routing graph.
    assert_eq!(routing_edges(&reload_lanelet2(&map)), 2);
}

// 5 -------------------------------------------------------------------------

#[test]
fn a_road_that_splits() {
    let map = scenarios::split();
    assert_exports_cleanly(&map);

    let trunk = LaneId::of_road(&RoadId::new("trunk"), 0);
    let first_hop = map.successors(&trunk);
    assert_eq!(first_hop.len(), 2, "the trunk feeds two connectors");

    let reached: Vec<LaneId> = first_hop
        .iter()
        .flat_map(|lane| map.successors(lane))
        .collect();
    assert!(reached.contains(&LaneId::of_road(&RoadId::new("straight_on"), 0)));
    assert!(reached.contains(&LaneId::of_road(&RoadId::new("slip"), 0)));

    // Two movements, two generated connector roads, and the junction knows both.
    let junction = map.junction(&JunctionId::new("fork")).unwrap();
    assert_eq!(junction.connecting_roads.len(), 2);

    // OpenDRIVE says the same: one junction, two connections.
    let document = reparse_opendrive(&map);
    assert_eq!(document.junction.len(), 1);
    assert_eq!(document.junction[0].connection.len(), 2);
    // Both connections come from the same incoming road — that is what a split is.
    let incoming: Vec<_> = document.junction[0]
        .connection
        .iter()
        .map(|connection| connection.incoming_road.clone())
        .collect();
    assert_eq!(incoming[0], incoming[1]);
    // The junction's number is no road's number. The standard keeps the two
    // spaces apart, but CARLA tells a road's successor apart from a junction by
    // whether a road has that number — and a junction numbered like a road is a
    // road whose lanes lead nowhere.
    let road_ids: Vec<&str> = document.road.iter().map(|road| road.id.as_str()).collect();
    assert!(
        !road_ids.contains(&document.junction[0].id.as_str()),
        "junction {} is numbered like a road",
        document.junction[0].id
    );

    // And Lanelet2 routes through both, two hops each.
    assert_eq!(routing_edges(&reload_lanelet2(&map)), 4);
}

// 6 -------------------------------------------------------------------------

#[test]
fn two_roads_that_merge() {
    let map = scenarios::merge();
    assert_exports_cleanly(&map);

    let onward = LaneId::of_road(&RoadId::new("onward"), 0);
    let feeding = map.predecessors(&onward);
    assert_eq!(feeding.len(), 2, "two connectors feed the onward road");

    let sources: Vec<LaneId> = feeding
        .iter()
        .flat_map(|lane| map.predecessors(lane))
        .collect();
    assert!(sources.contains(&LaneId::of_road(&RoadId::new("main"), 0)));
    assert!(sources.contains(&LaneId::of_road(&RoadId::new("on_ramp"), 0)));

    let document = reparse_opendrive(&map);
    let connections = &document.junction[0].connection;
    assert_eq!(connections.len(), 2);
    // A merge has two *different* incoming roads.
    assert_ne!(connections[0].incoming_road, connections[1].incoming_road);

    assert_eq!(routing_edges(&reload_lanelet2(&map)), 4);
}

// 7 -------------------------------------------------------------------------

#[test]
fn a_four_way_crossroads() {
    let map = scenarios::crossroads();
    assert_exports_cleanly(&map);

    // Four approaches, each of which can reach the other three: twelve movements,
    // each carried by its own connector.
    let junction = map.junction(&JunctionId::new("x")).unwrap();
    assert_eq!(junction.incoming_roads.len(), 4);
    assert_eq!(junction.connecting_roads.len(), 12);

    for arm in ["north", "east", "south", "west"] {
        let approach = LaneId::of_road(&RoadId::new(arm), 0);
        let exits: Vec<LaneId> = map
            .successors(&approach)
            .iter()
            .flat_map(|lane| map.successors(lane))
            .collect();
        assert_eq!(exits.len(), 3, "{arm} should reach the other three arms");
        // Never back the way it came.
        assert!(!exits.contains(&LaneId::of_road(&RoadId::new(arm), 1)));
    }

    let document = reparse_opendrive(&map);
    assert_eq!(document.junction.len(), 1);
    assert_eq!(document.junction[0].connection.len(), 12);

    // Twelve movements, two hops each.
    assert_eq!(routing_edges(&reload_lanelet2(&map)), 24);
}

// 8 -------------------------------------------------------------------------

#[test]
fn a_road_with_a_gradient() {
    let map = scenarios::graded_road();
    assert_exports_cleanly(&map);

    let lane = map.lane(&LaneId::of_road(&RoadId::new("hill"), 0)).unwrap();
    let polyline = lane.centerline.to_polyline(map.metadata.sampling).unwrap();
    let heights: Vec<f64> = polyline.points().iter().map(|point| point.z).collect();
    assert!((heights[0] - 0.0).abs() < 1e-9);
    assert!((heights[heights.len() - 1] - 15.0).abs() < 1e-9);
    // The lane's centreline climbs with the reference line: lane offset is lateral,
    // and never changes a height.
    assert!(heights.windows(2).all(|pair| pair[1] >= pair[0] - 1e-9));

    // OpenDRIVE keeps the climb in its elevation profile, one linear piece per
    // stretch: up, level, up again.
    let document = reparse_opendrive(&map);
    let profile = document.road[0].elevation_profile.as_ref().unwrap();
    assert_eq!(profile.elevation.len(), 3);
    assert!((profile.elevation[0].b - 0.06).abs() < 1e-9);
    assert!(profile.elevation[1].b.abs() < 1e-12);
    assert!((profile.elevation[2].b - 0.09).abs() < 1e-9);

    // Lanelet2 keeps it in the `z` of every node.
    let loaded = reload_lanelet2(&map);
    let heights: Vec<f64> = loaded
        .points
        .all()
        .iter()
        .filter_map(ll2_core::map::as_point)
        .map(|point| point.z())
        .collect();
    assert!(heights.iter().any(|z| z.abs() < 1e-3));
    assert!(heights.iter().any(|z| (z - 15.0).abs() < 1e-3));
}

// 9 -------------------------------------------------------------------------

#[test]
fn the_opendrive_export_is_a_document_a_parser_accepts() {
    for map in [
        scenarios::straight_road(),
        scenarios::bidirectional_road(),
        scenarios::multi_lane_road(),
        scenarios::two_roads_joined(),
        scenarios::split(),
        scenarios::merge(),
        scenarios::crossroads(),
        scenarios::graded_road(),
    ] {
        let document = reparse_opendrive(&map);
        assert_eq!(document.header.rev_major, 1);
        assert_eq!(document.header.rev_minor, 7);
        assert_eq!(document.road.len(), map.roads.len());
        // Every road says how long its plan view is, and every road has a
        // cross-section.
        for road in &document.road {
            assert!(road.length.value > 0.0);
            let section = road.lanes.lane_section.first();
            assert!(section.left.is_some() || section.right.is_some());
        }
        // The coordinate system is recorded, so the file means something on a globe.
        assert!(document
            .header
            .geo_reference
            .as_ref()
            .and_then(|reference| reference.proj.as_ref())
            .is_some_and(|proj| proj.contains("+proj=")));
    }
}

// 10 ------------------------------------------------------------------------

#[test]
fn the_lanelet2_export_is_a_map_autoware_can_read() {
    for map in [
        scenarios::straight_road(),
        scenarios::bidirectional_road(),
        scenarios::multi_lane_road(),
        scenarios::two_roads_joined(),
        scenarios::split(),
        scenarios::merge(),
        scenarios::crossroads(),
        scenarios::graded_road(),
    ] {
        let loaded = reload_lanelet2(&map);
        let drivable = map
            .lanes
            .iter()
            .filter(|lane| lane.lane_type.is_drivable())
            .count();
        assert_eq!(loaded.lanelets.len(), drivable);

        for primitive in loaded.lanelets.all() {
            let lanelet = as_lanelet(&primitive).unwrap();
            let attributes = lanelet.attributes().read();
            // The tags Autoware's loader and its traffic rules look for.
            assert_eq!(attributes["type"].value(), "lanelet");
            assert_eq!(attributes["subtype"].value(), "road");
            assert!(attributes.contains_key("location"));
            assert!(attributes.contains_key("one_way"));
            assert_eq!(attributes["participant:vehicle"].value(), "yes");
            // A lanelet needs boundaries that actually have points in them.
            assert!(lanelet.left_bound().len() >= 2);
            assert!(lanelet.right_bound().len() >= 2);
        }

        // Every node carries the metric position Autoware's parsers read.
        for primitive in loaded.points.all() {
            let point = ll2_core::map::as_point(&primitive).unwrap();
            let attributes = point.attributes().read();
            assert!(attributes.contains_key("local_x"));
            assert!(attributes.contains_key("local_y"));
        }
    }
}

// Semantics ------------------------------------------------------------------

#[test]
fn traffic_control_reaches_the_lanelet2_map() {
    let map = scenarios::controlled_crossroads();
    assert_exports_cleanly(&map);

    let loaded = reload_lanelet2(&map);
    let subtypes: Vec<String> = loaded
        .regulatory_elements
        .all()
        .iter()
        .filter_map(ll2_core::map::as_regulatory_element)
        .map(|element| element.attributes().read()["subtype"].value().to_owned())
        .collect();
    assert!(subtypes.contains(&"traffic_light".to_owned()));
    assert!(subtypes.contains(&"right_of_way".to_owned()));

    // The stop line and the light are ways of their own, and the crosswalk is a
    // lanelet with the subtype Autoware expects.
    let line_types: Vec<String> = loaded
        .line_strings
        .all()
        .iter()
        .filter_map(ll2_core::map::as_linestring)
        .filter_map(|line| {
            line.attributes()
                .read()
                .get("type")
                .map(|value| value.value().to_owned())
        })
        .collect();
    assert!(line_types.contains(&"stop_line".to_owned()));
    assert!(line_types.contains(&"traffic_light".to_owned()));
    assert!(line_types.contains(&"pedestrian_marking".to_owned()));

    let crosswalks = loaded
        .lanelets
        .all()
        .iter()
        .filter_map(as_lanelet)
        .filter(|lanelet| lanelet.attributes().read()["subtype"].value() == "crosswalk")
        .count();
    assert_eq!(crosswalks, 1);
}
