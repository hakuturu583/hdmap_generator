//! A cross-section that changes along a road, end to end.
//!
//! Two things are being checked, and they are different. A lane that *tapers* stays
//! one lane, so it stays one lanelet and one OpenDRIVE lane with a width polynomial.
//! A lane that *ends* changes how many lanes the road has, so a new cross-section
//! begins there — a new `<laneSection>`, and a new pair of lanelets.

use ll2_core::map::as_lanelet;
use roadgen_core::prelude::*;
use roadgen_integration_tests::opendrive_eval::{Position, RoadEvaluator};
use roadgen_integration_tests::scenarios;
use roadgen_integration_tests::{reload_lanelet2, reparse_opendrive, routing_edges};

/// The largest gap between where the OpenDRIVE document puts a lane centre and where
/// the IR does, over each lane's own stations.
fn largest_disagreement(map: &ValidatedMap) -> f64 {
    let document = reparse_opendrive(map);
    let mut worst: f64 = 0.0;
    for (index, road) in map.roads.iter().enumerate() {
        let evaluator = RoadEvaluator::find(&document, &index.to_string()).unwrap();
        for lane in map.lanes_of(&road.id) {
            let lane_id = match lane.side {
                LateralSide::Left => lane.ordinal as i64,
                LateralSide::Right => -(lane.ordinal as i64),
            };
            let vertices = lane.centerline.to_polyline(map.metadata.sampling).unwrap();
            let (start, end) = lane.station_range;
            // The stations the road's geometry was generated at, restricted to this
            // lane's own stretch: one per vertex, and not evenly spaced, because a
            // taper adds stations of its own.
            let stations: Vec<f64> = map
                .vertex_stations(&road.id)
                .unwrap()
                .into_iter()
                .filter(|station| *station >= start - 1e-9 && *station <= end + 1e-9)
                .collect();
            assert_eq!(stations.len(), vertices.len());

            for (station, vertex) in stations.iter().zip(vertices.points()) {
                // A lane section governs up to the next one's station, not including
                // it, so the very last vertex is asked for a hair inside its own
                // section rather than at the boundary it shares with the next.
                let station = station.min(end - 1e-9);
                let from_document = evaluator.lane_center(lane_id, station).unwrap();
                worst = worst.max(from_document.distance_to(Position {
                    x: vertex.x,
                    y: vertex.y,
                    z: vertex.z,
                }));
            }
        }
    }
    worst
}

#[test]
fn a_tapering_lane_stays_one_lane() {
    let map = scenarios::widening_road();
    let road = map.road(&RoadId::new("layby")).unwrap();

    // One cross-section: the shoulder widens and narrows without the road ever
    // having a different number of lanes.
    assert_eq!(road.sections.len(), 1);
    assert_eq!(road.lanes.len(), 2);

    let shoulder = map
        .lanes_of(&road.id)
        .into_iter()
        .find(|lane| lane.lane_type == LaneType::Shoulder)
        .unwrap();
    assert!((shoulder.width_at(0.0) - 2.0).abs() < 1e-9);
    assert!((shoulder.width_at(130.0) - 5.0).abs() < 1e-9);
    assert!((shoulder.width_at(260.0) - 2.0).abs() < 1e-9);

    // The generated boundary follows: the outer edge bulges out and comes back.
    let outer = shoulder
        .right_boundary
        .to_polyline(map.metadata.sampling)
        .unwrap();
    let widest = outer
        .points()
        .iter()
        .map(|point| point.y.abs())
        .fold(0.0_f64, f64::max);
    assert!((widest - 8.5).abs() < 1e-6, "outermost edge at {widest}");
    assert!((outer.first().y + 5.5).abs() < 1e-9);
    assert!((outer.last().y + 5.5).abs() < 1e-9);
}

#[test]
fn a_tapering_lane_reaches_opendrive_as_a_width_polynomial() {
    let map = scenarios::widening_road();
    let document = reparse_opendrive(&map);
    let evaluator = RoadEvaluator::find(&document, "0").unwrap();

    // Still one lane section, with the taper in the lane's own `<width>`.
    assert_eq!(evaluator.section_count(), 1);
    // Lane -2 is the shoulder; its centre moves out as it widens.
    assert!((evaluator.lane_center_offset(-2, 0.0).unwrap() + 4.5).abs() < 1e-9);
    assert!((evaluator.lane_center_offset(-2, 130.0).unwrap() + 6.0).abs() < 1e-9);
    assert!((evaluator.lane_center_offset(-2, 260.0).unwrap() + 4.5).abs() < 1e-9);

    assert!(largest_disagreement(&map) < 1e-6);
}

#[test]
fn a_lane_that_ends_starts_a_new_cross_section() {
    let map = scenarios::lane_drop();
    let road = map.road(&RoadId::new("wide")).unwrap();

    assert_eq!(road.sections.len(), 2);
    assert!((road.sections[0].station - 0.0).abs() < 1e-9);
    assert!((road.sections[1].station - 200.0).abs() < 1e-9);
    assert_eq!(map.lanes_of_section(&road.id, 0).len(), 3);
    assert_eq!(map.lanes_of_section(&road.id, 1).len(), 2);

    // Each section covers its own stretch, and they meet exactly.
    assert_eq!(road.section_range(0).unwrap(), (0.0, 200.0));
    let (start, end) = road.section_range(1).unwrap();
    assert!((start - 200.0).abs() < 1e-9);
    assert!((end - 320.0).abs() < 1e-6);

    // The outer lane tapers away over the second half of its section and then ends.
    let outer = map.lanes_of_section(&road.id, 0)[2];
    assert!((outer.width_at(0.0) - 3.5).abs() < 1e-9);
    assert!((outer.width_at(120.0) - 3.5).abs() < 1e-9);
    assert!((outer.width_at(200.0) - 0.4).abs() < 1e-9);
}

#[test]
fn lanes_that_carry_on_are_connected_and_the_dropped_one_is_not() {
    let map = scenarios::lane_drop();
    let road = RoadId::new("wide");

    let before = map.lanes_of_section(&road, 0);
    let after = map.lanes_of_section(&road, 1);

    // The two inner lanes continue into the second cross-section.
    for (earlier, later) in before.iter().zip(&after) {
        assert_eq!(
            map.successors(&earlier.id),
            vec![later.id.clone()],
            "lane {} should continue into {}",
            earlier.id,
            later.id
        );
    }
    // The outer one does not: that is what "the lane ends" means.
    assert!(map.successors(&before[2].id).is_empty());

    // Lanelet2 finds the same two continuations, from the shared boundary points
    // alone — so a vehicle can route along the road but not off the end of the lane
    // that stops.
    assert_eq!(routing_edges(&reload_lanelet2(&map)), 2);
    assert_eq!(map.connections.len(), 2);
}

#[test]
fn the_lane_drop_reaches_opendrive_as_two_lane_sections() {
    let map = scenarios::lane_drop();
    let document = reparse_opendrive(&map);
    let evaluator = RoadEvaluator::find(&document, "0").unwrap();

    assert_eq!(evaluator.section_count(), 2);
    let stations = evaluator.section_stations();
    assert!((stations[0] - 0.0).abs() < 1e-9);
    assert!((stations[1] - 200.0).abs() < 1e-6);

    // Three lanes before the drop, two after.
    let section_lane_count = |index: usize| {
        document.road[0].lanes.lane_section[index]
            .right
            .as_ref()
            .map(|side| side.lane.len())
            .unwrap_or(0)
    };
    assert_eq!(section_lane_count(0), 3);
    assert_eq!(section_lane_count(1), 2);

    // The outer lane's width really does taper in the document, not just in the IR.
    assert!((evaluator.lane_center_offset(-3, 0.0).unwrap() + 8.75).abs() < 1e-9);
    let near_the_drop = evaluator.lane_center_offset(-3, 199.0).unwrap();
    assert!(
        (near_the_drop + 7.2).abs() < 0.05,
        "the dropped lane's centre is at {near_the_drop} just before it ends"
    );

    assert!(largest_disagreement(&map) < 1e-6);
}

#[test]
fn the_lane_drop_reaches_lanelet2_as_five_lanelets() {
    let map = scenarios::lane_drop();
    let loaded = reload_lanelet2(&map);

    // Three lanelets before the drop and two after: a lane that ends is a lanelet
    // that ends, and the ones that carry on are new lanelets that follow.
    assert_eq!(loaded.lanelets.len(), 5);

    // The shared boundary between two neighbouring lanes is still shared, even
    // though the outer lane tapers and its offsets no longer match anything fixed.
    let lanelets: Vec<_> = loaded
        .lanelets
        .all()
        .into_iter()
        .filter_map(|primitive| as_lanelet(&primitive).cloned())
        .collect();
    let neighbours = lanelets
        .iter()
        .flat_map(|a| {
            lanelets
                .iter()
                .filter(move |b| !a.is_same_view(b) && ll2_core::geometry::lanelet::left_of(a, b))
        })
        .count();
    // Two shared boundaries in the first section, one in the second.
    assert_eq!(neighbours, 3);
}

#[test]
fn a_connector_tapers_between_lanes_of_different_widths() {
    // A narrow slip road feeding a wide carriageway: the connector used to have to
    // pick one width and miss the other. With a width profile it runs between them.
    let mut builder = MapBuilder::new(scenarios::metadata("taper-through"));
    let narrow = builder
        .add_road(
            RoadSpec::line(
                Point3::ORIGIN,
                Point3::new(100.0, 0.0, 0.0),
                vec![scenarios::lane(2.75, Direction::Forward)],
            )
            .unwrap()
            .with_name("narrow"),
        )
        .unwrap();
    let wide = builder
        .add_road(
            RoadSpec::line(
                Point3::new(140.0, 20.0, 0.0),
                Point3::new(260.0, 40.0, 0.0),
                vec![scenarios::lane(4.25, Direction::Forward)],
            )
            .unwrap()
            .with_name("wide"),
        )
        .unwrap();
    let junction = builder.add_junction(Some("j"));
    builder.connect_via(&junction, &narrow, &wide).unwrap();
    let map = builder.finish().unwrap().validate().unwrap();

    let connector = map
        .roads
        .iter()
        .find(|road| road.is_connector())
        .expect("a connector was generated");
    let lane = map.lanes_of(&connector.id)[0];
    assert!((lane.entry_width() - 2.75).abs() < 1e-9);
    assert!((lane.exit_width() - 4.25).abs() < 1e-9);

    // Its boundaries meet both neighbours exactly, which is what the taper is for.
    assert!(roadgen_lanelet2::check(&map).is_empty());
    assert_eq!(routing_edges(&reload_lanelet2(&map)), 2);

    // The connector's reference line still runs down the middle of its lane, so the
    // `<laneOffset>` follows the taper rather than sitting at one width.
    assert!(!connector.lane_offset.is_zero());
    assert!((connector.lane_offset.evaluate(0.0) - 2.75 / 2.0).abs() < 1e-9);
}

#[test]
fn the_new_scenarios_export_cleanly_and_deterministically() {
    for map in [scenarios::lane_drop(), scenarios::widening_road()] {
        assert!(roadgen_opendrive::check(&map).is_empty());
        // The widening road's shoulder is the one lane Lanelet2 has no word for,
        // and the export says so rather than dropping it quietly.
        let problems = roadgen_lanelet2::check(&map);
        let shoulders = map
            .lanes
            .iter()
            .filter(|lane| lane.lane_type == LaneType::Shoulder)
            .count();
        if shoulders == 0 {
            assert!(problems.is_empty(), "{problems:?}");
        } else {
            assert_eq!(problems.len(), 1, "{problems:?}");
            assert!(
                problems[0].contains(&format!("{shoulders} shoulder")),
                "{problems:?}"
            );
        }
        reparse_opendrive(&map);
        reload_lanelet2(&map);
    }
    assert_eq!(
        roadgen_opendrive::to_xml(&scenarios::lane_drop()).unwrap(),
        roadgen_opendrive::to_xml(&scenarios::lane_drop()).unwrap()
    );
    assert_eq!(
        roadgen_lanelet2::to_osm_xml(&scenarios::lane_drop()).unwrap(),
        roadgen_lanelet2::to_osm_xml(&scenarios::lane_drop()).unwrap()
    );
}
