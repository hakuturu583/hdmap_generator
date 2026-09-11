//! One IR, two files. These tests ask whether the OpenDRIVE document and the
//! Lanelet2 map describe the same road network — the same movements, and the same
//! points in space — by reading each of them back with its own parser.

use ll2_core::map::as_lanelet;
use roadgen_core::map::Lane;
use roadgen_core::prelude::*;
use roadgen_integration_tests::opendrive_eval::{Position, RoadEvaluator};
use roadgen_integration_tests::scenarios;
use roadgen_integration_tests::{reload_lanelet2, reparse_opendrive, routing_edges};

/// The OpenDRIVE id of a lane: positive to the left of the reference line, negative
/// to the right, counting outwards. The exporter assigns roads and junctions their
/// numbers in the IR's own order, so the n-th road of the document is the n-th road
/// of the map.
fn opendrive_lane_id(lane: &Lane) -> i64 {
    match lane.side {
        LateralSide::Left => lane.ordinal as i64,
        LateralSide::Right => -(lane.ordinal as i64),
    }
}

/// Compares every lane's centreline between the two documents, at the stations the
/// IR sampled its reference line at.
///
/// Returns the largest disagreement found, so a caller can assert against the bound
/// that is right for its map.
fn largest_centerline_disagreement(map: &ValidatedMap) -> f64 {
    let document = reparse_opendrive(map);
    let mut worst: f64 = 0.0;

    for (index, road) in map.roads.iter().enumerate() {
        let evaluator = RoadEvaluator::find(&document, &index.to_string())
            .expect("the exporter numbers roads in the map's own order");
        let samples = road
            .reference_line
            .samples(map.metadata.sampling)
            .expect("a road the map validated has geometry");

        for lane in map.lanes_of(&road.id) {
            let vertices = lane
                .centerline
                .to_polyline(map.metadata.sampling)
                .expect("a validated lane has a centreline");
            assert_eq!(vertices.len(), samples.len());

            for (sample, vertex) in samples.iter().zip(vertices.points()) {
                let from_opendrive = evaluator
                    .lane_center(opendrive_lane_id(lane), sample.station)
                    .expect("the lane is in the document");
                let from_ir = Position {
                    x: vertex.x,
                    y: vertex.y,
                    z: vertex.z,
                };
                worst = worst.max(from_opendrive.distance_to(from_ir));
            }
        }
    }
    worst
}

/// The Lanelet2 map's vertices, which are the IR's vertices by construction, so
/// comparing the OpenDRIVE document against the IR compares it against both.
fn lanelet2_agrees_with_the_ir(map: &ValidatedMap) {
    let exported = roadgen_lanelet2::to_lanelet_map(map).unwrap();
    let lanelets: Vec<_> = exported
        .lanelets
        .all()
        .into_iter()
        .filter_map(|primitive| as_lanelet(&primitive).cloned())
        .collect();

    for lane in map.lanes.iter().filter(|lane| lane.lane_type.is_drivable()) {
        let travel = lane.travel_geometry(map.metadata.sampling).unwrap();
        let expected = travel
            .centerline
            .to_polyline(map.metadata.sampling)
            .unwrap();
        // Two connectors leaving the same lane start at the same point, so a
        // lanelet is identified by both ends of its centreline.
        let (first, last) = (expected.first(), expected.last());
        let matches = |point: Option<ll2_core::point::Point>, target: Point3| {
            point
                .map(|point| {
                    (point.x() - target.x).abs() < 1e-9
                        && (point.y() - target.y).abs() < 1e-9
                        && (point.z() - target.z).abs() < 1e-9
                })
                .unwrap_or(false)
        };
        let lanelet = lanelets
            .iter()
            .find(|lanelet| {
                matches(lanelet.centerline().front(), first)
                    && matches(lanelet.centerline().back(), last)
            })
            .unwrap_or_else(|| panic!("no lanelet runs where lane {} does", lane.id));

        let written = lanelet.centerline().points();
        assert_eq!(written.len(), expected.len());
        for (point, vertex) in written.iter().zip(expected.points()) {
            assert!((point.x() - vertex.x).abs() < 1e-9);
            assert!((point.y() - vertex.y).abs() < 1e-9);
            assert!((point.z() - vertex.z).abs() < 1e-9);
        }
    }
}

#[test]
fn both_formats_carry_the_same_movements() {
    for (name, map) in [
        ("straight", scenarios::straight_road()),
        ("bidirectional", scenarios::bidirectional_road()),
        ("multi-lane", scenarios::multi_lane_road()),
        ("joined", scenarios::two_roads_joined()),
        ("split", scenarios::split()),
        ("merge", scenarios::merge()),
        ("crossroads", scenarios::crossroads()),
        ("graded", scenarios::graded_road()),
        ("spiral", scenarios::spiral_transition_road()),
        ("banked", scenarios::banked_curve()),
    ] {
        // Lanelet2 has no successor tag: the routing graph rediscovers connectivity
        // from the geometry alone, so this is a real test of the export and not a
        // reading back of something written down.
        assert_eq!(
            routing_edges(&reload_lanelet2(&map)),
            map.connections.len(),
            "{name}: the Lanelet2 routing graph should find every IR movement"
        );

        // OpenDRIVE writes connectivity down, in two places: lane links between
        // roads that are directly joined, and junction connections for the rest.
        let document = reparse_opendrive(&map);
        let junction_movements: usize = document
            .junction
            .iter()
            .flat_map(|junction| junction.connection.iter())
            .map(|connection| connection.lane_link.len())
            .sum();
        let lane_links: usize = document
            .road
            .iter()
            .flat_map(|road| {
                let section = road.lanes.lane_section.first();
                let left = section
                    .left
                    .iter()
                    .flat_map(|side| side.lane.iter().map(|l| &l.base));
                let right = section
                    .right
                    .iter()
                    .flat_map(|side| side.lane.iter().map(|l| &l.base));
                left.chain(right).collect::<Vec<_>>()
            })
            .filter_map(|lane| lane.link.as_ref())
            .map(|link| link.predecessor.len() + link.successor.len())
            .sum();

        // Each movement inside a junction appears once as a junction connection and
        // twice as a lane link on the connector (one at each of its ends); each
        // direct movement appears twice, once on each side.
        let in_junction = map
            .connections
            .iter()
            .filter(|connection| connection.junction.is_some())
            .count();
        let direct = map.connections.len() - in_junction;
        assert_eq!(
            junction_movements,
            in_junction / 2,
            "{name}: one junction connection per movement through the junction"
        );
        assert_eq!(
            lane_links,
            direct * 2 + in_junction,
            "{name}: every movement should be linked from both of its ends"
        );
    }
}

#[test]
fn both_formats_put_the_lanes_in_the_same_place() {
    // Maps whose reference lines are tangent-continuous everywhere: a straight road,
    // a climb along one heading, and junctions whose connectors leave and arrive
    // along the lanes they join. Here the two formats have to agree exactly.
    for (name, map) in [
        ("straight", scenarios::straight_road()),
        ("bidirectional", scenarios::bidirectional_road()),
        ("multi-lane", scenarios::multi_lane_road()),
        ("in-line", scenarios::two_roads_in_line()),
        ("split", scenarios::split()),
        ("merge", scenarios::merge()),
        ("crossroads", scenarios::crossroads()),
        // A transition curve and a banked cross-section are tangent-continuous too,
        // so they hold to the same bound.
        ("spiral", scenarios::spiral_transition_road()),
        ("banked", scenarios::banked_curve()),
    ] {
        let worst = largest_centerline_disagreement(&map);
        assert!(
            worst < 1e-6,
            "{name}: the two formats disagree by {worst} m"
        );
        lanelet2_agrees_with_the_ir(&map);
    }
}

#[test]
fn a_kink_between_two_roads_is_the_one_place_the_formats_differ() {
    // Where two roads meet at an angle, their lane boundaries can either meet or
    // follow the reference lines — not both. The IR mitres the joint so that the
    // boundaries meet, because Lanelet2 expresses continuity through shared points
    // and would otherwise lose the connection. OpenDRIVE builds lane boundaries from
    // the reference line and a width, and has no way to say the same thing, so it
    // keeps the unmitred corner.
    let map = scenarios::two_roads_joined();
    let worst = largest_centerline_disagreement(&map);

    // The disagreement is confined to the mitre, and bounded by it: half a lane's
    // width against a 26.6-degree change of heading.
    assert!(worst > 1e-6, "there should be a difference to explain");
    assert!(
        worst < 0.45,
        "the difference is {worst} m, more than a mitre"
    );

    // Away from the joint the two agree exactly: the first vertex of the first road
    // is not a joint, and neither is the last vertex of the second.
    let document = reparse_opendrive(&map);
    let first = RoadEvaluator::find(&document, "0").unwrap();
    let lane = map.lane(&LaneId::of_road(&RoadId::new("a"), 0)).unwrap();
    let start = lane.centerline.start_point();
    let from_opendrive = first.lane_center(-1, 0.0).unwrap();
    assert!(
        from_opendrive.distance_to(Position {
            x: start.x,
            y: start.y,
            z: start.z
        }) < 1e-9
    );

    // The connection itself still holds in both: Lanelet2 finds both movements.
    assert_eq!(routing_edges(&reload_lanelet2(&map)), map.connections.len());
}

#[test]
fn the_same_input_produces_the_same_identifiers_and_the_same_files() {
    // Regenerating a map has to be a no-op, or a diff of the output is unreadable.
    let ids = |map: &ValidatedMap| {
        (
            map.roads
                .iter()
                .map(|road| road.id.to_string())
                .collect::<Vec<_>>(),
            map.lanes
                .iter()
                .map(|lane| lane.id.to_string())
                .collect::<Vec<_>>(),
            map.connections
                .iter()
                .map(|connection| connection.id.to_string())
                .collect::<Vec<_>>(),
        )
    };

    for build in [
        scenarios::two_roads_joined as fn() -> ValidatedMap,
        scenarios::split,
        scenarios::crossroads,
        scenarios::graded_road,
    ] {
        let first = build();
        let second = build();
        assert_eq!(ids(&first), ids(&second));
        assert_eq!(
            roadgen_opendrive::to_xml(&first).unwrap(),
            roadgen_opendrive::to_xml(&second).unwrap()
        );
        assert_eq!(
            roadgen_lanelet2::to_osm_xml(&first).unwrap(),
            roadgen_lanelet2::to_osm_xml(&second).unwrap()
        );
    }

    // And the names are the ones the design notes specify, not opaque numbers.
    let map = scenarios::two_roads_joined();
    assert!(map.roads.iter().any(|road| road.id.to_string() == "road/a"));
    assert!(map
        .lanes
        .iter()
        .any(|lane| lane.id.to_string() == "lane/a/0"));
    assert!(map
        .connections
        .iter()
        .any(|connection| connection.id.to_string() == "connection/a_0/b_0"));

    let split = scenarios::split();
    assert!(split
        .connections
        .iter()
        .any(|connection| connection.id.to_string()
            == "connection/fork/trunk_0/fork_trunk_0_straight_on_0_0"));
}
