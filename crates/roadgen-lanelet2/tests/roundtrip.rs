//! The exported map has to load as a Lanelet2 map, and the topology the IR holds
//! has to be the topology a Lanelet2 routing graph finds in it. The loader and the
//! routing graph used here are `simple_lanelet2`'s, which know nothing about the IR.

use ll2_core::map::{as_lanelet, LaneletMap};
use ll2_routing::{default_costs, RoutingGraph};
use ll2_traffic_rules::{locations, participants, TrafficRules};
use roadgen_core::prelude::*;
use roadgen_core::validation::ValidatedMap;

fn lanes() -> Vec<LaneSpec> {
    vec![
        LaneSpec::new(PositiveWidth::new(3.5).unwrap(), Direction::Forward),
        LaneSpec::new(PositiveWidth::new(3.5).unwrap(), Direction::Backward),
    ]
}

fn metadata() -> MapMetadata {
    MapMetadata {
        name: Some("roundtrip".into()),
        origin: GeoOrigin::new(35.68, 139.76, 0.0).unwrap(),
        ..MapMetadata::default()
    }
}

/// Two roads, a bend and a climb — the map from the design notes.
fn two_roads() -> ValidatedMap {
    let mut builder = MapBuilder::new(metadata());
    let a = builder
        .add_road(
            RoadSpec::line(
                Point3::new(0.0, 0.0, 10.0),
                Point3::new(100.0, 0.0, 12.0),
                lanes(),
            )
            .unwrap()
            .with_name("a"),
        )
        .unwrap();
    let b = builder
        .add_road(
            RoadSpec::line(
                Point3::new(100.0, 0.0, 12.0),
                Point3::new(200.0, 50.0, 15.0),
                lanes(),
            )
            .unwrap()
            .with_name("b"),
        )
        .unwrap();
    builder.connect(&a, &b).unwrap();
    builder.finish().unwrap().validate().unwrap()
}

fn load(xml: &str, map: &ValidatedMap) -> std::sync::Arc<LaneletMap> {
    let projector = roadgen_lanelet2::projector_for(map).unwrap();
    ll2_io::load_str(xml, projector.as_ref()).expect("a Lanelet2 loader should accept this")
}

/// A routing graph the way a consumer would build one: the standard vehicle rules
/// and the standard cost models.
fn lanelets_of(map: &LaneletMap) -> Vec<ll2_core::lanelet::Lanelet> {
    map.lanelets
        .all()
        .into_iter()
        .filter_map(|primitive| as_lanelet(&primitive).cloned())
        .collect()
}

fn graph_of(loaded: &LaneletMap) -> RoutingGraph {
    let rules = TrafficRules::create(locations::GERMANY, participants::VEHICLE)
        .expect("the standard vehicle rules");
    RoutingGraph::build(loaded, std::sync::Arc::new(rules), &default_costs())
}

#[test]
fn the_exported_map_loads_again() {
    let map = two_roads();
    assert!(roadgen_lanelet2::check(&map).is_empty());
    let loaded = load(&roadgen_lanelet2::to_osm_xml(&map).unwrap(), &map);

    // Four lanes, four lanelets.
    assert_eq!(loaded.lanelets.len(), 4);
    // Three distinct boundary offsets per road, plus a written centreline per lane.
    assert!(loaded.line_strings.len() >= 6);
}

#[test]
fn lanelets_carry_the_tags_autoware_reads() {
    let map = two_roads();
    let loaded = load(&roadgen_lanelet2::to_osm_xml(&map).unwrap(), &map);
    for primitive in loaded.lanelets.all() {
        let lanelet = as_lanelet(&primitive).unwrap();
        let attributes = lanelet.attributes().read();
        assert_eq!(attributes["type"].value(), "lanelet");
        assert_eq!(attributes["subtype"].value(), "road");
        assert_eq!(attributes["location"].value(), "urban");
        assert_eq!(attributes["one_way"].value(), "yes");
        assert_eq!(attributes["participant:vehicle"].value(), "yes");
    }
}

#[test]
fn a_connected_pair_of_lanes_is_a_routable_pair_of_lanelets() {
    let map = two_roads();
    let loaded = load(&roadgen_lanelet2::to_osm_xml(&map).unwrap(), &map);
    let graph = graph_of(&loaded);

    // Every movement the IR holds has to be an edge of the routing graph, which
    // Lanelet2 derives purely from shared boundary points.
    let edges: usize = loaded
        .lanelets
        .all()
        .iter()
        .filter_map(as_lanelet)
        .map(|lanelet| graph.following(lanelet, false).len())
        .sum();
    assert_eq!(edges, map.connections.len());
    assert_eq!(edges, 2);
}

#[test]
fn a_backward_lane_is_a_lanelet_that_runs_backwards() {
    let map = two_roads();
    let exported = roadgen_lanelet2::to_lanelet_map(&map).unwrap();
    let opposing = map
        .lanes
        .iter()
        .find(|lane| lane.direction == Direction::Backward && lane.road.local_name() == "a")
        .unwrap();

    // The lanelet for a backward lane starts where the IR lane's reference line
    // ends, and its left boundary is the IR's right boundary.
    let lanelet = exported
        .lanelets
        .all()
        .into_iter()
        .filter_map(|primitive| as_lanelet(&primitive).cloned())
        .find(|lanelet| {
            lanelet
                .centerline()
                .front()
                .map(|point| {
                    (point.x() - opposing.centerline.end_point().x).abs() < 1e-6
                        && (point.y() - opposing.centerline.end_point().y).abs() < 1e-6
                })
                .unwrap_or(false)
        })
        .expect("a lanelet running against the reference line");

    let left_start = lanelet.left_bound().front().unwrap();
    let expected = opposing.right_boundary.end_point();
    assert!((left_start.x() - expected.x).abs() < 1e-6);
    assert!((left_start.y() - expected.y).abs() < 1e-6);
    assert!((left_start.z() - expected.z).abs() < 1e-6);
}

#[test]
fn opposing_carriageways_share_the_centre_line() {
    let map = two_roads();
    let exported = roadgen_lanelet2::to_lanelet_map(&map).unwrap();
    let lanelets = lanelets_of(&exported);

    // Two lanes running opposite ways share the boundary between them, so their
    // lanelets both have it as their *left* bound — one of them inverted.
    let shared = lanelets.iter().any(|a| {
        lanelets.iter().any(|b| {
            !a.is_same_view(b)
                && a.left_bound().is_same_data(&b.left_bound())
                && a.left_bound().is_inverted() != b.left_bound().is_inverted()
        })
    });
    assert!(shared, "opposing lanes should share the centre linestring");
}

#[test]
fn lanes_side_by_side_are_laterally_adjacent() {
    let mut builder = MapBuilder::new(metadata());
    builder
        .add_road(
            RoadSpec::line(
                Point3::ORIGIN,
                Point3::new(100.0, 0.0, 0.0),
                vec![
                    LaneSpec::new(PositiveWidth::new(3.5).unwrap(), Direction::Forward),
                    LaneSpec::new(PositiveWidth::new(3.25).unwrap(), Direction::Forward),
                ],
            )
            .unwrap()
            .with_name("dual"),
        )
        .unwrap();
    let map = builder.finish().unwrap().validate().unwrap();
    let exported = roadgen_lanelet2::to_lanelet_map(&map).unwrap();
    let lanelets = lanelets_of(&exported);
    assert_eq!(lanelets.len(), 2);

    // The outer lane is to the right of the inner one, which Lanelet2 reads off the
    // linestring they have in common.
    let adjacent = lanelets.iter().any(|a| {
        lanelets
            .iter()
            .any(|b| !a.is_same_view(b) && ll2_core::geometry::lanelet::left_of(a, b))
    });
    assert!(adjacent, "lanes of one carriageway should be neighbours");
}

#[test]
fn elevation_survives_the_round_trip() {
    let map = two_roads();
    let loaded = load(&roadgen_lanelet2::to_osm_xml(&map).unwrap(), &map);
    let heights: Vec<f64> = loaded
        .points
        .all()
        .iter()
        .filter_map(ll2_core::map::as_point)
        .map(|point| point.z())
        .collect();
    // The map climbs from 10 m to 15 m; both ends have to be there.
    assert!(heights.iter().any(|z| (z - 10.0).abs() < 1e-3));
    assert!(heights.iter().any(|z| (z - 15.0).abs() < 1e-3));
}

#[test]
fn a_junction_exports_as_lanelets_that_route_through_it() {
    let mut builder = MapBuilder::new(metadata());
    let approach = builder
        .add_road(
            RoadSpec::line(Point3::ORIGIN, Point3::new(100.0, 0.0, 0.0), lanes())
                .unwrap()
                .with_name("approach"),
        )
        .unwrap();
    let exit = builder
        .add_road(
            RoadSpec::line(
                Point3::new(130.0, 30.0, 0.0),
                Point3::new(200.0, 100.0, 0.0),
                lanes(),
            )
            .unwrap()
            .with_name("exit"),
        )
        .unwrap();
    let junction = builder.add_junction(Some("j0"));
    builder.connect_via(&junction, &approach, &exit).unwrap();
    let map = builder.finish().unwrap().validate().unwrap();
    assert!(roadgen_lanelet2::check(&map).is_empty());

    let loaded = load(&roadgen_lanelet2::to_osm_xml(&map).unwrap(), &map);
    let graph = graph_of(&loaded);
    let edges: usize = loaded
        .lanelets
        .all()
        .iter()
        .filter_map(as_lanelet)
        .map(|lanelet| graph.following(lanelet, false).len())
        .sum();
    // Two movements through the junction, each two hops: four edges.
    assert_eq!(edges, 4);
}

#[test]
fn the_same_map_exports_identically_twice() {
    assert_eq!(
        roadgen_lanelet2::to_osm_xml(&two_roads()).unwrap(),
        roadgen_lanelet2::to_osm_xml(&two_roads()).unwrap()
    );
}
