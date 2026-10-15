//! The IR written as Protocol Buffers and read back is the IR that was written:
//! every scenario map, the motorway with its bridges, tunnels, interchanges and
//! town, the mine with its terrain, and a map read from OpenDRIVE.
//!
//! "The same" is checked twice over: the maps compare equal field for field, and
//! the one read back exports to the same OpenDRIVE and Lanelet2 documents byte for
//! byte, which is what a caller who keeps the document to export later relies on.

use std::sync::LazyLock;

use roadgen_core::map::TrafficHandedness;
use roadgen_core::validation::ValidatedMap;
use roadgen_integration_tests::{highway, mine, scenarios};
use roadgen_proto::{v1, WriteOptions};

/// The motorway, built once: it is the largest map in the suite, and two tests
/// read it.
static HIGHWAY: LazyLock<ValidatedMap> = LazyLock::new(highway::highway);

/// Writes `map` with and without curve samples, reads both back, and checks that
/// each is the map that was written.
fn round_trip(name: &str, map: &ValidatedMap) {
    let opendrive = roadgen_opendrive::to_xml(map).unwrap();
    let lanelet2 = roadgen_lanelet2::to_osm_xml(map).unwrap();
    for curve_samples in [true, false] {
        let options = WriteOptions { curve_samples };
        let bytes = roadgen_proto::to_bytes_with(map, &options)
            .unwrap_or_else(|error| panic!("{name}: the map should write: {error}"));
        let back = roadgen_proto::from_bytes(&bytes)
            .unwrap_or_else(|error| panic!("{name}: the document should read: {error}"))
            .validate()
            .unwrap_or_else(|error| panic!("{name}: what was read should validate: {error}"));
        assert!(
            back.as_map() == map.as_map(),
            "{name}: the map read back differs from the one written"
        );
        assert!(
            roadgen_opendrive::to_xml(&back).unwrap() == opendrive,
            "{name}: the OpenDRIVE export changed"
        );
        assert!(
            roadgen_lanelet2::to_osm_xml(&back).unwrap() == lanelet2,
            "{name}: the Lanelet2 export changed"
        );
    }
}

#[test]
fn every_scenario_comes_back_as_it_was_written() {
    let maps = [
        ("straight_road", scenarios::straight_road()),
        ("bidirectional_road", scenarios::bidirectional_road()),
        ("multi_lane_road", scenarios::multi_lane_road()),
        ("two_roads_joined", scenarios::two_roads_joined()),
        ("split", scenarios::split()),
        ("merge", scenarios::merge()),
        ("crossroads", scenarios::crossroads()),
        ("graded_road", scenarios::graded_road()),
        ("bent_polyline", scenarios::bent_polyline()),
        (
            "spiral_transition_road",
            scenarios::spiral_transition_road(),
        ),
        ("banked_curve", scenarios::banked_curve()),
        ("lane_drop", scenarios::lane_drop()),
        ("lane_type_change", scenarios::lane_type_change()),
        ("backward_lane_drop", scenarios::backward_lane_drop()),
        ("widening_road", scenarios::widening_road()),
        ("controlled_crossroads", scenarios::controlled_crossroads()),
        (
            "sidewalk_crossroads",
            scenarios::sidewalk_crossroads(TrafficHandedness::LeftHand),
        ),
        (
            "signalised_crosswalk_crossroads",
            scenarios::signalised_crosswalk_crossroads(TrafficHandedness::RightHand).0,
        ),
    ];
    for (name, map) in &maps {
        round_trip(name, map);
    }
}

#[test]
fn the_motorway_comes_back_with_its_structures_interchanges_and_town() {
    let map = &*HIGHWAY;
    // What makes this map the one to check: every kind of feature beside the
    // roads is in it.
    assert!(map.structures.iter().any(|s| s.kind.bridge().is_some()));
    assert!(map.structures.iter().any(|s| s.kind.tunnel().is_some()));
    assert!(!map.grade_separations.is_empty());
    assert!(!map.interchanges.is_empty());
    assert!(!map.buildings.is_empty());
    assert!(!map.land_covers.is_empty());
    round_trip("highway", map);
}

#[test]
fn the_mine_comes_back_with_its_terrain() {
    let map = mine::mine();
    assert!(!map.sites.is_empty());
    assert!(!map.tin_surfaces.is_empty());
    assert!(!map.tin_breaklines.is_empty());
    assert!(!map.land_covers.is_empty());
    round_trip("mine", &map);
}

#[test]
fn a_map_read_from_opendrive_comes_back_as_it_was_read() {
    let xml = roadgen_opendrive::to_xml(&scenarios::controlled_crossroads()).unwrap();
    let read = roadgen_opendrive::from_xml(&xml)
        .unwrap()
        .map
        .validate()
        .unwrap();
    round_trip("read", &read);
}

/// Every curve in a written document, outermost only, with where it was found.
fn curves(message: &v1::Map) -> Vec<(String, &v1::Curve)> {
    let mut found = Vec::new();
    for road in &message.roads {
        found.push((road.id.clone(), road.reference_line.as_ref().unwrap()));
    }
    for lane in &message.lanes {
        for curve in [&lane.left_boundary, &lane.right_boundary, &lane.centerline] {
            found.push((lane.id.clone(), curve.as_ref().unwrap()));
        }
    }
    found
}

#[test]
fn every_curve_carries_the_vertices_an_importer_needs() {
    let map = &*HIGHWAY;
    let message = roadgen_proto::to_message(map).unwrap();
    let config = map.metadata.sampling;

    let mut kinds = std::collections::HashSet::new();
    for (owner, curve) in curves(&message) {
        kinds.insert(std::mem::discriminant(curve.kind.as_ref().unwrap()));

        // The samples are the polyline every polyline export gets: the curve's own
        // vertices at the map's resolution, in order, ending where it ends.
        assert!(curve.samples.len() >= 2, "{owner}: too few samples");
        assert!(
            curve
                .samples
                .windows(2)
                .all(|pair| pair[0].station < pair[1].station),
            "{owner}: samples should ascend by station"
        );
        for sample in &curve.samples {
            let tangent = sample.tangent.as_ref().unwrap();
            let length =
                (tangent.x * tangent.x + tangent.y * tangent.y + tangent.z * tangent.z).sqrt();
            assert!(
                (length - 1.0).abs() < 1e-9,
                "{owner}: a tangent is not unit"
            );
        }
    }
    // Several kinds of curve occur, so the samples above stood in for more than
    // polylines.
    assert!(kinds.len() >= 3, "{kinds:?}");

    for road in map.roads.iter() {
        let written = message
            .roads
            .iter()
            .find(|candidate| candidate.id == road.id.as_str())
            .unwrap();
        let samples = &written.reference_line.as_ref().unwrap().samples;
        let expected = road.reference_line.samples(config).unwrap();
        assert_eq!(samples.len(), expected.len(), "{}", road.id);
        let last = samples.last().unwrap().point.as_ref().unwrap();
        let end = road.reference_line.end_point();
        assert!(
            (last.x - end.x).abs() < 1e-9 && (last.y - end.y).abs() < 1e-9,
            "{}: the samples should end where the road does",
            road.id
        );
    }

    // The pieces of a composite carry none: the composite's own are the curve's.
    for road in &message.roads {
        if let Some(v1::curve::Kind::Composite(composite)) =
            &road.reference_line.as_ref().unwrap().kind
        {
            assert!(composite
                .pieces
                .iter()
                .all(|piece| piece.samples.is_empty()));
        }
    }
}
