//! A plane has one level: where a motorway bridges a street, each road's edges are
//! cut where they cross the other, so neither walls the other off.

use roadgen_core::prelude::*;
use roadgen_core::structure::{BridgeSpec, RoadSpan};
use roadgen_core::FormOfWay;
use roadgen_gpudrive::{MapElement, RoadKind};

fn two_way() -> Vec<LaneSpec> {
    vec![
        LaneSpec::new(PositiveWidth::new(3.5).unwrap(), Direction::Forward),
        LaneSpec::new(PositiveWidth::new(3.5).unwrap(), Direction::Backward),
    ]
}

/// A motorway along x on a bridge over a street along y, and a slip road of a town
/// road type well away from both.
fn bridged() -> ValidatedMap {
    let mut builder = MapBuilder::default();
    let motorway = builder
        .add_road(
            RoadSpec::line(
                Point3::new(-300.0, 0.0, 9.0),
                Point3::new(300.0, 0.0, 9.0),
                two_way(),
            )
            .unwrap()
            .with_name("motorway")
            .with_form(FormOfWay::Motorway),
        )
        .unwrap();
    builder
        .add_road(
            RoadSpec::line(
                Point3::new(0.0, -200.0, 0.0),
                Point3::new(0.0, 200.0, 0.0),
                two_way(),
            )
            .unwrap()
            .with_name("street"),
        )
        .unwrap();
    builder
        .add_road(
            RoadSpec::line(
                Point3::new(0.0, 500.0, 0.0),
                Point3::new(200.0, 500.0, 0.0),
                two_way(),
            )
            .unwrap()
            .with_name("ramp")
            .with_form(FormOfWay::SlipRoad),
        )
        .unwrap();
    builder
        .add_bridge(
            vec![RoadSpan::new(motorway, 250.0, 350.0)],
            BridgeSpec::default(),
        )
        .unwrap();
    builder.finish().unwrap().validate().unwrap()
}

#[test]
fn no_motorway_edge_runs_across_the_street_beneath_it() {
    let map = bridged();
    assert_eq!(map.grade_separations.len(), 1);
    let roads = roadgen_gpudrive::roads::all(&map).unwrap();
    let edges: Vec<_> = roads
        .iter()
        .filter(|road| road.kind == RoadKind::RoadEdge)
        .collect();
    // The motorway's edges lie along y = ±3.5, the street's along x = ±3.5.
    let motorway: Vec<_> = edges
        .iter()
        .filter(|road| {
            road.geometry.iter().all(|p| p.y.abs() < 10.0) && road.geometry[0].y.abs() > 3.0
        })
        .collect();
    let street: Vec<_> = edges
        .iter()
        .filter(|road| road.geometry.iter().all(|p| p.x.abs() < 10.0))
        .collect();
    // Each edge of each road is in two pieces, either side of the crossing.
    assert_eq!(motorway.len(), 4, "{motorway:?}");
    assert_eq!(street.len(), 4, "{street:?}");

    for edge in motorway {
        for pair in edge.geometry.windows(2) {
            let middle = (pair[0].x + pair[1].x) / 2.0;
            assert!(
                middle.abs() >= 3.5,
                "a motorway edge crosses the street at x = {middle}"
            );
        }
        for point in &edge.geometry {
            assert!(point.x.abs() >= 3.5 - 1e-9, "{point:?}");
        }
    }
    for edge in street {
        for point in &edge.geometry {
            assert!(
                point.y.abs() >= 3.5 - 1e-9,
                "a street edge runs under the motorway at y = {}",
                point.y
            );
        }
    }
}

#[test]
fn a_motorway_or_a_slip_road_is_a_freeway_lane() {
    let map = bridged();
    let roads = roadgen_gpudrive::roads::all(&map).unwrap();
    let lanes: Vec<_> = roads
        .iter()
        .filter(|road| road.kind == RoadKind::Lane)
        .collect();
    for lane in lanes {
        let street = lane.geometry.iter().all(|p| p.x.abs() < 10.0);
        let expected = if street {
            MapElement::LaneSurfaceStreet
        } else {
            MapElement::LaneFreeway
        };
        assert_eq!(lane.map_element_id, expected, "{lane:?}");
    }
}

#[test]
fn check_says_the_crossing_is_on_one_level() {
    let notes = roadgen_gpudrive::check(&bridged(), None);
    assert!(
        notes.iter().any(|note| note.contains("grade separations")),
        "{notes:?}"
    );
}
