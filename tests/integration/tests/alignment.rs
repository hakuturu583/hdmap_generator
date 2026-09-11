//! Transition curves and banked cross-sections, end to end.
//!
//! A clothoid and a superelevation profile are only useful if they survive to both
//! files, so these check the IR, then the OpenDRIVE document read back by its own
//! parser, then the Lanelet2 map read back by its own loader.

use ll2_core::map::as_lanelet;
use opendrive::road::geometry::geometry_type::GeometryType;
use roadgen_core::prelude::*;
use roadgen_core::ValidationIssue;
use roadgen_integration_tests::opendrive_eval::{Position, RoadEvaluator};
use roadgen_integration_tests::scenarios;
use roadgen_integration_tests::{reload_lanelet2, reparse_opendrive};

/// The largest gap between where the OpenDRIVE document puts a lane centre and where
/// the IR does, at the stations the IR generated its vertices at.
fn largest_disagreement(map: &ValidatedMap) -> f64 {
    let document = reparse_opendrive(map);
    let mut worst: f64 = 0.0;
    for (index, road) in map.roads.iter().enumerate() {
        let evaluator = RoadEvaluator::find(&document, &index.to_string()).unwrap();
        let samples = road.reference_line.samples(map.metadata.sampling).unwrap();
        for lane in map.lanes_of(&road.id) {
            let lane_id = match lane.side {
                LateralSide::Left => lane.ordinal as i64,
                LateralSide::Right => -(lane.ordinal as i64),
            };
            let vertices = lane.centerline.to_polyline(map.metadata.sampling).unwrap();
            for (sample, vertex) in samples.iter().zip(vertices.points()) {
                let from_document = evaluator.lane_center(lane_id, sample.station).unwrap();
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
fn a_transition_curve_reaches_opendrive_as_a_spiral() {
    let map = scenarios::spiral_transition_road();
    let document = reparse_opendrive(&map);
    let geometry = &document.road[0].plan_view.geometry;

    // Five pieces, in the order the alignment was written, each surviving as the
    // element OpenDRIVE has for it rather than as a chain of straight segments.
    assert_eq!(geometry.len(), 5);
    let kinds: Vec<&str> = geometry
        .iter()
        .map(|entry| match entry.r#type {
            GeometryType::Line(_) => "line",
            GeometryType::Arc(_) => "arc",
            GeometryType::Spiral(_) => "spiral",
            _ => "other",
        })
        .collect();
    assert_eq!(kinds, ["line", "spiral", "arc", "spiral", "line"]);

    // The transitions run from straight to the bend's curvature and back.
    let radius = 120.0;
    let GeometryType::Spiral(entering) = &geometry[1].r#type else {
        panic!("the second piece is the entry transition")
    };
    assert!(entering.curvature_start.value.abs() < 1e-12);
    assert!((entering.curvature_end.value - 1.0 / radius).abs() < 1e-12);
    let GeometryType::Spiral(leaving) = &geometry[3].r#type else {
        panic!("the fourth piece is the exit transition")
    };
    assert!((leaving.curvature_start.value - 1.0 / radius).abs() < 1e-12);
    assert!(leaving.curvature_end.value.abs() < 1e-12);

    // The pieces are laid end to end along `s`, covering the road exactly.
    let mut station = 0.0;
    for entry in geometry.iter() {
        assert!((entry.s.value - station).abs() < 1e-9);
        station += entry.length.value;
    }
    assert!((station - document.road[0].length.value).abs() < 1e-6);
}

#[test]
fn the_two_formats_agree_along_a_spiral() {
    // The document says "spiral" and the Lanelet2 map says "these vertices". They
    // have to be the same curve: the evaluator walks the spiral from the document's
    // own curvStart/curvEnd, independently of how the IR generated it.
    let map = scenarios::spiral_transition_road();
    let worst = largest_disagreement(&map);
    assert!(worst < 1e-6, "the two formats disagree by {worst} m");

    // And the Lanelet2 side really did get the curve, not a straight line: the
    // lanelet boundaries bend.
    let loaded = reload_lanelet2(&map);
    let lanelet = loaded
        .lanelets
        .all()
        .into_iter()
        .filter_map(|primitive| as_lanelet(&primitive).cloned())
        .next()
        .unwrap();
    let vertices: Vec<[f64; 3]> = lanelet
        .left_bound()
        .points()
        .iter()
        .map(|point| point.xyz())
        .collect();
    let distance = |a: [f64; 3], b: [f64; 3]| {
        ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2) + (a[2] - b[2]).powi(2)).sqrt()
    };
    let along: f64 = vertices
        .windows(2)
        .map(|pair| distance(pair[0], pair[1]))
        .sum();
    let chord = distance(vertices[0], vertices[vertices.len() - 1]);
    assert!(
        along > chord + 1.0,
        "a boundary that follows the bend is longer than its chord: {along} vs {chord}"
    );
}

#[test]
fn banking_lifts_one_side_of_the_road_and_drops_the_other() {
    let map = scenarios::banked_curve();
    let road = map.road(&RoadId::new("bend")).unwrap();
    assert!(!road.superelevation.is_zero());

    let lanes = map.lanes_of(&road.id);
    let left = lanes
        .iter()
        .find(|lane| lane.side == LateralSide::Left)
        .unwrap();
    let right = lanes
        .iter()
        .find(|lane| lane.side == LateralSide::Right)
        .unwrap();

    let config = map.metadata.sampling;
    let left_edge = left.left_boundary.to_polyline(config).unwrap();
    let right_edge = right.right_boundary.to_polyline(config).unwrap();
    let centre = left.right_boundary.to_polyline(config).unwrap();

    // At the start the road is flat: both edges sit at the height of the centre.
    assert!((left_edge.first().z - centre.first().z).abs() < 1e-9);
    assert!((right_edge.first().z - centre.first().z).abs() < 1e-9);

    // Through the bend it is rolled by -0.06 rad, so the left edge is below the
    // centre by 3.5·sin(0.06) and the right edge is above it by the same.
    let rise = 3.5 * 0.06_f64.sin();
    let last = left_edge.len() - 1;
    assert!((centre.points()[last].z - left_edge.points()[last].z - rise).abs() < 1e-6);
    assert!((right_edge.points()[last].z - centre.points()[last].z - rise).abs() < 1e-6);

    // The lane is still its full width across the banked surface, not its shadow.
    let width = left_edge.points()[last].distance_to(centre.points()[last]);
    assert!((width - 3.5).abs() < 1e-9);
}

#[test]
fn superelevation_reaches_opendrive_and_lanelet2_alike() {
    let map = scenarios::banked_curve();

    // OpenDRIVE keeps the roll in its own element, as the same piecewise polynomial
    // the IR holds.
    let document = reparse_opendrive(&map);
    let profile = document.road[0]
        .lateral_profile
        .as_ref()
        .expect("a banked road has a lateral profile");
    assert_eq!(profile.super_elevation.len(), 4);
    assert!(profile.super_elevation[0].a.abs() < 1e-12);
    assert!((profile.super_elevation[2].a + 0.06).abs() < 1e-12);

    // Lanelet2 has no notion of superelevation; it has vertices, and the roll is
    // already in their heights. So the two agree without either format converting
    // anything.
    let worst = largest_disagreement(&map);
    assert!(worst < 1e-6, "the two formats disagree by {worst} m");
}

#[test]
fn a_road_banked_past_a_right_angle_is_refused() {
    let mut builder = MapBuilder::new(scenarios::metadata("absurd"));
    builder
        .add_road(
            RoadSpec::line(
                Point3::ORIGIN,
                Point3::new(100.0, 0.0, 0.0),
                scenarios::two_way(),
            )
            .unwrap()
            .with_name("wall")
            // 60 degrees: a bank steeper than any road, and steep enough that the
            // cross-section stops describing a surface a vehicle can sit on.
            .with_superelevation(Poly3Profile::constant(1.05)),
        )
        .unwrap();

    let error = builder.finish().unwrap().validate().unwrap_err();
    assert!(error
        .issues
        .iter()
        .any(|issue| matches!(issue, ValidationIssue::ImplausibleSuperelevation { .. })));
}

#[test]
fn the_new_geometry_still_exports_cleanly_and_deterministically() {
    for map in [
        scenarios::spiral_transition_road(),
        scenarios::banked_curve(),
    ] {
        assert!(roadgen_opendrive::check(&map).is_empty());
        assert!(roadgen_lanelet2::check(&map).is_empty());
        reparse_opendrive(&map);
        reload_lanelet2(&map);
    }

    assert_eq!(
        roadgen_opendrive::to_xml(&scenarios::spiral_transition_road()).unwrap(),
        roadgen_opendrive::to_xml(&scenarios::spiral_transition_road()).unwrap()
    );
    assert_eq!(
        roadgen_lanelet2::to_osm_xml(&scenarios::banked_curve()).unwrap(),
        roadgen_lanelet2::to_osm_xml(&scenarios::banked_curve()).unwrap()
    );
}
