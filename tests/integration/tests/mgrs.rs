//! Exporting a map with MGRS coordinates.
//!
//! Autoware's MGRS projector reads a node's metric position from `local_x`/`local_y`
//! and expects metres within a 100 km grid square. A generated map's coordinates start
//! at its own origin, so these check that the two are reconciled — exactly, per node —
//! and that a map which cannot be expressed that way says so rather than wrapping
//! round the edge of the square.

use roadgen_core::prelude::*;
use roadgen_core::ValidationIssue;
use roadgen_integration_tests::scenarios;
use roadgen_integration_tests::{reload_lanelet2, routing_edges};

fn mgrs_map(origin: GeoOrigin, length: f64) -> ValidatedMap {
    let mut builder = MapBuilder::new(MapMetadata {
        name: Some("mgrs".into()),
        origin,
        projection: Projection::Mgrs,
        ..MapMetadata::default()
    });
    let a = builder
        .add_road(
            RoadSpec::line(
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(length, 0.0, 2.0),
                scenarios::two_way(),
            )
            .unwrap()
            .with_name("a"),
        )
        .unwrap();
    let b = builder
        .add_road(
            RoadSpec::line(
                Point3::new(length, 0.0, 2.0),
                Point3::new(length * 2.0, 0.0, 4.0),
                scenarios::two_way(),
            )
            .unwrap()
            .with_name("b"),
        )
        .unwrap();
    builder.connect(&a, &b).unwrap();
    builder.finish().unwrap().validate().unwrap()
}

/// The `local_x`/`local_y` of every node of an exported map.
fn local_coordinates(map: &ValidatedMap) -> Vec<(f64, f64)> {
    reload_lanelet2(map)
        .points
        .all()
        .iter()
        .filter_map(ll2_core::map::as_point)
        .map(|point| {
            let attributes = point.attributes().read();
            (
                attributes["local_x"].value().parse::<f64>().unwrap(),
                attributes["local_y"].value().parse::<f64>().unwrap(),
            )
        })
        .collect()
}

#[test]
fn an_mgrs_map_reports_grid_coordinates_not_its_own() {
    let origin = GeoOrigin::new(35.68, 139.76, 0.0).unwrap();
    let map = mgrs_map(origin, 200.0);
    assert!(roadgen_lanelet2::check(&map).is_empty());

    let grid = roadgen_lanelet2::grid_for(&map).unwrap().unwrap();
    assert!(grid.code().starts_with("54S"), "square {}", grid.code());

    // The map is built about (0, 0), but its nodes report where they are in the
    // square — tens of kilometres from its corner, not metres from the map's origin.
    let local = local_coordinates(&map);
    assert!(!local.is_empty());
    for (x, y) in &local {
        assert!((0.0..100_000.0).contains(x), "local_x {x}");
        assert!((0.0..100_000.0).contains(y), "local_y {y}");
        assert!(
            *x > 1000.0 && *y > 1000.0,
            "these are grid metres, not map metres"
        );
    }

    // The same map without MGRS reports its own metres, which are small.
    let plain = mgrs_map(origin, 200.0);
    let plain = {
        let mut builder = MapBuilder::new(MapMetadata {
            projection: Projection::LocalCartesian,
            ..plain.metadata.clone()
        });
        builder
            .add_road(
                RoadSpec::line(
                    Point3::ORIGIN,
                    Point3::new(200.0, 0.0, 2.0),
                    scenarios::two_way(),
                )
                .unwrap()
                .with_name("a"),
            )
            .unwrap();
        builder.finish().unwrap().validate().unwrap()
    };
    assert!(local_coordinates(&plain)
        .iter()
        .all(|(x, y)| x.abs() < 1000.0 && y.abs() < 1000.0));
}

#[test]
fn the_grid_position_is_worked_out_per_node() {
    // The map runs four kilometres east. If its grid coordinates were the origin's
    // position shifted by the map's own metres, the far end would be out by metres,
    // because a metre of local east is not a metre of UTM easting. Compare the two
    // ends against what the shift would have given.
    let origin = GeoOrigin::new(35.68, 139.76, 0.0).unwrap();
    let map = mgrs_map(origin, 2_000.0);
    assert!(roadgen_lanelet2::check(&map).is_empty());

    let local = local_coordinates(&map);
    let east_min = local.iter().map(|(x, _)| *x).fold(f64::INFINITY, f64::min);
    let east_max = local
        .iter()
        .map(|(x, _)| *x)
        .fold(f64::NEG_INFINITY, f64::max);

    // Four kilometres of map is not four kilometres of grid: UTM's scale factor is
    // 0.9996 at the central meridian and grows away from it, so the difference is a
    // metre or so — small, and exactly the error a whole-map shift would have kept.
    let spanned = east_max - east_min;
    assert!(
        (spanned - 4_000.0).abs() > 0.2,
        "the grid span should differ from the map span, got {spanned}"
    );
    assert!(
        (spanned - 4_000.0).abs() < 5.0,
        "but only by the scale factor, got {spanned}"
    );
}

#[test]
fn the_latitudes_and_the_topology_are_unaffected() {
    let origin = GeoOrigin::new(35.68, 139.76, 0.0).unwrap();
    let map = mgrs_map(origin, 200.0);

    // MGRS changes what is reported alongside a node, not where the node is: the
    // lanelets and the routing graph come out the same as any other projection.
    let loaded = reload_lanelet2(&map);
    assert_eq!(loaded.lanelets.len(), 4);
    assert_eq!(routing_edges(&loaded), map.connections.len());

    // And the file still carries true latitudes and longitudes near the origin.
    for primitive in loaded.points.all() {
        let point = ll2_core::map::as_point(&primitive).unwrap();
        // Loaded back through the local projection, a node is where the IR put it.
        assert!(point.x().abs() < 500.0);
        assert!(point.y().abs() < 500.0);
    }
}

#[test]
fn a_map_that_leaves_its_square_is_refused_rather_than_wrapped() {
    // 150 km of road cannot fit in a 100 km square however it is placed.
    let origin = GeoOrigin::new(35.68, 139.76, 0.0).unwrap();
    let map = mgrs_map(origin, 75_000.0);

    let problems = roadgen_lanelet2::check(&map);
    assert!(
        problems
            .iter()
            .any(|problem| problem.contains("MGRS square")),
        "{problems:?}"
    );
    // And exporting says so rather than writing a map whose far end has wrapped.
    let error = roadgen_lanelet2::to_osm_xml(&map).unwrap_err();
    assert!(matches!(
        error,
        roadgen_lanelet2::ExportError::GridCrossed { .. }
    ));
}

#[test]
fn an_origin_outside_the_grid_is_refused() {
    let arctic = GeoOrigin::new(88.0, 20.0, 0.0).unwrap();
    let mut builder = MapBuilder::new(MapMetadata {
        origin: arctic,
        projection: Projection::Mgrs,
        ..MapMetadata::default()
    });
    builder
        .add_road(
            RoadSpec::line(
                Point3::ORIGIN,
                Point3::new(100.0, 0.0, 0.0),
                scenarios::two_way(),
            )
            .unwrap()
            .with_name("a"),
        )
        .unwrap();

    // The IR catches the origin before the exporter is ever asked.
    let error = builder.finish().unwrap().validate().unwrap_err();
    assert!(error
        .issues
        .iter()
        .any(|issue| matches!(issue, ValidationIssue::InvalidCoordinateMetadata { .. })));
}

#[test]
fn an_mgrs_map_exports_identically_twice() {
    let origin = GeoOrigin::new(35.68, 139.76, 0.0).unwrap();
    assert_eq!(
        roadgen_lanelet2::to_osm_xml(&mgrs_map(origin, 200.0)).unwrap(),
        roadgen_lanelet2::to_osm_xml(&mgrs_map(origin, 200.0)).unwrap()
    );
}
