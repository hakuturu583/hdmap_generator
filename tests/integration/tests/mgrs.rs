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

// UTM ----------------------------------------------------------------------

fn utm_map(origin: GeoOrigin) -> ValidatedMap {
    let mut builder = MapBuilder::new(MapMetadata {
        name: Some("utm".into()),
        origin,
        projection: Projection::Utm,
        ..MapMetadata::default()
    });
    let a = builder
        .add_road(
            RoadSpec::line(
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(400.0, 300.0, 2.0),
                scenarios::two_way(),
            )
            .unwrap()
            .with_name("a"),
        )
        .unwrap();
    let b = builder
        .add_road(
            RoadSpec::line(
                Point3::new(400.0, 300.0, 2.0),
                Point3::new(900.0, 300.0, 4.0),
                scenarios::two_way(),
            )
            .unwrap()
            .with_name("b"),
        )
        .unwrap();
    builder.connect(&a, &b).unwrap();
    builder.finish().unwrap().validate().unwrap()
}

/// The `<geoReference>` of the exported OpenDRIVE, as PROJ key/value pairs.
fn geo_reference(xml: &str) -> std::collections::HashMap<String, String> {
    let start = xml.find("<![CDATA[").expect("a geoReference") + "<![CDATA[".len();
    let end = xml[start..].find("]]>").expect("a closed CDATA") + start;
    xml[start..end]
        .split_whitespace()
        .filter_map(|term| {
            let (key, value) = term.split_once('=')?;
            Some((key.trim_start_matches('+').to_owned(), value.to_owned()))
        })
        .collect()
}

/// The Lanelet2 nodes of an exported map: latitude, longitude, `local_x`, `local_y`.
fn lanelet2_nodes(map: &ValidatedMap) -> Vec<(f64, f64, f64, f64)> {
    let xml = roadgen_lanelet2::to_osm_xml(map).unwrap();
    let (document, _) = ll2_io::osm::parse(&xml).unwrap();
    document
        .nodes
        .values()
        .map(|node| {
            (
                node.lat,
                node.lon,
                node.tags["local_x"].parse().unwrap(),
                node.tags["local_y"].parse().unwrap(),
            )
        })
        .collect()
}

#[test]
fn a_utm_maps_georeference_describes_the_metres_in_the_file() {
    // The file's coordinates are the map's own metres, whatever the projection. A
    // bare `+proj=utm` would say they are eastings and northings and put (0, 0) on
    // the equator; what is written is the zone's transverse Mercator with the
    // origin's easting and northing folded into the false origin, so that reading
    // the file's metres through it lands on the latitudes the Lanelet2 export wrote.
    for (origin, zone, northern) in [
        (GeoOrigin::new(35.68, 139.76, 30.0).unwrap(), 54, true),
        (GeoOrigin::new(-33.87, 151.21, 0.0).unwrap(), 56, false),
    ] {
        let map = utm_map(origin);
        let reference = geo_reference(&roadgen_opendrive::to_xml(&map).unwrap());
        assert_eq!(reference["proj"], "tmerc");
        assert_eq!(reference["k"], "0.9996");
        assert_eq!(
            reference["lon_0"].parse::<f64>().unwrap(),
            ll2_projection::utmups::central_meridian(zone)
        );
        let x_0: f64 = reference["x_0"].parse().unwrap();
        let y_0: f64 = reference["y_0"].parse().unwrap();

        // Every node's latitude and longitude, put through UTM, less the false
        // origin the header states, is the node's own local position.
        let false_northing = if northern { 0.0 } else { 10_000_000.0 };
        for (lat, lon, local_x, local_y) in lanelet2_nodes(&map) {
            let (found_zone, found_northern, easting, northing) =
                ll2_projection::utmups::forward(lat, lon).unwrap();
            assert_eq!((found_zone, found_northern), (zone, northern));
            let x = easting - 500_000.0 + x_0;
            let y = northing - false_northing + y_0;
            assert!(
                (x - local_x).abs() < 1e-6 && (y - local_y).abs() < 1e-6,
                "({lat}, {lon}) reads back as ({x}, {y}), not ({local_x}, {local_y})"
            );
        }
    }
}

#[test]
fn the_carla_package_names_the_origin_in_its_georeference() {
    // CARLA's GNSS sensor takes `+lat_0`/`+lon_0` as where the map's (0, 0) stands
    // and reads nothing else of the string, so the package's `.xodr` is written
    // with the origin there whatever the projection — the exact UTM string would
    // hand it the zone's central meridian.
    let map = utm_map(GeoOrigin::new(35.68, 139.76, 30.0).unwrap());
    let options = roadgen_opendrive::Options {
        geo_reference: Some(roadgen_opendrive::origin_proj_string(&map)),
        ..Default::default()
    };
    let reference = geo_reference(&roadgen_opendrive::to_xml_with(&map, &options).unwrap());
    assert_eq!(reference["lat_0"], "35.68");
    assert_eq!(reference["lon_0"], "139.76");
    // And the exporter's own string for a local-Cartesian map is that same string.
    let local = geo_reference(&roadgen_opendrive::to_xml(&scenarios::straight_road()).unwrap());
    assert_eq!(local["lat_0"], "35.68");
    assert_eq!(local["lon_0"], "139.76");
    assert_eq!(local["k"], "1");
}
