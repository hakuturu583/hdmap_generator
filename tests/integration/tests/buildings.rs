//! Generating a town, and getting it out again.
//!
//! Three questions, and they are separate. Whether the generator puts buildings where
//! a town has them — beside the streets, clear of the road surface, clear of each
//! other, and the same every time. Whether what it puts there is a *solid*: parts
//! that stand on each other, walls that rise, roofs that close. And whether the two
//! formats that can carry a building carry *this* one: these tests read the written
//! file back through the format's own parser and compare what comes out against what
//! went in, which is the only comparison worth making.

use std::collections::HashSet;

use opendrive::object::corner::Corner;
use opendrive::object::orientation::ObjectType;
use roadgen_buildings::{Rules, PRESETS};
use roadgen_core::buildings::RoofShape;
use roadgen_core::prelude::*;
use roadgen_integration_tests::{redraw_opendrive, reload_osm, reparse_opendrive, scenarios};
use roadgen_viewer::{Kind, Shape};
use uom::si::length::meter;

/// Every shape the IR can name, so a tag read back out of a file can be checked
/// against the vocabulary rather than against a hand-written list of strings.
const ROOF_SHAPES: [RoofShape; 5] = [
    RoofShape::Flat,
    RoofShape::Skillion,
    RoofShape::Gabled,
    RoofShape::Hipped,
    RoofShape::Pyramidal,
];

/// A crossroads whose arms are long enough to have frontages worth building on.
fn town(rules: &Rules) -> ValidatedMap {
    let mut map = scenarios::crossroads_builder("town", 260.0)
        .finish()
        .expect("the town should build");
    roadgen_buildings::generate(&mut map, rules).expect("the rules should derive");
    map.validate().expect("a generated town should validate")
}

#[test]
fn a_crossroads_gets_a_town_around_it() {
    let map = town(&Rules::default());
    assert!(map.buildings.len() > 20, "{}", map.buildings.len());

    // Every arm builds on both of its frontages, and none of the four connectors
    // through the junction does: the land beside a movement is the junction.
    let frontages: HashSet<String> = map
        .buildings
        .iter()
        .filter_map(|building| {
            let mut parts = building.id.as_str().splitn(4, '/');
            let (_, road, side) = (parts.next()?, parts.next()?, parts.next()?);
            Some(format!("{road}/{side}"))
        })
        .collect();
    assert_eq!(frontages.len(), 8, "{frontages:?}");
    for arm in ["north", "east", "south", "west"] {
        for side in ["left", "right"] {
            assert!(frontages.contains(&format!("{arm}/{side}")), "{arm}/{side}");
        }
    }
}

#[test]
fn nothing_is_built_over_the_junction() {
    let map = town(&Rules::default());
    // The four arms stop 14 m short of the centre and the junction fills the gap, so
    // anything within that of the middle would be standing in it.
    for part in map.building_parts.iter() {
        for point in part.solid.footprint.points() {
            let from_centre = (point.x * point.x + point.y * point.y).sqrt();
            assert!(
                from_centre > 14.0,
                "{} reaches the junction at {point:?}",
                part.id
            );
        }
    }
}

#[test]
fn every_building_is_a_solid_made_of_parts() {
    let map = town(&Rules::default());
    assert!(map.building_parts.len() >= map.buildings.len());

    for building in map.buildings.iter() {
        let parts = map.parts_of(&building.id);
        assert!(!parts.is_empty(), "{} has no parts", building.id);
        assert_eq!(parts.len(), building.parts.len());
        // The composition is recorded from both ends and the two agree.
        assert!(parts.iter().all(|part| part.building == building.id));

        // Which road it faces is a relationship the IR holds, not something an
        // exporter has to work out.
        let frontage = building
            .frontage
            .as_ref()
            .expect("a generated building faces a road");
        assert!(map.roads.contains(&frontage.road));

        for part in &parts {
            let solid = &part.solid;
            assert!(solid.wall_height > 0.0, "{}", part.id);
            assert!(part.levels >= 1, "{}", part.id);
            // The compact form means a closed surface, and this is it.
            assert!(solid.is_closed(), "{} does not close", part.id);
            let shell = solid.shell();
            // A base, a wall per side, and at least one roof face.
            assert!(
                shell.len() >= solid.footprint.len() + 2,
                "{} has {} faces over {} sides",
                part.id,
                shell.len(),
                solid.footprint.len()
            );
            assert!(shell
                .iter()
                .flatten()
                .all(|point| point.z >= solid.base_height() - 1e-6
                    && point.z <= solid.top_height() + 1e-6));
        }
    }
}

#[test]
fn a_town_with_pitched_roofs_says_which_way_the_ridges_run() {
    let map = town(&Rules::default());
    let pitched: Vec<_> = map
        .building_parts
        .iter()
        .filter(|part| part.solid.roof.shape != RoofShape::Flat)
        .collect();
    assert!(
        !pitched.is_empty(),
        "the default rules should pitch some roofs"
    );

    for part in &pitched {
        let roof = part.solid.roof;
        assert!(roof.height > 0.0, "{}", part.id);
        // A ridge runs along the street or across it, because the lots do.
        if roof.shape.is_directed() {
            let along_or_across = (roof.direction % std::f64::consts::FRAC_PI_2).abs();
            assert!(
                along_or_across < 0.05
                    || (std::f64::consts::FRAC_PI_2 - along_or_across).abs() < 0.05,
                "{} has a ridge at {} rad",
                part.id,
                roof.direction
            );
        }
    }
}

#[test]
fn every_preset_builds_a_town_that_survives_every_export() {
    for (name, _) in PRESETS {
        let map = town(&Rules::preset(name).unwrap());
        assert!(!map.buildings.is_empty(), "{name} generated nothing");

        // The two formats that can hold a building.
        roadgen_opendrive::to_xml(&map).unwrap_or_else(|e| panic!("{name}: {e}"));
        roadgen_osm::to_xml(&map).unwrap_or_else(|e| panic!("{name}: {e}"));
        // And the ones that cannot, which have to say so rather than fail.
        for warnings in [
            roadgen_lanelet2::check(&map),
            roadgen_sumo::check(&map),
            roadgen_clipgt::check(&map, None),
            roadgen_gpudrive::check(&map, None),
        ] {
            assert!(
                warnings.iter().any(|warning| warning.contains("buildings")),
                "{name}: a format that drops the buildings should say so: {warnings:?}"
            );
        }
    }
}

#[test]
fn an_osm_reader_gets_the_footprints_back() {
    let map = town(&Rules::default());
    let osm = reload_osm(&map);

    let ways: Vec<_> = osm
        .document
        .ways
        .values()
        .filter(|way| way.tags.contains_key("building"))
        .collect();
    assert_eq!(ways.len(), map.buildings.len());

    for way in &ways {
        // A building is a *closed* way: first node and last are the same one.
        assert!(way.nodes.len() >= 4, "{:?}", way.nodes);
        assert_eq!(way.nodes.first(), way.nodes.last());
        assert!(way.tags.contains_key("height"));
    }

    // Every part of every building comes back, as the building's own way when it has
    // one part and as a `building:part` way when it has more.
    let parts: Vec<_> = osm
        .document
        .ways
        .values()
        .filter(|way| way.tags.contains_key("building:part"))
        .collect();
    let multi: Vec<_> = map
        .buildings
        .iter()
        .filter(|building| building.parts.len() > 1)
        .collect();
    assert_eq!(
        parts.len(),
        multi
            .iter()
            .map(|building| building.parts.len())
            .sum::<usize>()
    );

    // Each outline's corners come back where they were put, to the centimetre an OSM
    // latitude is worth.
    let landed = |wanted: &[Point3]| {
        osm.document.ways.values().any(|way| {
            way.nodes.len() == wanted.len() + 1
                && wanted.iter().all(|point| {
                    way.nodes.iter().any(|node| {
                        osm.metres
                            .get(node)
                            .is_some_and(|read| read.horizontal_distance_to(*point) < 0.05)
                    })
                })
        })
    };
    for part in map.building_parts.iter() {
        assert!(
            landed(part.solid.footprint.points()),
            "{} did not land in the file",
            part.id
        );
    }
}

#[test]
fn a_pitched_roof_reaches_openstreetmap_as_a_pitched_roof() {
    let map = town(&Rules::default());
    let osm = reload_osm(&map);

    let pitched = map
        .building_parts
        .iter()
        .filter(|part| part.solid.roof.shape != RoofShape::Flat)
        .count();
    let tagged = osm
        .document
        .ways
        .values()
        .filter(|way| way.tags.contains_key("roof:shape"))
        .count();
    assert!(pitched > 0);
    assert_eq!(tagged, pitched);

    // The shape, the rise and the bearing all survive, and the bearing is the one a
    // compass would read rather than the angle the IR holds.
    for way in osm.document.ways.values() {
        let Some(shape) = way.tags.get("roof:shape") else {
            continue;
        };
        assert!(
            ROOF_SHAPES.iter().any(|known| known.as_str() == shape),
            "{shape:?} is not a roof shape"
        );
        let height: f64 = way
            .tags
            .get("roof:height")
            .expect("a rise")
            .parse()
            .unwrap();
        assert!(height > 0.0);
        if let Some(direction) = way.tags.get("roof:direction") {
            let bearing: f64 = direction.parse().unwrap();
            assert!((0.0..360.0).contains(&bearing), "{bearing}");
        }
    }
}

#[test]
fn an_opendrive_reader_gets_the_footprints_back() {
    let map = town(&Rules::default());
    let document = reparse_opendrive(&map);

    let objects: Vec<_> = document
        .road
        .iter()
        .filter_map(|road| road.objects.as_ref())
        .flat_map(|objects| objects.object.iter())
        .filter(|object| object.r#type == Some(ObjectType::Building))
        .collect();
    assert_eq!(objects.len(), map.buildings.len());

    for object in &objects {
        let outlines = object.outlines.as_ref().expect("a building has outlines");
        let name = object.name.clone().expect("a building object is named");
        let building = map
            .buildings
            .iter()
            .find(|building| building.id.to_string() == name)
            .unwrap_or_else(|| panic!("no building called {name}"));
        let parts = map.parts_of(&building.id);

        // One outline per part, in the order the building lists them.
        assert_eq!(outlines.outline.len(), parts.len(), "{name}");
        for (outline, part) in outlines.outline.iter().zip(&parts) {
            assert_eq!(outline.closed, Some(true));
            assert!(outline.choice.len() >= 3);
            // Local corners, so that a wall beside a bend stays straight.
            assert!(outline
                .choice
                .iter()
                .all(|corner| matches!(corner, Corner::Local(_))));

            let corners: Vec<(f64, f64, f64, f64)> = outline
                .choice
                .iter()
                .map(|corner| match corner {
                    Corner::Local(local) => (
                        local.u.get::<meter>(),
                        local.v.get::<meter>(),
                        local.z.get::<meter>(),
                        local.height.get::<meter>(),
                    ),
                    Corner::Road(_) => unreachable!("checked above"),
                })
                .collect();

            // The outline encloses the area the part's outline did — which is the
            // thing `cornerRoad` would have lost beside a bend.
            let mut twice_area = 0.0;
            for index in 0..corners.len() {
                let (ax, ay, ..) = corners[index];
                let (bx, by, ..) = corners[(index + 1) % corners.len()];
                twice_area += ax * by - bx * ay;
            }
            assert!(
                (twice_area.abs() / 2.0 - part.solid.footprint.area()).abs() < 0.01,
                "{name}: {} m² written for a {} m² outline",
                twice_area.abs() / 2.0,
                part.solid.footprint.area()
            );
            // Each corner's height is the whole of what stands over it, walls and
            // roof together, because an outline has nowhere to put a ridge.
            let expected = part.solid.wall_height + part.solid.roof.height;
            assert!(
                corners
                    .iter()
                    .all(|(.., height)| (height - expected).abs() < 1e-6),
                "{name}: heights {corners:?} for {expected} m"
            );
        }
        // Parts at different heights are written at different heights.
        if parts.len() > 1 {
            let bases: Vec<f64> = outlines
                .outline
                .iter()
                .flat_map(|outline| outline.choice.iter())
                .map(|corner| match corner {
                    Corner::Local(local) => local.z.get::<meter>(),
                    Corner::Road(_) => unreachable!(),
                })
                .collect();
            let low = bases.iter().copied().fold(f64::INFINITY, f64::min);
            let high = bases.iter().copied().fold(f64::NEG_INFINITY, f64::max);
            assert!(
                high > low + 1e-6,
                "{name}: every part written at one height"
            );
        }
    }
}

#[test]
fn the_viewer_draws_the_town_the_file_describes() {
    let map = town(&Rules::default());
    let drawing = redraw_opendrive(&map);

    let areas: Vec<_> = drawing
        .shapes
        .iter()
        .filter(|shape| shape.kind() == Kind::Building)
        .collect();
    // Every part, not only every building: a plan view shows the massing.
    assert_eq!(areas.len(), map.building_parts.len());

    // The picture is of the file, so an outline drawn from it should land where the
    // map put it — within the metre the reference line is re-evaluated to.
    for part in map.building_parts.iter() {
        let centre = part.solid.footprint.centroid();
        assert!(
            areas.iter().any(|shape| {
                let Shape::Area { points, .. } = shape else {
                    return false;
                };
                let count = points.len() as f64;
                let x = points.iter().map(|point| point.x).sum::<f64>() / count;
                let y = points.iter().map(|point| point.y).sum::<f64>() / count;
                (x - centre.x).hypot(y - centre.y) < 1.0
            }),
            "{} was not drawn where it stands",
            part.id
        );
    }
    assert!(drawing
        .notes
        .iter()
        .any(|note| note.contains("building parts, drawn from the outlines")));
}

#[test]
fn a_map_with_no_buildings_exports_and_draws_exactly_as_it_did_before() {
    let plain = scenarios::crossroads();
    let xml = roadgen_opendrive::to_xml(&plain).expect("the map should export");
    assert!(!xml.contains("building"));
    assert!(redraw_opendrive(&plain)
        .shapes
        .iter()
        .all(|shape| shape.kind() != Kind::Building));

    let osm = reload_osm(&plain);
    assert!(osm
        .document
        .ways
        .values()
        .all(|way| !way.tags.contains_key("building")));
    for warnings in [
        roadgen_lanelet2::check(&plain),
        roadgen_sumo::check(&plain),
        roadgen_clipgt::check(&plain, None),
        roadgen_gpudrive::check(&plain, None),
    ] {
        assert!(
            !warnings.iter().any(|warning| warning.contains("buildings")),
            "a map with no buildings should not be told they were dropped: {warnings:?}"
        );
    }
}

#[test]
fn a_building_beside_a_bend_comes_back_the_shape_it_went_in() {
    // The reason the outline is written as `cornerLocal`: a corner of its own
    // `(s, t)` follows the arc, so on the inside of a bend a fourteen-metre wall
    // comes back as a fourteen-metre curve and the building fans out. Every corner
    // here is measured in one frame, so the rectangle stays a rectangle.
    let mut builder = MapBuilder::new(scenarios::metadata("bend"));
    let alignment = Alignment::new(Point3::new(0.0, 0.0, 0.0), 0.0)
        .line(80.0, 0.0)
        .unwrap()
        // Radius 50 m, which is as tight as a street gets.
        .arc(160.0, 0.02, 0.0)
        .unwrap()
        .line(80.0, 0.0)
        .unwrap();
    builder
        .add_road(
            RoadSpec::new(alignment.finish().unwrap(), scenarios::two_way()).with_name("bend"),
        )
        .unwrap();

    let mut map = builder.finish().expect("the bend should build");
    roadgen_buildings::generate(&mut map, &Rules::default()).expect("the rules should derive");
    let map = map.validate().expect("it should validate");
    assert!(map.buildings.len() > 4);

    for shape in redraw_opendrive(&map)
        .shapes
        .iter()
        .filter(|shape| shape.kind() == Kind::Building)
    {
        // Every part is a rectangle; the roof is not drawn in plan.
        let Shape::Area { points, .. } = shape else {
            panic!("a building is drawn as an area");
        };
        // Opposite sides of a rectangle are equal, and the drawing walks the ring in
        // order, so a fanned-out building fails this by metres.
        assert_eq!(points.len(), 4, "{points:?}");
        let side =
            |a: usize, b: usize| (points[a].x - points[b].x).hypot(points[a].y - points[b].y);
        assert!(
            (side(0, 1) - side(2, 3)).abs() < 0.05,
            "sides {} and {} should match: {points:?}",
            side(0, 1),
            side(2, 3)
        );
        assert!(
            (side(1, 2) - side(3, 0)).abs() < 0.05,
            "sides {} and {} should match: {points:?}",
            side(1, 2),
            side(3, 0)
        );
    }
}
