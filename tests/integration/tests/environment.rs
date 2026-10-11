//! What is generated onto a CARLA map once its roads are built — pits, heaps,
//! pads, buildings, the land — and what comes of it in every exporter.
//!
//! The mine is the map in the workspace whose IR states the land, so it is the test
//! of what each format makes of that: CARLA builds its ground from the TIN,
//! OpenDRIVE writes the gravel and the berm, OpenStreetMap writes the quarry, and
//! the formats with no word for any of it still write the roads.

use roadgen_carla::environment::{self, PitSpec, Places};
use roadgen_carla::{materials, PackageConfig, Role};
use roadgen_core::prelude::*;
use roadgen_core::semantics::{MapObjectKind, Pavement, UnpavedSurface};
use roadgen_integration_tests::mine::{mine, open_pit_mine};

#[test]
fn carla_builds_its_ground_from_the_terrain_the_map_states() {
    let map = mine();
    let config = PackageConfig::new("NorthPit");
    let meshes = roadgen_carla::to_meshes(&map, &config);

    // The pit is rock and the country round it grass, as two slots of one surface.
    let terrain = meshes
        .iter()
        .find(|mesh| mesh.role == Role::Terrain && mesh.materials.contains(&materials::ROCK))
        .expect("a terrain mesh with rock on it");
    assert!(terrain.materials.contains(&materials::GRASS));

    // The land goes as deep as the pit's floor, which a ground made from the roads
    // alone never could: the ramp reaches the floor, and the benches step down to it.
    let spec = PitSpec::default();
    let depth = spec.depth();
    let lowest = terrain
        .positions
        .iter()
        .map(|p| p.z)
        .fold(f64::INFINITY, f64::min);
    assert!((lowest - (spec.centre.z - depth)).abs() < 1e-6, "{lowest}");

    // The haul roads are gravel, not asphalt.
    let roads: Vec<_> = meshes
        .iter()
        .filter(|mesh| mesh.role == Role::Road)
        .collect();
    assert!(!roads.is_empty());
    assert!(roads
        .iter()
        .all(|mesh| !mesh.materials.contains(&materials::ASPHALT)));
    assert!(roads
        .iter()
        .any(|mesh| mesh.materials.contains(&materials::GRAVEL)));

    // The berm stands as a ridge along the ramp.
    let berm = meshes
        .iter()
        .filter(|mesh| mesh.role == Role::Terrain && mesh.materials == [materials::GRAVEL])
        .count();
    assert_eq!(berm, 1);

    let warnings = roadgen_carla::check(&map, &config);
    assert!(
        warnings.iter().any(|w| w.contains("TIN surfaces")),
        "{warnings:#?}"
    );
    assert!(
        warnings.iter().any(|w| w.contains("other than asphalt")),
        "{warnings:#?}"
    );

    let directory = tempfile::tempdir().unwrap();
    roadgen_carla::write(&map, directory.path(), &config).expect("the package writes");
}

#[test]
fn opendrive_carries_the_gravel_and_the_berm_and_reads_the_gravel_back() {
    let map = mine();
    let xml = roadgen_opendrive::to_xml(&map).expect("writes");
    assert!(xml.contains("surface=\"gravel\""));
    assert!(xml.contains("type=\"barrier\""));
    assert!(xml.contains("subtype=\"berm\""));

    let directory = tempfile::tempdir().unwrap();
    let path = directory.path().join("mine.xodr");
    std::fs::write(&path, &xml).unwrap();
    let read = roadgen_opendrive::read(&path).expect("reads back");
    let gravel = Some(Pavement::Unpaved(UnpavedSurface::Gravel));
    let roads: Vec<_> = read.map.as_map().roads.iter().collect();
    assert!(!roads.is_empty());
    for road in roads {
        assert_eq!(road.pavement.at(0.0), gravel, "{}", road.id);
    }
}

#[test]
fn openstreetmap_writes_the_quarry_and_the_surface() {
    let map = mine();
    let xml = roadgen_osm::to_xml(&map).expect("writes");
    assert!(xml.contains("\"quarry\""));
    assert!(xml.contains("\"spoil_heap\""));
    assert!(xml.contains("\"gravel\""));
    assert!(xml.contains("k=\"barrier\""));
    assert!(roadgen_osm::check(&map)
        .iter()
        .any(|warning| warning.contains("TIN")));
}

#[test]
fn the_formats_with_no_land_still_write_the_roads() {
    let map = mine();
    let directory = tempfile::tempdir().unwrap();
    roadgen_lanelet2::write(&map, directory.path().join("mine.osm")).expect("Lanelet2");
    roadgen_sumo::to_plain_xml(&map).expect("SUMO");
    let scene = roadgen_gpudrive::SceneConfig::for_map(&map);
    roadgen_gpudrive::to_scene(&map, &scene).expect("GPUDrive");
    let trace = roadgen_trace::IrCatalog::of(&map);
    assert_eq!(trace.sites.len(), 1);
    assert_eq!(trace.tin_surfaces.len(), 1);
    assert_eq!(trace.land_covers.len(), 4);
    assert!(map
        .objects
        .iter()
        .any(|object| matches!(object.kind, MapObjectKind::Barrier { .. })));
}

/// A mine with a road of the caller's own beside it: two driving lanes and a
/// pavement each side, the pavements lifted 0.3 m by the IR — twice the kerb the
/// CARLA export would put under a pavement the IR left flat.
fn mine_with_a_street() -> ValidatedMap {
    use roadgen_core::geometry::LaneHeight;
    use roadgen_core::semantics::{LaneType, PavedSurface, PavementProfile};
    use roadgen_core::topology::Direction;
    use roadgen_core::units::PositiveWidth;

    let mut builder = MapBuilder::default();
    let width = |metres| PositiveWidth::new(metres).unwrap();
    let pavement = |direction| {
        LaneSpec::new(width(2.5), direction)
            .with_type(LaneType::Sidewalk)
            .with_height(LaneHeight::constant(0.3, 0.3).unwrap())
    };
    builder
        .add_road(
            RoadSpec::line(
                Point3::new(-200.0, 260.0, 0.0),
                Point3::new(200.0, 260.0, 0.0),
                vec![
                    LaneSpec::new(width(3.5), Direction::Forward),
                    pavement(Direction::Forward),
                    LaneSpec::new(width(3.5), Direction::Backward),
                    pavement(Direction::Backward),
                ],
            )
            .unwrap()
            .with_name("street")
            // Concrete for the first half, cobbles for the rest.
            .with_pavement(
                PavementProfile::new([
                    (0.0, Pavement::Paved(PavedSurface::Concrete)),
                    (200.0, Pavement::Paved(PavedSurface::Cobblestone)),
                ])
                .unwrap(),
            ),
        )
        .unwrap();
    let places = open_pit_mine(&mut builder, "mine", Point3::ORIGIN);
    let mut map = builder.finish().unwrap();
    environment::generate(&mut map, &places).unwrap();
    map.validate().expect("validates")
}

#[test]
fn a_road_the_caller_adds_to_a_mine_is_on_the_land_with_its_pavements() {
    let map = mine_with_a_street();
    let surface = map.tin_surfaces.iter().next().unwrap();
    let locator = surface.tin.locator();
    // The street's edges are breaklines of the land, at the pavements' own height.
    let street = RoadId::new("street");
    let edges: Vec<_> = map
        .tin_breaklines
        .iter()
        .filter(|line| line.id.as_str().contains("/road/street/"))
        .collect();
    assert_eq!(edges.len(), 2);
    for edge in edges {
        for point in edge.line.points() {
            assert!((point.z - 0.3).abs() < 1e-6, "{}", point.z);
        }
    }
    for lane in map.lanes_of_section(&street, 0) {
        for point in lane
            .centerline
            .to_polyline(map.metadata.sampling)
            .unwrap()
            .points()
        {
            let ground = locator.height_at(point.x, point.y).expect("on the land");
            assert!(
                (ground - point.z).abs() < 0.31,
                "{} at {:.2}",
                lane.id,
                ground
            );
        }
    }

    // And CARLA draws all of it: pavements, kerbs, the land.
    let meshes = roadgen_carla::to_meshes(&map, &PackageConfig::new("Quarry"));
    for role in [Role::Sidewalk, Role::Curb, Role::Terrain, Role::Road] {
        assert!(meshes.iter().any(|mesh| mesh.role == role), "{role:?}");
    }
}

#[test]
fn carla_draws_what_the_ir_says_rather_than_its_defaults() {
    let map = mine_with_a_street();
    let meshes = roadgen_carla::to_meshes(&map, &PackageConfig::new("Quarry"));

    // The pavements stand where the IR lifted them, not at the configured kerb.
    let top = meshes
        .iter()
        .filter(|mesh| mesh.role == Role::Sidewalk)
        .flat_map(|mesh| mesh.positions.iter())
        .filter(|p| p.y > 250.0)
        .map(|p| p.z)
        .fold(f64::NEG_INFINITY, f64::max);
    assert!((top - 0.3).abs() < 1e-6, "{top}");

    // The street is concrete, then cobbles; the haul roads gravel.
    let materials: std::collections::HashSet<usize> = meshes
        .iter()
        .filter(|mesh| mesh.role == Role::Road)
        .flat_map(|mesh| mesh.materials.iter().copied())
        .collect();
    for material in [
        materials::ROAD_CONCRETE,
        materials::COBBLESTONE,
        materials::GRAVEL,
    ] {
        assert!(materials.contains(&material), "{material}");
    }
    assert!(!materials.contains(&materials::ASPHALT));
}

#[test]
fn a_barrier_is_built_the_way_its_kind_is() {
    let mut map = mine().into_map();
    let berm = map
        .objects
        .iter()
        .find(|object| matches!(object.kind, MapObjectKind::Barrier { .. }))
        .unwrap()
        .id
        .clone();
    let draw = |map: &roadgen_core::map::Map| {
        let map = roadgen_core::validation::UnvalidatedMap::from_map(map.clone())
            .validate()
            .unwrap();
        roadgen_carla::to_meshes(&map, &PackageConfig::new("Quarry"))
            .into_iter()
            .filter(|mesh| mesh.role == Role::Terrain && mesh.materials.len() == 1)
            .map(|mesh| mesh.materials[0])
            .collect::<Vec<_>>()
    };
    assert!(draw(&map).contains(&materials::GRAVEL));
    map.objects.get_mut(&berm).unwrap().kind = MapObjectKind::Barrier {
        kind: "wall".into(),
        height: 1.0,
    };
    assert!(draw(&map).contains(&materials::CURB));
}

#[test]
fn a_mine_puts_buildings_on_its_yard_from_the_towns_grammar() {
    let map = mine();
    assert!(!map.buildings.is_empty());
    let yard = map
        .land_covers
        .iter()
        .find(|cover| cover.kind == "pad")
        .unwrap();
    let ring: Vec<[f64; 2]> = yard.area.points().iter().map(|p| [p.x, p.y]).collect();
    for part in map.building_parts.iter() {
        for corner in part.solid.footprint.points() {
            assert!(
                roadgen_core::geometry::plan::ring_contains(&ring, [corner.x, corner.y]),
                "{} stands off the yard",
                part.id
            );
        }
    }
    // Facing the haul road, the way a town's buildings face their street.
    for building in map.buildings.iter() {
        let road = &building.frontage.as_ref().unwrap().road;
        assert!(road.as_str().contains("haul_waste_dump"), "{road}");
    }
}

#[test]
fn a_town_builds_its_land_the_way_a_mine_does() {
    use roadgen_carla::environment::GroundSpec;
    use roadgen_core::topology::Direction;
    use roadgen_core::units::PositiveWidth;

    let mut builder = MapBuilder::default();
    let lanes = || {
        vec![
            LaneSpec::new(PositiveWidth::new(3.5).unwrap(), Direction::Forward),
            LaneSpec::new(PositiveWidth::new(3.5).unwrap(), Direction::Backward),
        ]
    };
    builder
        .add_road(
            RoadSpec::line(
                Point3::new(0.0, 0.0, 10.0),
                Point3::new(300.0, 0.0, 16.0),
                lanes(),
            )
            .unwrap()
            .with_name("high_street"),
        )
        .unwrap();
    let town = Places {
        town: roadgen_buildings::Rules::preset("town"),
        ground: Some(GroundSpec::default()),
        seed: 3,
        ..Places::default()
    };
    let mut map = builder.finish().unwrap();
    environment::generate(&mut map, &town).unwrap();
    let map = map.validate().expect("validates");

    assert!(!map.buildings.is_empty());
    let surface = map.tin_surfaces.iter().next().expect("the town's land");
    // The road's edges are breaklines of it, as a haul road's are of a mine's.
    assert_eq!(
        map.tin_breaklines
            .iter()
            .filter(|line| line.kind == "road_edge")
            .count(),
        2
    );
    // And the land sits at the road's height beside it, climbing as it does.
    let locator = surface.tin.locator();
    for (x, z) in [(20.0, 10.4), (280.0, 15.6)] {
        let ground = locator.height_at(x, 12.0).unwrap();
        assert!((ground - z).abs() < 0.5, "{x}: {ground}");
    }
    // CARLA draws the land from the TIN, as it does a mine's.
    let warnings = roadgen_carla::check(&map, &PackageConfig::new("Town"));
    assert!(
        warnings.iter().any(|w| w.contains("TIN surfaces")),
        "{warnings:#?}"
    );
}

#[test]
fn a_town_beside_a_mine_does_not_line_the_mines_roads() {
    let map = {
        let mut builder = MapBuilder::default();
        let places = Places {
            town: roadgen_buildings::Rules::preset("town"),
            ..open_pit_mine(&mut builder, "mine", Point3::ORIGIN)
        };
        let mut map = builder.finish().unwrap();
        environment::generate(&mut map, &places).unwrap();
        map.validate().expect("validates")
    };
    // Every building is the mine's own, on its yard: the town has no street of its
    // own here, and the ramp and haul roads are the mine's.
    for building in map.buildings.iter() {
        let road = &building.frontage.as_ref().unwrap().road;
        assert!(road.as_str().contains("haul_waste_dump"), "{road}");
    }
}

#[test]
fn two_mines_lie_on_one_land() {
    let mut builder = MapBuilder::default();
    let north = open_pit_mine(&mut builder, "North Pit", Point3::ORIGIN);
    let south = open_pit_mine(&mut builder, "South Pit", Point3::new(0.0, -900.0, 0.0));
    let places = Places {
        pits: [north.pits, south.pits].concat(),
        heaps: [north.heaps, south.heaps].concat(),
        pads: [north.pads, south.pads].concat(),
        site_roads: [north.site_roads, south.site_roads].concat(),
        ..Places::default()
    };
    let mut map = builder.finish().unwrap();
    environment::generate(&mut map, &places).unwrap();
    let map = map.validate().expect("validates");

    assert_eq!(map.tin_surfaces.len(), 1);
    assert_eq!(map.sites.len(), 2);
    let surface = map.tin_surfaces.iter().next().unwrap();
    for site in map.sites.iter() {
        assert_eq!(site.kind, "open_pit_mine");
        assert_eq!(site.surfaces, std::slice::from_ref(&surface.id));
        assert_eq!(site.land_covers.len(), 4, "{}", site.id);
    }
    // Both floors are in the one land.
    let locator = surface.tin.locator();
    let floor = -PitSpec::default().depth();
    for y in [0.0, -900.0] {
        let z = locator.height_at(0.0, y).unwrap();
        assert!((z - floor).abs() < 1e-6, "{y}: {z}");
    }
    // And every connector through a hub is its mine's.
    let north = map.sites.get(&SiteId::new("north_pit")).unwrap();
    assert!(north
        .roads
        .iter()
        .any(|road| map.road(road).unwrap().is_connector()));
}

#[test]
fn parts_that_overlap_are_refused() {
    let mut map = MapBuilder::default().finish().unwrap();
    let places = Places {
        pits: vec![
            PitSpec::default(),
            PitSpec {
                name: "other".into(),
                centre: Point3::new(150.0, 0.0, 0.0),
                ..PitSpec::default()
            },
        ],
        ..Places::default()
    };
    let error = environment::generate(&mut map, &places).unwrap_err();
    assert!(error.to_string().contains("overlap"), "{error}");
}
