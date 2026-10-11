//! A generated open-pit mine, through every exporter.
//!
//! The mine is the one map in the workspace whose IR states the land, so it is the
//! test of what each format makes of that: CARLA builds its ground from the TIN,
//! OpenDRIVE writes the gravel and the berm, OpenStreetMap writes the quarry, and
//! the formats with no word for any of it still write the roads.

use roadgen_carla::{materials, PackageConfig, Role};
use roadgen_core::prelude::*;
use roadgen_core::semantics::{MapObjectKind, Pavement, UnpavedSurface};
use roadgen_mine::MineSpec;

fn mine() -> ValidatedMap {
    let mut builder = MapBuilder::default();
    let plan = roadgen_mine::lay_out(&mut builder, &MineSpec::default()).expect("lays out");
    let mut map = builder.finish().expect("builds");
    roadgen_mine::generate(&mut map, &plan).expect("generates");
    map.validate().expect("validates")
}

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
    let spec = MineSpec::default();
    let depth = spec.pit.benches as f64 * spec.pit.bench_height;
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
        warnings.iter().any(|w| w.contains("unpaved")),
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
