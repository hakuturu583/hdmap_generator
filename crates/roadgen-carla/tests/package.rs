//! Writing a whole package, and checking what came out.
//!
//! The unit tests in the crate check the pieces: that a name classifies the way
//! CARLA classifies it, that a strip faces up, that a document is FBX. These check
//! the thing a caller actually gets — a folder CARLA's importer would be pointed at —
//! and they check it by reading the files back rather than by trusting the writer.

use std::fs;

use roadgen_carla::{BuildingPlacement, Label, PackageConfig};
use roadgen_core::prelude::*;
use roadgen_core::semantics::LaneType;

/// A street with pavements either side, which is what exercises every class: a
/// carriageway, paint on it, two pavements, the kerbs between them and the gutters at
/// the foot of those.
fn street(name: &str) -> ValidatedMap {
    let mut builder = MapBuilder::new(MapMetadata {
        name: Some(name.to_owned()),
        ..MapMetadata::default()
    });
    let width = |metres: f64| PositiveWidth::new(metres).expect("a positive width");
    let lanes = || {
        vec![
            // Outwards from the reference line on each side, which is the order the
            // builder counts ordinals in: carriageway first, then the pavement
            // beyond it.
            LaneSpec::new(width(3.5), Direction::Backward),
            LaneSpec::new(width(2.0), Direction::Backward).with_type(LaneType::Sidewalk),
            LaneSpec::new(width(3.5), Direction::Forward),
            LaneSpec::new(width(2.0), Direction::Forward).with_type(LaneType::Sidewalk),
        ]
    };
    let first = builder
        .add_road(
            RoadSpec::line(
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(120.0, 0.0, 0.0),
                lanes(),
            )
            .expect("a road")
            .with_name("first"),
        )
        .expect("a road");
    let second = builder
        .add_road(
            RoadSpec::line(
                Point3::new(120.0, 0.0, 0.0),
                Point3::new(240.0, 0.0, 0.0),
                lanes(),
            )
            .expect("a road")
            .with_name("second"),
        )
        .expect("a road");
    builder.connect(&first, &second).expect("a connection");

    builder
        .finish()
        .expect("a map")
        .validate()
        .expect("a valid map")
}

fn with_a_town(name: &str) -> ValidatedMap {
    let mut builder = MapBuilder::new(MapMetadata {
        name: Some(name.to_owned()),
        ..MapMetadata::default()
    });
    let width = |metres: f64| PositiveWidth::new(metres).expect("a positive width");
    let road = builder
        .add_road(
            RoadSpec::line(
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(200.0, 0.0, 0.0),
                vec![
                    LaneSpec::new(width(3.5), Direction::Forward),
                    LaneSpec::new(width(3.5), Direction::Backward),
                ],
            )
            .expect("a road")
            .with_name("high_street"),
        )
        .expect("a road");
    let _ = road;
    let mut map = builder.finish().expect("a map");
    roadgen_buildings::generate(&mut map, &roadgen_buildings::Rules::default())
        .expect("a town should generate");
    map.validate().expect("a valid map")
}

#[test]
fn a_package_is_a_descriptor_an_fbx_and_the_road_network_under_it() {
    let map = street("Town01");
    let directory = tempfile::tempdir().expect("a temporary directory");
    let config = PackageConfig::for_map(&map);
    let package = roadgen_carla::write(&map, directory.path(), &config).expect("a package");

    assert!(package.descriptor.exists(), "no descriptor was written");
    assert!(package.fbx.exists(), "no mesh was written");
    assert!(package.xodr.exists(), "no road network was written");

    // CARLA pairs the two by name in three separate places, so this is the one thing
    // about the layout that is not a preference.
    assert_eq!(
        package.fbx.file_stem(),
        package.xodr.file_stem(),
        "the mesh and the road network have to share a name"
    );

    let descriptor: serde_json::Value =
        serde_json::from_str(&fs::read_to_string(&package.descriptor).expect("the descriptor"))
            .expect("the descriptor should be JSON");
    assert_eq!(descriptor["maps"][0]["name"], "Town01");
    assert_eq!(descriptor["maps"][0]["source"], "./Town01/Town01.fbx");
    assert_eq!(descriptor["maps"][0]["xodr"], "./Town01/Town01.xodr");
    // `GetArrayField("props")` is called without checking whether it is there.
    assert!(descriptor["props"].is_array());
}

#[test]
fn every_class_a_street_has_comes_out_tagged_as_that_class() {
    // The whole point of the exporter. A pavement that renders correctly and segments
    // as ground is the failure this test exists to catch, and it catches it by running
    // the written mesh names back through CARLA's own classifier.
    let map = street("Town01");
    let directory = tempfile::tempdir().expect("a temporary directory");
    let config = PackageConfig::for_map(&map);
    let package = roadgen_carla::write(&map, directory.path(), &config).expect("a package");

    for label in [
        Label::Roads,
        Label::RoadLines,
        Label::Sidewalks,
        Label::Terrain,
    ] {
        assert!(
            package.labels.get(&label).copied().unwrap_or_default() > 0,
            "nothing in the map will be tagged {label}: {:?}",
            package.labels
        );
    }
}

#[test]
fn the_mesh_names_in_the_file_are_the_ones_the_report_counted() {
    // The report is computed from the meshes; this reads the names out of the file
    // that was actually written, so the two cannot drift apart.
    let map = street("Town01");
    let directory = tempfile::tempdir().expect("a temporary directory");
    let config = PackageConfig::for_map(&map);
    let package = roadgen_carla::write(&map, directory.path(), &config).expect("a package");

    let text = fs::read_to_string(&package.fbx).expect("the mesh file");
    let names: Vec<&str> = text
        .lines()
        .filter_map(|line| line.trim().strip_prefix("Model: "))
        .filter_map(|line| line.split('"').nth(1))
        .filter_map(|name| name.strip_prefix("Model::"))
        .collect();
    assert_eq!(names.len(), package.meshes);
    assert!(names.iter().all(|name| name.starts_with("Town01_")));

    let mut counted = std::collections::BTreeMap::new();
    for name in &names {
        *counted
            .entry(roadgen_carla::tags::label_of(name))
            .or_insert(0usize) += 1;
    }
    assert_eq!(counted, package.labels);

    // No two meshes share a name: Unreal renames a duplicate on import, and a renamed
    // mesh is one nothing can account for afterwards.
    let mut unique = names.clone();
    unique.sort_unstable();
    unique.dedup();
    assert_eq!(unique.len(), names.len(), "two meshes share a name");
}

#[test]
fn the_surface_stands_where_the_road_network_says_it_does() {
    // The mesh and the .xodr are read by two different parts of CARLA and have to
    // describe the same place. Both are in the map's own metres here — the handedness
    // flip happens inside Unreal, to both of them — so the vertices should cover the
    // road's own extent.
    let map = street("Town01");
    let directory = tempfile::tempdir().expect("a temporary directory");
    let config = PackageConfig::for_map(&map);
    let meshes = roadgen_carla::to_meshes(&map, &config);

    // The road's own surfaces, without the land that is written out past them
    // for a lidar to reach — that one is the terrain module's own business — and
    // without the paint, which straddles the edge it is on.
    let street: Vec<&roadgen_carla::Mesh> = meshes
        .iter()
        .filter(|mesh| {
            mesh.role != roadgen_carla::Role::Terrain && mesh.role != roadgen_carla::Role::Marking
        })
        .collect();
    let xs: Vec<f64> = street
        .iter()
        .flat_map(|mesh| mesh.positions.iter().map(|point| point.x))
        .collect();
    let ys: Vec<f64> = street
        .iter()
        .flat_map(|mesh| mesh.positions.iter().map(|point| point.y))
        .collect();
    let min = |values: &[f64]| values.iter().copied().fold(f64::INFINITY, f64::min);
    let max = |values: &[f64]| values.iter().copied().fold(f64::NEG_INFINITY, f64::max);

    assert!(
        (min(&xs) - 0.0).abs() < 1e-6,
        "the street starts at {}",
        min(&xs)
    );
    assert!(
        (max(&xs) - 240.0).abs() < 1e-6,
        "the street ends at {}",
        max(&xs)
    );
    // Eleven metres of cross-section: the pavements' outer edges, and past them
    // the verges are the land's.
    assert!(
        (max(&ys) - 5.5).abs() < 1e-6,
        "the street reaches {}",
        max(&ys)
    );
    assert!((min(&ys) + 5.5).abs() < 1e-6);
    let land = meshes
        .iter()
        .find(|mesh| mesh.role == roadgen_carla::Role::Terrain)
        .expect("the land");
    let far = land
        .positions
        .iter()
        .map(|point| point.y)
        .fold(f64::NEG_INFINITY, f64::max);
    assert!(
        far >= 5.5 + config.surfaces.verge_width,
        "the land reaches {far}"
    );
    let _ = directory;
}

#[test]
fn a_pavement_stands_above_the_road_beside_it() {
    let map = street("Town01");
    let config = PackageConfig::for_map(&map);
    let meshes = roadgen_carla::to_meshes(&map, &config);

    let highest = |role: roadgen_carla::Role| {
        meshes
            .iter()
            .filter(|mesh| mesh.role == role)
            .flat_map(|mesh| mesh.positions.iter().map(|point| point.z))
            .fold(f64::NEG_INFINITY, f64::max)
    };
    let kerb = config.surfaces.kerb_height;
    assert!((highest(roadgen_carla::Role::Sidewalk) - kerb).abs() < 1e-9);
    assert!(highest(roadgen_carla::Role::Road).abs() < 1e-9);
    // And the kerb face spans the two.
    assert!((highest(roadgen_carla::Role::Curb) - kerb).abs() < 1e-9);
}

#[test]
fn the_textures_are_listed_rather_than_fetched() {
    // An exporter that reached for the network could not run offline, in CI, or in the
    // browser — and the demo page runs this crate compiled to WebAssembly.
    let map = street("Town01");
    let directory = tempfile::tempdir().expect("a temporary directory");
    let config = PackageConfig::for_map(&map);
    let package = roadgen_carla::write(&map, directory.path(), &config).expect("a package");

    let manifest = directory.path().join("Town01/Textures/polyhaven.manifest");
    let listed: serde_json::Value =
        serde_json::from_str(&fs::read_to_string(&manifest).expect("the manifest"))
            .expect("the manifest should be JSON");
    let files = listed["files"].as_array().expect("a list of files");
    assert!(!files.is_empty());
    assert_eq!(listed["license"], "CC0-1.0");

    // Nothing has been fetched, so every one of them is still outstanding.
    assert_eq!(package.textures.len(), files.len());
    assert!(directory.path().join("Town01/Textures/CREDITS.md").exists());

    // The manifest is JSON but is not called `.json`: CARLA's `Import.py` imports
    // every `.json` it finds under `Import/` as a package, and a manifest so named
    // becomes an empty package called `polyhaven` beside the map. The descriptor
    // is the one `.json` a package carries.
    let jsons: Vec<_> = walk(directory.path())
        .into_iter()
        .filter(|path| path.extension().is_some_and(|ext| ext == "json"))
        .collect();
    assert_eq!(jsons, vec![package.descriptor.clone()], "{jsons:?}");

    // And the FBX refers to them where they will be, by a relative path.
    let text = fs::read_to_string(&package.fbx).expect("the mesh file");
    for file in files {
        let path = file["path"].as_str().expect("a path");
        if path.contains("_rough_") {
            continue; // Fetched for a material rebuilt by hand; not wired into Phong.
        }
        assert!(
            text.contains(&format!("RelativeFilename: \"{path}\"")),
            "{path} is listed but not referenced"
        );
    }
}

#[test]
fn a_town_in_the_map_is_placed_and_mislabelled_and_says_so() {
    // CARLA's MoveAssets commandlet knows six mesh names and none of them is a
    // building, so a building inside a map's FBX is sorted into Terrain. That is not
    // a bug in this exporter and it is not something it can fix; what it can do is
    // put the building in the map, tell the caller what CARLA will call it, and offer
    // the other trade.
    let map = with_a_town("Town01");
    assert!(!map.buildings.is_empty(), "the fixture grew no town");

    let config = PackageConfig::for_map(&map).with_buildings(BuildingPlacement::InMap);
    let meshes = roadgen_carla::to_meshes(&map, &config);
    let town: Vec<_> = meshes
        .iter()
        .filter(|mesh| mesh.role == roadgen_carla::Role::Building)
        .collect();
    assert_eq!(town.len(), map.buildings.len());
    for mesh in &town {
        assert_eq!(roadgen_carla::tags::label_of(&mesh.name), Label::Terrain);
    }

    let warnings = roadgen_carla::check(&map, &config);
    assert!(
        warnings.iter().any(|warning| warning.contains("Terrain")
            && warning.contains("Buildings")
            && warning.contains("props")),
        "the mislabelling was not reported: {warnings:#?}"
    );
}

#[test]
fn a_town_as_props_is_tagged_correctly_and_not_placed() {
    let map = with_a_town("Town01");
    let directory = tempfile::tempdir().expect("a temporary directory");
    let config = PackageConfig::for_map(&map).with_buildings(BuildingPlacement::Props);
    let package = roadgen_carla::write(&map, directory.path(), &config).expect("a package");

    let props = package.props.expect("a props file");
    assert!(props.exists());

    let descriptor: serde_json::Value =
        serde_json::from_str(&fs::read_to_string(&package.descriptor).expect("the descriptor"))
            .expect("the descriptor should be JSON");
    // The one place in CARLA's whole pipeline where a tag is stated rather than
    // spelled into a mesh name.
    assert_eq!(descriptor["props"][0]["tag"], "Building");

    // And the map itself no longer holds them.
    assert_eq!(package.labels.get(&Label::Buildings), None);
    let warnings = roadgen_carla::check(&map, &config);
    assert!(warnings
        .iter()
        .any(|warning| warning.contains("none of them will be in the level")));
}

#[test]
fn a_map_named_after_a_semantic_token_is_refused_in_the_report() {
    let map = street("Terrain Town");
    let config = PackageConfig::for_map(&map);
    assert_eq!(config.map, "Terrain_Town");

    let warnings = roadgen_carla::check(&map, &config);
    assert!(
        warnings
            .iter()
            .any(|warning| warning.contains("Rename the map")),
        "a map name that tags the whole map as one thing went unreported: {warnings:#?}"
    );

    // And it really would: every pavement in it classifies as ground.
    let meshes = roadgen_carla::to_meshes(&map, &config);
    let pavements: Vec<_> = meshes
        .iter()
        .filter(|mesh| mesh.role == roadgen_carla::Role::Sidewalk)
        .collect();
    assert!(!pavements.is_empty());
    for mesh in pavements {
        assert_eq!(roadgen_carla::tags::label_of(&mesh.name), Label::Terrain);
    }
}

#[test]
fn the_same_map_exported_twice_is_the_same_bytes() {
    let map = street("Town01");
    let config = PackageConfig::for_map(&map);
    let write = || {
        let directory = tempfile::tempdir().expect("a temporary directory");
        let package = roadgen_carla::write(&map, directory.path(), &config).expect("a package");
        let text = fs::read_to_string(&package.fbx).expect("the mesh file");
        (directory, text)
    };
    let (_first, one) = write();
    let (_second, two) = write();
    assert_eq!(one, two);
}

/// Every file under `root`, recursively.
fn walk(root: &std::path::Path) -> Vec<std::path::PathBuf> {
    let mut files = Vec::new();
    for entry in fs::read_dir(root).expect("a directory") {
        let path = entry.expect("an entry").path();
        if path.is_dir() {
            files.extend(walk(&path));
        } else {
            files.push(path);
        }
    }
    files
}
