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
fn a_town_as_props_is_tagged_correctly_and_placed_by_the_script() {
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

    // CARLA's importer will not place a prop, so the manifest lists every
    // building for the package's script to stand in the level, by the asset name
    // the import gives it.
    let manifest: serde_json::Value =
        serde_json::from_str(&fs::read_to_string(package.furniture.as_ref().unwrap()).unwrap())
            .unwrap();
    let buildings = manifest["buildings"].as_array().unwrap();
    assert_eq!(buildings.len(), map.buildings.len());
    assert!(buildings[0]["asset"]
        .as_str()
        .unwrap()
        .starts_with("/Game/Town01Package/Static/Building/Town01_Buildings/Town01_Buildings_"));
    // And the pedestrians' mesh still knows where the town is.
    let obj = fs::read_to_string(&package.obj).unwrap();
    assert!(
        obj.contains("usemtl block"),
        "no buildings in the navigation mesh"
    );

    // And the map itself no longer holds them.
    assert_eq!(package.labels.get(&Label::Buildings), None);
    let warnings = roadgen_carla::check(&map, &config);
    assert!(warnings
        .iter()
        .any(|warning| warning.contains("stood in the level by the package's script")));
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

/// A signalised T with pavements: a light and a stop sign on the north approach,
/// a light on the east one, and a speed limit on the way in. What the furniture is
/// built to.
fn signalised(name: &str) -> ValidatedMap {
    let mut builder = MapBuilder::new(MapMetadata {
        name: Some(name.to_owned()),
        ..MapMetadata::default()
    });
    let width = |metres: f64| PositiveWidth::new(metres).expect("a positive width");
    let lanes = || {
        vec![
            LaneSpec::new(width(3.5), Direction::Backward),
            LaneSpec::new(width(2.0), Direction::Backward).with_type(LaneType::Sidewalk),
            LaneSpec::new(width(3.5), Direction::Forward),
            LaneSpec::new(width(2.0), Direction::Forward).with_type(LaneType::Sidewalk),
        ]
    };
    let north = builder
        .add_road(
            RoadSpec::line(
                Point3::new(0.0, 80.0, 0.0),
                Point3::new(0.0, 14.0, 0.0),
                lanes(),
            )
            .expect("a road")
            .with_name("north"),
        )
        .expect("a road");
    let east = builder
        .add_road(
            RoadSpec::line(
                Point3::new(80.0, 0.0, 0.0),
                Point3::new(14.0, 0.0, 0.0),
                lanes(),
            )
            .expect("a road")
            .with_name("east"),
        )
        .expect("a road");
    let junction = builder.add_junction(Some("x"));
    builder
        .connect_ends(&north, RoadEnd::End, &east, RoadEnd::End, Some(&junction))
        .expect("a connection");
    let north_in = LaneRef::new(north.clone(), 2);
    let east_in = LaneRef::new(east.clone(), 2);
    // Eight metres before the mouth, where a stop line goes when a crosswalk is
    // between it and the junction.
    let stop = builder
        .add_stop_line_at(&north_in, LaneEnd::End, 8.0)
        .expect("a stop line");
    let north_light = builder
        .add_traffic_light(&north_in, LaneEnd::End, 5.5)
        .expect("a light");
    let east_light = builder
        .add_traffic_light(&east_in, LaneEnd::End, 5.5)
        .expect("a light");
    builder.add_traffic_light_rule(vec![north_light], Some(stop), vec![north_in.clone()]);
    builder.add_traffic_light_rule(vec![east_light], None, vec![east_in]);
    builder
        .add_traffic_sign(&north_in, LaneEnd::End, "stop", 2.4)
        .expect("a sign");
    builder
        .add_traffic_sign(&north_in, LaneEnd::Start, "de274-50", 2.4)
        .expect("a sign");
    builder
        .finish()
        .expect("a map")
        .validate()
        .expect("a valid map")
}

#[test]
fn a_light_stands_on_the_pavement_with_an_arm_that_reaches_its_lane() {
    let map = signalised("Town01");
    let config = PackageConfig::for_map(&map);
    let furniture = roadgen_carla::furniture::build(
        &map,
        &config.map,
        &config.surfaces,
        &config.furniture.unwrap(),
    );
    assert_eq!(furniture.lights.len(), 2);
    assert_eq!(furniture.signs.len(), 2);

    // The north approach runs south down x = 0; its forward lane is the west one
    // (right-hand traffic, travelling south), from x = 0 to x = -3.5, with the
    // pavement from -3.5 to -5.5. The pole is on that pavement, set back from the
    // kerb, at the road's end, and stands up on it.
    let light = furniture
        .lights
        .iter()
        .find(|placed| placed.object.to_string().contains("north"))
        .expect("the north light");
    assert!(
        (light.position.x + 4.1).abs() < 0.05,
        "the pole is at x = {}, not 0.6 m in from the kerb at -3.5",
        light.position.x
    );
    assert!(
        (light.position.y - 14.0).abs() < 0.05,
        "y = {}",
        light.position.y
    );
    assert!(
        (light.position.z - config.surfaces.kerb_height).abs() < 1e-6,
        "the pole stands on the pavement, at z = {}",
        light.position.z
    );
    // Facing north: the traffic comes down from there.
    assert!(
        (light.heading - std::f64::consts::FRAC_PI_2).abs() < 1e-6,
        "heading {}",
        light.heading
    );
    // The arm reaches from the pole at 4.1 to the lane's middle at 1.75 and half a
    // metre past it.
    assert!(
        (light.arm_length - (4.1 - 1.75 + 0.5)).abs() < 0.05,
        "arm {}",
        light.arm_length
    );
    // The mesh is in the pole's own frame: the foot at the origin, the top of the
    // pole above the bar, the far end of the arm where the arm length says.
    let top = light
        .mesh
        .positions
        .iter()
        .map(|point| point.z)
        .fold(f64::MIN, f64::max);
    assert!(
        top > 5.5 - config.surfaces.kerb_height,
        "the pole tops out at {top}"
    );
    let reach = light
        .mesh
        .positions
        .iter()
        .map(|point| point.y.abs())
        .fold(0.0_f64, f64::max);
    assert!(
        (reach - light.arm_length).abs() < 0.2,
        "the arm reaches {reach}"
    );
    assert!(
        light.mesh.positions.iter().all(|point| point.z > -1e-9),
        "nothing is below the foot"
    );

    // The stop sign stands two metres before the line, on the same pavement, and
    // the speed limit at the far end of the road; both are read for what they are.
    let stop = furniture
        .signs
        .iter()
        .find(|placed| placed.kind == roadgen_carla::SignKind::Stop)
        .expect("the stop sign");
    assert!(
        (stop.position.y - 16.0).abs() < 0.05,
        "y = {}",
        stop.position.y
    );
    assert!(
        (stop.position.x + 4.1).abs() < 0.05,
        "x = {}",
        stop.position.x
    );
    let limit = furniture
        .signs
        .iter()
        .find(|placed| placed.kind == roadgen_carla::SignKind::SpeedLimit(50))
        .expect("the speed limit");
    assert!(limit.position.y > 70.0, "y = {}", limit.position.y);
}

#[test]
fn the_furniture_is_props_the_xodr_agrees_with_and_map_logic_ties_together() {
    let map = signalised("Town01");
    let directory = tempfile::tempdir().expect("a temporary directory");
    let config = PackageConfig::for_map(&map);
    let package = roadgen_carla::write(&map, directory.path(), &config).expect("a package");
    assert_eq!((package.lights, package.signs), (2, 2));
    assert_eq!(package.labels.get(&Label::TrafficLight), Some(&2));
    assert_eq!(package.labels.get(&Label::TrafficSigns), Some(&2));

    // Props, in tagged folders, whose meshes are named the way the manifest names
    // them.
    let descriptor: serde_json::Value =
        serde_json::from_str(&fs::read_to_string(&package.descriptor).expect("the descriptor"))
            .expect("JSON");
    let tags: Vec<&str> = descriptor["props"]
        .as_array()
        .unwrap()
        .iter()
        .map(|prop| prop["tag"].as_str().unwrap())
        .collect();
    assert_eq!(tags, vec!["TrafficLight", "TrafficSign"]);
    let lights_fbx = fs::read_to_string(package.lights_fbx.as_ref().unwrap()).expect("the FBX");
    let manifest: serde_json::Value =
        serde_json::from_str(&fs::read_to_string(package.furniture.as_ref().unwrap()).unwrap())
            .unwrap();
    for light in manifest["lights"].as_array().unwrap() {
        let name = light["name"].as_str().unwrap();
        assert!(
            lights_fbx.contains(&format!("Model::{name}")),
            "{name} is not in the FBX"
        );
        // The lamps are a mesh of their own, beside the post in the same file: the
        // one CARLA converts into the light that switches.
        let lamps = light["lamps_asset"].as_str().unwrap();
        let lamps_name = lamps
            .rsplit('/')
            .next()
            .unwrap()
            .trim_start_matches("Town01_TrafficLights_");
        assert!(lamps_name.contains("Lamps"), "{lamps}");
        assert!(
            lights_fbx.contains(&format!("Model::{lamps_name}")),
            "{lamps_name} is not in the FBX"
        );
        assert_eq!(
            light["asset"].as_str().unwrap(),
            format!("/Game/Town01Package/Static/TrafficLight/Town01_TrafficLights/Town01_TrafficLights_{name}")
        );
    }
    // The lamps' slots are in the FBX by the names the manifest gives the script.
    for lamp in ["red", "amber", "green"] {
        let slot = manifest["lamps"][lamp].as_str().unwrap();
        assert!(
            lights_fbx.contains(&format!("Material::{slot}")),
            "{slot} is not a slot"
        );
    }
    // A sign CARLA knows gets a marker state; the one it does not, none.
    let states: Vec<Option<&str>> = manifest["signs"]
        .as_array()
        .unwrap()
        .iter()
        .map(|sign| sign["carla_state"].as_str())
        .collect();
    assert!(states.contains(&Some("STOP_SIGN")));
    assert!(states.contains(&Some("SPEED_LIMIT_50")));

    // The .xodr: every signal at the foot of its post, in a controller the junction
    // owns, with the catalogue code CARLA reads.
    let xodr = fs::read_to_string(&package.xodr).expect("the road network");
    assert!(xodr.contains("<positionInertial"), "no inertial positions");
    assert!(xodr.contains("<controller"), "no controllers");
    assert!(xodr.contains(r#"type="206""#), "the stop sign is not a 206");
    assert!(xodr.contains(r#"type="274""#) && xodr.contains(r#"subtype="50""#));
    let document = opendrive::core::OpenDrive::from_xml_str(&xodr).expect("parses back");
    assert_eq!(document.controller.len(), 2, "one controller per approach");
    assert_eq!(document.junction[0].controller.len(), 2);

    // map_logic.json names the signals and the controllers the .xodr has.
    let logic: serde_json::Value =
        serde_json::from_str(&fs::read_to_string(package.map_logic.as_ref().unwrap()).unwrap())
            .unwrap();
    let entries = logic["TrafficLights"].as_array().unwrap();
    assert_eq!(entries.len(), 2);
    let signal_ids: Vec<&str> = document
        .road
        .iter()
        .filter_map(|road| road.signals.as_ref())
        .flat_map(|signals| signals.signal.iter().map(|signal| signal.id.as_str()))
        .collect();
    let junction_id: i64 = document.junction[0].id.parse().unwrap();
    for entry in entries {
        assert!(signal_ids.contains(&entry["SignalID"].as_str().unwrap()));
        assert_eq!(entry["JunctionID"].as_i64().unwrap(), junction_id);
        let group = entry["TrafficLightGroupID"].as_str().unwrap();
        assert!(document
            .controller
            .iter()
            .any(|controller| controller.id == group));
        assert_eq!(entry["Modules"][0]["LaneIds"].as_array().unwrap().len(), 1);
    }
    // The north light's rule has a stop line eight metres back, so that is where
    // its signal applies — CARLA builds the stop boxes from `s` — while the pole
    // stays at the mouth.
    let north = document
        .road
        .iter()
        .find(|road| road.name.as_deref() == Some("north"))
        .expect("the north road");
    let light = north
        .signals
        .as_ref()
        .unwrap()
        .signal
        .iter()
        .find(|signal| signal.r#type == "1000001")
        .expect("the north light");
    let length = map
        .road(&roadgen_core::id::RoadId::new("north"))
        .unwrap()
        .horizontal_length()
        .unwrap();
    assert!(
        (light.s.value - (length - 8.0)).abs() < 1e-3,
        "s = {}",
        light.s.value
    );
    assert!((light.t.value + 4.1).abs() < 0.05, "t = {}", light.t.value);
    // And the stop line itself is painted: a bar of marking across the lane at
    // y = 22, which the map's meshes now include.
    let meshes = roadgen_carla::to_meshes(&map, &config);
    let bar = meshes
        .iter()
        .filter(|mesh| mesh.role == roadgen_carla::Role::Marking)
        .find(|mesh| {
            mesh.positions
                .iter()
                .all(|point| (point.y - 22.0).abs() < 0.5 && point.x <= 0.01 && point.x >= -3.51)
        })
        .expect("a painted stop line across the north approach's lane");
    assert!(bar.triangle_count() >= 2);

    // And each light's signal is placed where its actor will be spawned, which is
    // how CARLA's fifty-centimetre search finds it.
    for light in manifest["lights"].as_array().unwrap() {
        let id = light["signal"].as_str().unwrap();
        let signal = document
            .road
            .iter()
            .filter_map(|road| road.signals.as_ref())
            .flat_map(|signals| signals.signal.iter())
            .find(|signal| signal.id == id)
            .expect("the light's signal");
        let Some(opendrive::signal::position::Position::Inertial(inertial)) = &signal.choice else {
            panic!("no inertial position on {id}");
        };
        let position = light["position"].as_array().unwrap();
        assert!((inertial.x.value - position[0].as_f64().unwrap()).abs() < 1e-6);
        assert!((inertial.y.value - position[1].as_f64().unwrap()).abs() < 1e-6);
        assert!((inertial.z.value - position[2].as_f64().unwrap()).abs() < 1e-6);
    }
}

#[test]
fn a_package_without_furniture_leaves_the_signals_where_the_ir_put_them() {
    let map = signalised("Town01");
    let directory = tempfile::tempdir().expect("a temporary directory");
    let config = PackageConfig::for_map(&map).with_furniture(None);
    let package = roadgen_carla::write(&map, directory.path(), &config).expect("a package");
    assert_eq!((package.lights, package.signs), (0, 0));
    assert!(package.furniture.is_none() && package.map_logic.is_none());
    let xodr = fs::read_to_string(&package.xodr).expect("the road network");
    assert!(!xodr.contains("<positionInertial"));
    assert!(
        xodr.contains(r#"type="stop""#),
        "the code goes through verbatim"
    );
    let warnings = roadgen_carla::check(&map, &config);
    assert!(warnings
        .iter()
        .any(|warning| warning.contains("CARLA spawns its own")));
}
