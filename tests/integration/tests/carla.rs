//! Exporting to CARLA, checked against the rest of the workspace.
//!
//! The crate's own tests check that a package is well formed and that its mesh names
//! classify the way CARLA classifies them. These check the two things that are only
//! visible from outside it.
//!
//! **A CARLA map is two files that have to agree.** The `.xodr` is what the traffic
//! drives and the `.fbx` is what the sensors see, and they are read by different parts
//! of the simulator. So the road network in the package is compared against the one
//! the OpenDRIVE exporter writes on its own, and the surface is compared against the
//! road network it was built from.
//!
//! **The surface is read back rather than trusted.** Every picture here comes from
//! `roadgen-viewer` reading the written FBX, which is the same path the demo page
//! takes — so a test that passes is a file a reader can make sense of.

use roadgen_carla::{BuildingPlacement, Label, PackageConfig, Role};
use roadgen_core::prelude::*;
use roadgen_integration_tests::{redraw_opendrive, scenarios};
use roadgen_viewer::{Bounds, Kind};

/// Writes a package into a fresh directory and hands back both, so the directory
/// lives as long as the test does.
fn write(
    map: &ValidatedMap,
    config: &PackageConfig,
) -> (tempfile::TempDir, roadgen_carla::Package) {
    let directory = tempfile::tempdir().expect("a temporary directory");
    let package =
        roadgen_carla::write(map, directory.path(), config).expect("the map should export");
    (directory, package)
}

/// The package's FBX, read back and drawn — the same path the demo page takes.
fn redraw(map: &ValidatedMap, config: &PackageConfig) -> roadgen_viewer::Drawing {
    let (_directory, package) = write(map, config);
    roadgen_viewer::fbx_file(&package.fbx).expect("the export should draw")
}

fn contains(outer: Bounds, inner: Bounds) -> bool {
    outer.min.x <= inner.min.x + 1e-6
        && outer.min.y <= inner.min.y + 1e-6
        && outer.max.x >= inner.max.x - 1e-6
        && outer.max.y >= inner.max.y - 1e-6
}

#[test]
fn the_road_network_in_a_package_is_the_one_the_opendrive_export_writes() {
    // Not "equivalent to": the same document. The package does not write a second
    // OpenDRIVE, it calls the OpenDRIVE exporter — because two writers for one format
    // is two chances to disagree about the map CARLA drives and the map it draws.
    for map in [
        scenarios::crossroads(),
        scenarios::graded_road(),
        scenarios::banked_curve(),
    ] {
        let config = PackageConfig::for_map(&map);
        let (_directory, package) = write(&map, &config);
        let written = std::fs::read_to_string(&package.xodr).expect("the road network");
        let alone = roadgen_opendrive::to_xml(&map).expect("the map should export");
        assert_eq!(written, alone);
    }
}

#[test]
fn every_scenario_exports_and_reads_back_as_a_surface() {
    for map in [
        scenarios::straight_road(),
        scenarios::bidirectional_road(),
        scenarios::multi_lane_road(),
        scenarios::two_roads_joined(),
        scenarios::split(),
        scenarios::merge(),
        scenarios::crossroads(),
        scenarios::graded_road(),
        scenarios::spiral_transition_road(),
        scenarios::banked_curve(),
        scenarios::lane_drop(),
        scenarios::widening_road(),
        scenarios::controlled_crossroads(),
    ] {
        let config = PackageConfig::for_map(&map);
        let drawing = redraw(&map, &config);
        assert!(
            !drawing.shapes.is_empty(),
            "{:?} exported a surface with nothing in it",
            map.metadata.name
        );
        // Every mesh in a map is one of the four classes CARLA's import will place.
        // A fifth would be a mesh that arrives in the content browser and never in
        // the level.
        for kind in drawing.kinds() {
            assert!(
                matches!(
                    kind,
                    Kind::Surface | Kind::Sidewalk | Kind::Terrain | Kind::Marking(_)
                ),
                "{:?} produced a {kind:?}, which CARLA's map import has no folder for",
                map.metadata.name
            );
        }
    }
}

#[test]
fn the_surface_covers_the_road_network_it_was_built_from() {
    // The two files are read by different parts of CARLA and have to describe the same
    // place, so the surface is compared against the road network rather than against
    // itself: the mesh should cover every road the OpenDRIVE holds, and reach further
    // by the verge beside them.
    let map = scenarios::crossroads();
    let mut config = PackageConfig::for_map(&map);

    let network = redraw_opendrive(&map).bounds().expect("a road network");
    let surface = redraw(&map, &config).bounds().expect("a surface");

    assert!(
        contains(surface, network),
        "the surface {surface:?} does not cover the road network {network:?}"
    );
    // And not by an unbounded amount: past the verge there is the ground, written
    // out to a stated distance for a lidar to reach, and nothing beyond that.
    let reach =
        config.surfaces.ground_extent + config.surfaces.verge_width + config.surfaces.ground_cell;
    assert!(surface.min.x >= network.min.x - reach);
    assert!(surface.max.x <= network.max.x + reach);
    // Without it, the verge is the only thing beyond the road.
    config.surfaces.ground_extent = 0.0;
    let surface = redraw(&map, &config).bounds().expect("a surface");
    let reach = config.surfaces.verge_width + 1.0;
    assert!(surface.min.x >= network.min.x - reach);
    assert!(surface.max.x <= network.max.x + reach);
}

#[test]
fn the_ground_goes_out_as_far_as_a_lidar_does() {
    // A long-range lidar sees about 200 m; the ground past the last road is there so
    // that every return it makes lands on something, and it sits just under the
    // roads rather than on them.
    let map = scenarios::crossroads();
    let config = PackageConfig::for_map(&map);
    let meshes = roadgen_carla::to_meshes(&map, &config);
    let ground = meshes
        .iter()
        .find(|mesh| mesh.role == Role::Ground)
        .expect("a ground mesh");
    assert_eq!(roadgen_carla::tags::label_of(&ground.name), Label::Terrain);
    let network = redraw_opendrive(&map).bounds().expect("a road network");
    let far = ground
        .positions
        .iter()
        .map(|point| point.x)
        .fold(f64::NEG_INFINITY, f64::max);
    let edge = network.max.x + config.surfaces.verge_width + config.surfaces.ground_extent;
    assert!(
        far >= edge && far < edge + config.surfaces.ground_cell,
        "the ground reaches x = {far}, the network ends at {}",
        network.max.x
    );
    let roads: Vec<f64> = meshes
        .iter()
        .filter(|mesh| mesh.role == Role::Road)
        .flat_map(|mesh| mesh.positions.iter().map(|point| point.z))
        .collect();
    let lowest_road = roads.iter().copied().fold(f64::INFINITY, f64::min);
    let highest_ground = ground
        .positions
        .iter()
        .map(|point| point.z)
        .fold(f64::NEG_INFINITY, f64::max);
    assert!(
        highest_ground < lowest_road,
        "the ground ({highest_ground}) pokes through the road ({lowest_road})"
    );
}

#[test]
fn a_map_with_no_verge_stops_where_the_road_stops() {
    let map = scenarios::straight_road();
    let mut config = PackageConfig::for_map(&map);
    config.surfaces.verge_width = 0.0;
    config.surfaces.ground_extent = 0.0;

    let network = redraw_opendrive(&map).bounds().expect("a road network");
    let surface = redraw(&map, &config).bounds().expect("a surface");
    // Within a metre: the OpenDRIVE drawing evaluates lane widths out of the document
    // and this reads vertices, so the two agree about the edge of the road rather than
    // about every sample along it.
    assert!((surface.max.y - network.max.y).abs() < 1.0);

    // And the report says the roads now stand on nothing, because they do.
    let warnings = roadgen_carla::check(&map, &config);
    assert!(warnings
        .iter()
        .any(|warning| warning.contains("no ground at all")));
}

#[test]
fn a_climbing_road_takes_its_surface_up_with_it() {
    // The drawing is a plan view, so the heights are checked on the meshes rather than
    // on the picture. A surface generated flat under a graded road is the classic way
    // an imported map has its road floating over its own terrain.
    let map = scenarios::graded_road();
    let config = PackageConfig::for_map(&map);
    let meshes = roadgen_carla::to_meshes(&map, &config);

    let heights: Vec<f64> = meshes
        .iter()
        .flat_map(|mesh| mesh.positions.iter().map(|point| point.z))
        .collect();
    let lowest = heights.iter().copied().fold(f64::INFINITY, f64::min);
    let highest = heights.iter().copied().fold(f64::NEG_INFINITY, f64::max);
    assert!(
        highest - lowest > 1.0,
        "a graded road produced a surface {lowest}..{highest} high"
    );

    // Including the grass beside it: a verge level with the map's origin would leave
    // the road on a ledge.
    let verge: Vec<f64> = meshes
        .iter()
        .filter(|mesh| mesh.role == Role::Terrain)
        .flat_map(|mesh| mesh.positions.iter().map(|point| point.z))
        .collect();
    let span = verge.iter().copied().fold(f64::NEG_INFINITY, f64::max)
        - verge.iter().copied().fold(f64::INFINITY, f64::min);
    assert!(span > 1.0, "the ground beside a climbing road is flat");
}

#[test]
fn a_junction_writes_a_surface_per_movement_and_says_so() {
    // A junction in the IR is a connector road per movement, and each carries its own
    // carriageway — so the middle of a junction is written once per turn through it.
    // CARLA's own maps have one junction surface; a road network has no way to say so.
    // The overlap is real, it z-fights, and the report is where that is said.
    let map = scenarios::crossroads();
    let config = PackageConfig::for_map(&map);

    let connectors = map.roads.iter().filter(|road| road.is_connector()).count();
    assert!(connectors > 1, "the fixture has no junction to overlap");

    let warnings = roadgen_carla::check(&map, &config);
    assert!(
        warnings
            .iter()
            .any(|warning| warning.contains("z-fight") && warning.contains("junction")),
        "the overlap inside a junction went unreported: {warnings:#?}"
    );
}

#[test]
fn a_town_is_the_one_thing_the_import_cannot_both_place_and_tag() {
    // The finding the whole exporter turns on, pinned from outside the crate: CARLA's
    // MoveAssets commandlet sorts a map's meshes into four folders and puts everything
    // it does not recognise into Terrain, and PrepareAssetsForCooking places what is
    // in those four folders and nothing else. A building can be in the level or
    // correctly tagged. Both placements are written, and both are reported.
    let mut unvalidated = scenarios::crossroads_builder("town", 60.0)
        .finish()
        .expect("the scenario should build");
    roadgen_buildings::generate(&mut unvalidated, &roadgen_buildings::Rules::default())
        .expect("a town should generate");
    let map = unvalidated
        .validate()
        .expect("the scenario should validate");
    assert!(!map.buildings.is_empty(), "the fixture grew no town");

    let in_map = PackageConfig::for_map(&map).with_buildings(BuildingPlacement::InMap);
    let (_directory, placed) = write(&map, &in_map);
    // In the map, and counted as ground.
    assert_eq!(placed.labels.get(&Label::Buildings), None);
    assert!(placed.labels.get(&Label::Terrain).copied().unwrap_or(0) >= map.buildings.len());

    let as_props = PackageConfig::for_map(&map).with_buildings(BuildingPlacement::Props);
    let (_directory, propped) = write(&map, &as_props);
    assert!(propped.props.is_some(), "no props file was written");
    // Tagged, and nowhere in the level.
    assert_eq!(propped.labels.get(&Label::Buildings), None);
    assert!(propped.meshes < placed.meshes);

    for config in [&in_map, &as_props] {
        let warnings = roadgen_carla::check(&map, config);
        assert!(
            warnings
                .iter()
                .any(|warning| warning.contains("buildings") && warning.contains("Buildings")),
            "what CARLA will do with the town went unsaid: {warnings:#?}"
        );
    }
}

#[test]
fn the_town_drawn_from_a_package_is_the_town_the_map_holds() {
    let mut unvalidated = scenarios::crossroads_builder("town", 60.0)
        .finish()
        .expect("the scenario should build");
    roadgen_buildings::generate(&mut unvalidated, &roadgen_buildings::Rules::default())
        .expect("a town should generate");
    let map = unvalidated
        .validate()
        .expect("the scenario should validate");

    let config = PackageConfig::for_map(&map);
    let drawing = redraw(&map, &config);
    // Drawn as ground, because that is what CARLA will call them — the picture shows
    // the mislabelling rather than hiding it — and the note says how many.
    assert!(drawing.kinds().contains(&Kind::Terrain));
    assert!(
        drawing
            .notes
            .iter()
            .any(|note| note.contains("triangles") && note.contains("silhouette")),
        "the drawing did not say how the closed solids were drawn: {:#?}",
        drawing.notes
    );
}

#[test]
fn a_package_names_every_mesh_after_its_map() {
    // Which is what makes the map's name decide the whole map's tagging, and what
    // `check` is guarding when it refuses a name.
    let map = scenarios::crossroads();
    let config = PackageConfig::for_map(&map).with_package("MyPackage");
    let meshes = roadgen_carla::to_meshes(&map, &config);
    assert!(!meshes.is_empty());
    for mesh in &meshes {
        assert!(
            mesh.name.starts_with(&format!("{}_", config.map)),
            "{} is not named for the map",
            mesh.name
        );
    }
}

#[test]
fn nothing_a_generated_map_produces_is_a_name_carla_throws_away() {
    // `ValidateStaticMesh` drops any mesh whose name holds `light` or `sign`, without
    // logging it. A road called `Sign Street` is the caller's problem and `check` says
    // so; a mesh name this exporter made up would be the exporter's.
    let map = scenarios::controlled_crossroads();
    let config = PackageConfig::for_map(&map);
    for mesh in roadgen_carla::to_meshes(&map, &config) {
        assert_eq!(
            roadgen_carla::tags::is_rejected_by_import(&mesh.name),
            None,
            "{} would be dropped before it reached the map",
            mesh.name
        );
    }
    assert!(
        roadgen_carla::check(&map, &config)
            .iter()
            .all(|warning| !warning.contains("mesh names hold")),
        "the report claims a name will be dropped when none will be"
    );
}

#[test]
fn a_painted_line_comes_back_painted() {
    // The colours survive the round trip through the file, and they survive it the way
    // CARLA reads them: off the material slot's name, which is what
    // `PrepareAssetsForCooking` matches `Yellow` against.
    let map = scenarios::bidirectional_road();
    let config = PackageConfig::for_map(&map);
    let drawing = redraw(&map, &config);
    assert!(
        drawing
            .kinds()
            .iter()
            .any(|kind| matches!(kind, Kind::Marking(_))),
        "a two-way road came back with no paint on it"
    );
}

#[test]
fn two_roads_that_meet_at_an_angle_share_one_edge() {
    // Where two roads meet at an angle the IR mitres the joint, so the lane boundaries
    // of one end on the same line the other's begin on. The surface has to be cut
    // along that same line: cut perpendicular to each reference line instead, the two
    // roads leave a wedge of nothing on the outside of the kink and overlap on the
    // inside — which is what a camera sees, so it is checked on the mesh and not on
    // the network. Every driving-lane boundary point at either end of every road has
    // to be a vertex of the road surface.
    let map = scenarios::two_roads_joined();
    let config = PackageConfig::for_map(&map);
    let meshes = roadgen_carla::to_meshes(&map, &config);
    let surface: Vec<Point3> = meshes
        .iter()
        .filter(|mesh| mesh.role == Role::Road)
        .flat_map(|mesh| mesh.positions.iter().copied())
        .collect();

    let mut missing = Vec::new();
    for lane in map.lanes.iter() {
        if lane.lane_type != LaneType::Driving {
            continue;
        }
        for (which, point) in [
            ("left start", lane.left_boundary.start_point()),
            ("left end", lane.left_boundary.end_point()),
            ("right start", lane.right_boundary.start_point()),
            ("right end", lane.right_boundary.end_point()),
        ] {
            let nearest = surface
                .iter()
                .map(|vertex| vertex.distance_to(point))
                .fold(f64::INFINITY, f64::min);
            if nearest > 1e-6 {
                missing.push(format!(
                    "{} of {} is {nearest:.3} m from the surface",
                    which, lane.id
                ));
            }
        }
    }
    assert!(
        missing.is_empty(),
        "the surface does not pass through the lane boundaries at the joint:\n  {}",
        missing.join("\n  ")
    );
}

#[test]
fn the_ground_stays_under_every_road_everywhere() {
    // Not just the highest ground under the lowest road: at every vertex of every
    // road surface, the ground directly beneath it is lower. A ground that comes up
    // through a road anywhere is a road a vehicle falls through.
    for (name, map) in [
        ("joined", scenarios::two_roads_joined()),
        ("graded", scenarios::graded_road()),
        ("crossroads", scenarios::crossroads()),
        ("banked", scenarios::banked_curve()),
        ("spiral", scenarios::spiral_transition_road()),
    ] {
        ground_stays_under(name, &map);
    }
}

fn ground_stays_under(name: &str, map: &ValidatedMap) {
    let config = PackageConfig::for_map(map);
    let meshes = roadgen_carla::to_meshes(map, &config);
    let ground = meshes
        .iter()
        .find(|mesh| mesh.role == Role::Ground)
        .expect("a ground mesh");
    let height_under = |x: f64, y: f64| -> Option<f64> {
        for triangle in &ground.triangles {
            let [a, b, c] = triangle.map(|index| ground.positions[index as usize]);
            let det = (b.x - a.x) * (c.y - a.y) - (c.x - a.x) * (b.y - a.y);
            if det.abs() < 1e-12 {
                continue;
            }
            let u = ((x - a.x) * (c.y - a.y) - (c.x - a.x) * (y - a.y)) / det;
            let v = ((b.x - a.x) * (y - a.y) - (x - a.x) * (b.y - a.y)) / det;
            if u >= -1e-9 && v >= -1e-9 && u + v <= 1.0 + 1e-9 {
                return Some(a.z + u * (b.z - a.z) + v * (c.z - a.z));
            }
        }
        None
    };
    // Roads and verges both: the verge is the ground's neighbour, and the one a
    // vehicle drives onto first.
    let mut worst: Option<(f64, Point3)> = None;
    for mesh in meshes
        .iter()
        .filter(|mesh| mesh.role == Role::Road || mesh.role == Role::Terrain)
    {
        for point in &mesh.positions {
            let under = height_under(point.x, point.y).expect("ground under every road vertex");
            let clearance = point.z - under;
            if worst.is_none_or(|(c, _)| clearance < c) {
                worst = Some((clearance, *point));
            }
        }
    }
    let (clearance, at) = worst.expect("road vertices");
    assert!(
        clearance > 0.0,
        "{name}: the ground is {} m above the surface at {at:?}",
        -clearance
    );
}
