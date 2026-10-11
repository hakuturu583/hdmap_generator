//! A motorway generated from its rules: the structures it needs to cross the land,
//! its interchange onto a town, and what every format makes of it.

use std::sync::OnceLock;

use roadgen_carla::{materials, PackageConfig, Role};
use roadgen_core::geometry::plan::ring_contains;
use roadgen_core::prelude::*;
use roadgen_core::structure::{BridgeForm, StructureKind, TunnelForm, Underneath};
use roadgen_core::{FormOfWay, MapObject, StructureId};
use roadgen_integration_tests::highway::build;

/// Built once: generating the land round five kilometres of motorway is the slow
/// part, and every test here reads the same map.
fn highway() -> &'static ValidatedMap {
    static MAP: OnceLock<ValidatedMap> = OnceLock::new();
    MAP.get_or_init(|| {
        let (map, report) = build(TrafficHandedness::RightHand);
        assert!(report.notes.is_empty(), "{:#?}", report.notes);
        map.validate().unwrap_or_else(|error| panic!("{error}"))
    })
}

fn structures_of(map: &Map, form: &str) -> Vec<String> {
    map.structures
        .iter()
        .filter(|s| s.kind.form() == form)
        .map(|s| s.id.to_string())
        .collect()
}

#[test]
fn the_river_is_bridged_and_the_ridge_tunnelled_on_both_carriageways() {
    let map = highway();
    // Over water a bridge is a bridge, not a viaduct, however long.
    let bridges = structures_of(map, BridgeForm::Bridge.as_str());
    assert_eq!(bridges.len(), 2, "{bridges:?}");
    for bridge in &bridges {
        assert!(
            map.grade_separations.iter().any(|g| g.via.as_str() == bridge
                && matches!(&g.under, Underneath::LandCover(cover) if cover.as_str().contains("ara_river"))),
            "{bridge} is over the river"
        );
    }
    // Twin bores through the ridge.
    let tunnels = structures_of(map, TunnelForm::Bored.as_str());
    assert_eq!(tunnels.len(), 2, "{tunnels:?}");
    for tunnel in map.structures.iter().filter(|s| s.kind.tunnel().is_some()) {
        let span = &tunnel.carries[0];
        assert!(
            span.length() > 300.0,
            "{} is {:.0} m",
            tunnel.id,
            span.length()
        );
    }
}

#[test]
fn the_crossroad_goes_over_the_motorway_on_an_overpass() {
    let map = highway();
    let overpass = map
        .structures
        .iter()
        .find(|s| s.kind.form() == "overpass")
        .expect("an overpass");
    assert!(overpass.carries[0].road.as_str().contains("kita_ic_cross"));
    // It passes over both carriageways.
    let under: Vec<_> = map
        .grade_separations
        .iter()
        .filter(|g| g.via == overpass.id)
        .filter_map(|g| match &g.under {
            Underneath::Road(span) => Some(span.road.to_string()),
            _ => None,
        })
        .collect();
    assert_eq!(under.len(), 2, "{under:?}");
    assert!(under.iter().any(|road| road.contains("e1_left")));
    assert!(under.iter().any(|road| road.contains("e1_right")));
}

#[test]
fn every_pier_stands_on_the_land() {
    let map = highway();
    let piers: Vec<_> = map
        .structures
        .iter()
        .filter_map(|s| s.kind.bridge())
        .flat_map(|bridge| bridge.supports.iter())
        .collect();
    assert!(piers.len() > 10);
    assert!(piers.iter().all(|pier| pier.base.is_some()));
}

#[test]
fn the_diamond_has_four_slip_roads_and_a_signal_on_each_off_ramp() {
    let map = highway();
    let interchange = map.interchanges.iter().next().expect("an interchange");
    assert_eq!(interchange.kind, "diamond");
    let slip: Vec<_> = interchange
        .roads
        .iter()
        .filter(|road| {
            map.road(road)
                .is_some_and(|r| r.form == FormOfWay::SlipRoad && !r.is_connector())
        })
        .collect();
    assert_eq!(slip.len(), 4, "{slip:?}");
    // Two terminals on the crossroad and four noses on the carriageways.
    assert_eq!(interchange.junctions.len(), 6);
    for junction in &interchange.junctions {
        assert_eq!(
            map.junction(junction).unwrap().interchange.as_ref(),
            Some(&interchange.id)
        );
    }
    let lights = map
        .objects
        .iter()
        .filter(|o| o.kind.is_traffic_light())
        .filter(|o| o.lanes.iter().any(|lane| lane.as_str().contains("_off/")))
        .count();
    assert_eq!(lights, 2);
}

#[test]
fn no_building_faces_the_motorway_or_stands_under_a_bridge() {
    let map = highway();
    assert!(!map.buildings.is_empty(), "the crossroad is a town street");
    for building in map.buildings.iter() {
        if let Some(frontage) = &building.frontage {
            let road = map.road(&frontage.road).unwrap();
            assert!(
                !road.form.is_controlled_access(),
                "{} faces {}",
                building.id,
                road.id
            );
        }
    }
    let decks: Vec<Vec<[f64; 2]>> = map
        .structures
        .iter()
        .filter(|s| s.kind.bridge().is_some())
        .map(|s| s.outline.points().iter().map(|p| [p.x, p.y]).collect())
        .collect();
    for part in map.building_parts.iter() {
        for p in part.solid.footprint.points() {
            assert!(
                !decks.iter().any(|deck| ring_contains(deck, [p.x, p.y])),
                "{} stands under a bridge",
                part.id
            );
        }
    }
}

#[test]
fn the_far_end_becomes_an_ordinary_road() {
    let map = highway();
    let forms: Vec<FormOfWay> = map
        .roads
        .iter()
        .filter(|road| road.id.as_str().starts_with("road/e1_") && !road.is_connector())
        .map(|road| road.form)
        .collect();
    assert!(forms.contains(&FormOfWay::MultipleCarriageway));
    assert!(forms.contains(&FormOfWay::Motorway));
    // On the way: an 80 km/h stage of the motorway, joined end to end to the
    // stretches either side of it with no junction between.
    let stages: Vec<_> = map
        .roads
        .iter()
        .filter(|road| {
            road.id.as_str().starts_with("road/e1_")
                && road
                    .speed_limit
                    .map(|limit| (limit.kph() - 80.0).abs() < 1e-6)
                    == Some(true)
        })
        .collect();
    assert_eq!(stages.len(), 2, "one on each carriageway");
    for stage in stages {
        assert_eq!(stage.form, FormOfWay::Motorway);
        for link in [&stage.link.predecessor, &stage.link.successor] {
            assert!(
                matches!(link, Some(roadgen_core::topology::RoadLinkTarget::Road(_))),
                "{link:?}"
            );
        }
        assert!(stage.junction.is_none());
    }
}

#[test]
fn the_corridor_is_a_motorway_site_with_its_cuttings_and_embankments() {
    let map = highway();
    let site = map
        .sites
        .iter()
        .find(|site| site.kind == "motorway")
        .expect("a motorway site");
    let kinds: Vec<&str> = site
        .land_covers
        .iter()
        .filter_map(|id| map.land_covers.get(id))
        .map(|cover| cover.kind.as_str())
        .collect();
    assert!(
        kinds.contains(&"cutting") && kinds.contains(&"embankment"),
        "{kinds:?}"
    );
}

#[test]
fn left_hand_traffic_builds_the_same_motorway_mirrored() {
    let (map, report) = build(TrafficHandedness::LeftHand);
    assert!(report.notes.is_empty(), "{:#?}", report.notes);
    let map = map.validate().unwrap_or_else(|error| panic!("{error}"));
    assert_eq!(structures_of(&map, "bored").len(), 2);
    assert_eq!(map.interchanges.len(), 1);
}

#[test]
fn every_format_writes_the_motorway() {
    let map = highway();
    let xml = roadgen_opendrive::to_xml(map).expect("OpenDRIVE");
    assert!(xml.contains("<bridge"));
    assert!(xml.contains("<tunnel"));
    let osm = roadgen_osm::to_xml(map).expect("OSM");
    for tag in [
        "\"bridge\"",
        "\"tunnel\"",
        "\"motorway_link\"",
        "\"motorway\"",
        "k=\"layer\"",
    ] {
        assert!(osm.contains(tag), "no {tag} in the OSM");
    }
    let directory = tempfile::tempdir().unwrap();
    roadgen_lanelet2::write(map, directory.path().join("e1.osm")).expect("Lanelet2");
    roadgen_sumo::to_plain_xml(map).expect("SUMO");
    let scene = roadgen_gpudrive::SceneConfig::for_map(map);
    roadgen_gpudrive::to_scene(map, &scene).expect("GPUDrive");
    let trace = roadgen_trace::IrCatalog::of(map);
    assert_eq!(trace.structures.len(), map.structures.len());
}

/// The parapets the map has along `structure`: its barriers that belong to it.
fn parapets_of<'m>(map: &'m Map, structure: &StructureId) -> Vec<&'m MapObject> {
    map.objects
        .iter()
        .filter(|object| {
            matches!(
                &object.kind,
                MapObjectKind::Barrier { kind, structure: Some(of), .. }
                    if kind == "parapet" && of == structure
            )
        })
        .collect()
}

#[test]
fn every_bridge_the_rules_build_stands_a_parapet_each_side_and_every_format_writes_them() {
    let map = highway();
    let mut all = 0;
    for structure in map.structures.iter() {
        let parapets = parapets_of(map, &structure.id).len();
        match &structure.kind {
            // The rules give each carriageway a deck of its own.
            StructureKind::Bridge(_) => {
                assert_eq!(parapets, 2 * structure.carries.len(), "{}", structure.id)
            }
            StructureKind::Tunnel(_) => assert_eq!(parapets, 0, "{}", structure.id),
        }
        all += parapets;
    }
    assert!(all > 0);

    let xml = roadgen_opendrive::to_xml(map).expect("OpenDRIVE");
    assert_eq!(xml.matches("subtype=\"parapet\"").count(), all);
    let osm = roadgen_osm::to_xml(map).expect("OSM");
    assert_eq!(osm.matches("v=\"wall\"").count(), all);
    let directory = tempfile::tempdir().unwrap();
    let path = directory.path().join("e1.osm");
    roadgen_lanelet2::write(map, &path).expect("Lanelet2");
    let lanelet2 = std::fs::read_to_string(&path).unwrap();
    assert_eq!(
        lanelet2.matches("v=\"parapet\"").count(),
        2 * all,
        "each a road border down both edges of its band"
    );

    // The IR dump says whose parapet each is, and a lookup steps from one to it.
    let document = roadgen_trace::IrDocument::of(map);
    let owned: Vec<_> = document
        .body
        .objects
        .iter()
        .filter(|object| object.structure.is_some())
        .collect();
    assert_eq!(owned.len(), all);
    let mut index = roadgen_trace::TraceIndex::new();
    index.add_ir(document.clone()).unwrap();
    for object in owned {
        let structure = object.structure.clone().unwrap();
        assert!(map.structures.contains(&StructureId::from_raw(&structure)));
        assert!(index.neighbours(&object.id).contains(&structure));
    }
}

#[test]
fn a_map_without_a_bridge_has_no_parapet_and_dumps_its_barriers_as_before() {
    let mut builder = MapBuilder::default();
    let road = builder
        .add_road(
            RoadSpec::line(
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(200.0, 0.0, 0.0),
                roadgen_integration_tests::highway::street_lanes(),
            )
            .unwrap()
            .with_name("haul"),
        )
        .unwrap();
    let berm = builder
        .add_barrier(&LaneRef::new(road, 0), "berm", 2.0)
        .unwrap();
    let map = builder.finish().unwrap().validate().unwrap();
    assert_eq!(map.objects.len(), 1, "the berm and nothing else");
    let json = serde_json::to_string(&roadgen_trace::IrCatalog::of(&map).objects).unwrap();
    assert_eq!(
        json,
        format!(
            "[{{\"id\":\"{berm}\",\"kind\":\"barrier\",\"code\":null,\"lanes\":[\"{}\"]}}]",
            map.objects.get(&berm).unwrap().lanes[0]
        )
    );
}

#[test]
fn carla_draws_the_decks_piers_and_tunnels() {
    let map = highway();
    let config = PackageConfig::new("Expressway");
    let meshes = roadgen_carla::to_meshes(map, &config);
    let concrete = meshes
        .iter()
        .filter(|mesh| mesh.role == Role::Terrain && mesh.materials == [materials::ROAD_CONCRETE])
        .count();
    assert_eq!(concrete, map.structures.len());
    // The land goes over the tunnels: higher than any road on the map.
    let terrain = meshes
        .iter()
        .find(|mesh| mesh.role == Role::Terrain && mesh.materials.contains(&materials::GRASS))
        .expect("land");
    let top = terrain
        .positions
        .iter()
        .map(|p| p.z)
        .fold(f64::MIN, f64::max);
    assert!(top > 40.0, "the ridge reaches {top:.1} m");
    // The river's channel is riverbed, and water lies over it below its banks.
    assert!(terrain.materials.contains(&materials::RIVERBED));
    let water: Vec<_> = meshes
        .iter()
        .filter(|mesh| mesh.materials == [materials::WATER])
        .collect();
    assert_eq!(water.len(), 1, "one river, one sheet of water");
    let level = water[0].positions[0].z;
    assert!(water[0]
        .positions
        .iter()
        .all(|p| (p.z - level).abs() < 1e-9));
    assert!(
        level > -6.0 && level < 0.0,
        "the water stands at {level:.2} m"
    );
}

#[test]
fn the_land_beside_the_motorway_is_no_strip_of_slivers() {
    // A sliver: a triangle whose height over its longest side is under a tenth of
    // that side. Where the road's edge and the slope's toe beside it had two
    // vertices a hand's breadth apart, every triangle on the pair was one.
    let sliver = |corners: [Point3; 3]| {
        let longest = (0..3)
            .map(|k| corners[k].horizontal_distance_to(corners[(k + 1) % 3]))
            .fold(0.0, f64::max);
        let [a, b, c] = corners;
        let area = ((b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x)).abs() / 2.0;
        2.0 * area / longest < 0.1 * longest
    };
    let map = highway();
    let land = map.tin_surfaces.iter().next().expect("land");
    let nodes = land.tin.nodes();
    let slivers = land
        .tin
        .triangles()
        .iter()
        .filter(|triangle| sliver(triangle.map(|i| nodes[i as usize])))
        .count();
    let triangles = land.tin.triangles().len();
    assert!(
        slivers * 1000 < triangles,
        "{slivers} of the land's {triangles} triangles are slivers"
    );
    // And CARLA's ground, which is the land re-triangulated round the roads.
    let meshes = roadgen_carla::to_meshes(map, &PackageConfig::new("Expressway"));
    let ground = meshes
        .iter()
        .find(|mesh| mesh.role == Role::Terrain && mesh.materials.contains(&materials::GRASS))
        .expect("land");
    let slivers = ground
        .triangles
        .iter()
        .filter(|triangle| sliver(triangle.map(|i| ground.positions[i as usize])))
        .count();
    let triangles = ground.triangles.len();
    assert!(
        slivers * 1000 < triangles,
        "{slivers} of the ground's {triangles} triangles are slivers"
    );
}

#[test]
fn every_building_stands_on_the_land_on_a_foundation_where_it_falls_away() {
    let map = highway();
    let tin = map.tin_surfaces.iter().next().expect("land").tin.locator();
    let mut founded = 0;
    for building in map.buildings.iter() {
        let parts: Vec<_> = building
            .parts
            .iter()
            .filter_map(|id| map.building_parts.get(id))
            .collect();
        founded += usize::from(
            parts
                .iter()
                .any(|p| p.kind.as_deref() == Some("foundation")),
        );
        let corners = || parts.iter().flat_map(|p| p.solid.footprint.points());
        let base = corners().map(|p| p.z).fold(f64::INFINITY, f64::min);
        let land = corners()
            .filter_map(|p| tin.height_at(p.x, p.y))
            .fold(f64::INFINITY, f64::min);
        assert!(
            base - land < 0.3,
            "{} floats {:.2} m over the land",
            building.id,
            base - land
        );
    }
    assert!(
        founded > 0,
        "the crossroad's approach leaves some building a foundation"
    );
}

#[test]
fn a_post_on_a_bridge_stands_on_its_parapet_not_in_the_air() {
    use roadgen_core::structure::{BridgeSpec, RoadSpan};
    use roadgen_core::topology::LaneEnd;
    let lane = |direction| LaneSpec::new(PositiveWidth::new(3.5).unwrap(), direction);
    let lanes = vec![lane(Direction::Backward), lane(Direction::Forward)];
    let mut builder = MapBuilder::default();
    let road = builder
        .add_road(
            RoadSpec::line(
                Point3::new(0.0, 0.0, 9.0),
                Point3::new(300.0, 0.0, 9.0),
                lanes,
            )
            .unwrap()
            .with_name("deck"),
        )
        .unwrap();
    builder
        .add_bridge(
            vec![RoadSpan::new(road.clone(), 0.0, 300.0)],
            BridgeSpec::default(),
        )
        .unwrap();
    builder
        .add_traffic_sign(&LaneRef::new(road, 1), LaneEnd::End, "stop", 2.0)
        .unwrap();
    let map = builder.finish().unwrap();
    let map = map.as_map();
    let bridge = map.structures.iter().next().unwrap();
    let curb = bridge.kind.bridge().unwrap().curb;
    let furniture = roadgen_carla::furniture::build(
        map,
        "deck",
        &PackageConfig::new("deck").surfaces,
        &roadgen_carla::FurnitureConfig::default(),
    );
    let [sign] = furniture.signs.as_slice() else {
        panic!("one sign: {furniture:?}");
    };
    // On the parapet over the deck's curb, 3.5 m out: inside the deck, and up
    // on the parapet rather than level with the road.
    let out = sign.position.y.abs();
    assert!(out > 3.5 && out < 3.5 + curb, "{:?}", sign.position);
    assert!((sign.position.z - (9.0 + roadgen_carla::structures::PARAPET)).abs() < 1e-6);
}

#[test]
fn a_river_falls_along_its_length_and_its_water_with_it() {
    use roadgen_carla::environment::{self, Places, RiverSpec};
    let mut builder = MapBuilder::default();
    builder
        .add_road(
            RoadSpec::line(
                Point3::new(-200.0, -300.0, 6.0),
                Point3::new(-200.0, 300.0, 6.0),
                roadgen_integration_tests::highway::street_lanes(),
            )
            .unwrap()
            .with_name("bank"),
        )
        .unwrap();
    // Down from 12 m at its source to 0 at its mouth, with a rise on the way
    // that the bed does not follow.
    let places = Places {
        rivers: vec![RiverSpec {
            name: "Tama".into(),
            centre: vec![
                Point3::new(200.0, -400.0, 12.0),
                Point3::new(200.0, -100.0, 6.0),
                Point3::new(200.0, 100.0, 8.0),
                Point3::new(200.0, 400.0, 0.0),
            ],
            width: 40.0,
            depth: 4.0,
            bank_slope: 2.0,
            site: None,
        }],
        ..Places::default()
    };
    let mut map = builder.finish().unwrap();
    environment::generate(&mut map, &places).expect("generates");
    let map = map.validate().unwrap_or_else(|error| panic!("{error}"));
    let tin = map.tin_surfaces.iter().next().unwrap().tin.locator();
    let beds: Vec<f64> = [-350.0, -200.0, 0.0, 150.0, 350.0]
        .iter()
        .map(|&y| tin.height_at(200.0, y).unwrap())
        .collect();
    assert!(beds.windows(2).all(|w| w[1] <= w[0] + 1e-6), "{beds:?}");
    assert!(beds[0] - beds[4] > 8.0, "{beds:?}");
    let meshes = roadgen_carla::to_meshes(&map, &PackageConfig::new("tama"));
    let water = meshes
        .iter()
        .find(|mesh| mesh.materials == [materials::WATER])
        .expect("water");
    // Level across the river, down along it, and over the bed everywhere.
    let mut along: Vec<&Point3> = water.positions.iter().collect();
    along.sort_by(|a, b| a.y.total_cmp(&b.y));
    // Short of its two ends, where the channel stops and the land does not.
    for p in along.iter().filter(|p| p.y.abs() < 380.0) {
        let bed = tin.height_at(200.0, p.y).unwrap();
        assert!(p.z > bed && p.z < bed + 4.0, "{p:?} over a bed at {bed:.2}");
    }
    assert!(along.first().unwrap().z - along.last().unwrap().z > 8.0);

    // The same hollow as a lake: standing water, one level all over.
    let mut lake = (*map).clone();
    let ids: Vec<_> = lake
        .land_covers
        .iter()
        .map(|cover| cover.id.clone())
        .collect();
    for id in ids {
        if let Some(cover) = lake.land_covers.get_mut(&id) {
            if cover.kind == "water" {
                cover.kind = "lake".into();
            }
        }
    }
    let lake = UnvalidatedMap::from_map(lake)
        .validate()
        .unwrap_or_else(|error| panic!("{error}"));
    let meshes = roadgen_carla::to_meshes(&lake, &PackageConfig::new("tama"));
    let water = meshes
        .iter()
        .find(|mesh| mesh.materials == [materials::WATER])
        .expect("water");
    let level = water.positions[0].z;
    assert!(water.positions.iter().all(|p| (p.z - level).abs() < 1e-9));
}

#[test]
fn one_bridge_over_both_carriageways_is_one_deck_across_the_median() {
    use roadgen_core::structure::{BridgeSpec, RoadSpan};
    let one_way = || {
        vec![LaneSpec::new(
            PositiveWidth::new(3.5).unwrap(),
            Direction::Forward,
        )]
    };
    let mut builder = MapBuilder::default();
    // Drawn opposite ways, each lane outside its line: a 12 m median between.
    let east = builder
        .add_road(
            RoadSpec::line(
                Point3::new(0.0, -6.0, 9.0),
                Point3::new(300.0, -6.0, 9.0),
                one_way(),
            )
            .unwrap()
            .with_name("east"),
        )
        .unwrap();
    let west = builder
        .add_road(
            RoadSpec::line(
                Point3::new(300.0, 6.0, 9.0),
                Point3::new(0.0, 6.0, 9.0),
                one_way(),
            )
            .unwrap()
            .with_name("west"),
        )
        .unwrap();
    builder
        .add_bridge(
            vec![
                RoadSpan::new(east, 0.0, 300.0),
                RoadSpan::new(west, 0.0, 300.0),
            ],
            BridgeSpec::default(),
        )
        .unwrap();
    let map = builder
        .finish()
        .unwrap()
        .validate()
        .unwrap_or_else(|e| panic!("{e}"));
    let meshes = roadgen_carla::to_meshes(&map, &PackageConfig::new("deck"));
    let deck: Vec<Point3> = meshes
        .iter()
        .filter(|mesh| mesh.role == Role::Terrain && mesh.materials == [materials::ROAD_CONCRETE])
        .flat_map(|mesh| mesh.positions.iter().copied())
        .collect();
    assert!(!deck.is_empty());
    let parapet = 9.0 + roadgen_carla::structures::PARAPET;
    // Parapets along the two outer edges only, none down the median.
    assert!(deck
        .iter()
        .any(|p| (p.z - parapet).abs() < 1e-6 && p.y.abs() > 9.0));
    assert!(
        !deck
            .iter()
            .any(|p| (p.z - parapet).abs() < 1e-6 && p.y.abs() < 6.0 - 1e-6),
        "a parapet down the median"
    );
    // The slab is there across the middle of the median, at the road's height.
    assert!(deck
        .iter()
        .any(|p| p.y.abs() < 1e-6 && (p.z - 9.0).abs() < 1e-6));
}

#[test]
fn on_a_shared_deck_the_river_is_crossed_by_one_bridge_carrying_both_carriageways() {
    use roadgen_core::structure::SupportKind;
    let mut places = roadgen_integration_tests::highway::places();
    places.corridors[0].shared_deck = true;
    let median = places.corridors[0].standard.median;
    let mut builder = MapBuilder::default();
    places.corridors[0]
        .plan(TrafficHandedness::RightHand)
        .expect("plans")
        .apply(&mut builder, &mut places)
        .expect("applies");
    let mut map = builder.finish().expect("builds");
    let report = roadgen_carla::environment::generate(&mut map, &places).expect("generates");
    assert!(report.notes.is_empty(), "{:#?}", report.notes);
    let map = map.validate().unwrap_or_else(|error| panic!("{error}"));

    let bridges: Vec<_> = map
        .structures
        .iter()
        .filter(|s| s.kind.form() == BridgeForm::Bridge.as_str())
        .collect();
    assert_eq!(bridges.len(), 1, "one deck over the river");
    let bridge = bridges[0];
    assert_eq!(bridge.carries.len(), 2);
    assert_ne!(bridge.carries[0].road, bridge.carries[1].road);
    // Both carriageways cross the river through it.
    let over_river = map
        .grade_separations
        .iter()
        .filter(|g| {
            g.via == bridge.id
                && matches!(&g.under, Underneath::LandCover(cover) if cover.as_str().contains("ara_river"))
        })
        .count();
    assert_eq!(over_river, 2);
    // Its two halves begin and end across from each other: the union of where
    // each carriageway would be bridged.
    let ends: Vec<[f64; 2]> = bridge
        .carries
        .iter()
        .map(|span| {
            let road = map.road(&span.road).unwrap();
            let x = |s: f64| {
                road.reference_line
                    .sample_at(s, map.metadata.sampling)
                    .unwrap()
                    .point
                    .x
            };
            let (a, b) = (x(span.from), x(span.to));
            [a.min(b), a.max(b)]
        })
        .collect();
    for k in 0..2 {
        assert!(
            (ends[0][k] - ends[1][k]).abs() < 6.0,
            "the halves end apart: {ends:?}"
        );
    }
    // Its piers stand down the median.
    let piers: Vec<_> = bridge
        .kind
        .bridge()
        .unwrap()
        .supports
        .iter()
        .filter(|s| s.kind == SupportKind::Pier)
        .collect();
    assert!(!piers.is_empty());
    for pier in piers {
        let c = pier.footprint.centroid();
        assert!(c.y.abs() < median / 2.0, "a pier off the median at {c:?}");
    }
    // The ridge's twin bores stay two tunnels.
    assert_eq!(structures_of(&map, TunnelForm::Bored.as_str()).len(), 2);
}

/// An ordinary two-way road, no motorway and of no corridor, along `points`, and
/// what is generated onto it: its structures chosen from the land when
/// `structured`, with `places` round it, and on the site named `site` if any.
fn ordinary_road(
    points: &[Point3],
    places: roadgen_carla::environment::Places,
    structured: bool,
    site: Option<&str>,
) -> (UnvalidatedMap, roadgen_carla::environment::Report) {
    let mut builder = MapBuilder::default();
    let road = builder
        .add_road(
            RoadSpec::new(
                Curve3::polyline(points.iter().copied()).unwrap(),
                roadgen_integration_tests::highway::street_lanes(),
            )
            .with_name("pass"),
        )
        .unwrap();
    let mut places = places;
    if let Some(site) = site {
        places.site_roads.push((road.clone(), site.into()));
    }
    if structured {
        places.structured_roads.push(road);
    }
    let mut map = builder.finish().unwrap();
    let report = roadgen_carla::environment::generate(&mut map, &places).expect("generates");
    (map, report)
}

/// A road high across a river's valley, down to the country either side.
fn across_the_valley(
    structured: bool,
    site: Option<&str>,
) -> (UnvalidatedMap, roadgen_carla::environment::Report) {
    use roadgen_carla::environment::{Places, RiverSpec};
    ordinary_road(
        &[
            Point3::new(-600.0, 0.0, 0.0),
            Point3::new(-200.0, 0.0, 12.0),
            Point3::new(200.0, 0.0, 12.0),
            Point3::new(600.0, 0.0, 0.0),
        ],
        Places {
            rivers: vec![RiverSpec {
                name: "Naka".into(),
                centre: vec![Point3::new(0.0, -400.0, 0.0), Point3::new(0.0, 400.0, 0.0)],
                width: 60.0,
                depth: 6.0,
                bank_slope: 2.0,
                site: None,
            }],
            ..Places::default()
        },
        structured,
        site,
    )
}

/// A road level through a hill.
fn through_the_hill(structured: bool) -> (UnvalidatedMap, roadgen_carla::environment::Report) {
    use roadgen_carla::environment::{HillSpec, Places};
    ordinary_road(
        &[Point3::new(-800.0, 0.0, 0.0), Point3::new(800.0, 0.0, 0.0)],
        Places {
            hills: vec![HillSpec {
                name: "Kobu".into(),
                crest: vec![Point3::new(0.0, 0.0, 0.0)],
                width: 900.0,
                height: 60.0,
            }],
            ..Places::default()
        },
        structured,
        None,
    )
}

/// One concrete solid for each structure in CARLA's meshes.
fn carla_draws_every_structure(map: &ValidatedMap) {
    let meshes = roadgen_carla::to_meshes(map, &PackageConfig::new("pass"));
    let concrete = meshes
        .iter()
        .filter(|mesh| mesh.role == Role::Terrain && mesh.materials == [materials::ROAD_CONCRETE])
        .count();
    assert_eq!(concrete, map.structures.len());
}

#[test]
fn an_ordinary_road_asked_to_have_its_structures_chosen_bridges_a_river_valley() {
    let (map, report) = across_the_valley(true, None);
    assert!(report.notes.is_empty(), "{:#?}", report.notes);
    let map = map.validate().unwrap_or_else(|error| panic!("{error}"));
    let bridges = structures_of(&map, BridgeForm::Bridge.as_str());
    assert_eq!(bridges.len(), 1, "{bridges:?}");
    assert_eq!(map.structures.len(), 1);
    assert!(map.grade_separations.iter().any(|g| g.via.as_str() == bridges[0]
        && matches!(&g.under, Underneath::LandCover(cover) if cover.as_str().contains("naka"))));
    // It reaches from where the road is well above the land on one side to the
    // other, and the rest is on embankments of no site.
    let span = &map.structures.iter().next().unwrap().carries[0];
    assert!(span.length() > 300.0, "{:.0} m", span.length());
    let embankments: Vec<_> = map
        .land_covers
        .iter()
        .filter(|cover| cover.kind == "embankment")
        .collect();
    assert!(!embankments.is_empty());
    assert!(
        map.sites.is_empty(),
        "no motorway site for an ordinary road"
    );
    let road = map.roads.iter().next().unwrap();
    assert!(!road.form.is_controlled_access());
    carla_draws_every_structure(&map);
}

#[test]
fn an_ordinary_road_asked_to_have_its_structures_chosen_tunnels_through_a_hill() {
    let (map, report) = through_the_hill(true);
    assert!(report.notes.is_empty(), "{:#?}", report.notes);
    let map = map.validate().unwrap_or_else(|error| panic!("{error}"));
    let tunnels = structures_of(&map, TunnelForm::Bored.as_str());
    assert_eq!(tunnels.len(), 1, "{tunnels:?}");
    let tunnel = map.structures.iter().next().unwrap();
    let span = &tunnel.carries[0];
    assert!(span.length() > 200.0, "{:.0} m", span.length());
    let portals = &tunnel.kind.tunnel().unwrap().portals;
    assert!(portals[0].station < portals[1].station);
    // Cut down to the portals either side.
    assert!(map.land_covers.iter().any(|cover| cover.kind == "cutting"));
    carla_draws_every_structure(&map);
}

#[test]
fn an_ordinary_road_not_asked_to_have_its_structures_chosen_gets_none() {
    for (map, report) in [across_the_valley(false, None), through_the_hill(false)] {
        assert_eq!(report.structures, 0);
        assert!(map.as_map().structures.is_empty());
        assert!(!map
            .as_map()
            .land_covers
            .iter()
            .any(|cover| cover.kind == "embankment" || cover.kind == "cutting"));
    }
}

#[test]
fn the_cuttings_and_embankments_of_an_ordinary_road_are_on_its_own_site() {
    let (map, report) = across_the_valley(true, Some("Quarry Haul"));
    assert!(report.notes.is_empty(), "{:#?}", report.notes);
    let map = map.validate().unwrap_or_else(|error| panic!("{error}"));
    let site = map.sites.iter().next().expect("the road's site");
    assert_eq!(map.sites.len(), 1);
    assert_eq!(site.kind, "works");
    let embankments = map
        .land_covers
        .iter()
        .filter(|cover| cover.kind == "embankment")
        .count();
    assert!(embankments > 0);
    let on_site = site
        .land_covers
        .iter()
        .filter_map(|id| map.land_covers.get(id))
        .filter(|cover| cover.kind == "embankment")
        .count();
    assert_eq!(on_site, embankments);
}
