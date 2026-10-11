//! A motorway generated from its rules: the structures it needs to cross the land,
//! its interchange onto a town, and what every format makes of it.

use std::sync::OnceLock;

use roadgen_carla::{materials, PackageConfig, Role};
use roadgen_core::geometry::plan::ring_contains;
use roadgen_core::prelude::*;
use roadgen_core::structure::{BridgeForm, TunnelForm, Underneath};
use roadgen_core::FormOfWay;
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
