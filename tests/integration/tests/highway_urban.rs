//! A motorway through a town: lifted onto a viaduct over the streets, or sunk into
//! a cut-and-cover tunnel under them, with slip roads down to street level.

use roadgen_buildings::Rules;
use roadgen_carla::environment::{
    self, Control, CorridorSpec, InterchangeKind, InterchangeSite, Places, RampSite, Side, Town,
    Urban,
};
use roadgen_core::geometry::plan::ring_contains;
use roadgen_core::prelude::*;
use roadgen_core::structure::Underneath;
use roadgen_core::FormOfWay;
use roadgen_integration_tests::highway::street_lanes;

/// Two streets crossing the corridor's line, north to south.
fn streets(builder: &mut MapBuilder) {
    for (name, x) in [("first", -100.0), ("second", 200.0)] {
        builder
            .add_road(
                RoadSpec::line(
                    Point3::new(x, -300.0, 0.0),
                    Point3::new(x, 300.0, 0.0),
                    street_lanes(),
                )
                .unwrap()
                .with_name(name),
            )
            .unwrap();
    }
}

fn corridor(elevated: bool) -> CorridorSpec {
    let mut spec = CorridorSpec::new(
        "Shuto",
        vec![Point3::new(-800.0, 0.0, 0.0), Point3::new(900.0, 0.0, 0.0)],
        60.0,
    )
    .unwrap();
    if elevated {
        spec.elevated = vec![(200.0, 1400.0)];
    } else {
        spec.sunken = vec![(200.0, 1400.0)];
    }
    spec
}

fn generate(
    spec: CorridorSpec,
    extra: impl FnOnce(&mut MapBuilder),
) -> (ValidatedMap, Vec<String>) {
    let (map, _, notes) = build(spec, extra);
    (map, notes)
}

/// The map, what its plan departed from, and what generating it did.
fn build(
    spec: CorridorSpec,
    extra: impl FnOnce(&mut MapBuilder),
) -> (ValidatedMap, Vec<String>, Vec<String>) {
    let mut builder = MapBuilder::default();
    streets(&mut builder);
    extra(&mut builder);
    let mut places = Places {
        corridors: vec![spec.clone()],
        town: Some(Rules::preset("town").unwrap()),
        ..Places::default()
    };
    // The town is what is on the map before the corridor: the streets.
    let town = Town::of(&builder, &places);
    let plan = spec
        .plan_in(TrafficHandedness::RightHand, &town)
        .expect("plans");
    plan.apply(&mut builder, &mut places).expect("applies");
    let mut map = builder.finish().expect("builds");
    let report = environment::generate(&mut map, &places).expect("generates");
    let map = map.validate().unwrap_or_else(|error| panic!("{error}"));
    (map, plan.notes, report.notes)
}

#[test]
fn an_elevated_motorway_passes_over_the_streets_on_a_viaduct() {
    let (map, notes) = generate(corridor(true), |_| {});
    assert!(notes.is_empty(), "{notes:#?}");
    let viaducts: Vec<_> = map
        .structures
        .iter()
        .filter(|s| s.kind.form() == "viaduct")
        .collect();
    assert_eq!(viaducts.len(), 2, "one deck per carriageway");
    for street in ["road/first", "road/second"] {
        let under = map
            .grade_separations
            .iter()
            .filter(|g| matches!(&g.under, Underneath::Road(span) if span.road.as_str() == street))
            .count();
        assert_eq!(under, 2, "both carriageways pass over {street}");
    }
    // No pier on either street's carriageway.
    for structure in &viaducts {
        for support in &structure.kind.bridge().unwrap().supports {
            for p in support.footprint.points() {
                for x in [-100.0, 200.0] {
                    assert!(
                        (p.x - x).abs() > 3.5 || p.y.abs() > 300.0,
                        "a pier on the street at {p:?}"
                    );
                }
            }
        }
    }
    // The town is built along the streets, never under the deck.
    assert!(!map.buildings.is_empty());
    let decks: Vec<Vec<[f64; 2]>> = viaducts
        .iter()
        .map(|s| s.outline.points().iter().map(|p| [p.x, p.y]).collect())
        .collect();
    for part in map.building_parts.iter() {
        for p in part.solid.footprint.points() {
            assert!(!decks.iter().any(|deck| ring_contains(deck, [p.x, p.y])));
        }
    }
}

#[test]
fn the_land_beside_a_long_straight_street_is_no_fan_of_long_triangles() {
    let (map, _) = generate(corridor(true), |_| {});
    let longest = |corners: [Point3; 3]| {
        (0..3)
            .map(|k| corners[k].horizontal_distance_to(corners[(k + 1) % 3]))
            .fold(0.0, f64::max)
    };
    // The streets are 600 m of straight road, with vertices only at their ends.
    let land = map.tin_surfaces.iter().next().expect("land");
    let nodes = land.tin.nodes();
    for triangle in land.tin.triangles() {
        let corners = triangle.map(|i| nodes[i as usize]);
        assert!(longest(corners) < 40.0, "a land triangle {corners:?}");
    }
    let meshes = roadgen_carla::to_meshes(&map, &roadgen_carla::PackageConfig::new("shuto"));
    for mesh in meshes
        .iter()
        .filter(|mesh| mesh.role == roadgen_carla::Role::Terrain)
    {
        for triangle in &mesh.triangles {
            let corners = triangle.map(|i| mesh.positions[i as usize]);
            assert!(longest(corners) < 60.0, "a terrain triangle {corners:?}");
        }
    }
}

#[test]
fn a_sunken_motorway_passes_under_the_streets_in_a_cut_and_cover_tunnel() {
    let (map, notes) = generate(corridor(false), |_| {});
    assert!(notes.is_empty(), "{notes:#?}");
    let tunnels: Vec<_> = map
        .structures
        .iter()
        .filter(|s| s.kind.form() == "cut_and_cover")
        .collect();
    assert_eq!(tunnels.len(), 2);
    // The streets pass over the tunnels: each crossing is a separation through one.
    let through = map
        .grade_separations
        .iter()
        .filter(|g| tunnels.iter().any(|t| t.id == g.via))
        .count();
    assert_eq!(through, 4);
}

/// The corridor of [`corridor`] with nothing given by hand: it finds the town.
fn urban_corridor(urban: Urban) -> CorridorSpec {
    let mut spec = corridor(true);
    spec.elevated.clear();
    spec.urban = Some(urban);
    spec
}

/// The stretches of a corridor's centre line it finds through the two streets,
/// and what it notes in finding them.
fn planned_stretches(spec: &CorridorSpec) -> (Vec<(f64, f64)>, Vec<String>) {
    let mut builder = MapBuilder::default();
    streets(&mut builder);
    let town = Town::of(&builder, &Places::default());
    let mut notes = Vec::new();
    let stretches = spec.urban_stretches(&town, &mut notes).expect("finds");
    (stretches, notes)
}

#[test]
fn an_urban_motorway_finds_the_streets_and_passes_over_them_on_a_viaduct() {
    let spec = urban_corridor(Urban::Elevated);
    // One stretch over both streets, 700 m and 1000 m along the centre line, with
    // the climb onto it and the descent off it either side.
    let (stretches, notes) = planned_stretches(&spec);
    assert!(notes.is_empty(), "{notes:#?}");
    assert_eq!(stretches.len(), 1, "{stretches:?}");
    let (from, to) = stretches[0];
    assert!(from > 0.0 && from < 700.0 - 300.0, "{from}");
    assert!(to > 1000.0 + 300.0 && to < 1700.0, "{to}");

    let (map, planned, notes) = build(spec, |_| {});
    assert!(
        planned.is_empty() && notes.is_empty(),
        "{planned:#?} {notes:#?}"
    );
    let viaducts: Vec<_> = map
        .structures
        .iter()
        .filter(|s| s.kind.form() == "viaduct")
        .collect();
    assert_eq!(viaducts.len(), 2, "one deck per carriageway");
    for street in ["road/first", "road/second"] {
        let under = map
            .grade_separations
            .iter()
            .filter(|g| matches!(&g.under, Underneath::Road(span) if span.road.as_str() == street))
            .filter(|g| viaducts.iter().any(|v| v.id == g.via))
            .count();
        assert_eq!(under, 2, "both carriageways pass over {street}");
    }
    // And it goes to CARLA: the decks are in its meshes, at their full height
    // over the streets.
    let meshes = roadgen_carla::to_meshes(&map, &roadgen_carla::PackageConfig::new("shuto"));
    let deck = meshes
        .iter()
        .flat_map(|mesh| mesh.positions.iter())
        .any(|p| (p.x + 100.0).abs() < 5.0 && p.y.abs() < 10.0 && (p.z - 9.0).abs() < 0.05);
    assert!(deck, "no deck over the first street");
}

#[test]
fn an_urban_motorway_sunk_under_the_town_passes_under_the_streets_in_a_cut_and_cover_tunnel() {
    let (map, planned, notes) = build(urban_corridor(Urban::Sunken), |_| {});
    assert!(
        planned.is_empty() && notes.is_empty(),
        "{planned:#?} {notes:#?}"
    );
    let tunnels: Vec<_> = map
        .structures
        .iter()
        .filter(|s| s.kind.form() == "cut_and_cover")
        .collect();
    assert_eq!(tunnels.len(), 2);
    let through = map
        .grade_separations
        .iter()
        .filter(|g| tunnels.iter().any(|t| t.id == g.via))
        .count();
    assert_eq!(through, 4);
    assert!(!map.structures.iter().any(|s| s.kind.form() == "viaduct"));
}

#[test]
fn an_urban_motorway_far_from_the_streets_stays_on_the_ground() {
    let mut spec = urban_corridor(Urban::Elevated);
    for p in &mut spec.centre {
        p.y = 1000.0;
    }
    let (stretches, notes) = planned_stretches(&spec);
    assert!(stretches.is_empty() && notes.is_empty(), "{stretches:?}");
    let (map, planned, notes) = build(spec, |_| {});
    assert!(
        planned.is_empty() && notes.is_empty(),
        "{planned:#?} {notes:#?}"
    );
    assert!(!map
        .structures
        .iter()
        .any(|s| matches!(s.kind.form(), "viaduct" | "cut_and_cover")));
}

#[test]
fn an_urban_motorway_planned_without_the_town_says_so() {
    let plan = urban_corridor(Urban::Elevated)
        .plan(TrafficHandedness::RightHand)
        .expect("plans");
    assert!(plan.structures.is_empty());
    assert_eq!(plan.notes.len(), 1, "{:?}", plan.notes);
}

#[test]
fn a_town_given_by_hand_the_other_way_wins_over_the_one_found() {
    let mut spec = urban_corridor(Urban::Elevated);
    spec.sunken = vec![(200.0, 1400.0)];
    let (map, planned, notes) = build(spec, |_| {});
    assert_eq!(planned.len(), 1, "{planned:#?}");
    assert!(notes.is_empty(), "{notes:#?}");
    assert!(!map.structures.iter().any(|s| s.kind.form() == "viaduct"));
    assert_eq!(
        map.structures
            .iter()
            .filter(|s| s.kind.form() == "cut_and_cover")
            .count(),
        2
    );
}

/// The corridor of [`a_lone_slip_road_and_a_half_diamond_join_the_streets`]:
/// a slip road off before the town, and a half diamond after it.
fn with_slip_roads(mut spec: CorridorSpec) -> CorridorSpec {
    spec.centre[1] = Point3::new(1300.0, 0.0, 0.0);
    // Off the right carriageway before the town, to a point the caller joins up.
    spec.ramps.push(RampSite {
        name: "West Off".into(),
        on: false,
        side: Side::Right,
        at: 150.0,
        end: Point3::new(-450.0, -70.0, 0.0),
    });
    spec.interchanges.push(InterchangeSite {
        name: "East".into(),
        kind: InterchangeKind::HalfDiamond { side: Side::Left },
        at: 1700.0,
        crossroad: vec![
            Point3::new(900.0, -250.0, 0.0),
            Point3::new(900.0, 250.0, 0.0),
        ],
        crossroad_lanes: street_lanes(),
        over: true,
        control: Control::Yield,
        terminal: 60.0,
        reach: 180.0,
    });
    spec
}

#[test]
fn an_urban_motorway_keeps_its_slip_roads_and_half_diamond_beside_the_viaduct_it_finds() {
    let (map, planned, notes) = build(with_slip_roads(urban_corridor(Urban::Elevated)), |_| {});
    assert!(
        planned.is_empty() && notes.is_empty(),
        "{planned:#?} {notes:#?}"
    );
    assert_eq!(
        map.road(&RoadId::new("west_off")).map(|r| r.form),
        Some(FormOfWay::SlipRoad)
    );
    let interchange = map.interchanges.iter().next().expect("the half diamond");
    assert_eq!(interchange.kind, "half_diamond");
    // The crossroad is the corridor's own, so not part of the town: the viaduct
    // is over the two streets only, and the crossroad goes over the motorway.
    let viaducts: Vec<_> = map
        .structures
        .iter()
        .filter(|s| s.kind.form() == "viaduct")
        .collect();
    assert_eq!(viaducts.len(), 2, "one deck per carriageway");
    for street in ["road/first", "road/second"] {
        assert!(map
            .grade_separations
            .iter()
            .any(|g| matches!(&g.under, Underneath::Road(span) if span.road.as_str() == street)));
    }
}

#[test]
fn a_lone_slip_road_and_a_half_diamond_join_the_streets() {
    let spec = with_slip_roads(corridor(true));
    let (map, notes) = generate(spec, |_| {});
    assert!(notes.is_empty(), "{notes:#?}");
    let ramp = map
        .road(&RoadId::new("west_off"))
        .expect("the lone slip road");
    assert_eq!(ramp.form, FormOfWay::SlipRoad);
    let interchange = map.interchanges.iter().next().expect("the half diamond");
    assert_eq!(interchange.kind, "half_diamond");
    let slips = interchange
        .roads
        .iter()
        .filter(|id| {
            map.road(id)
                .is_some_and(|r| r.form == FormOfWay::SlipRoad && !r.is_connector())
        })
        .count();
    assert_eq!(slips, 2);
    // Given way to, not signalled.
    assert!(!map.rules.is_empty());
    assert!(
        !map.objects
            .iter()
            .any(|o| o.kind.is_traffic_light()
                && o.lanes.iter().any(|l| l.as_str().contains("east_")))
    );
}

#[test]
fn an_elevated_motorway_on_a_shared_deck_is_one_viaduct_on_piers_down_the_median() {
    let mut spec = corridor(true);
    spec.shared_deck = true;
    let median = spec.standard.median;
    let (map, notes) = generate(spec, |_| {});
    assert!(notes.is_empty(), "{notes:#?}");
    let viaducts: Vec<_> = map
        .structures
        .iter()
        .filter(|s| s.kind.form() == "viaduct")
        .collect();
    assert_eq!(viaducts.len(), 1, "one deck for both carriageways");
    let viaduct = viaducts[0];
    let mut carried: Vec<&str> = viaduct.carries.iter().map(|s| s.road.as_str()).collect();
    carried.sort();
    assert_eq!(carried, ["road/shuto_left_0", "road/shuto_right_0"]);
    // Both carriageways still pass over each street, through the one viaduct.
    for street in ["road/first", "road/second"] {
        let under: Vec<_> = map
            .grade_separations
            .iter()
            .filter(|g| matches!(&g.under, Underneath::Road(span) if span.road.as_str() == street))
            .collect();
        assert_eq!(under.len(), 2, "both carriageways pass over {street}");
        assert!(under.iter().all(|g| g.via == viaduct.id));
    }
    // The piers stand in the median, one row of them, and none on a street; the
    // abutments reach across the whole deck.
    let supports = &viaduct.kind.bridge().unwrap().supports;
    let piers: Vec<_> = supports
        .iter()
        .filter(|s| s.kind == roadgen_core::structure::SupportKind::Pier)
        .collect();
    assert!(piers.len() >= 10, "{} piers", piers.len());
    for pier in &piers {
        for p in pier.footprint.points() {
            assert!(p.y.abs() <= median / 2.0, "a pier off the median at {p:?}");
            for x in [-100.0, 200.0] {
                assert!((p.x - x).abs() > 3.5, "a pier on the street at {p:?}");
            }
        }
    }
    for abutment in supports
        .iter()
        .filter(|s| s.kind == roadgen_core::structure::SupportKind::Abutment)
    {
        let ys: Vec<f64> = abutment.footprint.points().iter().map(|p| p.y).collect();
        let (low, high) = (
            ys.iter().copied().fold(f64::INFINITY, f64::min),
            ys.iter().copied().fold(f64::NEG_INFINITY, f64::max),
        );
        assert!(low < -9.0 && high > 9.0, "an abutment {low:.1}–{high:.1}");
    }
    // Its parapets stand along the outer edges only, none down the median.
    let parapets: Vec<_> = map
        .objects
        .iter()
        .filter(|o| {
            matches!(
                &o.kind,
                MapObjectKind::Barrier { structure: Some(of), .. } if *of == viaduct.id
            )
        })
        .collect();
    assert_eq!(parapets.len(), 2, "one along each outer edge");

    // In CARLA the deck is one slab across the median, and the parapets stand on
    // its outer edges only.
    let meshes = roadgen_carla::to_meshes(&map, &roadgen_carla::PackageConfig::new("shuto"));
    let terrain: Vec<[Point3; 3]> = meshes
        .iter()
        .filter(|mesh| mesh.role == roadgen_carla::Role::Terrain)
        .flat_map(|mesh| {
            mesh.triangles
                .iter()
                .map(|t| t.map(|i| mesh.positions[i as usize]))
        })
        .collect();
    let deck_top = 9.0;
    let covered = |x: f64, y: f64| {
        terrain.iter().any(|corners| {
            corners.iter().all(|p| (p.z - deck_top).abs() < 0.05)
                && ring_contains(&corners.map(|p| [p.x, p.y]), [x, y])
        })
    };
    // Where it is up to its full height, between the ramps up to it.
    for x in [-150.0, 50.0, 150.0] {
        for y in [-1.25, -0.5, 0.5, 1.25] {
            assert!(covered(x, y), "no deck over the median at ({x}, {y})");
        }
    }
    let parapet_top = deck_top + roadgen_core::structure::PARAPET;
    let tops: Vec<Point3> = terrain
        .iter()
        .filter(|corners| corners.iter().all(|p| (p.z - parapet_top).abs() < 0.05))
        .flatten()
        .copied()
        .collect();
    assert!(!tops.is_empty());
    assert!(
        tops.iter().all(|p| p.y.abs() > 9.0),
        "a parapet over the median"
    );
}

#[test]
fn carriageways_leaving_the_viaduct_at_different_places_share_its_deck_wherever_side_by_side() {
    let mut spec = corridor(true);
    spec.shared_deck = true;
    // A slip road down off each carriageway, at different stations.
    spec.ramps.push(RampSite {
        name: "West Off".into(),
        on: false,
        side: Side::Right,
        at: 700.0,
        end: Point3::new(60.0, -90.0, 0.0),
    });
    spec.ramps.push(RampSite {
        name: "East Off".into(),
        on: false,
        side: Side::Left,
        at: 1000.0,
        end: Point3::new(40.0, 90.0, 0.0),
    });
    let (map, notes) = generate(spec, |_| {});
    assert!(notes.is_empty(), "{notes:#?}");
    let viaducts: Vec<_> = map
        .structures
        .iter()
        .filter(|s| s.kind.form() == "viaduct")
        .collect();
    // Before the first slip road, between the two and after the second: each a
    // deck of both carriageways.
    assert_eq!(viaducts.len(), 3, "{viaducts:#?}");
    for viaduct in &viaducts {
        let roads: Vec<&str> = viaduct.carries.iter().map(|s| s.road.as_str()).collect();
        assert_eq!(roads.len(), 2, "{roads:?}");
        assert_eq!(roads.iter().filter(|r| r.contains("_left_")).count(), 1);
        assert_eq!(roads.iter().filter(|r| r.contains("_right_")).count(), 1);
    }
}

#[test]
fn the_town_leaves_out_every_corridors_roads_and_crossroads() {
    let mut builder = MapBuilder::default();
    streets(&mut builder);
    let mut places = Places::default();
    let spec = with_slip_roads(corridor(true));
    let plan = spec.plan(TrafficHandedness::RightHand).expect("plans");
    plan.apply(&mut builder, &mut places).expect("applies");
    let town = Town::of(&builder, &places);
    assert_eq!(town.streets.len(), 2, "the two streets only");
}
