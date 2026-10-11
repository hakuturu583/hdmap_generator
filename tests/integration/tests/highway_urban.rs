//! A motorway through a town: lifted onto a viaduct over the streets, or sunk into
//! a cut-and-cover tunnel under them, with slip roads down to street level.

use roadgen_buildings::Rules;
use roadgen_carla::environment::{
    self, Control, CorridorSpec, InterchangeKind, InterchangeSite, Places, RampSite, Side,
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
    let mut builder = MapBuilder::default();
    streets(&mut builder);
    extra(&mut builder);
    let mut places = Places {
        corridors: vec![spec.clone()],
        town: Some(Rules::preset("town").unwrap()),
        ..Places::default()
    };
    let plan = spec.plan(TrafficHandedness::RightHand).expect("plans");
    plan.apply(&mut builder, &mut places.corridor_roads)
        .expect("applies");
    let mut map = builder.finish().expect("builds");
    let report = environment::generate(&mut map, &places).expect("generates");
    let map = map.validate().unwrap_or_else(|error| panic!("{error}"));
    (map, report.notes)
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

#[test]
fn a_lone_slip_road_and_a_half_diamond_join_the_streets() {
    let mut spec = corridor(true);
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
