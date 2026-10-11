//! The interchanges a corridor plans beyond the diamond: a partial cloverleaf,
//! whose turns on across the crossroad are loops, and a trumpet, whose crossroad
//! stops at the motorway.

use roadgen_carla::environment::{
    self, CorridorSpec, InterchangeKind, InterchangeSite, Places, Side,
};
use roadgen_core::map::Road;
use roadgen_core::prelude::*;
use roadgen_core::FormOfWay;
use roadgen_integration_tests::highway::street_lanes;

fn corridor(kind: InterchangeKind) -> CorridorSpec {
    let mut spec = CorridorSpec::new(
        "E5",
        vec![
            Point3::new(-1500.0, 0.0, 0.0),
            Point3::new(1500.0, 0.0, 0.0),
        ],
        100.0,
    )
    .unwrap();
    spec.interchanges.push(InterchangeSite {
        name: "Minami IC".into(),
        kind,
        at: 1500.0,
        crossroad: vec![Point3::new(0.0, -500.0, 0.0), Point3::new(0.0, 500.0, 0.0)],
        crossroad_lanes: street_lanes(),
        over: true,
        control: environment::Control::Signal,
        terminal: 100.0,
        reach: 320.0,
    });
    spec
}

fn generate(kind: InterchangeKind, handedness: TrafficHandedness) -> (ValidatedMap, Vec<String>) {
    let spec = corridor(kind);
    let mut places = Places {
        corridors: vec![spec.clone()],
        ..Places::default()
    };
    let plan = spec.plan(handedness).expect("plans");
    assert!(plan.notes.is_empty(), "{:#?}", plan.notes);
    let mut builder = MapBuilder::new(MapMetadata {
        handedness,
        ..MapMetadata::default()
    });
    plan.apply(&mut builder, &mut places).expect("applies");
    let mut map = builder.finish().expect("builds");
    let report = environment::generate(&mut map, &places).expect("generates");
    let map = map.validate().unwrap_or_else(|error| panic!("{error}"));
    (map, report.notes)
}

/// How far a road turns, end to end, radians: positive anticlockwise.
fn turn(road: &Road) -> f64 {
    let points = road
        .reference_line
        .to_polyline(SamplingConfig::default())
        .unwrap();
    let points = points.points();
    let mut total = 0.0;
    let mut last: Option<f64> = None;
    for pair in points.windows(2) {
        let heading = (pair[1].y - pair[0].y).atan2(pair[1].x - pair[0].x);
        if let Some(last) = last {
            let mut d = heading - last;
            while d > std::f64::consts::PI {
                d -= std::f64::consts::TAU;
            }
            while d < -std::f64::consts::PI {
                d += std::f64::consts::TAU;
            }
            total += d;
        }
        last = Some(heading);
    }
    total
}

fn road<'m>(map: &'m ValidatedMap, name: &str) -> Option<&'m Road> {
    map.roads
        .iter()
        .find(|road| road.id.as_str() == format!("road/{name}"))
}

#[test]
fn a_parclo_turns_on_across_the_crossroad_by_a_loop_each_side() {
    for handedness in [TrafficHandedness::RightHand, TrafficHandedness::LeftHand] {
        let (map, notes) = generate(InterchangeKind::Parclo, handedness);
        assert!(notes.is_empty(), "{notes:#?}");
        let [interchange] = map.interchanges.iter().collect::<Vec<_>>()[..] else {
            panic!("one interchange");
        };
        assert_eq!(interchange.kind, "parclo");
        for side in ["left", "right"] {
            for ramp in ["off", "on", "loop"] {
                let name = format!("minami_ic_{side}_{ramp}");
                let road = road(&map, &name).unwrap_or_else(|| panic!("no {name}"));
                assert_eq!(road.form, FormOfWay::SlipRoad, "{name}");
            }
            // A loop turns back on itself, and then some, to join the traffic.
            let looped = turn(road(&map, &format!("minami_ic_{side}_loop")).unwrap());
            assert!(looped.abs() > 2.6, "{side}: {looped:.2}");
        }
        // The crossroad still crosses on its overpass.
        assert!(map.structures.iter().any(|s| s.kind.form() == "overpass"));
    }
}

#[test]
fn a_trumpets_crossroad_stops_at_the_far_terminal() {
    for handedness in [TrafficHandedness::RightHand, TrafficHandedness::LeftHand] {
        let (map, notes) = generate(InterchangeKind::Trumpet { side: Side::Right }, handedness);
        assert!(notes.is_empty(), "{notes:#?}");
        let [interchange] = map.interchanges.iter().collect::<Vec<_>>()[..] else {
            panic!("one interchange");
        };
        assert_eq!(interchange.kind, "trumpet");
        // The near side a diamond's pair, the far side an off and a loop.
        for name in ["left_off", "left_on", "right_off", "right_loop"] {
            assert!(
                road(&map, &format!("minami_ic_{name}")).is_some(),
                "no {name}"
            );
        }
        assert!(road(&map, "minami_ic_right_on").is_none());
        assert!(road(&map, "minami_ic_left_loop").is_none());
        // The crossroad is two pieces, not three: nothing beyond the far
        // terminal.
        let pieces = map
            .roads
            .iter()
            .filter(|r| r.id.as_str().starts_with("road/minami_ic_cross_"))
            .count();
        assert_eq!(pieces, 2);
        // Nothing at a trumpet is signalled.
        assert!(!map.objects.iter().any(|o| o.kind.is_traffic_light()));
    }
}

#[test]
fn carla_draws_both() {
    for kind in [
        InterchangeKind::Parclo,
        InterchangeKind::Trumpet { side: Side::Left },
    ] {
        let (map, _) = generate(kind, TrafficHandedness::RightHand);
        let meshes = roadgen_carla::to_meshes(&map, &roadgen_carla::PackageConfig::new("e5"));
        assert!(meshes.iter().any(|m| m.role == roadgen_carla::Role::Road));
    }
}
