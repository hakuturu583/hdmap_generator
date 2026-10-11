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

fn corridor(kind: InterchangeKind, terminal: f64) -> CorridorSpec {
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
        terminal,
        reach: 320.0,
    });
    spec
}

fn generate(kind: InterchangeKind, handedness: TrafficHandedness) -> (ValidatedMap, Vec<String>) {
    generate_at(kind, 100.0, handedness)
}

/// The map with the terminals `terminal` out from the centre line.
fn generate_at(
    kind: InterchangeKind,
    terminal: f64,
    handedness: TrafficHandedness,
) -> (ValidatedMap, Vec<String>) {
    generate_spec(corridor(kind, terminal), handedness)
}

fn generate_spec(spec: CorridorSpec, handedness: TrafficHandedness) -> (ValidatedMap, Vec<String>) {
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

#[test]
fn two_corridors_each_keep_their_own_cuttings_and_embankments() {
    let corridor = |name: &str, y: f64| {
        let mut spec = CorridorSpec::new(
            name,
            vec![Point3::new(-1500.0, y, 0.0), Point3::new(1500.0, y, 0.0)],
            100.0,
        )
        .unwrap();
        // Over a rise in the middle, so the land is cut and filled to it.
        spec.profile = vec![(0.0, 0.0), (1500.0, 12.0), (3000.0, 0.0)];
        spec
    };
    let specs = [corridor("E5", 0.0), corridor("E6", 1200.0)];
    let mut places = Places {
        corridors: specs.to_vec(),
        ..Places::default()
    };
    let mut builder = MapBuilder::default();
    for spec in &specs {
        spec.plan(TrafficHandedness::RightHand)
            .unwrap()
            .apply(&mut builder, &mut places)
            .unwrap();
    }
    let mut map = builder.finish().unwrap();
    environment::generate(&mut map, &places).expect("generates");
    let map = map.validate().unwrap_or_else(|error| panic!("{error}"));
    for name in ["e5", "e6"] {
        let site = map
            .sites
            .iter()
            .find(|site| site.id.as_str() == format!("site/{name}"))
            .unwrap_or_else(|| panic!("no site {name}"));
        let slopes = site
            .land_covers
            .iter()
            .filter_map(|id| map.land_covers.get(id))
            .filter(|cover| cover.kind == "cutting" || cover.kind == "embankment")
            .collect::<Vec<_>>();
        assert!(!slopes.is_empty(), "{name} has no slopes of its own");
        // And they are beside its own road, not the other's.
        let y = if name == "e5" { 0.0 } else { 1200.0 };
        for cover in slopes {
            let p = cover.area.centroid();
            assert!((p.y - y).abs() < 200.0, "{name}: a slope at {p:?}");
        }
    }
}

/// The tightest radius along a road's reference line, metres: the circle
/// through each three points a few metres apart.
fn tightest(road: &Road) -> f64 {
    let points = road
        .reference_line
        .to_polyline(SamplingConfig::default())
        .unwrap();
    let points = points.points();
    // Points about 3 m apart, so that sampling noise is not read as a bend.
    let mut spaced = vec![points[0]];
    for p in points {
        let last = spaced.last().unwrap();
        if (p.x - last.x).hypot(p.y - last.y) >= 3.0 {
            spaced.push(*p);
        }
    }
    spaced
        .windows(3)
        .map(|w| {
            let (a, b, c) = (w[0], w[1], w[2]);
            let ab = (b.x - a.x).hypot(b.y - a.y);
            let bc = (c.x - b.x).hypot(c.y - b.y);
            let ca = (a.x - c.x).hypot(a.y - c.y);
            let cross = ((b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x)).abs();
            if cross < 1e-9 {
                f64::INFINITY
            } else {
                ab * bc * ca / (2.0 * cross)
            }
        })
        .fold(f64::INFINITY, f64::min)
}

/// The tightest a loop's reference line may bend, metres: a loop is 30 m round
/// along the middle of its lane, and its reference line is its outer edge, the
/// lane's middle 2.25 m in from it.
const LOOP_REFERENCE: f64 = 32.0;

#[test]
fn loops_fit_with_the_terminals_at_the_default_80_m() {
    for lanes in [2, 3] {
        for kind in [
            InterchangeKind::Parclo,
            InterchangeKind::Trumpet { side: Side::Right },
        ] {
            for handedness in [TrafficHandedness::RightHand, TrafficHandedness::LeftHand] {
                let case = format!("{lanes} lanes, {kind:?}, {handedness:?}");
                // Plans with no notes, lands, and validates: the loop's outline
                // crosses neither the off slip road's nor the crossroad's.
                let mut spec = corridor(kind, 80.0);
                spec.lanes = lanes;
                let (map, notes) = generate_spec(spec, handedness);
                assert!(notes.is_empty(), "{case}: {notes:#?}");
                let sides: &[&str] = match kind {
                    InterchangeKind::Parclo => &["left", "right"],
                    _ => &["right"],
                };
                for side in sides {
                    let name = format!("minami_ic_{side}_loop");
                    let looped = road(&map, &name).unwrap_or_else(|| panic!("no {name}"));
                    // Still a loop, and no tighter than a loop may be.
                    assert!(
                        turn(looped).abs() > 2.6,
                        "{case} {name}: {:.2}",
                        turn(looped)
                    );
                    let radius = tightest(looped);
                    assert!(radius >= LOOP_REFERENCE, "{case} {name}: {radius:.1} m");
                }
                let meshes =
                    roadgen_carla::to_meshes(&map, &roadgen_carla::PackageConfig::new("e5"));
                assert!(meshes.iter().any(|m| m.role == roadgen_carla::Role::Road));
            }
        }
    }
}

#[test]
fn a_loops_radius_does_not_follow_the_terminal() {
    // Farther out, the loop is no larger: its radius is what a loop is designed
    // to, and the room is taken up by a straight towards the motorway.
    for terminal in [80.0, 100.0, 130.0] {
        let (map, notes) = generate_at(
            InterchangeKind::Parclo,
            terminal,
            TrafficHandedness::RightHand,
        );
        assert!(notes.is_empty(), "{terminal}: {notes:#?}");
        let looped = road(&map, "minami_ic_left_loop").unwrap();
        let radius = tightest(looped);
        assert!(
            (LOOP_REFERENCE..LOOP_REFERENCE + 1.0).contains(&radius),
            "{terminal}: {radius:.1} m"
        );
    }
}

#[test]
fn a_terminal_with_no_room_for_a_loop_is_refused_with_how_far_out_to_put_it() {
    // At 80 km/h the least distance falls between whole metres.
    for speed in [100.0, 80.0] {
        let spec = |terminal: f64| {
            let mut spec = corridor(InterchangeKind::Parclo, terminal);
            spec.standard = roadgen_carla::environment::DesignStandard::for_speed(speed).unwrap();
            spec
        };
        let error = spec(60.0).plan(TrafficHandedness::RightHand).unwrap_err();
        assert!(
            error.contains("too near the motorway for a loop"),
            "{error}"
        );
        // Where it says to put it is enough.
        let least: f64 = error
            .split("put it ")
            .nth(1)
            .and_then(|rest| rest.split(" m").next())
            .and_then(|n| n.parse().ok())
            .unwrap_or_else(|| panic!("{error}"));
        let plan = spec(least)
            .plan(TrafficHandedness::RightHand)
            .unwrap_or_else(|error| panic!("{speed} km/h at {least} m: {error}"));
        assert!(plan.notes.is_empty(), "{:#?}", plan.notes);
    }
}
