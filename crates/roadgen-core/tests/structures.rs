//! Bridges, tunnels, grade separations and interchanges in the IR.

use roadgen_core::prelude::*;
use roadgen_core::structure::{
    BridgeForm, BridgeSpec, RoadSpan, StructureKind, SupportKind, TunnelForm, TunnelSpec,
    Underneath,
};
use roadgen_core::{FormOfWay, ValidationIssue, VerticalProfile};

fn two_way() -> Vec<LaneSpec> {
    vec![
        LaneSpec::new(PositiveWidth::new(3.5).unwrap(), Direction::Forward),
        LaneSpec::new(PositiveWidth::new(3.5).unwrap(), Direction::Backward),
    ]
}

/// A motorway along x at `height`, over a street along y at grade.
fn crossing(height: f64) -> (MapBuilder, RoadId, RoadId) {
    let mut builder = MapBuilder::default();
    let motorway = builder
        .add_road(
            RoadSpec::line(
                Point3::new(-300.0, 0.0, height),
                Point3::new(300.0, 0.0, height),
                two_way(),
            )
            .unwrap()
            .with_name("motorway")
            .with_type(RoadType::Motorway)
            .with_form(FormOfWay::Motorway),
        )
        .unwrap();
    let street = builder
        .add_road(
            RoadSpec::line(
                Point3::new(0.0, -200.0, 0.0),
                Point3::new(0.0, 200.0, 0.0),
                two_way(),
            )
            .unwrap()
            .with_name("street"),
        )
        .unwrap();
    (builder, motorway, street)
}

#[test]
fn a_motorway_on_a_bridge_over_a_street_is_a_grade_separation() {
    let (mut builder, motorway, street) = crossing(9.0);
    builder
        .add_bridge(
            vec![RoadSpan::new(motorway.clone(), 200.0, 400.0)],
            BridgeSpec {
                name: Some("Street Viaduct".into()),
                form: BridgeForm::Viaduct,
                ..BridgeSpec::default()
            },
        )
        .unwrap();
    let map = builder.finish().unwrap().validate().unwrap();

    let structure = map.structures.iter().next().unwrap();
    assert_eq!(structure.id.as_str(), "structure/street_viaduct");
    assert_eq!(structure.kind.form(), "viaduct");
    let road = map.road(&motorway).unwrap();
    assert_eq!(road.structure.at(300.0), Some(&structure.id));
    assert_eq!(road.structure.at(100.0), None);

    let separation = map.grade_separations.iter().next().unwrap();
    assert_eq!(separation.over.road, motorway);
    assert!(matches!(&separation.under, Underneath::Road(span) if span.road == street));
    assert_eq!(separation.via, structure.id);

    // Abutments at both ends, piers between, none on the street.
    let StructureKind::Bridge(bridge) = &structure.kind else {
        panic!("a bridge")
    };
    let abutments = bridge
        .supports
        .iter()
        .filter(|s| s.kind == SupportKind::Abutment)
        .count();
    assert_eq!(abutments, 2);
    let piers: Vec<_> = bridge
        .supports
        .iter()
        .filter(|s| s.kind == SupportKind::Pier)
        .collect();
    assert!(piers.len() >= 3, "{} piers", piers.len());
    for pier in piers {
        for p in pier.footprint.points() {
            assert!(
                p.x.abs() > 3.5,
                "a pier stands on the street at x = {}",
                p.x
            );
            assert!(
                (p.z - (9.0 - 2.5)).abs() < 1e-6,
                "a pier's top is the deck's underside"
            );
        }
    }
}

#[test]
fn a_motorway_crossing_a_street_on_the_level_is_refused() {
    let (builder, _, _) = crossing(0.0);
    let error = builder.finish().unwrap().validate().unwrap_err();
    assert!(error
        .issues
        .iter()
        .any(|issue| matches!(issue, ValidationIssue::UnseparatedCrossing { .. })));
}

#[test]
fn a_motorway_above_a_street_with_nothing_under_it_is_refused() {
    let (builder, _, _) = crossing(9.0);
    let error = builder.finish().unwrap().validate().unwrap_err();
    assert!(error.issues.iter().any(|issue| matches!(
        issue,
        ValidationIssue::UnseparatedCrossing { detail, .. } if detail.contains("nothing")
    )));
}

#[test]
fn two_streets_crossing_on_the_level_are_still_a_map() {
    let mut builder = MapBuilder::default();
    for (name, start, end) in [
        (
            "a",
            Point3::new(-100.0, 0.0, 0.0),
            Point3::new(100.0, 0.0, 0.0),
        ),
        (
            "b",
            Point3::new(0.0, -100.0, 0.0),
            Point3::new(0.0, 100.0, 0.0),
        ),
    ] {
        builder
            .add_road(
                RoadSpec::line(start, end, two_way())
                    .unwrap()
                    .with_name(name),
            )
            .unwrap();
    }
    builder.finish().unwrap().validate().unwrap();
}

#[test]
fn a_tunnel_has_a_portal_at_each_end_of_what_it_carries() {
    let mut builder = MapBuilder::default();
    let road = builder
        .add_road(
            RoadSpec::line(
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(1000.0, 0.0, 10.0),
                two_way(),
            )
            .unwrap()
            .with_name("through"),
        )
        .unwrap();
    builder
        .add_tunnel(
            vec![RoadSpan::new(road.clone(), 300.0, 700.0)],
            TunnelSpec {
                form: TunnelForm::Bored,
                ..TunnelSpec::default()
            },
        )
        .unwrap();
    let map = builder.finish().unwrap().validate().unwrap();
    let structure = map.structures.iter().next().unwrap();
    assert_eq!(structure.id.as_str(), "structure/through/tunnel/0");
    let tunnel = structure.kind.tunnel().unwrap();
    assert!((tunnel.portals[0].station - 300.0).abs() < 1e-9);
    assert!((tunnel.portals[1].station - 700.0).abs() < 1e-9);
    assert!((tunnel.portals[0].left.y - 3.5).abs() < 1e-6);
    assert!((tunnel.portals[0].right.y + 3.5).abs() < 1e-6);
    // The bore is wider than the road by its walkway and lining.
    let widest = structure
        .outline
        .points()
        .iter()
        .map(|p| p.y)
        .fold(0.0_f64, f64::max);
    assert!((widest - (3.5 + 0.75 + 0.6)).abs() < 1e-6);
}

#[test]
fn one_deck_carries_both_carriageways() {
    let mut builder = MapBuilder::default();
    let one_way = || {
        vec![
            LaneSpec::new(PositiveWidth::new(3.5).unwrap(), Direction::Forward),
            LaneSpec::new(PositiveWidth::new(3.5).unwrap(), Direction::Forward),
        ]
    };
    let left = builder
        .add_road(
            RoadSpec::line(
                Point3::new(0.0, 9.0, 10.0),
                Point3::new(400.0, 9.0, 10.0),
                one_way(),
            )
            .unwrap()
            .with_name("left"),
        )
        .unwrap();
    let right = builder
        .add_road(
            RoadSpec::line(
                Point3::new(400.0, -9.0, 10.0),
                Point3::new(0.0, -9.0, 10.0),
                one_way(),
            )
            .unwrap()
            .with_name("right"),
        )
        .unwrap();
    builder
        .add_bridge(
            vec![
                RoadSpan::new(left, 100.0, 300.0),
                RoadSpan::new(right, 100.0, 300.0),
            ],
            BridgeSpec::default(),
        )
        .unwrap();
    let map = builder.finish().unwrap().validate().unwrap();
    let structure = map.structures.iter().next().unwrap();
    let (low, high) = structure
        .outline
        .points()
        .iter()
        .fold((f64::INFINITY, f64::NEG_INFINITY), |(l, h), p| {
            (l.min(p.y), h.max(p.y))
        });
    // Each carriageway's lanes lie to its right: from one outer edge to the other,
    // the median between, and the curbs.
    assert!((high - (9.0 + 0.6)).abs() < 1e-6, "{high}");
    assert!((low - (-9.0 - 0.6)).abs() < 1e-6, "{low}");
    // A deck that wide stands on pairs of columns.
    let bridge = structure.kind.bridge().unwrap();
    let piers = bridge
        .supports
        .iter()
        .filter(|s| s.kind == SupportKind::Pier)
        .count();
    assert_eq!(piers % 2, 0);
}

#[test]
fn a_vertical_profile_rounds_the_crest_of_a_road() {
    let profile =
        VerticalProfile::new(vec![(0.0, 0.0), (300.0, 12.0), (600.0, 0.0)], 4000.0).unwrap();
    let spec = RoadSpec::line(
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(600.0, 0.0, 0.0),
        two_way(),
    )
    .unwrap()
    .with_vertical(&profile)
    .unwrap();
    let mut builder = MapBuilder::default();
    let road = builder.add_road(spec.with_name("crest")).unwrap();
    let map = builder.finish().unwrap().validate().unwrap();
    let line = &map.road(&road).unwrap().reference_line;
    let top = line
        .sample_at(300.0, map.metadata.sampling)
        .unwrap()
        .point
        .z;
    assert!((top - profile.height(300.0)).abs() < 1e-3);
    assert!(top < 12.0 - 1.0, "the crest is rounded: {top}");
    assert!((line.horizontal_length().unwrap() - 600.0).abs() < 1e-6);
}

#[test]
fn an_interchange_points_its_junctions_back_at_it() {
    let mut builder = MapBuilder::default();
    let a = builder
        .add_road(
            RoadSpec::line(
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(100.0, 0.0, 0.0),
                two_way(),
            )
            .unwrap()
            .with_name("a"),
        )
        .unwrap();
    let b = builder
        .add_road(
            RoadSpec::line(
                Point3::new(130.0, 0.0, 0.0),
                Point3::new(230.0, 0.0, 0.0),
                two_way(),
            )
            .unwrap()
            .with_name("b"),
        )
        .unwrap();
    let junction = builder.add_junction(Some("j"));
    builder.connect_via(&junction, &a, &b).unwrap();
    builder
        .add_interchange(
            "Kita IC",
            "diamond",
            vec![junction.clone()],
            vec![a.clone()],
        )
        .unwrap();
    let map = builder.finish().unwrap().validate().unwrap();
    let interchange = map.interchanges.iter().next().unwrap();
    assert_eq!(interchange.id.as_str(), "interchange/kita_ic");
    assert_eq!(
        map.junction(&junction).unwrap().interchange.as_ref(),
        Some(&interchange.id)
    );
    // The movements through its junction are its roads too.
    assert!(interchange.roads.len() > 1);
}

#[test]
fn a_deck_over_two_carriageways_running_opposite_ways_sits_on_its_supports() {
    // Two one-way carriageways side by side, climbing east, drawn opposite ways.
    let one_way = || {
        vec![LaneSpec::new(
            PositiveWidth::new(3.5).unwrap(),
            Direction::Forward,
        )]
    };
    let mut builder = MapBuilder::default();
    let east = builder
        .add_road(
            RoadSpec::line(
                Point3::new(0.0, -6.0, 0.0),
                Point3::new(300.0, -6.0, 15.0),
                one_way(),
            )
            .unwrap()
            .with_name("east"),
        )
        .unwrap();
    let west = builder
        .add_road(
            RoadSpec::line(
                Point3::new(300.0, 6.0, 15.0),
                Point3::new(0.0, 6.0, 0.0),
                one_way(),
            )
            .unwrap()
            .with_name("west"),
        )
        .unwrap();
    let spec = BridgeSpec::default();
    let depth = spec.deck_depth;
    builder
        .add_bridge(
            vec![
                RoadSpan::new(east, 50.0, 250.0),
                RoadSpan::new(west, 50.0, 250.0),
            ],
            spec,
        )
        .unwrap();
    let map = builder.finish().unwrap();
    let map = map.as_map();
    let bridge = map.structures.iter().next().unwrap();
    let abutments: Vec<_> = bridge
        .kind
        .bridge()
        .unwrap()
        .supports
        .iter()
        .filter(|s| s.kind == SupportKind::Abutment)
        .collect();
    assert_eq!(abutments.len(), 2);
    for abutment in abutments {
        // The deck is 0.05 up per metre east; its underside over the abutment is
        // there, not at the far end of the other carriageway.
        let x = abutment.footprint.centroid().x;
        let top = abutment.footprint.highest();
        let deck = 0.05 * x;
        assert!(
            (top - (deck - depth)).abs() < 0.6,
            "an abutment at x {x:.0} topped at {top:.2}, under a deck at {deck:.2}"
        );
    }
}
