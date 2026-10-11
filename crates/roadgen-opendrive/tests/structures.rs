//! Bridges and tunnels go out as `<bridge>` and `<tunnel>` on the roads they carry,
//! and come back as structures over the same stretches — laid out again, with the
//! grade separations they make derived again.

use opendrive::core::OpenDrive;
use opendrive::object::bridge_type::BridgeType;
use opendrive::object::tunnel_type::TunnelType;
use roadgen_core::prelude::*;
use roadgen_core::structure::{
    BridgeForm, BridgeSpec, RoadSpan, StructureKind, StructureMaterial, TunnelForm, TunnelSpec,
    Underneath,
};
use roadgen_core::FormOfWay;
use roadgen_opendrive::{check, from_xml, to_xml};

fn two_way() -> Vec<LaneSpec> {
    vec![
        LaneSpec::new(PositiveWidth::new(3.5).unwrap(), Direction::Forward),
        LaneSpec::new(PositiveWidth::new(3.5).unwrap(), Direction::Backward),
    ]
}

/// A motorway on a steel viaduct over a street, and a road through an underpass
/// and a bored tunnel further off.
fn structures() -> ValidatedMap {
    let mut builder = MapBuilder::default();
    let motorway = builder
        .add_road(
            RoadSpec::line(
                Point3::new(-300.0, 0.0, 9.0),
                Point3::new(300.0, 0.0, 9.0),
                two_way(),
            )
            .unwrap()
            .with_name("motorway")
            .with_type(RoadType::Motorway)
            .with_form(FormOfWay::Motorway),
        )
        .unwrap();
    builder
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
    let through = builder
        .add_road(
            RoadSpec::line(
                Point3::new(0.0, 500.0, 0.0),
                Point3::new(1000.0, 500.0, 10.0),
                two_way(),
            )
            .unwrap()
            .with_name("through"),
        )
        .unwrap();
    builder
        .add_bridge(
            vec![RoadSpan::new(motorway, 200.0, 400.0)],
            BridgeSpec {
                name: Some("Street Viaduct".into()),
                form: BridgeForm::Viaduct,
                material: StructureMaterial::Steel,
                ..BridgeSpec::default()
            },
        )
        .unwrap();
    builder
        .add_tunnel(
            vec![RoadSpan::new(through.clone(), 100.0, 200.0)],
            TunnelSpec {
                form: TunnelForm::Underpass,
                lighting: false,
                ..TunnelSpec::default()
            },
        )
        .unwrap();
    builder
        .add_tunnel(
            vec![RoadSpan::new(through, 300.0, 700.0)],
            TunnelSpec {
                name: Some("Hill Tunnel".into()),
                ..TunnelSpec::default()
            },
        )
        .unwrap();
    builder.finish().unwrap().validate().unwrap()
}

#[test]
fn each_structure_is_an_element_on_the_road_it_carries() {
    let xml = to_xml(&structures()).unwrap();
    let parsed = OpenDrive::from_xml_str(&xml).unwrap();
    let road = |name: &str| {
        parsed
            .road
            .iter()
            .find(|road| road.name.as_deref() == Some(name))
            .unwrap()
    };

    let bridges = &road("motorway").objects.as_ref().unwrap().bridge;
    assert_eq!(bridges.len(), 1);
    assert!((bridges[0].s.value - 200.0).abs() < 1e-6);
    assert!((bridges[0].length.value - 200.0).abs() < 1e-6);
    assert_eq!(bridges[0].r#type, BridgeType::Steel);
    assert_eq!(bridges[0].name.as_deref(), Some("Street Viaduct"));

    let tunnels = &road("through").objects.as_ref().unwrap().tunnel;
    assert_eq!(tunnels.len(), 2);
    assert_eq!(tunnels[0].r#type, TunnelType::Underpass);
    assert_eq!(tunnels[0].lighting, Some(0.0));
    assert_eq!(
        tunnels[0].name, None,
        "an unnamed structure is written unnamed"
    );
    assert_eq!(tunnels[1].r#type, TunnelType::Standard);
    assert_eq!(tunnels[1].lighting, Some(1.0));
    assert_eq!(tunnels[1].daylight, Some(0.0));
    assert_ne!(tunnels[0].id, tunnels[1].id);

    assert!(road("street").objects.is_none());
}

#[test]
fn check_says_what_a_structure_loses() {
    let notes = check(&structures());
    assert!(notes.iter().any(|note| note.contains("piers")), "{notes:?}");
    assert!(
        notes.iter().any(|note| note.contains("form of way")),
        "{notes:?}"
    );
}

#[test]
fn structures_come_back_over_the_same_stretches() {
    let written = structures();
    let imported = from_xml(&to_xml(&written).unwrap()).unwrap();
    assert!(
        imported
            .approximations
            .iter()
            .any(|note| note.contains("laid out again")),
        "{:?}",
        imported.approximations
    );
    let map = imported.map.validate().unwrap();
    assert_eq!(map.structures.len(), written.structures.len());

    for original in written.structures.iter() {
        let span = &original.carries[0];
        let road_name = written.road(&span.road).unwrap().name.clone();
        let read_road = map
            .roads
            .iter()
            .find(|road| road.name == road_name)
            .unwrap();
        let id = read_road
            .structure
            .at((span.from + span.to) / 2.0)
            .unwrap_or_else(|| panic!("nothing over the middle of {}", original.id));
        let read = map.structures.get(id).unwrap();
        assert_eq!(read.name, original.name);
        assert_eq!(read.kind.as_str(), original.kind.as_str());
        assert_eq!(read.kind.form(), original.kind.form());
        assert_eq!(read.carries.len(), 1);
        assert!((read.carries[0].from - span.from).abs() < 1e-6);
        assert!((read.carries[0].to - span.to).abs() < 1e-6);
        if let (StructureKind::Bridge(a), StructureKind::Bridge(b)) = (&read.kind, &original.kind) {
            assert_eq!(a.material, b.material);
            assert!(!a.supports.is_empty(), "the supports are laid out again");
        }
        if let (StructureKind::Tunnel(a), StructureKind::Tunnel(b)) = (&read.kind, &original.kind) {
            assert_eq!(a.lighting, b.lighting);
        }
    }

    // The viaduct over the street is a grade separation again.
    assert!(map.grade_separations.iter().any(|separation| {
        matches!(&separation.under, Underneath::Road(span)
            if map.road(&span.road).unwrap().name.as_deref() == Some("street"))
    }));
}

#[test]
fn one_named_deck_over_two_carriageways_comes_back_as_one_bridge() {
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
            BridgeSpec {
                name: Some("Twin".into()),
                ..BridgeSpec::default()
            },
        )
        .unwrap();
    let written = builder.finish().unwrap().validate().unwrap();
    let map = from_xml(&to_xml(&written).unwrap())
        .unwrap()
        .map
        .validate()
        .unwrap();
    assert_eq!(map.structures.len(), 1);
    assert_eq!(map.structures.iter().next().unwrap().carries.len(), 2);
}
