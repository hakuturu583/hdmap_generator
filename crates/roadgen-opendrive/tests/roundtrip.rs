//! The exported document has to be readable by an OpenDRIVE parser, and has to say
//! what the IR said. The parser used here is the one in the `opendrive` crate — an
//! implementation of the 1.7 schema that knows nothing about this project.

use opendrive::core::OpenDrive;
use opendrive::lane::lane_choice::LaneChoice;
use roadgen_core::prelude::*;
use roadgen_core::validation::ValidatedMap;

fn lanes() -> Vec<LaneSpec> {
    vec![
        LaneSpec::new(PositiveWidth::new(3.5).unwrap(), Direction::Forward),
        LaneSpec::new(PositiveWidth::new(3.5).unwrap(), Direction::Backward),
    ]
}

/// The map from the design notes: two roads, a bend, and a climb.
fn two_roads() -> ValidatedMap {
    let mut builder = MapBuilder::default();
    let a = builder
        .add_road(
            RoadSpec::line(
                Point3::new(0.0, 0.0, 10.0),
                Point3::new(100.0, 0.0, 12.0),
                lanes(),
            )
            .unwrap()
            .with_name("a"),
        )
        .unwrap();
    let b = builder
        .add_road(
            RoadSpec::line(
                Point3::new(100.0, 0.0, 12.0),
                Point3::new(200.0, 50.0, 15.0),
                lanes(),
            )
            .unwrap()
            .with_name("b"),
        )
        .unwrap();
    builder.connect(&a, &b).unwrap();
    builder.finish().unwrap().validate().unwrap()
}

#[test]
fn the_exported_document_parses_again() {
    let xml = roadgen_opendrive::to_xml(&two_roads()).unwrap();
    let parsed = OpenDrive::from_xml_str(&xml).expect("an OpenDRIVE parser should accept this");
    assert_eq!(parsed.road.len(), 2);
    assert_eq!(parsed.header.rev_major, 1);
    assert_eq!(parsed.header.rev_minor, 7);
}

#[test]
fn the_reference_line_keeps_its_length_and_its_climb() {
    let parsed =
        OpenDrive::from_xml_str(&roadgen_opendrive::to_xml(&two_roads()).unwrap()).unwrap();
    let first = &parsed.road[0];
    assert!((first.length.value - 100.0).abs() < 1e-6);

    // The 2 m climb over 100 m of plan view becomes a slope of 0.02 in the
    // elevation profile, with the road starting at z = 10.
    let elevation = &first.elevation_profile.as_ref().unwrap().elevation[0];
    assert!((elevation.a - 10.0).abs() < 1e-9);
    assert!((elevation.b - 0.02).abs() < 1e-9);
}

#[test]
fn the_cross_section_survives_with_its_widths_and_ids() {
    let parsed =
        OpenDrive::from_xml_str(&roadgen_opendrive::to_xml(&two_roads()).unwrap()).unwrap();
    let section = &parsed.road[0].lanes.lane_section.first();

    // Right-hand traffic: the forward lane is lane -1, the opposing one is +1.
    let right = section.right.as_ref().expect("a forward lane on the right");
    assert_eq!(right.lane.len(), 1);
    assert_eq!(right.lane.first().id, -1);
    let left = section.left.as_ref().expect("a backward lane on the left");
    assert_eq!(left.lane.first().id, 1);

    let LaneChoice::Width(width) = &right.lane.first().base.choice[0] else {
        panic!("a lane is described by a width");
    };
    assert!((width.a - 3.5).abs() < 1e-9);
}

#[test]
fn the_two_roads_are_linked_to_each_other() {
    let parsed =
        OpenDrive::from_xml_str(&roadgen_opendrive::to_xml(&two_roads()).unwrap()).unwrap();
    let successor = parsed.road[0]
        .link
        .as_ref()
        .and_then(|link| link.successor.as_ref())
        .expect("the first road continues into the second");
    assert_eq!(successor.element_id, parsed.road[1].id);

    let predecessor = parsed.road[1]
        .link
        .as_ref()
        .and_then(|link| link.predecessor.as_ref())
        .expect("the second road follows the first");
    assert_eq!(predecessor.element_id, parsed.road[0].id);

    // Lane -1 of the first road continues into lane -1 of the second.
    let lane = parsed.road[0]
        .lanes
        .lane_section
        .first()
        .right
        .as_ref()
        .unwrap()
        .lane
        .first();
    let link = lane.base.link.as_ref().expect("a lane-level link");
    assert_eq!(link.successor.len(), 1);
    assert_eq!(link.successor[0].id, -1);
}

#[test]
fn a_junction_becomes_a_junction_element() {
    let mut builder = MapBuilder::default();
    let approach = builder
        .add_road(
            RoadSpec::line(Point3::ORIGIN, Point3::new(100.0, 0.0, 0.0), lanes())
                .unwrap()
                .with_name("approach"),
        )
        .unwrap();
    let exit = builder
        .add_road(
            RoadSpec::line(
                Point3::new(120.0, 0.0, 0.0),
                Point3::new(220.0, 0.0, 0.0),
                lanes(),
            )
            .unwrap()
            .with_name("exit"),
        )
        .unwrap();
    let junction = builder.add_junction(Some("j0"));
    builder.connect_via(&junction, &approach, &exit).unwrap();
    let map = builder.finish().unwrap().validate().unwrap();
    assert!(roadgen_opendrive::check(&map).is_empty());

    let parsed = OpenDrive::from_xml_str(&roadgen_opendrive::to_xml(&map).unwrap()).unwrap();
    assert_eq!(parsed.junction.len(), 1);
    let element = &parsed.junction[0];
    // One connector each way through the junction.
    assert_eq!(element.connection.len(), 2);
    for connection in element.connection.iter() {
        assert_eq!(connection.lane_link.len(), 1);
        assert_eq!(connection.lane_link[0].to, -1);
    }

    // The connectors are roads that belong to the junction; the approach roads are
    // not.
    let connectors = parsed
        .road
        .iter()
        .filter(|road| road.junction != "-1")
        .count();
    assert_eq!(connectors, 2);
}

#[test]
fn the_same_map_exports_byte_for_byte_the_same_twice() {
    assert_eq!(
        roadgen_opendrive::to_xml(&two_roads()).unwrap(),
        roadgen_opendrive::to_xml(&two_roads()).unwrap()
    );
}
