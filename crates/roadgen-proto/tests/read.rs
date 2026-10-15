//! What the reader refuses, and how it says so.
//!
//! A document need not have come from roadgen: anything that writes the schema can
//! write one. So the reader rebuilds every value through the IR's own constructors,
//! and when one is refused it names the field, in the schema's own spelling, so
//! the writer of the document knows what to fix.

use prost::Message;
use roadgen_core::prelude::*;
use roadgen_proto::{v1, ReadError};

fn two_way_road() -> ValidatedMap {
    let mut builder = MapBuilder::default();
    let lanes = vec![
        LaneSpec::new(PositiveWidth::new(3.5).unwrap(), Direction::Forward),
        LaneSpec::new(PositiveWidth::new(3.0).unwrap(), Direction::Backward),
    ];
    builder
        .add_road(
            RoadSpec::line(
                Point3::new(0.0, 0.0, 1.0),
                Point3::new(80.0, 10.0, 3.0),
                lanes,
            )
            .unwrap()
            .with_name("main"),
        )
        .unwrap();
    builder.finish().unwrap().validate().unwrap()
}

fn written() -> v1::Map {
    roadgen_proto::to_message(&two_way_road()).unwrap()
}

/// The field a refusal names.
fn refused_at(message: &v1::Map) -> String {
    match roadgen_proto::from_message(message) {
        Err(ReadError::Invalid { path, .. }) => path,
        Err(other) => panic!("refused for the wrong reason: {other}"),
        Ok(_) => panic!("the document should have been refused"),
    }
}

#[test]
fn a_map_comes_back_through_a_file() {
    let map = two_way_road();
    let directory = tempfile::tempdir().unwrap();
    let path = directory.path().join("map.roadgen.pb");
    roadgen_proto::write(&map, &path).unwrap();
    let back = roadgen_proto::read(&path).unwrap().validate().unwrap();
    assert_eq!(back.as_map(), map.as_map());
}

#[test]
fn the_document_says_what_wrote_it() {
    assert!(written().generator.starts_with("roadgen "));
}

#[test]
fn a_zero_width_is_refused_at_the_knot() {
    let mut message = written();
    message.lanes[1].width.as_mut().unwrap().knots[0].width = 0.0;
    assert_eq!(refused_at(&message), "lanes[1].width.knots[0].width");
}

#[test]
fn an_unset_enum_is_refused_rather_than_guessed() {
    let mut message = written();
    message.lanes[0].direction = v1::Direction::Unspecified as i32;
    assert_eq!(refused_at(&message), "lanes[0].direction");

    let mut message = written();
    message.lanes[0].lane_type = 99;
    assert_eq!(refused_at(&message), "lanes[0].lane_type");
}

#[test]
fn a_missing_curve_is_refused() {
    let mut message = written();
    message.roads[0].reference_line = None;
    assert_eq!(refused_at(&message), "roads[0].reference_line");

    let mut message = written();
    message.lanes[0].centerline = Some(v1::Curve {
        kind: Some(v1::curve::Kind::Polyline(v1::Polyline {
            points: vec![v1::Point3::default()],
        })),
        samples: Vec::new(),
    });
    assert_eq!(refused_at(&message), "lanes[0].centerline.polyline.points");
}

#[test]
fn an_identifier_that_comes_twice_is_refused() {
    let mut message = written();
    let copy = message.lanes[0].clone();
    message.lanes.push(copy);
    assert_eq!(refused_at(&message), "lanes[2].id");
}

#[test]
fn an_origin_off_the_globe_is_refused() {
    let mut message = written();
    message.metadata.as_mut().unwrap().origin = Some(v1::GeoOrigin {
        latitude: 95.0,
        longitude: 0.0,
        altitude: 0.0,
    });
    assert_eq!(refused_at(&message), "metadata.origin");
}

#[test]
fn bytes_that_are_not_a_document_are_refused() {
    assert!(matches!(
        roadgen_proto::from_bytes(&[0xff, 0xff, 0xff]),
        Err(ReadError::Decode(_))
    ));
}

#[test]
fn samples_are_only_ever_read_by_other_readers() {
    // A writer whose samples disagree with its curves is believed about the curves.
    let mut message = written();
    for sample in &mut message.lanes[0].left_boundary.as_mut().unwrap().samples {
        sample.point = Some(v1::Point3 {
            x: 1e6,
            y: 1e6,
            z: 1e6,
        });
    }
    let back = roadgen_proto::from_message(&message)
        .unwrap()
        .validate()
        .unwrap();
    assert_eq!(back.as_map(), two_way_road().as_map());

    // And a document without them is half the size and the same map.
    let lean = roadgen_proto::to_bytes_with(
        &two_way_road(),
        &roadgen_proto::WriteOptions {
            curve_samples: false,
        },
    )
    .unwrap();
    assert!(lean.len() < written().encoded_len());
    let back = roadgen_proto::from_bytes(&lean)
        .unwrap()
        .validate()
        .unwrap();
    assert_eq!(back.as_map(), two_way_road().as_map());
}

#[test]
fn the_schema_ships_compiled_and_as_source() {
    let set = prost_types::FileDescriptorSet::decode(roadgen_proto::FILE_DESCRIPTOR_SET).unwrap();
    let names: Vec<&str> = set.file.iter().map(|file| file.name()).collect();
    for (path, source) in roadgen_proto::SCHEMA {
        assert!(names.contains(path), "{path} is not in the descriptor set");
        assert!(source.contains("package roadgen_ir.v1;"));
    }
    let map = set
        .file
        .iter()
        .find(|file| file.name() == "roadgen_ir/v1/map.proto")
        .unwrap();
    assert_eq!(
        format!("{}.{}", map.package(), "Map"),
        roadgen_proto::MAP_MESSAGE
    );
    assert!(map
        .message_type
        .iter()
        .any(|message| message.name() == "Map"));
}
