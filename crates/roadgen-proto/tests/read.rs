//! What the reader refuses, and how it says so.
//!
//! A document need not have come from roadgen: anything that writes the schema can
//! write one. So the reader rebuilds every value through the IR's own constructors,
//! and when one is refused it names the field, in the schema's own spelling, so
//! the writer of the document knows what to fix.

use prost::Message;
use roadgen_core::prelude::*;
use roadgen_core::SamplingConfig;
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
    let map = two_way_road();
    let mut message = roadgen_proto::to_message(&map).unwrap();
    let full = message.encoded_len();

    // A writer whose samples disagree with its curves is believed about the curves.
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
    assert_eq!(back.as_map(), map.as_map());

    // And a document without them is smaller and the same map.
    let lean = roadgen_proto::to_bytes_with(
        &map,
        &roadgen_proto::WriteOptions {
            curve_samples: false,
        },
    )
    .unwrap();
    assert!(lean.len() < full);
    let back = roadgen_proto::from_bytes(&lean)
        .unwrap()
        .validate()
        .unwrap();
    assert_eq!(back.as_map(), map.as_map());
}

#[test]
fn what_an_unset_field_means_is_what_the_schema_says() {
    // A writer that never touched a lane's height, a road's offset, bank or links
    // means a flat lane on a level road that ends where it ends.
    let map = two_way_road();
    let mut message = roadgen_proto::to_message(&map).unwrap();
    message.roads[0].lane_offset = None;
    message.roads[0].superelevation = None;
    message.roads[0].link = None;
    for lane in &mut message.lanes {
        lane.height = None;
    }
    let back = roadgen_proto::from_message(&message)
        .unwrap()
        .validate()
        .unwrap();
    assert_eq!(back.as_map(), map.as_map());
}

/// The road's reference line, replaced.
fn with_reference_line(kind: v1::curve::Kind) -> v1::Map {
    let mut message = written();
    message.roads[0].reference_line = Some(v1::Curve {
        kind: Some(kind),
        samples: Vec::new(),
    });
    message
}

fn at(x: f64, y: f64) -> v1::Point3 {
    v1::Point3 { x, y, z: 0.0 }
}

#[test]
fn a_composite_with_a_gap_in_it_is_refused() {
    let line = |from: f64, to: f64| v1::Curve {
        kind: Some(v1::curve::Kind::Line(v1::Line {
            start: Some(at(from, 0.0)),
            end: Some(at(to, 0.0)),
        })),
        samples: Vec::new(),
    };
    let message = with_reference_line(v1::curve::Kind::Composite(v1::Composite {
        pieces: vec![line(0.0, 50.0), line(80.0, 100.0)],
    }));
    assert_eq!(
        refused_at(&message),
        "roads[0].reference_line.composite.pieces[1]"
    );
}

#[test]
fn a_bezier_of_more_segments_than_can_be_afforded_is_refused() {
    // Every sampling allocates a vertex per segment, so four billion of them
    // would take the reading process down rather than fail.
    let message = with_reference_line(v1::curve::Kind::Bezier(v1::Bezier {
        control: vec![at(0.0, 0.0), at(30.0, 0.0), at(70.0, 0.0), at(100.0, 0.0)],
        segments: u32::MAX,
    }));
    assert_eq!(refused_at(&message), "roads[0].reference_line.bezier");
}

#[test]
fn a_point_that_is_not_finite_is_refused() {
    let message = with_reference_line(v1::curve::Kind::Line(v1::Line {
        start: Some(at(f64::NAN, 0.0)),
        end: Some(at(100.0, 0.0)),
    }));
    assert_eq!(refused_at(&message), "roads[0].reference_line.line.start");
}

#[test]
fn a_road_without_a_cross_section_is_refused() {
    let mut message = written();
    message.roads[0].sections.clear();
    assert_eq!(refused_at(&message), "roads[0].sections");
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

#[test]
fn a_map_without_metadata_is_refused_by_name() {
    let mut message = written();
    message.metadata = None;
    assert_eq!(refused_at(&message), "metadata");
}

#[test]
fn a_structure_span_that_runs_backwards_is_refused() {
    let mut message = written();
    message.roads[0].structure.push(v1::StructureSpan {
        from: 50.0,
        to: 10.0,
        structure: "structure/bridge".into(),
    });
    assert_eq!(refused_at(&message), "roads[0].structure[0]");
}

#[test]
fn however_long_a_curve_its_samples_are_bounded() {
    // An arc a hundred million kilometres round, read from a document, costs what
    // any curve can cost, rather than all the memory there is.
    let message = with_reference_line(v1::curve::Kind::Arc(v1::Arc {
        start: Some(at(0.0, 0.0)),
        heading: 0.0,
        curvature: 0.0,
        horizontal_length: 1e14,
        end_z: 0.0,
    }));
    let map = roadgen_proto::from_message(&message).unwrap();
    let road = map.as_map().roads.iter().next().unwrap();
    let samples = road
        .reference_line
        .samples(map.as_map().metadata.sampling)
        .unwrap();
    assert_eq!(samples.len(), SamplingConfig::MAX_SEGMENTS + 1);
}
