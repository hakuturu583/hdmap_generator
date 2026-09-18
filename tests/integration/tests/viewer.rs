//! Drawing what was exported.
//!
//! These tests go the whole way round: a scenario is built, written out as a file, and
//! then read back by `roadgen-viewer` and drawn — which means every assertion here is
//! about the bytes a consumer would receive rather than about the IR they came from.
//! A map and its picture agreeing is only worth something because the picture was made
//! without the map.
//!
//! What they check is the part a picture can be wrong about and a parser cannot: that
//! the shapes land where the map put them, and that the four formats — which sample,
//! project and name things differently — end up drawing the same road in the same
//! place.

use std::path::Path;

use roadgen_core::prelude::*;
use roadgen_integration_tests::scenarios;
use roadgen_viewer::drawing::{Bounds, Kind, Shape};
use roadgen_viewer::Drawing;

/// Every scenario, in every format the viewer reads. A picture with nothing in it is
/// the failure this is looking for: it is what a reader that silently found no rows
/// produces, and it is indistinguishable from success anywhere the output is not
/// looked at.
#[test]
fn every_scenario_draws_in_every_format() {
    for (name, map) in every_scenario() {
        let directory = tempfile::tempdir().expect("a temporary directory");
        let drawings = draw_all(&map, directory.path());
        for (format, drawing) in drawings {
            assert!(
                !drawing.shapes.is_empty(),
                "{name} drew nothing as {format}"
            );
            assert!(
                drawing.bounds().is_some(),
                "{name} as {format} has no extent"
            );
            let svg = drawing.to_svg();
            assert!(svg.starts_with("<svg"), "{name} as {format}");
            assert!(!svg.contains("NaN"), "{name} as {format} drew a NaN");
        }
    }
}

/// The four formats put the same map in the same place.
///
/// They have every reason not to: OpenDRIVE states a reference line and widths, SUMO
/// states lane shapes, ClipGT states rails and GPUDrive states centrelines, and all
/// four are sampled differently. What they share is the coordinate frame — east-north
/// in metres, the same origin — so the extents should agree to within the half
/// cross-section that separates a centreline from a road edge.
#[test]
fn the_formats_agree_about_where_the_map_is() {
    for (name, map) in every_scenario() {
        let directory = tempfile::tempdir().expect("a temporary directory");
        let drawings = draw_all(&map, directory.path());
        let mut extents: Vec<(&str, Bounds)> = Vec::new();
        for (format, drawing) in &drawings {
            extents.push((format, drawing.bounds().expect("an extent")));
        }

        let (first_name, first) = extents[0];
        for (format, bounds) in &extents[1..] {
            // Half a wide cross-section. GPUDrive draws no surface, so its extent is
            // the outermost *line* where the others reach the outermost *edge*, and
            // that difference is real rather than an error.
            let slack = 12.0;
            for (label, a, b) in [
                ("min x", first.min.x, bounds.min.x),
                ("min y", first.min.y, bounds.min.y),
                ("max x", first.max.x, bounds.max.x),
                ("max y", first.max.y, bounds.max.y),
            ] {
                assert!(
                    (a - b).abs() < slack,
                    "{name}: {first_name} and {format} disagree about {label}: \
                     {a} against {b}"
                );
            }
        }
    }
}

/// A crossroads has a junction in it, and three of the four formats say so in a way a
/// picture can show. The fourth — SUMO — says it with a node, because a plain-XML
/// network has no junction shape in it at all until netconvert builds one.
#[test]
fn a_junction_is_drawn_as_the_format_states_it() {
    let map = scenarios::controlled_crossroads();
    let directory = tempfile::tempdir().expect("a temporary directory");
    let drawings = draw_all(&map, directory.path());
    let by_format = |wanted: &str| -> &Drawing {
        &drawings
            .iter()
            .find(|(format, _)| *format == wanted)
            .expect("the format was drawn")
            .1
    };

    // ClipGT writes the junction's outline as a layer of its own.
    assert!(
        has(by_format("ClipGT"), Kind::Junction),
        "a clip states the intersection area"
    );
    // SUMO states it as a node the connections meet at, and the movements through it.
    assert!(has(by_format("SUMO"), Kind::Node));
    assert!(has(by_format("SUMO"), Kind::Movement));
    // OpenDRIVE and GPUDrive both state it as the connecting roads themselves, so
    // what shows is lanes through the middle rather than an area.
    assert!(has(by_format("OpenDRIVE"), Kind::Center));
    assert!(has(by_format("GPUDrive"), Kind::Center));
}

/// Traffic control reaches the pictures that can hold it.
#[test]
fn a_controlled_crossroads_draws_its_stop_lines_and_lights() {
    let map = scenarios::controlled_crossroads();
    let directory = tempfile::tempdir().expect("a temporary directory");
    let drawings = draw_all(&map, directory.path());
    let clip = &drawings
        .iter()
        .find(|(format, _)| *format == "ClipGT")
        .expect("a clip")
        .1;
    assert!(has(clip, Kind::StopLine), "a clip holds the wait lines");
    assert!(
        has(clip, Kind::TrafficLight),
        "a clip holds the traffic lights"
    );
}

/// A graded road is drawn flat, and the picture says so rather than pretending.
///
/// The heights are not lost by accident: a plan view has nowhere to put them. What
/// would be a bug is a viewer that quietly projected a climbing road and left a reader
/// thinking the map was level.
#[test]
fn a_plan_view_says_it_is_one() {
    let map = scenarios::graded_road();
    let directory = tempfile::tempdir().expect("a temporary directory");
    for (format, drawing) in draw_all(&map, directory.path()) {
        let notes = drawing.notes.join(" ");
        assert!(!notes.is_empty(), "{format} explained nothing");
        assert!(
            drawing.to_svg().contains("<desc>"),
            "{format} kept its notes out of the document"
        );
    }
}

/// A lane the map made 3.5 m wide is 3.5 m wide in the picture.
///
/// SUMO is where this is checkable directly: the width is an attribute rather than
/// something the viewer works out, so a band of any other width means the reader
/// misread the file.
#[test]
fn a_sumo_lane_is_drawn_the_width_the_file_gives_it() {
    let map = scenarios::bidirectional_road();
    let directory = tempfile::tempdir().expect("a temporary directory");
    roadgen_sumo::write(&map, directory.path()).expect("a SUMO network");
    let drawing = roadgen_viewer::sumo_directory(directory.path()).expect("a drawing");

    let widths: Vec<f64> = drawing
        .shapes
        .iter()
        .filter_map(|shape| match shape {
            Shape::Band { width, .. } => Some(*width),
            _ => None,
        })
        .collect();
    assert!(!widths.is_empty(), "no lane was drawn with a width");
    for width in widths {
        assert!(
            (width - 3.5).abs() < 1e-6,
            "a 3.5 m lane was drawn {width} m wide"
        );
    }
}

/// An OpenDRIVE lane's shape is evaluated from polynomials, so it is the one place a
/// picture can be wrong while the file is right. A straight two-lane road puts its
/// lane centres exactly half a lane either side of the reference line.
#[test]
fn an_opendrive_lane_centre_lands_half_a_lane_off_the_reference_line() {
    let map = scenarios::bidirectional_road();
    let directory = tempfile::tempdir().expect("a temporary directory");
    let path = directory.path().join("map.xodr");
    roadgen_opendrive::write(&map, &path).expect("an OpenDRIVE file");
    let drawing = roadgen_viewer::opendrive_file(&path).expect("a drawing");

    let centres: Vec<&Shape> = drawing
        .shapes
        .iter()
        .filter(|shape| shape.kind() == Kind::Center)
        .collect();
    assert_eq!(centres.len(), 2, "two lanes, two centre lines");

    // The scenario runs along +x from the origin, so the offsets are the y values.
    let mut offsets: Vec<f64> = centres.iter().map(|shape| shape.extent()[0].y).collect();
    offsets.sort_by(|a, b| a.partial_cmp(b).expect("finite"));
    assert!((offsets[0] + 1.75).abs() < 1e-6, "{offsets:?}");
    assert!((offsets[1] - 1.75).abs() < 1e-6, "{offsets:?}");
}

/// A bent road bends in the picture. A viewer that dropped the curvature — drew every
/// geometry record as the straight line from its start — would still produce something
/// that looks like a road, and would put the far end in the wrong place.
#[test]
fn a_spiral_transition_is_drawn_as_a_curve_rather_than_a_chord() {
    let map = scenarios::spiral_transition_road();
    let directory = tempfile::tempdir().expect("a temporary directory");
    let path = directory.path().join("map.xodr");
    roadgen_opendrive::write(&map, &path).expect("an OpenDRIVE file");
    let drawn = roadgen_viewer::opendrive_file(&path).expect("a drawing");

    // The same road as SUMO, whose lane shapes are vertices rather than polynomials
    // and so cannot be wrong in this particular way.
    roadgen_sumo::write(&map, directory.path()).expect("a SUMO network");
    let sampled = roadgen_viewer::sumo_directory(directory.path()).expect("a drawing");

    // Lane centres against lane centres. Comparing whole extents would compare an
    // outermost lane *edge* against an outermost lane *centre* and be slack by half a
    // lane; centre lines are the same thing in both pictures, so they can be held to
    // the sampling step rather than to the width of a road.
    let drawn = centres(&drawn).expect("OpenDRIVE drew lane centres");
    let sampled = centres(&sampled).expect("SUMO drew lane centres");
    for (label, a, b) in [
        ("min x", drawn.min.x, sampled.min.x),
        ("min y", drawn.min.y, sampled.min.y),
        ("max x", drawn.max.x, sampled.max.x),
        ("max y", drawn.max.y, sampled.max.y),
    ] {
        assert!(
            (a - b).abs() < 0.5,
            "the evaluated geometry and the sampled one disagree about {label}: \
             {a} against {b}"
        );
    }
}

/// What the lane centre lines of a drawing cover, which is the one extent two formats
/// measure the same way.
fn centres(drawing: &Drawing) -> Option<Bounds> {
    let mut only = Drawing::new("centres");
    for shape in &drawing.shapes {
        if shape.kind() == Kind::Center {
            only.push(shape.clone());
        }
    }
    only.bounds()
}

fn has(drawing: &Drawing, kind: Kind) -> bool {
    drawing.shapes.iter().any(|shape| shape.kind() == kind)
}

fn every_scenario() -> Vec<(&'static str, ValidatedMap)> {
    vec![
        ("straight", scenarios::straight_road()),
        ("bidirectional", scenarios::bidirectional_road()),
        ("multi-lane", scenarios::multi_lane_road()),
        ("joined", scenarios::two_roads_joined()),
        ("split", scenarios::split()),
        ("merge", scenarios::merge()),
        ("crossroads", scenarios::crossroads()),
        ("controlled", scenarios::controlled_crossroads()),
        ("graded", scenarios::graded_road()),
        ("spiral", scenarios::spiral_transition_road()),
        ("banked", scenarios::banked_curve()),
        ("lane drop", scenarios::lane_drop()),
        ("widening", scenarios::widening_road()),
    ]
}

/// Writes the map out in all four formats and reads every one of them back.
fn draw_all(map: &ValidatedMap, directory: &Path) -> Vec<(&'static str, Drawing)> {
    let xodr = directory.join("map.xodr");
    roadgen_opendrive::write(map, &xodr).expect("an OpenDRIVE file");

    let sumo = directory.join("sumo");
    std::fs::create_dir_all(&sumo).expect("a directory");
    roadgen_sumo::write(map, &sumo).expect("a SUMO network");

    let clip = directory.join("clip");
    std::fs::create_dir_all(&clip).expect("a directory");
    roadgen_clipgt::write(map, &clip, &clip_config()).expect("a clip");

    let scene = directory.join("scene.json");
    roadgen_gpudrive::write(map, &scene, &scene_config()).expect("a scene");

    vec![
        (
            "OpenDRIVE",
            roadgen_viewer::opendrive_file(&xodr).expect("an OpenDRIVE drawing"),
        ),
        (
            "SUMO",
            roadgen_viewer::sumo_directory(&sumo).expect("a SUMO drawing"),
        ),
        (
            "ClipGT",
            roadgen_viewer::clipgt_directory(&clip).expect("a ClipGT drawing"),
        ),
        (
            "GPUDrive",
            roadgen_viewer::gpudrive_file(&scene).expect("a GPUDrive drawing"),
        ),
    ]
}

/// A clip needs something driving through it — a directory with no egomotion is not a
/// clip — and a scene needs an agent for the same reason. Both take the default route.
fn clip_config() -> roadgen_clipgt::ClipConfig {
    roadgen_clipgt::ClipConfig::new("clip")
}

fn scene_config() -> roadgen_gpudrive::SceneConfig {
    roadgen_gpudrive::SceneConfig::default()
}
