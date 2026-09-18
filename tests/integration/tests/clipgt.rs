//! Exporting to ClipGT.
//!
//! ClipGT is a flat format: rails, lines and outlines, with no topology to check
//! against. So what these tests can check is that every layer a reader looks for is
//! there, that what is in it is the geometry the IR holds — in all three dimensions,
//! which is the part a format made of "lists of points" invites you to lose — and
//! that the parts ClipGT has and the IR does not are derived rather than invented.

use roadgen_clipgt::ClipConfig;
use roadgen_core::prelude::*;
use roadgen_core::Vector3;
use roadgen_integration_tests::clipgt_read::{exists, layer};
use roadgen_integration_tests::{scenarios, write_clip};

/// Every file `ClipGTLoader` opens, and the two it refuses a directory without.
const LAYERS: [&str; 10] = [
    "calibration_estimate",
    "egomotion_estimate",
    "lane",
    "lane_line",
    "road_boundary",
    "crosswalk",
    "wait_line",
    "traffic_light",
    "traffic_sign",
    "intersection_area",
];

fn nearest(points: &[Point3], to: Point3) -> f64 {
    points
        .iter()
        .map(|point| point.distance_to(to))
        .fold(f64::INFINITY, f64::min)
}

#[test]
fn the_tables_can_be_built_without_writing_a_clip() {
    // The other exporters each hand back the document they built; this is ClipGT's,
    // and a caller that only wants to look at a clip should not have to make files to
    // do it.
    let map = scenarios::controlled_crossroads();
    let layers =
        roadgen_clipgt::to_layers(&map, &ClipConfig::new("x")).expect("the map should render");

    let names: Vec<&str> = layers.iter().map(|layer| layer.name).collect();
    assert_eq!(
        names, LAYERS,
        "the same layers, in the order they are written"
    );

    for layer in &layers {
        // Every layer a reader refuses a clip without has to carry its row here too.
        if ["calibration_estimate", "egomotion_estimate", "lane"].contains(&layer.name) {
            assert!(layer.batch.num_rows() > 0, "{} is empty", layer.name);
        }
    }
}

#[test]
fn a_clip_holds_every_layer_a_reader_looks_for() {
    let map = scenarios::controlled_crossroads();
    let (directory, clip) = write_clip(&map, &ClipConfig::new("x"));
    assert_eq!(clip, "x");
    for name in LAYERS {
        assert!(exists(directory.path(), &clip, name), "{name} is missing");
    }

    // The two that decide whether a directory is a clip at all.
    let calibration = layer(directory.path(), &clip, "calibration_estimate");
    assert_eq!(calibration.len(), 1, "a reader takes the first row");
    assert!(calibration[0].strings["rig_json"].contains("sensors"));
    assert!(!layer(directory.path(), &clip, "egomotion_estimate").is_empty());
}

#[test]
fn a_lanes_rails_are_the_irs_own_boundaries() {
    let map = scenarios::two_roads_joined();
    let (directory, clip) = write_clip(&map, &ClipConfig::new("x"));
    let rows = layer(directory.path(), &clip, "lane");

    let drivable: Vec<_> = map
        .lanes
        .iter()
        .filter(|lane| lane.lane_type.is_drivable())
        .collect();
    assert_eq!(rows.len(), drivable.len());

    for (row, lane) in rows.iter().zip(&drivable) {
        let travel = lane.travel_geometry(map.metadata.sampling).unwrap();
        for (field, curve) in [("left_rail", &travel.left), ("right_rail", &travel.right)] {
            let written = row.list(field);
            let expected = curve.to_polyline(map.metadata.sampling).unwrap();
            assert_eq!(written.len(), expected.len(), "{} {field}", lane.id);
            for (got, want) in written.iter().zip(expected.points()) {
                assert!(got.distance_to(*want) < 1e-9, "{} {field}", lane.id);
            }
        }
        // And they are the driver's left and right, not the reference line's: a
        // backward lane's rails are swapped and reversed, which is the mistake that
        // produces a lane you can only drive the wrong way down.
        assert!(row.list("left_rail")[0].distance_to(lane.entry_point()) < 5.0);
    }
}

#[test]
fn a_graded_road_keeps_its_heights() {
    // The whole format is lists of points with a `z` each, so nothing forces an
    // exporter to fill them in properly. This is the road that notices.
    let map = scenarios::graded_road();
    let (directory, clip) = write_clip(&map, &ClipConfig::new("x"));

    let lane = map.lanes.iter().next().unwrap();
    let expected: Vec<f64> = lane
        .left_boundary
        .to_polyline(map.metadata.sampling)
        .unwrap()
        .points()
        .iter()
        .map(|point| point.z)
        .collect();
    let climb = expected.last().unwrap() - expected.first().unwrap();
    assert!(climb.abs() > 1.0, "the fixture should actually climb");

    let heights: Vec<f64> = layer(directory.path(), &clip, "lane")[0]
        .list("left_rail")
        .iter()
        .map(|point| point.z)
        .collect();
    assert_eq!(heights.len(), expected.len());
    for (got, want) in heights.iter().zip(&expected) {
        assert!((got - want).abs() < 1e-9, "{got} vs {want}");
    }

    // The road boundary and the ego track climb with it too.
    let boundary = layer(directory.path(), &clip, "road_boundary");
    let spans = |points: &[Point3]| {
        let (min, max) = points.iter().fold((f64::MAX, f64::MIN), |(lo, hi), point| {
            (lo.min(point.z), hi.max(point.z))
        });
        max - min
    };
    assert!(spans(boundary[0].list("location")) > 1.0);
    let ego = layer(directory.path(), &clip, "egomotion_estimate");
    let track: Vec<Point3> = ego.iter().map(|pose| pose.point("location")).collect();
    assert!(
        spans(&track) > 1.0,
        "the ego drives up the hill, not through it"
    );
}

#[test]
fn a_banked_curve_rolls_the_vehicle_rather_than_leaving_it_upright() {
    let map = scenarios::banked_curve();
    let (directory, clip) = write_clip(&map, &ClipConfig::new("x"));
    let poses = layer(directory.path(), &clip, "egomotion_estimate");

    // `up` in the vehicle's own frame, turned into the map's by each pose.
    let tilts: Vec<f64> = poses
        .iter()
        .map(|pose| {
            let up = roadgen_clipgt::ego::rotate(
                pose.quaternions["orientation"],
                Vector3::new(0.0, 0.0, 1.0),
            );
            up.z.clamp(-1.0, 1.0).acos()
        })
        .collect();

    // Flat where the road is flat, rolled where it is banked — and the roll matches
    // the superelevation the IR holds rather than being some other tilt.
    assert!(tilts[0] < 1e-6, "the approach is flat");
    let steepest = tilts.iter().cloned().fold(0.0, f64::max);
    assert!(
        (steepest - 0.06).abs() < 1e-3,
        "a 6 % cross-fall should tip the vehicle by 0.06 rad, got {steepest}"
    );
}

#[test]
fn timestamps_and_spacing_follow_the_speed_and_frame_rate() {
    let map = scenarios::straight_road();
    let (directory, clip) = write_clip(
        &map,
        &ClipConfig::new("x").with_speed(20.0).with_frame_rate(10.0),
    );
    let poses = layer(directory.path(), &clip, "egomotion_estimate");
    assert!(poses.len() > 2);

    for pair in poses.windows(2) {
        let step = pair[1].integers["timestamp_micros"] - pair[0].integers["timestamp_micros"];
        assert_eq!(step, 100_000, "10 Hz is a frame every 100 ms");
        let moved = pair[0]
            .point("location")
            .distance_to(pair[1].point("location"));
        // 20 m/s at 10 Hz is 2 m a frame, except the last, which stops at the end.
        assert!(moved <= 2.0 + 1e-6, "{moved} m in one frame");
    }
    let first = poses.first().unwrap().point("location");
    let last = poses.last().unwrap().point("location");
    assert!(first.distance_to(last) > 50.0);
}

#[test]
fn a_marked_edge_becomes_one_line_carrying_its_own_paint() {
    let map = scenarios::multi_lane_road();
    let (directory, clip) = write_clip(&map, &ClipConfig::new("x"));
    let lines = layer(directory.path(), &clip, "lane_line");

    // Three lanes across is four edges, and the two lanes either side of an inner
    // edge share it — one line each, not one per lane.
    let edges: usize = map
        .lanes
        .iter()
        .flat_map(|lane| [lane.left_edge, lane.right_edge])
        .collect::<std::collections::BTreeSet<_>>()
        .len();
    assert_eq!(lines.len(), edges, "one row per edge");

    // Each line carries the paint the IR gave that edge, in ClipGT's vocabulary —
    // not a single default for the whole map.
    let mut written: Vec<(String, String)> = lines
        .iter()
        .map(|line| {
            assert!(line.list("line_rail").len() >= 2);
            assert_eq!(line.string_lists["colors"].len(), 1);
            assert_eq!(line.string_lists["styles"].len(), 1);
            (
                line.string_lists["colors"][0].clone(),
                line.string_lists["styles"][0].clone(),
            )
        })
        .collect();
    written.sort();

    let mut expected: Vec<(String, String)> = vec![
        // The kerbside edge of the outer lane, the two broken lines between the
        // three forward lanes, and the yellow one against oncoming traffic.
        ("WHITE".into(), "SOLID_SINGLE".into()),
        ("WHITE".into(), "DASHED_SINGLE".into()),
        ("WHITE".into(), "DASHED_SINGLE".into()),
        ("YELLOW".into(), "SOLID_SINGLE".into()),
        ("WHITE".into(), "SOLID_SINGLE".into()),
    ];
    expected.sort();
    assert_eq!(written, expected);
}

#[test]
fn traffic_control_arrives_where_the_ir_put_it() {
    let map = scenarios::controlled_crossroads();
    let (directory, clip) = write_clip(&map, &ClipConfig::new("x"));

    let ir_light = map
        .objects
        .iter()
        .find(|object| object.kind == MapObjectKind::TrafficLight)
        .unwrap();
    let ObjectGeometry::Line(bar) = &ir_light.geometry else {
        panic!("a light is a bar across the lane")
    };
    let centre = bar.start_point().lerp(bar.end_point(), 0.5);

    let lights = layer(directory.path(), &clip, "traffic_light");
    assert_eq!(lights.len(), 1);
    assert!(lights[0].point("center").distance_to(centre) < 1e-9);
    // Five metres up the road's normal, and that height is in the file.
    assert!(lights[0].point("center").z > 4.9);

    // It faces the traffic it stops, which on the north approach drives south.
    let facing = roadgen_clipgt::ego::rotate(
        lights[0].quaternions["orientation"],
        Vector3::new(1.0, 0.0, 0.0),
    );
    assert!(
        facing.y > 0.99,
        "the light looks back up the approach: {facing:?}"
    );

    let stop_lines = layer(directory.path(), &clip, "wait_line");
    assert_eq!(stop_lines.len(), 1);
    assert!(stop_lines[0].list("location").len() >= 2);

    let crossings = layer(directory.path(), &clip, "crosswalk");
    assert_eq!(crossings.len(), 1);
    assert!(
        crossings[0].list("location").len() >= 4,
        "a crossing is a ring, not a line"
    );
}

#[test]
fn a_sign_keeps_the_callers_own_category() {
    let mut builder = MapBuilder::new(scenarios::metadata("signed"));
    let road = builder
        .add_road(
            RoadSpec::line(
                Point3::ORIGIN,
                Point3::new(100.0, 0.0, 0.0),
                scenarios::two_way(),
            )
            .unwrap()
            .with_name("main"),
        )
        .unwrap();
    builder
        .add_traffic_sign(&LaneRef::new(road, 0), LaneEnd::End, "STOP", 2.4)
        .unwrap();
    let map = builder.finish().unwrap().validate().unwrap();

    let (directory, clip) = write_clip(&map, &ClipConfig::new("x"));
    let signs = layer(directory.path(), &clip, "traffic_sign");
    assert_eq!(signs.len(), 1);
    assert_eq!(signs[0].strings["category"], "STOP");
    // The IR does not model the plate, so the format's own default size is written
    // rather than a measurement nobody made.
    let dimensions = signs[0].point("dimensions");
    assert_eq!((dimensions.x, dimensions.y, dimensions.z), (0.8, 0.3, 0.8));
}

#[test]
fn a_junction_becomes_an_outline_that_covers_it() {
    let map = scenarios::crossroads();
    let (directory, clip) = write_clip(&map, &ClipConfig::new("x"));
    let areas = layer(directory.path(), &clip, "intersection_area");
    assert_eq!(areas.len(), map.junctions.len());

    let outline = areas[0].list("location");
    assert!(outline.len() >= 3);

    let covered: Vec<Point3> = map
        .junctions
        .iter()
        .next()
        .unwrap()
        .connecting_roads
        .iter()
        .flat_map(|road| map.lanes_of(road))
        .flat_map(|lane| {
            [&lane.left_boundary, &lane.right_boundary].map(|boundary| {
                boundary
                    .to_polyline(map.metadata.sampling)
                    .unwrap()
                    .points()
                    .to_vec()
            })
        })
        .flatten()
        .collect();

    // Every connector's boundary vertex is inside the outline, which is what makes it
    // an area rather than a shape near the junction.
    for point in &covered {
        assert!(
            inside(outline, *point),
            "{point:?} is outside the junction outline"
        );
    }
    // And the outline's own vertices are those points, not flattened copies of them:
    // the hull is taken in plan view, but what comes out keeps its height.
    for vertex in outline {
        assert!(
            nearest(&covered, *vertex) < 1e-12,
            "{vertex:?} is not one of the points it was built from"
        );
    }
}

#[test]
fn an_empty_layer_is_still_written() {
    // A map with no furniture. A reader that finds no file and one that finds an
    // empty table should not be told different things.
    let map = scenarios::straight_road();
    let (directory, clip) = write_clip(&map, &ClipConfig::new("x"));
    for name in ["crosswalk", "wait_line", "traffic_light", "traffic_sign"] {
        assert!(exists(directory.path(), &clip, name), "{name}");
        assert!(layer(directory.path(), &clip, name).is_empty(), "{name}");
    }
}

#[test]
fn the_same_map_exports_identically_twice() {
    let map = scenarios::controlled_crossroads();
    let (first, clip) = write_clip(&map, &ClipConfig::new("x"));
    let (second, _) = write_clip(&map, &ClipConfig::new("x"));
    for name in LAYERS {
        let a = std::fs::read(first.path().join(format!("{clip}.{name}.parquet"))).unwrap();
        let b = std::fs::read(second.path().join(format!("{clip}.{name}.parquet"))).unwrap();
        assert_eq!(a, b, "{name} differs between two exports of one map");
    }
}

#[test]
fn what_the_format_cannot_carry_is_reported() {
    let map = scenarios::controlled_crossroads();
    let problems = roadgen_clipgt::check(&map, None);
    assert!(
        problems.iter().any(|problem| problem.contains("topology")),
        "{problems:?}"
    );
    // A map with nothing to lose says nothing.
    assert!(roadgen_clipgt::check(&scenarios::straight_road(), None).is_empty());
}

/// Whether `point`'s shadow lies in the polygon's, by the winding rule.
fn inside(polygon: &[Point3], point: Point3) -> bool {
    const EDGE: f64 = 1e-6;
    let mut crossings = 0;
    for index in 0..polygon.len() {
        let (a, b) = (polygon[index], polygon[(index + 1) % polygon.len()]);
        // A point exactly on an edge counts as inside; the boundary vertices of the
        // outline itself are all on one.
        let along = b - a;
        let length = along.horizontal_norm();
        if length > EDGE {
            let side = ((point.x - a.x) * along.y - (point.y - a.y) * along.x) / length;
            let position =
                ((point.x - a.x) * along.x + (point.y - a.y) * along.y) / (length * length);
            if side.abs() < EDGE && (-EDGE..=1.0 + EDGE).contains(&position) {
                return true;
            }
        }
        if (a.y > point.y) != (b.y > point.y) {
            let x = a.x + (point.y - a.y) / (b.y - a.y) * (b.x - a.x);
            if point.x < x {
                crossings += 1;
            }
        }
    }
    crossings % 2 == 1
}
