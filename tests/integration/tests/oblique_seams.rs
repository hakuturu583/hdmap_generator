//! Two lanelets one after the other whose seam is drawn nearly along the lane.
//!
//! OpenDRIVE ends a lane square to its road, and a seam twenty metres long across
//! a lane three and a half wide cannot be met that way: one side of it or the
//! other comes out metres from the file. So the reader takes the two as one lane.

use roadgen_core::map::TrafficHandedness;
use roadgen_core::validation::ValidatedMap;
use roadgen_integration_tests::opendrive_eval::RoadEvaluator;
use roadgen_integration_tests::osm::Osm;
use roadgen_integration_tests::reparse_opendrive;
use roadgen_lanelet2::ReadOptions;

/// Is `p` inside the polygon? Even–odd rule, in plan.
fn inside(polygon: &[(f64, f64)], p: (f64, f64)) -> bool {
    let mut odd = false;
    let n = polygon.len();
    for i in 0..n {
        let (a, b) = (polygon[i], polygon[(i + 1) % n]);
        if (a.1 > p.1) != (b.1 > p.1) && p.0 < a.0 + (p.1 - a.1) * (b.0 - a.0) / (b.1 - a.1) {
            odd = !odd;
        }
    }
    odd
}

/// How far `p` lies outside the polygon, in plan: nothing inside it.
fn outside(polygon: &[(f64, f64)], p: (f64, f64)) -> f64 {
    if inside(polygon, p) {
        return 0.0;
    }
    let n = polygon.len();
    (0..n)
        .map(|i| {
            let (a, b) = (polygon[i], polygon[(i + 1) % n]);
            let (dx, dy) = (b.0 - a.0, b.1 - a.1);
            let length = dx * dx + dy * dy;
            let u = if length > 0.0 {
                (((p.0 - a.0) * dx + (p.1 - a.1) * dy) / length).clamp(0.0, 1.0)
            } else {
                0.0
            };
            (a.0 + u * dx - p.0).hypot(a.1 + u * dy - p.1)
        })
        .fold(f64::INFINITY, f64::min)
}

/// A lane 3.5 m wide, gently curving, in two lanelets whose seam runs from 20 m
/// along the left boundary to 40 m along the right: the file, and the two ids.
fn seam_along_the_lane() -> (String, i64, i64) {
    let curve = |x: f64| 0.002 * x * x;
    let at = |x: f64, y: f64| (x, curve(x) + y);
    let mut osm = Osm::new();
    let left_a = osm.line(None, &[at(0.0, 3.5), at(10.0, 3.5), at(20.0, 3.5)]);
    let left_b = osm.line(
        Some(left_a[2]),
        &[at(30.0, 3.5), at(45.0, 3.5), at(60.0, 3.5)],
    );
    let right_a = osm.line(None, &[at(0.0, 0.0), at(20.0, 0.0), at(40.0, 0.0)]);
    let right_b = osm.line(Some(right_a[2]), &[at(50.0, 0.0), at(60.0, 0.0)]);
    let first = osm.lanelet(left_a, right_a);
    let second = osm.lanelet(left_b, right_b);
    (osm.xml(), first, second)
}

fn read(xml: &str, max_seam_along: Option<f64>) -> ValidatedMap {
    roadgen_lanelet2::from_osm_str(
        xml,
        &ReadOptions {
            handedness: TrafficHandedness::LeftHand,
            max_seam_along,
            ..ReadOptions::default()
        },
    )
    .unwrap()
    .map
    .validate()
    .unwrap()
}

#[test]
fn how_long_a_seam_is_before_the_lanelets_are_joined_is_an_option() {
    let (xml, _, _) = seam_along_the_lane();
    // The seam runs some 20 m along the lane.
    assert_eq!(read(&xml, Some(15.0)).roads.len(), 1);
    assert_eq!(read(&xml, Some(25.0)).roads.len(), 2);
    assert_eq!(read(&xml, None).roads.len(), 2);
}

#[test]
fn a_seam_drawn_along_the_lane_is_read_as_one_lane() {
    let (xml, first, second) = seam_along_the_lane();
    let map = read(&xml, Some(roadgen_lanelet2::DEFAULT_MAX_SEAM_ALONG));

    assert_eq!(map.roads.len(), 1);
    let road = map.roads.iter().next().unwrap();
    assert_eq!(
        road.name.as_deref(),
        Some(format!("lanelet {first}+{second}").as_str())
    );

    // The lane's outline as the file draws it, both lanelets together.
    let config = map.metadata.sampling;
    let lane = map.lanes.iter().next().unwrap();
    let mut outline: Vec<(f64, f64)> = lane
        .left_boundary
        .to_polyline(config)
        .unwrap()
        .points()
        .iter()
        .map(|p| (p.x, p.y))
        .collect();
    outline.extend(
        lane.right_boundary
            .to_polyline(config)
            .unwrap()
            .points()
            .iter()
            .rev()
            .map(|p| (p.x, p.y)),
    );

    // The lane OpenDRIVE draws, and how far each outline strays from the other.
    let document = reparse_opendrive(&map);
    let evaluator = RoadEvaluator::find(&document, "0").unwrap();
    let id = roadgen_opendrive::lane_number(lane.side, lane.ordinal);
    let steps = 400;
    let (mut near, mut far) = (Vec::new(), Vec::new());
    for i in 0..=steps {
        let s = (evaluator.length() * i as f64 / steps as f64).min(evaluator.length() - 1e-9);
        let (a, b) = evaluator.lane_edges(id, s).unwrap();
        near.push((a.x, a.y));
        far.push((b.x, b.y));
    }
    let drawn: Vec<(f64, f64)> = near.iter().chain(far.iter().rev()).copied().collect();
    let off_the_file = drawn
        .iter()
        .map(|p| outside(&outline, *p))
        .fold(0.0, f64::max);
    let left_out = outline
        .iter()
        .map(|p| outside(&drawn, *p))
        .fold(0.0, f64::max);
    assert!(
        off_the_file < 0.05 && left_out < 0.05,
        "OpenDRIVE draws the lane {off_the_file} m past the file's and leaves out \
         {left_out} m of it"
    );
}

#[test]
fn an_option_the_reader_cannot_use_is_refused() {
    let (xml, _, _) = seam_along_the_lane();
    let refused = |options: ReadOptions| {
        roadgen_lanelet2::from_osm_str(&xml, &options)
            .err()
            .map(|error| error.to_string())
            .unwrap_or_default()
    };
    let default = ReadOptions::default;
    for (options, name) in [
        (
            ReadOptions {
                max_seam_along: Some(-1.0),
                ..default()
            },
            "max_seam_along",
        ),
        (
            ReadOptions {
                max_end_lean_degrees: 90.0,
                ..default()
            },
            "max_end_lean_degrees",
        ),
        (
            ReadOptions {
                max_edge_miss: 0.0,
                ..default()
            },
            "max_edge_miss",
        ),
        (
            ReadOptions {
                lift_tolerance: f64::NAN,
                ..default()
            },
            "lift_tolerance",
        ),
        (
            ReadOptions {
                junction_end_distance: -0.5,
                ..default()
            },
            "junction_end_distance",
        ),
        (
            ReadOptions {
                grade_separation: f64::INFINITY,
                ..default()
            },
            "grade_separation",
        ),
    ] {
        assert!(refused(options).contains(name), "{name} was not refused");
    }
}
