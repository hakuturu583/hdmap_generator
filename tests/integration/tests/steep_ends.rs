//! A lane rounding a corner whose ends are drawn steeply across it.
//!
//! OpenDRIVE ends a lane square to its reference line, so the Lanelet2 reader turns
//! the reference line near each end until it meets the lanelet's end square. With
//! the end drawn at sixty-odd degrees and the reference line on the inside of a
//! corner, that turn used to be refused, and the far edge swung out past the end —
//! a triangle of road where the file has none.

use roadgen_core::map::TrafficHandedness;
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

#[test]
fn a_lane_rounding_a_corner_with_steep_ends_stays_on_the_road() {
    // Lanelets 468 and 471 of Autoware's Nishi-Shinjuku map, as drawn: the two
    // lanes of a U-shaped corner under left-hand traffic, sharing the line between
    // them. 468 is the outer one, and its ends are drawn at 64° and 75° across it.
    let outer = [
        (14.81, 2.84),
        (14.12, 1.80),
        (13.03, 0.55),
        (11.67, -0.60),
        (9.95, -1.61),
        (8.29, -2.17),
        (3.85, -3.10),
        (-1.71, -4.27),
        (-5.91, -5.15),
        (-7.48, -5.32),
        (-9.26, -5.21),
        (-10.88, -4.86),
        (-12.40, -4.32),
        (-13.88, -3.56),
        (-14.26, -3.29),
    ];
    let between = [
        (13.38, 9.67),
        (13.49, 8.40),
        (13.32, 6.98),
        (12.95, 5.73),
        (12.42, 4.61),
        (11.76, 3.60),
        (10.94, 2.67),
        (9.95, 1.83),
        (8.71, 1.10),
        (7.51, 0.70),
        (3.25, -0.20),
        (-2.32, -1.37),
        (-6.38, -2.22),
        (-7.54, -2.35),
        (-8.86, -2.26),
        (-10.07, -2.01),
        (-11.22, -1.59),
        (-12.33, -1.03),
        (-13.40, -0.27),
        (-14.41, 0.75),
        (-15.30, 2.09),
        (-15.79, 3.45),
    ];
    let inner = [
        (8.66, 8.97),
        (8.71, 8.47),
        (8.64, 7.95),
        (8.49, 7.44),
        (8.26, 6.94),
        (7.95, 6.48),
        (7.60, 6.07),
        (7.19, 5.72),
        (6.73, 5.46),
        (6.26, 5.30),
        (2.26, 4.46),
        (-3.30, 3.29),
        (-7.13, 2.49),
        (-7.65, 2.43),
        (-8.21, 2.46),
        (-8.77, 2.59),
        (-9.32, 2.78),
        (-9.84, 3.05),
        (-10.31, 3.38),
        (-10.71, 3.78),
        (-11.01, 4.24),
        (-11.19, 4.74),
    ];
    let mut osm = Osm::new();
    let (outer, between, inner) = (
        osm.line(None, &outer),
        osm.line(None, &between),
        osm.line(None, &inner),
    );
    let (outer, between, inner) = (osm.way(outer), osm.way(between), osm.way(inner));
    osm.lanelet_on(outer, between, None);
    osm.lanelet_on(between, inner, None);
    let map = roadgen_lanelet2::from_osm_str(
        &osm.xml(),
        &ReadOptions {
            handedness: TrafficHandedness::LeftHand,
            ..ReadOptions::default()
        },
    )
    .unwrap()
    .map
    .validate()
    .unwrap();
    // Each lanelet's outline in the map's own coordinates, as the reader placed it.
    let config = map.metadata.sampling;
    let outlines: Vec<Vec<(f64, f64)>> = map
        .lanes
        .iter()
        .map(|lane| {
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
            outline
        })
        .collect();
    assert_eq!(outlines.len(), 2);

    // Every edge of every lane OpenDRIVE draws lies on the two lanelets.
    let document = reparse_opendrive(&map);
    let mut worst: f64 = 0.0;
    for (index, road) in map.roads.iter().enumerate() {
        let evaluator = RoadEvaluator::find(&document, &index.to_string()).unwrap();
        for lane in map.lanes_of(&road.id) {
            let id = roadgen_opendrive::lane_number(lane.side, lane.ordinal);
            let steps = 400;
            for i in 0..=steps {
                let s =
                    (evaluator.length() * i as f64 / steps as f64).min(evaluator.length() - 1e-9);
                let (a, b) = evaluator.lane_edges(id, s).unwrap();
                for p in [(a.x, a.y), (b.x, b.y)] {
                    let off = outlines
                        .iter()
                        .map(|outline| outside(outline, p))
                        .fold(f64::INFINITY, f64::min);
                    worst = worst.max(off);
                }
            }
        }
    }
    // Before the reader could turn a reference line more than 45°, the outer lane
    // swung 1.2 m out past its ends.
    assert!(
        worst < 0.05,
        "OpenDRIVE draws a lane {worst} m off the road"
    );
}
