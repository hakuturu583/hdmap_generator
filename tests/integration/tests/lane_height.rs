//! Lane heights: a lane whose edges stand off the road surface — a pavement a kerb
//! above the carriageway. The IR lifts its boundaries, a junction carries the lift
//! round its corners, OpenDRIVE writes it as `<height>` and reads it back, and
//! Lanelet2 gives a kerb two lines instead of one.

use roadgen_core::map::{Lane, Road};
use roadgen_core::prelude::*;
use roadgen_core::topology::LateralSide;
use roadgen_integration_tests::opendrive_eval::{Position, RoadEvaluator};
use roadgen_integration_tests::{reparse_opendrive, scenarios};

/// A kerb's height, metres.
const KERB: f64 = 0.15;

/// A carriageway each way with a pavement on each side, raised a kerb.
fn raised_street() -> Vec<LaneSpec> {
    let pavement = |direction| {
        scenarios::lane(2.0, direction)
            .with_type(LaneType::Sidewalk)
            .with_height(LaneHeight::constant(KERB, KERB).unwrap())
    };
    vec![
        scenarios::lane(3.5, Direction::Backward),
        pavement(Direction::Backward),
        scenarios::lane(3.5, Direction::Forward),
        pavement(Direction::Forward),
    ]
}

fn straight_street(roll: f64) -> ValidatedMap {
    let mut builder = MapBuilder::new(scenarios::metadata("kerb"));
    builder
        .add_road(
            RoadSpec::line(
                Point3::new(0.0, 0.0, 10.0),
                Point3::new(80.0, 0.0, 12.0),
                raised_street(),
            )
            .unwrap()
            .with_superelevation(Poly3Profile::constant(roll))
            .with_name("street"),
        )
        .unwrap();
    builder.finish().unwrap().validate().unwrap()
}

/// Four raised streets meeting at a junction, their pavements joined round each
/// corner by the builder.
fn crossroads() -> ValidatedMap {
    let mut builder = MapBuilder::new(scenarios::metadata("kerbed crossroads"));
    let junction = builder.add_junction(Some("x"));
    let mut arm = |name: &str, from: (f64, f64), to: (f64, f64)| {
        builder
            .add_road(
                RoadSpec::line(
                    Point3::new(from.0, from.1, 0.0),
                    Point3::new(to.0, to.1, 0.0),
                    raised_street(),
                )
                .unwrap()
                .with_name(name),
            )
            .unwrap()
    };
    let w = arm("w", (-60.0, 0.0), (-12.0, 0.0));
    let e = arm("e", (12.0, 0.0), (60.0, 0.0));
    let n = arm("n", (0.0, 12.0), (0.0, 60.0));
    let s = arm("s", (0.0, -60.0), (0.0, -12.0));
    for (from, to) in [(&w, &e), (&s, &n), (&w, &n), (&s, &e)] {
        builder.connect_via(&junction, from, to).unwrap();
    }
    builder.finish().unwrap().validate().unwrap()
}

fn opendrive_lane_id(lane: &Lane) -> i64 {
    match lane.side {
        LateralSide::Left => lane.ordinal as i64,
        LateralSide::Right => -(lane.ordinal as i64),
    }
}

#[test]
fn a_raised_pavement_stands_a_kerb_above_the_carriageway() {
    let map = straight_street(0.0);
    let config = map.metadata.sampling;
    for lane in map.lanes.iter() {
        let lift = if lane.lane_type == LaneType::Sidewalk {
            KERB
        } else {
            0.0
        };
        let road = map.road(&lane.road).unwrap();
        for boundary in [&lane.left_boundary, &lane.right_boundary, &lane.centerline] {
            for point in boundary.to_polyline(config).unwrap().points() {
                // Off the surface along its normal: the road climbs, so the normal
                // leans back and a kerb rises a hair less than its height in z.
                let frame = road.frame_at(point.x.clamp(0.0, 80.0), config).unwrap();
                let above = (*point - frame.origin).dot(frame.up.get());
                assert!(
                    (above - lift).abs() < 1e-9,
                    "{} stands {above} m off the road, not {lift}",
                    lane.id
                );
            }
        }
    }
}

#[test]
fn on_a_banked_road_the_kerb_rises_along_the_surface_normal() {
    // Rolled 3°, the pavement's edges stand a kerb off the tilted surface: along its
    // normal, which leans away from the high side.
    let roll = 3.0_f64.to_radians();
    let map = straight_street(roll);
    let config = map.metadata.sampling;
    let flat = straight_street(0.0);
    for (lane, level) in map.lanes.iter().zip(flat.lanes.iter()) {
        if lane.lane_type != LaneType::Sidewalk {
            continue;
        }
        let road = map.road(&lane.road).unwrap();
        for (point, _) in lane
            .left_boundary
            .to_polyline(config)
            .unwrap()
            .points()
            .iter()
            .zip(level.left_boundary.to_polyline(config).unwrap().points())
        {
            let frame = road.frame_at(point.x, config).unwrap();
            let lift = (*point - frame.origin).dot(frame.up.get());
            assert!((lift - KERB).abs() < 1e-9, "{} lifted {lift}", lane.id);
        }
    }
}

#[test]
fn a_raised_pavement_turns_each_corner_raised_and_without_a_step() {
    // Validation compares the boundaries where every connection joins, so a corner
    // pavement laid flat against raised arms would fail here.
    let map = crossroads();
    let config = map.metadata.sampling;
    let corners: Vec<&Lane> = map
        .lanes
        .iter()
        .filter(|lane| {
            lane.lane_type == LaneType::Sidewalk
                && map.road(&lane.road).is_some_and(|road| road.is_connector())
        })
        .collect();
    assert_eq!(corners.len(), 4);
    for lane in corners {
        // The corner's reference line runs through the arms' lifted lane centres,
        // so a kerb the same height all across leaves nothing to lift off it.
        assert!(lane.height.is_flat(), "{}", lane.id);
        for point in lane.centerline.to_polyline(config).unwrap().points() {
            assert!(
                (point.z - KERB).abs() < 1e-9,
                "{} dips to {} round its corner",
                lane.id,
                point.z
            );
        }
    }
}

/// Every lane edge the independent evaluator finds in the document, against the
/// IR's, at the IR's own stations.
fn largest_edge_disagreement(map: &ValidatedMap) -> f64 {
    let document = reparse_opendrive(map);
    let mut worst: f64 = 0.0;
    for (index, road) in map.roads.iter().enumerate() {
        let evaluator = RoadEvaluator::find(&document, &index.to_string()).unwrap();
        let stations = map.vertex_stations(&road.id).unwrap();
        for lane in map.lanes_of(&road.id) {
            let (inner, outer) = match lane.side {
                LateralSide::Left => (&lane.right_boundary, &lane.left_boundary),
                LateralSide::Right => (&lane.left_boundary, &lane.right_boundary),
            };
            let inner = inner.to_polyline(map.metadata.sampling).unwrap();
            let outer = outer.to_polyline(map.metadata.sampling).unwrap();
            let (start, end) = lane.station_range;
            let own: Vec<f64> = stations
                .iter()
                .copied()
                .filter(|station| *station >= start - 1e-9 && *station <= end + 1e-9)
                .collect();
            for ((station, inner), outer) in own.iter().zip(inner.points()).zip(outer.points()) {
                let (found_inner, found_outer) = evaluator
                    .lane_edges(opendrive_lane_id(lane), station.min(end - 1e-9))
                    .unwrap();
                for (found, wanted) in [(found_inner, inner), (found_outer, outer)] {
                    worst = worst.max(found.distance_to(Position {
                        x: wanted.x,
                        y: wanted.y,
                        z: wanted.z,
                    }));
                }
            }
        }
    }
    worst
}

#[test]
fn opendrive_writes_the_lift_as_height_and_an_evaluator_finds_it_there() {
    for (name, map) in [
        ("level", straight_street(0.0)),
        ("banked", straight_street(3.0_f64.to_radians())),
        ("crossroads", crossroads()),
    ] {
        let xml = roadgen_opendrive::to_xml(&map).unwrap();
        assert!(xml.contains("<height "), "{name}: no <height> written");
        let worst = largest_edge_disagreement(&map);
        assert!(
            worst < 1e-3,
            "{name}: the document puts an edge {worst} m off"
        );
    }
}

#[test]
fn a_lift_read_back_from_opendrive_is_the_lift_that_was_written() {
    for (name, map) in [
        ("banked", straight_street(3.0_f64.to_radians())),
        ("crossroads", crossroads()),
    ] {
        let xml = roadgen_opendrive::to_xml(&map).unwrap();
        let back = roadgen_opendrive::from_xml(&xml)
            .unwrap()
            .map
            .validate()
            .unwrap_or_else(|e| panic!("{name}: {e}"));
        let config = map.metadata.sampling;
        for (index, road) in map.roads.iter().enumerate() {
            let twins = back.lanes_of(&RoadId::new(index.to_string()));
            for lane in map.lanes_of(&road.id) {
                let twin = twins
                    .iter()
                    .find(|other| other.side == lane.side && other.ordinal == lane.ordinal)
                    .unwrap();
                for station in [lane.station_range.0, lane.station_range.1] {
                    let (a, b) = (lane.height.evaluate(station), twin.height.evaluate(station));
                    assert!(
                        (a.0 - b.0).abs() < 1e-9 && (a.1 - b.1).abs() < 1e-9,
                        "{name}: {} came back lifted {b:?}, not {a:?}",
                        lane.id
                    );
                }
                let (mine, theirs) = (
                    lane.left_boundary.to_polyline(config).unwrap(),
                    twin.left_boundary.to_polyline(config).unwrap(),
                );
                for (p, q) in [(mine.first(), theirs.first()), (mine.last(), theirs.last())] {
                    assert!(p.distance_to(q) < 1e-3, "{name}: {} moved", lane.id);
                }
            }
        }
    }
}

#[test]
fn lanelet2_draws_a_kerb_as_two_lines_and_every_other_edge_as_one() {
    let map = straight_street(0.0);
    let xml = roadgen_lanelet2::to_osm_xml(&map).unwrap();
    let back = roadgen_lanelet2::from_osm_str(
        &xml,
        &roadgen_lanelet2::ReadOptions {
            handedness: map.metadata.handedness,
            origin: Some(map.metadata.origin),
            sampling: map.metadata.sampling,
        },
    )
    .unwrap()
    .map
    .validate()
    .unwrap();
    // The two carriageway lanes share their middle line and read back as one road
    // each way; a pavement shares no line with the carriageway beside it, since the
    // kerb puts its edge higher, and reads back as a road of its own.
    let (pavements, carriageways): (Vec<&Road>, Vec<&Road>) = back.roads.iter().partition(|road| {
        back.lanes_of(&road.id)
            .iter()
            .all(|lane| lane.lane_type == LaneType::Sidewalk)
    });
    assert_eq!(pavements.len(), 2);
    assert_eq!(carriageways.len(), 2);
    // And the pavements come back a kerb up.
    let config = back.metadata.sampling;
    for road in pavements {
        for lane in back.lanes_of(&road.id) {
            for point in lane.left_boundary.to_polyline(config).unwrap().points() {
                assert!(
                    (point.z - (10.0 + point.x / 80.0 * 2.0) - KERB).abs() < 1e-4,
                    "{} came back at {}",
                    lane.id,
                    point.z
                );
            }
        }
    }
}

#[test]
fn a_lift_that_peaks_between_the_road_s_vertices_keeps_its_peak() {
    // A straight road is two vertices; a lift that rises to a knot halfway and falls
    // again needs a vertex there, or the edge runs straight past the peak.
    let peaked = scenarios::lane(2.0, Direction::Forward)
        .with_type(LaneType::Sidewalk)
        .with_height(
            LaneHeight::new([(0.0, 0.1, 0.1), (40.0, 0.3, 0.4), (80.0, 0.1, 0.1)]).unwrap(),
        );
    let mut builder = MapBuilder::new(scenarios::metadata("peak"));
    builder
        .add_road(
            RoadSpec::line(
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(80.0, 0.0, 0.0),
                vec![scenarios::lane(3.5, Direction::Forward), peaked],
            )
            .unwrap(),
        )
        .unwrap();
    let map = builder.finish().unwrap().validate().unwrap();
    let pavement = map
        .lanes
        .iter()
        .find(|lane| lane.lane_type == LaneType::Sidewalk)
        .unwrap();
    let outer = match pavement.side {
        LateralSide::Left => &pavement.left_boundary,
        LateralSide::Right => &pavement.right_boundary,
    };
    let top = outer
        .to_polyline(map.metadata.sampling)
        .unwrap()
        .points()
        .iter()
        .map(|point| point.z)
        .fold(f64::MIN, f64::max);
    assert!(
        (top - 0.4).abs() < 1e-9,
        "the outer edge peaks at {top}, not 0.4"
    );
    // And the document says the same.
    assert!(largest_edge_disagreement(&map) < 1e-3);
}

#[test]
fn a_lift_that_slopes_across_a_banked_approach_meets_its_connector() {
    // Rolled 3° and lifting its outer edge 0.1 m more than its inner one, an
    // approach lane hands its connector a lift that varies across the lane. The
    // connector has to lift its ends along the approach's tilted normal, or they
    // miss the approach's edges sideways by the tilt.
    let roll = 3.0_f64.to_radians();
    let lanes = || {
        vec![
            scenarios::lane(3.5, Direction::Backward),
            scenarios::lane(3.5, Direction::Forward)
                .with_height(LaneHeight::constant(0.0, 0.1).unwrap()),
        ]
    };
    let mut builder = MapBuilder::new(scenarios::metadata("banked turn"));
    let junction = builder.add_junction(Some("x"));
    let west = builder
        .add_road(
            RoadSpec::line(
                Point3::new(-60.0, 0.0, 0.0),
                Point3::new(-12.0, 0.0, 1.0),
                lanes(),
            )
            .unwrap()
            .with_superelevation(Poly3Profile::constant(roll)),
        )
        .unwrap();
    let north = builder
        .add_road(
            RoadSpec::line(
                Point3::new(0.0, 12.0, 1.0),
                Point3::new(0.0, 60.0, 2.0),
                lanes(),
            )
            .unwrap()
            .with_superelevation(Poly3Profile::constant(-roll)),
        )
        .unwrap();
    builder.connect_via(&junction, &west, &north).unwrap();
    // Validation compares the boundaries where the connector meets each approach,
    // to a millimetre; lifted along its own upright normal the connector misses
    // them by almost three.
    builder
        .finish()
        .unwrap()
        .validate()
        .unwrap_or_else(|e| panic!("{e}"));
}

#[test]
fn a_cross_section_that_is_not_one_plane_keeps_its_heights_from_lanelet2_to_opendrive() {
    // Two lanes side by side, the outer one falling away 0.1 m more than a single
    // tilt would put it: Lanelet2 draws the boundaries where they are and shares
    // the line between the lanes. Read back, the two are one road with one tilt,
    // and each lane's height has to make up the rest — or OpenDRIVE flattens the
    // crown onto a plane and moves an edge by centimetres.
    let mut builder = MapBuilder::new(scenarios::metadata("crown"));
    builder
        .add_road(
            RoadSpec::line(
                Point3::new(0.0, 0.0, 10.0),
                Point3::new(80.0, 0.0, 11.0),
                vec![
                    scenarios::lane(3.5, Direction::Forward),
                    scenarios::lane(3.5, Direction::Forward)
                        .with_height(LaneHeight::constant(0.0, -0.1).unwrap()),
                ],
            )
            .unwrap(),
        )
        .unwrap();
    let map = builder.finish().unwrap().validate().unwrap();
    let read = roadgen_lanelet2::from_osm_str(
        &roadgen_lanelet2::to_osm_xml(&map).unwrap(),
        &roadgen_lanelet2::ReadOptions {
            handedness: map.metadata.handedness,
            origin: Some(map.metadata.origin),
            sampling: map.metadata.sampling,
        },
    )
    .unwrap()
    .map
    .validate()
    .unwrap();
    assert_eq!(read.roads.len(), 1, "the lanes share their middle line");
    assert!(
        read.lanes.iter().any(|lane| !lane.height.is_flat()),
        "one tilt cannot hold the crown, so some lane is lifted"
    );
    let worst = largest_edge_distance(&read);
    assert!(worst < 0.01, "OpenDRIVE puts an edge {worst} m off");
}

/// Every lane edge the independent evaluator finds in the document, against the
/// IR's boundaries as curves rather than vertex for vertex: a map read from a file
/// keeps the file's vertices, which need not stand at the reference line's stations.
fn largest_edge_distance(map: &ValidatedMap) -> f64 {
    let document = reparse_opendrive(map);
    let config = map.metadata.sampling;
    let distance = |polyline: &[Point3], point: Position| {
        let point = Point3::new(point.x, point.y, point.z);
        polyline
            .windows(2)
            .map(|pair| {
                let along = pair[1] - pair[0];
                let u = ((point - pair[0]).dot(along) / along.dot(along)).clamp(0.0, 1.0);
                pair[0].lerp(pair[1], u).distance_to(point)
            })
            .fold(f64::INFINITY, f64::min)
    };
    let mut worst: f64 = 0.0;
    for (index, road) in map.roads.iter().enumerate() {
        let evaluator = RoadEvaluator::find(&document, &index.to_string()).unwrap();
        for lane in map.lanes_of(&road.id) {
            let (inner, outer) = match lane.side {
                LateralSide::Left => (&lane.right_boundary, &lane.left_boundary),
                LateralSide::Right => (&lane.left_boundary, &lane.right_boundary),
            };
            let inner = inner.to_polyline(config).unwrap();
            let outer = outer.to_polyline(config).unwrap();
            let (start, end) = lane.station_range;
            let steps = ((end - start) / 0.5).ceil() as usize;
            for step in 0..=steps {
                let station = (start + (end - start) * step as f64 / steps as f64).min(end - 1e-9);
                let (found_inner, found_outer) = evaluator
                    .lane_edges(opendrive_lane_id(lane), station)
                    .unwrap();
                worst = worst
                    .max(distance(inner.points(), found_inner))
                    .max(distance(outer.points(), found_outer));
            }
        }
    }
    worst
}
