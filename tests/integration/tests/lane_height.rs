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
