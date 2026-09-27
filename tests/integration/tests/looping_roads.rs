//! A road whose two ends run into one junction, cut in two for CARLA.
//!
//! CARLA looks for a lane's continuation at a road's end among the junction's
//! connecting roads whose predecessor *or successor* is the road, without asking at
//! which end. When both ends of a road run into one junction, a connecting road
//! that enters it is taken for one that leaves it, and the movement is dropped. The
//! writer cuts such a road in two, so neither half has both ends there; nothing may
//! move in the process.

use opendrive::core::OpenDrive;
use opendrive::road::element_type::ElementType;
use roadgen_core::geometry::LaneHeight;
use roadgen_core::map::Lane;
use roadgen_core::prelude::*;
use roadgen_core::topology::{LateralSide, RoadLinkTarget};
use roadgen_integration_tests::opendrive_eval::RoadEvaluator;
use roadgen_integration_tests::{reparse_opendrive, scenarios};

/// A street, a short link and another street in a row, the link between two
/// turns of one junction at either end. The link climbs, is banked, and its kerb
/// side lane rises along it, so every profile has something to carry over the cut.
fn link_in_a_junction() -> ValidatedMap {
    let mut builder = MapBuilder::new(scenarios::metadata("link inside a junction"));
    let junction = builder.add_junction(Some("x"));
    let lanes = || {
        vec![
            scenarios::lane(3.5, Direction::Forward),
            scenarios::lane(3.0, Direction::Forward)
                .with_height(LaneHeight::new([(0.0, 0.0, 0.1), (20.0, 0.1, 0.3)]).unwrap()),
        ]
    };
    let mut road = |name: &str, from: (f64, f64, f64), to: (f64, f64, f64), roll: f64| {
        builder
            .add_road(
                RoadSpec::line(
                    Point3::new(from.0, from.1, from.2),
                    Point3::new(to.0, to.1, to.2),
                    lanes(),
                )
                .unwrap()
                .with_superelevation(Poly3Profile::constant(roll))
                .with_name(name),
            )
            .unwrap()
    };
    let west = road("west", (-80.0, 0.0, 0.0), (-30.0, 0.0, 0.0), 0.0);
    let link = road("link", (-10.0, 0.0, 0.0), (10.0, 0.0, 1.0), 0.03);
    let east = road("east", (30.0, 0.0, 1.0), (80.0, 0.0, 1.0), 0.0);
    builder.connect_via(&junction, &west, &link).unwrap();
    builder.connect_via(&junction, &link, &east).unwrap();
    builder.finish().unwrap().validate().unwrap()
}

fn road<'a>(document: &'a OpenDrive, id: &str) -> &'a opendrive::road::Road {
    document.road.iter().find(|road| road.id == id).unwrap()
}

fn successor(document: &OpenDrive, id: &str) -> (Option<ElementType>, String) {
    road(document, id)
        .link
        .as_ref()
        .and_then(|link| link.successor.as_ref())
        .map_or((None, String::new()), |end| {
            (end.element_type.clone(), end.element_id.clone())
        })
}

fn lane_number(lane: &Lane) -> i64 {
    roadgen_opendrive::lane_number(lane.side, lane.ordinal)
}

#[test]
fn a_road_with_both_ends_in_one_junction_is_cut_where_nothing_moves() {
    let map = link_in_a_junction();
    let link = map
        .roads
        .iter()
        .find(|road| road.name.as_deref() == Some("link"))
        .unwrap();
    assert!(
        matches!(
            (&link.link.predecessor, &link.link.successor),
            (Some(RoadLinkTarget::Junction(a)), Some(RoadLinkTarget::Junction(b))) if a == b
        ),
        "the IR has the link's two ends in one junction"
    );
    let document = reparse_opendrive(&map);

    // No ordinary road of the document has both ends in one junction, and no
    // connecting road leads into the road whose end CARLA asks about.
    for written in document.road.iter().filter(|road| road.junction == "-1") {
        let link = written.link.as_ref();
        let ends: Vec<_> = [
            link.and_then(|link| link.predecessor.as_ref()),
            link.and_then(|link| link.successor.as_ref()),
        ]
        .into_iter()
        .flatten()
        .filter(|end| end.element_type == Some(ElementType::Junction))
        .map(|end| end.element_id.clone())
        .collect();
        assert!(
            !(ends.len() == 2 && ends[0] == ends[1]),
            "road {} still has both ends in junction {}",
            written.id,
            ends[0]
        );
        let (kind, junction) = successor(&document, &written.id);
        if kind != Some(ElementType::Junction) {
            continue;
        }
        let entering = document
            .road
            .iter()
            .filter(|other| other.junction == junction)
            .filter(|other| {
                other
                    .link
                    .as_ref()
                    .and_then(|link| link.successor.as_ref())
                    .is_some_and(|end| end.element_id == written.id)
            })
            .count();
        assert_eq!(
            entering, 0,
            "CARLA would loop at the end of road {}",
            written.id
        );
    }

    // The link's first half keeps its id; the second follows it road to road, and
    // the two are as long as the link.
    let index = map
        .roads
        .iter()
        .position(|road| road.id == link.id)
        .unwrap();
    let first = index.to_string();
    let (kind, second) = successor(&document, &first);
    assert_eq!(kind, Some(ElementType::Road));
    let (a, b) = (
        RoadEvaluator::find(&document, &first).unwrap(),
        RoadEvaluator::find(&document, &second).unwrap(),
    );
    let length = link.horizontal_length().unwrap();
    assert!((a.length() + b.length() - length).abs() < 1e-9);

    // Every lane's edges run on across the cut, and along the second half are the
    // IR's, profiles, bank and lift included.
    let config = map.metadata.sampling;
    for lane in map.lanes_of(&link.id) {
        let id = lane_number(lane);
        let (end_inner, end_outer) = a.lane_edges(id, a.length()).unwrap();
        let (start_inner, start_outer) = b.lane_edges(id, 0.0).unwrap();
        assert!(end_inner.distance_to(start_inner) < 1e-9);
        assert!(end_outer.distance_to(start_outer) < 1e-9);
        let (inner, outer) = match lane.side {
            LateralSide::Left => (&lane.right_boundary, &lane.left_boundary),
            LateralSide::Right => (&lane.left_boundary, &lane.right_boundary),
        };
        for (edge, which) in [(inner, 0), (outer, 1)] {
            let polyline = edge.to_polyline(config).unwrap();
            for step in 0..=20 {
                let s = b.length() * step as f64 / 20.0;
                let edges = b.lane_edges(id, s).unwrap();
                let point = if which == 0 { edges.0 } else { edges.1 };
                let point = Point3::new(point.x, point.y, point.z);
                let gap = polyline
                    .points()
                    .windows(2)
                    .map(|pair| {
                        let along = pair[1] - pair[0];
                        let u = ((point - pair[0]).dot(along) / along.dot(along)).clamp(0.0, 1.0);
                        pair[0].lerp(pair[1], u).distance_to(point)
                    })
                    .fold(f64::INFINITY, f64::min);
                assert!(
                    gap < 1e-6,
                    "lane {id} is {gap} m off at s = {s} of the second half"
                );
            }
        }
    }

    // And the movements: the west street's connecting road leads into the first
    // half, and the second half's into the east street.
    let east = map
        .roads
        .iter()
        .position(|road| road.name.as_deref() == Some("east"))
        .unwrap()
        .to_string();
    let (kind, junction) = successor(&document, &second);
    assert_eq!(kind, Some(ElementType::Junction));
    let junction = document.junction.iter().find(|j| j.id == junction).unwrap();
    assert!(junction.connection.iter().any(|connection| {
        connection.incoming_road.as_deref() == Some(second.as_str())
            && successor(&document, connection.connecting_road.as_ref().unwrap()).1 == east
    }));
}
