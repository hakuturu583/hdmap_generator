//! Out and back: every scenario written as OpenDRIVE, read again, and written a
//! second time. The map that comes back has to be the map that went out — the
//! same roads, lanes, movements, furniture and rules — and the second document has
//! to put every lane edge where the first did, which is asked of the two documents
//! by the independent evaluator rather than of the IR by itself.

use std::collections::BTreeSet;

use roadgen_core::map::{Lane, Map};
use roadgen_core::prelude::*;
use roadgen_core::topology::RoadLinkTarget;
use roadgen_integration_tests::opendrive_eval::RoadEvaluator;
use roadgen_integration_tests::scenarios;
use roadgen_opendrive::{from_xml, to_xml};

/// How far a lane edge may move on the way out and back, metres.
///
/// The one place the reading approximates a scenario is the height along a junction
/// connector: the exporter writes its elevation straight between vertices and the
/// reader cannot cut a cubic, so the connector climbs in one straight grade. On
/// these maps that is a difference of nothing; the bound leaves room for rounding.
const TOLERANCE: f64 = 1e-3;

/// The note every scenario gets, because a PROJ string has no altitude in it.
const ALTITUDE_NOTE: &str =
    "a PROJ string carries no altitude, so the origin is read at 0 m; the map's own \
     heights are unaffected";

fn every_scenario() -> Vec<(&'static str, ValidatedMap)> {
    vec![
        ("straight", scenarios::straight_road()),
        ("bidirectional", scenarios::bidirectional_road()),
        ("multi_lane", scenarios::multi_lane_road()),
        ("two_roads_joined", scenarios::two_roads_joined()),
        ("two_roads_in_line", scenarios::two_roads_in_line()),
        ("split", scenarios::split()),
        ("merge", scenarios::merge()),
        ("crossroads", scenarios::crossroads()),
        ("graded", scenarios::graded_road()),
        ("bent_polyline", scenarios::bent_polyline()),
        ("spiral", scenarios::spiral_transition_road()),
        ("banked", scenarios::banked_curve()),
        ("lane_drop", scenarios::lane_drop()),
        ("widening", scenarios::widening_road()),
        ("controlled", scenarios::controlled_crossroads()),
    ]
}

/// The OpenDRIVE id of a lane, as the exporter numbers it.
fn od_lane(lane: &Lane) -> i64 {
    match lane.side {
        LateralSide::Left => lane.ordinal as i64,
        LateralSide::Right => -(lane.ordinal as i64),
    }
}

/// Everything about a map that has to survive the round trip, in a form that does
/// not depend on the IR's own identifiers: roads by position, lanes by their
/// OpenDRIVE number, junctions by position.
fn signature(map: &Map) -> BTreeSet<String> {
    let road_index = |id: &RoadId| {
        map.roads
            .iter()
            .position(|road| &road.id == id)
            .expect("a road of the map")
    };
    let junction_index = |id: &JunctionId| {
        map.junctions
            .iter()
            .position(|junction| &junction.id == id)
            .expect("a junction of the map")
    };
    let lane_name = |id: &LaneId| {
        let lane = map.lanes.get(id).expect("a lane of the map");
        format!(
            "{}:{}:{}",
            road_index(&lane.road),
            lane.section,
            od_lane(lane)
        )
    };
    let target = |link: Option<&RoadLinkTarget>| match link {
        None => "none".to_owned(),
        Some(RoadLinkTarget::Road(other)) => {
            format!("road {} {}", road_index(&other.road), other.end.as_str())
        }
        Some(RoadLinkTarget::Junction(junction)) => {
            format!("junction {}", junction_index(junction))
        }
    };

    let mut lines = BTreeSet::new();
    for (index, road) in map.roads.iter().enumerate() {
        lines.insert(format!(
            "road {index} name={:?} junction={:?} type={} speed={:?} sections={:?} \
             predecessor=({}) successor=({})",
            road.name,
            road.junction.as_ref().map(junction_index),
            road.road_type.as_str(),
            road.speed_limit.map(|limit| (limit.mps() * 1000.0).round()),
            road.sections
                .iter()
                .map(|section| (section.station * 1000.0).round())
                .collect::<Vec<_>>(),
            target(road.link.predecessor.as_ref()),
            target(road.link.successor.as_ref()),
        ));
        // OpenDRIVE paints one line between two lanes where the IR lets each lane
        // say its own, so what has to survive is each lane's outer marking and the
        // one centre line the exporter picks — the first right lane's inner, else
        // the last left lane's.
        for (section, entry) in road.sections.iter().enumerate() {
            let lanes = map.lanes_of_section(&road.id, section);
            let centre = lanes
                .iter()
                .find(|lane| lane.side == LateralSide::Right && lane.ordinal == 1)
                .map(|lane| lane.left_marking)
                .or_else(|| {
                    lanes
                        .iter()
                        .filter(|lane| lane.side == LateralSide::Left)
                        .max_by_key(|lane| lane.ordinal)
                        .map(|lane| lane.right_marking)
                });
            lines.insert(format!(
                "centre {index}:{section} at {} {centre:?}",
                (entry.station * 1000.0).round()
            ));
        }
        for lane in map.lanes_of(&road.id) {
            let outer = match lane.side {
                LateralSide::Left => lane.left_marking,
                LateralSide::Right => lane.right_marking,
            };
            lines.insert(format!(
                "lane {} type={} direction={} outer={:?}/{:?} speed={:?} \
                 width={:?} taper={:?}",
                lane_name(&lane.id),
                lane.lane_type.as_str(),
                lane.direction.as_str(),
                outer.marking,
                outer.color,
                lane.speed_limit.map(|limit| (limit.mps() * 1000.0).round()),
                // A profile whose knots all agree is a constant width however many
                // knots it was written with.
                if lane.width.is_constant() {
                    vec![(0.0, (lane.width.narrowest().metres() * 1000.0).round())]
                } else {
                    lane.width
                        .knots()
                        .iter()
                        .map(|(station, width)| {
                            (
                                (station * 1000.0).round(),
                                (width.metres() * 1000.0).round(),
                            )
                        })
                        .collect::<Vec<_>>()
                },
                if lane.width.is_constant() {
                    Taper::Linear
                } else {
                    lane.width.taper()
                },
            ));
        }
    }
    for (index, junction) in map.junctions.iter().enumerate() {
        let mut arms: Vec<usize> = junction.incoming_roads.iter().map(road_index).collect();
        arms.sort_unstable();
        let mut connectors: Vec<usize> = junction.connecting_roads.iter().map(road_index).collect();
        connectors.sort_unstable();
        lines.insert(format!(
            "junction {index} name={:?} arms={arms:?} connectors={connectors:?}",
            junction.name
        ));
    }
    for connection in map.connections.iter() {
        lines.insert(format!(
            "connection {} {} -> {} {} junction={:?}",
            lane_name(&connection.from.lane),
            connection.from.end.as_str(),
            lane_name(&connection.to.lane),
            connection.to.end.as_str(),
            connection.junction.as_ref().map(junction_index),
        ));
    }
    for object in map.objects.iter() {
        let geometry = match &object.geometry {
            ObjectGeometry::Point(_) => "point",
            ObjectGeometry::Line(_) => "line",
            ObjectGeometry::Band { .. } => "band",
        };
        lines.insert(format!(
            "object {} {} {geometry} lanes={:?}",
            object.id,
            object.kind.as_str(),
            object.lanes.iter().map(lane_name).collect::<Vec<_>>()
        ));
    }
    for rule in &map.rules {
        let mut lanes: Vec<String> = rule.lanes().iter().map(lane_name).collect();
        lanes.sort();
        let detail = match rule {
            TrafficRule::TrafficLight {
                lights, stop_line, ..
            } => format!("lights={lights:?} stop_line={stop_line:?}"),
            TrafficRule::RightOfWay { stop_line, .. } => format!("stop_line={stop_line:?}"),
            TrafficRule::SpeedLimit { limit, .. } => format!("limit={}", limit.mps()),
        };
        lines.insert(format!("rule {} {detail} lanes={lanes:?}", rule.kind_str()));
    }
    lines
}

/// The largest distance any lane edge moves between two documents, at every metre
/// of every road, measured by the independent evaluator.
fn largest_edge_disagreement(first: &str, second: &str) -> f64 {
    let before = opendrive::core::OpenDrive::from_xml_str(first).unwrap();
    let after = opendrive::core::OpenDrive::from_xml_str(second).unwrap();
    assert_eq!(before.road.len(), after.road.len());
    let mut worst: f64 = 0.0;
    for road in &before.road {
        let was = RoadEvaluator::new(road);
        let is = RoadEvaluator::find(&after, &road.id).expect("the same road ids");
        assert!(
            (was.length() - is.length()).abs() < 1e-6,
            "road {}",
            road.id
        );
        let length = was.length();
        let mut stations: Vec<f64> = (0..)
            .map(|n| n as f64)
            .take_while(|s| *s < length)
            .collect();
        stations.push(length - 1e-6);
        for section in road.lanes.lane_section.iter() {
            let ids = section
                .left
                .iter()
                .flat_map(|left| left.lane.iter().map(|lane| lane.id))
                .chain(
                    section
                        .right
                        .iter()
                        .flat_map(|right| right.lane.iter().map(|lane| lane.id)),
                );
            for id in ids {
                for &s in &stations {
                    let (Some(a), Some(b)) = (was.lane_edges(id, s), is.lane_edges(id, s)) else {
                        continue;
                    };
                    worst = worst.max(a.0.distance_to(b.0)).max(a.1.distance_to(b.1));
                }
            }
        }
    }
    worst
}

#[test]
fn every_scenario_comes_back_as_itself() {
    for (name, map) in every_scenario() {
        let first = to_xml(&map).unwrap();
        let imported = from_xml(&first).unwrap_or_else(|error| panic!("{name}: {error}"));
        assert_eq!(
            imported.approximations,
            vec![ALTITUDE_NOTE.to_owned()],
            "{name}"
        );
        let reread = imported
            .map
            .validate()
            .unwrap_or_else(|error| panic!("{name}: {error}"));

        assert_eq!(reread.metadata.name, map.metadata.name, "{name}");
        assert_eq!(
            reread.metadata.handedness, map.metadata.handedness,
            "{name}"
        );
        assert!(
            (reread.metadata.origin.latitude() - map.metadata.origin.latitude()).abs() < 1e-9
                && (reread.metadata.origin.longitude() - map.metadata.origin.longitude()).abs()
                    < 1e-9,
            "{name}"
        );

        let before = signature(map.as_map());
        let after = signature(reread.as_map());
        let missing: Vec<_> = before.difference(&after).collect();
        let extra: Vec<_> = after.difference(&before).collect();
        assert!(
            missing.is_empty() && extra.is_empty(),
            "{name}: lost {missing:#?}, gained {extra:#?}"
        );

        let second = to_xml(&reread).unwrap();
        let worst = largest_edge_disagreement(&first, &second);
        assert!(worst < TOLERANCE, "{name}: a lane edge moved {worst} m");
    }
}

#[test]
fn a_map_of_buildings_comes_back_with_its_town() {
    let mut map = scenarios::crossroads_builder("town", 60.0)
        .finish()
        .unwrap();
    roadgen_buildings::generate(&mut map, &roadgen_buildings::Rules::default()).unwrap();
    let map = map.validate().unwrap();
    assert!(!map.buildings.is_empty());

    let first = to_xml(&map).unwrap();
    let imported = from_xml(&first).unwrap();
    let reread = imported.map.validate().unwrap();
    assert_eq!(reread.buildings.len(), map.buildings.len());
    assert_eq!(reread.building_parts.len(), map.building_parts.len());
    for building in map.buildings.iter() {
        let back = reread
            .building(&building.id)
            .unwrap_or_else(|| panic!("{} should come back under its own name", building.id));
        assert_eq!(back.kind, building.kind);
        assert_eq!(back.parts.len(), building.parts.len());
        for (before, after) in map
            .parts_of(&building.id)
            .iter()
            .zip(reread.parts_of(&building.id))
        {
            // The walls and the roof's rise together, because the outline has one
            // height per corner and no ridge.
            let wanted = before.solid.wall_height + before.solid.roof.height;
            assert!((after.solid.wall_height - wanted).abs() < 1e-6);
            assert_eq!(after.solid.footprint.len(), before.solid.footprint.len());
            for (was, is) in before
                .solid
                .footprint
                .points()
                .iter()
                .zip(after.solid.footprint.points())
            {
                assert!(
                    was.distance_to(*is) < 0.02,
                    "{} moved {}",
                    building.id,
                    was.distance_to(*is)
                );
            }
        }
    }
    assert!(imported
        .approximations
        .iter()
        .any(|note| note.contains("flat roofs")));
}
