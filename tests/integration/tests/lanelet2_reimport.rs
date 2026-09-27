//! Out and back through Lanelet2: every scenario written as a Lanelet2 map and read
//! again. Lanelet2 keeps lanes and not roads, so what has to survive is the lanes —
//! every boundary where it was — and the movements between them; the roads and
//! junctions come back reconstructed, and the map has to validate.

use roadgen_core::map::Map;
use roadgen_core::prelude::*;
use roadgen_integration_tests::scenarios;
use roadgen_lanelet2::{from_osm_str, to_osm_xml, ReadOptions};

/// How far a boundary vertex may move on the way out and back, metres. The file
/// carries each node's latitude and longitude to eleven decimal places, which is
/// finer than this by far.
const TOLERANCE: f64 = 1e-4;

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

fn options_for(map: &ValidatedMap) -> ReadOptions {
    ReadOptions {
        handedness: map.metadata.handedness,
        origin: Some(map.metadata.origin),
        sampling: map.metadata.sampling,
    }
}

/// A lane's ends as traffic sees them: where its left and right boundaries start
/// and stop.
fn ends(map: &Map, lane: &LaneId) -> [Point3; 4] {
    let lane = map.lanes.get(lane).expect("a lane of the map");
    let travel = lane
        .travel_geometry(map.metadata.sampling)
        .expect("a lane with geometry");
    [
        travel.left.start_point(),
        travel.left.end_point(),
        travel.right.start_point(),
        travel.right.end_point(),
    ]
}

fn same(a: &[Point3; 4], b: &[Point3; 4]) -> bool {
    a.iter().zip(b).all(|(p, q)| p.distance_to(*q) <= TOLERANCE)
}

/// The lanes a Lanelet2 export writes: the ones a lanelet is made for.
fn written(map: &Map) -> Vec<LaneId> {
    map.lanes
        .iter()
        .filter(|lane| {
            matches!(
                lane.lane_type,
                LaneType::Driving | LaneType::Biking | LaneType::Sidewalk
            )
        })
        .map(|lane| lane.id.clone())
        .collect()
}

/// The original lane each read lane is, found by where its boundaries end.
fn correspondence(original: &Map, read: &Map) -> Vec<(LaneId, LaneId)> {
    let candidates: Vec<(LaneId, [Point3; 4])> = written(original)
        .into_iter()
        .map(|id| {
            let ends = ends(original, &id);
            (id, ends)
        })
        .collect();
    read.lanes
        .iter()
        .map(|lane| {
            let here = ends(read, &lane.id);
            let (id, _) = candidates
                .iter()
                .find(|(_, there)| same(&here, there))
                .unwrap_or_else(|| {
                    panic!("read lane {} matches no lane that was written", lane.id)
                });
            (lane.id.clone(), id.clone())
        })
        .collect()
}

#[test]
fn every_scenario_comes_back_as_a_valid_map_with_the_same_lanes() {
    for (name, map) in every_scenario() {
        let xml = to_osm_xml(&map).unwrap_or_else(|e| panic!("{name}: export failed: {e}"));
        let imported =
            from_osm_str(&xml, &options_for(&map)).unwrap_or_else(|e| panic!("{name}: {e}"));
        let read = imported
            .map
            .validate()
            .unwrap_or_else(|e| panic!("{name}: the map read back does not validate: {e}"));

        assert_eq!(
            read.lanes.len(),
            written(&map).len(),
            "{name}: one lane per lanelet"
        );
        let pairs = correspondence(&map, &read);
        let mut originals: Vec<&LaneId> = pairs.iter().map(|(_, original)| original).collect();
        originals.sort();
        originals.dedup();
        assert_eq!(
            originals.len(),
            pairs.len(),
            "{name}: two lanes read as one"
        );
    }
}

#[test]
fn every_movement_between_written_lanes_comes_back() {
    for (name, map) in every_scenario() {
        let xml = to_osm_xml(&map).unwrap();
        let read = from_osm_str(&xml, &options_for(&map))
            .unwrap()
            .map
            .validate()
            .unwrap();
        let original_of: std::collections::HashMap<LaneId, LaneId> =
            correspondence(&map, &read).into_iter().collect();
        let kept = written(&map);

        let mut expected: Vec<(LaneId, LaneId)> = map
            .connections
            .iter()
            .filter(|c| kept.contains(&c.from.lane) && kept.contains(&c.to.lane))
            .map(|c| (c.from.lane.clone(), c.to.lane.clone()))
            .collect();
        let mut found: Vec<(LaneId, LaneId)> = read
            .connections
            .iter()
            .map(|c| {
                (
                    original_of[&c.from.lane].clone(),
                    original_of[&c.to.lane].clone(),
                )
            })
            .collect();
        expected.sort();
        found.sort();
        assert_eq!(found, expected, "{name}: the movements differ");
    }
}

/// Autoware marks the lanes vehicles take through an intersection with
/// `turn_direction`; the exporter does not write it, so it is added here to the
/// lanelets that came from vehicle junction connectors, the way an Autoware map
/// would carry it. A footway round a corner carries none.
fn with_turn_directions(map: &ValidatedMap) -> String {
    use ll2_io::osm::{parse, MemberType, WriteParams};

    let xml = to_osm_xml(map).unwrap();
    let (mut document, errors) = parse(&xml).unwrap();
    assert!(errors.is_empty());
    let connector_ends: Vec<(Point3, Point3)> = map
        .lanes
        .iter()
        .filter(|lane| is_vehicle_connector(map, lane))
        .map(|lane| {
            let travel = lane.travel_geometry(map.metadata.sampling).unwrap();
            (
                travel.centerline.start_point(),
                travel.centerline.end_point(),
            )
        })
        .collect();
    let local = |node: i64| {
        let tags = &document.nodes[&node].tags;
        let number = |key: &str| tags[key].parse::<f64>().unwrap();
        (number("local_x"), number("local_y"))
    };
    let mut turning = Vec::new();
    for relation in document.relations.values() {
        let Some(centerline) = relation
            .members
            .iter()
            .find(|m| m.kind == MemberType::Way && m.role == "centerline")
        else {
            continue;
        };
        let nodes = &document.ways[&centerline.reference].nodes;
        let (first, last) = (local(nodes[0]), local(*nodes.last().unwrap()));
        let near = |point: Point3, (x, y): (f64, f64)| (point.x - x).hypot(point.y - y) < 1e-3;
        if connector_ends
            .iter()
            .any(|(start, end)| near(*start, first) && near(*end, last))
        {
            turning.push(relation.id);
        }
    }
    assert!(!turning.is_empty());
    for id in turning {
        document
            .relations
            .get_mut(&id)
            .unwrap()
            .tags
            .insert("turn_direction".into(), "straight".into());
    }
    document.to_xml(WriteParams {
        josm_upload: false,
        josm_format_elevation: false,
    })
}

fn is_vehicle_connector(map: &Map, lane: &roadgen_core::map::Lane) -> bool {
    lane.lane_type == LaneType::Driving
        && map.road(&lane.road).is_some_and(|road| road.is_connector())
}

#[test]
fn turn_direction_lanelets_come_back_as_one_junction() {
    for (name, map) in [
        ("crossroads", scenarios::crossroads()),
        ("controlled", scenarios::controlled_crossroads()),
    ] {
        let xml = with_turn_directions(&map);
        let read = from_osm_str(&xml, &options_for(&map))
            .unwrap()
            .map
            .validate()
            .unwrap_or_else(|e| panic!("{name}: {e}"));

        assert_eq!(
            read.junctions.len(),
            1,
            "{name}: one intersection, one junction"
        );
        let junction = read.junctions.iter().next().unwrap();
        let connectors = map
            .lanes
            .iter()
            .filter(|lane| is_vehicle_connector(&map, lane))
            .count();
        assert_eq!(junction.connecting_roads.len(), connectors, "{name}");
        // Every carriageway runs into the junction at the end nearer the middle, and
        // links to it there.
        let carriageways = read.roads.iter().filter(|road| {
            !road.is_connector()
                && read
                    .lanes_of(&road.id)
                    .iter()
                    .all(|lane| lane.lane_type == LaneType::Driving)
        });
        for road in carriageways {
            let linked = [RoadEnd::Start, RoadEnd::End].iter().any(|end| {
                road.link.at(*end)
                    == Some(&roadgen_core::topology::RoadLinkTarget::Junction(
                        junction.id.clone(),
                    ))
            });
            assert!(linked, "{name}: {} does not link to the junction", road.id);
        }
        for connection in read.connections.iter() {
            let vehicles = [&connection.from.lane, &connection.to.lane]
                .iter()
                .all(|lane| read.lanes.get(lane).unwrap().lane_type == LaneType::Driving);
            if vehicles {
                assert_eq!(connection.junction.as_ref(), Some(&junction.id), "{name}");
            }
        }
    }
}

#[test]
fn the_rules_come_back_over_the_same_lanes() {
    let map = scenarios::controlled_crossroads();
    let xml = to_osm_xml(&map).unwrap();
    let read = from_osm_str(&xml, &options_for(&map))
        .unwrap()
        .map
        .validate()
        .unwrap();
    let original_of: std::collections::HashMap<LaneId, LaneId> =
        correspondence(&map, &read).into_iter().collect();

    let signature = |map: &Map, rename: &dyn Fn(&LaneId) -> LaneId| {
        let mut rules: Vec<(String, Vec<LaneId>)> = map
            .rules
            .iter()
            .map(|rule| {
                let mut lanes: Vec<LaneId> = rule.lanes().iter().map(rename).collect();
                lanes.sort();
                (rule.kind_str().to_owned(), lanes)
            })
            .collect();
        rules.sort();
        rules
    };
    assert!(!map.rules.is_empty());
    assert_eq!(
        signature(&read, &|lane| original_of[lane].clone()),
        signature(&map, &|lane| lane.clone())
    );
}

/// How far a lane edge of a map read from Lanelet2 may move, in 3D, on its way
/// through OpenDRIVE, metres. The reference line is a chain of cubics through the
/// boundary's vertices, the widths are straight runs between stations abeam every
/// vertex and the cross-fall is one tilt per station, so what is left is the cubics
/// bowing between vertices and the straight runs cutting the bow.
const OPENDRIVE_TOLERANCE: f64 = 0.02;

fn distance_to(polyline: &[Point3], point: Point3) -> f64 {
    polyline
        .windows(2)
        .map(|pair| {
            let (a, b) = (pair[0], pair[1]);
            let (dx, dy, dz) = (b.x - a.x, b.y - a.y, b.z - a.z);
            let length_squared = dx * dx + dy * dy + dz * dz;
            let u = if length_squared > 0.0 {
                (((point.x - a.x) * dx + (point.y - a.y) * dy + (point.z - a.z) * dz)
                    / length_squared)
                    .clamp(0.0, 1.0)
            } else {
                0.0
            };
            a.lerp(b, u).distance_to(point)
        })
        .fold(f64::INFINITY, f64::min)
}

#[test]
fn a_map_read_from_lanelet2_keeps_its_lanes_through_opendrive() {
    for (name, map) in every_scenario() {
        let read = from_osm_str(&to_osm_xml(&map).unwrap(), &options_for(&map))
            .unwrap()
            .map
            .validate()
            .unwrap();
        let config = read.metadata.sampling;
        let xml = roadgen_opendrive::to_xml(&read).unwrap();
        let back = roadgen_opendrive::from_xml(&xml).unwrap().map;
        let back = back.as_map();
        for (index, road) in read.roads.iter().enumerate() {
            let twins = back.lanes_of(&RoadId::new(index.to_string()));
            for lane in read.lanes_of(&road.id) {
                let twin = twins
                    .iter()
                    .find(|other| other.side == lane.side && other.ordinal == lane.ordinal)
                    .unwrap_or_else(|| panic!("{name}: {} has no twin", lane.id));
                for (mine, theirs) in [
                    (&lane.left_boundary, &twin.left_boundary),
                    (&lane.right_boundary, &twin.right_boundary),
                ] {
                    let theirs = theirs.to_polyline(config).unwrap();
                    let mine = mine.to_polyline(config).unwrap();
                    // Both ends where they were, not merely somewhere on the other
                    // boundary: a lane that ends early or late is on the line.
                    for (a, b) in [(mine.first(), theirs.first()), (mine.last(), theirs.last())] {
                        let drift = a.distance_to(b);
                        assert!(
                            drift <= OPENDRIVE_TOLERANCE,
                            "{name}: an end of {} moves {drift:.3} m through OpenDRIVE",
                            lane.id
                        );
                    }
                    // The document's boundary against the file's too: a boundary
                    // that loops out and back between the file's vertices passes
                    // near every one of them.
                    for point in theirs.points() {
                        let drift = distance_to(mine.points(), *point);
                        assert!(
                            drift <= OPENDRIVE_TOLERANCE,
                            "{name}: {} strays {drift:.3} m in OpenDRIVE",
                            lane.id
                        );
                    }
                    for point in mine.points() {
                        let drift = distance_to(theirs.points(), *point);
                        assert!(
                            drift <= OPENDRIVE_TOLERANCE,
                            "{name}: {} moves {drift:.3} m through OpenDRIVE",
                            lane.id
                        );
                    }
                }
            }
        }
    }
}
