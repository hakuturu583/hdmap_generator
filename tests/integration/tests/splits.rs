//! Lanes that split or merge outside a junction, on their way from Lanelet2 to
//! OpenDRIVE.
//!
//! Lanelet2 draws a fork as two lanelets that start on the points one lanelet ends
//! on, and the IR keeps it as it is: lane connections into two roads, no junction.
//! OpenDRIVE links a road end to one road or one junction, so the writer has to put
//! a junction there — and every movement has to survive the trip, on lanes whose
//! edges still meet.

use std::collections::BTreeSet;

use opendrive::core::OpenDrive;
use opendrive::junction::contact_point::ContactPoint;
use opendrive::road::element_type::ElementType;
use roadgen_core::map::{Map, TrafficHandedness};
use roadgen_core::validation::ValidatedMap;
use roadgen_integration_tests::opendrive_eval::RoadEvaluator;
use roadgen_integration_tests::reparse_opendrive;
use roadgen_lanelet2::ReadOptions;

/// Metres to degrees near the origin, which is all a map this small needs.
const LAT0: f64 = 35.0;
const LON0: f64 = 139.0;

struct Osm {
    nodes: Vec<(i64, f64, f64, f64)>,
    ways: Vec<(i64, Vec<i64>)>,
    /// `(id, left way, right way, turn_direction)`.
    lanelets: Vec<(i64, i64, i64, Option<&'static str>)>,
    next: i64,
}

impl Osm {
    fn new() -> Self {
        Osm {
            nodes: Vec::new(),
            ways: Vec::new(),
            lanelets: Vec::new(),
            next: 1,
        }
    }

    fn id(&mut self) -> i64 {
        self.next += 1;
        self.next
    }

    fn node(&mut self, x: f64, y: f64, z: f64) -> i64 {
        let id = self.id();
        self.nodes.push((id, x, y, z));
        id
    }

    fn way(&mut self, nodes: Vec<i64>) -> i64 {
        let id = self.id();
        self.ways.push((id, nodes));
        id
    }

    fn lanelet(&mut self, left: Vec<i64>, right: Vec<i64>) -> i64 {
        let (left, right) = (self.way(left), self.way(right));
        self.lanelet_on(left, right, None)
    }

    /// A lanelet between two ways that already exist, which is how two lanelets
    /// side by side share the line between them.
    fn lanelet_on(&mut self, left: i64, right: i64, turn: Option<&'static str>) -> i64 {
        let id = self.id();
        self.lanelets.push((id, left, right, turn));
        id
    }

    /// A lanelet tagged `turn_direction`: a movement through an intersection.
    fn turn(&mut self, left: Vec<i64>, right: Vec<i64>, direction: &'static str) -> i64 {
        let (left, right) = (self.way(left), self.way(right));
        self.lanelet_on(left, right, Some(direction))
    }

    /// `first`, then new nodes along `points`, all at height zero.
    fn line(&mut self, first: Option<i64>, points: &[(f64, f64)]) -> Vec<i64> {
        first
            .into_iter()
            .chain(points.iter().map(|(x, y)| self.node(*x, *y, 0.0)))
            .collect()
    }

    fn xml(&self) -> String {
        let metres_per_degree = 111_320.0;
        let mut xml = String::from("<?xml version=\"1.0\"?>\n<osm version=\"0.6\">\n");
        for (id, x, y, z) in &self.nodes {
            let lat = LAT0 + y / metres_per_degree;
            let lon = LON0 + x / (metres_per_degree * LAT0.to_radians().cos());
            xml.push_str(&format!(
                "<node id=\"{id}\" lat=\"{lat:.12}\" lon=\"{lon:.12}\"><tag k=\"ele\" v=\"{z}\"/></node>\n"
            ));
        }
        for (id, nodes) in &self.ways {
            xml.push_str(&format!("<way id=\"{id}\">"));
            for node in nodes {
                xml.push_str(&format!("<nd ref=\"{node}\"/>"));
            }
            xml.push_str(
                "<tag k=\"type\" v=\"line_thin\"/><tag k=\"subtype\" v=\"solid\"/></way>\n",
            );
        }
        for (id, left, right, turn) in &self.lanelets {
            let turn = turn
                .map(|direction| format!("<tag k=\"turn_direction\" v=\"{direction}\"/>"))
                .unwrap_or_default();
            xml.push_str(&format!(
                "<relation id=\"{id}\"><member type=\"way\" ref=\"{left}\" role=\"left\"/>\
                 <member type=\"way\" ref=\"{right}\" role=\"right\"/>\
                 <tag k=\"type\" v=\"lanelet\"/><tag k=\"subtype\" v=\"road\"/>{turn}\
                 <tag k=\"one_way\" v=\"yes\"/><tag k=\"speed_limit\" v=\"40\"/></relation>\n"
            ));
        }
        xml.push_str("</osm>\n");
        xml
    }
}

/// A lane that forks in two — one branch straight on, one bearing away and down a
/// banked slip — and, further on, two lanes that merge into one. Traffic keeps to
/// `handedness`, which decides which side of its reference line a stub's lane is on.
fn forks(handedness: TrafficHandedness) -> ValidatedMap {
    let mut osm = Osm::new();
    let half = 1.75;
    // The whole map falls 0.1 m to the right across a lane, so every stub is banked.
    let fall = 0.1;
    // The fork: `approach` ends on two points that `straight` and `slip` both start on.
    let approach_left: Vec<i64> = (0..=3)
        .map(|i| osm.node(i as f64 * 10.0, half, 10.0))
        .collect();
    let approach_right: Vec<i64> = (0..=3)
        .map(|i| osm.node(i as f64 * 10.0, -half, 10.0 - fall))
        .collect();
    let (fork_left, fork_right) = (approach_left[3], approach_right[3]);
    osm.lanelet(approach_left, approach_right);
    let straight_left: Vec<i64> = std::iter::once(fork_left)
        .chain((4..=6).map(|i| osm.node(i as f64 * 10.0, half, 10.0)))
        .collect();
    let straight_right: Vec<i64> = std::iter::once(fork_right)
        .chain((4..=6).map(|i| osm.node(i as f64 * 10.0, -half, 10.0 - fall)))
        .collect();
    osm.lanelet(straight_left, straight_right);
    // The slip bears right and falls, and its right edge falls further: banked.
    let slip = |osm: &mut Osm, y: f64, bank: f64| -> Vec<i64> {
        (1..=3)
            .map(|i| {
                let t = i as f64;
                osm.node(30.0 + t * 10.0, y - t * t * 1.5, 10.0 - t * 0.3 - bank)
            })
            .collect()
    };
    let slip_left: Vec<i64> = std::iter::once(fork_left)
        .chain(slip(&mut osm, half, 0.0))
        .collect();
    let slip_right: Vec<i64> = std::iter::once(fork_right)
        .chain(slip(&mut osm, -half, fall + 0.05))
        .collect();
    osm.lanelet(slip_left, slip_right);

    // The merge, 200 m away: `inner` and `outer` both end on the points `joined`
    // starts on.
    let at = |x: f64| x + 200.0;
    let joined_left: Vec<i64> = (3..=6)
        .map(|i| osm.node(at(i as f64 * 10.0), half, 5.0))
        .collect();
    let joined_right: Vec<i64> = (3..=6)
        .map(|i| osm.node(at(i as f64 * 10.0), -half, 5.0 - fall))
        .collect();
    let (merge_left, merge_right) = (joined_left[0], joined_right[0]);
    osm.lanelet(joined_left, joined_right);
    for bend in [0.0, 1.0] {
        let left: Vec<i64> = (0..3)
            .map(|i| {
                let t = (3 - i) as f64;
                osm.node(at(i as f64 * 10.0), half - bend * t * t, 5.0)
            })
            .chain(std::iter::once(merge_left))
            .collect();
        let right: Vec<i64> = (0..3)
            .map(|i| {
                let t = (3 - i) as f64;
                osm.node(at(i as f64 * 10.0), -half - bend * t * t, 5.0 - fall)
            })
            .chain(std::iter::once(merge_right))
            .collect();
        osm.lanelet(left, right);
    }

    read(&osm, handedness)
}

fn read(osm: &Osm, handedness: TrafficHandedness) -> ValidatedMap {
    let imported = roadgen_lanelet2::from_osm_str(
        &osm.xml(),
        &ReadOptions {
            handedness,
            ..ReadOptions::default()
        },
    )
    .unwrap_or_else(|error| panic!("{error}"));
    imported
        .map
        .validate()
        .unwrap_or_else(|error| panic!("{error}"))
}

/// The document's road id for an IR road: the writer numbers them in map order.
fn document_id(map: &Map, road: &roadgen_core::id::RoadId) -> String {
    map.roads
        .iter()
        .position(|other| &other.id == road)
        .expect("a road of the map")
        .to_string()
}

/// Every movement the document says, by (road, lane) to (road, lane), read by
/// OpenDRIVE's rules: a road end linked to a road carries on by its lanes' links,
/// one linked to a junction by the junction's connections, and a connecting road's
/// lanes carry on by their own.
fn movements(document: &OpenDrive) -> BTreeSet<((String, i64), (String, i64))> {
    let road = |id: &str| document.road.iter().find(|road| road.id == id);
    let mut found = BTreeSet::new();
    for junction in &document.junction {
        for connection in junction.connection.iter() {
            let (Some(incoming), Some(connecting)) =
                (&connection.incoming_road, &connection.connecting_road)
            else {
                continue;
            };
            let connecting_road = road(connecting).expect("a connecting road");
            let successor = connecting_road
                .link
                .as_ref()
                .and_then(|link| link.successor.as_ref())
                .expect("a connecting road leads somewhere");
            assert_eq!(successor.element_type, Some(ElementType::Road));
            for lane_link in &connection.lane_link {
                let lane = connecting_road.lanes.lane_section[0]
                    .left
                    .iter()
                    .flat_map(|left| left.lane.iter().map(|lane| (lane.id, &lane.base)))
                    .chain(
                        connecting_road.lanes.lane_section[0]
                            .right
                            .iter()
                            .flat_map(|right| right.lane.iter().map(|lane| (lane.id, &lane.base))),
                    )
                    .find(|(id, _)| *id == lane_link.to)
                    .map(|(_, lane)| lane)
                    .expect("the connecting road has the linked lane");
                for next in &lane.link.as_ref().expect("a stub lane is linked").successor {
                    found.insert((
                        (incoming.clone(), lane_link.from),
                        (successor.element_id.clone(), next.id),
                    ));
                }
            }
        }
    }
    found
}

fn check(handedness: TrafficHandedness) {
    let map = forks(handedness);
    let problems = roadgen_opendrive::check(&map);
    assert!(
        problems.iter().all(|problem| !problem.contains("junction")),
        "{problems:?}"
    );

    // The IR has a split and a merge outside any junction.
    let direct: Vec<_> = map
        .connections
        .iter()
        .filter(|connection| connection.junction.is_none())
        .collect();
    assert_eq!(
        direct.len(),
        4,
        "two movements at the fork and two at the merge"
    );
    assert!(map.junctions.is_empty());

    let document = reparse_opendrive(&map);
    assert_eq!(
        document.junction.len(),
        2,
        "one junction at the fork, one at the merge"
    );

    // Every movement comes through, and nothing else.
    let expected: BTreeSet<_> = direct
        .iter()
        .map(|connection| {
            let lane = |id| {
                let lane = map.lanes.get(id).unwrap();
                (
                    document_id(&map, &lane.road),
                    roadgen_opendrive::lane_number(lane.side, lane.ordinal),
                )
            };
            (lane(&connection.from.lane), lane(&connection.to.lane))
        })
        .collect();
    assert_eq!(movements(&document), expected);

    // The ordinary roads link to the junctions, and no lane of them links past one.
    for road in map.roads.iter() {
        let written = document
            .road
            .iter()
            .find(|other| other.id == document_id(&map, &road.id))
            .unwrap();
        let link = written.link.as_ref();
        for end in [
            link.and_then(|link| link.predecessor.as_ref()),
            link.and_then(|link| link.successor.as_ref()),
        ]
        .into_iter()
        .flatten()
        {
            assert_eq!(end.element_type, Some(ElementType::Junction));
        }
        for section in written.lanes.lane_section.iter() {
            let lanes = section
                .left
                .iter()
                .flat_map(|left| left.lane.iter().map(|lane| &lane.base))
                .chain(
                    section
                        .right
                        .iter()
                        .flat_map(|right| right.lane.iter().map(|lane| &lane.base)),
                );
            for lane in lanes {
                assert!(
                    lane.link.is_none(),
                    "road {} links a lane past a junction",
                    written.id
                );
            }
        }
    }

    // Each stub runs from the edges the lane traffic leaves has where the document
    // ends it to the edges the lane it enters starts with. Where those are the same
    // edges, it misses each by no more than the half of its length that reaches past
    // it; where the two lanes' own ends disagree — the slip's first cross-section is
    // square to the slip, not to the fork — by no more than that disagreement too.
    let stubs: Vec<_> = document
        .road
        .iter()
        .filter(|road| road.junction != "-1")
        .collect();
    assert_eq!(stubs.len(), 4);
    let stub_lane = match handedness {
        TrafficHandedness::RightHand => -1,
        TrafficHandedness::LeftHand => 1,
    };
    // Every lane here runs along its road, so the driver's left is the inner edge
    // on the right of the reference line and the outer on its left.
    let travel = |(inner, outer), id: i64| {
        if id < 0 {
            (inner, outer)
        } else {
            (outer, inner)
        }
    };
    let mut exact = 0;
    for stub in stubs {
        let link = stub.link.as_ref().unwrap();
        let lane_link = stub.lanes.lane_section[0]
            .left
            .iter()
            .flat_map(|left| left.lane.iter().map(|lane| &lane.base))
            .chain(
                stub.lanes.lane_section[0]
                    .right
                    .iter()
                    .flat_map(|right| right.lane.iter().map(|lane| &lane.base)),
            )
            .next()
            .unwrap()
            .link
            .as_ref()
            .unwrap();
        let evaluator = RoadEvaluator::new(stub);
        let ends = [
            (
                link.predecessor.as_ref().unwrap(),
                lane_link.predecessor[0].id,
                0.0,
            ),
            (
                link.successor.as_ref().unwrap(),
                lane_link.successor[0].id,
                evaluator.length(),
            ),
        ]
        .map(|(end, lane, stub_s)| {
            let road = RoadEvaluator::find(&document, &end.element_id).unwrap();
            let s = match end.contact_point {
                Some(ContactPoint::Start) => 0.0,
                _ => road.length(),
            };
            let theirs = travel(road.lane_edges(lane, s).unwrap(), lane);
            let mine = travel(evaluator.lane_edges(stub_lane, stub_s).unwrap(), stub_lane);
            (theirs, mine)
        });
        let disagreement = ends[0]
            .0
             .0
            .distance_to(ends[1].0 .0)
            .max(ends[0].0 .1.distance_to(ends[1].0 .1));
        if disagreement < 1e-6 {
            exact += 1;
        }
        let tolerance = 0.0005 + disagreement + 1e-6;
        for (theirs, mine) in ends {
            for (a, b) in [(theirs.0, mine.0), (theirs.1, mine.1)] {
                let gap = a.distance_to(b);
                assert!(
                    gap < tolerance,
                    "stub {} misses a lane it joins by {gap} m, where the two lanes \
                     disagree by {disagreement} m",
                    stub.id
                );
            }
        }
    }
    assert!(exact >= 2, "the straight branch and the merge meet exactly");
}

#[test]
fn a_split_and_a_merge_outside_a_junction_become_junctions_in_opendrive() {
    check(TrafficHandedness::RightHand);
}

#[test]
fn a_split_and_a_merge_keep_their_movements_when_traffic_keeps_left() {
    check(TrafficHandedness::LeftHand);
}

#[test]
fn opendrive_read_back_keeps_every_movement_of_a_split() {
    let map = forks(TrafficHandedness::RightHand);
    let xml = roadgen_opendrive::to_xml(&map).unwrap();
    // Not validated: the slip's first cross-section is a few millimetres off the
    // fork's, which reading back puts in a connection and validation measures.
    let back = roadgen_opendrive::from_xml(&xml).unwrap().map;
    // Through a stub, from each lane that leaves to each lane that is reached.
    let through = |map: &Map| -> BTreeSet<(String, String)> {
        let mut found = BTreeSet::new();
        for connection in map.connections.iter() {
            let from = map.lanes.get(&connection.from.lane).unwrap();
            if map.road(&from.road).unwrap().is_connector() {
                continue;
            }
            let into = map.lanes.get(&connection.to.lane).unwrap();
            let reached: Vec<_> = if map.road(&into.road).unwrap().is_connector() {
                map.connections_from(&into.id)
                    .into_iter()
                    .map(|next| map.lanes.get(&next.to.lane).unwrap())
                    .collect()
            } else {
                vec![into]
            };
            for to in reached {
                found.insert((document_id(map, &from.road), document_id(map, &to.road)));
            }
        }
        found
    };
    let through_back = through(back.as_map());
    assert_eq!(through_back.len(), 4);
    assert_eq!(through_back, through(map.as_map()));
}

#[test]
fn a_turn_that_branches_into_turns_is_read_as_a_road_into_the_junction() {
    // One lane runs into an intersection, carries on a few metres as a turn lane,
    // and there fans out into a left turn and a straight run. A connecting road
    // runs from one road to another, never into another connecting road, so the
    // lanelet before the fan cannot be one: it is the road the two turns leave.
    let mut osm = Osm::new();
    let approach_left = osm.line(None, &[(-30.0, 1.75), (-15.0, 1.75), (0.0, 1.75)]);
    let approach_right = osm.line(None, &[(-30.0, -1.75), (-15.0, -1.75), (0.0, -1.75)]);
    let pocket_left = osm.line(approach_left.last().copied(), &[(10.0, 1.75)]);
    let pocket_right = osm.line(approach_right.last().copied(), &[(10.0, -1.75)]);
    let (fan_left, fan_right) = (pocket_left.last().copied(), pocket_right.last().copied());
    osm.lanelet(approach_left, approach_right);
    let pocket = osm.turn(pocket_left, pocket_right, "left");
    let left_left = osm.line(fan_left, &[(15.0, 3.0), (18.0, 8.0), (19.75, 15.0)]);
    let left_right = osm.line(fan_right, &[(17.5, 0.0), (21.5, 7.0), (23.25, 15.0)]);
    let north_left = osm.line(left_left.last().copied(), &[(19.75, 35.0)]);
    let north_right = osm.line(left_right.last().copied(), &[(23.25, 35.0)]);
    osm.turn(left_left, left_right, "left");
    osm.lanelet(north_left, north_right);
    let on_left = osm.line(fan_left, &[(20.0, 1.75), (30.0, 1.75)]);
    let on_right = osm.line(fan_right, &[(20.0, -1.75), (30.0, -1.75)]);
    let east_left = osm.line(on_left.last().copied(), &[(50.0, 1.75)]);
    let east_right = osm.line(on_right.last().copied(), &[(50.0, -1.75)]);
    osm.turn(on_left, on_right, "straight");
    osm.lanelet(east_left, east_right);

    let map = read(&osm, TrafficHandedness::RightHand);
    let pocket = map
        .road(&roadgen_core::id::RoadId::new(pocket.to_string()))
        .expect("the turn lane before the fan is a road of its own");
    assert!(!pocket.is_connector());
    assert_eq!(
        map.roads.iter().filter(|road| road.is_connector()).count(),
        2,
        "the left turn and the straight run"
    );
    assert_eq!(map.junctions.len(), 1);
    assert!(roadgen_opendrive::check(&map).is_empty());

    // Through the document, the lane before the fan reaches both exits.
    let document = reparse_opendrive(&map);
    let pocket = document_id(&map, &pocket.id);
    let reached: BTreeSet<String> = movements(&document)
        .into_iter()
        .filter(|((road, _), _)| *road == pocket)
        .map(|(_, (road, _))| road)
        .collect();
    let exits: BTreeSet<String> = map
        .roads
        .iter()
        .filter(|road| !road.is_connector() && road.link.successor.is_none())
        .map(|road| document_id(&map, &road.id))
        .collect();
    assert_eq!(exits.len(), 2);
    assert_eq!(reached, exits);
}

#[test]
fn turns_from_the_lanes_of_one_road_are_one_junction() {
    // Two wide lanes side by side, the left one turning left and the right one
    // turning right. The turns neither cross nor share a line, and they start more
    // than a lane's width apart — but the road they leave runs into one junction,
    // so they are in it together.
    let mut osm = Osm::new();
    let along = |y: f64| [(-30.0, y), (-15.0, y), (0.0, y)];
    let top = osm.line(None, &along(5.0));
    let middle = osm.line(None, &along(0.0));
    let bottom = osm.line(None, &along(-5.0));
    let (top_end, middle_end, bottom_end) = (
        top.last().copied(),
        middle.last().copied(),
        bottom.last().copied(),
    );
    let (top, middle, bottom) = (osm.way(top), osm.way(middle), osm.way(bottom));
    osm.lanelet_on(top, middle, None);
    osm.lanelet_on(middle, bottom, None);
    // The left turn's two edges; the right turn's are the same mirrored.
    let inside = [(3.0, 6.5), (5.0, 10.0), (5.5, 15.0)];
    let outside = [(6.0, 1.5), (9.5, 6.0), (10.5, 15.0)];
    let mirrored = |points: &[(f64, f64)]| -> Vec<(f64, f64)> {
        points.iter().map(|(x, y)| (*x, -*y)).collect()
    };
    let left = osm.line(top_end, &inside);
    let right = osm.line(middle_end, &outside);
    osm.turn(left, right, "left");
    let left = osm.line(middle_end, &mirrored(&outside));
    let right = osm.line(bottom_end, &mirrored(&inside));
    osm.turn(left, right, "right");

    let map = read(&osm, TrafficHandedness::RightHand);
    assert_eq!(
        map.roads.iter().filter(|road| road.is_connector()).count(),
        2
    );
    assert_eq!(
        map.junctions.len(),
        1,
        "the road's end runs into one junction"
    );
    assert!(roadgen_opendrive::check(&map).is_empty());
}

/// Where a lane leads through the junction its road runs into, found the way
/// CARLA finds it: from the connecting roads whose own `<link>` names the road as
/// their predecessor and whose lane names the lane — not from the junction's lane
/// links — and then along that lane's own successor.
fn carla_next(document: &OpenDrive, road: &str, lane: i64) -> BTreeSet<(String, i64)> {
    let find = |id: &str| document.road.iter().find(|road| road.id == id).unwrap();
    let lanes_of = |road: &opendrive::road::Road| -> Vec<(i64, opendrive::lane::Lane)> {
        let section = &road.lanes.lane_section[0];
        section
            .left
            .iter()
            .flat_map(|left| left.lane.iter().map(|lane| (lane.id, lane.base.clone())))
            .chain(
                section
                    .right
                    .iter()
                    .flat_map(|right| right.lane.iter().map(|lane| (lane.id, lane.base.clone()))),
            )
            .collect()
    };
    let Some(junction) = find(road)
        .link
        .as_ref()
        .and_then(|link| link.successor.as_ref())
        .filter(|end| end.element_type == Some(ElementType::Junction))
    else {
        return BTreeSet::new();
    };
    let junction = document
        .junction
        .iter()
        .find(|candidate| candidate.id == junction.element_id)
        .unwrap();
    let mut found = BTreeSet::new();
    for connection in junction.connection.iter() {
        let connecting = find(connection.connecting_road.as_ref().unwrap());
        let link = connecting.link.as_ref().unwrap();
        if link.predecessor.as_ref().map(|end| end.element_id.as_str()) != Some(road) {
            continue;
        }
        let exit = link.successor.as_ref().unwrap().element_id.clone();
        for (_, base) in lanes_of(connecting) {
            let Some(lane_link) = &base.link else {
                continue;
            };
            if lane_link.predecessor.first().map(|p| p.id) == Some(lane) {
                if let Some(next) = lane_link.successor.first() {
                    found.insert((exit.clone(), next.id));
                }
            }
        }
    }
    found
}

#[test]
fn a_connector_entered_from_two_roads_is_written_once_for_each() {
    // Two approaches meet right at the mouth of one turn: both end on the two
    // points the turn starts on. A connecting road has one predecessor, so the
    // turn is written twice, once linked to each approach.
    let mut osm = Osm::new();
    let mouth_left = osm.node(0.0, 1.75, 0.0);
    let mouth_right = osm.node(0.0, -1.75, 0.0);
    let west_left = osm.line(None, &[(-40.0, 1.75), (-20.0, 1.75)]);
    let west_right = osm.line(None, &[(-40.0, -1.75), (-20.0, -1.75)]);
    let south_left = osm.line(None, &[(-40.0, -18.25), (-15.0, -8.0)]);
    let south_right = osm.line(None, &[(-40.0, -21.75), (-13.0, -11.5)]);
    for (left, right) in [(west_left, west_right), (south_left, south_right)] {
        let left = left.into_iter().chain([mouth_left]).collect();
        let right = right.into_iter().chain([mouth_right]).collect();
        osm.lanelet(left, right);
    }
    let turn_left = osm.line(Some(mouth_left), &[(8.0, 2.5), (13.25, 8.0), (13.25, 15.0)]);
    let turn_right = osm.line(
        Some(mouth_right),
        &[(10.0, -1.0), (16.75, 6.0), (16.75, 15.0)],
    );
    let exit_left = osm.line(turn_left.last().copied(), &[(13.25, 40.0)]);
    let exit_right = osm.line(turn_right.last().copied(), &[(16.75, 40.0)]);
    osm.turn(turn_left, turn_right, "left");
    let exit = osm.lanelet(exit_left, exit_right);

    let map = read(&osm, TrafficHandedness::RightHand);
    assert_eq!(
        map.roads.iter().filter(|road| road.is_connector()).count(),
        1
    );
    let connector = map.roads.iter().find(|road| road.is_connector()).unwrap();
    let entered_from: BTreeSet<_> = map
        .lanes_of(&connector.id)
        .iter()
        .flat_map(|lane| map.connections_to(&lane.id))
        .map(|connection| map.lanes.get(&connection.from.lane).unwrap().road.clone())
        .collect();
    assert_eq!(entered_from.len(), 2, "both approaches run into the turn");

    let document = reparse_opendrive(&map);
    let exit = document_id(&map, &roadgen_core::id::RoadId::new(exit.to_string()));
    for approach in &entered_from {
        let road = document_id(&map, approach);
        let lane = map.lanes_of(approach)[0];
        let number = roadgen_opendrive::lane_number(lane.side, lane.ordinal);
        let reached = carla_next(&document, &road, number);
        assert!(
            reached.iter().any(|(road, _)| *road == exit),
            "from road {road}, following the connecting roads' own links reaches {reached:?}"
        );
    }
    assert_eq!(
        document
            .road
            .iter()
            .filter(|road| road.junction != "-1")
            .count(),
        2,
        "the turn and its copy"
    );
}
