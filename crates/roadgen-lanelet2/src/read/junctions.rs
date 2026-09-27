//! Junctions, lane connections and road links.
//!
//! Lanelet2 has no junction. Autoware marks the lanes that cross an intersection
//! with `turn_direction`, and each of those is already a connector road; what is
//! left is deciding which connectors are the *same* junction. Two are when
//!
//! * one follows the other (a turn drawn as more than one lanelet),
//! * they leave the same road or reach the same road,
//! * they share a boundary (two turn lanes side by side), or
//! * their centrelines cross at the same level — a connector passing *over* another
//!   on a flyover is not in its junction, which is why the heights are compared — or
//! * they start or finish within a lane's width of each other, at the same level,
//!   as the two movements round one corner do.
//!
//! Every pair of lanelets where one starts at the points the other ends at becomes a
//! lane connection, inside the junction of whichever of the two is a connector.
//! Road links follow from those: a road that runs into a connector links to its
//! junction, and two roads link to each other only where they meet one to one.

use std::collections::{BTreeMap, BTreeSet, HashMap};

use ll2_core::id::Id;

use roadgen_core::geometry::Point3;
use roadgen_core::id::{ConnectionId, JunctionId, RoadId};
use roadgen_core::map::Map;
use roadgen_core::topology::{
    Junction, LaneConnection, LaneEnd, LaneEndpoint, RoadEnd, RoadEndpoint, RoadLinkTarget,
};

use super::roads::Built;
use super::source::Source;
use super::Approximations;
use crate::error::ImportError;

/// Two connectors that cross in plan with more than this between their heights are
/// on different levels, metres: one passes over the other.
const GRADE_SEPARATION: f64 = 3.0;

/// Two connectors whose ends come this close are at the same intersection, metres:
/// about a lane's width, which is how far apart the two movements round one corner
/// of a crossroads start and finish — they neither cross nor share a lane.
const SAME_PLACE: f64 = 4.0;

/// How close two roads' reference lines have to end to be linked, metres — the
/// validation's own tolerance.
const LINK_TOLERANCE: f64 = 1e-3;

pub(crate) fn build(
    source: &Source,
    built: &Built,
    map: &mut Map,
    approximations: &mut Approximations,
) -> Result<(), ImportError> {
    let junction_of = cluster(source, built);

    // The junctions, with their connectors.
    let mut junctions: BTreeMap<JunctionId, Junction> = BTreeMap::new();
    for (lanelet, junction) in &junction_of {
        let entry = junctions
            .entry(junction.clone())
            .or_insert_with(|| Junction::new(junction.clone(), None));
        let road = built.road_of[lanelet].clone();
        entry.connecting_roads.push(road.clone());
        if let Some(road) = map.roads.get_mut(&road) {
            road.junction = Some(junction.clone());
        }
    }
    let road_junction = |road: &RoadId| -> Option<JunctionId> {
        built.lanelets_of[road]
            .first()
            .and_then(|lanelet| junction_of.get(lanelet))
            .cloned()
    };

    // One connection per pair of lanelets that follow each other.
    for (from, from_lane) in &built.lane_of {
        for to in source.successors_of(*from) {
            let Some(to_lane) = built.lane_of.get(to) else {
                continue;
            };
            let (from_road, to_road) = (&built.road_of[from], &built.road_of[to]);
            let junction = road_junction(to_road).or_else(|| road_junction(from_road));
            if let Some(junction) = &junction {
                let entry = junctions.get_mut(junction).expect("made above");
                for road in [from_road, to_road] {
                    if road_junction(road).is_none() && !entry.incoming_roads.contains(road) {
                        entry.incoming_roads.push(road.clone());
                    }
                }
            }
            let connection = LaneConnection {
                id: ConnectionId::between(junction.as_ref(), from_lane, to_lane),
                from: LaneEndpoint::new(from_lane.clone(), LaneEnd::End),
                to: LaneEndpoint::new(to_lane.clone(), LaneEnd::Start),
                junction,
            };
            map.connections
                .insert(connection.id.clone(), connection)
                .ok();
        }
    }
    for junction in junctions.into_values() {
        map.junctions.insert(junction.id.clone(), junction).ok();
    }

    link_roads(source, built, map, &road_junction, approximations);
    Ok(())
}

/// Which junction each connector belongs to.
fn cluster(source: &Source, built: &Built) -> BTreeMap<Id, JunctionId> {
    let connectors: Vec<Id> = built.connectors.iter().copied().collect();
    let index: HashMap<Id, usize> = connectors
        .iter()
        .enumerate()
        .map(|(i, id)| (*id, i))
        .collect();
    let mut sets = DisjointSets::new(connectors.len());

    for (i, id) in connectors.iter().enumerate() {
        // One follows the other.
        for next in source.successors_of(*id) {
            if let Some(j) = index.get(next) {
                sets.union(i, *j);
            }
        }
    }
    // They leave the same road, or reach the same road: a road end links to one
    // junction, and every lane of it that turns turns into that one. One lane is
    // the narrowest road.
    let mut by_road: BTreeMap<(&RoadId, bool), Vec<usize>> = BTreeMap::new();
    for lanelet in source.lanelets.keys() {
        let Some(road) = built.road_of.get(lanelet) else {
            continue;
        };
        if built.connectors.contains(lanelet) {
            continue;
        }
        for (neighbours, leaving) in [
            (source.successors_of(*lanelet), true),
            (source.predecessors_of(*lanelet), false),
        ] {
            by_road
                .entry((road, leaving))
                .or_default()
                .extend(neighbours.iter().filter_map(|n| index.get(n)));
        }
    }
    for members in by_road.values() {
        for pair in members.windows(2) {
            sets.union(pair[0], pair[1]);
        }
    }
    // They share a boundary.
    let mut by_way: HashMap<Id, Vec<usize>> = HashMap::new();
    for (i, id) in connectors.iter().enumerate() {
        let lanelet = &source.lanelets[id];
        by_way.entry(lanelet.left_way).or_default().push(i);
        by_way.entry(lanelet.right_way).or_default().push(i);
    }
    for members in by_way.values() {
        for pair in members.windows(2) {
            sets.union(pair[0], pair[1]);
        }
    }
    // Their centrelines cross, at the same level.
    let lines: Vec<Vec<Point3>> = connectors
        .iter()
        .map(|id| {
            let lanelet = &source.lanelets[id];
            let (left, right) = (
                source.polyline(&lanelet.left),
                source.polyline(&lanelet.right),
            );
            let count = left.len().max(right.len());
            (0..count)
                .map(|i| {
                    let pick = |line: &[Point3]| line[(i * (line.len() - 1)) / (count - 1).max(1)];
                    pick(&left).lerp(pick(&right), 0.5)
                })
                .collect()
        })
        .collect();
    let boxes: Vec<[f64; 4]> = lines.iter().map(|line| bounds(line)).collect();
    for i in 0..lines.len() {
        for j in i + 1..lines.len() {
            if sets.find(i) == sets.find(j) || !overlap(&grown(&boxes[i]), &boxes[j]) {
                continue;
            }
            if ends_meet(&lines[i], &lines[j]) || cross_at_level(&lines[i], &lines[j]) {
                sets.union(i, j);
            }
        }
    }

    // Each junction is named after the lowest lanelet id in it, so the name does not
    // depend on the order anything was visited in.
    let mut lowest: HashMap<usize, Id> = HashMap::new();
    for (i, id) in connectors.iter().enumerate() {
        let root = sets.find(i);
        let entry = lowest.entry(root).or_insert(*id);
        *entry = (*entry).min(*id);
    }
    connectors
        .iter()
        .enumerate()
        .map(|(i, id)| (*id, JunctionId::new(lowest[&sets.find(i)].to_string())))
        .collect()
}

fn bounds(line: &[Point3]) -> [f64; 4] {
    line.iter().fold(
        [
            f64::INFINITY,
            f64::INFINITY,
            f64::NEG_INFINITY,
            f64::NEG_INFINITY,
        ],
        |[x0, y0, x1, y1], p| [x0.min(p.x), y0.min(p.y), x1.max(p.x), y1.max(p.y)],
    )
}

fn grown(a: &[f64; 4]) -> [f64; 4] {
    [
        a[0] - SAME_PLACE,
        a[1] - SAME_PLACE,
        a[2] + SAME_PLACE,
        a[3] + SAME_PLACE,
    ]
}

/// Whether an end of one line is within [`SAME_PLACE`] of an end of the other, at
/// the same level.
fn ends_meet(a: &[Point3], b: &[Point3]) -> bool {
    let ends = |line: &[Point3]| [line[0], line[line.len() - 1]];
    ends(a).iter().any(|p| {
        ends(b).iter().any(|q| {
            p.horizontal_distance_to(*q) <= SAME_PLACE && (p.z - q.z).abs() <= GRADE_SEPARATION
        })
    })
}

fn overlap(a: &[f64; 4], b: &[f64; 4]) -> bool {
    a[0] <= b[2] && b[0] <= a[2] && a[1] <= b[3] && b[1] <= a[3]
}

/// Whether two polylines cross in plan where their heights are within
/// [`GRADE_SEPARATION`] of each other.
fn cross_at_level(a: &[Point3], b: &[Point3]) -> bool {
    for p in a.windows(2) {
        for q in b.windows(2) {
            let (rx, ry) = (p[1].x - p[0].x, p[1].y - p[0].y);
            let (sx, sy) = (q[1].x - q[0].x, q[1].y - q[0].y);
            let denominator = rx * sy - ry * sx;
            if denominator.abs() < 1e-12 {
                continue;
            }
            let (wx, wy) = (q[0].x - p[0].x, q[0].y - p[0].y);
            let t = (wx * sy - wy * sx) / denominator;
            let u = (wx * ry - wy * rx) / denominator;
            if (0.0..=1.0).contains(&t) && (0.0..=1.0).contains(&u) {
                let za = p[0].z + (p[1].z - p[0].z) * t;
                let zb = q[0].z + (q[1].z - q[0].z) * u;
                if (za - zb).abs() <= GRADE_SEPARATION {
                    return true;
                }
            }
        }
    }
    false
}

/// Road links: to a junction where a road runs into a connector, to another road
/// where two meet one to one, and nothing where they merge or split outside a
/// junction — the lane connections still say where traffic goes there.
fn link_roads(
    source: &Source,
    built: &Built,
    map: &mut Map,
    road_junction: &dyn Fn(&RoadId) -> Option<JunctionId>,
    approximations: &mut Approximations,
) {
    // The lanelets beyond one end of a road.
    let beyond = |road: &RoadId, end: RoadEnd| -> BTreeSet<Id> {
        built.lanelets_of[road]
            .iter()
            .flat_map(|lanelet| match end {
                RoadEnd::End => source.successors_of(*lanelet),
                RoadEnd::Start => source.predecessors_of(*lanelet),
            })
            .copied()
            .filter(|lanelet| built.lane_of.contains_key(lanelet))
            .collect()
    };
    let roads_beyond = |road: &RoadId, end: RoadEnd| -> BTreeSet<RoadId> {
        beyond(road, end)
            .iter()
            .map(|lanelet| built.road_of[lanelet].clone())
            .collect()
    };
    let touches_junction = |road: &RoadId, end: RoadEnd| -> Vec<JunctionId> {
        let mut junctions: Vec<JunctionId> = roads_beyond(road, end)
            .iter()
            .filter_map(road_junction)
            .collect();
        junctions.sort();
        junctions.dedup();
        junctions
    };

    let mut links: Vec<(RoadId, RoadEnd, RoadLinkTarget)> = Vec::new();
    for road in built.lanelets_of.keys() {
        let connector = road_junction(road).is_some();
        for end in [RoadEnd::Start, RoadEnd::End] {
            let neighbours = roads_beyond(road, end);
            if neighbours.is_empty() {
                continue;
            }
            if connector {
                // A connector links to the road it comes from and the road it goes
                // to. Another connector links back only one to one; a road links
                // back through its junction.
                let target = neighbours.iter().find(|other| {
                    road_junction(other).is_none()
                        || (neighbours.len() == 1
                            && roads_beyond(other, end.opposite())
                                == BTreeSet::from([road.clone()]))
                });
                match target {
                    Some(other) => {
                        if neighbours.len() > 1 {
                            approximations.count(
                                "a road links to one road at each end, so {n} connector ends \
                                 that meet several keep the first; the lane connections keep \
                                 the rest",
                            );
                        }
                        links.push((
                            road.clone(),
                            end,
                            RoadLinkTarget::Road(RoadEndpoint::new(other.clone(), end.opposite())),
                        ));
                    }
                    None => approximations.count(
                        "{n} connector ends meet several other connectors and are not linked; \
                         the lane connections still join them",
                    ),
                }
                continue;
            }

            let junctions = touches_junction(road, end);
            if let Some(first) = junctions.first() {
                if junctions.len() > 1 {
                    approximations.count(
                        "a road end links to one junction, so {n} road ends that run into \
                         several keep the first",
                    );
                }
                links.push((road.clone(), end, RoadLinkTarget::Junction(first.clone())));
                continue;
            }
            // Two ordinary roads: linked only when each is all the other meets, and
            // their reference lines end at the same point.
            let [other] = neighbours.iter().collect::<Vec<_>>()[..] else {
                approximations.count(
                    "{n} road ends continue into more than one road outside any junction; \
                     they are not linked, and the lane connections say where traffic goes",
                );
                continue;
            };
            let back = roads_beyond(other, end.opposite());
            let meets =
                map.roads
                    .get(road)
                    .zip(map.roads.get(other))
                    .is_some_and(|(here, there)| {
                        here.endpoint(end)
                            .distance_to(there.endpoint(end.opposite()))
                            <= LINK_TOLERANCE
                    });
            if back != BTreeSet::from([road.clone()]) {
                approximations.count(
                    "{n} road ends continue into a road that more than one road runs into \
                     outside any junction; \
                     they are not linked, and the lane connections say where traffic goes",
                );
            } else if !touches_junction(other, end.opposite()).is_empty() {
                approximations.count(
                    "{n} road ends continue into a road that also meets a junction there, \
                     which is the road's link; they are not linked, and the lane connections \
                     say where traffic goes",
                );
            } else if !meets {
                approximations.count(
                    "{n} road ends continue into a road whose reference line starts elsewhere \
                     (a lane is added or dropped on the reference line's side); they are not \
                     linked, and the lane connections say where traffic goes",
                );
            } else {
                links.push((
                    road.clone(),
                    end,
                    RoadLinkTarget::Road(RoadEndpoint::new(other.clone(), end.opposite())),
                ));
            }
        }
    }
    for (road, end, target) in links {
        if let Some(road) = map.roads.get_mut(&road) {
            road.link.set(end, target);
        }
    }
}

/// Union–find over connector indices.
struct DisjointSets {
    parent: Vec<usize>,
}

impl DisjointSets {
    fn new(count: usize) -> Self {
        DisjointSets {
            parent: (0..count).collect(),
        }
    }

    fn find(&mut self, mut i: usize) -> usize {
        while self.parent[i] != i {
            self.parent[i] = self.parent[self.parent[i]];
            i = self.parent[i];
        }
        i
    }

    fn union(&mut self, a: usize, b: usize) {
        let (a, b) = (self.find(a), self.find(b));
        if a != b {
            self.parent[a.max(b)] = a.min(b);
        }
    }
}
