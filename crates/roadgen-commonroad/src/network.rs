//! Turning the IR into a CommonRoad lanelet network, its signs and lights, its
//! intersections and a planning problem.
//!
//! CommonRoad's lanelet is Lanelet2's, written out rather than shared: each lanelet
//! carries its own left and right bound as point lists of equal length, in the
//! direction its users travel, and names its predecessors, successors and the
//! lanelets either side of it by id. So the IR's lanes map one for one, and the
//! connectivity the IR holds as connections and shared cross-section edges is
//! written as references instead of being left to shared vertices.
//!
//! The decisions that shape the network:
//!
//! - **A lanelet is a lane somebody moves along.** Driving and bicycle lanes and
//!   footways are written — the lanes the Lanelet2 export writes — and shoulders,
//!   borders, parking strips and verges are not.
//! - **Neighbours share an edge and a kind.** Two lanelets are adjacent when their
//!   lanes meet at one cross-section edge and are both lanes a vehicle drives, or
//!   both footways: a kerb between a carriageway and its pavement is not a lane
//!   change. A kerb between two carriageways is written as adjacency with a `curb`
//!   line, which is how CommonRoad says "beside, but not to be crossed".
//! - **Rules are signs.** CommonRoad holds a speed limit, a stop and a give-way as
//!   traffic signs the lanelets refer to: the IR's sign objects with a code
//!   CommonRoad's catalogue has, and — virtual, standing nowhere — one speed-limit
//!   sign per distinct limit and one give-way sign per right-of-way rule.
//! - **A junction is an intersection.** Each at-grade junction becomes an
//!   intersection with one incoming per approach — the lanes of one road going one
//!   way that feed its connectors — and the connectors each feeds sorted into
//!   left, straight and right by how far they turn.
//!
//! Every id comes from one counter, from 1, in the order elements are written:
//! lanelets for lanes, crosswalk lanelets, signs, lights, intersections with their
//! incomings, then the planning problem. The [`Trace`] names them `lanelet:<id>`,
//! `traffic_sign:<id>`, `traffic_light:<id>`, `intersection:<id>`,
//! `incoming:<id>` and `planning_problem:<id>`.

use std::collections::{BTreeMap, HashMap, HashSet};

use roadgen_core::geometry::{Curve3, Point3, SamplingConfig};
use roadgen_core::map::Lane;
use roadgen_core::semantics::{
    BoundaryMarking, MapObject, MapObjectKind, ObjectGeometry, RoadMarking, TrafficRule,
};
use roadgen_core::{
    Direction, FormOfWay, IrRef, JunctionId, LaneConnection, LaneId, LaneType, ObjectId, Relation,
    RoadType, SpeedLimit, Trace, ValidatedMap,
};

use crate::error::ExportError;
use crate::model::{
    number, Adjacent, Bound, CycleElement, GoalState, Incoming, InitialState, Intersection,
    Lanelet, PlanningProblem, StopLine, TrafficLight, TrafficSign, TrafficSignElement,
};
use crate::signs;

/// Everything the scenario holds besides its header.
pub(crate) struct Network {
    pub lanelets: Vec<Lanelet>,
    pub traffic_signs: Vec<TrafficSign>,
    pub traffic_lights: Vec<TrafficLight>,
    pub intersections: Vec<Intersection>,
    pub planning_problems: Vec<PlanningProblem>,
    pub tags: Vec<&'static str>,
}

/// How the planning problem is posed.
pub(crate) struct Planning {
    pub enabled: bool,
    pub time_step: f64,
    pub goal_steps: Option<u64>,
}

/// Whether a lane of this type becomes a lanelet.
pub(crate) fn is_written(lane_type: LaneType) -> bool {
    matches!(
        lane_type,
        LaneType::Driving | LaneType::Biking | LaneType::Sidewalk
    )
}

/// The speed a planning problem's goal window is sized for: a car making its way
/// through a town, junctions and all.
const GOAL_SPEED: f64 = 5.0;

pub(crate) fn build(
    map: &ValidatedMap,
    planning: &Planning,
    trace: &mut Trace,
) -> Result<Network, ExportError> {
    let mut builder = Builder::new(map);
    builder.lanelets(trace)?;
    builder.crosswalks(trace)?;
    builder.sign_objects(trace);
    builder.speed_limits(trace);
    builder.right_of_way(trace);
    builder.lights(trace);
    builder.light_rules(trace);
    builder.intersections(trace)?;
    builder.planning_problem(planning, trace)?;
    let tags = builder.tags();
    let Builder {
        lanelets,
        traffic_signs,
        traffic_lights,
        intersections,
        planning_problems,
        ..
    } = builder;
    Ok(Network {
        lanelets,
        traffic_signs,
        traffic_lights,
        intersections,
        planning_problems,
        tags,
    })
}

struct Builder<'a> {
    map: &'a ValidatedMap,
    config: SamplingConfig,
    next_id: u64,
    /// The lanelet each written lane became, and its index in `lanelets`.
    lane_ids: HashMap<&'a LaneId, (u64, usize)>,
    /// The lanelet each crosswalk became.
    crosswalk_ids: HashMap<&'a ObjectId, u64>,
    light_ids: HashMap<&'a ObjectId, u64>,
    /// Lanes a stop sign stops, so that a right-of-way rule over them adds no
    /// give-way sign beside it.
    stopped: HashSet<&'a LaneId>,
    orientation: Orientation<'a>,
    lanelets: Vec<Lanelet>,
    traffic_signs: Vec<TrafficSign>,
    traffic_lights: Vec<TrafficLight>,
    intersections: Vec<Intersection>,
    planning_problems: Vec<PlanningProblem>,
}

impl<'a> Builder<'a> {
    fn new(map: &'a ValidatedMap) -> Self {
        Builder {
            map,
            config: map.metadata.sampling,
            next_id: 1,
            lane_ids: HashMap::new(),
            crosswalk_ids: HashMap::new(),
            light_ids: HashMap::new(),
            stopped: HashSet::new(),
            orientation: Orientation::of(map),
            lanelets: Vec::new(),
            traffic_signs: Vec::new(),
            traffic_lights: Vec::new(),
            intersections: Vec::new(),
            planning_problems: Vec::new(),
        }
    }

    fn take_id(&mut self) -> u64 {
        let id = self.next_id;
        self.next_id += 1;
        id
    }

    fn lanelet_of(&self, lane: &LaneId) -> Option<u64> {
        self.lane_ids.get(lane).map(|&(id, _)| id)
    }

    fn lanelet_mut(&mut self, lane: &LaneId) -> Option<&mut Lanelet> {
        let &(_, index) = self.lane_ids.get(lane)?;
        self.lanelets.get_mut(index)
    }

    /// One lanelet per written lane, with its bounds, its links and its neighbours.
    fn lanelets(&mut self, trace: &mut Trace) -> Result<(), ExportError> {
        let map = self.map;
        let lanes: Vec<&'a Lane> = map
            .lanes
            .iter()
            .filter(|lane| is_written(lane.lane_type))
            .collect();
        // Numbered before anything is built, because a lanelet names its
        // neighbours and successors.
        for (index, lane) in lanes.iter().enumerate() {
            let id = self.take_id();
            self.lane_ids.insert(&lane.id, (id, index));
        }

        let mut predecessors: HashMap<&LaneId, Vec<u64>> = HashMap::new();
        let mut successors: HashMap<&LaneId, Vec<u64>> = HashMap::new();
        for connection in map.connections.iter() {
            let Some((from, to)) = self.orientation.link(map, connection) else {
                continue;
            };
            let (Some(from_id), Some(to_id)) = (self.lanelet_of(from), self.lanelet_of(to)) else {
                continue;
            };
            push_unique(successors.entry(from).or_default(), to_id);
            push_unique(predecessors.entry(to).or_default(), from_id);
        }

        for lane in lanes {
            let id = self.lanelet_of(&lane.id).expect("numbered above");
            // Which edge, and which paint, is on the driver's left: the lane's own
            // left for a lane that runs with the reference line, its right for one
            // that runs against it — the way it is written, which for a footway
            // need not be the way it is drawn.
            let reversed = self.orientation.direction(lane) == Direction::Backward;
            let (left, right) = if reversed {
                (
                    lane.right_boundary.reversed(self.config)?,
                    lane.left_boundary.reversed(self.config)?,
                )
            } else {
                (lane.left_boundary.clone(), lane.right_boundary.clone())
            };
            let (left, right) = matched(
                left.to_polyline(self.config)?.points(),
                right.to_polyline(self.config)?.points(),
            );
            let ((left_edge, left_marking), (right_edge, right_marking)) = if reversed {
                (
                    (lane.right_edge, lane.right_marking),
                    (lane.left_edge, lane.left_marking),
                )
            } else {
                (
                    (lane.left_edge, lane.left_marking),
                    (lane.right_edge, lane.right_marking),
                )
            };
            let (one_way, bidirectional) = users(lane.lane_type);
            let lanelet = Lanelet {
                id,
                left: Bound {
                    points: left,
                    line_marking: Some(line_marking(left_marking, reversed)),
                },
                right: Bound {
                    points: right,
                    line_marking: Some(line_marking(right_marking, reversed)),
                },
                predecessors: predecessors.remove(&lane.id).unwrap_or_default(),
                successors: successors.remove(&lane.id).unwrap_or_default(),
                adjacent_left: self.across(lane, left_edge),
                adjacent_right: self.across(lane, right_edge),
                stop_line: None,
                lanelet_types: self.lanelet_types(lane),
                user_one_way: one_way,
                user_bidirectional: bidirectional,
                traffic_signs: Vec::new(),
                traffic_lights: Vec::new(),
            };
            trace.link_as(
                lane.id.clone(),
                format!("lanelet:{id}"),
                Relation::Exact,
                "lanelet",
            );
            self.lanelets.push(lanelet);
        }
        Ok(())
    }

    /// The lanelet on the other side of `edge` from `lane`, when it is written and
    /// of the same kind: two lanes a vehicle drives, or two footways.
    fn across(&self, lane: &Lane, edge: i32) -> Option<Adjacent> {
        let edges = self.map.section_edges(&lane.road, lane.section);
        let other = edges
            .get(&edge)?
            .lanes()
            .find(|other| other.id != lane.id)?;
        if other.lane_type.is_drivable() != lane.lane_type.is_drivable() {
            return None;
        }
        Some(Adjacent {
            lanelet: self.lanelet_of(&other.id)?,
            same_direction: self.orientation.direction(other) == self.orientation.direction(lane),
        })
    }

    /// What a lane is to CommonRoad: what it is for, and where.
    fn lanelet_types(&self, lane: &Lane) -> Vec<&'static str> {
        let map = self.map;
        let road = map.road(&lane.road);
        let mut types = Vec::new();
        match lane.lane_type {
            LaneType::Sidewalk => types.push("sidewalk"),
            LaneType::Biking => types.push("bicycleLane"),
            _ => match road {
                Some(road) if road.form == FormOfWay::Motorway => {
                    types.extend(["highway", "mainCarriageWay"])
                }
                Some(road) if road.form == FormOfWay::SlipRoad => {
                    types.push("highway");
                    types.extend(self.ramp(lane));
                }
                Some(road) => types.push(match road.road_type {
                    RoadType::Motorway => "highway",
                    RoadType::Rural => "country",
                    RoadType::Town | RoadType::LowSpeed | RoadType::Pedestrian => "urban",
                }),
                None => types.push("unknown"),
            },
        }
        if self.in_intersection(lane) {
            types.push("intersection");
        }
        types
    }

    /// Whether a slip road's lane leads onto a motorway or off one.
    fn ramp(&self, lane: &Lane) -> Option<&'static str> {
        let on_motorway = |lanes: Vec<LaneId>| {
            lanes.iter().any(|next| {
                self.map
                    .lane(next)
                    .and_then(|next| self.map.road(&next.road))
                    .is_some_and(|road| road.form == FormOfWay::Motorway)
            })
        };
        if on_motorway(self.map.successors(&lane.id)) {
            Some("accessRamp")
        } else if on_motorway(self.map.predecessors(&lane.id)) {
            Some("exitRamp")
        } else {
            None
        }
    }

    /// Whether a lane runs through an intersection: a connector of a junction
    /// [`is_intersection`] says is one.
    fn in_intersection(&self, lane: &Lane) -> bool {
        self.map
            .road(&lane.road)
            .and_then(|road| road.junction.as_ref())
            .is_some_and(|junction| is_intersection(self.map, junction))
    }

    /// A crossing, as a lanelet pedestrians walk either way, after the lanes.
    fn crosswalks(&mut self, trace: &mut Trace) -> Result<(), ExportError> {
        let map = self.map;
        for object in map.objects.iter() {
            if object.kind != MapObjectKind::Crosswalk {
                continue;
            }
            let ObjectGeometry::Band { left, right } = &object.geometry else {
                continue;
            };
            let (left, right) = matched(
                left.to_polyline(self.config)?.points(),
                right.to_polyline(self.config)?.points(),
            );
            let id = self.take_id();
            self.lanelets.push(Lanelet {
                id,
                left: Bound {
                    points: left,
                    line_marking: Some("dashed"),
                },
                right: Bound {
                    points: right,
                    line_marking: Some("dashed"),
                },
                predecessors: Vec::new(),
                successors: Vec::new(),
                adjacent_left: None,
                adjacent_right: None,
                stop_line: None,
                lanelet_types: vec!["crosswalk"],
                user_one_way: Vec::new(),
                user_bidirectional: vec!["pedestrian"],
                traffic_signs: Vec::new(),
                traffic_lights: Vec::new(),
            });
            self.crosswalk_ids.insert(&object.id, id);
            trace.link_as(
                object.id.clone(),
                format!("lanelet:{id}"),
                Relation::Exact,
                "crosswalk",
            );
        }
        Ok(())
    }

    /// The IR's signs CommonRoad's catalogue has, each referred to by the lanelets
    /// it governs. A stop sign also stops them, at the stop line across them when
    /// the map draws one.
    fn sign_objects(&mut self, trace: &mut Trace) {
        let map = self.map;
        for object in map.objects.iter() {
            let MapObjectKind::TrafficSign { code } = &object.kind else {
                continue;
            };
            let Some(element) = signs::element(code) else {
                continue;
            };
            let lanes: Vec<&'a LaneId> = object
                .lanes
                .iter()
                .filter(|lane| self.lane_ids.contains_key(lane))
                .collect();
            // CommonRoad's reader refuses a sign no lanelet refers to.
            if lanes.is_empty() {
                continue;
            }
            let id = self.take_id();
            let stop = object.kind.is_stop_sign();
            for lane in lanes {
                let line = stop.then(|| self.stop_line_across(lane)).flatten();
                let lanelet = self.lanelet_mut(lane).expect("written");
                push_unique(&mut lanelet.traffic_signs, id);
                if stop {
                    let stop_line = stop_line(lanelet, line);
                    push_unique(&mut stop_line.traffic_signs, id);
                    self.stopped.insert(lane);
                }
            }
            self.traffic_signs.push(TrafficSign {
                id,
                elements: vec![element],
                position: Some(position(&object.geometry)),
                virtual_sign: false,
            });
            trace.link_as(
                object.id.clone(),
                format!("traffic_sign:{id}"),
                Relation::Exact,
                "traffic_sign",
            );
        }
    }

    /// The two ends of the first stop line the map draws across `lane`.
    fn stop_line_across(&self, lane: &LaneId) -> Option<[Point3; 2]> {
        self.map
            .objects
            .iter()
            .filter(|object| object.kind == MapObjectKind::StopLine)
            .find(|object| object.lanes.contains(lane))
            .and_then(line_ends)
    }

    /// One virtual speed-limit sign per distinct limit, referred to by every
    /// lanelet held to it. A lane's limit is the lowest rule over it, or its own
    /// when no rule names it — the reading the SUMO and WOMD exports make.
    fn speed_limits(&mut self, trace: &mut Trace) {
        let map = self.map;
        let mut ruled: HashMap<&LaneId, (SpeedLimit, usize)> = HashMap::new();
        for (index, rule) in map.rules.iter().enumerate() {
            let TrafficRule::SpeedLimit { limit, lanes } = rule else {
                continue;
            };
            for lane in lanes {
                ruled
                    .entry(lane)
                    .and_modify(|held| {
                        if limit.mps() < held.0.mps() {
                            *held = (*limit, index);
                        }
                    })
                    .or_insert((*limit, index));
            }
        }

        // Keyed by the limit in micrometres per second, so that two limits written
        // the same way share a sign, in the order they are first met.
        let mut groups: Vec<(i64, SpeedLimit, Vec<u64>, Vec<IrRef>)> = Vec::new();
        for lane in map.lanes.iter() {
            if !lane.lane_type.is_drivable() {
                continue;
            }
            let Some(lanelet) = self.lanelet_of(&lane.id) else {
                continue;
            };
            let (limit, source) = match ruled.get(&lane.id) {
                Some(&(limit, rule)) => (limit, IrRef::Rule(rule)),
                None => match lane.speed_limit {
                    Some(limit) => (limit, IrRef::Lane(lane.id.clone())),
                    None => continue,
                },
            };
            let key = (limit.mps() * 1e6).round() as i64;
            let at = match groups.iter().position(|group| group.0 == key) {
                Some(at) => at,
                None => {
                    groups.push((key, limit, Vec::new(), Vec::new()));
                    groups.len() - 1
                }
            };
            groups[at].2.push(lanelet);
            if !groups[at].3.contains(&source) {
                groups[at].3.push(source);
            }
        }

        for (_, limit, lanelets, sources) in groups {
            let id = self.take_id();
            for lanelet in &lanelets {
                let index = self.index_of_lanelet(*lanelet);
                push_unique(&mut self.lanelets[index].traffic_signs, id);
            }
            self.traffic_signs.push(TrafficSign {
                id,
                elements: vec![TrafficSignElement {
                    sign_id: signs::MAX_SPEED,
                    // CommonRoad's speed limits are in metres per second.
                    additional_values: vec![number(limit.mps())],
                }],
                position: None,
                virtual_sign: true,
            });
            let relation = Relation::shared_by(sources.len());
            for source in sources {
                trace.link_as(
                    source,
                    format!("traffic_sign:{id}"),
                    relation,
                    "speed_limit",
                );
            }
        }
    }

    fn index_of_lanelet(&self, id: u64) -> usize {
        // Lane lanelets are numbered from 1 in the order they are stored.
        (id - 1) as usize
    }

    /// One virtual give-way sign per right-of-way rule, referred to by the lanes
    /// that yield — but for those a stop sign already stops, which yield anyway.
    fn right_of_way(&mut self, trace: &mut Trace) {
        let map = self.map;
        for (index, rule) in map.rules.iter().enumerate() {
            let TrafficRule::RightOfWay {
                yielding,
                stop_line: line,
                ..
            } = rule
            else {
                continue;
            };
            let lanes: Vec<&'a LaneId> = yielding
                .iter()
                .filter(|lane| self.lane_ids.contains_key(lane) && !self.stopped.contains(lane))
                .collect();
            if lanes.is_empty() {
                continue;
            }
            let ends = line
                .as_ref()
                .and_then(|line| map.objects.get(line))
                .and_then(line_ends);
            let id = self.take_id();
            for lane in lanes {
                let lanelet = self.lanelet_mut(lane).expect("written");
                push_unique(&mut lanelet.traffic_signs, id);
                if line.is_some() {
                    let stop_line = stop_line(lanelet, ends);
                    push_unique(&mut stop_line.traffic_signs, id);
                }
            }
            self.traffic_signs.push(TrafficSign {
                id,
                elements: vec![TrafficSignElement {
                    sign_id: signs::YIELD,
                    additional_values: Vec::new(),
                }],
                position: None,
                virtual_sign: true,
            });
            trace.link_as(
                IrRef::Rule(index),
                format!("traffic_sign:{id}"),
                Relation::Exact,
                "right_of_way",
            );
        }
    }

    /// Every traffic light, where it stands. The IR holds no timing, so each one's
    /// cycle is a single `inactive` phase: what it shows is the simulator's to decide.
    fn lights(&mut self, trace: &mut Trace) {
        let map = self.map;
        for object in map.objects.iter() {
            if !object.kind.is_traffic_light() {
                continue;
            }
            let id = self.take_id();
            self.light_ids.insert(&object.id, id);
            self.traffic_lights.push(TrafficLight {
                id,
                cycle: vec![CycleElement {
                    duration: 1,
                    color: "inactive",
                }],
                position: Some(position(&object.geometry)),
                direction: "all",
            });
            trace.link_as(
                object.id.clone(),
                format!("traffic_light:{id}"),
                Relation::Exact,
                "traffic_light",
            );
        }
    }

    /// The lights a traffic-light rule names, referred to by the lanes that stop
    /// for them, with the stop line [`Map::traffic_light_stops`] puts each one at.
    ///
    /// [`Map::traffic_light_stops`]: roadgen_core::map::Map::traffic_light_stops
    fn light_rules(&mut self, trace: &mut Trace) {
        let map = self.map;
        for (index, rule) in map.rules.iter().enumerate() {
            let TrafficRule::TrafficLight {
                lights,
                stop_lines,
                lanes,
            } = rule
            else {
                continue;
            };
            let ids: Vec<u64> = lights
                .iter()
                .filter_map(|light| self.light_ids.get(light).copied())
                .collect();
            if ids.is_empty() {
                continue;
            }
            for (line, group) in map.traffic_light_stops(stop_lines, lanes) {
                let ends = line
                    .as_ref()
                    .and_then(|line| map.objects.get(line))
                    .and_then(line_ends);
                for lane in &group {
                    let Some(lanelet) = self.lanelet_mut(lane) else {
                        continue;
                    };
                    for &id in &ids {
                        push_unique(&mut lanelet.traffic_lights, id);
                    }
                    let stop_line = stop_line(lanelet, ends);
                    for &id in &ids {
                        push_unique(&mut stop_line.traffic_lights, id);
                    }
                }
            }
            for id in ids {
                trace.link_as(
                    IrRef::Rule(index),
                    format!("traffic_light:{id}"),
                    Relation::Part,
                    "traffic_light",
                );
            }
        }
    }

    /// One intersection per at-grade junction.
    fn intersections(&mut self, trace: &mut Trace) -> Result<(), ExportError> {
        let map = self.map;
        for junction in map.junctions.iter() {
            if !is_intersection(map, &junction.id) {
                continue;
            }
            let Some(intersection) = self.intersection(&junction.id, trace)? else {
                continue;
            };
            trace.link_as(
                junction.id.clone(),
                format!("intersection:{}", intersection.id),
                Relation::Exact,
                "intersection",
            );
            self.intersections.push(intersection);
        }
        Ok(())
    }

    /// The intersection of one junction: an incoming per approach, and the
    /// crossings over it. `None` for a junction no written lane drives through.
    fn intersection(
        &mut self,
        junction: &JunctionId,
        trace: &mut Trace,
    ) -> Result<Option<Intersection>, ExportError> {
        let map = self.map;
        let connectors: Vec<&'a Lane> = map
            .lanes
            .iter()
            .filter(|lane| lane.lane_type.is_drivable() && self.lane_ids.contains_key(&lane.id))
            .filter(|lane| {
                map.road(&lane.road)
                    .is_some_and(|road| road.junction.as_ref() == Some(junction))
            })
            .collect();
        // An approach is the lanes of one road going one way, in the order first
        // reached.
        let mut approaches: Vec<((&'a roadgen_core::RoadId, Direction), Incoming)> = Vec::new();
        let mut involved: HashSet<&LaneId> = HashSet::new();
        for connector in &connectors {
            involved.insert(&connector.id);
            let turn = self.turn(connector)?;
            let connector_id = self.lanelet_of(&connector.id).expect("written");
            for from in map.predecessors(&connector.id) {
                let Some(from) = map.lane(&from) else {
                    continue;
                };
                if map.is_connector_lane(&from.id) {
                    continue;
                }
                let Some(from_id) = self.lanelet_of(&from.id) else {
                    continue;
                };
                involved.insert(&from.id);
                let key = (&from.road, from.direction);
                let at = match approaches.iter().position(|(k, _)| *k == key) {
                    Some(at) => at,
                    None => {
                        approaches.push((key, Incoming::default()));
                        approaches.len() - 1
                    }
                };
                let incoming = &mut approaches[at].1;
                push_unique(&mut incoming.incoming_lanelets, from_id);
                let successors = match turn {
                    Turn::Left => &mut incoming.successors_left,
                    Turn::Straight => &mut incoming.successors_straight,
                    Turn::Right => &mut incoming.successors_right,
                };
                push_unique(successors, connector_id);
            }
        }
        if approaches.is_empty() {
            return Ok(None);
        }
        let id = self.take_id();
        let mut incomings = Vec::new();
        for (_, mut incoming) in approaches {
            incoming.id = self.take_id();
            trace.link_as(
                junction.clone(),
                format!("incoming:{}", incoming.id),
                Relation::Part,
                "incoming",
            );
            incomings.push(incoming);
        }
        let crossing = map
            .objects
            .iter()
            .filter(|object| object.lanes.iter().any(|lane| involved.contains(lane)))
            .filter_map(|object| self.crosswalk_ids.get(&object.id).copied())
            .collect();
        Ok(Some(Intersection {
            id,
            incomings,
            crossing,
        }))
    }

    /// Which way a lane through a junction turns, by how far its heading swings
    /// between its ends — the reading the Lanelet2 export's `turn_direction` makes.
    fn turn(&self, lane: &Lane) -> Result<Turn, ExportError> {
        use std::f64::consts::{PI, TAU};
        let centreline = lane.travel_geometry(self.config)?.centerline;
        let swing = centreline.end_tangent()?.heading() - centreline.start_tangent()?.heading();
        let swing = (swing + PI).rem_euclid(TAU) - PI;
        Ok(if swing.abs() <= PI / 4.0 {
            Turn::Straight
        } else if swing > 0.0 {
            Turn::Left
        } else {
            Turn::Right
        })
    }

    /// A drive from the first lane a vehicle can set off from to the end of the
    /// route the map's own policy takes from it, at rest to begin with.
    fn planning_problem(
        &mut self,
        planning: &Planning,
        trace: &mut Trace,
    ) -> Result<(), ExportError> {
        if !planning.enabled {
            return Ok(());
        }
        let map = self.map;
        let Some(start) = map.default_start() else {
            return Ok(());
        };
        let route: Vec<LaneId> = map
            .route_from(&start)
            .into_iter()
            .filter(|lane| self.lane_ids.contains_key(lane))
            .collect();
        let (Some(first), Some(last)) = (route.first(), route.last()) else {
            return Ok(());
        };
        let lane = map.lane(first).expect("routed");
        let centreline: Curve3 = lane.travel_geometry(self.config)?.centerline;
        let mut length = 0.0;
        for lane in &route {
            if let Some(lane) = map.lane(lane) {
                length += lane.centerline.length(self.config)?;
            }
        }
        let horizon = planning
            .goal_steps
            .unwrap_or_else(|| ((length / GOAL_SPEED / planning.time_step).ceil() as u64).max(1));
        let goal = self.lanelet_of(last).expect("written");
        let id = self.take_id();
        self.planning_problems.push(PlanningProblem {
            id,
            initial_state: InitialState {
                position: centreline.start_point(),
                orientation: centreline.start_tangent()?.heading(),
                velocity: 0.0,
            },
            goal_states: vec![GoalState {
                time: (0, horizon),
                lanelets: vec![goal],
            }],
        });
        let local = format!("planning_problem:{id}");
        trace.link_as(first.clone(), local.clone(), Relation::Part, "start");
        trace.link_as(last.clone(), local, Relation::Part, "goal");
        Ok(())
    }

    /// The scenario's tags: what kind of roads it has, and that it is made up.
    fn tags(&self) -> Vec<&'static str> {
        let map = self.map;
        let mut tags = BTreeMap::new();
        for road in map.roads.iter() {
            let tag = if road.form.is_controlled_access() || road.road_type == RoadType::Motorway {
                "highway"
            } else if road.road_type == RoadType::Rural {
                "rural"
            } else {
                "urban"
            };
            tags.insert(tag, ());
        }
        if !self.intersections.is_empty() {
            tags.insert("intersection", ());
        }
        if self.lanelets.iter().any(|lanelet| {
            lanelet
                .adjacent_left
                .into_iter()
                .chain(lanelet.adjacent_right)
                .any(|adjacent| adjacent.same_direction)
        }) {
            tags.insert("multi_lane", ());
        }
        if self.lanelets.iter().any(|lanelet| {
            lanelet
                .adjacent_left
                .is_some_and(|adjacent| !adjacent.same_direction)
        }) {
            tags.insert("oncoming_traffic", ());
        }
        tags.insert("simulated", ());
        tags.into_keys().collect()
    }
}

/// Which way each lane is written.
///
/// The way it is drawn, but for a footway round a junction's corner. A footway is
/// walked both ways, so the IR joins two pavements end to end whichever way each
/// is drawn, while CommonRoad's successor relation runs from one lanelet's end to
/// the next one's start. The pavements along the arms keep their way; a corner
/// between them, which is drawn whichever way the junction's connectors are, is
/// written the other way round when more of its links then run end to start.
pub(crate) struct Orientation<'m> {
    flipped: HashSet<&'m LaneId>,
}

impl<'m> Orientation<'m> {
    pub fn of(map: &'m ValidatedMap) -> Self {
        let mut flipped = HashSet::new();
        for lane in map.lanes.iter() {
            if lane.lane_type != LaneType::Sidewalk || !map.is_connector_lane(&lane.id) {
                continue;
            }
            let linked = |reversed: bool| {
                let mut trial = HashSet::new();
                if reversed {
                    trial.insert(&lane.id);
                }
                let trial = Orientation { flipped: trial };
                map.connections
                    .iter()
                    .filter(|connection| {
                        connection.from.lane == lane.id || connection.to.lane == lane.id
                    })
                    .filter(|connection| trial.link(map, connection).is_some())
                    .count()
            };
            if linked(true) > linked(false) {
                flipped.insert(&lane.id);
            }
        }
        Orientation { flipped }
    }

    /// The way `lane` is written.
    pub fn direction(&self, lane: &Lane) -> Direction {
        if self.flipped.contains(&lane.id) {
            lane.direction.reversed()
        } else {
            lane.direction
        }
    }

    /// A connection as CommonRoad's successor relation: the lane whose exit it
    /// leaves from and the lane whose entry it reaches.
    ///
    /// A vehicle's connection is always that. A footway's need not be: one that
    /// joins an entry to an exit is the same link read backwards, and one that
    /// joins two exits or two entries is no successor at all, and `None`.
    pub fn link<'c>(
        &self,
        map: &'c ValidatedMap,
        connection: &'c LaneConnection,
    ) -> Option<(&'c LaneId, &'c LaneId)> {
        let (from, to) = (
            map.lane(&connection.from.lane)?,
            map.lane(&connection.to.lane)?,
        );
        let leaves = connection.from.end == self.direction(from).exit_end();
        let enters = connection.to.end == self.direction(to).entry_end();
        match (leaves, enters) {
            (true, true) => Some((&from.id, &to.id)),
            (false, false) => Some((&to.id, &from.id)),
            _ => None,
        }
    }
}

/// Whether a junction is an intersection: one a road that is not a motorway or a
/// slip road meets — the crossroad end of a slip road is, even inside an
/// interchange. A junction of motorway carriageways and slip roads alone is a merge
/// or a diverge, and CommonRoad's intersection, with its turns left and right, is
/// not what it is.
pub(crate) fn is_intersection(map: &ValidatedMap, junction: &JunctionId) -> bool {
    map.junction(junction).is_some_and(|junction| {
        junction.incoming_roads.iter().any(|road| {
            map.road(road)
                .is_some_and(|road| !road.form.is_controlled_access())
        })
    })
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Turn {
    Left,
    Straight,
    Right,
}

/// The lanelet's stop line, made with `ends` the first time it is asked for. A
/// later rule's line is the same line or none: a lanelet stops once.
fn stop_line(lanelet: &mut Lanelet, ends: Option<[Point3; 2]>) -> &mut StopLine {
    let line = lanelet.stop_line.get_or_insert_with(|| StopLine {
        points: None,
        line_marking: "solid",
        traffic_signs: Vec::new(),
        traffic_lights: Vec::new(),
    });
    if line.points.is_none() {
        line.points = ends;
    }
    line
}

/// A stop line object's two ends.
fn line_ends(object: &MapObject) -> Option<[Point3; 2]> {
    match &object.geometry {
        ObjectGeometry::Line(line) => Some([line.start_point(), line.end_point()]),
        _ => None,
    }
}

/// Where an object stands, as one point: a post's foot, the middle of a bar, the
/// middle of a band's leading edge.
fn position(geometry: &ObjectGeometry) -> Point3 {
    match geometry {
        ObjectGeometry::Point(point) => *point,
        ObjectGeometry::Line(line) => line.start_point().lerp(line.end_point(), 0.5),
        ObjectGeometry::Band { left, right } => left.start_point().lerp(right.start_point(), 0.5),
    }
}

/// Who uses a lanelet: those who travel it one way, and those who go both.
fn users(lane_type: LaneType) -> (Vec<&'static str>, Vec<&'static str>) {
    match lane_type {
        LaneType::Biking => (vec!["bicycle"], Vec::new()),
        // A footway is walked both ways whichever side of the road it is on.
        LaneType::Sidewalk => (Vec::new(), vec!["pedestrian"]),
        _ => (vec!["vehicle"], Vec::new()),
    }
}

/// The CommonRoad line marking a boundary's paint is written as.
///
/// A double line's two parts are listed left to right along the reference line, as
/// OpenDRIVE reads them, so a lane running against the reference line sees them the
/// other way round. CommonRoad has no colours.
fn line_marking(marking: BoundaryMarking, reversed: bool) -> &'static str {
    match (marking.marking, reversed) {
        (RoadMarking::None, _) => "no_marking",
        (RoadMarking::Solid, _) => "solid",
        (RoadMarking::Broken, _) => "dashed",
        (RoadMarking::SolidSolid, _) => "solid_solid",
        (RoadMarking::SolidBroken, false) | (RoadMarking::BrokenSolid, true) => "solid_dashed",
        (RoadMarking::BrokenSolid, false) | (RoadMarking::SolidBroken, true) => "dashed_solid",
        (RoadMarking::Curbstone, _) => "curb",
    }
}

/// Two bounds with as many points as each other, which CommonRoad requires: as
/// they are when they already have, else both resampled to the longer one's count
/// at even fractions of their own length.
fn matched(left: &[Point3], right: &[Point3]) -> (Vec<Point3>, Vec<Point3>) {
    if left.len() == right.len() {
        return (left.to_vec(), right.to_vec());
    }
    let count = left.len().max(right.len()).max(2);
    (resample(left, count), resample(right, count))
}

fn resample(points: &[Point3], count: usize) -> Vec<Point3> {
    let mut lengths = Vec::with_capacity(points.len());
    let mut total = 0.0;
    lengths.push(0.0);
    for pair in points.windows(2) {
        total += pair[0].distance_to(pair[1]);
        lengths.push(total);
    }
    let Some(&first) = points.first() else {
        return Vec::new();
    };
    if total <= 0.0 || points.len() < 2 {
        return vec![first; count];
    }
    let mut out = Vec::with_capacity(count);
    let mut segment = 0;
    for step in 0..count {
        let target = total * step as f64 / (count - 1) as f64;
        while segment + 2 < points.len() && lengths[segment + 1] < target {
            segment += 1;
        }
        let span = lengths[segment + 1] - lengths[segment];
        let t = if span > 0.0 {
            ((target - lengths[segment]) / span).clamp(0.0, 1.0)
        } else {
            0.0
        };
        out.push(points[segment].lerp(points[segment + 1], t));
    }
    // The ends exactly, so that a lanelet still meets the next one where it did.
    out[0] = first;
    out[count - 1] = *points.last().expect("non-empty");
    out
}

fn push_unique(ids: &mut Vec<u64>, id: u64) {
    if !ids.contains(&id) {
        ids.push(id);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use roadgen_core::semantics::MarkingColor;

    #[test]
    fn bounds_of_different_lengths_are_resampled_to_one_count() {
        let left = [Point3::new(0.0, 1.0, 0.0), Point3::new(10.0, 1.0, 0.0)];
        let right = [
            Point3::new(0.0, -1.0, 0.0),
            Point3::new(4.0, -1.0, 0.0),
            Point3::new(10.0, -1.0, 0.0),
        ];
        let (left, right) = matched(&left, &right);
        assert_eq!(left.len(), 3);
        assert_eq!(right.len(), 3);
        assert!((left[1].x - 5.0).abs() < 1e-9);
        assert_eq!(right[2], Point3::new(10.0, -1.0, 0.0));
    }

    #[test]
    fn a_passing_line_reads_the_other_way_against_the_reference_line() {
        let marking = BoundaryMarking::new(RoadMarking::SolidBroken, MarkingColor::White);
        assert_eq!(line_marking(marking, false), "solid_dashed");
        assert_eq!(line_marking(marking, true), "dashed_solid");
    }
}
