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

use std::collections::{BTreeMap, BTreeSet, HashMap, HashSet};

use roadgen_core::geometry::{Curve3, Point3, SamplingConfig, UnitVector3};
use roadgen_core::map::{Lane, SectionEdge};
use roadgen_core::semantics::{
    BoundaryMarking, MapObject, MapObjectKind, ObjectGeometry, RoadMarking, TrafficRule,
};
use roadgen_core::{
    Direction, FormOfWay, IrRef, JunctionId, LaneConnection, LaneId, LaneType, ObjectId, Relation,
    RoadId, RoadLinkTarget, RoadType, SpeedLimit, Trace, Turn, ValidatedMap,
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
    /// The light cycle in time steps, or `None` for dark lights.
    pub light_cycle: Option<Vec<(crate::LightPhase, u64)>>,
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
    builder.lights(planning.light_cycle.as_deref(), trace)?;
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
    index: Index<'a>,
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
            index: Index::of(map),
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
            // Sampled as drawn and then reversed, as `Lane::travel_polyline` does:
            // a polyline's vertices are the same either way along it.
            let sample = |curve: &Curve3| -> Result<Vec<Point3>, ExportError> {
                let polyline = curve.to_polyline(self.config)?;
                let polyline = if reversed {
                    polyline.reversed()
                } else {
                    polyline
                };
                Ok(polyline.points().to_vec())
            };
            let (left, right) = if reversed {
                (sample(&lane.right_boundary)?, sample(&lane.left_boundary)?)
            } else {
                (sample(&lane.left_boundary)?, sample(&lane.right_boundary)?)
            };
            let (left, right) = matched(left, right);
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
            let mut lanelet = Lanelet::new(
                id,
                Bound {
                    points: left,
                    line_marking: line_marking(left_marking, reversed),
                },
                Bound {
                    points: right,
                    line_marking: line_marking(right_marking, reversed),
                },
                self.lanelet_types(lane),
                one_way,
                bidirectional,
            );
            lanelet.predecessors = predecessors.remove(&lane.id).unwrap_or_default();
            lanelet.successors = successors.remove(&lane.id).unwrap_or_default();
            lanelet.adjacent_left = self.across(lane, left_edge);
            lanelet.adjacent_right = self.across(lane, right_edge);
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
        let other = self
            .index
            .sections
            .get(&(&lane.road, lane.section))?
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
        let on_motorway = |lanes: &[&Lane]| {
            lanes.iter().any(|next| {
                self.map
                    .road(&next.road)
                    .is_some_and(|road| road.form == FormOfWay::Motorway)
            })
        };
        if on_motorway(self.index.successors(&lane.id)) {
            Some("accessRamp")
        } else if on_motorway(self.index.predecessors(&lane.id)) {
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
                left.to_polyline(self.config)?.points().to_vec(),
                right.to_polyline(self.config)?.points().to_vec(),
            );
            let id = self.take_id();
            let bound = |points| Bound {
                points,
                line_marking: "dashed",
            };
            self.lanelets.push(Lanelet::new(
                id,
                bound(left),
                bound(right),
                vec!["crosswalk"],
                Vec::new(),
                vec!["pedestrian"],
            ));
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
            let Some((element, lanes)) = writable_sign(map, object) else {
                continue;
            };
            let id = self.take_id();
            // Read off the element written, so every spelling the catalogue takes
            // for a stop — `DE206` as well as `stop` — stops the lanes too.
            let stop = element.sign_id == signs::STOP;
            for lane in lanes {
                let line = stop.then(|| self.index.stop_line(lane)).flatten();
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

    /// One virtual speed-limit sign per distinct limit, referred to by every
    /// lanelet held to it. A lane's limit is the lowest rule over it, or its own
    /// when no rule names it — the reading the SUMO and WOMD exports make.
    fn speed_limits(&mut self, trace: &mut Trace) {
        let map = self.map;
        let ruled = map.rule_speed_limits();

        // Keyed by the limit in micrometres per second, so that two limits written
        // the same way share a sign, in the order they are first met.
        struct Group<'l> {
            key: i64,
            limit: SpeedLimit,
            lanes: Vec<&'l LaneId>,
            sources: Vec<IrRef>,
        }
        let mut groups: Vec<Group> = Vec::new();
        for lane in map.lanes.iter() {
            if !lane.lane_type.is_drivable() || !self.lane_ids.contains_key(&lane.id) {
                continue;
            }
            let (limit, source) = match ruled.get(&lane.id) {
                Some(&(limit, rule)) => (limit, IrRef::Rule(rule)),
                None => match lane.speed_limit {
                    Some(limit) => (limit, IrRef::Lane(lane.id.clone())),
                    None => continue,
                },
            };
            let key = (limit.mps() * 1e6).round() as i64;
            let at = position_or_push(
                &mut groups,
                |group| group.key == key,
                || Group {
                    key,
                    limit,
                    lanes: Vec::new(),
                    sources: Vec::new(),
                },
            );
            groups[at].lanes.push(&lane.id);
            push_unique(&mut groups[at].sources, source);
        }

        for group in groups {
            let id = self.take_id();
            for lane in group.lanes {
                let lanelet = self.lanelet_mut(lane).expect("written");
                push_unique(&mut lanelet.traffic_signs, id);
            }
            self.traffic_signs.push(TrafficSign {
                id,
                elements: vec![TrafficSignElement {
                    sign_id: signs::MAX_SPEED,
                    // CommonRoad's speed limits are in metres per second.
                    additional_values: vec![number(group.limit.mps())],
                }],
                position: None,
                virtual_sign: true,
            });
            let relation = Relation::shared_by(group.sources.len());
            for source in group.sources {
                trace.link_as(
                    source,
                    format!("traffic_sign:{id}"),
                    relation,
                    "speed_limit",
                );
            }
        }
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
    fn lights(
        &mut self,
        cycle: Option<&[(crate::LightPhase, u64)]>,
        trace: &mut Trace,
    ) -> Result<(), ExportError> {
        let map = self.map;
        let (elements, half) = match cycle {
            None => (
                vec![CycleElement {
                    duration: 1,
                    color: "inactive",
                }],
                0,
            ),
            Some(cycle) => (
                cycle
                    .iter()
                    .map(|&(phase, duration)| CycleElement {
                        duration,
                        color: phase.as_str(),
                    })
                    .collect(),
                // Half a cycle, as the second group needs it.
                crate::before_green(cycle),
            ),
        };
        let across = self.lights_across()?;
        for object in map.objects.iter() {
            if !object.kind.is_traffic_light() {
                continue;
            }
            let id = self.take_id();
            self.light_ids.insert(&object.id, id);
            self.traffic_lights.push(TrafficLight {
                id,
                cycle: elements.clone(),
                time_offset: if across.contains(&object.id) { half } else { 0 },
                position: position(&object.geometry),
            });
            trace.link_as(
                object.id.clone(),
                format!("traffic_light:{id}"),
                Relation::Exact,
                "traffic_light",
            );
        }
        Ok(())
    }

    /// The lights that run half a cycle behind the rest: at each junction, those
    /// that stop an approach across the first approach a light there stops.
    ///
    /// An approach's axis is the heading its lane arrives at, either way along it,
    /// so the approaches from north and south are one axis and those from east and
    /// west the other. A light short of no junction runs with no offset.
    fn lights_across(&self) -> Result<HashSet<&'a ObjectId>, ExportError> {
        use std::f64::consts::PI;
        let map = self.map;
        let mut axes: HashMap<&'a JunctionId, f64> = HashMap::new();
        let mut placed: HashSet<&'a ObjectId> = HashSet::new();
        let mut across = HashSet::new();
        for rule in map.rules.iter() {
            let TrafficRule::TrafficLight { lights, lanes, .. } = rule else {
                continue;
            };
            for light in lights {
                // A light two rules name runs as the first one placed it.
                if placed.contains(light) {
                    continue;
                }
                // The approach the light itself stands at: the first lane it governs
                // that the rule names too, as a rule over several junction mouths
                // has lights at each; else the rule's own first lane.
                let own = map
                    .objects
                    .get(light)
                    .map(|object| object.lanes.as_slice())
                    .unwrap_or_default();
                let approach = own
                    .iter()
                    .find(|lane| lanes.contains(lane))
                    .or_else(|| own.first())
                    .or_else(|| lanes.first())
                    .and_then(|lane| map.lane(lane));
                let Some(lane) = approach else {
                    continue;
                };
                let Some((junction, axis)) = self.approach_axis(lane)? else {
                    continue;
                };
                placed.insert(light);
                let first = *axes.entry(junction).or_insert(axis);
                let apart = (axis - first).rem_euclid(PI);
                if apart.min(PI - apart) > PI / 4.0 {
                    across.insert(light);
                }
            }
        }
        Ok(across)
    }

    /// The junction an approach runs into, and the axis it arrives along: its
    /// heading at the junction, either way along it. `None` for a lane that runs
    /// into no junction.
    fn approach_axis(&self, lane: &'a Lane) -> Result<Option<(&'a JunctionId, f64)>, ExportError> {
        let map = self.map;
        // The junction the approach's road runs into where traffic leaves it, or
        // failing that the one its movements go through.
        let linked = map
            .road(&lane.road)
            .and_then(|road| road.link.at(lane.direction.exit_end().as_road_end()))
            .and_then(|target| match target {
                RoadLinkTarget::Junction(junction) => Some(junction),
                _ => None,
            });
        let junction = linked.or_else(|| {
            self.index
                .successors(&lane.id)
                .iter()
                .find_map(|next| map.road(&next.road).and_then(|road| road.junction.as_ref()))
        });
        let Some(junction) = junction else {
            return Ok(None);
        };
        let heading = travel_tangents(lane)?.1.heading();
        Ok(Some((junction, heading.rem_euclid(std::f64::consts::PI))))
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
            let written: Vec<(&ObjectId, u64)> = lights
                .iter()
                .filter_map(|light| Some((light, *self.light_ids.get(light)?)))
                .collect();
            if written.is_empty() {
                continue;
            }
            // The lights a lane stops for: those that govern it themselves, when
            // any does — a rule over several mouths has a light at each, and they
            // run in different groups — else every light of the rule.
            let lights_of = |lane: &LaneId| -> Vec<u64> {
                let own: Vec<u64> = written
                    .iter()
                    .filter(|(light, _)| {
                        map.objects
                            .get(light)
                            .is_some_and(|object| object.lanes.contains(lane))
                    })
                    .map(|&(_, id)| id)
                    .collect();
                if own.is_empty() {
                    written.iter().map(|&(_, id)| id).collect()
                } else {
                    own
                }
            };
            for (line, group) in map.traffic_light_stops(stop_lines, lanes) {
                let ends = line
                    .as_ref()
                    .and_then(|line| map.objects.get(line))
                    .and_then(line_ends);
                for lane in &group {
                    let ids = lights_of(lane);
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
            for (_, id) in written {
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
        let connectors: Vec<&'a Lane> = self
            .index
            .connectors
            .get(junction)
            .into_iter()
            .flatten()
            .copied()
            .filter(|lane| lane.lane_type.is_drivable() && self.lane_ids.contains_key(&lane.id))
            .collect();
        // An approach is the lanes of one road going one way, in the order first
        // reached.
        let mut approaches: Vec<((&'a RoadId, Direction), Incoming)> = Vec::new();
        let mut involved: HashSet<&LaneId> = HashSet::new();
        for connector in &connectors {
            involved.insert(&connector.id);
            let (start, end) = travel_tangents(connector)?;
            let turn = Turn::from_swing(end.heading() - start.heading());
            let connector_id = self.lanelet_of(&connector.id).expect("written");
            for &from in self.index.predecessors(&connector.id) {
                if map.is_connector_lane(&from.id) {
                    continue;
                }
                let Some(from_id) = self.lanelet_of(&from.id) else {
                    continue;
                };
                involved.insert(&from.id);
                let key = (&from.road, from.direction);
                let at = position_or_push(
                    &mut approaches,
                    |(k, _)| *k == key,
                    || (key, Incoming::default()),
                );
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
            .filter_map(|object| Some((object, *self.crosswalk_ids.get(&object.id)?)))
            .filter(|(object, _)| object.lanes.iter().any(|lane| involved.contains(lane)))
            .map(|(_, id)| id)
            .collect();
        Ok(Some(Intersection {
            id,
            incomings,
            crossing,
        }))
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
        // The map's default start is any drivable lane, a bicycle lane among them;
        // a CommonRoad planning problem is posed to a vehicle, so a lane cars drive
        // comes first when the map has one.
        let start = map
            .lanes
            .iter()
            .find(|lane| lane.lane_type == LaneType::Driving && !map.is_connector_lane(&lane.id))
            .map(|lane| lane.id.clone())
            .or_else(|| map.default_start());
        let Some(start) = start else {
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
                position: lane.entry_point(),
                orientation: travel_tangents(lane)?.0.heading(),
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
        let mut tags = BTreeSet::new();
        for road in map.roads.iter() {
            let tag = if road.form.is_controlled_access() || road.road_type == RoadType::Motorway {
                "highway"
            } else if road.road_type == RoadType::Rural {
                "rural"
            } else {
                "urban"
            };
            tags.insert(tag);
        }
        if !self.intersections.is_empty() {
            tags.insert("intersection");
        }
        if self.lanelets.iter().any(|lanelet| {
            lanelet
                .adjacent_left
                .into_iter()
                .chain(lanelet.adjacent_right)
                .any(|adjacent| adjacent.same_direction)
        }) {
            tags.insert("multi_lane");
        }
        if self.lanelets.iter().any(|lanelet| {
            lanelet
                .adjacent_left
                .is_some_and(|adjacent| !adjacent.same_direction)
        }) {
            tags.insert("oncoming_traffic");
        }
        tags.insert("simulated");
        tags.into_iter().collect()
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
        let corner = |id: &LaneId| {
            map.lane(id)
                .is_some_and(|lane| lane.lane_type == LaneType::Sidewalk)
                && map.is_connector_lane(id)
        };
        // Each corner's own connections, from one pass over them all.
        let mut touching: HashMap<&LaneId, Vec<&LaneConnection>> = HashMap::new();
        for connection in map.connections.iter() {
            for lane in [&connection.from.lane, &connection.to.lane] {
                if corner(lane) {
                    touching.entry(lane).or_default().push(connection);
                }
            }
        }
        let mut flipped = HashSet::new();
        for (lane, connections) in touching {
            let linked = |reversed: bool| {
                let trial = Orientation {
                    flipped: if reversed {
                        HashSet::from([lane])
                    } else {
                        HashSet::new()
                    },
                };
                connections
                    .iter()
                    .filter(|connection| trial.link(map, connection).is_some())
                    .count()
            };
            if linked(true) > linked(false) {
                flipped.insert(lane);
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

/// The lanelet's stop line, made with `ends` the first time it is asked for. A
/// later rule's line is the same line or none: a lanelet stops once.
fn stop_line(lanelet: &mut Lanelet, ends: Option<[Point3; 2]>) -> &mut StopLine {
    let line = lanelet.stop_line.get_or_insert_with(|| StopLine {
        points: None,
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
fn matched(left: Vec<Point3>, right: Vec<Point3>) -> (Vec<Point3>, Vec<Point3>) {
    if left.len() == right.len() {
        return (left, right);
    }
    let count = left.len().max(right.len()).max(2);
    (resample(&left, count), resample(&right, count))
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

fn push_unique<T: PartialEq>(items: &mut Vec<T>, item: T) {
    if !items.contains(&item) {
        items.push(item);
    }
}

/// The index of the first item `matches`, pushing `make()` when there is none.
fn position_or_push<T>(
    items: &mut Vec<T>,
    matches: impl Fn(&T) -> bool,
    make: impl FnOnce() -> T,
) -> usize {
    match items.iter().position(matches) {
        Some(at) => at,
        None => {
            items.push(make());
            items.len() - 1
        }
    }
}

/// The directions a lane is travelled in where traffic enters it and where it
/// leaves, read off its centreline as drawn rather than a reversed copy of it.
fn travel_tangents(lane: &Lane) -> Result<(UnitVector3, UnitVector3), ExportError> {
    let (start, end) = (
        lane.centerline.start_tangent()?,
        lane.centerline.end_tangent()?,
    );
    Ok(match lane.direction {
        Direction::Forward => (start, end),
        Direction::Backward => (end.reversed(), start.reversed()),
    })
}

/// A sign object CommonRoad can hold — one whose code its catalogue has — with
/// the written lanes it governs, or `None`: CommonRoad's reader refuses a sign no
/// lanelet refers to.
pub(crate) fn writable_sign<'m>(
    map: &'m ValidatedMap,
    object: &'m MapObject,
) -> Option<(TrafficSignElement, Vec<&'m LaneId>)> {
    let MapObjectKind::TrafficSign { code } = &object.kind else {
        return None;
    };
    let element = signs::element(code)?;
    let lanes: Vec<&LaneId> = object
        .lanes
        .iter()
        .filter(|lane| {
            map.lane(lane)
                .is_some_and(|lane| is_written(lane.lane_type))
        })
        .collect();
    (!lanes.is_empty()).then_some((element, lanes))
}

/// Lookups the builder makes over and over, worked out in one pass each: the IR
/// keeps connections as one list, and asking it for a lane's successors scans it.
struct Index<'a> {
    successors: HashMap<&'a LaneId, Vec<&'a Lane>>,
    predecessors: HashMap<&'a LaneId, Vec<&'a Lane>>,
    /// The lanes on each junction's connecting roads, in the map's order.
    connectors: HashMap<&'a JunctionId, Vec<&'a Lane>>,
    /// The first stop line the map draws across each lane.
    stop_lines: HashMap<&'a LaneId, &'a MapObject>,
    sections: HashMap<(&'a RoadId, usize), BTreeMap<i32, SectionEdge<'a>>>,
}

impl<'a> Index<'a> {
    fn of(map: &'a ValidatedMap) -> Self {
        let mut successors: HashMap<&LaneId, Vec<&Lane>> = HashMap::new();
        let mut predecessors: HashMap<&LaneId, Vec<&Lane>> = HashMap::new();
        for connection in map.connections.iter() {
            let (from, to) = (&connection.from.lane, &connection.to.lane);
            if let (Some(from_lane), Some(to_lane)) = (map.lane(from), map.lane(to)) {
                successors.entry(from).or_default().push(to_lane);
                predecessors.entry(to).or_default().push(from_lane);
            }
        }
        let mut connectors: HashMap<&JunctionId, Vec<&Lane>> = HashMap::new();
        for lane in map.lanes.iter() {
            if let Some(junction) = map.road(&lane.road).and_then(|road| road.junction.as_ref()) {
                connectors.entry(junction).or_default().push(lane);
            }
        }
        let mut stop_lines: HashMap<&LaneId, &MapObject> = HashMap::new();
        for object in map.objects.iter() {
            if object.kind == MapObjectKind::StopLine {
                for lane in &object.lanes {
                    stop_lines.entry(lane).or_insert(object);
                }
            }
        }
        let sections = map
            .roads
            .iter()
            .flat_map(|road| {
                (0..road.sections.len())
                    .map(move |section| ((&road.id, section), map.section_edges(&road.id, section)))
            })
            .collect();
        Index {
            successors,
            predecessors,
            connectors,
            stop_lines,
            sections,
        }
    }

    fn successors(&self, lane: &LaneId) -> &[&'a Lane] {
        self.successors
            .get(lane)
            .map(Vec::as_slice)
            .unwrap_or_default()
    }

    fn predecessors(&self, lane: &LaneId) -> &[&'a Lane] {
        self.predecessors
            .get(lane)
            .map(Vec::as_slice)
            .unwrap_or_default()
    }

    /// The two ends of the first stop line the map draws across `lane`.
    fn stop_line(&self, lane: &LaneId) -> Option<[Point3; 2]> {
        self.stop_lines
            .get(lane)
            .and_then(|object| line_ends(object))
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
        let (left, right) = matched(left.to_vec(), right.to_vec());
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
