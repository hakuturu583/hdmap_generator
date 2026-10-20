//! Turning the IR into WOMD map features and traffic-signal lane states.
//!
//! WOMD's map vocabulary is the one GPUDrive borrowed — a lane is its centreline, a
//! painted marking a `RoadLine`, the edge of the drivable surface a `RoadEdge` — but
//! WOMD keeps what GPUDrive threw away: every point has a z, a lane names the lanes
//! it is entered from and exits to, its neighbours going the same way, and the line
//! or edge on either side of it, and a stop sign names the lanes it controls. So the
//! map written here is a graph as well as a set of polylines.
//!
//! The same three decisions as the GPUDrive export hold, for the same reasons:
//!
//! - **One feature per cross-section edge.** An edge is the boundary of the drivable
//!   surface or a painted line between lanes, not both — [`SectionEdge::kind`] says
//!   which, for both exports.
//! - **A junction connector has no edges.** Its centreline is written — marked
//!   `interpolating`, which is what WOMD calls a lane through an intersection — and
//!   its boundaries are not: there is no wall down the middle of a junction.
//! - **The road edge stays whole where another road crosses over it.** Unlike
//!   GPUDrive's, a WOMD point has a height, so a bridge's edges and the street's
//!   beneath it are apart where they cross and nothing is cut.
//!
//! Features are numbered from 1 in the order they are written — lanes, then the
//! edges and lines, crossings and stop signs — and that number is the only
//! identifier WOMD keeps. The [`Trace`] names a feature `feature:<id>` and a signal's
//! lane state `signal:<lane id>`.

use std::collections::{BTreeMap, HashMap, HashSet};

use roadgen_core::geometry::{Point3, SamplingConfig};
use roadgen_core::map::{EdgeKind, Lane, SectionEdge};
use roadgen_core::semantics::{
    BoundaryMarking, MapObjectKind, MarkingColor, ObjectGeometry, RoadMarking, TrafficRule,
};
use roadgen_core::{
    Direction, IrRef, LaneId, LaneType, Relation, RoadId, SpeedLimit, Trace, ValidatedMap,
};

use crate::error::ExportError;
use crate::pb::{
    self, lane_center, map_feature::FeatureData, road_edge, road_line, traffic_signal_lane_state,
};

/// Every map feature of the scenario, in the order they are written, and one step's
/// signal states.
pub(crate) fn all_traced(
    map: &ValidatedMap,
    trace: &mut Trace,
) -> Result<(Vec<pb::MapFeature>, Vec<pb::TrafficSignalLaneState>), ExportError> {
    let config = map.metadata.sampling;
    let lanes: Vec<&Lane> = map
        .lanes
        .iter()
        .filter(|lane| lane.lane_type.is_drivable())
        .collect();
    // Lane `i` is feature `i + 1`. They are numbered before anything is built,
    // because a lane names the lines either side of it and its neighbours.
    let ids: HashMap<&LaneId, i64> = lanes
        .iter()
        .enumerate()
        .map(|(index, lane)| (&lane.id, index as i64 + 1))
        .collect();
    let polylines = lanes
        .iter()
        .map(|lane| Ok(points(lane.travel_polyline(config)?.points())))
        .collect::<Result<Vec<_>, ExportError>>()?;
    let context = Context {
        map,
        ids: &ids,
        polylines: &polylines,
        graph: graph(map, &ids),
        speeds: rule_speeds(map),
        sections: map
            .roads
            .iter()
            .flat_map(|road| {
                (0..road.sections.len())
                    .map(move |section| ((&road.id, section), map.section_edges(&road.id, section)))
            })
            .collect(),
    };

    let mut out = Features {
        features: Vec::new(),
        trace,
    };
    let (boundaries, edges) = edges_and_lines(&context, lanes.len() as i64 + 1, config)?;
    for (index, lane) in lanes.iter().enumerate() {
        out.push(
            context.lane_feature(lane, index, &edges),
            vec![Source::exact(lane.id.clone())],
        );
    }
    for (data, sources) in boundaries {
        out.push(data, sources);
    }
    crosswalks(map, config, &mut out)?;
    stop_signs(map, &ids, &mut out);
    let signals = signal_states(&context, out.trace);
    Ok((out.features, signals))
}

/// What building a lane needs to know about the map, worked out once.
struct Context<'a> {
    map: &'a ValidatedMap,
    ids: &'a HashMap<&'a LaneId, i64>,
    /// Each written lane's centreline in travel order, by feature id − 1.
    polylines: &'a [Vec<pb::MapPoint>],
    /// The feature ids each written lane is entered from and exits to.
    graph: HashMap<&'a LaneId, (Vec<i64>, Vec<i64>)>,
    speeds: HashMap<&'a LaneId, SpeedLimit>,
    sections: HashMap<(&'a RoadId, usize), BTreeMap<i32, SectionEdge<'a>>>,
}

/// The features under construction, and the trace of where each came from.
struct Features<'t> {
    features: Vec<pb::MapFeature>,
    trace: &'t mut Trace,
}

/// One IR element a feature is drawn from, and how.
struct Source {
    ir: IrRef,
    relation: Relation,
    role: Option<&'static str>,
}

impl Source {
    fn exact(ir: impl Into<IrRef>) -> Self {
        Source {
            ir: ir.into(),
            relation: Relation::Exact,
            role: None,
        }
    }
}

impl Features<'_> {
    /// Adds a feature under the next number, linking each of `sources` to it.
    fn push(&mut self, data: FeatureData, sources: Vec<Source>) {
        let id = self.features.len() as i64 + 1;
        let local = format!("feature:{id}");
        for source in sources {
            match source.role {
                Some(role) => self
                    .trace
                    .link_as(source.ir, local.clone(), source.relation, role),
                None => self.trace.link(source.ir, local.clone(), source.relation),
            }
        }
        self.features.push(pb::MapFeature {
            id: Some(id),
            feature_data: Some(data),
        });
    }
}

/// Entry and exit lanes for every written lane, from one pass over the connections:
/// the order of each list is the connections' own.
fn graph<'a>(
    map: &'a ValidatedMap,
    ids: &HashMap<&'a LaneId, i64>,
) -> HashMap<&'a LaneId, (Vec<i64>, Vec<i64>)> {
    let mut graph: HashMap<&LaneId, (Vec<i64>, Vec<i64>)> = HashMap::new();
    for connection in map.connections.iter() {
        let (from, to) = (&connection.from.lane, &connection.to.lane);
        let (Some(&from_id), Some(&to_id)) = (ids.get(from), ids.get(to)) else {
            continue;
        };
        graph.entry(from).or_default().1.push(to_id);
        graph.entry(to).or_default().0.push(from_id);
    }
    graph
}

/// The limit each lane a speed-limit rule names is held to: where several rules
/// name one lane, the lowest, which is the one a driver has to keep to — as the
/// SUMO export reads it.
fn rule_speeds(map: &ValidatedMap) -> HashMap<&LaneId, SpeedLimit> {
    let mut speeds: HashMap<&LaneId, SpeedLimit> = HashMap::new();
    for rule in &map.rules {
        let TrafficRule::SpeedLimit { limit, lanes } = rule else {
            continue;
        };
        for lane in lanes {
            speeds
                .entry(lane)
                .and_modify(|held| {
                    if limit.mps() < held.mps() {
                        *held = *limit;
                    }
                })
                .or_insert(*limit);
        }
    }
    speeds
}

/// A boundary feature as the lanes beside it refer to it.
struct EdgeFeature {
    id: i64,
    /// The line's type; `None` for a road edge.
    line: Option<road_line::RoadLineType>,
}

impl EdgeFeature {
    /// The boundary as it runs the whole length of a lane whose last vertex is `last`.
    /// A road edge's type is `TYPE_UNKNOWN`, as WOMD has it.
    fn segment(&self, last: i32) -> pb::BoundarySegment {
        pb::BoundarySegment {
            lane_start_index: Some(0),
            lane_end_index: Some(last),
            boundary_feature_id: Some(self.id),
            boundary_type: Some(self.line.unwrap_or(road_line::RoadLineType::TypeUnknown) as i32),
        }
    }
}

/// The boundary features, by the road, cross-section and edge index they lie on.
type EdgeFeatures<'a> = HashMap<(&'a RoadId, usize, i32), EdgeFeature>;

impl<'a> Context<'a> {
    /// A lane: its centreline in travel order, and everything it says about the
    /// lanes and lines around it.
    fn lane_feature(&self, lane: &Lane, index: usize, edges: &EdgeFeatures) -> FeatureData {
        let polyline = self.polylines[index].clone();
        let last = last_index(&polyline);

        // Which edge is on the driver's left: the lane's own left for a lane that
        // runs with the road's reference line, its right for one that runs against it.
        let (left, right) = if lane.direction == Direction::Forward {
            (lane.left_edge, lane.right_edge)
        } else {
            (lane.right_edge, lane.left_edge)
        };
        let side = |edge: i32| {
            let written = edges.get(&(&lane.road, lane.section, edge));
            let boundaries: Vec<pb::BoundarySegment> = written
                .map(|written| written.segment(last))
                .into_iter()
                .collect();
            // A kerb between two lanes is a median: they are side by side, but nobody
            // changes lanes across it.
            let neighbors = match (written, self.across(lane, edge)) {
                (Some(EdgeFeature { line: None, .. }), _) | (_, None) => Vec::new(),
                (_, Some(feature_id)) => vec![pb::LaneNeighbor {
                    feature_id: Some(feature_id),
                    self_start_index: Some(0),
                    self_end_index: Some(last),
                    neighbor_start_index: Some(0),
                    neighbor_end_index: Some(last_index(&self.polylines[feature_id as usize - 1])),
                    boundaries: boundaries.clone(),
                }],
            };
            (boundaries, neighbors)
        };
        let (left_boundaries, left_neighbors) = side(left);
        let (right_boundaries, right_neighbors) = side(right);
        let (entry_lanes, exit_lanes) = self.graph.get(&lane.id).cloned().unwrap_or_default();

        FeatureData::Lane(pb::LaneCenter {
            speed_limit_mph: self
                .speeds
                .get(&lane.id)
                .copied()
                .or(lane.speed_limit)
                .map(SpeedLimit::mph),
            r#type: Some(self.lane_type(lane) as i32),
            interpolating: Some(self.map.is_connector_lane(&lane.id)),
            polyline,
            entry_lanes,
            exit_lanes,
            left_boundaries,
            right_boundaries,
            left_neighbors,
            right_neighbors,
        })
    }

    /// The feature id of the lane on the other side of `edge` from `lane`, when it
    /// is written and goes the same way: WOMD's neighbours are only those.
    fn across(&self, lane: &Lane, edge: i32) -> Option<i64> {
        let sides = self.sections.get(&(&lane.road, lane.section))?.get(&edge)?;
        let other = sides.lanes().find(|other| other.id != lane.id)?;
        (other.direction == lane.direction)
            .then(|| self.ids.get(&other.id).copied())
            .flatten()
    }

    /// Which WOMD lane type a lane is: a bike lane is its own, and the rest of the
    /// distinction is the road's — the same reading the GPUDrive export makes.
    fn lane_type(&self, lane: &Lane) -> lane_center::LaneType {
        use lane_center::LaneType as T;
        if lane.lane_type == LaneType::Biking {
            return T::TypeBikeLane;
        }
        match self.map.road(&lane.road) {
            Some(road) if road.is_freeway() => T::TypeFreeway,
            Some(_) => T::TypeSurfaceStreet,
            None => T::TypeUndefined,
        }
    }
}

/// The edges and lines of every cross-section but a connector's, numbered from
/// `first` in the order they come back, with their sources; and how the lanes
/// beside each find it.
#[allow(clippy::type_complexity)]
fn edges_and_lines<'a>(
    context: &Context<'a>,
    first: i64,
    config: SamplingConfig,
) -> Result<(Vec<(FeatureData, Vec<Source>)>, EdgeFeatures<'a>), ExportError> {
    let mut features = Vec::new();
    let mut written = EdgeFeatures::new();
    for road in context.map.roads.iter().filter(|road| !road.is_connector()) {
        for section in 0..road.sections.len() {
            let Some(edges) = context.sections.get(&(&road.id, section)) else {
                continue;
            };
            for (&index, edge) in edges {
                let Some((curve, kind)) = edge.kind() else {
                    continue;
                };
                let polyline = points(curve.to_polyline(config)?.points());
                if polyline.is_empty() {
                    continue;
                }
                let (data, line, role) = match kind {
                    EdgeKind::RoadBoundary | EdgeKind::Median => {
                        let kind = if kind == EdgeKind::Median {
                            road_edge::RoadEdgeType::TypeRoadEdgeMedian
                        } else {
                            road_edge::RoadEdgeType::TypeRoadEdgeBoundary
                        };
                        let edge = pb::RoadEdge {
                            r#type: Some(kind as i32),
                            polyline,
                        };
                        (FeatureData::RoadEdge(edge), None, "road_edge")
                    }
                    EdgeKind::Line(marking) => {
                        let kind = line_type(marking);
                        let line = pb::RoadLine {
                            r#type: Some(kind as i32),
                            polyline,
                        };
                        (FeatureData::RoadLine(line), Some(kind), "road_line")
                    }
                };
                let id = first + features.len() as i64;
                written.insert((&road.id, section, index), EdgeFeature { id, line });
                features.push((data, edge_sources(edge, role)));
            }
        }
    }
    Ok((features, written))
}

/// The lanes either side of an edge, each linked to the one feature written for it:
/// `Merged` when two lanes share the edge, `Exact` when only one touches it.
fn edge_sources(edge: &SectionEdge<'_>, role: &'static str) -> Vec<Source> {
    let relation = Relation::shared_by(edge.lanes().count());
    edge.lanes()
        .map(|lane| Source {
            ir: IrRef::Lane(lane.id.clone()),
            relation,
            role: Some(role),
        })
        .collect()
}

/// Which WOMD line type a marking is written as.
fn line_type(marking: BoundaryMarking) -> road_line::RoadLineType {
    use road_line::RoadLineType as L;
    match (marking.marking, marking.color) {
        (RoadMarking::Solid, MarkingColor::White) => L::TypeSolidSingleWhite,
        (RoadMarking::Broken, MarkingColor::White) => L::TypeBrokenSingleWhite,
        (RoadMarking::SolidSolid, MarkingColor::White) => L::TypeSolidDoubleWhite,
        (RoadMarking::Solid, MarkingColor::Yellow) => L::TypeSolidSingleYellow,
        (RoadMarking::Broken, MarkingColor::Yellow) => L::TypeBrokenSingleYellow,
        (RoadMarking::SolidSolid, MarkingColor::Yellow) => L::TypeSolidDoubleYellow,
        (RoadMarking::BrokenSolid | RoadMarking::SolidBroken, MarkingColor::Yellow) => {
            L::TypePassingDoubleYellow
        }
        // WOMD only has the yellow passing line, so a white one is the unknown line
        // rather than a colour it is not.
        (RoadMarking::BrokenSolid | RoadMarking::SolidBroken, MarkingColor::White) => {
            L::TypeUnknown
        }
        (RoadMarking::None | RoadMarking::Curbstone, _) => L::TypeUnknown,
    }
}

/// A crossing, as the outline of its strip: up one edge and back down the other.
fn crosswalks(
    map: &ValidatedMap,
    config: SamplingConfig,
    out: &mut Features,
) -> Result<(), ExportError> {
    let crossings = map
        .objects
        .iter()
        .filter(|object| matches!(object.kind, MapObjectKind::Crosswalk));
    for object in crossings {
        let ObjectGeometry::Band { left, right } = &object.geometry else {
            continue;
        };
        let mut polygon = points(left.to_polyline(config)?.points());
        let mut back = points(right.to_polyline(config)?.points());
        back.reverse();
        polygon.extend(back);
        out.push(
            FeatureData::Crosswalk(pb::Crosswalk { polygon }),
            vec![Source::exact(object.id.clone())],
        );
    }
    Ok(())
}

/// A stop sign, at the middle of the bar the IR hangs it on, naming the lanes it
/// controls.
fn stop_signs(map: &ValidatedMap, ids: &HashMap<&LaneId, i64>, out: &mut Features) {
    for object in map
        .objects
        .iter()
        .filter(|object| object.kind.is_stop_sign())
    {
        let position = match &object.geometry {
            ObjectGeometry::Point(point) => *point,
            ObjectGeometry::Line(bar) => bar.start_point().lerp(bar.end_point(), 0.5),
            ObjectGeometry::Band { .. } => continue,
        };
        out.push(
            FeatureData::StopSign(pb::StopSign {
                lane: object
                    .lanes
                    .iter()
                    .filter_map(|lane| ids.get(lane).copied())
                    .collect(),
                position: Some(point(position)),
            }),
            vec![Source::exact(object.id.clone())],
        );
    }
}

/// One step's signal states: a lane state for every lane a traffic light's state is
/// carried by ([`roadgen_core::Map::signal_carriers`]: the lanes through the junction
/// its approaches feed), stopping where the lane begins.
///
/// The IR holds no timing, so every state is `LANE_STATE_UNKNOWN`: what a light
/// shows is the simulator's to decide.
fn signal_states(context: &Context, trace: &mut Trace) -> Vec<pb::TrafficSignalLaneState> {
    let mut states = Vec::new();
    let mut seen = HashSet::new();
    for (index, rule) in context.map.rules.iter().enumerate() {
        let TrafficRule::TrafficLight { lanes, .. } = rule else {
            continue;
        };
        for lane in context.map.signal_carriers(lanes) {
            let Some(&id) = context.ids.get(&lane) else {
                continue;
            };
            trace.link_as(
                IrRef::Rule(index),
                format!("signal:{id}"),
                Relation::Part,
                "traffic_light",
            );
            if seen.insert(id) {
                states.push(pb::TrafficSignalLaneState {
                    lane: Some(id),
                    state: Some(traffic_signal_lane_state::State::LaneStateUnknown as i32),
                    stop_point: context.polylines[id as usize - 1].first().cloned(),
                });
            }
        }
    }
    states
}

fn last_index(polyline: &[pb::MapPoint]) -> i32 {
    polyline.len().saturating_sub(1) as i32
}

fn point(point: Point3) -> pb::MapPoint {
    pb::MapPoint {
        x: Some(point.x),
        y: Some(point.y),
        z: Some(point.z),
    }
}

fn points(points: &[Point3]) -> Vec<pb::MapPoint> {
    points.iter().copied().map(point).collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_white_passing_line_has_no_womd_type() {
        assert_eq!(
            line_type(BoundaryMarking::new(
                RoadMarking::SolidBroken,
                MarkingColor::White
            )),
            road_line::RoadLineType::TypeUnknown
        );
        assert_eq!(
            line_type(BoundaryMarking::new(
                RoadMarking::SolidBroken,
                MarkingColor::Yellow
            )),
            road_line::RoadLineType::TypePassingDoubleYellow
        );
    }
}
