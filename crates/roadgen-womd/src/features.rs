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
//!   surface or a painted line between lanes, not both.
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

use std::collections::{BTreeMap, HashMap};

use roadgen_core::geometry::{Curve3, Point3, SamplingConfig};
use roadgen_core::map::Lane;
use roadgen_core::semantics::{
    BoundaryMarking, MapObject, MapObjectKind, MarkingColor, ObjectGeometry, RoadMarking, RoadType,
    TrafficRule,
};
use roadgen_core::{Direction, IrRef, LaneId, LaneType, Relation, RoadId, Trace, ValidatedMap};

use crate::error::ExportError;
use crate::pb::{
    self, lane_center, map_feature::FeatureData, road_edge, road_line, traffic_signal_lane_state,
};

/// Metres per second in one mile per hour, which is how WOMD holds a speed limit.
const MPS_PER_MPH: f64 = 0.447_04;

/// The map features of a scenario, and the number each written lane was given.
pub struct Written {
    /// In the order they are written; feature `n` is at index `n - 1`.
    pub features: Vec<pb::MapFeature>,
    pub lanes: HashMap<LaneId, i64>,
}

/// Every map feature of the scenario, in the order they are written.
pub fn all_traced(map: &ValidatedMap, trace: &mut Trace) -> Result<Written, ExportError> {
    let config = map.metadata.sampling;
    let mut out = Features {
        features: Vec::new(),
        trace,
    };

    // The lanes are numbered before anything is built, because a lane names the
    // features either side of it and they name nothing back: the lanes come first in
    // the scenario all the same, which is where a reader looks for them.
    let lanes: Vec<&Lane> = map
        .lanes
        .iter()
        .filter(|lane| lane.lane_type.is_drivable())
        .collect();
    let ids: HashMap<&LaneId, i64> = lanes
        .iter()
        .enumerate()
        .map(|(index, lane)| (&lane.id, index as i64 + 1))
        .collect();

    let mut boundaries = Vec::new();
    let edges = edges_and_lines(map, config, lanes.len() as i64 + 1, &mut boundaries)?;

    for lane in &lanes {
        let feature = lane_feature(map, lane, &ids, &edges, config)?;
        out.push(feature, vec![Source::exact(lane.id.clone())]);
    }
    for (feature, sources) in boundaries {
        out.push(feature, sources);
    }
    crosswalks(map, config, &mut out)?;
    stop_signs(map, &ids, &mut out);
    let features = out.features;
    let lanes = ids
        .into_iter()
        .map(|(lane, id)| (lane.clone(), id))
        .collect();
    Ok(Written { features, lanes })
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

/// A lane: its centreline in travel order, and everything it says about the lanes
/// and lines around it.
fn lane_feature(
    map: &ValidatedMap,
    lane: &Lane,
    ids: &HashMap<&LaneId, i64>,
    edges: &EdgeFeatures,
    config: SamplingConfig,
) -> Result<FeatureData, ExportError> {
    let polyline = points(lane.travel_polyline(config)?.points());
    let last = polyline.len().saturating_sub(1) as i32;
    let written = |lanes: Vec<LaneId>| -> Vec<i64> {
        lanes
            .iter()
            .filter_map(|lane| ids.get(lane).copied())
            .collect()
    };

    // Which edge is on the driver's left: the lane's own left for a lane that runs
    // with the road's reference line, its right for one that runs against it.
    let (left, right) = if lane.direction == Direction::Forward {
        (lane.left_edge, lane.right_edge)
    } else {
        (lane.right_edge, lane.left_edge)
    };
    let boundary = |edge: i32| -> Vec<pb::BoundarySegment> {
        edges
            .get(&(lane.road.clone(), lane.section, edge))
            .map(|written| vec![written.segment(last)])
            .unwrap_or_default()
    };
    let neighbors = |edge: i32| -> Result<Vec<pb::LaneNeighbor>, ExportError> {
        let Some(other) = across(map, lane, edge) else {
            return Ok(Vec::new());
        };
        let Some(&feature_id) = ids.get(&other.id) else {
            return Ok(Vec::new());
        };
        let written = edges.get(&(lane.road.clone(), lane.section, edge));
        // A kerb between two lanes is a median: they are side by side, but nobody
        // changes lanes across it.
        if written.is_some_and(|written| written.is_edge) {
            return Ok(Vec::new());
        }
        let theirs = other
            .travel_polyline(config)?
            .points()
            .len()
            .saturating_sub(1) as i32;
        Ok(vec![pb::LaneNeighbor {
            feature_id: Some(feature_id),
            self_start_index: Some(0),
            self_end_index: Some(last),
            neighbor_start_index: Some(0),
            neighbor_end_index: Some(theirs),
            boundaries: written
                .map(|written| vec![written.segment(last)])
                .unwrap_or_default(),
        }])
    };

    Ok(FeatureData::Lane(pb::LaneCenter {
        speed_limit_mph: speed_limit(map, lane).map(|mps| mps / MPS_PER_MPH),
        r#type: Some(lane_type(map, lane) as i32),
        interpolating: Some(map.road(&lane.road).is_some_and(|road| road.is_connector())),
        polyline,
        entry_lanes: written(map.predecessors(&lane.id)),
        exit_lanes: written(map.successors(&lane.id)),
        left_boundaries: boundary(left),
        right_boundaries: boundary(right),
        left_neighbors: neighbors(left)?,
        right_neighbors: neighbors(right)?,
    }))
}

/// The lane on the other side of `edge` from `lane` in its cross-section, when it is
/// one going the same way: WOMD's neighbours are only those.
fn across<'a>(map: &'a ValidatedMap, lane: &Lane, edge: i32) -> Option<&'a Lane> {
    map.lanes_of_section(&lane.road, lane.section)
        .into_iter()
        .find(|other| {
            other.id != lane.id
                && ((edge == lane.left_edge && other.right_edge == edge)
                    || (edge == lane.right_edge && other.left_edge == edge))
        })
        .filter(|other| other.direction == lane.direction && other.lane_type.is_drivable())
}

/// The lane's limit in metres per second: a rule naming the lane, else the lane's
/// own, which already carries its road's.
fn speed_limit(map: &ValidatedMap, lane: &Lane) -> Option<f64> {
    map.rules
        .iter()
        .find_map(|rule| match rule {
            TrafficRule::SpeedLimit { limit, lanes } if lanes.contains(&lane.id) => Some(*limit),
            _ => None,
        })
        .or(lane.speed_limit)
        .map(|limit| limit.mps())
}

/// Which WOMD lane type a lane is: a bike lane is its own, and the rest of the
/// distinction is the road's — the same reading the GPUDrive export makes.
fn lane_type(map: &ValidatedMap, lane: &Lane) -> lane_center::LaneType {
    use lane_center::LaneType as T;
    if lane.lane_type == LaneType::Biking {
        return T::TypeBikeLane;
    }
    match map.road(&lane.road) {
        Some(road) if road.form.is_controlled_access() => T::TypeFreeway,
        Some(road) if road.road_type == RoadType::Motorway => T::TypeFreeway,
        Some(_) => T::TypeSurfaceStreet,
        None => T::TypeUndefined,
    }
}

/// A boundary feature as the lanes beside it refer to it.
struct EdgeFeature {
    id: i64,
    /// A `RoadEdge` rather than a `RoadLine`.
    is_edge: bool,
    /// The line's type; `TYPE_UNKNOWN` for an edge, as WOMD has it.
    line_type: road_line::RoadLineType,
}

impl EdgeFeature {
    /// The boundary as it runs the whole length of a lane whose last vertex is `last`.
    fn segment(&self, last: i32) -> pb::BoundarySegment {
        pb::BoundarySegment {
            lane_start_index: Some(0),
            lane_end_index: Some(last),
            boundary_feature_id: Some(self.id),
            boundary_type: Some(self.line_type as i32),
        }
    }
}

/// The boundary features, by the road, cross-section and edge index they lie on.
type EdgeFeatures = HashMap<(RoadId, usize, i32), EdgeFeature>;

/// What sits against one cross-section edge.
#[derive(Default)]
struct Edge<'a> {
    /// The lane whose `right_edge` it is.
    left: Option<&'a Lane>,
    /// The lane whose `left_edge` it is.
    right: Option<&'a Lane>,
}

impl<'a> Edge<'a> {
    /// The curve and the paint, from the lane on the right when there is one — the
    /// same fixed choice the GPUDrive export makes.
    fn boundary(&self) -> Option<(&'a Curve3, BoundaryMarking)> {
        self.right
            .map(|lane| (&lane.left_boundary, lane.left_marking))
            .or_else(|| {
                self.left
                    .map(|lane| (&lane.right_boundary, lane.right_marking))
            })
    }

    fn drivable_sides(&self) -> usize {
        [self.left, self.right]
            .iter()
            .filter(|lane| lane.is_some_and(|lane| lane.lane_type.is_drivable()))
            .count()
    }

    fn sources(&self, role: &'static str) -> Vec<Source> {
        let lanes: Vec<&Lane> = [self.left, self.right].into_iter().flatten().collect();
        let relation = Relation::shared_by(lanes.len());
        lanes
            .into_iter()
            .map(|lane| Source {
                ir: IrRef::Lane(lane.id.clone()),
                relation,
                role: Some(role),
            })
            .collect()
    }
}

/// What a boundary is in WOMD's terms.
enum Boundary {
    Edge(road_edge::RoadEdgeType),
    Line(road_line::RoadLineType),
}

/// The edges and lines of every cross-section, numbered from `first`, into
/// `features` with their sources; and how the lanes beside each find it.
fn edges_and_lines(
    map: &ValidatedMap,
    config: SamplingConfig,
    first: i64,
    features: &mut Vec<(FeatureData, Vec<Source>)>,
) -> Result<EdgeFeatures, ExportError> {
    let mut written = EdgeFeatures::new();
    for road in map.roads.iter() {
        if road.is_connector() {
            continue;
        }
        for section in 0..road.sections.len() {
            let lanes = map.lanes_of_section(&road.id, section);
            for (&index, edge) in &edges_of(&lanes) {
                let Some((curve, marking)) = edge.boundary() else {
                    continue;
                };
                let Some(boundary) = classify(edge, marking) else {
                    continue;
                };
                let polyline = points(curve.to_polyline(config)?.points());
                if polyline.is_empty() {
                    continue;
                }
                let id = first + features.len() as i64;
                let (data, feature, role) = match boundary {
                    Boundary::Edge(kind) => (
                        FeatureData::RoadEdge(pb::RoadEdge {
                            r#type: Some(kind as i32),
                            polyline,
                        }),
                        EdgeFeature {
                            id,
                            is_edge: true,
                            line_type: road_line::RoadLineType::TypeUnknown,
                        },
                        "road_edge",
                    ),
                    Boundary::Line(kind) => (
                        FeatureData::RoadLine(pb::RoadLine {
                            r#type: Some(kind as i32),
                            polyline,
                        }),
                        EdgeFeature {
                            id,
                            is_edge: false,
                            line_type: kind,
                        },
                        "road_line",
                    ),
                };
                features.push((data, edge.sources(role)));
                written.insert((road.id.clone(), section, index), feature);
            }
        }
    }
    Ok(written)
}

/// The edges of one cross-section, keyed by edge index.
fn edges_of<'a>(lanes: &[&'a Lane]) -> BTreeMap<i32, Edge<'a>> {
    let mut edges: BTreeMap<i32, Edge<'a>> = BTreeMap::new();
    for lane in lanes {
        edges.entry(lane.left_edge).or_default().right = Some(lane);
        edges.entry(lane.right_edge).or_default().left = Some(lane);
    }
    edges
}

/// What an edge becomes, or `None` when WOMD has nothing to say about it.
fn classify(edge: &Edge<'_>, marking: BoundaryMarking) -> Option<Boundary> {
    use road_edge::RoadEdgeType as E;
    match edge.drivable_sides() {
        0 => None,
        1 => Some(Boundary::Edge(E::TypeRoadEdgeBoundary)),
        _ => match marking.marking {
            RoadMarking::Curbstone => Some(Boundary::Edge(E::TypeRoadEdgeMedian)),
            RoadMarking::None => None,
            _ => Some(Boundary::Line(line_type(marking))),
        },
    }
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
    for object in objects_of(map, |kind| matches!(kind, MapObjectKind::Crosswalk)) {
        if let ObjectGeometry::Band { left, right } = &object.geometry {
            let mut polygon = points(left.to_polyline(config)?.points());
            let mut back = points(right.to_polyline(config)?.points());
            back.reverse();
            polygon.extend(back);
            out.push(
                FeatureData::Crosswalk(pb::Crosswalk { polygon }),
                vec![Source::exact(object.id.clone())],
            );
        }
    }
    Ok(())
}

/// A stop sign, at the middle of the bar the IR hangs it on, naming the lanes it
/// controls.
fn stop_signs(map: &ValidatedMap, ids: &HashMap<&LaneId, i64>, out: &mut Features) {
    for object in objects_of(map, |kind| match kind {
        MapObjectKind::TrafficSign { code } => is_stop_sign(code),
        _ => false,
    }) {
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

/// Whether a sign's catalogue code names a stop sign: the spellings the GPUDrive
/// export recognises, since the IR defines no catalogue of its own.
pub fn is_stop_sign(code: &str) -> bool {
    let code = code.trim().to_ascii_lowercase().replace([' ', '_'], "-");
    code.starts_with("stop") || code == "r1-1" || code == "206"
}

/// One step's signal states: a lane state for every lane through a junction a
/// traffic light governs, stopping where the lane begins.
///
/// The IR's traffic-light rule names the approaches, the lanes that stop for the
/// light. WOMD gives a signal's state to the lane through the intersection, with
/// the stop point at its mouth, so each approach hands the state on to the junction
/// connectors it feeds — the reading the Lanelet2 export makes — and keeps it
/// itself only when it feeds none. The IR holds no timing, so every state is
/// `LANE_STATE_UNKNOWN`: what a light shows is the simulator's to decide.
pub fn signal_states(
    map: &ValidatedMap,
    written: &Written,
    trace: &mut Trace,
) -> Vec<pb::TrafficSignalLaneState> {
    let mut states: Vec<pb::TrafficSignalLaneState> = Vec::new();
    let mut links: Vec<(usize, i64)> = Vec::new();
    for (index, rule) in map.rules.iter().enumerate() {
        let TrafficRule::TrafficLight { lanes, .. } = rule else {
            continue;
        };
        for lane in junction_lanes(map, lanes) {
            let Some(&id) = written.lanes.get(&lane) else {
                continue;
            };
            let Some(FeatureData::Lane(center)) = written
                .features
                .get(id as usize - 1)
                .and_then(|feature| feature.feature_data.as_ref())
            else {
                continue;
            };
            links.push((index, id));
            if states.iter().any(|state| state.lane == Some(id)) {
                continue;
            }
            states.push(pb::TrafficSignalLaneState {
                lane: Some(id),
                state: Some(traffic_signal_lane_state::State::LaneStateUnknown as i32),
                stop_point: center.polyline.first().cloned(),
            });
        }
    }
    for (index, id) in links {
        trace.link_as(
            IrRef::Rule(index),
            format!("signal:{id}"),
            Relation::Part,
            "traffic_light",
        );
    }
    states
}

/// The lanes a traffic light's state is given to, for a rule over `lanes`.
fn junction_lanes(map: &ValidatedMap, lanes: &[LaneId]) -> Vec<LaneId> {
    let connector = |lane: &LaneId| {
        map.lane(lane)
            .and_then(|lane| map.road(&lane.road))
            .is_some_and(|road| road.is_connector())
    };
    let mut carriers: Vec<LaneId> = Vec::new();
    for lane in lanes {
        let through: Vec<LaneId> = if connector(lane) {
            Vec::new()
        } else {
            map.successors(lane)
                .into_iter()
                .filter(|next| connector(next))
                .collect()
        };
        let held = if through.is_empty() {
            vec![lane.clone()]
        } else {
            through
        };
        for lane in held {
            if !carriers.contains(&lane) {
                carriers.push(lane);
            }
        }
    }
    carriers
}

pub(crate) fn objects_of<'a>(
    map: &'a ValidatedMap,
    wanted: impl Fn(&MapObjectKind) -> bool + 'a,
) -> impl Iterator<Item = &'a MapObject> {
    map.objects
        .iter()
        .filter(move |object| wanted(&object.kind))
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
