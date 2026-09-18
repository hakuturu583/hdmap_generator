//! Turning the IR's roads into GPUDrive road elements.
//!
//! GPUDrive's map vocabulary is Waymo Open Motion's: a lane is a *centreline*, a
//! painted marking is a `road_line`, the edge of the drivable surface is a
//! `road_edge`, and a crossing, a hump or a stop sign is its own element. There is no
//! lane width, no cross-section and no topology anywhere in it — a lane does not say
//! what it leads to — so what is written here is the map as a set of polylines and
//! nothing more.
//!
//! Two decisions are worth stating, because neither falls out of the IR on its own.
//!
//! **One element per cross-section edge.** An edge is either the boundary of the
//! drivable surface or a painted line between lanes, not both. Waymo's own maps carry
//! an edge line and a road edge as separate features; the IR has one curve there, and
//! writing it twice would put two elements on top of each other and spend twice the
//! simulator's road budget on it.
//!
//! **A junction connector has no edges.** Its lane centreline is written, because that
//! is the path through the intersection; its boundaries are not, because a
//! `road_edge` is something an agent collides with and there is no wall down the
//! middle of a junction.

use std::collections::BTreeMap;

use roadgen_core::geometry::{Curve3, Point3, SamplingConfig};
use roadgen_core::map::Lane;
use roadgen_core::semantics::{
    BoundaryMarking, MapObject, MapObjectKind, MarkingColor, ObjectGeometry, RoadMarking, RoadType,
};
use roadgen_core::{LaneType, ValidatedMap};

use crate::error::ExportError;
use crate::scene::{MapElement, Road, RoadKind, Vector2};

/// Every road element of the scene, in the order they are written: lanes first, then
/// the edges and lines of each road, then the crossings and signs.
pub fn all(map: &ValidatedMap) -> Result<Vec<Road>, ExportError> {
    let mut rows = Rows::default();
    lanes(map, &mut rows)?;
    edges_and_lines(map, &mut rows)?;
    crosswalks(map, &mut rows)?;
    stop_signs(map, &mut rows)?;
    Ok(rows.finish())
}

/// Whether a sign's catalogue code names a stop sign.
///
/// The IR does not define a sign catalogue — `code` is whatever the caller uses — so
/// this recognises the spellings a caller is likely to have written, and nothing
/// else. GPUDrive's map vocabulary has exactly one sign in it, so a sign this does not
/// recognise has nowhere to go; [`crate::check`] says how many were left out.
pub fn is_stop_sign(code: &str) -> bool {
    let code = code.trim().to_ascii_lowercase().replace([' ', '_'], "-");
    // "stop" and "stop-sign" are the plain spellings; R1-1 is the MUTCD code and 206
    // the Vienna Convention one.
    code.starts_with("stop") || code == "r1-1" || code == "206"
}

/// The rows under construction, numbered as they are added.
///
/// The identifier is the row's own index rather than anything from the IR: GPUDrive's
/// ids are `uint32_t` and the IR's are strings, so a stable identifier cannot survive
/// the crossing. `check` says so.
#[derive(Default)]
struct Rows {
    roads: Vec<Road>,
}

impl Rows {
    fn push(&mut self, kind: RoadKind, map_element_id: MapElement, geometry: Vec<Vector2>) {
        // An element with no geometry is one the reader would size as `-1` segments,
        // so it is dropped here rather than written.
        if geometry.is_empty() {
            return;
        }
        self.roads.push(Road {
            kind,
            geometry,
            id: self.roads.len() as u32,
            map_element_id,
        });
    }

    fn finish(self) -> Vec<Road> {
        self.roads
    }
}

/// A lane is its centreline, in travel order.
fn lanes(map: &ValidatedMap, rows: &mut Rows) -> Result<(), ExportError> {
    let config = map.metadata.sampling;
    for lane in map.lanes.iter().filter(|lane| lane.lane_type.is_drivable()) {
        let centerline = lane.travel_polyline(config)?;
        rows.push(
            RoadKind::Lane,
            lane_element(map, lane),
            flatten(centerline.points()),
        );
    }
    Ok(())
}

/// Which Waymo lane code a lane carries: the bike lane is its own, and the rest of
/// the distinction the code makes is the road's, not the lane's.
fn lane_element(map: &ValidatedMap, lane: &Lane) -> MapElement {
    if lane.lane_type == LaneType::Biking {
        return MapElement::LaneBikeLane;
    }
    match map.road(&lane.road).map(|road| road.road_type) {
        Some(RoadType::Motorway) => MapElement::LaneFreeway,
        Some(_) => MapElement::LaneSurfaceStreet,
        None => MapElement::LaneUndefined,
    }
}

/// What sits against one cross-section edge, from the edge's own point of view.
#[derive(Default)]
struct Edge<'a> {
    /// The lane to the left of the edge, which is the one whose `right_edge` it is.
    left: Option<&'a Lane>,
    /// The lane to the right of it: the one whose `left_edge` it is.
    right: Option<&'a Lane>,
}

impl<'a> Edge<'a> {
    /// The curve and the paint, from whichever lane bounds the edge.
    ///
    /// The lane on the right is asked first — an arbitrary but fixed choice, and the
    /// two answers differ only in a map that said two different things about one line.
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
}

/// The boundaries of every cross-section, as one element each.
fn edges_and_lines(map: &ValidatedMap, rows: &mut Rows) -> Result<(), ExportError> {
    let config = map.metadata.sampling;
    for road in map.roads.iter() {
        // A connector's boundaries are not the edge of anything: see the module
        // documentation.
        if road.is_connector() {
            continue;
        }
        for section in 0..road.sections.len() {
            for edge in edges_of(&map.lanes_of_section(&road.id, section)).values() {
                let Some((curve, marking)) = edge.boundary() else {
                    continue;
                };
                let Some((kind, element)) = classify(edge, marking) else {
                    continue;
                };
                rows.push(kind, element, vertices(curve, config)?);
            }
        }
    }
    Ok(())
}

/// The edges of one cross-section, keyed by edge index so that two lanes meeting at
/// one produce one element rather than two.
fn edges_of<'a>(lanes: &[&'a Lane]) -> BTreeMap<i32, Edge<'a>> {
    let mut edges: BTreeMap<i32, Edge<'a>> = BTreeMap::new();
    for lane in lanes {
        edges.entry(lane.left_edge).or_default().right = Some(lane);
        edges.entry(lane.right_edge).or_default().left = Some(lane);
    }
    edges
}

/// What an edge becomes, or `None` when GPUDrive has nothing to say about it.
fn classify(edge: &Edge<'_>, marking: BoundaryMarking) -> Option<(RoadKind, MapElement)> {
    match edge.drivable_sides() {
        // Out in the verge or between two footways: nothing an agent drives on, and
        // nothing GPUDrive's vocabulary has a word for.
        0 => None,
        // The drivable surface ends here, whatever is painted on it.
        1 => Some((RoadKind::RoadEdge, MapElement::RoadEdgeBoundary)),
        _ => match marking.marking {
            // A kerb between two drivable lanes is a median: something an agent hits
            // rather than something it may cross.
            RoadMarking::Curbstone => Some((RoadKind::RoadEdge, MapElement::RoadEdgeMedian)),
            RoadMarking::None => None,
            _ => Some((RoadKind::RoadLine, line_element(marking))),
        },
    }
}

/// Which Waymo line code a marking is written as.
fn line_element(marking: BoundaryMarking) -> MapElement {
    match (marking.marking, marking.color) {
        (RoadMarking::Solid, MarkingColor::White) => MapElement::RoadLineSolidSingleWhite,
        (RoadMarking::Broken, MarkingColor::White) => MapElement::RoadLineBrokenSingleWhite,
        (RoadMarking::SolidSolid, MarkingColor::White) => MapElement::RoadLineSolidDoubleWhite,
        (RoadMarking::Solid, MarkingColor::Yellow) => MapElement::RoadLineSolidSingleYellow,
        (RoadMarking::Broken, MarkingColor::Yellow) => MapElement::RoadLineBrokenSingleYellow,
        (RoadMarking::SolidSolid, MarkingColor::Yellow) => MapElement::RoadLineSolidDoubleYellow,
        (RoadMarking::BrokenSolid | RoadMarking::SolidBroken, MarkingColor::Yellow) => {
            MapElement::RoadLinePassingDoubleYellow
        }
        // A broken-and-solid pair painted white has no Waymo code — the vocabulary
        // only has the yellow passing line — so it goes through as the unknown line
        // rather than as a colour it is not.
        (RoadMarking::BrokenSolid | RoadMarking::SolidBroken, MarkingColor::White) => {
            MapElement::RoadLineUnknown
        }
        // Neither reaches here: both are decided before the marking is looked up.
        (RoadMarking::None | RoadMarking::Curbstone, _) => MapElement::RoadLineUnknown,
    }
}

/// A crossing, as the outline of its strip: up one edge and back down the other.
fn crosswalks(map: &ValidatedMap, rows: &mut Rows) -> Result<(), ExportError> {
    let config = map.metadata.sampling;
    for object in objects_of(map, |kind| matches!(kind, MapObjectKind::Crosswalk)) {
        if let ObjectGeometry::Band { left, right } = &object.geometry {
            let mut polygon = vertices(left, config)?;
            let mut back = vertices(right, config)?;
            back.reverse();
            polygon.extend(back);
            rows.push(RoadKind::Crosswalk, MapElement::Crosswalk, polygon);
        }
    }
    Ok(())
}

/// A stop sign, as the single point GPUDrive holds it at: the middle of the bar the
/// IR hangs it on, which is where it applies rather than where the post stands.
fn stop_signs(map: &ValidatedMap, rows: &mut Rows) -> Result<(), ExportError> {
    for object in objects_of(map, |kind| match kind {
        MapObjectKind::TrafficSign { code } => is_stop_sign(code),
        _ => false,
    }) {
        let point = match &object.geometry {
            ObjectGeometry::Point(point) => *point,
            ObjectGeometry::Line(bar) => bar.start_point().lerp(bar.end_point(), 0.5),
            ObjectGeometry::Band { .. } => continue,
        };
        rows.push(
            RoadKind::StopSign,
            MapElement::StopSign,
            vec![Vector2::new(point.x, point.y)],
        );
    }
    Ok(())
}

pub(crate) fn objects_of<'a>(
    map: &'a ValidatedMap,
    wanted: impl Fn(&MapObjectKind) -> bool + 'a,
) -> impl Iterator<Item = &'a MapObject> {
    map.objects
        .iter()
        .filter(move |object| wanted(&object.kind))
}

/// A curve's vertices, with the height dropped: GPUDrive is a plane.
fn vertices(curve: &Curve3, config: SamplingConfig) -> Result<Vec<Vector2>, ExportError> {
    Ok(flatten(curve.to_polyline(config)?.points()))
}

fn flatten(points: &[Point3]) -> Vec<Vector2> {
    points
        .iter()
        .map(|point| Vector2::new(point.x, point.y))
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_spellings_a_caller_writes_a_stop_sign_as_are_recognised() {
        for code in ["stop", "STOP", "Stop_Sign", "R1-1", "206"] {
            assert!(is_stop_sign(code), "{code}");
        }
        for code in ["yield", "R1-2", "speed_limit_50", ""] {
            assert!(!is_stop_sign(code), "{code}");
        }
    }

    #[test]
    fn a_white_passing_line_has_no_waymo_code_and_says_so() {
        assert_eq!(
            line_element(BoundaryMarking::new(
                RoadMarking::SolidBroken,
                MarkingColor::White
            )),
            MapElement::RoadLineUnknown
        );
        assert_eq!(
            line_element(BoundaryMarking::new(
                RoadMarking::SolidBroken,
                MarkingColor::Yellow
            )),
            MapElement::RoadLinePassingDoubleYellow
        );
    }
}
