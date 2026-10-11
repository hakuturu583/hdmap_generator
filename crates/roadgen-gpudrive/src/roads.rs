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
//!
//! **A road edge stops where another road passes over or under it.** The scene is a
//! plane, so a motorway on a bridge and the street beneath it are drawn on top of
//! each other, and each road's edges cross the other's lanes. Each edge is cut
//! where it lies on the other road of a grade separation, so an agent on either
//! road can drive through the crossing without hitting the other's wall.
//!
//! Every element also records, as it is pushed, which IR elements it came from. The
//! row's index is the only identifier GPUDrive keeps, so the push is the one place the
//! correspondence is known; the [`Trace`] names an element `road:<id>`.

use std::collections::BTreeMap;

use roadgen_core::geometry::plan::{ring_contains, PlanPoint};
use roadgen_core::geometry::{Curve3, Point3, SamplingConfig};
use roadgen_core::map::Lane;
use roadgen_core::semantics::{
    BoundaryMarking, MapObject, MapObjectKind, MarkingColor, ObjectGeometry, RoadMarking, RoadType,
};
use roadgen_core::structure::{self, RoadSpan, Underneath};
use roadgen_core::{IrRef, LaneType, Relation, RoadId, Trace, ValidatedMap};

use crate::error::ExportError;
use crate::scene::{MapElement, Road, RoadKind, Vector2};

/// Every road element of the scene, in the order they are written: lanes first, then
/// the edges and lines of each road, then the crossings and signs.
pub fn all(map: &ValidatedMap) -> Result<Vec<Road>, ExportError> {
    all_traced(map, &mut Trace::new(crate::TRACE_FORMAT))
}

/// The same, recording in `trace` which IR elements each road element came from.
pub fn all_traced(map: &ValidatedMap, trace: &mut Trace) -> Result<Vec<Road>, ExportError> {
    let mut rows = Rows {
        roads: Vec::new(),
        trace,
    };
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

/// The rows under construction, numbered as they are added, and the trace of where
/// each came from.
///
/// The identifier is the row's own index rather than anything from the IR: GPUDrive's
/// ids are `uint32_t` and the IR's are strings, so a stable identifier cannot survive
/// the crossing. `check` says so.
struct Rows<'t> {
    roads: Vec<Road>,
    trace: &'t mut Trace,
}

/// One IR element a road element is drawn from, and how.
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

impl Rows<'_> {
    /// Adds an element drawn from `sources`, linking each of them to it in the trace
    /// under the id it is written with.
    fn push(
        &mut self,
        kind: RoadKind,
        map_element_id: MapElement,
        geometry: Vec<Vector2>,
        sources: Vec<Source>,
    ) {
        // An element with no geometry is one the reader would size as `-1` segments,
        // so it is dropped here rather than written — and has nothing to link to.
        if geometry.is_empty() {
            return;
        }
        let id = self.roads.len() as u32;
        let local = format!("road:{id}");
        for source in sources {
            match source.role {
                Some(role) => self
                    .trace
                    .link_as(source.ir, local.clone(), source.relation, role),
                None => self.trace.link(source.ir, local.clone(), source.relation),
            }
        }
        self.roads.push(Road {
            kind,
            geometry,
            id,
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
            vec![Source::exact(lane.id.clone())],
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
    // A motorway carriageway or a slip road is a freeway lane by its form of way,
    // whatever its road type says about where it runs.
    match map.road(&lane.road) {
        Some(road) if road.form.is_controlled_access() => MapElement::LaneFreeway,
        Some(road) if road.road_type == RoadType::Motorway => MapElement::LaneFreeway,
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

    /// The lanes either side of the edge, each linked to the one element written for
    /// it: `Merged` when two lanes share the edge, `Exact` when only one touches it.
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

/// The boundaries of every cross-section, as one element each — a road edge cut
/// where it passes over or under another road (see [`crossed_ground`]).
fn edges_and_lines(map: &ValidatedMap, rows: &mut Rows) -> Result<(), ExportError> {
    let config = map.metadata.sampling;
    for road in map.roads.iter() {
        // A connector's boundaries are not the edge of anything: see the module
        // documentation.
        if road.is_connector() {
            continue;
        }
        let zones = crossed_ground(map, &road.id)?;
        for section in 0..road.sections.len() {
            for edge in edges_of(&map.lanes_of_section(&road.id, section)).values() {
                let Some((curve, marking)) = edge.boundary() else {
                    continue;
                };
                let Some((kind, element)) = classify(edge, marking) else {
                    continue;
                };
                let role = match kind {
                    RoadKind::RoadLine => "road_line",
                    _ => "road_edge",
                };
                let line = vertices(curve, config)?;
                if kind == RoadKind::RoadEdge && !zones.is_empty() {
                    for piece in cut_out(&line, &zones) {
                        rows.push(kind, element, piece, edge.sources(role));
                    }
                } else {
                    rows.push(kind, element, line, edge.sources(role));
                }
            }
        }
    }
    Ok(())
}

/// How far past the other road's outline a road edge is cut, metres: enough that
/// an agent brushing the other road's kerb does not meet the stub of an edge.
const CUT_MARGIN: f64 = 0.5;

/// How far along the other road its outline is drawn past the stretch the grade
/// separation names, metres. The separation's stretch is as long as the crossing is
/// wide when the roads meet square; a skewed crossing reaches further along each,
/// and an outline drawn a little long cuts nothing it should not, since a road edge
/// only meets it where the two roads overlap in plan.
const CUT_REACH: f64 = 30.0;

/// The ground, in plan, where `road` passes over or under another road: for every
/// grade separation it is on either side of, the other road's outline along the
/// crossing, widened by [`CUT_MARGIN`].
///
/// GPUDrive is a plane, and a `road_edge` is a wall: an agent that touches one has
/// collided. Where a motorway bridges a street, the motorway's edges run straight
/// across the street's lanes in plan, and the street's edges across the motorway's;
/// written whole, they would wall each road off from itself at the crossing. So
/// each road's edges are cut where they lie on the other road — the over road's
/// where it passes above the under road's carriageway, and the under road's where
/// it passes beneath the deck. Both cuts are needed: with either left whole, the
/// agents on the other road drive into it. What is lost is a deck's width of edge on
/// each, where an agent on either road could leave it without hitting anything.
fn crossed_ground(map: &ValidatedMap, road: &RoadId) -> Result<Vec<Vec<PlanPoint>>, ExportError> {
    let mut zones = Vec::new();
    for separation in map.grade_separations.iter() {
        let Underneath::Road(under) = &separation.under else {
            continue;
        };
        let other = if separation.over.road == *road {
            under
        } else if under.road == *road {
            &separation.over
        } else {
            continue;
        };
        if let Some(zone) = outline_along(map, other)? {
            zones.push(zone);
        }
    }
    Ok(zones)
}

/// The outline in plan of a road's full width along a stretch of it, reaching
/// [`CUT_REACH`] past each end and [`CUT_MARGIN`] past each side.
fn outline_along(
    map: &ValidatedMap,
    span: &RoadSpan,
) -> Result<Option<Vec<PlanPoint>>, ExportError> {
    let Some(road) = map.road(&span.road) else {
        return Ok(None);
    };
    let config = map.metadata.sampling;
    let length = road.horizontal_length()?;
    let from = (span.from - CUT_REACH).max(0.0);
    let to = (span.to + CUT_REACH).min(length);
    if to <= from {
        return Ok(None);
    }
    let steps = ((to - from) / 2.0).ceil().max(1.0) as usize;
    let (mut left, mut right) = (Vec::new(), Vec::new());
    for step in 0..=steps {
        let station = from + (to - from) * step as f64 / steps as f64;
        let Some((high, low)) = structure::road_extent(map, &span.road, station) else {
            continue;
        };
        let frame = road.frame_at(station, config)?;
        let (a, b) = (
            frame.offset(high + CUT_MARGIN),
            frame.offset(low - CUT_MARGIN),
        );
        left.push([a.x, a.y]);
        right.push([b.x, b.y]);
    }
    if left.len() < 2 {
        return Ok(None);
    }
    right.reverse();
    left.extend(right);
    Ok(Some(left))
}

/// A polyline with every stretch of it that lies inside one of `zones` taken out:
/// the pieces left, in order, each of two or more vertices.
fn cut_out(line: &[Vector2], zones: &[Vec<PlanPoint>]) -> Vec<Vec<Vector2>> {
    let inside = |p: PlanPoint| zones.iter().any(|zone| ring_contains(zone, p));
    let mut pieces: Vec<Vec<Vector2>> = Vec::new();
    let mut current: Vec<Vector2> = Vec::new();
    let mut close = |current: &mut Vec<Vector2>| {
        if current.len() >= 2 {
            pieces.push(std::mem::take(current));
        } else {
            current.clear();
        }
    };
    for pair in line.windows(2) {
        let (a, b) = ([pair[0].x, pair[0].y], [pair[1].x, pair[1].y]);
        // Where the segment crosses any zone's outline, as fractions along it: the
        // places it can go in or come out.
        let mut cuts = vec![0.0, 1.0];
        for zone in zones {
            for index in 0..zone.len() {
                let (r, s) = (zone[index], zone[(index + 1) % zone.len()]);
                if let Some(t) = crossing_fraction(a, b, r, s) {
                    cuts.push(t);
                }
            }
        }
        cuts.sort_by(f64::total_cmp);
        cuts.dedup_by(|x, y| (*x - *y).abs() < 1e-12);
        let at = |t: f64| Vector2::new(a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t);
        for window in cuts.windows(2) {
            let (t0, t1) = (window[0], window[1]);
            let middle = at((t0 + t1) / 2.0);
            if inside([middle.x, middle.y]) {
                close(&mut current);
                continue;
            }
            let start = at(t0);
            if current.last() != Some(&start) {
                close(&mut current);
                current.push(start);
            }
            current.push(at(t1));
        }
    }
    close(&mut current);
    pieces
}

/// Where segment `a–b` crosses segment `r–s`, as a fraction along `a–b`.
fn crossing_fraction(a: PlanPoint, b: PlanPoint, r: PlanPoint, s: PlanPoint) -> Option<f64> {
    let (dx1, dy1) = (b[0] - a[0], b[1] - a[1]);
    let (dx2, dy2) = (s[0] - r[0], s[1] - r[1]);
    let denominator = dx1 * dy2 - dy1 * dx2;
    if denominator.abs() < 1e-12 {
        return None;
    }
    let (ex, ey) = (r[0] - a[0], r[1] - a[1]);
    let t = (ex * dy2 - ey * dx2) / denominator;
    let u = (ex * dy1 - ey * dx1) / denominator;
    ((0.0..=1.0).contains(&t) && (0.0..=1.0).contains(&u)).then_some(t)
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
            rows.push(
                RoadKind::Crosswalk,
                MapElement::Crosswalk,
                polygon,
                vec![Source::exact(object.id.clone())],
            );
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
            vec![Source::exact(object.id.clone())],
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
