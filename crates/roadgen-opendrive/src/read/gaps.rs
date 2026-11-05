//! The hairline gaps a document leaves between what it links, closed.
//!
//! A document from another tool can link two roads, or two lanes, whose ends miss
//! each other by a rounding error: CARLA's maps store a road's start in single
//! precision, so a road can begin a millimetre and a half from the end of the road
//! it continues. Validation calls that a gap, and rightly — a Lanelet2 map expresses
//! continuity through shared points, so lanes a millimetre apart are two lanes that
//! do not meet. But the document plainly means them to meet, so a gap of up to
//! [`SNAP_TOLERANCE`] is closed the way the reader closes one between two pieces of
//! one road, and reported. A wider gap is left for validation to find: that is not
//! rounding.
//!
//! Two things are moved, at two moments:
//!
//! - before the roads are laid out, a road whose end is linked to another road is
//!   moved, whole, onto that road's end ([`closing_shifts`]). Whole, so that nothing
//!   along it changes shape — which is only possible if its other end is not linked
//!   to a road as well, since that end would open the same gap again. The lanes are
//!   then laid out where the road now is, and meet the neighbour's.
//! - after the movements are read, a lane that still misses the lane it continues
//!   has its end moved onto that lane's ([`close_lane_gaps`]) — the lane of a
//!   junction's connecting road where there is one, since that road is there only to
//!   join the two, and otherwise the lane entered. The lanes beside it that share the
//!   moved edge move with it, so the cross-section stays one.

use std::collections::{HashMap, HashSet};

use opendrive::core::OpenDrive;

use roadgen_core::geometry::{Curve3, Point3, Polyline3, SamplingConfig, Vector3};
use roadgen_core::id::RoadId;
use roadgen_core::map::Map;
use roadgen_core::topology::{Direction, LaneEnd, RoadEnd, RoadLink, RoadLinkTarget};

use super::geometry::{self, SNAP_TOLERANCE};
use super::{links, Approximations, Names};
use crate::error::ImportError;

/// Gaps narrower than this are left alone, metres: they are the arithmetic's, not
/// the document's, and moving anything over them would only disturb a document
/// whose ends already meet.
const NEGLIGIBLE: f64 = 1e-6;

/// How far each road that misses the road it is linked to is moved. A road that is
/// not moved is not listed.
pub fn closing_shifts(
    document: &OpenDrive,
    names: &Names,
    config: SamplingConfig,
    approximations: &mut Approximations,
) -> Result<HashMap<RoadId, Vector3>, ImportError> {
    // What the roads say, read once here only to find their ends: anything this
    // reading would report, the real one reports again.
    let mut scratch = Approximations::default();
    let mut ends: HashMap<RoadId, (Point3, Point3)> = HashMap::new();
    let mut links: HashMap<RoadId, RoadLink> = HashMap::new();
    for road in &document.road {
        // A junction's connecting road is not one of these: its reference line need
        // not meet the road it joins at all — validation compares its lanes, not its
        // line — so its lanes are what is closed.
        if !matches!(road.junction.trim(), "" | "-1") {
            continue;
        }
        let line = geometry::reference_line(road, Vector3::ZERO, config, &mut scratch)?;
        let id = RoadId::new(&road.id);
        ends.insert(id.clone(), (line.start_point(), line.end_point()));
        links.insert(id, links::road_link(road, names, &mut scratch)?);
    }
    let end_of = |(road, end): (&RoadId, RoadEnd)| {
        let (start, finish) = ends[road];
        match end {
            RoadEnd::Start => start,
            RoadEnd::End => finish,
        }
    };
    // A road can be moved whole only if nothing but this one link holds it in
    // place: its other end meets no road.
    let movable = |(road, end): (&RoadId, RoadEnd)| {
        !matches!(
            links[road].at(end.opposite()),
            Some(RoadLinkTarget::Road(_))
        )
    };

    let mut shifts = HashMap::new();
    let mut seen = HashSet::new();
    let (mut moved, mut widest) = (0, 0.0_f64);
    for road in &document.road {
        let id = &names.roads[&road.id];
        if !links.contains_key(id) {
            continue;
        }
        for end in [RoadEnd::Start, RoadEnd::End] {
            let Some(RoadLinkTarget::Road(other)) = links[id].at(end) else {
                continue;
            };
            let here = (id, end);
            let there = (&other.road, other.end);
            let pair = {
                let (a, b) = (
                    (here.0.as_str(), here.1.as_str()),
                    (there.0.as_str(), there.1.as_str()),
                );
                if a < b {
                    (a, b)
                } else {
                    (b, a)
                }
            };
            if !ends.contains_key(there.0) || !seen.insert(pair) {
                continue;
            }
            let gap = end_of(here).distance_to(end_of(there));
            if gap <= NEGLIGIBLE || gap > SNAP_TOLERANCE {
                continue;
            }
            // The road the link continues into moves onto the one it continues
            // from: the one whose start this is, if either can.
            let order = if here.1 == RoadEnd::Start {
                [(here, there), (there, here)]
            } else {
                [(there, here), (here, there)]
            };
            let Some((mover, anchor)) = order
                .into_iter()
                .find(|(mover, _)| movable(*mover) && !shifts.contains_key(mover.0))
            else {
                continue;
            };
            shifts.insert(mover.0.clone(), end_of(anchor) - end_of(mover));
            moved += 1;
            widest = widest.max(gap);
        }
    }
    if moved > 0 {
        approximations.note(format!(
            "{moved} roads end up to {widest:.4} m from the road they are linked to, and are \
             moved, whole, to meet it"
        ));
    }
    Ok(shifts)
}

/// Moves the end of every lane that misses the lane it continues by no more than
/// [`SNAP_TOLERANCE`] onto that lane's.
pub fn close_lane_gaps(map: &mut Map, approximations: &mut Approximations) {
    let config = map.metadata.sampling;
    let connections: Vec<_> = map.connections.iter().cloned().collect();
    let (mut closed, mut widest) = (0, 0.0_f64);
    for connection in connections {
        let (Some(from), Some(to)) = (
            map.lanes.get(&connection.from.lane),
            map.lanes.get(&connection.to.lane),
        ) else {
            continue;
        };
        let on_connector = |lane: &roadgen_core::map::Lane| {
            map.roads
                .get(&lane.road)
                .is_some_and(|road| road.is_connector())
        };
        let ((mover, mover_end), (anchor, anchor_end)) = if on_connector(from) && !on_connector(to)
        {
            ((from, connection.from.end), (to, connection.to.end))
        } else {
            ((to, connection.to.end), (from, connection.from.end))
        };
        // The two lanes' edges on the same side of travel meet; for a lane that
        // runs against its road, the side of travel is the road's other side.
        let moves: Vec<(i32, Vector3)> = [true, false]
            .into_iter()
            .map(|travel_left| {
                let (rank, here) = edge(mover, travel_left, mover_end);
                let (_, there) = edge(anchor, travel_left, anchor_end);
                (rank, there - here)
            })
            .collect();
        let gap = moves
            .iter()
            .map(|(_, delta)| delta.norm())
            .fold(0.0_f64, f64::max);
        if gap <= NEGLIGIBLE || gap > SNAP_TOLERANCE {
            continue;
        }
        let (road, section) = (mover.road.clone(), mover.section);
        let Some(lanes) = map.roads.get(&road).map(|road| road.lanes.clone()) else {
            continue;
        };
        let mut moved = false;
        for lane in lanes {
            let Some(lane) = map.lanes.get_mut(&lane) else {
                continue;
            };
            if lane.section != section {
                continue;
            }
            for &(rank, delta) in &moves {
                if lane.left_edge == rank {
                    moved |= nudge(&mut lane.left_boundary, mover_end, delta, config);
                    nudge(&mut lane.centerline, mover_end, delta * 0.5, config);
                }
                if lane.right_edge == rank {
                    moved |= nudge(&mut lane.right_boundary, mover_end, delta, config);
                    nudge(&mut lane.centerline, mover_end, delta * 0.5, config);
                }
            }
        }
        if moved {
            closed += 1;
            widest = widest.max(gap);
        }
    }
    if closed > 0 {
        approximations.note(format!(
            "{closed} lane connections join lane ends up to {widest:.4} m apart, and the end \
             of one lane of each is moved to meet the other's"
        ));
    }
}

/// The cross-section edge on one side of a lane's travel, and where it is at `end`.
fn edge(lane: &roadgen_core::map::Lane, travel_left: bool, end: LaneEnd) -> (i32, Point3) {
    let left = travel_left == (lane.direction == Direction::Forward);
    let (rank, curve) = if left {
        (lane.left_edge, &lane.left_boundary)
    } else {
        (lane.right_edge, &lane.right_boundary)
    };
    let point = match end {
        LaneEnd::Start => curve.start_point(),
        LaneEnd::End => curve.end_point(),
    };
    (rank, point)
}

/// Moves one end of a lane's line by `delta`. Says whether it could: a line
/// whose end would land on its neighbouring vertex is left as it is.
fn nudge(curve: &mut Curve3, end: LaneEnd, delta: Vector3, config: SamplingConfig) -> bool {
    let Ok(polyline) = curve.to_polyline(config) else {
        return false;
    };
    let mut points = polyline.points().to_vec();
    let index = match end {
        LaneEnd::Start => 0,
        LaneEnd::End => points.len() - 1,
    };
    points[index] = points[index] + delta;
    match Polyline3::new(points) {
        Ok(moved) if moved.len() == polyline.len() => {
            *curve = Curve3::Polyline(moved);
            true
        }
        _ => false,
    }
}
