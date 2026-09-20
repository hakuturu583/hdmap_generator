//! The paint of a crosswalk, and of a stop line.
//!
//! A crosswalk in the IR is a band across the road: a rectangle the width of the
//! crossing, reaching from kerb to kerb and beyond, and the lanes it governs. What a
//! camera sees of one is its stripes, so that is what the surface paints: bars of
//! white the length of the crossing, laid along the road, every metre across the
//! carriageway and only the carriageway — the pavements at either end are where the
//! crossing leads, not part of it. They stand off the road the way every other
//! marking does, and are named as markings, so CARLA tags them `RoadLines`.
//!
//! A stop line is a line across one lane in the IR, and is painted as a bar
//! [`STOP_LINE_DEPTH`] deep on the traffic's side of it — the side a driver stops
//! on — so that where CARLA halts a vehicle has a line on the road to be halted at.

use roadgen_core::map::{Map, Road};
use roadgen_core::semantics::{MapObject, MapObjectKind, ObjectGeometry};
use roadgen_core::topology::Direction;
use roadgen_opendrive::road_coordinates;

use crate::materials;
use crate::mesh::Mesh;
use crate::surfaces::{Layout, Ordinals, SurfaceConfig};
use crate::tags::{mesh_name, Role};

/// How wide each painted bar is, and the gap after it, metres.
pub const BAR: f64 = 0.5;
pub const GAP: f64 = 0.5;

/// How far a stop line's paint reaches back from the line, metres.
pub const STOP_LINE_DEPTH: f64 = 0.45;

/// Paints every crosswalk and every stop line in the map, one mesh each.
pub fn paint(
    map: &Map,
    map_name: &str,
    config: &SurfaceConfig,
    ordinals: &mut Ordinals,
    out: &mut Vec<Mesh>,
) {
    for object in map.objects.iter() {
        let Some(lane) = object.lanes.first().and_then(|lane| map.lanes.get(lane)) else {
            continue;
        };
        let Some(road) = map.road(&lane.road) else {
            continue;
        };
        let mesh = match object.kind {
            MapObjectKind::Crosswalk => bars(map, road, object, map_name, config, ordinals),
            MapObjectKind::StopLine => {
                stop_line(map, road, lane, object, map_name, config, ordinals)
            }
            _ => None,
        };
        if let Some(mesh) = mesh {
            out.push(mesh);
        }
    }
}

/// The paint of one stop line: a bar across the lane, reaching back from the
/// line against the traffic's travel.
fn stop_line(
    map: &Map,
    road: &Road,
    lane: &roadgen_core::map::Lane,
    object: &MapObject,
    map_name: &str,
    config: &SurfaceConfig,
    ordinals: &mut Ordinals,
) -> Option<Mesh> {
    let ObjectGeometry::Line(curve) = &object.geometry else {
        return None;
    };
    let ends = [
        road_coordinates::locate(map, road, curve.start_point())?,
        road_coordinates::locate(map, road, curve.end_point())?,
    ];
    let station = (ends[0].s + ends[1].s) / 2.0;
    let (outer, inner) = (ends[0].t.max(ends[1].t), ends[0].t.min(ends[1].t));
    let back = match lane.direction {
        Direction::Forward => -STOP_LINE_DEPTH,
        Direction::Backward => STOP_LINE_DEPTH,
    };
    let length = road.horizontal_length().ok()?;
    let sampling = map.metadata.sampling;
    let (at, behind) = (
        road.frame_at(station.clamp(0.0, length), sampling).ok()?,
        road.frame_at((station + back).clamp(0.0, length), sampling)
            .ok()?,
    );
    let rise = config.marking_rise;
    let mut mesh = Mesh::new(
        mesh_name(map_name, Role::Marking, ordinals.take(Role::Marking)),
        Role::Marking,
        materials::MARKING_WHITE,
    );
    // Rails run along the road so the strip faces up whichever way the lane goes:
    // the left rail on the left of the right rail, seen from above, is the one at
    // the greater lateral offset.
    let (first, second) = if back < 0.0 {
        (behind, at)
    } else {
        (at, behind)
    };
    mesh.strip(
        &[
            first.to_global([0.0, outer, rise]),
            second.to_global([0.0, outer, rise]),
        ],
        &[
            first.to_global([0.0, inner, rise]),
            second.to_global([0.0, inner, rise]),
        ],
        0,
    );
    (!mesh.is_empty()).then_some(mesh)
}

/// The bars of one crosswalk, or `None` when there is nowhere to paint them.
fn bars(
    map: &Map,
    road: &Road,
    object: &MapObject,
    map_name: &str,
    config: &SurfaceConfig,
    ordinals: &mut Ordinals,
) -> Option<Mesh> {
    let ObjectGeometry::Band { left, right } = &object.geometry else {
        return None;
    };
    // Where the band is along the road, and how far it reaches either way.
    let near = road_coordinates::locate(map, road, left.start_point().lerp(left.end_point(), 0.5))?;
    let far =
        road_coordinates::locate(map, road, right.start_point().lerp(right.end_point(), 0.5))?;
    let (from, to) = (near.s.min(far.s), near.s.max(far.s));
    let station = (from + to) / 2.0;

    // The carriageway at that station: from the outer edge of the outermost driving
    // lane on one side to the other. The cuts run left to right.
    let layout = Layout::of(map, road, road.section_at(station)?, config)?;
    let (outer_left, outer_right) = layout.carriageway(station)?;

    let sampling = map.metadata.sampling;
    let (start, end) = (
        road.frame_at(from, sampling).ok()?,
        road.frame_at(to, sampling).ok()?,
    );
    let rise = config.marking_rise;

    let mut mesh = Mesh::new(
        mesh_name(map_name, Role::Marking, ordinals.take(Role::Marking)),
        Role::Marking,
        materials::MARKING_WHITE,
    );
    // Bars from the left kerb to the right, each `BAR` wide with `GAP` after it,
    // and the same margin at both ends so that the pattern is centred.
    let span = outer_left - outer_right;
    let count = ((span + GAP) / (BAR + GAP)).floor().max(0.0) as usize;
    let margin = (span - (count as f64 * (BAR + GAP) - GAP)) / 2.0;
    for index in 0..count {
        let outer = outer_left - margin - index as f64 * (BAR + GAP);
        let inner = outer - BAR;
        let left_rail = [
            start.to_global([0.0, outer, rise]),
            end.to_global([0.0, outer, rise]),
        ];
        let right_rail = [
            start.to_global([0.0, inner, rise]),
            end.to_global([0.0, inner, rise]),
        ];
        mesh.strip(&left_rail, &right_rail, 0);
    }
    (!mesh.is_empty()).then_some(mesh)
}
