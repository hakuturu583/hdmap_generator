//! The paint of a crosswalk.
//!
//! A crosswalk in the IR is a band across the road: a rectangle the width of the
//! crossing, reaching from kerb to kerb and beyond, and the lanes it governs. What a
//! camera sees of one is its stripes, so that is what the surface paints: bars of
//! white the length of the crossing, laid along the road, every metre across the
//! carriageway and only the carriageway — the pavements at either end are where the
//! crossing leads, not part of it. They stand off the road the way every other
//! marking does, and are named as markings, so CARLA tags them `RoadLines`.

use roadgen_core::map::{Map, Road};
use roadgen_core::semantics::{MapObject, MapObjectKind, ObjectGeometry};
use roadgen_opendrive::road_coordinates;

use crate::materials;
use crate::mesh::Mesh;
use crate::surfaces::{Layout, Ordinals, SurfaceConfig};
use crate::tags::{mesh_name, Role};

/// How wide each painted bar is, and the gap after it, metres.
pub const BAR: f64 = 0.5;
pub const GAP: f64 = 0.5;

/// Paints every crosswalk in the map, one mesh each.
pub fn paint(
    map: &Map,
    map_name: &str,
    config: &SurfaceConfig,
    ordinals: &mut Ordinals,
    out: &mut Vec<Mesh>,
) {
    for object in map.objects.iter() {
        if object.kind != MapObjectKind::Crosswalk {
            continue;
        }
        let Some(road) = object
            .lanes
            .first()
            .and_then(|lane| map.lanes.get(lane))
            .and_then(|lane| map.road(&lane.road))
        else {
            continue;
        };
        if let Some(mesh) = bars(map, road, object, map_name, config, ordinals) {
            out.push(mesh);
        }
    }
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
    let section = (0..road.sections.len()).find(|&index| {
        road.section_range(index)
            .is_ok_and(|(start, end)| station >= start && station <= end)
    })?;
    let layout = Layout::of(map, road, section, config)?;
    let (outer_left, outer_right) = layout.carriageway(station)?;

    let sampling = map.metadata.sampling;
    let frame_at = |s: f64| {
        let sample = road.reference_line.sample_at(s, sampling).ok()?;
        Some(sample.frame().ok()?.banked(road.superelevation.evaluate(s)))
    };
    let (start, end) = (frame_at(from)?, frame_at(to)?);
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
