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
//!
//! Both lie on the road as its [surface features](roadgen_core::surface) leave
//! it: a crossing on a speed table is painted over the table, ramps and all, and
//! a stop line across a rut dips into it.

use roadgen_core::map::{Map, Road};
use roadgen_core::semantics::{MapObject, MapObjectKind, ObjectGeometry};
use roadgen_core::surface::Relief;
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
    let (at, behind) = (
        station.clamp(0.0, length),
        (station + back).clamp(0.0, length),
    );
    let mut mesh = Mesh::new(
        mesh_name(map_name, Role::Marking, ordinals.take(Role::Marking)),
        Role::Marking,
        materials::MARKING_WHITE,
    );
    patch(
        map,
        road,
        &Relief::of(map, &road.id),
        (at.min(behind), at.max(behind)),
        (outer, inner),
        config,
        &mut mesh,
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

    let relief = Relief::of(map, &road.id);
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
        patch(
            map,
            road,
            &relief,
            (from, to),
            (outer, outer - BAR),
            config,
            &mut mesh,
        );
    }
    (!mesh.is_empty()).then_some(mesh)
}

/// Paints a rectangle of the road onto `mesh`: from station `from` to `to`, and
/// from `outer` across to `inner` (left to right), the paint's rise above the
/// surface as the road's features leave it there. One quad where no feature
/// reaches it; where one does, a grid as fine as the features ask, so that the
/// paint follows a hump's ramp or a rut's trough rather than cutting through it.
fn patch(
    map: &Map,
    road: &Road,
    relief: &Relief,
    (from, to): (f64, f64),
    (outer, inner): (f64, f64),
    config: &SurfaceConfig,
    mesh: &mut Mesh,
) {
    let displaced = relief.touches_area((from, to), (inner, outer));
    let (mut stations, columns) = if displaced {
        let step = relief.lateral_step(from, to, config.relief_cell);
        (
            relief.stations(from, to, config.relief_cell),
            (((outer - inner) / step).ceil() as usize).max(1),
        )
    } else {
        (Vec::new(), 1)
    };
    stations.extend([from, to]);
    stations.sort_by(f64::total_cmp);
    stations.dedup_by(|a, b| (*a - *b).abs() < 1e-6);
    let sampling = map.metadata.sampling;
    let Some(rows) = stations
        .iter()
        .map(|&station| {
            let frame = road.frame_at(station, sampling).ok()?;
            Some(
                (0..=columns)
                    .map(|k| {
                        let lateral = outer + (inner - outer) * k as f64 / columns as f64;
                        let lift = if displaced {
                            relief.displacement(station, lateral)
                        } else {
                            0.0
                        };
                        frame.to_global([0.0, lateral, config.marking_rise + lift])
                    })
                    .collect(),
            )
        })
        .collect::<Option<Vec<Vec<_>>>>()
    else {
        return;
    };
    mesh.grid(&rows, 0);
}

#[cfg(test)]
mod tests {
    use roadgen_core::prelude::*;

    use crate::mesh::Mesh;
    use crate::surfaces::{build, SurfaceConfig};
    use crate::tags::Role;

    #[test]
    fn a_crossing_on_a_speed_table_is_painted_over_the_table() {
        // A 100 m two-lane road at 10 m, a crossing 4 m wide at its middle, and
        // a table 10 cm high under it with 1.5 m ramps either side.
        let mut builder = MapBuilder::default();
        let road = builder
            .add_road(
                RoadSpec::line(
                    Point3::new(0.0, 0.0, 10.0),
                    Point3::new(100.0, 0.0, 10.0),
                    vec![
                        LaneSpec::new(PositiveWidth::new(3.5).unwrap(), Direction::Forward),
                        LaneSpec::new(PositiveWidth::new(3.5).unwrap(), Direction::Backward),
                    ],
                )
                .unwrap(),
            )
            .unwrap();
        builder.add_crosswalk(&road, 0.5, 4.0).unwrap();
        builder
            .add_surface_feature(
                &road,
                46.0,
                8.0,
                0.0,
                7.0,
                SurfaceFeatureKind::Bump {
                    kind: BumpKind::SpeedTable,
                    height: 0.1,
                    profile: BumpProfile::Flat { ramp: 1.5 },
                },
            )
            .unwrap();
        let map = builder.finish().unwrap().validate().unwrap();
        let meshes = build(&map, "t", &SurfaceConfig::default());
        let paint: Vec<&Mesh> = meshes
            .iter()
            .filter(|mesh| mesh.role == Role::Marking)
            .collect();
        // The crossing's bars are on the table's flat top, 48..52 m: every
        // vertex of paint there stands above the table, none buried in it.
        let on_table: Vec<f64> = paint
            .iter()
            .flat_map(|mesh| mesh.positions.iter())
            .filter(|p| p.x > 48.0 && p.x < 52.0 && p.y.abs() < 3.0)
            .map(|p| p.z)
            .collect();
        assert!(!on_table.is_empty());
        assert!(
            on_table.iter().all(|&z| z > 10.1),
            "paint at {:?}",
            on_table.iter().copied().fold(f64::INFINITY, f64::min)
        );
    }
}
