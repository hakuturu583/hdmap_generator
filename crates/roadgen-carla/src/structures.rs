//! The bridges and tunnels a map's roads are carried on, as solids.
//!
//! The road surface on a structure is drawn like any other road's; what this adds
//! is what holds it up or closes it in. A bridge gets the underside of its deck,
//! the deck's sides, its parapets — the map's own barriers that are part of it,
//! standing on the deck's curb — and its abutments and piers down to the land. A
//! tunnel gets its walls, its ceiling and the walkway each side, all facing in,
//! and a portal face round each mouth facing out.
//!
//! None of it is a road a vehicle drives on, and CARLA's import has no class for a
//! bridge a map can reach, so it is all terrain — the same as a berm — in concrete,
//! or in steel for a steel bridge.

use roadgen_core::geometry::{Frame3, Point3};
use roadgen_core::map::Map;
use roadgen_core::semantics::MapObjectKind;
use roadgen_core::structure::{
    road_extent, shared_sides, Bridge, RoadSpan, StructureKind, StructureMaterial, Tunnel, MEDIAN,
};

use crate::ground::Ground;
use crate::materials;
use crate::mesh::Mesh;
use crate::surfaces::Ordinals;
use crate::tags::{mesh_name, Role};

/// How tall a bridge's parapet stands above its deck, metres: the core's, which
/// is what the parapets the core stands along a bridge are built to.
pub use roadgen_core::structure::PARAPET;

/// How far apart the sections of a structure's solids are, metres.
const STEP: f64 = 2.0;
/// How far a pier goes down where nothing says where the ground is, metres.
const FALLBACK_DEPTH: f64 = 10.0;
/// The wall over a tunnel's mouth, above its lining, metres.
const HEADWALL: f64 = 1.0;

/// One station of a carried road: its banked frame and the road's two edges.
struct Row {
    frame: Frame3,
    left: f64,
    right: f64,
}

impl Row {
    fn at(&self, lateral: f64, rise: f64) -> Point3 {
        self.frame.to_global([0.0, lateral, rise])
    }
}

fn rows(map: &Map, span: &RoadSpan) -> Vec<Row> {
    let Some(road) = map.road(&span.road) else {
        return Vec::new();
    };
    let count = ((span.to - span.from) / STEP).ceil().max(1.0) as usize;
    (0..=count)
        .filter_map(|index| {
            let station = span.from + (span.to - span.from) * index as f64 / count as f64;
            let frame = road.frame_at(station, map.metadata.sampling).ok()?;
            let (left, right) = road_extent(map, &span.road, station)?;
            Some(Row { frame, left, right })
        })
        .collect()
}

/// What says whether a point of some road is on a bridge or in a tunnel: inside a
/// structure's outline in plan and within a couple of metres of the height of the
/// road it carries there. A road passing under a bridge is inside its outline too,
/// but a deck's depth and a clearance below it.
pub struct Carried {
    outlines: Vec<Outline>,
}

/// One structure's outline: in plan, with its heights, and its bounds.
type Outline = (Vec<[f64; 2]>, Vec<Point3>, [f64; 4]);

impl Carried {
    pub fn of(map: &Map) -> Carried {
        Carried {
            outlines: map
                .structures
                .iter()
                .map(|structure| {
                    let points = structure.outline.points().to_vec();
                    let ring: Vec<[f64; 2]> = points.iter().map(|p| [p.x, p.y]).collect();
                    let bounds = roadgen_core::geometry::plan::bounds(&ring);
                    (ring, points, bounds)
                })
                .collect(),
        }
    }

    pub fn contains(&self, point: &Point3) -> bool {
        self.outlines.iter().any(|(ring, points, b)| {
            point.x >= b[0]
                && point.x <= b[2]
                && point.y >= b[1]
                && point.y <= b[3]
                && roadgen_core::geometry::plan::ring_contains(ring, [point.x, point.y])
                && points
                    .iter()
                    .min_by(|a, b| {
                        a.horizontal_distance_to(*point)
                            .total_cmp(&b.horizontal_distance_to(*point))
                    })
                    .is_some_and(|nearest| (nearest.z - point.z).abs() < 2.0)
        })
    }
}

/// Every structure on the map, each as a mesh of its own.
pub fn meshes(
    map: &Map,
    ground: Option<&Ground>,
    map_name: &str,
    ordinals: &mut Ordinals,
) -> Vec<Mesh> {
    let mut out = Vec::new();
    for structure in map.structures.iter() {
        let material = match &structure.kind {
            StructureKind::Bridge(Bridge {
                material: StructureMaterial::Steel,
                ..
            }) => materials::STEEL,
            _ => materials::ROAD_CONCRETE,
        };
        let mut mesh = Mesh::new(
            mesh_name(map_name, Role::Terrain, ordinals.take(Role::Terrain)),
            Role::Terrain,
            material,
        );
        match &structure.kind {
            StructureKind::Bridge(bridge) => {
                let carried: Vec<Vec<Row>> = structure
                    .carries
                    .iter()
                    .map(|span| rows(map, span))
                    .collect();
                for (index, own) in carried.iter().enumerate() {
                    let others: Vec<&[Row]> = carried
                        .iter()
                        .enumerate()
                        .filter(|(other, _)| *other != index)
                        .map(|(_, rows)| rows.as_slice())
                        .collect();
                    let shared = shared_sides(map, &structure.carries, index);
                    let [left, right] = joins(own, &others, shared);
                    deck(&mut mesh, own, bridge, left.as_deref(), right.as_deref());
                }
                // Its parapets, the map's barriers that are part of it, on its curb
                // and in its material.
                for parapet in map.objects.iter().filter(|object| {
                    matches!(
                        &object.kind,
                        MapObjectKind::Barrier { structure: Some(of), .. } if *of == structure.id
                    )
                }) {
                    crate::barriers::raise(&mut mesh, parapet, map.metadata.sampling);
                }
                for support in &bridge.supports {
                    let ring = support.footprint.points();
                    let top = support.footprint.lowest();
                    let centre = support.footprint.centroid();
                    let base = support.base.unwrap_or_else(|| {
                        ground.map_or(top - FALLBACK_DEPTH, |ground| {
                            ground.height(centre.x, centre.y, top - FALLBACK_DEPTH)
                        })
                    });
                    column(&mut mesh, ring, base.min(top), top);
                }
            }
            StructureKind::Tunnel(tunnel) => {
                for span in &structure.carries {
                    bore(&mut mesh, &rows(map, span), tunnel);
                }
            }
        }
        out.push(mesh);
    }
    out
}

/// Where a road's deck runs on into another's, side by side on one deck: for each
/// of its two sides, left then right, how far out — to halfway across the gap to
/// the other road — row by row, or `None` on a side with no road beside it.
///
/// Which sides those are is `shared`, the core's own answer, which is also what
/// decides where the bridge has no parapet: so a side the slab runs on across
/// never has one standing on it.
fn joins(rows: &[Row], others: &[&[Row]], shared: [bool; 2]) -> [Option<Vec<f64>>; 2] {
    // The other roads' edges beside each row, as offsets to its left.
    let beside = |row: &Row| -> Vec<f64> {
        let left = row.frame.left.get();
        let norm = (left.x * left.x + left.y * left.y).sqrt().max(1e-9);
        others
            .iter()
            .filter_map(|other| {
                other.iter().min_by(|a, b| {
                    a.frame
                        .origin
                        .horizontal_distance_to(row.frame.origin)
                        .total_cmp(&b.frame.origin.horizontal_distance_to(row.frame.origin))
                })
            })
            .flat_map(|near| [near.at(near.left, 0.0), near.at(near.right, 0.0)])
            .map(|p| {
                let (dx, dy) = (p.x - row.frame.origin.x, p.y - row.frame.origin.y);
                (dx * left.x + dy * left.y) / norm
            })
            .collect()
    };
    let gap = |row: &Row, side: f64| -> Option<f64> {
        let edge = if side > 0.0 { row.left } else { row.right };
        beside(row)
            .into_iter()
            .map(|lateral| (lateral - edge) * side)
            .filter(|&d| d > -1e-6 && d <= MEDIAN)
            .min_by(f64::total_cmp)
            .map(|d| edge + side * d / 2.0)
    };
    [(1.0, shared[0]), (-1.0, shared[1])].map(|(side, shared)| {
        if !shared || rows.is_empty() {
            return None;
        }
        Some(
            rows.iter()
                .map(|row| gap(row, side).unwrap_or(if side > 0.0 { row.left } else { row.right }))
                .collect(),
        )
    })
}

/// A deck's sides and underside. Each side runs from the underside up to the
/// curb, where the parapet — a barrier of the map's, raised beside the deck —
/// stands on it. A side running on into another road on the same deck has no
/// face: the slab goes on across, at the road's height, to `left_to` or
/// `right_to`, where the other road's half of it begins.
fn deck(
    mesh: &mut Mesh,
    rows: &[Row],
    bridge: &Bridge,
    left_to: Option<&[f64]>,
    right_to: Option<&[f64]>,
) {
    if rows.len() < 2 {
        return;
    }
    let curb = bridge.curb;
    let depth = bridge.deck_depth;
    let at = |lateral: &dyn Fn(usize, &Row) -> f64, rise: f64| -> Vec<Point3> {
        rows.iter()
            .enumerate()
            .map(|(index, row)| row.at(lateral(index, row), rise))
            .collect()
    };
    let left_inner = |_: usize, row: &Row| row.left;
    let right_inner = |_: usize, row: &Row| row.right;
    let left_outer = |index: usize, row: &Row| left_to.map_or(row.left + curb, |to| to[index]);
    let right_outer = |index: usize, row: &Row| right_to.map_or(row.right - curb, |to| to[index]);
    match left_to {
        // The slab across to the next road, facing up.
        Some(_) => mesh.strip(&at(&left_outer, 0.0), &at(&left_inner, 0.0), 0),
        // The deck's side, facing out, from its underside up to the curb, whose
        // top the parapet stands on and covers.
        None => mesh.strip(&at(&left_outer, -depth), &at(&left_outer, 0.0), 0),
    }
    match right_to {
        Some(_) => mesh.strip(&at(&right_inner, 0.0), &at(&right_outer, 0.0), 0),
        None => mesh.strip(&at(&right_outer, 0.0), &at(&right_outer, -depth), 0),
    }
    // The underside, facing down.
    mesh.strip(&at(&right_outer, -depth), &at(&left_outer, -depth), 0);
}

/// A column standing on `base` up to `top`: its sides, facing out, and its top.
fn column(mesh: &mut Mesh, ring: &[Point3], base: f64, top: f64) {
    for index in 0..ring.len() {
        let (a, b) = (ring[index], ring[(index + 1) % ring.len()]);
        let at = |p: Point3, z: f64| Point3::new(p.x, p.y, z);
        mesh.face(&[at(a, base), at(b, base), at(b, top), at(a, top)], 0);
    }
}

/// A bore's walls, ceiling and walkways, facing in, and a portal at each mouth
/// facing out.
fn bore(mesh: &mut Mesh, rows: &[Row], tunnel: &Tunnel) {
    if rows.len() < 2 {
        return;
    }
    let walkway = tunnel.walkway;
    let height = tunnel.clear_height;
    let line = |lateral: &dyn Fn(&Row) -> f64, rise: f64| -> Vec<Point3> {
        rows.iter().map(|row| row.at(lateral(row), rise)).collect()
    };
    let left_edge = |row: &Row| row.left;
    let right_edge = |row: &Row| row.right;
    let left_wall = |row: &Row| row.left + walkway;
    let right_wall = |row: &Row| row.right - walkway;
    // Walkways, facing up.
    mesh.strip(&line(&left_wall, 0.0), &line(&left_edge, 0.0), 0);
    mesh.strip(&line(&right_edge, 0.0), &line(&right_wall, 0.0), 0);
    // Walls, facing the road.
    mesh.strip(&line(&left_wall, height), &line(&left_wall, 0.0), 0);
    mesh.strip(&line(&right_wall, 0.0), &line(&right_wall, height), 0);
    // The ceiling, facing down.
    mesh.strip(&line(&right_wall, height), &line(&left_wall, height), 0);

    // The portals: the face round each mouth, its two piers and the lintel over
    // the opening, facing out of the tunnel.
    let top = height + tunnel.lining + HEADWALL;
    let lining = tunnel.lining;
    for (row, outward) in [(&rows[0], false), (&rows[rows.len() - 1], true)] {
        let (high, low) = (row.left + walkway, row.right - walkway);
        let quads = [
            (high, high + lining, 0.0, top),
            (low - lining, low, 0.0, top),
            (low, high, height, top),
        ];
        for (from, to, bottom, upper) in quads {
            // Anticlockwise seen from outside: at the start the viewer looks along
            // the road and its left is the road's left; at the end, the other way.
            let ring = if outward {
                [
                    row.at(from, bottom),
                    row.at(to, bottom),
                    row.at(to, upper),
                    row.at(from, upper),
                ]
            } else {
                [
                    row.at(to, bottom),
                    row.at(from, bottom),
                    row.at(from, upper),
                    row.at(to, upper),
                ]
            };
            mesh.face(&ring, 0);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use roadgen_core::prelude::*;
    use roadgen_core::structure::{BridgeSpec, TunnelSpec};

    /// Each triangle's facing, with its first corner.
    fn faces(mesh: &Mesh) -> Vec<(roadgen_core::geometry::Vector3, Point3)> {
        mesh.triangles
            .iter()
            .filter_map(|t| {
                let [a, b, c] = t.map(|i| mesh.positions[i as usize]);
                let n = (b - a).cross(c - a).normalize().ok()?.get();
                Some((n, a))
            })
            .collect()
    }

    fn map(tunnel: bool) -> ValidatedMap {
        let mut builder = MapBuilder::default();
        let lanes = vec![
            LaneSpec::new(PositiveWidth::new(3.5).unwrap(), Direction::Forward),
            LaneSpec::new(PositiveWidth::new(3.5).unwrap(), Direction::Backward),
        ];
        let road = builder
            .add_road(
                RoadSpec::line(
                    Point3::new(0.0, 0.0, 10.0),
                    Point3::new(300.0, 0.0, 10.0),
                    lanes,
                )
                .unwrap()
                .with_name("r"),
            )
            .unwrap();
        let span = vec![RoadSpan::new(road, 100.0, 200.0)];
        if tunnel {
            builder.add_tunnel(span, TunnelSpec::default()).unwrap();
        } else {
            builder.add_bridge(span, BridgeSpec::default()).unwrap();
        }
        builder.finish().unwrap().validate().unwrap()
    }

    #[test]
    fn a_bridge_has_an_underside_that_faces_down_and_piers_to_the_ground() {
        let map = map(false);
        let meshes = meshes(&map, None, "m", &mut Ordinals::default());
        assert_eq!(meshes.len(), 1);
        let mesh = &meshes[0];
        assert_eq!(crate::tags::label_of(&mesh.name), crate::Label::Terrain);
        // Under the deck: 10 − 2.5.
        let underside = faces(mesh)
            .iter()
            .filter(|(n, a)| n.z < -0.99 && (a.z - 7.5).abs() < 1e-6)
            .count();
        assert!(underside > 0, "a deck seen from below");
        // A pier reaches down below the deck.
        let lowest = mesh
            .positions
            .iter()
            .map(|p| p.z)
            .fold(f64::INFINITY, f64::min);
        assert!((lowest - (7.5 - FALLBACK_DEPTH)).abs() < 1e-6, "{lowest}");
    }

    #[test]
    fn a_bridges_parapets_are_the_maps_barriers_drawn_once_with_its_deck() {
        let map = map(false);
        let top = 10.0 + PARAPET;
        let parapet_tops = |map: &ValidatedMap| -> (usize, Vec<f64>) {
            let mut ordinals = Ordinals::default();
            let mut all = meshes(map, None, "m", &mut ordinals);
            all.extend(crate::barriers::berms(map, "m", &mut ordinals));
            let tops = all
                .iter()
                .flat_map(faces)
                .filter(|(n, a)| n.z > 0.99 && (a.z - top).abs() < 1e-6)
                .map(|(_, a)| a.y)
                .collect();
            (all.len(), tops)
        };
        // One mesh for the bridge and nothing more: the barriers' own pass leaves
        // the parapets to it.
        let (count, tops) = parapet_tops(&map);
        assert_eq!(count, 1);
        // Their tops, over the curb each side: from the road's edge 3.5 m out to
        // the deck's 0.6 m further.
        assert!(tops.iter().any(|y| *y > 3.5 - 1e-6 && *y < 4.1 + 1e-6));
        assert!(tops.iter().any(|y| *y < -3.5 + 1e-6 && *y > -4.1 - 1e-6));
        assert!(tops
            .iter()
            .all(|y| y.abs() > 3.5 - 1e-6 && y.abs() < 4.1 + 1e-6));

        // A parapet taken off the map is not drawn: they are the map's, not the
        // deck's own.
        let mut bare = map.into_map();
        let ids: Vec<_> = bare
            .objects
            .iter()
            .filter(|object| {
                matches!(
                    &object.kind,
                    MapObjectKind::Barrier { kind, .. } if kind == roadgen_core::structure::PARAPET_KIND
                )
            })
            .map(|object| object.id.clone())
            .collect();
        assert_eq!(ids.len(), 2);
        let mut kept = roadgen_core::Arena::new();
        for object in bare.objects.iter() {
            if object.id != ids[0] {
                kept.insert(object.id.clone(), object.clone()).unwrap();
            }
        }
        bare.objects = kept;
        let bare = roadgen_core::validation::UnvalidatedMap::from_map(bare)
            .validate()
            .unwrap();
        let (_, tops) = parapet_tops(&bare);
        let sides: Vec<f64> = tops.iter().map(|y| y.signum()).collect();
        assert!(!sides.is_empty());
        assert!(sides.iter().all(|side| *side == sides[0]), "one side only");
    }

    #[test]
    fn a_tunnel_is_closed_over_its_road_and_open_at_its_mouths() {
        let map = map(true);
        let meshes = meshes(&map, None, "m", &mut Ordinals::default());
        let mesh = &meshes[0];
        let faces = faces(mesh);
        // A ceiling over the road, facing down at 10 + 5.
        assert!(faces
            .iter()
            .any(|(n, a)| n.z < -0.99 && (a.z - 15.0).abs() < 1e-6));
        // The portal faces point out of the tunnel: −x at the start, +x at the end,
        // and none of them the other way.
        let facing = |want: f64, x: f64| {
            faces
                .iter()
                .any(|(n, a)| (n.x - want).abs() < 1e-6 && (a.x - x).abs() < 1e-6)
        };
        assert!(!facing(1.0, 100.0) && !facing(-1.0, 200.0));
        assert!(facing(-1.0, 100.0));
        assert!(facing(1.0, 200.0));
        // Nothing of the tunnel stands where the road is: the opening is clear.
        for p in &mesh.positions {
            let in_opening = p.y.abs() < 3.5 && p.z > 10.0 + 1e-6 && p.z < 15.0 - 1e-6;
            assert!(!in_opening, "{p:?}");
        }
    }
}
