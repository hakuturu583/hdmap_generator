//! Open water: the surface of a river, laid over the channel the land is cut to.
//!
//! The land under a river is its bed and banks, surfaced as a riverbed. Over it,
//! across the whole of the land cover, lies a sheet of water two thirds of the
//! channel's depth over its bed — level across the river, and falling along it
//! with the bed: where the banks rise above it the land hides it, so its edge is
//! wherever the banks cross that height, with nothing to compute. The sheet
//! is terrain to CARLA, like everything else of the land, and is surfaced with
//! [`materials::WATER`](crate::materials::WATER), its texture coordinates in plan.

use roadgen_core::geometry::plan::{bounds, ring_contains, within_bounds, PlanPoint};
use roadgen_core::geometry::Point3;
use roadgen_core::map::Map;
use spade::{ConstrainedDelaunayTriangulation, Point2, Triangulation};

use crate::materials;
use crate::mesh::Mesh;
use crate::surfaces::Ordinals;
use crate::tags::{mesh_name, Role};

/// How far up from its bed to its banks' tops a river's water stands.
const FILL: f64 = 2.0 / 3.0;

/// Whether a land cover of `kind` is open water.
pub fn is_water(kind: &str) -> bool {
    matches!(kind, "water" | "river" | "canal" | "lake" | "pond")
}

/// The surface of every body of water the map's land states, each a mesh.
pub fn meshes(map: &Map, map_name: &str, ordinals: &mut Ordinals) -> Vec<Mesh> {
    let Some(land) = map.tin_surfaces.iter().next() else {
        return Vec::new();
    };
    let mut out = Vec::new();
    for cover in map.land_covers.iter().filter(|cover| is_water(&cover.kind)) {
        let outline = cover.area.points();
        let ring: Vec<PlanPoint> = outline.iter().map(|p| [p.x, p.y]).collect();
        let b = bounds(&ring);
        // The bed: the land inside the outline; the banks' tops: the outline
        // itself, which is draped on them.
        let inside: Vec<Point3> = land
            .tin
            .nodes()
            .iter()
            .filter(|p| within_bounds(&b, [p.x, p.y], 0.0) && ring_contains(&ring, [p.x, p.y]))
            .copied()
            .collect();
        let bed = inside.iter().map(|p| p.z).fold(f64::INFINITY, f64::min);
        let top = cover.area.lowest();
        if !bed.is_finite() || top <= bed {
            continue;
        }
        // The water stands the same depth over the bed all along, and the bed
        // may fall along the river: at each point of the outline the bed is the
        // lowest the land goes within a river's width of it.
        let depth = (top - bed) * FILL;
        let reach = width(&ring);
        let level = |at: PlanPoint| {
            let local = inside
                .iter()
                .filter(|p| (p.x - at[0]).hypot(p.y - at[1]) <= reach)
                .map(|p| p.z)
                .fold(f64::INFINITY, f64::min);
            (if local.is_finite() { local } else { bed }) + depth
        };
        let Some((positions, triangles)) = sheet(&ring, level) else {
            continue;
        };
        let mut mesh = Mesh::new(
            mesh_name(map_name, Role::Terrain, ordinals.take(Role::Terrain)),
            Role::Terrain,
            materials::WATER,
        );
        mesh.sheet(&positions, &triangles, 0);
        out.push(mesh);
    }
    out
}

/// How wide a long thin `ring` is: twice its area over its perimeter.
fn width(ring: &[PlanPoint]) -> f64 {
    let n = ring.len();
    let (mut area, mut perimeter) = (0.0, 0.0);
    for i in 0..n {
        let (a, b) = (ring[i], ring[(i + 1) % n]);
        area += a[0] * b[1] - b[0] * a[1];
        perimeter += (b[0] - a[0]).hypot(b[1] - a[1]);
    }
    (area.abs() / perimeter.max(1e-9)).max(1.0)
}

/// The polygon `ring` triangulated, facing up, each corner at the height `z`
/// gives there.
fn sheet(ring: &[PlanPoint], z: impl Fn(PlanPoint) -> f64) -> Option<(Vec<Point3>, Vec<[u32; 3]>)> {
    let mut cdt: ConstrainedDelaunayTriangulation<Point2<f64>> =
        ConstrainedDelaunayTriangulation::new();
    let mut handles = Vec::with_capacity(ring.len());
    for p in ring {
        handles.push(cdt.insert(Point2::new(p[0], p[1])).ok()?);
    }
    for index in 0..handles.len() {
        let (a, b) = (handles[index], handles[(index + 1) % handles.len()]);
        if a != b && cdt.can_add_constraint(a, b) {
            cdt.add_constraint(a, b);
        }
    }
    let mut positions = Vec::new();
    let mut slots = vec![u32::MAX; cdt.num_vertices()];
    let mut triangles = Vec::new();
    for face in cdt.inner_faces() {
        let corners = face.vertices();
        let centre = corners.iter().fold([0.0, 0.0], |sum, corner| {
            let at = corner.position();
            [sum[0] + at.x / 3.0, sum[1] + at.y / 3.0]
        });
        if !ring_contains(ring, centre) {
            continue;
        }
        // Spade winds an inner face anticlockwise in the plane: up, seen from above.
        triangles.push(corners.map(|corner| {
            let slot = &mut slots[corner.fix().index()];
            if *slot == u32::MAX {
                *slot = positions.len() as u32;
                let at = corner.position();
                positions.push(Point3::new(at.x, at.y, z([at.x, at.y])));
            }
            *slot
        }));
    }
    (!triangles.is_empty()).then_some((positions, triangles))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_sheet_fills_a_bent_channel_and_nothing_outside_it() {
        // An L, which a fan from one corner would get wrong.
        let ring = [
            [0.0, 0.0],
            [10.0, 0.0],
            [10.0, 10.0],
            [5.0, 10.0],
            [5.0, 5.0],
            [0.0, 5.0],
        ];
        let (positions, triangles) = sheet(&ring, |_| 3.0).unwrap();
        let area: f64 = triangles
            .iter()
            .map(|t| {
                let [a, b, c] = t.map(|i| positions[i as usize]);
                ((b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x)) / 2.0
            })
            .sum();
        assert!((area - 75.0).abs() < 1e-9, "{area}");
        assert!(positions.iter().all(|p| p.z == 3.0));
    }
}
