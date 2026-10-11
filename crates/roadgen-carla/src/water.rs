//! Open water: the surface of a river, laid over the channel the land is cut to.
//!
//! The land under a river is its bed and banks, surfaced as a riverbed. Over it,
//! across the whole of the land cover, lies a sheet of water two thirds of the
//! channel's depth over its bed — level across a river, and falling along it
//! with the bed; level all over a lake or a pond: where the banks rise above it the land hides it, so its edge is
//! wherever the banks cross that height, with nothing to compute. The sheet
//! is terrain to CARLA, like everything else of the land.
//!
//! Running water and standing water are surfaced apart. A river's sheet is
//! [`materials::FLOWING_WATER`](crate::materials::FLOWING_WATER), its texture
//! coordinates `u` down its centre line from its source and `v` across it, so
//! that a material panning its normals along `u` flows the way the river does;
//! a lake's or a pond's is [`materials::STILL_WATER`](crate::materials::STILL_WATER),
//! its texture coordinates in plan, since standing water has no way to run.
//!
//! The IR keeps a river as its outline and nothing more, so the centre line is
//! read back off it: the outline of a channel is its two banks and the two short
//! ends across it, the ends are where the outline turns back on itself, and
//! halfway between the banks is the middle. Which end is the source is the
//! land's to say: the water stands higher there.

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

/// Whether water of `kind` runs downhill along its length, rather than standing
/// level.
fn is_flowing(kind: &str) -> bool {
    matches!(kind, "water" | "river" | "canal")
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
        // The water stands the same depth over the bed all along, and a river's
        // bed may fall along it: at each point of the outline the bed is the
        // lowest the land goes within the river's width of it. Standing water —
        // a lake, a pond — is one level, whatever its bed does.
        let depth = (top - bed) * FILL;
        let reach = width(&ring);
        let flows = is_flowing(&cover.kind);
        let level = |at: PlanPoint| {
            if !flows {
                return bed + depth;
            }
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
        let flow = if flows {
            flow(&ring, &positions, level)
        } else {
            None
        };
        let mut mesh = Mesh::new(
            mesh_name(map_name, Role::Terrain, ordinals.take(Role::Terrain)),
            Role::Terrain,
            if flows {
                materials::FLOWING_WATER
            } else {
                materials::STILL_WATER
            },
        );
        match flow {
            Some(uvs) => mesh.sheet_mapped(&positions, &triangles, &uvs, 0),
            // A river whose outline is no channel — too few corners to have two
            // ends — still runs; it just has no along to lay its texture by.
            None => mesh.sheet(&positions, &triangles, 0),
        }
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

/// Where each of `positions` lies on the river outlined by `ring`, in metres: how
/// far down its centre line from its source, and how far left of that line,
/// facing downstream. The source is the end where `level` is higher; `None` when
/// the ring has no two ends to run between.
fn flow(
    ring: &[PlanPoint],
    positions: &[Point3],
    level: impl Fn(PlanPoint) -> f64,
) -> Option<Vec<[f64; 2]>> {
    let mut centre = centre_line(ring)?;
    if level(centre[0]) < level(centre[centre.len() - 1]) {
        centre.reverse();
    }
    let mut along = vec![0.0];
    for pair in centre.windows(2) {
        let run = (pair[1][0] - pair[0][0]).hypot(pair[1][1] - pair[0][1]);
        along.push(along[along.len() - 1] + run);
    }
    Some(
        positions
            .iter()
            .map(|p| {
                // The nearest point of the centre line, and which side of it.
                let mut best = (f64::INFINITY, [0.0, 0.0]);
                for (index, pair) in centre.windows(2).enumerate() {
                    let (a, b) = (pair[0], pair[1]);
                    let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
                    let run = dx.hypot(dy);
                    if run < 1e-9 {
                        continue;
                    }
                    let (ex, ey) = (p.x - a[0], p.y - a[1]);
                    let t = ((ex * dx + ey * dy) / (run * run)).clamp(0.0, 1.0);
                    let off = (ex - dx * t).hypot(ey - dy * t);
                    if off < best.0 {
                        let left = (dx * ey - dy * ex) / run;
                        best = (off, [along[index] + t * run, left]);
                    }
                }
                best.1
            })
            .collect(),
    )
}

/// The middle of a channel outlined by `ring`, from one end to the other.
///
/// The two ends are the two edges across which the outline turns furthest —
/// half a turn, a quarter at each corner, where a bank meets an end — and
/// apart from each other; the banks are what lies between them, and the middle
/// is halfway from one bank to the other, each taken by its share of its own
/// length so that the inside of a bend keeps pace with the outside.
fn centre_line(ring: &[PlanPoint]) -> Option<Vec<PlanPoint>> {
    let n = ring.len();
    if n < 4 {
        return None;
    }
    let heading = |i: usize| {
        let (a, b) = (ring[i % n], ring[(i + 1) % n]);
        (b[1] - a[1]).atan2(b[0] - a[0])
    };
    // How far the outline turns at corner `i`, anticlockwise positive, which is
    // the way an end turns on an outline the IR keeps anticlockwise.
    let corner = |i: usize| {
        let angle = heading(i) - heading(i + n - 1);
        angle - std::f64::consts::TAU * (angle / std::f64::consts::TAU).round()
    };
    // And across the edge from corner `i` to the next, at both of its corners.
    let turn = |i: usize| corner(i) + corner(i + 1);
    let first = (0..n).max_by(|&a, &b| turn(a).total_cmp(&turn(b)))?;
    let second = (0..n)
        .filter(|&i| {
            let apart = (i + n - first) % n;
            apart >= 2 && apart <= n - 2
        })
        .max_by(|&a, &b| turn(a).total_cmp(&turn(b)))?;
    // Both banks, each from the first end to the second.
    let walk = |from: usize, to: usize| -> Vec<PlanPoint> {
        let count = (to + n - from) % n + 1;
        (0..count).map(|k| ring[(from + k) % n]).collect()
    };
    let one = walk((first + 1) % n, second);
    let mut other = walk((second + 1) % n, first);
    other.reverse();
    let samples = one.len().max(other.len()).max(2);
    Some(
        (0..samples)
            .map(|k| {
                let share = k as f64 / (samples - 1) as f64;
                let (a, b) = (at_share(&one, share), at_share(&other, share));
                [(a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0]
            })
            .collect(),
    )
}

/// The point `share` of the way along the polyline `line`, by length.
fn at_share(line: &[PlanPoint], share: f64) -> PlanPoint {
    let runs: Vec<f64> = line
        .windows(2)
        .map(|pair| (pair[1][0] - pair[0][0]).hypot(pair[1][1] - pair[0][1]))
        .collect();
    let mut left = runs.iter().sum::<f64>() * share;
    for (pair, run) in line.windows(2).zip(&runs) {
        if left <= *run && *run > 0.0 {
            let t = left / run;
            return [
                pair[0][0] + (pair[1][0] - pair[0][0]) * t,
                pair[0][1] + (pair[1][1] - pair[0][1]) * t,
            ];
        }
        left -= run;
    }
    line[line.len() - 1]
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
    use crate::environment::river::RiverSpec;

    /// A channel 10 m wide, east from the origin and then north, as the IR
    /// keeps its outline: anticlockwise.
    fn bent() -> Vec<PlanPoint> {
        let river = RiverSpec {
            name: "bent".into(),
            centre: vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(100.0, 0.0, 0.0),
                Point3::new(100.0, 100.0, 0.0),
            ],
            width: 10.0,
            depth: 1.0,
            bank_slope: 1.0,
            site: None,
        };
        let mut ring: Vec<PlanPoint> = river.outline(5.0).iter().map(|p| [p.x, p.y]).collect();
        let area: f64 = (0..ring.len())
            .map(|i| {
                let (a, b) = (ring[i], ring[(i + 1) % ring.len()]);
                a[0] * b[1] - b[0] * a[1]
            })
            .sum();
        if area < 0.0 {
            ring.reverse();
        }
        ring
    }

    #[test]
    fn a_rivers_texture_runs_down_it_from_whichever_end_stands_higher() {
        let ring = bent();
        let at = |x: f64, y: f64| Point3::new(x, y, 0.0);
        let positions = [
            at(2.0, 0.0),
            at(50.0, 0.0),
            at(100.0, 50.0),
            at(100.0, 98.0),
            at(50.0, 5.0),
            at(50.0, -5.0),
        ];
        // The land falls to the north-east: the source is the river's first point.
        let uvs = flow(&ring, &positions, |p| -(p[0] + p[1])).expect("a channel");
        let down: Vec<f64> = uvs[..4].iter().map(|uv| uv[0]).collect();
        assert!(down.windows(2).all(|w| w[1] > w[0]), "{down:?}");
        for (u, expected) in down.iter().zip([2.0, 50.0, 150.0, 198.0]) {
            assert!((u - expected).abs() < 3.0, "{down:?}");
        }
        assert!(uvs[..4].iter().all(|uv| uv[1].abs() < 0.5), "{uvs:?}");
        // Facing east, north is to the left.
        assert!(
            (uvs[4][1] - 5.0).abs() < 0.5 && (uvs[5][1] + 5.0).abs() < 0.5,
            "{uvs:?}"
        );

        // The same channel running the other way, because the land says so.
        let uvs = flow(&ring, &positions, |p| p[0] + p[1]).expect("a channel");
        let down: Vec<f64> = uvs[..4].iter().map(|uv| uv[0]).collect();
        assert!(down.windows(2).all(|w| w[1] < w[0]), "{down:?}");
        assert!(
            (down[3] - 2.0).abs() < 3.0 && (down[0] - 198.0).abs() < 3.0,
            "{down:?}"
        );
        assert!(
            (uvs[4][1] + 5.0).abs() < 0.5 && (uvs[5][1] - 5.0).abs() < 0.5,
            "{uvs:?}"
        );
    }

    #[test]
    fn the_ends_of_a_channel_are_found_wherever_its_outline_starts() {
        let ring = bent();
        let length = |line: &[PlanPoint]| {
            line.windows(2)
                .map(|w| (w[1][0] - w[0][0]).hypot(w[1][1] - w[0][1]))
                .sum::<f64>()
        };
        for start in [0, 7, ring.len() / 2, ring.len() - 1] {
            let mut turned = ring.clone();
            turned.rotate_left(start);
            let centre = centre_line(&turned).expect("a channel");
            let ends = [centre[0], centre[centre.len() - 1]];
            let at_origin = ends.iter().any(|p| p[0].hypot(p[1]) < 1e-6);
            let at_mouth = ends
                .iter()
                .any(|p| (p[0] - 100.0).hypot(p[1] - 100.0) < 1e-6);
            assert!(at_origin && at_mouth, "from {start}: {ends:?}");
            assert!((length(&centre) - 200.0).abs() < 3.0, "{}", length(&centre));
        }
    }

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
