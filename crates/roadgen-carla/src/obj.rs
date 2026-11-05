//! The `.obj` CARLA's pedestrian navigation is built from.
//!
//! Where a walker may go in CARLA is a navigation mesh, built by Recast from an
//! `.obj` of the map whose materials say what each surface is: `sidewalk` and
//! `crosswalk` are where a pedestrian belongs, `road` and `grass` are where one may
//! go at a cost, and anything else — a wall, a roof — is a block. CARLA's own
//! tooling gets that `.obj` by converting the map's FBX with a tool the UE5 branch
//! no longer builds; this exporter writes it directly, from the same meshes, so
//! the file is there before the importer asks.
//!
//! The frame is the one Recast reads and CARLA's crosswalk tool writes: CARLA's
//! coordinates — `y` mirrored from the map's — with `y` up, in metres. Materials are
//! set after the vertices they apply to, because the loader resets the material at
//! every vertex line.
//!
//! Where two surfaces lie within a climb (0.3 m) of each other, Recast's
//! rasterizer keeps the *higher area id* — block 0, sidewalk 1, crosswalk 2,
//! road 3, grass 4 — whatever order they come in. So a crosswalk laid over the
//! road it crosses would become road, which a pedestrian that does not cross roads
//! is never routed over: the road under each crossing is cut away instead, and
//! there only the crossing is left.

use std::fmt::Write;

use roadgen_core::geometry::Point3;
use roadgen_core::map::Map;
use roadgen_core::semantics::{MapObjectKind, ObjectGeometry};

use crate::mesh::Mesh;
use crate::tags::Role;

/// The `.obj`, as text.
pub fn document(meshes: &[Mesh], map: &Map) -> String {
    let mut out = Writer {
        text: String::from("# Written by roadgen for CARLA's navigation builder.\n"),
        vertices: 0,
    };

    // The crossings, as the IR holds them: a rectangle each, kerb to kerb and a
    // little beyond, so that the navigation mesh joins them to the pavements.
    let crossings: Vec<(&roadgen_core::id::ObjectId, [Point3; 4])> = map
        .objects
        .iter()
        .filter(|object| object.kind == MapObjectKind::Crosswalk)
        .filter_map(|object| match &object.geometry {
            ObjectGeometry::Band { left, right } => Some((
                &object.id,
                [
                    left.start_point(),
                    left.end_point(),
                    right.end_point(),
                    right.start_point(),
                ],
            )),
            _ => None,
        })
        .collect();

    let holes: Vec<Hole> = crossings
        .iter()
        .map(|(_, corners)| Hole::new(corners))
        .collect();

    // Ground first, then the road, then the pavement: the order changes nothing
    // where Recast merges (see above) and keeps the file readable.
    let mut ordered: Vec<&Mesh> = meshes.iter().collect();
    ordered.sort_by_key(|mesh| precedence(mesh.role));
    for mesh in ordered {
        let Some(material) = material_of(mesh.role) else {
            continue;
        };
        if material == "road" && !holes.is_empty() {
            let (positions, triangles) = cut_away(mesh, &holes);
            out.object(&mesh.name, &positions, &triangles, material);
        } else {
            out.object(&mesh.name, &mesh.positions, &mesh.triangles, material);
        }
    }

    for (id, corners) in &crossings {
        out.object(id, corners, &[[0, 1, 2], [0, 2, 3]], "crosswalk");
    }
    out.text
}

/// A crossing's footprint, counter-clockwise in plan, with its extent.
struct Hole {
    ring: Vec<Point3>,
    min: Point3,
    max: Point3,
}

impl Hole {
    fn new(corners: &[Point3; 4]) -> Hole {
        let ring = counter_clockwise(corners.to_vec());
        let fold = |f: fn(f64, f64) -> f64, init: f64| {
            Point3::new(
                ring.iter().map(|p| p.x).fold(init, f),
                ring.iter().map(|p| p.y).fold(init, f),
                ring.iter().map(|p| p.z).fold(init, f),
            )
        };
        Hole {
            min: fold(f64::min, f64::MAX),
            max: fold(f64::max, f64::MIN),
            ring,
        }
    }

    /// Whether the hole can touch a surface within `min`..`max`: overlapping in
    /// plan, and at about its height -- a crossing on a bridge is no reason to cut
    /// the road below it.
    fn reaches(&self, min: Point3, max: Point3) -> bool {
        const HEIGHT: f64 = 2.0;
        self.min.x < max.x
            && self.max.x > min.x
            && self.min.y < max.y
            && self.max.y > min.y
            && self.min.z - HEIGHT < max.z
            && self.max.z + HEIGHT > min.z
    }
}

fn bounds(points: impl IntoIterator<Item = Point3>) -> (Point3, Point3) {
    points.into_iter().fold(
        (
            Point3::new(f64::MAX, f64::MAX, f64::MAX),
            Point3::new(f64::MIN, f64::MIN, f64::MIN),
        ),
        |(lo, hi), p| {
            (
                Point3::new(lo.x.min(p.x), lo.y.min(p.y), lo.z.min(p.z)),
                Point3::new(hi.x.max(p.x), hi.y.max(p.y), hi.z.max(p.z)),
            )
        },
    )
}

/// `mesh`'s triangles with every crossing's footprint (in plan) cut out of them.
fn cut_away(mesh: &Mesh, holes: &[Hole]) -> (Vec<Point3>, Vec<[u32; 3]>) {
    // Only the crossings this mesh can touch, and per triangle only those that
    // reach it: most triangles meet none and are passed through as they are.
    let (lo, hi) = bounds(mesh.positions.iter().copied());
    let near: Vec<&Hole> = holes.iter().filter(|h| h.reaches(lo, hi)).collect();
    if near.is_empty() {
        return (mesh.positions.clone(), mesh.triangles.clone());
    }
    let mut positions = Vec::new();
    let mut triangles = Vec::new();
    for [a, b, c] in &mesh.triangles {
        let corners = [
            mesh.positions[*a as usize],
            mesh.positions[*b as usize],
            mesh.positions[*c as usize],
        ];
        let (lo, hi) = bounds(corners);
        let mut pieces = vec![corners.to_vec()];
        for hole in near.iter().filter(|h| h.reaches(lo, hi)) {
            pieces = pieces
                .into_iter()
                .flat_map(|piece| subtract(&piece, &hole.ring))
                .collect();
        }
        for piece in pieces {
            // Each piece is convex: a fan, in the triangle's own winding.
            let first = positions.len() as u32;
            positions.extend(piece.iter().copied());
            for k in 1..piece.len() as u32 - 1 {
                triangles.push([first, first + k, first + k + 1]);
            }
        }
    }
    (positions, triangles)
}

fn counter_clockwise(mut ring: Vec<Point3>) -> Vec<Point3> {
    let area: f64 = (0..ring.len())
        .map(|i| {
            let (p, q) = (ring[i], ring[(i + 1) % ring.len()]);
            p.x * q.y - q.x * p.y
        })
        .sum();
    if area < 0.0 {
        ring.reverse();
    }
    ring
}

/// The convex polygon `piece` minus the convex, counter-clockwise `hole`, in plan:
/// convex pieces, heights carried along the edges they are cut from.
fn subtract(piece: &[Point3], hole: &[Point3]) -> Vec<Vec<Point3>> {
    let mut out = Vec::new();
    let mut rest = piece.to_vec();
    for i in 0..hole.len() {
        let (a, b) = (hole[i], hole[(i + 1) % hole.len()]);
        // Left of a->b is inside a counter-clockwise hole.
        let side = |p: &Point3| (b.x - a.x) * (p.y - a.y) - (b.y - a.y) * (p.x - a.x);
        let outside = clip(&rest, |p| -side(p));
        if outside.len() >= 3 {
            out.push(outside);
        }
        rest = clip(&rest, side);
        if rest.len() < 3 {
            return out;
        }
    }
    out // what is left of `rest` is inside the hole
}

/// Sutherland–Hodgman: the part of `ring` where `f >= 0`.
fn clip(ring: &[Point3], f: impl Fn(&Point3) -> f64) -> Vec<Point3> {
    let mut out = Vec::new();
    for i in 0..ring.len() {
        let (p, q) = (ring[i], ring[(i + 1) % ring.len()]);
        let (fp, fq) = (f(&p), f(&q));
        if fp >= 0.0 {
            out.push(p);
        }
        if (fp >= 0.0) != (fq >= 0.0) {
            let t = fp / (fp - fq);
            out.push(p.lerp(q, t));
        }
    }
    // Drop points the cut left on top of each other.
    out.dedup_by(|a, b| a.is_close(*b, 1e-9));
    if out.len() > 1 && out[0].is_close(out[out.len() - 1], 1e-9) {
        out.pop();
    }
    out
}

/// The document being written, and how many vertices it holds so far: an `.obj`
/// numbers its vertices from one across the whole file.
struct Writer {
    text: String,
    vertices: usize,
}

impl Writer {
    /// One object: its vertices, then its material, then its faces.
    fn object(
        &mut self,
        name: impl std::fmt::Display,
        positions: &[Point3],
        triangles: &[[u32; 3]],
        material: &str,
    ) {
        let _ = writeln!(self.text, "o {name}");
        let first = self.vertices + 1;
        for point in positions {
            // Map (x east, y north, z up) to CARLA's (x, -y, z), and that to
            // Recast's y-up (x, z, -y).
            let _ = writeln!(self.text, "v {:.4} {:.4} {:.4}", point.x, point.z, -point.y);
        }
        self.vertices += positions.len();
        let _ = writeln!(self.text, "usemtl {material}");
        for [a, b, c] in triangles {
            let _ = writeln!(
                self.text,
                "f {} {} {}",
                first + *a as usize,
                first + *b as usize,
                first + *c as usize
            );
        }
    }
}

/// The order surfaces are written in: the ground, then the road, then the
/// pavement standing on it.
fn precedence(role: Role) -> u8 {
    match role {
        Role::Building | Role::Curb | Role::TrafficLight | Role::TrafficSign => 0,
        Role::Terrain => 1,
        Role::Road | Role::Gutter => 2,
        Role::Sidewalk => 3,
        Role::Marking => 4,
    }
}

/// What Recast is told a surface of this role is, or `None` for one it need not
/// see at all — paint is a hair above the road it is on, and would only give the
/// navigation mesh a second surface there.
fn material_of(role: Role) -> Option<&'static str> {
    Some(match role {
        Role::Road | Role::Gutter => "road",
        Role::Sidewalk => "sidewalk",
        Role::Terrain => "grass",
        Role::Curb | Role::Building => "block",
        // Furniture is never in the map's meshes, and a post is not something a
        // pedestrian walks through in any case.
        Role::TrafficLight | Role::TrafficSign => "block",
        Role::Marking => return None,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_mesh_is_written_in_recasts_frame_under_its_recast_material() {
        let mut mesh = Mesh::new("T_Road_Sidewalk_0", Role::Sidewalk, 0);
        mesh.strip(
            &[Point3::new(0.0, 2.0, 1.0), Point3::new(10.0, 2.0, 1.0)],
            &[Point3::new(0.0, 0.0, 1.0), Point3::new(10.0, 0.0, 1.0)],
            0,
        );
        let map = Map::new(roadgen_core::map::MapMetadata::default());
        let text = document(&[mesh], &map);
        // y north becomes -y, and up becomes the middle coordinate.
        assert!(text.contains("v 0.0000 1.0000 -2.0000\n"), "{text}");
        assert!(text.contains("\nusemtl sidewalk\nf "), "{text}");
        assert!(text.contains("f 1 4 3\n"), "{text}");
    }

    #[test]
    fn the_ground_is_written_before_the_road_and_the_road_before_the_pavement() {
        let mut ground = Mesh::new("T_Terrain_Ground_0", Role::Terrain, 0);
        ground.strip(
            &[Point3::new(0.0, 5.0, 0.0), Point3::new(10.0, 5.0, 0.0)],
            &[Point3::new(0.0, -5.0, 0.0), Point3::new(10.0, -5.0, 0.0)],
            0,
        );
        let mut road = Mesh::new("T_Road_Road_0", Role::Road, 0);
        road.strip(
            &[Point3::new(0.0, 2.0, 0.05), Point3::new(10.0, 2.0, 0.05)],
            &[Point3::new(0.0, -2.0, 0.05), Point3::new(10.0, -2.0, 0.05)],
            0,
        );
        let map = Map::new(roadgen_core::map::MapMetadata::default());
        let text = document(&[road, ground], &map);
        assert!(text.find("usemtl grass").unwrap() < text.find("usemtl road").unwrap());
    }

    /// The road triangles of an `.obj` (in the map's plan: x, -z), as lists of
    /// three points.
    fn road_triangles(text: &str) -> Vec<[(f64, f64); 3]> {
        let mut vertices = Vec::new();
        let mut material = "";
        let mut out = Vec::new();
        for line in text.lines() {
            let mut parts = line.split_whitespace();
            match parts.next() {
                Some("v") => {
                    let v: Vec<f64> = parts.map(|s| s.parse().unwrap()).collect();
                    vertices.push((v[0], -v[2]));
                }
                Some("usemtl") => material = line.split_whitespace().nth(1).unwrap(),
                Some("f") if material == "road" => {
                    let i: Vec<usize> = parts.map(|s| s.parse::<usize>().unwrap() - 1).collect();
                    out.push([vertices[i[0]], vertices[i[1]], vertices[i[2]]]);
                }
                _ => {}
            }
        }
        out
    }

    fn area(t: &[(f64, f64); 3]) -> f64 {
        ((t[1].0 - t[0].0) * (t[2].1 - t[0].1) - (t[2].0 - t[0].0) * (t[1].1 - t[0].1)).abs() / 2.0
    }

    #[test]
    fn the_road_under_a_crossing_is_cut_away_so_the_crossing_is_walkable() {
        use roadgen_core::prelude::*;

        // A straight two-lane road 100 m long with a 4 m crossing half way.
        let mut builder = MapBuilder::new(MapMetadata::default());
        let width = |m: f64| PositiveWidth::new(m).unwrap();
        let road = builder
            .add_road(
                RoadSpec::line(
                    Point3::new(0.0, 0.0, 0.0),
                    Point3::new(100.0, 0.0, 0.0),
                    vec![
                        LaneSpec::new(width(3.5), Direction::Backward),
                        LaneSpec::new(width(3.5), Direction::Forward),
                    ],
                )
                .unwrap()
                .with_name("main"),
            )
            .unwrap();
        builder.add_crosswalk(&road, 0.5, 4.0).unwrap();
        let map = builder.finish().unwrap().into_map();
        // The carriageway as one mesh: 100 m by 7 m, in two triangles.
        let mut mesh = Mesh::new("T_Road_Road_0", Role::Road, 0);
        mesh.strip(
            &[Point3::new(0.0, 3.5, 0.0), Point3::new(100.0, 3.5, 0.0)],
            &[Point3::new(0.0, -3.5, 0.0), Point3::new(100.0, -3.5, 0.0)],
            0,
        );
        let text = document(&[mesh], &map);
        let road = road_triangles(&text);
        let total: f64 = road.iter().map(area).sum();
        // The crossing spans the carriageway (and beyond), 4 m along it.
        assert!(
            (total - (700.0 - 4.0 * 7.0)).abs() < 1e-6,
            "road area {total}"
        );
        for t in &road {
            let centre = (
                (t[0].0 + t[1].0 + t[2].0) / 3.0,
                (t[0].1 + t[1].1 + t[2].1) / 3.0,
            );
            assert!(
                !(48.0 < centre.0 && centre.0 < 52.0),
                "road left under the crossing: {t:?}"
            );
        }
        assert_eq!(text.matches("usemtl crosswalk").count(), 1);
    }

    #[test]
    fn paint_is_left_out_and_a_building_is_a_block() {
        assert_eq!(material_of(Role::Marking), None);
        assert_eq!(material_of(Role::Building), Some("block"));
        assert_eq!(material_of(Role::Terrain), Some("grass"));
    }
}
