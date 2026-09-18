//! Triangles, and the two ways this exporter makes them.
//!
//! Everything CARLA receives is a triangle mesh, and everything the IR holds is a
//! curve, a width or a ring. Two constructions bridge them, and there are only two on
//! purpose — each answers a different question about *shading*, which is the only
//! reason a mesh generator ever needs more than one.
//!
//! **A strip** ([`Mesh::strip`]) is a ribbon between two rails: a road surface, a
//! painted line, a kerb face. Its vertices are shared from one rung to the next, so
//! the averaged normals run smoothly along it and a curving road is not faceted.
//!
//! **A face** ([`Mesh::face`]) is one flat polygon with vertices of its own: a wall,
//! a roof panel, a patch of ground. Nothing is shared with the face beside it, so the
//! crease between two walls of a building stays a crease.
//!
//! That is the whole of the shading model. There are no smoothing groups here,
//! because the choice of where an edge is hard has already been made by which of the
//! two constructions a surface was built with.
//!
//! # Coordinates
//!
//! A mesh holds the IR's own metres, in the IR's own right-handed frame — x east,
//! y north, z up. Nothing is flipped, scaled or re-based here. The single place that
//! happens is [`crate::fbx`], where the file is written, because that is where the
//! convention stops being roadgen's and starts being Unreal's.

use roadgen_core::geometry::{Point3, Vector3};

use crate::tags::Role;

/// How far a texture tiles, metres per repeat.
///
/// Every surface here is textured by *projection* rather than by an unwrapped chart:
/// asphalt, grass and concrete are materials without a seam anyone can point at, so a
/// planar or along-the-strip projection at a stated real-world scale is both what
/// looks right and what a substitute texture can be dropped into without re-authoring.
pub const TEXTURE_SCALE: f64 = 4.0;

/// One triangle mesh, named the way CARLA's classifier will read it.
#[derive(Debug, Clone, PartialEq)]
pub struct Mesh {
    /// The asset name, which is also the semantic class. See [`crate::tags`].
    pub name: String,
    pub role: Role,
    pub positions: Vec<Point3>,
    /// Texture coordinates, one per position.
    pub uvs: Vec<[f64; 2]>,
    /// Triangles, as indices into `positions`, anticlockwise seen from the front.
    pub triangles: Vec<[u32; 3]>,
    /// The materials this mesh carries, as indices into the export's material table.
    /// A mesh always has at least one.
    pub materials: Vec<usize>,
    /// Which of `materials` each triangle uses, by position in that list.
    ///
    /// Kept per-triangle rather than per-mesh because a lane marking has to carry its
    /// white and its yellow paint in one mesh: CARLA's `use_carla_materials` walks the
    /// *material slots* of a marking mesh and picks its yellow instance for the slot
    /// whose name holds `Yellow`, so the colours have to be slots of one mesh rather
    /// than two meshes of one colour.
    pub slots: Vec<usize>,
}

impl Mesh {
    pub fn new(name: impl Into<String>, role: Role, material: usize) -> Mesh {
        Mesh {
            name: name.into(),
            role,
            positions: Vec::new(),
            uvs: Vec::new(),
            triangles: Vec::new(),
            materials: vec![material],
            slots: Vec::new(),
        }
    }

    pub fn is_empty(&self) -> bool {
        self.triangles.is_empty()
    }

    /// The slot a material takes on this mesh, adding it if it is not there yet.
    pub fn slot_for(&mut self, material: usize) -> usize {
        match self.materials.iter().position(|&held| held == material) {
            Some(slot) => slot,
            None => {
                self.materials.push(material);
                self.materials.len() - 1
            }
        }
    }

    /// Adds a ribbon between two rails of equal length, with shared vertices.
    ///
    /// `left` and `right` are the two edges, sampled at the same stations. The `u`
    /// coordinate runs along the ribbon by real distance and the `v` across it, so a
    /// texture laid on a road follows the road rather than the compass, and a lane
    /// that widens stretches its paint rather than repeating it mid-line.
    ///
    /// A rung where the two rails coincide is kept: a lane that tapers to nothing
    /// ends in degenerate triangles, and dropping them would leave a hole where the
    /// taper met its neighbour.
    pub fn strip(&mut self, left: &[Point3], right: &[Point3], slot: usize) {
        let rungs = left.len().min(right.len());
        if rungs < 2 {
            return;
        }
        let base = self.positions.len() as u32;

        let mut along = 0.0;
        for index in 0..rungs {
            if index > 0 {
                // Measured down the middle, so the two rails of a widening ribbon
                // agree about how far along they are.
                let previous = midpoint(left[index - 1], right[index - 1]);
                let current = midpoint(left[index], right[index]);
                along += previous.distance_to(current);
            }
            let across = left[index].distance_to(right[index]);
            let u = along / TEXTURE_SCALE;
            self.positions.push(left[index]);
            self.uvs.push([u, 0.0]);
            self.positions.push(right[index]);
            self.uvs.push([u, across / TEXTURE_SCALE]);
        }

        for index in 0..rungs - 1 {
            let (a, b) = (base + index as u32 * 2, base + index as u32 * 2 + 1);
            let (c, d) = (a + 2, b + 2);
            // Anticlockwise seen from the side `left` is to the left of `right` on,
            // which is how every caller here builds a ribbon: from above for a road
            // surface, from the carriageway for a kerb face.
            self.push_triangle([a, d, c], slot);
            self.push_triangle([a, b, d], slot);
        }
    }

    /// Adds one flat polygon with vertices of its own, fanned into triangles.
    ///
    /// The ring is taken in the order given: anticlockwise seen from the side the
    /// face is meant to be seen from. Texture coordinates are a planar projection
    /// onto the polygon's own plane, so a brick wall reads as brick at the scale it
    /// was authored at whichever way the wall faces.
    ///
    /// A fan is enough because every ring this exporter makes is convex — a wall
    /// quad, a roof panel, a rectangle of ground. A concave one would need an ear
    /// clip, and [`Mesh::face`] would be the wrong place to hide it.
    pub fn face(&mut self, ring: &[Point3], slot: usize) {
        if ring.len() < 3 {
            return;
        }
        let Some(normal) = ring_normal(ring) else {
            return;
        };
        let (u_axis, v_axis) = plane_axes(normal);
        let base = self.positions.len() as u32;
        for point in ring {
            let vector = point.to_vector();
            self.positions.push(*point);
            self.uvs.push([
                vector.dot(u_axis) / TEXTURE_SCALE,
                vector.dot(v_axis) / TEXTURE_SCALE,
            ]);
        }
        for index in 1..ring.len() as u32 - 1 {
            self.push_triangle([base, base + index, base + index + 1], slot);
        }
    }

    fn push_triangle(&mut self, triangle: [u32; 3], slot: usize) {
        self.triangles.push(triangle);
        self.slots.push(slot);
    }

    /// Whether every triangle uses the same slot, which is what lets the FBX say so
    /// once rather than once per triangle.
    pub fn is_one_material(&self) -> bool {
        self.materials.len() == 1 || self.slots.windows(2).all(|pair| pair[0] == pair[1])
    }

    /// Area-weighted vertex normals, one per position.
    ///
    /// Area weighting rather than a plain average, so that a long thin triangle where
    /// a lane tapers away does not pull the normal of the vertex it shares. A vertex
    /// used by no triangle with area — the tip of a taper — keeps a normal pointing
    /// up rather than nothing, since a zero normal is what makes a renderer draw a
    /// black wedge.
    pub fn normals(&self) -> Vec<Vector3> {
        let mut normals = vec![Vector3::ZERO; self.positions.len()];
        for triangle in &self.triangles {
            let [a, b, c] = triangle.map(|index| self.positions[index as usize]);
            // The cross product's length is twice the triangle's area, so using it
            // unnormalised *is* the area weighting.
            let weighted = (b - a).cross(c - a);
            for index in triangle {
                let normal = &mut normals[*index as usize];
                *normal = *normal + weighted;
            }
        }
        normals
            .into_iter()
            .map(|normal| match normal.normalize() {
                Ok(unit) => unit.get(),
                Err(_) => Vector3::UP,
            })
            .collect()
    }

    /// How many triangles, for a report.
    pub fn triangle_count(&self) -> usize {
        self.triangles.len()
    }
}

fn midpoint(a: Point3, b: Point3) -> Point3 {
    a.lerp(b, 0.5)
}

/// The plane normal of a ring, by Newell's method.
///
/// Newell rather than one cross product of the first three vertices: three vertices
/// of a roof panel can be very nearly collinear, and the cross product of two nearly
/// parallel edges is noise. `None` when the ring encloses no area at all.
fn ring_normal(ring: &[Point3]) -> Option<Vector3> {
    let mut normal = Vector3::ZERO;
    for index in 0..ring.len() {
        let current = ring[index];
        let next = ring[(index + 1) % ring.len()];
        normal = normal
            + Vector3::new(
                (current.y - next.y) * (current.z + next.z),
                (current.z - next.z) * (current.x + next.x),
                (current.x - next.x) * (current.y + next.y),
            );
    }
    normal.normalize().ok().map(|unit| unit.get())
}

/// Two axes spanning the plane a normal stands on, for a planar texture projection.
///
/// The seed is whichever world axis the normal leans on least, so the cross product
/// is never taken between two nearly parallel vectors — the classic way a wall facing
/// exactly north ends up with its texture stretched to infinity.
fn plane_axes(normal: Vector3) -> (Vector3, Vector3) {
    let seed = if normal.z.abs() < 0.9 {
        Vector3::UP
    } else {
        Vector3::new(1.0, 0.0, 0.0)
    };
    let u = normal
        .cross(seed)
        .normalize()
        .map(|unit| unit.get())
        .unwrap_or(Vector3::new(1.0, 0.0, 0.0));
    let v = normal
        .cross(u)
        .normalize()
        .map(|unit| unit.get())
        .unwrap_or(Vector3::new(0.0, 1.0, 0.0));
    (u, v)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn rail(y: f64) -> Vec<Point3> {
        (0..4)
            .map(|index| Point3::new(index as f64 * 10.0, y, 0.0))
            .collect()
    }

    #[test]
    fn a_strip_shares_its_vertices_from_one_rung_to_the_next() {
        let mut mesh = Mesh::new("t", Role::Road, 0);
        mesh.strip(&rail(2.0), &rail(-2.0), 0);
        // Two vertices a rung, not four a quad: the sharing is what makes the
        // averaged normals run smoothly along a curving road.
        assert_eq!(mesh.positions.len(), 8);
        assert_eq!(mesh.triangles.len(), 6);
    }

    #[test]
    fn a_strip_faces_up_when_its_left_rail_is_on_the_left() {
        let mut mesh = Mesh::new("t", Role::Road, 0);
        mesh.strip(&rail(2.0), &rail(-2.0), 0);
        for normal in mesh.normals() {
            assert!(normal.z > 0.99, "a road surface pointing {normal:?}");
        }
    }

    #[test]
    fn a_strip_measures_its_texture_along_the_ground_it_covers() {
        let mut mesh = Mesh::new("t", Role::Road, 0);
        mesh.strip(&rail(2.0), &rail(-2.0), 0);
        // Thirty metres of road at four metres a repeat is seven and a half repeats,
        // which is what stops a texture sliding as a lane widens.
        let last = mesh.uvs.last().unwrap();
        assert!((last[0] - 30.0 / TEXTURE_SCALE).abs() < 1e-9);
        assert!((last[1] - 4.0 / TEXTURE_SCALE).abs() < 1e-9);
    }

    #[test]
    fn a_face_keeps_its_own_vertices_so_a_corner_stays_a_corner() {
        let mut mesh = Mesh::new("t", Role::Building, 0);
        let wall = |x: f64| {
            vec![
                Point3::new(x, 0.0, 0.0),
                Point3::new(x, 4.0, 0.0),
                Point3::new(x, 4.0, 3.0),
                Point3::new(x, 0.0, 3.0),
            ]
        };
        mesh.face(&wall(0.0), 0);
        mesh.face(&wall(0.0), 0);
        assert_eq!(mesh.positions.len(), 8);
        // Both faces are the same plane, so every normal is that plane's: nothing was
        // averaged across the two.
        let normals = mesh.normals();
        assert!(normals.iter().all(|normal| normal.x.abs() > 0.99));
    }

    #[test]
    fn a_ring_that_encloses_nothing_adds_nothing() {
        let mut mesh = Mesh::new("t", Role::Terrain, 0);
        mesh.face(
            &[
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(2.0, 0.0, 0.0),
            ],
            0,
        );
        assert!(mesh.is_empty());
    }

    #[test]
    fn a_taper_that_closes_keeps_a_normal_pointing_somewhere() {
        // Where a lane tapers to nothing the last rung is a single point, and the
        // triangles that reach it have no area. A zero normal there is a black wedge
        // in the render, so it becomes `up` instead.
        let mut mesh = Mesh::new("t", Role::Road, 0);
        let left = vec![Point3::new(0.0, 2.0, 0.0), Point3::new(10.0, 0.0, 0.0)];
        let right = vec![Point3::new(0.0, -2.0, 0.0), Point3::new(10.0, 0.0, 0.0)];
        mesh.strip(&left, &right, 0);
        for normal in mesh.normals() {
            assert!((normal.norm() - 1.0).abs() < 1e-9);
        }
    }

    #[test]
    fn a_mesh_takes_a_second_slot_only_once() {
        let mut mesh = Mesh::new("t", Role::Marking, 3);
        assert_eq!(mesh.slot_for(3), 0);
        assert_eq!(mesh.slot_for(7), 1);
        assert_eq!(mesh.slot_for(7), 1);
        assert_eq!(mesh.materials, vec![3, 7]);
    }
}
