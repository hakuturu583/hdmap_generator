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

    // In order of precedence, lowest first. The builder stamps each triangle's
    // material onto every cell under it and five metres up, one triangle after
    // another, so whatever is written last wins where surfaces overlap — and the
    // ground reaches under every road, the road under every pavement.
    let mut ordered: Vec<&Mesh> = meshes.iter().collect();
    ordered.sort_by_key(|mesh| precedence(mesh.role));
    for mesh in ordered {
        let Some(material) = material_of(mesh.role) else {
            continue;
        };
        out.object(&mesh.name, &mesh.positions, &mesh.triangles, material);
    }

    // The crossings, as the IR holds them: a rectangle each, kerb to kerb and a
    // little beyond, so that the navigation mesh joins them to the pavements.
    for object in map.objects.iter() {
        if object.kind != MapObjectKind::Crosswalk {
            continue;
        }
        let ObjectGeometry::Band { left, right } = &object.geometry else {
            continue;
        };
        let corners = [
            left.start_point(),
            left.end_point(),
            right.end_point(),
            right.start_point(),
        ];
        out.object(
            &object.id.to_string(),
            &corners,
            &[[0, 1, 2], [0, 2, 3]],
            "crosswalk",
        );
    }
    out.text
}

/// The document being written, and how many vertices it holds so far: an `.obj`
/// numbers its vertices from one across the whole file.
struct Writer {
    text: String,
    vertices: usize,
}

impl Writer {
    /// One object: its vertices, then its material, then its faces.
    fn object(&mut self, name: &str, positions: &[Point3], triangles: &[[u32; 3]], material: &str) {
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

/// Which surfaces are written over which: the ground under everything, then the
/// road, then the pavement standing on it.
fn precedence(role: Role) -> u8 {
    match role {
        Role::Building | Role::Curb => 0,
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

    #[test]
    fn paint_is_left_out_and_a_building_is_a_block() {
        assert_eq!(material_of(Role::Marking), None);
        assert_eq!(material_of(Role::Building), Some("block"));
        assert_eq!(material_of(Role::Terrain), Some("grass"));
    }
}
