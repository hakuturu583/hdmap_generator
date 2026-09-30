//! Autoware's point-cloud map, from the surface a CARLA package is built of.
//!
//! Autoware localizes by matching its LiDAR against a point-cloud map (NDT), and a
//! map with none leaves the stack waiting for a pose it never gets. A generated map
//! has no survey to take one from — but it has something better for a simulator:
//! the very meshes the level is built from, which are exactly what the simulated
//! LiDAR sees. This samples them.
//!
//! What is written is Autoware's *divided* point-cloud map, the layout its map
//! loader reads for partial loading:
//!
//! ```text
//!   <directory>/
//!   ├── pointcloud_map/
//!   │   ├── <ix>_<iy>.pcd          one grid cell each, binary, x y z
//!   │   └── …
//!   └── pointcloud_map_metadata.yaml   the grid, and where each cell starts
//! ```
//!
//! ```yaml
//! x_resolution: 20.0
//! y_resolution: 20.0
//! 0_-1.pcd: [0.0, -20.0]      # 0 <= x < 20, -20 <= y < 0
//! ```
//!
//! Points are in the frame Autoware reads the Lanelet2 map in — its `local_x` /
//! `local_y` — which is the map's own metres for every projection but MGRS. The
//! caller supplies that conversion, because it is the Lanelet2 exporter's to make
//! and this crate does not depend on it.

use std::collections::{BTreeMap, HashMap};
use std::fmt::Write as _;
use std::fs;
use std::path::{Path, PathBuf};

use roadgen_core::geometry::Point3;

use crate::error::ExportError;
use crate::mesh::Mesh;

/// The directory the cells are written into, and the metadata file beside it:
/// the names Autoware's own launch files look for under a map directory.
pub const DIRECTORY: &str = "pointcloud_map";
pub const METADATA: &str = "pointcloud_map_metadata.yaml";

/// How densely the surface is sampled, and how the map is divided.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PointCloudConfig {
    /// Metres between samples along a triangle's edges, and the size of the voxel
    /// that keeps one point of the many that land in it. Finer than the voxels
    /// Autoware downsamples the map (and its scans) to for NDT.
    pub spacing: f64,
    /// The side of a grid cell, in metres: one `.pcd` each.
    pub cell_size: f64,
}

impl Default for PointCloudConfig {
    fn default() -> Self {
        PointCloudConfig {
            spacing: 0.5,
            cell_size: 20.0,
        }
    }
}

/// What was written.
#[derive(Debug, Clone, PartialEq)]
pub struct PointCloudMap {
    /// The `pointcloud_map/` directory, which is what Autoware's
    /// `pointcloud_map_file` names when the map is divided.
    pub directory: PathBuf,
    pub metadata: PathBuf,
    /// How many cells, and so how many `.pcd` files.
    pub cells: usize,
    pub points: usize,
}

/// Points over every triangle of `meshes`, one per voxel of `spacing`, in the
/// map's own frame.
pub fn sample(meshes: &[Mesh], spacing: f64) -> Vec<Point3> {
    let mut kept: HashMap<[i64; 3], Point3> = HashMap::new();
    for mesh in meshes {
        for [a, b, c] in &mesh.triangles {
            let (a, b, c) = (
                mesh.positions[*a as usize],
                mesh.positions[*b as usize],
                mesh.positions[*c as usize],
            );
            triangle(a, b, c, spacing, |point| {
                // The first point to land in a voxel keeps it: triangles share their
                // edges, and the roads are laid over the land, so the same place is
                // sampled many times over.
                kept.entry(voxel(point, spacing)).or_insert(point);
            });
        }
    }
    let mut points: Vec<Point3> = kept.into_values().collect();
    // A stable order, so that the same map writes the same files.
    points.sort_by(|p, q| {
        (p.x, p.y, p.z)
            .partial_cmp(&(q.x, q.y, q.z))
            .unwrap_or(std::cmp::Ordering::Equal)
    });
    points
}

/// A barycentric grid over triangle `abc`, `spacing` apart along its longest edge.
fn triangle(a: Point3, b: Point3, c: Point3, spacing: f64, mut each: impl FnMut(Point3)) {
    let longest = a.distance_to(b).max(b.distance_to(c)).max(c.distance_to(a));
    let n = (longest / spacing).ceil().max(1.0) as usize;
    for i in 0..=n {
        for j in 0..=(n - i) {
            let (u, v) = (i as f64 / n as f64, j as f64 / n as f64);
            each(Point3::new(
                a.x + (b.x - a.x) * u + (c.x - a.x) * v,
                a.y + (b.y - a.y) * u + (c.y - a.y) * v,
                a.z + (b.z - a.z) * u + (c.z - a.z) * v,
            ));
        }
    }
}

fn voxel(point: Point3, size: f64) -> [i64; 3] {
    [
        (point.x / size).floor() as i64,
        (point.y / size).floor() as i64,
        (point.z / size).floor() as i64,
    ]
}

/// Writes `points` — already in Autoware's frame — as a divided point-cloud map
/// under `directory`.
pub fn write(
    points: &[[f64; 3]],
    directory: impl AsRef<Path>,
    config: &PointCloudConfig,
) -> Result<PointCloudMap, ExportError> {
    let root = directory.as_ref();
    let cells_dir = root.join(DIRECTORY);
    fs::create_dir_all(&cells_dir).map_err(|error| io(&cells_dir, error))?;

    let size = config.cell_size;
    let mut cells: BTreeMap<(i64, i64), Vec<[f64; 3]>> = BTreeMap::new();
    for point in points {
        let cell = (
            (point[0] / size).floor() as i64,
            (point[1] / size).floor() as i64,
        );
        cells.entry(cell).or_default().push(*point);
    }

    let mut metadata = format!("x_resolution: {size:?}\ny_resolution: {size:?}\n");
    for ((ix, iy), cell) in &cells {
        let name = format!("{ix}_{iy}.pcd");
        let path = cells_dir.join(&name);
        fs::write(&path, pcd(cell)).map_err(|error| io(&path, error))?;
        // Where the cell starts: the loader takes a cell to run one resolution on
        // from here in x and in y.
        let _ = writeln!(
            metadata,
            "{name}: [{:?}, {:?}]",
            *ix as f64 * size,
            *iy as f64 * size
        );
    }
    let metadata_path = root.join(METADATA);
    fs::write(&metadata_path, metadata).map_err(|error| io(&metadata_path, error))?;

    Ok(PointCloudMap {
        directory: cells_dir,
        metadata: metadata_path,
        cells: cells.len(),
        points: points.len(),
    })
}

/// One `.pcd`: binary, `x y z` as 32-bit floats, which is what Autoware's maps are.
pub fn pcd(points: &[[f64; 3]]) -> Vec<u8> {
    let n = points.len();
    let mut out = format!(
        "# .PCD v0.7 - Point Cloud Data file format\n\
         VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n\
         WIDTH {n}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {n}\nDATA binary\n"
    )
    .into_bytes();
    out.reserve(12 * n);
    for point in points {
        for value in point {
            out.extend_from_slice(&(*value as f32).to_le_bytes());
        }
    }
    out
}

fn io(path: &Path, error: std::io::Error) -> ExportError {
    ExportError::Io(format!("{}: {error}", path.display()))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::tags::Role;

    fn square(size: f64, z: f64) -> Mesh {
        let mut mesh = Mesh::new("T_Terrain_Ground_0", Role::Terrain, 0);
        mesh.positions = vec![
            Point3::new(0.0, 0.0, z),
            Point3::new(size, 0.0, z),
            Point3::new(size, size, z),
            Point3::new(0.0, size, z),
        ];
        mesh.triangles = vec![[0, 1, 2], [0, 2, 3]];
        mesh
    }

    #[test]
    fn a_surface_is_sampled_once_per_voxel_at_its_own_height() {
        let points = sample(&[square(10.0, 2.0)], 0.5);
        // A 10 m square at 0.5 m is 21 x 21 voxel columns at most, one point each.
        assert!(points.len() <= 21 * 21, "{}", points.len());
        assert!(points.len() >= 20 * 20, "{}", points.len());
        assert!(points.iter().all(|p| (p.z - 2.0).abs() < 1e-9));
        // Overlapping surfaces add nothing where they coincide.
        let twice = sample(&[square(10.0, 2.0), square(10.0, 2.0)], 0.5);
        assert_eq!(twice.len(), points.len());
    }

    #[test]
    fn the_map_is_divided_the_way_autowares_loader_reads_it() {
        let dir = tempfile::tempdir().unwrap();
        let points = [[1.0, 1.0, 0.0], [21.0, -3.0, 0.5], [22.0, -4.0, 0.5]];
        let written = write(&points, dir.path(), &PointCloudConfig::default()).unwrap();
        assert_eq!((written.cells, written.points), (2, 3));
        let metadata = fs::read_to_string(dir.path().join(METADATA)).unwrap();
        assert!(
            metadata.starts_with("x_resolution: 20.0\ny_resolution: 20.0\n"),
            "{metadata}"
        );
        assert!(metadata.contains("0_0.pcd: [0.0, 0.0]\n"), "{metadata}");
        assert!(metadata.contains("1_-1.pcd: [20.0, -20.0]\n"), "{metadata}");

        let cell = fs::read(dir.path().join(DIRECTORY).join("1_-1.pcd")).unwrap();
        let body = cell
            .split_at(
                cell.windows(12)
                    .position(|w| w == b"DATA binary\n")
                    .unwrap()
                    + 12,
            )
            .1;
        assert_eq!(body.len(), 2 * 12);
        let x = f32::from_le_bytes(body[0..4].try_into().unwrap());
        assert_eq!(x, 21.0);
        let header = String::from_utf8_lossy(&cell[..cell.len() - body.len()]).to_string();
        assert!(
            header.contains("POINTS 2\n") && header.contains("FIELDS x y z\n"),
            "{header}"
        );
    }
}
