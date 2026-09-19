//! The ground under and around the road network.
//!
//! A road network says nothing about the shape of the land it runs through, and the
//! verge beside each road is as far as it can honestly go. A simulator needs more
//! than that: a lidar reaches a hundred metres or more, and a vehicle that leaves
//! the verge should land on something. So the surface is finished with a **ground**
//! — one mesh, a grid over the network's extent plus a margin, whose height at each
//! vertex is taken from the nearest stretches of road. It is not terrain, and it is
//! not pretending to be: it is the flattest thing that meets every road at the
//! road's own height, which is what a road network *does* say about its land.
//!
//! It sits a couple of centimetres under the verges and the roads, so the two never
//! fight for the same depth value, and the step at the verge's edge is smaller than
//! anything a sensor resolves.

use roadgen_core::geometry::Point3;
use roadgen_core::map::Map;

use crate::materials;
use crate::mesh::Mesh;
use crate::surfaces::{Ordinals, SurfaceConfig};
use crate::tags::{mesh_name, Role};

/// How far under the roads the ground sits, metres.
pub const DROP: f64 = 0.02;
/// How many road samples a ground vertex takes its height from.
const NEIGHBOURS: usize = 6;

/// The ground mesh, or `None` when the map has no road to take a height from or
/// the configuration asks for none.
pub fn ground(
    map: &Map,
    map_name: &str,
    config: &SurfaceConfig,
    ordinals: &mut Ordinals,
) -> Option<Mesh> {
    if config.ground_extent <= 0.0 || config.ground_cell <= 0.0 {
        return None;
    }
    let sampling = map.metadata.sampling;
    let mut samples: Vec<Point3> = Vec::new();
    for road in map.roads.iter() {
        if let Ok(points) = road.reference_line.samples(sampling) {
            samples.extend(points.into_iter().map(|sample| sample.point));
        }
    }
    if samples.is_empty() {
        return None;
    }

    // The extent: every road and every building, and the margin beyond them.
    let mut min = [f64::INFINITY; 2];
    let mut max = [f64::NEG_INFINITY; 2];
    let mut widen = |point: &Point3| {
        min[0] = min[0].min(point.x);
        min[1] = min[1].min(point.y);
        max[0] = max[0].max(point.x);
        max[1] = max[1].max(point.y);
    };
    samples.iter().for_each(&mut widen);
    for part in map.building_parts.iter() {
        part.solid.footprint.points().iter().for_each(&mut widen);
    }
    let margin = config.ground_extent;
    let (x0, y0) = (min[0] - margin, min[1] - margin);
    let (x1, y1) = (max[0] + margin, max[1] + margin);
    let cell = config.ground_cell;
    let columns = ((x1 - x0) / cell).ceil().max(1.0) as usize;
    let rows = ((y1 - y0) / cell).ceil().max(1.0) as usize;

    let height = |x: f64, y: f64| -> f64 {
        // Inverse-distance weighting over the nearest few samples: smooth between
        // roads at different heights, and exactly a road's height on the road.
        let mut nearest: Vec<(f64, f64)> = samples
            .iter()
            .map(|point| ((point.x - x).powi(2) + (point.y - y).powi(2), point.z))
            .collect();
        let keep = NEIGHBOURS.min(nearest.len());
        if keep < nearest.len() {
            nearest.select_nth_unstable_by(keep - 1, |a, b| a.0.total_cmp(&b.0));
            nearest.truncate(keep);
        }
        let mut weighted = 0.0;
        let mut weights = 0.0;
        for (distance_squared, z) in nearest {
            let weight = 1.0 / (distance_squared + 1e-6);
            weighted += weight * z;
            weights += weight;
        }
        weighted / weights - DROP
    };

    // Rows of vertices, south to north; each pair of rows is one strip. The
    // northern row is the strip's left rail so that the surface faces up.
    let row = |j: usize| -> Vec<Point3> {
        let y = y0 + (y1 - y0) * j as f64 / rows as f64;
        (0..=columns)
            .map(|i| {
                let x = x0 + (x1 - x0) * i as f64 / columns as f64;
                Point3::new(x, y, height(x, y))
            })
            .collect()
    };
    let mut mesh = Mesh::new(
        mesh_name(map_name, Role::Ground, ordinals.take(Role::Ground)),
        Role::Ground,
        materials::GRASS,
    );
    let mut south = row(0);
    for j in 1..=rows {
        let north = row(j);
        mesh.strip(&north, &south, 0);
        south = north;
    }
    (!mesh.is_empty()).then_some(mesh)
}

#[cfg(test)]
mod tests {
    use super::*;
    use roadgen_core::prelude::*;

    fn map() -> ValidatedMap {
        let mut builder = MapBuilder::new(MapMetadata {
            name: Some("g".into()),
            ..MapMetadata::default()
        });
        let lanes = vec![LaneSpec::new(
            PositiveWidth::new(3.5).unwrap(),
            Direction::Forward,
        )];
        builder
            .add_road(
                RoadSpec::line(
                    Point3::new(0.0, 0.0, 10.0),
                    Point3::new(200.0, 0.0, 20.0),
                    lanes,
                )
                .unwrap(),
            )
            .unwrap();
        builder.finish().unwrap().validate().unwrap()
    }

    fn config() -> SurfaceConfig {
        SurfaceConfig {
            ground_extent: 100.0,
            ground_cell: 10.0,
            ..SurfaceConfig::default()
        }
    }

    #[test]
    fn the_ground_reaches_the_margin_past_the_network() {
        let map = map();
        let mesh = ground(&map, "g", &config(), &mut Ordinals::default()).expect("a ground");
        let xs: Vec<f64> = mesh.positions.iter().map(|p| p.x).collect();
        let ys: Vec<f64> = mesh.positions.iter().map(|p| p.y).collect();
        let span = |v: &[f64]| {
            (
                v.iter().cloned().fold(f64::MAX, f64::min),
                v.iter().cloned().fold(f64::MIN, f64::max),
            )
        };
        assert_eq!(span(&xs), (-100.0, 300.0));
        assert_eq!(span(&ys), (-100.0, 100.0));
        assert_eq!(mesh.role, Role::Ground);
        assert!(mesh.name.starts_with("g_Terrain_Land_"));
        assert_eq!(crate::tags::label_of(&mesh.name), crate::Label::Terrain);
        assert!(
            mesh.normals().iter().all(|n| n.z > 0.5),
            "the ground faces up"
        );
    }

    #[test]
    fn the_ground_meets_each_road_just_under_its_own_height() {
        let map = map();
        let mesh = ground(&map, "g", &config(), &mut Ordinals::default()).expect("a ground");
        // Under the road at x = 100 the road is at z = 15; the ground a hair below.
        let under = mesh
            .positions
            .iter()
            .find(|p| (p.x - 100.0).abs() < 1e-6 && p.y.abs() < 1e-6)
            .expect("a vertex on the road's line");
        assert!(
            (under.z - (15.0 - DROP)).abs() < 0.05,
            "ground at {}",
            under.z
        );
        // Far from it the ground holds the nearest road's height rather than
        // dropping to zero or drifting off.
        let far = mesh
            .positions
            .iter()
            .find(|p| (p.x - 100.0).abs() < 1e-6 && (p.y - 100.0).abs() < 1e-6)
            .unwrap();
        assert!(far.z > 13.0 && far.z < 17.0, "ground far out at {}", far.z);
    }

    #[test]
    fn no_extent_means_no_ground() {
        let config = SurfaceConfig {
            ground_extent: 0.0,
            ..SurfaceConfig::default()
        };
        assert!(ground(&map(), "g", &config, &mut Ordinals::default()).is_none());
    }
}
