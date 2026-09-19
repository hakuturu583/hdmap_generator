//! The height of the ground around the road network.
//!
//! A road network says nothing about the shape of the land it runs through, and
//! nothing here pretends otherwise: the ground is the flattest thing that stays
//! under every road at the road's own height, which is what a road network *does*
//! say about its land. It is a height field — a grid over the network's extent plus
//! a margin, whose height at each vertex is taken from the roads near it — that the
//! land in [`crate::terrain`] reads its heights off, away from the roads.
//!
//! Each vertex takes the *lowest* road vertex within a cell's reach of it, a little
//! under — never an average, because an average of a road climbing away from a
//! vertex sits above the road at that vertex, and the land would come up through
//! the road there.

use roadgen_core::geometry::{Curve3, Point3};
use roadgen_core::map::Map;

use crate::mesh::Mesh;
use crate::surfaces::SurfaceConfig;
use crate::tags::Role;

/// How far under the roads the ground sits, metres: more than a road can bend
/// under the chord between two ground vertices.
pub const DROP: f64 = 0.05;
/// How far apart the reference line is sampled for the ground, metres.
const SAMPLE_SPACING: f64 = 2.0;

/// The ground as a height field: a grid, with the triangulation the mesh will have.
pub struct Field {
    x0: f64,
    y0: f64,
    cell: f64,
    columns: usize,
    rows: usize,
    /// Row-major from the south-west corner, `(columns + 1) * (rows + 1)` of them.
    heights: Vec<f64>,
}

impl Field {
    /// Lays the field out under `surface` — the roads built so far — reaching
    /// `config.ground_extent` past them and past every building. `None` when there
    /// is nothing to lay it under or the configuration asks for none.
    pub fn under(map: &Map, surface: &[Mesh], config: &SurfaceConfig) -> Option<Field> {
        if config.ground_extent <= 0.0 || config.ground_cell <= 0.0 {
            return None;
        }
        // Every vertex a vehicle drives on: the roads and the gutters. Pavements
        // stand above the road beside them and are left out, or the ground under
        // a pavement would come up through the road.
        let mut samples: Vec<Point3> = surface
            .iter()
            .filter(|mesh| matches!(mesh.role, Role::Road | Role::Gutter))
            .flat_map(|mesh| mesh.positions.iter().copied())
            .collect();
        if samples.is_empty() {
            return None;
        }
        // And the reference lines, densely: a straight road's surface has vertices
        // at its two ends and nowhere between, and a ground that read only those
        // would take the road's far end's height all the way along it. A polyline
        // is already as dense as it is going to get, and its own samples are cheaper
        // than resampling it.
        let sampling = map.metadata.sampling;
        for road in map.roads.iter() {
            let line = &road.reference_line;
            if matches!(line, Curve3::Polyline(_) | Curve3::Bezier(_)) {
                if let Ok(points) = line.samples(sampling) {
                    samples.extend(points.into_iter().map(|sample| sample.point));
                }
                continue;
            }
            let Ok(length) = line.horizontal_length() else {
                continue;
            };
            let steps = (length / SAMPLE_SPACING).ceil().max(1.0) as usize;
            for step in 0..=steps {
                let station = length * step as f64 / steps as f64;
                if let Ok(sample) = line.sample_at(station, sampling) {
                    samples.push(sample.point);
                }
            }
        }

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
        // The verge is what the ground is drawn up to, and it reaches past the road.
        let margin = config.ground_extent + config.verge_width;
        let cell = config.ground_cell;
        let x0 = min[0] - margin;
        let y0 = min[1] - margin;
        let columns = ((max[0] + margin - x0) / cell).ceil().max(1.0) as usize;
        let rows = ((max[1] + margin - y0) / cell).ceil().max(1.0) as usize;
        let stride = columns + 1;

        // A ground vertex governs the triangles around it, which reach a cell's
        // diagonal; every road sample that could be over one of them is looked at,
        // and the lowest wins. Each sample reaches only the few vertices within
        // that diagonal of it, so the samples are scattered onto the grid rather
        // than the grid searched from every vertex.
        let reach = cell * std::f64::consts::SQRT_2;
        let mut heights = vec![f64::INFINITY; stride * (rows + 1)];
        let span = |centre: f64, count: usize| -> std::ops::RangeInclusive<usize> {
            let low = ((centre - reach) / cell).ceil().max(0.0) as usize;
            let high = ((centre + reach) / cell).floor().max(0.0) as usize;
            low..=high.min(count)
        };
        for point in &samples {
            for j in span(point.y - y0, rows) {
                for i in span(point.x - x0, columns) {
                    let x = x0 + cell * i as f64;
                    let y = y0 + cell * j as f64;
                    if (point.x - x).powi(2) + (point.y - y).powi(2) <= reach * reach {
                        let slot = &mut heights[j * stride + i];
                        *slot = slot.min(point.z);
                    }
                }
            }
        }
        // The rest of the grid — most of it, out in the margin — takes the height
        // of the nearest vertex that has one, ring by ring outwards: flat away from
        // the roads, and at the height of whichever road is closest.
        let mut queue: std::collections::VecDeque<usize> = (0..heights.len())
            .filter(|&index| heights[index].is_finite())
            .collect();
        while let Some(index) = queue.pop_front() {
            let (i, j) = (index % stride, index / stride);
            let neighbours = [
                (i > 0).then(|| index - 1),
                (i < columns).then(|| index + 1),
                (j > 0).then(|| index - stride),
                (j < rows).then(|| index + stride),
            ];
            for next in neighbours.into_iter().flatten() {
                if !heights[next].is_finite() {
                    heights[next] = heights[index];
                    queue.push_back(next);
                }
            }
        }
        for height in &mut heights {
            *height -= DROP;
        }
        Some(Field {
            x0,
            y0,
            cell,
            columns,
            rows,
            heights,
        })
    }

    fn vertex(&self, i: usize, j: usize) -> Point3 {
        Point3::new(
            self.x0 + self.cell * i as f64,
            self.y0 + self.cell * j as f64,
            self.heights[j * (self.columns + 1) + i],
        )
    }

    /// The ground's height at a position, interpolated on the triangles the mesh
    /// is made of, so that a point laid at this height is on the mesh and not
    /// merely near it. Beyond the grid, the edge's height.
    pub fn height(&self, x: f64, y: f64) -> f64 {
        let fx = ((x - self.x0) / self.cell).clamp(0.0, self.columns as f64 - 1e-9);
        let fy = ((y - self.y0) / self.cell).clamp(0.0, self.rows as f64 - 1e-9);
        let (i, j) = (fx.floor() as usize, fy.floor() as usize);
        let (u, v) = (fx - i as f64, fy - j as f64);
        // The cell is split along the diagonal from its north-west corner to its
        // south-east one.
        let (sw, se, nw, ne) = (
            self.vertex(i, j).z,
            self.vertex(i + 1, j).z,
            self.vertex(i, j + 1).z,
            self.vertex(i + 1, j + 1).z,
        );
        if u + v <= 1.0 {
            // South-west of the diagonal: the triangle sw, se, nw.
            sw + (se - sw) * u + (nw - sw) * v
        } else {
            // North-east of it: the triangle ne, nw, se.
            ne + (nw - ne) * (1.0 - u) + (se - ne) * (1.0 - v)
        }
    }

    /// Every vertex of the grid, at its height, row by row from the south-west.
    pub fn vertices(&self) -> impl Iterator<Item = Point3> + '_ {
        (0..=self.rows).flat_map(move |j| (0..=self.columns).map(move |i| self.vertex(i, j)))
    }
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

    fn build(map: &ValidatedMap, config: &SurfaceConfig) -> Option<Mesh> {
        crate::surfaces::build(map, "g", config)
            .into_iter()
            .find(|mesh| mesh.role == Role::Terrain)
    }

    #[test]
    fn the_land_reaches_the_margin_past_the_network() {
        let map = map();
        let mesh = build(&map, &config()).expect("a terrain");
        let span = |pick: fn(&Point3) -> f64| {
            let values: Vec<f64> = mesh.positions.iter().map(pick).collect();
            (
                values.iter().cloned().fold(f64::MAX, f64::min),
                values.iter().cloned().fold(f64::MIN, f64::max),
            )
        };
        let verge = config().verge_width;
        let (x_min, x_max) = span(|p| p.x);
        let (y_min, y_max) = span(|p| p.y);
        // The one lane is on the right of the reference line: y from 0 to -3.5.
        assert!((x_min - (-100.0 - verge)).abs() < 1e-9 && x_max >= 300.0 + verge);
        assert!((y_min - (-100.0 - 3.5 - verge)).abs() < 1e-9 && y_max >= 100.0 + verge);
        assert!(mesh.name.starts_with("g_Terrain_Ground_"));
        assert_eq!(crate::tags::label_of(&mesh.name), crate::Label::Terrain);
        assert!(
            mesh.normals().iter().all(|n| n.z > 0.5),
            "the ground faces up"
        );
    }

    #[test]
    fn the_field_sits_just_under_the_road_and_holds_its_height_far_out() {
        let map = map();
        let config = config();
        let roads = crate::surfaces::build(
            &map,
            "g",
            &SurfaceConfig {
                ground_extent: 0.0,
                ..config
            },
        );
        let field = Field::under(&map, &roads, &config).expect("a field");
        // Under the road at x = 100 the road is at z = 15; the field a little under
        // the lowest road vertex within a cell's reach, which on a 5 % grade is up
        // to 14 m back down the road.
        let under = field.height(100.0, -1.75);
        assert!(
            (15.0 - DROP - 1.0..=15.0 - DROP + 0.5).contains(&under),
            "ground under the road at {under}"
        );
        // Far from it the field holds the nearest road's height rather than
        // dropping to zero or drifting off.
        let far = field.height(100.0, 100.0);
        assert!(far > 13.0 && far < 17.0, "ground far out at {far}");
    }

    #[test]
    fn the_field_reads_back_the_height_its_vertices_have() {
        let map = map();
        let config = config();
        let roads = crate::surfaces::build(
            &map,
            "g",
            &SurfaceConfig {
                ground_extent: 0.0,
                ..config
            },
        );
        let field = Field::under(&map, &roads, &config).expect("a field");
        let vertices: Vec<Point3> = field.vertices().collect();
        for point in vertices.iter().step_by(7) {
            assert!((field.height(point.x, point.y) - point.z).abs() < 1e-9);
        }
        // And between vertices it is on the cell's own two triangles: a point on
        // the first cell's diagonal, and one on each edge, read back exactly.
        let stride = field.columns + 1;
        for (a, b) in [
            (vertices[stride], vertices[1]),
            (vertices[0], vertices[1]),
            (vertices[0], vertices[stride]),
        ] {
            let between = a.lerp(b, 0.3);
            assert!((field.height(between.x, between.y) - between.z).abs() < 1e-9);
        }
    }

    #[test]
    fn no_extent_means_no_field_but_still_a_verge() {
        let config = SurfaceConfig {
            ground_extent: 0.0,
            ..SurfaceConfig::default()
        };
        let map = map();
        assert!(Field::under(&map, &[], &config).is_none());
        // The land is then the verges alone, level with the road they are beside.
        let mesh = build(&map, &config).expect("a verge");
        let (x_min, x_max) = mesh
            .positions
            .iter()
            .fold((f64::MAX, f64::MIN), |(lo, hi), p| {
                (lo.min(p.x), hi.max(p.x))
            });
        assert!(
            x_min >= -1e-9 && x_max <= 200.0 + 1e-9,
            "a verge past the road's ends"
        );
    }
}
