//! The land: one continuous surface from every road's edge out to where a lidar
//! reaches.
//!
//! A road network says nothing about the shape of the country it runs through, and
//! a simulator needs a ground all the same: a lidar reaches a hundred metres or
//! more, and a vehicle that leaves the road should land on something. What it must
//! not be is several somethings. A grass strip beside each road laid over a grid
//! under all of them is two surfaces wherever they overlap and a step wherever they
//! do not quite meet — strips of neighbouring roads cross each other at a junction,
//! a strip round a tight corner folds over itself, and a lidar return or a wheel
//! finds every seam. So the land is one mesh, triangulated once, that meets every
//! road's edge exactly and has nothing under the roads at all.
//!
//! It is a constrained Delaunay triangulation of three kinds of point. The **rails**
//! are the outer edges of every road section, vertex for vertex the same points the
//! road surface ends on, and they are the constraints: no triangle crosses a road's
//! edge. The **toes** are those edges pushed `verge_width` out on the side that has
//! land, at the ground's height there, so the grass slopes down from the road the
//! way a verge does. And the **grid** is the ground's own height field past the
//! verges, out to `ground_extent`. What is inside a road's outline is cut away
//! afterwards, so the junction's roads and the pavements round it sit in a hole cut
//! to their shape, and everything else is one surface.

use std::collections::HashMap;

use roadgen_core::geometry::Point3;
use spade::handles::FixedVertexHandle;
use spade::{ConstrainedDelaunayTriangulation, HasPosition, Point2, Triangulation};

use crate::ground::Field;
use crate::materials;
use crate::mesh::Mesh;
use crate::surfaces::Ordinals;
use crate::tags::{mesh_name, Role};

/// One side of one road section, as the land sees it.
pub struct Flank {
    /// The road's edge, at the height the land meets it: the top of a pavement
    /// that faces land, the foot of one that faces a junction.
    pub rail: Vec<Point3>,
    /// The verge's outer edge, one entry per rail vertex: `verge_width` out from
    /// the rail on the side that has land, and `None` where there is no room for
    /// it — on a side that faces a junction, or on the inside of a bend tighter
    /// than the verge is wide, where a toe would fold back over the rail. The
    /// heights are the land's to set.
    pub toe: Vec<Option<Point3>>,
}

/// What the land is built round: every road section's two flanks and its outline.
pub struct Land {
    /// How far from a road's edge the grid keeps back, so that the slope down from
    /// the edge is the verge's and not whatever the nearest grid vertex makes it.
    verge: f64,
    flanks: Vec<Flank>,
    /// Each section's outline in plan, which the land is cut out of.
    footprints: Vec<Polygon>,
}

impl Land {
    pub fn new(verge_width: f64) -> Land {
        Land {
            verge: verge_width.max(0.0),
            flanks: Vec::new(),
            footprints: Vec::new(),
        }
    }

    /// Adds one road section: its left and right flanks, whose rails also bound
    /// its footprint.
    pub fn section(&mut self, left: Flank, right: Flank) {
        let outline = left
            .rail
            .iter()
            .chain(right.rail.iter().rev())
            .map(|point| [point.x, point.y])
            .collect();
        self.footprints.push(Polygon::new(outline));
        self.flanks.push(left);
        self.flanks.push(right);
    }

    /// The land as one mesh, tagged as terrain. `None` when there is no road to
    /// build it round.
    pub fn mesh(
        &self,
        field: Option<&Field>,
        map_name: &str,
        ordinals: &mut Ordinals,
    ) -> Option<Mesh> {
        if self.flanks.iter().all(|flank| flank.rail.len() < 2) {
            return None;
        }
        let roads = Index::new(&self.footprints);
        // The grid keeps a verge's width back from every rail as well as out of
        // the roads, so the grass gets its slope from the toes — or, where a rail
        // has none, from grid vertices a verge away — rather than from whichever
        // grid vertex happens to be nearest a kerb.
        let rails = Rails::new(
            self.flanks.iter().map(|flank| flank.rail.as_slice()),
            self.verge,
        );

        let mut cdt: ConstrainedDelaunayTriangulation<Vertex> =
            ConstrainedDelaunayTriangulation::new();
        // Free points first and the rails last, so that a rail that lands on the
        // same spot as a grid or toe point keeps its own height: the road's edge is
        // the one height here that is not negotiable.
        if let Some(field) = field {
            for point in field.vertices() {
                if !roads.contains(point.x, point.y) && !rails.within(point.x, point.y, self.verge)
                {
                    let _ = cdt.insert(Vertex::from(point));
                }
            }
        }
        for flank in &self.flanks {
            for point in flank.toe.iter().flatten() {
                if roads.contains(point.x, point.y) {
                    continue;
                }
                let z = field.map_or(point.z, |field| field.height(point.x, point.y));
                let _ = cdt.insert(Vertex {
                    x: point.x,
                    y: point.y,
                    z,
                });
            }
        }
        for flank in &self.flanks {
            let mut previous: Option<(FixedVertexHandle, Point3)> = None;
            for &point in &flank.rail {
                let Ok(handle) = cdt.insert(Vertex::from(point)) else {
                    continue;
                };
                if let Some((from, start)) = previous {
                    if from != handle {
                        // Rails of different roads cross inside a junction; where
                        // they do, both are split at the crossing, which takes the
                        // height of the rail being added at that point.
                        cdt.add_constraint_and_split(from, handle, |at| {
                            let run = start.horizontal_distance_to(point);
                            let t = if run < 1e-12 {
                                0.0
                            } else {
                                ((at.x - start.x) * (point.x - start.x)
                                    + (at.y - start.y) * (point.y - start.y))
                                    / (run * run)
                            };
                            Vertex {
                                x: at.x,
                                y: at.y,
                                z: start.z + (point.z - start.z) * t.clamp(0.0, 1.0),
                            }
                        });
                    }
                }
                previous = Some((handle, point));
            }
        }

        // Every triangle outside the roads, with the vertices it needs and no other.
        let mut positions = Vec::new();
        let mut renumbered: HashMap<usize, u32> = HashMap::new();
        let mut triangles = Vec::new();
        for face in cdt.inner_faces() {
            let corners = face.vertices();
            let centre = corners.iter().fold([0.0, 0.0], |sum, corner| {
                let at = corner.position();
                [sum[0] + at.x / 3.0, sum[1] + at.y / 3.0]
            });
            if roads.contains(centre[0], centre[1]) {
                continue;
            }
            // Spade winds an inner face anticlockwise in the plane, which is
            // anticlockwise seen from above: the face is up.
            let triangle = corners.map(|corner| {
                *renumbered.entry(corner.fix().index()).or_insert_with(|| {
                    let vertex = corner.data();
                    positions.push(Point3::new(vertex.x, vertex.y, vertex.z));
                    positions.len() as u32 - 1
                })
            });
            triangles.push(triangle);
        }
        if triangles.is_empty() {
            return None;
        }
        let mut mesh = Mesh::new(
            mesh_name(map_name, Role::Terrain, ordinals.take(Role::Terrain)),
            Role::Terrain,
            materials::GRASS,
        );
        mesh.sheet(&positions, &triangles, 0);
        Some(mesh)
    }
}

struct Vertex {
    x: f64,
    y: f64,
    z: f64,
}

impl From<Point3> for Vertex {
    fn from(point: Point3) -> Vertex {
        Vertex {
            x: point.x,
            y: point.y,
            z: point.z,
        }
    }
}

impl HasPosition for Vertex {
    type Scalar = f64;

    fn position(&self) -> Point2<f64> {
        Point2::new(self.x, self.y)
    }
}

/// A closed outline in plan, with the box round it.
struct Polygon {
    points: Vec<[f64; 2]>,
    min: [f64; 2],
    max: [f64; 2],
}

impl Polygon {
    fn new(points: Vec<[f64; 2]>) -> Polygon {
        let mut min = [f64::INFINITY; 2];
        let mut max = [f64::NEG_INFINITY; 2];
        for point in &points {
            for axis in 0..2 {
                min[axis] = min[axis].min(point[axis]);
                max[axis] = max[axis].max(point[axis]);
            }
        }
        Polygon { points, min, max }
    }

    /// Whether the point is inside, by the crossings of a ray cast from it.
    fn contains(&self, x: f64, y: f64) -> bool {
        if x < self.min[0] || x > self.max[0] || y < self.min[1] || y > self.max[1] {
            return false;
        }
        let mut inside = false;
        let count = self.points.len();
        for i in 0..count {
            let a = self.points[i];
            let b = self.points[(i + 1) % count];
            if (a[1] > y) != (b[1] > y) {
                let cross = (b[0] - a[0]) * (y - a[1]) / (b[1] - a[1]) + a[0];
                if x < cross {
                    inside = !inside;
                }
            }
        }
        inside
    }
}

/// Every rail's segments, bucketed by cell, so that a point can ask how near the
/// nearest road edge is without measuring itself against every edge on the map.
struct Rails {
    cell: f64,
    buckets: HashMap<(i64, i64), Vec<Segment>>,
}

/// One straight piece of a rail, in plan.
type Segment = ([f64; 2], [f64; 2]);

impl Rails {
    /// `reach` is the farthest any query will ask about, and sizes the cells so
    /// that a query never has to look past the cells round its own.
    fn new<'a>(rails: impl Iterator<Item = &'a [Point3]>, reach: f64) -> Rails {
        let cell = reach.max(1.0);
        let mut buckets: HashMap<(i64, i64), Vec<Segment>> = HashMap::new();
        for rail in rails {
            for pair in rail.windows(2) {
                let (a, b) = ([pair[0].x, pair[0].y], [pair[1].x, pair[1].y]);
                let (x0, x1) = (
                    (a[0].min(b[0]) / cell).floor() as i64,
                    (a[0].max(b[0]) / cell).floor() as i64,
                );
                let (y0, y1) = (
                    (a[1].min(b[1]) / cell).floor() as i64,
                    (a[1].max(b[1]) / cell).floor() as i64,
                );
                for i in x0..=x1 {
                    for j in y0..=y1 {
                        buckets.entry((i, j)).or_default().push((a, b));
                    }
                }
            }
        }
        Rails { cell, buckets }
    }

    /// Whether any rail passes within `distance` of the point, for a distance no
    /// more than the reach the rails were bucketed for.
    fn within(&self, x: f64, y: f64, distance: f64) -> bool {
        if distance <= 0.0 {
            return false;
        }
        debug_assert!(distance <= self.cell);
        let (i, j) = (
            (x / self.cell).floor() as i64,
            (y / self.cell).floor() as i64,
        );
        let square = distance * distance;
        (i - 1..=i + 1).any(|i| {
            (j - 1..=j + 1).any(|j| {
                self.buckets.get(&(i, j)).is_some_and(|segments| {
                    segments.iter().any(|&(a, b)| {
                        let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
                        let length = dx * dx + dy * dy;
                        let t = if length < 1e-18 {
                            0.0
                        } else {
                            (((x - a[0]) * dx + (y - a[1]) * dy) / length).clamp(0.0, 1.0)
                        };
                        let (px, py) = (a[0] + dx * t - x, a[1] + dy * t - y);
                        px * px + py * py < square
                    })
                })
            })
        })
    }
}

/// Polygons bucketed by the cells their boxes cover, so that a point is tested
/// against the few near it rather than all of them.
struct Index<'a> {
    polygons: &'a [Polygon],
    cell: f64,
    buckets: HashMap<(i64, i64), Vec<usize>>,
}

impl<'a> Index<'a> {
    const CELL: f64 = 25.0;

    fn new(polygons: &'a [Polygon]) -> Index<'a> {
        let cell = Self::CELL;
        let mut buckets: HashMap<(i64, i64), Vec<usize>> = HashMap::new();
        for (index, polygon) in polygons.iter().enumerate() {
            if !polygon.min[0].is_finite() {
                continue;
            }
            let (x0, x1) = (
                (polygon.min[0] / cell).floor() as i64,
                (polygon.max[0] / cell).floor() as i64,
            );
            let (y0, y1) = (
                (polygon.min[1] / cell).floor() as i64,
                (polygon.max[1] / cell).floor() as i64,
            );
            for i in x0..=x1 {
                for j in y0..=y1 {
                    buckets.entry((i, j)).or_default().push(index);
                }
            }
        }
        Index {
            polygons,
            cell,
            buckets,
        }
    }

    fn contains(&self, x: f64, y: f64) -> bool {
        let key = (
            (x / self.cell).floor() as i64,
            (y / self.cell).floor() as i64,
        );
        self.buckets.get(&key).is_some_and(|near| {
            near.iter()
                .any(|&index| self.polygons[index].contains(x, y))
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn flank(rail: Vec<Point3>, toe: Option<Vec<Point3>>) -> Flank {
        let toe = match toe {
            Some(toe) => toe.into_iter().map(Some).collect(),
            None => vec![None; rail.len()],
        };
        Flank { rail, toe }
    }

    fn straight_road() -> Land {
        // A road 40 m long and 8 m wide along x, with a verge each side.
        let rail = |y: f64| -> Vec<Point3> {
            (0..=4)
                .map(|i| Point3::new(i as f64 * 10.0, y, 0.0))
                .collect()
        };
        let mut land = Land::new(8.0);
        land.section(
            flank(rail(4.0), Some(rail(12.0))),
            flank(rail(-4.0), Some(rail(-12.0))),
        );
        land
    }

    #[test]
    fn the_land_meets_the_road_at_every_edge_vertex_and_has_nothing_under_it() {
        let land = straight_road();
        let mesh = land
            .mesh(None, "t", &mut Ordinals::default())
            .expect("a terrain");
        for i in 0..=4 {
            for y in [4.0, -4.0] {
                let edge = Point3::new(i as f64 * 10.0, y, 0.0);
                assert!(
                    mesh.positions.iter().any(|p| p.distance_to(edge) < 1e-9),
                    "the land misses the road's edge at {edge:?}"
                );
            }
        }
        for triangle in &mesh.triangles {
            let [a, b, c] = triangle.map(|i| mesh.positions[i as usize]);
            let centre_y = (a.y + b.y + c.y) / 3.0;
            assert!(
                centre_y.abs() > 4.0 - 1e-9,
                "a land triangle under the road"
            );
            let normal = (b - a).cross(c - a);
            assert!(normal.z > 0.0, "a land triangle facing down");
        }
    }

    #[test]
    fn a_polygon_knows_its_inside() {
        let square = Polygon::new(vec![[0.0, 0.0], [10.0, 0.0], [10.0, 10.0], [0.0, 10.0]]);
        assert!(square.contains(5.0, 5.0));
        assert!(!square.contains(15.0, 5.0));
        assert!(!square.contains(5.0, -1.0));
        let index = Index::new(std::slice::from_ref(&square));
        assert!(index.contains(9.0, 9.0));
        assert!(!index.contains(-9.0, 9.0));
    }

    #[test]
    fn the_rails_know_what_is_near_them() {
        let rail = vec![Point3::new(0.0, 0.0, 0.0), Point3::new(100.0, 0.0, 0.0)];
        let rails = Rails::new(std::iter::once(rail.as_slice()), 8.0);
        assert!(rails.within(50.0, 7.9, 8.0));
        assert!(!rails.within(50.0, 8.1, 8.0));
        assert!(
            rails.within(-3.0, 0.0, 8.0),
            "the end of a rail is part of it"
        );
        assert!(!rails.within(50.0, 30.0, 8.0));
    }

    #[test]
    fn two_roads_crossing_still_make_one_land() {
        // Two roads through one another, like connectors in a junction: their
        // rails cross, and the land is cut round the union of the two.
        let along = |y: f64| -> Vec<Point3> {
            (0..=4)
                .map(|i| Point3::new(i as f64 * 10.0 - 20.0, y, 0.0))
                .collect()
        };
        let across = |x: f64| -> Vec<Point3> {
            (0..=4)
                .map(|i| Point3::new(x, i as f64 * 10.0 - 20.0, 0.0))
                .collect()
        };
        let mut land = Land::new(8.0);
        land.section(flank(along(4.0), None), flank(along(-4.0), None));
        land.section(flank(across(-4.0), None), flank(across(4.0), None));
        let mesh = land
            .mesh(None, "t", &mut Ordinals::default())
            .expect("a terrain");
        for triangle in &mesh.triangles {
            let [a, b, c] = triangle.map(|i| mesh.positions[i as usize]);
            let (x, y) = ((a.x + b.x + c.x) / 3.0, (a.y + b.y + c.y) / 3.0);
            assert!(
                y.abs() > 4.0 - 1e-9 && x.abs() > 4.0 - 1e-9,
                "a land triangle at ({x}, {y}) is on a road"
            );
        }
    }
}
