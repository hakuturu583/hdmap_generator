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
//! road surface ends on, and with the cuts across each section's two ends they are
//! the constraints: no triangle crosses a road's outline. The **toes** are those
//! edges pushed `verge_width` out on the side that has land, at the ground's height
//! there, so the grass slopes down from the road the way a verge does. And the
//! **grid** is the ground's own height field past the verges, out to
//! `ground_extent`. What is inside a road's outline is cut away afterwards, so the
//! junction's roads and the pavements round it sit in a hole cut to their shape, and
//! everything else is one surface.
//!
//! A toe or a grid vertex goes in only where it really is a verge's width from
//! every road's edge — for a toe, every edge but the two chords of its own rail
//! it was pushed out from. That one test is what keeps the land off the roads and
//! free of ledges: a toe on the inside of a bend tighter than the verge is wide
//! has folded back over its own rail and is nearer than a verge to the rest of
//! it; a toe pushed out from a road in a junction lands beside the next road, or
//! the pavement round the corner, and is nearer than a verge to that; a grid
//! vertex just past a kerb would make the slope down from it a step.

use std::collections::HashMap;

use roadgen_core::geometry::Point3;
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
    /// The verge's outer edge: `verge_width` out from the rail on the side that
    /// has land, one point per rail vertex, and empty on a side that faces a
    /// junction. The heights are the land's to set, and so is whether each point
    /// is used.
    pub toe: Vec<Point3>,
}

/// One road section as the land sees it: its outline and the toes beside it.
struct Section {
    /// The outline, closed: the left rail, then the right rail back. Every edge
    /// of it is a constraint the land's triangles do not cross.
    ring: Vec<Point3>,
    /// The toes, each with the index of the ring vertex it was pushed out from.
    toes: Vec<(usize, Point3)>,
    /// The outline in plan, which the land is cut out of.
    footprint: Polygon,
}

/// What the land is built round: every road section's outline.
#[derive(Default)]
pub struct Land {
    sections: Vec<Section>,
}

impl Land {
    /// Adds one road section from its left and right flanks.
    pub fn section(&mut self, left: Flank, right: Flank) {
        let ring: Vec<Point3> = left
            .rail
            .iter()
            .chain(right.rail.iter().rev())
            .copied()
            .collect();
        // The right rail runs backwards round the ring, so its vertex `k` is the
        // ring's last but `k`.
        let last = ring.len() - 1;
        let toes = left
            .toe
            .into_iter()
            .enumerate()
            .chain(
                right
                    .toe
                    .into_iter()
                    .enumerate()
                    .map(|(index, toe)| (last - index, toe)),
            )
            .collect();
        let footprint = Polygon::new(ring.iter().map(|point| [point.x, point.y]).collect());
        self.sections.push(Section {
            ring,
            toes,
            footprint,
        });
    }

    /// The land as one mesh, tagged as terrain, with the ground's height `verge`
    /// metres out from every road. `None` when there is no road to build it round.
    pub fn mesh(
        &self,
        field: Option<&Field>,
        verge: f64,
        map_name: &str,
        ordinals: &mut Ordinals,
    ) -> Option<Mesh> {
        if self.sections.is_empty() {
            return None;
        }
        let footprints: Vec<&Polygon> = self.sections.iter().map(|s| &s.footprint).collect();
        let roads = Index::new(&footprints);
        let edges = Edges::new(&self.sections, verge.max(0.0));

        // The free points first, in bulk, and the outlines inserted after them,
        // so that an outline vertex that lands on the same spot as a grid or toe
        // point keeps its own height: the road's edge is the one height here that
        // is not negotiable.
        let mut points: Vec<Vertex> = Vec::new();
        if let Some(field) = field {
            points.extend(
                field
                    .vertices()
                    .filter(|point| !roads.contains(point.x, point.y) && !edges.within(point, None))
                    .map(Vertex::free),
            );
        }
        for (index, section) in self.sections.iter().enumerate() {
            for &(vertex, toe) in &section.toes {
                if roads.contains(toe.x, toe.y) || edges.within(&toe, Some((index, vertex))) {
                    continue;
                }
                let z = field.map_or(toe.z, |field| field.height(toe.x, toe.y));
                points.push(Vertex::free(Point3::new(toe.x, toe.y, z)));
            }
        }
        let mut cdt: ConstrainedDelaunayTriangulation<Vertex> =
            ConstrainedDelaunayTriangulation::bulk_load(points).ok()?;
        for section in &self.sections {
            let mut handles = Vec::with_capacity(section.ring.len());
            for &point in &section.ring {
                if let Ok(handle) = cdt.insert(Vertex::on_road(point)) {
                    handles.push((handle, point));
                }
            }
            for pair in handles.windows(2).chain(
                handles
                    .last()
                    .zip(handles.first())
                    .map(|(a, b)| [*a, *b])
                    .as_ref()
                    .map(|end| end.as_slice()),
            ) {
                let ((from, start), (to, end)) = (pair[0], pair[1]);
                if from == to {
                    continue;
                }
                // Outlines of different roads cross inside a junction; where they
                // do, both are split at the crossing, which takes the height of
                // the outline being added at that point.
                cdt.add_constraint_and_split(from, to, |at| {
                    let t = fraction_along(start, end, [at.x, at.y]);
                    Vertex::on_road(Point3::new(at.x, at.y, start.lerp(end, t).z))
                });
            }
        }

        // Every triangle outside the roads, with the vertices it needs and no other.
        // A triangle with a free corner is outside every road already: a free
        // point is, and no triangle crosses an outline.
        let mut positions = Vec::with_capacity(cdt.num_vertices());
        let mut renumbered = vec![u32::MAX; cdt.num_vertices()];
        let mut triangles = Vec::with_capacity(cdt.num_inner_faces());
        for face in cdt.inner_faces() {
            let corners = face.vertices();
            if corners.iter().all(|corner| corner.data().on_road) {
                let centre = corners.iter().fold([0.0, 0.0], |sum, corner| {
                    let at = corner.position();
                    [sum[0] + at.x / 3.0, sum[1] + at.y / 3.0]
                });
                if roads.contains(centre[0], centre[1]) {
                    continue;
                }
            }
            // Spade winds an inner face anticlockwise in the plane, which is
            // anticlockwise seen from above: the face is up.
            let triangle = corners.map(|corner| {
                let slot = &mut renumbered[corner.fix().index()];
                if *slot == u32::MAX {
                    *slot = positions.len() as u32;
                    positions.push(corner.data().at);
                }
                *slot
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

/// How far along the segment `a..b` the point `p` falls, in plan, clamped to the
/// segment: `0` at `a`, `1` at `b`.
fn fraction_along(a: Point3, b: Point3, p: [f64; 2]) -> f64 {
    let (dx, dy) = (b.x - a.x, b.y - a.y);
    let length = dx * dx + dy * dy;
    if length < 1e-18 {
        return 0.0;
    }
    (((p[0] - a.x) * dx + (p[1] - a.y) * dy) / length).clamp(0.0, 1.0)
}

/// A point of the land, as spade sees it: its plan position is the vertex.
struct Vertex {
    at: Point3,
    /// On a road's outline, as against free on the land.
    on_road: bool,
}

impl Vertex {
    fn free(at: Point3) -> Vertex {
        Vertex { at, on_road: false }
    }

    fn on_road(at: Point3) -> Vertex {
        Vertex { at, on_road: true }
    }
}

impl HasPosition for Vertex {
    type Scalar = f64;

    fn position(&self) -> Point2<f64> {
        Point2::new(self.at.x, self.at.y)
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

/// Things bucketed by the cells their boxes cover, so that a point is tested
/// against the few near it rather than all of them.
struct Buckets<T> {
    cell: f64,
    buckets: HashMap<(i64, i64), Vec<T>>,
}

impl<T> Buckets<T> {
    fn new(cell: f64) -> Buckets<T> {
        Buckets {
            cell,
            buckets: HashMap::new(),
        }
    }

    /// Files `item` under every cell its box touches.
    fn insert(&mut self, min: [f64; 2], max: [f64; 2], item: T)
    where
        T: Clone,
    {
        if !min[0].is_finite() || !max[0].is_finite() {
            return;
        }
        let (x0, x1) = (self.index(min[0]), self.index(max[0]));
        let (y0, y1) = (self.index(min[1]), self.index(max[1]));
        for i in x0..=x1 {
            for j in y0..=y1 {
                self.buckets.entry((i, j)).or_default().push(item.clone());
            }
        }
    }

    fn index(&self, value: f64) -> i64 {
        (value / self.cell).floor() as i64
    }

    /// The items filed under the point's own cell.
    fn at(&self, x: f64, y: f64) -> impl Iterator<Item = &T> {
        self.buckets
            .get(&(self.index(x), self.index(y)))
            .into_iter()
            .flatten()
    }

    /// The items filed under the point's cell and the eight round it.
    fn around(&self, x: f64, y: f64) -> impl Iterator<Item = &T> {
        let (i, j) = (self.index(x), self.index(y));
        (i - 1..=i + 1).flat_map(move |i| {
            (j - 1..=j + 1).flat_map(move |j| self.buckets.get(&(i, j)).into_iter().flatten())
        })
    }
}

/// One edge of a section's outline, in plan, with the section and the two ring
/// vertices it joins.
type Edge = ([f64; 2], [f64; 2], (usize, usize, usize));

/// Every outline's edges, bucketed so that a point can ask whether a road's edge
/// is within `reach` of it without measuring itself against every edge on the map.
struct Edges {
    reach: f64,
    buckets: Buckets<Edge>,
}

impl Edges {
    fn new(sections: &[Section], reach: f64) -> Edges {
        // Cells no smaller than the reach, so that a query never has to look past
        // the cells round its own. No reach, no index: nothing is ever within it.
        let mut buckets = Buckets::new(reach.max(1.0));
        if reach > 0.0 {
            for (index, section) in sections.iter().enumerate() {
                let count = section.ring.len();
                for (vertex, &from) in section.ring.iter().enumerate() {
                    let next = (vertex + 1) % count;
                    let to = section.ring[next];
                    let (a, b) = ([from.x, from.y], [to.x, to.y]);
                    buckets.insert(
                        [a[0].min(b[0]), a[1].min(b[1])],
                        [a[0].max(b[0]), a[1].max(b[1])],
                        (a, b, (index, vertex, next)),
                    );
                }
            }
        }
        Edges { reach, buckets }
    }

    /// Whether any outline passes within the reach of the point — leaving out,
    /// for a toe, the two edges that meet at the ring vertex `own` it was pushed
    /// out from: a toe stands exactly a verge from that vertex, and a hair less
    /// from the chords either side of it on a bend, and neither is a fold.
    fn within(&self, point: &Point3, own: Option<(usize, usize)>) -> bool {
        if self.reach <= 0.0 {
            return false;
        }
        let (x, y) = (point.x, point.y);
        // A whisker under the reach, so that a toe's own vertex, on an edge that
        // is not skipped, does not count as within it.
        let square = self.reach * self.reach * (1.0 - 1e-9);
        self.buckets
            .around(x, y)
            .any(|&(a, b, (section, from, to))| {
                if own.is_some_and(|(own_section, own_vertex)| {
                    section == own_section && (from == own_vertex || to == own_vertex)
                }) {
                    return false;
                }
                let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
                let t = fraction_along(
                    Point3::new(a[0], a[1], 0.0),
                    Point3::new(b[0], b[1], 0.0),
                    [x, y],
                );
                let (px, py) = (a[0] + dx * t - x, a[1] + dy * t - y);
                px * px + py * py < square
            })
    }
}

/// Polygons bucketed by the cells their boxes cover.
struct Index<'a> {
    polygons: &'a [&'a Polygon],
    buckets: Buckets<usize>,
}

impl<'a> Index<'a> {
    const CELL: f64 = 25.0;

    fn new(polygons: &'a [&'a Polygon]) -> Index<'a> {
        let mut buckets = Buckets::new(Self::CELL);
        for (index, polygon) in polygons.iter().enumerate() {
            buckets.insert(polygon.min, polygon.max, index);
        }
        Index { polygons, buckets }
    }

    fn contains(&self, x: f64, y: f64) -> bool {
        self.buckets
            .at(x, y)
            .any(|&index| self.polygons[index].contains(x, y))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn flank(rail: Vec<Point3>, toe: Vec<Point3>) -> Flank {
        Flank { rail, toe }
    }

    fn straight_road() -> Land {
        // A road 40 m long and 8 m wide along x, with a verge each side.
        let rail = |y: f64| -> Vec<Point3> {
            (0..=4)
                .map(|i| Point3::new(i as f64 * 10.0, y, 0.0))
                .collect()
        };
        let mut land = Land::default();
        land.section(flank(rail(4.0), rail(12.0)), flank(rail(-4.0), rail(-12.0)));
        land
    }

    #[test]
    fn the_land_meets_the_road_at_every_edge_vertex_and_has_nothing_under_it() {
        let land = straight_road();
        let mesh = land
            .mesh(None, 8.0, "t", &mut Ordinals::default())
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
        let squares = [&square];
        let index = Index::new(&squares);
        assert!(index.contains(9.0, 9.0));
        assert!(!index.contains(-9.0, 9.0));
    }

    #[test]
    fn the_edges_know_what_is_near_them_and_whose_toe_is_whose() {
        // A road 100 m long and 8 m wide, with a toe 8 m out from each vertex of
        // its left rail.
        let mut land = Land::default();
        let rail = |y: f64| -> Vec<Point3> {
            (0..=10)
                .map(|i| Point3::new(i as f64 * 10.0, y, 0.0))
                .collect()
        };
        // The toes stand a hair under the verge from the rail, as they do from the
        // chords of a bend.
        land.section(flank(rail(4.0), rail(11.9)), flank(rail(-4.0), Vec::new()));
        let edges = Edges::new(&land.sections, 8.0);
        assert!(edges.within(&Point3::new(50.0, 11.9, 0.0), None));
        assert!(!edges.within(&Point3::new(50.0, 12.1, 0.0), None));
        assert!(
            edges.within(&Point3::new(-3.0, 4.0, 0.0), None),
            "the end of an outline is part of it"
        );
        assert!(!edges.within(&Point3::new(50.0, 30.0, 0.0), None));
        // A toe's own rail does not count against it — not at the end of the rail
        // either, where its vertex is also on the cut across the road's end — but
        // any other edge does.
        for vertex in [0, 5, 10] {
            let toe = land.sections[0].toes[vertex].1;
            assert!(
                edges.within(&toe, None),
                "toe {vertex} is not near its rail"
            );
            assert!(
                !edges.within(&toe, Some((0, vertex))),
                "toe {vertex} is within its own rail"
            );
            assert!(
                edges.within(&toe, Some((0, (vertex + 3) % 11))),
                "toe {vertex} is not within another vertex's rail"
            );
        }
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
        let mut land = Land::default();
        land.section(
            flank(along(4.0), Vec::new()),
            flank(along(-4.0), Vec::new()),
        );
        land.section(
            flank(across(-4.0), Vec::new()),
            flank(across(4.0), Vec::new()),
        );
        let mesh = land
            .mesh(None, 8.0, "t", &mut Ordinals::default())
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
