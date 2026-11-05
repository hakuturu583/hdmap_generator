//! The shape of the land: the terrain elevation a road network runs across.
//!
//! A road says what height the road is at and nothing about the ground beside it.
//! Where the ground matters — a quarry's benches, the spoil heaped beside it, the
//! hillside a road is cut into — it has to be stated, and this is where it is.
//!
//! # The shape of the model
//!
//! The model is GDF's Terrain Elevation theme, taken the way the rest of this IR
//! takes GDF: the ideas, not the catalogue.
//!
//! A [`TinSurface`] is a feature: a triangulated irregular network over the land
//! with an identifier of its own. Its [`Tin`] — the nodes and the triangles over
//! them — is that feature's geometry and not a set of features, the way a
//! [`Polyline3`]'s vertices are not: a triangle has nothing to be called that a
//! regenerated map would call it again, so an identifier on each would make two
//! runs of the same input incomparable rather than comparable.
//!
//! A [`TinBreakline`] *is* a feature. It is a line along which the land changes
//! slope abruptly — the crest of a bench, the toe of a heap, the edge of a road
//! cut into a hillside — and it means something: a generator names it after what
//! it is the edge of, and a consumer that draws the land, or drives on it, wants to
//! know where the edges are rather than to rediscover them from the triangles.
//!
//! # A breakline is an edge of its surface
//!
//! The invariant that ties the two together: every segment of a breakline is an
//! edge of its surface's triangulation, and every vertex of one is a node of it.
//! That is what a breakline is for — the triangles do not cross it, so the slope
//! really does change there — and it is what lets a consumer re-triangulate the
//! land round something of its own (a road surface, say) and keep the breaklines
//! as constraints without them disagreeing with the heights it read off the TIN.
//! [`validate`](crate::validation) checks it.
//!
//! # Not the road
//!
//! The terrain is the ground, and a road is not part of it: the road keeps its own
//! three-dimensional geometry, and where the road is, the TIN is the ground under
//! it. A generator that wants the two to agree — which it should — puts the road's
//! edges into the TIN as breaklines.

use std::collections::HashSet;

use crate::geometry::{Point3, Polyline3};
use crate::id::{SiteId, TinBreaklineId, TinSurfaceId};

/// The nodes and triangles of a triangulated irregular network.
///
/// A 2.5-dimensional surface: one height over each point of its plan, the
/// triangles anticlockwise seen from above and none of them folded over another.
/// [`Tin::new`] establishes the winding and the indices; whether the triangles
/// overlap is validation's to report.
#[derive(Debug, Clone, PartialEq)]
pub struct Tin {
    nodes: Vec<Point3>,
    triangles: Vec<[u32; 3]>,
}

/// Why a set of nodes and triangles is not a TIN.
#[derive(Debug, Clone, PartialEq)]
pub enum TinError {
    NonFiniteNode {
        node: usize,
    },
    IndexOutOfRange {
        triangle: usize,
        index: u32,
    },
    /// A triangle whose plan has no area: a sliver, or two corners on one node.
    Degenerate {
        triangle: usize,
    },
}

impl std::fmt::Display for TinError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            TinError::NonFiniteNode { node } => write!(f, "node {node} is not finite"),
            TinError::IndexOutOfRange { triangle, index } => {
                write!(
                    f,
                    "triangle {triangle} names node {index}, which is not there"
                )
            }
            TinError::Degenerate { triangle } => {
                write!(f, "triangle {triangle} has no area in plan")
            }
        }
    }
}

impl std::error::Error for TinError {}

/// Twice the signed area of a triangle's plan: positive when anticlockwise.
fn doubled_area(a: Point3, b: Point3, c: Point3) -> f64 {
    (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x)
}

impl Tin {
    /// Plan area below which a triangle is taken for a sliver, square metres.
    pub const MIN_AREA: f64 = 1e-9;

    /// A TIN from its nodes and the triangles over them. Every triangle is turned
    /// anticlockwise seen from above, whichever way round it was given.
    pub fn new(nodes: Vec<Point3>, triangles: Vec<[u32; 3]>) -> Result<Tin, TinError> {
        if let Some(node) = nodes.iter().position(|point| !point.is_finite()) {
            return Err(TinError::NonFiniteNode { node });
        }
        let mut oriented = Vec::with_capacity(triangles.len());
        for (index, triangle) in triangles.into_iter().enumerate() {
            for &corner in &triangle {
                if corner as usize >= nodes.len() {
                    return Err(TinError::IndexOutOfRange {
                        triangle: index,
                        index: corner,
                    });
                }
            }
            let [a, b, c] = triangle.map(|corner| nodes[corner as usize]);
            let area = doubled_area(a, b, c);
            if area.abs() / 2.0 < Self::MIN_AREA {
                return Err(TinError::Degenerate { triangle: index });
            }
            oriented.push(if area > 0.0 {
                triangle
            } else {
                [triangle[0], triangle[2], triangle[1]]
            });
        }
        Ok(Tin {
            nodes,
            triangles: oriented,
        })
    }

    pub fn nodes(&self) -> &[Point3] {
        &self.nodes
    }

    pub fn triangles(&self) -> &[[u32; 3]] {
        &self.triangles
    }

    pub fn is_empty(&self) -> bool {
        self.triangles.is_empty()
    }

    /// The corners of triangle `index`.
    pub fn corners(&self, index: usize) -> [Point3; 3] {
        self.triangles[index].map(|corner| self.nodes[corner as usize])
    }

    /// Every edge, once, as its two node indices in ascending order.
    pub fn edges(&self) -> HashSet<(u32, u32)> {
        self.triangles
            .iter()
            .flat_map(|&[a, b, c]| [(a, b), (b, c), (c, a)])
            .map(|(a, b)| (a.min(b), a.max(b)))
            .collect()
    }

    /// The smallest box round the nodes in plan: `[min_x, min_y, max_x, max_y]`.
    pub fn bounds(&self) -> Option<[f64; 4]> {
        let first = self.nodes.first()?;
        Some(self.nodes.iter().fold(
            [first.x, first.y, first.x, first.y],
            |[x0, y0, x1, y1], point| {
                [
                    x0.min(point.x),
                    y0.min(point.y),
                    x1.max(point.x),
                    y1.max(point.y),
                ]
            },
        ))
    }

    /// An index for asking the surface its height, which a TIN of any size needs:
    /// finding the triangle over a point by trying them all is a scan per query.
    pub fn locator(&self) -> TinLocator<'_> {
        TinLocator::new(self)
    }
}

/// Answers "how high is the land here" of a [`Tin`], from a bucket grid over its
/// triangles.
pub struct TinLocator<'a> {
    tin: &'a Tin,
    origin: [f64; 2],
    cell: f64,
    columns: usize,
    rows: usize,
    buckets: Vec<Vec<u32>>,
}

impl<'a> TinLocator<'a> {
    fn new(tin: &'a Tin) -> Self {
        let bounds = tin.bounds().unwrap_or([0.0; 4]);
        let width = (bounds[2] - bounds[0]).max(1e-6);
        let height = (bounds[3] - bounds[1]).max(1e-6);
        // About one triangle per cell on average, which keeps both the grid and
        // each bucket small.
        let count = tin.triangles.len().max(1) as f64;
        let cell = ((width * height) / count).sqrt().max(1e-3);
        let columns = ((width / cell).ceil() as usize).clamp(1, 4096);
        let rows = ((height / cell).ceil() as usize).clamp(1, 4096);
        let cell = (width / columns as f64).max(height / rows as f64);
        let mut buckets = vec![Vec::new(); columns * rows];
        let clamp = |value: f64, count: usize| (value.floor().max(0.0) as usize).min(count - 1);
        for (index, triangle) in tin.triangles.iter().enumerate() {
            let corners = triangle.map(|corner| tin.nodes[corner as usize]);
            let (mut x0, mut y0, mut x1, mut y1) = (f64::MAX, f64::MAX, f64::MIN, f64::MIN);
            for corner in corners {
                x0 = x0.min(corner.x);
                y0 = y0.min(corner.y);
                x1 = x1.max(corner.x);
                y1 = y1.max(corner.y);
            }
            let (i0, i1) = (
                clamp((x0 - bounds[0]) / cell, columns),
                clamp((x1 - bounds[0]) / cell, columns),
            );
            let (j0, j1) = (
                clamp((y0 - bounds[1]) / cell, rows),
                clamp((y1 - bounds[1]) / cell, rows),
            );
            for j in j0..=j1 {
                for i in i0..=i1 {
                    buckets[j * columns + i].push(index as u32);
                }
            }
        }
        TinLocator {
            tin,
            origin: [bounds[0], bounds[1]],
            cell,
            columns,
            rows,
            buckets,
        }
    }

    /// The triangle whose plan holds `(x, y)`, and the point's barycentric weights
    /// in it. `None` off the surface.
    pub fn locate(&self, x: f64, y: f64) -> Option<(usize, [f64; 3])> {
        let i = ((x - self.origin[0]) / self.cell).floor();
        let j = ((y - self.origin[1]) / self.cell).floor();
        // A point a rounding error outside the box still finds the edge triangle.
        let tolerance = 1e-9 * self.cell.max(1.0);
        if i < -1.0 || j < -1.0 || i > self.columns as f64 || j > self.rows as f64 {
            return None;
        }
        let i = (i.max(0.0) as usize).min(self.columns - 1);
        let j = (j.max(0.0) as usize).min(self.rows - 1);
        let probe = Point3::new(x, y, 0.0);
        for &triangle in &self.buckets[j * self.columns + i] {
            let [a, b, c] = self.tin.corners(triangle as usize);
            let area = doubled_area(a, b, c);
            let wa = doubled_area(probe, b, c) / area;
            let wb = doubled_area(a, probe, c) / area;
            let wc = 1.0 - wa - wb;
            if wa >= -tolerance && wb >= -tolerance && wc >= -tolerance {
                return Some((triangle as usize, [wa, wb, wc]));
            }
        }
        None
    }

    /// The land's height at `(x, y)`, read off the triangle over it. `None` off
    /// the surface.
    pub fn height_at(&self, x: f64, y: f64) -> Option<f64> {
        let (triangle, [wa, wb, wc]) = self.locate(x, y)?;
        let [a, b, c] = self.tin.corners(triangle);
        Some(wa * a.z + wb * b.z + wc * c.z)
    }
}

/// A TIN surface: the land over some extent, as one feature.
#[derive(Debug, Clone, PartialEq)]
pub struct TinSurface {
    pub id: TinSurfaceId,
    /// The site the surface is part of, when it is part of one.
    pub site: Option<SiteId>,
    /// The breaklines of this surface, in the order they were laid.
    pub breaklines: Vec<TinBreaklineId>,
    pub tin: Tin,
}

/// A line along which the land changes slope abruptly: an edge of its surface's
/// triangulation that means something.
#[derive(Debug, Clone, PartialEq)]
pub struct TinBreakline {
    pub id: TinBreaklineId,
    pub surface: TinSurfaceId,
    /// What the line is the edge of, in the generator's words: `crest`, `toe`,
    /// `ridge`, `valley`, `road_edge`.
    pub kind: String,
    /// Closed when its last vertex is its first.
    pub line: Polyline3,
}

impl TinBreakline {
    pub fn is_closed(&self) -> bool {
        self.line.first().distance_to(self.line.last()) < Polyline3::MIN_SEGMENT
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn square() -> Tin {
        Tin::new(
            vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(10.0, 0.0, 10.0),
                Point3::new(10.0, 10.0, 20.0),
                Point3::new(0.0, 10.0, 10.0),
            ],
            // The second one clockwise, to be turned round.
            vec![[0, 1, 2], [0, 3, 2]],
        )
        .unwrap()
    }

    #[test]
    fn every_triangle_is_turned_anticlockwise() {
        let tin = square();
        for index in 0..tin.triangles().len() {
            let [a, b, c] = tin.corners(index);
            assert!(doubled_area(a, b, c) > 0.0);
        }
        assert_eq!(tin.edges().len(), 5);
    }

    #[test]
    fn the_height_is_read_off_the_triangle_over_the_point() {
        let tin = square();
        let locator = tin.locator();
        // The plane z = x + y, sampled anywhere on the square.
        for (x, y) in [(0.0, 0.0), (2.5, 7.5), (10.0, 10.0), (9.0, 1.0), (5.0, 5.0)] {
            let height = locator.height_at(x, y).unwrap();
            assert!((height - (x + y)).abs() < 1e-9, "{x},{y} -> {height}");
        }
        assert_eq!(locator.height_at(11.0, 5.0), None);
    }

    #[test]
    fn a_sliver_or_a_missing_node_is_not_a_tin() {
        let nodes = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(2.0, 0.0, 0.0),
        ];
        assert_eq!(
            Tin::new(nodes.clone(), vec![[0, 1, 2]]),
            Err(TinError::Degenerate { triangle: 0 })
        );
        assert_eq!(
            Tin::new(nodes, vec![[0, 1, 3]]),
            Err(TinError::IndexOutOfRange {
                triangle: 0,
                index: 3
            })
        );
    }
}
