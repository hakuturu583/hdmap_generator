//! The little bit of planar geometry that deciding where a building fits needs.
//!
//! Placing buildings is the one job in this project that is genuinely two-dimensional:
//! "does this outline hit that road?" is a question about ground plan, and answering
//! it in 3D would mean answering a different question. So the projection happens
//! once, here, at the door — everything below is `[f64; 2]` and says so — and the
//! height is put back on in [`crate::masses`].
//!
//! Nothing here is a general polygon library. There is no union, no difference, no
//! arrangement: the only question asked is whether two convex shapes overlap, and the
//! only shapes asked about are triangles and the ground plan of a box.

use roadgen_core::geometry::Grid;

/// A point in the horizontal plane, metres.
pub type Point2 = [f64; 2];

/// An axis-aligned box, used to skip the work of an exact test.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Bounds {
    pub min: Point2,
    pub max: Point2,
}

impl Bounds {
    pub fn of(points: &[Point2]) -> Bounds {
        let mut bounds = Bounds {
            min: [f64::INFINITY; 2],
            max: [f64::NEG_INFINITY; 2],
        };
        for point in points {
            for (axis, value) in point.iter().enumerate() {
                bounds.min[axis] = bounds.min[axis].min(*value);
                bounds.max[axis] = bounds.max[axis].max(*value);
            }
        }
        bounds
    }

    /// Whether two boxes share any area, touching included.
    pub fn overlaps(&self, other: &Bounds) -> bool {
        self.min[0] <= other.max[0]
            && other.min[0] <= self.max[0]
            && self.min[1] <= other.max[1]
            && other.min[1] <= self.max[1]
    }
}

/// Whether two convex polygons share any area, by the separating-axis test.
///
/// Both are taken as convex and given in either winding. Touching counts as clear:
/// a lot that ends exactly where the setback ends is a lot that fits, and a `>` here
/// rather than a `>=` is what stops a row of buildings rejecting itself for sharing
/// an edge.
pub fn convex_overlap(a: &[Point2], b: &[Point2]) -> bool {
    if a.len() < 3 || b.len() < 3 {
        return false;
    }
    !(separated_along_edges_of(a, b) || separated_along_edges_of(b, a))
}

/// Whether some edge normal of `polygon` separates the two polygons.
fn separated_along_edges_of(polygon: &[Point2], other: &[Point2]) -> bool {
    for index in 0..polygon.len() {
        let from = polygon[index];
        let to = polygon[(index + 1) % polygon.len()];
        // The outward normal of this edge, unnormalised: scaling an axis moves both
        // projections by the same factor, so its length never changes the answer.
        let axis = [-(to[1] - from[1]), to[0] - from[0]];
        if axis[0] == 0.0 && axis[1] == 0.0 {
            continue;
        }
        let (min_a, max_a) = project(polygon, axis);
        let (min_b, max_b) = project(other, axis);
        // A hair of slack, so that two shapes laid edge to edge by the same
        // arithmetic do not overlap by a rounding error.
        const TOUCHING: f64 = 1e-9;
        if max_a <= min_b + TOUCHING || max_b <= min_a + TOUCHING {
            return true;
        }
    }
    false
}

fn project(polygon: &[Point2], axis: Point2) -> (f64, f64) {
    let mut min = f64::INFINITY;
    let mut max = f64::NEG_INFINITY;
    for point in polygon {
        let value = point[0] * axis[0] + point[1] * axis[1];
        min = min.min(value);
        max = max.max(value);
    }
    (min, max)
}

/// The convex hull of a set of points, anticlockwise, by Andrew's monotone chain.
///
/// Used to get the ground plan of a box: four of a box's eight corners are directly
/// above the other four, so an upright box hulls down to its base exactly, and a box
/// the grammar tilted hulls down to the outline it really casts.
pub fn convex_hull(points: &[Point2]) -> Vec<Point2> {
    if points.len() < 3 {
        return points.to_vec();
    }
    let mut sorted = points.to_vec();
    sorted.sort_by(|a, b| {
        a[0].partial_cmp(&b[0])
            .unwrap_or(std::cmp::Ordering::Equal)
            .then(a[1].partial_cmp(&b[1]).unwrap_or(std::cmp::Ordering::Equal))
    });
    sorted.dedup_by(|a, b| (a[0] - b[0]).abs() < 1e-12 && (a[1] - b[1]).abs() < 1e-12);
    if sorted.len() < 3 {
        return sorted;
    }

    let mut hull: Vec<Point2> = Vec::with_capacity(sorted.len() * 2);
    for pass in 0..2 {
        // What the pass before it left: the lower chain is never popped into while
        // the upper one is built, which is the whole of the algorithm's bookkeeping.
        let base = hull.len();
        for step in 0..sorted.len() {
            // Left to right for the lower chain, right to left for the upper one.
            let point = if pass == 0 {
                sorted[step]
            } else {
                sorted[sorted.len() - 1 - step]
            };
            while hull.len() >= base + 2
                && cross(hull[hull.len() - 2], hull[hull.len() - 1], point) <= 0.0
            {
                hull.pop();
            }
            hull.push(point);
        }
        // The last point of a pass is the first of the next one.
        hull.pop();
    }
    hull
}

/// Twice the signed area of the triangle `a b c`: positive when it turns left.
fn cross(a: Point2, b: Point2, c: Point2) -> f64 {
    (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
}

/// Convex shapes on a grid, so that "does this outline hit anything?" costs the
/// shapes nearby rather than all of them: a generated town asks it once per
/// candidate building against every stretch of every road.
#[derive(Debug, Clone)]
pub struct Index {
    /// Each shape with the box around it, which is what a query is filtered on.
    shapes: Vec<(Bounds, Vec<Point2>)>,
    grid: Grid<usize>,
}

impl Index {
    /// An empty index whose cells are `cell` metres across.
    pub fn new(cell: f64) -> Index {
        Index {
            shapes: Vec::new(),
            grid: Grid::new(cell),
        }
    }

    pub fn is_empty(&self) -> bool {
        self.shapes.is_empty()
    }

    /// Adds a convex shape.
    pub fn insert(&mut self, shape: Vec<Point2>) {
        if shape.len() < 3 {
            return;
        }
        let bounds = Bounds::of(&shape);
        self.grid.insert(bounds.min, bounds.max, self.shapes.len());
        self.shapes.push((bounds, shape));
    }

    /// Whether `shape` overlaps anything in the index.
    pub fn hits(&self, shape: &[Point2]) -> bool {
        let bounds = Bounds::of(shape);
        self.grid
            .covering(bounds.min, bounds.max)
            .any(|&candidate| {
                let (around, other) = &self.shapes[candidate];
                // Two shapes sharing a cell usually miss, and the boxes say so for
                // the price of four comparisons. It is also why a shape met twice —
                // once per cell it spans — needs no bookkeeping to skip.
                bounds.overlaps(around) && convex_overlap(shape, other)
            })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn square(x: f64, y: f64, size: f64) -> Vec<Point2> {
        vec![[x, y], [x + size, y], [x + size, y + size], [x, y + size]]
    }

    #[test]
    fn overlapping_squares_are_found_and_separated_ones_are_not() {
        assert!(convex_overlap(
            &square(0.0, 0.0, 10.0),
            &square(5.0, 5.0, 10.0)
        ));
        assert!(!convex_overlap(
            &square(0.0, 0.0, 10.0),
            &square(20.0, 0.0, 10.0)
        ));
    }

    #[test]
    fn squares_laid_edge_to_edge_do_not_overlap() {
        // Two lots in a row share a boundary; if that counted, every row of
        // buildings would reject its own second building.
        assert!(!convex_overlap(
            &square(0.0, 0.0, 10.0),
            &square(10.0, 0.0, 10.0)
        ));
    }

    #[test]
    fn a_diagonal_shape_that_misses_a_box_is_not_caught_by_its_bounds() {
        // The triangle's bounding box covers the square; the triangle does not.
        let triangle = vec![[0.0, 12.0], [12.0, 0.0], [12.0, 12.0]];
        assert!(!convex_overlap(&square(0.0, 0.0, 5.0), &triangle));
    }

    #[test]
    fn the_hull_of_a_boxs_eight_corners_is_its_base() {
        let mut corners = square(0.0, 0.0, 10.0);
        // The top four sit directly above the bottom four.
        corners.extend(square(0.0, 0.0, 10.0));
        let hull = convex_hull(&corners);
        assert_eq!(hull.len(), 4);
        assert!(hull.contains(&[0.0, 0.0]));
        assert!(hull.contains(&[10.0, 10.0]));
    }

    #[test]
    fn the_index_answers_what_the_pairwise_test_answers() {
        let mut index = Index::new(20.0);
        index.insert(square(0.0, 0.0, 10.0));
        index.insert(square(100.0, 100.0, 10.0));
        assert!(index.hits(&square(5.0, 5.0, 10.0)));
        assert!(index.hits(&square(95.0, 95.0, 10.0)));
        assert!(!index.hits(&square(50.0, 50.0, 10.0)));
    }
}
