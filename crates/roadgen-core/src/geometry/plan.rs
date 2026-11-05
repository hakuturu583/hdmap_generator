//! Questions about the plan view: is a point inside an outline, how far is it from a
//! line.
//!
//! Every generator asks them — where may a building stand, which ground points are
//! clear of the roads, which triangles of the land are rock — and so does every
//! exporter that draws the land. They are asked here once, in plan, with the
//! projection in the name: a point is `[x, y]` metres, and nothing in this module
//! has a `z` to drop.

use super::Grid;

/// A point in plan, metres.
pub type PlanPoint = [f64; 2];

/// Whether `p` is inside the closed `ring` — first vertex not repeated — by the
/// crossings of a ray cast from it. A point on the outline may come out either way.
pub fn ring_contains(ring: &[PlanPoint], p: PlanPoint) -> bool {
    let mut inside = false;
    let count = ring.len();
    for i in 0..count {
        let (a, b) = (ring[i], ring[(i + 1) % count]);
        if (a[1] > p[1]) != (b[1] > p[1]) {
            let cross = (b[0] - a[0]) * (p[1] - a[1]) / (b[1] - a[1]) + a[0];
            if p[0] < cross {
                inside = !inside;
            }
        }
    }
    inside
}

/// Whether two closed rings share any ground: one holds a vertex of the other, or
/// their edges cross. Two long, narrow rectangles laid across each other do, with
/// every vertex outside the other; so does a small ring wholly inside a large one.
pub fn rings_overlap(a: &[PlanPoint], b: &[PlanPoint]) -> bool {
    let edges = |ring: &[PlanPoint]| {
        (0..ring.len())
            .map(|i| (ring[i], ring[(i + 1) % ring.len()]))
            .collect::<Vec<_>>()
    };
    if a.iter().any(|&p| ring_contains(b, p)) || b.iter().any(|&p| ring_contains(a, p)) {
        return true;
    }
    let (ea, eb) = (edges(a), edges(b));
    ea.iter()
        .any(|&(p, q)| eb.iter().any(|&(r, s)| segments_cross(p, q, r, s)))
}

/// Whether the segments `p..q` and `r..s` cross, each passing strictly between the
/// other's ends.
pub fn segments_cross(p: PlanPoint, q: PlanPoint, r: PlanPoint, s: PlanPoint) -> bool {
    let side = |a: PlanPoint, b: PlanPoint, c: PlanPoint| {
        (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
    };
    side(p, q, r) * side(p, q, s) < 0.0 && side(r, s, p) * side(r, s, q) < 0.0
}

/// The distance from `p` to the segment `a..b`.
pub fn segment_distance(p: PlanPoint, a: PlanPoint, b: PlanPoint) -> f64 {
    let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
    let length = dx * dx + dy * dy;
    let t = if length < 1e-18 {
        0.0
    } else {
        (((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / length).clamp(0.0, 1.0)
    };
    (p[0] - a[0] - t * dx).hypot(p[1] - a[1] - t * dy)
}

/// The distance from `p` to the nearest segment of an open polyline.
pub fn polyline_distance(p: PlanPoint, line: &[PlanPoint]) -> f64 {
    match line {
        [] => f64::INFINITY,
        [only] => (p[0] - only[0]).hypot(p[1] - only[1]),
        _ => line
            .windows(2)
            .map(|pair| segment_distance(p, pair[0], pair[1]))
            .fold(f64::INFINITY, f64::min),
    }
}

/// The plan bounds of some points: `[min x, min y, max x, max y]`, or infinities
/// inside out for none.
pub fn bounds(points: &[PlanPoint]) -> [f64; 4] {
    points.iter().fold(
        [
            f64::INFINITY,
            f64::INFINITY,
            f64::NEG_INFINITY,
            f64::NEG_INFINITY,
        ],
        |b, p| {
            [
                b[0].min(p[0]),
                b[1].min(p[1]),
                b[2].max(p[0]),
                b[3].max(p[1]),
            ]
        },
    )
}

/// Whether `p` is inside bounds from [`bounds`], `margin` metres widened.
pub fn within_bounds(b: &[f64; 4], p: PlanPoint, margin: f64) -> bool {
    p[0] >= b[0] - margin && p[0] <= b[2] + margin && p[1] >= b[1] - margin && p[1] <= b[3] + margin
}

/// Many segments, filed on a [`Grid`], for asking how near the nearest of them is
/// without measuring against all of them.
#[derive(Debug, Clone)]
pub struct Segments {
    grid: Grid<(PlanPoint, PlanPoint)>,
}

impl Segments {
    /// `cell` metres per grid cell: a query reads the cells its reach covers, so a
    /// cell about the size of the usual reach keeps that to a handful.
    pub fn new(cell: f64) -> Segments {
        Segments {
            grid: Grid::new(cell),
        }
    }

    pub fn insert(&mut self, a: PlanPoint, b: PlanPoint) {
        self.grid.insert(
            [a[0].min(b[0]), a[1].min(b[1])],
            [a[0].max(b[0]), a[1].max(b[1])],
            (a, b),
        );
    }

    /// Every segment of an open polyline.
    pub fn insert_polyline(&mut self, line: &[PlanPoint]) {
        for pair in line.windows(2) {
            self.insert(pair[0], pair[1]);
        }
    }

    /// Every segment of a closed ring, the closing one included.
    pub fn insert_ring(&mut self, ring: &[PlanPoint]) {
        self.insert_polyline(ring);
        if let (Some(&first), Some(&last)) = (ring.first(), ring.last()) {
            self.insert(last, first);
        }
    }

    /// The distance to the nearest segment, when one is within `reach` of `p`.
    pub fn within(&self, p: PlanPoint, reach: f64) -> Option<f64> {
        self.grid
            .covering([p[0] - reach, p[1] - reach], [p[0] + reach, p[1] + reach])
            .map(|&(a, b)| segment_distance(p, a, b))
            .fold(None, |best: Option<f64>, d| {
                Some(best.map_or(d, |b| b.min(d)))
            })
            .filter(|&d| d <= reach)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_point_is_inside_a_ring_or_not() {
        let square = [[0.0, 0.0], [10.0, 0.0], [10.0, 10.0], [0.0, 10.0]];
        assert!(ring_contains(&square, [5.0, 5.0]));
        assert!(!ring_contains(&square, [15.0, 5.0]));
    }

    #[test]
    fn rings_overlap_when_they_cross_or_one_holds_the_other() {
        let rect = |w: f64, h: f64| [[-w, -h], [w, -h], [w, h], [-w, h]];
        // A plus: no vertex of either inside the other.
        assert!(rings_overlap(&rect(100.0, 5.0), &rect(5.0, 100.0)));
        // One wholly inside the other.
        assert!(rings_overlap(&rect(100.0, 100.0), &rect(1.0, 1.0)));
        assert!(rings_overlap(&rect(1.0, 1.0), &rect(100.0, 100.0)));
        let far = [[200.0, 0.0], [210.0, 0.0], [210.0, 10.0], [200.0, 10.0]];
        assert!(!rings_overlap(&rect(10.0, 10.0), &far));
    }

    #[test]
    fn the_nearest_segment_is_found_within_reach_and_not_beyond() {
        let mut segments = Segments::new(10.0);
        segments.insert_polyline(&[[0.0, 0.0], [100.0, 0.0]]);
        assert_eq!(segments.within([50.0, 3.0], 5.0), Some(3.0));
        assert_eq!(segments.within([50.0, 30.0], 5.0), None);
        assert_eq!(
            polyline_distance([-4.0, 3.0], &[[0.0, 0.0], [10.0, 0.0]]),
            5.0
        );
    }
}
