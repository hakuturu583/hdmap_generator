//! The plane geometry a pit is laid out with.
//!
//! A pit is designed in its own frame — the centre of its floor at the origin, its
//! long axis along `x` — and placed in the map's by one rotation and one shift, so
//! that every offset, bearing and ring below is written as though the pit sat at
//! the origin pointing east.

use std::f64::consts::TAU;

use roadgen_core::geometry::Point3;

/// The pit's frame: where its centre is and which way its long axis points.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Frame {
    pub centre: Point3,
    /// Radians anticlockwise from the map's `+x` to the pit's long axis.
    pub heading: f64,
}

impl Frame {
    /// A point of the pit's frame, at height `z`, in the map's.
    pub fn place(&self, x: f64, y: f64, z: f64) -> Point3 {
        let (sin, cos) = self.heading.sin_cos();
        Point3::new(
            self.centre.x + x * cos - y * sin,
            self.centre.y + x * sin + y * cos,
            z,
        )
    }

    /// A direction of the pit's frame, in the map's.
    pub fn turn(&self, direction: [f64; 2]) -> [f64; 2] {
        let (sin, cos) = self.heading.sin_cos();
        [
            direction[0] * cos - direction[1] * sin,
            direction[0] * sin + direction[1] * cos,
        ]
    }
}

/// An ellipse in the pit's frame, and the curves parallel to it.
///
/// A bench of a pit is the floor's outline pushed outwards: every toe and crest is
/// a parallel curve of the floor, a fixed distance out along the normal. Parallel
/// curves of an ellipse outside it never fold — the ellipse is convex — so a ring
/// at any offset is a simple closed curve, which is what lets the benches be
/// stacked without checking them against each other.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Ellipse {
    /// Half its length along `x` and half its width along `y`.
    pub a: f64,
    pub b: f64,
}

impl Ellipse {
    /// The point at parameter `theta`, pushed `offset` out along the normal.
    pub fn at(&self, theta: f64, offset: f64) -> [f64; 2] {
        let (sin, cos) = theta.sin_cos();
        let normal = self.normal(theta);
        [
            self.a * cos + offset * normal[0],
            self.b * sin + offset * normal[1],
        ]
    }

    /// The outward unit normal at parameter `theta`.
    pub fn normal(&self, theta: f64) -> [f64; 2] {
        let (sin, cos) = theta.sin_cos();
        let (nx, ny) = (self.b * cos, self.a * sin);
        let length = nx.hypot(ny);
        [nx / length, ny / length]
    }

    /// The unit tangent at `theta`, the way `theta` increases: anticlockwise.
    pub fn tangent(&self, theta: f64) -> [f64; 2] {
        let normal = self.normal(theta);
        [-normal[1], normal[0]]
    }

    /// About how long the parallel curve at `offset` is.
    pub fn perimeter(&self, offset: f64) -> f64 {
        // Ramanujan's approximation, for the ellipse whose half-axes the parallel
        // curve has; the parallel curve is not an ellipse, but it is near enough
        // one to say how many vertices to put on it.
        let (a, b) = (self.a + offset, self.b + offset);
        let h = ((a - b) / (a + b)).powi(2);
        std::f64::consts::PI * (a + b) * (1.0 + 3.0 * h / (10.0 + (4.0 - 3.0 * h).sqrt()))
    }

    /// The parallel curve at `offset`, closed and open — the first vertex is not
    /// repeated — with vertices about `spacing` apart.
    pub fn ring(&self, offset: f64, spacing: f64) -> Vec<[f64; 2]> {
        let count = (self.perimeter(offset) / spacing).ceil().max(12.0) as usize;
        (0..count)
            .map(|index| self.at(TAU * index as f64 / count as f64, offset))
            .collect()
    }
}

/// The distance in plan from `p` to the segment `a..b`.
pub fn segment_distance(p: [f64; 2], a: [f64; 2], b: [f64; 2]) -> f64 {
    let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
    let length = dx * dx + dy * dy;
    let t = if length < 1e-18 {
        0.0
    } else {
        (((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / length).clamp(0.0, 1.0)
    };
    (p[0] - a[0] - t * dx).hypot(p[1] - a[1] - t * dy)
}

/// The distance in plan from `p` to the nearest segment of a polyline.
pub fn polyline_distance(p: [f64; 2], line: &[[f64; 2]]) -> f64 {
    match line {
        [] => f64::INFINITY,
        [only] => (p[0] - only[0]).hypot(p[1] - only[1]),
        _ => line
            .windows(2)
            .map(|pair| segment_distance(p, pair[0], pair[1]))
            .fold(f64::INFINITY, f64::min),
    }
}

/// Whether `p` is inside a closed ring, by the crossings of a ray cast from it.
pub fn ring_contains(ring: &[[f64; 2]], p: [f64; 2]) -> bool {
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

/// `0` below `edge0`, `1` above `edge1`, and a smooth step between.
pub fn smoothstep(edge0: f64, edge1: f64, x: f64) -> f64 {
    let t = ((x - edge0) / (edge1 - edge0)).clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

/// A segment in plan, from one end to the other.
type Segment = ([f64; 2], [f64; 2]);

/// A uniform grid of buckets over the plan, for asking which of many segments are
/// near a point without measuring against all of them.
pub struct SegmentIndex {
    cell: f64,
    buckets: std::collections::HashMap<(i64, i64), Vec<Segment>>,
}

impl SegmentIndex {
    pub fn new(cell: f64) -> Self {
        SegmentIndex {
            cell: cell.max(1e-3),
            buckets: std::collections::HashMap::new(),
        }
    }

    fn key(&self, x: f64, y: f64) -> (i64, i64) {
        (
            (x / self.cell).floor() as i64,
            (y / self.cell).floor() as i64,
        )
    }

    pub fn insert_polyline(&mut self, line: &[[f64; 2]]) {
        for pair in line.windows(2) {
            self.insert(pair[0], pair[1]);
        }
    }

    pub fn insert(&mut self, a: [f64; 2], b: [f64; 2]) {
        let (i0, j0) = self.key(a[0].min(b[0]), a[1].min(b[1]));
        let (i1, j1) = self.key(a[0].max(b[0]), a[1].max(b[1]));
        for j in j0..=j1 {
            for i in i0..=i1 {
                self.buckets.entry((i, j)).or_default().push((a, b));
            }
        }
    }

    /// The distance to the nearest segment, if one is within `reach`; segments
    /// further than that may be missed, and `reach` should be no more than a few
    /// cells.
    pub fn distance_within(&self, p: [f64; 2], reach: f64) -> Option<f64> {
        let (i0, j0) = self.key(p[0] - reach, p[1] - reach);
        let (i1, j1) = self.key(p[0] + reach, p[1] + reach);
        let mut best = f64::INFINITY;
        for j in j0..=j1 {
            for i in i0..=i1 {
                if let Some(segments) = self.buckets.get(&(i, j)) {
                    for &(a, b) in segments {
                        best = best.min(segment_distance(p, a, b));
                    }
                }
            }
        }
        (best <= reach).then_some(best)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_parallel_ring_stands_its_offset_off_the_ellipse() {
        let ellipse = Ellipse { a: 60.0, b: 35.0 };
        let ring = ellipse.ring(20.0, 2.0);
        let base = ellipse.ring(0.0, 0.25);
        let mut closed = base.clone();
        closed.push(base[0]);
        for point in ring {
            let distance = polyline_distance(point, &closed);
            assert!((distance - 20.0).abs() < 0.05, "{distance}");
        }
    }

    #[test]
    fn the_frame_turns_then_shifts() {
        let frame = Frame {
            centre: Point3::new(100.0, 50.0, 7.0),
            heading: std::f64::consts::FRAC_PI_2,
        };
        let point = frame.place(10.0, 0.0, 3.0);
        assert!((point.x - 100.0).abs() < 1e-9 && (point.y - 60.0).abs() < 1e-9);
        assert_eq!(point.z, 3.0);
    }
}
