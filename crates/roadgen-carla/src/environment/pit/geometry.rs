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

    /// The radius of curvature at `theta`. The parallel curve at `offset` curves
    /// with this plus `offset`, so an *inward* one — a negative offset — folds where
    /// the offset reaches past it: at the ends of the long axis first, where the
    /// radius is `b² / a`.
    pub fn radius(&self, theta: f64) -> f64 {
        let (sin, cos) = theta.sin_cos();
        (self.a * self.a * sin * sin + self.b * self.b * cos * cos).powf(1.5) / (self.a * self.b)
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

#[cfg(test)]
mod tests {
    use super::*;
    use roadgen_core::geometry::plan::polyline_distance;

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
