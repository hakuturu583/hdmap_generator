//! A road's vertical alignment: grades meeting at points of intersection, each
//! meeting rounded by a parabolic vertical curve.
//!
//! The IR's curves carry a constant grade per piece, so a vertical curve is not a
//! piece of its own. It is a function of horizontal station that a plan is
//! [`draped`](crate::geometry::Curve3::draped) over, which cuts the plan finely
//! enough that the parabola is followed to the millimetre.

use crate::error::GeometryError;

/// Points of vertical intersection, with the radius of the curve that rounds each.
#[derive(Debug, Clone, PartialEq)]
pub struct VerticalProfile {
    /// `(station, height)`, ascending by station. The first and last are the
    /// road's ends; a curve is put at every one between.
    points: Vec<(f64, f64)>,
    /// The radius of the vertical curve at each interior point, metres. The ends
    /// have none; their entries are ignored.
    radii: Vec<f64>,
}

impl VerticalProfile {
    /// Grades between `points`, each interior point rounded with a curve of
    /// `radius` metres — or as large a one as fits between its neighbours.
    pub fn new(points: Vec<(f64, f64)>, radius: f64) -> Result<Self, GeometryError> {
        let count = points.len();
        VerticalProfile::with_radii(points, vec![radius; count])
    }

    pub fn with_radii(points: Vec<(f64, f64)>, radii: Vec<f64>) -> Result<Self, GeometryError> {
        if points.len() < 2 {
            return Err(GeometryError::TooFewPoints { got: points.len() });
        }
        if points.iter().any(|(s, z)| !s.is_finite() || !z.is_finite())
            || radii.iter().any(|r| !r.is_finite() || *r < 0.0)
            || radii.len() != points.len()
        {
            return Err(GeometryError::NonFiniteCoordinate);
        }
        if points.windows(2).any(|pair| pair[1].0 - pair[0].0 < 1e-6) {
            return Err(GeometryError::NoHorizontalExtent);
        }
        Ok(VerticalProfile { points, radii })
    }

    pub fn points(&self) -> &[(f64, f64)] {
        &self.points
    }

    fn grade(&self, index: usize) -> f64 {
        let (a, b) = (self.points[index], self.points[index + 1]);
        (b.1 - a.1) / (b.0 - a.0)
    }

    /// Half the length of the curve at interior point `index`: `R·|Δg|/2`, but
    /// never more than half the tangent on either side, so neighbouring curves do
    /// not overlap.
    fn half_length(&self, index: usize) -> f64 {
        let change = (self.grade(index) - self.grade(index - 1)).abs();
        let wanted = self.radii[index] * change / 2.0;
        let before = (self.points[index].0 - self.points[index - 1].0) / 2.0;
        let after = (self.points[index + 1].0 - self.points[index].0) / 2.0;
        wanted.min(before).min(after)
    }

    /// The height at `station`. Beyond the ends, the end grades carry on.
    pub fn height(&self, station: f64) -> f64 {
        let last = self.points.len() - 1;
        // The tangent line the station is on.
        let segment = self
            .points
            .windows(2)
            .position(|pair| station <= pair[1].0)
            .unwrap_or(last - 1);
        let (s0, z0) = self.points[segment];
        let mut z = z0 + self.grade(segment) * (station - s0);
        // Inside a curve at either end of that segment, the parabola instead.
        for index in [segment, segment + 1] {
            if index == 0 || index == last {
                continue;
            }
            let half = self.half_length(index);
            if half <= 0.0 {
                continue;
            }
            let (pi, zi) = self.points[index];
            if (station - pi).abs() < half {
                let (g1, g2) = (self.grade(index - 1), self.grade(index));
                let start = pi - half;
                let x = station - start;
                z = zi - g1 * half + g1 * x + (g2 - g1) * x * x / (4.0 * half);
            }
        }
        z
    }

    /// The steepest grade anywhere on it.
    pub fn steepest(&self) -> f64 {
        (0..self.points.len() - 1)
            .map(|index| self.grade(index).abs())
            .fold(0.0, f64::max)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_crest_is_rounded_and_the_tangents_are_kept() {
        let profile =
            VerticalProfile::new(vec![(0.0, 0.0), (500.0, 20.0), (1000.0, 0.0)], 3000.0).unwrap();
        // Grades of ±4 %, a curve 240 m long.
        assert!((profile.height(100.0) - 4.0).abs() < 1e-9);
        assert!((profile.height(900.0) - 4.0).abs() < 1e-9);
        // At the PVI the parabola sits L·Δg/8 below the intersection.
        let sag = 240.0 * 0.08 / 8.0;
        assert!((profile.height(500.0) - (20.0 - sag)).abs() < 1e-9);
        // And it meets the tangents without a step.
        assert!((profile.height(380.0) - 15.2).abs() < 1e-9);
    }
}
