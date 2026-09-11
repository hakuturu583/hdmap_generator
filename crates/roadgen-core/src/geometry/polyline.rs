//! A 3D polyline that cannot be empty.

use crate::error::GeometryError;
use crate::geometry::vector::{Frame3, Point3, UnitVector3};

/// An open 3D polyline with at least two distinct vertices.
///
/// The invariant is established by [`Polyline3::new`] and never rechecked: holding
/// one of these is proof that `first()`, `last()` and every tangent exist. (This is
/// the type the design notes call `NonEmptyPolyline`; there is no weaker polyline
/// type to confuse it with.)
#[derive(Debug, Clone, PartialEq)]
pub struct Polyline3 {
    points: Vec<Point3>,
}

impl Polyline3 {
    /// Two consecutive vertices closer than this are the same vertex.
    pub const MIN_SEGMENT: f64 = 1e-9;

    /// Builds a polyline, dropping vertices that repeat the one before them.
    pub fn new(points: impl IntoIterator<Item = Point3>) -> Result<Self, GeometryError> {
        let mut kept: Vec<Point3> = Vec::new();
        for point in points {
            if !point.x.is_finite() || !point.y.is_finite() || !point.z.is_finite() {
                return Err(GeometryError::NonFiniteCoordinate);
            }
            match kept.last() {
                Some(previous) if previous.distance_to(point) < Self::MIN_SEGMENT => continue,
                _ => kept.push(point),
            }
        }
        if kept.len() < 2 {
            return Err(GeometryError::TooFewPoints { got: kept.len() });
        }
        Ok(Polyline3 { points: kept })
    }

    pub fn points(&self) -> &[Point3] {
        &self.points
    }

    pub fn len(&self) -> usize {
        self.points.len()
    }

    /// Never true; kept so the type reads like a normal collection.
    pub fn is_empty(&self) -> bool {
        false
    }

    pub fn first(&self) -> Point3 {
        self.points[0]
    }

    pub fn last(&self) -> Point3 {
        self.points[self.points.len() - 1]
    }

    /// Total length in three dimensions.
    pub fn length(&self) -> f64 {
        self.points
            .windows(2)
            .map(|pair| pair[0].distance_to(pair[1]))
            .sum()
    }

    /// Total length after an explicit projection onto the horizontal plane.
    pub fn horizontal_length(&self) -> f64 {
        self.points
            .windows(2)
            .map(|pair| pair[0].horizontal_distance_to(pair[1]))
            .sum()
    }

    pub fn reversed(&self) -> Polyline3 {
        let mut points = self.points.clone();
        points.reverse();
        Polyline3 { points }
    }

    /// The direction of the polyline at vertex `index`.
    ///
    /// At an interior vertex this is the direction of the corner's bisector, so that
    /// a boundary offset from it lands on the miter of the two segments.
    pub fn tangent_at(&self, index: usize) -> Result<UnitVector3, GeometryError> {
        let last = self.points.len() - 1;
        let incoming = (index > 0)
            .then(|| (self.points[index] - self.points[index - 1]).normalize())
            .transpose()?;
        let outgoing = (index < last)
            .then(|| (self.points[index + 1] - self.points[index]).normalize())
            .transpose()?;
        match (incoming, outgoing) {
            (Some(a), Some(b)) => (a.get() + b.get()).normalize().or(Ok(b)),
            (Some(a), None) => Ok(a),
            (None, Some(b)) => Ok(b),
            (None, None) => Err(GeometryError::TooFewPoints { got: 1 }),
        }
    }

    /// The road local frame at vertex `index`.
    pub fn frame_at(&self, index: usize) -> Result<Frame3, GeometryError> {
        Frame3::from_tangent(self.points[index], self.tangent_at(index)?)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn p(x: f64, y: f64, z: f64) -> Point3 {
        Point3::new(x, y, z)
    }

    #[test]
    fn a_single_point_is_not_a_polyline() {
        assert!(matches!(
            Polyline3::new([p(0.0, 0.0, 0.0)]),
            Err(GeometryError::TooFewPoints { got: 1 })
        ));
    }

    #[test]
    fn repeated_vertices_collapse() {
        let line = Polyline3::new([p(0.0, 0.0, 0.0), p(0.0, 0.0, 0.0), p(1.0, 0.0, 0.0)]).unwrap();
        assert_eq!(line.len(), 2);
    }

    #[test]
    fn lengths_are_3d_unless_asked_otherwise() {
        let line = Polyline3::new([p(0.0, 0.0, 0.0), p(3.0, 4.0, 12.0)]).unwrap();
        assert!((line.length() - 13.0).abs() < 1e-12);
        assert!((line.horizontal_length() - 5.0).abs() < 1e-12);
    }

    #[test]
    fn an_interior_tangent_bisects_the_corner() {
        let line = Polyline3::new([p(-1.0, 0.0, 0.0), p(0.0, 0.0, 0.0), p(0.0, 1.0, 0.0)]).unwrap();
        let tangent = line.tangent_at(1).unwrap();
        let half = std::f64::consts::FRAC_1_SQRT_2;
        assert!((tangent.x() - half).abs() < 1e-12);
        assert!((tangent.y() - half).abs() < 1e-12);
    }
}
