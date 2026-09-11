//! Building a road alignment one piece at a time.
//!
//! A real alignment is a chain: straight, transition, bend, transition, straight.
//! Writing that as raw [`Curve3`] values means restating each piece's start point,
//! heading and curvature — the values the piece before it just finished with — and
//! getting one of them wrong is how a gap or a kink gets in.
//!
//! [`Alignment`] carries that state instead. Each call appends a piece that begins
//! exactly where the last one ended, pointing the way it was pointing, curving the
//! way it was curving, so the result is smooth by construction.

use crate::error::GeometryError;
use crate::geometry::curve::{Arc3, Clothoid3, Curve3, Line3};
use crate::geometry::vector::Point3;

/// A chain of curve pieces under construction.
#[derive(Debug, Clone)]
pub struct Alignment {
    segments: Vec<Curve3>,
    point: Point3,
    heading: f64,
    curvature: f64,
}

impl Alignment {
    /// Starts an alignment at `start`, heading `heading` radians counter-clockwise
    /// from the +x axis, curving not at all.
    pub fn new(start: Point3, heading: f64) -> Self {
        Alignment {
            segments: Vec::new(),
            point: start,
            heading,
            curvature: 0.0,
        }
    }

    /// Where the alignment has got to.
    pub fn point(&self) -> Point3 {
        self.point
    }

    /// The heading it is pointing in, radians.
    pub fn heading(&self) -> f64 {
        self.heading
    }

    /// The curvature it is turning at, per metre.
    pub fn curvature(&self) -> f64 {
        self.curvature
    }

    /// Appends `length` metres of straight, climbing `rise` metres over it.
    pub fn line(mut self, length: f64, rise: f64) -> Result<Self, GeometryError> {
        let end = Point3::new(
            self.point.x + length * self.heading.cos(),
            self.point.y + length * self.heading.sin(),
            self.point.z + rise,
        );
        let segment = Curve3::Line(Line3::new(self.point, end)?);
        self.point = end;
        self.curvature = 0.0;
        self.segments.push(segment);
        Ok(self)
    }

    /// Appends `length` metres of constant-curvature bend, climbing `rise` metres.
    ///
    /// A positive curvature turns to the left.
    pub fn arc(mut self, length: f64, curvature: f64, rise: f64) -> Result<Self, GeometryError> {
        let arc = Arc3::new(
            self.point,
            self.heading,
            curvature,
            length,
            self.point.z + rise,
        )?;
        let segment = Curve3::Arc(arc);
        self.point = segment.end_point();
        self.heading = segment.end_tangent()?.heading();
        self.curvature = curvature;
        self.segments.push(segment);
        Ok(self)
    }

    /// Appends `length` metres of transition, curving from wherever the alignment
    /// currently curves to `curvature_end`, and climbing `rise` metres.
    ///
    /// This is the piece that makes an alignment drivable: entering a bend straight
    /// from a straight is a step change in lateral acceleration, and a spiral is what
    /// spreads that change over a distance.
    pub fn spiral(
        mut self,
        length: f64,
        curvature_end: f64,
        rise: f64,
    ) -> Result<Self, GeometryError> {
        let clothoid = Clothoid3::new(
            self.point,
            self.heading,
            self.curvature,
            curvature_end,
            length,
            self.point.z + rise,
        )?;
        let segment = Curve3::Clothoid(clothoid);
        self.point = segment.end_point();
        self.heading = segment.end_tangent()?.heading();
        self.curvature = curvature_end;
        self.segments.push(segment);
        Ok(self)
    }

    /// Appends an already-built curve, which has to start where the alignment is.
    pub fn segment(mut self, curve: Curve3) -> Result<Self, GeometryError> {
        let gap = self.point.distance_to(curve.start_point());
        if gap > Curve3::JOIN_TOLERANCE {
            return Err(GeometryError::DisjointSegments { gap });
        }
        self.point = curve.end_point();
        self.heading = curve.end_tangent()?.heading();
        // An arbitrary curve does not report a curvature to carry forward; a spiral
        // after one has to be told where to start.
        self.curvature = 0.0;
        self.segments.push(curve);
        Ok(self)
    }

    /// The finished curve. A chain of one piece is that piece.
    pub fn finish(self) -> Result<Curve3, GeometryError> {
        Curve3::composite(self.segments)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::SamplingConfig;

    #[test]
    fn a_chain_is_continuous_and_smooth_by_construction() {
        let radius = 80.0;
        let curve = Alignment::new(Point3::new(10.0, 5.0, 0.0), 0.0)
            .line(100.0, 2.0)
            .unwrap()
            .spiral(60.0, 1.0 / radius, 1.0)
            .unwrap()
            .arc(90.0, 1.0 / radius, 2.0)
            .unwrap()
            .spiral(60.0, 0.0, 1.0)
            .unwrap()
            .line(100.0, 2.0)
            .unwrap()
            .finish()
            .unwrap();

        assert!((curve.horizontal_length().unwrap() - 410.0).abs() < 1e-6);
        assert!(curve
            .start_point()
            .is_close(Point3::new(10.0, 5.0, 0.0), 1e-12));
        // Eight metres of climb, spread over the five pieces.
        assert!((curve.end_point().z - 8.0).abs() < 1e-9);

        // Smooth: consecutive tangents along the whole chain agree closely, which is
        // the property the transitions exist to provide.
        let samples = curve.samples(SamplingConfig::new(2.0).unwrap()).unwrap();
        assert!(samples
            .windows(2)
            .all(|pair| pair[0].tangent.dot(pair[1].tangent) > 0.999));

        // It leaves and arrives straight, having turned through the bend and back.
        assert!(curve.start_tangent().unwrap().heading().abs() < 1e-12);
        let total_turn = curve.end_tangent().unwrap().heading();
        // 60/(2R) + 90/R + 60/(2R) = 150/R
        assert!((total_turn - 150.0 / radius).abs() < 1e-6);
    }

    #[test]
    fn a_spiral_picks_up_the_curvature_it_was_left_at() {
        let alignment = Alignment::new(Point3::ORIGIN, 0.0)
            .arc(50.0, 0.02, 0.0)
            .unwrap();
        assert!((alignment.curvature() - 0.02).abs() < 1e-12);

        // Ramping back to straight from there turns by the average curvature.
        let before = alignment.heading();
        let alignment = alignment.spiral(40.0, 0.0, 0.0).unwrap();
        assert!((alignment.heading() - before - 40.0 * 0.01).abs() < 1e-9);
        assert!(alignment.curvature().abs() < 1e-12);
    }

    #[test]
    fn appending_a_curve_that_starts_elsewhere_is_refused() {
        let stray =
            Curve3::line(Point3::new(500.0, 0.0, 0.0), Point3::new(600.0, 0.0, 0.0)).unwrap();
        assert!(matches!(
            Alignment::new(Point3::ORIGIN, 0.0).segment(stray),
            Err(GeometryError::DisjointSegments { .. })
        ));
    }

    #[test]
    fn a_chain_of_one_is_that_one() {
        let curve = Alignment::new(Point3::ORIGIN, 0.0)
            .line(100.0, 0.0)
            .unwrap()
            .finish()
            .unwrap();
        assert!(matches!(curve, Curve3::Line(_)));
    }
}
