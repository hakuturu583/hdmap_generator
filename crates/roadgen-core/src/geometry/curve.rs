//! Canonical 3D curves.
//!
//! Every curve is parameterised by its *horizontal station* `s`: the arc length of
//! its shadow on the horizontal plane, measured from the start. That is the same
//! parameter OpenDRIVE uses, which keeps the export lowering trivial, and it stays a
//! purely 3D object here because elevation is carried along rather than discarded.
//!
//! The variants below are the ones the generator needs today. Adding `Clothoid`,
//! `Spline` or a superelevated variant is a matter of adding an arm and its
//! `samples` implementation: nothing outside this module inspects the variants.

use crate::error::GeometryError;
use crate::geometry::polyline::Polyline3;
use crate::geometry::vector::{Frame3, Point3, UnitVector3, Vector3};

/// How finely a curved element is turned into vertices.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SamplingConfig {
    /// Upper bound on the horizontal length of one generated segment, metres.
    pub max_segment_length: f64,
}

impl SamplingConfig {
    pub fn new(max_segment_length: f64) -> Result<Self, GeometryError> {
        if !(max_segment_length.is_finite() && max_segment_length > 0.0) {
            return Err(GeometryError::InvalidSampling { max_segment_length });
        }
        Ok(SamplingConfig { max_segment_length })
    }

    /// Number of segments needed to cover `length` without exceeding the bound.
    fn segments_for(&self, length: f64) -> usize {
        ((length / self.max_segment_length).ceil() as usize).max(1)
    }
}

impl Default for SamplingConfig {
    fn default() -> Self {
        SamplingConfig {
            max_segment_length: 2.0,
        }
    }
}

/// One evaluated station of a curve.
#[derive(Debug, Clone, Copy)]
pub struct Sample {
    /// Horizontal station from the start of the curve, metres.
    pub station: f64,
    pub point: Point3,
    pub tangent: UnitVector3,
}

impl Sample {
    /// The road local frame at this station.
    pub fn frame(&self) -> Result<Frame3, GeometryError> {
        Frame3::from_tangent(self.point, self.tangent)
    }
}

/// A straight segment in three dimensions.
#[derive(Debug, Clone, PartialEq)]
pub struct Line3 {
    start: Point3,
    end: Point3,
}

impl Line3 {
    /// Fails when the two ends share a horizontal position: such a segment has no
    /// heading, and no road can be built along it.
    pub fn new(start: Point3, end: Point3) -> Result<Self, GeometryError> {
        let horizontal = start.horizontal_distance_to(end);
        if horizontal < Polyline3::MIN_SEGMENT {
            return Err(GeometryError::NoHorizontalExtent);
        }
        Ok(Line3 { start, end })
    }

    pub fn start(&self) -> Point3 {
        self.start
    }

    pub fn end(&self) -> Point3 {
        self.end
    }
}

/// A constant-curvature arc in plan, with a constant grade.
#[derive(Debug, Clone, PartialEq)]
pub struct Arc3 {
    start: Point3,
    heading: f64,
    curvature: f64,
    horizontal_length: f64,
    end_z: f64,
}

impl Arc3 {
    pub fn new(
        start: Point3,
        heading: f64,
        curvature: f64,
        horizontal_length: f64,
        end_z: f64,
    ) -> Result<Self, GeometryError> {
        if !(horizontal_length.is_finite() && horizontal_length > Polyline3::MIN_SEGMENT) {
            return Err(GeometryError::NoHorizontalExtent);
        }
        if !heading.is_finite() || !curvature.is_finite() || !end_z.is_finite() {
            return Err(GeometryError::NonFiniteCoordinate);
        }
        Ok(Arc3 {
            start,
            heading,
            curvature,
            horizontal_length,
            end_z,
        })
    }

    fn grade(&self) -> f64 {
        (self.end_z - self.start.z) / self.horizontal_length
    }

    fn at(&self, station: f64) -> (Point3, UnitVector3) {
        let heading = self.heading + self.curvature * station;
        let (x, y) = if self.curvature.abs() < 1e-12 {
            (
                self.start.x + station * self.heading.cos(),
                self.start.y + station * self.heading.sin(),
            )
        } else {
            (
                self.start.x + (heading.sin() - self.heading.sin()) / self.curvature,
                self.start.y - (heading.cos() - self.heading.cos()) / self.curvature,
            )
        };
        let grade = self.grade();
        let point = Point3::new(x, y, self.start.z + grade * station);
        let tangent = Vector3::new(heading.cos(), heading.sin(), grade)
            .normalize()
            .expect("an arc tangent always has a unit horizontal part");
        (point, tangent)
    }
}

/// A cubic Bézier in three dimensions.
///
/// This is what a junction connector is made of: the two end tangents are pinned to
/// the lanes it joins, so the connector leaves and arrives without a kink.
#[derive(Debug, Clone, PartialEq)]
pub struct Bezier3 {
    control: [Point3; 4],
}

impl Bezier3 {
    pub fn new(control: [Point3; 4]) -> Result<Self, GeometryError> {
        for point in control {
            if !point.x.is_finite() || !point.y.is_finite() || !point.z.is_finite() {
                return Err(GeometryError::NonFiniteCoordinate);
            }
        }
        Ok(Bezier3 { control })
    }

    /// The cubic through `start` and `end` leaving along `start_tangent` and
    /// arriving along `end_tangent`, the Hermite form written as control points.
    pub fn hermite(
        start: Point3,
        start_tangent: UnitVector3,
        end: Point3,
        end_tangent: UnitVector3,
    ) -> Result<Self, GeometryError> {
        let chord = start.distance_to(end);
        if chord < Polyline3::MIN_SEGMENT {
            return Err(GeometryError::NoHorizontalExtent);
        }
        // A handle of a third of the chord is the standard choice: long enough to
        // round the corner, short enough not to overshoot when the ends face away
        // from each other.
        let handle = chord / 3.0;
        Ok(Bezier3 {
            control: [
                start,
                start + start_tangent.scaled(handle),
                end - end_tangent.scaled(handle),
                end,
            ],
        })
    }

    pub fn control(&self) -> &[Point3; 4] {
        &self.control
    }

    fn at(&self, t: f64) -> Point3 {
        let [a, b, c, d] = self.control;
        let u = 1.0 - t;
        let (w0, w1, w2, w3) = (u * u * u, 3.0 * u * u * t, 3.0 * u * t * t, t * t * t);
        Point3::new(
            a.x * w0 + b.x * w1 + c.x * w2 + d.x * w3,
            a.y * w0 + b.y * w1 + c.y * w2 + d.y * w3,
            a.z * w0 + b.z * w1 + c.z * w2 + d.z * w3,
        )
    }

    fn derivative(&self, t: f64) -> Vector3 {
        let [a, b, c, d] = self.control;
        let u = 1.0 - t;
        (b - a) * (3.0 * u * u) + (c - b) * (6.0 * u * t) + (d - c) * (3.0 * t * t)
    }

    fn tangent(&self, t: f64) -> Result<UnitVector3, GeometryError> {
        // A cusp in the parameterisation is not a cusp in the shape; step off it.
        for probe in [t, t + 1e-6, t - 1e-6] {
            if let Ok(tangent) = self.derivative(probe.clamp(0.0, 1.0)).normalize() {
                return Ok(tangent);
            }
        }
        Err(GeometryError::ZeroLengthVector)
    }

    /// Length of the control polygon, an upper bound used to pick a step count.
    fn control_polygon_length(&self) -> f64 {
        self.control
            .windows(2)
            .map(|pair| pair[0].distance_to(pair[1]))
            .sum()
    }
}

/// The canonical curve type of the IR.
///
/// Every road reference line and every lane boundary is one of these.
#[derive(Debug, Clone, PartialEq)]
pub enum Curve3 {
    Line(Line3),
    Arc(Arc3),
    Bezier(Bezier3),
    Polyline(Polyline3),
}

impl Curve3 {
    pub fn line(start: Point3, end: Point3) -> Result<Self, GeometryError> {
        Ok(Curve3::Line(Line3::new(start, end)?))
    }

    pub fn polyline(points: impl IntoIterator<Item = Point3>) -> Result<Self, GeometryError> {
        Ok(Curve3::Polyline(Polyline3::new(points)?))
    }

    /// Evaluated stations, always including both ends, in increasing station order.
    pub fn samples(&self, config: SamplingConfig) -> Result<Vec<Sample>, GeometryError> {
        match self {
            Curve3::Line(line) => {
                let direction = (line.end() - line.start()).normalize()?;
                let station = line.start().horizontal_distance_to(line.end());
                Ok(vec![
                    Sample {
                        station: 0.0,
                        point: line.start(),
                        tangent: direction,
                    },
                    Sample {
                        station,
                        point: line.end(),
                        tangent: direction,
                    },
                ])
            }
            Curve3::Arc(arc) => {
                let steps = config.segments_for(arc.horizontal_length);
                Ok((0..=steps)
                    .map(|i| {
                        let station = arc.horizontal_length * (i as f64 / steps as f64);
                        let (point, tangent) = arc.at(station);
                        Sample {
                            station,
                            point,
                            tangent,
                        }
                    })
                    .collect())
            }
            Curve3::Bezier(bezier) => {
                let steps = config.segments_for(bezier.control_polygon_length()).max(8);
                let mut samples = Vec::with_capacity(steps + 1);
                let mut station = 0.0;
                let mut previous = bezier.at(0.0);
                for i in 0..=steps {
                    let t = i as f64 / steps as f64;
                    let point = bezier.at(t);
                    station += previous.horizontal_distance_to(point);
                    previous = point;
                    samples.push(Sample {
                        station,
                        point,
                        tangent: bezier.tangent(t)?,
                    });
                }
                Ok(samples)
            }
            Curve3::Polyline(polyline) => {
                let mut samples = Vec::with_capacity(polyline.len());
                let mut station = 0.0;
                for (index, point) in polyline.points().iter().enumerate() {
                    if index > 0 {
                        station += polyline.points()[index - 1].horizontal_distance_to(*point);
                    }
                    samples.push(Sample {
                        station,
                        point: *point,
                        tangent: polyline.tangent_at(index)?,
                    });
                }
                Ok(samples)
            }
        }
    }

    /// The curve as vertices, at the given resolution.
    pub fn to_polyline(&self, config: SamplingConfig) -> Result<Polyline3, GeometryError> {
        Polyline3::new(self.samples(config)?.into_iter().map(|s| s.point))
    }

    pub fn start_point(&self) -> Point3 {
        match self {
            Curve3::Line(line) => line.start(),
            Curve3::Arc(arc) => arc.start,
            Curve3::Bezier(bezier) => bezier.control[0],
            Curve3::Polyline(polyline) => polyline.first(),
        }
    }

    pub fn end_point(&self) -> Point3 {
        match self {
            Curve3::Line(line) => line.end(),
            Curve3::Arc(arc) => arc.at(arc.horizontal_length).0,
            Curve3::Bezier(bezier) => bezier.control[3],
            Curve3::Polyline(polyline) => polyline.last(),
        }
    }

    pub fn start_tangent(&self) -> Result<UnitVector3, GeometryError> {
        Ok(self
            .samples(SamplingConfig::default())?
            .first()
            .expect("a curve always samples at least its two ends")
            .tangent)
    }

    pub fn end_tangent(&self) -> Result<UnitVector3, GeometryError> {
        Ok(self
            .samples(SamplingConfig::default())?
            .last()
            .expect("a curve always samples at least its two ends")
            .tangent)
    }

    /// Arc length of the curve's shadow on the horizontal plane — the length an
    /// OpenDRIVE `<road>` reports.
    pub fn horizontal_length(&self) -> Result<f64, GeometryError> {
        match self {
            Curve3::Line(line) => Ok(line.start().horizontal_distance_to(line.end())),
            Curve3::Arc(arc) => Ok(arc.horizontal_length),
            Curve3::Bezier(_) => Ok(self
                .samples(SamplingConfig::default())?
                .last()
                .expect("a curve always samples at least its two ends")
                .station),
            Curve3::Polyline(polyline) => Ok(polyline.horizontal_length()),
        }
    }

    /// True arc length, in three dimensions.
    pub fn length(&self, config: SamplingConfig) -> Result<f64, GeometryError> {
        Ok(self.to_polyline(config)?.length())
    }

    pub fn reversed(&self, config: SamplingConfig) -> Result<Curve3, GeometryError> {
        match self {
            Curve3::Line(line) => Curve3::Line(Line3::new(line.end(), line.start())?),
            Curve3::Arc(arc) => {
                let (end_point, end_tangent) = arc.at(arc.horizontal_length);
                Curve3::Arc(Arc3::new(
                    end_point,
                    end_tangent.heading() + std::f64::consts::PI,
                    -arc.curvature,
                    arc.horizontal_length,
                    arc.start.z,
                )?)
            }
            Curve3::Bezier(bezier) => {
                let [a, b, c, d] = bezier.control;
                Curve3::Bezier(Bezier3::new([d, c, b, a])?)
            }
            Curve3::Polyline(polyline) => Curve3::Polyline(polyline.reversed()),
        }
        .tap_validate(config)
    }

    /// Cheap sanity check used after constructing a derived curve.
    fn tap_validate(self, config: SamplingConfig) -> Result<Curve3, GeometryError> {
        self.samples(config)?;
        Ok(self)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn p(x: f64, y: f64, z: f64) -> Point3 {
        Point3::new(x, y, z)
    }

    #[test]
    fn a_line_keeps_its_grade() {
        let curve = Curve3::line(p(0.0, 0.0, 10.0), p(100.0, 0.0, 12.0)).unwrap();
        assert!((curve.horizontal_length().unwrap() - 100.0).abs() < 1e-9);
        assert!((curve.length(SamplingConfig::default()).unwrap() - 100.0199_f64).abs() < 1e-3);
        assert!((curve.start_tangent().unwrap().grade() - 0.02).abs() < 1e-12);
    }

    #[test]
    fn a_vertical_line_is_rejected() {
        assert!(matches!(
            Curve3::line(p(0.0, 0.0, 0.0), p(0.0, 0.0, 5.0)),
            Err(GeometryError::NoHorizontalExtent)
        ));
    }

    #[test]
    fn a_quarter_circle_arc_lands_where_the_geometry_says() {
        let radius = 50.0;
        let arc = Arc3::new(
            p(0.0, 0.0, 0.0),
            0.0,
            1.0 / radius,
            std::f64::consts::FRAC_PI_2 * radius,
            3.0,
        )
        .unwrap();
        let curve = Curve3::Arc(arc);
        let end = curve.end_point();
        assert!(end.is_close(p(radius, radius, 3.0), 1e-9));
        // Heading has turned by exactly 90 degrees.
        let heading = curve.end_tangent().unwrap().heading();
        assert!((heading - std::f64::consts::FRAC_PI_2).abs() < 1e-9);
    }

    #[test]
    fn a_hermite_connector_leaves_and_arrives_along_the_given_tangents() {
        let start_tangent = Vector3::new(1.0, 0.0, 0.0).normalize().unwrap();
        let end_tangent = Vector3::new(0.0, 1.0, 0.0).normalize().unwrap();
        let bezier = Bezier3::hermite(
            p(0.0, 0.0, 0.0),
            start_tangent,
            p(20.0, 20.0, 1.0),
            end_tangent,
        )
        .unwrap();
        let curve = Curve3::Bezier(bezier);
        assert!(curve.start_tangent().unwrap().dot(start_tangent) > 1.0 - 1e-9);
        assert!(curve.end_tangent().unwrap().dot(end_tangent) > 1.0 - 1e-9);
    }

    #[test]
    fn reversing_swaps_the_ends_of_every_variant() {
        let config = SamplingConfig::default();
        let curves = [
            Curve3::line(p(0.0, 0.0, 1.0), p(10.0, 5.0, 3.0)).unwrap(),
            Curve3::Arc(Arc3::new(p(0.0, 0.0, 0.0), 0.3, 0.02, 30.0, 2.0).unwrap()),
            Curve3::polyline([p(0.0, 0.0, 0.0), p(5.0, 1.0, 1.0), p(9.0, 6.0, 2.0)]).unwrap(),
        ];
        for curve in curves {
            let reversed = curve.reversed(config).unwrap();
            assert!(reversed.start_point().is_close(curve.end_point(), 1e-9));
            assert!(reversed.end_point().is_close(curve.start_point(), 1e-9));
        }
    }

    #[test]
    fn sampling_respects_the_segment_bound() {
        let arc = Curve3::Arc(Arc3::new(p(0.0, 0.0, 0.0), 0.0, 0.01, 100.0, 0.0).unwrap());
        let samples = arc.samples(SamplingConfig::new(5.0).unwrap()).unwrap();
        assert_eq!(samples.len(), 21);
        assert!(samples
            .windows(2)
            .all(|w| w[1].station - w[0].station <= 5.0 + 1e-9));
    }
}
