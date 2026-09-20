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

    /// Stations from `from` to `to`, both included, evenly spaced and no further
    /// apart than the bound.
    pub fn stations_between(&self, from: f64, to: f64) -> impl Iterator<Item = f64> {
        let steps = self.segments_for(to - from);
        (0..=steps).map(move |step| from + (to - from) * step as f64 / steps as f64)
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

    pub fn start(&self) -> Point3 {
        self.start
    }

    pub fn heading(&self) -> f64 {
        self.heading
    }

    pub fn curvature(&self) -> f64 {
        self.curvature
    }

    pub fn horizontal_length(&self) -> f64 {
        self.horizontal_length
    }

    pub fn end_z(&self) -> f64 {
        self.end_z
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

/// A clothoid: an arc whose curvature changes linearly along it.
///
/// This is the transition a road actually uses between a straight and a bend — the
/// curve a vehicle traces while the steering wheel turns at a constant rate — and it
/// is what OpenDRIVE's `<spiral>` describes. A straight-to-arc joint without one is a
/// step change in lateral acceleration.
///
/// Grade is constant along the piece, as it is for [`Arc3`].
#[derive(Debug, Clone, PartialEq)]
pub struct Clothoid3 {
    start: Point3,
    heading: f64,
    curvature_start: f64,
    curvature_end: f64,
    horizontal_length: f64,
    end_z: f64,
}

impl Clothoid3 {
    /// Integration step for the spiral, metres.
    ///
    /// The position of a clothoid has no closed form in elementary functions — it is
    /// a Fresnel integral — so it is integrated. Simpson's error falls as the fourth
    /// power of the step, and a centimetre keeps it below a nanometre even for a
    /// spiral far sharper than any road, which is three orders below the micrometre
    /// at which this project welds points together.
    const STEP: f64 = 0.01;
    /// Bounds on the interval count, so a short span is still integrated accurately
    /// and a very long one cannot cost unboundedly much.
    const MIN_INTERVALS: usize = 16;
    const MAX_INTERVALS: usize = 4096;

    /// An even interval count for a span of `span` metres.
    fn intervals_for(span: f64) -> usize {
        let wanted = (span / Self::STEP).ceil() as usize;
        let clamped = wanted.clamp(Self::MIN_INTERVALS, Self::MAX_INTERVALS);
        // Simpson's rule needs an even number of intervals.
        clamped + clamped % 2
    }

    pub fn new(
        start: Point3,
        heading: f64,
        curvature_start: f64,
        curvature_end: f64,
        horizontal_length: f64,
        end_z: f64,
    ) -> Result<Self, GeometryError> {
        if !(horizontal_length.is_finite() && horizontal_length > Polyline3::MIN_SEGMENT) {
            return Err(GeometryError::NoHorizontalExtent);
        }
        if ![heading, curvature_start, curvature_end, end_z]
            .iter()
            .all(|value| value.is_finite())
        {
            return Err(GeometryError::NonFiniteCoordinate);
        }
        Ok(Clothoid3 {
            start,
            heading,
            curvature_start,
            curvature_end,
            horizontal_length,
            end_z,
        })
    }

    pub fn start(&self) -> Point3 {
        self.start
    }

    pub fn heading(&self) -> f64 {
        self.heading
    }

    pub fn curvature_start(&self) -> f64 {
        self.curvature_start
    }

    pub fn curvature_end(&self) -> f64 {
        self.curvature_end
    }

    pub fn horizontal_length(&self) -> f64 {
        self.horizontal_length
    }

    pub fn end_z(&self) -> f64 {
        self.end_z
    }

    /// Rate of change of curvature along the piece, per metre.
    fn sharpness(&self) -> f64 {
        (self.curvature_end - self.curvature_start) / self.horizontal_length
    }

    fn grade(&self) -> f64 {
        (self.end_z - self.start.z) / self.horizontal_length
    }

    /// Heading after travelling `station` metres, which is the integral of the
    /// curvature and so is available in closed form even though the position is not.
    fn heading_at(&self, station: f64) -> f64 {
        self.heading + self.curvature_start * station + 0.5 * self.sharpness() * station * station
    }

    fn tangent_at(&self, station: f64) -> UnitVector3 {
        let heading = self.heading_at(station);
        Vector3::new(heading.cos(), heading.sin(), self.grade())
            .normalize()
            .expect("a clothoid tangent always has a unit horizontal part")
    }

    /// Walks the spiral, returning the position at each requested station.
    ///
    /// `stations` must ascend and start at zero; they are integrated through in one
    /// pass so that the cost is linear in the number of samples rather than
    /// quadratic.
    fn walk(&self, stations: &[f64]) -> Vec<Point3> {
        let grade = self.grade();
        let mut points = Vec::with_capacity(stations.len());
        let (mut x, mut y) = (self.start.x, self.start.y);
        let mut from = 0.0;

        for &station in stations {
            let span = station - from;
            if span > 0.0 {
                let intervals = Self::intervals_for(span);
                let step = span / intervals as f64;
                // Composite Simpson: the ends count once, interior odd samples four
                // times and interior even samples twice.
                let (mut sum_x, mut sum_y) = (0.0, 0.0);
                for index in 0..=intervals {
                    let heading = self.heading_at(from + step * index as f64);
                    let weight = if index == 0 || index == intervals {
                        1.0
                    } else if index % 2 == 1 {
                        4.0
                    } else {
                        2.0
                    };
                    sum_x += weight * heading.cos();
                    sum_y += weight * heading.sin();
                }
                x += sum_x * step / 3.0;
                y += sum_y * step / 3.0;
                from = station;
            }
            points.push(Point3::new(x, y, self.start.z + grade * station));
        }
        points
    }

    fn end(&self) -> Point3 {
        self.walk(&[self.horizontal_length])
            .pop()
            .expect("one station in, one point out")
    }
}

/// A cubic Bézier in three dimensions.
///
/// This is what a junction connector is made of: the two end tangents are pinned to
/// the lanes it joins, so the connector leaves and arrives without a kink.
#[derive(Debug, Clone, PartialEq)]
pub struct Bezier3 {
    control: [Point3; 4],
    /// How many straight segments this curve is realised as.
    ///
    /// A cubic is parameterised by `t` rather than by arc length, so its vertices
    /// are placed at equal steps of `t` and the station of each is the arc length of
    /// the curve up to it. The resolution is part of the curve, fixed when it is
    /// built, rather than something each caller brings: [`Curve3::sample_at`] says
    /// the same thing from the other side, that a Bézier "is already an approximation
    /// of itself at the configured resolution".
    ///
    /// Were the resolution the caller's, the same curve would have one set of
    /// vertices for a road generated at two metres and another for one generated at
    /// one; a lane cut to the first would stop short of the last vertex the second
    /// produced, and a junction connector would end a sampling step away from the
    /// lane it joins.
    segments: usize,
}

impl Bezier3 {
    /// The curve through these control points, realised at `config`'s resolution.
    pub fn new(control: [Point3; 4], config: SamplingConfig) -> Result<Self, GeometryError> {
        for point in control {
            if !point.x.is_finite() || !point.y.is_finite() || !point.z.is_finite() {
                return Err(GeometryError::NonFiniteCoordinate);
            }
        }
        let polygon: f64 = control
            .windows(2)
            .map(|pair| pair[0].distance_to(pair[1]))
            .sum();
        // The control polygon is longer than the curve, so stepping it out at the
        // configured length errs towards more vertices rather than fewer. Eight is
        // the floor: below that a corner stops looking like one. And a curve that
        // turns hard in a short distance — a tight turn through a junction — gets a
        // vertex at least every five degrees of turn, so that the chords between the
        // vertices (which is what every polyline format receives) stay within a few
        // centimetres of the curve (which is what OpenDRIVE receives).
        let turn = {
            let leaving = (control[1] - control[0]).normalize();
            let arriving = (control[3] - control[2]).normalize();
            match (leaving, arriving) {
                (Ok(a), Ok(b)) => a.dot(b).clamp(-1.0, 1.0).acos(),
                _ => 0.0,
            }
        };
        let per_five_degrees = (turn / (5.0_f64).to_radians()).ceil() as usize;
        Ok(Bezier3 {
            control,
            segments: config.segments_for(polygon).max(8).max(per_five_degrees),
        })
    }

    /// How many straight segments the curve is realised as.
    pub fn segments(&self) -> usize {
        self.segments
    }

    /// The same curve walked the other way, at the same resolution.
    pub fn reversed(&self) -> Bezier3 {
        let [a, b, c, d] = self.control;
        Bezier3 {
            control: [d, c, b, a],
            segments: self.segments,
        }
    }

    /// The horizontal arc length of the curve.
    ///
    /// The arc length of the curve itself, not of the chords between its vertices —
    /// which is what OpenDRIVE's `<paramPoly3>` measures its stations in, so a vertex
    /// written at this station is the point the consumer evaluates there. It is the
    /// sum of [`Bezier3::segment_lengths`], so the last vertex's station is exactly
    /// this number.
    pub fn horizontal_length(&self) -> f64 {
        self.segment_lengths().iter().sum()
    }

    /// The horizontal arc length of each of the [`Bezier3::segments`] pieces the
    /// curve is cut into, from `t = 0` to `t = 1`.
    fn segment_lengths(&self) -> Vec<f64> {
        (0..self.segments)
            .map(|step| {
                let from = step as f64 / self.segments as f64;
                let to = (step + 1) as f64 / self.segments as f64;
                self.horizontal_arc_length(from, to)
            })
            .collect()
    }

    /// The horizontal arc length between parameters `from` and `to`.
    ///
    /// ∫ |dB/dt|_horizontal dt, by eight-point Gauss–Legendre quadrature: the
    /// integrand of a cubic's speed is smooth, so eight points on an interval an
    /// eighth of the curve long are exact to well below a micrometre.
    fn horizontal_arc_length(&self, from: f64, to: f64) -> f64 {
        // Nodes and weights for [-1, 1].
        const NODES: [f64; 4] = [
            0.183_434_642_495_649_8,
            0.525_532_409_916_329,
            0.796_666_477_413_626_7,
            0.960_289_856_497_536_3,
        ];
        const WEIGHTS: [f64; 4] = [
            0.362_683_783_378_362,
            0.313_706_645_877_887_3,
            0.222_381_034_453_374_5,
            0.101_228_536_290_376_3,
        ];
        let half = (to - from) / 2.0;
        let middle = (to + from) / 2.0;
        let speed = |t: f64| {
            let d = self.derivative(t);
            (d.x * d.x + d.y * d.y).sqrt()
        };
        let mut total = 0.0;
        for (node, weight) in NODES.iter().zip(WEIGHTS) {
            total += weight * (speed(middle - half * node) + speed(middle + half * node));
        }
        total * half
    }

    /// The cubic through `start` and `end` leaving along `start_tangent` and
    /// arriving along `end_tangent`, the Hermite form written as control points.
    pub fn hermite(
        start: Point3,
        start_tangent: UnitVector3,
        end: Point3,
        end_tangent: UnitVector3,
        config: SamplingConfig,
    ) -> Result<Self, GeometryError> {
        let chord = start.distance_to(end);
        if chord < Polyline3::MIN_SEGMENT {
            return Err(GeometryError::NoHorizontalExtent);
        }
        // A handle of a third of the chord is the standard choice: long enough to
        // round the corner, short enough not to overshoot when the ends face away
        // from each other.
        let handle = chord / 3.0;
        Bezier3::new(
            [
                start,
                start + start_tangent.scaled(handle),
                end - end_tangent.scaled(handle),
                end,
            ],
            config,
        )
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
}

/// The canonical curve type of the IR.
///
/// Every road reference line and every lane boundary is one of these.
#[derive(Debug, Clone, PartialEq)]
pub enum Curve3 {
    Line(Line3),
    Arc(Arc3),
    /// A transition whose curvature changes linearly.
    Clothoid(Clothoid3),
    Bezier(Bezier3),
    Polyline(Polyline3),
    /// Several curves end to end, each starting where the last one finished.
    ///
    /// This is what a real alignment is: straight, transition, bend, transition,
    /// straight. Build one with [`Curve3::composite`], which checks that the pieces
    /// actually meet.
    Composite(Vec<Curve3>),
}

impl Curve3 {
    pub fn line(start: Point3, end: Point3) -> Result<Self, GeometryError> {
        Ok(Curve3::Line(Line3::new(start, end)?))
    }

    pub fn polyline(points: impl IntoIterator<Item = Point3>) -> Result<Self, GeometryError> {
        Ok(Curve3::Polyline(Polyline3::new(points)?))
    }

    /// Chains curves end to end.
    ///
    /// A single curve is returned as itself rather than wrapped, so the exporters
    /// keep seeing the shape the caller meant. Fails if a piece does not start where
    /// its predecessor ended — a composite with a gap in it is not a curve.
    pub fn composite(segments: impl IntoIterator<Item = Curve3>) -> Result<Self, GeometryError> {
        let segments: Vec<Curve3> = segments.into_iter().collect();
        match segments.len() {
            0 => return Err(GeometryError::TooFewPoints { got: 0 }),
            1 => return Ok(segments.into_iter().next().expect("length checked")),
            _ => {}
        }
        for pair in segments.windows(2) {
            let gap = pair[0].end_point().distance_to(pair[1].start_point());
            if gap > Self::JOIN_TOLERANCE {
                return Err(GeometryError::DisjointSegments { gap });
            }
        }
        Ok(Curve3::Composite(segments))
    }

    /// How far apart two pieces of a composite may be and still count as joined.
    ///
    /// A millimetre: below what any map means to distinguish, and far above the
    /// rounding that accumulates when one piece's end is computed to start the next.
    pub const JOIN_TOLERANCE: f64 = 1e-3;

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
            Curve3::Clothoid(clothoid) => {
                let steps = config.segments_for(clothoid.horizontal_length);
                let stations: Vec<f64> = (0..=steps)
                    .map(|i| clothoid.horizontal_length * (i as f64 / steps as f64))
                    .collect();
                Ok(clothoid
                    .walk(&stations)
                    .into_iter()
                    .zip(&stations)
                    .map(|(point, &station)| Sample {
                        station,
                        point,
                        tangent: clothoid.tangent_at(station),
                    })
                    .collect())
            }
            Curve3::Composite(segments) => {
                let mut samples: Vec<Sample> = Vec::new();
                let mut offset = 0.0;
                for segment in segments {
                    let segment_samples = segment.samples(config)?;
                    let length = segment_samples
                        .last()
                        .expect("a curve samples at least its two ends")
                        .station;
                    for sample in segment_samples {
                        // The joint is one station, not two: the previous piece
                        // already put a sample there, and a repeated vertex would
                        // become a zero-length boundary segment downstream.
                        if !samples.is_empty() && sample.station == 0.0 {
                            continue;
                        }
                        samples.push(Sample {
                            station: offset + sample.station,
                            ..sample
                        });
                    }
                    offset += length;
                }
                Ok(samples)
            }
            Curve3::Bezier(bezier) => {
                // The curve's own resolution, not the caller's: see [`Bezier3`]. The
                // stations are arc lengths along the curve, so that the OpenDRIVE
                // export, which writes the curve itself, puts each vertex where the
                // IR has it.
                let steps = bezier.segments();
                let mut samples = Vec::with_capacity(steps + 1);
                let mut station = 0.0;
                samples.push(Sample {
                    station,
                    point: bezier.at(0.0),
                    tangent: bezier.tangent(0.0)?,
                });
                for (i, length) in bezier.segment_lengths().into_iter().enumerate() {
                    let t = (i + 1) as f64 / steps as f64;
                    station += length;
                    samples.push(Sample {
                        station,
                        point: bezier.at(t),
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

    /// The same stations as [`Curve3::samples`], plus each of `required`.
    ///
    /// A cross-section that changes partway along a road has to change at the station
    /// the caller asked for, not at the nearest vertex the sampler happened to
    /// produce, so the generator asks for those stations explicitly.
    pub fn samples_including(
        &self,
        config: SamplingConfig,
        required: &[f64],
    ) -> Result<Vec<Sample>, GeometryError> {
        let mut samples = self.samples(config)?;
        let length = samples
            .last()
            .expect("a curve samples at least its two ends")
            .station;
        for &station in required {
            if !station.is_finite() || station <= 0.0 || station >= length {
                continue;
            }
            if samples
                .iter()
                .any(|sample| (sample.station - station).abs() < Polyline3::MIN_SEGMENT)
            {
                continue;
            }
            samples.push(self.sample_at(station, config)?);
        }
        samples.sort_by(|left, right| {
            left.station
                .partial_cmp(&right.station)
                .expect("stations are finite")
        });
        Ok(samples)
    }

    /// The curve at one station, whether or not the sampler would have put a vertex
    /// there.
    ///
    /// Exact for the analytic variants. A Bézier or a polyline is already an
    /// approximation of itself at the configured resolution, so a station between two
    /// of its vertices is interpolated between them rather than re-solved.
    pub fn sample_at(&self, station: f64, config: SamplingConfig) -> Result<Sample, GeometryError> {
        match self {
            Curve3::Line(line) => {
                let length = line.start().horizontal_distance_to(line.end());
                let tangent = (line.end() - line.start()).normalize()?;
                Ok(Sample {
                    station,
                    point: line.start().lerp(line.end(), station / length),
                    tangent,
                })
            }
            Curve3::Arc(arc) => {
                let (point, tangent) = arc.at(station);
                Ok(Sample {
                    station,
                    point,
                    tangent,
                })
            }
            Curve3::Clothoid(clothoid) => Ok(Sample {
                station,
                point: clothoid.walk(&[station])[0],
                tangent: clothoid.tangent_at(station),
            }),
            Curve3::Composite(segments) => {
                let mut offset = 0.0;
                for segment in segments {
                    let length = segment.horizontal_length()?;
                    if station <= offset + length
                        || std::ptr::eq(segment, &segments[segments.len() - 1])
                    {
                        return Ok(Sample {
                            station,
                            ..segment.sample_at(station - offset, config)?
                        });
                    }
                    offset += length;
                }
                Err(GeometryError::NoHorizontalExtent)
            }
            Curve3::Bezier(_) | Curve3::Polyline(_) => {
                let samples = self.samples(config)?;
                let index = samples
                    .iter()
                    .rposition(|sample| sample.station <= station)
                    .unwrap_or(0)
                    .min(samples.len() - 2);
                let (before, after) = (samples[index], samples[index + 1]);
                let span = after.station - before.station;
                let t = if span > 0.0 {
                    (station - before.station) / span
                } else {
                    0.0
                };
                Ok(Sample {
                    station,
                    point: before.point.lerp(after.point, t),
                    tangent: (before.tangent.get() * (1.0 - t) + after.tangent.get() * t)
                        .normalize()
                        .unwrap_or(before.tangent),
                })
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
            Curve3::Clothoid(clothoid) => clothoid.start,
            Curve3::Bezier(bezier) => bezier.control[0],
            Curve3::Polyline(polyline) => polyline.first(),
            Curve3::Composite(segments) => segments[0].start_point(),
        }
    }

    pub fn end_point(&self) -> Point3 {
        match self {
            Curve3::Line(line) => line.end(),
            Curve3::Arc(arc) => arc.at(arc.horizontal_length).0,
            Curve3::Clothoid(clothoid) => clothoid.end(),
            Curve3::Bezier(bezier) => bezier.control[3],
            Curve3::Polyline(polyline) => polyline.last(),
            Curve3::Composite(segments) => segments[segments.len() - 1].end_point(),
        }
    }

    /// The tangent at the start, in closed form where the curve has one; a Bézier
    /// is sampled, as it is everywhere else.
    pub fn start_tangent(&self) -> Result<UnitVector3, GeometryError> {
        match self {
            Curve3::Line(line) => (line.end() - line.start()).normalize(),
            Curve3::Arc(arc) => Ok(arc.at(0.0).1),
            Curve3::Clothoid(clothoid) => Ok(clothoid.tangent_at(0.0)),
            Curve3::Polyline(polyline) => polyline.tangent_at(0),
            Curve3::Composite(segments) => segments[0].start_tangent(),
            Curve3::Bezier(_) => Ok(self
                .samples(SamplingConfig::default())?
                .first()
                .expect("a curve always samples at least its two ends")
                .tangent),
        }
    }

    /// The tangent at the end; see [`Curve3::start_tangent`].
    pub fn end_tangent(&self) -> Result<UnitVector3, GeometryError> {
        match self {
            Curve3::Line(line) => (line.end() - line.start()).normalize(),
            Curve3::Arc(arc) => Ok(arc.at(arc.horizontal_length).1),
            Curve3::Clothoid(clothoid) => Ok(clothoid.tangent_at(clothoid.horizontal_length)),
            Curve3::Polyline(polyline) => polyline.tangent_at(polyline.len() - 1),
            Curve3::Composite(segments) => segments[segments.len() - 1].end_tangent(),
            Curve3::Bezier(_) => Ok(self
                .samples(SamplingConfig::default())?
                .last()
                .expect("a curve always samples at least its two ends")
                .tangent),
        }
    }

    /// Arc length of the curve's shadow on the horizontal plane — the length an
    /// OpenDRIVE `<road>` reports.
    pub fn horizontal_length(&self) -> Result<f64, GeometryError> {
        match self {
            Curve3::Line(line) => Ok(line.start().horizontal_distance_to(line.end())),
            Curve3::Arc(arc) => Ok(arc.horizontal_length),
            Curve3::Clothoid(clothoid) => Ok(clothoid.horizontal_length),
            // A Bézier is parameterised by `t` rather than by arc length, so its
            // length is its vertices walked end to end — the same vertices
            // `samples` produces, whatever resolution the caller asks for.
            Curve3::Bezier(bezier) => Ok(bezier.horizontal_length()),
            Curve3::Polyline(polyline) => Ok(polyline.horizontal_length()),
            Curve3::Composite(segments) => segments
                .iter()
                .map(Curve3::horizontal_length)
                .sum::<Result<f64, GeometryError>>(),
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
            Curve3::Clothoid(clothoid) => {
                // Walking the spiral backwards swaps its two curvatures and negates
                // them, because the turn is now the other way round.
                let end_heading = clothoid.tangent_at(clothoid.horizontal_length).heading();
                Curve3::Clothoid(Clothoid3::new(
                    clothoid.end(),
                    end_heading + std::f64::consts::PI,
                    -clothoid.curvature_end,
                    -clothoid.curvature_start,
                    clothoid.horizontal_length,
                    clothoid.start.z,
                )?)
            }
            Curve3::Bezier(bezier) => Curve3::Bezier(bezier.reversed()),
            Curve3::Polyline(polyline) => Curve3::Polyline(polyline.reversed()),
            Curve3::Composite(segments) => Curve3::composite(
                segments
                    .iter()
                    .rev()
                    .map(|segment| segment.reversed(config))
                    .collect::<Result<Vec<_>, _>>()?,
            )?,
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

    /// The position `fraction` of the way along a polyline, by 3D arc length.
    fn point_at_fraction(polyline: &Polyline3, fraction: f64) -> Point3 {
        let target = polyline.length() * fraction;
        let mut walked = 0.0;
        for pair in polyline.points().windows(2) {
            let span = pair[0].distance_to(pair[1]);
            if walked + span >= target {
                return pair[0].lerp(pair[1], (target - walked) / span);
            }
            walked += span;
        }
        polyline.last()
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
            SamplingConfig::default(),
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
    fn a_clothoid_with_constant_curvature_is_an_arc() {
        // The degenerate case is the one with a closed form to check against: a
        // spiral whose curvature does not change has to trace the same path as the
        // arc of that curvature.
        let (radius, length) = (60.0, 40.0);
        let clothoid = Curve3::Clothoid(
            Clothoid3::new(
                p(3.0, -4.0, 2.0),
                0.4,
                1.0 / radius,
                1.0 / radius,
                length,
                5.0,
            )
            .unwrap(),
        );
        let arc =
            Curve3::Arc(Arc3::new(p(3.0, -4.0, 2.0), 0.4, 1.0 / radius, length, 5.0).unwrap());

        let config = SamplingConfig::new(1.0).unwrap();
        for (spiral, circle) in clothoid
            .samples(config)
            .unwrap()
            .iter()
            .zip(arc.samples(config).unwrap())
        {
            assert!(
                spiral.point.is_close(circle.point, 1e-9),
                "at s={}: {:?} vs {:?}",
                spiral.station,
                spiral.point,
                circle.point
            );
        }
    }

    #[test]
    fn a_clothoid_turns_by_the_integral_of_its_curvature() {
        // Curvature ramps linearly from 0 to 1/R over L, so the heading turns by
        // L/(2R) — the average curvature times the length.
        let (radius, length) = (50.0, 30.0);
        let clothoid = Curve3::Clothoid(
            Clothoid3::new(p(0.0, 0.0, 0.0), 0.0, 0.0, 1.0 / radius, length, 0.0).unwrap(),
        );
        let turn = clothoid.end_tangent().unwrap().heading();
        assert!((turn - length / (2.0 * radius)).abs() < 1e-9);

        // It leaves straight, which is the whole point of a transition curve.
        assert!(clothoid.start_tangent().unwrap().heading().abs() < 1e-12);
        assert!((clothoid.horizontal_length().unwrap() - length).abs() < 1e-9);
    }

    #[test]
    fn a_clothoid_matches_its_fresnel_integral() {
        // Against the standard Euler spiral: with curvature k(s) = s (sharpness 1,
        // starting straight at the origin), the position is (C(s), S(s)) scaled by
        // sqrt(pi) — the Fresnel integrals. Check one station against a series.
        let clothoid = Clothoid3::new(p(0.0, 0.0, 0.0), 0.0, 0.0, 1.0, 1.0, 0.0).unwrap();
        let end = clothoid.walk(&[1.0])[0];

        // x = ∫cos(s²/2) ds and y = ∫sin(s²/2) ds over [0, 1], by series expansion.
        let (mut x, mut y) = (0.0, 0.0);
        let steps = 200_000;
        for index in 0..steps {
            let s = (index as f64 + 0.5) / steps as f64;
            x += (s * s / 2.0).cos() / steps as f64;
            y += (s * s / 2.0).sin() / steps as f64;
        }
        assert!((end.x - x).abs() < 1e-9, "{} vs {x}", end.x);
        assert!((end.y - y).abs() < 1e-9, "{} vs {y}", end.y);
        // Walking there in steps gives the same answer as walking there at once: the
        // integration is accurate enough that how it is subdivided does not show.
        let stepped = clothoid.walk(&[0.3, 0.7, 1.0]);
        assert!(stepped[2].is_close(end, 1e-9));
    }

    #[test]
    fn a_composite_runs_through_its_pieces_in_order() {
        // The alignment a road really has: straight, transition, bend.
        let straight = Curve3::line(p(0.0, 0.0, 0.0), p(100.0, 0.0, 2.0)).unwrap();
        let transition = Curve3::Clothoid(
            Clothoid3::new(straight.end_point(), 0.0, 0.0, 0.02, 50.0, 3.0).unwrap(),
        );
        let bend = Curve3::Arc(
            Arc3::new(
                transition.end_point(),
                transition.end_tangent().unwrap().heading(),
                0.02,
                80.0,
                5.0,
            )
            .unwrap(),
        );
        let curve = Curve3::composite([straight, transition, bend]).unwrap();

        assert!((curve.horizontal_length().unwrap() - 230.0).abs() < 1e-6);
        assert!(curve.start_point().is_close(p(0.0, 0.0, 0.0), 1e-12));

        let samples = curve.samples(SamplingConfig::new(5.0).unwrap()).unwrap();
        // Stations ascend without repeating, and the joints appear once each.
        assert!(samples.windows(2).all(|w| w[1].station > w[0].station));
        assert!(samples
            .last()
            .unwrap()
            .point
            .is_close(curve.end_point(), 1e-6));

        // The curve is smooth: no two consecutive tangents disagree by much, which is
        // exactly what the transition piece buys.
        assert!(samples
            .windows(2)
            .all(|w| w[0].tangent.dot(w[1].tangent) > 0.99));
    }

    #[test]
    fn a_composite_with_a_gap_is_refused() {
        let first = Curve3::line(p(0.0, 0.0, 0.0), p(10.0, 0.0, 0.0)).unwrap();
        let second = Curve3::line(p(15.0, 0.0, 0.0), p(25.0, 0.0, 0.0)).unwrap();
        assert!(matches!(
            Curve3::composite([first, second]),
            Err(GeometryError::DisjointSegments { .. })
        ));
    }

    #[test]
    fn a_composite_of_one_is_that_one() {
        let line = Curve3::line(p(0.0, 0.0, 0.0), p(10.0, 0.0, 0.0)).unwrap();
        assert_eq!(Curve3::composite([line.clone()]).unwrap(), line);
        assert!(Curve3::composite([]).is_err());
    }

    #[test]
    fn reversing_a_clothoid_and_a_composite_swaps_their_ends() {
        let config = SamplingConfig::default();
        let clothoid =
            Curve3::Clothoid(Clothoid3::new(p(1.0, 2.0, 3.0), 0.3, 0.0, 0.02, 40.0, 4.0).unwrap());
        let composite = Curve3::composite([
            Curve3::line(p(0.0, 0.0, 0.0), p(50.0, 0.0, 1.0)).unwrap(),
            Curve3::Arc(Arc3::new(p(50.0, 0.0, 1.0), 0.0, 0.01, 40.0, 2.0).unwrap()),
        ])
        .unwrap();

        for curve in [clothoid, composite] {
            let reversed = curve.reversed(config).unwrap();
            assert!(reversed.start_point().is_close(curve.end_point(), 1e-6));
            assert!(reversed.end_point().is_close(curve.start_point(), 1e-6));
            // And it is the same path, not merely the same two ends: walking each
            // to the same fraction of its length lands in the same place. The two
            // polylines do not share vertices — a reversed composite samples its
            // pieces in the other order — so they are compared by arc length.
            let forward = curve.to_polyline(config).unwrap();
            let backward = reversed.to_polyline(config).unwrap();
            for fraction in [0.25, 0.5, 0.75] {
                let there = point_at_fraction(&forward, fraction);
                let back = point_at_fraction(&backward, 1.0 - fraction);
                assert!(there.is_close(back, 1e-6), "{there:?} vs {back:?}");
            }
        }
    }

    #[test]
    fn a_beziers_length_is_what_it_samples_to_at_any_resolution() {
        // A curve that reports one length and samples to another is a curve whose
        // last vertex falls outside the range a lane is cut to — which is a junction
        // connector that stops a sampling step short of the lane it joins.
        let built_at = SamplingConfig::new(1.0).unwrap();
        let bezier = Curve3::Bezier(
            Bezier3::hermite(
                p(0.0, 0.0, 0.0),
                Vector3::new(1.0, 0.0, 0.05).normalize().unwrap(),
                p(14.0, 9.0, 0.7),
                Vector3::new(0.0, 1.0, 0.02).normalize().unwrap(),
                built_at,
            )
            .unwrap(),
        );
        let stated = bezier.horizontal_length().unwrap();

        for step in [0.1, 0.25, 0.5, 1.0, 2.0, 4.0, 10.0] {
            let config = SamplingConfig::new(step).unwrap();
            let samples = bezier.samples(config).unwrap();
            let sampled = samples.last().unwrap().station;
            assert!(
                (sampled - stated).abs() < 1e-12,
                "asked at {step} m it samples to {sampled} but says it is {stated} long"
            );
            // The length is the curve's own arc length, which the chords between
            // the vertices fall a little short of — and the finer the vertices,
            // the less short.
            let walked = bezier.to_polyline(config).unwrap().horizontal_length();
            assert!(walked <= stated + 1e-9, "{walked} vs {stated}");
            assert!((walked - stated).abs() < 0.02, "{walked} vs {stated}");
        }
        // Against a brute-force walk of the curve, the quadrature is exact.
        let Curve3::Bezier(inner) = &bezier else {
            unreachable!()
        };
        let mut brute = 0.0;
        let mut previous = inner.control()[0];
        for step in 1..=200_000 {
            let point = inner.at(step as f64 / 200_000.0);
            brute += previous.horizontal_distance_to(point);
            previous = point;
        }
        assert!((brute - stated).abs() < 1e-7, "{brute} vs {stated}");
    }

    #[test]
    fn a_reversed_bezier_keeps_its_resolution_and_its_length() {
        let config = SamplingConfig::new(0.75).unwrap();
        let bezier = Curve3::Bezier(
            Bezier3::hermite(
                p(0.0, 0.0, 0.0),
                Vector3::new(1.0, 0.0, 0.0).normalize().unwrap(),
                p(20.0, 12.0, 0.0),
                Vector3::new(0.0, 1.0, 0.0).normalize().unwrap(),
                config,
            )
            .unwrap(),
        );
        let reversed = bezier.reversed(config).unwrap();
        assert!(
            (reversed.horizontal_length().unwrap() - bezier.horizontal_length().unwrap()).abs()
                < 1e-9
        );
        assert_eq!(
            reversed.samples(config).unwrap().len(),
            bezier.samples(config).unwrap().len()
        );
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
