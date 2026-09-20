//! The reference line, from `<planView>` and `<elevationProfile>`.
//!
//! OpenDRIVE keeps the plan view and the elevation in separate elements, on
//! separate stations; the IR holds one 3D curve whose pieces each climb at a
//! constant grade. So a plan-view piece is cut wherever the elevation profile has a
//! knot inside it, and each cut piece takes the height at both its ends. Where the
//! profile is straight across several cuts they are joined back up, so a road
//! written from the IR — whose elevation is straight over every sampled span, and
//! straight across all of them on a road of constant grade — comes back as the
//! pieces it went out as.
//!
//! A `<paramPoly3>` is a cubic and cannot be cut, so it takes a straight grade
//! from its start height to its end height; and a `<poly3>` is a shape the IR has
//! no piece for, so it is walked into a polyline. Both are reported.

use opendrive::road::geometry::geometry_type::GeometryType;
use opendrive::road::geometry::param_poly_3_p_range::ParamPoly3pRange;
use opendrive::road::geometry::Geometry;
use opendrive::road::Road as OdRoad;
use uom::si::angle::radian;
use uom::si::curvature::radian_per_meter;
use uom::si::length::meter;

use roadgen_core::geometry::{
    Arc3, Bezier3, Clothoid3, Curve3, Line3, Point3, Poly3Piece, Poly3Profile, Polyline3,
    SamplingConfig,
};

use super::Approximations;
use crate::error::ImportError;

/// How far apart the end of one plan-view piece and the start of the next may be
/// and still be joined by moving the second onto the first, metres. Beyond this the
/// document describes two roads, not one with a rounding error in it.
const SNAP_TOLERANCE: f64 = 0.1;

/// Heights closer than this are the same height.
const HEIGHT_TOLERANCE: f64 = 1e-6;

/// The road's reference line as one [`Curve3`].
pub fn reference_line(
    road: &OdRoad,
    config: SamplingConfig,
    approximations: &mut Approximations,
) -> Result<Curve3, ImportError> {
    let elevation = Elevation::of(road)?;
    let mut pieces: Vec<Curve3> = Vec::new();
    let mut previous_end: Option<Point3> = None;

    for entry in road.plan_view.geometry.iter() {
        let declared = Point3::new(
            entry.x.get::<meter>(),
            entry.y.get::<meter>(),
            elevation.at(entry.s.get::<meter>()),
        );
        // The file's own start, unless the previous piece ended a hair away from
        // it, in which case the previous piece's end is where this one starts.
        let start = match previous_end {
            Some(end) if end.horizontal_distance_to(declared) > Curve3::JOIN_TOLERANCE => {
                let gap = end.horizontal_distance_to(declared);
                if gap > SNAP_TOLERANCE {
                    return Err(ImportError::Inconsistent(format!(
                        "road {}: the plan-view piece at s={} starts {gap:.3} m from where \
                         the previous one ends",
                        road.id,
                        entry.s.get::<meter>()
                    )));
                }
                approximations.note(format!(
                    "road {}: the plan-view piece at s={} starts {gap:.4} m from where the \
                     previous one ends, and is moved to meet it",
                    road.id,
                    entry.s.get::<meter>()
                ));
                Point3::new(end.x, end.y, declared.z)
            }
            _ => declared,
        };
        let piece = Piece::new(entry, start);
        let built = piece.build(&elevation, config, &road.id, approximations)?;
        previous_end = built.last().map(Curve3::end_point);
        pieces.extend(built);
    }
    if pieces.is_empty() {
        return Err(ImportError::Inconsistent(format!(
            "road {} has no plan-view geometry",
            road.id
        )));
    }
    Ok(Curve3::composite(pieces)?)
}

/// The elevation profile as a function of station.
///
/// The pieces are the profile's own, sorted by station; every question below is
/// answered by a binary search for the governing piece rather than a scan, because
/// a road written from the IR has a piece per change of grade and a foreign one
/// may have one per metre.
struct Elevation {
    profile: Poly3Profile,
}

impl Elevation {
    fn of(road: &OdRoad) -> Result<Elevation, ImportError> {
        let rows = road
            .elevation_profile
            .iter()
            .flat_map(|profile| profile.elevation.iter())
            .map(|row| (row.s, row.a, row.b, row.c, row.d));
        Ok(Elevation {
            profile: super::profile(rows)?,
        })
    }

    fn pieces(&self) -> &[Poly3Piece] {
        self.profile.pieces()
    }

    /// The index of the piece that governs `station`: the last one starting at or
    /// before it, a hair's tolerance allowed at a knot so that a knot's station is
    /// its own piece's.
    fn governing(&self, station: f64) -> usize {
        self.pieces()
            .partition_point(|piece| piece.station <= station + 1e-9)
            .saturating_sub(1)
    }

    fn at(&self, station: f64) -> f64 {
        self.pieces()[self.governing(station)].evaluate(station)
    }

    /// The knots strictly inside `(from, to)`, ascending.
    fn knots_within(&self, from: f64, to: f64) -> impl Iterator<Item = f64> + '_ {
        let pieces = self.pieces();
        let lo = pieces.partition_point(|piece| piece.station <= from + 1e-9);
        let hi = pieces.partition_point(|piece| piece.station < to - 1e-9);
        pieces[lo..hi.max(lo)].iter().map(|piece| piece.station)
    }

    /// Whether one straight piece governs the whole of `(from, to)`.
    fn is_straight_over(&self, from: f64, to: f64) -> bool {
        self.pieces()[self.governing(from)].is_straight()
            && self.knots_within(from, to).next().is_none()
    }

    /// Whether the elevation over `(from, to)` is one straight line, whether or not
    /// it is written as one piece: every piece in the span is straight, and every
    /// knot inside it lies on the line through the ends.
    fn is_linear_over(&self, from: f64, to: f64) -> bool {
        let cuts = self.cuts(from, to);
        cuts.windows(2)
            .all(|pair| self.is_straight_over(pair[0], pair[1]))
            && cuts[1..cuts.len() - 1]
                .iter()
                .all(|knot| self.collinear(from, *knot, to))
    }

    /// `from`, the knots inside, and `to`.
    fn cuts(&self, from: f64, to: f64) -> Vec<f64> {
        let mut cuts = vec![from];
        cuts.extend(self.knots_within(from, to));
        cuts.push(to);
        cuts
    }

    /// Whether the height at `middle` lies on the straight line from `from` to `to`.
    fn collinear(&self, from: f64, middle: f64, to: f64) -> bool {
        let (z0, z1, z2) = (self.at(from), self.at(middle), self.at(to));
        let expected = z0 + (z2 - z0) * (middle - from) / (to - from);
        (z1 - expected).abs() < HEIGHT_TOLERANCE
    }
}

/// One `<geometry>` entry, in its own terms.
struct Piece<'a> {
    entry: &'a Geometry,
    start: Point3,
    station: f64,
    heading: f64,
    length: f64,
}

impl<'a> Piece<'a> {
    fn new(entry: &'a Geometry, start: Point3) -> Self {
        Piece {
            entry,
            start,
            station: entry.s.get::<meter>(),
            heading: entry.hdg.get::<radian>(),
            length: entry.length.get::<meter>(),
        }
    }

    fn end_station(&self) -> f64 {
        self.station + self.length
    }

    /// The IR pieces this entry becomes: one, or several where the elevation
    /// profile bends inside it.
    fn build(
        &self,
        elevation: &Elevation,
        config: SamplingConfig,
        road: &str,
        approximations: &mut Approximations,
    ) -> Result<Vec<Curve3>, ImportError> {
        match &self.entry.r#type {
            GeometryType::Line(_) | GeometryType::Arc(_) | GeometryType::Spiral(_) => {
                let spans = self.grade_spans(elevation, config, road, approximations);
                let mut pieces: Vec<Curve3> = Vec::with_capacity(spans.len());
                for (from, to) in spans {
                    // Each sub-piece starts where the last one ended, so a spiral
                    // is walked once from end to end rather than from its start
                    // for every cut.
                    let start = pieces.last().map(Curve3::end_point).unwrap_or(self.start);
                    pieces.push(self.analytic(from, to, start, elevation)?);
                }
                Ok(pieces)
            }
            GeometryType::ParamPoly3(cubic) => {
                if !elevation.is_linear_over(self.station, self.end_station()) {
                    approximations.count(
                        "a cubic reference-line piece cannot be cut, so the elevation along \
                         {n} `<paramPoly3>` pieces is read as a straight grade between their \
                         ends",
                    );
                }
                let end_z = elevation.at(self.end_station());
                let (sin, cos) = self.heading.sin_cos();
                // The polynomial's parameter runs to 1 or to the piece's length;
                // scaling the coefficients makes it run to 1 either way.
                let scale = match cubic.p_range {
                    ParamPoly3pRange::Normalized => 1.0,
                    ParamPoly3pRange::ArcLength => self.length,
                };
                let scaled = |b: f64, c: f64, d: f64| {
                    (b * scale, c * scale * scale, d * scale * scale * scale)
                };
                let (bu, cu, du) = scaled(cubic.b_u, cubic.c_u, cubic.d_u);
                let (bv, cv, dv) = scaled(cubic.b_v, cubic.c_v, cubic.d_v);
                // Power basis to Bernstein: P₀ = a, P₁ = a + b/3, P₂ = a + 2b/3 + c/3,
                // P₃ = a + b + c + d — the exporter's expansion run backwards.
                let local = [
                    (cubic.a_u, cubic.a_v),
                    (cubic.a_u + bu / 3.0, cubic.a_v + bv / 3.0),
                    (
                        cubic.a_u + 2.0 * bu / 3.0 + cu / 3.0,
                        cubic.a_v + 2.0 * bv / 3.0 + cv / 3.0,
                    ),
                    (cubic.a_u + bu + cu + du, cubic.a_v + bv + cv + dv),
                ];
                let rise = end_z - self.start.z;
                let control: Vec<Point3> = local
                    .iter()
                    .enumerate()
                    .map(|(index, (u, v))| {
                        Point3::new(
                            self.start.x + u * cos - v * sin,
                            self.start.y + u * sin + v * cos,
                            self.start.z + rise * index as f64 / 3.0,
                        )
                    })
                    .collect();
                Ok(vec![Curve3::Bezier(Bezier3::new(
                    [control[0], control[1], control[2], control[3]],
                    config,
                )?)])
            }
            GeometryType::Poly3(poly) => {
                approximations.count(
                    "the IR has no piece for a `<poly3>`, so {n} of them are walked into \
                     polylines",
                );
                // v as a cubic in u, u along the heading: stepped in u so that each
                // step is about one sampling length of arc, until the arc length
                // the piece declares is used up.
                let (sin, cos) = self.heading.sin_cos();
                let mut points = vec![self.start];
                let (mut u, mut walked) = (0.0, 0.0);
                while walked < self.length - 1e-9 {
                    let slope = poly.b + 2.0 * poly.c * u + 3.0 * poly.d * u * u;
                    let du = (config.max_segment_length / (1.0 + slope * slope).sqrt())
                        .min(self.length - walked)
                        .max(1e-3);
                    let next_u = u + du;
                    let (v0, v1) = (poly.v(u), poly.v(next_u));
                    walked += (du * du + (v1 - v0) * (v1 - v0)).sqrt();
                    u = next_u;
                    points.push(Point3::new(
                        self.start.x + u * cos - v1 * sin,
                        self.start.y + u * sin + v1 * cos,
                        elevation.at(self.station + walked.min(self.length)),
                    ));
                }
                Ok(vec![Curve3::Polyline(Polyline3::new(points)?)])
            }
        }
    }

    /// The spans this piece is cut into, each with a straight grade.
    ///
    /// Cut at every elevation knot inside the piece, then joined back where two
    /// neighbouring spans lie on one straight line; a span whose elevation piece is
    /// a real cubic is walked at the sampling length instead.
    fn grade_spans(
        &self,
        elevation: &Elevation,
        config: SamplingConfig,
        road: &str,
        approximations: &mut Approximations,
    ) -> Vec<(f64, f64)> {
        let cuts = elevation.cuts(self.station, self.end_station());
        let mut spans: Vec<(f64, f64)> = Vec::new();
        for pair in cuts.windows(2) {
            let (a, b) = (pair[0], pair[1]);
            if elevation.is_straight_over(a, b) {
                spans.push((a, b));
            } else {
                approximations.count(format!(
                    "the IR climbs at a constant grade along each piece, so the curved \
                     elevation on road {road} is read as straight grades every {} m, over \
                     {{n}} stretches of it",
                    config.max_segment_length
                ));
                let stations: Vec<f64> = config.stations_between(a, b).collect();
                spans.extend(stations.windows(2).map(|pair| (pair[0], pair[1])));
            }
        }

        // Join spans that continue one another's grade: the elevation at the joint
        // is on the line through the neighbours' ends.
        let mut joined: Vec<(f64, f64)> = Vec::new();
        for span in spans {
            match joined.last_mut() {
                Some(last) if elevation.collinear(last.0, last.1, span.1) => last.1 = span.1,
                _ => joined.push(span),
            }
        }
        joined
    }

    /// The analytic sub-piece over `(from, to)`, starting at `start` — where the
    /// previous sub-piece ended — with its heading and curvature taken from the
    /// entry's own closed forms at that station.
    fn analytic(
        &self,
        from: f64,
        to: f64,
        start: Point3,
        elevation: &Elevation,
    ) -> Result<Curve3, ImportError> {
        let (offset, length) = (from - self.station, to - from);
        let start = Point3::new(start.x, start.y, elevation.at(from));
        let end_z = elevation.at(to);
        Ok(match &self.entry.r#type {
            GeometryType::Line(_) => {
                let (sin, cos) = self.heading.sin_cos();
                Curve3::Line(Line3::new(
                    start,
                    Point3::new(start.x + length * cos, start.y + length * sin, end_z),
                )?)
            }
            GeometryType::Arc(arc) => {
                let curvature = arc.curvature.get::<radian_per_meter>();
                Curve3::Arc(Arc3::new(
                    start,
                    self.heading + curvature * offset,
                    curvature,
                    length,
                    end_z,
                )?)
            }
            GeometryType::Spiral(spiral) => {
                let (k0, k1) = (
                    spiral.curvature_start.get::<radian_per_meter>(),
                    spiral.curvature_end.get::<radian_per_meter>(),
                );
                let sharpness = (k1 - k0) / self.length;
                // The heading of a clothoid is the integral of its curvature.
                let heading = self.heading + k0 * offset + sharpness * offset * offset / 2.0;
                Curve3::Clothoid(Clothoid3::new(
                    start,
                    heading,
                    k0 + sharpness * offset,
                    k0 + sharpness * (offset + length),
                    length,
                    end_z,
                )?)
            }
            _ => unreachable!("only the analytic kinds are cut"),
        })
    }
}
