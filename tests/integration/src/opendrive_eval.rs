//! An independent reader of the OpenDRIVE geometry this project writes.
//!
//! Checking that the two exports agree needs a way to ask the OpenDRIVE document
//! where a lane actually is, and asking `roadgen-opendrive` would only prove it
//! agrees with itself. So this walks `<planView>`, `<elevationProfile>`,
//! `<laneOffset>` and the lane widths the way the specification says to, using only
//! the parsed document.

use opendrive::core::OpenDrive;
use opendrive::lane::lane_choice::LaneChoice;
use opendrive::road::geometry::geometry_type::GeometryType;
use opendrive::road::Road as OdRoad;

/// A position in the document's coordinates.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Position {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Position {
    pub fn distance_to(self, other: Position) -> f64 {
        ((self.x - other.x).powi(2) + (self.y - other.y).powi(2) + (self.z - other.z).powi(2))
            .sqrt()
    }
}

/// Evaluates one `<road>` of a parsed document.
pub struct RoadEvaluator<'a> {
    road: &'a OdRoad,
}

impl<'a> RoadEvaluator<'a> {
    pub fn new(road: &'a OdRoad) -> Self {
        RoadEvaluator { road }
    }

    /// Finds the road with the given id.
    pub fn find(document: &'a OpenDrive, id: &str) -> Option<Self> {
        document
            .road
            .iter()
            .find(|road| road.id == id)
            .map(RoadEvaluator::new)
    }

    /// Integration step used for a `<spiral>`, metres. Independent of whatever the
    /// exporter used: this walks the element from its own attributes.
    const SPIRAL_STEP: f64 = 0.002;

    /// The reference line's position and heading at plan-view station `s`.
    fn reference_at(&self, s: f64) -> (f64, f64, f64) {
        let geometry = self
            .road
            .plan_view
            .geometry
            .iter()
            .rfind(|geometry| geometry.s.value <= s + 1e-9)
            .unwrap_or_else(|| self.road.plan_view.geometry.first());
        let ds = s - geometry.s.value;
        let hdg = geometry.hdg.value;
        match &geometry.r#type {
            GeometryType::Arc(arc) => {
                let curvature = arc.curvature.value;
                let heading = hdg + curvature * ds;
                if curvature.abs() < 1e-12 {
                    (
                        geometry.x.value + ds * hdg.cos(),
                        geometry.y.value + ds * hdg.sin(),
                        heading,
                    )
                } else {
                    (
                        geometry.x.value + (heading.sin() - hdg.sin()) / curvature,
                        geometry.y.value - (heading.cos() - hdg.cos()) / curvature,
                        heading,
                    )
                }
            }
            GeometryType::Spiral(spiral) => {
                // A spiral's curvature runs linearly from curvStart to curvEnd over
                // the element, so its heading is a quadratic and its position is the
                // integral of that heading's cosine and sine — a Fresnel integral,
                // walked here with the midpoint rule at a fine step.
                let length = geometry.length.value;
                let (start, end) = (spiral.curvature_start.value, spiral.curvature_end.value);
                let sharpness = if length > 0.0 {
                    (end - start) / length
                } else {
                    0.0
                };
                let heading_at = |u: f64| hdg + start * u + 0.5 * sharpness * u * u;
                let steps = ((ds / Self::SPIRAL_STEP).ceil() as usize).max(1);
                let step = ds / steps as f64;
                let (mut x, mut y) = (geometry.x.value, geometry.y.value);
                for index in 0..steps {
                    let u = step * (index as f64 + 0.5);
                    let heading = heading_at(u);
                    x += step * heading.cos();
                    y += step * heading.sin();
                }
                (x, y, heading_at(ds))
            }
            GeometryType::ParamPoly3(poly) => {
                // A parametric cubic in the frame of its start. Its parameter is not
                // arc length, so the `p` at which the curve is `ds` metres along is
                // found by solving for it: Newton's method on the arc-length
                // integral, which is smooth and monotone. Written the way the
                // specification says and not the way the exporter wrote it, so that
                // this reads the file rather than the exporter's intent.
                let normalized = matches!(
                    poly.p_range,
                    opendrive::road::geometry::param_poly_3_p_range::ParamPoly3pRange::Normalized
                );
                let range = if normalized {
                    1.0
                } else {
                    geometry.length.value
                };
                let du = |p: f64| poly.b_u + 2.0 * poly.c_u * p + 3.0 * poly.d_u * p * p;
                let dv = |p: f64| poly.b_v + 2.0 * poly.c_v * p + 3.0 * poly.d_v * p * p;
                let speed = |p: f64| du(p).hypot(dv(p));
                let arc_length = |to: f64| gauss_legendre(speed, 0.0, to, 64);
                let mut p = range * ds / geometry.length.value.max(f64::EPSILON);
                for _ in 0..50 {
                    let residual = arc_length(p) - ds;
                    let slope = speed(p);
                    if slope <= 0.0 || residual.abs() < 1e-12 {
                        break;
                    }
                    p = (p - residual / slope).clamp(0.0, range);
                }
                let (u, v) = (poly.u(p), poly.v(p));
                let local_heading = dv(p).atan2(du(p));
                (
                    geometry.x.value + u * hdg.cos() - v * hdg.sin(),
                    geometry.y.value + u * hdg.sin() + v * hdg.cos(),
                    hdg + local_heading,
                )
            }
            // Everything else this project writes is a straight segment.
            _ => (
                geometry.x.value + ds * hdg.cos(),
                geometry.y.value + ds * hdg.sin(),
                hdg,
            ),
        }
    }

    fn elevation_at(&self, s: f64) -> f64 {
        let Some(profile) = &self.road.elevation_profile else {
            return 0.0;
        };
        let Some(entry) = profile
            .elevation
            .iter()
            .rfind(|entry| entry.s <= s + 1e-9)
            .or_else(|| profile.elevation.first())
        else {
            return 0.0;
        };
        let ds = s - entry.s;
        entry.a + entry.b * ds + entry.c * ds * ds + entry.d * ds * ds * ds
    }

    /// The road's roll at station `s`, from `<lateralProfile><superelevation>`.
    fn superelevation_at(&self, s: f64) -> f64 {
        let Some(profile) = &self.road.lateral_profile else {
            return 0.0;
        };
        let Some(entry) = profile
            .super_elevation
            .iter()
            .rfind(|entry| entry.s <= s + 1e-9)
        else {
            return 0.0;
        };
        let ds = s - entry.s;
        entry.a + entry.b * ds + entry.c * ds * ds + entry.d * ds * ds * ds
    }

    fn lane_offset_at(&self, s: f64) -> f64 {
        let Some(entry) = self
            .road
            .lanes
            .lane_offset
            .iter()
            .rfind(|entry| entry.s <= s + 1e-9)
        else {
            return 0.0;
        };
        let ds = s - entry.s;
        entry.a + entry.b * ds + entry.c * ds * ds + entry.d * ds * ds * ds
    }

    /// The `<laneSection>` governing station `s`, and the station it starts at.
    ///
    /// A section governs from its own `s` up to but *not including* the next one's:
    /// the lanes either side of a boundary are different lanes, so asking a hair
    /// before it has to give the earlier section.
    fn section_at(&self, s: f64) -> (&opendrive::lane::lane_section::LaneSection, f64) {
        let section = self
            .road
            .lanes
            .lane_section
            .iter()
            .rfind(|section| section.s <= s)
            .unwrap_or_else(|| self.road.lanes.lane_section.first());
        (section, section.s)
    }

    /// Widths of the lanes on one side at station `s`, ordered outwards from the
    /// centre.
    ///
    /// A `<width>` is a cubic in the distance from the start of its lane section, so
    /// the polynomial is evaluated there rather than read as a constant.
    fn widths(&self, left: bool, s: f64) -> Vec<(i64, f64)> {
        let (section, section_start) = self.section_at(s);
        let width_of = |lane: &opendrive::lane::Lane| {
            let ds = s - section_start;
            lane.choice
                .iter()
                .filter_map(|choice| match choice {
                    LaneChoice::Width(width) => Some(width),
                    LaneChoice::Border(_) => None,
                })
                .rfind(|width| width.s_offset.value <= ds + 1e-9)
                .or_else(|| {
                    lane.choice.iter().find_map(|choice| match choice {
                        LaneChoice::Width(width) => Some(width),
                        LaneChoice::Border(_) => None,
                    })
                })
                .map(|width| {
                    let local = (ds - width.s_offset.value).max(0.0);
                    width.a
                        + width.b * local
                        + width.c * local * local
                        + width.d * local * local * local
                })
                .unwrap_or(0.0)
        };
        let mut widths: Vec<(i64, f64)> = if left {
            section
                .left
                .as_ref()
                .map(|side| {
                    side.lane
                        .iter()
                        .map(|lane| (lane.id, width_of(&lane.base)))
                        .collect()
                })
                .unwrap_or_default()
        } else {
            section
                .right
                .as_ref()
                .map(|side| {
                    side.lane
                        .iter()
                        .map(|lane| (lane.id, width_of(&lane.base)))
                        .collect()
                })
                .unwrap_or_default()
        };
        widths.sort_by_key(|(id, _)| id.abs());
        widths
    }

    /// The lateral offset of the middle of lane `id` at station `s`.
    pub fn lane_center_offset(&self, id: i64, s: f64) -> Option<f64> {
        let mut offset = self.lane_offset_at(s);
        for (lane_id, width) in self.widths(id > 0, s) {
            let half = width / 2.0;
            offset += if id > 0 { half } else { -half };
            if lane_id == id {
                return Some(offset);
            }
            offset += if id > 0 { half } else { -half };
        }
        None
    }

    /// The lateral offsets of the two edges of lane `id` at station `s`: the one
    /// nearer the reference line first.
    pub fn lane_edge_offsets(&self, id: i64, s: f64) -> Option<(f64, f64)> {
        let mut offset = self.lane_offset_at(s);
        for (lane_id, width) in self.widths(id > 0, s) {
            let inner = offset;
            offset += if id > 0 { width } else { -width };
            if lane_id == id {
                return Some((inner, offset));
            }
        }
        None
    }

    /// The centre of lane `id` at station `s`.
    pub fn lane_center(&self, id: i64, s: f64) -> Option<Position> {
        Some(self.position_at(s, self.lane_center_offset(id, s)?))
    }

    /// The two edges of lane `id` at station `s`, the one nearer the reference line
    /// first.
    pub fn lane_edges(&self, id: i64, s: f64) -> Option<(Position, Position)> {
        let (inner, outer) = self.lane_edge_offsets(id, s)?;
        Some((self.position_at(s, inner), self.position_at(s, outer)))
    }

    /// The point `t` metres to the left of the reference line at station `s`.
    pub fn position_at(&self, s: f64, t: f64) -> Position {
        let (x, y, heading) = self.reference_at(s);
        // Superelevation rolls the cross-section about the s-axis, so an offset `t`
        // to the left rises by t·sin(roll) and reaches only t·cos(roll) across.
        let roll = self.superelevation_at(s);
        let (across, rise) = (t * roll.cos(), t * roll.sin());
        Position {
            // `t` is positive to the left of the reference line.
            x: x - across * heading.sin(),
            y: y + across * heading.cos(),
            z: self.elevation_at(s) + rise,
        }
    }

    pub fn length(&self) -> f64 {
        self.road.length.value
    }

    /// How many `<laneSection>` elements the road has.
    pub fn section_count(&self) -> usize {
        self.road.lanes.lane_section.len()
    }

    /// The station each lane section starts at.
    pub fn section_stations(&self) -> Vec<f64> {
        self.road
            .lanes
            .lane_section
            .iter()
            .map(|section| section.s)
            .collect()
    }
}

/// ∫ f over [from, to], by composite eight-point Gauss–Legendre quadrature in
/// `intervals` pieces.
fn gauss_legendre(f: impl Fn(f64) -> f64, from: f64, to: f64, intervals: usize) -> f64 {
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
    let width = (to - from) / intervals as f64;
    let mut total = 0.0;
    for interval in 0..intervals {
        let a = from + interval as f64 * width;
        let (half, middle) = (width / 2.0, a + width / 2.0);
        for (node, weight) in NODES.iter().zip(WEIGHTS) {
            total += weight * half * (f(middle - half * node) + f(middle + half * node));
        }
    }
    total
}
