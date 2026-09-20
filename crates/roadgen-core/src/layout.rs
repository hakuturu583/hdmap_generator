//! Laying a cross-section out against a reference line.
//!
//! A road's lanes are not drawn; they are *derived*. The reference line is sampled,
//! at each sample the road frame is banked by the superelevation there, and every
//! cross-section edge is placed along that frame's lateral axis at the offset the
//! stacked lane widths put it at. That is the one computation every lane boundary
//! and centreline in the IR comes from, whichever way the map arrived: the builder
//! does it for a road it generated, and a reader does it for a road whose reference
//! line and widths came out of a file. Keeping it here, in one place, is what makes
//! the two agree to the last vertex.
//!
//! What this module does *not* do is anything that needs more than one road: a
//! joint is mitred by adjusting a [`RoadGeometry`]'s laterals from outside, and a
//! junction connector adopts the laterals of the lanes it joins the same way.

use crate::builder::LaneSpec;
use crate::error::GeometryError;
use crate::geometry::{
    Curve3, Poly3Profile, Polyline3, Sample, SamplingConfig, Vector3, WidthProfile,
};
use crate::id::{LaneId, RoadId};
use crate::map::{Lane, TrafficHandedness};
use crate::topology::{LateralSide, RoadEnd};
use crate::units::SpeedLimit;

/// Where each edge of one cross-section sits, as a function of station.
///
/// A cross-section is laid out by stacking lane widths outwards from the origin, and
/// with width profiles those widths vary along the road — so an edge's offset is a
/// function rather than a number.
#[derive(Debug, Clone)]
pub struct SectionLayout {
    pub station_range: (f64, f64),
    pub lane_offset: Poly3Profile,
    /// Where each lane landed, in the order the lanes were given: its side and its
    /// rank outwards on that side, counting from 1. Decided once, here, so that the
    /// lanes built against the layout sit exactly where its widths were stacked.
    pub slots: Vec<(LateralSide, usize)>,
    /// Widths of the left-hand lanes, ordinal 1 first.
    pub left: Vec<WidthProfile>,
    /// Widths of the right-hand lanes, ordinal 1 first.
    pub right: Vec<WidthProfile>,
}

impl SectionLayout {
    /// Works out which side and rank each lane of one cross-section sits at.
    ///
    /// A lane goes to the side it names, or to the side the map's handedness puts
    /// a lane of its direction on; within a side the lanes stack outwards in the
    /// order given.
    pub fn new(
        lanes: &[LaneSpec],
        station_range: (f64, f64),
        lane_offset: Poly3Profile,
        handedness: TrafficHandedness,
    ) -> SectionLayout {
        let mut layout = SectionLayout {
            station_range,
            lane_offset,
            slots: Vec::with_capacity(lanes.len()),
            left: Vec::new(),
            right: Vec::new(),
        };
        for lane in lanes {
            let side = side_of(lane, handedness);
            let stack = match side {
                LateralSide::Left => &mut layout.left,
                LateralSide::Right => &mut layout.right,
            };
            stack.push(lane.width.clone());
            layout.slots.push((side, stack.len()));
        }
        layout
    }

    /// The lateral offset of the edge at `rank` — `0` the cross-section origin, `+n`
    /// the n-th edge to its left, `-n` the n-th to its right — at `station`.
    pub fn edge_offset(&self, rank: i32, station: f64) -> f64 {
        let (widths, sign) = if rank >= 0 {
            (&self.left, 1.0)
        } else {
            (&self.right, -1.0)
        };
        let stacked: f64 = widths
            .iter()
            .take(rank.unsigned_abs() as usize)
            .map(|width| width.evaluate(station).metres())
            .sum();
        self.lane_offset.evaluate(station) + sign * stacked
    }

    /// How far the cross-section reaches from its reference line at `station`.
    pub fn extent(&self, station: f64) -> f64 {
        let outermost = |rank: i32| self.edge_offset(rank, station).abs();
        outermost(self.left.len() as i32).max(outermost(-(self.right.len() as i32)))
    }
}

/// The side a lane sits on: the one it asks for, else the one its direction lands
/// on under the map's handedness.
pub fn side_of(lane: &LaneSpec, handedness: TrafficHandedness) -> LateralSide {
    lane.side
        .unwrap_or_else(|| handedness.side_for(lane.direction))
}

/// The geometry of one road's reference line, plus the lateral direction to use at
/// each sampled station.
#[derive(Debug, Clone)]
pub struct RoadGeometry {
    pub samples: Vec<Sample>,
    /// The horizontal lateral direction at each station, mitred where the road meets
    /// another. Superelevation is *not* baked in here: a joint has to be mitred in
    /// plan, and the roll is applied afterwards, per station.
    pub laterals: Vec<Vector3>,
    /// Superelevation at each station, radians.
    pub rolls: Vec<f64>,
}

impl RoadGeometry {
    /// Samples the reference line, at its own stations and at every one of
    /// `required_stations` — a cross-section has to change at the station the caller
    /// asked for, so those stations get a vertex whether or not the sampler would
    /// have put one there.
    pub fn new(
        reference_line: &Curve3,
        superelevation: &Poly3Profile,
        config: SamplingConfig,
        required_stations: &[f64],
    ) -> Result<Self, GeometryError> {
        let samples = reference_line.samples_including(config, required_stations)?;
        let laterals = samples
            .iter()
            .map(|sample| Ok(sample.frame()?.left.get()))
            .collect::<Result<Vec<_>, GeometryError>>()?;
        let rolls = samples
            .iter()
            .map(|sample| superelevation.evaluate(sample.station))
            .collect();
        Ok(RoadGeometry {
            samples,
            laterals,
            rolls,
        })
    }

    /// The lateral direction to offset along at one station, once the road's
    /// superelevation has tilted it.
    ///
    /// The roll turns the lateral about the tangent, so an offset along it gains
    /// height — which is what banking is. A mitred lateral is longer than a unit
    /// vector, and the rotation preserves that.
    pub fn banked_lateral(&self, index: usize) -> Vector3 {
        let roll = self.rolls[index];
        if roll == 0.0 {
            return self.laterals[index];
        }
        self.laterals[index].rotated_about(self.samples[index].tangent, roll)
    }

    /// The curve that runs `offset(station)` metres to the left of the reference
    /// line, over the stations within `range`.
    ///
    /// The offset is a function because a tapering lane's boundary is not a constant
    /// distance from the reference line; the range is there because a cross-section
    /// covers only part of its road.
    pub fn boundary_over(
        &self,
        range: (f64, f64),
        offset: impl Fn(f64) -> f64,
    ) -> Result<Curve3, GeometryError> {
        let tolerance = Polyline3::MIN_SEGMENT;
        Ok(Curve3::Polyline(Polyline3::new(
            self.samples
                .iter()
                .enumerate()
                .filter(|(_, sample)| {
                    sample.station >= range.0 - tolerance && sample.station <= range.1 + tolerance
                })
                .map(|(index, sample)| {
                    sample.point + self.banked_lateral(index) * offset(sample.station)
                }),
        )?))
    }

    /// The lateral direction in plan at one end, before any roll. This is what a
    /// joint is mitred in: two roads have to agree on a direction across the ground
    /// whatever each of them is banked to.
    pub fn lateral_at(&self, end: RoadEnd) -> Vector3 {
        self.laterals[self.index_at(end)]
    }

    /// The lateral direction at one end with the road's roll applied — the direction
    /// a boundary is actually offset along there.
    pub fn banked_lateral_at(&self, end: RoadEnd) -> Vector3 {
        self.banked_lateral(self.index_at(end))
    }

    pub fn index_at(&self, end: RoadEnd) -> usize {
        match end {
            RoadEnd::Start => 0,
            RoadEnd::End => self.samples.len() - 1,
        }
    }
}

/// Generates the lanes of one cross-section against the road's geometry.
///
/// `index_offset` is where this section's lanes begin in the road's lane list, so
/// that a lane's identifier is its position as the caller wrote it. `road_speed` is
/// the limit a lane falls back to when it states none of its own.
pub fn section_lanes(
    road: &RoadId,
    geometry: &RoadGeometry,
    lanes: &[LaneSpec],
    section: usize,
    index_offset: usize,
    layout: &SectionLayout,
    road_speed: Option<SpeedLimit>,
) -> Result<Vec<Lane>, GeometryError> {
    let mut built = Vec::with_capacity(lanes.len());

    for (position, lane_spec) in lanes.iter().enumerate() {
        let (side, ordinal) = layout.slots[position];
        // A lane spans one slot of the cross-section: the edges either side of
        // it, counted outwards from the origin.
        let (left_edge, right_edge) = match side {
            LateralSide::Left => (ordinal as i32, ordinal as i32 - 1),
            LateralSide::Right => (1 - ordinal as i32, -(ordinal as i32)),
        };
        let index = index_offset + position;
        built.push(Lane {
            id: LaneId::of_road(road, index),
            road: road.clone(),
            index,
            side,
            ordinal,
            direction: lane_spec.direction,
            lane_type: lane_spec.lane_type,
            width: lane_spec.width.clone(),
            speed_limit: lane_spec.speed_limit.or(road_speed),
            section,
            station_range: layout.station_range,
            left_edge,
            right_edge,
            left_offset: layout.edge_offset(left_edge, layout.station_range.0),
            right_offset: layout.edge_offset(right_edge, layout.station_range.0),
            left_boundary: geometry
                .boundary_over(layout.station_range, |s| layout.edge_offset(left_edge, s))?,
            right_boundary: geometry
                .boundary_over(layout.station_range, |s| layout.edge_offset(right_edge, s))?,
            centerline: geometry.boundary_over(layout.station_range, |s| {
                (layout.edge_offset(left_edge, s) + layout.edge_offset(right_edge, s)) / 2.0
            })?,
            left_marking: lane_spec.left_marking,
            right_marking: lane_spec.right_marking,
        });
    }
    Ok(built)
}
