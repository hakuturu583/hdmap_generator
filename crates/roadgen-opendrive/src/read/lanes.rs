//! The cross-sections, from `<lanes>`.
//!
//! A `<laneSection>` is a cross-section and a `<lane>` is a lane: the same two
//! things the IR has, so most of this is a rename. The work is in the width. An
//! OpenDRIVE `<width>` is a free cubic and the IR's [`WidthProfile`] is a taper
//! between positive knots, which is a narrower thing on purpose — see the width
//! module's own account of why. So a width is read as the taper it is when it is
//! one, and sampled into a straight one when it is not, and a width that reaches
//! zero is held at a floor rather than allowed to cross it. Each of those is
//! reported.
//!
//! Which way a lane runs is not in a 1.7 document at all: it follows from the side
//! the lane is on and the rule of the road, which is exactly how the IR decides the
//! side from the direction, run backwards.

use opendrive::lane::lane_choice::LaneChoice;
use opendrive::lane::lane_section::LaneSection;
use opendrive::lane::lane_type::LaneType as OdLaneType;
use opendrive::lane::road_mark::color::Color;
use opendrive::lane::road_mark::type_simplified::TypeSimplified;
use opendrive::lane::road_mark::RoadMark;
use opendrive::lane::Lane as OdLane;
use opendrive::road::unit::SpeedUnit;
use opendrive::road::Road as OdRoad;
use uom::si::length::meter;

use roadgen_core::builder::LaneSpec;
use roadgen_core::geometry::{Poly3Profile, SamplingConfig, Taper, WidthProfile};
use roadgen_core::id::RoadId;
use roadgen_core::map::TrafficHandedness;
use roadgen_core::semantics::{BoundaryMarking, LaneType, MarkingColor, RoadMarking};
use roadgen_core::topology::{Direction, LateralSide};
use roadgen_core::units::{PositiveWidth, SpeedLimit};

use super::Approximations;
use crate::error::ImportError;

/// The narrowest a lane is allowed to be, metres.
///
/// A centimetre: a lane the document takes to nothing is held here instead, which
/// is below anything a consumer draws and above what validation calls zero.
const MIN_WIDTH: f64 = 0.01;

/// One lane of a section, as the layout takes it and as the document numbered it.
pub struct LaneEntry {
    pub opendrive_id: i64,
    pub spec: LaneSpec,
}

/// One `<laneSection>`.
pub struct Section {
    pub station: f64,
    /// Outwards from the reference line on each side, which is the order the
    /// layout stacks them in.
    pub lanes: Vec<LaneEntry>,
}

/// The road's sections, each with its lanes as specs.
pub fn sections(
    road_id: &RoadId,
    road: &OdRoad,
    length: f64,
    handedness: TrafficHandedness,
    config: SamplingConfig,
    approximations: &mut Approximations,
) -> Result<Vec<Section>, ImportError> {
    let lane_offset = super::profile(
        road.lanes
            .lane_offset
            .iter()
            .map(|offset| (offset.s, offset.a, offset.b, offset.c, offset.d)),
    )?;
    let sections: Vec<&LaneSection> = road.lanes.lane_section.iter().collect();
    let mut built = Vec::with_capacity(sections.len());

    for (index, section) in sections.iter().enumerate() {
        let end = sections.get(index + 1).map(|next| next.s).unwrap_or(length);
        if end - section.s <= 0.0 {
            return Err(ImportError::Inconsistent(format!(
                "road {}: the lane section at s={} has no length",
                road.id, section.s
            )));
        }
        let mut reader = SectionReader {
            road: road_id,
            section,
            range: (section.s, end),
            lane_offset: &lane_offset,
            handedness,
            config,
            approximations,
        };
        let mut lanes = Vec::new();
        // The lane the map's handedness puts forward traffic on comes first, then
        // the other side; each side outwards from the reference line.
        let forward_side = handedness.side_for(Direction::Forward);
        for side in [forward_side, forward_side.opposite()] {
            lanes.extend(reader.side(side)?);
        }
        built.push(Section {
            station: section.s,
            lanes,
        });
    }
    Ok(built)
}

/// Reads the lanes of one side of one section.
struct SectionReader<'a> {
    road: &'a RoadId,
    section: &'a LaneSection,
    range: (f64, f64),
    lane_offset: &'a Poly3Profile,
    handedness: TrafficHandedness,
    config: SamplingConfig,
    approximations: &'a mut Approximations,
}

impl SectionReader<'_> {
    /// The lanes of one side, innermost first.
    fn side(&mut self, side: LateralSide) -> Result<Vec<LaneEntry>, ImportError> {
        let mut lanes: Vec<(i64, &OdLane)> = match side {
            LateralSide::Left => self
                .section
                .left
                .iter()
                .flat_map(|left| left.lane.iter())
                .map(|lane| (lane.id, &lane.base))
                .collect(),
            LateralSide::Right => self
                .section
                .right
                .iter()
                .flat_map(|right| right.lane.iter())
                .map(|lane| (lane.id, &lane.base))
                .collect(),
        };
        // Outwards: ascending on the left, descending on the right.
        lanes.sort_by_key(|(id, _)| id.abs());
        for (expected, (id, _)) in lanes.iter().enumerate() {
            if id.unsigned_abs() as usize != expected + 1 {
                return Err(ImportError::Inconsistent(format!(
                    "road {}: the {} lanes of the section at s={} are not numbered 1, 2, … \
                     outwards",
                    self.road,
                    side.as_str(),
                    self.section.s
                )));
            }
        }

        let direction = if self.handedness.side_for(Direction::Forward) == side {
            Direction::Forward
        } else {
            Direction::Backward
        };
        // The marking on the reference-line side of the innermost lane is the
        // centre lane's; every other lane's inner marking is its neighbour's outer.
        let mut inner_marking = self
            .section
            .center
            .lane
            .first()
            .base
            .road_mark
            .first()
            .map(|mark| self.marking(mark))
            .unwrap_or(BoundaryMarking::new(RoadMarking::None, MarkingColor::White));
        let mut inner_widths: Vec<WidthProfile> = Vec::new();
        let mut entries = Vec::with_capacity(lanes.len());

        for (id, lane) in lanes {
            let width = self.width(id, lane, side, &inner_widths)?;
            let outer_marking = match lane.road_mark.first() {
                Some(mark) => {
                    if lane.road_mark.len() > 1 {
                        self.approximations.count(
                            "a lane boundary has one marking in the IR, so {n} lanes whose \
                             `<roadMark>` changes along them keep their first",
                        );
                    }
                    self.marking(mark)
                }
                None => BoundaryMarking::new(RoadMarking::None, MarkingColor::White),
            };
            let (left_marking, right_marking) = match side {
                LateralSide::Left => (outer_marking, inner_marking),
                LateralSide::Right => (inner_marking, outer_marking),
            };
            let mut spec = LaneSpec::new(width.clone(), direction)
                .with_type(self.lane_type(&lane.r#type))
                .with_side(side)
                .with_markings(left_marking, right_marking);
            if let Some(speed) = lane.speed.first() {
                match SpeedLimit::from_mps(to_mps(speed.max, speed.unit.as_ref())) {
                    Ok(limit) => spec = spec.with_speed_limit(limit),
                    Err(_) => self.approximations.count(
                        "a speed limit is a positive speed in the IR, so the `<speed>` of \
                         {n} lanes that state none is dropped",
                    ),
                }
            }
            entries.push(LaneEntry {
                opendrive_id: id,
                spec,
            });
            inner_marking = outer_marking;
            inner_widths.push(width);
        }
        Ok(entries)
    }

    /// The lane's width along the section, as a profile the IR can hold.
    fn width(
        &mut self,
        id: i64,
        lane: &OdLane,
        side: LateralSide,
        inner: &[WidthProfile],
    ) -> Result<WidthProfile, ImportError> {
        let (start, end) = self.range;
        let mut widths: Vec<Cubic> = Vec::new();
        let mut borders: Vec<Cubic> = Vec::new();
        for choice in &lane.choice {
            match choice {
                LaneChoice::Width(width) => widths.push(Cubic {
                    station: start + width.s_offset.get::<meter>(),
                    a: width.a,
                    b: width.b,
                    c: width.c,
                    d: width.d,
                }),
                LaneChoice::Border(border) => borders.push(Cubic {
                    station: start + border.s_offset.get::<meter>(),
                    a: border.a,
                    b: border.b,
                    c: border.c,
                    d: border.d,
                }),
            }
        }
        widths.sort_by(|left, right| left.station.total_cmp(&right.station));
        borders.sort_by(|left, right| left.station.total_cmp(&right.station));

        if widths.is_empty() && borders.is_empty() {
            return Err(ImportError::Inconsistent(format!(
                "road {}: lane {id} of the section at s={start} has neither a width nor a \
                 border",
                self.road
            )));
        }

        let knots = if !widths.is_empty() {
            if !borders.is_empty() {
                self.approximations.count(
                    "a lane states its edge once in the IR, so the `<border>` of {n} lanes \
                     that also state a `<width>` is ignored",
                );
            }
            self.width_knots(id, &widths, end)?
        } else {
            // A border is where the edge is, measured from the reference line; the
            // width is what is left after the lanes inside it.
            self.approximations.count(
                "the IR measures a lane by its width, so the `<border>` of {n} lanes is \
                 read as the distance beyond the lane inside it, sampled along the section",
            );
            let sign = side.sign();
            let inner_edge = |s: f64| {
                self.lane_offset.evaluate(s)
                    + sign
                        * inner
                            .iter()
                            .map(|width| width.evaluate(s).metres())
                            .sum::<f64>()
            };
            let mut knots = Vec::new();
            for (index, border) in borders.iter().enumerate() {
                let piece_end = borders
                    .get(index + 1)
                    .map(|next| next.station)
                    .unwrap_or(end);
                for s in stations_along(border.station, piece_end, self.config) {
                    let edge = border.evaluate(s);
                    knots.push((s, sign * (edge - inner_edge(s))));
                }
            }
            (knots, Taper::Linear)
        };
        self.profile(id, knots)
    }

    /// The knots of a lane's `<width>` pieces, and the taper that joins them.
    ///
    /// A piece that is straight is a knot at each end; one that is the IR's smooth
    /// ease is the same, under the smooth taper; anything else is sampled. A profile
    /// has one taper, so a lane that mixes the smooth ease with a straight run is
    /// sampled too.
    fn width_knots(
        &mut self,
        id: i64,
        widths: &[Cubic],
        end: f64,
    ) -> Result<(Vec<(f64, f64)>, Taper), ImportError> {
        let mut kinds = Vec::with_capacity(widths.len());
        for (index, piece) in widths.iter().enumerate() {
            let piece_end = widths
                .get(index + 1)
                .map(|next| next.station)
                .unwrap_or(end);
            kinds.push((piece, piece_end, piece.kind(piece_end - piece.station)));
        }
        let any_cubic = kinds.iter().any(|(_, _, kind)| *kind == Kind::Cubic);
        let any_smooth = kinds.iter().any(|(_, _, kind)| *kind == Kind::Smooth);
        let any_sloped = kinds.iter().any(|(_, _, kind)| *kind == Kind::Linear);

        let sample_everything = any_cubic || (any_smooth && any_sloped);
        if any_cubic {
            self.approximations.count(
                "a lane width is a taper between knots in the IR, so the free cubic \
                 `<width>` of {n} lanes is sampled into straight runs",
            );
        } else if any_smooth && any_sloped {
            self.approximations.count(
                "a lane has one kind of taper in the IR, so {n} lanes that mix a smooth \
                 ease with a straight run are sampled into straight runs",
            );
        }
        let _ = id;

        let mut knots = Vec::new();
        for (piece, piece_end, kind) in kinds {
            if sample_everything && kind != Kind::Constant {
                for s in stations_along(piece.station, piece_end, self.config) {
                    knots.push((s, piece.evaluate(s)));
                }
            } else {
                knots.push((piece.station, piece.a));
                knots.push((piece_end, piece.evaluate(piece_end)));
            }
        }
        let taper = if any_smooth && !sample_everything {
            Taper::Smooth
        } else {
            Taper::Linear
        };
        Ok((knots, taper))
    }

    /// A profile from knots: repeats dropped, a floor put under the width, and a
    /// constant width kept as the one knot it is.
    fn profile(
        &mut self,
        id: i64,
        (knots, taper): (Vec<(f64, f64)>, Taper),
    ) -> Result<WidthProfile, ImportError> {
        let mut cleaned: Vec<(f64, f64)> = Vec::with_capacity(knots.len());
        for (station, width) in knots {
            if let Some(&(last_station, last_width)) = cleaned.last() {
                if (station - last_station).abs() < 1e-9 && (width - last_width).abs() < 1e-9 {
                    continue;
                }
                if (station - last_station).abs() < 1e-9 {
                    self.approximations.count(
                        "a lane width is continuous in the IR, so at {n} places where a \
                         `<width>` steps the lane changes width over no distance",
                    );
                }
            }
            cleaned.push((station, width));
        }
        // A profile holds its last width to the end of the road, so a knot that
        // only repeats the one before it says nothing — and the last `<width>`
        // piece of a section, held to the section's end, always writes one.
        while cleaned.len() >= 2
            && (cleaned[cleaned.len() - 1].1 - cleaned[cleaned.len() - 2].1).abs() < 1e-9
        {
            cleaned.pop();
        }
        let mut floored = false;
        let positive: Vec<(f64, PositiveWidth)> = cleaned
            .into_iter()
            .map(|(station, width)| {
                let held = if width < MIN_WIDTH {
                    floored = true;
                    MIN_WIDTH
                } else {
                    width
                };
                (
                    station,
                    PositiveWidth::new(held).expect("held above the floor"),
                )
            })
            .collect();
        if floored {
            self.approximations.count(format!(
                "a lane width is greater than zero everywhere in the IR, so {{n}} lanes whose \
                 `<width>` reaches zero are held at {MIN_WIDTH} m there"
            ));
        }
        let _ = id;
        if positive.len() == 1 || positive.iter().all(|(_, w)| *w == positive[0].1) {
            return Ok(WidthProfile::constant(positive[0].1));
        }
        Ok(WidthProfile::new(positive, taper)?)
    }

    fn marking(&mut self, mark: &RoadMark) -> BoundaryMarking {
        let marking = match mark.type_simplified {
            TypeSimplified::None => RoadMarking::None,
            TypeSimplified::Solid => RoadMarking::Solid,
            TypeSimplified::Broken => RoadMarking::Broken,
            TypeSimplified::SolidSolid => RoadMarking::SolidSolid,
            TypeSimplified::SolidBroken => RoadMarking::SolidBroken,
            TypeSimplified::BrokenSolid => RoadMarking::BrokenSolid,
            TypeSimplified::Curb => RoadMarking::Curbstone,
            TypeSimplified::BrokenBroken | TypeSimplified::BottsDots => {
                self.approximations.count(format!(
                    "the IR has no {:?} marking, so {{n}} boundaries of that kind are read as \
                     broken",
                    mark.type_simplified
                ));
                RoadMarking::Broken
            }
            TypeSimplified::Grass => {
                self.approximations.count(
                    "the IR has no grass marking, so {n} boundaries of that kind are read as \
                     unpainted",
                );
                RoadMarking::None
            }
            TypeSimplified::Custom | TypeSimplified::Edge => {
                self.approximations.count(format!(
                    "the IR has no {:?} marking, so {{n}} boundaries of that kind are read as \
                     solid",
                    mark.type_simplified
                ));
                RoadMarking::Solid
            }
        };
        let color = match &mark.color {
            Color::Yellow | Color::Orange => MarkingColor::Yellow,
            Color::White | Color::Standard => MarkingColor::White,
            other => {
                self.approximations.count(format!(
                    "the IR paints in white and yellow, so {{n}} {other:?} markings are read as \
                     white"
                ));
                MarkingColor::White
            }
        };
        BoundaryMarking::new(marking, color)
    }

    fn lane_type(&mut self, lane_type: &OdLaneType) -> LaneType {
        let (found, exact) = match lane_type {
            OdLaneType::Driving => (LaneType::Driving, true),
            OdLaneType::Shoulder => (LaneType::Shoulder, true),
            OdLaneType::Border => (LaneType::Border, true),
            OdLaneType::Sidewalk => (LaneType::Sidewalk, true),
            OdLaneType::Biking => (LaneType::Biking, true),
            OdLaneType::Parking => (LaneType::Parking, true),
            OdLaneType::Restricted => (LaneType::Restricted, true),
            OdLaneType::None => (LaneType::None, true),
            // Lanes traffic drives, under some restriction the IR does not record.
            OdLaneType::Entry
            | OdLaneType::Exit
            | OdLaneType::OnRamp
            | OdLaneType::OffRamp
            | OdLaneType::ConnectingRamp
            | OdLaneType::Bidirectional
            | OdLaneType::Bus
            | OdLaneType::Taxi
            | OdLaneType::HOV => (LaneType::Driving, false),
            // A hard shoulder by another name.
            OdLaneType::Stop => (LaneType::Shoulder, false),
            // Part of the road surface that carries nothing.
            OdLaneType::Curb => (LaneType::Border, false),
            OdLaneType::Median
            | OdLaneType::Special1
            | OdLaneType::Special2
            | OdLaneType::Special3
            | OdLaneType::RoadWorks
            | OdLaneType::Tram
            | OdLaneType::Rail => (LaneType::Restricted, false),
        };
        if !exact {
            self.approximations.count(format!(
                "the IR has eight lane types, so {{n}} lanes of type {lane_type:?} are read as {}",
                found.as_str()
            ));
        }
        found
    }
}

/// A cubic in `ds` from its own station, as every OpenDRIVE width is.
#[derive(Debug, Clone, Copy)]
struct Cubic {
    station: f64,
    a: f64,
    b: f64,
    c: f64,
    d: f64,
}

/// What shape a width piece is, which decides how it is read.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Kind {
    Constant,
    /// Straight, with a slope.
    Linear,
    /// The IR's smooth ease: `3Δ/L²·ds² − 2Δ/L³·ds³`, and nothing linear.
    Smooth,
    Cubic,
}

impl Cubic {
    fn evaluate(&self, station: f64) -> f64 {
        let ds = station - self.station;
        self.a + self.b * ds + self.c * ds * ds + self.d * ds * ds * ds
    }

    fn kind(&self, length: f64) -> Kind {
        let tiny = 1e-12;
        if self.c.abs() < tiny && self.d.abs() < tiny {
            return if self.b.abs() < tiny {
                Kind::Constant
            } else {
                Kind::Linear
            };
        }
        if self.b.abs() < tiny && length > 0.0 {
            // c = 3Δ/L² and d = −2Δ/L³ for the same Δ.
            let delta = self.c * length * length / 3.0;
            let expected_d = -2.0 * delta / (length * length * length);
            if (self.d - expected_d).abs() <= 1e-9 * (1.0 + expected_d.abs()) {
                return Kind::Smooth;
            }
        }
        Kind::Cubic
    }
}

/// Stations from `from` to `to`, both included, no further apart than the
/// sampling length.
fn stations_along(from: f64, to: f64, config: SamplingConfig) -> Vec<f64> {
    let steps = ((to - from) / config.max_segment_length).ceil().max(1.0) as usize;
    (0..=steps)
        .map(|step| from + (to - from) * step as f64 / steps as f64)
        .collect()
}

/// A speed in the document's unit, in metres per second. OpenDRIVE's default unit
/// for a speed is m/s.
pub fn to_mps(value: f64, unit: Option<&SpeedUnit>) -> f64 {
    match unit {
        Some(SpeedUnit::KilometersPerHour) => value / 3.6,
        Some(SpeedUnit::MilesPerHour) => value * 0.44704,
        Some(SpeedUnit::MetersPerSecond) | None => value,
    }
}
