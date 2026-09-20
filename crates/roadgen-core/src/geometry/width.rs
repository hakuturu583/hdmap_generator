//! How wide a lane is, along its length.
//!
//! A lane that tapers is ordinary — a slip road narrowing to nothing, a bay widening
//! out of a carriageway — so a width is a function of station rather than a number.
//! What must not be ordinary is a width that reaches zero or goes negative, and this
//! type cannot describe one:
//!
//! * every knot is a [`PositiveWidth`], which cannot hold a non-positive number;
//! * between two knots the width either runs straight or eases with a smooth step,
//!   and **both are monotone**, so the value between two positive knots stays between
//!   them.
//!
//! That is why there is no cubic with free coefficients here. An OpenDRIVE `<width>`
//! is a cubic in general and a cubic can dip below zero between its ends; the smooth
//! taper below is exactly the cubic that cannot.

use crate::error::{GeometryError, QuantityError};
use crate::geometry::profile::{Poly3Piece, Poly3Profile};
use crate::units::PositiveWidth;

/// How the width gets from one knot to the next.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Taper {
    /// Straight from one width to the other. The width changes at a constant rate,
    /// and the boundary has a corner at each knot.
    Linear,
    /// An ease in and out: `3t² − 2t³`, the cubic with zero slope at both ends.
    ///
    /// This is what a lane drop looks like when it is built rather than drawn, and it
    /// is monotone, so the width stays between the two knots it joins.
    Smooth,
}

/// A lane width as a function of the station along its road.
#[derive(Debug, Clone, PartialEq)]
pub struct WidthProfile {
    /// Ascending by station, never empty.
    knots: Vec<(f64, PositiveWidth)>,
    taper: Taper,
}

impl WidthProfile {
    /// The same width everywhere.
    pub fn constant(width: PositiveWidth) -> Self {
        WidthProfile {
            knots: vec![(0.0, width)],
            taper: Taper::Linear,
        }
    }

    /// Goes from `from` to `to` over `length` metres, starting at station `start`.
    pub fn tapered(
        start: f64,
        length: f64,
        from: PositiveWidth,
        to: PositiveWidth,
        taper: Taper,
    ) -> Result<Self, GeometryError> {
        if !(length.is_finite() && length > 0.0 && start.is_finite()) {
            return Err(GeometryError::NonFiniteCoordinate);
        }
        WidthProfile::new([(start, from), (start + length, to)], taper)
    }

    /// A width passing through each knot in turn, held flat before the first and
    /// after the last.
    ///
    /// The knots are kept canonical: one that only repeats its neighbours' width
    /// changes nothing — the profile is flat there with or without it — and is
    /// dropped, and knots that all agree are the one knot of a constant profile.
    /// So two profiles that describe the same width are the same profile, whichever
    /// way they were written.
    pub fn new(
        knots: impl IntoIterator<Item = (f64, PositiveWidth)>,
        taper: Taper,
    ) -> Result<Self, GeometryError> {
        let mut knots: Vec<(f64, PositiveWidth)> = knots.into_iter().collect();
        if knots.is_empty() {
            return Err(GeometryError::TooFewPoints { got: 0 });
        }
        if !knots.iter().all(|(station, _)| station.is_finite()) {
            return Err(GeometryError::NonFiniteCoordinate);
        }
        knots.sort_by(|left, right| {
            left.0
                .partial_cmp(&right.0)
                .expect("stations were checked to be finite")
        });
        let same = |a: PositiveWidth, b: PositiveWidth| (a.metres() - b.metres()).abs() < 1e-12;
        if knots.iter().all(|(_, width)| same(*width, knots[0].1)) {
            return Ok(WidthProfile::constant(knots[0].1));
        }
        // A knot is redundant when the profile is the same width on both sides of
        // it: before the first knot it holds the first width, after the last the
        // last, so an end knot goes when its one neighbour agrees with it.
        let kept: Vec<(f64, PositiveWidth)> = knots
            .iter()
            .enumerate()
            .filter(|(index, (_, width))| {
                let before = index
                    .checked_sub(1)
                    .is_none_or(|i| same(knots[i].1, *width));
                let after = knots
                    .get(index + 1)
                    .is_none_or(|(_, next)| same(*next, *width));
                !(before && after)
            })
            .map(|(_, knot)| *knot)
            .collect();
        Ok(WidthProfile { knots: kept, taper })
    }

    pub fn taper(&self) -> Taper {
        self.taper
    }

    pub fn knots(&self) -> &[(f64, PositiveWidth)] {
        &self.knots
    }

    /// Whether the width is the same everywhere, so a caller can take the short path.
    pub fn is_constant(&self) -> bool {
        let first = self.knots[0].1.metres();
        self.knots
            .iter()
            .all(|(_, width)| (width.metres() - first).abs() < 1e-12)
    }

    /// The width at `station`. Always greater than zero.
    pub fn evaluate(&self, station: f64) -> PositiveWidth {
        let metres = self.evaluate_metres(station);
        PositiveWidth::new(metres)
            .expect("every knot is positive and both tapers are monotone between knots")
    }

    fn evaluate_metres(&self, station: f64) -> f64 {
        let last = self.knots.len() - 1;
        if station <= self.knots[0].0 {
            return self.knots[0].1.metres();
        }
        if station >= self.knots[last].0 {
            return self.knots[last].1.metres();
        }
        let index = self
            .knots
            .iter()
            .rposition(|(knot, _)| *knot <= station)
            .unwrap_or(0)
            .min(last - 1);
        let (from_station, from_width) = self.knots[index];
        let (to_station, to_width) = self.knots[index + 1];
        let span = to_station - from_station;
        if span <= 0.0 {
            return to_width.metres();
        }
        let t = (station - from_station) / span;
        let blend = match self.taper {
            Taper::Linear => t,
            Taper::Smooth => t * t * (3.0 - 2.0 * t),
        };
        from_width.metres() + (to_width.metres() - from_width.metres()) * blend
    }

    /// The narrowest the lane ever gets.
    ///
    /// Both tapers are monotone, so the minimum is always at a knot — there is no
    /// dip between two of them to search for.
    pub fn narrowest(&self) -> PositiveWidth {
        self.knots
            .iter()
            .map(|(_, width)| *width)
            .reduce(|narrow, width| if width < narrow { width } else { narrow })
            .expect("a profile always has at least one knot")
    }

    /// The same width as a piecewise cubic, which is the form OpenDRIVE writes.
    ///
    /// Exact for both tapers: a straight run is a cubic with zero second and third
    /// coefficients, and the smooth ease is `3Δ/L²·ds² − 2Δ/L³·ds³`.
    pub fn to_poly3(&self, offset: f64) -> Poly3Profile {
        let mut pieces = Vec::with_capacity(self.knots.len());
        for (index, &(station, width)) in self.knots.iter().enumerate() {
            let piece = match self.knots.get(index + 1) {
                Some(&(next_station, next_width)) if next_station - station > 1e-12 => {
                    let span = next_station - station;
                    let delta = next_width.metres() - width.metres();
                    match self.taper {
                        Taper::Linear => Poly3Piece::new(
                            station - offset,
                            width.metres(),
                            delta / span,
                            0.0,
                            0.0,
                        ),
                        Taper::Smooth => Poly3Piece::new(
                            station - offset,
                            width.metres(),
                            0.0,
                            3.0 * delta / (span * span),
                            -2.0 * delta / (span * span * span),
                        ),
                    }
                }
                // The last knot, or a repeated station: hold the value.
                _ => Poly3Piece::new(station - offset, width.metres(), 0.0, 0.0, 0.0),
            };
            pieces.push(piece);
        }
        Poly3Profile::new(pieces).expect("every coefficient comes from a finite knot")
    }
}

impl From<PositiveWidth> for WidthProfile {
    fn from(width: PositiveWidth) -> Self {
        WidthProfile::constant(width)
    }
}

impl TryFrom<f64> for WidthProfile {
    type Error = QuantityError;
    fn try_from(metres: f64) -> Result<Self, QuantityError> {
        Ok(WidthProfile::constant(PositiveWidth::new(metres)?))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn w(metres: f64) -> PositiveWidth {
        PositiveWidth::new(metres).unwrap()
    }

    #[test]
    fn a_constant_profile_is_that_width_everywhere() {
        let profile = WidthProfile::constant(w(3.5));
        assert!(profile.is_constant());
        for station in [-100.0, 0.0, 1e6] {
            assert!((profile.evaluate(station).metres() - 3.5).abs() < 1e-12);
        }
        assert!((profile.narrowest().metres() - 3.5).abs() < 1e-12);
    }

    #[test]
    fn knots_that_change_nothing_are_not_kept() {
        // Flat, then a taper, then flat: the two knots that only repeat their
        // neighbours go, and the ones that shape the width stay.
        let profile = WidthProfile::new(
            [
                (0.0, w(2.0)),
                (30.0, w(2.0)),
                (60.0, w(2.0)),
                (100.0, w(5.0)),
                (160.0, w(5.0)),
                (200.0, w(5.0)),
            ],
            Taper::Linear,
        )
        .unwrap();
        let stations: Vec<f64> = profile.knots().iter().map(|(s, _)| *s).collect();
        assert_eq!(stations, vec![60.0, 100.0]);
        assert!((profile.evaluate(0.0).metres() - 2.0).abs() < 1e-12);
        assert!((profile.evaluate(80.0).metres() - 3.5).abs() < 1e-12);
        assert!((profile.evaluate(200.0).metres() - 5.0).abs() < 1e-12);

        // Two knots of one width are a constant profile, however far apart.
        let flat = WidthProfile::tapered(0.0, 50.0, w(3.5), w(3.5), Taper::Smooth).unwrap();
        assert_eq!(flat, WidthProfile::constant(w(3.5)));
    }

    #[test]
    fn a_linear_taper_runs_straight_between_its_ends() {
        let profile = WidthProfile::tapered(100.0, 50.0, w(3.5), w(1.0), Taper::Linear).unwrap();
        assert!(!profile.is_constant());
        // Held flat before and after.
        assert!((profile.evaluate(0.0).metres() - 3.5).abs() < 1e-12);
        assert!((profile.evaluate(1000.0).metres() - 1.0).abs() < 1e-12);
        // Straight between.
        assert!((profile.evaluate(125.0).metres() - 2.25).abs() < 1e-12);
        assert!((profile.narrowest().metres() - 1.0).abs() < 1e-12);
    }

    #[test]
    fn a_smooth_taper_leaves_and_arrives_flat() {
        let profile = WidthProfile::tapered(0.0, 100.0, w(3.5), w(1.5), Taper::Smooth).unwrap();
        // Halfway it is halfway, by symmetry of 3t² − 2t³.
        assert!((profile.evaluate(50.0).metres() - 2.5).abs() < 1e-12);
        // Near the ends it has barely moved, which is what "eases" means.
        assert!((profile.evaluate(2.0).metres() - 3.5).abs() < 0.01);
        assert!((profile.evaluate(98.0).metres() - 1.5).abs() < 0.01);
    }

    #[test]
    fn every_taper_stays_between_its_knots() {
        // The property the type exists to guarantee: sweep both tapers finely and
        // check the width never leaves the interval its knots bound, and so never
        // reaches zero.
        for taper in [Taper::Linear, Taper::Smooth] {
            let profile =
                WidthProfile::new([(0.0, w(3.5)), (40.0, w(0.25)), (90.0, w(5.0))], taper).unwrap();
            for step in 0..=2_000 {
                let station = -50.0 + step as f64 * 0.1;
                let width = profile.evaluate(station).metres();
                assert!(
                    width >= 0.25 - 1e-12,
                    "{taper:?} dipped to {width} at {station}"
                );
                assert!(width <= 5.0 + 1e-12);
            }
            assert!((profile.narrowest().metres() - 0.25).abs() < 1e-12);
        }
    }

    #[test]
    fn the_cubic_form_is_the_same_function() {
        for taper in [Taper::Linear, Taper::Smooth] {
            let profile =
                WidthProfile::new([(20.0, w(3.5)), (70.0, w(1.0)), (120.0, w(3.5))], taper)
                    .unwrap();
            let cubic = profile.to_poly3(0.0);
            for step in 0..=150 {
                let station = step as f64;
                let direct = profile.evaluate(station).metres();
                let lowered = cubic.evaluate(station);
                assert!(
                    (direct - lowered).abs() < 1e-9,
                    "{taper:?} at {station}: {direct} vs {lowered}"
                );
            }
        }
    }

    #[test]
    fn the_cubic_form_can_be_rebased_onto_a_section() {
        // OpenDRIVE measures a `<width>` from the start of its lane section, so the
        // lowering shifts the stations by the section's own.
        let profile = WidthProfile::tapered(100.0, 50.0, w(3.5), w(1.0), Taper::Linear).unwrap();
        let cubic = profile.to_poly3(100.0);
        assert!((cubic.evaluate(0.0) - 3.5).abs() < 1e-12);
        assert!((cubic.evaluate(25.0) - 2.25).abs() < 1e-12);
    }

    #[test]
    fn a_profile_needs_at_least_one_knot() {
        assert!(WidthProfile::new([], Taper::Linear).is_err());
        assert!(WidthProfile::tapered(0.0, 0.0, w(3.5), w(1.0), Taper::Linear).is_err());
    }
}
