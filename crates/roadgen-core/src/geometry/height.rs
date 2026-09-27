//! How far a lane's surface stands off the road's, along its length.
//!
//! A road's surface is its reference line, its elevation and its superelevation:
//! one tilted plane across the road at every station. Real carriageways are not
//! always that — a pavement stands a kerb's height above the carriageway beside it,
//! and a surveyed map's lanes rarely lie on one plane — so a lane may lift its two
//! edges off the road surface: `inner` at the edge nearer the road's reference
//! line, `outer` at the edge further out, straight across the lane between them.
//! This is OpenDRIVE's `<height>`, with its meaning.
//!
//! Along the road the heights run straight from one knot to the next and are held
//! before the first and after the last, as a [`WidthProfile`](super::WidthProfile)
//! is. A lane with no knots is flat on the road surface, which is every lane the IR
//! held before heights existed.

use crate::error::GeometryError;

/// The lift of a lane's inner and outer edges off the road surface, metres, as a
/// function of station along the road's reference line.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct LaneHeight {
    /// `(station, inner, outer)`, ascending by station. Empty for a flat lane.
    knots: Vec<(f64, f64, f64)>,
}

impl LaneHeight {
    /// On the road surface everywhere.
    pub fn flat() -> Self {
        LaneHeight::default()
    }

    /// The same lift everywhere.
    pub fn constant(inner: f64, outer: f64) -> Result<Self, GeometryError> {
        LaneHeight::new([(0.0, inner, outer)])
    }

    /// Heights passing through each `(station, inner, outer)` in turn, straight
    /// between them and held before the first and after the last.
    ///
    /// Kept canonical, as a width profile is: knots that all lift nothing are the
    /// flat lane, so a lane described as zero is the same lane as one described not
    /// at all.
    pub fn new(knots: impl IntoIterator<Item = (f64, f64, f64)>) -> Result<Self, GeometryError> {
        let mut knots: Vec<(f64, f64, f64)> = knots.into_iter().collect();
        if knots.iter().any(|(station, inner, outer)| {
            !(station.is_finite() && inner.is_finite() && outer.is_finite())
        }) {
            return Err(GeometryError::NonFiniteCoordinate);
        }
        knots.sort_by(|a, b| a.0.total_cmp(&b.0));
        if knots
            .iter()
            .all(|(_, inner, outer)| inner.abs() < 1e-12 && outer.abs() < 1e-12)
        {
            return Ok(LaneHeight::flat());
        }
        Ok(LaneHeight { knots })
    }

    pub fn knots(&self) -> &[(f64, f64, f64)] {
        &self.knots
    }

    /// Whether the lane lies on the road surface everywhere.
    pub fn is_flat(&self) -> bool {
        self.knots.is_empty()
    }

    /// The lift of the inner and outer edges at `station`.
    pub fn evaluate(&self, station: f64) -> (f64, f64) {
        let Some(first) = self.knots.first() else {
            return (0.0, 0.0);
        };
        let last = self.knots[self.knots.len() - 1];
        if station <= first.0 {
            return (first.1, first.2);
        }
        if station >= last.0 {
            return (last.1, last.2);
        }
        let index = self
            .knots
            .iter()
            .rposition(|knot| knot.0 <= station)
            .unwrap_or(0)
            .min(self.knots.len() - 2);
        let (from, to) = (self.knots[index], self.knots[index + 1]);
        let span = to.0 - from.0;
        if span <= 0.0 {
            return (to.1, to.2);
        }
        let t = (station - from.0) / span;
        (from.1 + (to.1 - from.1) * t, from.2 + (to.2 - from.2) * t)
    }

    /// The lift at a fraction `across` of the way from the inner edge (0) to the
    /// outer edge (1), at `station`.
    pub fn across(&self, station: f64, across: f64) -> f64 {
        let (inner, outer) = self.evaluate(station);
        inner + (outer - inner) * across
    }

    /// Goes straight from `from` to `to`, each `(inner, outer)`, over the stations
    /// `start` to `end`.
    pub fn tapered(
        start: f64,
        end: f64,
        from: (f64, f64),
        to: (f64, f64),
    ) -> Result<Self, GeometryError> {
        LaneHeight::new([(start, from.0, from.1), (end, to.0, to.1)])
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_lane_described_as_zero_is_flat() {
        assert!(LaneHeight::constant(0.0, 0.0).unwrap().is_flat());
        assert_eq!(LaneHeight::flat().evaluate(12.0), (0.0, 0.0));
    }

    #[test]
    fn heights_run_straight_between_knots_and_hold_beyond_them() {
        let height = LaneHeight::new([(10.0, 0.0, 0.2), (20.0, 0.1, 0.4)]).unwrap();
        assert_eq!(height.evaluate(0.0), (0.0, 0.2));
        let (inner, outer) = height.evaluate(15.0);
        assert!((inner - 0.05).abs() < 1e-12 && (outer - 0.3).abs() < 1e-12);
        assert_eq!(height.evaluate(30.0), (0.1, 0.4));
        assert!((height.across(15.0, 0.5) - 0.175).abs() < 1e-12);
    }

    #[test]
    fn a_height_that_is_not_a_number_is_refused() {
        assert!(LaneHeight::constant(f64::NAN, 0.0).is_err());
    }
}
