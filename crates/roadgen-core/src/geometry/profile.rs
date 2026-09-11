//! A quantity that varies along a reference line.
//!
//! Elevation, superelevation and lane width are all the same shape: a value defined
//! piecewise by a cubic in the distance from the start of its piece. That is how
//! OpenDRIVE writes all three, and it is general enough that a caller who only wants
//! a constant or a straight ramp never has to think about the polynomial.

use crate::error::GeometryError;

/// One piece of a profile: `a + b·ds + c·ds² + d·ds³`, where `ds` is measured from
/// `station`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Poly3Piece {
    /// Horizontal station where this piece takes over, metres.
    pub station: f64,
    pub a: f64,
    pub b: f64,
    pub c: f64,
    pub d: f64,
}

impl Poly3Piece {
    pub fn new(station: f64, a: f64, b: f64, c: f64, d: f64) -> Self {
        Poly3Piece {
            station,
            a,
            b,
            c,
            d,
        }
    }

    pub fn evaluate(&self, station: f64) -> f64 {
        let ds = station - self.station;
        self.a + self.b * ds + self.c * ds * ds + self.d * ds * ds * ds
    }
}

/// A value defined piecewise along a reference line.
///
/// Pieces are kept sorted by station, and the first one governs everything before it
/// as well — a profile is total, so evaluating it never fails and never has a hole.
#[derive(Debug, Clone, PartialEq)]
pub struct Poly3Profile {
    pieces: Vec<Poly3Piece>,
}

impl Poly3Profile {
    /// The profile that is `value` everywhere.
    pub fn constant(value: f64) -> Self {
        Poly3Profile {
            pieces: vec![Poly3Piece::new(0.0, value, 0.0, 0.0, 0.0)],
        }
    }

    /// Builds a profile from its pieces, sorting them and rejecting a piece whose
    /// coefficients are not finite.
    pub fn new(pieces: impl IntoIterator<Item = Poly3Piece>) -> Result<Self, GeometryError> {
        let mut pieces: Vec<Poly3Piece> = pieces.into_iter().collect();
        if pieces.is_empty() {
            return Ok(Poly3Profile::constant(0.0));
        }
        for piece in &pieces {
            if ![piece.station, piece.a, piece.b, piece.c, piece.d]
                .iter()
                .all(|value| value.is_finite())
            {
                return Err(GeometryError::NonFiniteCoordinate);
            }
        }
        pieces.sort_by(|left, right| {
            left.station
                .partial_cmp(&right.station)
                .expect("stations were checked to be finite")
        });
        Ok(Poly3Profile { pieces })
    }

    /// A profile that passes through each `(station, value)` in turn, straight
    /// between them and flat outside them.
    ///
    /// This is the form a caller reaches for — "level here, banked 4 degrees there" —
    /// without having to write a polynomial.
    pub fn piecewise_linear(
        points: impl IntoIterator<Item = (f64, f64)>,
    ) -> Result<Self, GeometryError> {
        let mut points: Vec<(f64, f64)> = points.into_iter().collect();
        if points.is_empty() {
            return Ok(Poly3Profile::constant(0.0));
        }
        if !points
            .iter()
            .all(|(station, value)| station.is_finite() && value.is_finite())
        {
            return Err(GeometryError::NonFiniteCoordinate);
        }
        points.sort_by(|left, right| {
            left.0
                .partial_cmp(&right.0)
                .expect("stations were checked to be finite")
        });

        let mut pieces = Vec::with_capacity(points.len());
        for (index, &(station, value)) in points.iter().enumerate() {
            // The last point has nothing to slope towards, so it holds its value.
            let slope = match points.get(index + 1) {
                Some(&(next_station, next_value)) if next_station - station > 1e-12 => {
                    (next_value - value) / (next_station - station)
                }
                _ => 0.0,
            };
            pieces.push(Poly3Piece::new(station, value, slope, 0.0, 0.0));
        }
        Poly3Profile::new(pieces)
    }

    /// The value at `station`.
    ///
    /// Before the first piece the profile holds that piece's starting value rather
    /// than extrapolating its polynomial backwards, which for a cubic runs away
    /// fast. In practice a profile starts at zero and this never arises; it is
    /// defined so that evaluating one can never surprise a caller.
    pub fn evaluate(&self, station: f64) -> f64 {
        let first = self.pieces[0].station;
        self.piece_at(station).evaluate(station.max(first))
    }

    /// The piece that governs `station`.
    fn piece_at(&self, station: f64) -> &Poly3Piece {
        self.pieces
            .iter()
            .rfind(|piece| piece.station <= station + 1e-9)
            .unwrap_or(&self.pieces[0])
    }

    pub fn pieces(&self) -> &[Poly3Piece] {
        &self.pieces
    }

    /// Whether the profile is zero everywhere, so a caller can skip writing it out.
    pub fn is_zero(&self) -> bool {
        self.pieces
            .iter()
            .all(|piece| piece.a == 0.0 && piece.b == 0.0 && piece.c == 0.0 && piece.d == 0.0)
    }

    /// The same profile with every value multiplied by `factor`.
    pub fn scaled(&self, factor: f64) -> Poly3Profile {
        Poly3Profile {
            pieces: self
                .pieces
                .iter()
                .map(|piece| Poly3Piece {
                    station: piece.station,
                    a: piece.a * factor,
                    b: piece.b * factor,
                    c: piece.c * factor,
                    d: piece.d * factor,
                })
                .collect(),
        }
    }

    /// The largest absolute value the profile takes at any of `stations`.
    pub fn peak_over(&self, stations: impl IntoIterator<Item = f64>) -> f64 {
        stations
            .into_iter()
            .map(|station| self.evaluate(station).abs())
            .fold(0.0_f64, f64::max)
    }
}

impl Default for Poly3Profile {
    fn default() -> Self {
        Poly3Profile::constant(0.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_constant_profile_is_that_value_everywhere() {
        let profile = Poly3Profile::constant(3.5);
        assert!((profile.evaluate(-10.0) - 3.5).abs() < 1e-12);
        assert!((profile.evaluate(1000.0) - 3.5).abs() < 1e-12);
        assert!(!profile.is_zero());
        assert!(Poly3Profile::constant(0.0).is_zero());
    }

    #[test]
    fn a_piecewise_linear_profile_ramps_between_its_points() {
        let profile =
            Poly3Profile::piecewise_linear([(0.0, 0.0), (100.0, 0.08), (200.0, 0.08)]).unwrap();
        assert!(profile.evaluate(0.0).abs() < 1e-12);
        assert!((profile.evaluate(50.0) - 0.04).abs() < 1e-12);
        assert!((profile.evaluate(100.0) - 0.08).abs() < 1e-12);
        assert!((profile.evaluate(150.0) - 0.08).abs() < 1e-12);
        // Past the last point the value holds rather than running away.
        assert!((profile.evaluate(1e6) - 0.08).abs() < 1e-12);
    }

    #[test]
    fn the_first_piece_governs_everything_before_it() {
        let profile = Poly3Profile::new([Poly3Piece::new(50.0, 2.0, 0.1, 0.0, 0.0)]).unwrap();
        assert!((profile.evaluate(0.0) - 2.0).abs() < 1e-12);
        assert!((profile.evaluate(60.0) - 3.0).abs() < 1e-12);
    }

    #[test]
    fn pieces_are_sorted_however_they_arrive() {
        let profile = Poly3Profile::new([
            Poly3Piece::new(100.0, 1.0, 0.0, 0.0, 0.0),
            Poly3Piece::new(0.0, 0.0, 0.0, 0.0, 0.0),
        ])
        .unwrap();
        assert_eq!(
            profile
                .pieces()
                .iter()
                .map(|p| p.station)
                .collect::<Vec<_>>(),
            [0.0, 100.0]
        );
    }

    #[test]
    fn scaling_scales_the_value_everywhere() {
        let profile = Poly3Profile::piecewise_linear([(0.0, 4.0), (100.0, 2.0)]).unwrap();
        let half = profile.scaled(0.5);
        for station in [0.0, 25.0, 100.0, 200.0] {
            assert!((half.evaluate(station) - profile.evaluate(station) / 2.0).abs() < 1e-12);
        }
    }

    #[test]
    fn a_profile_with_a_non_finite_coefficient_is_refused() {
        assert!(Poly3Profile::new([Poly3Piece::new(0.0, f64::NAN, 0.0, 0.0, 0.0)]).is_err());
        assert!(Poly3Profile::piecewise_linear([(0.0, f64::INFINITY)]).is_err());
    }

    #[test]
    fn the_cubic_terms_are_evaluated_too() {
        let profile = Poly3Profile::new([Poly3Piece::new(0.0, 1.0, 2.0, 3.0, 4.0)]).unwrap();
        // 1 + 2·2 + 3·4 + 4·8
        assert!((profile.evaluate(2.0) - 49.0).abs() < 1e-12);
    }
}
