//! Quantities whose range is part of their type.
//!
//! A `PositiveWidth` cannot hold zero or a negative number, so "lane width > 0" is
//! not a rule validation has to look for — it is a state the program cannot reach.

use crate::error::QuantityError;

/// A lane width in metres, guaranteed greater than zero.
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd)]
pub struct PositiveWidth(f64);

impl PositiveWidth {
    pub fn new(metres: f64) -> Result<Self, QuantityError> {
        if !(metres.is_finite() && metres > 0.0) {
            return Err(QuantityError::NonPositiveWidth(metres));
        }
        Ok(PositiveWidth(metres))
    }

    pub fn metres(self) -> f64 {
        self.0
    }

    pub fn half(self) -> f64 {
        self.0 / 2.0
    }
}

/// A speed limit, guaranteed greater than zero.
///
/// Stored in metres per second: OpenDRIVE writes a unit alongside the number and
/// Lanelet2 writes km/h, so neither format's unit is privileged in the IR.
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd)]
pub struct SpeedLimit(f64);

impl SpeedLimit {
    pub fn from_mps(value: f64) -> Result<Self, QuantityError> {
        if !(value.is_finite() && value > 0.0) {
            return Err(QuantityError::NonPositiveSpeed(value));
        }
        Ok(SpeedLimit(value))
    }

    pub fn from_kph(value: f64) -> Result<Self, QuantityError> {
        SpeedLimit::from_mps(value / 3.6)
    }

    pub fn mps(self) -> f64 {
        self.0
    }

    pub fn kph(self) -> f64 {
        self.0 * 3.6
    }
}

/// A geodetic anchor for the map's metric coordinates.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GeoOrigin {
    latitude: f64,
    longitude: f64,
    altitude: f64,
}

impl GeoOrigin {
    pub fn new(latitude: f64, longitude: f64, altitude: f64) -> Result<Self, QuantityError> {
        if !latitude.is_finite() || latitude.abs() > 90.0 {
            return Err(QuantityError::LatitudeOutOfRange(latitude));
        }
        if !longitude.is_finite() || longitude.abs() > 180.0 {
            return Err(QuantityError::LongitudeOutOfRange(longitude));
        }
        Ok(GeoOrigin {
            latitude,
            longitude,
            altitude: if altitude.is_finite() { altitude } else { 0.0 },
        })
    }

    pub fn latitude(self) -> f64 {
        self.latitude
    }

    pub fn longitude(self) -> f64 {
        self.longitude
    }

    pub fn altitude(self) -> f64 {
        self.altitude
    }
}

impl Default for GeoOrigin {
    fn default() -> Self {
        GeoOrigin {
            latitude: 0.0,
            longitude: 0.0,
            altitude: 0.0,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_lane_cannot_be_built_with_a_non_positive_width() {
        for bad in [0.0, -3.5, f64::NAN] {
            assert!(PositiveWidth::new(bad).is_err());
        }
        assert!((PositiveWidth::new(3.5).unwrap().metres() - 3.5).abs() < 1e-12);
    }

    #[test]
    fn speed_limits_convert_both_ways() {
        let limit = SpeedLimit::from_kph(60.0).unwrap();
        assert!((limit.mps() - 16.6667).abs() < 1e-3);
        assert!((limit.kph() - 60.0).abs() < 1e-9);
    }

    #[test]
    fn an_origin_off_the_globe_is_refused() {
        assert!(GeoOrigin::new(91.0, 0.0, 0.0).is_err());
        assert!(GeoOrigin::new(0.0, 181.0, 0.0).is_err());
        assert!(GeoOrigin::new(35.6, 139.7, 40.0).is_ok());
    }
}
