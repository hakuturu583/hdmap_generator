//! Reporting a map's coordinates as MGRS grid metres.
//!
//! Autoware's MGRS projector reads a node's metric position out of its `local_x` and
//! `local_y` tags, and expects them to be metres **within a 100 km MGRS square** — a
//! coordinate system whose origin is the square's south-west corner, somewhere in the
//! world, and not the map's own origin.
//!
//! A generated map's coordinates start at its origin instead, so the two have to be
//! reconciled. The honest way is to do it per node: take the node's real latitude and
//! longitude, put *that* on the UTM grid, and subtract the square's corner. Shifting
//! the whole map by the origin's own position in the square would be a metre or so out
//! across a few kilometres, because a metre of local east is not a metre of UTM
//! easting.
//!
//! Anything that leaves the square cannot be expressed this way at all. `ll2`'s MGRS
//! projector takes the easting and northing modulo 100 km, so a map that runs over the
//! edge would silently come back on the other side; here it is an error instead.

use ll2_projection::{mgrs, utmups, GpsPoint};

use roadgen_core::units::GeoOrigin;

use crate::error::ExportError;

/// The side of an MGRS square, metres.
const TILE: f64 = 100_000.0;

/// The 100 km MGRS square a map lives in.
#[derive(Debug, Clone, PartialEq)]
pub struct MgrsGrid {
    /// The square's grid reference, e.g. `54SUE`.
    code: String,
    zone: i32,
    northern: bool,
    /// The square's south-west corner, in UTM metres.
    corner: (f64, f64),
}

impl MgrsGrid {
    /// The square containing `origin`.
    pub fn containing(origin: GeoOrigin) -> Result<Self, ExportError> {
        let failed = |detail: String| ExportError::Projection(detail);
        let (zone, northern, x, y) = utmups::forward(origin.latitude(), origin.longitude())
            .map_err(|error| failed(error.message().to_owned()))?;
        // Precision 0 names the square and nothing finer, which is exactly the
        // reference Autoware's MGRS projector wants.
        let code = mgrs::forward(zone, northern, x, y, origin.latitude(), 0)
            .map_err(|error| failed(error.message().to_owned()))?;
        let (zone, northern, corner_x, corner_y, _) =
            mgrs::reverse(&code).map_err(|error| failed(error.message().to_owned()))?;
        Ok(MgrsGrid {
            code,
            zone,
            northern,
            corner: (corner_x, corner_y),
        })
    }

    /// The grid reference of the square, for a caller writing out map metadata.
    pub fn code(&self) -> &str {
        &self.code
    }

    /// Where `position` sits within the square, metres east and north of its corner.
    ///
    /// Fails rather than wrapping if the position is not in this square: a map that
    /// straddles a grid boundary has no MGRS coordinates, and pretending otherwise
    /// would put half of it 100 km away.
    pub fn locate(&self, position: GpsPoint) -> Result<(f64, f64), ExportError> {
        let (zone, northern, x, y) = utmups::forward(position.lat, position.lon)
            .map_err(|error| ExportError::Projection(error.message().to_owned()))?;
        if zone != self.zone || northern != self.northern {
            return Err(ExportError::GridCrossed {
                code: self.code.clone(),
                detail: format!(
                    "a position at {:.6}, {:.6} falls in UTM zone {zone}{}, not {}{}",
                    position.lat,
                    position.lon,
                    if northern { "N" } else { "S" },
                    self.zone,
                    if self.northern { "N" } else { "S" },
                ),
            });
        }
        let (east, north) = (x - self.corner.0, y - self.corner.1);
        if !(0.0..TILE).contains(&east) || !(0.0..TILE).contains(&north) {
            return Err(ExportError::GridCrossed {
                code: self.code.clone(),
                detail: format!(
                    "a position at {:.6}, {:.6} is {east:.1} m east and {north:.1} m \
                     north of the square's corner, which is outside it",
                    position.lat, position.lon
                ),
            });
        }
        Ok((east, north))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn tokyo() -> GeoOrigin {
        GeoOrigin::new(35.68, 139.76, 0.0).unwrap()
    }

    #[test]
    fn a_grid_is_named_for_the_square_its_origin_falls_in() {
        let grid = MgrsGrid::containing(tokyo()).unwrap();
        // Zone 54, band S — central Tokyo.
        assert!(grid.code().starts_with("54S"), "code {}", grid.code());
        assert_eq!(grid.code().len(), 5, "the square alone, no digits");
    }

    #[test]
    fn the_origin_sits_inside_its_own_square() {
        let origin = tokyo();
        let grid = MgrsGrid::containing(origin).unwrap();
        let (east, north) = grid
            .locate(GpsPoint::new(origin.latitude(), origin.longitude(), 0.0))
            .unwrap();
        assert!((0.0..TILE).contains(&east));
        assert!((0.0..TILE).contains(&north));
    }

    #[test]
    fn a_step_north_does_not_move_straight_up_the_grid() {
        let grid = MgrsGrid::containing(tokyo()).unwrap();
        let here = grid.locate(GpsPoint::new(35.68, 139.76, 0.0)).unwrap();
        // A hundredth of a degree of latitude is about 1.1 km north.
        let there = grid.locate(GpsPoint::new(35.69, 139.76, 0.0)).unwrap();
        assert!(
            (there.1 - here.1 - 1109.0).abs() < 5.0,
            "northing {:?} -> {:?}",
            here,
            there
        );

        // And the easting moves too, by about fourteen metres: Tokyo is over a degree
        // west of its zone's central meridian, so grid north is not true north there.
        // This is exactly why a map's MGRS coordinates are worked out for each node
        // rather than by shifting the whole map by its origin's grid position.
        let drift = there.0 - here.0;
        assert!(
            (5.0..30.0).contains(&drift.abs()),
            "meridian convergence should move the easting, got {drift}"
        );
    }

    #[test]
    fn somewhere_in_another_square_is_refused_rather_than_wrapped() {
        let grid = MgrsGrid::containing(tokyo()).unwrap();
        // Osaka: same band, a different square entirely.
        let error = grid.locate(GpsPoint::new(34.69, 135.50, 0.0)).unwrap_err();
        assert!(matches!(error, ExportError::GridCrossed { .. }));
    }

    #[test]
    fn the_poles_have_no_grid() {
        let arctic = GeoOrigin::new(88.0, 20.0, 0.0).unwrap();
        assert!(MgrsGrid::containing(arctic).is_err());
    }
}
