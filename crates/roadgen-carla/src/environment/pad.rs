//! A pad: a levelled yard, with what stands on it — a mine's workshops and offices
//! beside its haul road, from the same shape grammar a town's buildings come from.

use roadgen_core::geometry::Point3;

use super::land::{plan_of, Landforms};

/// A pad.
#[derive(Debug, Clone, PartialEq)]
pub struct PadSpec {
    /// What the pad's features are named after.
    pub name: String,
    /// Its outline, anticlockwise, the first point not repeated.
    pub outline: Vec<Point3>,
    /// What stands on it: lots along the roads beside it, kept to the pad.
    pub buildings: Option<roadgen_buildings::Rules>,
    /// The site it belongs to, if any.
    pub site: Option<String>,
}

impl PadSpec {
    pub fn check(&self) -> Result<(), String> {
        if self.outline.len() < 3 {
            return Err(format!(
                "{}: a pad's outline has at least three points",
                self.name
            ));
        }
        if self.outline.iter().any(|p| !p.is_finite()) {
            return Err(format!("{}: the outline must be finite", self.name));
        }
        Ok(())
    }

    /// What the pad shapes the land with: a region that does not roll.
    pub fn landforms(&self) -> Landforms {
        Landforms {
            flats: vec![plan_of(&self.outline)],
            ..Landforms::default()
        }
    }
}
