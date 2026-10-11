//! A hill or a ridge: the country swelling up along a line, for a tunnel to go
//! through or a road to climb over.
//!
//! A hill has no edge the way a heap has a toe: it rises smoothly out of the
//! country and falls back into it, so it adds no breakline and no land cover, only
//! height ([`Rise`]).

use roadgen_core::geometry::Point3;

use super::land::{Landforms, Rise};

/// A hill round a point, or a ridge along a line.
#[derive(Debug, Clone, PartialEq)]
pub struct HillSpec {
    pub name: String,
    /// The crest: one point for a round hill, several for a ridge. Heights are
    /// ignored; the crest stands `height` above the country.
    pub crest: Vec<Point3>,
    /// From foot to foot across it, metres.
    pub width: f64,
    pub height: f64,
}

impl HillSpec {
    pub fn check(&self) -> Result<(), String> {
        if self.crest.is_empty() || self.crest.iter().any(|p| !p.is_finite()) {
            return Err(format!(
                "{}: a hill needs a crest of finite points",
                self.name
            ));
        }
        if !(self.width.is_finite() && self.width > 0.0) {
            return Err(format!("{}: a width of {} m", self.name, self.width));
        }
        if !(self.height.is_finite() && self.height > 0.0) {
            return Err(format!("{}: a height of {} m", self.name, self.height));
        }
        Ok(())
    }

    pub fn landforms(&self) -> Landforms {
        Landforms {
            rises: vec![Rise {
                crest: self.crest.iter().map(|p| [p.x, p.y]).collect(),
                half_width: self.width / 2.0,
                height: self.height,
            }],
            ..Landforms::default()
        }
    }
}
