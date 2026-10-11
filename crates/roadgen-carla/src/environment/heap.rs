//! A heap of broken rock: a waste dump, a stockpile. A frustum, its sides at the
//! material's angle of repose.

use std::f64::consts::TAU;

use roadgen_core::geometry::Point3;

use super::land::{plan_of, Landforms, Level, Line};

/// A heap.
#[derive(Debug, Clone, PartialEq)]
pub struct HeapSpec {
    /// What the heap's features are named after.
    pub name: String,
    /// What it is a heap of, which is its land cover's kind: `waste_dump`,
    /// `stockpile`.
    pub kind: String,
    /// The centre of its toe, at the height it stands on.
    pub centre: Point3,
    /// The radius of its toe.
    pub radius: f64,
    pub height: f64,
    /// Degrees from the horizontal.
    pub repose: f64,
    /// The site it belongs to, if any.
    pub site: Option<String>,
}

impl Default for HeapSpec {
    fn default() -> Self {
        HeapSpec {
            name: "heap".into(),
            kind: "waste_dump".into(),
            centre: Point3::ORIGIN,
            radius: 60.0,
            height: 20.0,
            repose: 37.0,
            site: None,
        }
    }
}

impl HeapSpec {
    pub fn check(&self) -> Result<(), String> {
        if !(self.radius.is_finite() && self.radius > 0.0) {
            return Err(format!("{}: a radius of {} m", self.name, self.radius));
        }
        if !(self.height.is_finite() && self.height > 0.0) {
            return Err(format!("{}: a height of {} m", self.name, self.height));
        }
        if !(5.0..80.0).contains(&self.repose) {
            return Err(format!(
                "{}: a repose of {} degrees",
                self.name, self.repose
            ));
        }
        if self.top_radius() <= 0.0 {
            return Err(format!(
                "{}: {} m high at {} degrees needs a wider toe than {} m",
                self.name, self.height, self.repose, self.radius
            ));
        }
        if !self.centre.is_finite() {
            return Err(format!("{}: the centre must be finite", self.name));
        }
        if self.kind.trim().is_empty() {
            return Err(format!("{}: a heap is a heap of something", self.name));
        }
        Ok(())
    }

    fn top_radius(&self) -> f64 {
        self.radius - self.height / self.repose.to_radians().tan()
    }

    fn ring(&self, radius: f64, z: f64, spacing: f64) -> Vec<Point3> {
        let count = (TAU * radius / spacing).ceil().max(12.0) as usize;
        (0..count)
            .map(|index| {
                let angle = TAU * index as f64 / count as f64;
                Point3::new(
                    self.centre.x + radius * angle.cos(),
                    self.centre.y + radius * angle.sin(),
                    z,
                )
            })
            .collect()
    }

    /// The heap's toe: its outline on the ground.
    pub fn toe(&self, spacing: f64) -> Vec<Point3> {
        self.ring(self.radius, self.centre.z, spacing)
    }

    /// What the heap shapes the land with: its toe and crest as lines, its top as a
    /// level, and its toe as a hole the country stays out of, at the height the
    /// heap stands on.
    pub fn landforms(&self, key: &str, spacing: f64) -> Landforms {
        let toe = self.toe(spacing);
        let top = self.centre.z + self.height;
        let crest = self.ring(self.top_radius(), top, spacing);
        let mut forms = Landforms::default();
        forms.holes.push(plan_of(&toe));
        forms.levels.push(Level {
            ring: plan_of(&crest),
            z: top,
        });
        forms.anchors.extend(toe.iter().copied());
        forms.lines.push(Line {
            key: format!("{key}/toe"),
            kind: "toe",
            points: toe,
            closed: true,
        });
        forms.lines.push(Line {
            key: format!("{key}/crest"),
            kind: "crest",
            points: crest,
            closed: true,
        });
        forms
    }
}
