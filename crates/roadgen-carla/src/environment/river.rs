//! A river: a channel with a flat bed and sloping banks, for a bridge to cross.
//!
//! Its banks' tops and feet are breaklines, its bed a level, and what lies between
//! its banks' tops is a land cover of kind `water` — which is what makes a road over
//! it a bridge over water, to every format that reads one.

use roadgen_core::geometry::Point3;

use super::land::{plan_of, Landforms, Level, Line};

/// A river.
#[derive(Debug, Clone, PartialEq)]
pub struct RiverSpec {
    pub name: String,
    /// Down the middle, at the height of the tops of its banks.
    pub centre: Vec<Point3>,
    /// From the top of one bank to the top of the other, metres.
    pub width: f64,
    pub depth: f64,
    /// How many metres a bank runs out for every metre it falls.
    pub bank_slope: f64,
    pub site: Option<String>,
}

impl RiverSpec {
    pub fn check(&self) -> Result<(), String> {
        if self.centre.len() < 2 || self.centre.iter().any(|p| !p.is_finite()) {
            return Err(format!(
                "{}: a river needs a centre line of two finite points",
                self.name
            ));
        }
        if !(self.width.is_finite()
            && self.width > 0.0
            && self.depth.is_finite()
            && self.depth > 0.0)
        {
            return Err(format!(
                "{}: a river needs a positive width and depth",
                self.name
            ));
        }
        if !(self.bank_slope.is_finite() && self.bank_slope >= 0.0) || self.bed_half() <= 0.5 {
            return Err(format!(
                "{}: banks {} m deep at 1:{} leave no bed in a river {} m wide",
                self.name, self.depth, self.bank_slope, self.width
            ));
        }
        Ok(())
    }

    fn bed_half(&self) -> f64 {
        self.width / 2.0 - self.depth * self.bank_slope
    }

    /// The centre line moved `offset` metres to its left, `drop` metres down,
    /// with a vertex every `spacing` metres.
    fn line(&self, offset: f64, drop: f64, spacing: f64) -> Vec<Point3> {
        let mut out = Vec::new();
        let n = self.centre.len();
        for index in 0..n {
            let a = self.centre[index.saturating_sub(1)];
            let b = self.centre[(index + 1).min(n - 1)];
            let (dx, dy) = (b.x - a.x, b.y - a.y);
            let len = dx.hypot(dy).max(1e-9);
            let (nx, ny) = (-dy / len, dx / len);
            let p = self.centre[index];
            let here = Point3::new(p.x + nx * offset, p.y + ny * offset, p.z - drop);
            if let (Some(&last), true) = (out.last(), index > 0) {
                let last: Point3 = last;
                let steps = (last.horizontal_distance_to(here) / spacing)
                    .ceil()
                    .max(1.0) as usize;
                for step in 1..steps {
                    out.push(last.lerp(here, step as f64 / steps as f64));
                }
            }
            out.push(here);
        }
        out
    }

    /// Its outline: the tops of both banks, round.
    pub fn outline(&self, spacing: f64) -> Vec<Point3> {
        let half = self.width / 2.0;
        let mut ring = self.line(half, 0.0, spacing);
        ring.extend(self.line(-half, 0.0, spacing).into_iter().rev());
        ring
    }

    pub fn landforms(&self, key: &str, spacing: f64) -> Landforms {
        let (half, bed) = (self.width / 2.0, self.bed_half());
        let line = |name: &str, kind: &'static str, points: Vec<Point3>| Line {
            key: format!("{key}/{name}"),
            kind,
            points,
            closed: false,
        };
        let top_left = self.line(half, 0.0, spacing);
        let top_right = self.line(-half, 0.0, spacing);
        let bed_left = self.line(bed, self.depth, spacing);
        let bed_right = self.line(-bed, self.depth, spacing);
        let bed_z = self
            .centre
            .iter()
            .map(|p| p.z - self.depth)
            .fold(f64::INFINITY, f64::min);
        let bed_ring: Vec<Point3> = bed_left
            .iter()
            .copied()
            .chain(bed_right.iter().rev().copied())
            .collect();
        let outline = self.outline(spacing);
        Landforms {
            levels: vec![Level {
                ring: plan_of(&bed_ring),
                z: bed_z,
            }],
            holes: vec![plan_of(&outline)],
            anchors: top_left.iter().chain(&top_right).copied().collect(),
            lines: vec![
                line("bank/left", "bank", top_left),
                line("bank/right", "bank", top_right),
                line("bed/left", "bed", bed_left),
                line("bed/right", "bed", bed_right),
            ],
            ..Landforms::default()
        }
    }
}
