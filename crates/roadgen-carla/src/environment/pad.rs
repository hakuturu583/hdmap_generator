//! A pad: a levelled yard, with what stands on it — a mine's workshops and offices
//! beside its haul road, from the same shape grammar a town's buildings come from.

use roadgen_core::geometry::Point3;

use super::land::{plan_of, Landforms, Level, Line};

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

    /// The pad's height: where its outline is, on average.
    pub fn height(&self) -> f64 {
        self.outline.iter().map(|p| p.z).sum::<f64>() / self.outline.len() as f64
    }

    /// What the pad shapes the land with: a region levelled at the pad's height,
    /// edged by a breakline there, that the country stays out of and does not roll
    /// beside.
    pub fn landforms(&self, key: &str, spacing: f64) -> Landforms {
        let z = self.height();
        // A vertex every `spacing` metres along the edge, so that where a road runs
        // beside it the edge is cut back there and not across a whole side.
        let mut edge = Vec::new();
        for (index, &from) in self.outline.iter().enumerate() {
            let to = self.outline[(index + 1) % self.outline.len()];
            let steps = (from.horizontal_distance_to(to) / spacing).ceil().max(1.0) as usize;
            for step in 0..steps {
                let p = from.lerp(to, step as f64 / steps as f64);
                edge.push(Point3::new(p.x, p.y, z));
            }
        }
        let ring = plan_of(&edge);
        Landforms {
            levels: vec![Level::flat(ring.clone(), z)],
            holes: vec![ring.clone()],
            flats: vec![ring],
            anchors: edge.clone(),
            lines: vec![Line {
                key: format!("{key}/edge"),
                kind: "edge",
                points: edge,
                closed: true,
            }],
            ..Landforms::default()
        }
    }
}
