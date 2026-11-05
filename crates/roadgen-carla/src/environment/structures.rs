//! What the map's bridges and tunnels ask of the land.
//!
//! The land is one 2.5-dimensional TIN, and that is enough: under a bridge there is
//! one ground, over a tunnel there is one ground. What changes is that a road on a
//! structure is not on the ground, so its edges are no breaklines of the land and
//! its height says nothing about the land's (see [`super::land`]), and the land has
//! to keep clear of the structure:
//!
//! - over a tunnel it stands at least a metre above the lining, and slopes away
//!   from there to the country — the cover a cut-and-cover tunnel is buried under,
//!   or the hill a bored one goes through, whichever is higher;
//! - under a bridge, away from its abutments, it stays half a metre below the
//!   underside of the deck.

use roadgen_core::geometry::plan::{ring_contains, PlanPoint, Segments};
use roadgen_core::geometry::Grid;
use roadgen_core::map::Map;
use roadgen_core::structure::StructureKind;

use super::land::smoothstep;

/// The least cover over a tunnel's lining, and the least clearance under a deck,
/// metres.
const COVER: f64 = 1.0;
const CLEARANCE: f64 = 0.5;
/// How far past a tunnel's outline its cover slopes away to the country, metres.
const SHOULDER: f64 = 40.0;

enum Need {
    /// The land stays this far above the road, at least.
    Over(f64),
    /// The land stays this far below the road, at most, away from the ends.
    Under(f64),
}

struct Zone {
    need: Need,
    ring: Vec<PlanPoint>,
    edges: Segments,
    bounds: [f64; 4],
    /// The carried road down its middle, a row every two metres.
    rows: Vec<Row>,
    /// The rows filed by where they are, for the nearest to a point.
    index: Grid<usize>,
}

struct Row {
    at: PlanPoint,
    z: f64,
    /// Clear of the structure's ends, where it meets the ground.
    interior: bool,
    /// At a portal, the way out of the structure, in plan: a point ahead of it is
    /// in front of the mouth, not over the bore.
    mouth: Option<PlanPoint>,
}

/// Every structure's demand on the land.
pub(crate) struct Zones {
    zones: Vec<Zone>,
    /// How far past an outline its demand reaches in full: about a cell and a
    /// half, so no triangle of the land reaches across it.
    reach: f64,
}

impl Zone {
    /// The row nearest `p`, looked for within `reach` and past it if need be.
    fn nearest(&self, p: PlanPoint, reach: f64) -> Option<&Row> {
        let distance = |row: &Row| (row.at[0] - p[0]).powi(2) + (row.at[1] - p[1]).powi(2);
        let near = self
            .index
            .covering([p[0] - reach, p[1] - reach], [p[0] + reach, p[1] + reach])
            .map(|&slot| &self.rows[slot])
            .min_by(|a, b| distance(a).total_cmp(&distance(b)));
        near.or_else(|| {
            self.rows
                .iter()
                .min_by(|a, b| distance(a).total_cmp(&distance(b)))
        })
    }
}

impl Zones {
    pub(crate) fn of(map: &Map, cell: f64) -> Zones {
        let mut zones = Vec::new();
        for structure in map.structures.iter() {
            let need = match &structure.kind {
                StructureKind::Tunnel(tunnel) => {
                    Need::Over(tunnel.clear_height + tunnel.lining + COVER)
                }
                StructureKind::Bridge(bridge) => Need::Under(bridge.deck_depth + CLEARANCE),
            };
            let ring: Vec<PlanPoint> = structure
                .outline
                .points()
                .iter()
                .map(|p| [p.x, p.y])
                .collect();
            let mut edges = Segments::new(20.0);
            edges.insert_ring(&ring);
            let bounds = roadgen_core::geometry::plan::bounds(&ring);
            let mut rows = Vec::new();
            for span in &structure.carries {
                let Some(road) = map.road(&span.road) else {
                    continue;
                };
                let margin = 10.0_f64.min(span.length() / 3.0);
                let count = (span.length() / 2.0).ceil().max(1.0) as usize;
                for index in 0..=count {
                    let station = span.from + span.length() * index as f64 / count as f64;
                    let Ok(sample) = road
                        .reference_line
                        .sample_at(station, map.metadata.sampling)
                    else {
                        continue;
                    };
                    let tangent = sample.tangent.get();
                    let mouth = match index {
                        0 => Some([-tangent.x, -tangent.y]),
                        last if last == count => Some([tangent.x, tangent.y]),
                        _ => None,
                    };
                    rows.push(Row {
                        at: [sample.point.x, sample.point.y],
                        z: sample.point.z,
                        interior: station >= span.from + margin && station <= span.to - margin,
                        mouth,
                    });
                }
            }
            let mut index = Grid::new(40.0);
            for (slot, row) in rows.iter().enumerate() {
                index.insert(row.at, row.at, slot);
            }
            if !rows.is_empty() {
                zones.push(Zone {
                    need,
                    ring,
                    edges,
                    bounds,
                    rows,
                    index,
                });
            }
        }
        Zones {
            zones,
            reach: cell * 1.5,
        }
    }

    /// Every outline, for the land's extent.
    pub(crate) fn rings(&self) -> impl Iterator<Item = &Vec<PlanPoint>> {
        self.zones.iter().map(|zone| &zone.ring)
    }

    /// `height` at `(x, y)`, raised over a tunnel and lowered under a bridge.
    pub(crate) fn adjust(&self, x: f64, y: f64, mut height: f64) -> f64 {
        let p = [x, y];
        for zone in &self.zones {
            let far = match zone.need {
                Need::Over(_) => self.reach + SHOULDER,
                Need::Under(_) => self.reach,
            };
            let b = zone.bounds;
            if x < b[0] - far || x > b[2] + far || y < b[1] - far || y > b[3] + far {
                continue;
            }
            let distance = if ring_contains(&zone.ring, p) {
                0.0
            } else {
                match zone.edges.within(p, far) {
                    Some(distance) => distance,
                    None => continue,
                }
            };
            let Some(row) = zone.nearest(p, far) else {
                continue;
            };
            let z = row.z;
            match zone.need {
                Need::Over(rise) => {
                    // In front of a portal the land is the approach's cutting, not
                    // the cover over the bore.
                    if let Some(out) = row.mouth {
                        if (x - row.at[0]) * out[0] + (y - row.at[1]) * out[1] > 0.0 {
                            continue;
                        }
                    }
                    let top = z + rise;
                    let t = smoothstep(self.reach, self.reach + SHOULDER, distance);
                    height = height.max(top * (1.0 - t) + height * t);
                }
                Need::Under(drop) => {
                    if row.interior && distance <= self.reach {
                        height = height.min(z - drop);
                    }
                }
            }
        }
        height
    }
}
