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
    /// The carried road down its middle: where, how high, and whether the place is
    /// clear of the structure's ends.
    rows: Vec<(PlanPoint, f64, bool)>,
}

/// Every structure's demand on the land.
pub(crate) struct Zones {
    zones: Vec<Zone>,
    /// How far past an outline its demand reaches in full: about a cell and a
    /// half, so no triangle of the land reaches across it.
    reach: f64,
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
            let bounds = ring.iter().fold(
                [
                    f64::INFINITY,
                    f64::INFINITY,
                    f64::NEG_INFINITY,
                    f64::NEG_INFINITY,
                ],
                |b, p| {
                    [
                        b[0].min(p[0]),
                        b[1].min(p[1]),
                        b[2].max(p[0]),
                        b[3].max(p[1]),
                    ]
                },
            );
            let mut rows = Vec::new();
            for span in &structure.carries {
                let Some(road) = map.road(&span.road) else {
                    continue;
                };
                let margin = 10.0_f64.min(span.length() / 3.0);
                let count = (span.length() / 2.0).ceil().max(1.0) as usize;
                for index in 0..=count {
                    let station = span.from + span.length() * index as f64 / count as f64;
                    if let Ok(sample) = road
                        .reference_line
                        .sample_at(station, map.metadata.sampling)
                    {
                        let interior = station >= span.from + margin && station <= span.to - margin;
                        rows.push(([sample.point.x, sample.point.y], sample.point.z, interior));
                    }
                }
            }
            if !rows.is_empty() {
                zones.push(Zone {
                    need,
                    ring,
                    edges,
                    bounds,
                    rows,
                });
            }
        }
        Zones {
            zones,
            reach: cell * 1.5,
        }
    }

    pub(crate) fn is_empty(&self) -> bool {
        self.zones.is_empty()
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
            let Some(&(_, z, interior)) = zone.rows.iter().min_by(|a, b| {
                let da = (a.0[0] - x).powi(2) + (a.0[1] - y).powi(2);
                let db = (b.0[0] - x).powi(2) + (b.0[1] - y).powi(2);
                da.total_cmp(&db)
            }) else {
                continue;
            };
            match zone.need {
                Need::Over(rise) => {
                    let top = z + rise;
                    let t = smoothstep(self.reach, self.reach + SHOULDER, distance);
                    height = height.max(top * (1.0 - t) + height * t);
                }
                Need::Under(drop) => {
                    if interior && distance <= self.reach {
                        height = height.min(z - drop);
                    }
                }
            }
        }
        height
    }
}
