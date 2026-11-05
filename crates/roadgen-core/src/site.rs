//! A place made of several features: a mine, a works, a depot.
//!
//! GDF composes complex features out of simpler ones — a road out of road
//! elements, an intersection out of junctions — and a site is that idea applied to
//! a place rather than to a road: the land covers, the terrain surfaces and the
//! roads that together are one quarry. The site lists them, and each of them that
//! has a field for it names the site back, the way a road and its lanes, or a
//! building and its parts, refer to each other.
//!
//! A site owns nothing: take it away and its features are still on the map. It is
//! what a consumer that wants "the mine" asks for, instead of collecting features by
//! the prefix of their names.

use crate::id::{LandCoverId, RoadId, SiteId, TinSurfaceId};

/// A complex feature: the features one place is made of.
#[derive(Debug, Clone, PartialEq)]
pub struct Site {
    pub id: SiteId,
    pub name: Option<String>,
    /// What it is, in the generator's words: `open_pit_mine`.
    pub kind: String,
    pub land_covers: Vec<LandCoverId>,
    pub surfaces: Vec<TinSurfaceId>,
    pub roads: Vec<RoadId>,
}
