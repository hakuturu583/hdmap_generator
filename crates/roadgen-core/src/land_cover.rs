//! What the land is used for, or covered with.
//!
//! GDF's Land Cover and Use theme, taken for its idea: an area of land that is one
//! thing — a quarry's pit, a spoil heap, a stockpile, a yard — as a feature of its
//! own, with what it *is* kept apart from the shape of the ground it is on.
//!
//! The shape is the terrain's ([`crate::terrain`]); a land cover only says where its
//! area is and which surface that area lies on, so that a consumer that colours the
//! ground, labels it or decides what it is made of reads it from here rather than
//! guessing from the slopes.

use crate::buildings::Footprint;
use crate::id::{LandCoverId, SiteId, TinSurfaceId};

/// One area of land that is one thing.
#[derive(Debug, Clone, PartialEq)]
pub struct LandCover {
    pub id: LandCoverId,
    /// The site it is part of, when it is part of one.
    pub site: Option<SiteId>,
    /// What it is, in the generator's words: `pit`, `waste_dump`, `stockpile`,
    /// `pad`. The IR keeps no catalogue of these, the way it keeps none of
    /// building kinds.
    pub kind: String,
    /// Its outline, at the land's height along it.
    pub area: Footprint,
    /// The surface the area lies on — a relationship, kept as an object rather
    /// than inferred from the outline — or `None` where no terrain was stated.
    pub draped_on: Option<TinSurfaceId>,
}
