//! `roadgen-core` — the Canonical Road IR and the generator that fills it.
//!
//! This crate models a 3D road network so that one description can be written out as
//! OpenDRIVE, as Lanelet2, or as anything added later. It is a *generator*, not a
//! converter: nothing here parses an existing map.
//!
//! # How the pieces fit
//!
//! ```text
//!   Topology ── Geometry ── Semantics
//!        └──────────┬──────────┘
//!             Canonical Road IR
//!                   │
//!               Validation
//!                   │
//!            ┌──────┴──────┐
//!        OpenDRIVE      Lanelet2
//! ```
//!
//! Four separations are load-bearing, and every one of them is visible in the module
//! list: connectivity ([`topology`]) says nothing about coordinates; coordinates
//! ([`geometry`]) say nothing about meaning; meaning ([`semantics`]) says nothing
//! about file formats; and the physical encodings live in their own crates, which
//! depend on this one and are never depended upon by it.
//!
//! # Example
//!
//! ```
//! use roadgen_core::prelude::*;
//!
//! let mut builder = MapBuilder::default();
//! let lanes = || vec![
//!     LaneSpec::new(PositiveWidth::new(3.5).unwrap(), Direction::Forward),
//!     LaneSpec::new(PositiveWidth::new(3.5).unwrap(), Direction::Backward),
//! ];
//! let a = builder.add_road(
//!     RoadSpec::line(Point3::new(0.0, 0.0, 10.0), Point3::new(100.0, 0.0, 12.0), lanes())?
//!         .with_name("a"),
//! )?;
//! let b = builder.add_road(
//!     RoadSpec::line(Point3::new(100.0, 0.0, 12.0), Point3::new(200.0, 50.0, 15.0), lanes())?
//!         .with_name("b"),
//! )?;
//! builder.connect(&a, &b)?;
//!
//! let map = builder.finish()?.validate()?;
//! assert_eq!(map.roads.len(), 2);
//! # Ok::<(), Box<dyn std::error::Error>>(())
//! ```
//!
//! # Design lineage
//!
//! The separation of features from topology, of geometry from semantics, of a
//! conceptual model from its physical encoding, relationships as first-class objects
//! and stable identifiers are ideas taken from GDF 5.1. Only the ideas: there is no
//! GDF feature catalogue, no feature or attribute codes, no GDF XML, and this is not
//! a GDF-conformant implementation.

pub mod arena;
pub mod builder;
pub mod buildings;
pub mod error;
pub mod geometry;
pub mod id;
pub mod map;
pub mod semantics;
pub mod topology;
pub mod units;
pub mod validation;

pub use arena::Arena;
pub use builder::{CrossSectionSpec, LaneRef, LaneSpec, MapBuilder, RoadSpec};
pub use buildings::{Building, Footprint};
pub use error::{BuildError, GeometryError, QuantityError, ValidationError, ValidationIssue};
pub use geometry::{
    Alignment, Arc3, Clothoid3, Curve3, Frame3, Point3, Poly3Piece, Poly3Profile, Polyline3,
    SamplingConfig, Taper, Vector3, WidthProfile,
};
pub use id::{BuildingId, ConnectionId, JunctionId, LaneId, ObjectId, RoadId};
pub use map::{CrossSection, Lane, Map, MapMetadata, Projection, Road, Route, TrafficHandedness};
pub use semantics::{
    BoundaryMarking, LaneType, MapObject, MapObjectKind, MarkingColor, ObjectGeometry, RoadMarking,
    RoadType, TrafficRule,
};
pub use topology::{
    Direction, Junction, LaneConnection, LaneEnd, LaneEndpoint, LateralSide, RoadEnd, RoadEndpoint,
    RoadLink, RoadLinkTarget,
};
pub use units::{GeoOrigin, PositiveWidth, SpeedLimit};
pub use validation::{UnvalidatedMap, ValidatedMap, ValidationConfig};

/// Everything a caller normally needs, in one import.
pub mod prelude {
    pub use crate::builder::{CrossSectionSpec, LaneRef, LaneSpec, MapBuilder, RoadSpec};
    pub use crate::buildings::{Building, Footprint};
    pub use crate::geometry::{
        Alignment, Arc3, Clothoid3, Curve3, Point3, Poly3Piece, Poly3Profile, SamplingConfig,
        Taper, WidthProfile,
    };
    pub use crate::id::{BuildingId, ConnectionId, JunctionId, LaneId, ObjectId, RoadId};
    pub use crate::map::{Map, MapMetadata, Projection, Route, TrafficHandedness};
    pub use crate::semantics::{
        BoundaryMarking, LaneType, MapObjectKind, MarkingColor, ObjectGeometry, RoadMarking,
        RoadType, TrafficRule,
    };
    pub use crate::topology::{Direction, LaneEnd, LateralSide, RoadEnd};
    pub use crate::units::{GeoOrigin, PositiveWidth, SpeedLimit};
    pub use crate::validation::{UnvalidatedMap, ValidatedMap};
}
