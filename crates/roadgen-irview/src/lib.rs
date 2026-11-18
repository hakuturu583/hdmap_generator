//! `roadgen-irview` — draws the canonical road IR itself.
//!
//! `roadgen-viewer` reads an exported *file* back and draws it, which is the check
//! that a format says what it should. This crate draws the IR before any format is
//! involved: what the generator built, lane by lane, with the identifiers the IR
//! gives each thing. That makes it what an editor needs — a picture whose every
//! shape can be traced back to the road, lane or object it came from.
//!
//! ```text
//!   Map ──▶ Scene ──┬──▶ SVG
//!                   └──▶ JSON (feature `serde`), for a canvas in a browser
//! ```
//!
//! Surface features — ruts, potholes, puddles, humps, roughness — are drawn as
//! their footprints on the road, with the depth or height they reach.
//!
//! A [`Scene`] is a plan view: x and y in the map's own metres, heights dropped. It
//! keeps no topology beyond the identifiers, and nothing goes back into the IR from
//! here.
//!
//! # Example
//!
//! ```
//! use roadgen_core::prelude::*;
//!
//! let mut builder = MapBuilder::default();
//! let lanes = vec![
//!     LaneSpec::new(PositiveWidth::new(3.5).unwrap(), Direction::Forward),
//!     LaneSpec::new(PositiveWidth::new(3.5).unwrap(), Direction::Backward),
//! ];
//! builder.add_road(RoadSpec::line(
//!     Point3::new(0.0, 0.0, 0.0),
//!     Point3::new(50.0, 0.0, 0.0),
//!     lanes,
//! )?)?;
//! let map = builder.finish()?;
//!
//! let scene = roadgen_irview::Scene::from_map(map.as_map());
//! assert_eq!(scene.lanes.len(), 2);
//! assert!(scene.to_svg(&Default::default()).starts_with("<svg"));
//! # Ok::<(), Box<dyn std::error::Error>>(())
//! ```

pub mod scene;
pub mod svg;

pub use scene::{
    AreaView, Bounds, ContourView, EndView, JunctionView, LaneView, LineView, ObjectView, RoadView,
    Scene, SpanView, StructureView, SurfaceView, TerrainView,
};
pub use svg::SvgOptions;
