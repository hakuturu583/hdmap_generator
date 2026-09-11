//! Canonical geometry: three-dimensional from the start.
//!
//! There is no 2D type in this module, and nothing here silently drops a `z`. Where
//! a planar computation is genuinely required — an OpenDRIVE `s` coordinate, a lane
//! offset — the projection goes through [`Frame3::to_local`] or through a method
//! whose name says `horizontal`, so it is always visible at the call site.

pub mod curve;
pub mod polyline;
pub mod vector;

pub use curve::{Arc3, Bezier3, Curve3, Line3, Sample, SamplingConfig};
pub use polyline::Polyline3;
pub use vector::{Frame3, Point3, UnitVector3, Vector3};
