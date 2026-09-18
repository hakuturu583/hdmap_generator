//! `roadgen-viewer` — draws what roadgen wrote.
//!
//! Every other crate here writes a file. This one reads one back and turns it into a
//! picture, which makes it the only place in the workspace where an export is looked
//! at rather than produced — and the only check that runs over the bytes a consumer
//! would actually receive.
//!
//! ```text
//!   map.xodr    sumo/     Town01.fbx    clip/      scene.json
//!      │          │           │           │            │
//!   OpenDRIVE   SUMO       CARLA       ClipGT      GPUDrive
//!      └──────────┴─────┬─────┴───────────┴────────────┘
//!                    Drawing
//!                       │
//!                      SVG
//! ```
//!
//! # Why these five
//!
//! Because the other two have viewers already. A Lanelet2 map and a plain
//! OpenStreetMap file are both OSM XML in geographic coordinates, so any map library
//! draws them, and the demo page hands them to Leaflet rather than to this. What is
//! here is the formats with nothing to hand them to: an OpenDRIVE document whose
//! shape has to be evaluated before it can be drawn, SUMO's plain XML, a CARLA
//! package's FBX, ClipGT's Parquet layers, and a GPUDrive scene.
//!
//! # It reads, it does not convert
//!
//! A [`Drawing`] is a plan view and a small vocabulary of road things. It drops the
//! heights, it keeps no topology, and nothing goes back into the IR from here — so
//! this is not a route back from a file to a map, and reading a clip in and writing it
//! out again would lose most of it. What it is for is looking.
//!
//! # Example
//!
//! ```
//! # fn main() -> Result<(), roadgen_viewer::ViewError> {
//! let scene = r#"{"name":"t","scenario_id":"t","objects":[],"roads":[
//!     {"type":"lane","geometry":[{"x":0.0,"y":0.0},{"x":10.0,"y":0.0}],
//!      "id":1,"map_element_id":2}],
//!     "metadata":{"sdc_track_index":-1,"tracks_to_predict":[],"objects_of_interest":[]}}"#;
//! let svg = roadgen_viewer::gpudrive::draw(scene)?.to_svg();
//! assert!(svg.starts_with("<svg"));
//! # Ok(())
//! # }
//! ```

pub mod clipgt;
pub mod drawing;
pub mod error;
pub mod fbx;
pub mod gpudrive;
pub mod opendrive;
pub mod read;
pub mod sumo;
pub mod svg;

pub use clipgt::Clip;
pub use drawing::{Bounds, Drawing, Kind, Mark, MarkColor, Point, Shape};
pub use error::ViewError;
pub use sumo::Network;

use std::path::Path;

/// Draws an OpenDRIVE file.
pub fn opendrive_file(path: &Path) -> Result<Drawing, ViewError> {
    opendrive::draw(&read::text(path)?)
}

/// Draws the SUMO plain-XML network in a directory.
pub fn sumo_directory(path: &Path) -> Result<Drawing, ViewError> {
    sumo::draw(&read::sumo(path)?)
}

/// Draws the ClipGT clip in a directory.
pub fn clipgt_directory(path: &Path) -> Result<Drawing, ViewError> {
    clipgt::draw(&read::clipgt(path)?)
}

/// Draws a GPUDrive scene file.
pub fn gpudrive_file(path: &Path) -> Result<Drawing, ViewError> {
    gpudrive::draw(&read::text(path)?)
}

/// Draws the FBX of a CARLA package.
pub fn fbx_file(path: &Path) -> Result<Drawing, ViewError> {
    fbx::draw(&read::text(path)?)
}
