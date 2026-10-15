//! `roadgen-proto` — the Canonical Road IR as Protocol Buffers.
//!
//! Every other crate in this workspace lowers the IR onto some format and loses
//! something on the way: OpenDRIVE has no buildings, Lanelet2 no clothoids. This
//! one loses nothing. A [`Map`] written here and read back is equal to the map that
//! was written, field for field and in the same order, so a document is the IR
//! itself rather than a picture of it: something to keep, to hand to another
//! process, or to export from later.
//!
//! The one thing that would not survive is the sign of a zero — proto3 writes no
//! field that holds zero, so a `-0.0` would come back as `0.0` — and the IR does
//! not make one.
//!
//! # The schema is the contract
//!
//! The `.proto` files in `proto/roadgen_ir/v1/` are written for whoever reads a
//! document without this crate. They say what every field means, in what unit and
//! against what reference, and every curve can carry its own vertices
//! ([`WriteOptions::curve_samples`]), so an importer in another language compiles
//! the schema with `protoc` and reads polylines without evaluating a clothoid.
//! [`SCHEMA`] holds the same files, for a program that has this crate but not the
//! repository, and [`FILE_DESCRIPTOR_SET`] is them compiled, for a reader that
//! works from descriptors at run time.
//!
//! # Example
//!
//! ```
//! use roadgen_core::prelude::*;
//!
//! let mut builder = MapBuilder::default();
//! builder.add_road(RoadSpec::line(
//!     Point3::new(0.0, 0.0, 0.0),
//!     Point3::new(100.0, 0.0, 0.0),
//!     vec![
//!         LaneSpec::new(PositiveWidth::new(3.5).unwrap(), Direction::Forward),
//!         LaneSpec::new(PositiveWidth::new(3.5).unwrap(), Direction::Backward),
//!     ],
//! )?)?;
//! let map = builder.finish()?.validate()?;
//!
//! let bytes = roadgen_proto::to_bytes(&map)?;
//! let back = roadgen_proto::from_bytes(&bytes)?.validate()?;
//! assert_eq!(back.as_map(), map.as_map());
//! # Ok::<(), Box<dyn std::error::Error>>(())
//! ```

mod decode;
mod encode;
mod error;

use std::path::Path;

use prost::Message;
use roadgen_core::map::Map;
use roadgen_core::validation::UnvalidatedMap;

pub use error::{ReadError, WriteError};

/// The generated message types, one per message of the schema.
pub mod v1 {
    include!(concat!(env!("OUT_DIR"), "/roadgen_ir.v1.rs"));
}

/// The schema's files, by their import path, in dependency order.
pub const SCHEMA: &[(&str, &str)] = include!(concat!(env!("OUT_DIR"), "/schema.rs"));

/// The schema compiled to a `google.protobuf.FileDescriptorSet`, encoded.
pub const FILE_DESCRIPTOR_SET: &[u8] =
    include_bytes!(concat!(env!("OUT_DIR"), "/roadgen_ir_v1_descriptor.bin"));

/// The format a reader's trace names, as the other formats' crates name theirs.
pub const TRACE_FORMAT: &str = "roadgen_proto";

/// The fully qualified name of a document's top-level message.
pub const MAP_MESSAGE: &str = "roadgen_ir.v1.Map";

/// What a writer can be asked to add beyond the IR itself.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct WriteOptions {
    /// Whether every curve carries `samples`: itself evaluated at the map's
    /// sampling resolution, the vertices every polyline export of the map gets.
    ///
    /// On by default, because it is what makes a document easy to read anywhere
    /// else. The reader here ignores them, so turning it off loses nothing but the
    /// convenience, and makes a document mostly of lane polylines about half the
    /// size.
    pub curve_samples: bool,
}

impl Default for WriteOptions {
    fn default() -> Self {
        WriteOptions {
            curve_samples: true,
        }
    }
}

/// The map as a message.
pub fn to_message(map: &Map) -> Result<v1::Map, WriteError> {
    to_message_with(map, &WriteOptions::default())
}

/// The map as a message, with the caller's options.
pub fn to_message_with(map: &Map, options: &WriteOptions) -> Result<v1::Map, WriteError> {
    encode::map(map, options)
}

/// The map as an encoded document.
pub fn to_bytes(map: &Map) -> Result<Vec<u8>, WriteError> {
    to_bytes_with(map, &WriteOptions::default())
}

/// The map as an encoded document, with the caller's options.
pub fn to_bytes_with(map: &Map, options: &WriteOptions) -> Result<Vec<u8>, WriteError> {
    Ok(to_message_with(map, options)?.encode_to_vec())
}

/// Writes the map to `path` as an encoded document.
pub fn write(map: &Map, path: impl AsRef<Path>) -> Result<(), WriteError> {
    write_with(map, path, &WriteOptions::default())
}

/// Writes the map to `path` as an encoded document, with the caller's options.
pub fn write_with(
    map: &Map,
    path: impl AsRef<Path>,
    options: &WriteOptions,
) -> Result<(), WriteError> {
    std::fs::write(path, to_bytes_with(map, options)?).map_err(WriteError::Io)
}

/// The map a message describes, not yet validated.
///
/// Every value is rebuilt through the IR's own constructors, so a message that
/// says something the IR cannot hold — a zero lane width, a footprint with no
/// area — is refused here, naming the field. Whether the result is a consistent
/// road network, every reference resolving and every lane meeting its neighbour,
/// is [`UnvalidatedMap::validate`]'s question, as it is for any other map.
pub fn from_message(message: &v1::Map) -> Result<UnvalidatedMap, ReadError> {
    decode::map(message).map(UnvalidatedMap::from_map)
}

/// The map an encoded document describes, not yet validated.
pub fn from_bytes(bytes: &[u8]) -> Result<UnvalidatedMap, ReadError> {
    let message = v1::Map::decode(bytes).map_err(ReadError::Decode)?;
    from_message(&message)
}

/// Reads the document at `path`.
pub fn read(path: impl AsRef<Path>) -> Result<UnvalidatedMap, ReadError> {
    let bytes = std::fs::read(path).map_err(ReadError::Io)?;
    from_bytes(&bytes)
}
