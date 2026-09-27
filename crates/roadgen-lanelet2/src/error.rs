//! Errors raised while lowering onto Lanelet2.

use std::fmt;

use roadgen_core::{GeometryError, QuantityError};

#[derive(Debug, Clone, PartialEq)]
pub enum ExportError {
    Geometry(GeometryError),
    /// The map's origin and projection do not describe a usable coordinate system.
    Projection(String),
    /// The map does not fit inside the MGRS square its origin falls in.
    GridCrossed {
        code: String,
        detail: String,
    },
    /// An identifier in the IR has no Lanelet2 primitive, which means the map
    /// changed under the exporter.
    Unknown(String),
    Serialization(String),
    Io(String),
}

impl From<GeometryError> for ExportError {
    fn from(value: GeometryError) -> Self {
        ExportError::Geometry(value)
    }
}

impl fmt::Display for ExportError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            ExportError::Geometry(error) => write!(f, "{error}"),
            ExportError::Projection(detail) => write!(f, "the map cannot be projected: {detail}"),
            ExportError::GridCrossed { code, detail } => write!(
                f,
                "the map does not fit inside MGRS square {code}: {detail}. A map with \
                 MGRS coordinates has to lie within one 100 km square; move the origin, \
                 split the map, or use the local_cartesian projection"
            ),
            ExportError::Unknown(what) => write!(f, "no Lanelet2 primitive was built for {what}"),
            ExportError::Serialization(detail) => {
                write!(f, "the Lanelet2 map could not be written: {detail}")
            }
            ExportError::Io(detail) => write!(f, "{detail}"),
        }
    }
}

impl std::error::Error for ExportError {}

/// Errors raised while reading a Lanelet2 map into the IR.
///
/// Only what stops the file being read as a map at all is an error. A lanelet the
/// IR cannot hold is left out and reported through [`crate::Imported`]'s
/// approximations instead, the way an exporter's `check` reports what it drops.
#[derive(Debug, Clone, PartialEq)]
pub enum ImportError {
    /// The file is not OSM XML the parser accepts.
    Parse(String),
    Geometry(GeometryError),
    Quantity(QuantityError),
    /// The file says something the IR cannot hold and the reader has no way to
    /// approximate.
    Unsupported(String),
    /// The file contradicts itself: a lanelet whose boundary names a way that is
    /// not there, a way that names a node that is not there.
    Inconsistent(String),
    Io(String),
}

impl From<GeometryError> for ImportError {
    fn from(value: GeometryError) -> Self {
        ImportError::Geometry(value)
    }
}

impl From<QuantityError> for ImportError {
    fn from(value: QuantityError) -> Self {
        ImportError::Quantity(value)
    }
}

impl fmt::Display for ImportError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            ImportError::Parse(detail) => write!(f, "the file is not OSM XML: {detail}"),
            ImportError::Geometry(error) => write!(f, "{error}"),
            ImportError::Quantity(error) => write!(f, "{error}"),
            ImportError::Unsupported(what) => write!(f, "the IR cannot hold {what}"),
            ImportError::Inconsistent(what) => write!(f, "the map contradicts itself: {what}"),
            ImportError::Io(detail) => write!(f, "{detail}"),
        }
    }
}

impl std::error::Error for ImportError {}
