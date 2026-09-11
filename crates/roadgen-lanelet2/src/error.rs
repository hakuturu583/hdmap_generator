//! Errors raised while lowering onto Lanelet2.

use std::fmt;

use roadgen_core::GeometryError;

#[derive(Debug, Clone, PartialEq)]
pub enum ExportError {
    Geometry(GeometryError),
    /// The map's origin and projection do not describe a usable coordinate system.
    Projection(String),
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
            ExportError::Unknown(what) => write!(f, "no Lanelet2 primitive was built for {what}"),
            ExportError::Serialization(detail) => {
                write!(f, "the Lanelet2 map could not be written: {detail}")
            }
            ExportError::Io(detail) => write!(f, "{detail}"),
        }
    }
}

impl std::error::Error for ExportError {}
