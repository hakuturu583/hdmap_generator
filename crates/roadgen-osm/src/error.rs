//! Errors raised while lowering onto OpenStreetMap.

use std::fmt;

use roadgen_core::GeometryError;

#[derive(Debug, Clone, PartialEq)]
pub enum ExportError {
    Geometry(GeometryError),
    /// The map's origin and projection do not describe a usable coordinate system.
    Projection(String),
    /// A reference in the IR had nothing behind it, which means the map changed
    /// under the exporter.
    Unknown(String),
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
            ExportError::Unknown(what) => write!(f, "no OSM primitive was built for {what}"),
            ExportError::Io(detail) => write!(f, "{detail}"),
        }
    }
}

impl std::error::Error for ExportError {}
