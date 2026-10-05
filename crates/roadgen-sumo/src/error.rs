//! Errors raised while lowering onto a SUMO network.

use std::fmt;

use roadgen_core::GeometryError;

#[derive(Debug, Clone, PartialEq)]
pub enum ExportError {
    Geometry(GeometryError),
    /// A reference in the IR had nothing behind it, which means the map changed
    /// under the exporter.
    Unknown(String),
    /// The map's origin, or a point of the network, cannot be projected — off the
    /// UTM grid, say — so the network's `<location>` cannot be written.
    Projection(String),
    Io(String),
    /// An export option out of its range.
    Option(String),
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
            ExportError::Unknown(what) => write!(f, "no SUMO element was built for {what}"),
            ExportError::Projection(detail) => {
                write!(f, "the network cannot be georeferenced: {detail}")
            }
            ExportError::Io(detail) | ExportError::Option(detail) => write!(f, "{detail}"),
        }
    }
}

impl std::error::Error for ExportError {}
