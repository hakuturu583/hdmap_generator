//! Errors raised while lowering onto a CommonRoad scenario.

use std::fmt;

use roadgen_core::GeometryError;

#[derive(Debug, Clone, PartialEq)]
pub enum ExportError {
    Geometry(GeometryError),
    /// The configuration asks for a scenario CommonRoad cannot describe.
    Config(String),
    /// The map's origin cannot be described in its projection.
    Projection(String),
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
            ExportError::Config(detail) => write!(f, "the scenario is unusable: {detail}"),
            ExportError::Projection(detail) => write!(f, "cannot georeference the map: {detail}"),
            ExportError::Io(detail) => write!(f, "{detail}"),
        }
    }
}

impl std::error::Error for ExportError {}
