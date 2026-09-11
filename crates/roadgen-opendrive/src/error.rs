//! Errors raised while lowering onto OpenDRIVE.

use std::fmt;

use roadgen_core::GeometryError;

#[derive(Debug, Clone, PartialEq)]
pub enum ExportError {
    Geometry(GeometryError),
    /// An identifier in the IR has no OpenDRIVE number, which means the map changed
    /// under the exporter.
    Unknown(String),
    /// OpenDRIVE requires at least one of something the map has none of.
    Empty(String),
    /// The map uses something OpenDRIVE cannot express.
    Unsupported(String),
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
            ExportError::Unknown(what) => write!(f, "no OpenDRIVE id was assigned to {what}"),
            ExportError::Empty(what) => write!(f, "{what}"),
            ExportError::Unsupported(what) => write!(f, "OpenDRIVE cannot express {what}"),
            ExportError::Serialization(detail) => {
                write!(f, "the OpenDRIVE document could not be written: {detail}")
            }
            ExportError::Io(detail) => write!(f, "{detail}"),
        }
    }
}

impl std::error::Error for ExportError {}
