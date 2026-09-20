//! Errors raised while lowering onto OpenDRIVE.

use std::fmt;

use roadgen_core::{GeometryError, QuantityError};

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
    /// The map's origin cannot be projected — off the UTM grid, say.
    Projection(String),
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
            ExportError::Projection(detail) => {
                write!(f, "the map's origin cannot be georeferenced: {detail}")
            }
            ExportError::Serialization(detail) => {
                write!(f, "the OpenDRIVE document could not be written: {detail}")
            }
            ExportError::Io(detail) => write!(f, "{detail}"),
        }
    }
}

impl std::error::Error for ExportError {}

/// Errors raised while reading an OpenDRIVE document back into the IR.
#[derive(Debug, Clone, PartialEq)]
pub enum ImportError {
    /// The document is not OpenDRIVE the parser accepts.
    Parse(String),
    /// A piece of the document's geometry cannot be built as the IR's.
    Geometry(GeometryError),
    /// A number in the document is outside what the IR's units allow.
    Quantity(QuantityError),
    /// The document says something the IR cannot hold and the reader has no way
    /// to approximate. Named by the element it was found in.
    Unsupported(String),
    /// The document contradicts itself: a link to a road that is not there, a lane
    /// link to a lane the neighbour does not have.
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
            ImportError::Parse(detail) => write!(f, "the document is not OpenDRIVE: {detail}"),
            ImportError::Geometry(error) => write!(f, "{error}"),
            ImportError::Quantity(error) => write!(f, "{error}"),
            ImportError::Unsupported(what) => write!(f, "the IR cannot hold {what}"),
            ImportError::Inconsistent(what) => write!(f, "the document contradicts itself: {what}"),
            ImportError::Io(detail) => write!(f, "{detail}"),
        }
    }
}

impl std::error::Error for ImportError {}
