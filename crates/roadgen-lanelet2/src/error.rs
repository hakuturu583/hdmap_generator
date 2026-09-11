//! Errors raised while lowering onto Lanelet2.

use std::fmt;

use roadgen_core::GeometryError;

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
