//! Errors raised while lowering onto GPUDrive.

use std::fmt;

use roadgen_core::GeometryError;

#[derive(Debug, Clone, PartialEq)]
pub enum ExportError {
    Geometry(GeometryError),
    /// An agent was asked for but the map has nowhere to drive it.
    NoRoute(String),
    /// The scenario file does not describe a scenario.
    Scenario(String),
    /// The document could not be rendered as JSON, which means this crate is wrong.
    Json(String),
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
            ExportError::NoRoute(detail) => write!(
                f,
                "no agent route could be found: {detail}. A GPUDrive scene is a map \
                 and the agents driving it; a scene with no objects loads, but there \
                 is nothing in it to control"
            ),
            ExportError::Scenario(detail) => write!(f, "the scenario is unusable: {detail}"),
            ExportError::Json(detail) => write!(f, "the GPUDrive scene is malformed: {detail}"),
            ExportError::Io(detail) => write!(f, "{detail}"),
        }
    }
}

impl std::error::Error for ExportError {}
