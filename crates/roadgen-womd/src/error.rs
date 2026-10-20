//! Errors raised while lowering onto a WOMD scenario.

use std::fmt;

use roadgen_core::GeometryError;

#[derive(Debug, Clone, PartialEq)]
pub enum ExportError {
    Geometry(GeometryError),
    /// The configuration asks for a scenario WOMD cannot describe.
    Config(String),
    /// A TFRecord file that is not one, or not whole.
    TfRecord(String),
    /// A record that does not decode as a `Scenario`.
    Decode(String),
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
            ExportError::TfRecord(detail) => write!(f, "not a TFRecord file: {detail}"),
            ExportError::Decode(detail) => write!(f, "not a WOMD scenario: {detail}"),
            ExportError::Io(detail) => write!(f, "{detail}"),
        }
    }
}

impl std::error::Error for ExportError {}
