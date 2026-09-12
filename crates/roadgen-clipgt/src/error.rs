//! Errors raised while lowering onto ClipGT.

use std::fmt;

use roadgen_core::GeometryError;

#[derive(Debug, Clone, PartialEq)]
pub enum ExportError {
    Geometry(GeometryError),
    /// A clip id that cannot be part of a file name.
    InvalidClipId(String),
    /// An ego track was asked for but the map has nowhere to drive.
    NoRoute(String),
    /// The scenario file does not describe a scenario.
    Scenario(String),
    /// The Arrow schema and the data disagreed, which means this crate is wrong.
    Schema(String),
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
            ExportError::InvalidClipId(id) => write!(
                f,
                "{id:?} cannot be a clip id: a ClipGT clip names every one of its \
                 files {{clip_id}}.{{layer}}.parquet, so the id has to be a single \
                 path component with no separators and no dots"
            ),
            ExportError::NoRoute(detail) => write!(
                f,
                "no ego route could be found: {detail}. ClipGT needs an egomotion \
                 track before a reader will accept the directory as a clip"
            ),
            ExportError::Scenario(detail) => write!(f, "the scenario is unusable: {detail}"),
            ExportError::Schema(detail) => write!(f, "the ClipGT tables are malformed: {detail}"),
            ExportError::Io(detail) => write!(f, "{detail}"),
        }
    }
}

impl std::error::Error for ExportError {}
