//! Errors raised while lowering onto RDS-HQ.

use std::fmt;

use roadgen_core::GeometryError;

#[derive(Debug, Clone, PartialEq)]
pub enum ExportError {
    Geometry(GeometryError),
    /// A clip id that cannot be a file name in every folder.
    InvalidClipId(String),
    /// The clip could not be configured or driven: a route that goes
    /// nowhere, a scenario file that does not parse. ClipGT and RDS-HQ share the
    /// scenario, so they share what can go wrong with it.
    Clip(roadgen_clipgt::ExportError),
    /// A camera whose lens does not fit the intrinsics array a reader expects.
    Camera(String),
    Io(String),
}

impl From<GeometryError> for ExportError {
    fn from(value: GeometryError) -> Self {
        ExportError::Geometry(value)
    }
}

impl From<roadgen_clipgt::ExportError> for ExportError {
    fn from(value: roadgen_clipgt::ExportError) -> Self {
        match value {
            roadgen_clipgt::ExportError::Geometry(error) => ExportError::Geometry(error),
            other => ExportError::Clip(other),
        }
    }
}

impl fmt::Display for ExportError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            ExportError::Geometry(error) => write!(f, "{error}"),
            ExportError::InvalidClipId(id) => write!(
                f,
                "{id:?} cannot be a clip id: an RDS-HQ clip is a file named \
                 {{clip_id}}.tar in every folder, and a WebDataset reader takes \
                 everything before a member's first dot as the clip, so the id has to \
                 be a single path component with no separators and no dots"
            ),
            ExportError::Clip(error) => write!(f, "{error}"),
            ExportError::Camera(detail) => write!(f, "the camera is unusable: {detail}"),
            ExportError::Io(detail) => write!(f, "{detail}"),
        }
    }
}

impl std::error::Error for ExportError {}
