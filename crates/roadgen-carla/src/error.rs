//! Errors raised while writing a CARLA package.

use std::fmt;

use roadgen_core::GeometryError;

#[derive(Debug, Clone, PartialEq)]
pub enum ExportError {
    Geometry(GeometryError),
    /// The map's name cannot be used, and no name can be made from it.
    Name(String),
    /// The OpenDRIVE beside the meshes could not be written. A CARLA map is the pair;
    /// a package with one of them is not a map.
    OpenDrive(String),
    /// The package descriptor could not be rendered as JSON, which means this crate
    /// is wrong.
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
            ExportError::Name(detail) => write!(
                f,
                "the map cannot be named for CARLA: {detail}. Every mesh in the FBX \
                 is named after the map, and CARLA reads those names to decide what \
                 each mesh is"
            ),
            ExportError::OpenDrive(detail) => write!(
                f,
                "the OpenDRIVE beside the meshes could not be written: {detail}. A \
                 CARLA map is an .fbx and an .xodr of the same name; the mesh alone \
                 is scenery"
            ),
            ExportError::Json(detail) => {
                write!(f, "the package descriptor is malformed: {detail}")
            }
            ExportError::Io(detail) => write!(f, "{detail}"),
        }
    }
}

impl std::error::Error for ExportError {}
