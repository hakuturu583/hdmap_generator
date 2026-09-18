//! Errors raised while reading an exported file back.

use std::fmt;

#[derive(Debug, Clone, PartialEq)]
pub enum ViewError {
    /// The file is not the format it was handed to.
    Parse(String),
    /// The file parses, but a field the drawing needs is missing or malformed.
    /// Separate from [`ViewError::Parse`] because it means the writer and the reader
    /// disagree rather than that the bytes are damaged.
    Shape(String),
    /// A directory of files was handed over with one of them missing.
    Missing(String),
    Io(String),
}

impl fmt::Display for ViewError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            ViewError::Parse(detail) => write!(f, "the file could not be read: {detail}"),
            ViewError::Shape(detail) => write!(f, "the file is not shaped as expected: {detail}"),
            ViewError::Missing(what) => write!(f, "{what} is not there"),
            ViewError::Io(detail) => write!(f, "{detail}"),
        }
    }
}

impl std::error::Error for ViewError {}
