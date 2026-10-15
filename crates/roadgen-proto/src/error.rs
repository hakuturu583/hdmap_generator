//! What can go wrong writing or reading a document.

use std::fmt;

/// Why a map could not be written.
#[derive(Debug)]
pub enum WriteError {
    /// A value the schema cannot hold, or a curve that could not be sampled.
    /// `path` is the field, e.g. `lanes[3].left_boundary`.
    Invalid {
        path: String,
        message: String,
    },
    Io(std::io::Error),
}

impl fmt::Display for WriteError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            WriteError::Invalid { path, message } => write!(f, "{path}: {message}"),
            WriteError::Io(error) => write!(f, "could not write the document: {error}"),
        }
    }
}

impl std::error::Error for WriteError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            WriteError::Io(error) => Some(error),
            WriteError::Invalid { .. } => None,
        }
    }
}

/// Why a document could not be read.
#[derive(Debug)]
pub enum ReadError {
    Io(std::io::Error),
    /// The bytes are not a `roadgen_ir.v1.Map` message.
    Decode(prost::DecodeError),
    /// The message says something the IR cannot hold. `path` is the field, e.g.
    /// `roads[2].reference_line.arc.horizontal_length`.
    Invalid {
        path: String,
        message: String,
    },
}

impl fmt::Display for ReadError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            ReadError::Io(error) => write!(f, "could not read the document: {error}"),
            ReadError::Decode(error) => write!(f, "not a roadgen IR document: {error}"),
            ReadError::Invalid { path, message } => write!(f, "{path}: {message}"),
        }
    }
}

impl std::error::Error for ReadError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            ReadError::Io(error) => Some(error),
            ReadError::Decode(error) => Some(error),
            ReadError::Invalid { .. } => None,
        }
    }
}

/// Adds a field to the front of an error's path, as the error travels up out of
/// the message it was found in.
pub(crate) trait Within {
    fn within(self, field: impl fmt::Display) -> Self;
}

/// An error that may name the field it was found at.
pub(crate) trait Located {
    fn path_mut(&mut self) -> Option<&mut String>;
}

impl Located for ReadError {
    fn path_mut(&mut self) -> Option<&mut String> {
        match self {
            ReadError::Invalid { path, .. } => Some(path),
            ReadError::Io(_) | ReadError::Decode(_) => None,
        }
    }
}

impl Located for WriteError {
    fn path_mut(&mut self) -> Option<&mut String> {
        match self {
            WriteError::Invalid { path, .. } => Some(path),
            WriteError::Io(_) => None,
        }
    }
}

impl<T, E: Located> Within for Result<T, E> {
    fn within(self, field: impl fmt::Display) -> Self {
        self.map_err(|mut error| {
            if let Some(path) = error.path_mut() {
                *path = if path.is_empty() {
                    field.to_string()
                } else if path.starts_with('[') {
                    format!("{field}{path}")
                } else {
                    format!("{field}.{path}")
                };
            }
            error
        })
    }
}

/// `name[index]`, as a path segment: formatted only if an error needs it, since
/// every element read or written is named in case it fails.
pub(crate) struct Indexed<'a>(pub &'a str, pub usize);

impl fmt::Display for Indexed<'_> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}[{}]", self.0, self.1)
    }
}

/// An error at the field being read, which the callers above it will name.
pub(crate) fn invalid(message: impl fmt::Display) -> ReadError {
    ReadError::Invalid {
        path: String::new(),
        message: message.to_string(),
    }
}

/// An error at the field being written.
pub(crate) fn unwritable(message: impl fmt::Display) -> WriteError {
    WriteError::Invalid {
        path: String::new(),
        message: message.to_string(),
    }
}
