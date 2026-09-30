//! Errors raised while writing, reading or joining traces.

use std::fmt;
use std::path::Path;

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum TraceError {
    Io(String),
    /// A file that is not what it says it is.
    Parse(String, String),
    Schema {
        path: String,
        found: String,
        expected: &'static str,
    },
    /// A file a trace describes has changed since the trace was written, so its
    /// numbers may now name different things.
    Stale {
        trace: String,
        file: String,
    },
    /// An IR dump whose body no longer hashes to the fingerprint it states: edited or
    /// damaged since it was written, so the traces it would be joined to describe a
    /// different map from the one it now holds.
    Altered {
        path: String,
        stated: String,
        actual: String,
    },
    /// Two files that were to be joined came from different maps.
    Mismatch {
        path: String,
        found: String,
        expected: String,
    },
    /// A second trace of a format already loaded.
    Duplicate(String),
    /// A lookup needed a trace that has not been loaded.
    Missing(String),
}

impl TraceError {
    pub(crate) fn io(path: &Path, error: std::io::Error) -> Self {
        TraceError::Io(format!("{}: {error}", path.display()))
    }
}

impl fmt::Display for TraceError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            TraceError::Io(detail) => write!(f, "{detail}"),
            TraceError::Parse(path, detail) => write!(f, "{path}: {detail}"),
            TraceError::Schema {
                path,
                found,
                expected,
            } => write!(f, "{path}: schema `{found}`, expected `{expected}`"),
            TraceError::Stale { trace, file } => write!(
                f,
                "{file} has changed since {trace} was written; export it again to get a \
                 trace that matches"
            ),
            TraceError::Altered {
                path,
                stated,
                actual,
            } => write!(
                f,
                "{path} states the fingerprint {stated} but its contents hash to {actual}; \
                 it has been changed since it was written"
            ),
            TraceError::Mismatch {
                path,
                found,
                expected,
            } => write!(
                f,
                "{path} was written from a different map ({found}, expected {expected})"
            ),
            TraceError::Duplicate(format) => {
                write!(f, "a `{format}` trace is already loaded")
            }
            TraceError::Missing(format) => write!(f, "no `{format}` trace is loaded"),
        }
    }
}

impl std::error::Error for TraceError {}
