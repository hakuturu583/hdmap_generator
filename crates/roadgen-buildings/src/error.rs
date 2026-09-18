//! What can go wrong between a grammar and a town.

use std::fmt;

/// Something the caller asked for that cannot be done.
///
/// Every variant is about the *rules*. A lot whose derivation produces nothing, or
/// produces something that will not fit beside the road, is not an error — it is a
/// gap in the street, and [`Report`](crate::Report) counts it. A grammar that fails
/// on *every* lot is the exception, because that is not a street with gaps in it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Error {
    /// The grammar did not parse, or the interpreter refused a statement.
    Grammar {
        /// Line of the rules text the statement starts on, counting from one.
        line: usize,
        statement: String,
        detail: String,
    },
    /// The grammar has no rule to start a lot's derivation at.
    NoRootRule { root: &'static str },
    /// A name was given where a preset was expected, and no preset has it.
    UnknownPreset {
        name: String,
        known: Vec<&'static str>,
    },
    /// The grammar parsed, and then failed to derive a single lot of the map.
    ///
    /// One lot the rules cannot handle is a gap in a street. Every lot is a grammar
    /// that does not work — a name nothing defines, a split wider than any frontage,
    /// a rule that recurses past the depth limit — and reporting that as an empty
    /// town would leave the caller looking at the map for a fault that is in the
    /// rules.
    Derivation { lots: usize, detail: String },
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Error::Grammar {
                line,
                statement,
                detail,
            } => write!(f, "line {line}: {detail}\n  in: {statement}"),
            Error::NoRootRule { root } => write!(
                f,
                "the rules have no `{root}` rule, which is where a lot's derivation \
                 starts; add one, as in `{root} --> Extrude(7) I(\"house\")`"
            ),
            Error::Derivation { lots, detail } => write!(
                f,
                "the rules parsed but derived nothing on any of the {lots} lots of \
                 this map; the first one failed with: {detail}"
            ),
            Error::UnknownPreset { name, known } => write!(
                f,
                "no building rules are called {name:?}; the ones built in are {}, and \
                 anything else has to be a grammar (which contains `-->`)",
                known.join(", ")
            ),
        }
    }
}

impl std::error::Error for Error {}
