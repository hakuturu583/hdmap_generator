//! The rules: one piece of CGA text that says everything.
//!
//! # Why the layout is in the grammar too
//!
//! A shape grammar describes what stands on a lot. It cannot describe where the lot
//! is, because a lot here is cut out of a road's frontage and the grammar has never
//! heard of a road. That leaves two places a caller could say "sixteen metres of
//! frontage, five metres back from the kerb" — as arguments beside the grammar, or as
//! `attr` declarations inside it.
//!
//! It is the second, because then there is one thing to pass around and one thing to
//! edit, and because the numbers are then visible to the grammar as well: a rule can
//! read `LotDepth` and set a building's footprint against it. So [`Rules`] is a
//! string, and everything else is read out of it.
//!
//! ```text
//! attr Setback = 5             // kerb to lot, metres
//! attr LotWidth = 15           // frontage per lot
//! attr LotDepth = 18           // how far back a lot reaches
//! attr LotGap = 4              // left clear between neighbours
//! attr CornerClearance = 12    // left clear at each end of a frontage
//! attr FloorHeight = 3.2       // what a storey is, for the storey count
//!
//! Lot   --> Size(scope.x - 2, 0, scope.z - 2) Center(XZ) Plot
//! Plot  --> 60% House | else: Shop
//! House --> Extrude(rand(6, 9)) I("house")
//! Shop  --> Extrude(FloorHeight * 2) I("retail")
//! ```
//!
//! Derivation starts at the rule named [`ROOT`], and the word a mass is emitted with
//! — `house`, `retail` — becomes the building's
//! [`kind`](roadgen_core::buildings::Building::kind).
//!
//! # Splitting the text into statements
//!
//! `symbios-shape` parses one statement at a time and has no whole-file parser, so
//! the text has to be cut up before it can be handed over. A statement starts at a
//! line that declares an `attr`, a `const` or a `style`, or that heads a rule
//! (`Name -->`, `Name(a, b) -->`); every other line continues the statement before
//! it. That is what lets a rule's body run over as many lines as it reads best on.

use std::collections::BTreeMap;

use symbios_shape::grammar::{parse_statement, Statement};
use symbios_shape::Interpreter;

use crate::error::Error;
use crate::land::Layout;

/// The rule a lot's derivation starts at.
pub const ROOT: &str = "Lot";

/// Rules built in, by name. The first is what [`Rules::default`] uses.
pub const PRESETS: &[(&str, &str)] = &[
    ("town", TOWN),
    ("suburban", SUBURBAN),
    ("downtown", DOWNTOWN),
    ("industrial", INDUSTRIAL),
];

/// A mixed street: houses, short terraces and the odd shop.
const TOWN: &str = r#"// A mixed street, which is what most of a town is.
attr Setback = 5
attr LotWidth = 15
attr LotDepth = 18
attr LotGap = 4
attr CornerClearance = 12
attr FloorHeight = 3.2

// Two metres of garden all round, so that neighbours do not meet at the fence.
Lot     --> Size(scope.x - 2, 0, scope.z - 2) Center(XZ) Plot
Plot    --> 45% House | 30% Terrace | 15% Shop | else: Block

House   --> Extrude(rand(6, 9)) I("house")
Terrace --> Split(X) { ~1: Unit | ~1: Unit }
Unit    --> Extrude(rand(6.5, 8.5)) I("terrace")
Shop    --> Extrude(FloorHeight * 2) I("retail")
Block   --> Extrude(FloorHeight * rand(3, 5)) I("apartments")
"#;

/// Detached houses on generous plots, well back from the kerb.
const SUBURBAN: &str = r#"// Detached houses on generous plots.
attr Setback = 7
attr LotWidth = 18
attr LotDepth = 22
attr LotGap = 6
attr CornerClearance = 14
attr FloorHeight = 3.0

Lot      --> Size(scope.x - 4, 0, scope.z - 8) Center(XZ) Plot
Plot     --> 70% Detached | else: WithWing

Detached --> Extrude(rand(5.5, 8.0)) I("house")
// An L plan: a front range with a wing behind it, which is two footprints.
WithWing --> ShapeL(rand(7, 9), rand(6, 8)) { Shape: Wing | Remainder: NIL }
Wing     --> Extrude(rand(5.5, 7.5)) I("house")
"#;

/// A terraced high street, built to the pavement and going up.
const DOWNTOWN: &str = r#"// A high street: built to the pavement, and going up.
attr Setback = 2
attr LotWidth = 18
attr LotDepth = 24
attr LotGap = 0
attr CornerClearance = 9
attr FloorHeight = 3.6

Lot      --> Size(scope.x, 0, scope.z - 4) Center(XZ) Frontage
Frontage --> 45% MidRise | 25% Tower | else: Retail

MidRise  --> Extrude(FloorHeight * rand(4, 7)) I("commercial")
// A tower stands on a smaller plan than its neighbours and goes up instead of out.
Tower    --> Size(scope.x - 4, 0, scope.z - 4) Center(XZ) Shaft
Shaft    --> Extrude(FloorHeight * rand(9, 16)) I("office")
Retail   --> Extrude(FloorHeight * rand(1, 3)) I("retail")
"#;

/// Sheds on large plots, with room to turn a lorry.
const INDUSTRIAL: &str = r#"// Sheds on large plots, with room to turn a lorry.
attr Setback = 9
attr LotWidth = 42
attr LotDepth = 36
attr LotGap = 12
attr CornerClearance = 18
attr FloorHeight = 6.0

Lot  --> Size(scope.x - 10, 0, scope.z - 12) Center(XZ) Yard
Yard --> 75% Shed | else: Pair

Shed --> Extrude(rand(7, 11)) I("industrial")
Pair --> Split(X) { ~1: Half | 6: NIL | ~1: Half }
Half --> Extrude(rand(6, 9)) I("warehouse")
"#;

/// The grammar for `name`, if there is one built in.
pub fn preset(name: &str) -> Option<&'static str> {
    PRESETS
        .iter()
        .find(|(preset, _)| preset.eq_ignore_ascii_case(name))
        .map(|(_, grammar)| *grammar)
}

/// What buildings to generate, said once.
///
/// Either a built-in preset by name or a CGA grammar of the caller's own — and the
/// second is the first with the text edited, because [`Rules::source`] hands the
/// preset's own text back.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Rules {
    source: String,
    seed: u64,
}

impl Default for Rules {
    /// The first preset: a mixed town street.
    fn default() -> Self {
        Rules::from_grammar(PRESETS[0].1)
    }
}

impl Rules {
    /// The named preset, or `None` if there is no such preset.
    pub fn preset(name: &str) -> Option<Rules> {
        preset(name).map(Rules::from_grammar)
    }

    /// Rules written out in full, as CGA text.
    pub fn from_grammar(source: impl Into<String>) -> Rules {
        Rules {
            source: source.into(),
            seed: 0,
        }
    }

    /// The named preset if `name` is one, and rules written out in full otherwise.
    ///
    /// A single-token argument that is not a preset name is taken as a name and
    /// refused, because a grammar is never one word and a misspelt preset would
    /// otherwise become an empty grammar that generates nothing and says nothing.
    pub fn parse(text: &str) -> Result<Rules, Error> {
        let trimmed = text.trim();
        if let Some(rules) = Rules::preset(trimmed) {
            return Ok(rules);
        }
        if !trimmed.contains("-->") {
            return Err(Error::UnknownPreset {
                name: trimmed.to_owned(),
                known: PRESETS.iter().map(|(name, _)| *name).collect(),
            });
        }
        Ok(Rules::from_grammar(text))
    }

    /// The same rules, derived from `seed`.
    ///
    /// A derivation is a pure function of the rules, the lot and the seed, so the
    /// same three always produce the same town — and a different seed produces a
    /// different one without a word of the grammar changing.
    pub fn with_seed(mut self, seed: u64) -> Rules {
        self.seed = seed;
        self
    }

    pub fn seed(&self) -> u64 {
        self.seed
    }

    /// The grammar text, which is the whole of the rules.
    pub fn source(&self) -> &str {
        &self.source
    }

    /// Reads the grammar into everything deriving a town from it needs.
    pub fn compile(&self) -> Result<Compiled, Error> {
        let mut interpreter = Interpreter::new();
        let mut declared: BTreeMap<String, f64> = BTreeMap::new();

        for (line, statement) in statements(&self.source) {
            let parsed = parse_statement(&statement).map_err(|error| Error::Grammar {
                line,
                statement: statement.trim().to_owned(),
                detail: error.to_string(),
            })?;
            if let Statement::Attr { name, value } = &parsed {
                declared.insert(name.clone(), *value);
            }
            interpreter
                .add_statement(parsed)
                .map_err(|error| Error::Grammar {
                    line,
                    statement: statement.trim().to_owned(),
                    detail: error.to_string(),
                })?;
        }

        if !interpreter.has_rule(ROOT) {
            return Err(Error::NoRootRule { root: ROOT });
        }

        let layout = layout_from(&declared);
        let floor_height = match declared.get("FloorHeight") {
            Some(value) if value.is_finite() && *value > 0.0 => *value,
            _ => DEFAULT_FLOOR_HEIGHT,
        };
        // Declared or not, the numbers the lots were cut to are what the grammar gets
        // to read: a rule that asks for `LotDepth` means the depth of the lot it is
        // standing on, and one that asks for `FloorHeight` means the storey the
        // levels will be counted in — whether or not the grammar bothered to declare
        // either. A knob that only worked once you had declared it would be a knob
        // with a trap in it.
        for (name, value) in layout_attrs(&layout, floor_height) {
            interpreter.set_attr(name, value);
        }
        interpreter.seed = self.seed;
        Ok(Compiled {
            interpreter,
            layout,
            floor_height,
        })
    }
}

/// Rules read: what deriving a town from them needs.
pub struct Compiled {
    pub interpreter: Interpreter,
    /// How the frontages are cut into lots.
    pub layout: Layout,
    /// What a storey is, metres, for the storey count a footprint's height implies.
    pub floor_height: f64,
}

/// What a storey is when the grammar does not say, metres.
pub const DEFAULT_FLOOR_HEIGHT: f64 = 3.2;

fn layout_from(declared: &BTreeMap<String, f64>) -> Layout {
    let default = Layout::default();
    let read = |name: &str, fallback: f64, least: f64| -> f64 {
        match declared.get(name) {
            Some(value) if value.is_finite() && *value >= least => *value,
            _ => fallback,
        }
    };
    Layout {
        setback: read("Setback", default.setback, 0.0),
        lot_width: read("LotWidth", default.lot_width, 1.0),
        lot_depth: read("LotDepth", default.lot_depth, 1.0),
        lot_gap: read("LotGap", default.lot_gap, 0.0),
        corner_clearance: read("CornerClearance", default.corner_clearance, 0.0),
    }
}

fn layout_attrs(layout: &Layout, floor_height: f64) -> [(&'static str, f64); 6] {
    [
        ("Setback", layout.setback),
        ("LotWidth", layout.lot_width),
        ("LotDepth", layout.lot_depth),
        ("LotGap", layout.lot_gap),
        ("CornerClearance", layout.corner_clearance),
        ("FloorHeight", floor_height),
    ]
}

/// Cuts grammar text into statements, each with the line it starts on.
///
/// See the module documentation for the rule; the short of it is that a line either
/// heads a statement or continues one.
fn statements(source: &str) -> Vec<(usize, String)> {
    let mut statements: Vec<(usize, String)> = Vec::new();
    for (index, line) in source.lines().enumerate() {
        if line.trim().is_empty() {
            continue;
        }
        if heads_a_statement(line) || statements.is_empty() {
            statements.push((index + 1, line.to_owned()));
        } else {
            let last = statements.len() - 1;
            statements[last].1.push('\n');
            statements[last].1.push_str(line);
        }
    }
    // A statement that is nothing but comment lines parses to nothing and would be
    // an error rather than a comment.
    statements.retain(|(_, statement)| !is_only_comment(statement));
    statements
}

fn heads_a_statement(line: &str) -> bool {
    let code = strip_line_comment(line);
    let trimmed = code.trim_start();
    for keyword in ["attr", "const", "style"] {
        if let Some(rest) = trimmed.strip_prefix(keyword) {
            if rest.starts_with(|c: char| c.is_whitespace()) {
                return true;
            }
        }
    }
    // `Name -->` or `Name(a, b) -->`, with nothing but the head in between.
    let Some(head) = code.split("-->").next().filter(|_| code.contains("-->")) else {
        return false;
    };
    let head = head.trim();
    let name = head.split('(').next().unwrap_or(head).trim();
    !name.is_empty()
        && name.starts_with(|c: char| c.is_alphabetic() || c == '_')
        && name.chars().all(|c| c.is_alphanumeric() || c == '_')
}

fn strip_line_comment(line: &str) -> &str {
    match line.find("//") {
        Some(at) => &line[..at],
        None => line,
    }
}

fn is_only_comment(statement: &str) -> bool {
    statement
        .lines()
        .all(|line| strip_line_comment(line).trim().is_empty())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_preset_compiles_and_has_a_root_rule() {
        for (name, _) in PRESETS {
            let rules = Rules::preset(name).unwrap();
            let compiled = rules.compile().unwrap_or_else(|error| {
                panic!("preset {name} does not compile: {error}");
            });
            assert!(compiled.interpreter.has_rule(ROOT), "{name}");
            assert!(compiled.layout.lot_width > 0.0, "{name}");
            assert!(compiled.layout.lot_depth > 0.0, "{name}");
        }
    }

    #[test]
    fn a_rule_body_may_run_over_several_lines() {
        let rules = Rules::from_grammar(
            "Lot -->\n  Split(X) {\n    ~1: Unit\n    | ~1: Unit\n  }\nUnit --> Extrude(6) I(\"house\")",
        );
        let compiled = rules.compile().unwrap();
        assert!(compiled.interpreter.has_rule("Lot"));
        assert!(compiled.interpreter.has_rule("Unit"));
    }

    #[test]
    fn the_layout_comes_out_of_the_grammars_own_declarations() {
        let rules = Rules::from_grammar(
            "attr LotWidth = 40\nattr Setback = 9\nLot --> Extrude(5) I(\"shed\")",
        );
        let compiled = rules.compile().unwrap();
        assert_eq!(compiled.layout.lot_width, 40.0);
        assert_eq!(compiled.layout.setback, 9.0);
        // Undeclared knobs keep their defaults.
        assert_eq!(compiled.layout.lot_depth, Layout::default().lot_depth);
    }

    #[test]
    fn every_knob_is_readable_from_the_grammar_even_undeclared() {
        // A rule may read any of them without declaring it, and gets the value the
        // lot was actually cut to.
        for knob in [
            "Setback",
            "LotWidth",
            "LotDepth",
            "LotGap",
            "CornerClearance",
            "FloorHeight",
        ] {
            let rules = Rules::from_grammar(format!("Lot --> Extrude({knob} / 4) I(\"house\")"));
            let compiled = rules.compile().unwrap_or_else(|error| {
                panic!("{knob} should be readable: {error}");
            });
            assert!(compiled.interpreter.attrs().contains_key(knob), "{knob}");
        }
    }

    #[test]
    fn a_grammar_without_a_root_rule_is_refused() {
        let rules = Rules::from_grammar("House --> Extrude(6) I(\"house\")");
        assert!(matches!(rules.compile(), Err(Error::NoRootRule { .. })));
    }

    #[test]
    fn a_broken_grammar_says_which_line() {
        let rules = Rules::from_grammar("Lot --> Extrude(6) I(\"house\")\nBad --> Extrude(");
        match rules.compile().err() {
            Some(Error::Grammar { line, .. }) => assert_eq!(line, 2),
            other => panic!("expected a grammar error, got {other:?}"),
        }
    }

    #[test]
    fn a_misspelt_preset_is_not_taken_for_a_grammar() {
        assert!(matches!(
            Rules::parse("suburbon"),
            Err(Error::UnknownPreset { .. })
        ));
        assert_eq!(
            Rules::parse("suburban").unwrap(),
            Rules::preset("suburban").unwrap()
        );
    }

    #[test]
    fn a_comment_only_line_is_not_a_statement() {
        let rules =
            Rules::from_grammar("// nothing here\nLot --> Extrude(6) I(\"house\")\n// nor here");
        assert!(rules.compile().is_ok());
    }

    #[test]
    fn the_storey_height_is_the_one_the_grammar_declared() {
        assert_eq!(
            Rules::from_grammar("attr FloorHeight = 4.5\nLot --> Extrude(9) I(\"a\")")
                .compile()
                .unwrap()
                .floor_height,
            4.5
        );
        assert_eq!(
            Rules::from_grammar("Lot --> Extrude(9) I(\"a\")")
                .compile()
                .unwrap()
                .floor_height,
            DEFAULT_FLOOR_HEIGHT
        );
    }
}
