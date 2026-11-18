//! The land and the buildings a document asks for, generated round the roads it
//! built.
//!
//! The buildings come from `roadgen-buildings`' shape grammar, lot by lot along
//! the road frontages. With land, they are the environment generator's town —
//! kept off the hills, heaps and pads and from under bridges, and given
//! foundations where the land falls away — and a pad can have some of its own.
//! Without it they are lined along the roads directly.
//!
//! The document says the land in shapes — hills, rivers, heaps, pads, and the roll
//! of the country between them — and `roadgen-carla`'s environment generator
//! makes a triangulated surface of them that meets every road, exactly as the
//! CARLA export and the Python package make it. It runs after the roads are
//! built, because the land is shaped round them; and it can put roads on bridges
//! and through tunnels of its own, so the map is checked again after it.

use roadgen_buildings::Rules;
use roadgen_carla::environment::{
    generate, GroundSpec, HeapSpec, HillSpec, PadSpec, Places, RiverSpec,
};
use roadgen_core::{Point3, RoadId, ValidationConfig};
use roadgen_editor::{Built, Document, Problem, Severity};

fn point([x, y, z]: [f64; 3]) -> Point3 {
    Point3::new(x, y, z)
}

/// The shape grammar the document's buildings are derived from.
pub fn rules(document: &Document) -> Option<Result<Rules, String>> {
    let buildings = document.buildings.as_ref()?;
    let text = if buildings.preset.trim() == "custom" {
        buildings.grammar.as_str()
    } else {
        buildings.preset.as_str()
    };
    Some(
        Rules::parse(text)
            .map(|rules| rules.with_seed(buildings.seed))
            .map_err(|error| format!("the buildings' rules: {error}")),
    )
}

/// What the generator is told to make of the document's land; `None` when the
/// document has none.
pub fn places(document: &Document, town: Option<Rules>) -> Option<Places> {
    let terrain = document.terrain.as_ref()?;
    let roads_by = |mode: &str| -> Vec<RoadId> {
        document
            .roads
            .iter()
            .filter(|road| road.ground == mode)
            .map(|road| RoadId::new(&road.id))
            .collect()
    };
    Some(Places {
        ground: Some(GroundSpec {
            margin: terrain.margin,
            cell: terrain.cell,
            spacing: terrain.spacing,
            relief: terrain.relief,
            wavelength: terrain.wavelength,
        }),
        seed: terrain.seed,
        hills: terrain
            .hills
            .iter()
            .map(|hill| HillSpec {
                name: hill.name.clone(),
                crest: hill.crest.iter().copied().map(point).collect(),
                width: hill.width,
                height: hill.height,
            })
            .collect(),
        rivers: terrain
            .rivers
            .iter()
            .map(|river| RiverSpec {
                name: river.name.clone(),
                centre: river.centre.iter().copied().map(point).collect(),
                width: river.width,
                depth: river.depth,
                bank_slope: river.bank_slope,
                site: None,
            })
            .collect(),
        heaps: terrain
            .heaps
            .iter()
            .map(|heap| HeapSpec {
                name: heap.name.clone(),
                kind: heap.kind.clone(),
                centre: point(heap.centre),
                radius: heap.radius,
                height: heap.height,
                repose: heap.repose,
                site: None,
            })
            .collect(),
        pads: terrain
            .pads
            .iter()
            .map(|pad| PadSpec {
                name: pad.name.clone(),
                outline: pad.outline.iter().copied().map(point).collect(),
                buildings: town.clone().filter(|_| pad.buildings),
                site: None,
            })
            .collect(),
        town,
        fitted_roads: roads_by("fitted"),
        structured_roads: roads_by("structured"),
        ..Places::default()
    })
}

/// Puts the document's land and buildings on a built map, and checks the map
/// again: the land can add bridges and tunnels of its own, and buildings are
/// checked too. Returns what the generators noted. What cannot be generated is
/// left off, with the reason among the problems.
pub fn apply(built: &mut Built, document: &Document) -> Vec<String> {
    let Some(map) = built.map.as_mut() else {
        return Vec::new();
    };
    let mut notes = Vec::new();
    let mut failures = Vec::new();
    let town = match rules(document) {
        Some(Ok(rules)) => Some(rules),
        Some(Err(error)) => {
            failures.push(error);
            None
        }
        None => None,
    };
    let mut changed = false;
    if let Some(places) = places(document, town.clone()) {
        let mut landed = map.clone();
        match generate(&mut landed, &places) {
            Ok(report) => {
                *map = landed;
                changed = true;
                if town.is_some() {
                    notes.push(format!("{} building(s) along the roads", report.buildings));
                }
                notes.extend(report.notes);
            }
            Err(error) => failures.push(format!("the land could not be generated: {error}")),
        }
    } else if let Some(rules) = &town {
        let mut built_up = map.clone();
        match roadgen_buildings::generate(&mut built_up, rules) {
            Ok(report) => {
                *map = built_up;
                changed = true;
                notes.push(format!(
                    "{} building(s) on {} lot(s) along the roads",
                    report.buildings, report.lots
                ));
            }
            Err(error) => failures.push(format!("the buildings could not be generated: {error}")),
        }
    }
    if changed {
        built
            .problems
            .retain(|problem| problem.severity != Severity::Invalid);
        let issues = map.issues(ValidationConfig::default());
        built.problems.extend(
            issues
                .into_iter()
                .map(|issue| problem(Severity::Invalid, issue.to_string())),
        );
    }
    built.problems.extend(
        failures
            .into_iter()
            .map(|message| problem(Severity::Error, message)),
    );
    notes
}

fn problem(severity: Severity, message: String) -> Problem {
    Problem {
        severity,
        road: None,
        link: None,
        surface: None,
        structure: None,
        message,
    }
}
