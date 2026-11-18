//! The land a document asks for, generated round the roads it built.
//!
//! The document says the land in shapes — hills, rivers, heaps, pads, and the roll
//! of the country between them — and `roadgen-carla`'s environment generator
//! makes a triangulated surface of them that meets every road, exactly as the
//! CARLA export and the Python package make it. It runs after the roads are
//! built, because the land is shaped round them; and it can put roads on bridges
//! and through tunnels of its own, so the map is checked again after it.

use roadgen_carla::environment::{
    generate, GroundSpec, HeapSpec, HillSpec, PadSpec, Places, RiverSpec,
};
use roadgen_core::{Point3, RoadId, ValidationConfig};
use roadgen_editor::{Built, Document, Problem, Severity};

fn point([x, y, z]: [f64; 3]) -> Point3 {
    Point3::new(x, y, z)
}

/// What the generator is told to make of the document's land; `None` when the
/// document has none.
pub fn places(document: &Document) -> Option<Places> {
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
                buildings: None,
                site: None,
            })
            .collect(),
        fitted_roads: roads_by("fitted"),
        structured_roads: roads_by("structured"),
        ..Places::default()
    })
}

/// Puts the document's land on a built map, and checks the map again: the land
/// can add bridges and tunnels of its own. Returns what the generator noted. A
/// map the land cannot be generated for is left as it was, with the reason among
/// the problems.
pub fn apply(built: &mut Built, document: &Document) -> Vec<String> {
    let (Some(places), Some(map)) = (places(document), built.map.as_mut()) else {
        return Vec::new();
    };
    let mut landed = map.clone();
    match generate(&mut landed, &places) {
        Ok(report) => {
            *map = landed;
            built
                .problems
                .retain(|problem| problem.severity != Severity::Invalid);
            let issues = map.issues(ValidationConfig::default());
            built
                .problems
                .extend(issues.into_iter().map(|issue| Problem {
                    severity: Severity::Invalid,
                    road: None,
                    link: None,
                    surface: None,
                    structure: None,
                    message: issue.to_string(),
                }));
            report.notes
        }
        Err(error) => {
            built.problems.push(Problem {
                severity: Severity::Error,
                road: None,
                link: None,
                surface: None,
                structure: None,
                message: format!("the land could not be generated: {error}"),
            });
            Vec::new()
        }
    }
}
