//! `roadgen-buildings` — puts a town beside the roads.
//!
//! This crate fills the [`buildings`](roadgen_core::buildings) arena of a map the
//! builder has already generated. It is a *generator*, like `roadgen-core` and unlike
//! the exporter crates: it reads roads and writes buildings, and no file format is
//! mentioned anywhere in it.
//!
//! ```text
//!        MapBuilder::finish()
//!                │
//!          UnvalidatedMap ──────┐
//!                │             │  roads
//!                │             ▼
//!                │         lots along each frontage
//!                │             │
//!                │             ▼
//!                │      symbios-shape derivation   (one per lot, one seed each)
//!                │             │
//!                │             ▼
//!                │        masses ──▶ footprints
//!                │             │
//!                ◀─────────────┘  buildings
//!                │
//!            validate()
//! ```
//!
//! # The division of labour
//!
//! Two questions have to be answered to put a building somewhere, and they are
//! completely different questions:
//!
//! - **Where may a building stand?** That is about the road network — where the
//!   surface is, how far back the kerb line runs, where a junction needs elbow room.
//!   It is answered in [`land`], from the generated geometry.
//! - **What stands there?** That is architecture, and it is answered by a **CGA shape
//!   grammar** — the CityEngine formalism — derived by the [`symbios_shape`] crate.
//!
//! So this crate hands each lot to the grammar as a rectangle on the ground and takes
//! back a set of masses, and the only thing it insists on afterwards is that nothing
//! ends up in the road or on top of a neighbour.
//!
//! # A town is a pure function
//!
//! [`Rules`] plus a map plus a seed is a town, every time. Each lot is seeded from its
//! own identity rather than from a running counter, so adding a road changes that
//! road's buildings and leaves every other street exactly as it was — the same
//! property the IR's identifiers have, for the same reason.
//!
//! # Example
//!
//! ```
//! use roadgen_core::prelude::*;
//! use roadgen_core::units::PositiveWidth;
//! use roadgen_core::topology::Direction;
//!
//! let mut builder = MapBuilder::default();
//! builder.add_road(
//!     RoadSpec::line(
//!         Point3::new(0.0, 0.0, 0.0),
//!         Point3::new(300.0, 0.0, 0.0),
//!         vec![
//!             LaneSpec::new(PositiveWidth::new(3.5)?, Direction::Forward),
//!             LaneSpec::new(PositiveWidth::new(3.5)?, Direction::Backward),
//!         ],
//!     )?
//!     .with_name("high_street"),
//! )?;
//!
//! let mut map = builder.finish()?;
//! let rules = roadgen_buildings::Rules::preset("downtown").unwrap();
//! let report = roadgen_buildings::generate(&mut map, &rules)?;
//!
//! assert!(report.buildings > 0);
//! let map = map.validate()?;
//! assert_eq!(map.buildings.len(), report.buildings);
//! # Ok::<(), Box<dyn std::error::Error>>(())
//! ```

pub mod error;
pub mod land;
pub mod masses;
pub mod plane;
pub mod rules;

use roadgen_core::validation::UnvalidatedMap;

pub use error::Error;
pub use land::{Layout, Lot};
pub use rules::{preset, Compiled, Rules, PRESETS, ROOT};

use crate::land::Blocked;
use crate::plane::Index;

/// What generating a town came to.
///
/// Nothing here is a failure. A lot with no building on it is a gap in a street, and
/// a street with gaps in it is a street; what the counts are for is telling the
/// difference between rules that are doing something and rules that are not.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Report {
    /// Lots cut out of the road frontages.
    pub lots: usize,
    /// Buildings placed.
    pub buildings: usize,
    /// Lots whose derivation produced no mass at all.
    pub empty_lots: usize,
    /// Lots the rules could not be derived on at all. A handful is a grammar meeting
    /// a frontage it was not written for; all of them is [`Error::Derivation`].
    pub failed_lots: usize,
    /// Lots dropped because what the grammar put on them reached into a road or
    /// onto a neighbour.
    pub clashes: usize,
}

/// Generates buildings beside the roads of `map`, replacing any it already has.
///
/// The map has to have been through [`MapBuilder::finish`](roadgen_core::MapBuilder::finish)
/// already, because a lot is measured against generated geometry and a road has none
/// until then. It does not have to have been validated: buildings are part of what
/// validation checks, so they go in first.
pub fn generate(map: &mut UnvalidatedMap, rules: &Rules) -> Result<Report, Error> {
    let Compiled {
        mut interpreter,
        layout,
        floor_height,
    } = rules.compile()?;

    let blocked = Blocked::of(map.as_map(), layout.setback);
    let lots = land::lots(map.as_map(), &layout);

    // Neighbours are tested against as they are placed, so the first lot of a street
    // wins a contested corner and the map's own order decides the rest. Cells wide
    // enough to hold a lot, so a candidate touches few of them.
    let mut placed = Index::new(25.0);
    let mut report = Report {
        lots: lots.len(),
        ..Report::default()
    };
    let mut buildings = Vec::new();
    let mut first_failure: Option<String> = None;

    for lot in &lots {
        let seed = rules.seed() ^ lot_seed(lot);
        let derived = match masses::derive(&mut interpreter, lot, ROOT, seed) {
            Ok(masses) => masses,
            // A grammar that cannot derive *this* lot — a split that overflows a
            // short frontage, a depth limit on a recursive rule — is a gap in the
            // street rather than a broken grammar: the rules compiled, and the next
            // lot may well work. The first failure is kept all the same, because a
            // grammar that fails on every lot is a broken grammar and the caller
            // deserves to be told which way.
            Err(error) => {
                report.failed_lots += 1;
                first_failure.get_or_insert_with(|| error.to_string());
                continue;
            }
        };
        if derived.is_empty() {
            report.empty_lots += 1;
            continue;
        }

        // A lot is placed whole or not at all. Within one lot the grammar is the
        // authority — an L plan's two wings meet along an edge and neither is in the
        // other's way — so the masses of one lot are never tested against each other,
        // and a lot that cannot fit does not leave half a building behind.
        let plans: Vec<_> = derived.iter().map(|mass| mass.plan.clone()).collect();
        if plans
            .iter()
            .any(|plan| blocked.hits(plan) || placed.hits(plan))
        {
            report.clashes += 1;
            continue;
        }

        for (part, mass) in derived.into_iter().enumerate() {
            let Some(building) = mass.into_building(lot, part, floor_height) else {
                continue;
            };
            placed.insert(plans[part].clone());
            buildings.push(building);
        }
    }

    if let Some(detail) = first_failure {
        if buildings.is_empty() {
            return Err(Error::Derivation {
                lots: report.lots,
                detail,
            });
        }
    }

    let map = map.as_map_mut();
    map.buildings = roadgen_core::Arena::new();
    for building in buildings {
        let id = building.id.clone();
        // Identifiers are derived from the road, the side and the position along it,
        // so a duplicate would mean two lots claiming one place on one frontage.
        if map.buildings.insert(id, building).is_ok() {
            report.buildings += 1;
        }
    }
    Ok(report)
}

/// A lot's own seed, from what names it rather than from where it came in the queue.
///
/// FNV-1a over the identifier, which is enough mixing for a seed and short enough to
/// read. What matters is that it depends on the lot and on nothing else.
fn lot_seed(lot: &Lot) -> u64 {
    let mut hash: u64 = 0xcbf2_9ce4_8422_2325;
    let name = format!("{}/{}/{}", lot.road, lot.side.as_str(), lot.index);
    for byte in name.as_bytes() {
        hash ^= *byte as u64;
        hash = hash.wrapping_mul(0x0000_0100_0000_01b3);
    }
    hash
}

#[cfg(test)]
mod tests {
    use roadgen_core::prelude::*;
    use roadgen_core::topology::Direction;
    use roadgen_core::units::PositiveWidth;

    use super::*;

    fn lanes() -> Vec<LaneSpec> {
        vec![
            LaneSpec::new(PositiveWidth::new(3.5).unwrap(), Direction::Forward),
            LaneSpec::new(PositiveWidth::new(3.5).unwrap(), Direction::Backward),
        ]
    }

    fn street(name: &str, from: Point3, to: Point3) -> UnvalidatedMap {
        let mut builder = MapBuilder::default();
        builder
            .add_road(RoadSpec::line(from, to, lanes()).unwrap().with_name(name))
            .unwrap();
        builder.finish().unwrap()
    }

    #[test]
    fn a_street_gets_buildings_down_both_sides() {
        let mut map = street(
            "high",
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(300.0, 0.0, 0.0),
        );
        let report = generate(&mut map, &Rules::default()).unwrap();

        assert!(report.lots > 10, "{report:?}");
        assert!(report.buildings > 0, "{report:?}");
        let map = map.as_map();
        assert_eq!(map.buildings.len(), report.buildings);
        assert!(map.buildings.iter().any(|b| b.footprint.centroid().y > 0.0));
        assert!(map.buildings.iter().any(|b| b.footprint.centroid().y < 0.0));
    }

    #[test]
    fn nothing_stands_in_the_road() {
        let mut map = street(
            "high",
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(300.0, 0.0, 0.0),
        );
        generate(&mut map, &Rules::default()).unwrap();

        // The road is 3.5 m of lane each side, so every footprint vertex has to be
        // clear of that — and of the setback the rules asked for.
        let setback = Rules::default().compile().unwrap().layout.setback;
        for building in map.as_map().buildings.iter() {
            for point in building.footprint.points() {
                assert!(
                    point.y.abs() >= 3.5 + setback - 1e-6,
                    "{} reaches the road at {point:?}",
                    building.id
                );
            }
        }
    }

    #[test]
    fn every_generated_map_validates() {
        for (name, _) in PRESETS {
            let mut map = street(
                "high",
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(400.0, 0.0, 0.0),
            );
            generate(&mut map, &Rules::preset(name).unwrap()).unwrap();
            let count = map.as_map().buildings.len();
            assert!(count > 0, "{name} generated nothing");
            map.validate()
                .unwrap_or_else(|error| panic!("{name}: {error}"));
        }
    }

    /// Every `I("kind")` a grammar names, as the set of words it can emit.
    fn kinds_named_by(grammar: &str) -> Vec<String> {
        let mut kinds = Vec::new();
        let mut rest = grammar;
        while let Some(at) = rest.find("I(\"") {
            rest = &rest[at + 3..];
            let Some(end) = rest.find('"') else { break };
            kinds.push(rest[..end].to_owned());
            rest = &rest[end..];
        }
        kinds.sort();
        kinds.dedup();
        kinds
    }

    #[test]
    fn a_preset_that_names_a_kind_of_building_produces_one() {
        // A rule that derives nothing — a mass consumed by the op that was meant to
        // stand on it, a branch that never fires — is silent otherwise: the town
        // still comes out, just without that kind of building in it.
        for (name, grammar) in PRESETS {
            let mut builder = MapBuilder::default();
            for (index, y) in [0.0, 200.0, 400.0, 600.0].into_iter().enumerate() {
                builder
                    .add_road(
                        RoadSpec::line(
                            Point3::new(0.0, y, 0.0),
                            Point3::new(900.0, y, 0.0),
                            lanes(),
                        )
                        .unwrap()
                        .with_name(format!("street{index}")),
                    )
                    .unwrap();
            }
            let mut map = builder.finish().unwrap();
            generate(&mut map, &Rules::preset(name).unwrap()).unwrap();

            let built: Vec<String> = map
                .as_map()
                .buildings
                .iter()
                .map(|building| building.kind.clone())
                .collect();
            for kind in kinds_named_by(grammar) {
                assert!(
                    built.contains(&kind),
                    "preset {name} names {kind:?} and never builds one"
                );
            }
        }
    }

    #[test]
    fn a_grammar_that_derives_nothing_anywhere_says_so_rather_than_building_nothing() {
        let mut map = street(
            "high",
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(300.0, 0.0, 0.0),
        );
        // `Storeys` is not a knob, a parameter or a declaration, so every lot fails
        // the same way. An empty town would send the caller looking at the map.
        let rules = Rules::from_grammar(r#"Lot --> Extrude(Storeys * 3) I("house")"#);
        match generate(&mut map, &rules) {
            Err(Error::Derivation { lots, detail }) => {
                assert!(lots > 0);
                assert!(detail.contains("Storeys"), "{detail}");
            }
            other => panic!("expected a derivation error, got {other:?}"),
        }
        assert!(map.as_map().buildings.is_empty());
    }

    #[test]
    fn a_grammar_that_fails_on_some_lots_still_builds_the_rest() {
        let mut map = street(
            "high",
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(300.0, 0.0, 0.0),
        );
        // A split into slots wider than the lot: it overflows on a narrow frontage
        // and fits on a wide one, so the seed decides which lots come out.
        let rules = Rules::from_grammar(
            "attr LotWidth = 14\nLot --> Split(X) { rand(4, 40): Unit | ~1: NIL }\n             Unit --> Extrude(7) I(\"house\")",
        );
        let report = generate(&mut map, &rules).expect("some lots should derive");
        assert!(report.failed_lots > 0, "{report:?}");
        assert!(report.buildings > 0, "{report:?}");
    }

    #[test]
    fn the_same_rules_and_seed_build_the_same_town() {
        let build = || {
            let mut map = street(
                "high",
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(300.0, 0.0, 0.0),
            );
            generate(&mut map, &Rules::default().with_seed(7)).unwrap();
            map.into_map()
                .buildings
                .iter()
                .map(|b| (b.id.to_string(), b.kind.clone(), b.height))
                .collect::<Vec<_>>()
        };
        assert_eq!(build(), build());
    }

    #[test]
    fn a_different_seed_builds_a_different_town() {
        let build = |seed: u64| {
            let mut map = street(
                "high",
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(300.0, 0.0, 0.0),
            );
            generate(&mut map, &Rules::default().with_seed(seed)).unwrap();
            map.into_map()
                .buildings
                .iter()
                .map(|b| (b.kind.clone(), b.height))
                .collect::<Vec<_>>()
        };
        assert_ne!(build(1), build(2));
    }

    #[test]
    fn adding_a_street_leaves_the_other_streets_alone() {
        let one = {
            let mut map = street(
                "high",
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(300.0, 0.0, 0.0),
            );
            generate(&mut map, &Rules::default()).unwrap();
            map.into_map()
                .buildings
                .iter()
                .map(|b| (b.id.to_string(), b.kind.clone(), b.height))
                .collect::<Vec<_>>()
        };

        let two = {
            let mut builder = MapBuilder::default();
            builder
                .add_road(
                    RoadSpec::line(
                        Point3::new(0.0, 0.0, 0.0),
                        Point3::new(300.0, 0.0, 0.0),
                        lanes(),
                    )
                    .unwrap()
                    .with_name("high"),
                )
                .unwrap();
            builder
                .add_road(
                    RoadSpec::line(
                        Point3::new(0.0, 400.0, 0.0),
                        Point3::new(300.0, 400.0, 0.0),
                        lanes(),
                    )
                    .unwrap()
                    .with_name("back"),
                )
                .unwrap();
            let mut map = builder.finish().unwrap();
            generate(&mut map, &Rules::default()).unwrap();
            map.into_map()
                .buildings
                .iter()
                .filter(|b| b.id.as_str().starts_with("building/high/"))
                .map(|b| (b.id.to_string(), b.kind.clone(), b.height))
                .collect::<Vec<_>>()
        };

        assert_eq!(one, two);
    }

    #[test]
    fn generating_twice_replaces_rather_than_doubles() {
        let mut map = street(
            "high",
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(300.0, 0.0, 0.0),
        );
        let first = generate(&mut map, &Rules::default()).unwrap();
        let second = generate(&mut map, &Rules::default()).unwrap();
        assert_eq!(first, second);
        assert_eq!(map.as_map().buildings.len(), second.buildings);
    }

    #[test]
    fn two_streets_that_nearly_touch_do_not_build_into_each_other() {
        let mut builder = MapBuilder::default();
        builder
            .add_road(
                RoadSpec::line(
                    Point3::new(0.0, 0.0, 0.0),
                    Point3::new(300.0, 0.0, 0.0),
                    lanes(),
                )
                .unwrap()
                .with_name("front"),
            )
            .unwrap();
        // Close enough that both frontages want the same strip of land.
        builder
            .add_road(
                RoadSpec::line(
                    Point3::new(0.0, 30.0, 0.0),
                    Point3::new(300.0, 30.0, 0.0),
                    lanes(),
                )
                .unwrap()
                .with_name("back"),
            )
            .unwrap();
        let mut map = builder.finish().unwrap();
        let report = generate(&mut map, &Rules::default()).unwrap();
        assert!(report.clashes > 0, "{report:?}");

        // Nothing placed overlaps anything else placed.
        let buildings: Vec<Vec<_>> = map
            .as_map()
            .buildings
            .iter()
            .map(|building| {
                building
                    .footprint
                    .points()
                    .iter()
                    .map(|point| [point.x, point.y])
                    .collect()
            })
            .collect();
        for (index, plan) in buildings.iter().enumerate() {
            for other in &buildings[index + 1..] {
                assert!(!plane::convex_overlap(plan, other));
            }
        }
    }
}
