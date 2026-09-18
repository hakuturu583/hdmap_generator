//! From a lot to the buildings standing on it.
//!
//! # Two frames, one conversion
//!
//! CGA is Y-up, because it grew up describing buildings; the IR is Z-up, because it
//! grew up describing roads. Rather than letting that difference leak into the
//! grammar, it is resolved once, here, by handing the shape grammar a world of its
//! own:
//!
//! ```text
//!   CGA          IR
//!    x     =      x
//!    y     =      z     (up)
//!    z     =     -y
//! ```
//!
//! which is a right-handed frame either way round, so a lot's rotation is a plain
//! yaw and a grammar never has to know which way is up.
//!
//! # What counts as a building
//!
//! A derivation is a flat list of oriented boxes, and most grammars produce more of
//! them than there are buildings: a mass split into floors is several boxes, a roof
//! adds panels, a facade adds a box per window. A footprint is an outline on the
//! ground, so:
//!
//! - a box with extent on all three axes **standing on the lot's ground** is a
//!   building, and its outline is the ground plan of that box;
//! - every other box **raises the height** of the building it stands over, which is
//!   what turns a stack of floors, a set-back tower and a pitched roof into one
//!   building of the right height rather than three of the wrong one;
//! - a box that stands over nothing is dropped, because there is no footprint to
//!   give it.
//!
//! A facade's windows fall out of this on their own: `Comp(Faces)` produces scopes
//! with no thickness, and a box with no thickness is not a mass.

use roadgen_core::buildings::{Building, Footprint};
use roadgen_core::geometry::Point3;
use roadgen_core::id::BuildingId;
use symbios_shape::{Interpreter, Quat, Scope, ShapeError, Terminal, Vec3};

use crate::land::Lot;
use crate::plane::{convex_hull, Point2};

/// How far above the lot's ground a mass may start and still be standing on it.
///
/// Generous, because a grammar that raises a plinth or lands a hair off zero through
/// a `rand` is still describing a building that meets the ground.
const STANDS_ON_GROUND: f64 = 0.5;

/// Extents below this are no extent: a `Comp(Faces)` canvas is exactly zero, and a
/// split that leaves a sliver is not a building.
const NO_EXTENT: f64 = 1e-6;

/// One building derived from a lot, before it is known whether it fits.
#[derive(Debug, Clone)]
pub struct Mass {
    /// The outline on the ground, anticlockwise, in the map's metres.
    pub plan: Vec<Point2>,
    /// Ground level, metres.
    pub base: f64,
    /// Top of everything standing over this outline, metres.
    pub top: f64,
    /// The word the grammar emitted the mass with.
    pub kind: String,
}

impl Mass {
    pub fn height(&self) -> f64 {
        self.top - self.base
    }

    /// The building this mass is, named for the lot it stands on.
    pub fn into_building(self, lot: &Lot, part: usize, floor_height: f64) -> Option<Building> {
        let ring: Vec<Point3> = self
            .plan
            .iter()
            .map(|point| Point3::new(point[0], point[1], self.base))
            .collect();
        let footprint = Footprint::new(ring).ok()?;
        let height = self.height();
        if !height.is_finite() || height <= 0.0 {
            return None;
        }
        Some(Building {
            id: BuildingId::new(format!(
                "{}/{}/{}/{}",
                lot.road.local_name(),
                lot.side.as_str(),
                lot.index,
                part
            )),
            footprint,
            height,
            levels: ((height / floor_height).round() as i64).max(1) as u32,
            kind: self.kind,
        })
    }
}

/// Derives `lot` and returns the masses standing on it, in the derivation's order.
///
/// `interpreter` is taken by value-of-a-mutable-reference because the seed it derives
/// from is a field on it: every lot is seeded from its own identity, so a town is the
/// same town every time and editing one street does not reshuffle the next.
pub fn derive(
    interpreter: &mut Interpreter,
    lot: &Lot,
    root: &str,
    seed: u64,
) -> Result<Vec<Mass>, ShapeError> {
    interpreter.seed = seed;
    let model = interpreter.derive(root_scope(lot), root)?;
    Ok(masses(&model.terminals, lot))
}

/// The scope a lot hands the grammar: its ground rectangle, no height yet.
fn root_scope(lot: &Lot) -> Scope {
    Scope::new(
        to_cga(lot.origin),
        // The yaw that takes CGA +X to the lot's `along`. It takes +Z to `away` at
        // the same time, which is what puts the street at the `Front` face.
        Quat::from_rotation_y(lot.along[1].atan2(lot.along[0])),
        Vec3::new(lot.width, 0.0, lot.depth),
    )
}

fn to_cga(point: Point3) -> Vec3 {
    Vec3::new(point.x, point.z, -point.y)
}

/// The map position of a CGA point.
fn from_cga(point: Vec3) -> Point3 {
    Point3::new(point.x, -point.z, point.y)
}

/// Groups a derivation's terminals into the buildings they describe.
fn masses(terminals: &[Terminal], lot: &Lot) -> Vec<Mass> {
    let mut grounded: Vec<Mass> = Vec::new();
    let mut aloft: Vec<(Point2, f64)> = Vec::new();

    for terminal in terminals {
        let corners = ground_corners(&terminal.scope);
        let plan = convex_hull(&corners);
        if plan.len() < 3 {
            continue;
        }
        let (base, top) = height_span(&terminal.scope);
        let volume = terminal.scope.size.x > NO_EXTENT
            && terminal.scope.size.y > NO_EXTENT
            && terminal.scope.size.z > NO_EXTENT;

        if volume && base <= lot.origin.z + STANDS_ON_GROUND {
            grounded.push(Mass {
                plan,
                base,
                top,
                kind: terminal.mesh_id.clone(),
            });
        } else {
            aloft.push((centre(&plan), top));
        }
    }

    // Whatever stands over a building is part of that building's height: a floor, a
    // roof, a tower on a podium. Done after the pass above rather than during it,
    // because a derivation emits in breadth-first order and the mass under a roof is
    // not reliably seen before the roof.
    for (over, top) in aloft {
        let Some(mass) = grounded
            .iter_mut()
            .filter(|mass| contains(&mass.plan, over))
            // The smallest outline containing it, for the case where a grammar has
            // put one ground mass inside another: the inner one is what is really
            // underneath.
            .min_by(|a, b| {
                plan_area(&a.plan)
                    .partial_cmp(&plan_area(&b.plan))
                    .unwrap_or(std::cmp::Ordering::Equal)
            })
        else {
            continue;
        };
        mass.top = mass.top.max(top);
    }

    grounded
}

/// A scope's eight corners, projected to the ground.
fn ground_corners(scope: &Scope) -> Vec<Point2> {
    let mut corners = Vec::with_capacity(8);
    for u in [0.0, 1.0] {
        for v in [0.0, 1.0] {
            for w in [0.0, 1.0] {
                let point = from_cga(scope.world_point(u, v, w));
                corners.push([point.x, point.y]);
            }
        }
    }
    corners
}

/// The lowest and highest a scope reaches, in the map's metres.
fn height_span(scope: &Scope) -> (f64, f64) {
    let mut low = f64::INFINITY;
    let mut high = f64::NEG_INFINITY;
    for u in [0.0, 1.0] {
        for v in [0.0, 1.0] {
            for w in [0.0, 1.0] {
                let z = from_cga(scope.world_point(u, v, w)).z;
                low = low.min(z);
                high = high.max(z);
            }
        }
    }
    (low, high)
}

fn centre(plan: &[Point2]) -> Point2 {
    let count = plan.len() as f64;
    [
        plan.iter().map(|point| point[0]).sum::<f64>() / count,
        plan.iter().map(|point| point[1]).sum::<f64>() / count,
    ]
}

fn plan_area(plan: &[Point2]) -> f64 {
    let mut total = 0.0;
    for index in 0..plan.len() {
        let a = plan[index];
        let b = plan[(index + 1) % plan.len()];
        total += a[0] * b[1] - b[0] * a[1];
    }
    (total / 2.0).abs()
}

/// Whether a convex, anticlockwise plan contains a point. Edges count as inside.
fn contains(plan: &[Point2], point: Point2) -> bool {
    for index in 0..plan.len() {
        let a = plan[index];
        let b = plan[(index + 1) % plan.len()];
        let cross = (b[0] - a[0]) * (point[1] - a[1]) - (b[1] - a[1]) * (point[0] - a[0]);
        if cross < -1e-9 {
            return false;
        }
    }
    true
}

#[cfg(test)]
mod tests {
    use roadgen_core::topology::LateralSide;
    use roadgen_core::RoadId;
    use symbios_shape::grammar::parse_statement;

    use super::*;

    fn lot() -> Lot {
        Lot {
            road: RoadId::new("main"),
            side: LateralSide::Right,
            index: 0,
            origin: Point3::new(100.0, -10.0, 4.0),
            along: [1.0, 0.0],
            away: [0.0, -1.0],
            width: 14.0,
            depth: 16.0,
        }
    }

    fn interpreter(lines: &[&str]) -> Interpreter {
        let mut interpreter = Interpreter::new();
        for line in lines {
            interpreter
                .add_statement(parse_statement(line).unwrap())
                .unwrap();
        }
        interpreter
    }

    #[test]
    fn the_lot_scope_lands_where_the_lot_is_and_faces_the_street() {
        let lot = lot();
        let scope = root_scope(&lot);
        // The scope's origin corner is the lot's origin.
        let origin = from_cga(scope.world_point(0.0, 0.0, 0.0));
        assert!(origin.is_close(lot.origin, 1e-9), "{origin:?}");
        // Local +X runs along the frontage, local +Z away from the road.
        let along = from_cga(scope.world_point(1.0, 0.0, 0.0));
        assert!(
            (along.x - (lot.origin.x + lot.width)).abs() < 1e-9,
            "{along:?}"
        );
        assert!((along.y - lot.origin.y).abs() < 1e-9, "{along:?}");
        let away = from_cga(scope.world_point(0.0, 0.0, 1.0));
        assert!(
            (away.y - (lot.origin.y - lot.depth)).abs() < 1e-9,
            "{away:?}"
        );
        assert!((away.x - lot.origin.x).abs() < 1e-9, "{away:?}");
    }

    #[test]
    fn a_single_extruded_mass_is_one_building_of_that_height() {
        let mut interpreter = interpreter(&[r#"Lot --> Extrude(9) I("house")"#]);
        let masses = derive(&mut interpreter, &lot(), "Lot", 1).unwrap();
        assert_eq!(masses.len(), 1);
        assert_eq!(masses[0].kind, "house");
        assert!((masses[0].height() - 9.0).abs() < 1e-9);
        assert!((masses[0].base - 4.0).abs() < 1e-9);
        // Fourteen by sixteen, wherever the lot sits.
        let area = plan_area(&masses[0].plan);
        assert!((area - 14.0 * 16.0).abs() < 1e-6, "{area}");
    }

    #[test]
    fn a_mass_split_into_floors_is_still_one_building() {
        let mut interpreter = interpreter(&[
            "Lot --> Extrude(12) Split(Y) { 4: Base | ~1: Upper }",
            r#"Base --> I("retail")"#,
            r#"Upper --> I("office")"#,
        ]);
        let masses = derive(&mut interpreter, &lot(), "Lot", 1).unwrap();
        assert_eq!(masses.len(), 1);
        // The building is named for the mass that meets the ground, and is as tall
        // as everything stacked on it.
        assert_eq!(masses[0].kind, "retail");
        assert!((masses[0].height() - 12.0).abs() < 1e-9);
    }

    #[test]
    fn a_split_footprint_is_two_buildings_side_by_side() {
        let mut interpreter = interpreter(&[
            "Lot --> Split(X) { ~1: Unit | ~1: Unit }",
            r#"Unit --> Extrude(7) I("terrace")"#,
        ]);
        let masses = derive(&mut interpreter, &lot(), "Lot", 1).unwrap();
        assert_eq!(masses.len(), 2);
        for mass in &masses {
            assert!((plan_area(&mass.plan) - 7.0 * 16.0).abs() < 1e-6);
        }
    }

    #[test]
    fn a_facades_windows_are_not_buildings() {
        let mut interpreter = interpreter(&[
            "Lot --> Extrude(9) Comp(Faces) { Side: Facade | All: NIL }",
            "Facade --> Repeat(X, 2.5) { Window }",
        ]);
        let masses = derive(&mut interpreter, &lot(), "Lot", 1).unwrap();
        // Face scopes have no thickness, so none of them is a mass — and with the
        // mass itself consumed by `Comp`, there is nothing to stand on.
        assert!(masses.is_empty(), "{masses:?}");
    }

    #[test]
    fn a_tower_set_back_on_a_podium_is_one_building_as_tall_as_the_tower() {
        let mut interpreter = interpreter(&[
            "Lot --> Extrude(30) Split(Y) { 8: Podium | ~1: Shaft }",
            r#"Podium --> I("retail")"#,
            // Half the plan, centred: a tower that does not reach the lot's edges,
            // and so is not the outline anyone would draw on a map.
            r#"Shaft --> Size(scope.x * 0.5, scope.y, scope.z * 0.5) Center(XZ) I("office")"#,
        ]);
        let masses = derive(&mut interpreter, &lot(), "Lot", 1).unwrap();

        // One footprint — the podium's, which is what meets the ground — and the
        // height of the tower that stands on it.
        assert_eq!(masses.len(), 1);
        assert_eq!(masses[0].kind, "retail");
        assert!((masses[0].height() - 30.0).abs() < 1e-9, "{masses:?}");
        assert!((plan_area(&masses[0].plan) - 14.0 * 16.0).abs() < 1e-6);
    }

    #[test]
    fn a_pitched_roof_is_part_of_the_height_and_not_a_building() {
        let mut interpreter = interpreter(&[
            "Lot --> Extrude(9) Split(Y) { ~1: Body | 3: Cap }",
            r#"Body --> I("house")"#,
            "Cap --> Roof(Gable, height=3) { Slope: Tiles | GableEnd: Wall }",
        ]);
        let masses = derive(&mut interpreter, &lot(), "Lot", 1).unwrap();

        // The roof's panels have no thickness, so none of them is a building; what
        // they do is put the ridge where the ridge is.
        assert_eq!(masses.len(), 1);
        assert_eq!(masses[0].kind, "house");
        assert!((masses[0].height() - 9.0).abs() < 1e-6, "{masses:?}");
    }

    #[test]
    fn the_same_seed_derives_the_same_masses() {
        let grammar = &[
            "Lot --> 50% A | else: B",
            r#"A --> Extrude(rand(5, 12)) I("house")"#,
            r#"B --> Extrude(rand(5, 12)) I("retail")"#,
        ];
        let once = derive(&mut interpreter(grammar), &lot(), "Lot", 99).unwrap();
        let twice = derive(&mut interpreter(grammar), &lot(), "Lot", 99).unwrap();
        assert_eq!(once.len(), twice.len());
        for (a, b) in once.iter().zip(&twice) {
            assert_eq!(a.kind, b.kind);
            assert_eq!(a.height(), b.height());
        }
    }
}
