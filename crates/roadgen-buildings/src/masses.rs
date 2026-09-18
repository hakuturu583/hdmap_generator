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
//! # What a derivation is, and what a building is
//!
//! A derivation is a flat list of oriented boxes and flat panels. A building is a
//! volume with parts and a roof. Turning the first into the second is three rules,
//! and each one is about what a box *is* rather than about what it was called:
//!
//! - A box with extent on all three axes is a **mass**. One standing on the lot's
//!   ground starts a building; one standing on another mass is a further **part** of
//!   that building. So a stack of floors, a tower on a podium and a wing behind a
//!   house all come out as one building of several parts, while two masses side by
//!   side on the ground are two buildings — which is what a terrace is.
//! - A box with no thickness is a **panel**. Panels sitting on a part's eaves are its
//!   **roof**: how far they rise is the roof's height, and where their highest points
//!   lie is its ridge. A facade's windows are panels too, but they stand against a
//!   wall rather than on the eaves, so they join no roof and change nothing.
//! - Anything standing over nothing is dropped, because there is no part to give it.
//!
//! # Reading a roof
//!
//! The grammar knows fifteen roof types; the IR has words for five, because those are
//! the five a consumer can be told about. What is read back is geometry, not the
//! grammar's vocabulary — the ridge is where the roof is highest, and the shape
//! follows from where that ridge lies over the part it covers:
//!
//! | The highest points of the roof | The shape |
//! | --- | --- |
//! | one point | pyramidal |
//! | a line down the middle, spanning the part | gabled |
//! | a line down the middle, stopping short of the ends | hipped |
//! | a line along one side | skillion |
//!
//! A roof that is none of those is not called the nearest one. It becomes the flat
//! top of the volume that contains it: the massing stays right, and only the word for
//! the shape is missing.

use roadgen_core::buildings::{
    ridge_direction, Building, BuildingPart, Footprint, Frontage, Roof, RoofShape, Solid,
};
use roadgen_core::geometry::Point3;
use roadgen_core::id::{BuildingId, BuildingPartId};
use symbios_shape::{FaceProfile, Interpreter, Quat, Scope, ShapeError, Terminal, Vec3};

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

/// How far a panel's foot may be from a part's eaves and still be its roof.
const SITS_ON_THE_EAVES: f64 = 0.5;

/// One building derived from a lot, before it is known whether it fits.
#[derive(Debug, Clone)]
pub struct Massing {
    /// What the grammar called the part that meets the ground.
    pub kind: String,
    /// The parts, lowest first, so the first is the one on the ground.
    pub parts: Vec<Part>,
}

/// One massing element: an outline in plan, the heights it spans, and its roof.
#[derive(Debug, Clone)]
pub struct Part {
    /// What the grammar called this part.
    pub kind: String,
    /// The outline in plan, anticlockwise, in the map's metres.
    pub plan: Vec<Point2>,
    /// Where the part starts, metres.
    pub base: f64,
    /// Where its walls stop, metres.
    pub eaves: f64,
    pub roof: Roof,
}

impl Part {
    pub fn wall_height(&self) -> f64 {
        self.eaves - self.base
    }

    /// The highest the part reaches, roof included.
    pub fn top(&self) -> f64 {
        self.eaves + self.roof.height
    }

    fn solid(&self) -> Option<Solid> {
        let wall_height = self.wall_height();
        if !wall_height.is_finite() || wall_height <= 0.0 {
            return None;
        }
        let ring: Vec<Point3> = self
            .plan
            .iter()
            .map(|point| Point3::new(point[0], point[1], self.base))
            .collect();
        Some(Solid {
            footprint: Footprint::new(ring).ok()?,
            wall_height,
            roof: self.roof,
        })
    }
}

impl Massing {
    /// Every plan the massing occupies, which is what has to fit beside the road.
    pub fn plans(&self) -> Vec<&[Point2]> {
        self.parts.iter().map(|part| part.plan.as_slice()).collect()
    }

    /// The building this massing is, named for the lot it stands on.
    ///
    /// `None` when no part of it survives being turned into a solid, which is a
    /// massing that was never a building.
    pub fn into_building(
        self,
        lot: &Lot,
        index: usize,
        floor_height: f64,
    ) -> Option<(Building, Vec<BuildingPart>)> {
        let id = BuildingId::new(format!(
            "{}/{}/{}/{}",
            lot.road.local_name(),
            lot.side.as_str(),
            lot.index,
            index
        ));

        let mut parts = Vec::with_capacity(self.parts.len());
        for part in &self.parts {
            let Some(solid) = part.solid() else {
                continue;
            };
            let levels = ((solid.height() / floor_height).round() as i64).max(1) as u32;
            parts.push(BuildingPart {
                id: BuildingPartId::of_building(&id, parts.len()),
                building: id.clone(),
                solid,
                // The building already says what its ground part is, so repeating it
                // on the part would be noise; a part that is something else says so.
                kind: (part.kind != self.kind).then(|| part.kind.clone()),
                levels,
            });
        }
        if parts.is_empty() {
            return None;
        }

        Some((
            Building {
                parts: parts.iter().map(|part| part.id.clone()).collect(),
                id,
                kind: self.kind,
                frontage: Some(Frontage {
                    road: lot.road.clone(),
                    side: lot.side,
                    station: lot.station,
                }),
            },
            parts,
        ))
    }
}

/// Derives `lot` and returns the buildings standing on it, in the derivation's order.
///
/// `interpreter` is taken by mutable reference because the seed it derives from is a
/// field on it: every lot is seeded from its own identity, so a town is the same town
/// every time and editing one street does not reshuffle the next.
pub fn derive(
    interpreter: &mut Interpreter,
    lot: &Lot,
    root: &str,
    seed: u64,
) -> Result<Vec<Massing>, ShapeError> {
    interpreter.seed = seed;
    let model = interpreter.derive(root_scope(lot), root)?;
    Ok(massings(&model.terminals, lot))
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

/// One box of a derivation, read in the map's own frame.
struct Box3 {
    /// The outline it casts on the ground. Empty for a box standing on edge — a
    /// gable end, a window — which casts a line rather than an outline.
    plan: Vec<Point2>,
    /// The middle of the box in plan, which a box standing on edge has even though
    /// it has no outline.
    middle: Point2,
    base: f64,
    top: f64,
    /// The corners, so that a roof's ridge can be found among them.
    corners: Vec<Point3>,
    volume: bool,
}

/// Groups a derivation's terminals into the buildings they describe.
fn massings(terminals: &[Terminal], lot: &Lot) -> Vec<Massing> {
    let mut masses: Vec<(Box3, String)> = Vec::new();
    let mut panels: Vec<Box3> = Vec::new();
    for terminal in terminals {
        let read = read_box(terminal);
        match (read.volume, read.plan.is_empty()) {
            // A volume with no outline is a sliver, and a sliver is not a building.
            (true, false) => masses.push((read, terminal.mesh_id.clone())),
            (true, true) => {}
            (false, _) => panels.push(read),
        }
    }

    // A mass on the ground starts a building; the rest are further parts of whichever
    // building they stand over. Two passes rather than one, because a derivation
    // emits breadth-first and the podium is not reliably seen before the tower.
    let mut buildings: Vec<Massing> = Vec::new();
    let mut upper: Vec<(&Box3, &String)> = Vec::new();
    for (mass, kind) in &masses {
        if mass.base <= lot.origin.z + STANDS_ON_GROUND {
            buildings.push(Massing {
                kind: kind.clone(),
                parts: vec![part_of(mass, kind)],
            });
        } else {
            upper.push((mass, kind));
        }
    }
    for (mass, kind) in upper {
        let Some(building) = building_under(&mut buildings, mass.middle) else {
            continue;
        };
        building.parts.push(part_of(mass, kind));
    }
    for building in &mut buildings {
        building.parts.sort_by(|a, b| {
            a.base
                .partial_cmp(&b.base)
                .unwrap_or(std::cmp::Ordering::Equal)
        });
    }

    roof_the_parts(&mut buildings, &panels);
    buildings
}

fn part_of(mass: &Box3, kind: &str) -> Part {
    Part {
        kind: kind.to_owned(),
        plan: mass.plan.clone(),
        base: mass.base,
        eaves: mass.top,
        roof: Roof::FLAT,
    }
}

/// One terminal, as a box in the map's frame.
///
/// A box standing on edge — a gable end, a window in a facade — casts a line rather
/// than an outline, and gets an empty plan rather than being thrown away: it is not a
/// part of a building, but it is part of a roof and its corners say where the ridge
/// is.
fn read_box(terminal: &Terminal) -> Box3 {
    let corners = surface_corners(terminal);
    let flat: Vec<Point2> = corners.iter().map(|point| [point.x, point.y]).collect();
    let plan = convex_hull(&flat);
    let size = terminal.scope.size;
    Box3 {
        plan: if plan.len() >= 3 { plan } else { Vec::new() },
        middle: centre(&flat),
        base: corners.iter().map(|p| p.z).fold(f64::INFINITY, f64::min),
        top: corners
            .iter()
            .map(|p| p.z)
            .fold(f64::NEG_INFINITY, f64::max),
        corners,
        volume: size.x > NO_EXTENT && size.y > NO_EXTENT && size.z > NO_EXTENT,
    }
}

/// Where a terminal's surface actually is, in the map's frame.
///
/// A scope is a box, but a panel inside one need not fill it: the engine says which
/// part of it the panel covers through the terminal's face profile, and a roof read
/// from the box instead of the profile is a roof whose gable ends reach the ridge
/// along their whole width. So the profile is honoured — it is the engine's own
/// statement of where the surface is, not an approximation of it.
fn surface_corners(terminal: &Terminal) -> Vec<Point3> {
    let scope = &terminal.scope;
    // Local (u, v) pairs in the panel's own plane, as fractions of its size.
    let panel = |pairs: Vec<(f64, f64)>| -> Vec<Point3> {
        pairs
            .into_iter()
            .map(|(u, v)| from_cga(scope.world_point(u, v, 0.0)))
            .collect()
    };
    match &terminal.face_profile {
        FaceProfile::Triangle { peak_offset } => panel(vec![
            (0.0, 0.0),
            (1.0, 0.0),
            (peak_offset.clamp(0.0, 1.0), 1.0),
        ]),
        FaceProfile::Trapezoid {
            top_width,
            offset_x,
        } => {
            let (width, offset) = (top_width.clamp(0.0, 1.0), offset_x.clamp(0.0, 1.0));
            panel(vec![
                (0.0, 0.0),
                (1.0, 0.0),
                ((offset + width).min(1.0), 1.0),
                (offset, 1.0),
            ])
        }
        // A polygon profile is a floor plan in the scope's own units, extruded
        // upwards; its outline is where the surface reaches.
        FaceProfile::Polygon(vertices) => vertices
            .iter()
            .flat_map(|vertex| {
                [0.0, scope.size.y].map(|rise| {
                    from_cga(scope.position + scope.rotation * Vec3::new(vertex.x, rise, vertex.y))
                })
            })
            .collect(),
        // A rectangle fills its scope, and a taper is a solid rather than a panel;
        // for both, the box is the surface.
        FaceProfile::Rectangle | FaceProfile::Taper(_) => {
            let mut corners = Vec::with_capacity(8);
            for u in [0.0, 1.0] {
                for v in [0.0, 1.0] {
                    for w in [0.0, 1.0] {
                        corners.push(from_cga(scope.world_point(u, v, w)));
                    }
                }
            }
            corners
        }
    }
}

/// The building whose ground part covers `point`, if one does.
fn building_under(buildings: &mut [Massing], point: Point2) -> Option<&mut Massing> {
    buildings
        .iter_mut()
        .filter(|building| contains(&building.parts[0].plan, point))
        // The smallest outline containing it, for the case where a grammar has put
        // one ground mass inside another: the inner one is what is really underneath.
        .min_by(|a, b| {
            plan_area(&a.parts[0].plan)
                .partial_cmp(&plan_area(&b.parts[0].plan))
                .unwrap_or(std::cmp::Ordering::Equal)
        })
}

/// Gives every part the roof the panels standing on it describe.
fn roof_the_parts(buildings: &mut [Massing], panels: &[Box3]) {
    // Which part each panel belongs to: the one whose plan covers it and on whose
    // eaves it sits. A window panel stands against a wall rather than on the eaves,
    // so it matches nothing — which is exactly what should happen to it.
    let slots: Vec<(usize, usize)> = buildings
        .iter()
        .enumerate()
        .flat_map(|(building, entry)| (0..entry.parts.len()).map(move |part| (building, part)))
        .collect();
    let mut assigned: Vec<Vec<&Box3>> = vec![Vec::new(); slots.len()];

    for panel in panels {
        let middle = panel.middle;
        let mut best: Option<(usize, f64)> = None;
        for (slot, (building, part)) in slots.iter().enumerate() {
            let part = &buildings[*building].parts[*part];
            if !contains(&part.plan, middle) {
                continue;
            }
            let gap = (panel.base - part.eaves).abs();
            if gap > SITS_ON_THE_EAVES || panel.top <= part.eaves + NO_EXTENT {
                continue;
            }
            if best.is_none_or(|(_, previous)| gap < previous) {
                best = Some((slot, gap));
            }
        }
        if let Some((slot, _)) = best {
            assigned[slot].push(panel);
        }
    }

    for (slot, (building, part)) in slots.iter().enumerate() {
        if assigned[slot].is_empty() {
            continue;
        }
        let part = &mut buildings[*building].parts[*part];
        match read_roof(&assigned[slot], part) {
            Some(roof) => part.roof = roof,
            // A roof shape the IR has no word for becomes the flat top of the volume
            // that contains it: the massing survives, the word does not.
            None => {
                part.eaves = assigned[slot]
                    .iter()
                    .map(|panel| panel.top)
                    .fold(part.eaves, f64::max);
            }
        }
    }
}

/// The roof `panels` describe over `part`, or `None` when it is not one of the five
/// shapes the IR can name.
fn read_roof(panels: &[&Box3], part: &Part) -> Option<Roof> {
    let top = panels
        .iter()
        .map(|panel| panel.top)
        .fold(f64::NEG_INFINITY, f64::max);
    let height = top - part.eaves;
    if height <= NO_EXTENT {
        return Some(Roof::FLAT);
    }

    // The ridge is where the roof is highest.
    let ridge: Vec<Point2> = panels
        .iter()
        .flat_map(|panel| panel.corners.iter())
        .filter(|corner| (corner.z - top).abs() < 1e-6)
        .map(|corner| [corner.x, corner.y])
        .collect();
    if ridge.is_empty() {
        return None;
    }
    let (from, to) = furthest_apart(&ridge);
    let length = (to[0] - from[0]).hypot(to[1] - from[1]);
    if length < 1e-6 {
        return Some(Roof::new(RoofShape::Pyramidal, height, 0.0));
    }

    // Every one of the five shapes has its highest points on a single line. A roof
    // whose highest points are two ridges, or a flat top with four corners, is a roof
    // this IR cannot name, and naming it the nearest thing would be a lie about the
    // building.
    let straight = ridge.iter().all(|point| {
        let cross =
            (to[0] - from[0]) * (point[1] - from[1]) - (to[1] - from[1]) * (point[0] - from[0]);
        (cross / length).abs() < 0.2
    });
    if !straight {
        return None;
    }

    let direction = (to[1] - from[1]).atan2(to[0] - from[0]);
    let (along, across) = extents(&part.plan, direction);
    if along < 1e-6 || across < 1e-6 {
        return None;
    }

    // How far the ridge sits from the middle of the part, measured across itself. A
    // gable and a hip run their ridge down the middle; a skillion puts its high edge
    // along one side, which is the whole difference between them.
    let middle = centre(&part.plan);
    let (cos, sin) = (direction.cos(), direction.sin());
    let offset = ridge
        .iter()
        .map(|point| ((point[0] - middle[0]) * -sin + (point[1] - middle[1]) * cos).abs())
        .fold(0.0, f64::max);

    let shape = if offset > across * 0.25 {
        RoofShape::Skillion
    } else if length > along * 0.9 {
        RoofShape::Gabled
    } else {
        RoofShape::Hipped
    };
    // A skillion falls across its high edge rather than along it, so the direction it
    // is recorded with is the one the slope runs in.
    let direction = match shape {
        RoofShape::Skillion => direction + std::f64::consts::FRAC_PI_2,
        _ => direction,
    };
    Some(Roof::new(shape, height, ridge_direction(direction)))
}

/// The two points furthest apart in a set, which for a ridge are its ends.
fn furthest_apart(points: &[Point2]) -> (Point2, Point2) {
    let mut best = (points[0], points[0], 0.0);
    for (index, a) in points.iter().enumerate() {
        for b in &points[index + 1..] {
            let distance = (b[0] - a[0]).hypot(b[1] - a[1]);
            if distance > best.2 {
                best = (*a, *b, distance);
            }
        }
    }
    (best.0, best.1)
}

/// How far a plan reaches along `direction` and across it, metres.
fn extents(plan: &[Point2], direction: f64) -> (f64, f64) {
    let (cos, sin) = (direction.cos(), direction.sin());
    let span = |values: Vec<f64>| {
        values.iter().copied().fold(f64::NEG_INFINITY, f64::max)
            - values.iter().copied().fold(f64::INFINITY, f64::min)
    };
    (
        span(plan.iter().map(|p| p[0] * cos + p[1] * sin).collect()),
        span(plan.iter().map(|p| -p[0] * sin + p[1] * cos).collect()),
    )
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

    pub(crate) fn lot() -> Lot {
        Lot {
            road: RoadId::new("main"),
            side: LateralSide::Right,
            index: 0,
            station: 30.0,
            origin: Point3::new(100.0, -10.0, 4.0),
            along: [1.0, 0.0],
            away: [0.0, -1.0],
            width: 14.0,
            depth: 16.0,
        }
    }

    fn derived(grammar: &[&str]) -> Vec<Massing> {
        derived_on(lot(), grammar)
    }

    fn derived_on(lot: Lot, grammar: &[&str]) -> Vec<Massing> {
        let mut interpreter = Interpreter::new();
        for line in grammar {
            interpreter
                .add_statement(parse_statement(line).unwrap())
                .unwrap();
        }
        derive(&mut interpreter, &lot, "Lot", 1).unwrap()
    }

    /// A house with a roof on it, for the roof-reading tests.
    fn with_roof(roof: &str) -> [String; 3] {
        [
            "Lot --> Extrude(9) Split(Y) { ~1: Body | 3: Cap }".to_owned(),
            r#"Body --> I("house")"#.to_owned(),
            format!("Cap --> {roof}"),
        ]
    }

    #[test]
    fn the_lot_scope_lands_where_the_lot_is_and_faces_the_street() {
        let lot = lot();
        let scope = root_scope(&lot);
        let origin = from_cga(scope.world_point(0.0, 0.0, 0.0));
        assert!(origin.is_close(lot.origin, 1e-9), "{origin:?}");
        let along = from_cga(scope.world_point(1.0, 0.0, 0.0));
        assert!((along.x - (lot.origin.x + lot.width)).abs() < 1e-9);
        assert!((along.y - lot.origin.y).abs() < 1e-9);
        let away = from_cga(scope.world_point(0.0, 0.0, 1.0));
        assert!((away.y - (lot.origin.y - lot.depth)).abs() < 1e-9);
        assert!((away.x - lot.origin.x).abs() < 1e-9);
    }

    #[test]
    fn a_single_extruded_mass_is_one_building_of_one_part() {
        let built = derived(&[r#"Lot --> Extrude(9) I("house")"#]);
        assert_eq!(built.len(), 1);
        assert_eq!(built[0].kind, "house");
        assert_eq!(built[0].parts.len(), 1);

        let part = &built[0].parts[0];
        assert!((part.base - 4.0).abs() < 1e-9);
        assert!((part.wall_height() - 9.0).abs() < 1e-9);
        assert_eq!(part.roof.shape, RoofShape::Flat);
        assert!((plan_area(&part.plan) - 14.0 * 16.0).abs() < 1e-6);
    }

    #[test]
    fn a_mass_split_into_floors_is_one_building_of_two_parts() {
        let built = derived(&[
            "Lot --> Extrude(12) Split(Y) { 4: Base | ~1: Upper }",
            r#"Base --> I("retail")"#,
            r#"Upper --> I("office")"#,
        ]);
        assert_eq!(built.len(), 1);
        // Named for the part that meets the ground, and made of both.
        assert_eq!(built[0].kind, "retail");
        assert_eq!(built[0].parts.len(), 2);
        assert!((built[0].parts[0].wall_height() - 4.0).abs() < 1e-9);
        assert!((built[0].parts[1].base - 8.0).abs() < 1e-9);
        assert!((built[0].parts[1].wall_height() - 8.0).abs() < 1e-9);
    }

    #[test]
    fn a_tower_on_a_podium_is_two_parts_of_different_plans() {
        let built = derived(&[
            "Lot --> Extrude(30) Split(Y) { 8: Podium | ~1: Shaft }",
            r#"Podium --> I("retail")"#,
            r#"Shaft --> Size(scope.x * 0.5, scope.y, scope.z * 0.5) Center(XZ) I("office")"#,
        ]);
        assert_eq!(built.len(), 1);
        assert_eq!(built[0].parts.len(), 2);
        let (podium, tower) = (&built[0].parts[0], &built[0].parts[1]);
        assert!((plan_area(&podium.plan) - 14.0 * 16.0).abs() < 1e-6);
        // A quarter of the plan, and standing on the podium rather than the ground.
        assert!((plan_area(&tower.plan) - 7.0 * 8.0).abs() < 1e-6);
        assert!((tower.base - podium.eaves).abs() < 1e-9);
        assert!((tower.top() - 34.0).abs() < 1e-9);
    }

    #[test]
    fn a_split_footprint_is_two_buildings_side_by_side() {
        let built = derived(&[
            "Lot --> Split(X) { ~1: Unit | ~1: Unit }",
            r#"Unit --> Extrude(7) I("terrace")"#,
        ]);
        assert_eq!(built.len(), 2);
        for building in &built {
            assert_eq!(building.parts.len(), 1);
            assert!((plan_area(&building.parts[0].plan) - 7.0 * 16.0).abs() < 1e-6);
        }
    }

    #[test]
    fn a_facades_windows_are_neither_buildings_nor_roofs() {
        let built = derived(&[
            "Lot --> Extrude(9) Comp(Faces) { Side: Facade | Bottom: NIL | Top: NIL }",
            "Facade --> Repeat(X, 2.5) { Window }",
        ]);
        // Face scopes have no thickness, so none of them is a mass — and with the
        // mass itself consumed by `Comp`, there is nothing to stand on.
        assert!(built.is_empty(), "{built:?}");
    }

    #[test]
    fn a_gable_a_hip_and_a_shed_are_read_back_as_themselves() {
        for (roof, expected, direction) in [
            (
                "Roof(Gable, height=3) { Slope: Tiles | GableEnd: Wall }",
                RoofShape::Gabled,
                // The ridge runs the long way, which on this lot is across the
                // street: the lot is 14 m of frontage and 16 m deep.
                Some(std::f64::consts::FRAC_PI_2),
            ),
            (
                "Roof(Hip, height=3) { Slope: Tiles | HipEnd: Tiles }",
                RoofShape::Hipped,
                Some(std::f64::consts::FRAC_PI_2),
            ),
            (
                "Roof(Shed, height=2) { Slope: Metal | Back: Glass }",
                RoofShape::Skillion,
                None,
            ),
        ] {
            let grammar = with_roof(roof);
            let built = derived(&grammar.each_ref().map(String::as_str));
            assert_eq!(built.len(), 1, "{roof}");
            assert_eq!(built[0].parts.len(), 1, "{roof}");

            let read = built[0].parts[0].roof;
            assert_eq!(read.shape, expected, "{roof} read back as {read:?}");
            assert!(read.height > 0.0, "{roof}");
            if let Some(direction) = direction {
                assert!(
                    (read.direction - direction).abs() < 0.05,
                    "{roof}: {read:?}"
                );
            }
            // And the part it sits on still stops at its own eaves.
            assert!((built[0].parts[0].eaves - 10.0).abs() < 1e-6, "{roof}");
        }
    }

    #[test]
    fn a_pyramid_over_a_square_plan_meets_at_a_point() {
        // Over an oblong it does not: what the engine draws there has a ridge, and
        // what has a ridge is a hip. The reading follows the geometry, not the word
        // the grammar used.
        let square = Lot {
            depth: 14.0,
            ..lot()
        };
        let grammar = with_roof("Roof(Pyramid, height=4) { Slope: Tiles }");
        let built = derived_on(square, &grammar.each_ref().map(String::as_str));
        assert_eq!(built[0].parts[0].roof.shape, RoofShape::Pyramidal);

        let built = derived(&grammar.each_ref().map(String::as_str));
        assert_eq!(built[0].parts[0].roof.shape, RoofShape::Hipped);
    }

    #[test]
    fn a_roof_the_ir_has_no_word_for_becomes_the_box_that_contains_it() {
        // An M-shaped roof has two ridges. The IR has no word for that, and calling
        // it the nearest one would be a lie about the building — so the part swallows
        // it whole instead, and the massing stays right.
        let grammar = with_roof("Roof(MShaped, height=3) { All: Tiles }");
        let built = derived(&grammar.each_ref().map(String::as_str));
        assert_eq!(built.len(), 1);

        let part = &built[0].parts[0];
        assert_eq!(part.roof.shape, RoofShape::Flat, "{part:?}");
        assert!(
            part.eaves > 10.0,
            "the walls should have risen to cover it: {part:?}"
        );
        assert!((part.top() - part.eaves).abs() < 1e-9);
    }

    #[test]
    fn a_roof_that_never_rises_above_the_eaves_is_not_a_roof() {
        // A butterfly falls inwards from the eaves: there is nothing above them to
        // describe, and nothing to swallow either.
        let grammar = with_roof("Roof(Butterfly, height=2.5) { All: Tiles }");
        let built = derived(&grammar.each_ref().map(String::as_str));
        assert_eq!(built[0].parts[0].roof, Roof::FLAT);
        assert!((built[0].parts[0].eaves - 10.0).abs() < 1e-6);
    }

    #[test]
    fn a_building_carries_the_frontage_it_was_generated_on() {
        let lot = lot();
        let built = derived(&[r#"Lot --> Extrude(9) I("house")"#]);
        let (building, parts) = built
            .into_iter()
            .next()
            .unwrap()
            .into_building(&lot, 0, 3.2)
            .unwrap();

        let frontage = building
            .frontage
            .expect("a generated building faces a road");
        assert_eq!(frontage.road, lot.road);
        assert_eq!(frontage.side, lot.side);
        assert_eq!(frontage.station, lot.station);
        // The building lists its parts and each part names the building back.
        assert_eq!(building.parts.len(), parts.len());
        assert!(parts.iter().all(|part| part.building == building.id));
        assert_eq!(building.id.to_string(), "building/main/right/0/0");
        assert_eq!(building.parts[0].to_string(), "part/main/right/0/0/0");
        assert!(parts[0].solid.is_closed());
    }

    #[test]
    fn the_same_seed_derives_the_same_buildings() {
        let grammar = &[
            "Lot --> 50% A | else: B",
            r#"A --> Extrude(rand(5, 12)) I("house")"#,
            r#"B --> Extrude(rand(5, 12)) I("retail")"#,
        ];
        let once = derived(grammar);
        let twice = derived(grammar);
        assert_eq!(once.len(), twice.len());
        for (a, b) in once.iter().zip(&twice) {
            assert_eq!(a.kind, b.kind);
            assert_eq!(a.parts[0].eaves, b.parts[0].eaves);
        }
    }
}
