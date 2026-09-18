//! What stands beside the road, as a solid.
//!
//! A building is not road furniture. A traffic light governs lanes, a stop line is
//! painted on one, and both live in [`crate::semantics`] because they mean something
//! to traffic. A building means nothing to traffic: it is a volume on the ground that
//! the road happens to run past. So it is its own thing, with its own identifiers and
//! its own arenas on the [`Map`](crate::map::Map).
//!
//! # The shape of the model
//!
//! The same four ideas the rest of this IR is built on, applied to a building.
//!
//! **A feature, composed of simpler features.** A [`Building`] is not a shape; it is
//! a set of [`BuildingPart`]s and what they add up to. A house with a wing behind it,
//! a tower standing on a podium, a shop with a flat over it: one building, several
//! parts, each with a volume of its own. That is the same relationship a [`Road`] has
//! to its [`Lane`]s, kept the same way — the building lists its parts, each part names
//! its building, and both live in arenas keyed by identifier.
//!
//! [`Road`]: crate::map::Road
//! [`Lane`]: crate::map::Lane
//!
//! **Geometry apart from semantics.** A part's [`Solid`] says where it is and nothing
//! about what it is for; a building's `kind` says what it is for and nothing about
//! where it is. Neither mentions a file format.
//!
//! **Relationships as objects.** Which road a building faces is a fact about the
//! building *and* the road, so it is neither's field: it is a [`Frontage`], which
//! names the road, the side and the station. An exporter that has to place a building
//! in some road's coordinates reads it instead of searching, and a caller who wants
//! the buildings of one street can ask for them.
//!
//! **Identifiers derived from the input.** `building/high_street/left/3` and
//! `part/high_street/left/3/0`, from the frontage the building was generated on,
//! never from a counter.
//!
//! # Solid, not footprint
//!
//! A [`Solid`] is a volume and [`Solid::shell`] hands back the faces that bound it —
//! the base, one quad per wall, and the roof. It is stated compactly, as an outline
//! and the heights above it, because that is what a building *is* and what every
//! format that can hold one asks for; but it is a closed surface in three dimensions
//! and [`Solid::shell`] is where to go and look at it.
//!
//! A part's outline carries its own heights, so a part that starts partway up a
//! building needs nothing extra to say so, and a part on sloping ground is on sloping
//! ground.
//!
//! # Where they come from
//!
//! Nothing in this crate makes buildings. `roadgen-core` is the model; the
//! `roadgen-buildings` crate is a generator that fills it, in the same way the
//! exporter crates read it. The IR would rather hold empty arenas than depend on a
//! shape grammar.

use std::f64::consts::PI;

use crate::error::GeometryError;
use crate::geometry::Point3;
use crate::id::{BuildingId, BuildingPartId, RoadId};
use crate::topology::LateralSide;

/// A closed outline in space.
///
/// The ring is stored **open** — the first vertex is not repeated at the end — and
/// **anticlockwise** seen from above, whichever way round the caller wrote it. Both
/// are normalised here rather than left to each exporter: OSM, OpenDRIVE and every
/// renderer want the same winding, and an outline that closes itself twice is a
/// classic way to produce a way with a duplicate node.
///
/// The vertices keep their own heights. A building on a slope has an outline that is
/// not level, and flattening it here would be deciding, in the IR, that the ground is
/// flat.
#[derive(Debug, Clone, PartialEq)]
pub struct Footprint {
    ring: Vec<Point3>,
}

impl Footprint {
    /// Builds an outline from a ring of vertices, closed or not, either winding.
    ///
    /// Fails when fewer than three distinct vertices survive, when a coordinate is
    /// not finite, or when the ring encloses no area in plan — a "polygon" whose
    /// vertices are collinear is a line, and no consumer of one draws anything.
    pub fn new(points: impl IntoIterator<Item = Point3>) -> Result<Self, GeometryError> {
        let mut given: Vec<Point3> = Vec::new();
        for point in points {
            if !point.x.is_finite() || !point.y.is_finite() || !point.z.is_finite() {
                return Err(GeometryError::NonFiniteCoordinate);
            }
            given.push(point);
        }
        // A ring written closed repeats its first vertex; so does one whose generator
        // emitted the same corner twice. Neither is a vertex.
        let mut ring = ring_without_repeats(given);
        if ring.len() < 3 {
            return Err(GeometryError::TooFewPoints { got: ring.len() });
        }

        let area = signed_area(&ring);
        if area.abs() < 1e-9 {
            return Err(GeometryError::ZeroArea);
        }
        if area < 0.0 {
            ring.reverse();
        }
        Ok(Footprint { ring })
    }

    /// The vertices, anticlockwise, without the closing repeat.
    pub fn points(&self) -> &[Point3] {
        &self.ring
    }

    pub fn len(&self) -> usize {
        self.ring.len()
    }

    /// Never true: an outline has at least three vertices by construction.
    pub fn is_empty(&self) -> bool {
        false
    }

    /// Area enclosed in the horizontal plane, square metres. Always positive.
    pub fn area(&self) -> f64 {
        signed_area(&self.ring)
    }

    /// The centre of area in plan, at the mean height of the ring.
    pub fn centroid(&self) -> Point3 {
        let area = self.area();
        let (mut x, mut y) = (0.0, 0.0);
        for index in 0..self.ring.len() {
            let a = self.ring[index];
            let b = self.ring[(index + 1) % self.ring.len()];
            let cross = a.x * b.y - b.x * a.y;
            x += (a.x + b.x) * cross;
            y += (a.y + b.y) * cross;
        }
        let z = self.ring.iter().map(|point| point.z).sum::<f64>() / self.ring.len() as f64;
        Point3::new(x / (6.0 * area), y / (6.0 * area), z)
    }

    /// The lowest vertex height.
    pub fn lowest(&self) -> f64 {
        self.ring
            .iter()
            .map(|point| point.z)
            .fold(f64::INFINITY, f64::min)
    }

    /// The highest vertex height. Equal to [`Footprint::lowest`] on level ground.
    pub fn highest(&self) -> f64 {
        self.ring
            .iter()
            .map(|point| point.z)
            .fold(f64::NEG_INFINITY, f64::max)
    }

    /// The same outline, every vertex raised by `rise` metres.
    pub fn raised(&self, rise: f64) -> Footprint {
        Footprint {
            ring: self
                .ring
                .iter()
                .map(|point| Point3::new(point.x, point.y, point.z + rise))
                .collect(),
        }
    }
}

/// Twice the signed area of the ring in the horizontal plane, halved: positive when
/// the ring runs anticlockwise seen from above.
fn signed_area(ring: &[Point3]) -> f64 {
    let mut total = 0.0;
    for index in 0..ring.len() {
        let a = ring[index];
        let b = ring[(index + 1) % ring.len()];
        total += a.x * b.y - b.x * a.y;
    }
    total / 2.0
}

/// What tops a part's walls.
///
/// The five shapes are the ones a consumer can be told about — they are OSM's
/// `roof:shape` values, and the set stops there because a word no format understands
/// is a word that does nothing. A roof a generator cannot say in these terms is not
/// approximated with the nearest one: it becomes the flat top of the volume that
/// contains it, so the massing stays right and only the word is missing.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Hash)]
pub enum RoofShape {
    /// The walls are the whole of it.
    #[default]
    Flat,
    /// One slope, falling across the building.
    Skillion,
    /// Two slopes meeting at a ridge, with a vertical triangle at each end.
    Gabled,
    /// Two slopes and two more falling to the ends, so every side slopes.
    Hipped,
    /// Every side slopes to a single point.
    Pyramidal,
}

impl RoofShape {
    /// Whether the shape needs a direction to be pinned down.
    ///
    /// A pyramid points nowhere and a flat roof has no ridge; the other three run a
    /// ridge, or fall, along one.
    pub fn is_directed(self) -> bool {
        matches!(
            self,
            RoofShape::Skillion | RoofShape::Gabled | RoofShape::Hipped
        )
    }

    /// The OSM `roof:shape` value, which is also how this IR spells it.
    pub fn as_str(self) -> &'static str {
        match self {
            RoofShape::Flat => "flat",
            RoofShape::Skillion => "skillion",
            RoofShape::Gabled => "gabled",
            RoofShape::Hipped => "hipped",
            RoofShape::Pyramidal => "pyramidal",
        }
    }
}

/// A part's roof.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Roof {
    pub shape: RoofShape,
    /// How far the roof rises above the eaves, metres. Zero exactly when the shape is
    /// [`RoofShape::Flat`].
    pub height: f64,
    /// Which way the ridge runs, or a skillion falls: radians anticlockwise from the
    /// `+x` axis. Zero, and meaningless, for the shapes that need no direction.
    pub direction: f64,
}

impl Roof {
    /// A roof that is only the top of the walls.
    pub const FLAT: Roof = Roof {
        shape: RoofShape::Flat,
        height: 0.0,
        direction: 0.0,
    };

    pub fn new(shape: RoofShape, height: f64, direction: f64) -> Roof {
        Roof {
            shape,
            height: if shape == RoofShape::Flat {
                0.0
            } else {
                height
            },
            direction: if shape.is_directed() { direction } else { 0.0 },
        }
    }
}

/// One face of a solid: a ring of vertices, anticlockwise seen from outside.
pub type Face = Vec<Point3>;

/// The volume one part of a building occupies.
///
/// Stated as an outline and the heights above it. That is compact, it is what every
/// format that can hold a building asks for, and it is what a massing model produces
/// — but it is a *solid*, and [`Solid::shell`] is the surface that bounds it.
#[derive(Debug, Clone, PartialEq)]
pub struct Solid {
    /// Where the walls meet the part's base, with each vertex at its own height.
    pub footprint: Footprint,
    /// How far the walls rise above each base vertex, metres. Always positive.
    pub wall_height: f64,
    pub roof: Roof,
}

impl Solid {
    /// A part with a flat roof: walls and nothing else.
    pub fn prism(footprint: Footprint, wall_height: f64) -> Solid {
        Solid {
            footprint,
            wall_height,
            roof: Roof::FLAT,
        }
    }

    pub fn with_roof(mut self, roof: Roof) -> Solid {
        self.roof = roof;
        self
    }

    /// The outline where the walls stop and the roof begins.
    pub fn eaves(&self) -> Footprint {
        self.footprint.raised(self.wall_height)
    }

    /// Height of the lowest point of the part.
    pub fn base_height(&self) -> f64 {
        self.footprint.lowest()
    }

    /// Height of the highest point of the part, roof included.
    pub fn top_height(&self) -> f64 {
        self.footprint.highest() + self.wall_height + self.roof.height
    }

    /// How far the part reaches from its lowest point to its highest.
    pub fn height(&self) -> f64 {
        self.top_height() - self.base_height()
    }

    /// The faces that bound the volume, each anticlockwise seen from outside.
    ///
    /// The base, one quad per wall, and the roof. Every edge of the surface is used
    /// by exactly two faces, which is what makes it a solid rather than a collection
    /// of panels — and what
    /// [`Solid::is_closed`] checks.
    ///
    /// The roof is built by running a ridge through the eaves outline and joining
    /// every eaves edge to the nearest point on it. For a rectangle — which is what a
    /// massing model produces — that is exactly a gable, a hip or a pyramid, with the
    /// ends vertical where they should be. For an outline that is not a rectangle it
    /// is still a closed roof over it, falling from the same ridge.
    pub fn shell(&self) -> Vec<Face> {
        let base = self.footprint.points();
        let eaves = self.eaves();
        let eaves = eaves.points();

        let mut faces = Vec::with_capacity(base.len() + 2);
        // The base, seen from below, so it winds the other way round.
        faces.push(base.iter().rev().copied().collect::<Face>());
        for index in 0..base.len() {
            let next = (index + 1) % base.len();
            faces.push(vec![base[index], base[next], eaves[next], eaves[index]]);
        }
        faces.extend(roof_faces(eaves, &self.roof));
        faces
    }

    /// Whether [`Solid::shell`] closes: every edge shared by exactly two faces.
    ///
    /// Not a validation rule — the construction cannot produce anything else — but
    /// the property the construction exists to have, so it is stated where it can be
    /// checked.
    pub fn is_closed(&self) -> bool {
        let mut edges: Vec<([i64; 6], usize)> = Vec::new();
        for face in self.shell() {
            if face.len() < 3 {
                return false;
            }
            for index in 0..face.len() {
                let key = edge_key(face[index], face[(index + 1) % face.len()]);
                match edges.iter_mut().find(|(seen, _)| *seen == key) {
                    Some((_, count)) => *count += 1,
                    None => edges.push((key, 1)),
                }
            }
        }
        !edges.is_empty() && edges.iter().all(|(_, count)| *count == 2)
    }
}

/// An undirected edge, quantised so that two faces meeting along it agree.
fn edge_key(a: Point3, b: Point3) -> [i64; 6] {
    let round = |value: f64| (value * 1e6).round() as i64;
    let (a, b) = (
        [round(a.x), round(a.y), round(a.z)],
        [round(b.x), round(b.y), round(b.z)],
    );
    let (low, high) = if a <= b { (a, b) } else { (b, a) };
    [low[0], low[1], low[2], high[0], high[1], high[2]]
}

/// The faces of a roof over `eaves`.
fn roof_faces(eaves: &[Point3], roof: &Roof) -> Vec<Face> {
    if roof.shape == RoofShape::Flat || roof.height <= 0.0 {
        return vec![eaves.to_vec()];
    }
    if roof.shape == RoofShape::Skillion {
        return skillion_faces(eaves, roof);
    }

    // The ridge: a segment through the centroid along the roof's direction, at the
    // ridge height. How far it stops short of the outline at each end is what makes
    // it a gable, a hip or a pyramid.
    //
    // Every eaves vertex is carried up to whichever *end* of the ridge it is nearer,
    // rather than to the nearest point along it. That is what makes the surface
    // close: the two ends are the only vertices the roof has, so no face can subdivide
    // the ridge in a way its neighbour does not. On a rectangle — which is what a
    // massing model produces — the two give the same answer, and it is the right one.
    let (from, to) = ridge(eaves, roof);
    let nearer = |point: &Point3| {
        if point.horizontal_distance_to(from) <= point.horizontal_distance_to(to) {
            from
        } else {
            to
        }
    };

    let mut faces = Vec::with_capacity(eaves.len());
    for index in 0..eaves.len() {
        let (a, b) = (eaves[index], eaves[(index + 1) % eaves.len()]);
        let (ridge_a, ridge_b) = (nearer(&a), nearer(&b));
        // An edge whose ends go to the same place — a gable end, a hip end, every
        // side of a pyramid — closes as a triangle.
        if ridge_a.is_close(ridge_b, 1e-9) {
            faces.push(vec![a, b, ridge_a]);
        } else {
            faces.push(vec![a, b, ridge_b, ridge_a]);
        }
    }
    faces
}

/// The ridge segment of a gabled, hipped or pyramidal roof over `eaves`.
fn ridge(eaves: &[Point3], roof: &Roof) -> (Point3, Point3) {
    let (cos, sin) = (roof.direction.cos(), roof.direction.sin());
    let centre = centre_of(eaves);
    let along = |point: &Point3| (point.x - centre.x) * cos + (point.y - centre.y) * sin;
    let across = |point: &Point3| -(point.x - centre.x) * sin + (point.y - centre.y) * cos;

    let half_length = eaves
        .iter()
        .map(|point| along(point).abs())
        .fold(0.0, f64::max);
    let half_width = eaves
        .iter()
        .map(|point| across(point).abs())
        .fold(0.0, f64::max);
    // A gable runs the ridge the whole way and leaves the ends vertical; a hip pulls
    // it in by half the width at each end, which is the slope the sides already have;
    // a pyramid pulls it in until there is no ridge left.
    let inset = match roof.shape {
        RoofShape::Gabled => 0.0,
        RoofShape::Hipped => half_width.min(half_length),
        _ => half_length,
    };
    let reach = (half_length - inset).max(0.0);
    let height = eaves
        .iter()
        .map(|point| point.z)
        .fold(f64::NEG_INFINITY, f64::max)
        + roof.height;
    (
        Point3::new(centre.x - reach * cos, centre.y - reach * sin, height),
        Point3::new(centre.x + reach * cos, centre.y + reach * sin, height),
    )
}

/// One slope, falling in the roof's direction.
///
/// The eaves rise linearly along that direction, which keeps the top one flat face
/// however many sides the outline has; the wedge between the level eaves and the
/// raised ones is closed by a gore per edge.
fn skillion_faces(eaves: &[Point3], roof: &Roof) -> Vec<Face> {
    let (cos, sin) = (roof.direction.cos(), roof.direction.sin());
    let along: Vec<f64> = eaves.iter().map(|p| p.x * cos + p.y * sin).collect();
    let low = along.iter().copied().fold(f64::INFINITY, f64::min);
    let high = along.iter().copied().fold(f64::NEG_INFINITY, f64::max);
    let span = high - low;

    let raise = |index: usize| {
        let fraction = if span > 1e-12 {
            (along[index] - low) / span
        } else {
            0.0
        };
        let point = eaves[index];
        Point3::new(point.x, point.y, point.z + roof.height * fraction)
    };

    let mut faces = Vec::with_capacity(eaves.len() + 1);
    for index in 0..eaves.len() {
        let next = (index + 1) % eaves.len();
        // Along the low edge the gore has no height and collapses to nothing; where
        // one end of an edge is on it, to a triangle.
        let gore = ring_without_repeats(vec![eaves[index], eaves[next], raise(next), raise(index)]);
        if gore.len() >= 3 {
            faces.push(gore);
        }
    }
    faces.push((0..eaves.len()).map(raise).collect());
    faces
}

/// A ring with consecutive duplicate vertices removed, the first and last counting as
/// consecutive.
fn ring_without_repeats(points: Vec<Point3>) -> Face {
    let mut ring: Vec<Point3> = Vec::with_capacity(points.len());
    for point in points {
        if ring.last().is_some_and(|last| last.is_close(point, 1e-9)) {
            continue;
        }
        ring.push(point);
    }
    while ring.len() > 1 && ring[0].is_close(ring[ring.len() - 1], 1e-9) {
        ring.pop();
    }
    ring
}

fn centre_of(ring: &[Point3]) -> Point3 {
    let count = ring.len() as f64;
    Point3::new(
        ring.iter().map(|point| point.x).sum::<f64>() / count,
        ring.iter().map(|point| point.y).sum::<f64>() / count,
        ring.iter().map(|point| point.z).sum::<f64>() / count,
    )
}

/// Which road a building stands beside.
///
/// A relationship rather than a coordinate. It says nothing about where the building
/// is — the geometry does that, completely — and everything about what it belongs to:
/// which street it is on, which side of it, and how far along. An exporter that has to
/// put a building in some road's own coordinates reads this instead of searching for
/// the nearest road, and a caller who wants the buildings of one street has a way to
/// ask.
#[derive(Debug, Clone, PartialEq)]
pub struct Frontage {
    pub road: RoadId,
    /// Which side of the road's reference line the building stands on.
    pub side: LateralSide,
    /// Where along the reference line it faces, metres from the road's start.
    pub station: f64,
}

/// One building: what it is, what it is made of, and what it stands beside.
///
/// Not a shape. The shape is in the parts, and a building that is one prism has one
/// part rather than a special case.
#[derive(Debug, Clone, PartialEq)]
pub struct Building {
    pub id: BuildingId,
    /// The parts it is made of, in the order they were generated. Never empty.
    pub parts: Vec<BuildingPartId>,
    /// What the building is — `house`, `apartments`, `retail`.
    ///
    /// The IR defines no catalogue of these, exactly as it defines none for a traffic
    /// sign's code. What it does do is pass the word through unchanged, so an
    /// exporter whose format has a vocabulary can use it and one whose format has
    /// none can ignore it.
    pub kind: String,
    /// The road it faces, when it was put there in relation to one.
    pub frontage: Option<Frontage>,
}

/// One massing element of a building.
#[derive(Debug, Clone, PartialEq)]
pub struct BuildingPart {
    pub id: BuildingPartId,
    pub building: BuildingId,
    pub solid: Solid,
    /// What this part is, when it is something of its own.
    ///
    /// A part is a feature in its own right and may be used for something its
    /// building as a whole is not: a tower of offices standing on a shopping podium
    /// is one building, and neither word describes both halves of it. `None` when the
    /// part is simply part of the building and has nothing to add.
    pub kind: Option<String>,
    /// Storeys within this part, at least one. Derived from its height rather than
    /// designed: the IR has no interior, and a consumer that draws a facade wants a
    /// floor count.
    pub levels: u32,
}

/// Radians anticlockwise from the `+x` axis, folded into `[0, π)`.
///
/// A ridge has no front and no back: pointing it the other way is the same ridge, and
/// a generator that measured it from the other end should not produce a different
/// building.
pub fn ridge_direction(radians: f64) -> f64 {
    let folded = radians.rem_euclid(PI);
    if folded.abs() < 1e-12 || (PI - folded).abs() < 1e-12 {
        0.0
    } else {
        folded
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn rectangle(width: f64, depth: f64, base: f64) -> Footprint {
        Footprint::new(vec![
            Point3::new(0.0, 0.0, base),
            Point3::new(width, 0.0, base),
            Point3::new(width, depth, base),
            Point3::new(0.0, depth, base),
        ])
        .unwrap()
    }

    #[test]
    fn an_outline_comes_out_anticlockwise_whichever_way_it_went_in() {
        let forwards = rectangle(10.0, 10.0, 0.0);
        let mut reversed: Vec<Point3> = forwards.points().to_vec();
        reversed.reverse();
        let backwards = Footprint::new(reversed).unwrap();

        assert!(forwards.area() > 0.0);
        assert_eq!(forwards.area(), backwards.area());
        assert_eq!(forwards.len(), backwards.len());
    }

    #[test]
    fn a_ring_written_closed_does_not_keep_the_repeat() {
        let mut closed: Vec<Point3> = rectangle(10.0, 10.0, 0.0).points().to_vec();
        closed.push(closed[0]);
        assert_eq!(Footprint::new(closed).unwrap().len(), 4);
    }

    #[test]
    fn a_line_is_not_an_outline() {
        let collinear = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(5.0, 0.0, 0.0),
            Point3::new(10.0, 0.0, 0.0),
        ];
        assert_eq!(Footprint::new(collinear), Err(GeometryError::ZeroArea));
        assert_eq!(
            Footprint::new(vec![Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0)]),
            Err(GeometryError::TooFewPoints { got: 2 })
        );
    }

    #[test]
    fn an_outline_keeps_the_height_of_the_ground_it_sits_on() {
        let sloping = Footprint::new(vec![
            Point3::new(0.0, 0.0, 10.0),
            Point3::new(10.0, 0.0, 11.0),
            Point3::new(10.0, 10.0, 11.5),
            Point3::new(0.0, 10.0, 10.5),
        ])
        .unwrap();
        assert_eq!(sloping.lowest(), 10.0);
        assert_eq!(sloping.highest(), 11.5);
        assert_eq!(sloping.raised(2.0).lowest(), 12.0);
    }

    #[test]
    fn a_flat_topped_prism_is_a_box() {
        let solid = Solid::prism(rectangle(10.0, 8.0, 3.0), 6.0);
        assert_eq!(solid.base_height(), 3.0);
        assert_eq!(solid.top_height(), 9.0);
        assert_eq!(solid.height(), 6.0);

        let shell = solid.shell();
        // A base, four walls and a top.
        assert_eq!(shell.len(), 6);
        assert!(solid.is_closed());
    }

    #[test]
    fn a_gable_puts_a_ridge_the_length_of_the_building() {
        // The ridge runs along +x, which is the long axis of this 20 x 8 plan.
        let solid = Solid::prism(rectangle(20.0, 8.0, 0.0), 5.0).with_roof(Roof::new(
            RoofShape::Gabled,
            3.0,
            0.0,
        ));
        assert_eq!(solid.top_height(), 8.0);
        assert!(solid.is_closed());

        let roof: Vec<_> = solid.shell().into_iter().skip(5).collect();
        assert_eq!(roof.len(), 4);
        // Two slopes of four corners, two vertical gable triangles.
        let mut sizes: Vec<usize> = roof.iter().map(|face| face.len()).collect();
        sizes.sort();
        assert_eq!(sizes, vec![3, 3, 4, 4]);
        // The ridge is as long as the building and runs down its middle.
        let ridge: Vec<&Point3> = roof
            .iter()
            .flatten()
            .filter(|point| (point.z - 8.0).abs() < 1e-9)
            .collect();
        assert!(ridge.iter().all(|point| (point.y - 4.0).abs() < 1e-9));
        let along: Vec<f64> = ridge.iter().map(|point| point.x).collect();
        assert!((along.iter().cloned().fold(f64::INFINITY, f64::min)).abs() < 1e-9);
        assert!((along.iter().cloned().fold(0.0, f64::max) - 20.0).abs() < 1e-9);
    }

    #[test]
    fn a_hip_pulls_the_ridge_in_and_slopes_every_side() {
        let solid = Solid::prism(rectangle(20.0, 8.0, 0.0), 5.0).with_roof(Roof::new(
            RoofShape::Hipped,
            3.0,
            0.0,
        ));
        assert!(solid.is_closed());

        let roof: Vec<_> = solid.shell().into_iter().skip(5).collect();
        assert_eq!(roof.len(), 4);
        // The ends are triangles that slope rather than vertical gables: their apex
        // is pulled in from the wall by half the building's width.
        let ends: Vec<&Face> = roof.iter().filter(|face| face.len() == 3).collect();
        assert_eq!(ends.len(), 2);
        for end in ends {
            let apex = end.iter().find(|point| point.z > 5.0).unwrap();
            assert!((apex.y - 4.0).abs() < 1e-9);
            assert!(apex.x > 3.0 && apex.x < 17.0, "{apex:?}");
        }
    }

    #[test]
    fn a_pyramid_meets_at_a_point() {
        let solid = Solid::prism(rectangle(10.0, 10.0, 0.0), 4.0).with_roof(Roof::new(
            RoofShape::Pyramidal,
            5.0,
            0.0,
        ));
        assert!(solid.is_closed());

        let roof: Vec<_> = solid.shell().into_iter().skip(5).collect();
        assert_eq!(roof.len(), 4);
        assert!(roof.iter().all(|face| face.len() == 3));
        let apex = roof[0].iter().find(|point| point.z > 4.0).unwrap();
        assert!((apex.x - 5.0).abs() < 1e-9 && (apex.y - 5.0).abs() < 1e-9);
        assert!((apex.z - 9.0).abs() < 1e-9);
    }

    #[test]
    fn a_skillion_falls_one_way_and_still_closes() {
        let solid = Solid::prism(rectangle(10.0, 6.0, 0.0), 4.0).with_roof(Roof::new(
            RoofShape::Skillion,
            2.0,
            0.0,
        ));
        assert!(solid.is_closed());
        assert_eq!(solid.top_height(), 6.0);
        // The high edge is the one furthest along the roof's direction.
        let shell = solid.shell();
        let highest: Vec<&Point3> = shell
            .iter()
            .flatten()
            .filter(|point| (point.z - 6.0).abs() < 1e-9)
            .collect();
        assert!(!highest.is_empty());
        assert!(highest.iter().all(|point| (point.x - 10.0).abs() < 1e-9));
    }

    #[test]
    fn a_roof_over_a_plan_that_is_not_a_rectangle_still_closes() {
        let hexagon = Footprint::new(
            (0..6)
                .map(|index| {
                    let angle = index as f64 * PI / 3.0;
                    Point3::new(8.0 * angle.cos(), 8.0 * angle.sin(), 0.0)
                })
                .collect::<Vec<_>>(),
        )
        .unwrap();
        for shape in [
            RoofShape::Gabled,
            RoofShape::Hipped,
            RoofShape::Pyramidal,
            RoofShape::Skillion,
        ] {
            let solid = Solid::prism(hexagon.clone(), 4.0).with_roof(Roof::new(shape, 2.5, 0.7));
            assert!(solid.is_closed(), "{shape:?} does not close over a hexagon");
        }
    }

    #[test]
    fn a_flat_roof_has_no_height_and_a_directionless_one_no_direction() {
        let flat = Roof::new(RoofShape::Flat, 4.0, 1.2);
        assert_eq!(flat.height, 0.0);
        let pyramid = Roof::new(RoofShape::Pyramidal, 4.0, 1.2);
        assert_eq!(pyramid.direction, 0.0);
        assert_eq!(pyramid.height, 4.0);
    }

    #[test]
    fn a_ridge_pointed_either_way_is_the_same_ridge() {
        for radians in [0.4, 1.9, -0.3, PI / 2.0] {
            let folded = ridge_direction(radians);
            assert!((0.0..PI).contains(&folded), "{radians} folded to {folded}");
            assert!((ridge_direction(radians + PI) - folded).abs() < 1e-12);
            assert!((ridge_direction(radians - PI) - folded).abs() < 1e-12);
        }
        assert_eq!(ridge_direction(PI), 0.0);
        assert_eq!(ridge_direction(0.0), 0.0);
    }
}
