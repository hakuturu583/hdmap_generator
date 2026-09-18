//! What stands beside the road.
//!
//! A building is not road furniture. A traffic light governs lanes, a stop line is
//! painted on one, and both live in [`crate::semantics`] because they mean something
//! to traffic. A building means nothing to traffic: it is a shape on the ground that
//! the road happens to run past. So it is its own thing, with its own identifier and
//! its own arena on the [`Map`](crate::map::Map).
//!
//! # A footprint, and only a footprint
//!
//! What the IR holds is the outline on the ground and how far the building rises
//! above it. There is no facade here, no roof shape, no window: those are surfaces,
//! and a road-network IR has nowhere to put a surface. A generator that wants to
//! describe one — and [`roadgen-buildings`] uses a shape grammar that can describe a
//! great deal — still hands this module an outline and a height, because that is what
//! every consumer of a generated map can actually receive.
//!
//! # Where they come from
//!
//! Nothing in this crate makes buildings. `roadgen-core` is the model; the
//! `roadgen-buildings` crate is a generator that fills it, in the same way the
//! exporter crates read it. The IR would rather hold an empty arena than depend on a
//! shape grammar.
//!
//! [`roadgen-buildings`]: https://github.com/hakuturu583/hdmap_generator

use crate::error::GeometryError;
use crate::geometry::Point3;
use crate::id::BuildingId;

/// A closed outline on the ground.
///
/// The ring is stored **open** — the first vertex is not repeated at the end — and
/// **anticlockwise** seen from above, whichever way round the caller wrote it. Both
/// are normalised here rather than left to each exporter: OSM, OpenDRIVE and every
/// renderer want the same winding, and a footprint that closes itself twice is a
/// classic way to produce a way with a duplicate node.
#[derive(Debug, Clone, PartialEq)]
pub struct Footprint {
    ring: Vec<Point3>,
}

impl Footprint {
    /// Builds a footprint from a ring of vertices, closed or not, either winding.
    ///
    /// Fails when fewer than three distinct vertices survive, when a coordinate is
    /// not finite, or when the ring encloses no area — a "polygon" whose vertices are
    /// collinear is a line, and every consumer of one draws nothing.
    pub fn new(points: impl IntoIterator<Item = Point3>) -> Result<Self, GeometryError> {
        let mut ring: Vec<Point3> = Vec::new();
        for point in points {
            if !point.x.is_finite() || !point.y.is_finite() || !point.z.is_finite() {
                return Err(GeometryError::NonFiniteCoordinate);
            }
            // A ring written closed repeats its first vertex; so does one whose
            // generator emitted the same corner twice. Neither is a vertex.
            if ring.last().is_some_and(|last| last.is_close(point, 1e-9)) {
                continue;
            }
            ring.push(point);
        }
        while ring.len() > 1 && ring[0].is_close(ring[ring.len() - 1], 1e-9) {
            ring.pop();
        }
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

    /// Never true: a footprint has at least three vertices by construction.
    pub fn is_empty(&self) -> bool {
        false
    }

    /// Area enclosed in the horizontal plane, square metres. Always positive.
    pub fn area(&self) -> f64 {
        signed_area(&self.ring)
    }

    /// The centre of area, at the mean height of the ring.
    pub fn centroid(&self) -> Point3 {
        let area = signed_area(&self.ring);
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

    /// The lowest vertex height, which is where the building meets the ground.
    pub fn base_height(&self) -> f64 {
        self.ring
            .iter()
            .map(|point| point.z)
            .fold(f64::INFINITY, f64::min)
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

/// One building.
#[derive(Debug, Clone, PartialEq)]
pub struct Building {
    pub id: BuildingId,
    pub footprint: Footprint,
    /// Metres from the footprint to the top of the building. Always positive.
    pub height: f64,
    /// Storeys, at least one. Derived from the height rather than designed: the IR
    /// has no interior, and a consumer that draws a facade wants a floor count.
    pub levels: u32,
    /// What the generator called this building — `house`, `apartments`, `retail`.
    ///
    /// The IR defines no catalogue of these, exactly as it defines none for a traffic
    /// sign's code. What it does do is pass the word through unchanged, so an
    /// exporter whose format has a vocabulary can use it and one whose format has
    /// none can ignore it.
    pub kind: String,
}

impl Building {
    /// The height of the top of the building above the map's datum.
    pub fn top_height(&self) -> f64 {
        self.footprint.base_height() + self.height
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn square() -> Vec<Point3> {
        vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(10.0, 0.0, 0.0),
            Point3::new(10.0, 10.0, 0.0),
            Point3::new(0.0, 10.0, 0.0),
        ]
    }

    #[test]
    fn a_footprint_comes_out_anticlockwise_whichever_way_it_went_in() {
        let forwards = Footprint::new(square()).unwrap();
        let mut backwards_points = square();
        backwards_points.reverse();
        let backwards = Footprint::new(backwards_points).unwrap();

        assert!(forwards.area() > 0.0);
        assert_eq!(forwards.area(), backwards.area());
        // Same ring, same winding; only the vertex the ring starts at may differ.
        assert_eq!(forwards.len(), backwards.len());
    }

    #[test]
    fn a_ring_written_closed_does_not_keep_the_repeat() {
        let mut closed = square();
        closed.push(closed[0]);
        assert_eq!(Footprint::new(closed).unwrap().len(), 4);
    }

    #[test]
    fn a_line_is_not_a_footprint() {
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
    fn area_and_centroid_are_the_ones_a_square_has() {
        let footprint = Footprint::new(square()).unwrap();
        assert!((footprint.area() - 100.0).abs() < 1e-9);
        let centre = footprint.centroid();
        assert!((centre.x - 5.0).abs() < 1e-9);
        assert!((centre.y - 5.0).abs() < 1e-9);
    }
}
