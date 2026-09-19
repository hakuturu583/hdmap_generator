//! What every format is read into before anything is drawn.
//!
//! The four readers in this crate have nothing in common — a Parquet table, an XML
//! attribute of packed triples, a JSON polyline, a lane width polynomial — and they
//! all end here, as plan-view shapes carrying what they *are* rather than how they
//! should look. Which is the same separation the exporters have in the other
//! direction: one model in the middle, and the encodings at the edges know only
//! about themselves.
//!
//! So [`Kind`] is a vocabulary of road things, not of colours. Colour is decided once,
//! in [`crate::svg`], and a reader never writes a hex code.

/// A point in the plane, metres, x east and y north.
///
/// Three of the four formats carry heights and one of them does not. The drawing
/// drops them all: this is a plan view, and a viewer that quietly projected a graded
/// road would be claiming the elevation is not there. [`Drawing::notes`] says it
/// instead.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Point {
    pub x: f64,
    pub y: f64,
}

impl Point {
    pub const fn new(x: f64, y: f64) -> Point {
        Point { x, y }
    }
}

/// What colour a lane marking is painted, as much of it as every format agrees on.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum MarkColor {
    White,
    Yellow,
    /// Painted, but in something this vocabulary has no name for — blue kerb
    /// markings, a colour a format spells in a way the others do not. Drawn as
    /// painted rather than as a colour it may not be.
    Other,
}

/// A painted line, as the two properties that change how it is drawn.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Mark {
    pub color: MarkColor,
    pub broken: bool,
}

impl Mark {
    pub const fn new(color: MarkColor, broken: bool) -> Mark {
        Mark { color, broken }
    }
}

/// What a shape is. The z-order the layers are drawn in is the order of this enum,
/// so an area lands under the lines that bound it and a device on top of everything.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum Kind {
    /// Ground: a verge, a patch of terrain. First of all, because everything else
    /// in a map stands on it.
    Terrain,
    /// A building's footprint, as an area. Early, so that everything the road
    /// network is made of is drawn over the town rather than under it.
    Building,
    /// A junction's extent, as an area.
    Junction,
    /// The ground a lane covers. Drawn where the format says how wide a lane is —
    /// which is not everywhere: GPUDrive's map is centrelines and has no widths at
    /// all, so its picture has no surface under them.
    Surface,
    /// A pavement, a kerb, a gutter: the made ground beside the carriageway that is
    /// not part of it. Over the surface, because a kerb stands on the road it edges.
    Sidewalk,
    /// The edge of the drivable surface, painted or not.
    Boundary,
    /// A painted line between lanes.
    Marking(Mark),
    /// Where a lane's traffic runs, as a line down the middle of it.
    Center,
    /// The route something drives through the map: an ego track, an agent's logged
    /// path.
    Track,
    /// A movement from one lane to another that the format states but does not draw
    /// — a SUMO `<connection>` is a pair of lane names and no geometry.
    Movement,
    Crosswalk,
    StopLine,
    /// A junction corner, a road end: a place the format names as a point.
    Node,
    TrafficLight,
    TrafficSign,
    /// An agent, as the box it occupies.
    Agent,
}

// `Mark` has to be ordered for `Kind` to be, and the order within markings does not
// matter — they are all drawn in the same pass.
impl PartialOrd for Mark {
    fn partial_cmp(&self, other: &Mark) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for Mark {
    fn cmp(&self, other: &Mark) -> std::cmp::Ordering {
        (self.color as u8, self.broken).cmp(&(other.color as u8, other.broken))
    }
}

/// One thing to draw.
#[derive(Debug, Clone, PartialEq)]
pub enum Shape {
    /// An open polyline.
    Path { kind: Kind, points: Vec<Point> },
    /// A closed outline, filled.
    Area { kind: Kind, points: Vec<Point> },
    /// A polyline with a width in metres: a lane known by its centreline and how
    /// wide it is, rather than by its two edges.
    Band {
        kind: Kind,
        points: Vec<Point>,
        width: f64,
    },
    /// A single position.
    Dot { kind: Kind, at: Point },
    /// A box on the ground: something with an extent and a direction, which is what
    /// an agent and a traffic light both are.
    Box {
        kind: Kind,
        at: Point,
        length: f64,
        width: f64,
        /// Radians counterclockwise from +x.
        heading: f64,
    },
}

impl Shape {
    pub fn kind(&self) -> Kind {
        match self {
            Shape::Path { kind, .. }
            | Shape::Area { kind, .. }
            | Shape::Band { kind, .. }
            | Shape::Dot { kind, .. }
            | Shape::Box { kind, .. } => *kind,
        }
    }

    /// The points the shape's extent is measured over. A box is measured over its
    /// four corners, and a band over the width it is drawn with, so neither a long
    /// vehicle nor a wide lane near the edge ends up half outside the picture.
    pub fn extent(&self) -> Vec<Point> {
        match self {
            Shape::Path { points, .. } | Shape::Area { points, .. } => points.clone(),
            // A band is a stroked line: it reaches half its width to either side of
            // the centreline, whichever way the centreline runs. Two opposite corners
            // of that square per vertex is all an extent has to say — the reach is the
            // same in x and y, and nothing here needs it tighter than that.
            Shape::Band { points, width, .. } => {
                let half = width / 2.0;
                points
                    .iter()
                    .flat_map(|point| {
                        [
                            Point::new(point.x - half, point.y - half),
                            Point::new(point.x + half, point.y + half),
                        ]
                    })
                    .collect()
            }
            Shape::Dot { at, .. } => vec![*at],
            Shape::Box { .. } => self.corners().unwrap_or_default(),
        }
    }

    /// The four corners of a [`Shape::Box`], counterclockwise from the near right.
    pub fn corners(&self) -> Option<Vec<Point>> {
        let Shape::Box {
            at,
            length,
            width,
            heading,
            ..
        } = self
        else {
            return None;
        };
        let (sin, cos) = heading.sin_cos();
        let (half_length, half_width) = (length / 2.0, width / 2.0);
        Some(
            [
                (half_length, -half_width),
                (half_length, half_width),
                (-half_length, half_width),
                (-half_length, -half_width),
            ]
            .into_iter()
            .map(|(along, across)| {
                Point::new(
                    at.x + along * cos - across * sin,
                    at.y + along * sin + across * cos,
                )
            })
            .collect(),
        )
    }
}

/// A rectangle in map coordinates.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Bounds {
    pub min: Point,
    pub max: Point,
}

impl Bounds {
    pub fn width(&self) -> f64 {
        self.max.x - self.min.x
    }

    pub fn height(&self) -> f64 {
        self.max.y - self.min.y
    }
}

/// Everything read out of one exported file, ready to draw.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Drawing {
    /// The format this came out of, for the picture's `<title>`.
    pub title: String,
    /// What the reader found, and what the format could not tell it. Shown as the
    /// picture's `<desc>`, which is where a screen reader looks and where a caller
    /// who wants a caption finds one.
    pub notes: Vec<String>,
    pub shapes: Vec<Shape>,
}

impl Drawing {
    pub fn new(title: impl Into<String>) -> Drawing {
        Drawing {
            title: title.into(),
            notes: Vec::new(),
            shapes: Vec::new(),
        }
    }

    pub fn note(&mut self, note: impl Into<String>) {
        self.notes.push(note.into());
    }

    pub fn push(&mut self, shape: Shape) {
        self.shapes.push(shape);
    }

    /// A polyline, dropped when it is a single point: an open path of one vertex has
    /// nothing to draw and every format produces one now and then, where a lane is
    /// shorter than the sampling step.
    pub fn path(&mut self, kind: Kind, points: Vec<Point>) {
        if points.len() >= 2 {
            self.push(Shape::Path { kind, points });
        }
    }

    /// A closed outline, dropped when it is not one — too few points, or no area
    /// between them, which is what a vertical face is from above.
    pub fn area(&mut self, kind: Kind, points: Vec<Point>) {
        if points.len() >= 3 && plan_area(&points) > 1e-6 {
            self.push(Shape::Area { kind, points });
        }
    }

    /// A lane's ground, dropped when there is no line to lay it along.
    pub fn band(&mut self, kind: Kind, points: Vec<Point>, width: f64) {
        if points.len() >= 2 && width > 0.0 {
            self.push(Shape::Band {
                kind,
                points,
                width,
            });
        }
    }

    /// What every shape covers, or `None` when there is nothing to cover it.
    pub fn bounds(&self) -> Option<Bounds> {
        let mut bounds: Option<Bounds> = None;
        for point in self.shapes.iter().flat_map(Shape::extent) {
            if !point.x.is_finite() || !point.y.is_finite() {
                continue;
            }
            bounds = Some(match bounds {
                None => Bounds {
                    min: point,
                    max: point,
                },
                Some(seen) => Bounds {
                    min: Point::new(seen.min.x.min(point.x), seen.min.y.min(point.y)),
                    max: Point::new(seen.max.x.max(point.x), seen.max.y.max(point.y)),
                },
            });
        }
        bounds
    }

    /// The kinds present, once each, in drawing order. This is what the legend is
    /// built from, so a picture explains exactly the things it contains.
    pub fn kinds(&self) -> Vec<Kind> {
        let mut kinds: Vec<Kind> = self.shapes.iter().map(Shape::kind).collect();
        kinds.sort();
        kinds.dedup();
        kinds
    }

    /// Draws this as a standalone SVG document.
    pub fn to_svg(&self) -> String {
        crate::svg::render(self)
    }
}

/// Twice the area of a ring, which is zero for a ring that is really a line.
fn plan_area(ring: &[Point]) -> f64 {
    (0..ring.len())
        .map(|index| {
            let (a, b) = (ring[index], ring[(index + 1) % ring.len()]);
            a.x * b.y - b.x * a.y
        })
        .sum::<f64>()
        .abs()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_drawing_covers_every_shape_it_holds() {
        let mut drawing = Drawing::new("test");
        drawing.path(
            Kind::Center,
            vec![Point::new(0.0, 0.0), Point::new(10.0, 4.0)],
        );
        drawing.push(Shape::Dot {
            kind: Kind::Node,
            at: Point::new(-5.0, 20.0),
        });
        let bounds = drawing.bounds().unwrap();
        assert_eq!(bounds.min, Point::new(-5.0, 0.0));
        assert_eq!(bounds.max, Point::new(10.0, 20.0));
    }

    #[test]
    fn a_band_is_measured_over_the_width_it_is_drawn_with() {
        // A band is a stroked line, so a picture fitted to its centreline alone clips
        // half the lane off either side of it.
        let mut drawing = Drawing::new("test");
        drawing.push(Shape::Band {
            kind: Kind::Surface,
            points: vec![Point::new(0.0, 0.0), Point::new(10.0, 0.0)],
            width: 4.0,
        });
        let bounds = drawing.bounds().unwrap();
        assert_eq!(bounds.min, Point::new(-2.0, -2.0));
        assert_eq!(bounds.max, Point::new(12.0, 2.0));
    }

    #[test]
    fn a_path_of_one_point_is_not_a_path() {
        let mut drawing = Drawing::new("test");
        drawing.path(Kind::Center, vec![Point::new(0.0, 0.0)]);
        assert!(drawing.shapes.is_empty());
        assert!(drawing.bounds().is_none());
    }

    #[test]
    fn a_box_is_measured_over_its_corners() {
        // A 4 m by 2 m box turned a quarter turn is 2 m along x and 4 m along y.
        let drawing = Drawing {
            title: "test".into(),
            notes: Vec::new(),
            shapes: vec![Shape::Box {
                kind: Kind::Agent,
                at: Point::new(0.0, 0.0),
                length: 4.0,
                width: 2.0,
                heading: std::f64::consts::FRAC_PI_2,
            }],
        };
        let bounds = drawing.bounds().unwrap();
        assert!((bounds.width() - 2.0).abs() < 1e-9);
        assert!((bounds.height() - 4.0).abs() < 1e-9);
    }

    #[test]
    fn the_legend_lists_each_kind_once_in_drawing_order() {
        let mut drawing = Drawing::new("test");
        let pair = vec![Point::new(0.0, 0.0), Point::new(1.0, 0.0)];
        drawing.path(Kind::Center, pair.clone());
        drawing.path(Kind::Boundary, pair.clone());
        drawing.path(Kind::Center, pair);
        assert_eq!(drawing.kinds(), vec![Kind::Boundary, Kind::Center]);
    }
}
