//! Reading an OpenDRIVE document back.
//!
//! OpenDRIVE does not hold a road as a shape. It holds a reference line as a chain of
//! analytic pieces, and every lane as a polynomial width measured sideways from it —
//! so a picture of an OpenDRIVE file is not read out of it, it is *evaluated* from it.
//! That evaluation is the whole of this module, and it is what makes the picture
//! worth having: it goes wrong in exactly the ways the geometry can, which is what a
//! viewer is for.
//!
//! The document is parsed by the `opendrive` crate — the same one the exporter writes
//! through — so nothing here has an opinion about the XML.
//!
//! # Where the lane borders come from
//!
//! At a station `s` the cross-section is laid out from the reference line: the centre
//! is displaced by the `<laneOffset>` polynomial, and each lane's outer border is the
//! one inside it plus that lane's `<width>` polynomial. Borders are accumulated
//! outwards, left and right separately, which is OpenDRIVE's own rule and the reason a
//! width error in lane 1 moves lanes 2 and 3 as well.
//!
//! A `<border>` lane — width stated as an absolute offset rather than a width — is
//! read as the offset it is. roadgen never writes one; a document from elsewhere may.

use std::f64::consts::PI;

use opendrive::core::OpenDrive;
use opendrive::lane::lane_choice::LaneChoice;
use opendrive::lane::lane_section::LaneSection;
use opendrive::lane::lane_type::LaneType;
use opendrive::lane::road_mark::color::Color;
use opendrive::lane::road_mark::type_simplified::TypeSimplified;
use opendrive::lane::road_mark::RoadMark;
use opendrive::lane::Lane;
use opendrive::road::geometry::geometry_type::GeometryType;
use opendrive::road::Road;
use uom::si::angle::radian;
use uom::si::curvature::radian_per_meter;
use uom::si::length::meter;

use crate::drawing::{Drawing, Kind, Mark, MarkColor, Point};
use crate::error::ViewError;

/// How finely a curved reference line is sampled, metres.
///
/// A straight piece needs two points and gets them; everything else is walked at this
/// step. One metre is finer than the geometry roadgen writes and far finer than a
/// picture can show, which is the point — the sampling should not be what a reader
/// sees.
const STEP: f64 = 1.0;

/// Draws an OpenDRIVE document.
pub fn draw(xml: &str) -> Result<Drawing, ViewError> {
    let document =
        OpenDrive::from_xml_str(xml).map_err(|error| ViewError::Parse(format!("{error:?}")))?;

    let mut drawing = Drawing::new("OpenDRIVE");
    let mut lanes = 0usize;

    for road in &document.road {
        let reference = ReferenceLine::of(road);
        if reference.is_empty() {
            continue;
        }
        for (index, section) in road.lanes.lane_section.iter().enumerate() {
            let end = road
                .lanes
                .lane_section
                .get(index + 1)
                .map(|next| next.s)
                .unwrap_or(reference.length);
            let stations = stations(section.s, end);
            if stations.len() < 2 {
                continue;
            }

            let centre: Vec<Point> = stations
                .iter()
                .map(|s| reference.at(*s, lane_offset(road, *s)))
                .collect();
            // The centre lane is a line, never a surface: it is where the reference
            // line runs, and what is painted on it.
            if let Some(mark) = mark_of(Some(&section.center.lane.first().base)) {
                drawing.path(Kind::Marking(mark), centre.clone());
            }

            for side in [Side::Left, Side::Right] {
                // Offsets are accumulated as numbers and turned into points once.
                // That is OpenDRIVE's own rule — a `<width>` is measured from the
                // border inside, not from the centre — and keeping it in one place is
                // what stops the second lane being laid out from the wrong edge.
                let mut inner_offset = vec![0.0; stations.len()];
                let mut inner = centre.clone();

                for lane in side.lanes(section) {
                    lanes += 1;
                    let outer_offset: Vec<f64> = stations
                        .iter()
                        .zip(inner_offset.iter())
                        .map(|(s, offset)| offset + lane.border(*s, section.s))
                        .collect();
                    let outer: Vec<Point> = stations
                        .iter()
                        .zip(outer_offset.iter())
                        .map(|(s, offset)| {
                            reference.at(*s, lane_offset(road, *s) + side.sign() * offset)
                        })
                        .collect();

                    let mut ring = inner.clone();
                    ring.extend(outer.iter().rev());
                    drawing.area(Kind::Surface, ring);
                    if lane.is_drivable() {
                        drawing.path(Kind::Center, midline(&inner, &outer));
                    }

                    match mark_of(Some(lane.base)) {
                        // A lane whose outer edge is painted gets the paint; one whose
                        // edge is a kerb or nothing at all gets the boundary, because
                        // the drivable surface ends there either way.
                        Some(mark) => drawing.path(Kind::Marking(mark), outer.clone()),
                        None => drawing.path(Kind::Boundary, outer.clone()),
                    }

                    inner_offset = outer_offset;
                    inner = outer;
                }
            }
        }
    }

    drawing.note(format!(
        "{} roads, {lanes} lanes, {} junctions",
        document.road.len(),
        document.junction.len()
    ));
    drawing.note(
        "the shapes are evaluated, not read: OpenDRIVE holds a reference line and \
         width polynomials, and this walks them"
            .to_owned(),
    );
    Ok(drawing)
}

/// Which side of the reference line a lane is on. OpenDRIVE numbers lanes outwards
/// from the centre — positive to the left, negative to the right — and the two sides
/// accumulate independently.
#[derive(Clone, Copy)]
enum Side {
    Left,
    Right,
}

impl Side {
    fn sign(self) -> f64 {
        match self {
            Side::Left => 1.0,
            Side::Right => -1.0,
        }
    }

    /// The side's lanes, innermost first, which is the order they stack in.
    fn lanes(self, section: &LaneSection) -> Vec<SideLane<'_>> {
        let mut lanes: Vec<SideLane<'_>> = match self {
            Side::Left => section
                .left
                .iter()
                .flat_map(|left| left.lane.iter())
                .map(|lane| SideLane {
                    id: lane.id,
                    base: &lane.base,
                })
                .collect(),
            Side::Right => section
                .right
                .iter()
                .flat_map(|right| right.lane.iter())
                .map(|lane| SideLane {
                    id: lane.id,
                    base: &lane.base,
                })
                .collect(),
        };
        lanes.sort_by_key(|lane| lane.id.abs());
        lanes
    }
}

struct SideLane<'a> {
    id: i64,
    base: &'a Lane,
}

impl SideLane<'_> {
    /// Whether traffic runs down this lane, which is what decides if it is drawn with
    /// a centre line. A verge or a kerb is part of the cross-section and has no
    /// traffic down the middle of it.
    fn is_drivable(&self) -> bool {
        matches!(
            self.base.r#type,
            LaneType::Driving
                | LaneType::Entry
                | LaneType::Exit
                | LaneType::OnRamp
                | LaneType::OffRamp
                | LaneType::ConnectingRamp
                | LaneType::Biking
                | LaneType::Bus
                | LaneType::Taxi
                | LaneType::HOV
                | LaneType::Bidirectional
        )
    }

    /// How far this lane reaches beyond the one inside it at station `s`, metres.
    ///
    /// `section_start` is where the lane section begins, because a `<width>` measures
    /// its `sOffset` from there rather than from the start of the road.
    fn border(&self, s: f64, section_start: f64) -> f64 {
        let ds = s - section_start;
        let mut value = 0.0;
        for choice in &self.base.choice {
            match choice {
                LaneChoice::Width(width) => {
                    if width.s_offset.get::<meter>() <= ds + 1e-9 {
                        value = poly(
                            width.a,
                            width.b,
                            width.c,
                            width.d,
                            ds - width.s_offset.get::<meter>(),
                        );
                    }
                }
                LaneChoice::Border(border) => {
                    if border.s_offset.get::<meter>() <= ds + 1e-9 {
                        value = poly(
                            border.a,
                            border.b,
                            border.c,
                            border.d,
                            ds - border.s_offset.get::<meter>(),
                        );
                    }
                }
            }
        }
        value.max(0.0)
    }
}

/// The lane offset polynomial at station `s`: how far the cross-section's centre is
/// displaced from the reference line there.
fn lane_offset(road: &Road, s: f64) -> f64 {
    let mut value = 0.0;
    for offset in &road.lanes.lane_offset {
        if offset.s <= s + 1e-9 {
            value = poly(offset.a, offset.b, offset.c, offset.d, s - offset.s);
        }
    }
    value
}

/// A cubic in `ds`, which is every polynomial OpenDRIVE states: widths, offsets,
/// elevation, superelevation. Written out rather than looped over its coefficients
/// because four terms read better than a fold.
fn poly(a: f64, b: f64, c: f64, d: f64, ds: f64) -> f64 {
    a + b * ds + c * ds * ds + d * ds * ds * ds
}

/// The reference line of one road, as the chain of pieces it is written as.
struct ReferenceLine {
    pieces: Vec<Piece>,
    length: f64,
}

struct Piece {
    start: f64,
    length: f64,
    x: f64,
    y: f64,
    heading: f64,
    kind: PieceKind,
}

enum PieceKind {
    Line,
    Arc {
        curvature: f64,
    },
    Spiral {
        start: f64,
        end: f64,
    },
    /// Anything else the standard allows and roadgen does not write. Drawn as the
    /// chord it spans, which is wrong in the middle and right at both ends — better
    /// than a gap, and [`Drawing::notes`] would be the place to say so if roadgen
    /// ever wrote one.
    Chord,
}

impl ReferenceLine {
    fn of(road: &Road) -> ReferenceLine {
        let mut pieces = Vec::new();
        for geometry in road.plan_view.geometry.iter() {
            pieces.push(Piece {
                start: geometry.s.get::<meter>(),
                length: geometry.length.get::<meter>(),
                x: geometry.x.get::<meter>(),
                y: geometry.y.get::<meter>(),
                heading: geometry.hdg.get::<radian>(),
                kind: match &geometry.r#type {
                    GeometryType::Line(_) => PieceKind::Line,
                    GeometryType::Arc(arc) => PieceKind::Arc {
                        curvature: arc.curvature.get::<radian_per_meter>(),
                    },
                    GeometryType::Spiral(spiral) => PieceKind::Spiral {
                        start: spiral.curvature_start.get::<radian_per_meter>(),
                        end: spiral.curvature_end.get::<radian_per_meter>(),
                    },
                    _ => PieceKind::Chord,
                },
            });
        }
        let length = road.length.get::<meter>().max(
            pieces
                .last()
                .map(|piece| piece.start + piece.length)
                .unwrap_or(0.0),
        );
        ReferenceLine { pieces, length }
    }

    fn is_empty(&self) -> bool {
        self.pieces.is_empty()
    }

    /// The point `offset` metres to the left of the reference line at station `s`.
    fn at(&self, s: f64, offset: f64) -> Point {
        let Some(piece) = self.piece_at(s) else {
            return Point::new(f64::NAN, f64::NAN);
        };
        let (x, y, heading) = piece.at(s - piece.start);
        // The normal is a quarter turn left of the heading, which is where a positive
        // t goes in OpenDRIVE.
        Point::new(
            x + offset * (heading + PI / 2.0).cos(),
            y + offset * (heading + PI / 2.0).sin(),
        )
    }

    fn piece_at(&self, s: f64) -> Option<&Piece> {
        self.pieces
            .iter()
            .rev()
            .find(|piece| piece.start <= s + 1e-9)
            .or_else(|| self.pieces.first())
    }
}

impl Piece {
    /// Where the piece is `ds` metres along it, and which way it points there.
    fn at(&self, ds: f64) -> (f64, f64, f64) {
        let ds = ds.clamp(0.0, self.length);
        match self.kind {
            PieceKind::Line | PieceKind::Chord => (
                self.x + ds * self.heading.cos(),
                self.y + ds * self.heading.sin(),
                self.heading,
            ),
            PieceKind::Arc { curvature } => {
                if curvature.abs() < 1e-12 {
                    return (
                        self.x + ds * self.heading.cos(),
                        self.y + ds * self.heading.sin(),
                        self.heading,
                    );
                }
                let radius = 1.0 / curvature;
                let turned = ds * curvature;
                // The centre of the circle is a radius to the left of the start.
                let centre_x = self.x - radius * self.heading.sin();
                let centre_y = self.y + radius * self.heading.cos();
                (
                    centre_x + radius * (self.heading + turned).sin(),
                    centre_y - radius * (self.heading + turned).cos(),
                    self.heading + turned,
                )
            }
            // A spiral has no closed form in elementary functions — its position is a
            // Fresnel integral — so it is integrated. The step is small next to
            // anything a road bends by, and the alternative is a Fresnel
            // implementation in a viewer.
            PieceKind::Spiral { start, end } => {
                let rate = if self.length > 0.0 {
                    (end - start) / self.length
                } else {
                    0.0
                };
                let steps = ((ds / 0.25).ceil() as usize).max(1);
                let step = ds / steps as f64;
                let (mut x, mut y) = (self.x, self.y);
                for index in 0..steps {
                    let at = index as f64 * step + step / 2.0;
                    let heading = self.heading + start * at + rate * at * at / 2.0;
                    x += step * heading.cos();
                    y += step * heading.sin();
                }
                (x, y, self.heading + start * ds + rate * ds * ds / 2.0)
            }
        }
    }
}

/// The stations a section is sampled at: every [`STEP`] metres, and both ends.
fn stations(start: f64, end: f64) -> Vec<f64> {
    if end <= start {
        return Vec::new();
    }
    let count = (((end - start) / STEP).ceil() as usize).max(1);
    let mut stations: Vec<f64> = (0..count)
        .map(|index| start + index as f64 * (end - start) / count as f64)
        .collect();
    stations.push(end);
    stations
}

/// The line between two borders, vertex by vertex. They are sampled at the same
/// stations, so pairing them is pairing stations.
fn midline(inner: &[Point], outer: &[Point]) -> Vec<Point> {
    inner
        .iter()
        .zip(outer.iter())
        .map(|(inner, outer)| Point::new((inner.x + outer.x) / 2.0, (inner.y + outer.y) / 2.0))
        .collect()
}

/// What a `<roadMark>` paints, or `None` where it paints nothing.
///
/// A kerb is nothing here on purpose: it is a physical edge, and it leaves as the road
/// boundary rather than as a painted line — the same split the exporters make.
fn mark_of(lane: Option<&Lane>) -> Option<Mark> {
    let marks: &Vec<RoadMark> = &lane?.road_mark;
    let mark = marks.first()?;
    let broken = match mark.type_simplified {
        TypeSimplified::None | TypeSimplified::Curb | TypeSimplified::Grass => return None,
        TypeSimplified::Broken | TypeSimplified::BrokenBroken | TypeSimplified::BottsDots => true,
        // A mixed pair is drawn as one line, and the one it is drawn as is the solid
        // half: a driver who may not cross is the stricter, safer reading of a
        // picture that can only show one.
        _ => false,
    };
    let color = match mark.color {
        Color::White | Color::Standard => MarkColor::White,
        Color::Yellow => MarkColor::Yellow,
        _ => MarkColor::Other,
    };
    Some(Mark::new(color, broken))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// One straight road, two lanes, painted on the centre.
    const STRAIGHT: &str = r#"<?xml version="1.0"?>
    <OpenDRIVE>
      <header revMajor="1" revMinor="7" name="t" version="1.00" date="now"
              north="0" south="0" east="0" west="0"/>
      <road id="1" junction="-1" length="100" name="straight">
        <planView>
          <geometry s="0" x="0" y="0" hdg="0" length="100"><line/></geometry>
        </planView>
        <lanes>
          <laneSection s="0">
            <left>
              <lane id="1" type="driving" level="false">
                <width sOffset="0" a="3.5" b="0" c="0" d="0"/>
                <roadMark sOffset="0" type="solid" color="white"/>
              </lane>
            </left>
            <center>
              <lane id="0" type="none" level="false">
                <roadMark sOffset="0" type="broken" color="yellow"/>
              </lane>
            </center>
            <right>
              <lane id="-1" type="driving" level="false">
                <width sOffset="0" a="3.5" b="0" c="0" d="0"/>
                <roadMark sOffset="0" type="solid" color="white"/>
              </lane>
            </right>
          </laneSection>
        </lanes>
      </road>
    </OpenDRIVE>"#;

    fn drawing() -> Drawing {
        draw(STRAIGHT).unwrap()
    }

    #[test]
    fn a_lane_is_as_wide_as_its_width_polynomial() {
        let drawing = drawing();
        let centres: Vec<&crate::drawing::Shape> = drawing
            .shapes
            .iter()
            .filter(|shape| shape.kind() == Kind::Center)
            .collect();
        assert_eq!(centres.len(), 2);
        // A 3.5 m lane on each side puts its middle 1.75 m off the reference line.
        let offsets: Vec<f64> = centres.iter().map(|shape| shape.extent()[0].y).collect();
        assert!(
            offsets.iter().any(|y| (y - 1.75).abs() < 1e-6),
            "{offsets:?}"
        );
        assert!(
            offsets.iter().any(|y| (y + 1.75).abs() < 1e-6),
            "{offsets:?}"
        );
    }

    #[test]
    fn the_centre_lanes_paint_is_drawn_as_written() {
        let drawing = drawing();
        assert!(drawing
            .shapes
            .iter()
            .any(|shape| shape.kind() == Kind::Marking(Mark::new(MarkColor::Yellow, true))));
    }

    #[test]
    fn a_road_reaches_the_length_it_declares() {
        let drawing = drawing();
        let bounds = drawing.bounds().unwrap();
        assert!((bounds.width() - 100.0).abs() < 1e-6, "{bounds:?}");
    }

    #[test]
    fn a_kerb_is_a_boundary_rather_than_a_painted_line() {
        let curbed = STRAIGHT.replace(
            r#"type="solid" color="white""#,
            r#"type="curb" color="standard""#,
        );
        let drawing = draw(&curbed).unwrap();
        assert!(drawing
            .shapes
            .iter()
            .any(|shape| shape.kind() == Kind::Boundary));
    }

    #[test]
    fn an_arc_bends_the_way_its_curvature_says() {
        let piece = Piece {
            start: 0.0,
            length: PI * 50.0,
            x: 0.0,
            y: 0.0,
            heading: 0.0,
            kind: PieceKind::Arc { curvature: 0.02 },
        };
        // A half turn of a 50 m radius circle, left, ends 100 m to the left.
        let (x, y, heading) = piece.at(PI * 50.0);
        assert!(x.abs() < 1e-6, "{x}");
        assert!((y - 100.0).abs() < 1e-6, "{y}");
        assert!((heading - PI).abs() < 1e-6, "{heading}");
    }

    #[test]
    fn a_spiral_of_no_curvature_is_a_straight_line() {
        let piece = Piece {
            start: 0.0,
            length: 10.0,
            x: 0.0,
            y: 0.0,
            heading: 0.0,
            kind: PieceKind::Spiral {
                start: 0.0,
                end: 0.0,
            },
        };
        let (x, y, _) = piece.at(10.0);
        assert!((x - 10.0).abs() < 1e-9);
        assert!(y.abs() < 1e-9);
    }

    #[test]
    fn a_spiral_ending_at_a_curvature_agrees_with_the_arc_it_becomes() {
        // Integrated over a short piece, a spiral from k to k is the arc of k.
        let spiral = Piece {
            start: 0.0,
            length: 20.0,
            x: 0.0,
            y: 0.0,
            heading: 0.0,
            kind: PieceKind::Spiral {
                start: 0.05,
                end: 0.05,
            },
        };
        let arc = Piece {
            start: 0.0,
            length: 20.0,
            x: 0.0,
            y: 0.0,
            heading: 0.0,
            kind: PieceKind::Arc { curvature: 0.05 },
        };
        let (sx, sy, _) = spiral.at(20.0);
        let (ax, ay, _) = arc.at(20.0);
        assert!((sx - ax).abs() < 1e-3, "{sx} vs {ax}");
        assert!((sy - ay).abs() < 1e-3, "{sy} vs {ay}");
    }

    #[test]
    fn xml_that_is_not_opendrive_is_an_error() {
        assert!(matches!(draw("<nope/>"), Err(ViewError::Parse(_))));
    }
}
