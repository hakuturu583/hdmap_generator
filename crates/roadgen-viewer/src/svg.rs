//! Turning a [`Drawing`] into an SVG document.
//!
//! This is the only module that knows a colour, and the only one that knows which way
//! up a picture goes. A reader says "this is a lane boundary"; what a lane boundary
//! looks like is decided once, here, so the six formats come out looking like six
//! views of one map rather than six unrelated pictures.
//!
//! # Pixels, not metres
//!
//! The map is fitted into a fixed pixel box rather than written in metres with a
//! `viewBox` left to scale it. That costs the free zoom a `viewBox` would give and
//! buys everything else: a stroke is a stroke at any map size, the legend text is
//! 12 px whether the map is a junction or a city, and the document is the same
//! picture in a notebook, in an `<img>` and on a page that has never heard of it.
//!
//! # Colours
//!
//! Every colour is `var(--rg-…, fallback)`. The fallbacks are mid-tones chosen to
//! stay legible on a white page and on a dark one, and a caller who wants its own
//! palette sets the custom properties on any ancestor of the `<svg>` — the document
//! does not have to be rendered again to be re-themed.

use std::fmt::Write as _;

use crate::drawing::{Bounds, Drawing, Kind, Mark, MarkColor, Point, Shape};

/// The largest the map itself is drawn, in pixels. The document is whatever the map
/// needs within this, so a long thin network is a long thin picture rather than one
/// stranded in a square.
///
/// It is deliberately modest. A picture wider than the column it is shown in gets
/// scaled down by whatever is showing it, and the legend goes with it — so the size
/// that matters is not how much detail fits but how far the text can be shrunk before
/// it stops being text.
const MAP_MAX: (f64, f64) = (760.0, 500.0);
/// The smallest, so that a map of one short road is still something to look at.
const MAP_MIN: (f64, f64) = (320.0, 180.0);
const MARGIN: f64 = 16.0;

const LEGEND_ROW: f64 = 21.0;
const LEGEND_SWATCH: f64 = 26.0;
const LEGEND_GAP: f64 = 8.0;
const LEGEND_FONT: f64 = 13.0;
/// Room for the label beside a swatch. Text is not measured — there is no font here
/// to measure with — so a legend column is as wide as its longest label could be.
const LEGEND_LABEL: f64 = 132.0;

/// Draws `drawing` as a standalone SVG document.
pub fn render(drawing: &Drawing) -> String {
    let kinds = drawing.kinds();
    let bounds = drawing.bounds();
    let view = View::fit(bounds);

    let columns = ((MARGIN * 2.0 + view.width) / (LEGEND_SWATCH + LEGEND_GAP + LEGEND_LABEL))
        .floor()
        .max(1.0) as usize;
    let rows = kinds.len().div_ceil(columns.max(1));
    let legend_height = if kinds.is_empty() {
        0.0
    } else {
        rows as f64 * LEGEND_ROW + MARGIN
    };

    let width = view.width + MARGIN * 2.0;
    let height = view.height + MARGIN * 2.0 + legend_height;

    let mut svg = String::new();
    let _ = write!(
        svg,
        "<svg xmlns=\"http://www.w3.org/2000/svg\" viewBox=\"0 0 {} {}\" \
         width=\"{}\" height=\"{}\" class=\"roadgen-view\" role=\"img\">",
        number(width),
        number(height),
        number(width),
        number(height),
    );
    let _ = write!(svg, "<title>{}</title>", escape(&drawing.title));
    if !drawing.notes.is_empty() {
        let _ = write!(svg, "<desc>{}</desc>", escape(&drawing.notes.join(" · ")));
    }
    svg.push_str(STYLE);

    if bounds.is_none() {
        // A map with nothing in it is a real answer — a scenario with no agents, a
        // crosswalk layer of no crosswalks — and saying so beats an empty frame.
        let _ = write!(
            svg,
            "<text class=\"rg-empty\" x=\"{}\" y=\"{}\" text-anchor=\"middle\">\
             nothing to draw</text>",
            number(width / 2.0),
            number(height / 2.0),
        );
        svg.push_str("</svg>");
        return svg;
    }

    svg.push_str("<g>");
    let mut shapes: Vec<&Shape> = drawing.shapes.iter().collect();
    shapes.sort_by_key(|shape| shape.kind());
    for shape in shapes {
        emit(&mut svg, &view, shape);
    }
    svg.push_str("</g>");

    if !kinds.is_empty() {
        legend(&mut svg, &kinds, columns, view.height + MARGIN * 2.0, width);
    }

    svg.push_str("</svg>");
    svg
}

/// The map-to-pixel transform: a uniform scale, a flip, and the margin.
struct View {
    scale: f64,
    bounds: Bounds,
    width: f64,
    height: f64,
}

impl View {
    fn fit(bounds: Option<Bounds>) -> View {
        let bounds = bounds.unwrap_or(Bounds {
            min: Point::new(0.0, 0.0),
            max: Point::new(0.0, 0.0),
        });
        // A map one road wide has no extent across it, and a scale computed from
        // zero is not a number. Both axes get a floor of a metre, which puts a
        // straight road across the middle of the picture rather than off it.
        let extent_x = bounds.width().max(1.0);
        let extent_y = bounds.height().max(1.0);
        let scale = (MAP_MAX.0 / extent_x).min(MAP_MAX.1 / extent_y);
        View {
            scale,
            bounds,
            width: (extent_x * scale).max(MAP_MIN.0),
            height: (extent_y * scale).max(MAP_MIN.1),
        }
    }

    /// Maps a point into the picture. North is up: the y axis is flipped here, which
    /// is why nothing downstream has to think about it.
    fn at(&self, point: Point) -> (f64, f64) {
        let centred_x = (point.x - self.bounds.min.x) * self.scale
            + (self.width - self.bounds.width().max(1.0) * self.scale) / 2.0;
        let centred_y = (self.bounds.max.y - point.y) * self.scale
            + (self.height - self.bounds.height().max(1.0) * self.scale) / 2.0;
        (MARGIN + centred_x, MARGIN + centred_y)
    }
}

fn emit(svg: &mut String, view: &View, shape: &Shape) {
    let class = class(shape.kind());
    match shape {
        Shape::Path { points, .. } => {
            let _ = write!(
                svg,
                "<polyline class=\"{class}\" fill=\"none\" points=\"{}\"/>",
                polygon_points(view, points)
            );
        }
        Shape::Area { points, .. } => {
            let _ = write!(
                svg,
                "<polygon class=\"{class}\" points=\"{}\"/>",
                polygon_points(view, points)
            );
        }
        Shape::Band { points, width, .. } => {
            // The width is metres, so it scales with the map: a 3.5 m lane is 3.5 m
            // wide in the picture rather than a fixed number of pixels.
            let _ = write!(
                svg,
                "<polyline class=\"{class}\" fill=\"none\" stroke-width=\"{}\" points=\"{}\"/>",
                number(width * view.scale),
                polygon_points(view, points)
            );
        }
        Shape::Dot { at, .. } => {
            let (x, y) = view.at(*at);
            let _ = write!(
                svg,
                "<circle class=\"{class}\" cx=\"{}\" cy=\"{}\" r=\"2.6\"/>",
                number(x),
                number(y)
            );
        }
        Shape::Box { .. } => {
            let corners = shape.corners().unwrap_or_default();
            let _ = write!(
                svg,
                "<polygon class=\"{class}\" points=\"{}\"/>",
                polygon_points(view, &corners)
            );
        }
    }
}

fn polygon_points(view: &View, points: &[Point]) -> String {
    let mut out = String::with_capacity(points.len() * 14);
    for point in points {
        if !point.x.is_finite() || !point.y.is_finite() {
            continue;
        }
        let (x, y) = view.at(*point);
        if !out.is_empty() {
            out.push(' ');
        }
        let _ = write!(out, "{},{}", number(x), number(y));
    }
    out
}

fn legend(svg: &mut String, kinds: &[Kind], columns: usize, top: f64, width: f64) {
    let column_width = (width - MARGIN * 2.0) / columns as f64;
    let _ = write!(svg, "<g class=\"rg-legend\">");
    for (index, kind) in kinds.iter().enumerate() {
        let column = index % columns;
        let row = index / columns;
        let x = MARGIN + column as f64 * column_width;
        let y = top + row as f64 * LEGEND_ROW + LEGEND_ROW / 2.0;
        let class = class(*kind);
        match kind {
            // A swatch for something drawn as an area or a point is that shape, not a
            // line of it: a legend that lied about the shape would be worse than none.
            Kind::Junction
            | Kind::Surface
            | Kind::Crosswalk
            | Kind::Agent
            | Kind::TrafficLight
            | Kind::TrafficSign => {
                let _ = write!(
                    svg,
                    "<rect class=\"{class}\" x=\"{}\" y=\"{}\" width=\"{}\" height=\"10\"/>",
                    number(x),
                    number(y - 5.0),
                    number(LEGEND_SWATCH)
                );
            }
            Kind::Node => {
                let _ = write!(
                    svg,
                    "<circle class=\"{class}\" cx=\"{}\" cy=\"{}\" r=\"2.6\"/>",
                    number(x + LEGEND_SWATCH / 2.0),
                    number(y)
                );
            }
            _ => {
                let _ = write!(
                    svg,
                    "<line class=\"{class}\" fill=\"none\" x1=\"{}\" y1=\"{}\" x2=\"{}\" y2=\"{}\"/>",
                    number(x),
                    number(y),
                    number(x + LEGEND_SWATCH),
                    number(y)
                );
            }
        }
        let _ = write!(
            svg,
            "<text class=\"rg-label\" x=\"{}\" y=\"{}\">{}</text>",
            number(x + LEGEND_SWATCH + LEGEND_GAP),
            number(y + LEGEND_FONT / 3.0),
            escape(label(*kind))
        );
    }
    svg.push_str("</g>");
}

/// The CSS class a kind is drawn with. One class per kind, named after the kind, so
/// a page that wants to hide the markings or highlight the tracks can.
fn class(kind: Kind) -> String {
    match kind {
        Kind::Junction => "rg-junction".into(),
        Kind::Surface => "rg-surface".into(),
        Kind::Boundary => "rg-boundary".into(),
        Kind::Marking(mark) => {
            let color = match mark.color {
                MarkColor::White => "rg-mark-white",
                MarkColor::Yellow => "rg-mark-yellow",
                MarkColor::Other => "rg-mark-other",
            };
            if mark.broken {
                format!("rg-mark {color} rg-broken")
            } else {
                format!("rg-mark {color}")
            }
        }
        Kind::Center => "rg-center".into(),
        Kind::Track => "rg-track".into(),
        Kind::Movement => "rg-movement".into(),
        Kind::Crosswalk => "rg-crosswalk".into(),
        Kind::StopLine => "rg-stop-line".into(),
        Kind::Node => "rg-node".into(),
        Kind::TrafficLight => "rg-traffic-light".into(),
        Kind::TrafficSign => "rg-traffic-sign".into(),
        Kind::Agent => "rg-agent".into(),
    }
}

fn label(kind: Kind) -> &'static str {
    match kind {
        Kind::Junction => "junction",
        Kind::Surface => "lane surface",
        Kind::Boundary => "road boundary",
        Kind::Marking(Mark {
            color: MarkColor::White,
            broken: false,
        }) => "solid white",
        Kind::Marking(Mark {
            color: MarkColor::White,
            broken: true,
        }) => "broken white",
        Kind::Marking(Mark {
            color: MarkColor::Yellow,
            broken: false,
        }) => "solid yellow",
        Kind::Marking(Mark {
            color: MarkColor::Yellow,
            broken: true,
        }) => "broken yellow",
        Kind::Marking(Mark {
            color: MarkColor::Other,
            ..
        }) => "lane line",
        Kind::Center => "lane centre",
        Kind::Track => "driven route",
        Kind::Movement => "movement",
        Kind::Crosswalk => "crosswalk",
        Kind::StopLine => "stop line",
        Kind::Node => "node",
        Kind::TrafficLight => "traffic light",
        Kind::TrafficSign => "traffic sign",
        Kind::Agent => "agent",
    }
}

const STYLE: &str = "<style>\
.roadgen-view{font-family:ui-sans-serif,system-ui,-apple-system,'Segoe UI',sans-serif}\
.roadgen-view :where(polyline,polygon,line,rect)\
{stroke-linejoin:round;stroke-linecap:round}\
.rg-junction{fill:var(--rg-junction-fill,rgba(37,99,235,.10));\
stroke:var(--rg-junction,#60a5fa);stroke-width:1}\
.rg-surface{fill:var(--rg-surface,rgba(100,116,139,.16));\
stroke:var(--rg-surface,rgba(100,116,139,.16));stroke-linecap:butt}\
.rg-boundary{stroke:var(--rg-boundary,#64748b);stroke-width:1.6}\
.rg-mark{stroke-width:1.1}\
.rg-mark-white{stroke:var(--rg-mark-white,#9ca3af)}\
.rg-mark-yellow{stroke:var(--rg-mark-yellow,#d4a017)}\
.rg-mark-other{stroke:var(--rg-mark-other,#94a3b8)}\
.rg-broken{stroke-dasharray:6 5}\
.rg-center{stroke:var(--rg-center,#2563eb);stroke-width:1}\
.rg-track{stroke:var(--rg-track,#f97316);stroke-width:2}\
.rg-movement{stroke:var(--rg-movement,#a855f7);stroke-width:.9;stroke-dasharray:3 4}\
.rg-crosswalk{fill:var(--rg-crosswalk-fill,rgba(16,185,129,.18));\
stroke:var(--rg-crosswalk,#10b981);stroke-width:1}\
.rg-stop-line{stroke:var(--rg-stop-line,#ef4444);stroke-width:2.4}\
.rg-node{fill:var(--rg-node,#64748b);stroke:none}\
.rg-traffic-light{fill:var(--rg-traffic-light,#ef4444);stroke:none}\
.rg-traffic-sign{fill:var(--rg-traffic-sign,#eab308);stroke:none}\
.rg-agent{fill:var(--rg-agent-fill,rgba(14,165,233,.35));\
stroke:var(--rg-agent,#0ea5e9);stroke-width:1}\
.rg-label{fill:var(--rg-text,#64748b);font-size:13px}\
.rg-empty{fill:var(--rg-text,#64748b);font-size:13px}\
</style>";

/// A number with no more precision than a picture can show. Trailing zeros go, so a
/// document of ten thousand vertices is not a third padding.
fn number(value: f64) -> String {
    if !value.is_finite() {
        return "0".into();
    }
    let rounded = (value * 100.0).round() / 100.0;
    let mut text = format!("{rounded:.2}");
    while text.contains('.') && (text.ends_with('0') || text.ends_with('.')) {
        text.pop();
    }
    // Rounding a small negative to zero leaves "-0", which is a number no reader
    // needs to see.
    if text.is_empty() || text == "-" || text == "-0" {
        "0".into()
    } else {
        text
    }
}

fn escape(text: &str) -> String {
    let mut out = String::with_capacity(text.len());
    for character in text.chars() {
        match character {
            '&' => out.push_str("&amp;"),
            '<' => out.push_str("&lt;"),
            '>' => out.push_str("&gt;"),
            '"' => out.push_str("&quot;"),
            _ => out.push(character),
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::drawing::Shape;

    fn square() -> Drawing {
        let mut drawing = Drawing::new("test");
        drawing.path(
            Kind::Center,
            vec![
                Point::new(0.0, 0.0),
                Point::new(100.0, 0.0),
                Point::new(100.0, 100.0),
            ],
        );
        drawing
    }

    #[test]
    fn north_is_up() {
        // The first vertex is the southern one, so it must land lower down the
        // picture — a larger y in SVG — than the northern one.
        let svg = square().to_svg();
        let points = svg
            .split("points=\"")
            .nth(1)
            .and_then(|rest| rest.split('"').next())
            .unwrap()
            .to_owned();
        let ys: Vec<f64> = points
            .split(' ')
            .map(|pair| pair.split(',').nth(1).unwrap().parse().unwrap())
            .collect();
        assert!(ys[0] > ys[2], "{points}");
    }

    #[test]
    fn a_square_map_is_drawn_square() {
        let svg = square().to_svg();
        let points: Vec<(f64, f64)> = svg
            .split("points=\"")
            .nth(1)
            .and_then(|rest| rest.split('"').next())
            .unwrap()
            .split(' ')
            .map(|pair| {
                let mut parts = pair.split(',');
                (
                    parts.next().unwrap().parse().unwrap(),
                    parts.next().unwrap().parse().unwrap(),
                )
            })
            .collect();
        let across = points[1].0 - points[0].0;
        let up = points[1].1 - points[2].1;
        assert!((across - up).abs() < 1e-6, "{across} vs {up}");
    }

    #[test]
    fn a_drawing_with_nothing_in_it_says_so_rather_than_dividing_by_zero() {
        let svg = Drawing::new("empty").to_svg();
        assert!(svg.contains("nothing to draw"));
        assert!(!svg.contains("NaN"));
    }

    #[test]
    fn the_legend_names_every_kind_the_drawing_holds() {
        let mut drawing = square();
        drawing.push(Shape::Dot {
            kind: Kind::Node,
            at: Point::new(0.0, 0.0),
        });
        let svg = drawing.to_svg();
        assert!(svg.contains(">lane centre<"));
        assert!(svg.contains(">node<"));
        assert!(!svg.contains(">crosswalk<"));
    }

    #[test]
    fn the_title_and_the_notes_are_escaped_into_the_document() {
        let mut drawing = square();
        drawing.title = "a & b".into();
        drawing.note("<not markup>");
        let svg = drawing.to_svg();
        assert!(svg.contains("<title>a &amp; b</title>"));
        assert!(svg.contains("<desc>&lt;not markup&gt;</desc>"));
    }

    #[test]
    fn numbers_are_written_short() {
        assert_eq!(number(3.20159), "3.2");
        assert_eq!(number(10.0), "10");
        assert_eq!(number(-0.001), "0");
        assert_eq!(number(f64::NAN), "0");
    }
}
