//! A scene as a standalone SVG document.

use std::fmt::Write;

use crate::scene::{Bounds, Scene, Xy};

/// How a scene is put on the page.
#[derive(Debug, Clone, PartialEq)]
pub struct SvgOptions {
    /// Width of the picture, pixels; the height follows the map's aspect.
    pub width: f64,
    /// Empty space round the map, metres.
    pub margin: f64,
    /// Draw each road's reference line as a thin dashed line.
    pub reference_lines: bool,
    /// Draw an arrow down each drivable lane, the way traffic goes.
    pub arrows: bool,
}

impl Default for SvgOptions {
    fn default() -> Self {
        SvgOptions {
            width: 1024.0,
            margin: 10.0,
            reference_lines: true,
            arrows: true,
        }
    }
}

/// Fill of a lane, by its type.
pub fn lane_fill(lane_type: &str, connector: bool) -> &'static str {
    match (lane_type, connector) {
        ("driving", false) => "#4a4f57",
        ("driving", true) => "#555b64",
        ("biking", _) => "#6b4f4f",
        ("sidewalk", _) => "#b8b2a7",
        ("shoulder", _) => "#6c6f73",
        ("parking", _) => "#5a6270",
        ("border", _) | ("restricted", _) => "#7d7a74",
        _ => "#8c8a86",
    }
}

/// Colour of a painted line.
pub fn line_stroke(color: &str, marking: &str) -> &'static str {
    match (marking, color) {
        ("curb", _) => "#d8d4cc",
        (_, "yellow") => "#f2c230",
        _ => "#f4f4f2",
    }
}

impl Scene {
    /// The scene as an SVG document, north up.
    pub fn to_svg(&self, options: &SvgOptions) -> String {
        let bounds = self.bounds.unwrap_or(Bounds {
            min_x: 0.0,
            min_y: 0.0,
            max_x: 1.0,
            max_y: 1.0,
        });
        let margin = options.margin.max(0.0);
        let view = Bounds {
            min_x: bounds.min_x - margin,
            min_y: bounds.min_y - margin,
            max_x: bounds.max_x + margin,
            max_y: bounds.max_y + margin,
        };
        let (w, h) = (view.width().max(1e-6), view.height().max(1e-6));
        let height = options.width * h / w;
        // One pixel, in metres: lines are drawn this many metres wide so that they
        // stay visible however large the map is.
        let px = w / options.width.max(1.0);

        let mut out = String::new();
        let _ = write!(
            out,
            r##"<svg xmlns="http://www.w3.org/2000/svg" width="{:.0}" height="{:.0}" viewBox="{} {} {} {}">"##,
            options.width,
            height,
            num(view.min_x),
            num(-view.max_y),
            num(w),
            num(h),
        );
        out.push_str(r##"<rect x="-1e9" y="-1e9" width="2e9" height="2e9" fill="#2b3a2b"/>"##);
        // Map y is north; SVG y is down. One flip on a group keeps every
        // coordinate below the map's own.
        out.push_str(r#"<g transform="scale(1,-1)">"#);

        for area in &self.areas {
            let fill = if area.category == "building" {
                "#9c8f80"
            } else {
                "#5b5040"
            };
            let _ = write!(
                out,
                r##"<polygon data-id="{}" points="{}" fill="{}" stroke="#3a332b" stroke-width="{}"/>"##,
                escape(&area.id),
                points(&area.outline),
                fill,
                num(px),
            );
        }

        for junction in &self.junctions {
            if junction.outline.len() < 3 {
                continue;
            }
            let _ = write!(
                out,
                r##"<polygon data-id="{}" points="{}" fill="#4a4f57"/>"##,
                escape(&junction.id),
                points(&junction.outline),
            );
        }

        // Ordinary roads first, then the connectors that cross them.
        let mut lanes: Vec<_> = self.lanes.iter().collect();
        lanes.sort_by_key(|lane| lane.connector);
        for lane in lanes {
            let _ = write!(
                out,
                r#"<polygon data-id="{}" points="{}" fill="{}"/>"#,
                escape(&lane.id),
                points(&lane.outline),
                lane_fill(&lane.lane_type, lane.connector),
            );
        }

        for line in &self.lines {
            let width = match line.marking.as_str() {
                "curb" => 0.25f64,
                _ => 0.15,
            }
            .max(px);
            let dash = if line.marking.starts_with("broken") {
                format!(r#" stroke-dasharray="{} {}""#, num(3.0), num(6.0))
            } else {
                String::new()
            };
            let _ = write!(
                out,
                r#"<polyline points="{}" fill="none" stroke="{}" stroke-width="{}"{}/>"#,
                points(&line.points),
                line_stroke(&line.color, &line.marking),
                num(width),
                dash,
            );
        }

        if options.arrows {
            for lane in self.lanes.iter().filter(|lane| lane.drivable) {
                if let Some(arrow) = arrow(&lane.centerline, lane.width.clamp(1.0, 4.0)) {
                    let _ = write!(
                        out,
                        r##"<polygon points="{}" fill="#ffffff" fill-opacity="0.55"/>"##,
                        points(&arrow),
                    );
                }
            }
        }

        if options.reference_lines {
            for road in &self.roads {
                let _ = write!(
                    out,
                    r##"<polyline data-id="{}" points="{}" fill="none" stroke="#5fb4ff" stroke-width="{}" stroke-dasharray="{} {}"/>"##,
                    escape(&road.id),
                    points(&road.reference_line),
                    num(px),
                    num(4.0 * px),
                    num(3.0 * px),
                );
            }
        }

        for object in &self.objects {
            let color = match object.kind.as_str() {
                "stop_line" => "#ffffff",
                "crosswalk" => "#e8e8e8",
                "traffic_light" => "#ff5a4f",
                "traffic_sign" => "#ffb000",
                _ => "#c0c0c0",
            };
            if object.points.len() == 1 {
                let [x, y] = object.points[0];
                let _ = write!(
                    out,
                    r#"<circle data-id="{}" cx="{}" cy="{}" r="{}" fill="{}"/>"#,
                    escape(&object.id),
                    num(x),
                    num(y),
                    num((4.0 * px).max(0.6)),
                    color,
                );
            } else if object.closed {
                let _ = write!(
                    out,
                    r#"<polygon data-id="{}" points="{}" fill="{}" fill-opacity="0.6"/>"#,
                    escape(&object.id),
                    points(&object.points),
                    color,
                );
            } else {
                let _ = write!(
                    out,
                    r#"<polyline data-id="{}" points="{}" fill="none" stroke="{}" stroke-width="{}"/>"#,
                    escape(&object.id),
                    points(&object.points),
                    color,
                    num(0.4f64.max(px)),
                );
            }
        }

        out.push_str("</g></svg>");
        out
    }
}

/// An arrowhead halfway down a centreline, `size` metres long.
pub fn arrow(centerline: &[Xy], size: f64) -> Option<[Xy; 3]> {
    let total: f64 = centerline
        .windows(2)
        .map(|pair| distance(pair[0], pair[1]))
        .sum();
    if total < size * 2.0 {
        return None;
    }
    let mut remaining = total / 2.0;
    for pair in centerline.windows(2) {
        let length = distance(pair[0], pair[1]);
        if length <= 0.0 {
            continue;
        }
        if remaining <= length {
            let (dx, dy) = (
                (pair[1][0] - pair[0][0]) / length,
                (pair[1][1] - pair[0][1]) / length,
            );
            let t = remaining / length;
            let tip = [
                pair[0][0] + (pair[1][0] - pair[0][0]) * t + dx * size / 2.0,
                pair[0][1] + (pair[1][1] - pair[0][1]) * t + dy * size / 2.0,
            ];
            let base = [tip[0] - dx * size, tip[1] - dy * size];
            let half = size * 0.35;
            return Some([
                tip,
                [base[0] - dy * half, base[1] + dx * half],
                [base[0] + dy * half, base[1] - dx * half],
            ]);
        }
        remaining -= length;
    }
    None
}

fn distance(a: Xy, b: Xy) -> f64 {
    (b[0] - a[0]).hypot(b[1] - a[1])
}

fn points(points: &[Xy]) -> String {
    let mut out = String::with_capacity(points.len() * 16);
    for (i, [x, y]) in points.iter().enumerate() {
        if i > 0 {
            out.push(' ');
        }
        let _ = write!(out, "{},{}", num(*x), num(*y));
    }
    out
}

/// A coordinate to the centimetre, without trailing zeros.
fn num(value: f64) -> String {
    if !value.is_finite() {
        return "0".to_owned();
    }
    let text = format!("{value:.3}");
    let text = text.trim_end_matches('0').trim_end_matches('.');
    match text {
        "-0" | "" => "0".to_owned(),
        _ => text.to_owned(),
    }
}

fn escape(text: &str) -> String {
    text.replace('&', "&amp;")
        .replace('<', "&lt;")
        .replace('"', "&quot;")
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn numbers_are_short() {
        assert_eq!(num(1.5), "1.5");
        assert_eq!(num(2.0), "2");
        assert_eq!(num(-0.0001), "0");
        assert_eq!(num(f64::NAN), "0");
        assert_eq!(num(123.4567), "123.457");
    }

    #[test]
    fn an_arrow_points_down_the_line() {
        let [tip, left, right] = arrow(&[[0.0, 0.0], [10.0, 0.0]], 2.0).unwrap();
        assert!((tip[0] - 6.0).abs() < 1e-9);
        assert!(left[0] < tip[0] && right[0] < tip[0]);
        assert!(left[1] > 0.0 && right[1] < 0.0);
        assert!(arrow(&[[0.0, 0.0], [1.0, 0.0]], 2.0).is_none());
    }

    #[test]
    fn an_empty_scene_is_still_a_document() {
        let svg = Scene::default().to_svg(&SvgOptions::default());
        assert!(svg.starts_with("<svg") && svg.ends_with("</svg>"));
    }

    #[test]
    fn identifiers_are_escaped() {
        assert_eq!(escape(r#"a"<&"#), "a&quot;&lt;&amp;");
    }
}
