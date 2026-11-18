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

/// The land's colour at `share` of the way from its lowest to its highest: green
/// low ground through olive to pale brown high ground.
pub fn land_fill(share: f64) -> String {
    const STOPS: [(f64, [f64; 3]); 4] = [
        (0.0, [44.0, 70.0, 42.0]),
        (0.4, [86.0, 104.0, 56.0]),
        (0.75, [134.0, 118.0, 78.0]),
        (1.0, [190.0, 176.0, 150.0]),
    ];
    let share = share.clamp(0.0, 1.0);
    let i = STOPS
        .iter()
        .rposition(|(at, _)| *at <= share)
        .unwrap_or(0)
        .min(STOPS.len() - 2);
    let ((a, from), (b, to)) = (STOPS[i], STOPS[i + 1]);
    let t = (share - a) / (b - a);
    let c = |k: usize| (from[k] + (to[k] - from[k]) * t).round() as u8;
    format!("#{:02x}{:02x}{:02x}", c(0), c(1), c(2))
}

/// Fill of a surface feature, and how opaque: water blue, a bump amber, a hole
/// dark, roughness a light wash over the lane.
pub fn surface_fill(kind: &str) -> (&'static str, f64) {
    match kind {
        "puddle" => ("#3b82c4", 0.75),
        "pothole" => ("#16181b", 0.9),
        "bump" => ("#e0a030", 0.8),
        "rut" => ("#2c3036", 0.55),
        _ => ("#b07d5a", 0.35),
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

        // The land first, under everything: each triangle by its mean height, and
        // the contours over it.
        let (low, high) = self
            .terrain
            .iter()
            .fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), t| {
                (lo.min(t.heights[0]), hi.max(t.heights[1]))
            });
        for surface in &self.terrain {
            for triangle in &surface.triangles {
                let corners = triangle.map(|i| surface.nodes[i as usize]);
                let mean = corners.iter().map(|p| p[2]).sum::<f64>() / 3.0;
                let share = if high > low {
                    (mean - low) / (high - low)
                } else {
                    0.0
                };
                let _ = write!(
                    out,
                    r#"<polygon points="{}" fill="{}"/>"#,
                    points(&corners.map(|p| [p[0], p[1]])),
                    land_fill(share),
                );
            }
        }
        for contour in &self.contours {
            for [a, b] in &contour.segments {
                let _ = write!(
                    out,
                    r##"<line x1="{}" y1="{}" x2="{}" y2="{}" stroke="#1b2414" stroke-opacity="{}" stroke-width="{}"/>"##,
                    num(a[0]),
                    num(a[1]),
                    num(b[0]),
                    num(b[1]),
                    if contour.major { "0.55" } else { "0.3" },
                    num(px * if contour.major { 1.4 } else { 0.8 }),
                );
            }
        }

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

        // A bridge's deck is wider than the road it carries, so it goes under it.
        for structure in self.structures.iter().filter(|s| s.kind == "bridge") {
            for support in &structure.supports {
                let _ = write!(
                    out,
                    r##"<polygon points="{}" fill="#1f2328"/>"##,
                    points(support),
                );
            }
            let _ = write!(
                out,
                r##"<polygon data-id="{}" points="{}" fill="#8a96a3" stroke="#d0d7de" stroke-width="{}"><title>{}</title></polygon>"##,
                escape(&structure.id),
                points(&structure.outline),
                num(2.0 * px),
                escape(&structure.label),
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

        for surface in &self.surfaces {
            let (fill, opacity) = surface_fill(&surface.kind);
            let _ = write!(
                out,
                r#"<polygon data-id="{}" points="{}" fill="{}" fill-opacity="{}"><title>{}</title></polygon>"#,
                escape(&surface.id),
                points(&surface.outline),
                fill,
                opacity,
                escape(&surface.label),
            );
            for trough in &surface.troughs {
                let _ = write!(
                    out,
                    r##"<polyline points="{}" fill="none" stroke="#1d2024" stroke-width="{}" stroke-linecap="round"/>"##,
                    points(trough),
                    num(0.3f64.max(px)),
                );
            }
        }

        // A tunnel goes over the road it holds: a shade with its portals drawn hard.
        for structure in self.structures.iter().filter(|s| s.kind == "tunnel") {
            let _ = write!(
                out,
                r##"<polygon data-id="{}" points="{}" fill="#0b0d10" fill-opacity="0.45" stroke="#9aa3ad" stroke-width="{}" stroke-dasharray="{} {}"><title>{}</title></polygon>"##,
                escape(&structure.id),
                points(&structure.outline),
                num(1.5 * px),
                num(6.0 * px),
                num(4.0 * px),
                escape(&structure.label),
            );
            for portal in &structure.portals {
                let _ = write!(
                    out,
                    r##"<polyline points="{}" fill="none" stroke="#d0d7de" stroke-width="{}"/>"##,
                    points(portal),
                    num(0.8f64.max(3.0 * px)),
                );
            }
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
    fn the_land_runs_green_to_brown() {
        assert_eq!(land_fill(0.0), "#2c462a");
        assert_eq!(land_fill(1.0), "#beb096");
        assert_eq!(land_fill(-1.0), land_fill(0.0));
    }

    #[test]
    fn identifiers_are_escaped() {
        assert_eq!(escape(r#"a"<&"#), "a&quot;&lt;&amp;");
    }
}
