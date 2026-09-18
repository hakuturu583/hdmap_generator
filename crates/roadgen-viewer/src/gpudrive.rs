//! Reading a GPUDrive scene back.
//!
//! Nothing here describes the document: `roadgen-gpudrive`'s model is `Deserialize`
//! so that a written scene can be read back the way a consumer reads it, and this is
//! that consumer. What is left is the mapping from the reader's vocabulary —
//! `road_line` and a Waymo feature code — onto what to draw.
//!
//! GPUDrive is a plane and its map is centrelines: there is no z in the document and
//! no lane width anywhere in it, so the picture has no surface under the lanes and no
//! relief in them. That is the format, not the viewer, and [`Drawing::notes`] says so.

use roadgen_gpudrive::scene::{MapElement, RoadKind, Scene, Vector2};

use crate::drawing::{Drawing, Kind, Mark, MarkColor, Point, Shape};
use crate::error::ViewError;

/// Draws a GPUDrive scene document.
pub fn draw(json: &str) -> Result<Drawing, ViewError> {
    let scene: Scene =
        serde_json::from_str(json).map_err(|error| ViewError::Parse(error.to_string()))?;

    let mut drawing = Drawing::new("GPUDrive");
    for road in &scene.roads {
        let points: Vec<Point> = road.geometry.iter().map(point).collect();
        match road.kind {
            RoadKind::Lane => drawing.path(Kind::Center, points),
            RoadKind::RoadLine => drawing.path(Kind::Marking(mark(road.map_element_id)), points),
            RoadKind::RoadEdge => drawing.path(Kind::Boundary, points),
            RoadKind::Crosswalk => drawing.area(Kind::Crosswalk, points),
            // A bump is a bar laid across the lane, which is the shape a stop line
            // has; the format gives no other transverse feature to tell it from.
            RoadKind::SpeedBump => drawing.path(Kind::StopLine, points),
            RoadKind::StopSign => {
                if let Some(at) = points.first() {
                    drawing.push(Shape::Dot {
                        kind: Kind::TrafficSign,
                        at: *at,
                    });
                }
            }
        }
    }

    let mut tracks = 0usize;
    for object in &scene.objects {
        // A track's steps are only meaningful where the agent exists: `valid` going
        // false is the agent leaving, and joining across the gap would draw a journey
        // it never made.
        let mut run: Vec<Point> = Vec::new();
        for (index, position) in object.position.iter().enumerate() {
            if object.valid.get(index).copied().unwrap_or(true) {
                run.push(point(position));
            } else {
                drawing.path(Kind::Track, std::mem::take(&mut run));
            }
        }
        if run.len() >= 2 {
            tracks += 1;
        }
        drawing.path(Kind::Track, run);

        let start = object
            .position
            .iter()
            .zip(object.valid.iter())
            .find(|(_, valid)| **valid)
            .map(|(position, _)| point(position));
        if let Some(at) = start {
            drawing.push(Shape::Box {
                kind: Kind::Agent,
                at,
                length: object.length,
                width: object.width,
                heading: object.heading.first().copied().unwrap_or(0.0),
            });
        }
    }

    drawing.note(format!(
        "{} road elements, {} agents",
        scene.roads.len(),
        scene.objects.len()
    ));
    if tracks > 0 {
        drawing.note(format!("{tracks} agents are drawn at their first timestep"));
    }
    drawing.note(
        "GPUDrive is a plane: the document has no z and no lane widths, so this is \
         centrelines without the surface under them"
            .to_owned(),
    );
    Ok(drawing)
}

fn point(vector: &Vector2) -> Point {
    Point::new(vector.x, vector.y)
}

/// The Waymo feature code, as the two things a picture can show: what colour the
/// paint is and whether the line is broken.
///
/// A double line is drawn as a single one. Two lines a few centimetres apart are one
/// line at any scale a whole map is drawn at, and the alternative is a picture that
/// claims a precision the geometry — one polyline, whatever the code says — does not
/// have.
fn mark(element: MapElement) -> Mark {
    match element {
        MapElement::RoadLineBrokenSingleWhite => Mark::new(MarkColor::White, true),
        MapElement::RoadLineSolidSingleWhite | MapElement::RoadLineSolidDoubleWhite => {
            Mark::new(MarkColor::White, false)
        }
        MapElement::RoadLineBrokenSingleYellow | MapElement::RoadLineBrokenDoubleYellow => {
            Mark::new(MarkColor::Yellow, true)
        }
        MapElement::RoadLineSolidSingleYellow
        | MapElement::RoadLineSolidDoubleYellow
        | MapElement::RoadLinePassingDoubleYellow => Mark::new(MarkColor::Yellow, false),
        // `ROAD_LINE_UNKNOWN` is what the exporter writes for a marking Waymo has no
        // code for, so it is painted — just not in a colour this can name.
        _ => Mark::new(MarkColor::Other, false),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const SCENE: &str = r#"{
        "name": "town",
        "scenario_id": "town-0001",
        "objects": [{
            "position": [{"x": 0.0, "y": 0.0}, {"x": 1.0, "y": 0.0}, {"x": 2.0, "y": 0.0}],
            "width": 2.0, "length": 4.6, "height": 1.6, "id": 1,
            "heading": [0.0, 0.0, 0.0],
            "velocity": [{"x": 1.0, "y": 0.0}, {"x": 1.0, "y": 0.0}, {"x": 1.0, "y": 0.0}],
            "valid": [true, false, true],
            "goalPosition": {"x": 2.0, "y": 0.0},
            "type": "vehicle",
            "mark_as_expert": false
        }],
        "roads": [
            {"type": "lane", "geometry": [{"x": 0.0, "y": 0.0}, {"x": 10.0, "y": 0.0}],
             "id": 1, "map_element_id": 2},
            {"type": "road_line", "geometry": [{"x": 0.0, "y": 1.75}, {"x": 10.0, "y": 1.75}],
             "id": 2, "map_element_id": 9},
            {"type": "road_edge", "geometry": [{"x": 0.0, "y": 3.5}, {"x": 10.0, "y": 3.5}],
             "id": 3, "map_element_id": 15},
            {"type": "stop_sign", "geometry": [{"x": 9.0, "y": 0.0}],
             "id": 4, "map_element_id": 17}
        ],
        "metadata": {"sdc_track_index": 0, "tracks_to_predict": [], "objects_of_interest": []}
    }"#;

    #[test]
    fn each_road_element_is_drawn_as_what_it_is() {
        let drawing = draw(SCENE).unwrap();
        let kinds: Vec<Kind> = drawing.shapes.iter().map(Shape::kind).collect();
        assert!(kinds.contains(&Kind::Center));
        assert!(kinds.contains(&Kind::Boundary));
        assert!(kinds.contains(&Kind::TrafficSign));
        assert!(kinds.contains(&Kind::Marking(Mark::new(MarkColor::Yellow, true))));
    }

    #[test]
    fn a_track_is_broken_where_the_agent_is_not_there() {
        // Three steps, the middle one invalid: two runs of one point, so no track is
        // drawn at all rather than one straight through the gap.
        let drawing = draw(SCENE).unwrap();
        assert!(!drawing
            .shapes
            .iter()
            .any(|shape| shape.kind() == Kind::Track));
    }

    #[test]
    fn an_agent_is_drawn_at_the_first_step_it_exists() {
        let drawing = draw(SCENE).unwrap();
        let agent = drawing
            .shapes
            .iter()
            .find(|shape| shape.kind() == Kind::Agent)
            .unwrap();
        let Shape::Box { at, length, .. } = agent else {
            panic!("an agent is a box");
        };
        assert_eq!(*at, Point::new(0.0, 0.0));
        assert_eq!(*length, 4.6);
    }

    #[test]
    fn a_double_yellow_is_drawn_as_a_solid_yellow_line() {
        assert_eq!(
            mark(MapElement::RoadLineSolidDoubleYellow),
            Mark::new(MarkColor::Yellow, false)
        );
        assert_eq!(
            mark(MapElement::RoadLineUnknown),
            Mark::new(MarkColor::Other, false)
        );
    }

    #[test]
    fn json_that_is_not_a_scene_is_an_error() {
        assert!(matches!(draw("{}"), Err(ViewError::Parse(_))));
    }
}
