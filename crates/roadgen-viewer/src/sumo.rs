//! Reading a SUMO plain-XML network back.
//!
//! The three files are netconvert's input, not its output, so what is in them is
//! exactly what the generator decided and nothing netconvert would go on to compute.
//! That shows in the picture: every lane is drawn from its own `shape`, and a
//! `<connection>` — which is two lane names and no geometry at all — is drawn as the
//! straight line between the lanes it joins, because that is as much as the file
//! says. netconvert's junction interiors are not guessed at here.

use std::collections::HashMap;

use quick_xml::events::{BytesStart, Event};
use quick_xml::{Reader, XmlVersion};

use crate::drawing::{Drawing, Kind, Point, Shape};
use crate::error::ViewError;

/// The three files of a plain-XML network. The connection file is optional because a
/// network of unconnected edges is a legal one.
#[derive(Debug, Clone, Default)]
pub struct Network {
    pub nodes: String,
    pub edges: String,
    pub connections: Option<String>,
}

/// Draws a plain-XML network.
pub fn draw(network: &Network) -> Result<Drawing, ViewError> {
    let mut drawing = Drawing::new("SUMO");

    let mut nodes = 0usize;
    for node in elements(&network.nodes, &["node"])? {
        let (Some(x), Some(y)) = (number(&node, "x"), number(&node, "y")) else {
            continue;
        };
        nodes += 1;
        drawing.push(Shape::Dot {
            kind: Kind::Node,
            at: Point::new(x, y),
        });
    }

    // Keyed the way a connection names a lane: the edge id and the index within it.
    let mut lanes: HashMap<(String, String), Vec<Point>> = HashMap::new();
    let mut edges = 0usize;
    let mut current_edge = String::new();
    let mut edge_shape: Vec<Point> = Vec::new();
    let mut edge_lanes = 0usize;

    for element in elements(&network.edges, &["edge", "lane"])? {
        match element.name.as_str() {
            "edge" => {
                if edges > 0 && edge_lanes == 0 {
                    // An edge whose lanes are implied by `numLanes` rather than
                    // written out has no per-lane shape to draw, so its own is the
                    // best the file offers.
                    drawing.path(Kind::Center, edge_shape.clone());
                }
                edges += 1;
                current_edge = element.get("id").unwrap_or_default();
                edge_shape = shape(&element).unwrap_or_default();
                edge_lanes = 0;
            }
            "lane" => {
                edge_lanes += 1;
                let index = element.get("index").unwrap_or_else(|| "0".into());
                let points = shape(&element).unwrap_or_else(|| edge_shape.clone());
                if let Some(width) = number(&element, "width") {
                    drawing.band(Kind::Surface, points.clone(), width);
                }
                drawing.path(Kind::Center, points.clone());
                lanes.insert((current_edge.clone(), index), points);
            }
            _ => {}
        }
    }
    if edges > 0 && edge_lanes == 0 {
        drawing.path(Kind::Center, edge_shape);
    }

    let mut movements = 0usize;
    if let Some(connections) = &network.connections {
        for connection in elements(connections, &["connection"])? {
            let from = lanes.get(&(
                connection.get("from").unwrap_or_default(),
                connection.get("fromLane").unwrap_or_else(|| "0".into()),
            ));
            let to = lanes.get(&(
                connection.get("to").unwrap_or_default(),
                connection.get("toLane").unwrap_or_else(|| "0".into()),
            ));
            if let (Some(from), Some(to)) = (from, to) {
                if let (Some(leave), Some(enter)) = (from.last(), to.first()) {
                    movements += 1;
                    drawing.path(Kind::Movement, vec![*leave, *enter]);
                }
            }
        }
    }

    drawing.note(format!("{edges} edges, {} lanes", lanes.len()));
    drawing.note(format!("{nodes} nodes, {movements} movements"));
    drawing.note(
        "a movement is drawn straight: a plain-XML connection is two lane names, and \
         the path through the junction is netconvert's to compute"
            .to_owned(),
    );
    Ok(drawing)
}

/// One element's attributes, with the name it came from.
struct Element {
    name: String,
    attributes: HashMap<String, String>,
}

impl Element {
    fn get(&self, key: &str) -> Option<String> {
        self.attributes.get(key).cloned()
    }
}

fn number(element: &Element, key: &str) -> Option<f64> {
    element.attributes.get(key)?.trim().parse().ok()
}

/// A SUMO shape: positions separated by spaces, each `x,y` or `x,y,z`. The z is read
/// and dropped — this is a plan view — but it has to be read to know it is there.
fn shape(element: &Element) -> Option<Vec<Point>> {
    let text = element.attributes.get("shape")?;
    let mut points = Vec::new();
    for position in text.split_whitespace() {
        let mut parts = position.split(',');
        let x: f64 = parts.next()?.parse().ok()?;
        let y: f64 = parts.next()?.parse().ok()?;
        points.push(Point::new(x, y));
    }
    Some(points)
}

/// Every element with one of these names, in document order.
///
/// A plain-XML file is attributes on flat tags, so there is nothing to keep a stack
/// for: the elements come out in the order they were written and their nesting is
/// recovered, where it matters, by the order alone.
fn elements(xml: &str, wanted: &[&str]) -> Result<Vec<Element>, ViewError> {
    let mut reader = Reader::from_str(xml);
    reader.config_mut().trim_text(true);
    let mut found = Vec::new();
    loop {
        match reader.read_event() {
            Ok(Event::Eof) => break,
            Ok(Event::Start(start)) | Ok(Event::Empty(start)) => {
                let name = String::from_utf8_lossy(start.name().as_ref()).into_owned();
                if wanted.contains(&name.as_str()) {
                    found.push(Element {
                        name,
                        attributes: attributes(&start)?,
                    });
                }
            }
            Ok(_) => {}
            Err(error) => {
                return Err(ViewError::Parse(format!(
                    "at byte {}: {error}",
                    reader.buffer_position()
                )))
            }
        }
    }
    Ok(found)
}

fn attributes(start: &BytesStart<'_>) -> Result<HashMap<String, String>, ViewError> {
    let mut attributes = HashMap::new();
    for attribute in start.attributes() {
        let attribute = attribute.map_err(|error| ViewError::Parse(error.to_string()))?;
        let key = String::from_utf8_lossy(attribute.key.as_ref()).into_owned();
        // The plain-XML files declare 1.0, and a file that declares nothing is 1.0
        // by the specification. Normalising against that is what turns `&amp;` in an
        // attribute back into the character the writer meant.
        let value = attribute
            .normalized_value(XmlVersion::Implicit1_0)
            .map_err(|error| ViewError::Parse(error.to_string()))?
            .into_owned();
        attributes.insert(key, value);
    }
    Ok(attributes)
}

#[cfg(test)]
mod tests {
    use super::*;

    const NODES: &str = r#"<nodes>
        <node id="j_x" x="0.000" y="0.000" z="0.000" type="traffic_light"/>
        <node id="n_north_start" x="0.000" y="70.000" z="0.000" type="dead_end"/>
    </nodes>"#;

    const EDGES: &str = r#"<edges>
        <edge id="north.fwd" from="n_north_start" to="j_x" numLanes="1"
              shape="-1.750,70.000,0.000 -1.750,14.000,0.000" name="north">
            <lane index="0" width="3.500" shape="-1.750,70.000,0.000 -1.750,14.000,0.000"/>
        </edge>
        <edge id="east.fwd" from="j_x" to="n_east_end" numLanes="1"
              shape="14.000,-1.750,0.000 70.000,-1.750,0.000" name="east">
            <lane index="0" width="3.500" shape="14.000,-1.750,0.000 70.000,-1.750,0.000"/>
        </edge>
    </edges>"#;

    const CONNECTIONS: &str = r#"<connections>
        <connection from="north.fwd" to="east.fwd" fromLane="0" toLane="0"/>
    </connections>"#;

    fn network() -> Network {
        Network {
            nodes: NODES.into(),
            edges: EDGES.into(),
            connections: Some(CONNECTIONS.into()),
        }
    }

    #[test]
    fn a_lane_is_drawn_from_its_own_shape() {
        let drawing = draw(&network()).unwrap();
        let centres: Vec<&Shape> = drawing
            .shapes
            .iter()
            .filter(|shape| shape.kind() == Kind::Center)
            .collect();
        assert_eq!(centres.len(), 2);
        let Shape::Path { points, .. } = centres[0] else {
            panic!("a lane is a path");
        };
        assert_eq!(points[0], Point::new(-1.75, 70.0));
        assert_eq!(points[1], Point::new(-1.75, 14.0));
    }

    #[test]
    fn a_lane_carries_the_width_it_was_written_with() {
        let drawing = draw(&network()).unwrap();
        let widths: Vec<f64> = drawing
            .shapes
            .iter()
            .filter_map(|shape| match shape {
                Shape::Band { width, .. } => Some(*width),
                _ => None,
            })
            .collect();
        assert_eq!(widths, vec![3.5, 3.5]);
    }

    #[test]
    fn a_connection_joins_the_two_lanes_it_names() {
        let drawing = draw(&network()).unwrap();
        let movements: Vec<&Shape> = drawing
            .shapes
            .iter()
            .filter(|shape| shape.kind() == Kind::Movement)
            .collect();
        assert_eq!(movements.len(), 1);
        let Shape::Path { points, .. } = movements[0] else {
            panic!("a movement is a path");
        };
        // Out of the end of north.fwd and into the start of east.fwd.
        assert_eq!(points[0], Point::new(-1.75, 14.0));
        assert_eq!(points[1], Point::new(14.0, -1.75));
    }

    #[test]
    fn a_connection_to_an_edge_that_is_not_there_is_not_drawn() {
        let mut network = network();
        network.connections =
            Some(r#"<connections><connection from="nowhere" to="east.fwd"/></connections>"#.into());
        let drawing = draw(&network).unwrap();
        assert!(!drawing
            .shapes
            .iter()
            .any(|shape| shape.kind() == Kind::Movement));
    }

    #[test]
    fn the_nodes_are_where_the_file_puts_them() {
        let drawing = draw(&network()).unwrap();
        let nodes: Vec<&Shape> = drawing
            .shapes
            .iter()
            .filter(|shape| shape.kind() == Kind::Node)
            .collect();
        assert_eq!(nodes.len(), 2);
    }

    #[test]
    fn xml_that_is_not_xml_is_an_error_rather_than_an_empty_picture() {
        let network = Network {
            nodes: "<nodes><node".into(),
            edges: EDGES.into(),
            connections: None,
        };
        assert!(matches!(draw(&network), Err(ViewError::Parse(_))));
    }
}
