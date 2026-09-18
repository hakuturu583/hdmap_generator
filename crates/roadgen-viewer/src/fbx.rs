//! Reading a CARLA package's FBX back, and drawing it.
//!
//! The other four readers here take a file that describes a road network and draw the
//! network. This one takes a file that describes a *surface* — triangles, with no
//! lanes, no topology and no widths in it — so the question it answers is a different
//! one: **what will CARLA think each of these meshes is?**
//!
//! That is the question worth asking of this format, because it is the one nothing
//! else can answer until the map is in the simulator. A mesh in the wrong semantic
//! class renders correctly, drives correctly, and is wrong only in the segmentation
//! image. So the picture is coloured by class, and the class is worked out the way
//! CARLA works it out: by matching the mesh's *name* against the substrings
//! `MoveAssetsCommandlet` matches, in the order it matches them.
//!
//! Nothing about this reader is roadgen-specific. It reads the names out of the file
//! and runs them through the classifier, so an FBX exported from RoadRunner — or one
//! hand-edited after roadgen wrote it — is classified exactly as CARLA would.
//!
//! # What is drawn
//!
//! The **silhouette** of each mesh: the edges used by one triangle rather than two,
//! chained into loops and flattened. For a road surface, a pavement or a kerb — each
//! of which is a ribbon — that is its outline, and a few dozen points instead of a
//! few thousand triangles.
//!
//! A building has no silhouette, because a closed solid has no edge used only once.
//! Those are drawn as their triangles, flattened, which from above is the footprint
//! with the roof over it.
//!
//! # Text FBX only
//!
//! This reads the encoding [`roadgen_carla`] writes, which is FBX 7.x in text form.
//! A binary FBX is the same node tree with a different envelope — one that needs
//! DEFLATE and a footer check — and neither this nor the writer has either.

use std::collections::BTreeMap;

use crate::drawing::{Drawing, Kind, Mark, MarkColor, Point};
use crate::error::ViewError;

/// Draws an FBX document.
pub fn draw(text: &str) -> Result<Drawing, ViewError> {
    let document = parse(text)?;
    let meshes = document.meshes();

    let mut drawing = Drawing::new("CARLA (FBX)");
    if meshes.is_empty() {
        drawing.note(
            "the file holds no mesh: CARLA imports an .fbx as static meshes, and a \
             document with none of them makes an empty level",
        );
        return Ok(drawing);
    }

    let mut classes: BTreeMap<&'static str, usize> = BTreeMap::new();
    let mut silhouettes = 0;
    let mut solids = 0;
    for mesh in &meshes {
        let label = roadgen_carla::tags::label_of(&mesh.name);
        *classes.entry(label.as_str()).or_default() += 1;
        let kind = kind_of(label, mesh);

        let loops = mesh.silhouette();
        if loops.is_empty() {
            solids += 1;
            for triangle in mesh.triangles() {
                drawing.area(kind, triangle);
            }
        } else {
            silhouettes += 1;
            for outline in loops {
                drawing.area(kind, outline);
            }
        }
    }

    let counted: Vec<String> = classes
        .iter()
        .map(|(label, count)| format!("{count} {label}"))
        .collect();
    drawing.note(format!(
        "{} meshes, tagged by name the way CARLA tags them: {}",
        meshes.len(),
        counted.join(", ")
    ));
    drawing.note(
        "the colours are CARLA's semantic classes, worked out from each mesh's name \
         by the same substring match its MoveAssets commandlet uses — so a mesh drawn \
         in the wrong colour here is one that will segment wrongly in the simulator",
    );
    if solids > 0 {
        drawing.note(format!(
            "{silhouettes} meshes are drawn as their outline and {solids} as their \
             triangles: a closed solid has no edge belonging to one face, so a \
             building has no silhouette to take"
        ));
    }
    let rejected = meshes
        .iter()
        .filter(|mesh| roadgen_carla::tags::is_rejected_by_import(&mesh.name).is_some())
        .count();
    if rejected > 0 {
        drawing.note(format!(
            "{rejected} of these will not reach the map at all: CARLA drops any mesh \
             whose name holds `light` or `sign`"
        ));
    }
    Ok(drawing)
}

/// Which of the drawing's kinds a CARLA class is shown as.
///
/// The vocabulary is the drawing's, not CARLA's: a reader says what a thing *is* and
/// [`crate::svg`] decides what it looks like. So the six classes a package can hold
/// land on the five the rest of the viewer already draws, and a class with no word
/// for it in that vocabulary is drawn as the ground it will be tagged as.
fn kind_of(label: roadgen_carla::Label, mesh: &Mesh) -> Kind {
    use roadgen_carla::Label;
    match label {
        Label::Roads => Kind::Surface,
        // The colour is read off the material, because that is where CARLA reads it:
        // `PrepareAssetsForCooking` gives its yellow lane material to the slot whose
        // name holds `Yellow`. Solid rather than broken, because a dashed line in
        // this format is a dashed line of *geometry* — each dash is its own strip, and
        // stroking the outline of one dash dashed would draw a dash of dashes.
        Label::RoadLines => Kind::Marking(Mark::new(
            if mesh.materials.iter().any(|name| name.contains("Yellow")) {
                MarkColor::Yellow
            } else {
                MarkColor::White
            },
            false,
        )),
        Label::Sidewalks => Kind::Sidewalk,
        Label::Buildings => Kind::Building,
        _ => Kind::Terrain,
    }
}

/// One static mesh, as much of it as a plan view needs.
#[derive(Debug, Clone, PartialEq)]
pub struct Mesh {
    /// The name CARLA will classify. Taken off the `Model` where the file connects
    /// one, because that is the name the imported asset takes.
    pub name: String,
    /// Vertices, in three dimensions.
    ///
    /// The drawing is a plan view and drops the heights, but the *silhouette* cannot:
    /// a kerb face is two rails at the same place on the ground and different heights,
    /// and flattening before welding would collapse it into a line.
    pub vertices: Vec<[f64; 3]>,
    /// Polygons, as indices into `vertices`.
    pub polygons: Vec<Vec<usize>>,
    /// The names of the materials connected to this mesh.
    pub materials: Vec<String>,
}

/// How finely two vertices have to agree to be the same vertex: a micrometre, which
/// is the tolerance the workspace holds geometry to everywhere else.
const WELD: f64 = 1e6;

impl Mesh {
    /// A vertex in plan view.
    fn plan(&self, index: usize) -> Point {
        let [x, y, _] = self.vertices[index];
        Point::new(x, y)
    }

    /// Every polygon, flattened.
    pub fn triangles(&self) -> Vec<Vec<Point>> {
        self.polygons
            .iter()
            .map(|polygon| polygon.iter().map(|&index| self.plan(index)).collect())
            .collect()
    }

    /// The loops bounding the surface: edges used by one polygon rather than two.
    ///
    /// Empty for a closed solid, which is the answer rather than a failure — a
    /// building is closed, and the caller draws its triangles instead.
    ///
    /// Edges are matched by **position** rather than by index. A writer is free to
    /// give every face its own vertices — this one does, so that the crease between
    /// two walls stays a crease — and an index-matched border would then find every
    /// face of a closed solid to be its own island. Welding first is what makes the
    /// question "is this surface closed" a question about the shape rather than about
    /// how the file happened to be written.
    pub fn silhouette(&self) -> Vec<Vec<Point>> {
        let mut welded: BTreeMap<[i64; 3], usize> = BTreeMap::new();
        let mut of: Vec<usize> = Vec::with_capacity(self.vertices.len());
        for vertex in &self.vertices {
            let key = vertex.map(|value| (value * WELD).round() as i64);
            let next = welded.len();
            of.push(*welded.entry(key).or_insert(next));
        }

        let mut counts: BTreeMap<(usize, usize), usize> = BTreeMap::new();
        for polygon in &self.polygons {
            for index in 0..polygon.len() {
                let a = of[polygon[index]];
                let b = of[polygon[(index + 1) % polygon.len()]];
                if a == b {
                    continue;
                }
                *counts.entry(edge(a, b)).or_default() += 1;
            }
        }

        // A vertex's unused-once neighbours. Chaining through these walks the border.
        let mut neighbours: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
        for (&(a, b), &count) in &counts {
            if count != 1 {
                continue;
            }
            neighbours.entry(a).or_default().push(b);
            neighbours.entry(b).or_default().push(a);
        }
        // One vertex per welded position, to read the border's points back from.
        let mut first: BTreeMap<usize, usize> = BTreeMap::new();
        for (index, &welded) in of.iter().enumerate() {
            first.entry(welded).or_insert(index);
        }

        let mut loops = Vec::new();
        let mut walked: BTreeMap<(usize, usize), bool> = BTreeMap::new();
        let starts: Vec<usize> = neighbours.keys().copied().collect();
        for start in starts {
            loop {
                let Some(next) = unwalked(&neighbours, &walked, start) else {
                    break;
                };
                let mut chain = vec![start];
                let mut at = next;
                walked.insert(edge(start, at), true);
                // A cap, because a malformed file can describe a border that does not
                // close, and a reader is not the place to find that out by hanging.
                while at != start && chain.len() <= self.vertices.len() {
                    chain.push(at);
                    let Some(step) = unwalked(&neighbours, &walked, at) else {
                        break;
                    };
                    walked.insert(edge(at, step), true);
                    at = step;
                }
                if chain.len() >= 3 {
                    loops.push(
                        chain
                            .into_iter()
                            .map(|welded| self.plan(first[&welded]))
                            .collect(),
                    );
                }
            }
        }
        loops
    }
}

fn edge(a: usize, b: usize) -> (usize, usize) {
    if a < b {
        (a, b)
    } else {
        (b, a)
    }
}

fn unwalked(
    neighbours: &BTreeMap<usize, Vec<usize>>,
    walked: &BTreeMap<(usize, usize), bool>,
    at: usize,
) -> Option<usize> {
    neighbours
        .get(&at)?
        .iter()
        .copied()
        .find(|&other| !walked.contains_key(&edge(at, other)))
}

// =============================================================================
// -- The node tree ------------------------------------------------------------
// =============================================================================

/// One value of a node's property list.
#[derive(Debug, Clone, PartialEq)]
pub enum Value {
    Number(f64),
    Text(String),
    /// FBX's `*count { a: … }`, which is how it writes anything long.
    Array(Vec<f64>),
}

impl Value {
    pub fn as_text(&self) -> Option<&str> {
        match self {
            Value::Text(text) => Some(text),
            _ => None,
        }
    }

    pub fn as_number(&self) -> Option<f64> {
        match self {
            Value::Number(number) => Some(*number),
            _ => None,
        }
    }

    pub fn as_array(&self) -> Option<&[f64]> {
        match self {
            Value::Array(values) => Some(values),
            _ => None,
        }
    }
}

/// A node of the FBX tree: a name, a property list and children.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Node {
    pub name: String,
    pub properties: Vec<Value>,
    pub children: Vec<Node>,
}

impl Node {
    pub fn child(&self, name: &str) -> Option<&Node> {
        self.children.iter().find(|child| child.name == name)
    }

    pub fn children_named<'a>(&'a self, name: &'a str) -> impl Iterator<Item = &'a Node> {
        self.children.iter().filter(move |child| child.name == name)
    }

    /// The first property of a child, as an array.
    fn array(&self, name: &str) -> Option<&[f64]> {
        self.child(name)?.properties.first()?.as_array()
    }
}

/// A parsed document, with the bits of it a drawing needs.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Document {
    pub root: Node,
}

impl Document {
    /// Every mesh in the document, named the way the import will name it.
    pub fn meshes(&self) -> Vec<Mesh> {
        let Some(objects) = self.root.child("Objects") else {
            return Vec::new();
        };

        // Who is connected to whom. FBX keeps a geometry, the model it hangs off and
        // the materials on that model as separate objects joined at the end of the
        // file, so a mesh's name and its materials are two hops from its triangles.
        let mut links: Vec<(i64, i64)> = Vec::new();
        if let Some(connections) = self.root.child("Connections") {
            for connection in connections.children_named("C") {
                let kind = connection.properties.first().and_then(Value::as_text);
                if kind != Some("OO") {
                    continue;
                }
                let from = connection.properties.get(1).and_then(Value::as_number);
                let to = connection.properties.get(2).and_then(Value::as_number);
                if let (Some(from), Some(to)) = (from, to) {
                    links.push((from as i64, to as i64));
                }
            }
        }

        let mut names: BTreeMap<i64, String> = BTreeMap::new();
        for model in objects.children_named("Model") {
            if let (Some(id), Some(name)) = (identifier(model), object_name(model)) {
                names.insert(id, name);
            }
        }
        let mut materials: BTreeMap<i64, String> = BTreeMap::new();
        for material in objects.children_named("Material") {
            if let (Some(id), Some(name)) = (identifier(material), object_name(material)) {
                materials.insert(id, name);
            }
        }

        let mut meshes = Vec::new();
        for geometry in objects.children_named("Geometry") {
            let Some(id) = identifier(geometry) else {
                continue;
            };
            let model = links
                .iter()
                .find(|(from, _)| *from == id)
                .map(|(_, to)| *to);
            let name = model
                .and_then(|model| names.get(&model).cloned())
                // A geometry connected to no model still has a name of its own, and a
                // name is the whole point of reading the file.
                .or_else(|| object_name(geometry))
                .unwrap_or_default();

            let on_mesh = model
                .map(|model| {
                    links
                        .iter()
                        .filter(|(_, to)| *to == model)
                        .filter_map(|(from, _)| materials.get(from).cloned())
                        .collect()
                })
                .unwrap_or_default();

            let Some(vertices) = geometry.array("Vertices") else {
                continue;
            };
            let Some(indices) = geometry.array("PolygonVertexIndex") else {
                continue;
            };
            let points: Vec<[f64; 3]> = vertices
                .chunks_exact(3)
                .map(|triple| [triple[0], triple[1], triple[2]])
                .collect();

            // FBX marks the last index of a polygon by flipping its sign and taking
            // one off: `~i`. That is the only thing saying where one polygon stops.
            let mut polygons = Vec::new();
            let mut polygon = Vec::new();
            for &value in indices {
                let raw = value as i64;
                let (index, last) = if raw < 0 {
                    ((!raw) as usize, true)
                } else {
                    (raw as usize, false)
                };
                if index < points.len() {
                    polygon.push(index);
                }
                if last {
                    if polygon.len() >= 3 {
                        polygons.push(std::mem::take(&mut polygon));
                    } else {
                        polygon.clear();
                    }
                }
            }

            meshes.push(Mesh {
                name,
                vertices: points,
                polygons,
                materials: on_mesh,
            });
        }
        meshes
    }
}

/// An object's identifier: the first property of its declaration.
fn identifier(node: &Node) -> Option<i64> {
    node.properties.first()?.as_number().map(|id| id as i64)
}

/// An object's name, with the `Type::` prefix the text encoding puts on it taken off.
fn object_name(node: &Node) -> Option<String> {
    let name = node.properties.get(1)?.as_text()?;
    Some(match name.split_once("::") {
        Some((_, rest)) => rest.to_owned(),
        None => name.to_owned(),
    })
}

/// Reads an FBX document.
pub fn parse(text: &str) -> Result<Document, ViewError> {
    let mut parser = Parser {
        bytes: text.as_bytes(),
        at: 0,
    };
    let root = Node {
        children: parser.nodes(0)?,
        ..Node::default()
    };
    Ok(Document { root })
}

struct Parser<'a> {
    bytes: &'a [u8],
    at: usize,
}

/// How deep a document may nest.
///
/// FBX is a tree and a malformed one can be a very deep tree. Six levels is twice
/// what any document this reads goes to, and a limit is what keeps a bad file from
/// becoming a stack overflow.
const MAX_DEPTH: usize = 16;

impl<'a> Parser<'a> {
    fn peek(&self) -> Option<u8> {
        self.bytes.get(self.at).copied()
    }

    fn skip_blank(&mut self) {
        while let Some(byte) = self.peek() {
            if byte == b';' {
                while let Some(byte) = self.peek() {
                    self.at += 1;
                    if byte == b'\n' {
                        break;
                    }
                }
            } else if byte.is_ascii_whitespace() {
                self.at += 1;
            } else {
                break;
            }
        }
    }

    /// Spaces and tabs, but not a newline — which is what ends a property list.
    fn skip_spaces(&mut self) {
        while matches!(self.peek(), Some(b' ' | b'\t' | b'\r')) {
            self.at += 1;
        }
    }

    fn nodes(&mut self, depth: usize) -> Result<Vec<Node>, ViewError> {
        if depth > MAX_DEPTH {
            return Err(ViewError::Parse(format!(
                "the FBX document nests more than {MAX_DEPTH} deep"
            )));
        }
        let mut nodes = Vec::new();
        loop {
            self.skip_blank();
            match self.peek() {
                None | Some(b'}') => return Ok(nodes),
                _ => {}
            }
            nodes.push(self.node(depth)?);
        }
    }

    fn node(&mut self, depth: usize) -> Result<Node, ViewError> {
        let start = self.at;
        while let Some(byte) = self.peek() {
            if byte == b':' {
                break;
            }
            if byte == b'\n' || byte == b'{' || byte == b'}' {
                return Err(ViewError::Parse(format!(
                    "an FBX node name with no colon after it, at byte {start}"
                )));
            }
            self.at += 1;
        }
        if self.peek().is_none() {
            return Err(ViewError::Parse(
                "the FBX document ends in the middle of a node name".into(),
            ));
        }
        let name = String::from_utf8_lossy(&self.bytes[start..self.at])
            .trim()
            .to_owned();
        self.at += 1; // the colon

        let mut node = Node {
            name,
            ..Node::default()
        };
        node.properties = self.properties()?;

        self.skip_spaces();
        if self.peek() == Some(b'{') {
            self.at += 1;
            node.children = self.nodes(depth + 1)?;
            self.skip_blank();
            if self.peek() != Some(b'}') {
                return Err(ViewError::Parse(format!(
                    "the FBX node `{}` is never closed",
                    node.name
                )));
            }
            self.at += 1;
        }
        Ok(node)
    }

    fn properties(&mut self) -> Result<Vec<Value>, ViewError> {
        let mut values = Vec::new();
        loop {
            self.skip_spaces();
            match self.peek() {
                None | Some(b'\n') | Some(b'{') | Some(b'}') => return Ok(values),
                Some(b',') => {
                    self.at += 1;
                }
                Some(b'"') => values.push(self.text()?),
                Some(b'*') => values.push(self.array()?),
                Some(b';') => {
                    // A trailing comment ends the property list as a newline would.
                    while !matches!(self.peek(), None | Some(b'\n')) {
                        self.at += 1;
                    }
                    return Ok(values);
                }
                _ => values.push(self.word()),
            }
        }
    }

    fn text(&mut self) -> Result<Value, ViewError> {
        self.at += 1; // the opening quote
        let start = self.at;
        while let Some(byte) = self.peek() {
            if byte == b'"' {
                let text = String::from_utf8_lossy(&self.bytes[start..self.at]).into_owned();
                self.at += 1;
                return Ok(Value::Text(text));
            }
            if byte == b'\n' {
                break;
            }
            self.at += 1;
        }
        Err(ViewError::Parse(format!(
            "an FBX string is never closed, from byte {start}"
        )))
    }

    /// `*count { a: 1,2,3 }` — FBX's way of writing anything long.
    fn array(&mut self) -> Result<Value, ViewError> {
        self.at += 1; // the star
        let start = self.at;
        while matches!(self.peek(), Some(byte) if byte.is_ascii_digit()) {
            self.at += 1;
        }
        let count: usize = String::from_utf8_lossy(&self.bytes[start..self.at])
            .parse()
            .unwrap_or(0);

        self.skip_blank();
        if self.peek() != Some(b'{') {
            return Err(ViewError::Parse(format!(
                "an FBX array of {count} has no body"
            )));
        }
        self.at += 1;
        self.skip_blank();
        // The body is a single `a:` holding the values.
        if self.peek() == Some(b'a') {
            self.at += 1;
            self.skip_blank();
            if self.peek() == Some(b':') {
                self.at += 1;
            }
        }

        let mut values = Vec::with_capacity(count.min(1 << 20));
        loop {
            self.skip_blank();
            match self.peek() {
                None => {
                    return Err(ViewError::Parse(
                        "the FBX document ends in the middle of an array".into(),
                    ))
                }
                Some(b'}') => {
                    self.at += 1;
                    return Ok(Value::Array(values));
                }
                Some(b',') => {
                    self.at += 1;
                }
                _ => {
                    let Value::Number(number) = self.word() else {
                        continue;
                    };
                    values.push(number);
                }
            }
        }
    }

    /// A bare token: a number if it reads as one, and otherwise a word.
    ///
    /// `Shading: T` and `Version: 232` are both property lists of one bare token, and
    /// which of the two a token is is decided by whether it parses.
    fn word(&mut self) -> Value {
        let start = self.at;
        while let Some(byte) = self.peek() {
            if matches!(byte, b',' | b'\n' | b'{' | b'}' | b' ' | b'\t' | b'\r') {
                break;
            }
            self.at += 1;
        }
        let text = String::from_utf8_lossy(&self.bytes[start..self.at]).into_owned();
        match text.parse::<f64>() {
            Ok(number) => Value::Number(number),
            Err(_) => Value::Text(text),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const SQUARE: &str = r#"; a comment
Objects:  {
	Geometry: 1000001, "Geometry::Town01_Road_Road_0", "Mesh" {
		Vertices: *12 {
			a: 0,0,0,10,0,0,10,5,0,0,5,0
		}
		PolygonVertexIndex: *4 {
			a: 0,1,2,-4
		}
	}
	Model: 1000002, "Model::Town01_Road_Road_0", "Mesh" {
		Version: 232
		Shading: T
	}
	Material: 1000003, "Material::M_Road_Asphalt", "" {
		Version: 102
	}
}
Connections:  {
	C: "OO",1000002,0
	C: "OO",1000001,1000002
	C: "OO",1000003,1000002
}
"#;

    #[test]
    fn a_document_reads_back_as_the_nodes_it_holds() {
        let document = parse(SQUARE).unwrap();
        let objects = document.root.child("Objects").unwrap();
        assert_eq!(objects.children_named("Geometry").count(), 1);
        assert_eq!(objects.children_named("Model").count(), 1);
    }

    #[test]
    fn a_mesh_takes_the_name_of_the_model_it_hangs_off() {
        // Which is the name the imported asset gets, and therefore the name CARLA
        // classifies. Reading the geometry's own name instead would be right here by
        // accident and wrong for a file whose two names differ.
        let meshes = parse(SQUARE).unwrap().meshes();
        assert_eq!(meshes.len(), 1);
        assert_eq!(meshes[0].name, "Town01_Road_Road_0");
        assert_eq!(meshes[0].materials, vec!["M_Road_Asphalt"]);
    }

    #[test]
    fn a_polygon_ends_where_the_flipped_index_says_it_does() {
        let meshes = parse(SQUARE).unwrap().meshes();
        assert_eq!(meshes[0].polygons, vec![vec![0, 1, 2, 3]]);
        assert_eq!(meshes[0].vertices.len(), 4);
        assert_eq!(meshes[0].vertices[2], [10.0, 5.0, 0.0]);
    }

    #[test]
    fn the_silhouette_of_one_polygon_is_that_polygon() {
        let meshes = parse(SQUARE).unwrap().meshes();
        let loops = meshes[0].silhouette();
        assert_eq!(loops.len(), 1);
        assert_eq!(loops[0].len(), 4);
    }

    #[test]
    fn a_closed_solid_has_no_silhouette() {
        // Every edge of a closed surface belongs to two faces, so there is no border
        // to walk. The caller draws the triangles instead, and the drawing says so.
        let mesh = Mesh {
            name: "Town01_Building_Part_0".into(),
            // Eight vertices for four corners, because a writer that keeps a crease
            // hard gives each face its own — which is exactly the case that makes
            // welding by position rather than by index the whole point.
            vertices: vec![
                [0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0],
                [1.0, 1.0, 0.0],
                [0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0],
                [1.0, 1.0, 0.0],
                [0.0, 1.0, 0.0],
            ],
            // Two triangles of a square, plus the same square the other way round on
            // its own copies of the vertices: every edge is used twice, once welded.
            polygons: vec![vec![0, 1, 2], vec![0, 2, 3], vec![6, 5, 4], vec![7, 6, 4]],
            materials: Vec::new(),
        };
        assert!(mesh.silhouette().is_empty());
    }

    #[test]
    fn the_drawing_is_coloured_by_what_carla_will_call_each_mesh() {
        let drawing = draw(SQUARE).unwrap();
        assert_eq!(drawing.kinds(), vec![Kind::Surface]);
        assert!(drawing.notes.iter().any(|note| note.contains("1 Roads")));
    }

    #[test]
    fn a_mistagged_mesh_is_drawn_in_the_colour_it_will_be_tagged() {
        // The point of the whole panel: this reads like a pavement and CARLA will
        // call it ground, so it is drawn as ground.
        let text = SQUARE.replace("Road_Road_0", "Road_Sidwalk_0");
        let drawing = draw(&text).unwrap();
        assert_eq!(drawing.kinds(), vec![Kind::Terrain]);
    }

    #[test]
    fn a_yellow_line_is_read_off_the_material_carla_reads_it_off() {
        let text = SQUARE
            .replace("Road_Road_0", "Road_Marking_0")
            .replace("M_Road_Asphalt", "M_RoadMarking_Yellow");
        let drawing = draw(&text).unwrap();
        assert_eq!(
            drawing.kinds(),
            vec![Kind::Marking(Mark::new(MarkColor::Yellow, false))]
        );
    }

    #[test]
    fn a_document_with_no_meshes_says_so_rather_than_drawing_nothing() {
        let drawing = draw("Objects:  {\n}\n").unwrap();
        assert!(drawing.shapes.is_empty());
        assert!(drawing.notes.iter().any(|note| note.contains("no mesh")));
    }

    #[test]
    fn a_truncated_document_is_an_error_rather_than_half_a_picture() {
        assert!(parse("Objects:  {\n\tGeometry: 1, \"a\", \"Mesh\" {\n").is_err());
        assert!(parse("Name: \"never closed\n").is_err());
    }
}
