//! A Lanelet2 map written by hand, node by node, for the tests that need a shape
//! no generator draws: a lane forking outside a junction, a turn that branches, a
//! lane whose ends are drawn steeply across it.

/// Metres to degrees near the origin, which is all a map this small needs.
const LAT0: f64 = 35.0;
const LON0: f64 = 139.0;

/// Tags of an element, as `(key, value)`.
pub type Tags = Vec<(&'static str, String)>;

/// A relation member: `(type, ref, role)`.
pub type Member = (&'static str, i64, &'static str);

pub struct Osm {
    nodes: Vec<(i64, f64, f64, f64, Tags)>,
    ways: Vec<(i64, Vec<i64>, Tags)>,
    /// `(id, left way, right way, turn_direction)`.
    lanelets: Vec<(i64, i64, i64, Option<&'static str>)>,
    /// `(id, tags, members as (type, ref, role))`.
    elements: Vec<(i64, Tags, Vec<Member>)>,
    /// `(lanelet, regulatory element)` the lanelet refers to.
    refers: Vec<(i64, i64)>,
    next: i64,
}

impl Osm {
    pub fn new() -> Self {
        Osm {
            nodes: Vec::new(),
            ways: Vec::new(),
            lanelets: Vec::new(),
            elements: Vec::new(),
            refers: Vec::new(),
            next: 1,
        }
    }

    fn id(&mut self) -> i64 {
        self.next += 1;
        self.next
    }

    pub fn node(&mut self, x: f64, y: f64, z: f64) -> i64 {
        self.tagged_node(x, y, z, Vec::new())
    }

    /// A node with tags besides its height: a traffic light's lamp.
    pub fn tagged_node(&mut self, x: f64, y: f64, z: f64, tags: Tags) -> i64 {
        let id = self.id();
        self.nodes.push((id, x, y, z, tags));
        id
    }

    /// A line of paint: `type=line_thin`, `subtype=solid`.
    pub fn way(&mut self, nodes: Vec<i64>) -> i64 {
        self.tagged_way(
            nodes,
            vec![("type", "line_thin".into()), ("subtype", "solid".into())],
        )
    }

    pub fn tagged_way(&mut self, nodes: Vec<i64>, tags: Tags) -> i64 {
        let id = self.id();
        self.ways.push((id, nodes, tags));
        id
    }

    /// A `regulatory_element` of `subtype`, with members `(type, ref, role)`.
    pub fn regulatory_element(&mut self, subtype: &str, members: Vec<Member>) -> i64 {
        let id = self.id();
        self.elements.push((
            id,
            vec![
                ("type", "regulatory_element".into()),
                ("subtype", subtype.into()),
            ],
            members,
        ));
        id
    }

    /// Makes `lanelet` refer to regulatory element `element`.
    pub fn refer(&mut self, lanelet: i64, element: i64) {
        self.refers.push((lanelet, element));
    }

    pub fn lanelet(&mut self, left: Vec<i64>, right: Vec<i64>) -> i64 {
        let (left, right) = (self.way(left), self.way(right));
        self.lanelet_on(left, right, None)
    }

    /// A lanelet between two ways that already exist, which is how two lanelets
    /// side by side share the line between them.
    pub fn lanelet_on(&mut self, left: i64, right: i64, turn: Option<&'static str>) -> i64 {
        let id = self.id();
        self.lanelets.push((id, left, right, turn));
        id
    }

    /// A lanelet tagged `turn_direction`: a movement through an intersection.
    pub fn turn(&mut self, left: Vec<i64>, right: Vec<i64>, direction: &'static str) -> i64 {
        let (left, right) = (self.way(left), self.way(right));
        self.lanelet_on(left, right, Some(direction))
    }

    /// `first`, then new nodes along `points`, all at height zero.
    pub fn line(&mut self, first: Option<i64>, points: &[(f64, f64)]) -> Vec<i64> {
        first
            .into_iter()
            .chain(points.iter().map(|(x, y)| self.node(*x, *y, 0.0)))
            .collect()
    }

    pub fn xml(&self) -> String {
        let metres_per_degree = 111_320.0;
        let mut xml = String::from("<?xml version=\"1.0\"?>\n<osm version=\"0.6\">\n");
        let tags = |tags: &Tags| -> String {
            tags.iter()
                .map(|(key, value)| format!("<tag k=\"{key}\" v=\"{value}\"/>"))
                .collect()
        };
        for (id, x, y, z, extra) in &self.nodes {
            let lat = LAT0 + y / metres_per_degree;
            let lon = LON0 + x / (metres_per_degree * LAT0.to_radians().cos());
            xml.push_str(&format!(
                "<node id=\"{id}\" lat=\"{lat:.12}\" lon=\"{lon:.12}\"><tag k=\"ele\" v=\"{z}\"/>{}</node>\n",
                tags(extra)
            ));
        }
        for (id, nodes, extra) in &self.ways {
            xml.push_str(&format!("<way id=\"{id}\">"));
            for node in nodes {
                xml.push_str(&format!("<nd ref=\"{node}\"/>"));
            }
            xml.push_str(&format!("{}</way>\n", tags(extra)));
        }
        for (id, extra, members) in &self.elements {
            xml.push_str(&format!("<relation id=\"{id}\">"));
            for (kind, reference, role) in members {
                xml.push_str(&format!(
                    "<member type=\"{kind}\" ref=\"{reference}\" role=\"{role}\"/>"
                ));
            }
            xml.push_str(&format!("{}</relation>\n", tags(extra)));
        }
        for (id, left, right, turn) in &self.lanelets {
            let elements: String = self
                .refers
                .iter()
                .filter(|(lanelet, _)| lanelet == id)
                .map(|(_, element)| {
                    format!(
                        "<member type=\"relation\" ref=\"{element}\" role=\"regulatory_element\"/>"
                    )
                })
                .collect();
            let turn = turn
                .map(|direction| format!("<tag k=\"turn_direction\" v=\"{direction}\"/>"))
                .unwrap_or_default();
            xml.push_str(&format!(
                "<relation id=\"{id}\"><member type=\"way\" ref=\"{left}\" role=\"left\"/>\
                 <member type=\"way\" ref=\"{right}\" role=\"right\"/>{elements}\
                 <tag k=\"type\" v=\"lanelet\"/><tag k=\"subtype\" v=\"road\"/>{turn}\
                 <tag k=\"one_way\" v=\"yes\"/><tag k=\"speed_limit\" v=\"40\"/></relation>\n"
            ));
        }
        xml.push_str("</osm>\n");
        xml
    }
}

impl Default for Osm {
    fn default() -> Self {
        Osm::new()
    }
}
