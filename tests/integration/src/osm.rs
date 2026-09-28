//! A Lanelet2 map written by hand, node by node, for the tests that need a shape
//! no generator draws: a lane forking outside a junction, a turn that branches, a
//! lane whose ends are drawn steeply across it.

/// Metres to degrees near the origin, which is all a map this small needs.
const LAT0: f64 = 35.0;
const LON0: f64 = 139.0;

pub struct Osm {
    nodes: Vec<(i64, f64, f64, f64)>,
    ways: Vec<(i64, Vec<i64>)>,
    /// `(id, left way, right way, turn_direction)`.
    lanelets: Vec<(i64, i64, i64, Option<&'static str>)>,
    next: i64,
}

impl Osm {
    pub fn new() -> Self {
        Osm {
            nodes: Vec::new(),
            ways: Vec::new(),
            lanelets: Vec::new(),
            next: 1,
        }
    }

    fn id(&mut self) -> i64 {
        self.next += 1;
        self.next
    }

    pub fn node(&mut self, x: f64, y: f64, z: f64) -> i64 {
        let id = self.id();
        self.nodes.push((id, x, y, z));
        id
    }

    pub fn way(&mut self, nodes: Vec<i64>) -> i64 {
        let id = self.id();
        self.ways.push((id, nodes));
        id
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
        for (id, x, y, z) in &self.nodes {
            let lat = LAT0 + y / metres_per_degree;
            let lon = LON0 + x / (metres_per_degree * LAT0.to_radians().cos());
            xml.push_str(&format!(
                "<node id=\"{id}\" lat=\"{lat:.12}\" lon=\"{lon:.12}\"><tag k=\"ele\" v=\"{z}\"/></node>\n"
            ));
        }
        for (id, nodes) in &self.ways {
            xml.push_str(&format!("<way id=\"{id}\">"));
            for node in nodes {
                xml.push_str(&format!("<nd ref=\"{node}\"/>"));
            }
            xml.push_str(
                "<tag k=\"type\" v=\"line_thin\"/><tag k=\"subtype\" v=\"solid\"/></way>\n",
            );
        }
        for (id, left, right, turn) in &self.lanelets {
            let turn = turn
                .map(|direction| format!("<tag k=\"turn_direction\" v=\"{direction}\"/>"))
                .unwrap_or_default();
            xml.push_str(&format!(
                "<relation id=\"{id}\"><member type=\"way\" ref=\"{left}\" role=\"left\"/>\
                 <member type=\"way\" ref=\"{right}\" role=\"right\"/>\
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
