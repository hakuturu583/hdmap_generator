//! The CommonRoad scenario as plain data, and how it is written.
//!
//! These types follow the 2020a XML schema element for element, in the order the
//! schema's sequences put them, so that [`Scenario::to_xml`] is a walk over them and
//! nothing else. Every id is drawn from one counter: CommonRoad keeps lanelets,
//! signs, lights, intersections, their incomings and planning problems in a single
//! id space, and a reference may name any of them.

use roadgen_core::geometry::Point3;

use crate::xml::Document;

/// The format version written in the `commonRoadVersion` attribute.
pub const COMMONROAD_VERSION: &str = "2020a";

/// One CommonRoad scenario: the header, the road network, and the planning
/// problems posed on it.
#[derive(Debug, Clone, PartialEq)]
pub struct Scenario {
    pub benchmark_id: String,
    /// `YYYY-MM-DD`.
    pub date: String,
    pub author: String,
    pub affiliation: String,
    pub source: String,
    /// Seconds per time step.
    pub time_step: f64,
    pub location: Location,
    /// The scenario tags, as the schema's element names: `simulated`, `urban`.
    pub tags: Vec<&'static str>,
    pub lanelets: Vec<Lanelet>,
    pub traffic_signs: Vec<TrafficSign>,
    pub traffic_lights: Vec<TrafficLight>,
    pub intersections: Vec<Intersection>,
    pub planning_problems: Vec<PlanningProblem>,
    /// Whether points carry their `z`. The schema allows it; most CommonRoad tools
    /// work in the plane and ignore it, or fail on it.
    pub elevation: bool,
}

#[derive(Debug, Clone, PartialEq)]
pub struct Location {
    /// GeoNames id of the place; -999 when the scenario is in no real one.
    pub geo_name_id: i64,
    pub latitude: f64,
    pub longitude: f64,
    /// The PROJ string the scenario's metres are in, which CommonRoad's tools use
    /// to put it on the globe.
    pub geo_reference: String,
}

/// A lane boundary as one lanelet sees it.
#[derive(Debug, Clone, PartialEq)]
pub struct Bound {
    pub points: Vec<Point3>,
    pub line_marking: &'static str,
}

/// Another lanelet beside this one, and whether it runs the same way.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Adjacent {
    pub lanelet: u64,
    pub same_direction: bool,
}

#[derive(Debug, Clone, PartialEq)]
pub struct StopLine {
    /// The line's two ends, or none for a line across the lanelet's end. It is
    /// written as a solid line.
    pub points: Option<[Point3; 2]>,
    pub traffic_signs: Vec<u64>,
    pub traffic_lights: Vec<u64>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct Lanelet {
    pub id: u64,
    /// The bound on the left of a road user going the lanelet's way; it has as many
    /// points as `right`.
    pub left: Bound,
    pub right: Bound,
    pub predecessors: Vec<u64>,
    pub successors: Vec<u64>,
    pub adjacent_left: Option<Adjacent>,
    pub adjacent_right: Option<Adjacent>,
    pub stop_line: Option<StopLine>,
    /// At least one.
    pub lanelet_types: Vec<&'static str>,
    pub user_one_way: Vec<&'static str>,
    pub user_bidirectional: Vec<&'static str>,
    pub traffic_signs: Vec<u64>,
    pub traffic_lights: Vec<u64>,
}

impl Lanelet {
    /// A lanelet with its bounds and what it is, and nothing yet beside, before
    /// or after it or governing it.
    pub fn new(
        id: u64,
        left: Bound,
        right: Bound,
        lanelet_types: Vec<&'static str>,
        user_one_way: Vec<&'static str>,
        user_bidirectional: Vec<&'static str>,
    ) -> Self {
        Lanelet {
            id,
            left,
            right,
            predecessors: Vec::new(),
            successors: Vec::new(),
            adjacent_left: None,
            adjacent_right: None,
            stop_line: None,
            lanelet_types,
            user_one_way,
            user_bidirectional,
            traffic_signs: Vec::new(),
            traffic_lights: Vec::new(),
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct TrafficSignElement {
    /// The sign's id in CommonRoad's German catalogue: `206`, `274`.
    pub sign_id: &'static str,
    pub additional_values: Vec<String>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct TrafficSign {
    pub id: u64,
    pub elements: Vec<TrafficSignElement>,
    /// Where it stands; `None` leaves it to the reader, which puts it beside the
    /// first lanelet that refers to it.
    pub position: Option<Point3>,
    /// A sign that holds a rule but stands nowhere: a speed limit the IR keeps on
    /// the lanes rather than on a post.
    pub virtual_sign: bool,
}

#[derive(Debug, Clone, PartialEq)]
pub struct CycleElement {
    /// Time steps.
    pub duration: u64,
    pub color: &'static str,
}

#[derive(Debug, Clone, PartialEq)]
pub struct TrafficLight {
    pub id: u64,
    pub cycle: Vec<CycleElement>,
    /// The time step the cycle starts at; before it, the cycle is already running,
    /// as far through as it would have got.
    pub time_offset: u64,
    /// Where it stands. It governs every movement from the lanes that stop for it:
    /// its direction is written as `all`.
    pub position: Point3,
}

#[derive(Debug, Clone, PartialEq, Default)]
pub struct Incoming {
    pub id: u64,
    pub incoming_lanelets: Vec<u64>,
    pub successors_right: Vec<u64>,
    pub successors_straight: Vec<u64>,
    pub successors_left: Vec<u64>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct Intersection {
    pub id: u64,
    pub incomings: Vec<Incoming>,
    /// The crosswalk lanelets across it.
    pub crossing: Vec<u64>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct InitialState {
    pub position: Point3,
    /// Radians anticlockwise from `+x`.
    pub orientation: f64,
    /// Metres per second.
    pub velocity: f64,
}

#[derive(Debug, Clone, PartialEq)]
pub struct GoalState {
    /// The window of time steps the goal has to be reached in.
    pub time: (u64, u64),
    pub lanelets: Vec<u64>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct PlanningProblem {
    pub id: u64,
    pub initial_state: InitialState,
    pub goal_states: Vec<GoalState>,
}

impl Scenario {
    /// The scenario as a CommonRoad XML document.
    pub fn to_xml(&self) -> String {
        let mut doc = Document::new();
        doc.open(
            "commonRoad",
            &[
                ("timeStepSize", number(self.time_step)),
                ("commonRoadVersion", COMMONROAD_VERSION.into()),
                ("author", self.author.clone()),
                ("affiliation", self.affiliation.clone()),
                ("source", self.source.clone()),
                ("benchmarkID", self.benchmark_id.clone()),
                ("date", self.date.clone()),
            ],
        );

        doc.open("location", &[]);
        doc.text("geoNameId", &self.location.geo_name_id.to_string());
        doc.text("gpsLatitude", &number(self.location.latitude));
        doc.text("gpsLongitude", &number(self.location.longitude));
        {
            // The schema has the transformation after the reference as part of one
            // sequence, so a scenario that has a reference has the identity too.
            doc.open("geoTransformation", &[]);
            doc.text("geoReference", &self.location.geo_reference);
            doc.open("additionalTransformation", &[]);
            doc.text("xTranslation", "0.0");
            doc.text("yTranslation", "0.0");
            doc.text("zRotation", "0.0");
            doc.text("scaling", "1.0");
            doc.close("additionalTransformation");
            doc.close("geoTransformation");
        }
        doc.close("location");

        if self.tags.is_empty() {
            doc.empty("scenarioTags", &[]);
        } else {
            doc.open("scenarioTags", &[]);
            for tag in &self.tags {
                doc.empty(tag, &[]);
            }
            doc.close("scenarioTags");
        }

        for lanelet in &self.lanelets {
            self.lanelet(&mut doc, lanelet);
        }
        for sign in &self.traffic_signs {
            self.traffic_sign(&mut doc, sign);
        }
        for light in &self.traffic_lights {
            self.traffic_light(&mut doc, light);
        }
        for intersection in &self.intersections {
            intersection_xml(&mut doc, intersection);
        }
        for problem in &self.planning_problems {
            self.planning_problem(&mut doc, problem);
        }

        doc.close("commonRoad");
        doc.finish()
    }

    fn point(&self, doc: &mut Document, name: &str, point: Point3) {
        doc.open(name, &[]);
        doc.text("x", &number(point.x));
        doc.text("y", &number(point.y));
        if self.elevation {
            doc.text("z", &number(point.z));
        }
        doc.close(name);
    }

    fn bound(&self, doc: &mut Document, name: &str, bound: &Bound) {
        doc.open(name, &[]);
        for point in &bound.points {
            self.point(doc, "point", *point);
        }
        doc.text("lineMarking", bound.line_marking);
        doc.close(name);
    }

    fn lanelet(&self, doc: &mut Document, lanelet: &Lanelet) {
        doc.open("lanelet", &[("id", lanelet.id.to_string())]);
        self.bound(doc, "leftBound", &lanelet.left);
        self.bound(doc, "rightBound", &lanelet.right);
        refs(doc, "predecessor", &lanelet.predecessors);
        refs(doc, "successor", &lanelet.successors);
        for (name, adjacent) in [
            ("adjacentLeft", lanelet.adjacent_left),
            ("adjacentRight", lanelet.adjacent_right),
        ] {
            if let Some(adjacent) = adjacent {
                let direction = if adjacent.same_direction {
                    "same"
                } else {
                    "opposite"
                };
                doc.empty(
                    name,
                    &[
                        ("ref", adjacent.lanelet.to_string()),
                        ("drivingDir", direction.into()),
                    ],
                );
            }
        }
        if let Some(stop) = &lanelet.stop_line {
            doc.open("stopLine", &[]);
            if let Some(points) = stop.points {
                for point in points {
                    self.point(doc, "point", point);
                }
            }
            doc.text("lineMarking", "solid");
            refs(doc, "trafficSignRef", &stop.traffic_signs);
            refs(doc, "trafficLightRef", &stop.traffic_lights);
            doc.close("stopLine");
        }
        for kind in &lanelet.lanelet_types {
            doc.text("laneletType", kind);
        }
        for user in &lanelet.user_one_way {
            doc.text("userOneWay", user);
        }
        for user in &lanelet.user_bidirectional {
            doc.text("userBidirectional", user);
        }
        refs(doc, "trafficSignRef", &lanelet.traffic_signs);
        refs(doc, "trafficLightRef", &lanelet.traffic_lights);
        doc.close("lanelet");
    }

    fn traffic_sign(&self, doc: &mut Document, sign: &TrafficSign) {
        doc.open("trafficSign", &[("id", sign.id.to_string())]);
        for element in &sign.elements {
            doc.open("trafficSignElement", &[]);
            doc.text("trafficSignID", element.sign_id);
            for value in &element.additional_values {
                doc.text("additionalValue", value);
            }
            doc.close("trafficSignElement");
        }
        if let Some(position) = sign.position {
            doc.open("position", &[]);
            self.point(doc, "point", position);
            doc.close("position");
        }
        if sign.virtual_sign {
            doc.text("virtual", "true");
        }
        doc.close("trafficSign");
    }

    fn traffic_light(&self, doc: &mut Document, light: &TrafficLight) {
        doc.open("trafficLight", &[("id", light.id.to_string())]);
        doc.open("cycle", &[]);
        for element in &light.cycle {
            doc.open("cycleElement", &[]);
            doc.text("duration", &element.duration.to_string());
            doc.text("color", element.color);
            doc.close("cycleElement");
        }
        // The schema's offset is a positive integer, so none is no offset.
        if light.time_offset > 0 {
            doc.text("timeOffset", &light.time_offset.to_string());
        }
        doc.close("cycle");
        doc.open("position", &[]);
        self.point(doc, "point", light.position);
        doc.close("position");
        doc.text("direction", "all");
        doc.text("active", "true");
        doc.close("trafficLight");
    }

    fn planning_problem(&self, doc: &mut Document, problem: &PlanningProblem) {
        doc.open("planningProblem", &[("id", problem.id.to_string())]);
        let state = &problem.initial_state;
        doc.open("initialState", &[]);
        doc.open("position", &[]);
        self.point(doc, "point", state.position);
        doc.close("position");
        exact(doc, "velocity", &number(state.velocity));
        exact(doc, "orientation", &number(state.orientation));
        exact(doc, "yawRate", "0.0");
        exact(doc, "slipAngle", "0.0");
        exact(doc, "time", "0");
        doc.close("initialState");
        for goal in &problem.goal_states {
            doc.open("goalState", &[]);
            doc.open("time", &[]);
            doc.text("intervalStart", &goal.time.0.to_string());
            doc.text("intervalEnd", &goal.time.1.to_string());
            doc.close("time");
            if !goal.lanelets.is_empty() {
                doc.open("position", &[]);
                refs(doc, "lanelet", &goal.lanelets);
                doc.close("position");
            }
            doc.close("goalState");
        }
        doc.close("planningProblem");
    }
}

fn intersection_xml(doc: &mut Document, intersection: &Intersection) {
    doc.open("intersection", &[("id", intersection.id.to_string())]);
    for incoming in &intersection.incomings {
        doc.open("incoming", &[("id", incoming.id.to_string())]);
        refs(doc, "incomingLanelet", &incoming.incoming_lanelets);
        refs(doc, "successorsRight", &incoming.successors_right);
        refs(doc, "successorsStraight", &incoming.successors_straight);
        refs(doc, "successorsLeft", &incoming.successors_left);
        doc.close("incoming");
    }
    if !intersection.crossing.is_empty() {
        doc.open("crossing", &[]);
        refs(doc, "crossingLanelet", &intersection.crossing);
        doc.close("crossing");
    }
    doc.close("intersection");
}

fn refs(doc: &mut Document, name: &str, ids: &[u64]) {
    for id in ids {
        doc.empty(name, &[("ref", id.to_string())]);
    }
}

fn exact(doc: &mut Document, name: &str, value: &str) {
    doc.open(name, &[]);
    doc.text("exact", value);
    doc.close(name);
}

/// A number as an `xs:decimal`: Rust's shortest round-tripping text, which never
/// uses an exponent — and the schema's decimal type has none.
pub(crate) fn number(value: f64) -> String {
    let value = if value == 0.0 { 0.0 } else { value };
    let text = format!("{value}");
    if text.contains('.') {
        text
    } else {
        format!("{text}.0")
    }
}

#[cfg(test)]
mod tests {
    use super::number;

    #[test]
    fn numbers_are_decimals_without_exponents() {
        assert_eq!(number(1e-7), "0.0000001");
        assert_eq!(number(12.0), "12.0");
        assert_eq!(number(-0.0), "0.0");
        assert_eq!(number(-3.25), "-3.25");
    }
}
