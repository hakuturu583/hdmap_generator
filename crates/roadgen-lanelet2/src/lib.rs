//! Lowers a [`ValidatedMap`] onto an Autoware-ready Lanelet2 map.
//!
//! The Lanelet2 data model, the OSM XML writer and the projections all come from
//! [`simple_lanelet2`](https://github.com/hakuturu583/simple_lanelet2); no node, way
//! or relation is assembled by hand here. This crate decides only which IR object
//! becomes which Lanelet2 primitive:
//!
//! ```text
//!   Lane                    → Lanelet
//!   3D left/right boundary  → LineString3d
//!   Geometry point          → Point3d
//!   Lane centreline         → the lanelet's `centerline` member
//!   Traffic rules           → RegulatoryElement
//!   Topology                → shared boundary points
//! ```
//!
//! ## How connectivity survives
//!
//! Lanelet2 has no successor tag. Two lanelets are continuous when the last points
//! of their boundaries *are* the first points of the next one's — the same objects,
//! not merely the same coordinates. So every vertex is interned by position: two
//! lanes that the IR says are connected end up sharing point objects, and a Lanelet2
//! routing graph then finds the same topology the IR holds. Lateral adjacency works
//! the same way, through the shared boundary linestring between neighbouring lanes.

use std::collections::HashMap;
use std::path::Path;
use std::sync::Arc;

use ll2_core::attribute::AttributeMap;
use ll2_core::id::Id;
use ll2_core::lanelet::Lanelet;
use ll2_core::linestring::LineString;
use ll2_core::map::{LaneletMap, Primitive};
use ll2_core::point::Point;
use ll2_core::regelem::{roles, RegElemKind, RegulatoryElement, RuleParameter, RuleParameterMap};
use ll2_io::osm::WriteParams;
use ll2_projection::{GpsPoint, LocalCartesian, Origin, Projector, Utm};

use roadgen_core::geometry::{Curve3, Point3, SamplingConfig};
use roadgen_core::id::{LaneId, ObjectId, RoadId};
use roadgen_core::map::{Lane, Map, Projection};
use roadgen_core::semantics::{MapObjectKind, ObjectGeometry, TrafficRule};
use roadgen_core::topology::Direction;
use roadgen_core::validation::ValidatedMap;

mod error;
mod tags;

pub use error::ExportError;

/// Positions this close together are the same vertex.
///
/// A micrometre: far below anything a generated map means to distinguish, and far
/// above the rounding of the arithmetic that produced two boundary endpoints which
/// ought to coincide exactly.
const WELD_TOLERANCE: f64 = 1e-6;

/// Builds the Lanelet2 map.
pub fn to_lanelet_map(map: &ValidatedMap) -> Result<Arc<LaneletMap>, ExportError> {
    Exporter::new(map).run()
}

/// Builds the Lanelet2 map and renders it as OSM XML.
pub fn to_osm_xml(map: &ValidatedMap) -> Result<String, ExportError> {
    let lanelet_map = to_lanelet_map(map)?;
    let projector = projector_for(map)?;
    let (document, problems) = ll2_io::save::from_map(&lanelet_map, projector.as_ref());
    if !problems.is_empty() {
        return Err(ExportError::Serialization(problems.join("; ")));
    }
    Ok(document.to_xml(WriteParams {
        josm_upload: false,
        josm_format_elevation: false,
    }))
}

/// Writes the map as a `.osm` Lanelet2 file.
pub fn write(map: &ValidatedMap, path: impl AsRef<Path>) -> Result<(), ExportError> {
    let xml = to_osm_xml(map)?;
    std::fs::write(path.as_ref(), xml)
        .map_err(|error| ExportError::Io(format!("{}: {error}", path.as_ref().display())))
}

/// Constraints Lanelet2 imposes that the IR does not.
///
/// Lanelet2 expresses continuity through shared vertices, so two lanes the IR calls
/// connected whose boundaries do not actually coincide would export as a map whose
/// routing graph is missing an edge. Validation already checks that for the default
/// tolerance; this repeats it against the tighter tolerance the welding uses.
pub fn check(map: &ValidatedMap) -> Vec<String> {
    let mut problems = Vec::new();
    let config = map.metadata.sampling;
    for connection in map.connections.iter() {
        let (Some(from), Some(to)) = (
            map.lanes.get(&connection.from.lane),
            map.lanes.get(&connection.to.lane),
        ) else {
            continue;
        };
        let (Ok(from_travel), Ok(to_travel)) =
            (from.travel_geometry(config), to.travel_geometry(config))
        else {
            continue;
        };
        let gap = from_travel
            .left
            .end_point()
            .distance_to(to_travel.left.start_point())
            .max(
                from_travel
                    .right
                    .end_point()
                    .distance_to(to_travel.right.start_point()),
            );
        if gap > WELD_TOLERANCE {
            problems.push(format!(
                "connection {} joins boundaries that are {gap:.9} m apart; Lanelet2 \
                 expresses continuity through shared points, so the lanelets will not \
                 be continuous",
                connection.id
            ));
        }
    }
    problems
}

/// The projector that turns the map's metric coordinates back into latitude and
/// longitude for the OSM file.
pub fn projector_for(map: &ValidatedMap) -> Result<Box<dyn Projector>, ExportError> {
    let origin = Origin::new(GpsPoint::new(
        map.metadata.origin.latitude(),
        map.metadata.origin.longitude(),
        map.metadata.origin.altitude(),
    ));
    Ok(match map.metadata.projection {
        Projection::LocalCartesian => Box::new(LocalCartesian::new(origin)),
        Projection::Utm => Box::new(
            Utm::new(origin, true, false)
                .map_err(|error| ExportError::Projection(error.message().to_owned()))?,
        ),
    })
}

/// Interns vertices so that coincident positions become one `Point`.
struct PointWelder {
    interned: HashMap<[i64; 3], Point>,
    next_id: i64,
}

impl PointWelder {
    fn new(first_id: i64) -> Self {
        PointWelder {
            interned: HashMap::new(),
            next_id: first_id,
        }
    }

    fn key(point: Point3) -> [i64; 3] {
        let quantum = 1.0 / WELD_TOLERANCE;
        [
            (point.x * quantum).round() as i64,
            (point.y * quantum).round() as i64,
            (point.z * quantum).round() as i64,
        ]
    }

    /// The `Point` for this position, creating it the first time it is seen.
    fn intern(&mut self, point: Point3) -> Point {
        if let Some(existing) = self.interned.get(&Self::key(point)) {
            return existing.clone();
        }
        let id = self.next_id;
        self.next_id += 1;
        // Autoware's OSM parsers read the metric position from `local_x`/`local_y`
        // rather than re-projecting the latitude and longitude, so both are written.
        let interned = Point::new(
            id,
            point.x,
            point.y,
            point.z,
            tags::attributes([
                ("local_x", format!("{:.6}", point.x)),
                ("local_y", format!("{:.6}", point.y)),
            ]),
        );
        self.interned.insert(Self::key(point), interned.clone());
        interned
    }
}

struct Exporter<'a> {
    map: &'a Map,
    lanelet_map: Arc<LaneletMap>,
    welder: PointWelder,
    next_id: i64,
    /// One linestring per road cross-section offset, shared by the lanes either side
    /// of it — which is what makes two lanelets laterally adjacent.
    boundaries: HashMap<(RoadId, i64), LineString>,
    lanelets: HashMap<LaneId, Lanelet>,
    objects: HashMap<ObjectId, Vec<LineString>>,
}

impl<'a> Exporter<'a> {
    /// Lanelet2 ids are free-form positive integers; upstream reserves everything
    /// below 1000 for hand-written maps, so generated ids start above that.
    const FIRST_ID: Id = 1000;

    fn new(map: &'a ValidatedMap) -> Self {
        Exporter {
            map: map.as_map(),
            lanelet_map: LaneletMap::new_map(),
            welder: PointWelder::new(Self::FIRST_ID),
            // Points are numbered from `FIRST_ID` upwards and everything else from a
            // block above them, so that adding a vertex cannot renumber a lanelet.
            next_id: Self::FIRST_ID + 1_000_000,
            boundaries: HashMap::new(),
            lanelets: HashMap::new(),
            objects: HashMap::new(),
        }
    }

    fn config(&self) -> SamplingConfig {
        self.map.metadata.sampling
    }

    fn take_id(&mut self) -> Id {
        let id = self.next_id;
        self.next_id += 1;
        id
    }

    fn run(mut self) -> Result<Arc<LaneletMap>, ExportError> {
        self.build_boundaries()?;
        self.build_lanelets()?;
        self.build_objects()?;
        self.build_rules()?;
        Ok(self.lanelet_map)
    }

    /// One linestring per distinct lateral offset of each road.
    fn build_boundaries(&mut self) -> Result<(), ExportError> {
        for road in self.map.roads.iter() {
            let lanes: Vec<Lane> = self.map.lanes_of(&road.id).into_iter().cloned().collect();
            // Left to right across the cross-section, so a boundary's id order
            // matches the way the road reads.
            let mut offsets: Vec<(i64, &Curve3, roadgen_core::semantics::BoundaryMarking)> =
                Vec::new();
            for lane in &lanes {
                for (offset, curve, marking) in [
                    (lane.left_offset, &lane.left_boundary, lane.left_marking),
                    (lane.right_offset, &lane.right_boundary, lane.right_marking),
                ] {
                    let key = offset_key(offset);
                    if !offsets.iter().any(|(existing, _, _)| *existing == key) {
                        offsets.push((key, curve, marking));
                    }
                }
            }
            offsets.sort_by(|a, b| b.0.cmp(&a.0));

            for (key, curve, marking) in offsets {
                let (kind, subtype) = tags::boundary_tags(marking.marking);
                let mut attributes = vec![("type", kind.to_owned())];
                if let Some(subtype) = subtype {
                    attributes.push(("subtype", subtype.to_owned()));
                }
                if kind == "line_thin" || kind == "line_thick" {
                    attributes.push(("color", marking.color.as_str().to_owned()));
                }
                let line = self.linestring(curve, tags::attributes(attributes))?;
                self.boundaries.insert((road.id.clone(), key), line);
            }
        }
        Ok(())
    }

    fn linestring(
        &mut self,
        curve: &Curve3,
        attributes: AttributeMap,
    ) -> Result<LineString, ExportError> {
        let polyline = curve.to_polyline(self.config())?;
        let points = polyline
            .points()
            .iter()
            .map(|point| self.welder.intern(*point))
            .collect();
        let id = self.take_id();
        let line = LineString::new(id, points, attributes);
        self.lanelet_map.add(Primitive::LineString(line.clone()));
        Ok(line)
    }

    fn boundary(&self, road: &RoadId, offset: f64) -> Result<LineString, ExportError> {
        self.boundaries
            .get(&(road.clone(), offset_key(offset)))
            .cloned()
            .ok_or_else(|| ExportError::Unknown(format!("boundary of {road} at {offset}")))
    }

    fn build_lanelets(&mut self) -> Result<(), ExportError> {
        for lane in self.map.lanes.iter().cloned().collect::<Vec<_>>() {
            let Some(subtype) = tags::lanelet_subtype(lane.lane_type) else {
                continue;
            };
            let inner_left = self.boundary(&lane.road, lane.left_offset)?;
            let inner_right = self.boundary(&lane.road, lane.right_offset)?;
            // A lanelet runs the way traffic does: for a backward lane that means
            // inverting both boundaries *and* swapping them, so that "left" is still
            // the driver's left.
            let (left, right) = match lane.direction {
                Direction::Forward => (inner_left, inner_right),
                Direction::Backward => (inner_right.invert(), inner_left.invert()),
            };

            let mut attributes = vec![
                ("type", "lanelet".to_owned()),
                ("subtype", subtype.to_owned()),
                ("location", self.location_of(&lane).to_owned()),
                ("one_way", "yes".to_owned()),
                (tags::participant(subtype), "yes".to_owned()),
            ];
            if let Some(limit) = lane.speed_limit {
                // A bare number in a Lanelet2 `speed_limit` tag is km/h.
                attributes.push(("speed_limit", format!("{:.0}", limit.kph())));
            }

            let id = self.take_id();
            let lanelet = Lanelet::new(id, left, right, tags::attributes(attributes));

            // The IR's centreline is a generated 3D curve, not the midpoint of the
            // boundaries, so it is written out rather than left to be recomputed.
            let travel = lane.travel_geometry(self.config())?;
            let centerline = self.linestring(
                &travel.centerline,
                tags::attributes([("type", "centerline".to_owned())]),
            )?;
            lanelet.set_centerline(centerline);

            self.lanelet_map.add(Primitive::Lanelet(lanelet.clone()));
            self.lanelets.insert(lane.id.clone(), lanelet);
        }
        Ok(())
    }

    fn location_of(&self, lane: &Lane) -> &'static str {
        self.map
            .road(&lane.road)
            .map(|road| road.road_type.lanelet2_location())
            .unwrap_or("urban")
    }

    fn build_objects(&mut self) -> Result<(), ExportError> {
        for object in self.map.objects.iter().cloned().collect::<Vec<_>>() {
            let kind = tags::object_type(&object.kind);
            let mut attributes = vec![("type", kind.to_owned())];
            match &object.kind {
                // A Lanelet2 traffic sign says which sign it is in its subtype, and
                // a regulatory element referring to one without a subtype is
                // rejected at load time.
                MapObjectKind::TrafficSign { code } => attributes.push(("subtype", code.clone())),
                MapObjectKind::TrafficLight => {
                    attributes.push(("subtype", "red_yellow_green".into()))
                }
                _ => {}
            }

            let lines = match &object.geometry {
                ObjectGeometry::Point(position) => {
                    // A single position still has to be a way; give it the shortest
                    // possible extent along x so the file stays well formed.
                    let curve = Curve3::polyline([
                        *position,
                        Point3::new(position.x + 0.01, position.y, position.z),
                    ])?;
                    vec![self.linestring(&curve, tags::attributes(attributes))?]
                }
                ObjectGeometry::Line(curve) => {
                    vec![self.linestring(curve, tags::attributes(attributes))?]
                }
                ObjectGeometry::Band { left, right } => {
                    let left_line = self.linestring(left, tags::attributes(attributes.clone()))?;
                    let right_line = self.linestring(right, tags::attributes(attributes))?;
                    if object.kind == MapObjectKind::Crosswalk {
                        // A crosswalk is a lanelet in Lanelet2, not a marking.
                        let id = self.take_id();
                        let crosswalk = Lanelet::new(
                            id,
                            left_line.clone(),
                            right_line.clone(),
                            tags::attributes([
                                ("type", "lanelet".to_owned()),
                                ("subtype", "crosswalk".to_owned()),
                                ("location", "urban".to_owned()),
                                ("one_way", "no".to_owned()),
                                (tags::participant("crosswalk"), "yes".to_owned()),
                            ]),
                        );
                        self.lanelet_map.add(Primitive::Lanelet(crosswalk));
                    }
                    vec![left_line, right_line]
                }
            };
            self.objects.insert(object.id.clone(), lines);
        }
        Ok(())
    }

    fn object_lines(&self, id: &ObjectId) -> Result<Vec<LineString>, ExportError> {
        self.objects
            .get(id)
            .cloned()
            .ok_or_else(|| ExportError::Unknown(id.to_string()))
    }

    fn build_rules(&mut self) -> Result<(), ExportError> {
        for rule in self.map.rules.clone() {
            match rule {
                TrafficRule::TrafficLight {
                    lights,
                    stop_line,
                    lanes,
                } => {
                    let mut parameters = RuleParameterMap::new();
                    let refers: Vec<RuleParameter> = lights
                        .iter()
                        .map(|light| self.object_lines(light))
                        .collect::<Result<Vec<_>, _>>()?
                        .into_iter()
                        .flatten()
                        .map(RuleParameter::LineString)
                        .collect();
                    if refers.is_empty() {
                        // Lanelet2's TrafficLight refuses to exist without one.
                        continue;
                    }
                    parameters.insert(roles::REFERS.to_owned(), refers);
                    if let Some(stop_line) = stop_line {
                        parameters.insert(
                            roles::REF_LINE.to_owned(),
                            self.object_lines(&stop_line)?
                                .into_iter()
                                .map(RuleParameter::LineString)
                                .collect(),
                        );
                    }
                    self.attach(
                        RegElemKind::TrafficLight,
                        "traffic_light",
                        parameters,
                        &lanes,
                    )?;
                }
                TrafficRule::RightOfWay {
                    right_of_way,
                    yielding,
                    stop_line,
                } => {
                    let priority: Vec<RuleParameter> = right_of_way
                        .iter()
                        .filter_map(|lane| self.lanelets.get(lane))
                        .map(|lanelet| RuleParameter::Lanelet(lanelet.downgrade()))
                        .collect();
                    if priority.is_empty() {
                        continue;
                    }
                    let mut parameters = RuleParameterMap::new();
                    parameters.insert(roles::RIGHT_OF_WAY.to_owned(), priority);
                    parameters.insert(
                        roles::YIELD.to_owned(),
                        yielding
                            .iter()
                            .filter_map(|lane| self.lanelets.get(lane))
                            .map(|lanelet| RuleParameter::Lanelet(lanelet.downgrade()))
                            .collect(),
                    );
                    if let Some(stop_line) = stop_line {
                        parameters.insert(
                            roles::REF_LINE.to_owned(),
                            self.object_lines(&stop_line)?
                                .into_iter()
                                .map(RuleParameter::LineString)
                                .collect(),
                        );
                    }
                    let attached: Vec<LaneId> =
                        right_of_way.iter().chain(&yielding).cloned().collect();
                    self.attach(
                        RegElemKind::RightOfWay,
                        "right_of_way",
                        parameters,
                        &attached,
                    )?;
                }
                // Lanelet2's `SpeedLimit` is a kind of traffic sign and needs one to
                // refer to. A limit with no sign behind it belongs on the lanelet,
                // which is also where Autoware looks for it.
                TrafficRule::SpeedLimit { limit, lanes } => {
                    for lane in &lanes {
                        if let Some(lanelet) = self.lanelets.get(lane) {
                            let mut attributes = lanelet.attributes().read().clone();
                            attributes.insert(
                                "speed_limit".to_owned(),
                                ll2_core::attribute::Attribute::new(format!("{:.0}", limit.kph())),
                            );
                            lanelet.set_attributes(attributes);
                        }
                    }
                }
            }
        }
        Ok(())
    }

    fn attach(
        &mut self,
        kind: RegElemKind,
        subtype: &str,
        parameters: RuleParameterMap,
        lanes: &[LaneId],
    ) -> Result<(), ExportError> {
        let id = self.take_id();
        let element = RegulatoryElement::new(
            kind,
            id,
            tags::attributes([
                ("type", "regulatory_element".to_owned()),
                ("subtype", subtype.to_owned()),
            ]),
            parameters,
        );
        for lane in lanes {
            if let Some(lanelet) = self.lanelets.get(lane) {
                lanelet.add_regulatory_element(element.clone());
            }
        }
        self.lanelet_map.add(Primitive::RegulatoryElement(element));
        Ok(())
    }
}

/// A lateral offset, quantised so that two lanes sharing a boundary agree on it.
fn offset_key(offset: f64) -> i64 {
    (offset / WELD_TOLERANCE).round() as i64
}
