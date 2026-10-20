//! Turning the IR into RDS-HQ's map layers.
//!
//! An RDS-HQ map layer is one JSON document, `{layer}.json`, holding a list of
//! labels. Each label is a shape — a `polyline3d`, a `surface` or a `cuboid3d` —
//! under `labelData.shape3d`, and each shape carries its points as `vertices`, a
//! list of `[x, y, z]`. A reader finds the points by looking for that key at any
//! depth, so the key must not appear anywhere else in a label.
//!
//! Which shape a layer holds is fixed by the reader, not by the file:
//!
//! | layer             | shape      | from the IR                                     |
//! |-------------------|------------|-------------------------------------------------|
//! | `lanes`           | polyline   | each drivable lane's centreline, in travel order |
//! | `lanelines`       | polyline   | each painted cross-section edge, with its colour and style |
//! | `road_boundaries` | polyline   | the outermost edge on each side of a cross-section |
//! | `wait_lines`      | polyline   | stop lines                                      |
//! | `poles`           | polyline   | nothing: the IR models no poles                 |
//! | `crosswalks`      | polygon    | crosswalk bands                                 |
//! | `road_markings`   | polygon    | nothing: the IR models no painted symbols       |
//! | `traffic_signs`   | cuboid     | signs, as ClipGT boxes them                     |
//! | `traffic_lights`  | cuboid     | lights, as ClipGT boxes them                    |
//!
//! Every layer is written, empty or not: a renderer opens all of them and stops at
//! the first one missing.

use std::collections::HashSet;

use roadgen_clipgt::layers::{
    band_polygon, clipgt_color, device_frames, link_edge, objects_of, outer_boundaries, row_ref,
    vertices, LIGHT_DIMENSIONS, SIGN_DIMENSIONS,
};
use roadgen_core::geometry::{Curve3, Frame3, Point3};
use roadgen_core::map::Lane;
use roadgen_core::semantics::{BoundaryMarking, MapObjectKind, ObjectGeometry, RoadMarking};
use roadgen_core::{IrRef, Relation, Trace, ValidatedMap};
use serde_json::{json, Value};

use crate::error::ExportError;

/// The format name an RDS-HQ [`Trace`] is recorded under.
pub const TRACE_FORMAT: &str = "rdshq";

/// One map layer: its name, which is also its folder's (`3d_{name}`) and its JSON
/// entry's (`{name}.json`), and its labels.
#[derive(Debug, Clone, PartialEq)]
pub struct MapLayer {
    pub name: &'static str,
    pub labels: Vec<Value>,
}

impl MapLayer {
    pub fn new(name: &'static str) -> Self {
        MapLayer {
            name,
            labels: Vec::new(),
        }
    }

    /// The name the next label is traced under: `<layer>:<index>`.
    fn next_local(&self) -> String {
        row_ref(self.name, self.labels.len())
    }

    /// Adds a label made from exactly one IR element.
    fn push(&mut self, trace: &mut Trace, ir: impl Into<IrRef>, label: Value) {
        trace.link(ir, self.next_local(), Relation::Exact);
        self.labels.push(label);
    }

    /// The document a reader decodes: `{"labels": [...]}`.
    pub fn into_document(self) -> Value {
        json!({ "labels": Value::Array(self.labels) })
    }
}

/// Every map layer, in the order they are written, recording in `trace` which IR
/// element each label came from. A label is named `<layer>:<index>`.
pub fn all_traced(map: &ValidatedMap, trace: &mut Trace) -> Result<Vec<MapLayer>, ExportError> {
    Ok(vec![
        lanes(map, trace)?,
        lanelines(map, trace)?,
        road_boundaries(map, trace)?,
        wait_lines(map, trace)?,
        MapLayer::new("poles"),
        crosswalks(map, trace)?,
        MapLayer::new("road_markings"),
        devices(map, trace, "traffic_signs", SIGN_DIMENSIONS, |kind| {
            matches!(kind, MapObjectKind::TrafficSign { .. })
        })?,
        devices(map, trace, "traffic_lights", LIGHT_DIMENSIONS, |kind| {
            kind.is_traffic_light()
        })?,
    ])
}

fn lanes(map: &ValidatedMap, trace: &mut Trace) -> Result<MapLayer, ExportError> {
    let config = map.metadata.sampling;
    let mut layer = MapLayer::new("lanes");
    for lane in map.lanes.iter().filter(|lane| lane.lane_type.is_drivable()) {
        let travel = lane.travel_geometry(config)?;
        let label = polyline(&vertices(&travel.centerline, config)?, &[]);
        layer.push(trace, lane.id.clone(), label);
    }
    Ok(layer)
}

/// A painted cross-section edge: a lane that has it, its edge index, the curve and
/// what is painted on it.
pub type PaintedEdge<'a> = (&'a Lane, i32, &'a Curve3, BoundaryMarking);

/// Every painted cross-section edge once, as ClipGT draws them: two lanes that meet
/// at an edge share one line rather than stacking two. A kerb is a physical edge,
/// written as a road boundary, not a line.
pub fn painted_edges(map: &ValidatedMap) -> Vec<PaintedEdge<'_>> {
    let mut seen = HashSet::new();
    let mut edges = Vec::new();
    for lane in map.lanes.iter() {
        for (edge, boundary, marking) in [
            (lane.left_edge, &lane.left_boundary, lane.left_marking),
            (lane.right_edge, &lane.right_boundary, lane.right_marking),
        ] {
            if style(marking.marking).is_some()
                && seen.insert((lane.road.clone(), lane.section, edge))
            {
                edges.push((lane, edge, boundary, marking));
            }
        }
    }
    edges
}

fn lanelines(map: &ValidatedMap, trace: &mut Trace) -> Result<MapLayer, ExportError> {
    let config = map.metadata.sampling;
    let mut layer = MapLayer::new("lanelines");
    for (lane, edge, boundary, marking) in painted_edges(map) {
        let section = map.lanes_of_section(&lane.road, lane.section);
        link_edge(trace, &section, edge, &layer.next_local());
        let style = style(marking.marking).expect("only painted edges are listed");
        layer.labels.push(polyline(
            &vertices(boundary, config)?,
            &[
                enumeration("colors", clipgt_color(marking.color)),
                enumeration("styles", style),
            ],
        ));
    }
    Ok(layer)
}

fn road_boundaries(map: &ValidatedMap, trace: &mut Trace) -> Result<MapLayer, ExportError> {
    let config = map.metadata.sampling;
    let mut layer = MapLayer::new("road_boundaries");
    for road in map.roads.iter() {
        for section in 0..road.sections.len() {
            let lanes = map.lanes_of_section(&road.id, section);
            for (edge, boundary) in outer_boundaries(&lanes) {
                link_edge(trace, &lanes, edge, &layer.next_local());
                layer
                    .labels
                    .push(polyline(&vertices(boundary, config)?, &[]));
            }
        }
    }
    Ok(layer)
}

fn wait_lines(map: &ValidatedMap, trace: &mut Trace) -> Result<MapLayer, ExportError> {
    let config = map.metadata.sampling;
    let mut layer = MapLayer::new("wait_lines");
    for object in objects_of(map, |kind| matches!(kind, MapObjectKind::StopLine)) {
        if let ObjectGeometry::Line(line) = &object.geometry {
            layer.push(
                trace,
                object.id.clone(),
                polyline(&vertices(line, config)?, &[]),
            );
        }
    }
    Ok(layer)
}

/// A crosswalk's outline, left edge up and right edge back. The reader closes the
/// ring itself, so the first vertex is not repeated at the end.
fn crosswalks(map: &ValidatedMap, trace: &mut Trace) -> Result<MapLayer, ExportError> {
    let config = map.metadata.sampling;
    let mut layer = MapLayer::new("crosswalks");
    for object in objects_of(map, |kind| matches!(kind, MapObjectKind::Crosswalk)) {
        if let ObjectGeometry::Band { left, right } = &object.geometry {
            let outline = band_polygon(left, right, config)?;
            layer.push(
                trace,
                object.id.clone(),
                json!({
                    "labelData": { "shape3d": { "surface": { "vertices": points(&outline) } } }
                }),
            );
        }
    }
    Ok(layer)
}

/// Lights and signs as the boxes ClipGT gives them: placed by
/// [`device_frames`], with ClipGT's default sizes — the IR has no measurement of a
/// housing, so these are the same boxes a ClipGT clip of the same map holds.
///
/// A cuboid carries its eight corners as `vertices`, the four at the bottom first,
/// in the order the reader's own conversion from centre, orientation and
/// dimensions produces; the centre, orientation and dimensions are written too.
/// A light also carries its IR id as `feature_id`, which is what a per-frame
/// signal state file is matched against.
fn devices(
    map: &ValidatedMap,
    trace: &mut Trace,
    name: &'static str,
    dimensions: Point3,
    wanted: impl Fn(&MapObjectKind) -> bool,
) -> Result<MapLayer, ExportError> {
    let mut layer = MapLayer::new(name);
    for (object, frame) in device_frames(map, wanted)? {
        let mut shape = json!({ "cuboid3d": cuboid(&frame, dimensions) });
        if object.kind.is_traffic_light() {
            shape["attributes"] = json!([{ "name": "feature_id", "text": object.id.as_str() }]);
        }
        layer.push(
            trace,
            object.id.clone(),
            json!({ "labelData": { "shape3d": shape } }),
        );
    }
    Ok(layer)
}

/// A box in `frame`, `dimensions` along its forward, left and up axes.
fn cuboid(frame: &Frame3, dimensions: Point3) -> Value {
    let half = [dimensions.x / 2.0, dimensions.y / 2.0, dimensions.z / 2.0];
    let corners = [
        [-1.0, -1.0, -1.0],
        [-1.0, 1.0, -1.0],
        [1.0, 1.0, -1.0],
        [1.0, -1.0, -1.0],
        [-1.0, -1.0, 1.0],
        [-1.0, 1.0, 1.0],
        [1.0, 1.0, 1.0],
        [1.0, -1.0, 1.0],
    ]
    .map(|[x, y, z]: [f64; 3]| frame.to_global([x * half[0], y * half[1], z * half[2]]));
    // Yaw, pitch and roll of a `ZYX` rotation whose columns are the box's axes,
    // filed under x, y and z the way the reader takes them.
    let (f, l, u) = (frame.tangent.get(), frame.left.get(), frame.up.get());
    let yaw = f.y.atan2(f.x);
    let pitch = (-f.z).clamp(-1.0, 1.0).asin();
    let roll = l.z.atan2(u.z);
    json!({
        "center": { "x": frame.origin.x, "y": frame.origin.y, "z": frame.origin.z },
        "orientation": { "x": yaw, "y": pitch, "z": roll },
        "dimensions": { "x": dimensions.x, "y": dimensions.y, "z": dimensions.z },
        "vertices": points(&corners),
    })
}

fn polyline(points_of: &[Point3], attributes: &[Value]) -> Value {
    let mut shape = json!({ "polyline3d": { "vertices": points(points_of) } });
    if !attributes.is_empty() {
        shape["attributes"] = Value::Array(attributes.to_vec());
    }
    json!({ "labelData": { "shape3d": shape } })
}

/// An attribute holding one value from a fixed vocabulary.
fn enumeration(name: &str, value: &str) -> Value {
    json!({ "name": name, "enumsList": { "enumsList": [value] } })
}

fn points(points: &[Point3]) -> Value {
    Value::Array(
        points
            .iter()
            .map(|point| json!([point.x, point.y, point.z]))
            .collect(),
    )
}

/// The style a line is drawn with, in the vocabulary the Cosmos world-scenario
/// renderer keys its dash patterns on; `None` for an edge with no line on it.
fn style(marking: RoadMarking) -> Option<&'static str> {
    Some(match marking {
        RoadMarking::Solid => "SOLID_SINGLE",
        RoadMarking::Broken => "LONG_DASHED_SINGLE",
        RoadMarking::SolidSolid => "SOLID_GROUP",
        RoadMarking::BrokenSolid => "DASHED_SOLID",
        RoadMarking::SolidBroken => "SOLID_DASHED",
        RoadMarking::None | RoadMarking::Curbstone => return None,
    })
}
