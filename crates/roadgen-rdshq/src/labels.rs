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

use roadgen_clipgt::layers::{
    band_polygon, facing_of, link_edge, outer_boundaries, vertices, LIGHT_DIMENSIONS,
    SIGN_DIMENSIONS,
};
use roadgen_core::geometry::{Frame3, Point3};
use roadgen_core::semantics::{MapObjectKind, MarkingColor, ObjectGeometry, RoadMarking};
use roadgen_core::{Relation, RoadId, Trace, ValidatedMap};
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
    /// The document a reader decodes: `{"labels": [...]}`.
    pub fn document(&self) -> Value {
        json!({ "labels": self.labels })
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
        MapLayer {
            name: "poles",
            labels: Vec::new(),
        },
        crosswalks(map, trace)?,
        MapLayer {
            name: "road_markings",
            labels: Vec::new(),
        },
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
    let mut labels = Vec::new();
    for lane in map.lanes.iter().filter(|lane| lane.lane_type.is_drivable()) {
        let travel = lane.travel_geometry(config)?;
        trace.link(
            lane.id.clone(),
            local("lanes", labels.len()),
            Relation::Exact,
        );
        labels.push(polyline(&vertices(&travel.centerline, config)?, Vec::new()));
    }
    Ok(MapLayer {
        name: "lanes",
        labels,
    })
}

/// One label per painted cross-section edge, as ClipGT draws them: two lanes that
/// meet at an edge share one line rather than stacking two.
fn lanelines(map: &ValidatedMap, trace: &mut Trace) -> Result<MapLayer, ExportError> {
    let config = map.metadata.sampling;
    let mut seen: std::collections::HashSet<(RoadId, usize, i32)> = Default::default();
    let mut labels = Vec::new();
    for lane in map.lanes.iter() {
        for (edge, boundary, marking) in [
            (lane.left_edge, &lane.left_boundary, lane.left_marking),
            (lane.right_edge, &lane.right_boundary, lane.right_marking),
        ] {
            // A kerb is a physical edge, written as a road boundary, not a line.
            let Some(style) = style(marking.marking) else {
                continue;
            };
            if !seen.insert((lane.road.clone(), lane.section, edge)) {
                continue;
            }
            let section = map.lanes_of_section(&lane.road, lane.section);
            link_edge(trace, &section, edge, &local("lanelines", labels.len()));
            labels.push(polyline(
                &vertices(boundary, config)?,
                vec![
                    enumeration("colors", color(marking.color)),
                    enumeration("styles", style),
                ],
            ));
        }
    }
    Ok(MapLayer {
        name: "lanelines",
        labels,
    })
}

fn road_boundaries(map: &ValidatedMap, trace: &mut Trace) -> Result<MapLayer, ExportError> {
    let config = map.metadata.sampling;
    let mut labels = Vec::new();
    for road in map.roads.iter() {
        for section in 0..road.sections.len() {
            let lanes = map.lanes_of_section(&road.id, section);
            for (edge, boundary) in outer_boundaries(&lanes) {
                link_edge(trace, &lanes, edge, &local("road_boundaries", labels.len()));
                labels.push(polyline(&vertices(boundary, config)?, Vec::new()));
            }
        }
    }
    Ok(MapLayer {
        name: "road_boundaries",
        labels,
    })
}

fn wait_lines(map: &ValidatedMap, trace: &mut Trace) -> Result<MapLayer, ExportError> {
    let config = map.metadata.sampling;
    let mut labels = Vec::new();
    for object in map.objects.iter() {
        if let (MapObjectKind::StopLine, ObjectGeometry::Line(line)) =
            (&object.kind, &object.geometry)
        {
            trace.link(
                object.id.clone(),
                local("wait_lines", labels.len()),
                Relation::Exact,
            );
            labels.push(polyline(&vertices(line, config)?, Vec::new()));
        }
    }
    Ok(MapLayer {
        name: "wait_lines",
        labels,
    })
}

/// A crosswalk's outline, left edge up and right edge back. The reader closes the
/// ring itself, so the first vertex is not repeated at the end.
fn crosswalks(map: &ValidatedMap, trace: &mut Trace) -> Result<MapLayer, ExportError> {
    let config = map.metadata.sampling;
    let mut labels = Vec::new();
    for object in map.objects.iter() {
        if let (MapObjectKind::Crosswalk, ObjectGeometry::Band { left, right }) =
            (&object.kind, &object.geometry)
        {
            trace.link(
                object.id.clone(),
                local("crosswalks", labels.len()),
                Relation::Exact,
            );
            let outline = band_polygon(left, right, config)?;
            labels.push(json!({
                "labelData": { "shape3d": { "surface": { "vertices": points(&outline) } } }
            }));
        }
    }
    Ok(MapLayer {
        name: "crosswalks",
        labels,
    })
}

/// Lights and signs as boxes: centred on the middle of the bar the IR draws across
/// the lanes, facing the traffic they govern, with ClipGT's default sizes — the IR
/// has no measurement of a housing, so these are the same boxes a ClipGT clip of
/// the same map holds.
///
/// A cuboid carries its eight corners as `vertices`, the four at the bottom first,
/// in the order the reader's own conversion from centre, orientation and
/// dimensions produces; the centre, orientation and dimensions are written too.
fn devices(
    map: &ValidatedMap,
    trace: &mut Trace,
    name: &'static str,
    dimensions: Point3,
    wanted: impl Fn(&MapObjectKind) -> bool,
) -> Result<MapLayer, ExportError> {
    let config = map.metadata.sampling;
    let mut labels = Vec::new();
    for object in map.objects.iter().filter(|object| wanted(&object.kind)) {
        let ObjectGeometry::Line(bar) = &object.geometry else {
            continue;
        };
        let center = bar.start_point().lerp(bar.end_point(), 0.5);
        let facing = facing_of(map, object, center, config)?;
        let frame = Frame3::from_tangent(center, facing)?;

        trace.link(
            object.id.clone(),
            local(name, labels.len()),
            Relation::Exact,
        );
        let mut shape = json!({ "cuboid3d": cuboid(&frame, dimensions) });
        if object.kind.is_traffic_light() {
            // What a per-frame signal state file is matched against, by id.
            shape["attributes"] = json!([{ "name": "feature_id", "text": object.id.as_str() }]);
        }
        labels.push(json!({ "labelData": { "shape3d": shape } }));
    }
    Ok(MapLayer { name, labels })
}

/// A box in `frame`, `dimensions` along its forward, left and up axes.
fn cuboid(frame: &Frame3, dimensions: Point3) -> Value {
    let (f, l, u) = (frame.tangent.get(), frame.left.get(), frame.up.get());
    let half = [dimensions.x / 2.0, dimensions.y / 2.0, dimensions.z / 2.0];
    let corner = |sx: f64, sy: f64, sz: f64| -> Point3 {
        frame.origin + f * (sx * half[0]) + l * (sy * half[1]) + u * (sz * half[2])
    };
    let corners = [
        corner(-1.0, -1.0, -1.0),
        corner(-1.0, 1.0, -1.0),
        corner(1.0, 1.0, -1.0),
        corner(1.0, -1.0, -1.0),
        corner(-1.0, -1.0, 1.0),
        corner(-1.0, 1.0, 1.0),
        corner(1.0, 1.0, 1.0),
        corner(1.0, -1.0, 1.0),
    ];
    // Yaw, pitch and roll of a `ZYX` rotation whose columns are the box's axes,
    // filed under x, y and z the way the reader takes them.
    let yaw = f.y.atan2(f.x);
    let pitch = (-f.z).clamp(-1.0, 1.0).asin();
    let roll = l.z.atan2(u.z);
    json!({
        "center": xyz(frame.origin),
        "orientation": { "x": yaw, "y": pitch, "z": roll },
        "dimensions": { "x": dimensions.x, "y": dimensions.y, "z": dimensions.z },
        "vertices": points(&corners),
    })
}

fn polyline(points_of: &[Point3], attributes: Vec<Value>) -> Value {
    let mut shape = json!({ "polyline3d": { "vertices": points(points_of) } });
    if !attributes.is_empty() {
        shape["attributes"] = Value::Array(attributes);
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

fn xyz(point: Point3) -> Value {
    json!({ "x": point.x, "y": point.y, "z": point.z })
}

fn local(layer: &str, index: usize) -> String {
    format!("{layer}:{index}")
}

fn color(color: MarkingColor) -> &'static str {
    match color {
        MarkingColor::White => "WHITE",
        MarkingColor::Yellow => "YELLOW",
    }
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
