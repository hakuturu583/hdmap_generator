//! Turning the IR into ClipGT's per-layer tables.
//!
//! One function per layer, each returning the rows that layer holds. A layer with no
//! rows is still written: a reader that finds no `crosswalk.parquet` and one that
//! finds an empty one learn the same thing, and an empty file is the less surprising
//! of the two.

use std::collections::{BTreeMap, HashSet};
use std::sync::Arc;

use arrow::array::{Array, ArrayRef, RecordBatch, StructArray};
use arrow::datatypes::{DataType, Field, Fields, Schema};

use roadgen_core::geometry::{Curve3, Frame3, Point3, SamplingConfig, UnitVector3};
use roadgen_core::map::{Lane, Map};
use roadgen_core::semantics::{MapObject, MapObjectKind, ObjectGeometry, RoadMarking};
use roadgen_core::{RoadId, ValidatedMap};

use crate::columns;
use crate::ego;
use crate::error::ExportError;
use crate::{scenario, ClipConfig, Route};

/// A layer's name and its single-batch contents.
pub struct Layer {
    pub name: &'static str,
    pub batch: RecordBatch,
}

/// The size of a traffic light's housing, metres.
///
/// The IR models a light as the bar across the lane it governs — where it applies,
/// not how big the box is — so there is nothing here to derive a housing from. These
/// are ClipGT's own defaults for a device whose dimensions are missing, which makes
/// them the one size a reader is already prepared to see, and keeps this exporter
/// from inventing a measurement the IR never made.
const LIGHT_DIMENSIONS: Point3 = Point3::new(0.6, 0.6, 1.0);
/// The same for a sign's plate.
const SIGN_DIMENSIONS: Point3 = Point3::new(0.8, 0.3, 0.8);

/// Every layer of the clip, in the order they are written.
pub fn all(map: &ValidatedMap, config: &ClipConfig) -> Result<Vec<Layer>, ExportError> {
    Ok(vec![
        calibration(config)?,
        egomotion(map, config)?,
        lanes(map)?,
        lane_lines(map)?,
        road_boundaries(map)?,
        crosswalks(map)?,
        wait_lines(map)?,
        traffic_lights(map)?,
        traffic_signs(map)?,
        intersection_areas(map)?,
    ])
}

// --------------------------------------------------------------------------- //
// Scene layers
// --------------------------------------------------------------------------- //

/// The rig, as the scenario describes it.
///
/// With no sensors configured this is a rig with none — which is what the IR knows
/// about cameras on its own. The table always has its one row either way, because a
/// reader takes the first one without looking.
fn calibration(config: &ClipConfig) -> Result<Layer, ExportError> {
    let rig = columns::strings(&[scenario::rig_json(&config.sensors)]);
    let record = Record::new().add("rig_json", Arc::new(rig));
    layer(
        "calibration_estimate",
        vec![("calibration_estimate", record)],
    )
}

/// Where the ego vehicle is, frame by frame.
fn egomotion(map: &ValidatedMap, config: &ClipConfig) -> Result<Layer, ExportError> {
    let poses = poses(map, config)?;

    let timestamps: Vec<i64> = poses.iter().map(|pose| pose.timestamp_micros).collect();
    let positions: Vec<Point3> = poses.iter().map(|pose| pose.position).collect();
    let orientations: Vec<[f64; 4]> = poses.iter().map(|pose| pose.orientation).collect();

    let key = Record::new().add("timestamp_micros", Arc::new(columns::integers(&timestamps)));
    let motion = Record::new()
        .add("location", Arc::new(columns::points(&positions)))
        .add("orientation", Arc::new(columns::quaternions(&orientations)));
    layer(
        "egomotion_estimate",
        vec![("key", key), ("egomotion_estimate", motion)],
    )
}

/// The ego track the clip is built around.
///
/// Public to the crate because the camera timestamps file has to be the same frames,
/// not a second set that happens to look alike.
pub fn poses(map: &ValidatedMap, config: &ClipConfig) -> Result<Vec<ego::Pose>, ExportError> {
    let route = match &config.route {
        Some(Route::Lanes(lanes)) => lanes.clone(),
        Some(Route::From(start)) => {
            if map.lane(start).is_none() {
                return Err(ExportError::NoRoute(format!(
                    "{start} is not a lane of this map"
                )));
            }
            map.route_from(start)
        }
        None => {
            let start = map.default_start().ok_or_else(|| {
                ExportError::NoRoute("the map has no drivable lane outside a junction".into())
            })?;
            map.route_from(&start)
        }
    };
    ego::track(map, &route, config.speed, config.frame_rate)
}

// --------------------------------------------------------------------------- //
// Map layers
// --------------------------------------------------------------------------- //

/// A lane is its two rails, in travel order: `left_rail` is the driver's left.
fn lanes(map: &ValidatedMap) -> Result<Layer, ExportError> {
    let config = map.metadata.sampling;
    let mut left = Vec::new();
    let mut right = Vec::new();
    for lane in map.lanes.iter().filter(|lane| lane.lane_type.is_drivable()) {
        let travel = lane.travel_geometry(config)?;
        left.push(vertices(&travel.left, config)?);
        right.push(vertices(&travel.right, config)?);
    }
    let record = Record::new()
        .add("left_rail", Arc::new(columns::point_lists(&left)))
        .add("right_rail", Arc::new(columns::point_lists(&right)));
    layer("lane", vec![("lane", record)])
}

/// Painted boundaries, one row per cross-section edge rather than per lane: two lanes
/// that meet at an edge are separated by one line, not two stacked on each other.
fn lane_lines(map: &ValidatedMap) -> Result<Layer, ExportError> {
    let config = map.metadata.sampling;
    let mut seen: HashSet<(RoadId, usize, i32)> = HashSet::new();
    let mut rails = Vec::new();
    let mut colors = Vec::new();
    let mut styles = Vec::new();

    for lane in map.lanes.iter() {
        for (edge, boundary, marking) in [
            (lane.left_edge, &lane.left_boundary, lane.left_marking),
            (lane.right_edge, &lane.right_boundary, lane.right_marking),
        ] {
            // Nothing is painted on an unmarked edge, and a kerb is a physical edge
            // rather than a line — it leaves through `road_boundary` instead.
            if matches!(marking.marking, RoadMarking::None | RoadMarking::Curbstone) {
                continue;
            }
            if !seen.insert((lane.road.clone(), lane.section, edge)) {
                continue;
            }
            rails.push(vertices(boundary, config)?);
            colors.push(vec![clipgt_color(marking.color).to_owned()]);
            styles.push(vec![clipgt_style(marking.marking).to_owned()]);
        }
    }

    let record = Record::new()
        .add("line_rail", Arc::new(columns::point_lists(&rails)))
        .add("colors", Arc::new(columns::string_lists(&colors)))
        .add("styles", Arc::new(columns::string_lists(&styles)));
    layer("lane_line", vec![("lane_line", record)])
}

/// The outer edge of the carriageway: for each cross-section, the boundary beyond the
/// outermost lane on each side, whether or not anything is painted on it.
fn road_boundaries(map: &ValidatedMap) -> Result<Layer, ExportError> {
    let config = map.metadata.sampling;
    let mut rows = Vec::new();
    for road in map.roads.iter() {
        for section in 0..road.sections.len() {
            let lanes = map.lanes_of_section(&road.id, section);
            for boundary in outer_boundaries(&lanes) {
                rows.push(vertices(boundary, config)?);
            }
        }
    }
    let record = Record::new().add("location", Arc::new(columns::point_lists(&rows)));
    layer("road_boundary", vec![("road_boundary", record)])
}

fn crosswalks(map: &ValidatedMap) -> Result<Layer, ExportError> {
    let config = map.metadata.sampling;
    let mut rows = Vec::new();
    for object in objects_of(map, |kind| matches!(kind, MapObjectKind::Crosswalk)) {
        if let ObjectGeometry::Band { left, right } = &object.geometry {
            rows.push(band_polygon(left, right, config)?);
        }
    }
    let record = Record::new().add("location", Arc::new(columns::point_lists(&rows)));
    layer("crosswalk", vec![("crosswalk", record)])
}

fn wait_lines(map: &ValidatedMap) -> Result<Layer, ExportError> {
    let config = map.metadata.sampling;
    let mut rows = Vec::new();
    for object in objects_of(map, |kind| matches!(kind, MapObjectKind::StopLine)) {
        if let ObjectGeometry::Line(line) = &object.geometry {
            rows.push(vertices(line, config)?);
        }
    }
    let record = Record::new().add("location", Arc::new(columns::point_lists(&rows)));
    layer("wait_line", vec![("wait_line", record)])
}

fn traffic_lights(map: &ValidatedMap) -> Result<Layer, ExportError> {
    let devices = devices(
        map,
        |kind| matches!(kind, MapObjectKind::TrafficLight),
        LIGHT_DIMENSIONS,
    )?;
    let record = Record::new()
        .add("center", Arc::new(columns::points(&devices.centers)))
        .add("dimensions", Arc::new(columns::points(&devices.dimensions)))
        .add(
            "orientation",
            Arc::new(columns::quaternions(&devices.orientations)),
        );
    layer("traffic_light", vec![("traffic_light", record)])
}

fn traffic_signs(map: &ValidatedMap) -> Result<Layer, ExportError> {
    let devices = devices(
        map,
        |kind| matches!(kind, MapObjectKind::TrafficSign { .. }),
        SIGN_DIMENSIONS,
    )?;
    let record = Record::new()
        .add("center", Arc::new(columns::points(&devices.centers)))
        .add("dimensions", Arc::new(columns::points(&devices.dimensions)))
        .add(
            "orientation",
            Arc::new(columns::quaternions(&devices.orientations)),
        )
        .add("category", Arc::new(columns::strings(&devices.categories)));
    layer("traffic_sign", vec![("traffic_sign", record)])
}

/// A junction as an outline: the hull of everything its connectors cover.
fn intersection_areas(map: &ValidatedMap) -> Result<Layer, ExportError> {
    let config = map.metadata.sampling;
    let mut rows = Vec::new();
    for junction in map.junctions.iter() {
        let mut points = Vec::new();
        for road in &junction.connecting_roads {
            for lane in map.lanes_of(road) {
                points.extend(vertices(&lane.left_boundary, config)?);
                points.extend(vertices(&lane.right_boundary, config)?);
            }
        }
        let hull = convex_hull(&points);
        if hull.len() >= 3 {
            rows.push(hull);
        }
    }
    let record = Record::new().add("location", Arc::new(columns::point_lists(&rows)));
    layer("intersection_area", vec![("intersection_area", record)])
}

// --------------------------------------------------------------------------- //
// Shared pieces
// --------------------------------------------------------------------------- //

/// A light or a sign, reduced to the box ClipGT describes it with.
struct Devices {
    centers: Vec<Point3>,
    dimensions: Vec<Point3>,
    orientations: Vec<[f64; 4]>,
    categories: Vec<String>,
}

fn devices(
    map: &ValidatedMap,
    wanted: impl Fn(&MapObjectKind) -> bool,
    dimensions: Point3,
) -> Result<Devices, ExportError> {
    let config = map.metadata.sampling;
    let mut devices = Devices {
        centers: Vec::new(),
        dimensions: Vec::new(),
        orientations: Vec::new(),
        categories: Vec::new(),
    };
    for object in objects_of(map, &wanted) {
        let ObjectGeometry::Line(bar) = &object.geometry else {
            continue;
        };
        let (from, to) = (bar.start_point(), bar.end_point());
        let center = from.lerp(to, 0.5);
        let facing = facing_of(map, object, center, config)?;

        devices.centers.push(center);
        devices.dimensions.push(dimensions);
        devices
            .orientations
            .push(ego::orientation_of(&Frame3::from_tangent(center, facing)?));
        devices.categories.push(match &object.kind {
            MapObjectKind::TrafficSign { code } => code.clone(),
            other => other.as_str().to_owned(),
        });
    }
    Ok(devices)
}

/// Which way a device faces: back along the traffic it governs, so it looks at the
/// drivers who have to read it.
fn facing_of(
    map: &Map,
    object: &MapObject,
    center: Point3,
    config: SamplingConfig,
) -> Result<UnitVector3, ExportError> {
    if let Some(lane) = object.lanes.first().and_then(|id| map.lane(id)) {
        let travel = lane.travel_geometry(config)?;
        let line = &travel.centerline;
        let tangent =
            if center.distance_to(line.start_point()) <= center.distance_to(line.end_point()) {
                line.start_tangent()?
            } else {
                line.end_tangent()?
            };
        return Ok(tangent.reversed());
    }
    // With no lane to face, the bar's own perpendicular is all there is: turn it a
    // quarter turn in the horizontal plane.
    let along = object_bar_direction(object)?;
    UnitVector3::try_new(roadgen_core::geometry::Vector3::new(
        along.get().y,
        -along.get().x,
        0.0,
    ))
    .map_err(ExportError::from)
}

fn object_bar_direction(object: &MapObject) -> Result<UnitVector3, ExportError> {
    match &object.geometry {
        ObjectGeometry::Line(line) => Ok(line.start_tangent()?),
        _ => Err(ExportError::Schema(format!(
            "{} is not a line, so it has no direction across the road",
            object.id
        ))),
    }
}

fn objects_of<'a>(
    map: &'a ValidatedMap,
    wanted: impl Fn(&MapObjectKind) -> bool + 'a,
) -> impl Iterator<Item = &'a MapObject> {
    map.objects
        .iter()
        .filter(move |object| wanted(&object.kind))
}

/// The two edges of a cross-section: the leftmost and the rightmost.
///
/// Found through the edge indices rather than by looking at each side's lanes,
/// because a cross-section need not have lanes on both sides — a junction connector
/// has one lane, and both of its boundaries are edges of the carriageway.
fn outer_boundaries<'a>(lanes: &[&'a Lane]) -> Vec<&'a Curve3> {
    let leftmost = lanes
        .iter()
        .max_by_key(|lane| lane.left_edge)
        .map(|lane| &lane.left_boundary);
    let rightmost = lanes
        .iter()
        .min_by_key(|lane| lane.right_edge)
        .map(|lane| &lane.right_boundary);
    leftmost.into_iter().chain(rightmost).collect()
}

fn vertices(curve: &Curve3, config: SamplingConfig) -> Result<Vec<Point3>, ExportError> {
    Ok(curve.to_polyline(config)?.points().to_vec())
}

/// A strip's outline: up one edge and back down the other.
fn band_polygon(
    left: &Curve3,
    right: &Curve3,
    config: SamplingConfig,
) -> Result<Vec<Point3>, ExportError> {
    let mut polygon = vertices(left, config)?;
    let mut back = vertices(right, config)?;
    back.reverse();
    polygon.extend(back);
    Ok(polygon)
}

/// The convex hull of the points' shadows, with each kept vertex's own height.
///
/// Andrew's monotone chain. The hull is computed in plan view because that is what an
/// intersection's outline means, but the vertices that come out are the real 3D
/// points — an outline over a graded junction is not flat, and flattening it here
/// would be the one place this exporter quietly dropped a dimension.
fn convex_hull(points: &[Point3]) -> Vec<Point3> {
    if points.len() < 3 {
        return points.to_vec();
    }
    let mut sorted: Vec<Point3> = points.to_vec();
    sorted.sort_by(|a, b| {
        a.x.partial_cmp(&b.x)
            .unwrap_or(std::cmp::Ordering::Equal)
            .then(a.y.partial_cmp(&b.y).unwrap_or(std::cmp::Ordering::Equal))
    });
    sorted.dedup_by(|a, b| (a.x - b.x).abs() < 1e-9 && (a.y - b.y).abs() < 1e-9);
    if sorted.len() < 3 {
        return sorted;
    }

    let cross =
        |o: Point3, a: Point3, b: Point3| (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);
    let half = |source: &mut dyn Iterator<Item = Point3>| -> Vec<Point3> {
        let mut chain: Vec<Point3> = Vec::new();
        for point in source {
            while chain.len() >= 2
                && cross(chain[chain.len() - 2], chain[chain.len() - 1], point) <= 0.0
            {
                chain.pop();
            }
            chain.push(point);
        }
        chain.pop();
        chain
    };

    let mut hull = half(&mut sorted.iter().copied());
    hull.extend(half(&mut sorted.iter().rev().copied()));
    hull
}

fn clipgt_color(color: roadgen_core::semantics::MarkingColor) -> &'static str {
    match color {
        roadgen_core::semantics::MarkingColor::White => "WHITE",
        roadgen_core::semantics::MarkingColor::Yellow => "YELLOW",
    }
}

fn clipgt_style(marking: RoadMarking) -> &'static str {
    match marking {
        RoadMarking::Solid => "SOLID_SINGLE",
        RoadMarking::Broken => "DASHED_SINGLE",
        RoadMarking::SolidSolid => "SOLID_DOUBLE",
        RoadMarking::BrokenSolid => "DASHED_SOLID",
        RoadMarking::SolidBroken => "SOLID_DASHED",
        // Neither of these reaches here: both are filtered out before the row is
        // built, because ClipGT's vocabulary has no word for "no line".
        RoadMarking::None | RoadMarking::Curbstone => "UNKNOWN",
    }
}

// --------------------------------------------------------------------------- //
// Arrow plumbing
// --------------------------------------------------------------------------- //

/// A struct column under construction: named fields, in the order they are added.
struct Record {
    fields: Vec<Field>,
    arrays: Vec<ArrayRef>,
}

impl Record {
    fn new() -> Self {
        Record {
            fields: Vec::new(),
            arrays: Vec::new(),
        }
    }

    fn add(mut self, name: &str, array: ArrayRef) -> Self {
        self.fields
            .push(Field::new(name, array.data_type().clone(), false));
        self.arrays.push(array);
        self
    }

    fn rows(&self) -> usize {
        self.arrays.first().map(|array| array.len()).unwrap_or(0)
    }

    fn finish(self) -> StructArray {
        StructArray::new(Fields::from(self.fields), self.arrays, None)
    }
}

/// Assembles a layer from its top-level columns, checking they agree on length.
fn layer(name: &'static str, records: Vec<(&str, Record)>) -> Result<Layer, ExportError> {
    let rows: BTreeMap<&str, usize> = records
        .iter()
        .map(|(column, record)| (*column, record.rows()))
        .collect();
    if rows.values().collect::<HashSet<_>>().len() > 1 {
        return Err(ExportError::Schema(format!(
            "{name}'s columns disagree on how many rows it has: {rows:?}"
        )));
    }
    let row_count = rows.values().copied().next().unwrap_or(0);

    let mut fields = Vec::new();
    let mut arrays: Vec<ArrayRef> = Vec::new();
    for (column, record) in records {
        let array = record.finish();
        fields.push(Field::new(
            column,
            DataType::Struct(array.fields().clone()),
            false,
        ));
        arrays.push(Arc::new(array));
    }

    let schema = Arc::new(Schema::new(fields));
    let batch = if row_count == 0 {
        RecordBatch::new_empty(schema)
    } else {
        RecordBatch::try_new(schema, arrays)
            .map_err(|error| ExportError::Schema(format!("{name}: {error}")))?
    };
    Ok(Layer { name, batch })
}
