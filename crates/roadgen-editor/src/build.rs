//! From a [`Document`] to the canonical IR.
//!
//! An editor rebuilds on every change, and most of the time in the middle of an
//! edit the document is wrong somewhere: a link whose ends have been dragged apart,
//! a lane list that pairs with nothing. So building never gives up on the whole
//! document because one part of it is wrong. A road that cannot be built is left
//! out; a link that cannot be made is left out; and if the generator still refuses,
//! the roads are drawn without their links. Each of those is a [`Problem`] the
//! editor can point at.

use std::collections::{BTreeSet, HashMap};

use roadgen_core::{
    BoundaryMarking, BuildError, Curve3, Direction, GeoOrigin, LaneSpec, LaneType, LateralSide,
    MapBuilder, MapMetadata, MarkingColor, Point3, PositiveWidth, RoadEnd, RoadId, RoadMarking,
    RoadSpec, RoadType, SamplingConfig, SpeedLimit, TrafficHandedness, UnvalidatedMap,
    ValidatedMap, ValidationConfig, Vector3,
};

use crate::document::{
    Document, End, Handedness, LaneDoc, LinkDoc, Marking, RoadDoc, Shape, Side, Travel,
};

/// How bad a problem is.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize))]
#[cfg_attr(feature = "serde", serde(rename_all = "snake_case"))]
pub enum Severity {
    /// Part of the document could not be built at all and is missing from the map.
    Error,
    /// The map was built but does not validate, so no format can be written from it.
    Invalid,
}

/// Something wrong with a document, and what it is about.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize))]
pub struct Problem {
    pub severity: Severity,
    /// The road the problem is about, by the document's id.
    pub road: Option<String>,
    /// The link the problem is about, by its position in the document.
    pub link: Option<usize>,
    pub message: String,
}

impl Problem {
    fn error(message: impl Into<String>) -> Self {
        Problem {
            severity: Severity::Error,
            road: None,
            link: None,
            message: message.into(),
        }
    }

    fn on_road(mut self, road: &str) -> Self {
        self.road = Some(road.to_owned());
        self
    }

    fn on_link(mut self, link: usize) -> Self {
        self.link = Some(link);
        self
    }
}

/// What came of building a document.
#[derive(Debug, Clone)]
pub struct Built {
    /// The map, as far as it could be built; `None` only when nothing could be.
    pub map: Option<UnvalidatedMap>,
    /// Everything that was left out or is wrong, in document order.
    pub problems: Vec<Problem>,
}

impl Built {
    /// Whether the map is the whole document and validates, so that every format
    /// can be written from it.
    pub fn is_valid(&self) -> bool {
        self.map.is_some() && self.problems.is_empty()
    }

    /// The validated map — only when it is the whole document. A map with part of
    /// the document left out would validate, and be written out, as something the
    /// user never drew.
    pub fn validated(&self) -> Result<ValidatedMap, String> {
        if let Some(problem) = self
            .problems
            .iter()
            .find(|problem| problem.severity == Severity::Error)
        {
            return Err(format!(
                "part of the document was left out: {}",
                problem.message
            ));
        }
        let map = self
            .map
            .clone()
            .ok_or_else(|| "nothing could be built from the document".to_owned())?;
        map.validate().map_err(|error| error.to_string())
    }
}

/// Builds the document into the IR, leaving out whatever cannot be built.
pub fn build(document: &Document) -> Built {
    let mut problems = Vec::new();
    let metadata = match metadata(document) {
        Ok(metadata) => metadata,
        Err(problem) => {
            problems.push(problem);
            metadata_without_origin(document)
        }
    };

    let joints = joint_tangents(document);
    let mut roads: Vec<(String, RoadSpec)> = Vec::new();
    let mut seen = BTreeSet::new();
    for road in &document.roads {
        if !seen.insert(road.id.as_str()) {
            problems.push(
                Problem::error(format!("two roads are called '{}'", road.id)).on_road(&road.id),
            );
            continue;
        }
        let ends = (
            joints.get(&(road.id.clone(), End::Start)).copied(),
            joints.get(&(road.id.clone(), End::End)).copied(),
        );
        match road_spec_with(road, ends) {
            Ok(spec) => roads.push((road.id.clone(), spec)),
            Err(message) => problems.push(Problem::error(message).on_road(&road.id)),
        }
    }

    let every_link: Vec<usize> = (0..document.links.len()).collect();
    let (result, accepted) = assemble(
        document,
        &metadata,
        &roads,
        &every_link,
        Some(&mut problems),
    );
    let map = match result {
        Ok(map) => Some(map),
        Err(error) => {
            // The generator refused the map as a whole. Put the links back one at a
            // time and keep each that the generator accepts, so the refusal lands
            // on the link that caused it and the rest of the map is still drawn.
            let mut kept: Vec<usize> = Vec::new();
            let mut blamed = false;
            for &index in &accepted {
                let mut trial = kept.clone();
                trial.push(index);
                match assemble(document, &metadata, &roads, &trial, None).0 {
                    Ok(_) => kept = trial,
                    Err(error) => {
                        blamed = true;
                        problems.push(Problem::error(error.to_string()).on_link(index));
                    }
                }
            }
            if !blamed {
                problems.push(Problem::error(format!(
                    "the map could not be generated: {error}"
                )));
            }
            assemble(document, &metadata, &roads, &kept, None).0.ok()
        }
    };

    if let Some(map) = &map {
        for issue in map.issues(ValidationConfig::default()) {
            problems.push(Problem {
                severity: Severity::Invalid,
                road: None,
                link: None,
                message: issue.to_string(),
            });
        }
    }
    Built { map, problems }
}

/// One pass of the builder over the roads and the given links. Returns the map, or
/// why the generator refused it, with the links that were accepted.
fn assemble(
    document: &Document,
    metadata: &MapMetadata,
    roads: &[(String, RoadSpec)],
    links: &[usize],
    mut problems: Option<&mut Vec<Problem>>,
) -> (Result<UnvalidatedMap, BuildError>, Vec<usize>) {
    let mut report = |problem: Problem| {
        if let Some(problems) = problems.as_deref_mut() {
            problems.push(problem);
        }
    };
    let mut builder = MapBuilder::new(metadata.clone());
    let mut built: BTreeSet<&str> = BTreeSet::new();
    for (id, spec) in roads {
        match builder.add_road(spec.clone()) {
            Ok(_) => {
                built.insert(id.as_str());
            }
            Err(error) => report(Problem::error(error.to_string()).on_road(id)),
        }
    }

    let mut accepted = Vec::new();
    for &index in links {
        let link = &document.links[index];
        if let Err(message) = check_link(link, &built) {
            report(Problem::error(message).on_link(index));
            continue;
        }
        let junction = link
            .junction
            .as_deref()
            .filter(|name| !name.trim().is_empty())
            .map(|name| builder.add_junction(Some(name)));
        match builder.connect_ends(
            &RoadId::new(&link.from),
            road_end(link.from_end),
            &RoadId::new(&link.to),
            road_end(link.to_end),
            junction.as_ref(),
        ) {
            Ok(_) => accepted.push(index),
            Err(error) => report(Problem::error(error.to_string()).on_link(index)),
        }
    }
    (builder.finish(), accepted)
}

fn check_link(link: &LinkDoc, built: &BTreeSet<&str>) -> Result<(), String> {
    for road in [&link.from, &link.to] {
        if !built.contains(road.as_str()) {
            return Err(format!("links road '{road}', which is not on the map"));
        }
    }
    if link.from == link.to && link.from_end == link.to_end {
        return Err(format!("links an end of '{}' to itself", link.from));
    }
    Ok(())
}

fn metadata_without_origin(document: &Document) -> MapMetadata {
    MapMetadata {
        name: Some(document.name.clone()).filter(|name| !name.is_empty()),
        handedness: match document.handedness {
            Handedness::Right => TrafficHandedness::RightHand,
            Handedness::Left => TrafficHandedness::LeftHand,
        },
        sampling: SamplingConfig::default(),
        ..MapMetadata::default()
    }
}

fn metadata(document: &Document) -> Result<MapMetadata, Problem> {
    let mut metadata = metadata_without_origin(document);
    if let Some([latitude, longitude, altitude]) = document.origin {
        metadata.origin = GeoOrigin::new(latitude, longitude, altitude)
            .map_err(|error| Problem::error(format!("origin: {error}")))?;
    }
    Ok(metadata)
}

fn road_end(end: End) -> RoadEnd {
    match end {
        End::Start => RoadEnd::Start,
        End::End => RoadEnd::End,
    }
}

/// The builder's description of one road.
pub fn road_spec(road: &RoadDoc) -> Result<RoadSpec, String> {
    road_spec_with(road, (None, None))
}

/// [`road_spec`], with the direction the reference line must have at its start
/// and at its end where a joint asks for one.
fn road_spec_with(road: &RoadDoc, ends: Tangents) -> Result<RoadSpec, String> {
    if road.id.trim().is_empty() {
        return Err("a road needs an id".to_owned());
    }
    if road.lanes.is_empty() {
        return Err(format!("road '{}' has no lanes", road.id));
    }
    let lanes = road
        .lanes
        .iter()
        .enumerate()
        .map(|(index, lane)| lane_spec(lane).map_err(|e| format!("lane {index}: {e}")))
        .collect::<Result<Vec<_>, _>>()?;
    let mut spec = RoadSpec::new(reference_line_with(road, ends)?, lanes).with_name(&road.id);
    spec = spec.with_type(
        RoadType::parse(&road.road_type)
            .ok_or_else(|| format!("'{}' is not a road type", road.road_type))?,
    );
    if let Some(limit) = road.speed_limit {
        spec = spec.with_speed_limit(SpeedLimit::from_kph(limit).map_err(|e| e.to_string())?);
    }
    Ok(spec)
}

fn lane_spec(lane: &LaneDoc) -> Result<LaneSpec, String> {
    let width = PositiveWidth::new(lane.width).map_err(|e| e.to_string())?;
    let direction = match lane.direction {
        Travel::Forward => Direction::Forward,
        Travel::Backward => Direction::Backward,
    };
    let lane_type = LaneType::parse(&lane.lane_type)
        .ok_or_else(|| format!("'{}' is not a lane type", lane.lane_type))?;
    let mut spec = LaneSpec::new(width, direction)
        .with_type(lane_type)
        .with_markings(marking(&lane.left_marking)?, marking(&lane.right_marking)?);
    if let Some(side) = lane.side {
        spec = spec.with_side(match side {
            Side::Left => LateralSide::Left,
            Side::Right => LateralSide::Right,
        });
    }
    Ok(spec)
}

fn marking(marking: &Marking) -> Result<BoundaryMarking, String> {
    let kind = RoadMarking::parse(&marking.kind)
        .ok_or_else(|| format!("'{}' is not a road marking", marking.kind))?;
    let color = match marking.color.to_ascii_lowercase().as_str() {
        "white" | "" => MarkingColor::White,
        "yellow" => MarkingColor::Yellow,
        other => return Err(format!("'{other}' is not a marking colour")),
    };
    Ok(BoundaryMarking::new(kind, color))
}

/// Directions a reference line is held to at its start and at its end, along
/// increasing station.
type Tangents = (Option<Vector3>, Option<Vector3>);

/// The reference line through a road's control points.
pub fn reference_line(road: &RoadDoc) -> Result<Curve3, String> {
    reference_line_with(road, (None, None))
}

/// The control points with the doubled ones dropped: a point dropped on top of the
/// last one is a double click, not a shape.
fn control_points(road: &RoadDoc) -> Result<Vec<Point3>, String> {
    let mut points: Vec<Point3> = Vec::with_capacity(road.points.len());
    for &[x, y, z] in &road.points {
        if !(x.is_finite() && y.is_finite() && z.is_finite()) {
            return Err(format!(
                "road '{}' has a point that is not a number",
                road.id
            ));
        }
        let point = Point3::new(x, y, z);
        if points
            .last()
            .is_none_or(|last| last.horizontal_distance_to(point) > 1e-3)
        {
            points.push(point);
        }
    }
    if points.len() < 2 {
        return Err(format!(
            "road '{}' needs two points at least a millimetre apart",
            road.id
        ));
    }
    Ok(points)
}

fn reference_line_with(road: &RoadDoc, ends: Tangents) -> Result<Curve3, String> {
    let points = control_points(road)?;
    let held = ends.0.is_some() || ends.1.is_some();
    let curve = match (road.shape, points.len()) {
        (Shape::Polyline, 2) => Curve3::line(points[0], points[1]),
        (Shape::Smooth, 2) if !held => Curve3::line(points[0], points[1]),
        (Shape::Polyline, _) => Curve3::polyline(points),
        (Shape::Smooth, _) => smooth(&points, ends),
    };
    curve.map_err(|error| format!("road '{}': {error}", road.id))
}

/// The natural direction of a smooth curve through `points` at control point `i`:
/// the chord between its neighbours, as Catmull-Rom has it.
fn chord_at(points: &[Point3], i: usize) -> Vector3 {
    let last = points.len() - 1;
    points[(i + 1).min(last)] - points[i.saturating_sub(1)]
}

/// Cubic Bézier pieces through every point, each meeting the next with the same
/// tangent; the two ends held to `ends` where given.
fn smooth(points: &[Point3], ends: Tangents) -> Result<Curve3, roadgen_core::GeometryError> {
    let last = points.len() - 1;
    let tangents = (0..=last)
        .map(|i| {
            let held = match i {
                0 => ends.0,
                i if i == last => ends.1,
                _ => None,
            };
            held.unwrap_or_else(|| chord_at(points, i)).normalize()
        })
        .collect::<Result<Vec<_>, _>>()?;
    let pieces = (0..last)
        .map(|i| {
            roadgen_core::geometry::Bezier3::hermite(
                points[i],
                tangents[i],
                points[i + 1],
                tangents[i + 1],
                SamplingConfig::default(),
            )
            .map(Curve3::Bezier)
        })
        .collect::<Result<Vec<_>, _>>()?;
    Curve3::composite(pieces)
}

/// One tangent for both roads at every direct joint between two smooth roads whose
/// ends meet, so that the joint has no corner in it.
///
/// The generator can round the corner where two straight roads meet, but not where
/// a curve arrives, and a road drawn by hand arrives at an angle whatever is done.
/// So a direct link between smooth roads bends both ends to the mean of their
/// directions there. A polyline road keeps its corner: the user drew one.
fn joint_tangents(document: &Document) -> HashMap<(String, End), Vector3> {
    let mut held: HashMap<(String, End), Vector3> = HashMap::new();
    for link in document.links.iter().filter(|link| link.junction.is_none()) {
        let (Some(a), Some(b)) = (document.road(&link.from), document.road(&link.to)) else {
            continue;
        };
        if a.shape != Shape::Smooth || b.shape != Shape::Smooth {
            continue;
        }
        let (Ok(pa), Ok(pb)) = (control_points(a), control_points(b)) else {
            continue;
        };
        let at = |points: &[Point3], end: End| match end {
            End::Start => (points[0], chord_at(points, 0)),
            End::End => (points[points.len() - 1], chord_at(points, points.len() - 1)),
        };
        let (joint_a, along_a) = at(&pa, link.from_end);
        let (joint_b, along_b) = at(&pb, link.to_end);
        if joint_a.horizontal_distance_to(joint_b) > 0.01 {
            continue;
        }
        // Both as the direction traffic passes through the joint, from a into b.
        let out_of_a = if link.from_end == End::End {
            along_a
        } else {
            -along_a
        };
        let into_b = if link.to_end == End::Start {
            along_b
        } else {
            -along_b
        };
        let (Ok(out_of_a), Ok(into_b)) = (out_of_a.normalize(), into_b.normalize()) else {
            continue;
        };
        // The mean of the two directions points forwards for both roads however
        // sharp the turn — a road drawn off the end of another at 120° is bent
        // to meet it, not refused — until they all but reverse into each other,
        // where it is too short to say which way anything goes.
        let through = out_of_a.get() + into_b.get();
        if through.horizontal_norm() < 0.2 {
            continue;
        }
        let a_key = (link.from.clone(), link.from_end);
        let b_key = (link.to.clone(), link.to_end);
        if held.contains_key(&a_key) || held.contains_key(&b_key) {
            continue;
        }
        held.insert(
            a_key,
            if link.from_end == End::End {
                through
            } else {
                -through
            },
        );
        held.insert(
            b_key,
            if link.to_end == End::Start {
                through
            } else {
                -through
            },
        );
    }
    held
}
