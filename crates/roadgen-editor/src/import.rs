//! From the canonical IR back to a [`Document`], so a map read from a file can be
//! edited.
//!
//! A document says far less than the IR can, so this loses things, and says which:
//! every road becomes its reference line — an alignment where it is straights, arcs
//! and clothoids, a polyline otherwise — and the lanes of its first
//! cross-section at their starting widths; every junction becomes the links its
//! connectors were made from; surface features, bridges and tunnels stay on their
//! roads; furniture, rules, buildings and terrain are dropped.
//! What comes back builds into a map with the same roads, lanes and movements, which
//! is what the editor is for.

use std::collections::BTreeSet;

use roadgen_core::{
    BumpProfile, Curve3, Direction, LateralSide, Map, MapObjectKind, Point3, RoadEnd, RoadEndpoint,
    RoadLinkTarget, Structure, StructureKind, SurfaceFeature, SurfaceFeatureKind,
    TrafficHandedness,
};

use crate::document::{
    Document, ElementDoc, End, Handedness, LaneDoc, LinkDoc, Marking, RoadDoc, RutPath, Shape,
    Side, StructureDoc, SurfaceDoc, Travel,
};

/// How far a simplified reference line may stray from the one it came from, metres.
pub const SIMPLIFY_TOLERANCE: f64 = 0.02;

/// A document saying what `map` says about roads, lanes and links, with a note for
/// each kind of thing it had to leave behind.
pub fn from_map(map: &Map) -> (Document, Vec<String>) {
    let mut notes = Vec::new();
    let handedness = map.metadata.handedness;
    let mut document = Document {
        name: map
            .metadata
            .name
            .clone()
            .unwrap_or_else(|| "imported".to_owned()),
        handedness: match handedness {
            TrafficHandedness::RightHand => Handedness::Right,
            TrafficHandedness::LeftHand => Handedness::Left,
        },
        origin: {
            let origin = map.metadata.origin;
            Some([origin.latitude(), origin.longitude(), origin.altitude()])
        },
        ..Document::default()
    };

    let mut sectioned = 0;
    let mut tapered = 0;
    for road in map.roads.iter().filter(|road| !road.is_connector()) {
        let id = key(road.id.as_str(), "road");
        let points = match road.reference_line.to_polyline(map.metadata.sampling) {
            Ok(polyline) => simplify(polyline.points(), SIMPLIFY_TOLERANCE),
            Err(error) => {
                notes.push(format!("road '{id}' left out: {error}"));
                continue;
            }
        };
        if road.sections.len() > 1 {
            sectioned += 1;
        }
        let lanes: Vec<LaneDoc> = map
            .lanes_of_section(&road.id, 0)
            .into_iter()
            .map(|lane| {
                if !lane.width.is_constant() {
                    tapered += 1;
                }
                let default_side = handedness.side_for(lane.direction);
                LaneDoc {
                    width: lane.width_at(lane.station_range.0),
                    direction: match lane.direction {
                        Direction::Forward => Travel::Forward,
                        Direction::Backward => Travel::Backward,
                    },
                    lane_type: lane.lane_type.as_str().to_owned(),
                    side: (lane.side != default_side).then_some(match lane.side {
                        LateralSide::Left => Side::Left,
                        LateralSide::Right => Side::Right,
                    }),
                    left_marking: Marking::new(
                        lane.left_marking.marking.as_str(),
                        lane.left_marking.color.as_str(),
                    ),
                    right_marking: Marking::new(
                        lane.right_marking.marking.as_str(),
                        lane.right_marking.color.as_str(),
                    ),
                }
            })
            .collect();
        // Straights, arcs and clothoids come back as the alignment they are, and
        // anything else as its line through the simplified points — which an
        // alignment keeps too, for switching to drawing it by hand.
        let aligned = alignment_of(&road.reference_line);
        document.roads.push(RoadDoc {
            id,
            points: points.iter().map(|p| p.as_array()).collect(),
            shape: if aligned.is_some() {
                Shape::Alignment
            } else {
                Shape::Polyline
            },
            handles: Vec::new(),
            heading: aligned.as_ref().map_or(0.0, |(heading, _)| *heading),
            elements: aligned.map(|(_, elements)| elements).unwrap_or_default(),
            lanes,
            road_type: road.road_type.as_str().to_owned(),
            speed_limit: road.speed_limit.map(|limit| limit.kph()),
            surface: map
                .surface_features
                .iter()
                .filter(|feature| feature.road == road.id)
                .map(surface_doc)
                .collect(),
            structures: map
                .structures
                .iter()
                .flat_map(|structure| {
                    structure
                        .carries
                        .iter()
                        .filter(|span| span.road == road.id)
                        .map(move |span| structure_doc(structure, span.from, span.to))
                })
                .collect(),
        });
    }

    // Links between two ordinary roads, once per pair of ends.
    // A road end, and a pair of them with the junction they meet in.
    type Joint = ((String, End), (String, End), Option<String>);
    let mut seen: BTreeSet<Joint> = BTreeSet::new();
    let mut add =
        |document: &mut Document, a: (String, End), b: (String, End), junction: Option<String>| {
            let key = if a <= b {
                (a.clone(), b.clone(), junction.clone())
            } else {
                (b.clone(), a.clone(), junction.clone())
            };
            if seen.insert(key) {
                document.links.push(LinkDoc {
                    from: a.0,
                    from_end: a.1,
                    to: b.0,
                    to_end: b.1,
                    junction,
                });
            }
        };
    let ordinary = |endpoint: &RoadEndpoint| {
        map.road(&endpoint.road)
            .is_some_and(|road| !road.is_connector())
    };
    for road in map.roads.iter().filter(|road| !road.is_connector()) {
        for end in [RoadEnd::Start, RoadEnd::End] {
            if let Some(RoadLinkTarget::Road(target)) = road.link.at(end) {
                if ordinary(target) {
                    add(
                        &mut document,
                        (key(road.id.as_str(), "road"), document_end(end)),
                        (key(target.road.as_str(), "road"), document_end(target.end)),
                        None,
                    );
                }
            }
        }
    }
    // A connector runs from one arm of its junction to another: that pair of arms
    // is a link through the junction, whatever the connector's lanes are.
    let mut unexplained = 0;
    for connector in map.roads.iter().filter(|road| road.is_connector()) {
        let junction = connector
            .junction
            .as_ref()
            .map(|junction| key(junction.as_str(), "junction"));
        match (&connector.link.predecessor, &connector.link.successor) {
            (Some(RoadLinkTarget::Road(a)), Some(RoadLinkTarget::Road(b)))
                if ordinary(a) && ordinary(b) =>
            {
                add(
                    &mut document,
                    (key(a.road.as_str(), "road"), document_end(a.end)),
                    (key(b.road.as_str(), "road"), document_end(b.end)),
                    junction,
                );
            }
            _ => unexplained += 1,
        }
    }

    if sectioned > 0 {
        notes.push(format!(
            "{sectioned} road(s) change their lane count partway; only the first cross-section was kept"
        ));
    }
    if tapered > 0 {
        notes.push(format!(
            "{tapered} lane(s) change width; each keeps the width it starts with"
        ));
    }
    if unexplained > 0 {
        notes.push(format!(
            "{unexplained} connector(s) do not run between two roads and were dropped"
        ));
    }
    let in_junctions = map
        .surface_features
        .iter()
        .filter(|feature| {
            map.road(&feature.road)
                .is_none_or(|road| road.is_connector())
        })
        .count();
    if in_junctions > 0 {
        notes.push(format!(
            "{in_junctions} surface feature(s) on roads through junctions dropped: the editor \
             generates those roads, and keeps nothing on them"
        ));
    }
    let shared = map
        .structures
        .iter()
        .filter(|structure| structure.carries.len() > 1)
        .count();
    if shared > 0 {
        notes.push(format!(
            "{shared} bridge(s) or tunnel(s) carry several roads; each road now has its own"
        ));
    }
    for (count, what) in [
        (
            // A bridge's parapets are its own: built again with it.
            map.objects
                .iter()
                .filter(|object| {
                    !matches!(
                        object.kind,
                        MapObjectKind::Barrier {
                            structure: Some(_),
                            ..
                        }
                    )
                })
                .count(),
            "traffic object(s)",
        ),
        (map.rules.len(), "traffic rule(s)"),
        (map.buildings.len(), "building(s)"),
        (map.land_covers.len(), "land cover(s)"),
        (map.tin_surfaces.len(), "terrain surface(s)"),
    ] {
        if count > 0 {
            notes.push(format!(
                "{count} {what} dropped: the editor does not edit them"
            ));
        }
    }
    (document, notes)
}

/// A reference line as an alignment — its heading at the start, degrees, and its
/// pieces — when it is a chain of straights, arcs and clothoids that
/// [`roadgen_core::Alignment`] would build again: each piece starting where the
/// last ended, pointing the way it pointed, and a clothoid starting at the
/// curvature the piece before it ended with. `None` for anything else, and for a
/// chain of straights alone, which is drawn as well by its points.
pub fn alignment_of(curve: &Curve3) -> Option<(f64, Vec<ElementDoc>)> {
    fn flatten<'a>(curve: &'a Curve3, into: &mut Vec<&'a Curve3>) {
        match curve {
            Curve3::Composite(pieces) => pieces.iter().for_each(|piece| flatten(piece, into)),
            piece => into.push(piece),
        }
    }
    let mut pieces = Vec::new();
    flatten(curve, &mut pieces);

    let start = curve.start_tangent().ok()?.heading();
    let mut heading = start;
    let mut point = curve.start_point();
    let mut curvature = 0.0;
    let mut elements = Vec::new();
    let turned = |a: f64, b: f64| {
        let d = (a - b).rem_euclid(std::f64::consts::TAU);
        d.min(std::f64::consts::TAU - d)
    };
    for piece in pieces {
        let (begin, direction, element, ends_curving) = match piece {
            Curve3::Line(line) => {
                let (from, to) = (line.start(), line.end());
                let mut element = ElementDoc::line(from.horizontal_distance_to(to));
                element.rise = to.z - from.z;
                (from, (to.y - from.y).atan2(to.x - from.x), element, 0.0)
            }
            Curve3::Arc(arc) => {
                if arc.curvature() == 0.0 {
                    return None;
                }
                let mut element = ElementDoc::arc(arc.horizontal_length(), 1.0 / arc.curvature());
                element.rise = arc.end_z() - arc.start().z;
                (arc.start(), arc.heading(), element, arc.curvature())
            }
            Curve3::Clothoid(spiral) => {
                if (spiral.curvature_start() - curvature).abs() > 1e-9 {
                    return None;
                }
                let end = spiral.curvature_end();
                let mut element =
                    ElementDoc::spiral(spiral.horizontal_length(), (end != 0.0).then(|| 1.0 / end));
                element.rise = spiral.end_z() - spiral.start().z;
                (spiral.start(), spiral.heading(), element, end)
            }
            _ => return None,
        };
        if begin.distance_to(point) > 1e-3 || turned(direction, heading) > 1e-6 {
            return None;
        }
        elements.push(element);
        point = piece.end_point();
        heading = piece.end_tangent().ok()?.heading();
        curvature = ends_curving;
    }
    elements
        .iter()
        .any(|element| element.kind != "line")
        .then(|| (start.to_degrees(), elements))
}

/// A bridge or a tunnel as the document says it, over `from`–`to` of one road.
fn structure_doc(structure: &Structure, from: f64, to: f64) -> StructureDoc {
    let mut doc = StructureDoc::new(structure.kind.as_str(), from, to);
    doc.name = structure.name.clone().unwrap_or_default();
    match &structure.kind {
        StructureKind::Bridge(bridge) => {
            doc.form = bridge.form.as_str().to_owned();
            doc.material = bridge.material.as_str().to_owned();
            doc.deck_depth = bridge.deck_depth;
            doc.curb = bridge.curb;
        }
        StructureKind::Tunnel(tunnel) => {
            doc.form = tunnel.form.as_str().to_owned();
            doc.clear_height = tunnel.clear_height;
            doc.lining = tunnel.lining;
            doc.lighting = tunnel.lighting;
            doc.walkway = tunnel.walkway;
        }
    }
    doc
}

/// A surface feature as the document says it: its footprint, and the parameters
/// of its kind, the others left at their defaults.
fn surface_doc(feature: &SurfaceFeature) -> SurfaceDoc {
    let mut doc = SurfaceDoc::new(
        feature.kind.as_str(),
        feature.station,
        feature.length,
        feature.offset,
        feature.width,
    );
    match &feature.kind {
        SurfaceFeatureKind::Rut {
            depth,
            track,
            trough,
            path,
        } => {
            doc.depth = *depth;
            doc.track = *track;
            doc.trough = *trough;
            doc.path = path.as_ref().map(|path| RutPath {
                lane: lane_index(path.lane.as_str()),
                knots: path.knots().iter().map(|&(s, t)| [s, t]).collect(),
            });
        }
        SurfaceFeatureKind::Pothole { depth, seed } => {
            doc.depth = *depth;
            doc.seed = *seed;
        }
        SurfaceFeatureKind::Puddle { depth } => doc.depth = *depth,
        SurfaceFeatureKind::Bump {
            kind,
            height,
            profile,
        } => {
            doc.bump = kind.as_str().to_owned();
            doc.height = *height;
            doc.profile = profile.as_str().to_owned();
            if let BumpProfile::Flat { ramp } = profile {
                doc.ramp = *ramp;
            }
        }
        SurfaceFeatureKind::Roughness { class, seed } => {
            doc.class = class.as_str().to_owned();
            doc.seed = *seed;
        }
    }
    doc
}

/// A lane's position in its road's list, from its identifier `lane/<road>/<n>`.
fn lane_index(id: &str) -> usize {
    id.rsplit('/')
        .next()
        .and_then(|n| n.parse().ok())
        .unwrap_or(0)
}

fn document_end(end: RoadEnd) -> End {
    match end {
        RoadEnd::Start => End::Start,
        RoadEnd::End => End::End,
    }
}

/// An identifier without its kind: `road/north` is `north` in a document.
fn key(id: &str, prefix: &str) -> String {
    id.strip_prefix(prefix)
        .and_then(|rest| rest.strip_prefix('/'))
        .unwrap_or(id)
        .to_owned()
}

/// Douglas–Peucker in plan, keeping the ends: the fewest vertices that stay within
/// `tolerance` of the line, so an imported road has handles where it bends and not
/// every two metres along it.
pub fn simplify(points: &[Point3], tolerance: f64) -> Vec<Point3> {
    if points.len() <= 2 {
        return points.to_vec();
    }
    let mut keep = vec![false; points.len()];
    keep[0] = true;
    keep[points.len() - 1] = true;
    let mut stack = vec![(0, points.len() - 1)];
    while let Some((first, last)) = stack.pop() {
        let (a, b) = (points[first], points[last]);
        let mut farthest = (0.0, first);
        for (i, point) in points.iter().enumerate().take(last).skip(first + 1) {
            let distance = plan_distance(*point, a, b);
            if distance > farthest.0 {
                farthest = (distance, i);
            }
        }
        if farthest.0 > tolerance {
            keep[farthest.1] = true;
            stack.push((first, farthest.1));
            stack.push((farthest.1, last));
        }
    }
    points
        .iter()
        .zip(keep)
        .filter_map(|(point, kept)| kept.then_some(*point))
        .collect()
}

/// Distance in plan from `p` to the segment `a`–`b`.
fn plan_distance(p: Point3, a: Point3, b: Point3) -> f64 {
    let (dx, dy) = (b.x - a.x, b.y - a.y);
    let length2 = dx * dx + dy * dy;
    let t = if length2 > 0.0 {
        (((p.x - a.x) * dx + (p.y - a.y) * dy) / length2).clamp(0.0, 1.0)
    } else {
        0.0
    };
    (p.x - (a.x + t * dx)).hypot(p.y - (a.y + t * dy))
}
