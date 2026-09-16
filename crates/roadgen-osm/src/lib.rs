//! `roadgen-osm` — writes the canonical IR as plain OpenStreetMap XML.
//!
//! Not to be confused with [`roadgen-lanelet2`], which also writes a `.osm` file.
//! That one is Lanelet2's *use* of the OSM container: its ways are lane boundaries
//! and its relations are lanelets, and nothing in it carries a `highway` tag. This
//! one writes OSM as OSM understands it, so that a router, a renderer or JOSM sees
//! roads.
//!
//! # What OSM says a road is
//!
//! One way down the centreline, with the lanes as a *count* in a tag. That is the
//! whole of the difference: the widths, boundaries, markings, tapers and
//! superelevation the IR holds have nowhere to go, and no amount of tagging will
//! give them one. [`check`] says so rather than letting the file look like a round
//! trip.
//!
//! Height is nearly as bad. An OSM node is a latitude and a longitude; elevation is
//! the `ele` *tag* convention rather than part of the geometry. The heights the
//! generator computed are written as those tags, which is the most the format
//! allows.
//!
//! # Junctions
//!
//! OSM has no connector roads. Arms meet at one shared node, and every turn is legal
//! unless a `type=restriction` relation says otherwise — the exact opposite of the
//! IR, which enumerates the movements that *are* permitted and draws a connector for
//! each.
//!
//! So a junction becomes a single node at the centre of the arm ends that face it,
//! every arm is extended to that node, and a pair of arms with no movement between
//! them becomes a restriction. Extending the arms is the one place this exporter
//! moves geometry, and it is what makes the result routable: four ways that stop
//! short of each other are four dead ends.

pub mod error;
pub mod tags;

use std::collections::{BTreeMap, HashMap, HashSet};
use std::path::Path;

use ll2_io::osm::{Document, Member, MemberType, Node, Relation, Tags, Way, WriteParams};
use ll2_projection::{GpsPoint, LocalCartesian, Origin, Projector, Utm};

use roadgen_core::geometry::Point3;
use roadgen_core::map::{Map, Projection, Road};
use roadgen_core::semantics::{MapObject, MapObjectKind, ObjectGeometry};
use roadgen_core::topology::{RoadEnd, RoadLinkTarget};
use roadgen_core::{JunctionId, RoadId, ValidatedMap};

pub use error::ExportError;

/// Two positions closer than this are the same node, which is how a way picks up
/// the node its neighbour ends on.
const WELD_TOLERANCE: f64 = 1e-6;

/// How close a piece of furniture has to be to a vertex of its way to sit on it
/// rather than get a node of its own.
///
/// Half a metre, because an OSM node on a road centreline does not mean anything
/// finer than that, and a control can easily land a little off a vertex without
/// being anywhere else: a traffic light hanging five metres over a graded road is a
/// quarter of a metre up-road of the stop line underneath it once the mounting
/// height is projected down. Two nodes that close apart are noise, and the
/// precedence below is what decides which control the one node carries.
const NODE_SPACING: f64 = 0.5;

/// Renders `map` as an OpenStreetMap XML document.
pub fn to_xml(map: &ValidatedMap) -> Result<String, ExportError> {
    let document = to_document(map)?;
    let xml = document.to_xml(WriteParams::default());
    // The writer is Lanelet2's, because it is a general OSM document model and there
    // is no reason to have a second one — but this file is not a Lanelet2 map, and
    // `generator` is the one attribute that says who wrote the data rather than what
    // is in it. The literal appears once, in the root element.
    Ok(xml.replacen("generator=\"lanelet2\"", "generator=\"roadgen\"", 1))
}

/// Writes `map` to `path` as OpenStreetMap XML.
pub fn write(map: &ValidatedMap, path: impl AsRef<Path>) -> Result<(), ExportError> {
    let path = path.as_ref();
    std::fs::write(path, to_xml(map)?)
        .map_err(|error| ExportError::Io(format!("{}: {error}", path.display())))
}

/// What this map loses on the way into OpenStreetMap.
///
/// All of these are properties of the format rather than faults in the map, so a
/// perfectly good map still reports them. That is the point: OSM is a description of
/// a road network for people and routers, not a description of a road surface.
pub fn check(map: &ValidatedMap) -> Vec<String> {
    let mut problems = vec![
        "OSM has no lane geometry: a lane is a count in a tag, so lane widths, \
         boundaries, markings and tapers are not written"
            .to_owned(),
    ];

    if map.roads.iter().any(|road| !road.superelevation.is_zero()) {
        problems.push("OSM cannot express superelevation, so the banking is dropped".to_owned());
    }
    let changing: Vec<&str> = map
        .roads
        .iter()
        .filter(|road| !road.is_connector() && road.sections.len() > 1)
        .map(|road| road.id.as_str())
        .collect();
    if !changing.is_empty() {
        problems.push(format!(
            "one way carries one lane count, so the changing cross-section of {} is \
             written as its first section's: {}",
            changing.len(),
            changing.join(", ")
        ));
    }
    if !map.junctions.is_empty() {
        problems.push(format!(
            "the {} junctions become a single node each, with every arm extended to \
             it — the only place this export moves geometry, and what makes the \
             result routable",
            map.junctions.len()
        ));
    }
    let heights = map
        .roads
        .iter()
        .filter_map(|road| road.reference_line.to_polyline(map.metadata.sampling).ok())
        .flat_map(|line| {
            line.points()
                .iter()
                .map(|point| point.z)
                .collect::<Vec<_>>()
        })
        .fold((f64::MAX, f64::MIN), |(low, high), z| {
            (low.min(z), high.max(z))
        });
    if heights.1 - heights.0 > 1e-6 {
        problems.push(format!(
            "an OSM node is a latitude and a longitude, so the map's {:.1} m of relief \
             survives only as `ele` tags",
            heights.1 - heights.0
        ));
    }
    problems
}

// --------------------------------------------------------------------------- //
// Building the document
// --------------------------------------------------------------------------- //

fn to_document(map: &ValidatedMap) -> Result<Document, ExportError> {
    let mut exporter = Exporter::new(map)?;
    exporter.build()?;
    Ok(exporter.document)
}

/// The projector that turns the map's metres into latitudes and longitudes.
///
/// MGRS is a way of *reporting* metres, not a different place on the globe, so a map
/// using it projects exactly as a local one does; the grid square is a Lanelet2
/// concern and has no meaning in an OSM file.
fn projector_for(map: &ValidatedMap) -> Result<Box<dyn Projector>, ExportError> {
    let origin = Origin::new(GpsPoint::new(
        map.metadata.origin.latitude(),
        map.metadata.origin.longitude(),
        map.metadata.origin.altitude(),
    ));
    Ok(match map.metadata.projection {
        Projection::LocalCartesian | Projection::Mgrs => Box::new(LocalCartesian::new(origin)),
        Projection::Utm => Box::new(
            Utm::new(origin, true, false)
                .map_err(|error| ExportError::Projection(error.message().to_owned()))?,
        ),
    })
}

struct Exporter<'a> {
    map: &'a ValidatedMap,
    projector: Box<dyn Projector>,
    document: Document,
    /// Identifiers count down from -1: negative ids are how a file says its contents
    /// were never uploaded, which is true of a generated map.
    next_id: i64,
    /// Nodes already placed, keyed by position, so that two roads meeting at a point
    /// share the node rather than stacking two on top of each other. Sharing a node
    /// *is* connectivity in OSM.
    welded: HashMap<(i64, i64, i64), i64>,
    /// Where each node sits in the map's own metres, so that a way can be measured
    /// along without going back through the projection.
    positions: HashMap<i64, Point3>,
    /// The way each road was written as.
    ways: HashMap<RoadId, i64>,
    /// The node each junction collapsed to.
    junction_nodes: HashMap<JunctionId, i64>,
}

impl<'a> Exporter<'a> {
    fn new(map: &'a ValidatedMap) -> Result<Self, ExportError> {
        Ok(Exporter {
            map,
            projector: projector_for(map)?,
            document: Document::default(),
            next_id: -1,
            welded: HashMap::new(),
            positions: HashMap::new(),
            ways: HashMap::new(),
            junction_nodes: HashMap::new(),
        })
    }

    fn build(&mut self) -> Result<(), ExportError> {
        // Junction nodes first: a road's way has to be able to end on one.
        for junction in self.map.junctions.iter() {
            let centre = self.map.junction_centre(&junction.id);
            if let Some(centre) = centre {
                let node = self.node_at(centre)?;
                self.junction_nodes.insert(junction.id.clone(), node);
            }
        }

        for road in self.map.roads.iter() {
            if road.is_connector() {
                // A connector is the IR's way of drawing a movement through a
                // junction. OSM draws no such thing — the movement is the pair of
                // ways sharing the junction's node, and what is forbidden is said in
                // a relation instead.
                continue;
            }
            let way = self.road_way(road)?;
            self.ways.insert(road.id.clone(), way);
        }

        self.add_furniture()?;
        self.add_restrictions();
        Ok(())
    }

    fn take_id(&mut self) -> i64 {
        let id = self.next_id;
        self.next_id -= 1;
        id
    }

    /// The node at `point`, reusing one already there.
    fn node_at(&mut self, point: Point3) -> Result<i64, ExportError> {
        let key = (
            (point.x / WELD_TOLERANCE).round() as i64,
            (point.y / WELD_TOLERANCE).round() as i64,
            (point.z / WELD_TOLERANCE).round() as i64,
        );
        if let Some(id) = self.welded.get(&key) {
            return Ok(*id);
        }
        let gps = self
            .projector
            .reverse([point.x, point.y, point.z])
            .map_err(|error| ExportError::Projection(error.message().to_owned()))?;
        let id = self.take_id();
        self.document.nodes.insert(
            id,
            Node {
                id,
                lat: gps.lat,
                lon: gps.lon,
                // Written as an `ele` tag, which is all OSM has for a height.
                ele: point.z,
                tags: Tags::new(),
            },
        );
        self.welded.insert(key, id);
        self.positions.insert(id, point);
        Ok(id)
    }

    fn road_way(&mut self, road: &Road) -> Result<i64, ExportError> {
        let line = road
            .reference_line
            .to_polyline(self.map.metadata.sampling)?;
        let mut nodes = Vec::with_capacity(line.len() + 2);
        for point in line.points() {
            nodes.push(self.node_at(*point)?);
        }

        // An end that faces a junction is carried on to the junction's node, so the
        // arms actually meet. In the IR they stop short of each other by half the
        // junction, with the connectors covering the gap.
        for end in [RoadEnd::Start, RoadEnd::End] {
            let Some(RoadLinkTarget::Junction(junction)) = road.link.at(end) else {
                continue;
            };
            let Some(node) = self.junction_nodes.get(junction).copied() else {
                continue;
            };
            match end {
                RoadEnd::Start => nodes.insert(0, node),
                RoadEnd::End => nodes.push(node),
            }
        }
        nodes.dedup();

        let id = self.take_id();
        self.document.ways.insert(
            id,
            Way {
                id,
                nodes,
                tags: tags::road_tags(self.map, road),
            },
        );
        Ok(id)
    }

    /// Furniture goes on a way's own nodes, which is how OSM places it: a traffic
    /// light is a property of the point on the road where you stop, not an object
    /// beside it. A node is inserted at the object's own position rather than the
    /// nearest vertex, because a straight road has two of those and everything on it
    /// would otherwise pile up on one end.
    fn add_furniture(&mut self) -> Result<(), ExportError> {
        for object in self.map.objects.iter() {
            let Some(road) = self.road_of(object) else {
                continue;
            };
            let Some(position) = object_position(object) else {
                continue;
            };
            let Some(node) = self.node_on_way(&road, position)? else {
                continue;
            };

            let (key, value) = match &object.kind {
                MapObjectKind::TrafficLight => ("highway", "traffic_signals".to_owned()),
                MapObjectKind::StopLine => ("highway", "stop".to_owned()),
                MapObjectKind::TrafficSign { code } => ("traffic_sign", code.clone()),
                MapObjectKind::Crosswalk => ("highway", "crossing".to_owned()),
            };
            if let Some(node) = self.document.nodes.get_mut(&node) {
                // Several controls can share a point — a stop line under a traffic
                // light is the usual case — and OSM has one `highway` tag for the
                // node. The stronger control wins rather than whichever was added
                // last: a signalised stop is signals, not a stop sign.
                let keep = key == "highway"
                    && node
                        .tags
                        .get("highway")
                        .is_some_and(|existing| control_rank(existing) >= control_rank(&value));
                if !keep {
                    node.tags.insert(key.into(), value);
                }
            }

            // A crossing is also a footway in its own right, so that it is a thing
            // pedestrians can be routed along rather than only a tag on the road.
            if object.kind == MapObjectKind::Crosswalk {
                self.add_crossing_way(object)?;
            }
        }
        Ok(())
    }

    fn add_crossing_way(&mut self, object: &MapObject) -> Result<(), ExportError> {
        let ObjectGeometry::Band { left, right } = &object.geometry else {
            return Ok(());
        };
        let config = self.map.metadata.sampling;
        let (left, right) = (left.to_polyline(config)?, right.to_polyline(config)?);
        if left.len() != right.len() {
            return Ok(());
        }
        // Down the middle of the painted strip, which is where someone crossing
        // actually walks.
        let mut nodes = Vec::with_capacity(left.len());
        for (a, b) in left.points().iter().zip(right.points()) {
            nodes.push(self.node_at(a.lerp(*b, 0.5))?);
        }
        nodes.dedup();
        if nodes.len() < 2 {
            return Ok(());
        }

        let id = self.take_id();
        let mut tags = Tags::new();
        tags.insert("highway".into(), "footway".into());
        tags.insert("footway".into(), "crossing".into());
        self.document.ways.insert(id, Way { id, nodes, tags });
        Ok(())
    }

    fn road_of(&self, object: &MapObject) -> Option<RoadId> {
        let lane = object.lanes.first()?;
        Some(self.map.lane(lane)?.road.clone())
    }

    /// The node of `road`'s way at `position`, inserting one if the way does not
    /// already pass through a vertex there.
    ///
    /// The search is horizontal, because a traffic light five metres above the road
    /// is still at the same place on it.
    fn node_on_way(&mut self, road: &RoadId, position: Point3) -> Result<Option<i64>, ExportError> {
        let Some(way_id) = self.ways.get(road).copied() else {
            return Ok(None);
        };
        let Some(nodes) = self.document.ways.get(&way_id).map(|way| way.nodes.clone()) else {
            return Ok(None);
        };
        if nodes.len() < 2 {
            return Ok(None);
        }

        // Which stretch of the way the object is abeam of, and where along it.
        let mut best: Option<(usize, f64, f64)> = None;
        for (index, pair) in nodes.windows(2).enumerate() {
            let (Some(a), Some(b)) = (
                self.positions.get(&pair[0]).copied(),
                self.positions.get(&pair[1]).copied(),
            ) else {
                continue;
            };
            let along = (b.x - a.x, b.y - a.y);
            let length = along.0 * along.0 + along.1 * along.1;
            let fraction = if length > 0.0 {
                (((position.x - a.x) * along.0 + (position.y - a.y) * along.1) / length)
                    .clamp(0.0, 1.0)
            } else {
                0.0
            };
            let nearest = (a.x + along.0 * fraction, a.y + along.1 * fraction);
            let distance = (nearest.0 - position.x).hypot(nearest.1 - position.y);
            if best.is_none_or(|(_, _, previous)| distance < previous) {
                best = Some((index, fraction, distance));
            }
        }
        let Some((index, fraction, _)) = best else {
            return Ok(None);
        };

        // A vertex near enough to the spot is used as it stands; only a point well
        // between two of them needs a node of its own.
        let (a, b) = (
            self.positions[&nodes[index]],
            self.positions[&nodes[index + 1]],
        );
        let point = a.lerp(b, fraction);
        if point.distance_to(a) < NODE_SPACING {
            return Ok(Some(nodes[index]));
        }
        if point.distance_to(b) < NODE_SPACING {
            return Ok(Some(nodes[index + 1]));
        }

        let node = self.node_at(point)?;
        if let Some(way) = self.document.ways.get_mut(&way_id) {
            way.nodes.insert(index + 1, node);
        }
        Ok(Some(node))
    }

    /// A `no_*_turn` for every pair of arms the IR does not join.
    ///
    /// This is the direction OSM reads in: a junction permits everything, and a
    /// relation takes something away. The IR reads the other way round, so what is
    /// written here is the complement of what it holds.
    fn add_restrictions(&mut self) {
        let mut relations: Vec<Relation> = Vec::new();
        for junction in self.map.junctions.iter() {
            let Some(via) = self.junction_nodes.get(&junction.id).copied() else {
                continue;
            };
            let permitted = permitted_movements(self.map, &junction.id);
            let arms: Vec<&RoadId> = junction
                .incoming_roads
                .iter()
                .filter(|road| self.ways.contains_key(road))
                .collect();

            for from in &arms {
                for to in &arms {
                    if from == to || permitted.contains(&((*from).clone(), (*to).clone())) {
                        continue;
                    }
                    let (Some(from_road), Some(to_road)) = (self.map.road(from), self.map.road(to))
                    else {
                        continue;
                    };
                    let Some(change) = heading_change(from_road, to_road, &junction.id) else {
                        continue;
                    };
                    let Some(restriction) = tags::restriction_value(change) else {
                        continue;
                    };

                    let mut tags = Tags::new();
                    tags.insert("type".into(), "restriction".into());
                    tags.insert("restriction".into(), restriction.into());
                    relations.push(Relation {
                        id: 0, // assigned below, once the borrow of `self` has ended
                        members: vec![
                            Member {
                                kind: MemberType::Way,
                                reference: self.ways[*from],
                                role: "from".into(),
                            },
                            Member {
                                kind: MemberType::Node,
                                reference: via,
                                role: "via".into(),
                            },
                            Member {
                                kind: MemberType::Way,
                                reference: self.ways[*to],
                                role: "to".into(),
                            },
                        ],
                        tags,
                    });
                }
            }
        }

        for mut relation in relations {
            relation.id = self.take_id();
            self.document.relations.insert(relation.id, relation);
        }
    }
}

// --------------------------------------------------------------------------- //
// Junction geometry
// --------------------------------------------------------------------------- //

/// How strong a control a `highway` value is, so that two at one point do not
/// overwrite each other by accident.
fn control_rank(value: &str) -> u8 {
    match value {
        "traffic_signals" => 3,
        "stop" => 2,
        "give_way" => 1,
        _ => 0,
    }
}

/// The end of `road` that faces `junction`.
fn end_facing(road: &Road, junction: &JunctionId) -> Option<RoadEnd> {
    [RoadEnd::Start, RoadEnd::End]
        .into_iter()
        .find(|end| road.link.at(*end) == Some(&RoadLinkTarget::Junction(junction.clone())))
}

/// How far the heading swings going in by `from` and out by `to`.
fn heading_change(from: &Road, to: &Road, junction: &JunctionId) -> Option<f64> {
    let from_end = end_facing(from, junction)?;
    let to_end = end_facing(to, junction)?;

    // Approaching by an end means travelling along the reference line; approaching
    // by the start means travelling against it.
    let incoming = match from_end {
        RoadEnd::End => from.reference_line.end_tangent().ok()?,
        RoadEnd::Start => from.reference_line.start_tangent().ok()?.reversed(),
    };
    let outgoing = match to_end {
        RoadEnd::Start => to.reference_line.start_tangent().ok()?,
        RoadEnd::End => to.reference_line.end_tangent().ok()?.reversed(),
    };
    Some(outgoing.heading() - incoming.heading())
}

/// The ordered pairs of arms a junction actually joins.
///
/// Read off the connections rather than the road links, because a movement goes
/// approach → connector → exit and only the pair of connections names both arms.
fn permitted_movements(map: &Map, junction: &JunctionId) -> HashSet<(RoadId, RoadId)> {
    let mut into: BTreeMap<RoadId, Vec<RoadId>> = BTreeMap::new();
    let mut out_of: BTreeMap<RoadId, Vec<RoadId>> = BTreeMap::new();

    for connection in map.connections.iter() {
        if connection.junction.as_ref() != Some(junction) {
            continue;
        }
        let (Some(from), Some(to)) = (
            map.lane(&connection.from.lane),
            map.lane(&connection.to.lane),
        ) else {
            continue;
        };
        let from_connector = map.road(&from.road).is_some_and(Road::is_connector);
        let to_connector = map.road(&to.road).is_some_and(Road::is_connector);
        match (from_connector, to_connector) {
            (false, true) => into
                .entry(to.road.clone())
                .or_default()
                .push(from.road.clone()),
            (true, false) => out_of
                .entry(from.road.clone())
                .or_default()
                .push(to.road.clone()),
            // A movement written straight between two arms, with no connector.
            (false, false) => {
                return direct_movements(map, junction);
            }
            (true, true) => {}
        }
    }

    let mut permitted = HashSet::new();
    for (connector, approaches) in &into {
        for exit in out_of.get(connector).into_iter().flatten() {
            for approach in approaches {
                permitted.insert((approach.clone(), exit.clone()));
            }
        }
    }
    permitted
}

/// The same, for a junction whose movements were written lane to lane without a
/// connector in between.
fn direct_movements(map: &Map, junction: &JunctionId) -> HashSet<(RoadId, RoadId)> {
    map.connections
        .iter()
        .filter(|connection| connection.junction.as_ref() == Some(junction))
        .filter_map(|connection| {
            let from = map.lane(&connection.from.lane)?;
            let to = map.lane(&connection.to.lane)?;
            (from.road != to.road).then(|| (from.road.clone(), to.road.clone()))
        })
        .collect()
}

/// Where an object sits, as one point.
fn object_position(object: &MapObject) -> Option<Point3> {
    Some(match &object.geometry {
        ObjectGeometry::Point(point) => *point,
        ObjectGeometry::Line(line) => line.start_point().lerp(line.end_point(), 0.5),
        ObjectGeometry::Band { left, right } => left
            .start_point()
            .lerp(left.end_point(), 0.5)
            .lerp(right.start_point().lerp(right.end_point(), 0.5), 0.5),
    })
}
