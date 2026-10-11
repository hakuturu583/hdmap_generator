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
//!
//! # Buildings
//!
//! The one thing OSM holds better than any other format here, because OSM has a model
//! for a building of several parts and the IR has the same one. **Simple 3D
//! Buildings** is the scheme: the building's outline is a closed way tagged
//! `building`, each of its parts is a closed way tagged `building:part=yes` inside it,
//! and `height`, `min_height`, `roof:shape`, `roof:height` and `roof:direction` say
//! what each one occupies. A building of a single part writes one way and puts all of
//! that on it, because a reader of a simple building expects to find it there.
//!
//! So nearly the whole of the IR's solid survives: the parts, the heights they span,
//! the roofs and which way their ridges run. What does not is the little the scheme
//! has no room for — a part standing on sloping ground becomes one `height` above one
//! ground level, a building of several parts is surrounded by the outline of the part
//! that meets the ground rather than by their union, and the frontage that says which
//! street a building faces has no tag at all. [`check`] says each of those rather
//! than letting the file look like a round trip.
//!
//! # Bridges, tunnels and levels
//!
//! OSM has no structure of its own: a road is on a bridge because its way says
//! `bridge=yes`, so the way has to begin and end where the bridge does. Each road
//! is cut at the ends of every structure that carries it, the pieces sharing the
//! node at each cut, and the piece on a bridge or in a tunnel is tagged for it —
//! `bridge=viaduct` for a viaduct, `tunnel=yes` for every tunnel, an underpass and
//! a covered cutting alike. `layer` is what keeps a renderer from drawing a
//! motorway under the street it bridges, and the IR keeps no level, only heights;
//! so the level is derived from the grade separations, from the ground up: 0 at
//! grade, a bridge one above the highest thing it passes over, a tunnel −1. A
//! bridge's deck outline is a closed way tagged `man_made=bridge`.
//!
//! The form of way chooses `highway` where it has a word for it: a motorway
//! carriageway is `motorway` whatever its road type, a slip road `motorway_link`
//! when it joins a motorway and `trunk_link` when it does not, and the junction
//! node where a slip road leaves a motorway is a `highway=motorway_junction`. A
//! land cover of water is `natural=water`.
//!
//! # Tracing
//!
//! Every id in the file comes off one negative counter, in the order things were
//! built, so nothing about the IR says which way a road became. The `_traced`
//! entry points keep a [`Trace`] of it as the ids are handed out, naming what was
//! written `node:<id>`, `way:<id>` or `relation:<id>`:
//!
//! - a road is its way, and a junction its node, one for one; a connector has no
//!   counterpart at all, since OSM draws the movement as two ways sharing a node;
//!   a road cut at its structures is a `part` of each of its ways;
//! - furniture is `merged` into the road node that carries it as a tag, with the tag
//!   as the role, and a crossing is also the footway across the road;
//! - a restriction is a `part` of its junction: it says what the junction forbids,
//!   and the IR has no element for a movement that does not exist;
//! - a building is its outline way, and each of its parts the way drawn for it;
//! - a structure is `merged` into each piece of road way it carries, with its tag —
//!   `bridge` or `tunnel` — as the role, and a bridge is also its deck's outline
//!   way; a grade separation is `collapsed` into the way of the bridge it raises,
//!   as the `layer` it counts toward, and an interchange is `merged` into each
//!   `motorway_junction` node it names.

pub mod error;
pub mod tags;

use std::collections::{BTreeMap, HashMap, HashSet};
use std::path::Path;

use ll2_io::osm::{Document, Member, MemberType, Node, Relation, Tags, Way, WriteParams};
use ll2_projection::{GpsPoint, LocalCartesian, Origin, Projector, Utm};

use roadgen_core::geometry::Point3;
use roadgen_core::map::{Map, Projection, Road};
use roadgen_core::semantics::{FormOfWay, MapObject, MapObjectKind, ObjectGeometry};
use roadgen_core::structure::{Structure, StructureKind, StructureSpan, Underneath};
use roadgen_core::topology::{RoadEnd, RoadLinkTarget};
// OSM has a relation of its own, which is the one this file means by the bare name.
use roadgen_core::trace::Relation as Link;
use roadgen_core::{JunctionId, RoadId, Trace, ValidatedMap};

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

/// The name a [`Trace`] of this exporter gives its format.
pub const TRACE_FORMAT: &str = "osm";

/// Renders `map` as an OpenStreetMap XML document.
pub fn to_xml(map: &ValidatedMap) -> Result<String, ExportError> {
    to_xml_traced(map).map(|(xml, _)| xml)
}

/// Renders `map` as an OpenStreetMap XML document, along with where each element of
/// the IR went in it. The trace names no file, because none was written.
pub fn to_xml_traced(map: &ValidatedMap) -> Result<(String, Trace), ExportError> {
    let (document, trace) = to_document(map)?;
    let xml = document.to_xml(WriteParams::default());
    // The writer is Lanelet2's, because it is a general OSM document model and there
    // is no reason to have a second one — but this file is not a Lanelet2 map, and
    // `generator` is the one attribute that says who wrote the data rather than what
    // is in it. The literal appears once, in the root element.
    let xml = xml.replacen("generator=\"lanelet2\"", "generator=\"roadgen\"", 1);
    Ok((xml, trace))
}

/// Writes `map` to `path` as OpenStreetMap XML.
pub fn write(map: &ValidatedMap, path: impl AsRef<Path>) -> Result<(), ExportError> {
    write_traced(map, path).map(drop)
}

/// Writes `map` to `path` as OpenStreetMap XML, and returns where each element of
/// the IR went in it.
pub fn write_traced(map: &ValidatedMap, path: impl AsRef<Path>) -> Result<Trace, ExportError> {
    let path = path.as_ref();
    let (xml, mut trace) = to_xml_traced(map)?;
    std::fs::write(path, xml)
        .map_err(|error| ExportError::Io(format!("{}: {error}", path.display())))?;
    trace.files.push(path.to_path_buf());
    Ok(trace)
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
    problems.extend(merged_controls(map));
    problems.extend(building_losses(map));
    let changing: Vec<&str> = map
        .roads
        .iter()
        .filter(|road| !road.is_connector() && road.pavement.spans().len() > 1)
        .map(|road| road.id.as_str())
        .collect();
    if !changing.is_empty() {
        problems.push(format!(
            "one way carries one `surface`, so the pavement of {} roads whose surface \
             changes along them is written as their first stretch's: {}",
            changing.len(),
            changing.join(", ")
        ));
    }
    if !map.structures.is_empty() {
        let tunnels = map
            .structures
            .iter()
            .filter(|structure| structure.kind.tunnel().is_some())
            .count();
        problems.push(format!(
            "OSM says a road is on a bridge or in a tunnel by tagging its way, so the \
             roads on the {} structures are cut where each begins and ends and given a \
             `layer` derived from what passes over what; a bridge's deck is a \
             `man_made=bridge` outline, but its piers and abutments, the {tunnels} \
             tunnels' bores and portals, and every height but `ele` are not written",
            map.structures.len()
        ));
    }
    let unspoken = map
        .roads
        .iter()
        .filter(|road| {
            matches!(
                road.form,
                FormOfWay::MultipleCarriageway
                    | FormOfWay::SingleCarriageway
                    | FormOfWay::ParallelRoad
            )
        })
        .count();
    if unspoken > 0 {
        problems.push(format!(
            "OSM has no tag for a carriageway's form beyond `highway`, `oneway` and \
             `junction=roundabout`, so whether the {unspoken} roads that say so are \
             single, dual or parallel carriageways is left to their `highway` rung"
        ));
    }
    if !map.tin_surfaces.is_empty() {
        problems.push(format!(
            "OSM has no terrain: the {} TIN surfaces are dropped, and the land covers on \
             them are written as flat areas",
            map.tin_surfaces.len()
        ));
    }
    problems
}

/// Controls that will share one node, and which of them keeps its `highway` tag.
///
/// Two controls within `NODE_SPACING` of each other on the same road land on one
/// node, and a node has one `highway` tag: the stronger control keeps it and the
/// other survives only as far as OSM's conventions carry it (a stop line under a
/// signal is what `traffic_signals` means). A crossing is not a control and always
/// gets a node of its own, so it is not in this list.
fn merged_controls(map: &ValidatedMap) -> Vec<String> {
    let controls: Vec<(&MapObject, Point3, &'static str)> = map
        .objects
        .iter()
        .filter_map(|object| {
            let value = match object.kind {
                MapObjectKind::TrafficLight { .. } => "traffic_signals",
                MapObjectKind::StopLine => "stop",
                _ => return None,
            };
            Some((object, object_position(object)?, value))
        })
        .collect();
    let mut problems = Vec::new();
    for (index, (object, position, value)) in controls.iter().enumerate() {
        for (other, other_position, other_value) in &controls[index + 1..] {
            if value == other_value
                || position.horizontal_distance_to(*other_position) >= NODE_SPACING
                || object
                    .lanes
                    .first()
                    .and_then(|lane| map.lane(lane))
                    .map(|lane| &lane.road)
                    != other
                        .lanes
                        .first()
                        .and_then(|lane| map.lane(lane))
                        .map(|lane| &lane.road)
            {
                continue;
            }
            let (kept, _) = if control_rank(value) >= control_rank(other_value) {
                (value, other_value)
            } else {
                (other_value, value)
            };
            problems.push(format!(
                "{} and {} stand within {NODE_SPACING} m of each other and share one node, \
                 which OSM tags `highway={kept}`",
                object.id, other.id
            ));
        }
    }
    problems
}

/// What a building loses, which is only what Simple 3D Buildings has no room for.
fn building_losses(map: &ValidatedMap) -> Vec<String> {
    if map.buildings.is_empty() {
        return Vec::new();
    }
    let mut problems = Vec::new();
    if map
        .buildings
        .iter()
        .any(|building| building.frontage.is_some())
    {
        problems.push(
            "OSM has no tag for which street a building faces, so the frontages are \
             dropped: the geometry still stands beside the way"
                .to_owned(),
        );
    }
    let sloping = map
        .building_parts
        .iter()
        .filter(|part| part.solid.footprint.highest() - part.solid.footprint.lowest() > 0.05)
        .count();
    if sloping > 0 {
        problems.push(format!(
            "`height` and `min_height` are single numbers above one ground level, so \
             the sloping base of {sloping} parts is written level"
        ));
    }
    let compound = map
        .buildings
        .iter()
        .filter(|building| building.parts.len() > 1)
        .count();
    if compound > 0 {
        problems.push(format!(
            "a building's outline is one way, so the {compound} buildings of several \
             parts are surrounded by the outline of the part that meets the ground \
             rather than by the union of them all"
        ));
    }
    problems
}

// --------------------------------------------------------------------------- //
// Building the document
// --------------------------------------------------------------------------- //

fn to_document(map: &ValidatedMap) -> Result<(Document, Trace), ExportError> {
    let mut exporter = Exporter::new(map)?;
    exporter.build()?;
    Ok((exporter.document, exporter.trace))
}

/// How a trace names a node of the file.
fn node_ref(id: i64) -> String {
    format!("node:{id}")
}

/// How a trace names a way of the file.
fn way_ref(id: i64) -> String {
    format!("way:{id}")
}

/// How a trace names a relation of the file.
fn relation_ref(id: i64) -> String {
    format!("relation:{id}")
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
    /// The ways each road was written as, in order along it: one, unless a
    /// structure carries part of it.
    ways: HashMap<RoadId, Vec<i64>>,
    /// The node each junction collapsed to.
    junction_nodes: HashMap<JunctionId, i64>,
    /// Where each IR element went, written down as each id is handed out: the ids
    /// come off one counter in build order, so they cannot be worked out afterwards.
    trace: Trace,
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
            trace: Trace::new(TRACE_FORMAT),
        })
    }

    fn build(&mut self) -> Result<(), ExportError> {
        // Junction nodes first: a road's way has to be able to end on one.
        for junction in self.map.junctions.iter() {
            let centre = self.map.junction_centre(&junction.id);
            if let Some(centre) = centre {
                let node = self.node_at(centre)?;
                self.junction_nodes.insert(junction.id.clone(), node);
                self.trace
                    .link(junction.id.clone(), node_ref(node), Link::Exact);
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
            let ways = self.road_ways(road)?;
            // A road cut at its structures is several ways, each a part of it.
            let relation = if ways.len() > 1 {
                Link::Part
            } else {
                Link::Exact
            };
            for way in &ways {
                self.trace.link(road.id.clone(), way_ref(*way), relation);
            }
            self.ways.insert(road.id.clone(), ways);
        }

        self.add_motorway_junctions();
        self.add_furniture()?;
        self.add_restrictions();
        self.add_buildings()?;
        self.add_bridge_outlines()?;
        self.add_land_covers()?;
        Ok(())
    }

    /// Each bridge's deck as a closed way tagged `man_made=bridge`, which is how
    /// OSM draws the structure as opposed to the roads on it.
    ///
    /// Traced as the structure's outline, the way a building's outline is.
    fn add_bridge_outlines(&mut self) -> Result<(), ExportError> {
        for structure in self.map.structures.iter() {
            if structure.kind.bridge().is_none() {
                continue;
            }
            let layer = structure
                .carries
                .iter()
                .map(|span| level(self.map, &span.road, (span.from + span.to) / 2.0))
                .max()
                .unwrap_or(1);
            let ring = structure.outline.points().to_vec();
            if let Some(way) =
                self.add_closed_way(&ring, tags::bridge_outline_tags(structure, layer))?
            {
                self.trace
                    .link_as(structure.id.clone(), way_ref(way), Link::Exact, "outline");
            }
        }
        Ok(())
    }

    /// `highway=motorway_junction` on the node where a slip road leaves a motorway.
    ///
    /// A junction is one node here, so the node is the junction's: one whose arms
    /// include a motorway carriageway and a slip road that starts there. It is named
    /// after the interchange the junction belongs to, which is what the sign at a
    /// motorway exit says. A node that already says something else under `highway`
    /// keeps it.
    fn add_motorway_junctions(&mut self) {
        for junction in self.map.junctions.iter() {
            let Some(node) = self.junction_nodes.get(&junction.id).copied() else {
                continue;
            };
            let arms: Vec<&Road> = junction
                .incoming_roads
                .iter()
                .filter_map(|road| self.map.road(road))
                .collect();
            let motorway = arms.iter().any(|road| tags::is_motorway(road));
            let leaving = arms.iter().any(|road| {
                road.form == FormOfWay::SlipRoad
                    && end_facing(road, &junction.id) == Some(RoadEnd::Start)
            });
            if !(motorway && leaving) {
                continue;
            }
            let name = junction
                .interchange
                .as_ref()
                .and_then(|id| self.map.interchanges.get(id))
                .and_then(|interchange| interchange.name.clone());
            if let Some(node) = self.document.nodes.get_mut(&node) {
                if node.tags.contains_key("highway") {
                    continue;
                }
                node.tags
                    .insert("highway".into(), "motorway_junction".into());
                if let Some(name) = name {
                    node.tags.insert("name".into(), name);
                    if let Some(interchange) = &junction.interchange {
                        self.trace.link_as(
                            interchange.clone(),
                            node_ref(node.id),
                            Link::Merged,
                            "motorway_junction",
                        );
                    }
                }
            }
        }
    }

    /// Every building, as a closed way, with a way per part where there is more than
    /// one.
    ///
    /// The outline goes down as it stands: OSM wants the ring anticlockwise and
    /// closed by repeating its first node, and a [`Footprint`](roadgen_core::Footprint)
    /// is already the first of those and one node short of the second.
    fn add_buildings(&mut self) -> Result<(), ExportError> {
        for building in self.map.buildings.iter() {
            let parts = self.map.parts_of(&building.id);
            // The building's own outline is the outline of the part that meets the
            // ground: OSM wants one way around the whole thing, and for a massing
            // built upwards from a footprint that is the footprint.
            let Some(lowest) = parts
                .iter()
                .min_by(|a, b| a.solid.base_height().total_cmp(&b.solid.base_height()))
            else {
                continue;
            };
            let ground = lowest.solid.base_height();

            let outline = lowest.solid.footprint.points().to_vec();
            let way =
                self.add_closed_way(&outline, tags::building_tags(building, &parts, ground))?;
            if let Some(way) = way {
                self.trace
                    .link_as(building.id.clone(), way_ref(way), Link::Exact, "outline");
            }
            if parts.len() == 1 {
                // The one way is the part as much as the building: its heights and
                // roof are written on it, so the part is traced to it too.
                if let Some(way) = way {
                    self.trace
                        .link_as(parts[0].id.clone(), way_ref(way), Link::Exact, "outline");
                }
                continue;
            }
            for part in &parts {
                let ring = part.solid.footprint.points().to_vec();
                if let Some(way) =
                    self.add_closed_way(&ring, tags::building_part_tags(part, ground))?
                {
                    self.trace.link(part.id.clone(), way_ref(way), Link::Exact);
                }
            }
        }
        Ok(())
    }

    /// A closed way through `ring`, or nothing when too few of its nodes are distinct.
    ///
    /// Two buildings in a terrace share the corner nodes between them, which is
    /// welding doing what it does for roads and is what OSM expects of a terrace.
    fn add_closed_way(&mut self, ring: &[Point3], tags: Tags) -> Result<Option<i64>, ExportError> {
        let mut nodes = Vec::with_capacity(ring.len() + 1);
        for point in ring {
            nodes.push(self.node_at(*point)?);
        }
        nodes.dedup();
        if nodes.len() < 3 {
            return Ok(None);
        }
        // A closed way is one whose first node is also its last.
        nodes.push(nodes[0]);

        let id = self.take_id();
        self.document.ways.insert(id, Way { id, nodes, tags });
        Ok(Some(id))
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

    /// A road's ways: one down its whole centreline, or one for each stretch
    /// between where a structure begins and ends.
    ///
    /// OSM says a road is on a bridge by tagging the way, so the way has to stop
    /// where the bridge does. The centreline is sampled with a vertex at every
    /// structure's ends, and the ways are cut there; the vertex at a cut is one node
    /// both pieces share, which keeps the road one connected line — two pieces that
    /// only meet in the plane are, to OSM, not joined at all.
    fn road_ways(&mut self, road: &Road) -> Result<Vec<i64>, ExportError> {
        let spans = road.structure.spans();
        let cuts: Vec<f64> = spans.iter().flat_map(|span| [span.from, span.to]).collect();
        let samples = road
            .reference_line
            .samples_including(self.map.metadata.sampling, &cuts)?;
        let mut nodes = Vec::with_capacity(samples.len());
        for sample in &samples {
            nodes.push(self.node_at(sample.point)?);
        }

        // Which sample each cut fell on, and what each stretch between two cuts is
        // on: the pieces, as ranges of samples sharing their end ones.
        let index_of = |station: f64| {
            samples
                .iter()
                .enumerate()
                .min_by(|a, b| {
                    (a.1.station - station)
                        .abs()
                        .total_cmp(&(b.1.station - station).abs())
                })
                .map_or(0, |(index, _)| index)
        };
        let mut pieces: Vec<(usize, usize, Option<&StructureSpan>)> = Vec::new();
        let mut at = 0;
        for span in spans {
            let (from, to) = (index_of(span.from).max(at), index_of(span.to));
            if to <= from {
                continue;
            }
            if from > at {
                pieces.push((at, from, None));
            }
            pieces.push((from, to, Some(span)));
            at = to;
        }
        if at + 1 < nodes.len() || pieces.is_empty() {
            pieces.push((at, nodes.len() - 1, None));
        }

        let mut ways = Vec::with_capacity(pieces.len());
        let last = pieces.len() - 1;
        for (number, (from, to, span)) in pieces.into_iter().enumerate() {
            let mut piece: Vec<i64> = nodes[from..=to].to_vec();
            // An end that faces a junction is carried on to the junction's node, so
            // the arms actually meet. In the IR they stop short of each other by
            // half the junction, with the connectors covering the gap.
            for end in [RoadEnd::Start, RoadEnd::End] {
                let outer = match end {
                    RoadEnd::Start => number == 0,
                    RoadEnd::End => number == last,
                };
                if !outer {
                    continue;
                }
                let Some(RoadLinkTarget::Junction(junction)) = road.link.at(end) else {
                    continue;
                };
                let Some(node) = self.junction_nodes.get(junction).copied() else {
                    continue;
                };
                match end {
                    RoadEnd::Start => piece.insert(0, node),
                    RoadEnd::End => piece.push(node),
                }
            }
            piece.dedup();

            let mut tags = tags::road_tags(self.map, road);
            let structure = span.and_then(|span| self.map.structures.get(&span.structure));
            if let (Some(span), Some(structure)) = (span, structure) {
                let layer = level(self.map, &road.id, (span.from + span.to) / 2.0);
                tags::structure_tags(&mut tags, structure, layer);
            }
            let id = self.take_id();
            if let (Some(span), Some(structure)) = (span, structure) {
                self.trace_structure_piece(road, span, structure, id);
            }
            self.document.ways.insert(
                id,
                Way {
                    id,
                    nodes: piece,
                    tags,
                },
            );
            ways.push(id);
        }
        Ok(ways)
    }

    /// Notes that the way `way`, the piece of `road` over `span`, is tagged for
    /// `structure`: the structure is merged into it under the tag that says so, and
    /// each grade separation the bridge makes is collapsed into the `layer` it
    /// raises. Those are the separations [`level`] reads off for the piece; one
    /// further down, under the road this one passes over, counts toward the number
    /// too, but is written on that road's way as well and is traced there.
    fn trace_structure_piece(
        &mut self,
        road: &Road,
        span: &StructureSpan,
        structure: &Structure,
        way: i64,
    ) {
        let tag = match structure.kind {
            StructureKind::Bridge(_) => "bridge",
            StructureKind::Tunnel(_) => "tunnel",
        };
        self.trace
            .link_as(structure.id.clone(), way_ref(way), Link::Merged, tag);
        if structure.kind.bridge().is_none() {
            return;
        }
        for separation in self.map.grade_separations.iter() {
            if separation.over.road == road.id
                && separation.over.from < span.to
                && span.from < separation.over.to
            {
                self.trace.link_as(
                    separation.id.clone(),
                    way_ref(way),
                    Link::Collapsed,
                    "layer",
                );
            }
        }
    }

    /// Furniture goes on a way's own nodes, which is how OSM places it: a traffic
    /// light is a property of the point on the road where you stop, not an object
    /// beside it. A node is inserted at the object's own position rather than the
    /// nearest vertex, because a straight road has two of those and everything on it
    /// would otherwise pile up on one end.
    fn add_furniture(&mut self) -> Result<(), ExportError> {
        for object in self.map.objects.iter() {
            // A barrier runs along the road beside it rather than standing on a
            // point of it, so it is a way of its own, not a node of the road's.
            if let MapObjectKind::Barrier { kind, height } = &object.kind {
                if let Some(way) = self.add_barrier_way(object, kind, *height)? {
                    self.trace
                        .link(object.id.clone(), way_ref(way), Link::Exact);
                }
                continue;
            }
            let Some(road) = self.road_of(object) else {
                continue;
            };
            let Some(position) = object_position(object) else {
                continue;
            };
            // A crossing is a place on the road, not a control of it, so it never
            // shares a node with a stop line or a signal however close they stand —
            // a stop line set back a few metres from the mouth with the crosswalk
            // between it and the junction is the ordinary layout, and the two land
            // well within `NODE_SPACING` of each other. Controls may share a node
            // with each other; a crossing only with another crossing.
            let is_crossing = object.kind == MapObjectKind::Crosswalk;
            let Some(node) = self.node_on_way(&road, position, |tags| {
                tags.get("highway")
                    .is_none_or(|existing| (existing == "crossing") == is_crossing)
            })?
            else {
                continue;
            };

            let (key, value) = match &object.kind {
                MapObjectKind::TrafficLight { .. } => ("highway", "traffic_signals".to_owned()),
                MapObjectKind::StopLine => ("highway", "stop".to_owned()),
                MapObjectKind::TrafficSign { code } => ("traffic_sign", code.clone()),
                MapObjectKind::Crosswalk => ("highway", "crossing".to_owned()),
                MapObjectKind::Barrier { .. } => unreachable!("written as a way above"),
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
                // Traced with its own tag as the role whether or not that tag won.
                // The node is where the object's information went either way — a
                // stop line under a signal is what `traffic_signals` means — and the
                // winner can change as later objects land on the node, so the tag the
                // node ends up with is not known yet. A sign's code varies, so its
                // role is the key it is written under.
                let role = match key {
                    "highway" => value.clone(),
                    _ => key.to_owned(),
                };
                if !keep {
                    node.tags.insert(key.into(), value);
                }
                self.trace
                    .link_as(object.id.clone(), node_ref(node.id), Link::Merged, role);
            }

            // A crossing is also a footway in its own right, so that it is a thing
            // pedestrians can be routed along rather than only a tag on the road.
            if is_crossing {
                if let Some(way) = self.add_crossing_way(object, node)? {
                    self.trace
                        .link_as(object.id.clone(), way_ref(way), Link::Exact, "crossing");
                }
            }
        }
        Ok(())
    }

    /// A barrier as a way down the middle of the band it occupies.
    fn add_barrier_way(
        &mut self,
        object: &MapObject,
        kind: &str,
        height: f64,
    ) -> Result<Option<i64>, ExportError> {
        let ObjectGeometry::Band { left, right } = &object.geometry else {
            return Ok(None);
        };
        let config = self.map.metadata.sampling;
        let (left, right) = (left.to_polyline(config)?, right.to_polyline(config)?);
        let mut nodes = Vec::with_capacity(left.len());
        for (a, b) in left.points().iter().zip(right.points()) {
            nodes.push(self.node_at(a.lerp(*b, 0.5))?);
        }
        nodes.dedup();
        if nodes.len() < 2 {
            return Ok(None);
        }
        let id = self.take_id();
        self.document.ways.insert(
            id,
            Way {
                id,
                nodes,
                tags: tags::barrier_tags(kind, height),
            },
        );
        Ok(Some(id))
    }

    /// Every land cover, as a closed way tagged for what it is.
    fn add_land_covers(&mut self) -> Result<(), ExportError> {
        for cover in self.map.land_covers.iter() {
            let site = cover
                .site
                .as_ref()
                .and_then(|site| self.map.sites.get(site));
            let ring = cover.area.points().to_vec();
            if let Some(way) = self.add_closed_way(&ring, tags::land_cover_tags(cover, site))? {
                self.trace.link(cover.id.clone(), way_ref(way), Link::Exact);
            }
        }
        Ok(())
    }

    /// The footway across the road at a crossing, through `crossing_node` — the
    /// road way's own node there — so that the two ways share a vertex and a
    /// pedestrian router can step from one onto the other. Two ways that merely
    /// cross in the plane are, to OSM, a bridge.
    fn add_crossing_way(
        &mut self,
        object: &MapObject,
        crossing_node: i64,
    ) -> Result<Option<i64>, ExportError> {
        let ObjectGeometry::Band { left, right } = &object.geometry else {
            return Ok(None);
        };
        let config = self.map.metadata.sampling;
        let (left, right) = (left.to_polyline(config)?, right.to_polyline(config)?);
        if left.len() != right.len() {
            return Ok(None);
        }
        // Down the middle of the painted strip, which is where someone crossing
        // actually walks.
        let mut nodes = Vec::with_capacity(left.len() + 1);
        for (a, b) in left.points().iter().zip(right.points()) {
            nodes.push(self.node_at(a.lerp(*b, 0.5))?);
        }
        nodes.dedup();
        if nodes.len() < 2 {
            return Ok(None);
        }
        // The road's node goes in where the footway passes it: after every vertex
        // that lies before it along the footway.
        if !nodes.contains(&crossing_node) {
            let first = self.positions[&nodes[0]];
            let last = self.positions[nodes.last().expect("two or more nodes")];
            let along = |point: Point3| {
                (point.x - first.x) * (last.x - first.x) + (point.y - first.y) * (last.y - first.y)
            };
            let here = along(self.positions[&crossing_node]);
            let index = nodes
                .iter()
                .take_while(|node| along(self.positions[node]) < here)
                .count();
            nodes.insert(index, crossing_node);
        }

        let id = self.take_id();
        let mut tags = Tags::new();
        tags.insert("highway".into(), "footway".into());
        tags.insert("footway".into(), "crossing".into());
        self.document.ways.insert(id, Way { id, nodes, tags });
        Ok(Some(id))
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
    fn node_on_way(
        &mut self,
        road: &RoadId,
        position: Point3,
        may_reuse: impl Fn(&Tags) -> bool,
    ) -> Result<Option<i64>, ExportError> {
        let Some(ways) = self.ways.get(road).cloned() else {
            return Ok(None);
        };
        // Which stretch of which of the road's ways the object is abeam of, and
        // where along it.
        let mut best: Option<(i64, Vec<i64>, usize, f64, f64)> = None;
        for way_id in ways {
            let Some(nodes) = self.document.ways.get(&way_id).map(|way| way.nodes.clone()) else {
                continue;
            };
            if let Some((index, fraction, distance)) = self.abeam(&nodes, position) {
                if best
                    .as_ref()
                    .is_none_or(|(_, _, _, _, previous)| distance < *previous)
                {
                    best = Some((way_id, nodes, index, fraction, distance));
                }
            }
        }
        let Some((way_id, nodes, index, fraction, _)) = best else {
            return Ok(None);
        };

        // A vertex near enough to the spot is used as it stands, if the caller lets
        // it be; only a point well between two of them needs a node of its own.
        let (a, b) = (
            self.positions[&nodes[index]],
            self.positions[&nodes[index + 1]],
        );
        let point = a.lerp(b, fraction);
        for near in [nodes[index], nodes[index + 1]] {
            if point.distance_to(self.positions[&near]) < NODE_SPACING
                && self
                    .document
                    .nodes
                    .get(&near)
                    .is_some_and(|node| may_reuse(&node.tags))
            {
                return Ok(Some(near));
            }
        }

        let node = self.node_at(point)?;
        if let Some(way) = self.document.ways.get_mut(&way_id) {
            way.nodes.insert(index + 1, node);
        }
        Ok(Some(node))
    }

    /// The stretch of a way's `nodes` nearest `position` in plan: its index, how far
    /// along it the nearest point lies, and how far that is from `position`.
    fn abeam(&self, nodes: &[i64], position: Point3) -> Option<(usize, f64, f64)> {
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
        best
    }

    /// A `no_*_turn` for every pair of arms the IR does not join.
    ///
    /// This is the direction OSM reads in: a junction permits everything, and a
    /// relation takes something away. The IR reads the other way round, so what is
    /// written here is the complement of what it holds.
    fn add_restrictions(&mut self) {
        // Each with the junction it restricts, for the trace.
        let mut relations: Vec<(JunctionId, Relation)> = Vec::new();
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
            // The piece of a road's way that touches the junction: its first when
            // the road starts there, its last when it ends there.
            let facing = |road: &Road| -> Option<i64> {
                let ways = self.ways.get(&road.id)?;
                match end_facing(road, &junction.id)? {
                    RoadEnd::Start => ways.first().copied(),
                    RoadEnd::End => ways.last().copied(),
                }
            };

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
                    let (Some(from_way), Some(to_way)) = (facing(from_road), facing(to_road))
                    else {
                        continue;
                    };

                    let mut tags = Tags::new();
                    tags.insert("type".into(), "restriction".into());
                    tags.insert("restriction".into(), restriction.into());
                    relations.push((
                        junction.id.clone(),
                        Relation {
                            id: 0, // assigned below, once the borrow of `self` has ended
                            members: vec![
                                Member {
                                    kind: MemberType::Way,
                                    reference: from_way,
                                    role: "from".into(),
                                },
                                Member {
                                    kind: MemberType::Node,
                                    reference: via,
                                    role: "via".into(),
                                },
                                Member {
                                    kind: MemberType::Way,
                                    reference: to_way,
                                    role: "to".into(),
                                },
                            ],
                            tags,
                        },
                    ));
                }
            }
        }

        for (junction, mut relation) in relations {
            relation.id = self.take_id();
            // A part of the junction rather than of any connection: it is a movement
            // the IR does not hold, and only the junction it is missing from does.
            self.trace.link_as(
                junction,
                relation_ref(relation.id),
                Link::Part,
                "restriction",
            );
            self.document.relations.insert(relation.id, relation);
        }
    }
}

// --------------------------------------------------------------------------- //
// Junction geometry
// --------------------------------------------------------------------------- //

/// How many levels above the ground a road is at `station`, as OSM's `layer`
/// counts them: 0 at grade, −1 in a tunnel, and on a bridge one more than the
/// highest thing it passes over — so a bridge over a street is 1 and a bridge over
/// that bridge 2.
///
/// The IR keeps no level, only heights, and says what passes over what in its grade
/// separations; the level is read off those, from the bottom up. A tunnel is −1
/// whatever is above it: what OSM needs is that it is below the ground, and a
/// tunnel under a tunnel is rare enough not to be worth a deeper number.
fn level(map: &Map, road: &RoadId, station: f64) -> i32 {
    level_within(map, road, station, 0)
}

/// [`level`], refusing to follow more than a few bridges down: the separations
/// cannot go round in a circle on a map that validated, and this makes sure.
fn level_within(map: &Map, road: &RoadId, station: f64, depth: usize) -> i32 {
    let Some(entry) = map.road(road) else {
        return 0;
    };
    let Some(span) = entry
        .structure
        .spans()
        .iter()
        .find(|span| span.from - 1e-6 <= station && station <= span.to + 1e-6)
    else {
        return 0;
    };
    let Some(structure) = map.structures.get(&span.structure) else {
        return 0;
    };
    match &structure.kind {
        StructureKind::Tunnel(_) => -1,
        StructureKind::Bridge(_) => {
            if depth >= 8 {
                return 1;
            }
            let below = map
                .grade_separations
                .iter()
                .filter(|separation| {
                    separation.over.road == *road
                        && separation.over.from < span.to
                        && span.from < separation.over.to
                })
                .filter_map(|separation| match &separation.under {
                    Underneath::Road(under) => Some(level_within(
                        map,
                        &under.road,
                        (under.from + under.to) / 2.0,
                        depth + 1,
                    )),
                    Underneath::LandCover(_) => None,
                })
                .max()
                .unwrap_or(0);
            below.max(0) + 1
        }
    }
}

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
