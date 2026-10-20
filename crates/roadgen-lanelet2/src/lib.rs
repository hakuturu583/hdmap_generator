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
//!
//! ## Lanelets end at a traffic light's stop line
//!
//! Autoware expects the lanelet carrying a traffic light's regulatory element to
//! end on the element's stop line: that is where it looks for the stop. The IR's
//! lanes need not: CARLA paints its lines a little way into the junction, across
//! the connecting lanes. So a lanelet the line crosses is cut in
//! two there — the stop line stays where it is painted — and the element hangs off
//! the lanelets that end on the line, and off none before or past it. The piece
//! past the line keeps the lanelet's id and everything else the IR's lane says;
//! the piece before it is a lanelet of its own, the same lane again.
//!
//! ## Tracing ids back to the IR
//!
//! Every id here comes from a counter, in the order the primitives are built, so no
//! lanelet id can be worked out from the IR alone. The `_traced` entry points return
//! a [`Trace`] read off the same tables the export built the map from, naming each
//! lanelet, linestring and regulatory element as `lanelet:<id>`, `linestring:<id>`
//! and `regulatory_element:<id>`. A lane cut at a stop line is each of its pieces.
//! Points are left out: there are far more of them than anything else, and none has
//! an IR element of its own.

use std::collections::{BTreeMap, HashMap, HashSet};
use std::path::{Path, PathBuf};
use std::sync::Arc;

use ll2_core::attribute::AttributeMap;
use ll2_core::id::Id;
use ll2_core::lanelet::Lanelet;
use ll2_core::linestring::LineString;
use ll2_core::map::{as_lanelet, LaneletMap, Primitive};
use ll2_core::point::Point;
use ll2_core::regelem::{roles, RegElemKind, RegulatoryElement, RuleParameter, RuleParameterMap};
use ll2_io::osm::WriteParams;
use ll2_projection::{GpsPoint, LocalCartesian, Origin, ProjectionError, Projector, Utm};

use roadgen_core::geometry::{Curve3, Point3, SamplingConfig};
use roadgen_core::id::{LaneId, ObjectId, RoadId};
use roadgen_core::map::{Lane, Map, Projection};
use roadgen_core::semantics::{MapObjectKind, ObjectGeometry, TrafficRule};
use roadgen_core::topology::Direction;
use roadgen_core::trace::{IrRef, Relation, Trace};
use roadgen_core::validation::ValidatedMap;

mod error;
mod grid;
mod tags;

pub use error::ExportError;
pub use grid::MgrsGrid;

/// Positions this close together are the same vertex.
///
/// A micrometre: far below anything a generated map means to distinguish, and far
/// above the rounding of the arithmetic that produced two boundary endpoints which
/// ought to coincide exactly.
const WELD_TOLERANCE: f64 = 1e-6;

/// Builds the Lanelet2 map.
pub fn to_lanelet_map(map: &ValidatedMap) -> Result<Arc<LaneletMap>, ExportError> {
    Ok(to_lanelet_map_traced(map)?.0)
}

/// Builds the Lanelet2 map, and the trace of which IR element became which of its
/// primitives.
pub fn to_lanelet_map_traced(map: &ValidatedMap) -> Result<(Arc<LaneletMap>, Trace), ExportError> {
    Exporter::new(map)?.run()
}

/// Builds the Lanelet2 map and renders it as OSM XML.
pub fn to_osm_xml(map: &ValidatedMap) -> Result<String, ExportError> {
    Ok(to_osm_xml_traced(map)?.0)
}

/// Builds the Lanelet2 map, renders it as OSM XML, and traces it.
///
/// The OSM writer keeps every primitive's id, so the trace of the in-memory map is
/// also the trace of the XML.
pub fn to_osm_xml_traced(map: &ValidatedMap) -> Result<(String, Trace), ExportError> {
    let (lanelet_map, trace) = to_lanelet_map_traced(map)?;
    let projector = projector_for(map)?;
    let (document, problems) = ll2_io::save::from_map(&lanelet_map, projector.as_ref());
    if !problems.is_empty() {
        return Err(ExportError::Serialization(problems.join("; ")));
    }
    let xml = document.to_xml(WriteParams {
        josm_upload: false,
        josm_format_elevation: false,
    });
    Ok((xml, trace))
}

/// Writes the map as a `.osm` Lanelet2 file.
pub fn write(map: &ValidatedMap, path: impl AsRef<Path>) -> Result<(), ExportError> {
    write_traced(map, path).map(drop)
}

/// Writes the map as a `.osm` Lanelet2 file, and returns the trace of what it wrote,
/// with the file among its [`files`](Trace::files).
pub fn write_traced(map: &ValidatedMap, path: impl AsRef<Path>) -> Result<Trace, ExportError> {
    let (xml, mut trace) = to_osm_xml_traced(map)?;
    std::fs::write(path.as_ref(), xml)
        .map_err(|error| ExportError::Io(format!("{}: {error}", path.as_ref().display())))?;
    trace.files.push(PathBuf::from(path.as_ref()));
    Ok(trace)
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
    if !map.tin_surfaces.is_empty() || !map.land_covers.is_empty() {
        problems.push(format!(
            "Lanelet2 has no terrain and no land cover, so \
             the map's {} TIN surfaces and {} land covers are not written",
            map.tin_surfaces.len(),
            map.land_covers.len()
        ));
    }

    if !map.buildings.is_empty() {
        problems.push(format!(
            "Lanelet2 describes the road and what governs it, not what stands beside \
             it, so the map's {} buildings ({} parts) are not written",
            map.buildings.len(),
            map.building_parts.len()
        ));
    }

    // Lanelet2 has no tag for a bridge or a tunnel. A lanelet on a deck is a lanelet
    // whose boundaries are high, and that is all a consumer can tell: the heights
    // are what keep a motorway's lanelets apart from the street's beneath them.
    if !map.structures.is_empty() {
        problems.push(format!(
            "Lanelet2 has no tag for a bridge or a tunnel, so the map's {} structures \
             survive only as the heights of the lanelets they carry, and their decks, \
             piers and portals are not written",
            map.structures.len()
        ));
    }

    // An MGRS map has to fit inside one 100 km square. Checking the corners of every
    // lane is enough to find a map that does not, and says so before the export turns
    // it into a file whose far end is in the wrong place.
    match LocalCoordinates::for_map(map) {
        Err(error) => problems.push(error.to_string()),
        Ok(coordinates @ LocalCoordinates::Mgrs { .. }) => {
            for lane in map.lanes.iter() {
                for curve in [&lane.left_boundary, &lane.right_boundary] {
                    for point in [curve.start_point(), curve.end_point()] {
                        if let Err(error) = coordinates.of(point) {
                            problems.push(format!("lane {}: {error}", lane.id));
                            return problems;
                        }
                    }
                }
            }
        }
        Ok(LocalCoordinates::AsIs) => {}
    }

    // A shoulder, a border or a parking strip becomes no lanelet at all — see
    // `tags::lanelet_subtype` — which is a whole lane gone rather than a detail of
    // one, so it is said here the way the other exporters say what they drop.
    let mut dropped: BTreeMap<&str, usize> = BTreeMap::new();
    for lane in map.lanes.iter() {
        if tags::lanelet_subtype(lane.lane_type).is_none() {
            *dropped.entry(lane.lane_type.as_str()).or_default() += 1;
        }
    }
    if !dropped.is_empty() {
        let counts: Vec<String> = dropped
            .iter()
            .map(|(lane_type, count)| format!("{count} {lane_type}"))
            .collect();
        problems.push(format!(
            "a lanelet is a lane traffic or pedestrians move along, so the map's {} \
             lanes are not written; the road surface they occupy is not in the map",
            counts.join(", ")
        ));
    }

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
///
/// An MGRS map is written about its origin like any other, so its latitudes and
/// longitudes come from the same local projection; what MGRS changes is the metric
/// position reported alongside them, which [`local_coordinates`] works out per node.
///
/// Heights are the map's own throughout: a node's `ele` is its `z`, as it is in every
/// other export, and a map loaded back through this projector gets that `z` again —
/// which is what Autoware's MGRS, UTM and local projectors do with `ele`.
pub fn projector_for(map: &ValidatedMap) -> Result<Box<dyn Projector>, ExportError> {
    let origin = Origin::new(GpsPoint::new(
        map.metadata.origin.latitude(),
        map.metadata.origin.longitude(),
        map.metadata.origin.altitude(),
    ));
    Ok(match map.metadata.projection {
        Projection::LocalCartesian | Projection::Mgrs => {
            Box::new(OwnHeights(LocalCartesian::new(origin)))
        }
        Projection::Utm => Box::new(OwnHeights(
            Utm::new(origin, true, false)
                .map_err(|error| ExportError::Projection(error.message().to_owned()))?,
        )),
    })
}

/// A projector that keeps the map's own heights.
///
/// A local Cartesian projection's reverse hands back the *ellipsoidal* height of an
/// east/north/up point — the origin's altitude, plus `z`, plus the amount the
/// ellipsoid falls away beneath the tangent plane, which is d²/2R and already 3 m at
/// 6 km out. Written as `ele`, that puts the Lanelet2 map that far above the
/// OpenDRIVE, SUMO, ClipGT and CARLA exports of the same IR, all of which carry the
/// map's `z`, and above the plain OSM export, whose `ele` is `z` too. The
/// latitude and longitude are the projection's; the height stays the map's.
struct OwnHeights<P: Projector>(P);

impl<P: Projector> Projector for OwnHeights<P> {
    fn forward(&self, point: GpsPoint) -> Result<[f64; 3], ProjectionError> {
        let projected = self.0.forward(point)?;
        Ok([projected[0], projected[1], point.ele])
    }

    fn reverse(&self, point: [f64; 3]) -> Result<GpsPoint, ProjectionError> {
        let position = self.0.reverse(point)?;
        Ok(GpsPoint::new(position.lat, position.lon, point[2]))
    }

    fn origin(&self) -> Origin {
        self.0.origin()
    }
}

/// The MGRS square a map's coordinates are reported in, if it uses that projection.
pub fn grid_for(map: &ValidatedMap) -> Result<Option<MgrsGrid>, ExportError> {
    match map.metadata.projection {
        Projection::Mgrs => Ok(Some(MgrsGrid::containing(map.metadata.origin)?)),
        _ => Ok(None),
    }
}

/// How a node's `local_x` and `local_y` are worked out.
///
/// Public because it is the frame Autoware reads everything beside the Lanelet2
/// map in — a point-cloud map written for the same map has to agree with it point
/// for point, MGRS included.
pub enum LocalCoordinates {
    /// The map's own metres, which is what every projection but MGRS reports.
    AsIs,
    /// Metres within an MGRS square, computed for each node from its own position.
    Mgrs {
        grid: MgrsGrid,
        projector: LocalCartesian,
    },
}

impl LocalCoordinates {
    pub fn for_map(map: &ValidatedMap) -> Result<Self, ExportError> {
        match grid_for(map)? {
            None => Ok(LocalCoordinates::AsIs),
            Some(grid) => Ok(LocalCoordinates::Mgrs {
                grid,
                projector: LocalCartesian::new(Origin::new(GpsPoint::new(
                    map.metadata.origin.latitude(),
                    map.metadata.origin.longitude(),
                    map.metadata.origin.altitude(),
                ))),
            }),
        }
    }

    /// `point`'s `local_x` and `local_y`.
    pub fn of(&self, point: Point3) -> Result<(f64, f64), ExportError> {
        match self {
            LocalCoordinates::AsIs => Ok((point.x, point.y)),
            LocalCoordinates::Mgrs { grid, projector } => {
                let position = projector
                    .reverse([point.x, point.y, point.z])
                    .map_err(|error| ExportError::Projection(error.message().to_owned()))?;
                grid.locate(position)
            }
        }
    }
}

/// The tags of a lane boundary carrying `marking`.
fn boundary_attributes(marking: roadgen_core::semantics::BoundaryMarking) -> AttributeMap {
    let (kind, subtype) = tags::boundary_tags(marking.marking);
    let mut attributes = vec![("type", kind.to_owned())];
    if let Some(subtype) = subtype {
        attributes.push(("subtype", subtype.to_owned()));
    }
    if kind == "line_thin" || kind == "line_thick" {
        attributes.push(("color", marking.color.as_str().to_owned()));
    }
    tags::attributes(attributes)
}

/// Whether two boundary curves are the same line: the same vertices, to within the
/// welding tolerance.
fn same_line(a: &Curve3, b: &Curve3, config: SamplingConfig) -> Result<bool, ExportError> {
    let (a, b) = (a.to_polyline(config)?, b.to_polyline(config)?);
    Ok(a.len() == b.len()
        && a.points()
            .iter()
            .zip(b.points())
            .all(|(p, q)| p.distance_to(*q) <= WELD_TOLERANCE))
}

/// A lanelet whose end is this close to a stop line ends on it.
const ON_LINE: f64 = 0.3;

/// A stop line that crosses a lanelet's bound this close to one of its ends
/// crosses it at that end, and does not cut it.
const AT_END: f64 = 0.2;

/// How far a stop line is carried on past each of its ends, to meet the outer
/// bound of a lane it is painted just short of.
const LINE_REACH: f64 = 1.5;

/// A cut this close to a vertex of the bound is made at that vertex.
const AT_VERTEX: f64 = 0.05;

/// The type of a traffic light's lamp way, and its role in the light's regulatory
/// element: Autoware's, not Lanelet2's own.
const LIGHT_BULBS: &str = "light_bulbs";

/// Interns vertices so that coincident positions become one `Point`.
struct PointWelder {
    interned: HashMap<[i64; 3], Point>,
    next_id: i64,
    coordinates: LocalCoordinates,
}

impl PointWelder {
    fn new(first_id: i64, coordinates: LocalCoordinates) -> Self {
        PointWelder {
            interned: HashMap::new(),
            next_id: first_id,
            coordinates,
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
    fn intern(&mut self, point: Point3) -> Result<Point, ExportError> {
        if let Some(existing) = self.interned.get(&Self::key(point)) {
            return Ok(existing.clone());
        }
        let interned = self.tagged(point, Vec::new())?;
        self.interned.insert(Self::key(point), interned.clone());
        Ok(interned)
    }

    /// A `Point` of its own for this position, carrying `tags` besides its
    /// coordinates: a lamp of a traffic light, which no other way shares.
    fn tagged(
        &mut self,
        point: Point3,
        tags: Vec<(&'static str, String)>,
    ) -> Result<Point, ExportError> {
        let id = self.next_id;
        self.next_id += 1;
        // Autoware's OSM parsers read the metric position from `local_x`/`local_y`
        // rather than re-projecting the latitude and longitude, so both are written.
        // Which metres those are is the projection's business: the map's own, or the
        // position within an MGRS square.
        let (local_x, local_y) = self.coordinates.of(point)?;
        let mut attributes = vec![
            ("local_x", format!("{local_x:.6}")),
            ("local_y", format!("{local_y:.6}")),
        ];
        attributes.extend(tags);
        Ok(Point::new(
            id,
            point.x,
            point.y,
            point.z,
            tags::attributes(attributes),
        ))
    }
}

struct Exporter<'a> {
    map: &'a Map,
    lanelet_map: Arc<LaneletMap>,
    welder: PointWelder,
    next_id: i64,
    /// One linestring per cross-section edge, shared by the lanes either side of it —
    /// which is what makes two lanelets laterally adjacent.
    boundaries: HashMap<(RoadId, usize, i32), LineString>,
    /// The boundaries a lane does not share with its neighbour, because the two
    /// put the edge at different heights.
    own_boundaries: HashMap<(LaneId, i32), LineString>,
    /// The lanelet each lane became — for a lane cut at a stop line, the piece
    /// that carries on past it.
    lanelets: HashMap<LaneId, Lanelet>,
    /// The pieces cut off the front of a lane at stop lines, in the order it runs
    /// through them; the lane's lanelet in [`lanelets`](Self::lanelets) follows the
    /// last of them.
    cut_off: HashMap<LaneId, Vec<Lanelet>>,
    /// Each linestring cut at a stop line, by its id and the line's: the part
    /// before the cut and the part after it, in the linestring's own direction.
    /// Two lanelets that share a bound share its halves too.
    cuts: HashMap<(Id, Id), (LineString, LineString)>,
    objects: HashMap<ObjectId, Vec<LineString>>,
    /// The lanelet a crosswalk object became, besides its two linestrings.
    crosswalks: HashMap<ObjectId, Id>,
    /// The regulatory elements each traffic rule became, by its index in
    /// [`Map::rules`]: one, or one per stop line of a traffic light that stops at
    /// several. A rule missing here was written as nothing at all.
    regulatory_elements: BTreeMap<usize, Vec<Id>>,
    /// The lanelets a speed limit was written onto as a tag, by rule index.
    speed_limits: BTreeMap<usize, Vec<Id>>,
    /// The `light_bulbs` way of each traffic light that has lamps.
    bulbs: HashMap<ObjectId, LineString>,
}

impl<'a> Exporter<'a> {
    /// Lanelet2 ids are free-form positive integers; upstream reserves everything
    /// below 1000 for hand-written maps, so generated ids start above that.
    const FIRST_ID: Id = 1000;

    fn new(map: &'a ValidatedMap) -> Result<Self, ExportError> {
        Ok(Exporter {
            map: map.as_map(),
            lanelet_map: LaneletMap::new_map(),
            welder: PointWelder::new(Self::FIRST_ID, LocalCoordinates::for_map(map)?),
            // Points are numbered from `FIRST_ID` upwards and everything else from a
            // block above them, so that adding a vertex cannot renumber a lanelet.
            next_id: Self::FIRST_ID + 1_000_000,
            boundaries: HashMap::new(),
            own_boundaries: HashMap::new(),
            lanelets: HashMap::new(),
            cut_off: HashMap::new(),
            cuts: HashMap::new(),
            objects: HashMap::new(),
            crosswalks: HashMap::new(),
            regulatory_elements: BTreeMap::new(),
            speed_limits: BTreeMap::new(),
            bulbs: HashMap::new(),
        })
    }

    fn config(&self) -> SamplingConfig {
        self.map.metadata.sampling
    }

    fn take_id(&mut self) -> Id {
        let id = self.next_id;
        self.next_id += 1;
        id
    }

    fn run(mut self) -> Result<(Arc<LaneletMap>, Trace), ExportError> {
        self.build_boundaries()?;
        self.build_lanelets()?;
        self.build_objects()?;
        self.split_at_stop_lines()?;
        self.build_rules()?;
        self.drop_cut_lines();
        let trace = self.trace();
        Ok((self.lanelet_map, trace))
    }

    /// Folds the tables the map was built from into a trace.
    ///
    /// It walks the IR's own arenas rather than the hash maps, so two exports of the
    /// same map give the same trace, link for link.
    fn trace(&self) -> Trace {
        let mut trace = Trace::new("lanelet2");

        // A boundary between two lanelets is one linestring both refer to — the very
        // sharing that makes them neighbours — so each lane is only part of the
        // reason it exists. Counted by id, since an inverted bound is the same line.
        let mut bound_users: HashMap<Id, usize> = HashMap::new();
        for lanelet in self
            .lanelets
            .values()
            .chain(self.cut_off.values().flatten())
        {
            for bound in [lanelet.left_bound(), lanelet.right_bound()] {
                *bound_users.entry(bound.id()).or_default() += 1;
            }
        }

        for lane in self.map.lanes.iter() {
            let ir = IrRef::Lane(lane.id.clone());
            // A lane cut at a stop line is each of its lanelets, as a whole: what
            // it says of itself, each of them says.
            for lanelet in self.pieces_of(&lane.id) {
                self.trace_lanelet(&mut trace, &ir, &lanelet, &bound_users);
            }
        }

        for object in self.map.objects.iter() {
            let Some(lines) = self.objects.get(&object.id) else {
                continue;
            };
            // A band is two lines; either one alone is only a piece of the object.
            let relation = if lines.len() == 1 {
                Relation::Exact
            } else {
                Relation::Part
            };
            for line in lines {
                trace.link(
                    object.id.clone(),
                    format!("linestring:{}", line.id()),
                    relation,
                );
            }
            if let Some(crosswalk) = self.crosswalks.get(&object.id) {
                trace.link_as(
                    object.id.clone(),
                    format!("lanelet:{crosswalk}"),
                    Relation::Exact,
                    "crosswalk",
                );
            }
            // A light's lamps are a way of their own, as its crosswalk lanelet is
            // for a crosswalk: the whole light again, seen as what it shows.
            if let Some(bulbs) = self.bulbs.get(&object.id) {
                trace.link_as(
                    object.id.clone(),
                    format!("linestring:{}", bulbs.id()),
                    Relation::Exact,
                    LIGHT_BULBS,
                );
            }
        }

        // A rule written as several elements is a part of each.
        for (index, elements) in &self.regulatory_elements {
            let relation = if elements.len() == 1 {
                Relation::Exact
            } else {
                Relation::Part
            };
            for element in elements {
                trace.link(
                    IrRef::Rule(*index),
                    format!("regulatory_element:{element}"),
                    relation,
                );
            }
        }
        // A speed limit is a tag on each lanelet it covers, and no lanelet is only
        // the limit, so each is a part of how the rule was written.
        for (index, lanelets) in &self.speed_limits {
            for lanelet in lanelets {
                trace.link_as(
                    IrRef::Rule(*index),
                    format!("lanelet:{lanelet}"),
                    Relation::Part,
                    "speed_limit",
                );
            }
        }
        trace
    }

    /// The links of one lanelet a lane became: the lanelet, its centreline and
    /// its bounds. A lane cut at a stop line is several lanelets, each a part of it.
    fn trace_lanelet(
        &self,
        trace: &mut Trace,
        ir: &IrRef,
        lanelet: &Lanelet,
        bound_users: &HashMap<Id, usize>,
    ) {
        let whole = match ir {
            IrRef::Lane(lane) => !self.cut_off.get(lane).is_some_and(|cut| !cut.is_empty()),
            _ => true,
        };
        let own = if whole {
            Relation::Exact
        } else {
            Relation::Part
        };
        trace.link_as(
            ir.clone(),
            format!("lanelet:{}", lanelet.id()),
            own,
            "lanelet",
        );
        trace.link_as(
            ir.clone(),
            format!("linestring:{}", lanelet.centerline().id()),
            own,
            "centerline",
        );
        // The lanelet's bounds, which for a backward lane are the IR's swapped:
        // the role is the side the written element is on, the driver's.
        for (role, bound) in [
            ("left_boundary", lanelet.left_bound()),
            ("right_boundary", lanelet.right_bound()),
        ] {
            let relation =
                match Relation::shared_by(bound_users.get(&bound.id()).copied().unwrap_or(0)) {
                    Relation::Exact => own,
                    shared => shared,
                };
            trace.link_as(
                ir.clone(),
                format!("linestring:{}", bound.id()),
                relation,
                role,
            );
        }
    }

    /// The lanelets `lane` became, in the order it runs through them: one, or one
    /// more for each stop line it was cut at. None for a lane written as nothing.
    fn pieces_of(&self, lane: &LaneId) -> Vec<Lanelet> {
        let mut pieces = self.cut_off.get(lane).cloned().unwrap_or_default();
        pieces.extend(self.lanelets.get(lane).cloned());
        pieces
    }

    /// One linestring per cross-section edge of each road.
    ///
    /// The key is the *edge* — which cross-section, and which rank out from its
    /// origin — rather than the lateral offset, so two lanes that share a boundary
    /// keep sharing it when they taper and their offsets stop being a fixed number.
    fn build_boundaries(&mut self) -> Result<(), ExportError> {
        for road in self.map.roads.iter() {
            for section in 0..road.sections.len() {
                let lanes: Vec<Lane> = self
                    .map
                    .lanes_of_section(&road.id, section)
                    .into_iter()
                    .cloned()
                    .collect();
                // Left to right across the cross-section, so a boundary's id order
                // matches the way the road reads.
                let mut edges: Vec<(i32, &Curve3, roadgen_core::semantics::BoundaryMarking)> =
                    Vec::new();
                // An edge the two lanes either side of it put at different heights —
                // a kerb, where a raised pavement meets the carriageway — is two
                // lines, not one: each lane keeps its own.
                let mut own: Vec<(
                    LaneId,
                    i32,
                    &Curve3,
                    roadgen_core::semantics::BoundaryMarking,
                )> = Vec::new();
                for lane in &lanes {
                    for (rank, curve, marking) in [
                        (lane.left_edge, &lane.left_boundary, lane.left_marking),
                        (lane.right_edge, &lane.right_boundary, lane.right_marking),
                    ] {
                        match edges.iter().find(|(existing, _, _)| *existing == rank) {
                            None => edges.push((rank, curve, marking)),
                            Some((_, shared, _)) => {
                                if !same_line(shared, curve, self.config())? {
                                    own.push((lane.id.clone(), rank, curve, marking));
                                }
                            }
                        }
                    }
                }
                edges.sort_by_key(|edge| std::cmp::Reverse(edge.0));

                for (lane, rank, curve, marking) in own {
                    let line = self.linestring(curve, boundary_attributes(marking))?;
                    self.own_boundaries.insert((lane, rank), line);
                }
                for (rank, curve, marking) in edges {
                    let line = self.linestring(curve, boundary_attributes(marking))?;
                    self.boundaries
                        .insert((road.id.clone(), section, rank), line);
                }
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
            .collect::<Result<Vec<_>, ExportError>>()?;
        let id = self.take_id();
        let line = LineString::new(id, points, attributes);
        self.lanelet_map.add(Primitive::LineString(line.clone()));
        Ok(line)
    }

    fn boundary(&self, lane: &Lane, rank: i32) -> Result<LineString, ExportError> {
        if let Some(line) = self.own_boundaries.get(&(lane.id.clone(), rank)) {
            return Ok(line.clone());
        }
        self.boundaries
            .get(&(lane.road.clone(), lane.section, rank))
            .cloned()
            .ok_or_else(|| {
                ExportError::Unknown(format!(
                    "boundary of {} section {} at edge {rank}",
                    lane.road, lane.section
                ))
            })
    }

    fn build_lanelets(&mut self) -> Result<(), ExportError> {
        for lane in self.map.lanes.iter().cloned().collect::<Vec<_>>() {
            let Some(subtype) = tags::lanelet_subtype(lane.lane_type) else {
                continue;
            };
            let subtype = match self.map.road(&lane.road) {
                Some(road) => tags::road_subtype(subtype, road.form),
                None => subtype,
            };
            let inner_left = self.boundary(&lane, lane.left_edge)?;
            let inner_right = self.boundary(&lane, lane.right_edge)?;
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
        // A motorway or a slip road is out of town by what it is, whatever the
        // road type says about where it runs.
        self.map
            .road(&lane.road)
            .map(|road| {
                if road.form.is_controlled_access() {
                    "nonurban"
                } else {
                    road.road_type.lanelet2_location()
                }
            })
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
                MapObjectKind::TrafficLight { head } => {
                    attributes.push(("subtype", "red_yellow_green".into()));
                    if let Some(height) = head.height {
                        attributes.push(("height", format!("{height:.6}")));
                    }
                }
                MapObjectKind::Barrier { kind, height, .. } => {
                    attributes.push(("subtype", kind.clone()));
                    attributes.push(("height", format!("{height:.6}")));
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
                        self.crosswalks.insert(object.id.clone(), crosswalk.id());
                        self.lanelet_map.add(Primitive::Lanelet(crosswalk));
                    }
                    vec![left_line, right_line]
                }
            };
            if let (MapObjectKind::TrafficLight { head }, Some(light)) =
                (&object.kind, lines.first())
            {
                if !head.bulbs.is_empty() {
                    // Autoware's light_bulbs way: one point per lamp, tagged with what
                    // it shows, naming the light it belongs to.
                    let points = head
                        .bulbs
                        .iter()
                        .map(|bulb| {
                            let mut tags = vec![("color", bulb.color.as_str().to_owned())];
                            if let Some(arrow) = bulb.arrow {
                                tags.push(("arrow", arrow.as_str().to_owned()));
                            }
                            self.welder.tagged(bulb.position, tags)
                        })
                        .collect::<Result<Vec<_>, _>>()?;
                    let id = self.take_id();
                    let bulbs = LineString::new(
                        id,
                        points,
                        tags::attributes([
                            ("type", LIGHT_BULBS.to_owned()),
                            ("traffic_light_id", light.id().to_string()),
                        ]),
                    );
                    self.lanelet_map.add(Primitive::LineString(bulbs.clone()));
                    self.bulbs.insert(object.id.clone(), bulbs);
                }
            }
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

    /// Cuts each lanelet a traffic light's stop line crosses in two at the line,
    /// so that the light's element can hang off lanelets that end on it.
    ///
    /// Only the lanelets of the lanes the element is for, and the lanes either
    /// side of them along the road, are cut: those are the lanes that run up to
    /// the line, and a line painted inside a junction also lies across lanes that
    /// merely pass by it. A lanelet is cut when the line — carried on a little past
    /// its ends, to meet the outer bound of a lane it is painted just short of —
    /// crosses both its bounds away from their ends; a lanelet the line meets at
    /// an end already ends, or starts, on it.
    ///
    /// The piece past the line is the lanelet as it was, now starting on the line:
    /// it keeps the id, and everything the rules hang on the lane. The piece before
    /// it is a new lanelet with the same tags. Both have the IR's centreline, cut
    /// where the line crosses it, and share the points on the line, so the routing
    /// graph runs from one into the other.
    fn split_at_stop_lines(&mut self) -> Result<(), ExportError> {
        for rule in self.map.rules.clone() {
            let TrafficRule::TrafficLight {
                lights,
                stop_lines,
                lanes,
            } = rule
            else {
                continue;
            };
            for (stop_line, at, lights) in self.traffic_light_groups(&lights, &stop_lines, &lanes) {
                let Some(stop_line) = stop_line else {
                    continue;
                };
                if !self.writes_traffic_light(&lights, &at) {
                    continue;
                }
                let lines = self.object_lines(&stop_line)?;
                let [line] = lines.as_slice() else {
                    continue;
                };
                for lane in self.around(&at) {
                    for piece in self.pieces_of(&lane) {
                        self.cut(&lane, &piece, line)?;
                    }
                }
            }
        }
        Ok(())
    }

    /// `lanes`, and the lanes that lead into and out of each of them, each once.
    fn around(&self, lanes: &[LaneId]) -> Vec<LaneId> {
        let mut seen = HashSet::new();
        lanes
            .iter()
            .flat_map(|lane| {
                std::iter::once(lane.clone())
                    .chain(self.map.predecessors(lane))
                    .chain(self.map.successors(lane))
            })
            .filter(|lane| seen.insert(lane.clone()))
            .collect()
    }

    /// The lanelets a traffic light stopping `lanes` at `line` hangs off.
    ///
    /// For each lane, those that end on the line among its own lanelets, those of
    /// the lanes either side of it along the road ([`Self::around`]) and those of
    /// the lanes leading into them: the light is for the lane, and is obeyed where
    /// the lane stops. A lane none of whose lanelets ends on the line — one beside
    /// a line drawn across only some of the rule's lanes, or a line drawn away
    /// from every lane — still has its own lanelet hold the element, as it did
    /// before any was cut: what the rule says of it does not go unsaid.
    fn ending_on(&self, line: &LineString, lanes: &[LaneId]) -> Vec<Lanelet> {
        let line: Vec<[f64; 2]> = line.points().iter().map(plan).collect();
        let ends_on_line = |lanelet: &Lanelet| {
            let (Some(left), Some(right)) =
                (lanelet.left_bound().back(), lanelet.right_bound().back())
            else {
                return false;
            };
            let (left, right) = (plan(&left), plan(&right));
            let end = [(left[0] + right[0]) / 2.0, (left[1] + right[1]) / 2.0];
            distance_to_polyline(end, &line) < ON_LINE
        };
        let mut seen = HashSet::new();
        let mut holders = Vec::new();
        for lane in lanes {
            let near = self.around(std::slice::from_ref(lane));
            let ending: Vec<Lanelet> = near
                .iter()
                .cloned()
                .chain(near.iter().flat_map(|lane| self.map.predecessors(lane)))
                .flat_map(|lane| self.pieces_of(&lane))
                .filter(ends_on_line)
                .collect();
            let ending = if ending.is_empty() {
                self.lanelets.get(lane).cloned().into_iter().collect()
            } else {
                ending
            };
            holders.extend(
                ending
                    .into_iter()
                    .filter(|lanelet| seen.insert(lanelet.id())),
            );
        }
        holders
    }

    /// Cuts `piece`, one of `lane`'s lanelets, in two where `line` crosses it, if
    /// it does away from its ends ([`Self::split_at_stop_lines`]).
    fn cut(
        &mut self,
        lane: &LaneId,
        piece: &Lanelet,
        line: &LineString,
    ) -> Result<(), ExportError> {
        if !piece.has_custom_centerline() {
            return Ok(());
        }
        let reach = carried_on(&line.points().iter().map(plan).collect::<Vec<_>>());
        let (left, right, centre) = (piece.left_bound(), piece.right_bound(), piece.centerline());
        let crossings = [&left, &right, &centre].map(|bound| crossing(&bound.points(), &reach));
        let [Some(on_left), Some(on_right), Some(on_centre)] = crossings else {
            return Ok(());
        };
        let inside = |bound: &LineString, at: &Crossing, margin: f64| {
            at.along >= margin && polyline_length(&bound.points()) - at.along >= margin
        };
        if !inside(&left, &on_left, AT_END)
            || !inside(&right, &on_right, AT_END)
            || !inside(&centre, &on_centre, AT_VERTEX)
        {
            return Ok(());
        }

        let (left_before, left_after) = self.cut_line(&left, &on_left, line.id())?;
        let (right_before, right_after) = self.cut_line(&right, &on_right, line.id())?;
        let (centre_before, centre_after) = self.cut_line(&centre, &on_centre, line.id())?;

        let id = self.take_id();
        let before = Lanelet::new(
            id,
            left_before,
            right_before,
            piece.attributes().read().clone(),
        );
        before.set_centerline(centre_before);
        piece.set_left_bound(left_after);
        piece.set_right_bound(right_after);
        piece.set_centerline(centre_after);
        self.lanelet_map.add(Primitive::Lanelet(before.clone()));

        let cut_off = self.cut_off.entry(lane.clone()).or_default();
        let at = cut_off
            .iter()
            .position(|earlier| earlier.id() == piece.id())
            .unwrap_or(cut_off.len());
        cut_off.insert(at, before);
        Ok(())
    }

    /// `bound` cut `at` a crossing of the stop line `line`: the part before the
    /// cut and the part after it, both running the way `bound` does. A linestring
    /// is cut once per line, however many lanelets it bounds.
    fn cut_line(
        &mut self,
        bound: &LineString,
        at: &Crossing,
        line: Id,
    ) -> Result<(LineString, LineString), ExportError> {
        if !self.cuts.contains_key(&(bound.id(), line)) {
            let points = bound.points();
            let (a, b) = (&points[at.segment], &points[at.segment + 1]);
            let length = distance(plan(a), plan(b));
            let (before, after) = if at.fraction * length < AT_VERTEX {
                (
                    points[..=at.segment].to_vec(),
                    points[at.segment..].to_vec(),
                )
            } else if (1.0 - at.fraction) * length < AT_VERTEX {
                (
                    points[..=at.segment + 1].to_vec(),
                    points[at.segment + 1..].to_vec(),
                )
            } else {
                let [ax, ay, az] = a.xyz();
                let [bx, by, bz] = b.xyz();
                let f = at.fraction;
                let middle = self.welder.intern(Point3::new(
                    ax + f * (bx - ax),
                    ay + f * (by - ay),
                    az + f * (bz - az),
                ))?;
                let mut before = points[..=at.segment].to_vec();
                before.push(middle.clone());
                let mut after = vec![middle];
                after.extend_from_slice(&points[at.segment + 1..]);
                (before, after)
            };
            // Stored the way the linestring itself runs, which an inverted bound
            // does not.
            let (first, second) = if bound.is_inverted() {
                (
                    after.into_iter().rev().collect(),
                    before.into_iter().rev().collect(),
                )
            } else {
                (before, after)
            };
            let attributes = bound.attributes().read().clone();
            let halves = [first, second].map(|points| {
                let half = LineString::new(self.take_id(), points, attributes.clone());
                self.lanelet_map.add(Primitive::LineString(half.clone()));
                half
            });
            let [first, second] = halves;
            self.cuts.insert((bound.id(), line), (first, second));
        }
        let (first, second) = &self.cuts[&(bound.id(), line)];
        Ok(if bound.is_inverted() {
            (second.invert(), first.invert())
        } else {
            (first.clone(), second.clone())
        })
    }

    /// Leaves out of the map the linestrings cut at a stop line that no lanelet
    /// is bounded by any more, nor has as its centreline: their halves replaced
    /// them. A layer cannot lose a primitive, so the map is built again without
    /// them.
    fn drop_cut_lines(&mut self) {
        let mut cut: HashSet<Id> = self.cuts.keys().map(|(line, _)| *line).collect();
        for primitive in self.lanelet_map.lanelets.all() {
            if let Some(lanelet) = as_lanelet(&primitive) {
                cut.remove(&lanelet.left_bound().id());
                cut.remove(&lanelet.right_bound().id());
                if lanelet.has_custom_centerline() {
                    cut.remove(&lanelet.centerline().id());
                }
            }
        }
        if cut.is_empty() {
            return;
        }
        let map = LaneletMap::new_map();
        for layer in [
            &self.lanelet_map.points,
            &self.lanelet_map.line_strings,
            &self.lanelet_map.polygons,
            &self.lanelet_map.lanelets,
            &self.lanelet_map.areas,
            &self.lanelet_map.regulatory_elements,
        ] {
            for primitive in layer.all() {
                let dropped =
                    matches!(&primitive, Primitive::LineString(line) if cut.contains(&line.id()));
                if !dropped {
                    map.add(primitive);
                }
            }
        }
        self.lanelet_map = map;
    }

    fn build_rules(&mut self) -> Result<(), ExportError> {
        for (index, rule) in self.map.rules.clone().into_iter().enumerate() {
            match rule {
                TrafficRule::TrafficLight {
                    lights,
                    stop_lines,
                    lanes,
                } => {
                    let mut elements = Vec::new();
                    for (stop_line, at, lights) in
                        self.traffic_light_groups(&lights, &stop_lines, &lanes)
                    {
                        if let Some(element) =
                            self.traffic_light_element(&lights, stop_line.as_ref(), &at)?
                        {
                            elements.push(element);
                        }
                    }
                    if !elements.is_empty() {
                        self.regulatory_elements.insert(index, elements);
                    }
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
                    let element = self.attach(
                        RegElemKind::RightOfWay,
                        "right_of_way",
                        parameters,
                        &attached,
                    );
                    self.regulatory_elements.insert(index, vec![element]);
                }
                // Lanelet2's `SpeedLimit` is a kind of traffic sign and needs one to
                // refer to. A limit with no sign behind it belongs on the lanelet,
                // which is also where Autoware looks for it.
                TrafficRule::SpeedLimit { limit, lanes } => {
                    for lane in &lanes {
                        for lanelet in self.pieces_of(lane) {
                            let mut attributes = lanelet.attributes().read().clone();
                            attributes.insert(
                                "speed_limit".to_owned(),
                                ll2_core::attribute::Attribute::new(format!("{:.0}", limit.kph())),
                            );
                            lanelet.set_attributes(attributes);
                            self.speed_limits
                                .entry(index)
                                .or_default()
                                .push(lanelet.id());
                        }
                    }
                }
            }
        }
        Ok(())
    }

    /// The elements a traffic-light rule is written as, each as `(stop line,
    /// lanes, lights)`.
    ///
    /// A Lanelet2 traffic light has one `ref_line`, so a rule that stops at several
    /// lines — one per junction mouth — is one element per line, on the rule's
    /// lanes that stop at it (`Map::traffic_light_stops`, as the SUMO export has
    /// it) and referring to the lights over them, or to all the rule's lights when
    /// none is. A rule with no line, or one, is one element over all its lanes.
    fn traffic_light_groups(
        &self,
        lights: &[ObjectId],
        stop_lines: &[ObjectId],
        lanes: &[LaneId],
    ) -> Vec<(Option<ObjectId>, Vec<LaneId>, Vec<ObjectId>)> {
        self.map
            .traffic_light_stops(stop_lines, lanes)
            .into_iter()
            .map(|(line, at)| {
                let over: Vec<ObjectId> = lights
                    .iter()
                    .filter(|light| {
                        self.map
                            .objects
                            .get(light)
                            .is_some_and(|object| object.lanes.iter().any(|lane| at.contains(lane)))
                    })
                    .cloned()
                    .collect();
                let over = if over.is_empty() {
                    lights.to_vec()
                } else {
                    over
                };
                (line, at, over)
            })
            .collect()
    }

    /// Whether `lights` stopping `lanes` is written as a `traffic_light` element at
    /// all: not when none of the lights has a way to refer to, since Lanelet2's
    /// TrafficLight refuses to exist without one, and not when none of the lanes
    /// was written as a lanelet, since an element no lanelet holds governs nothing.
    fn writes_traffic_light(&self, lights: &[ObjectId], lanes: &[LaneId]) -> bool {
        lanes.iter().any(|lane| self.lanelets.contains_key(lane))
            && lights.iter().any(|light| {
                self.objects
                    .get(light)
                    .is_some_and(|lines| !lines.is_empty())
            })
    }

    /// One `traffic_light` element: `lights` stopping `lanes` at `stop_line`, on
    /// the lanelets that end on the line ([`Self::ending_on`]), or on the lanes'
    /// own lanelets when there is no line. `None` when it is not written at all
    /// ([`Self::writes_traffic_light`]).
    fn traffic_light_element(
        &mut self,
        lights: &[ObjectId],
        stop_line: Option<&ObjectId>,
        lanes: &[LaneId],
    ) -> Result<Option<Id>, ExportError> {
        if !self.writes_traffic_light(lights, lanes) {
            return Ok(None);
        }
        let mut parameters = RuleParameterMap::new();
        let refers: Vec<RuleParameter> = lights
            .iter()
            .map(|light| self.object_lines(light))
            .collect::<Result<Vec<_>, _>>()?
            .into_iter()
            .flatten()
            .map(RuleParameter::LineString)
            .collect();
        parameters.insert(roles::REFERS.to_owned(), refers);
        let bulbs: Vec<RuleParameter> = lights
            .iter()
            .filter_map(|light| self.bulbs.get(light).cloned())
            .map(RuleParameter::LineString)
            .collect();
        if !bulbs.is_empty() {
            parameters.insert(LIGHT_BULBS.to_owned(), bulbs);
        }
        let mut holders = None;
        if let Some(stop_line) = stop_line {
            let lines = self.object_lines(stop_line)?;
            if let [line] = lines.as_slice() {
                holders = Some(self.ending_on(line, lanes));
            }
            parameters.insert(
                roles::REF_LINE.to_owned(),
                lines.into_iter().map(RuleParameter::LineString).collect(),
            );
        }
        let holders = holders.unwrap_or_else(|| {
            lanes
                .iter()
                .filter_map(|lane| self.lanelets.get(lane).cloned())
                .collect()
        });
        Ok(Some(self.attach_to(
            RegElemKind::TrafficLight,
            "traffic_light",
            parameters,
            &holders,
        )))
    }

    fn attach(
        &mut self,
        kind: RegElemKind,
        subtype: &str,
        parameters: RuleParameterMap,
        lanes: &[LaneId],
    ) -> Id {
        let lanelets: Vec<Lanelet> = lanes
            .iter()
            .filter_map(|lane| self.lanelets.get(lane).cloned())
            .collect();
        self.attach_to(kind, subtype, parameters, &lanelets)
    }

    /// A new regulatory element, held by each of `lanelets`.
    fn attach_to(
        &mut self,
        kind: RegElemKind,
        subtype: &str,
        parameters: RuleParameterMap,
        lanelets: &[Lanelet],
    ) -> Id {
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
        for lanelet in lanelets {
            lanelet.add_regulatory_element(element.clone());
        }
        self.lanelet_map.add(Primitive::RegulatoryElement(element));
        id
    }
}

/// Where a stop line crosses a bound: on which of its segments, how far along that
/// segment, and how far along the bound.
struct Crossing {
    segment: usize,
    fraction: f64,
    along: f64,
}

/// A point's position on the ground.
fn plan(point: &Point) -> [f64; 2] {
    [point.x(), point.y()]
}

fn distance(a: [f64; 2], b: [f64; 2]) -> f64 {
    (a[0] - b[0]).hypot(a[1] - b[1])
}

fn polyline_length(points: &[Point]) -> f64 {
    points
        .windows(2)
        .map(|pair| distance(plan(&pair[0]), plan(&pair[1])))
        .sum()
}

/// The first place, walking `bound` from its start, where `line` crosses it.
fn crossing(bound: &[Point], line: &[[f64; 2]]) -> Option<Crossing> {
    let mut along = 0.0;
    for (segment, pair) in bound.windows(2).enumerate() {
        let (a, b) = (plan(&pair[0]), plan(&pair[1]));
        for across in line.windows(2) {
            if let Some(fraction) = intersection(a, b, across[0], across[1]) {
                return Some(Crossing {
                    segment,
                    fraction,
                    along: along + fraction * distance(a, b),
                });
            }
        }
        along += distance(a, b);
    }
    None
}

/// How far along `p`–`q` the segment `a`–`b` crosses it, if it does.
fn intersection(p: [f64; 2], q: [f64; 2], a: [f64; 2], b: [f64; 2]) -> Option<f64> {
    let r = [q[0] - p[0], q[1] - p[1]];
    let s = [b[0] - a[0], b[1] - a[1]];
    let denominator = r[0] * s[1] - r[1] * s[0];
    if denominator.abs() < 1e-12 {
        return None;
    }
    let t = ((a[0] - p[0]) * s[1] - (a[1] - p[1]) * s[0]) / denominator;
    let u = ((a[0] - p[0]) * r[1] - (a[1] - p[1]) * r[0]) / denominator;
    ((0.0..=1.0).contains(&t) && (0.0..=1.0).contains(&u)).then_some(t)
}

/// `line` carried on [`LINE_REACH`] past each of its ends.
fn carried_on(line: &[[f64; 2]]) -> Vec<[f64; 2]> {
    let [first, second, ..] = line else {
        return line.to_vec();
    };
    let [.., before_last, last] = line else {
        unreachable!("two points or more");
    };
    let away = |end: [f64; 2], from: [f64; 2]| {
        let length = distance(end, from).max(1e-12);
        [
            end[0] + (end[0] - from[0]) / length * LINE_REACH,
            end[1] + (end[1] - from[1]) / length * LINE_REACH,
        ]
    };
    let mut carried = line.to_vec();
    carried[0] = away(*first, *second);
    let end = carried.len() - 1;
    carried[end] = away(*last, *before_last);
    carried
}

/// How far `point` is from the nearest part of `line`.
fn distance_to_polyline(point: [f64; 2], line: &[[f64; 2]]) -> f64 {
    line.windows(2)
        .map(|segment| {
            let (a, b) = (segment[0], segment[1]);
            let along = [b[0] - a[0], b[1] - a[1]];
            let squared = along[0] * along[0] + along[1] * along[1];
            let t = if squared > 0.0 {
                (((point[0] - a[0]) * along[0] + (point[1] - a[1]) * along[1]) / squared)
                    .clamp(0.0, 1.0)
            } else {
                0.0
            };
            distance(point, [a[0] + t * along[0], a[1] + t * along[1]])
        })
        .fold(f64::INFINITY, f64::min)
}
