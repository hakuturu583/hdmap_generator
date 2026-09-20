//! Lowers a [`ValidatedMap`] onto an OpenDRIVE document.
//!
//! Neither the OpenDRIVE schema nor its XML serializer is reimplemented here. The
//! [`opendrive`] crate owns the 1.7 data model and the writer; this crate is the thin
//! layer that decides which IR object becomes which OpenDRIVE element:
//!
//! ```text
//!   Road              → <road>
//!   3D reference line → <planView> + <elevationProfile>
//!   Lane              → <laneSection>/<lane>
//!   Road connectivity → <link><predecessor>/<successor>
//!   Junction          → <junction>
//!   LaneConnection    → <connection>/<laneLink>
//!   Traffic light,
//!   traffic sign      → <signals>/<signal> with <validity>
//!   Stop line,
//!   crosswalk         → <objects>/<object>
//!   Building          → <objects>/<object type="building"> with an <outline> per part
//!   Right of way      → <junction>/<priority>
//! ```
//!
//! # Buildings, which belong to no road
//!
//! Every OpenDRIVE object hangs off a `<road>` and is placed in that road's own
//! `(s, t)` coordinates. A building hangs off nothing: it stands on the ground, and
//! the road it happens to be beside is a fact about the map rather than about the
//! building. The IR records that fact as a
//! [`Frontage`](roadgen_core::buildings::Frontage), so the exporter reads which road
//! a building faces instead of searching for the nearest one — and falls back to
//! searching only for a building that was put on the map without one.
//!
//! A building of several parts becomes one object with an `<outlines>` of several
//! `<outline>`s, one per part, each corner carrying its own `z` for where the part
//! starts and its own `height` for how far the walls rise. That is the whole of the
//! IR's massing. What OpenDRIVE has no way to say is the *roof*: there is no ridge
//! in an outline, so a pitched roof is written as the height it reaches and nothing
//! more, and [`check`] says how many were flattened that way.
//!
//! The outline is written as `<cornerLocal>`, not `<cornerRoad>`. Both would place
//! the corners; only the first keeps them rigid. A `cornerRoad` corner is its own
//! `(s, t)` pair, so beside a bend a straight wall is written as a curved one, and a
//! consumer evaluating the file back gets a banana. `cornerLocal` measures every
//! corner in one frame — the road's, at the building's own station — so the shape
//! that comes back is the shape that went in.
//!
//! Format-specific decisions stay on this side of the boundary. OpenDRIVE's numeric
//! ids, its insistence that `s` be measured in the xy-plane, and its rule that a
//! split needs a junction are all handled here; none of them reaches the IR.

use std::collections::HashMap;
use std::path::Path;

use opendrive::core::additional_data::AdditionalData;
use opendrive::core::geo_reference::GeoReference;
use opendrive::core::header::Header;
use opendrive::core::OpenDrive;
use opendrive::junction::connection::Connection;
use opendrive::junction::contact_point::ContactPoint;
use opendrive::junction::controller::Controller as JunctionController;
use opendrive::junction::junction_type::JunctionType;
use opendrive::junction::lane_link::LaneLink as JunctionLaneLink;
use opendrive::junction::priority::Priority;
use opendrive::junction::Junction as OdJunction;
use opendrive::lane::center::Center;
use opendrive::lane::center_lane::CenterLane;
use opendrive::lane::lane_choice::LaneChoice;
use opendrive::lane::lane_link::LaneLink;
use opendrive::lane::lane_section::LaneSection;
use opendrive::lane::lane_type::LaneType as OdLaneType;
use opendrive::lane::lanes::Lanes;
use opendrive::lane::left::Left;
use opendrive::lane::left_lane::LeftLane;
use opendrive::lane::offset::Offset as LaneOffset;
use opendrive::lane::predecessor_successor::PredecessorSuccessor as LanePredecessorSuccessor;
use opendrive::lane::right::Right;
use opendrive::lane::right_lane::RightLane;
use opendrive::lane::road_mark::color::Color;
use opendrive::lane::road_mark::type_simplified::TypeSimplified;
use opendrive::lane::road_mark::RoadMark;
use opendrive::lane::speed::Speed as LaneSpeed;
use opendrive::lane::width::Width;
use opendrive::lane::Lane as OdLane;
use opendrive::object::corner::Corner;
use opendrive::object::corner_local::CornerLocal;
use opendrive::object::lane_validity::LaneValidity;
use opendrive::object::objects::Objects;
use opendrive::object::orientation::{ObjectType, Orientation};
use opendrive::object::outline::Outline;
use opendrive::object::outlines::Outlines;
use opendrive::object::Object;
use opendrive::road::country_code::CountryCode;
use opendrive::road::element_type::ElementType;
use opendrive::road::geometry::arc::Arc as OdArc;
use opendrive::road::geometry::geometry_type::GeometryType;
use opendrive::road::geometry::line::Line as OdLine;
use opendrive::road::geometry::plan_view::PlanView;
use opendrive::road::geometry::spiral::Spiral as OdSpiral;
use opendrive::road::geometry::Geometry;
use opendrive::road::link::Link;
use opendrive::road::predecessor_successor::PredecessorSuccessor;
use opendrive::road::profile::elevation::Elevation;
use opendrive::road::profile::lateral_profile::LateralProfile;
use opendrive::road::profile::super_elevation::SuperElevation;
use opendrive::road::profile::ElevationProfile;
use opendrive::road::road_type::RoadType as OdRoadType;
use opendrive::road::road_type_e::RoadTypeE;
use opendrive::road::rule::Rule;
use opendrive::road::speed::{MaxSpeed, Speed as RoadSpeed};
use opendrive::road::unit::SpeedUnit;
use opendrive::road::unit::Unit;
use opendrive::road::Road as OdRoad;
use opendrive::signal::control::Control;
use opendrive::signal::controller::Controller;
use opendrive::signal::position::inertial::PositionInertial;
use opendrive::signal::position::Position;
use opendrive::signal::signals::Signals;
use opendrive::signal::Signal;
use uom::si::angle::radian;
use uom::si::curvature::radian_per_meter;
use uom::si::f64::{Angle, Curvature, Length};
use uom::si::length::meter;
use vec1::Vec1;

use ll2_projection::utmups;
use roadgen_core::buildings::{Building, BuildingPart};
use roadgen_core::geometry::{Curve3, Point3, Sample};
use roadgen_core::id::ObjectId;
use roadgen_core::id::{BuildingId, JunctionId, LaneId, RoadId};
use roadgen_core::map::{Lane, Map, Projection, Road, TrafficHandedness};
use roadgen_core::semantics::{
    LaneType, MapObject, MapObjectKind, MarkingColor, ObjectGeometry, RoadMarking, RoadType,
    TrafficRule,
};
use roadgen_core::topology::{Direction, LaneEnd, LateralSide, RoadEnd, RoadLinkTarget};
use roadgen_core::units::GeoOrigin;
use roadgen_core::validation::ValidatedMap;
use roadgen_core::GeometryError;

pub mod controllers;
mod error;
pub mod options;
pub mod road_coordinates;

pub use controllers::{signal_groups, SignalGroup};
pub use error::ExportError;
pub use options::{Options, SignalCatalogue, SignalPlacement};
pub use road_coordinates::RoadPosition;

/// Width of a painted lane marking, metres. OpenDRIVE wants a number; this is the
/// usual one, and callers who care can post-process.
const MARKING_WIDTH: f64 = 0.13;

/// How far a stop line reaches along the road, metres — the width of the paint.
const STOP_LINE_DEPTH: f64 = 0.4;

/// The signal `type` a traffic light is written with.
///
/// OpenDRIVE identifies a signal by a code from a *country's* catalogue rather than by
/// a name of its own, so there is no universal value to use. This is the German
/// catalogue's three-colour light, which is what OpenDRIVE tooling expects to find in
/// a generated map; a caller with a different catalogue can rewrite the `type` after
/// export.
const TRAFFIC_LIGHT_TYPE: &str = "1000001";
const TRAFFIC_LIGHT_SUBTYPE: &str = "-1";

/// Turns a validated map into an OpenDRIVE document.
pub fn to_opendrive(map: &ValidatedMap) -> Result<OpenDrive, ExportError> {
    to_opendrive_with(map, &Options::default())
}

/// Turns a validated map into an OpenDRIVE document, with what the caller knows
/// beyond the map. See [`Options`].
pub fn to_opendrive_with(map: &ValidatedMap, options: &Options) -> Result<OpenDrive, ExportError> {
    Exporter::new(map, options).run()
}

/// Turns a validated map into OpenDRIVE XML.
pub fn to_xml(map: &ValidatedMap) -> Result<String, ExportError> {
    to_xml_with(map, &Options::default())
}

/// Turns a validated map into OpenDRIVE XML, with [`Options`].
pub fn to_xml_with(map: &ValidatedMap, options: &Options) -> Result<String, ExportError> {
    to_opendrive_with(map, options)?
        .to_xml_string()
        .map_err(|error| ExportError::Serialization(error.to_string()))
}

/// Writes a validated map to an `.xodr` file.
pub fn write(map: &ValidatedMap, path: impl AsRef<Path>) -> Result<(), ExportError> {
    write_with(map, path, &Options::default())
}

/// Writes a validated map to an `.xodr` file, with [`Options`].
pub fn write_with(
    map: &ValidatedMap,
    path: impl AsRef<Path>,
    options: &Options,
) -> Result<(), ExportError> {
    let xml = to_xml_with(map, options)?;
    std::fs::write(path.as_ref(), xml)
        .map_err(|error| ExportError::Io(format!("{}: {error}", path.as_ref().display())))
}

/// A `<geoReference>` that names the map's origin as `+lat_0`/`+lon_0`, whatever the
/// projection.
///
/// For a `local_cartesian` or `mgrs` map this is what the exporter writes anyway.
/// For a `utm` map the exporter writes the zone's transverse Mercator with the
/// origin folded into the false origin, which PROJ reads exactly but which puts the
/// zone's central meridian in `+lon_0` — and a consumer that reads only those two
/// numbers as "where `(0, 0)` is" (CARLA's GNSS sensor) would be a few degrees out.
/// This string is for that consumer, through [`Options::geo_reference`]: it places
/// the origin right and describes a UTM map's metres only to within the grid
/// convergence.
pub fn origin_proj_string(map: &ValidatedMap) -> String {
    proj_string_about(map.metadata.origin)
}

fn proj_string_about(origin: GeoOrigin) -> String {
    format!(
        "+proj=tmerc +lat_0={} +lon_0={} +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +no_defs",
        origin.latitude(),
        origin.longitude()
    )
}

/// The id a signal is written with, which is what a consumer knows the object by.
///
/// Returns `None` for an object that is not a signal — a stop line or a crosswalk
/// is an `<object>`, numbered from the same sequence but found under a different
/// element.
pub fn signal_id(map: &ValidatedMap, object: &ObjectId) -> Option<String> {
    let map = map.as_map();
    let entry = map.objects.get(object)?;
    matches!(
        entry.kind,
        MapObjectKind::TrafficLight | MapObjectKind::TrafficSign { .. }
    )
    .then(|| Numbering::new(map).objects.get(object).cloned())
    .flatten()
}

/// The id a junction is written with.
pub fn junction_id(map: &ValidatedMap, junction: &JunctionId) -> Option<String> {
    Numbering::new(map.as_map())
        .junctions
        .get(junction)
        .cloned()
}

/// The ids a lane is written with: its road's, and its own within that road.
pub fn lane_id(map: &ValidatedMap, lane: &LaneId) -> Option<(String, i64)> {
    let numbering = Numbering::new(map.as_map());
    let entry = map.as_map().lanes.get(lane)?;
    Some((
        numbering.roads.get(&entry.road)?.clone(),
        *numbering.lanes.get(lane)?,
    ))
}

/// Constraints OpenDRIVE imposes that the IR does not.
///
/// The IR is happy for a lane to fan out into several without a junction; OpenDRIVE
/// only allows a lane more than one continuation inside one. Reporting that here,
/// rather than in `roadgen-core`, keeps the format's rules on the format's side of
/// the boundary.
pub fn check(map: &ValidatedMap) -> Vec<String> {
    let mut problems = Vec::new();

    // An `<outline>` is a ring of corners with a height each. There is no ridge in
    // it, so a roof that has one cannot be written and the volume it encloses is all
    // that survives.
    let pitched = map
        .building_parts
        .iter()
        .filter(|part| part.solid.roof.height > 0.0)
        .count();
    if pitched > 0 {
        problems.push(format!(
            "an object outline is a ring of corner heights and has no ridge in it, so \
             the {pitched} pitched roofs are written as the height they reach and not \
             as the shape they are"
        ));
    }

    for lane in map.lanes.iter() {
        let in_junction = map
            .road(&lane.road)
            .map(Road::is_connector)
            .unwrap_or(false);
        if in_junction {
            continue;
        }
        for (direction, connections) in [
            ("continuations", map.connections_from(&lane.id)),
            ("predecessors", map.connections_to(&lane.id)),
        ] {
            let direct = connections
                .iter()
                .filter(|connection| connection.junction.is_none())
                .count();
            if direct > 1 {
                problems.push(format!(
                    "lane {} has {direct} {direction} outside a junction; OpenDRIVE \
                     allows more than one only for a lane of a connecting road",
                    lane.id
                ));
            }
        }
    }
    problems
}

/// Format-specific identifiers, assigned in the IR's own iteration order so that the
/// same map always produces the same numbers.
struct Numbering {
    roads: HashMap<RoadId, String>,
    junctions: HashMap<JunctionId, String>,
    lanes: HashMap<LaneId, i64>,
    objects: HashMap<ObjectId, String>,
    /// Buildings share the object id space, numbered after the furniture.
    buildings: HashMap<BuildingId, String>,
}

impl Numbering {
    fn new(map: &Map) -> Self {
        let mut roads = HashMap::new();
        for (index, road) in map.roads.iter().enumerate() {
            roads.insert(road.id.clone(), index.to_string());
        }
        // Junctions are numbered after the roads rather than from zero, so that no
        // junction shares a number with a road. The standard keeps the two in
        // separate spaces, but CARLA does not: a road's successor is taken for a
        // junction only if no road has that number, so a junction numbered like a
        // road is a road whose lanes lead nowhere, and every vehicle that reaches
        // it is a vehicle the traffic manager removes.
        let mut junctions = HashMap::new();
        for (index, junction) in map.junctions.iter().enumerate() {
            junctions.insert(junction.id.clone(), (map.roads.len() + index).to_string());
        }
        let mut lanes = HashMap::new();
        for lane in map.lanes.iter() {
            // OpenDRIVE numbers lanes outwards from the reference line: positive to
            // the left, negative to the right, with 0 reserved for the centre.
            let id = match lane.side {
                LateralSide::Left => lane.ordinal as i64,
                LateralSide::Right => -(lane.ordinal as i64),
            };
            lanes.insert(lane.id.clone(), id);
        }
        let mut objects = HashMap::new();
        for (index, object) in map.objects.iter().enumerate() {
            objects.insert(object.id.clone(), index.to_string());
        }
        let mut buildings = HashMap::new();
        for (index, building) in map.buildings.iter().enumerate() {
            buildings.insert(building.id.clone(), (map.objects.len() + index).to_string());
        }
        Numbering {
            roads,
            junctions,
            lanes,
            objects,
            buildings,
        }
    }
}

struct Exporter<'a> {
    map: &'a Map,
    options: &'a Options,
    numbering: Numbering,
    /// The controllers, worked out once: every junction asks which are its own.
    groups: Vec<SignalGroup>,
    /// The buildings each road is written against, in the map's own order.
    ///
    /// Resolving one is a search when it names no frontage, so every building is
    /// resolved once for the whole map rather than once for every road it is not on.
    buildings: HashMap<RoadId, Vec<&'a Building>>,
}

impl<'a> Exporter<'a> {
    fn new(map: &'a ValidatedMap, options: &'a Options) -> Self {
        let map = map.as_map();
        let mut buildings: HashMap<RoadId, Vec<&Building>> = HashMap::new();
        for building in map.buildings.iter() {
            if let Some(road) = owning_road_of(map, building) {
                buildings.entry(road.id.clone()).or_default().push(building);
            }
        }
        Exporter {
            numbering: Numbering::new(map),
            groups: signal_groups(map),
            buildings,
            options,
            map,
        }
    }

    fn run(self) -> Result<OpenDrive, ExportError> {
        let mut drive = OpenDrive {
            header: self.header()?,
            road: Vec::with_capacity(self.map.roads.len()),
            controller: Vec::new(),
            junction: Vec::new(),
            junction_group: Vec::new(),
            station: Vec::new(),
            additional_data: AdditionalData::default(),
        };
        for road in self.map.roads.iter() {
            drive.road.push(self.road(road)?);
        }
        for junction in self.map.junctions.iter() {
            if let Some(element) = self.junction(&junction.id)? {
                drive.junction.push(element);
            }
        }
        for group in &self.groups {
            let mut control = Vec::with_capacity(group.lights.len());
            for light in &group.lights {
                control.push(Control {
                    signal_id: self.object_id(light)?.to_owned(),
                    r#type: None,
                });
            }
            let Ok(control) = Vec1::try_from_vec(control) else {
                continue;
            };
            drive.controller.push(Controller {
                control,
                id: group.id.clone(),
                name: Some(group.name.clone()),
                sequence: None,
                additional_data: AdditionalData::default(),
            });
        }
        Ok(drive)
    }

    fn header(&self) -> Result<Header, ExportError> {
        let mut bounds: Option<[f64; 4]> = None;
        for road in self.map.roads.iter() {
            for sample in self.samples(road)? {
                let [west, south, east, north] = bounds.unwrap_or([
                    sample.point.x,
                    sample.point.y,
                    sample.point.x,
                    sample.point.y,
                ]);
                bounds = Some([
                    west.min(sample.point.x),
                    south.min(sample.point.y),
                    east.max(sample.point.x),
                    north.max(sample.point.y),
                ]);
            }
        }
        let [west, south, east, north] = bounds.unwrap_or([0.0; 4]);
        Ok(Header {
            rev_major: 1,
            rev_minor: 7,
            name: self.map.metadata.name.clone(),
            version: Some("1.00".into()),
            date: None,
            north: Some(Length::new::<meter>(north)),
            south: Some(Length::new::<meter>(south)),
            east: Some(Length::new::<meter>(east)),
            west: Some(Length::new::<meter>(west)),
            vendor: Some("roadgen".into()),
            geo_reference: Some(GeoReference {
                proj: Some(self.proj_string()?),
                additional_data: AdditionalData::default(),
            }),
            offset: None,
            additional_data: AdditionalData::default(),
        })
    }

    /// The PROJ description of the map's coordinates.
    ///
    /// The coordinates in the file are always the map's own metres about its origin,
    /// whatever the projection — that is what keeps this file in the same frame as
    /// the SUMO, ClipGT, GPUDrive and CARLA exports — so the string here has to
    /// describe *those* metres, not the frame they were derived from.
    ///
    /// PROJ has no exact local east/north/up projection, so a local-Cartesian map is
    /// described as a transverse Mercator at the same origin with unit scale — which
    /// agrees with it to well under a millimetre over the size of a generated map,
    /// and is what consumers of OpenDRIVE expect to find here. A UTM map's metres are
    /// UTM eastings and northings less the origin's, so it is the zone's transverse
    /// Mercator with the origin's easting and northing folded into the false origin:
    /// a bare `+proj=utm` would place the file's `(0, 0)` on the equator.
    fn proj_string(&self) -> Result<String, ExportError> {
        if let Some(reference) = &self.options.geo_reference {
            return Ok(reference.clone());
        }
        let origin = self.map.metadata.origin;
        Ok(match self.map.metadata.projection {
            // MGRS changes how the *Lanelet2* export reports a node's metric
            // position; the map's own coordinates are still metres about its origin,
            // so this file describes them the same way either way.
            Projection::LocalCartesian | Projection::Mgrs => proj_string_about(origin),
            Projection::Utm => {
                let (zone, northern, easting, northing) =
                    utmups::forward(origin.latitude(), origin.longitude())
                        .map_err(|error| ExportError::Projection(error.message().to_owned()))?;
                // UTM is a transverse Mercator on the zone's central meridian at
                // scale 0.9996 with a false easting of 500 km and, south of the
                // equator, a false northing of 10 000 km. Subtracting the origin's
                // easting and northing from those false values makes the projection
                // hand back the file's own metres.
                let false_northing = if northern { 0.0 } else { 10_000_000.0 };
                format!(
                    "+proj=tmerc +lat_0=0 +lon_0={} +k=0.9996 +x_0={} +y_0={} +datum=WGS84 +units=m +no_defs",
                    utmups::central_meridian(zone),
                    500_000.0 - easting,
                    false_northing - northing
                )
            }
        })
    }

    fn samples(&self, road: &Road) -> Result<Vec<Sample>, GeometryError> {
        road.reference_line.samples(self.map.metadata.sampling)
    }

    fn road(&self, road: &Road) -> Result<OdRoad, ExportError> {
        let samples = self.samples(road)?;
        let length = road.horizontal_length()?;
        Ok(OdRoad {
            id: self.road_id(&road.id)?.to_owned(),
            junction: match &road.junction {
                Some(junction) => self.junction_id(junction)?.to_owned(),
                // OpenDRIVE spells "not part of a junction" as -1.
                None => "-1".to_owned(),
            },
            length: Length::new::<meter>(length),
            name: road.name.clone(),
            rule: Some(match self.map.metadata.handedness {
                TrafficHandedness::RightHand => Rule::RightHandTraffic,
                TrafficHandedness::LeftHand => Rule::LeftHandTraffic,
            }),
            link: self.road_link(road)?,
            r#type: vec![OdRoadType {
                speed: road.speed_limit.map(|limit| RoadSpeed {
                    max: MaxSpeed::Limit(limit.mps()),
                    unit: Some(SpeedUnit::MetersPerSecond),
                }),
                country: None,
                s: Length::new::<meter>(0.0),
                r#type: match road.road_type {
                    RoadType::Town => RoadTypeE::Town,
                    RoadType::Rural => RoadTypeE::Rural,
                    RoadType::Motorway => RoadTypeE::Motorway,
                    RoadType::LowSpeed => RoadTypeE::LowSpeed,
                    RoadType::Pedestrian => RoadTypeE::Pedestrian,
                },
                additional_data: AdditionalData::default(),
            }],
            plan_view: self.plan_view(road, &samples)?,
            elevation_profile: Some(elevation_profile(&samples)),
            lateral_profile: self.lateral_profile(road),
            lanes: self.lanes(road)?,
            objects: self.objects(road)?,
            signals: self.signals(road)?,
            surface: None,
            railroad: None,
            additional_data: AdditionalData::default(),
        })
    }

    /// The reference line as OpenDRIVE plan-view geometry.
    ///
    /// A line and an arc survive as themselves; anything else is written as the
    /// chain of straight segments the IR would sample it into, so that the OpenDRIVE
    /// file and the Lanelet2 file describe the same vertices.
    fn plan_view(&self, road: &Road, samples: &[Sample]) -> Result<PlanView, ExportError> {
        let geometry = self.geometry_entries(&road.reference_line, 0.0, samples)?;
        Ok(PlanView {
            geometry: Vec1::try_from_vec(geometry)
                .map_err(|_| ExportError::Empty("a road has no plan-view geometry".into()))?,
            additional_data: AdditionalData::default(),
        })
    }

    /// One `<geometry>` entry per piece of the reference line, starting at `offset`.
    ///
    /// A line, an arc and a clothoid each survive as themselves — OpenDRIVE has an
    /// element for all three, so nothing is approximated. A composite recurses, one
    /// entry per piece. Anything else is written as the chain of straight segments
    /// the IR would sample it into, so that the OpenDRIVE file and the Lanelet2 file
    /// describe the same vertices.
    fn geometry_entries(
        &self,
        curve: &Curve3,
        offset: f64,
        samples: &[Sample],
    ) -> Result<Vec<Geometry>, ExportError> {
        let first = samples.first().expect("a curve samples both ends");
        let entry =
            |station: f64, point: Point3, heading: f64, length: f64, kind: GeometryType| Geometry {
                hdg: Angle::new::<radian>(heading),
                length: Length::new::<meter>(length),
                s: Length::new::<meter>(station),
                x: Length::new::<meter>(point.x),
                y: Length::new::<meter>(point.y),
                r#type: kind,
                additional_data: AdditionalData::default(),
            };

        Ok(match curve {
            Curve3::Line(line) => vec![entry(
                offset,
                line.start(),
                first.tangent.heading(),
                curve.horizontal_length()?,
                GeometryType::Line(OdLine {}),
            )],
            Curve3::Arc(arc) => vec![entry(
                offset,
                arc.start(),
                arc.heading(),
                arc.horizontal_length(),
                GeometryType::Arc(OdArc {
                    curvature: Curvature::new::<radian_per_meter>(arc.curvature()),
                }),
            )],
            Curve3::Clothoid(clothoid) => vec![entry(
                offset,
                clothoid.start(),
                clothoid.heading(),
                clothoid.horizontal_length(),
                GeometryType::Spiral(OdSpiral {
                    curvature_start: Curvature::new::<radian_per_meter>(clothoid.curvature_start()),
                    curvature_end: Curvature::new::<radian_per_meter>(clothoid.curvature_end()),
                }),
            )],
            Curve3::Composite(segments) => {
                let mut entries = Vec::with_capacity(segments.len());
                let mut station = offset;
                for segment in segments {
                    let segment_samples = segment.samples(self.map.metadata.sampling)?;
                    entries.extend(self.geometry_entries(segment, station, &segment_samples)?);
                    station += segment.horizontal_length()?;
                }
                entries
            }
            _ => samples
                .windows(2)
                .map(|pair| {
                    let (here, next) = (pair[0], pair[1]);
                    // The heading of a segment is the heading of its chord, not of
                    // the vertex tangent: the segment has to end where the next one
                    // starts.
                    let heading = (next.point.y - here.point.y).atan2(next.point.x - here.point.x);
                    entry(
                        offset + here.station,
                        here.point,
                        heading,
                        next.station - here.station,
                        GeometryType::Line(OdLine {}),
                    )
                })
                .collect(),
        })
    }

    /// The road's `<lateralProfile>`, which is where superelevation goes.
    ///
    /// The IR holds the roll as one profile against station; OpenDRIVE writes the
    /// same piecewise cubic, so this is a rename rather than a computation.
    fn lateral_profile(&self, road: &Road) -> Option<LateralProfile> {
        if road.superelevation.is_zero() {
            return None;
        }
        Some(LateralProfile {
            super_elevation: road
                .superelevation
                .pieces()
                .iter()
                .map(|piece| SuperElevation {
                    a: piece.a,
                    b: piece.b,
                    c: piece.c,
                    d: piece.d,
                    s: piece.station,
                })
                .collect(),
            shape: Vec::new(),
            additional_data: AdditionalData::default(),
        })
    }

    fn road_link(&self, road: &Road) -> Result<Option<Link>, ExportError> {
        let resolve = |end: RoadEnd| -> Result<Option<PredecessorSuccessor>, ExportError> {
            Ok(match road.link.at(end) {
                None => None,
                Some(RoadLinkTarget::Road(other)) => Some(PredecessorSuccessor {
                    contact_point: Some(match other.end {
                        RoadEnd::Start => ContactPoint::Start,
                        RoadEnd::End => ContactPoint::End,
                    }),
                    element_dir: None,
                    element_id: self.road_id(&other.road)?.to_owned(),
                    element_s: None,
                    element_type: Some(ElementType::Road),
                }),
                Some(RoadLinkTarget::Junction(junction)) => Some(PredecessorSuccessor {
                    // A junction link carries no contact point: which road is meant
                    // is decided by the junction's connections.
                    contact_point: None,
                    element_dir: None,
                    element_id: self.junction_id(junction)?.to_owned(),
                    element_s: None,
                    element_type: Some(ElementType::Junction),
                }),
            })
        };
        let predecessor = resolve(RoadEnd::Start)?;
        let successor = resolve(RoadEnd::End)?;
        if predecessor.is_none() && successor.is_none() {
            return Ok(None);
        }
        Ok(Some(Link {
            predecessor,
            successor,
            additional_data: AdditionalData::default(),
        }))
    }

    fn lanes(&self, road: &Road) -> Result<Lanes, ExportError> {
        // One `<laneSection>` per IR cross-section, which is exactly what OpenDRIVE
        // wants: a section is valid for a fixed set of lanes, and a new one begins
        // wherever that set changes.
        let sections = (0..road.sections.len())
            .map(|index| self.lane_section(road, index))
            .collect::<Result<Vec<_>, ExportError>>()?;

        Ok(Lanes {
            // `<laneOffset>` displaces the whole cross-section from the reference
            // line. It is a polynomial in OpenDRIVE as it is in the IR, so a
            // connector that tapers carries its taper here too.
            lane_offset: if road.lane_offset.is_zero() {
                Vec::new()
            } else {
                road.lane_offset
                    .pieces()
                    .iter()
                    .map(|piece| LaneOffset {
                        a: piece.a,
                        b: piece.b,
                        c: piece.c,
                        d: piece.d,
                        s: piece.station,
                    })
                    .collect()
            },
            lane_section: Vec1::try_from_vec(sections)
                .map_err(|_| ExportError::Empty("a road has no lane section".into()))?,
            additional_data: AdditionalData::default(),
        })
    }

    fn lane_section(&self, road: &Road, index: usize) -> Result<LaneSection, ExportError> {
        let entry = &road.sections[index];
        let lanes = self.map.lanes_of_section(&road.id, index);
        let mut left: Vec<&Lane> = lanes
            .iter()
            .copied()
            .filter(|lane| lane.side == LateralSide::Left)
            .collect();
        let mut right: Vec<&Lane> = lanes
            .iter()
            .copied()
            .filter(|lane| lane.side == LateralSide::Right)
            .collect();
        // OpenDRIVE lists lanes from left to right, that is by descending id.
        left.sort_by_key(|lane| std::cmp::Reverse(lane.ordinal));
        right.sort_by_key(|lane| lane.ordinal);

        let centre_marking = right
            .first()
            .map(|lane| (lane.left_marking.marking, lane.left_marking.color))
            .or_else(|| {
                left.last()
                    .map(|lane| (lane.right_marking.marking, lane.right_marking.color))
            })
            .unwrap_or((RoadMarking::None, MarkingColor::White));

        Ok(LaneSection {
            s: entry.station,
            single_side: None,
            left: Vec1::try_from_vec(
                left.iter()
                    .map(|lane| {
                        Ok(LeftLane {
                            id: self.lane_id(&lane.id)?,
                            base: self.lane(lane)?,
                        })
                    })
                    .collect::<Result<Vec<_>, ExportError>>()?,
            )
            .ok()
            .map(|lane| Left {
                lane,
                additional_data: AdditionalData::default(),
            }),
            center: Center {
                lane: Vec1::new(CenterLane {
                    id: 0,
                    base: OdLane {
                        link: None,
                        choice: Vec::new(),
                        road_mark: vec![road_mark(centre_marking.0, centre_marking.1)],
                        material: Vec::new(),
                        speed: Vec::new(),
                        access: Vec::new(),
                        height: Vec::new(),
                        rule: Vec::new(),
                        level: None,
                        r#type: OdLaneType::None,
                        additional_data: AdditionalData::default(),
                    },
                }),
                additional_data: AdditionalData::default(),
            },
            right: Vec1::try_from_vec(
                right
                    .iter()
                    .map(|lane| {
                        Ok(RightLane {
                            id: self.lane_id(&lane.id)?,
                            base: self.lane(lane)?,
                        })
                    })
                    .collect::<Result<Vec<_>, ExportError>>()?,
            )
            .ok()
            .map(|lane| Right {
                lane,
                additional_data: AdditionalData::default(),
            }),
            additional_data: AdditionalData::default(),
        })
    }

    fn lane(&self, lane: &Lane) -> Result<OdLane, ExportError> {
        // The `<roadMark>` of a lane describes its outer border — the one away from
        // the reference line.
        let outer = match lane.side {
            LateralSide::Left => lane.left_marking,
            LateralSide::Right => lane.right_marking,
        };
        Ok(OdLane {
            link: self.lane_link(lane)?,
            // `<width>` is measured from the start of the lane section, so the IR's
            // profile — which counts from the start of the road — is rebased onto it.
            // Both of the IR's tapers are exactly a cubic, so nothing is approximated.
            choice: lane
                .width
                .to_poly3(lane.station_range.0)
                .pieces()
                .iter()
                .map(|piece| {
                    LaneChoice::Width(Width {
                        a: piece.a,
                        b: piece.b,
                        c: piece.c,
                        d: piece.d,
                        s_offset: Length::new::<meter>(piece.station.max(0.0)),
                    })
                })
                .collect(),
            road_mark: vec![road_mark(outer.marking, outer.color)],
            material: Vec::new(),
            speed: lane
                .speed_limit
                .map(|limit| LaneSpeed {
                    max: limit.mps(),
                    s_offset: Length::new::<meter>(0.0),
                    unit: Some(SpeedUnit::MetersPerSecond),
                })
                .into_iter()
                .collect(),
            access: Vec::new(),
            height: Vec::new(),
            rule: Vec::new(),
            level: None,
            r#type: lane_type(lane.lane_type),
            additional_data: AdditionalData::default(),
        })
    }

    /// Lane-level predecessor/successor links.
    ///
    /// A movement through a junction is *not* written here for the approach roads:
    /// OpenDRIVE expresses those in the `<junction>` element, and duplicating them
    /// on the lane would claim a continuation that does not exist. The connecting
    /// road's own lanes do link to the lanes at either side of it.
    fn lane_link(&self, lane: &Lane) -> Result<Option<LaneLink>, ExportError> {
        let is_connector = self
            .map
            .road(&lane.road)
            .map(Road::is_connector)
            .unwrap_or(false);
        let mut predecessor = Vec::new();
        let mut successor = Vec::new();

        for connection in self.map.connections_from(&lane.id) {
            if connection.junction.is_some() && !is_connector {
                continue;
            }
            let Some(other) = self.map.lanes.get(&connection.to.lane) else {
                continue;
            };
            let entry = LanePredecessorSuccessor {
                id: self.lane_id(&other.id)?,
            };
            match connection.from.end {
                LaneEnd::End => successor.push(entry),
                LaneEnd::Start => predecessor.push(entry),
            }
        }
        for connection in self.map.connections_to(&lane.id) {
            if connection.junction.is_some() && !is_connector {
                continue;
            }
            let Some(other) = self.map.lanes.get(&connection.from.lane) else {
                continue;
            };
            let entry = LanePredecessorSuccessor {
                id: self.lane_id(&other.id)?,
            };
            match connection.to.end {
                LaneEnd::End => successor.push(entry),
                LaneEnd::Start => predecessor.push(entry),
            }
        }

        if predecessor.is_empty() && successor.is_empty() {
            return Ok(None);
        }
        Ok(Some(LaneLink {
            predecessor,
            successor,
            additional_data: AdditionalData::default(),
        }))
    }

    fn junction(&self, junction: &JunctionId) -> Result<Option<OdJunction>, ExportError> {
        let Some(entry) = self.map.junction(junction) else {
            return Ok(None);
        };
        let mut connections = Vec::new();
        for (index, connector_id) in entry.connecting_roads.iter().enumerate() {
            let Some(connector) = self.map.road(connector_id) else {
                continue;
            };
            let Some(RoadLinkTarget::Road(incoming)) = &connector.link.predecessor else {
                continue;
            };
            // The connector's single lane, and the approach lane that feeds it.
            let Some(connector_lane) = connector.lanes.first() else {
                continue;
            };
            let lane_link = self
                .map
                .connections_to(connector_lane)
                .into_iter()
                .map(|connection| {
                    Ok(JunctionLaneLink {
                        from: self.lane_id(&connection.from.lane)?,
                        to: self.lane_id(connector_lane)?,
                    })
                })
                .collect::<Result<Vec<_>, ExportError>>()?;

            connections.push(Connection {
                predecessor: None,
                successor: None,
                lane_link,
                connecting_road: Some(self.road_id(connector_id)?.to_owned()),
                // Every generated connector runs forwards from the approach, so
                // traffic always enters it at its start.
                contact_point: Some(ContactPoint::Start),
                id: index.to_string(),
                incoming_road: Some(self.road_id(&incoming.road)?.to_owned()),
                linked_road: None,
                r#type: None,
            });
        }

        let Ok(connection) = Vec1::try_from_vec(connections) else {
            // A junction with nothing through it is not an OpenDRIVE junction.
            return Ok(None);
        };
        Ok(Some(OdJunction {
            connection,
            priority: self.priorities(junction)?,
            controller: self
                .groups
                .iter()
                .filter(|group| group.junction.as_ref() == Some(junction))
                .map(|group| JunctionController {
                    id: group.id.clone(),
                    sequence: None,
                    r#type: None,
                })
                .collect(),
            surface: None,
            id: self.junction_id(junction)?.to_owned(),
            main_road: None,
            name: entry.name.clone(),
            orientation: None,
            s_end: None,
            s_start: None,
            r#type: Some(JunctionType::Default),
            additional_data: AdditionalData::default(),
        }))
    }

    /// The road a map object belongs to: the road of the first lane it governs.
    ///
    /// An object that governs lanes of more than one road — a light over a whole
    /// junction mouth — is written against the first, which is the one whose
    /// coordinates it is nearest to.
    fn owning_road(&self, object: &MapObject) -> Option<&Road> {
        let lane = self.map.lanes.get(object.lanes.first()?)?;
        self.map.road(&lane.road)
    }

    /// Objects belonging to `road`, in the map's own order.
    fn objects_of<'b>(&'b self, road: &'b Road) -> impl Iterator<Item = &'b MapObject> + 'b {
        self.map.objects.iter().filter(move |object| {
            self.owning_road(object)
                .is_some_and(|owner| owner.id == road.id)
        })
    }

    /// `<validity>` naming the lanes an object governs, as a range of OpenDRIVE ids.
    fn validity(&self, object: &MapObject, road: &Road) -> Result<Vec<LaneValidity>, ExportError> {
        let mut ids: Vec<i64> = Vec::new();
        for lane in &object.lanes {
            let Some(entry) = self.map.lanes.get(lane) else {
                continue;
            };
            if entry.road != road.id {
                continue;
            }
            ids.push(self.lane_id(lane)?);
        }
        if ids.is_empty() {
            return Ok(Vec::new());
        }
        ids.sort_unstable();
        Ok(vec![LaneValidity {
            from_lane: ids[0],
            to_lane: ids[ids.len() - 1],
        }])
    }

    /// Which way along the road an object faces: the direction of the traffic it
    /// governs.
    fn orientation(&self, object: &MapObject) -> Orientation {
        match object
            .lanes
            .first()
            .and_then(|lane| self.map.lanes.get(lane))
            .map(|lane| lane.direction)
        {
            Some(Direction::Backward) => Orientation::Minus,
            _ => Orientation::Plus,
        }
    }

    /// The middle of an object's geometry, and how far it reaches across.
    fn span(&self, object: &MapObject) -> Option<(Point3, f64)> {
        let (from, to) = match &object.geometry {
            ObjectGeometry::Point(position) => (*position, *position),
            ObjectGeometry::Line(curve) => (curve.start_point(), curve.end_point()),
            // A band's diagonal, whose midpoint is the middle of the band.
            ObjectGeometry::Band { left, right } => (left.start_point(), right.end_point()),
        };
        Some((from.lerp(to, 0.5), from.distance_to(to)))
    }

    fn signals(&self, road: &Road) -> Result<Option<Signals>, ExportError> {
        let mut signals = Vec::new();
        for object in self.objects_of(road) {
            let (mut kind, mut subtype, dynamic) = match &object.kind {
                MapObjectKind::TrafficLight => (
                    TRAFFIC_LIGHT_TYPE.to_owned(),
                    TRAFFIC_LIGHT_SUBTYPE.to_owned(),
                    true,
                ),
                // A traffic sign carries the caller's own catalogue code, which is
                // exactly what OpenDRIVE's `type` is.
                MapObjectKind::TrafficSign { code } => (code.clone(), "-1".to_owned(), false),
                _ => continue,
            };
            let Some((centre, width)) = self.span(object) else {
                continue;
            };
            // Where the signal applies is the object's own geometry; where it
            // stands is the caller's to say, and the caller's word puts the post
            // where the `t` and the `zOffset` say and the `s` where the bar is.
            let placement = self.options.signals.get(&object.id);
            let applies_at = placement
                .and_then(|placement| placement.applies_at)
                .unwrap_or(centre);
            let Some(applies) = road_coordinates::locate(self.map, road, applies_at) else {
                continue;
            };
            let stands = match placement {
                Some(placement) => {
                    road_coordinates::locate(self.map, road, placement.position).unwrap_or(applies)
                }
                None => applies,
            };
            let mut country = None;
            let mut value = None;
            let mut unit = None;
            if let Some(catalogue) = placement.and_then(|placement| placement.catalogue.as_ref()) {
                kind = catalogue.kind.clone();
                subtype = catalogue.subtype.clone();
                country = catalogue.country.clone().map(CountryCode::Iso3166alpha2);
                if let Some(kph) = catalogue.speed_kph {
                    value = Some(kph);
                    unit = Some(Unit::Speed(SpeedUnit::KilometersPerHour));
                }
            }
            let heading = placement.and_then(|placement| placement.heading);
            let h_offset = match heading {
                Some(heading) => {
                    let frame = road
                        .frame_at(applies.s, self.map.metadata.sampling)
                        .map_err(ExportError::Geometry)?;
                    let tangent = frame.tangent.get();
                    Some(wrap_angle(heading - tangent.y.atan2(tangent.x)))
                }
                None => None,
            };
            let choice = placement.map(|placement| {
                Position::Inertial(PositionInertial {
                    hdg: Angle::new::<radian>(heading.unwrap_or(0.0)),
                    pitch: None,
                    roll: None,
                    x: Length::new::<meter>(placement.position.x),
                    y: Length::new::<meter>(placement.position.y),
                    z: Length::new::<meter>(placement.position.z),
                })
            });
            signals.push(Signal {
                validity: self.validity(object, road)?,
                dependency: Vec::new(),
                reference: Vec::new(),
                choice,
                country,
                country_revision: None,
                dynamic,
                height: None,
                // Typed as a length by the schema crate; the attribute is radians.
                h_offset: h_offset.map(Length::new::<meter>),
                id: self.object_id(&object.id)?.to_owned(),
                name: Some(object.id.to_string()),
                orientation: self.orientation(object),
                pitch: None,
                roll: None,
                s: Length::new::<meter>(applies.s),
                subtype,
                t: Length::new::<meter>(stands.t),
                text: None,
                r#type: kind,
                unit,
                value,
                width: (placement.is_none() && width > 0.0).then(|| Length::new::<meter>(width)),
                z_offset: Length::new::<meter>(stands.height),
                additional_data: AdditionalData::default(),
            });
        }
        if signals.is_empty() {
            return Ok(None);
        }
        Ok(Some(Signals {
            signal: signals,
            signal_reference: Vec::new(),
            additional_data: AdditionalData::default(),
        }))
    }

    fn objects(&self, road: &Road) -> Result<Option<Objects>, ExportError> {
        let mut objects = Vec::new();
        for object in self.objects_of(road) {
            let entry = match &object.kind {
                // A stop line is paint on the road surface, so it is a `roadMark`
                // object reaching a little way along the road and right across it.
                MapObjectKind::StopLine => {
                    let Some((centre, width)) = self.span(object) else {
                        continue;
                    };
                    let Some(position) = road_coordinates::locate(self.map, road, centre) else {
                        continue;
                    };
                    Object {
                        dynamic: Some(false),
                        hdg: None,
                        height: None,
                        id: self.object_id(&object.id)?.to_owned(),
                        length: Some(Length::new::<meter>(STOP_LINE_DEPTH)),
                        name: Some("stopLine".to_owned()),
                        orientation: Some(self.orientation(object)),
                        perp_to_road: None,
                        pitch: None,
                        radius: None,
                        roll: None,
                        s: Length::new::<meter>(position.s),
                        subtype: Some("stopLine".to_owned()),
                        t: Length::new::<meter>(position.t),
                        r#type: Some(ObjectType::RoadMark),
                        valid_length: None,
                        width: Some(Length::new::<meter>(width)),
                        z_offset: Length::new::<meter>(position.height),
                        repeat: Vec::new(),
                        outline: None,
                        outlines: None,
                        material: Vec::new(),
                        validity: self.validity(object, road)?,
                        parking_space: None,
                        markings: None,
                        borders: None,
                        surface: None,
                        additional_data: AdditionalData::default(),
                    }
                }
                // A crosswalk has real extent, so it gets an outline. Its corners are
                // written the way RoadRunner writes them and CARLA reads them — and
                // CARLA reads nothing else: `<cornerLocal>` about a pivot at the
                // crosswalk's centre turned a quarter turn, so that `u` runs across the
                // road and `v` along it, and the first corner repeated to close the
                // ring, which is how its tools tell one crosswalk from the next.
                MapObjectKind::Crosswalk => {
                    let ObjectGeometry::Band { left, right } = &object.geometry else {
                        continue;
                    };
                    let ring = [
                        left.start_point(),
                        left.end_point(),
                        right.end_point(),
                        right.start_point(),
                    ];
                    let positions: Vec<road_coordinates::RoadPosition> = ring
                        .iter()
                        .filter_map(|point| road_coordinates::locate(self.map, road, *point))
                        .collect();
                    let Some(centre) =
                        road_coordinates::locate(self.map, road, ring[0].lerp(ring[2], 0.5))
                    else {
                        continue;
                    };
                    if positions.len() != 4 {
                        continue;
                    }
                    // A quarter turn puts `u` along `t`, and `v` back along `s`.
                    let mut corners: Vec<Corner> = positions
                        .iter()
                        .map(|position| {
                            Corner::Local(CornerLocal {
                                height: Length::new::<meter>(0.0),
                                id: None,
                                u: Length::new::<meter>(position.t - centre.t),
                                v: Length::new::<meter>(centre.s - position.s),
                                z: Length::new::<meter>(position.height - centre.height),
                            })
                        })
                        .collect();
                    corners.push(corners[0].clone());
                    let Ok(choice) = Vec1::try_from_vec(corners) else {
                        continue;
                    };
                    let extent = |pick: fn(&road_coordinates::RoadPosition) -> f64| {
                        let values = positions.iter().map(pick);
                        values.clone().fold(f64::NEG_INFINITY, f64::max)
                            - values.fold(f64::INFINITY, f64::min)
                    };
                    Object {
                        dynamic: Some(false),
                        hdg: Some(Angle::new::<radian>(std::f64::consts::FRAC_PI_2)),
                        height: None,
                        id: self.object_id(&object.id)?.to_owned(),
                        // Length across the road, width along it: the crosswalk's own
                        // axes, which is how RoadRunner and CARLA spell them.
                        length: Some(Length::new::<meter>(extent(|p| p.t))),
                        name: Some(object.id.to_string()),
                        orientation: Some(Orientation::Plus),
                        perp_to_road: None,
                        pitch: None,
                        radius: None,
                        roll: None,
                        s: Length::new::<meter>(centre.s),
                        subtype: None,
                        t: Length::new::<meter>(centre.t),
                        r#type: Some(ObjectType::Crosswalk),
                        valid_length: None,
                        width: Some(Length::new::<meter>(extent(|p| p.s))),
                        z_offset: Length::new::<meter>(centre.height),
                        repeat: Vec::new(),
                        outline: Some(Outline {
                            closed: Some(true),
                            fill_type: None,
                            id: None,
                            lane_type: None,
                            outer: Some(true),
                            choice,
                            additional_data: AdditionalData::default(),
                        }),
                        outlines: None,
                        material: Vec::new(),
                        validity: self.validity(object, road)?,
                        parking_space: None,
                        markings: None,
                        borders: None,
                        surface: None,
                        additional_data: AdditionalData::default(),
                    }
                }
                _ => continue,
            };
            objects.push(entry);
        }
        for building in self.buildings.get(&road.id).into_iter().flatten() {
            if let Some(entry) = self.building_object(road, building)? {
                objects.push(entry);
            }
        }
        if objects.is_empty() {
            return Ok(None);
        }
        Ok(Some(Objects {
            object: objects,
            object_reference: Vec::new(),
            tunnel: Vec::new(),
            bridge: Vec::new(),
            additional_data: AdditionalData::default(),
        }))
    }

    /// `<priority>` entries for a junction, from the map's right-of-way rules.
    ///
    /// OpenDRIVE says which *connecting road* has priority over which, so a rule
    /// written over lanes becomes the pairs of connectors those lanes feed.
    fn priorities(&self, junction: &JunctionId) -> Result<Vec<Priority>, ExportError> {
        let Some(entry) = self.map.junction(junction) else {
            return Ok(Vec::new());
        };
        // Which connector each approach lane feeds, within this junction.
        let connectors_for = |lanes: &[LaneId]| -> Vec<&RoadId> {
            let mut found: Vec<&RoadId> = Vec::new();
            for lane in lanes {
                for connection in self.map.connections_from(lane) {
                    if connection.junction.as_ref() != Some(junction) {
                        continue;
                    }
                    if let Some(target) = self.map.lanes.get(&connection.to.lane) {
                        if entry.connecting_roads.contains(&target.road)
                            && !found.contains(&&target.road)
                        {
                            found.push(&target.road);
                        }
                    }
                }
            }
            found
        };

        let mut priorities = Vec::new();
        for rule in &self.map.rules {
            let TrafficRule::RightOfWay {
                right_of_way,
                yielding,
                ..
            } = rule
            else {
                continue;
            };
            for high in connectors_for(right_of_way) {
                for low in connectors_for(yielding) {
                    priorities.push(Priority {
                        high: Some(self.road_id(high)?.to_owned()),
                        low: Some(self.road_id(low)?.to_owned()),
                    });
                }
            }
        }
        Ok(priorities)
    }

    /// One building, as an object carrying an outline per part.
    ///
    /// `None` when the road has no geometry to measure against, which leaves the
    /// building out of the file rather than putting it in the wrong place.
    fn building_object(
        &self,
        road: &Road,
        building: &Building,
    ) -> Result<Option<Object>, ExportError> {
        let parts = self.map.parts_of(&building.id);
        let Some(centre) = building_centre(&parts) else {
            return Ok(None);
        };
        let Some(position) = road_coordinates::locate(self.map, road, centre) else {
            return Ok(None);
        };
        // One frame for the whole building, taken at its own station: that is what
        // makes the corners rigid and keeps its parts in line with each other, and it
        // is why `hdg` below is zero.
        //
        // The plan view's frame, not the road surface's. OpenDRIVE's `s` is measured
        // in the xy-plane and a `u`/`v` pair lives in that plane too, so the heading
        // here is the reference line's *horizontal* heading and the banking is left
        // out of it: a building is not on the road surface, and measuring it along a
        // surface tilted for a bend would put the wall somewhere the wall is not.
        let Ok(sample) = road
            .reference_line
            .sample_at(position.s, self.map.metadata.sampling)
        else {
            return Ok(None);
        };
        let origin = sample.point;
        let heading = sample.tangent.heading();
        let (cos, sin) = (heading.cos(), heading.sin());
        let plan = |point: Point3| -> [f64; 3] {
            let (dx, dy) = (point.x - origin.x, point.y - origin.y);
            [
                dx * cos + dy * sin,
                -dx * sin + dy * cos,
                point.z - origin.z,
            ]
        };
        let pivot = plan(centre);

        let mut outlines = Vec::with_capacity(parts.len());
        for (index, part) in parts.iter().enumerate() {
            let corners: Vec<Corner> = part
                .solid
                .footprint
                .points()
                .iter()
                .enumerate()
                .map(|(corner, point)| {
                    let local = plan(*point);
                    Corner::Local(CornerLocal {
                        // The walls, and then whatever the roof adds on top of them:
                        // OpenDRIVE has one number for how tall a corner is, so the
                        // shape of the roof goes and its height stays.
                        height: Length::new::<meter>(
                            part.solid.wall_height + part.solid.roof.height,
                        ),
                        id: Some(corner as u64),
                        u: Length::new::<meter>(local[0] - pivot[0]),
                        v: Length::new::<meter>(local[1] - pivot[1]),
                        z: Length::new::<meter>(local[2] - pivot[2]),
                    })
                })
                .collect();
            let Ok(choice) = Vec1::try_from_vec(corners) else {
                continue;
            };
            outlines.push(Outline {
                closed: Some(true),
                fill_type: None,
                id: Some(index as u64),
                lane_type: None,
                outer: Some(true),
                choice,
                additional_data: AdditionalData::default(),
            });
        }
        let Ok(outline) = Vec1::try_from_vec(outlines) else {
            return Ok(None);
        };

        let top = parts
            .iter()
            .map(|part| part.solid.top_height())
            .fold(f64::NEG_INFINITY, f64::max);
        let ground = parts
            .iter()
            .map(|part| part.solid.base_height())
            .fold(f64::INFINITY, f64::min);

        Ok(Some(Object {
            dynamic: Some(false),
            hdg: Some(Angle::new::<radian>(0.0)),
            height: Some(Length::new::<meter>(top - ground)),
            id: self.building_id(&building.id)?.to_owned(),
            length: None,
            name: Some(building.id.to_string()),
            orientation: Some(Orientation::None),
            perp_to_road: None,
            pitch: None,
            radius: None,
            roll: None,
            s: Length::new::<meter>(position.s),
            // The word the generator used, which OpenDRIVE's `subtype` is exactly as
            // free-form as OSM's `building` value.
            subtype: Some(building.kind.clone()),
            t: Length::new::<meter>(pivot[1]),
            r#type: Some(ObjectType::Building),
            valid_length: None,
            width: None,
            z_offset: Length::new::<meter>(pivot[2]),
            repeat: Vec::new(),
            // `<outlines>` rather than `<outline>`, because a building is what its
            // parts add up to and the plural is where the standard puts them.
            outline: None,
            outlines: Some(Outlines {
                outline,
                additional_data: AdditionalData::default(),
            }),
            material: Vec::new(),
            // A building governs no lanes, so there is nothing for a validity to say.
            validity: Vec::new(),
            parking_space: None,
            markings: None,
            borders: None,
            surface: None,
            additional_data: AdditionalData::default(),
        }))
    }

    fn building_id(&self, building: &BuildingId) -> Result<&str, ExportError> {
        self.numbering
            .buildings
            .get(building)
            .map(String::as_str)
            .ok_or_else(|| ExportError::Unknown(building.to_string()))
    }

    fn object_id(&self, object: &ObjectId) -> Result<&str, ExportError> {
        self.numbering
            .objects
            .get(object)
            .map(String::as_str)
            .ok_or_else(|| ExportError::Unknown(object.to_string()))
    }

    fn road_id(&self, road: &RoadId) -> Result<&str, ExportError> {
        self.numbering
            .roads
            .get(road)
            .map(String::as_str)
            .ok_or_else(|| ExportError::Unknown(road.to_string()))
    }

    fn junction_id(&self, junction: &JunctionId) -> Result<&str, ExportError> {
        self.numbering
            .junctions
            .get(junction)
            .map(String::as_str)
            .ok_or_else(|| ExportError::Unknown(junction.to_string()))
    }

    fn lane_id(&self, lane: &LaneId) -> Result<i64, ExportError> {
        self.numbering
            .lanes
            .get(lane)
            .copied()
            .ok_or_else(|| ExportError::Unknown(lane.to_string()))
    }
}

/// An angle brought into `(-π, π]`.
fn wrap_angle(angle: f64) -> f64 {
    let wrapped = angle.rem_euclid(std::f64::consts::TAU);
    if wrapped > std::f64::consts::PI {
        wrapped - std::f64::consts::TAU
    } else {
        wrapped
    }
}

/// The elevation of the reference line, as one linear polynomial per sampled span.
///
/// This is where the third dimension of the IR's reference line goes: OpenDRIVE
/// keeps the plan view and the elevation in separate elements, so the lowering has
/// to split what the IR holds as one 3D curve.
fn elevation_profile(samples: &[Sample]) -> ElevationProfile {
    let mut elevation = Vec::new();
    for pair in samples.windows(2) {
        let (here, next) = (pair[0], pair[1]);
        let run = next.station - here.station;
        let slope = if run.abs() > f64::EPSILON {
            (next.point.z - here.point.z) / run
        } else {
            0.0
        };
        elevation.push(Elevation {
            a: here.point.z,
            b: slope,
            c: 0.0,
            d: 0.0,
            s: here.station,
        });
    }
    if elevation.is_empty() {
        if let Some(only) = samples.first() {
            elevation.push(Elevation {
                a: only.point.z,
                b: 0.0,
                c: 0.0,
                d: 0.0,
                s: 0.0,
            });
        }
    }
    ElevationProfile {
        elevation,
        additional_data: AdditionalData::default(),
    }
}

fn road_mark(marking: RoadMarking, color: MarkingColor) -> RoadMark {
    RoadMark {
        sway: Vec::new(),
        r#type: None,
        explicit: None,
        color: match color {
            MarkingColor::White => Color::White,
            MarkingColor::Yellow => Color::Yellow,
        },
        height: None,
        lane_change: None,
        material: None,
        s_offset: Length::new::<meter>(0.0),
        type_simplified: match marking {
            RoadMarking::None => TypeSimplified::None,
            RoadMarking::Solid => TypeSimplified::Solid,
            RoadMarking::Broken => TypeSimplified::Broken,
            RoadMarking::SolidSolid => TypeSimplified::SolidSolid,
            RoadMarking::BrokenSolid => TypeSimplified::BrokenSolid,
            RoadMarking::SolidBroken => TypeSimplified::SolidBroken,
            RoadMarking::Curbstone => TypeSimplified::Curb,
        },
        weight: None,
        width: match marking {
            RoadMarking::None => None,
            _ => Some(Length::new::<meter>(MARKING_WIDTH)),
        },
        additional_data: AdditionalData::default(),
    }
}

fn lane_type(lane_type: LaneType) -> OdLaneType {
    match lane_type {
        LaneType::Driving => OdLaneType::Driving,
        LaneType::Shoulder => OdLaneType::Shoulder,
        LaneType::Border => OdLaneType::Border,
        LaneType::Sidewalk => OdLaneType::Sidewalk,
        LaneType::Biking => OdLaneType::Biking,
        LaneType::Parking => OdLaneType::Parking,
        LaneType::Restricted => OdLaneType::Restricted,
        LaneType::None => OdLaneType::None,
    }
}

/// The road a building is written against.
///
/// The one its frontage names, which is where the generator put it. A building that
/// came from somewhere else and names no road falls back to the nearest by the
/// horizontal distance from its centre to a reference line — never a junction
/// connector, because a connector is one movement through a junction and measuring a
/// building from it would move the building depending on which turn happened to be
/// closest.
fn owning_road_of<'m>(map: &'m Map, building: &Building) -> Option<&'m Road> {
    if let Some(frontage) = &building.frontage {
        if let Some(road) = map.roads.get(&frontage.road) {
            return Some(road);
        }
    }
    let centre = building_centre(&map.parts_of(&building.id))?;
    let mut best: Option<(&Road, f64)> = None;
    for road in map.roads.iter() {
        if road.is_connector() {
            continue;
        }
        let Ok(line) = road.reference_line.to_polyline(map.metadata.sampling) else {
            continue;
        };
        let distance = line
            .points()
            .iter()
            .map(|point| point.horizontal_distance_to(centre))
            .fold(f64::INFINITY, f64::min);
        if best.is_none_or(|(_, previous)| distance < previous) {
            best = Some((road, distance));
        }
    }
    best.map(|(road, _)| road)
}

/// The middle of the part of a building that meets the ground.
fn building_centre(parts: &[&BuildingPart]) -> Option<Point3> {
    parts
        .iter()
        .min_by(|a, b| a.solid.base_height().total_cmp(&b.solid.base_height()))
        .map(|part| part.solid.footprint.centroid())
}
