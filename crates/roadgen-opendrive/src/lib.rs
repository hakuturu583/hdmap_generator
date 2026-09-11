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
//! ```
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
use opendrive::junction::junction_type::JunctionType;
use opendrive::junction::lane_link::LaneLink as JunctionLaneLink;
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
use opendrive::road::element_type::ElementType;
use opendrive::road::geometry::arc::Arc as OdArc;
use opendrive::road::geometry::geometry_type::GeometryType;
use opendrive::road::geometry::line::Line as OdLine;
use opendrive::road::geometry::plan_view::PlanView;
use opendrive::road::geometry::Geometry;
use opendrive::road::link::Link;
use opendrive::road::predecessor_successor::PredecessorSuccessor;
use opendrive::road::profile::elevation::Elevation;
use opendrive::road::profile::ElevationProfile;
use opendrive::road::road_type::RoadType as OdRoadType;
use opendrive::road::road_type_e::RoadTypeE;
use opendrive::road::rule::Rule;
use opendrive::road::speed::{MaxSpeed, Speed as RoadSpeed};
use opendrive::road::unit::SpeedUnit;
use opendrive::road::Road as OdRoad;
use uom::si::angle::radian;
use uom::si::curvature::radian_per_meter;
use uom::si::f64::{Angle, Curvature, Length};
use uom::si::length::meter;
use vec1::Vec1;

use roadgen_core::geometry::{Curve3, Sample};
use roadgen_core::id::{JunctionId, LaneId, RoadId};
use roadgen_core::map::{Lane, Map, Projection, Road, TrafficHandedness};
use roadgen_core::semantics::{LaneType, MarkingColor, RoadMarking, RoadType};
use roadgen_core::topology::{LaneEnd, LateralSide, RoadEnd, RoadLinkTarget};
use roadgen_core::validation::ValidatedMap;
use roadgen_core::GeometryError;

mod error;

pub use error::ExportError;

/// Width of a painted lane marking, metres. OpenDRIVE wants a number; this is the
/// usual one, and callers who care can post-process.
const MARKING_WIDTH: f64 = 0.13;

/// Turns a validated map into an OpenDRIVE document.
pub fn to_opendrive(map: &ValidatedMap) -> Result<OpenDrive, ExportError> {
    Exporter::new(map).run()
}

/// Turns a validated map into OpenDRIVE XML.
pub fn to_xml(map: &ValidatedMap) -> Result<String, ExportError> {
    to_opendrive(map)?
        .to_xml_string()
        .map_err(|error| ExportError::Serialization(error.to_string()))
}

/// Writes a validated map to an `.xodr` file.
pub fn write(map: &ValidatedMap, path: impl AsRef<Path>) -> Result<(), ExportError> {
    let xml = to_xml(map)?;
    std::fs::write(path.as_ref(), xml)
        .map_err(|error| ExportError::Io(format!("{}: {error}", path.as_ref().display())))
}

/// Constraints OpenDRIVE imposes that the IR does not.
///
/// The IR is happy for a lane to fan out into several without a junction; OpenDRIVE
/// only allows a lane more than one continuation inside one. Reporting that here,
/// rather than in `roadgen-core`, keeps the format's rules on the format's side of
/// the boundary.
pub fn check(map: &ValidatedMap) -> Vec<String> {
    let mut problems = Vec::new();
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
}

impl Numbering {
    fn new(map: &Map) -> Self {
        let mut roads = HashMap::new();
        for (index, road) in map.roads.iter().enumerate() {
            roads.insert(road.id.clone(), index.to_string());
        }
        let mut junctions = HashMap::new();
        for (index, junction) in map.junctions.iter().enumerate() {
            junctions.insert(junction.id.clone(), index.to_string());
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
        Numbering {
            roads,
            junctions,
            lanes,
        }
    }
}

struct Exporter<'a> {
    map: &'a Map,
    numbering: Numbering,
}

impl<'a> Exporter<'a> {
    fn new(map: &'a ValidatedMap) -> Self {
        Exporter {
            numbering: Numbering::new(map.as_map()),
            map: map.as_map(),
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
                proj: Some(self.proj_string()),
                additional_data: AdditionalData::default(),
            }),
            offset: None,
            additional_data: AdditionalData::default(),
        })
    }

    /// The PROJ description of the map's coordinates.
    ///
    /// PROJ has no exact local east/north/up projection, so a local-Cartesian map is
    /// described as a transverse Mercator at the same origin with unit scale — which
    /// agrees with it to well under a millimetre over the size of a generated map,
    /// and is what consumers of OpenDRIVE expect to find here.
    fn proj_string(&self) -> String {
        let origin = self.map.metadata.origin;
        match self.map.metadata.projection {
            Projection::LocalCartesian => format!(
                "+proj=tmerc +lat_0={} +lon_0={} +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +no_defs",
                origin.latitude(),
                origin.longitude()
            ),
            Projection::Utm => {
                let zone = (((origin.longitude() + 180.0) / 6.0).floor() as i32 + 1).clamp(1, 60);
                let south = if origin.latitude() < 0.0 {
                    " +south"
                } else {
                    ""
                };
                format!("+proj=utm +zone={zone}{south} +datum=WGS84 +units=m +no_defs")
            }
        }
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
            lateral_profile: None,
            lanes: self.lanes(road)?,
            objects: None,
            signals: None,
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
        let geometry = match &road.reference_line {
            Curve3::Line(_) => {
                let first = samples.first().expect("a curve samples both ends");
                vec![Geometry {
                    hdg: Angle::new::<radian>(first.tangent.heading()),
                    length: Length::new::<meter>(road.horizontal_length()?),
                    s: Length::new::<meter>(0.0),
                    x: Length::new::<meter>(first.point.x),
                    y: Length::new::<meter>(first.point.y),
                    r#type: GeometryType::Line(OdLine {}),
                    additional_data: AdditionalData::default(),
                }]
            }
            Curve3::Arc(_) => {
                let first = samples.first().expect("a curve samples both ends");
                let last = samples.last().expect("a curve samples both ends");
                // Curvature is the turn per metre of plan-view arc length, which is
                // exactly the heading change over the sampled span.
                let turn = last.tangent.heading() - first.tangent.heading();
                let turn = (turn + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU)
                    - std::f64::consts::PI;
                let length = road.horizontal_length()?;
                vec![Geometry {
                    hdg: Angle::new::<radian>(first.tangent.heading()),
                    length: Length::new::<meter>(length),
                    s: Length::new::<meter>(0.0),
                    x: Length::new::<meter>(first.point.x),
                    y: Length::new::<meter>(first.point.y),
                    r#type: GeometryType::Arc(OdArc {
                        curvature: Curvature::new::<radian_per_meter>(turn / length),
                    }),
                    additional_data: AdditionalData::default(),
                }]
            }
            _ => samples
                .windows(2)
                .map(|pair| {
                    let (here, next) = (pair[0], pair[1]);
                    // The heading of a segment is the heading of its chord, not of
                    // the vertex tangent: the segment has to end where the next one
                    // starts.
                    let heading = (next.point.y - here.point.y).atan2(next.point.x - here.point.x);
                    Geometry {
                        hdg: Angle::new::<radian>(heading),
                        length: Length::new::<meter>(next.station - here.station),
                        s: Length::new::<meter>(here.station),
                        x: Length::new::<meter>(here.point.x),
                        y: Length::new::<meter>(here.point.y),
                        r#type: GeometryType::Line(OdLine {}),
                        additional_data: AdditionalData::default(),
                    }
                })
                .collect(),
        };
        Ok(PlanView {
            geometry: Vec1::try_from_vec(geometry)
                .map_err(|_| ExportError::Empty("a road has no plan-view geometry".into()))?,
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
        let lanes = self.map.lanes_of(&road.id);
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

        let section = LaneSection {
            s: 0.0,
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
        };

        Ok(Lanes {
            lane_offset: if road.lane_offset.abs() > f64::EPSILON {
                vec![LaneOffset {
                    a: road.lane_offset,
                    b: 0.0,
                    c: 0.0,
                    d: 0.0,
                    s: 0.0,
                }]
            } else {
                Vec::new()
            },
            lane_section: Vec1::new(section),
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
            choice: vec![LaneChoice::Width(Width {
                a: lane.width.metres(),
                b: 0.0,
                c: 0.0,
                d: 0.0,
                s_offset: Length::new::<meter>(0.0),
            })],
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
            priority: Vec::new(),
            controller: Vec::new(),
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
