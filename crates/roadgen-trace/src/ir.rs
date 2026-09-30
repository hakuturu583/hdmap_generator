//! The IR dump: every element of a map by its identifier, with what it belongs to and
//! what it connects to, and none of its geometry.
//!
//! This is what a trace needs from the IR and no more. A lookup across two formats
//! only has to know which IR element each side came from; the dump is for the cases
//! where one side has no counterpart — a junction connector SUMO folds into a
//! connection — and the lookup has to step to a neighbour instead. Geometry would make
//! the file a hundred times larger and its schema a promise about every curve type,
//! for nothing a lookup uses.
//!
//! The [fingerprint](IrDocument::fingerprint) is a digest of the dump's body. Every
//! trace file records the fingerprint of the map it was written from, so that files
//! written apart can be checked to belong together before they are joined.

use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};

use roadgen_core::map::Map;
use roadgen_core::semantics::{MapObjectKind, TrafficRule};
use roadgen_core::topology::{LaneEndpoint, RoadLinkTarget};

/// The schema a dump is written with.
pub const IR_SCHEMA: &str = "roadgen-ir/1";

/// Who wrote a file, so that a change in how ids are assigned can be told apart from
/// a change in the map.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct Generator {
    pub name: String,
    pub version: String,
}

impl Generator {
    pub fn this() -> Self {
        Generator {
            name: "roadgen".into(),
            version: env!("CARGO_PKG_VERSION").into(),
        }
    }
}

/// A dump as it is written to disk.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct IrDocument {
    pub schema: String,
    pub generator: Generator,
    /// `sha256:<hex>` of [`body`](Self::body), serialised compactly.
    pub fingerprint: String,
    #[serde(flatten)]
    pub body: IrCatalog,
}

impl IrDocument {
    /// Whether the body still hashes to the fingerprint the document states.
    ///
    /// The fingerprint is what traces are matched against, so a body edited under an
    /// unchanged fingerprint would be joined to traces of a map it no longer
    /// describes.
    pub fn check(&self, name: &str) -> Result<(), crate::TraceError> {
        let actual = self.body.fingerprint();
        if actual == self.fingerprint {
            Ok(())
        } else {
            Err(crate::TraceError::Altered {
                path: name.to_owned(),
                stated: self.fingerprint.clone(),
                actual,
            })
        }
    }

    pub fn of(map: &Map) -> Self {
        let body = IrCatalog::of(map);
        IrDocument {
            schema: IR_SCHEMA.into(),
            generator: Generator::this(),
            fingerprint: body.fingerprint(),
            body,
        }
    }
}

/// The part of a dump the fingerprint covers.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct IrCatalog {
    pub metadata: MetadataEntry,
    pub roads: Vec<RoadEntry>,
    pub lanes: Vec<LaneEntry>,
    pub junctions: Vec<JunctionEntry>,
    pub connections: Vec<ConnectionEntry>,
    pub objects: Vec<ObjectEntry>,
    pub rules: Vec<RuleEntry>,
    pub buildings: Vec<BuildingEntry>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MetadataEntry {
    pub name: Option<String>,
    pub origin: OriginEntry,
    pub projection: String,
    pub handedness: String,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct OriginEntry {
    pub latitude: f64,
    pub longitude: f64,
    pub altitude: f64,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct RoadEntry {
    pub id: String,
    pub name: Option<String>,
    #[serde(rename = "type")]
    pub road_type: String,
    /// The junction a connector road runs through; `None` for every other road.
    pub junction: Option<String>,
    pub sections: Vec<SectionEntry>,
    pub predecessor: Option<LinkEntry>,
    pub successor: Option<LinkEntry>,
    pub speed_limit_mps: Option<f64>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SectionEntry {
    pub station: f64,
    pub lanes: Vec<String>,
}

/// What a road continues into: a road's end, or a junction.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(untagged)]
pub enum LinkEntry {
    Road { road: String, end: String },
    Junction { junction: String },
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct LaneEntry {
    pub id: String,
    pub road: String,
    pub section: usize,
    /// The lane's position in its road's list, as the caller wrote it.
    pub index: usize,
    pub side: String,
    pub ordinal: usize,
    pub direction: String,
    #[serde(rename = "type")]
    pub lane_type: String,
    pub station_range: [f64; 2],
    pub speed_limit_mps: Option<f64>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct JunctionEntry {
    pub id: String,
    pub name: Option<String>,
    pub incoming: Vec<String>,
    pub connecting: Vec<String>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ConnectionEntry {
    pub id: String,
    pub from: EndpointEntry,
    pub to: EndpointEntry,
    pub junction: Option<String>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct EndpointEntry {
    pub lane: String,
    pub end: String,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ObjectEntry {
    pub id: String,
    pub kind: String,
    /// A sign's code, in whatever catalogue the caller used.
    pub code: Option<String>,
    pub lanes: Vec<String>,
}

/// A traffic rule, which has no identifier of its own and is named `rule/<index>`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct RuleEntry {
    pub id: String,
    pub kind: String,
    pub lanes: Vec<String>,
    /// The lights and the stop line the rule refers to.
    pub objects: Vec<String>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct BuildingEntry {
    pub id: String,
    pub kind: String,
    pub parts: Vec<String>,
    /// The road it faces, when it was placed along one.
    pub frontage: Option<String>,
}

impl IrCatalog {
    pub fn of(map: &Map) -> Self {
        let metadata = &map.metadata;
        IrCatalog {
            metadata: MetadataEntry {
                name: metadata.name.clone(),
                origin: OriginEntry {
                    latitude: metadata.origin.latitude(),
                    longitude: metadata.origin.longitude(),
                    altitude: metadata.origin.altitude(),
                },
                projection: metadata.projection.as_str().into(),
                handedness: metadata.handedness.as_str().into(),
            },
            roads: map
                .roads
                .iter()
                .map(|road| RoadEntry {
                    id: road.id.to_string(),
                    name: road.name.clone(),
                    road_type: road.road_type.as_str().into(),
                    junction: road.junction.as_ref().map(ToString::to_string),
                    sections: road
                        .sections
                        .iter()
                        .map(|section| SectionEntry {
                            station: section.station,
                            lanes: section.lanes.iter().map(ToString::to_string).collect(),
                        })
                        .collect(),
                    predecessor: road.link.predecessor.as_ref().map(link_entry),
                    successor: road.link.successor.as_ref().map(link_entry),
                    speed_limit_mps: road.speed_limit.map(|limit| limit.mps()),
                })
                .collect(),
            lanes: map
                .lanes
                .iter()
                .map(|lane| LaneEntry {
                    id: lane.id.to_string(),
                    road: lane.road.to_string(),
                    section: lane.section,
                    index: lane.index,
                    side: lane.side.as_str().into(),
                    ordinal: lane.ordinal,
                    direction: lane.direction.as_str().into(),
                    lane_type: lane.lane_type.as_str().into(),
                    station_range: [lane.station_range.0, lane.station_range.1],
                    speed_limit_mps: lane.speed_limit.map(|limit| limit.mps()),
                })
                .collect(),
            junctions: map
                .junctions
                .iter()
                .map(|junction| JunctionEntry {
                    id: junction.id.to_string(),
                    name: junction.name.clone(),
                    incoming: junction
                        .incoming_roads
                        .iter()
                        .map(ToString::to_string)
                        .collect(),
                    connecting: junction
                        .connecting_roads
                        .iter()
                        .map(ToString::to_string)
                        .collect(),
                })
                .collect(),
            connections: map
                .connections
                .iter()
                .map(|connection| ConnectionEntry {
                    id: connection.id.to_string(),
                    from: endpoint_entry(&connection.from),
                    to: endpoint_entry(&connection.to),
                    junction: connection.junction.as_ref().map(ToString::to_string),
                })
                .collect(),
            objects: map
                .objects
                .iter()
                .map(|object| ObjectEntry {
                    id: object.id.to_string(),
                    kind: object.kind.as_str().into(),
                    code: match &object.kind {
                        MapObjectKind::TrafficSign { code } => Some(code.clone()),
                        _ => None,
                    },
                    lanes: object.lanes.iter().map(ToString::to_string).collect(),
                })
                .collect(),
            rules: map
                .rules
                .iter()
                .enumerate()
                .map(|(index, rule)| RuleEntry {
                    id: roadgen_core::IrRef::Rule(index).to_string(),
                    kind: rule.kind_str().into(),
                    lanes: rule.lanes().iter().map(ToString::to_string).collect(),
                    objects: rule_objects(rule),
                })
                .collect(),
            buildings: map
                .buildings
                .iter()
                .map(|building| BuildingEntry {
                    id: building.id.to_string(),
                    kind: building.kind.clone(),
                    parts: building.parts.iter().map(ToString::to_string).collect(),
                    frontage: building
                        .frontage
                        .as_ref()
                        .map(|frontage| frontage.road.to_string()),
                })
                .collect(),
        }
    }

    /// `sha256:<hex>` of the catalogue serialised compactly.
    ///
    /// Field order is the struct's and element order is the IR's own insertion order,
    /// so the same map always hashes the same.
    pub fn fingerprint(&self) -> String {
        let bytes = serde_json::to_vec(self).expect("a catalogue is plain data");
        format!("sha256:{}", hex(&Sha256::digest(bytes)))
    }
}

fn link_entry(target: &RoadLinkTarget) -> LinkEntry {
    match target {
        RoadLinkTarget::Road(endpoint) => LinkEntry::Road {
            road: endpoint.road.to_string(),
            end: endpoint.end.as_str().into(),
        },
        RoadLinkTarget::Junction(junction) => LinkEntry::Junction {
            junction: junction.to_string(),
        },
    }
}

fn endpoint_entry(endpoint: &LaneEndpoint) -> EndpointEntry {
    EndpointEntry {
        lane: endpoint.lane.to_string(),
        end: endpoint.end.as_str().into(),
    }
}

fn rule_objects(rule: &TrafficRule) -> Vec<String> {
    match rule {
        TrafficRule::TrafficLight {
            lights, stop_line, ..
        } => lights
            .iter()
            .chain(stop_line)
            .map(ToString::to_string)
            .collect(),
        TrafficRule::RightOfWay { stop_line, .. } => {
            stop_line.iter().map(ToString::to_string).collect()
        }
        TrafficRule::SpeedLimit { .. } => Vec::new(),
    }
}

pub(crate) fn hex(bytes: &[u8]) -> String {
    bytes.iter().map(|byte| format!("{byte:02x}")).collect()
}
