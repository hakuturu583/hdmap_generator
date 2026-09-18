//! The GPUDrive scene document, as a Rust data model.
//!
//! GPUDrive has no schema file. What it has is a reader — `src/json_serialization.hpp`
//! in the simulator — and a scene is whatever that reader accepts. So the types here
//! are shaped by it field for field: every key it calls `j.at(...)` on is a
//! non-optional field below, every key it guards with `j.contains(...)` is an
//! `Option`, and the strings it compares against are the variants of [`ObjectKind`]
//! and [`RoadKind`]. Nothing from that file is reproduced — only the names two
//! programs have to agree on to exchange data.
//!
//! The model is `Deserialize` as well as `Serialize` so that a test can read a written
//! scene back the way a consumer would, rather than asserting against the text.

use serde::{Deserialize, Serialize};

/// Objects beyond this are dropped by the reader (`MAX_OBJECTS`).
pub const MAX_OBJECTS: usize = 515;
/// Road elements beyond this are dropped by the reader (`MAX_ROADS`).
pub const MAX_ROADS: usize = 956;
/// Timesteps of a track beyond this are dropped by the reader (`MAX_POSITIONS`),
/// which is also GPUDrive's episode length.
pub const MAX_POSITIONS: usize = 91;
/// Vertices of one road element beyond this are dropped (`MAX_GEOMETRY`).
pub const MAX_GEOMETRY: usize = 1746;
/// Road *segments* the simulator has room for (`consts::kMaxRoadEntityCount`): a
/// polyline of n points is n − 1 of them, and a point element is one.
pub const MAX_ROAD_SEGMENTS: usize = 10_000;
/// How much of a name survives: the reader copies into a `char[32]` with `strncpy`,
/// which writes no terminator when the source fills it.
pub const MAX_NAME: usize = 31;
/// Seconds between timesteps in the datasets GPUDrive was built on.
pub const TIME_STEP: f64 = 0.1;

/// A point in the scene's plane, metres.
///
/// GPUDrive is a 2D simulator: there is no z anywhere in the document.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Vector2 {
    pub x: f64,
    pub y: f64,
}

impl Vector2 {
    pub fn new(x: f64, y: f64) -> Self {
        Vector2 { x, y }
    }
}

/// What an object is. The reader recognises these three and calls anything else
/// `EntityType::None`, so there is no fourth variant to offer.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ObjectKind {
    Vehicle,
    Pedestrian,
    Cyclist,
}

impl ObjectKind {
    pub fn as_str(self) -> &'static str {
        match self {
            ObjectKind::Vehicle => "vehicle",
            ObjectKind::Pedestrian => "pedestrian",
            ObjectKind::Cyclist => "cyclist",
        }
    }

    pub fn parse(value: &str) -> Option<ObjectKind> {
        Some(match value.to_ascii_lowercase().as_str() {
            "vehicle" | "car" => ObjectKind::Vehicle,
            "pedestrian" => ObjectKind::Pedestrian,
            "cyclist" | "bicycle" => ObjectKind::Cyclist,
            _ => return None,
        })
    }
}

/// What a road element is. Again the reader's vocabulary, exactly.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum RoadKind {
    /// A lane, as its centreline.
    Lane,
    /// A painted line between lanes.
    RoadLine,
    /// The edge of the drivable surface.
    RoadEdge,
    Crosswalk,
    SpeedBump,
    StopSign,
}

impl RoadKind {
    pub fn as_str(self) -> &'static str {
        match self {
            RoadKind::Lane => "lane",
            RoadKind::RoadLine => "road_line",
            RoadKind::RoadEdge => "road_edge",
            RoadKind::Crosswalk => "crosswalk",
            RoadKind::SpeedBump => "speed_bump",
            RoadKind::StopSign => "stop_sign",
        }
    }

    /// Whether the reader treats the element as a polyline of segments rather than as
    /// a single point, which is what decides how much of its road-entity budget it
    /// takes.
    pub fn is_polyline(self) -> bool {
        matches!(
            self,
            RoadKind::Lane | RoadKind::RoadLine | RoadKind::RoadEdge
        )
    }
}

/// The Waymo Open Motion map-feature code a road element carries, which is what
/// GPUDrive's `MapType` is: finer than [`RoadKind`], and what a consumer reads to
/// tell a kerb from a median or a broken white line from a solid yellow one.
///
/// The numbering is the reader's, gaps and all: 4 is skipped by the original
/// definition, and the reader turns it — like anything out of range — into
/// [`MapElement::Unknown`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(into = "i32", from = "i32")]
pub enum MapElement {
    LaneUndefined,
    LaneFreeway,
    LaneSurfaceStreet,
    LaneBikeLane,
    RoadLineUnknown,
    RoadLineBrokenSingleWhite,
    RoadLineSolidSingleWhite,
    RoadLineSolidDoubleWhite,
    RoadLineBrokenSingleYellow,
    RoadLineBrokenDoubleYellow,
    RoadLineSolidSingleYellow,
    RoadLineSolidDoubleYellow,
    RoadLinePassingDoubleYellow,
    RoadEdgeUnknown,
    RoadEdgeBoundary,
    RoadEdgeMedian,
    StopSign,
    Crosswalk,
    SpeedBump,
    Driveway,
    Unknown,
}

impl MapElement {
    pub fn code(self) -> i32 {
        match self {
            MapElement::LaneUndefined => 0,
            MapElement::LaneFreeway => 1,
            MapElement::LaneSurfaceStreet => 2,
            MapElement::LaneBikeLane => 3,
            MapElement::RoadLineUnknown => 5,
            MapElement::RoadLineBrokenSingleWhite => 6,
            MapElement::RoadLineSolidSingleWhite => 7,
            MapElement::RoadLineSolidDoubleWhite => 8,
            MapElement::RoadLineBrokenSingleYellow => 9,
            MapElement::RoadLineBrokenDoubleYellow => 10,
            MapElement::RoadLineSolidSingleYellow => 11,
            MapElement::RoadLineSolidDoubleYellow => 12,
            MapElement::RoadLinePassingDoubleYellow => 13,
            MapElement::RoadEdgeUnknown => 14,
            MapElement::RoadEdgeBoundary => 15,
            MapElement::RoadEdgeMedian => 16,
            MapElement::StopSign => 17,
            MapElement::Crosswalk => 18,
            MapElement::SpeedBump => 19,
            MapElement::Driveway => 20,
            MapElement::Unknown => -1,
        }
    }

    /// The code as the reader reads it: anything it would not recognise comes back as
    /// [`MapElement::Unknown`], which is what the reader itself stores.
    pub fn from_code(code: i32) -> MapElement {
        match code {
            0 => MapElement::LaneUndefined,
            1 => MapElement::LaneFreeway,
            2 => MapElement::LaneSurfaceStreet,
            3 => MapElement::LaneBikeLane,
            5 => MapElement::RoadLineUnknown,
            6 => MapElement::RoadLineBrokenSingleWhite,
            7 => MapElement::RoadLineSolidSingleWhite,
            8 => MapElement::RoadLineSolidDoubleWhite,
            9 => MapElement::RoadLineBrokenSingleYellow,
            10 => MapElement::RoadLineBrokenDoubleYellow,
            11 => MapElement::RoadLineSolidSingleYellow,
            12 => MapElement::RoadLineSolidDoubleYellow,
            13 => MapElement::RoadLinePassingDoubleYellow,
            14 => MapElement::RoadEdgeUnknown,
            15 => MapElement::RoadEdgeBoundary,
            16 => MapElement::RoadEdgeMedian,
            17 => MapElement::StopSign,
            18 => MapElement::Crosswalk,
            19 => MapElement::SpeedBump,
            20 => MapElement::Driveway,
            _ => MapElement::Unknown,
        }
    }
}

impl From<MapElement> for i32 {
    fn from(value: MapElement) -> i32 {
        value.code()
    }
}

impl From<i32> for MapElement {
    fn from(value: i32) -> MapElement {
        MapElement::from_code(value)
    }
}

/// One road element: a polyline, or a single point for a stop sign.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Road {
    #[serde(rename = "type")]
    pub kind: RoadKind,
    pub geometry: Vec<Vector2>,
    pub id: u32,
    pub map_element_id: MapElement,
}

/// One agent, as a track: where it is at each timestep, and how big it is.
///
/// `position`, `heading`, `velocity` and `valid` are read step by step in parallel, so
/// they are the same length; [`Object::is_consistent`] is that invariant written down.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Object {
    pub position: Vec<Vector2>,
    pub width: f64,
    pub length: f64,
    pub height: f64,
    pub id: u32,
    /// Yaw at each timestep, radians counterclockwise from +x.
    pub heading: Vec<f64>,
    pub velocity: Vec<Vector2>,
    /// Whether the agent exists at that timestep.
    pub valid: Vec<bool>,
    #[serde(rename = "goalPosition")]
    pub goal_position: Vector2,
    #[serde(rename = "type")]
    pub kind: ObjectKind,
    /// Makes the agent follow its logged track rather than be controlled.
    pub mark_as_expert: bool,
}

impl Object {
    /// How many timesteps the track covers, when its arrays agree.
    pub fn steps(&self) -> usize {
        self.position.len()
    }

    pub fn is_consistent(&self) -> bool {
        let steps = self.position.len();
        self.heading.len() == steps && self.velocity.len() == steps && self.valid.len() == steps
    }
}

/// An agent the scene asks a consumer to predict, and how hard it is held to be.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct TrackToPredict {
    /// Index into [`Scene::objects`], not an object id.
    pub track_index: i32,
    pub difficulty: i32,
}

/// The scene's metadata block. Every field is read unconditionally, so a scene
/// without one is a scene the reader throws on.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Metadata {
    /// Index into [`Scene::objects`] of the self-driving car, or `-1` for none.
    pub sdc_track_index: i32,
    pub tracks_to_predict: Vec<TrackToPredict>,
    /// Object *ids*, unlike `tracks_to_predict`, which holds indices.
    pub objects_of_interest: Vec<u32>,
}

/// A GPUDrive scene: one map, and the agents driving it.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Scene {
    pub name: String,
    pub scenario_id: String,
    pub objects: Vec<Object>,
    pub roads: Vec<Road>,
    pub metadata: Metadata,
}

impl Scene {
    /// The road-entity budget this scene spends: a polyline costs one per segment, a
    /// point element one.
    pub fn road_segments(&self) -> usize {
        self.roads
            .iter()
            .map(|road| {
                if road.kind.is_polyline() {
                    road.geometry.len().saturating_sub(1)
                } else {
                    1
                }
            })
            .sum()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_map_element_is_written_as_its_waymo_code() {
        let json = serde_json::to_string(&MapElement::RoadEdgeMedian).unwrap();
        assert_eq!(json, "16");
        assert_eq!(
            serde_json::from_str::<MapElement>("16").unwrap(),
            MapElement::RoadEdgeMedian
        );
    }

    #[test]
    fn a_code_the_reader_would_not_recognise_reads_back_as_unknown() {
        // 4 is the gap in the original definition, and the reader maps it — like
        // anything out of range — onto UNKNOWN rather than onto a neighbour.
        for code in [4, 21, -7, 9999] {
            assert_eq!(MapElement::from_code(code), MapElement::Unknown);
        }
    }

    #[test]
    fn the_readers_own_spelling_is_what_gets_written() {
        assert_eq!(
            serde_json::to_string(&RoadKind::SpeedBump).unwrap(),
            "\"speed_bump\""
        );
        assert_eq!(
            serde_json::to_string(&ObjectKind::Cyclist).unwrap(),
            "\"cyclist\""
        );
    }
}
