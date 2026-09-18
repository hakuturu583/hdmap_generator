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
//! scene back the way a consumer would, rather than asserting against the text. What
//! is *written* is exact either way — serde_json emits the shortest text that
//! round-trips an `f64` — but reading it back exactly needs serde_json's
//! `float_roundtrip` feature, which a caller who deserialises a scene turns on for
//! itself. GPUDrive's own reader parses through `strtod` and is exact.

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

/// As much of a name as the reader keeps.
///
/// It copies into a `char[32]` with `strncpy`, which writes no terminator when the
/// source fills the buffer, so the last byte is left to be the terminator. Truncation
/// is on characters rather than bytes: half a character would not be a name.
pub fn truncate_name(name: &str) -> String {
    name.chars().take(MAX_NAME).collect()
}

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
#[repr(i32)]
pub enum MapElement {
    LaneUndefined = 0,
    LaneFreeway = 1,
    LaneSurfaceStreet = 2,
    LaneBikeLane = 3,
    // The original definition skips 4.
    RoadLineUnknown = 5,
    RoadLineBrokenSingleWhite = 6,
    RoadLineSolidSingleWhite = 7,
    RoadLineSolidDoubleWhite = 8,
    RoadLineBrokenSingleYellow = 9,
    RoadLineBrokenDoubleYellow = 10,
    RoadLineSolidSingleYellow = 11,
    RoadLineSolidDoubleYellow = 12,
    RoadLinePassingDoubleYellow = 13,
    RoadEdgeUnknown = 14,
    RoadEdgeBoundary = 15,
    RoadEdgeMedian = 16,
    StopSign = 17,
    Crosswalk = 18,
    SpeedBump = 19,
    Driveway = 20,
    Unknown = -1,
}

impl MapElement {
    pub fn code(self) -> i32 {
        self as i32
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
    /// Where this scene runs past the reader's fixed buffers.
    ///
    /// None of these is refused: the reader silently keeps what fits and drops the
    /// rest, which is exactly why a caller wants to be told. Empty when the scene fits.
    pub fn over_limits(&self) -> Vec<String> {
        let mut problems = Vec::new();
        if self.objects.len() > MAX_OBJECTS {
            problems.push(format!(
                "the scene has {} agents and the reader keeps {MAX_OBJECTS}",
                self.objects.len()
            ));
        }
        if self.roads.len() > MAX_ROADS {
            problems.push(format!(
                "the map is {} road elements and the reader keeps {MAX_ROADS}: the \
                 rest are dropped as it loads",
                self.roads.len()
            ));
        }
        let long = self
            .roads
            .iter()
            .filter(|road| road.geometry.len() > MAX_GEOMETRY)
            .count();
        if long > 0 {
            problems.push(format!(
                "{long} road elements are longer than the {MAX_GEOMETRY} vertices the \
                 reader keeps, so they are written truncated; sample the map more \
                 coarsely to fit"
            ));
        }
        let steps = self.objects.iter().map(Object::steps).max().unwrap_or(0);
        if steps > MAX_POSITIONS {
            problems.push(format!(
                "the longest track is {steps} timesteps and the reader keeps \
                 {MAX_POSITIONS}, which is GPUDrive's episode length: the rest of \
                 every track is dropped as it loads"
            ));
        }
        let segments = self.road_segments();
        if segments > MAX_ROAD_SEGMENTS {
            problems.push(format!(
                "the map is {segments} road segments and the simulator has room for \
                 {MAX_ROAD_SEGMENTS}; GPUDrive reduces polylines as it loads, but a \
                 map this dense may not fit even so"
            ));
        }
        problems
    }

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
