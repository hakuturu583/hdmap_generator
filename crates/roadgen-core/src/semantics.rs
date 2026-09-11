//! What the road *means*, as opposed to where it is or what it connects to.
//!
//! These are IR concepts. Neither an OpenDRIVE `<signal>` nor a Lanelet2
//! `RegulatoryElement` appears here; each exporter maps these onto its own format's
//! vocabulary, and a format that cannot express one simply drops it.

use crate::geometry::{Curve3, Point3};
use crate::id::{LaneId, ObjectId};
use crate::units::SpeedLimit;

/// What a lane is for.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum LaneType {
    Driving,
    Shoulder,
    Border,
    Sidewalk,
    Biking,
    Parking,
    Restricted,
    /// Carries no traffic; used for the strip beyond the outermost boundary.
    None,
}

impl LaneType {
    /// Whether vehicles travel along this lane, which decides whether it becomes a
    /// lanelet and whether it takes part in connectivity.
    pub fn is_drivable(self) -> bool {
        matches!(self, LaneType::Driving | LaneType::Biking)
    }

    pub fn as_str(self) -> &'static str {
        match self {
            LaneType::Driving => "driving",
            LaneType::Shoulder => "shoulder",
            LaneType::Border => "border",
            LaneType::Sidewalk => "sidewalk",
            LaneType::Biking => "biking",
            LaneType::Parking => "parking",
            LaneType::Restricted => "restricted",
            LaneType::None => "none",
        }
    }

    pub fn parse(value: &str) -> Option<LaneType> {
        Some(match value.to_ascii_lowercase().as_str() {
            "driving" => LaneType::Driving,
            "shoulder" => LaneType::Shoulder,
            "border" => LaneType::Border,
            "sidewalk" | "walking" => LaneType::Sidewalk,
            "biking" | "bicycle" => LaneType::Biking,
            "parking" => LaneType::Parking,
            "restricted" => LaneType::Restricted,
            "none" => LaneType::None,
            _ => return None,
        })
    }
}

/// What kind of road a stretch is, kept separate from its lanes' types.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum RoadType {
    Town,
    Rural,
    Motorway,
    LowSpeed,
    Pedestrian,
}

impl RoadType {
    pub fn as_str(self) -> &'static str {
        match self {
            RoadType::Town => "town",
            RoadType::Rural => "rural",
            RoadType::Motorway => "motorway",
            RoadType::LowSpeed => "lowSpeed",
            RoadType::Pedestrian => "pedestrian",
        }
    }

    /// The Lanelet2 `location` tag that goes with this road type.
    pub fn lanelet2_location(self) -> &'static str {
        match self {
            RoadType::Rural | RoadType::Motorway => "nonurban",
            _ => "urban",
        }
    }

    pub fn parse(value: &str) -> Option<RoadType> {
        Some(match value.to_ascii_lowercase().as_str() {
            "town" | "urban" => RoadType::Town,
            "rural" => RoadType::Rural,
            "motorway" | "highway" => RoadType::Motorway,
            "lowspeed" | "low_speed" => RoadType::LowSpeed,
            "pedestrian" => RoadType::Pedestrian,
            _ => return None,
        })
    }
}

/// The paint on a lane boundary.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum RoadMarking {
    /// Nothing is painted, but the boundary exists as a geometric line.
    None,
    Solid,
    Broken,
    SolidSolid,
    BrokenSolid,
    SolidBroken,
    /// A physical edge such as a kerb.
    Curbstone,
}

impl RoadMarking {
    pub fn as_str(self) -> &'static str {
        match self {
            RoadMarking::None => "none",
            RoadMarking::Solid => "solid",
            RoadMarking::Broken => "broken",
            RoadMarking::SolidSolid => "solid solid",
            RoadMarking::BrokenSolid => "broken solid",
            RoadMarking::SolidBroken => "solid broken",
            RoadMarking::Curbstone => "curb",
        }
    }

    pub fn parse(value: &str) -> Option<RoadMarking> {
        Some(
            match value.to_ascii_lowercase().replace('_', " ").as_str() {
                "none" => RoadMarking::None,
                "solid" => RoadMarking::Solid,
                "broken" | "dashed" => RoadMarking::Broken,
                "solid solid" => RoadMarking::SolidSolid,
                "broken solid" => RoadMarking::BrokenSolid,
                "solid broken" => RoadMarking::SolidBroken,
                "curb" | "curbstone" | "kerb" => RoadMarking::Curbstone,
                _ => return None,
            },
        )
    }
}

/// Colour of a road marking.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum MarkingColor {
    White,
    Yellow,
}

impl MarkingColor {
    pub fn as_str(self) -> &'static str {
        match self {
            MarkingColor::White => "white",
            MarkingColor::Yellow => "yellow",
        }
    }
}

/// A lane boundary's paint, as the IR records it.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct BoundaryMarking {
    pub marking: RoadMarking,
    pub color: MarkingColor,
}

impl BoundaryMarking {
    pub fn new(marking: RoadMarking, color: MarkingColor) -> Self {
        BoundaryMarking { marking, color }
    }
}

impl Default for BoundaryMarking {
    fn default() -> Self {
        BoundaryMarking::new(RoadMarking::Solid, MarkingColor::White)
    }
}

/// Where a map object sits.
#[derive(Debug, Clone, PartialEq)]
pub enum ObjectGeometry {
    /// A single position, such as the base of a sign post.
    Point(Point3),
    /// A line across or along the road: a stop line, the bar of a traffic light.
    Line(Curve3),
    /// A strip with two edges, such as a crosswalk.
    Band { left: Curve3, right: Curve3 },
}

/// Kinds of physical furniture the IR knows about.
#[derive(Debug, Clone, PartialEq)]
pub enum MapObjectKind {
    TrafficLight,
    /// `code` is the sign's identifier in whatever catalogue the caller uses; the IR
    /// does not define one.
    TrafficSign {
        code: String,
    },
    StopLine,
    Crosswalk,
}

impl MapObjectKind {
    pub fn as_str(&self) -> &str {
        match self {
            MapObjectKind::TrafficLight => "traffic_light",
            MapObjectKind::TrafficSign { .. } => "traffic_sign",
            MapObjectKind::StopLine => "stop_line",
            MapObjectKind::Crosswalk => "crosswalk",
        }
    }
}

/// A piece of road furniture, and the lanes it applies to.
#[derive(Debug, Clone, PartialEq)]
pub struct MapObject {
    pub id: ObjectId,
    pub kind: MapObjectKind,
    pub geometry: ObjectGeometry,
    /// Lanes governed by the object, in the order the caller listed them.
    pub lanes: Vec<LaneId>,
}

/// A rule that governs movement, expressed over IR objects and lanes.
#[derive(Debug, Clone, PartialEq)]
pub enum TrafficRule {
    /// Lanes controlled by one or more traffic lights, optionally with a stop line.
    TrafficLight {
        lights: Vec<ObjectId>,
        stop_line: Option<ObjectId>,
        lanes: Vec<LaneId>,
    },
    /// `yielding` gives way to `right_of_way`.
    RightOfWay {
        right_of_way: Vec<LaneId>,
        yielding: Vec<LaneId>,
        stop_line: Option<ObjectId>,
    },
    /// A limit that applies to a set of lanes rather than to a whole road.
    SpeedLimit {
        limit: SpeedLimit,
        lanes: Vec<LaneId>,
    },
}

impl TrafficRule {
    /// Lanes the rule applies to, whatever its kind.
    pub fn lanes(&self) -> Vec<LaneId> {
        match self {
            TrafficRule::TrafficLight { lanes, .. } => lanes.clone(),
            TrafficRule::RightOfWay {
                right_of_way,
                yielding,
                ..
            } => right_of_way.iter().chain(yielding).cloned().collect(),
            TrafficRule::SpeedLimit { lanes, .. } => lanes.clone(),
        }
    }

    pub fn kind_str(&self) -> &'static str {
        match self {
            TrafficRule::TrafficLight { .. } => "traffic_light",
            TrafficRule::RightOfWay { .. } => "right_of_way",
            TrafficRule::SpeedLimit { .. } => "speed_limit",
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn lane_types_round_trip_through_their_names() {
        for kind in [
            LaneType::Driving,
            LaneType::Sidewalk,
            LaneType::Shoulder,
            LaneType::None,
        ] {
            assert_eq!(LaneType::parse(kind.as_str()), Some(kind));
        }
        assert_eq!(LaneType::parse("nonsense"), None);
    }

    #[test]
    fn only_vehicle_lanes_are_drivable() {
        assert!(LaneType::Driving.is_drivable());
        assert!(!LaneType::Sidewalk.is_drivable());
    }
}
