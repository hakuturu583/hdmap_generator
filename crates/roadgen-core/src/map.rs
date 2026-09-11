//! The Canonical Road IR.
//!
//! One model, generated once, from which every output format is lowered. Nothing
//! here is shaped by OpenDRIVE or Lanelet2: there is no `laneSection`, no `way`, no
//! `relation`. What there is, is the minimum a road generator needs — roads, lanes,
//! junctions, the connections between them, 3D geometry and semantics — with each of
//! those concerns kept in its own module.

use crate::arena::Arena;
use crate::error::GeometryError;
use crate::geometry::{Curve3, Point3, Poly3Profile, SamplingConfig};
use crate::id::{ConnectionId, JunctionId, LaneId, ObjectId, RoadId};
use crate::semantics::{BoundaryMarking, LaneType, MapObject, RoadType, TrafficRule};
use crate::topology::{
    Direction, Junction, LaneConnection, LaneEnd, LateralSide, RoadEnd, RoadLink,
};
use crate::units::{GeoOrigin, PositiveWidth, SpeedLimit};

/// Which side of the road traffic keeps to.
///
/// This decides which side of the reference line a `forward` lane lands on, and
/// nothing else; it is a property of the road network, not of any file format.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum TrafficHandedness {
    /// Right-hand traffic: forward lanes sit to the right of the reference line.
    RightHand,
    /// Left-hand traffic: forward lanes sit to the left of the reference line.
    LeftHand,
}

impl TrafficHandedness {
    /// The side a lane travelling in `direction` belongs on.
    pub fn side_for(self, direction: Direction) -> LateralSide {
        match (self, direction) {
            (TrafficHandedness::RightHand, Direction::Forward)
            | (TrafficHandedness::LeftHand, Direction::Backward) => LateralSide::Right,
            _ => LateralSide::Left,
        }
    }

    pub fn as_str(self) -> &'static str {
        match self {
            TrafficHandedness::RightHand => "RHT",
            TrafficHandedness::LeftHand => "LHT",
        }
    }

    pub fn parse(value: &str) -> Option<TrafficHandedness> {
        Some(match value.to_ascii_lowercase().as_str() {
            "rht" | "right" | "right_hand" | "right-hand" => TrafficHandedness::RightHand,
            "lht" | "left" | "left_hand" | "left-hand" => TrafficHandedness::LeftHand,
            _ => return None,
        })
    }
}

/// How the map's metric coordinates relate to the globe.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Projection {
    /// East/north/up metres about the origin. The natural choice for a generated
    /// map, whose coordinates start at the origin by construction.
    LocalCartesian,
    /// UTM, in the zone the origin falls in.
    Utm,
}

impl Projection {
    pub fn as_str(self) -> &'static str {
        match self {
            Projection::LocalCartesian => "local_cartesian",
            Projection::Utm => "utm",
        }
    }

    pub fn parse(value: &str) -> Option<Projection> {
        Some(match value.to_ascii_lowercase().as_str() {
            "local_cartesian" | "local" | "localcartesian" => Projection::LocalCartesian,
            "utm" => Projection::Utm,
            _ => return None,
        })
    }
}

/// Map-wide settings that are not about any one road.
#[derive(Debug, Clone, PartialEq)]
pub struct MapMetadata {
    pub name: Option<String>,
    /// Where the metric origin sits on the globe.
    pub origin: GeoOrigin,
    pub projection: Projection,
    pub handedness: TrafficHandedness,
    /// How finely curves are turned into vertices when geometry is generated.
    pub sampling: SamplingConfig,
}

impl Default for MapMetadata {
    fn default() -> Self {
        MapMetadata {
            name: None,
            origin: GeoOrigin::default(),
            projection: Projection::LocalCartesian,
            handedness: TrafficHandedness::RightHand,
            sampling: SamplingConfig::default(),
        }
    }
}

/// A stretch of road carrying a cross-section along one reference line.
#[derive(Debug, Clone, PartialEq)]
pub struct Road {
    pub id: RoadId,
    pub name: Option<String>,
    /// The 3D curve the cross-section is laid out against.
    pub reference_line: Curve3,
    /// Lateral displacement of the cross-section's origin from the reference line,
    /// metres to the left. Zero for an ordinary road; half a lane width for a
    /// junction connector, whose reference line runs down the middle of its lane.
    pub lane_offset: f64,
    /// Lanes in the order the caller wrote them.
    pub lanes: Vec<LaneId>,
    /// Set when the road exists to carry traffic through a junction.
    pub junction: Option<JunctionId>,
    pub link: RoadLink,
    pub road_type: RoadType,
    pub speed_limit: Option<SpeedLimit>,
    /// Roll of the road surface about its reference line, radians against horizontal
    /// station. Zero everywhere for a road that is flat across.
    pub superelevation: Poly3Profile,
}

impl Road {
    /// Length of the reference line's plan view, metres.
    pub fn horizontal_length(&self) -> Result<f64, GeometryError> {
        self.reference_line.horizontal_length()
    }

    pub fn endpoint(&self, end: RoadEnd) -> Point3 {
        match end {
            RoadEnd::Start => self.reference_line.start_point(),
            RoadEnd::End => self.reference_line.end_point(),
        }
    }

    pub fn is_connector(&self) -> bool {
        self.junction.is_some()
    }
}

/// One lane of one road.
///
/// Boundaries and centreline are stored in reference-line order, not in travel
/// order, so that `left` always means "to the left of the reference line". Use
/// [`Lane::travel_geometry`] to get them the way traffic sees them.
#[derive(Debug, Clone, PartialEq)]
pub struct Lane {
    pub id: LaneId,
    pub road: RoadId,
    /// Position in the road's cross-section list, as the caller wrote it.
    pub index: usize,
    pub side: LateralSide,
    /// Rank outwards from the cross-section origin on this side, counting from 1.
    pub ordinal: usize,
    pub direction: Direction,
    pub lane_type: LaneType,
    pub width: PositiveWidth,
    pub speed_limit: Option<SpeedLimit>,
    /// Lateral offset of the boundary to the left of the reference line, metres.
    pub left_offset: f64,
    /// Lateral offset of the boundary to the right of the reference line, metres.
    pub right_offset: f64,
    pub left_boundary: Curve3,
    pub right_boundary: Curve3,
    pub centerline: Curve3,
    pub left_marking: BoundaryMarking,
    pub right_marking: BoundaryMarking,
}

/// A lane's geometry oriented the way traffic travels it.
#[derive(Debug, Clone, PartialEq)]
pub struct TravelGeometry {
    pub left: Curve3,
    pub right: Curve3,
    pub centerline: Curve3,
}

impl Lane {
    pub fn center_offset(&self) -> f64 {
        (self.left_offset + self.right_offset) / 2.0
    }

    /// The lane's endpoint on its reference line, by reference-line end.
    pub fn endpoint(&self, end: LaneEnd) -> Point3 {
        match end {
            LaneEnd::Start => self.centerline.start_point(),
            LaneEnd::End => self.centerline.end_point(),
        }
    }

    /// Where traffic leaves the lane.
    pub fn exit_point(&self) -> Point3 {
        self.endpoint(self.direction.exit_end())
    }

    /// Where traffic enters the lane.
    pub fn entry_point(&self) -> Point3 {
        self.endpoint(self.direction.entry_end())
    }

    /// Boundaries and centreline in travel order, with `left` to the driver's left.
    ///
    /// For a backward lane this reverses each curve *and* swaps the two boundaries;
    /// doing only one of the two is the classic way to produce a mirrored lanelet.
    pub fn travel_geometry(&self, config: SamplingConfig) -> Result<TravelGeometry, GeometryError> {
        Ok(match self.direction {
            Direction::Forward => TravelGeometry {
                left: self.left_boundary.clone(),
                right: self.right_boundary.clone(),
                centerline: self.centerline.clone(),
            },
            Direction::Backward => TravelGeometry {
                left: self.right_boundary.reversed(config)?,
                right: self.left_boundary.reversed(config)?,
                centerline: self.centerline.reversed(config)?,
            },
        })
    }
}

/// The map itself: everything the generator produced, before or after validation.
#[derive(Debug, Clone, PartialEq)]
pub struct Map {
    pub metadata: MapMetadata,
    pub roads: Arena<RoadId, Road>,
    pub lanes: Arena<LaneId, Lane>,
    pub junctions: Arena<JunctionId, Junction>,
    pub connections: Arena<ConnectionId, LaneConnection>,
    pub objects: Arena<ObjectId, MapObject>,
    pub rules: Vec<TrafficRule>,
}

impl Map {
    pub fn new(metadata: MapMetadata) -> Self {
        Map {
            metadata,
            roads: Arena::new(),
            lanes: Arena::new(),
            junctions: Arena::new(),
            connections: Arena::new(),
            objects: Arena::new(),
            rules: Vec::new(),
        }
    }

    pub fn road(&self, id: &RoadId) -> Option<&Road> {
        self.roads.get(id)
    }

    pub fn lane(&self, id: &LaneId) -> Option<&Lane> {
        self.lanes.get(id)
    }

    pub fn junction(&self, id: &JunctionId) -> Option<&Junction> {
        self.junctions.get(id)
    }

    /// Lanes of a road, in cross-section order.
    pub fn lanes_of(&self, road: &RoadId) -> Vec<&Lane> {
        self.roads
            .get(road)
            .map(|road| {
                road.lanes
                    .iter()
                    .filter_map(|id| self.lanes.get(id))
                    .collect()
            })
            .unwrap_or_default()
    }

    /// Connections leaving `lane`, in insertion order.
    pub fn connections_from(&self, lane: &LaneId) -> Vec<&LaneConnection> {
        self.connections
            .iter()
            .filter(|connection| &connection.from.lane == lane)
            .collect()
    }

    /// Connections arriving at `lane`, in insertion order.
    pub fn connections_to(&self, lane: &LaneId) -> Vec<&LaneConnection> {
        self.connections
            .iter()
            .filter(|connection| &connection.to.lane == lane)
            .collect()
    }

    /// Lanes reachable in one step from `lane`.
    pub fn successors(&self, lane: &LaneId) -> Vec<LaneId> {
        self.connections_from(lane)
            .into_iter()
            .map(|connection| connection.to.lane.clone())
            .collect()
    }

    /// Lanes that reach `lane` in one step.
    pub fn predecessors(&self, lane: &LaneId) -> Vec<LaneId> {
        self.connections_to(lane)
            .into_iter()
            .map(|connection| connection.from.lane.clone())
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn handedness_decides_which_side_a_forward_lane_lands_on() {
        assert_eq!(
            TrafficHandedness::RightHand.side_for(Direction::Forward),
            LateralSide::Right
        );
        assert_eq!(
            TrafficHandedness::LeftHand.side_for(Direction::Forward),
            LateralSide::Left
        );
        assert_eq!(
            TrafficHandedness::LeftHand.side_for(Direction::Backward),
            LateralSide::Right
        );
    }
}
