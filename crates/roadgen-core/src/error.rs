//! Errors raised while building and validating a map.

use std::fmt;

use crate::id::{BuildingId, BuildingPartId, ConnectionId, JunctionId, LaneId, RoadId};

/// Something that cannot be expressed as geometry.
#[derive(Debug, Clone, PartialEq)]
pub enum GeometryError {
    /// A direction was asked of a vector that has no length.
    ZeroLengthVector,
    /// A road local frame was asked of a tangent that points straight up.
    VerticalTangent,
    /// A curve whose plan view collapses to a point has no station axis.
    NoHorizontalExtent,
    /// A polyline needs two distinct vertices.
    TooFewPoints { got: usize },
    /// A coordinate was NaN or infinite.
    NonFiniteCoordinate,
    /// Two pieces of a composite curve do not meet.
    DisjointSegments { gap: f64 },
    /// The sampling step has to be a positive, finite number of metres.
    InvalidSampling { max_segment_length: f64 },
    /// A closed ring encloses no area, so it is a line rather than a polygon.
    ZeroArea,
}

impl fmt::Display for GeometryError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            GeometryError::ZeroLengthVector => {
                f.write_str("a zero-length vector does not define a direction")
            }
            GeometryError::VerticalTangent => {
                f.write_str("a vertical tangent has no road local frame")
            }
            GeometryError::NoHorizontalExtent => {
                f.write_str("the curve has no extent in the horizontal plane")
            }
            GeometryError::TooFewPoints { got } => {
                write!(f, "a polyline needs at least 2 distinct points, got {got}")
            }
            GeometryError::NonFiniteCoordinate => f.write_str("a coordinate was not finite"),
            GeometryError::DisjointSegments { gap } => write!(
                f,
                "consecutive pieces of a composite curve are {gap:.6} m apart; each \
                 piece has to start where the last one ended"
            ),
            GeometryError::InvalidSampling { max_segment_length } => write!(
                f,
                "the maximum segment length must be positive and finite, got {max_segment_length}"
            ),
            GeometryError::ZeroArea => {
                f.write_str("the ring encloses no area, so it is a line and not a polygon")
            }
        }
    }
}

impl std::error::Error for GeometryError {}

/// Something a caller asked the builder to do that it cannot do.
#[derive(Debug, Clone, PartialEq)]
pub enum BuildError {
    Geometry(GeometryError),
    /// A lane width, speed limit or similar quantity was out of range.
    Quantity(QuantityError),
    /// Two roads, lanes or junctions ended up with the same identifier.
    DuplicateId(String),
    UnknownRoad(RoadId),
    UnknownLane(LaneId),
    UnknownJunction(JunctionId),
    /// A road has to have at least one lane.
    RoadWithoutLanes(RoadId),
    /// The lane the caller named is not in the road's cross-section.
    LaneIndexOutOfRange {
        road: RoadId,
        index: usize,
        lanes: usize,
    },
    /// The two lanes cannot carry traffic from the first to the second.
    IncompatibleConnection {
        from: LaneId,
        to: LaneId,
        reason: String,
    },
    /// Two roads were joined and a movement between them was possible, yet no lane
    /// of one paired with a lane of the other.
    NoLanePairs {
        from: RoadId,
        to: RoadId,
    },
    /// A road end already carries an incompatible link.
    ConflictingRoadLink {
        road: RoadId,
        detail: String,
    },
    /// Two roads meet at an angle the generator cannot round into an arc.
    CornerTooSharp {
        from: RoadId,
        to: RoadId,
        angle_degrees: f64,
        detail: String,
    },
    /// A polyline road bends at one of its vertices more sharply than the corner
    /// can be rounded.
    VertexTooSharp {
        road: RoadId,
        vertex: usize,
        angle_degrees: f64,
        detail: String,
    },
}

impl From<GeometryError> for BuildError {
    fn from(value: GeometryError) -> Self {
        BuildError::Geometry(value)
    }
}

impl From<QuantityError> for BuildError {
    fn from(value: QuantityError) -> Self {
        BuildError::Quantity(value)
    }
}

impl fmt::Display for BuildError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            BuildError::Geometry(error) => write!(f, "{error}"),
            BuildError::Quantity(error) => write!(f, "{error}"),
            BuildError::DuplicateId(id) => write!(f, "duplicate identifier: {id}"),
            BuildError::UnknownRoad(id) => write!(f, "no such road: {id}"),
            BuildError::UnknownLane(id) => write!(f, "no such lane: {id}"),
            BuildError::UnknownJunction(id) => write!(f, "no such junction: {id}"),
            BuildError::RoadWithoutLanes(id) => write!(f, "road {id} has no lanes"),
            BuildError::LaneIndexOutOfRange { road, index, lanes } => write!(
                f,
                "road {road} has {lanes} lane(s); there is no lane at index {index}"
            ),
            BuildError::IncompatibleConnection { from, to, reason } => {
                write!(f, "cannot connect {from} to {to}: {reason}")
            }
            BuildError::NoLanePairs { from, to } => write!(
                f,
                "no lane of {from} pairs with a lane of {to}, though traffic could pass \
                 between them: lanes are paired by side and by rank outwards from the \
                 reference line, so the two cross-sections do not correspond — check \
                 that each road's lanes are listed outwards from the reference line on \
                 each side (carriageway before pavement), or join the lanes you mean \
                 with connect_lanes"
            ),
            BuildError::ConflictingRoadLink { road, detail } => {
                write!(f, "conflicting link on road {road}: {detail}")
            }
            BuildError::CornerTooSharp {
                from,
                to,
                angle_degrees,
                detail,
            } => write!(
                f,
                "roads {from} and {to} meet at {angle_degrees:.1}° and the corner cannot be \
                 rounded: {detail}. Join them with an arc or spiral alignment, or through \
                 a junction"
            ),
            BuildError::VertexTooSharp {
                road,
                vertex,
                angle_degrees,
                detail,
            } => write!(
                f,
                "road {road} bends by {angle_degrees:.1}° at vertex {vertex} and the corner \
                 cannot be rounded: {detail}. Give the bend an arc or spiral alignment, or \
                 more room"
            ),
        }
    }
}

impl std::error::Error for BuildError {}

/// A quantity that a constrained numeric type refused.
#[derive(Debug, Clone, PartialEq)]
pub enum QuantityError {
    NonPositiveWidth(f64),
    NonPositiveSpeed(f64),
    LatitudeOutOfRange(f64),
    LongitudeOutOfRange(f64),
}

impl fmt::Display for QuantityError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            QuantityError::NonPositiveWidth(value) => {
                write!(f, "a lane width must be greater than zero, got {value}")
            }
            QuantityError::NonPositiveSpeed(value) => {
                write!(f, "a speed limit must be greater than zero, got {value}")
            }
            QuantityError::LatitudeOutOfRange(value) => {
                write!(f, "latitude must be within [-90, 90] degrees, got {value}")
            }
            QuantityError::LongitudeOutOfRange(value) => {
                write!(
                    f,
                    "longitude must be within [-180, 180] degrees, got {value}"
                )
            }
        }
    }
}

impl std::error::Error for QuantityError {}

/// One thing wrong with an otherwise-built map.
#[derive(Debug, Clone, PartialEq)]
pub enum ValidationIssue {
    DuplicateId(String),
    DanglingRoadReference {
        referrer: String,
        road: RoadId,
    },
    DanglingLaneReference {
        referrer: String,
        lane: LaneId,
    },
    DanglingJunctionReference {
        referrer: String,
        junction: JunctionId,
    },
    /// A lane claims a road that does not list it.
    LaneRoadMismatch {
        lane: LaneId,
        road: RoadId,
    },
    NonPositiveLaneWidth {
        lane: LaneId,
        width: f64,
    },
    /// Two roads that are linked do not meet in space.
    RoadEndpointGap {
        detail: String,
        gap: f64,
    },
    /// A connection's two lanes do not meet in space.
    ConnectionGap {
        connection: ConnectionId,
        gap: f64,
    },
    /// A lane boundary is degenerate or crosses its partner.
    InvalidLaneBoundary {
        lane: LaneId,
        detail: String,
    },
    /// A reference line or boundary jumps rather than continuing.
    GeometryDiscontinuity {
        detail: String,
        gap: f64,
    },
    /// A lane connection points the wrong way down one of its lanes.
    InconsistentTravelDirection {
        connection: ConnectionId,
        detail: String,
    },
    /// A junction connection names a road that is not part of the junction.
    JunctionMembershipMismatch {
        junction: JunctionId,
        detail: String,
    },
    /// The map's coordinate metadata cannot be interpreted.
    InvalidCoordinateMetadata {
        detail: String,
    },
    /// A road is banked so steeply that its surface is closer to a wall.
    ImplausibleSuperelevation {
        road: RoadId,
        radians: f64,
    },
    /// A building names a part that is not on the map.
    DanglingBuildingPartReference {
        referrer: String,
        part: BuildingPartId,
    },
    /// A building is not composed of what a building is composed of.
    ImplausibleBuilding {
        building: BuildingId,
        detail: String,
    },
    /// A part's solid is not one a building could be made of.
    ImplausibleBuildingPart {
        part: BuildingPartId,
        detail: String,
    },
    Geometry {
        detail: String,
    },
}

impl fmt::Display for ValidationIssue {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            ValidationIssue::DuplicateId(id) => write!(f, "duplicate identifier: {id}"),
            ValidationIssue::DanglingRoadReference { referrer, road } => {
                write!(f, "{referrer} refers to missing road {road}")
            }
            ValidationIssue::DanglingLaneReference { referrer, lane } => {
                write!(f, "{referrer} refers to missing lane {lane}")
            }
            ValidationIssue::DanglingJunctionReference { referrer, junction } => {
                write!(f, "{referrer} refers to missing junction {junction}")
            }
            ValidationIssue::LaneRoadMismatch { lane, road } => {
                write!(f, "lane {lane} claims road {road}, which does not list it")
            }
            ValidationIssue::NonPositiveLaneWidth { lane, width } => {
                write!(f, "lane {lane} has width {width}, which is not positive")
            }
            ValidationIssue::RoadEndpointGap { detail, gap } => {
                write!(f, "{detail}: endpoints are {gap:.6} m apart")
            }
            ValidationIssue::ConnectionGap { connection, gap } => write!(
                f,
                "connection {connection} joins lanes that are {gap:.6} m apart"
            ),
            ValidationIssue::InvalidLaneBoundary { lane, detail } => {
                write!(f, "lane {lane} has an invalid boundary: {detail}")
            }
            ValidationIssue::GeometryDiscontinuity { detail, gap } => {
                write!(f, "{detail}: discontinuity of {gap:.6} m")
            }
            ValidationIssue::InconsistentTravelDirection { connection, detail } => {
                write!(f, "connection {connection} is inconsistent: {detail}")
            }
            ValidationIssue::JunctionMembershipMismatch { junction, detail } => {
                write!(f, "junction {junction}: {detail}")
            }
            ValidationIssue::InvalidCoordinateMetadata { detail } => {
                write!(f, "invalid coordinate metadata: {detail}")
            }
            ValidationIssue::ImplausibleSuperelevation { road, radians } => write!(
                f,
                "road {road} is banked by {:.1} degrees; beyond 45 the cross-section                  is no longer a road surface",
                radians.to_degrees()
            ),
            ValidationIssue::DanglingBuildingPartReference { referrer, part } => {
                write!(f, "{referrer} refers to missing building part {part}")
            }
            ValidationIssue::ImplausibleBuilding { building, detail } => {
                write!(f, "building {building}: {detail}")
            }
            ValidationIssue::ImplausibleBuildingPart { part, detail } => {
                write!(f, "building part {part}: {detail}")
            }
            ValidationIssue::Geometry { detail } => write!(f, "geometry error: {detail}"),
        }
    }
}

/// Everything wrong with a map, reported at once rather than one failure at a time.
#[derive(Debug, Clone, PartialEq)]
pub struct ValidationError {
    pub issues: Vec<ValidationIssue>,
}

impl ValidationError {
    pub fn new(issues: Vec<ValidationIssue>) -> Self {
        ValidationError { issues }
    }
}

impl fmt::Display for ValidationError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        writeln!(
            f,
            "the map failed validation ({} issues):",
            self.issues.len()
        )?;
        for issue in &self.issues {
            writeln!(f, "  - {issue}")?;
        }
        Ok(())
    }
}

impl std::error::Error for ValidationError {}
