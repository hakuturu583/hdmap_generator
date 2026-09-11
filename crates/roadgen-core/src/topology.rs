//! Connectivity, independent of geometry and of any file format.
//!
//! Topology can be built before a single coordinate exists: a [`LaneConnection`] is
//! two lane endpoints and nothing else. Both exporters derive their own connectivity
//! from these — OpenDRIVE's `<link>`/`<laneLink>` and Lanelet2's shared boundary
//! points are *lowerings* of this, never the other way round.

use crate::id::{ConnectionId, JunctionId, LaneId, RoadId};

/// Which way traffic runs along a lane, relative to its road's reference line.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Direction {
    /// Along the reference line, from its start towards its end.
    Forward,
    /// Against the reference line.
    Backward,
}

impl Direction {
    /// The end of the lane traffic leaves by.
    pub fn exit_end(self) -> LaneEnd {
        match self {
            Direction::Forward => LaneEnd::End,
            Direction::Backward => LaneEnd::Start,
        }
    }

    /// The end of the lane traffic enters by.
    pub fn entry_end(self) -> LaneEnd {
        self.exit_end().opposite()
    }

    pub fn reversed(self) -> Direction {
        match self {
            Direction::Forward => Direction::Backward,
            Direction::Backward => Direction::Forward,
        }
    }

    pub fn as_str(self) -> &'static str {
        match self {
            Direction::Forward => "forward",
            Direction::Backward => "backward",
        }
    }
}

/// Which side of the reference line a lane sits on.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum LateralSide {
    Left,
    Right,
}

impl LateralSide {
    /// `+1` to the left of the reference line, `-1` to the right — the sign of a
    /// lateral offset on this side.
    pub fn sign(self) -> f64 {
        match self {
            LateralSide::Left => 1.0,
            LateralSide::Right => -1.0,
        }
    }

    pub fn opposite(self) -> LateralSide {
        match self {
            LateralSide::Left => LateralSide::Right,
            LateralSide::Right => LateralSide::Left,
        }
    }

    pub fn as_str(self) -> &'static str {
        match self {
            LateralSide::Left => "left",
            LateralSide::Right => "right",
        }
    }
}

/// One of the two ends of a lane, in reference-line order.
///
/// Deliberately *not* "entry" and "exit": which one traffic uses depends on the
/// lane's [`Direction`], and conflating the two is how connectivity gets reversed.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum LaneEnd {
    Start,
    End,
}

impl LaneEnd {
    pub fn opposite(self) -> LaneEnd {
        match self {
            LaneEnd::Start => LaneEnd::End,
            LaneEnd::End => LaneEnd::Start,
        }
    }

    pub fn as_str(self) -> &'static str {
        match self {
            LaneEnd::Start => "start",
            LaneEnd::End => "end",
        }
    }
}

/// One of the two ends of a road's reference line.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum RoadEnd {
    Start,
    End,
}

impl RoadEnd {
    pub fn opposite(self) -> RoadEnd {
        match self {
            RoadEnd::Start => RoadEnd::End,
            RoadEnd::End => RoadEnd::Start,
        }
    }

    pub fn as_lane_end(self) -> LaneEnd {
        match self {
            RoadEnd::Start => LaneEnd::Start,
            RoadEnd::End => LaneEnd::End,
        }
    }

    pub fn as_str(self) -> &'static str {
        match self {
            RoadEnd::Start => "start",
            RoadEnd::End => "end",
        }
    }
}

/// A lane, and which of its ends is meant.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub struct LaneEndpoint {
    pub lane: LaneId,
    pub end: LaneEnd,
}

impl LaneEndpoint {
    pub fn new(lane: LaneId, end: LaneEnd) -> Self {
        LaneEndpoint { lane, end }
    }
}

/// A road, and which of its ends is meant.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub struct RoadEndpoint {
    pub road: RoadId,
    pub end: RoadEnd,
}

impl RoadEndpoint {
    pub fn new(road: RoadId, end: RoadEnd) -> Self {
        RoadEndpoint { road, end }
    }
}

/// What a road continues into.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub enum RoadLinkTarget {
    Road(RoadEndpoint),
    Junction(JunctionId),
}

/// A road's neighbours along its reference line.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct RoadLink {
    /// What lies before the reference line's start.
    pub predecessor: Option<RoadLinkTarget>,
    /// What lies after the reference line's end.
    pub successor: Option<RoadLinkTarget>,
}

impl RoadLink {
    pub fn at(&self, end: RoadEnd) -> Option<&RoadLinkTarget> {
        match end {
            RoadEnd::Start => self.predecessor.as_ref(),
            RoadEnd::End => self.successor.as_ref(),
        }
    }

    pub fn set(&mut self, end: RoadEnd, target: RoadLinkTarget) {
        match end {
            RoadEnd::Start => self.predecessor = Some(target),
            RoadEnd::End => self.successor = Some(target),
        }
    }
}

/// The canonical truth about connectivity: traffic may pass from `from` to `to`.
///
/// `from.end` is the end of the source lane traffic leaves by and `to.end` the end
/// of the target lane it arrives at, so a connection reads the same whichever way
/// round the two reference lines happen to point.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct LaneConnection {
    pub id: ConnectionId,
    pub from: LaneEndpoint,
    pub to: LaneEndpoint,
    /// Set when the movement happens inside a junction.
    pub junction: Option<JunctionId>,
}

/// A place where roads meet and movements have to be enumerated.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Junction {
    pub id: JunctionId,
    pub name: Option<String>,
    /// Roads that approach or leave the junction, in the order they were added.
    pub incoming_roads: Vec<RoadId>,
    /// Generated roads that carry traffic through the junction.
    pub connecting_roads: Vec<RoadId>,
}

impl Junction {
    pub fn new(id: JunctionId, name: Option<String>) -> Self {
        Junction {
            id,
            name,
            incoming_roads: Vec::new(),
            connecting_roads: Vec::new(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_backward_lane_leaves_by_the_reference_lines_start() {
        assert_eq!(Direction::Forward.exit_end(), LaneEnd::End);
        assert_eq!(Direction::Backward.exit_end(), LaneEnd::Start);
        assert_eq!(Direction::Backward.entry_end(), LaneEnd::End);
    }

    #[test]
    fn lateral_sign_matches_the_left_handed_offset_convention() {
        assert_eq!(LateralSide::Left.sign(), 1.0);
        assert_eq!(LateralSide::Right.sign(), -1.0);
    }
}
