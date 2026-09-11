//! The Canonical Road IR.
//!
//! One model, generated once, from which every output format is lowered. Nothing
//! here is shaped by OpenDRIVE or Lanelet2: there is no `laneSection`, no `way`, no
//! `relation`. What there is, is the minimum a road generator needs — roads, lanes,
//! junctions, the connections between them, 3D geometry and semantics — with each of
//! those concerns kept in its own module.

use crate::arena::Arena;
use crate::error::GeometryError;
use crate::geometry::{Curve3, Point3, Poly3Profile, SamplingConfig, WidthProfile};
use crate::id::{ConnectionId, JunctionId, LaneId, ObjectId, RoadId};
use crate::semantics::{BoundaryMarking, LaneType, MapObject, RoadType, TrafficRule};
use crate::topology::{
    Direction, Junction, LaneConnection, LaneEnd, LateralSide, RoadEnd, RoadLink,
};
use crate::units::{GeoOrigin, SpeedLimit};

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

/// How the map's metric coordinates are tied to the globe, and how they are reported
/// to a consumer that wants grid coordinates.
///
/// In every case a map's own x and y are metres about its origin — that is what makes
/// a generated map easy to write. What differs is the frame those metres are read
/// against, and therefore which grid position a consumer reconstructs.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Projection {
    /// East/north/up metres about the origin. The natural choice for a generated
    /// map, whose coordinates start at the origin by construction.
    LocalCartesian,
    /// UTM, in the zone the origin falls in, with the origin's easting and northing
    /// subtracted.
    Utm,
    /// UTM again, but reported to Autoware as metres within the 100 km MGRS square
    /// the origin falls in — which is the coordinate system an Autoware map built
    /// with the MGRS projector uses.
    ///
    /// The map is still written about its origin; what the Lanelet2 export adds is
    /// each node's position within that square, computed for that node rather than
    /// assumed from the origin's. A map whose extent leaves the square cannot be
    /// expressed this way, and the exporter says so.
    Mgrs,
}

impl Projection {
    pub fn as_str(self) -> &'static str {
        match self {
            Projection::LocalCartesian => "local_cartesian",
            Projection::Utm => "utm",
            Projection::Mgrs => "mgrs",
        }
    }

    pub fn parse(value: &str) -> Option<Projection> {
        Some(match value.to_ascii_lowercase().as_str() {
            "local_cartesian" | "local" | "localcartesian" => Projection::LocalCartesian,
            "utm" => Projection::Utm,
            "mgrs" => Projection::Mgrs,
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

/// Stations a road's reference line has to be sampled at, beyond whatever its own
/// shape calls for.
///
/// Two things add stations. A cross-section boundary needs a vertex, so that a
/// section starts exactly where the caller said rather than at the nearest station
/// the sampler happened to produce. And a stretch where a lane tapers needs vertices
/// along it: a straight road is otherwise two points, and a width that varies between
/// them would have nothing to vary over.
pub fn required_stations<'a>(
    section_stations: impl Iterator<Item = f64>,
    widths: impl Iterator<Item = &'a WidthProfile>,
    config: SamplingConfig,
) -> Vec<f64> {
    let mut stations: Vec<f64> = section_stations.collect();
    for width in widths {
        if width.is_constant() {
            continue;
        }
        for pair in width.knots().windows(2) {
            let (from, to) = (pair[0].0, pair[1].0);
            if to - from <= 0.0 {
                continue;
            }
            let steps = ((to - from) / config.max_segment_length).ceil() as usize;
            stations
                .extend((0..=steps).map(|step| from + (to - from) * step as f64 / steps as f64));
        }
    }
    stations
}

/// One cross-section of a road, valid from `station` until the next one.
///
/// A road whose lane count changes partway along has more than one of these. A lane
/// that merely tapers does not: that is a width profile, and it needs no new section.
#[derive(Debug, Clone, PartialEq)]
pub struct CrossSection {
    /// Horizontal station where this cross-section takes over, metres.
    pub station: f64,
    /// Lanes of this section, in the order the caller wrote them.
    pub lanes: Vec<LaneId>,
}

/// A stretch of road carrying a cross-section along one reference line.
#[derive(Debug, Clone, PartialEq)]
pub struct Road {
    pub id: RoadId,
    pub name: Option<String>,
    /// The 3D curve the cross-section is laid out against.
    pub reference_line: Curve3,
    /// Lateral displacement of the cross-section's origin from the reference line,
    /// metres to the left, against station. Zero for an ordinary road; half the lane
    /// width for a junction connector, whose reference line runs down the middle of
    /// its single lane — and which follows that lane when it tapers.
    pub lane_offset: Poly3Profile,
    /// Every lane of the road, section by section, in the order they were written.
    pub lanes: Vec<LaneId>,
    /// The road's cross-sections, ascending by station. There is always at least
    /// one, and the first starts at zero.
    pub sections: Vec<CrossSection>,
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

    /// The station range section `index` covers: from its own station to the next
    /// one's, or to the end of the road.
    pub fn section_range(&self, index: usize) -> Result<(f64, f64), GeometryError> {
        let start = self.sections[index].station;
        let end = match self.sections.get(index + 1) {
            Some(next) => next.station,
            None => self.horizontal_length()?,
        };
        Ok((start, end))
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
    /// How wide the lane is along its length. Never zero anywhere.
    pub width: WidthProfile,
    pub speed_limit: Option<SpeedLimit>,
    /// Which of the road's cross-sections this lane belongs to.
    pub section: usize,
    /// The stations the lane spans, which are its section's.
    pub station_range: (f64, f64),
    /// Which cross-section edge bounds the lane on each side, counted outwards from
    /// the cross-section origin: `0` is the origin itself, `+n` the n-th edge to the
    /// left of it and `-n` the n-th to the right.
    ///
    /// Two lanes share an edge exactly when these agree, which is what tells the
    /// Lanelet2 export that they share a boundary — and it keeps saying so when the
    /// lanes taper and the lateral offsets no longer match.
    pub left_edge: i32,
    pub right_edge: i32,
    /// Lateral offset of each boundary at the lane's *start*, metres. Informational:
    /// where a lane tapers, the offset elsewhere differs, and the boundaries below
    /// are the truth.
    pub left_offset: f64,
    /// Lateral offset of the boundary to the right of the reference line at the
    /// lane's start, metres.
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
    /// Lateral offset of the lane's middle at its start, metres.
    pub fn center_offset(&self) -> f64 {
        (self.left_offset + self.right_offset) / 2.0
    }

    /// How wide the lane is at `station`.
    pub fn width_at(&self, station: f64) -> f64 {
        self.width.evaluate(station).metres()
    }

    /// How wide the lane is where traffic enters it.
    pub fn entry_width(&self) -> f64 {
        let (start, end) = self.station_range;
        self.width_at(match self.direction {
            Direction::Forward => start,
            Direction::Backward => end,
        })
    }

    /// How wide the lane is where traffic leaves it.
    pub fn exit_width(&self) -> f64 {
        let (start, end) = self.station_range;
        self.width_at(match self.direction {
            Direction::Forward => end,
            Direction::Backward => start,
        })
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

    /// The stations at which a road's geometry was generated — the stations of the
    /// vertices of every boundary and centreline it owns.
    ///
    /// A lane's vertices are these, restricted to the lane's own station range, which
    /// is what lets a caller line a boundary's points up with positions along the
    /// reference line.
    pub fn vertex_stations(&self, road: &RoadId) -> Result<Vec<f64>, GeometryError> {
        let Some(entry) = self.roads.get(road) else {
            return Ok(Vec::new());
        };
        let lanes = self.lanes_of(road);
        let required = required_stations(
            entry.sections.iter().map(|section| section.station),
            lanes.iter().map(|lane| &lane.width),
            self.metadata.sampling,
        );
        Ok(entry
            .reference_line
            .samples_including(self.metadata.sampling, &required)?
            .into_iter()
            .map(|sample| sample.station)
            .collect())
    }

    /// Lanes of one cross-section of a road.
    pub fn lanes_of_section(&self, road: &RoadId, section: usize) -> Vec<&Lane> {
        self.roads
            .get(road)
            .and_then(|road| road.sections.get(section))
            .map(|section| {
                section
                    .lanes
                    .iter()
                    .filter_map(|id| self.lanes.get(id))
                    .collect()
            })
            .unwrap_or_default()
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
