//! Generating a map: cross-sections, joints, junction connectors.
//!
//! The builder collects *topology* first — roads, their cross-sections and the
//! movements between them — and only turns that into coordinates in [`MapBuilder::finish`].
//! That order is what lets a joint be mitred (the boundaries of two roads that meet
//! have to agree on one lateral direction, which needs both roads) and what lets a
//! junction connector be drawn at all (it is shaped by the lanes it joins).

use std::collections::{HashMap, HashSet};

use crate::error::{BuildError, GeometryError};
use crate::geometry::{
    Arc3, Bezier3, Curve3, Point3, Poly3Piece, Poly3Profile, Polyline3, Sample, SamplingConfig,
    Taper, UnitVector3, Vector3, WidthProfile,
};
use crate::id::{ConnectionId, JunctionId, LaneId, ObjectId, RoadId};
use crate::map::{CrossSection, Lane, Map, MapMetadata, Road};
use crate::semantics::{
    BoundaryMarking, LaneType, MapObject, MapObjectKind, ObjectGeometry, RoadMarking, RoadType,
    TrafficRule,
};
use crate::topology::{
    Direction, Junction, LaneConnection, LaneEnd, LaneEndpoint, LateralSide, RoadEnd, RoadEndpoint,
    RoadLink, RoadLinkTarget,
};
use crate::units::{PositiveWidth, SpeedLimit};
use crate::validation::UnvalidatedMap;

/// One lane of a road's cross-section, as the caller describes it.
#[derive(Debug, Clone, PartialEq)]
pub struct LaneSpec {
    /// How wide the lane is along its length. A plain [`PositiveWidth`] converts, so
    /// a lane of constant width reads exactly as it did before profiles existed.
    pub width: WidthProfile,
    pub direction: Direction,
    pub lane_type: LaneType,
    pub speed_limit: Option<SpeedLimit>,
    /// Which side of the reference line to place the lane on. Left unset, the map's
    /// handedness decides: a `forward` lane goes right under right-hand traffic.
    pub side: Option<LateralSide>,
    pub left_marking: BoundaryMarking,
    pub right_marking: BoundaryMarking,
}

impl LaneSpec {
    pub fn new(width: impl Into<WidthProfile>, direction: Direction) -> Self {
        LaneSpec {
            width: width.into(),
            direction,
            lane_type: LaneType::Driving,
            speed_limit: None,
            side: None,
            left_marking: BoundaryMarking::default(),
            right_marking: BoundaryMarking::default(),
        }
    }

    pub fn with_type(mut self, lane_type: LaneType) -> Self {
        self.lane_type = lane_type;
        self
    }

    pub fn with_speed_limit(mut self, limit: SpeedLimit) -> Self {
        self.speed_limit = Some(limit);
        self
    }

    pub fn with_side(mut self, side: LateralSide) -> Self {
        self.side = Some(side);
        self
    }

    /// Gives the lane a width that changes along the road.
    ///
    /// Stations are measured along the road's reference line, not from the start of
    /// the lane's cross-section, so a taper is written where it is on the map.
    pub fn with_width_profile(mut self, width: WidthProfile) -> Self {
        self.width = width;
        self
    }

    pub fn with_markings(mut self, left: BoundaryMarking, right: BoundaryMarking) -> Self {
        self.left_marking = left;
        self.right_marking = right;
        self
    }
}

/// One cross-section of a road, as the caller describes it.
#[derive(Debug, Clone, PartialEq)]
pub struct CrossSectionSpec {
    /// Horizontal station along the road where this cross-section takes over.
    pub station: f64,
    pub lanes: Vec<LaneSpec>,
}

/// A road to generate.
#[derive(Debug, Clone, PartialEq)]
pub struct RoadSpec {
    pub name: Option<String>,
    pub reference_line: Curve3,
    /// The road's cross-sections. A road whose lane *count* changes partway has more
    /// than one; a lane that merely tapers needs only a width profile.
    pub cross_sections: Vec<CrossSectionSpec>,
    pub road_type: RoadType,
    pub speed_limit: Option<SpeedLimit>,
    /// Roll of the road surface about its reference line, radians, as a function of
    /// horizontal station. Zero everywhere is a road that is flat across.
    pub superelevation: Poly3Profile,
}

impl RoadSpec {
    pub fn new(reference_line: Curve3, lanes: Vec<LaneSpec>) -> Self {
        RoadSpec {
            name: None,
            reference_line,
            cross_sections: vec![CrossSectionSpec {
                station: 0.0,
                lanes,
            }],
            road_type: RoadType::Town,
            speed_limit: None,
            superelevation: Poly3Profile::default(),
        }
    }

    /// A straight road between two 3D positions.
    pub fn line(start: Point3, end: Point3, lanes: Vec<LaneSpec>) -> Result<Self, GeometryError> {
        Ok(RoadSpec::new(Curve3::line(start, end)?, lanes))
    }

    pub fn with_name(mut self, name: impl Into<String>) -> Self {
        self.name = Some(name.into());
        self
    }

    pub fn with_type(mut self, road_type: RoadType) -> Self {
        self.road_type = road_type;
        self
    }

    pub fn with_speed_limit(mut self, limit: SpeedLimit) -> Self {
        self.speed_limit = Some(limit);
        self
    }

    /// Banks the road: `superelevation` gives the roll in radians against the
    /// horizontal station, positive raising the left-hand side.
    pub fn with_superelevation(mut self, superelevation: Poly3Profile) -> Self {
        self.superelevation = superelevation;
        self
    }

    /// Adds a cross-section that takes over at `station`.
    ///
    /// Use this where the *number* of lanes changes — one appears, one ends. A lane
    /// that only narrows or widens stays in one cross-section and carries a width
    /// profile instead, which keeps it one lane rather than two joined ones.
    pub fn with_cross_section(mut self, station: f64, lanes: Vec<LaneSpec>) -> Self {
        self.cross_sections
            .push(CrossSectionSpec { station, lanes });
        self
    }

    /// Every lane of every cross-section, in order. A lane's position in this list
    /// is the index a [`LaneRef`] addresses it by.
    pub fn all_lanes(&self) -> impl Iterator<Item = &LaneSpec> {
        self.cross_sections
            .iter()
            .flat_map(|section| section.lanes.iter())
    }

    /// Where the lanes of section `index` begin in [`RoadSpec::all_lanes`].
    fn section_offset(&self, index: usize) -> usize {
        self.cross_sections[..index]
            .iter()
            .map(|section| section.lanes.len())
            .sum()
    }

    /// The section a global lane index falls in, and its position within it.
    fn locate(&self, index: usize) -> Option<(usize, usize)> {
        let mut remaining = index;
        for (section, entry) in self.cross_sections.iter().enumerate() {
            if remaining < entry.lanes.len() {
                return Some((section, remaining));
            }
            remaining -= entry.lanes.len();
        }
        None
    }
}

/// A lane addressed by its road and its position in that road's cross-section.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub struct LaneRef {
    pub road: RoadId,
    pub index: usize,
}

impl LaneRef {
    pub fn new(road: RoadId, index: usize) -> Self {
        LaneRef { road, index }
    }

    fn lane_id(&self) -> LaneId {
        LaneId::of_road(&self.road, self.index)
    }
}

#[derive(Debug, Clone)]
struct RoadDraft {
    id: RoadId,
    spec: RoadSpec,
    link: RoadLink,
}

#[derive(Debug, Clone)]
enum ConnectOp {
    Direct {
        from: LaneRef,
        to: LaneRef,
    },
    ViaJunction {
        junction: JunctionId,
        from: LaneRef,
        to: LaneRef,
    },
    /// A pavement round the corner of a junction, from the sidewalk of one arm to
    /// the sidewalk of the next arm round. Made by the generator rather than the
    /// caller, and with no direction: a footway has none, so the ends are named
    /// by which end of each road they are.
    Walkway {
        junction: JunctionId,
        from: (LaneRef, RoadEnd),
        to: (LaneRef, RoadEnd),
    },
}

#[derive(Debug, Clone)]
enum ObjectSpec {
    /// A line across a lane at one of its ends, raised by `height` metres, or
    /// `setback` metres back along the lane from that end.
    AcrossLane {
        kind: MapObjectKind,
        lane: LaneRef,
        end: LaneEnd,
        height: f64,
        setback: f64,
        lanes: Vec<LaneRef>,
    },
    /// A band across a whole road at a station along it.
    AcrossRoad {
        kind: MapObjectKind,
        road: RoadId,
        station: f64,
        width: f64,
        lanes: Vec<LaneRef>,
    },
}

#[derive(Debug, Clone)]
enum RuleSpec {
    TrafficLight {
        lights: Vec<ObjectId>,
        stop_line: Option<ObjectId>,
        lanes: Vec<LaneRef>,
    },
    RightOfWay {
        right_of_way: Vec<LaneRef>,
        yielding: Vec<LaneRef>,
        stop_line: Option<ObjectId>,
    },
    SpeedLimit {
        limit: SpeedLimit,
        lanes: Vec<LaneRef>,
    },
}

/// Collects roads, movements and semantics, then generates the map.
#[derive(Debug, Clone)]
pub struct MapBuilder {
    metadata: MapMetadata,
    roads: Vec<RoadDraft>,
    road_index: HashMap<RoadId, usize>,
    junctions: Vec<Junction>,
    junction_index: HashMap<JunctionId, usize>,
    operations: Vec<ConnectOp>,
    objects: Vec<(ObjectId, ObjectSpec)>,
    object_index: HashMap<ObjectId, usize>,
    rules: Vec<RuleSpec>,
    auto_road: usize,
    auto_junction: usize,
    auto_object: usize,
}

impl Default for MapBuilder {
    fn default() -> Self {
        MapBuilder::new(MapMetadata::default())
    }
}

impl MapBuilder {
    pub fn new(metadata: MapMetadata) -> Self {
        MapBuilder {
            metadata,
            roads: Vec::new(),
            road_index: HashMap::new(),
            junctions: Vec::new(),
            junction_index: HashMap::new(),
            operations: Vec::new(),
            objects: Vec::new(),
            object_index: HashMap::new(),
            rules: Vec::new(),
            auto_road: 0,
            auto_junction: 0,
            auto_object: 0,
        }
    }

    pub fn metadata(&self) -> &MapMetadata {
        &self.metadata
    }

    pub fn metadata_mut(&mut self) -> &mut MapMetadata {
        &mut self.metadata
    }

    /// Adds a road. Its identifier comes from `spec.name`, or from the road's
    /// position if it has none, so two identical scripts name the same roads.
    pub fn add_road(&mut self, spec: RoadSpec) -> Result<RoadId, BuildError> {
        if spec
            .cross_sections
            .iter()
            .any(|section| section.lanes.is_empty())
        {
            let name = spec
                .name
                .clone()
                .unwrap_or_else(|| self.auto_road.to_string());
            return Err(BuildError::RoadWithoutLanes(RoadId::new(name)));
        }
        let key = match &spec.name {
            Some(name) => name.clone(),
            None => {
                let key = format!("r{}", self.auto_road);
                self.auto_road += 1;
                key
            }
        };
        let id = RoadId::new(key);
        if self.road_index.contains_key(&id) {
            return Err(BuildError::DuplicateId(id.to_string()));
        }
        self.road_index.insert(id.clone(), self.roads.len());
        self.roads.push(RoadDraft {
            id: id.clone(),
            spec,
            link: RoadLink::default(),
        });
        Ok(id)
    }

    pub fn add_junction(&mut self, name: Option<&str>) -> JunctionId {
        let key = match name {
            Some(name) => name.to_owned(),
            None => {
                let key = format!("j{}", self.auto_junction);
                self.auto_junction += 1;
                key
            }
        };
        let id = JunctionId::new(key);
        if !self.junction_index.contains_key(&id) {
            self.junction_index.insert(id.clone(), self.junctions.len());
            self.junctions
                .push(Junction::new(id.clone(), name.map(str::to_owned)));
        }
        id
    }

    fn draft(&self, road: &RoadId) -> Result<&RoadDraft, BuildError> {
        self.road_index
            .get(road)
            .map(|&i| &self.roads[i])
            .ok_or_else(|| BuildError::UnknownRoad(road.clone()))
    }

    fn draft_mut(&mut self, road: &RoadId) -> Result<&mut RoadDraft, BuildError> {
        let index = *self
            .road_index
            .get(road)
            .ok_or_else(|| BuildError::UnknownRoad(road.clone()))?;
        Ok(&mut self.roads[index])
    }

    fn lane_spec(&self, lane: &LaneRef) -> Result<&LaneSpec, BuildError> {
        let draft = self.draft(&lane.road)?;
        draft
            .spec
            .all_lanes()
            .nth(lane.index)
            .ok_or_else(|| BuildError::LaneIndexOutOfRange {
                road: lane.road.clone(),
                index: lane.index,
                lanes: draft.spec.all_lanes().count(),
            })
    }

    /// Which cross-section a lane belongs to, and where it sits within it.
    fn locate_lane(&self, lane: &LaneRef) -> Result<(usize, usize), BuildError> {
        let draft = self.draft(&lane.road)?;
        draft
            .spec
            .locate(lane.index)
            .ok_or_else(|| BuildError::LaneIndexOutOfRange {
                road: lane.road.clone(),
                index: lane.index,
                lanes: draft.spec.all_lanes().count(),
            })
    }

    /// The cross-section adjacent to one end of a road: the first at its start, the
    /// last at its end. Those are the lanes another road meets there.
    fn section_at_end(&self, road: &RoadId, end: RoadEnd) -> Result<usize, BuildError> {
        let draft = self.draft(road)?;
        Ok(match end {
            RoadEnd::Start => 0,
            RoadEnd::End => draft.spec.cross_sections.len() - 1,
        })
    }

    /// Global lane indices of one cross-section.
    fn lanes_of_section(&self, road: &RoadId, section: usize) -> Result<Vec<usize>, BuildError> {
        let draft = self.draft(road)?;
        let offset = draft.spec.section_offset(section);
        Ok((0..draft.spec.cross_sections[section].lanes.len())
            .map(|index| offset + index)
            .collect())
    }

    /// Where a lane sits in its road's cross-section, resolved against handedness.
    fn lane_side(&self, lane: &LaneRef) -> Result<LateralSide, BuildError> {
        let spec = self.lane_spec(lane)?;
        Ok(spec
            .side
            .unwrap_or_else(|| self.metadata.handedness.side_for(spec.direction)))
    }

    /// Rank of the lane outwards from the cross-section origin, counting from 1.
    fn lane_ordinal(&self, lane: &LaneRef) -> Result<usize, BuildError> {
        // Counted within the lane's own cross-section: a lane is the n-th out from
        // the centre of *its* section, not of the whole road.
        let side = self.lane_side(lane)?;
        let (section, _) = self.locate_lane(lane)?;
        let mut ordinal = 0;
        for index in self.lanes_of_section(&lane.road, section)? {
            let candidate = LaneRef::new(lane.road.clone(), index);
            if self.lane_side(&candidate)? == side {
                ordinal += 1;
            }
            if index == lane.index {
                break;
            }
        }
        Ok(ordinal)
    }

    /// Joins the end of `from` to the start of `to`, pairing lanes across the joint.
    ///
    /// Returns the movements it created, source lane first.
    pub fn connect(
        &mut self,
        from: &RoadId,
        to: &RoadId,
    ) -> Result<Vec<(LaneId, LaneId)>, BuildError> {
        self.connect_ends(from, RoadEnd::End, to, RoadEnd::Start, None)
    }

    /// Joins two roads through a junction, generating a connector per movement.
    pub fn connect_via(
        &mut self,
        junction: &JunctionId,
        from: &RoadId,
        to: &RoadId,
    ) -> Result<Vec<(LaneId, LaneId)>, BuildError> {
        self.connect_ends(from, RoadEnd::End, to, RoadEnd::Start, Some(junction))
    }

    /// The general form: which end of each road meets, and whether via a junction.
    pub fn connect_ends(
        &mut self,
        from: &RoadId,
        from_end: RoadEnd,
        to: &RoadId,
        to_end: RoadEnd,
        junction: Option<&JunctionId>,
    ) -> Result<Vec<(LaneId, LaneId)>, BuildError> {
        // The lanes that meet are those of the cross-section at each road's end,
        // which for a road with several sections is not all of its lanes.
        let from_section = self.section_at_end(from, from_end)?;
        let to_section = self.section_at_end(to, to_end)?;
        let from_indices = self.lanes_of_section(from, from_section)?;
        let to_indices = self.lanes_of_section(to, to_section)?;

        // A road end that points into the joint carries its reference direction
        // forwards through it; one that points away carries it backwards.
        let from_sense = if from_end == RoadEnd::End { 1.0 } else { -1.0 };
        let to_sense = if to_end == RoadEnd::Start { 1.0 } else { -1.0 };
        let sides_agree = from_sense * to_sense > 0.0;

        // Pair lanes by the side and rank they present to the joint.
        let mut to_by_slot: HashMap<(LateralSide, usize), usize> = HashMap::new();
        for index in to_indices {
            let lane = LaneRef::new(to.clone(), index);
            let mut side = self.lane_side(&lane)?;
            if !sides_agree {
                side = side.opposite();
            }
            to_by_slot.insert((side, self.lane_ordinal(&lane)?), index);
        }

        // Which lanes pair up, and whether any movement was possible at all. A
        // movement needs a lane emitting into the joint on one side and one
        // accepting from it on the other; two arms that both only feed a junction
        // have none, and joining them records their adjacency and nothing else.
        let mut created = Vec::new();
        let mut operations = Vec::new();
        let mut emits = [false, false];
        let mut accepts = [false, false];
        for index in from_indices {
            let from_lane = LaneRef::new(from.clone(), index);
            let from_spec = self.lane_spec(&from_lane)?.clone();
            let counts = junction.is_none() || from_spec.lane_type.is_drivable();
            let from_flow = from_spec.direction.sign() * from_sense;
            if counts {
                emits[0] |= from_flow > 0.0;
                accepts[0] |= from_flow < 0.0;
            }
            let slot = (self.lane_side(&from_lane)?, self.lane_ordinal(&from_lane)?);
            let Some(&to_index) = to_by_slot.get(&slot) else {
                continue;
            };
            let to_lane = LaneRef::new(to.clone(), to_index);

            // Only a lane that carries a movement goes through a junction. A
            // shoulder, a parking lane or a border stops at the arm; a pavement goes
            // round the corner, which the generator lays for every junction on its
            // own.
            if junction.is_some()
                && !(from_spec.lane_type.is_drivable()
                    && self.lane_spec(&to_lane)?.lane_type.is_drivable())
            {
                continue;
            }
            let to_flow = self.lane_spec(&to_lane)?.direction.sign() * to_sense;
            // `from` emits into the joint and `to` accepts from it, or the mirror
            // image of that for the opposing carriageway.
            let (source, target) = if from_flow > 0.0 && to_flow > 0.0 {
                (from_lane.clone(), to_lane.clone())
            } else if from_flow < 0.0 && to_flow < 0.0 {
                (to_lane.clone(), from_lane.clone())
            } else {
                // The two lanes face each other; there is no movement between them.
                continue;
            };
            created.push((source.lane_id(), target.lane_id()));
            operations.push((source, target));
        }
        for index in self.lanes_of_section(to, to_section)? {
            let to_lane = LaneRef::new(to.clone(), index);
            let spec = self.lane_spec(&to_lane)?;
            if junction.is_some() && !spec.lane_type.is_drivable() {
                continue;
            }
            let to_flow = spec.direction.sign() * to_sense;
            accepts[1] |= to_flow > 0.0;
            emits[1] |= to_flow < 0.0;
        }
        let possible = (emits[0] && accepts[1]) || (emits[1] && accepts[0]);
        if created.is_empty() && possible {
            // Nothing paired although something could have: the two cross-sections
            // do not correspond slot for slot — the usual cause is a lane list
            // written left to right instead of outwards from the reference line,
            // which puts a pavement in the rank a driving lane holds on the other
            // road. Said here, at the call, rather than left for validation to
            // trip over three steps later.
            return Err(BuildError::NoLanePairs {
                from: from.clone(),
                to: to.clone(),
            });
        }

        // Record the adjacency: two roads that meet are adjacent even when no
        // through movement pairs up, and both exporters need to know it.
        self.set_link(from, from_end, to, to_end, junction)?;
        for (source, target) in operations {
            self.push_operation(source, target, junction)?;
        }
        Ok(created)
    }

    /// Connects one specific lane to one specific lane.
    pub fn connect_lanes(
        &mut self,
        from: &LaneRef,
        to: &LaneRef,
        junction: Option<&JunctionId>,
    ) -> Result<ConnectionId, BuildError> {
        let from_direction = self.lane_spec(from)?.direction;
        let to_direction = self.lane_spec(to)?.direction;
        let from_end = match from_direction.exit_end() {
            LaneEnd::Start => RoadEnd::Start,
            LaneEnd::End => RoadEnd::End,
        };
        let to_end = match to_direction.entry_end() {
            LaneEnd::Start => RoadEnd::Start,
            LaneEnd::End => RoadEnd::End,
        };
        self.set_link(&from.road, from_end, &to.road, to_end, junction)?;
        self.push_operation(from.clone(), to.clone(), junction)?;
        Ok(ConnectionId::between(
            junction,
            &from.lane_id(),
            &to.lane_id(),
        ))
    }

    fn push_operation(
        &mut self,
        from: LaneRef,
        to: LaneRef,
        junction: Option<&JunctionId>,
    ) -> Result<(), BuildError> {
        if from.road == to.road && from.index == to.index {
            return Err(BuildError::IncompatibleConnection {
                from: from.lane_id(),
                to: to.lane_id(),
                reason: "a lane cannot connect to itself".into(),
            });
        }
        self.operations.push(match junction {
            Some(junction) => {
                if !self.junction_index.contains_key(junction) {
                    return Err(BuildError::UnknownJunction(junction.clone()));
                }
                let index = self.junction_index[junction];
                for road in [&from.road, &to.road] {
                    if !self.junctions[index].incoming_roads.contains(road) {
                        self.junctions[index].incoming_roads.push(road.clone());
                    }
                }
                ConnectOp::ViaJunction {
                    junction: junction.clone(),
                    from,
                    to,
                }
            }
            None => ConnectOp::Direct { from, to },
        });
        Ok(())
    }

    fn set_link(
        &mut self,
        from: &RoadId,
        from_end: RoadEnd,
        to: &RoadId,
        to_end: RoadEnd,
        junction: Option<&JunctionId>,
    ) -> Result<(), BuildError> {
        let (from_target, to_target) = match junction {
            Some(junction) => (
                RoadLinkTarget::Junction(junction.clone()),
                RoadLinkTarget::Junction(junction.clone()),
            ),
            None => (
                RoadLinkTarget::Road(RoadEndpoint::new(to.clone(), to_end)),
                RoadLinkTarget::Road(RoadEndpoint::new(from.clone(), from_end)),
            ),
        };
        for (road, end, target) in [(from, from_end, from_target), (to, to_end, to_target)] {
            let draft = self.draft_mut(road)?;
            match draft.link.at(end) {
                Some(existing) if existing != &target => {
                    // Two different continuations at one end is exactly what a
                    // junction is for; say so rather than silently overwriting.
                    return Err(BuildError::ConflictingRoadLink {
                        road: road.clone(),
                        detail: format!(
                            "the {} end already continues elsewhere; model the split \
                             or merge with a junction",
                            end.as_str()
                        ),
                    });
                }
                _ => draft.link.set(end, target),
            }
        }
        Ok(())
    }

    /// Adds a stop line across `lane` at one of its ends.
    pub fn add_stop_line(&mut self, lane: &LaneRef, end: LaneEnd) -> Result<ObjectId, BuildError> {
        self.add_stop_line_at(lane, end, 0.0)
    }

    /// Adds a stop line across `lane`, `setback` metres back along the lane from
    /// one of its ends — before the crosswalk at a junction mouth, say.
    pub fn add_stop_line_at(
        &mut self,
        lane: &LaneRef,
        end: LaneEnd,
        setback: f64,
    ) -> Result<ObjectId, BuildError> {
        self.lane_spec(lane)?;
        self.push_object(
            format!("stopline/{}/{}", lane.lane_id().local_name(), end.as_str()),
            ObjectSpec::AcrossLane {
                kind: MapObjectKind::StopLine,
                lane: lane.clone(),
                end,
                height: 0.0,
                setback: setback.max(0.0),
                lanes: vec![lane.clone()],
            },
        )
    }

    /// Adds a traffic light bar above `lane` at one of its ends.
    pub fn add_traffic_light(
        &mut self,
        lane: &LaneRef,
        end: LaneEnd,
        height: f64,
    ) -> Result<ObjectId, BuildError> {
        self.add_traffic_light_at(lane, end, height, 0.0)
    }

    /// Adds a traffic light bar above `lane`, `setback` metres back from one of its
    /// ends.
    pub fn add_traffic_light_at(
        &mut self,
        lane: &LaneRef,
        end: LaneEnd,
        height: f64,
        setback: f64,
    ) -> Result<ObjectId, BuildError> {
        self.lane_spec(lane)?;
        self.push_object(
            format!(
                "trafficlight/{}/{}",
                lane.lane_id().local_name(),
                end.as_str()
            ),
            ObjectSpec::AcrossLane {
                kind: MapObjectKind::TrafficLight,
                lane: lane.clone(),
                end,
                height,
                setback: setback.max(0.0),
                lanes: vec![lane.clone()],
            },
        )
    }

    /// Adds a traffic sign beside `lane` at one of its ends.
    pub fn add_traffic_sign(
        &mut self,
        lane: &LaneRef,
        end: LaneEnd,
        code: impl Into<String>,
        height: f64,
    ) -> Result<ObjectId, BuildError> {
        self.add_traffic_sign_at(lane, end, code, height, 0.0)
    }

    /// Adds a traffic sign beside `lane`, `setback` metres back from one of its
    /// ends.
    pub fn add_traffic_sign_at(
        &mut self,
        lane: &LaneRef,
        end: LaneEnd,
        code: impl Into<String>,
        height: f64,
        setback: f64,
    ) -> Result<ObjectId, BuildError> {
        self.lane_spec(lane)?;
        let code = code.into();
        self.push_object(
            format!(
                "trafficsign/{}/{}/{}",
                lane.lane_id().local_name(),
                end.as_str(),
                code
            ),
            ObjectSpec::AcrossLane {
                kind: MapObjectKind::TrafficSign { code },
                lane: lane.clone(),
                end,
                height,
                setback: setback.max(0.0),
                lanes: vec![lane.clone()],
            },
        )
    }

    /// Adds a crosswalk across `road`, `fraction` of the way along it.
    pub fn add_crosswalk(
        &mut self,
        road: &RoadId,
        fraction: f64,
        width: f64,
    ) -> Result<ObjectId, BuildError> {
        let draft = self.draft(road)?;
        let lanes = (0..draft.spec.all_lanes().count())
            .map(|index| LaneRef::new(road.clone(), index))
            .collect();
        // Resolved to a station now, so that it is one more station the road
        // carries and moves with, rather than a fraction of whatever length the
        // road ends up with.
        let station = draft.spec.reference_line.horizontal_length()? * fraction.clamp(0.0, 1.0);
        self.push_object(
            format!("crosswalk/{}/{fraction}", road.local_name()),
            ObjectSpec::AcrossRoad {
                kind: MapObjectKind::Crosswalk,
                road: road.clone(),
                station,
                width,
                lanes,
            },
        )
    }

    fn push_object(&mut self, key: String, spec: ObjectSpec) -> Result<ObjectId, BuildError> {
        let id = ObjectId::new(key);
        if self.object_index.contains_key(&id) {
            return Err(BuildError::DuplicateId(id.to_string()));
        }
        self.auto_object += 1;
        self.object_index.insert(id.clone(), self.objects.len());
        self.objects.push((id.clone(), spec));
        Ok(id)
    }

    /// Records that `lanes` are controlled by `lights`.
    pub fn add_traffic_light_rule(
        &mut self,
        lights: Vec<ObjectId>,
        stop_line: Option<ObjectId>,
        lanes: Vec<LaneRef>,
    ) {
        self.rules.push(RuleSpec::TrafficLight {
            lights,
            stop_line,
            lanes,
        });
    }

    /// Records that `yielding` gives way to `right_of_way`.
    pub fn add_right_of_way(
        &mut self,
        right_of_way: Vec<LaneRef>,
        yielding: Vec<LaneRef>,
        stop_line: Option<ObjectId>,
    ) {
        self.rules.push(RuleSpec::RightOfWay {
            right_of_way,
            yielding,
            stop_line,
        });
    }

    /// Records a speed limit that applies to a set of lanes.
    pub fn add_speed_limit_rule(&mut self, limit: SpeedLimit, lanes: Vec<LaneRef>) {
        self.rules.push(RuleSpec::SpeedLimit { limit, lanes });
    }

    /// Generates geometry and produces the map, ready for validation.
    pub fn finish(self) -> Result<UnvalidatedMap, BuildError> {
        Generator::new(self).run()
    }
}

impl Direction {
    /// `+1` along the reference line, `-1` against it.
    fn sign(self) -> f64 {
        match self {
            Direction::Forward => 1.0,
            Direction::Backward => -1.0,
        }
    }
}

/// Where each edge of one cross-section sits, as a function of station.
///
/// A cross-section is laid out by stacking lane widths outwards from the origin, and
/// with width profiles those widths vary along the road — so an edge's offset is a
/// function rather than a number.
struct SectionLayout {
    station_range: (f64, f64),
    lane_offset: Poly3Profile,
    /// Widths of the left-hand lanes, ordinal 1 first.
    left: Vec<WidthProfile>,
    /// Widths of the right-hand lanes, ordinal 1 first.
    right: Vec<WidthProfile>,
}

impl SectionLayout {
    /// The lateral offset of the edge at `rank` — `0` the cross-section origin, `+n`
    /// the n-th edge to its left, `-n` the n-th to its right — at `station`.
    fn edge_offset(&self, rank: i32, station: f64) -> f64 {
        let (widths, sign) = if rank >= 0 {
            (&self.left, 1.0)
        } else {
            (&self.right, -1.0)
        };
        let stacked: f64 = widths
            .iter()
            .take(rank.unsigned_abs() as usize)
            .map(|width| width.evaluate(station).metres())
            .sum();
        self.lane_offset.evaluate(station) + sign * stacked
    }

    /// How far the cross-section reaches from its reference line at `station`.
    fn extent(&self, station: f64) -> f64 {
        let outermost = |rank: i32| self.edge_offset(rank, station).abs();
        outermost(self.left.len() as i32).max(outermost(-(self.right.len() as i32)))
    }
}

/// The geometry of one road's reference line, plus the lateral direction to use at
/// each sampled station.
struct RoadGeometry {
    samples: Vec<Sample>,
    /// The horizontal lateral direction at each station, mitred where the road meets
    /// another. Superelevation is *not* baked in here: a joint has to be mitred in
    /// plan, and the roll is applied afterwards, per station.
    laterals: Vec<Vector3>,
    /// Superelevation at each station, radians.
    rolls: Vec<f64>,
}

impl RoadGeometry {
    fn new(
        reference_line: &Curve3,
        superelevation: &Poly3Profile,
        config: SamplingConfig,
        section_stations: &[f64],
    ) -> Result<Self, GeometryError> {
        // A cross-section has to change at the station the caller asked for, so those
        // stations get a vertex whether or not the sampler would have put one there.
        let samples = reference_line.samples_including(config, section_stations)?;
        let laterals = samples
            .iter()
            .map(|sample| Ok(sample.frame()?.left.get()))
            .collect::<Result<Vec<_>, GeometryError>>()?;
        let rolls = samples
            .iter()
            .map(|sample| superelevation.evaluate(sample.station))
            .collect();
        Ok(RoadGeometry {
            samples,
            laterals,
            rolls,
        })
    }

    /// The lateral direction to offset along at one station, once the road's
    /// superelevation has tilted it.
    ///
    /// The roll turns the lateral about the tangent, so an offset along it gains
    /// height — which is what banking is. A mitred lateral is longer than a unit
    /// vector, and the rotation preserves that.
    fn banked_lateral(&self, index: usize) -> Vector3 {
        let roll = self.rolls[index];
        if roll == 0.0 {
            return self.laterals[index];
        }
        self.laterals[index].rotated_about(self.samples[index].tangent, roll)
    }

    /// The curve that runs `offset(station)` metres to the left of the reference
    /// line, over the stations within `range`.
    ///
    /// The offset is a function because a tapering lane's boundary is not a constant
    /// distance from the reference line; the range is there because a cross-section
    /// covers only part of its road.
    fn boundary_over(
        &self,
        range: (f64, f64),
        offset: impl Fn(f64) -> f64,
    ) -> Result<Curve3, GeometryError> {
        let tolerance = Polyline3::MIN_SEGMENT;
        Ok(Curve3::Polyline(Polyline3::new(
            self.samples
                .iter()
                .enumerate()
                .filter(|(_, sample)| {
                    sample.station >= range.0 - tolerance && sample.station <= range.1 + tolerance
                })
                .map(|(index, sample)| {
                    sample.point + self.banked_lateral(index) * offset(sample.station)
                }),
        )?))
    }

    /// The lateral direction in plan at one end, before any roll. This is what a
    /// joint is mitred in: two roads have to agree on a direction across the ground
    /// whatever each of them is banked to.
    fn lateral_at(&self, end: RoadEnd) -> Vector3 {
        self.laterals[self.index_at(end)]
    }

    /// The lateral direction at one end with the road's roll applied — the direction
    /// a boundary is actually offset along there.
    fn banked_lateral_at(&self, end: RoadEnd) -> Vector3 {
        self.banked_lateral(self.index_at(end))
    }

    fn index_at(&self, end: RoadEnd) -> usize {
        match end {
            RoadEnd::Start => 0,
            RoadEnd::End => self.samples.len() - 1,
        }
    }
}

/// Turns a builder's drafts into a map.
struct Generator {
    builder: MapBuilder,
    geometry: HashMap<RoadId, RoadGeometry>,
    /// One layout per cross-section of each road, so a station can be asked where
    /// the road's edges are after the lanes have been built.
    layouts: HashMap<RoadId, Vec<SectionLayout>>,
    map: Map,
}

impl Generator {
    fn new(builder: MapBuilder) -> Self {
        let map = Map::new(builder.metadata.clone());
        Generator {
            builder,
            geometry: HashMap::new(),
            layouts: HashMap::new(),
            map,
        }
    }

    fn config(&self) -> SamplingConfig {
        self.builder.metadata.sampling
    }

    fn run(mut self) -> Result<UnvalidatedMap, BuildError> {
        self.round_corners()?;
        self.build_reference_geometry()?;
        self.mitre_joints()?;
        self.build_roads()?;
        self.pave_corners()?;
        self.build_connections()?;
        self.build_objects()?;
        self.build_rules()?;
        Ok(UnvalidatedMap::from_map(self.map))
    }

    fn build_reference_geometry(&mut self) -> Result<(), BuildError> {
        let config = self.config();
        for draft in &self.builder.roads {
            let stations = crate::map::required_stations(
                draft
                    .spec
                    .cross_sections
                    .iter()
                    .map(|section| section.station),
                draft.spec.all_lanes().map(|lane| &lane.width),
                config,
            );
            self.geometry.insert(
                draft.id.clone(),
                RoadGeometry::new(
                    &draft.spec.reference_line,
                    &draft.spec.superelevation,
                    config,
                    &stations,
                )?,
            );
        }
        Ok(())
    }

    /// Rounds off every corner where two roads meet at an angle.
    ///
    /// Two roads joined end to end with different headings are a kink, and a kink is
    /// something only one of the formats can hold. Lanelet2 is a list of points and
    /// can be cut on the slant; OpenDRIVE derives every lane from the reference line
    /// and a width measured *perpendicular* to it, so the two roads' cross-sections
    /// end on different lines however they are written — a wedge of nothing on the
    /// outside of the turn and an overlap on the inside, in a file every consumer
    /// takes at face value. So the corner is not written: each road is cut back a
    /// little and a circular arc, tangent to both, is put in its place — half on
    /// each road, so that no road is added and every link stays where it was.
    ///
    /// The arc's radius is [`CORNER_INNER_RADIUS`] plus how far the cross-section
    /// reaches on the inside of the turn, which keeps the inside kerb a curve a
    /// vehicle can follow whatever the road's width. A road that arrives at the
    /// joint on a curve, or that is too short to give up what the arc needs, is
    /// reported rather than bent: the caller can join those with an alignment of
    /// their own, or through a junction.
    fn round_corners(&mut self) -> Result<(), BuildError> {
        // Every direct joint once, from whichever side it is met first.
        let mut joints: Vec<((RoadId, RoadEnd), (RoadId, RoadEnd))> = Vec::new();
        let mut seen: HashSet<(RoadId, RoadEnd)> = HashSet::new();
        for draft in &self.builder.roads {
            for end in [RoadEnd::Start, RoadEnd::End] {
                let Some(RoadLinkTarget::Road(other)) = draft.link.at(end) else {
                    continue;
                };
                let here = (draft.id.clone(), end);
                let there = (other.road.clone(), other.end);
                if seen.contains(&there) || here.0 == there.0 {
                    continue;
                }
                seen.insert(here.clone());
                joints.push((here, there));
            }
        }
        for (a, b) in joints {
            self.round_corner(&a, &b)?;
        }
        Ok(())
    }

    fn round_corner(
        &mut self,
        a: &(RoadId, RoadEnd),
        b: &(RoadId, RoadEnd),
    ) -> Result<(), BuildError> {
        let config = self.config();
        let curve_a = self.builder.draft(&a.0)?.spec.reference_line.clone();
        let curve_b = self.builder.draft(&b.0)?.spec.reference_line.clone();

        // The flow through the joint: `into` arrives along road a, `out` leaves along
        // road b. Both horizontal, because a corner is a plan-view thing.
        let into = horizontal(into_joint(&curve_a, a.1)?)?;
        let out = horizontal(-into_joint(&curve_b, b.1)?)?;
        let angle = into.dot(out).clamp(-1.0, 1.0).acos();
        if angle < CORNER_TOLERANCE {
            return Ok(());
        }
        let too_sharp = |detail: String| BuildError::CornerTooSharp {
            from: a.0.clone(),
            to: b.0.clone(),
            angle_degrees: angle.to_degrees(),
            detail,
        };
        if angle > CORNER_MAX_ANGLE {
            return Err(too_sharp(
                "the roads all but double back on each other".into(),
            ));
        }
        // Positive is a left turn, which puts the inside of the corner on the left.
        let turn = into.get().cross(out.get()).z.signum();
        let inside = if turn > 0.0 {
            LateralSide::Left
        } else {
            LateralSide::Right
        };

        // How far the wider of the two roads reaches into the corner.
        let reach = self
            .side_extent(&a.0, a.1, inside)?
            .max(self.side_extent(&b.0, b.1, inside)?);
        let mut radius = reach + CORNER_INNER_RADIUS;
        let half_tangent = (angle / 2.0).tan();

        // What each road can give up: the straight piece it arrives on, within the
        // cross-section that reaches the joint, less a metre to remain a road.
        let spare = |road: &(RoadId, RoadEnd), curve: &Curve3| -> Result<f64, BuildError> {
            let straight = straight_run(curve, road.1).ok_or_else(|| {
                too_sharp(format!(
                    "road {} arrives at the joint on a curve; give it an alignment that \
                     is tangent-continuous with the road it meets",
                    road.0
                ))
            })?;
            let sections = &self.builder.draft(&road.0)?.spec.cross_sections;
            let section = self.builder.section_at_end(&road.0, road.1)?;
            let length = curve.horizontal_length()?;
            let span = match road.1 {
                RoadEnd::End => length - sections[section].station,
                RoadEnd::Start => sections.get(1).map_or(length, |next| next.station),
            };
            Ok(straight.min(span) - 1.0)
        };
        let available = spare(a, &curve_a)?.min(spare(b, &curve_b)?);
        if radius * half_tangent > available {
            // Shrink the arc until it fits, but not into a corner tighter than the
            // road is wide.
            radius = available / half_tangent;
            if radius < reach + 1.0 {
                return Err(too_sharp(format!(
                    "rounding it needs {:.1} m of straight road on each side and the \
                     shorter of the two has {:.1} m to spare",
                    (reach + CORNER_INNER_RADIUS) * half_tangent,
                    available.max(0.0)
                )));
            }
        }
        let setback = radius * half_tangent;
        let half_arc = radius * angle / 2.0;

        // The arc, in the direction of flow: from where road a is cut to where road
        // b is cut, through the middle of the corner.
        let trimmed_a = trim(&curve_a, a.1, setback)?;
        let trimmed_b = trim(&curve_b, b.1, setback)?;
        let start = endpoint(&trimmed_a, a.1);
        let finish = endpoint(&trimmed_b, b.1);
        let curvature = turn / radius;
        let heading = into.heading();
        let mid_z = (start.z + finish.z) / 2.0;
        let first_half = Curve3::Arc(Arc3::new(start, heading, curvature, half_arc, mid_z)?);
        let second_half = Curve3::Arc(Arc3::new(
            first_half.end_point(),
            heading + curvature * half_arc,
            curvature,
            half_arc,
            finish.z,
        )?);
        let miss = second_half.end_point().horizontal_distance_to(finish);
        if miss > Curve3::JOIN_TOLERANCE {
            return Err(too_sharp(format!(
                "the arc misses the far road by {miss:.3} m"
            )));
        }

        // Each road takes its half, the right way round for its own reference line:
        // the halves run with the flow, and a road met at its start runs against it.
        for (road, trimmed, half, against_flow) in [
            (a, trimmed_a, first_half, a.1 == RoadEnd::Start),
            (b, trimmed_b, second_half, b.1 == RoadEnd::End),
        ] {
            let half = if against_flow {
                half.reversed(config)?
            } else {
                half
            };
            let reference_line = match road.1 {
                RoadEnd::End => Curve3::composite([trimmed, half])?,
                RoadEnd::Start => Curve3::composite([half, trimmed])?,
            };
            let moved = station_map(
                road.1,
                setback,
                half_arc,
                reference_line.horizontal_length()?,
            );
            let draft = self.builder.draft_mut(&road.0)?;
            draft.spec.reference_line = reference_line;
            draft.spec.map_stations(&moved);
            for (_, object) in &mut self.builder.objects {
                if let ObjectSpec::AcrossRoad {
                    road: at, station, ..
                } = object
                {
                    if at == &road.0 {
                        *station = moved(*station);
                    }
                }
            }
        }
        Ok(())
    }

    /// How far road `road`'s cross-section at `end` reaches to `side` of the flow
    /// through that end.
    fn side_extent(
        &self,
        road: &RoadId,
        end: RoadEnd,
        side: LateralSide,
    ) -> Result<f64, BuildError> {
        let section = self.builder.section_at_end(road, end)?;
        let station = match end {
            RoadEnd::Start => 0.0,
            RoadEnd::End => self
                .builder
                .draft(road)?
                .spec
                .reference_line
                .horizontal_length()?,
        };
        // A road met at its start runs against the flow, so its left is the flow's
        // right.
        let own_side = match end {
            RoadEnd::End => side,
            RoadEnd::Start => side.opposite(),
        };
        let mut extent = 0.0;
        for index in self.builder.lanes_of_section(road, section)? {
            let lane = LaneRef::new(road.clone(), index);
            if self.builder.lane_side(&lane)? == own_side {
                extent += self
                    .builder
                    .lane_spec(&lane)?
                    .width
                    .evaluate(station)
                    .metres();
            }
        }
        Ok(extent)
    }

    /// Makes roads that meet agree on one lateral direction at the shared end.
    ///
    /// Without this, two roads whose headings differ produce boundary points that
    /// are metres apart at a joint they share — and every downstream notion of
    /// "these lanes are continuous" is built on those points coinciding.
    fn mitre_joints(&mut self) -> Result<(), BuildError> {
        // Group road ends that meet, following the direct road links.
        let mut groups: Vec<Vec<(RoadId, RoadEnd)>> = Vec::new();
        let mut group_of: HashMap<(RoadId, RoadEnd), usize> = HashMap::new();

        for draft in &self.builder.roads {
            for end in [RoadEnd::Start, RoadEnd::End] {
                let Some(RoadLinkTarget::Road(other)) = draft.link.at(end) else {
                    continue;
                };
                let a = (draft.id.clone(), end);
                let b = (other.road.clone(), other.end);
                match (group_of.get(&a).copied(), group_of.get(&b).copied()) {
                    (None, None) => {
                        let index = groups.len();
                        groups.push(vec![a.clone(), b.clone()]);
                        group_of.insert(a, index);
                        group_of.insert(b, index);
                    }
                    (Some(index), None) => {
                        groups[index].push(b.clone());
                        group_of.insert(b, index);
                    }
                    (None, Some(index)) => {
                        groups[index].push(a.clone());
                        group_of.insert(a, index);
                    }
                    (Some(left), Some(right)) if left != right => {
                        let moved = std::mem::take(&mut groups[right]);
                        for member in &moved {
                            group_of.insert(member.clone(), left);
                        }
                        groups[left].extend(moved);
                    }
                    _ => {}
                }
            }
        }

        for group in &groups {
            if group.len() < 2 {
                continue;
            }
            let reference_flow = self.flow_tangent(&group[0])?;
            let mut normals = Vec::with_capacity(group.len());
            for member in group {
                let sense = if self.flow_tangent(member)?.dot(reference_flow) >= 0.0 {
                    1.0
                } else {
                    -1.0
                };
                let own = self.geometry[&member.0].lateral_at(member.1);
                normals.push((member.clone(), sense, own * sense));
            }

            // Two roads meeting get a true mitre, so the offset boundaries actually
            // intersect. More than two cannot all meet at one line, so they adopt
            // the first road's lateral and take the kink.
            let shared = if normals.len() == 2 {
                let (a, b) = (normals[0].2, normals[1].2);
                let cosine = a.dot(b);
                if 1.0 + cosine > 0.1 {
                    (a + b) * (1.0 / (1.0 + cosine))
                } else {
                    a
                }
            } else {
                normals[0].2
            };

            for (member, sense, _) in &normals {
                let geometry = self
                    .geometry
                    .get_mut(&member.0)
                    .expect("every road has geometry by now");
                let index = geometry.index_at(member.1);
                geometry.laterals[index] = shared * *sense;
            }
        }
        Ok(())
    }

    /// The reference-line tangent at a road end, oriented along the flow through it.
    fn flow_tangent(&self, member: &(RoadId, RoadEnd)) -> Result<Vector3, BuildError> {
        let geometry = &self.geometry[&member.0];
        let index = geometry.index_at(member.1);
        Ok(geometry.samples[index].tangent.get())
    }

    fn build_roads(&mut self) -> Result<(), BuildError> {
        for index in 0..self.builder.roads.len() {
            let draft = self.builder.roads[index].clone();
            let (road, lanes) = self.build_road(
                &draft.id,
                &draft.spec,
                Poly3Profile::default(),
                None,
                draft.link.clone(),
            )?;
            self.insert_road(road, lanes)?;
            self.join_sections(&draft.id)?;
        }
        Ok(())
    }

    /// Builds a road's cross-sections and the `Road` that owns them.
    fn build_road(
        &mut self,
        road: &RoadId,
        spec: &RoadSpec,
        lane_offset: Poly3Profile,
        junction: Option<JunctionId>,
        link: RoadLink,
    ) -> Result<(Road, Vec<Lane>), BuildError> {
        let length = spec.reference_line.horizontal_length()?;
        let mut lanes: Vec<Lane> = Vec::new();
        let mut sections: Vec<CrossSection> = Vec::new();
        let mut layouts: Vec<SectionLayout> = Vec::new();

        for (index, entry) in spec.cross_sections.iter().enumerate() {
            let end = match spec.cross_sections.get(index + 1) {
                Some(next) => next.station,
                None => length,
            };
            let layout = self.lay_out_section(entry, (entry.station, end), lane_offset.clone());
            let offset = spec.section_offset(index);
            let section_lanes =
                self.build_section_lanes(road, spec, entry, index, offset, &layout)?;
            sections.push(CrossSection {
                station: entry.station,
                lanes: section_lanes.iter().map(|lane| lane.id.clone()).collect(),
            });
            lanes.extend(section_lanes);
            layouts.push(layout);
        }
        self.layouts.insert(road.clone(), layouts);

        Ok((
            Road {
                id: road.clone(),
                name: spec.name.clone(),
                reference_line: spec.reference_line.clone(),
                lane_offset,
                lanes: lanes.iter().map(|lane| lane.id.clone()).collect(),
                sections,
                junction,
                link,
                road_type: spec.road_type,
                speed_limit: spec.speed_limit,
                superelevation: spec.superelevation.clone(),
            },
            lanes,
        ))
    }

    /// Works out which side and rank each lane of one cross-section sits at.
    fn lay_out_section(
        &self,
        entry: &CrossSectionSpec,
        station_range: (f64, f64),
        lane_offset: Poly3Profile,
    ) -> SectionLayout {
        let mut layout = SectionLayout {
            station_range,
            lane_offset,
            left: Vec::new(),
            right: Vec::new(),
        };
        for lane_spec in &entry.lanes {
            let side = lane_spec.side.unwrap_or_else(|| {
                self.builder
                    .metadata
                    .handedness
                    .side_for(lane_spec.direction)
            });
            match side {
                LateralSide::Left => layout.left.push(lane_spec.width.clone()),
                LateralSide::Right => layout.right.push(lane_spec.width.clone()),
            }
        }
        layout
    }

    /// Generates the lanes of one cross-section against the road's geometry.
    fn build_section_lanes(
        &self,
        road: &RoadId,
        spec: &RoadSpec,
        entry: &CrossSectionSpec,
        section: usize,
        index_offset: usize,
        layout: &SectionLayout,
    ) -> Result<Vec<Lane>, BuildError> {
        let geometry = &self.geometry[road];
        let (mut left_count, mut right_count) = (0usize, 0usize);
        let mut lanes = Vec::with_capacity(entry.lanes.len());

        for (position, lane_spec) in entry.lanes.iter().enumerate() {
            let side = lane_spec.side.unwrap_or_else(|| {
                self.builder
                    .metadata
                    .handedness
                    .side_for(lane_spec.direction)
            });
            // A lane spans one slot of the cross-section: the edges either side of
            // it, counted outwards from the origin.
            let (left_edge, right_edge, ordinal) = match side {
                LateralSide::Left => {
                    left_count += 1;
                    (left_count as i32, left_count as i32 - 1, left_count)
                }
                LateralSide::Right => {
                    right_count += 1;
                    (1 - right_count as i32, -(right_count as i32), right_count)
                }
            };
            let index = index_offset + position;
            lanes.push(Lane {
                id: LaneId::of_road(road, index),
                road: road.clone(),
                index,
                side,
                ordinal,
                direction: lane_spec.direction,
                lane_type: lane_spec.lane_type,
                width: lane_spec.width.clone(),
                speed_limit: lane_spec.speed_limit.or(spec.speed_limit),
                section,
                station_range: layout.station_range,
                left_edge,
                right_edge,
                left_offset: layout.edge_offset(left_edge, layout.station_range.0),
                right_offset: layout.edge_offset(right_edge, layout.station_range.0),
                left_boundary: geometry
                    .boundary_over(layout.station_range, |s| layout.edge_offset(left_edge, s))?,
                right_boundary: geometry
                    .boundary_over(layout.station_range, |s| layout.edge_offset(right_edge, s))?,
                centerline: geometry.boundary_over(layout.station_range, |s| {
                    (layout.edge_offset(left_edge, s) + layout.edge_offset(right_edge, s)) / 2.0
                })?,
                left_marking: lane_spec.left_marking,
                right_marking: lane_spec.right_marking,
            });
        }
        Ok(lanes)
    }

    /// Connects each lane that carries on across a cross-section boundary.
    ///
    /// A road with several sections holds several `Lane` objects for one stretch of
    /// tarmac, so the movement from one to the next has to be recorded like any
    /// other — which is also what makes the Lanelet2 export's lanelets continuous
    /// there.
    fn join_sections(&mut self, road: &RoadId) -> Result<(), BuildError> {
        let Some(entry) = self.map.road(road) else {
            return Ok(());
        };
        let sections = entry.sections.clone();
        for pair in sections.windows(2) {
            let before: Vec<Lane> = pair[0]
                .lanes
                .iter()
                .filter_map(|id| self.map.lanes.get(id).cloned())
                .collect();
            let after: Vec<Lane> = pair[1]
                .lanes
                .iter()
                .filter_map(|id| self.map.lanes.get(id).cloned())
                .collect();
            for earlier in &before {
                // A lane carries on if the next section has one in the same slot
                // going the same way; otherwise it ends here, which is the point of
                // having a second section.
                let Some(later) = after.iter().find(|candidate| {
                    candidate.side == earlier.side
                        && candidate.ordinal == earlier.ordinal
                        && candidate.direction == earlier.direction
                }) else {
                    continue;
                };
                let (source, target) = match earlier.direction {
                    Direction::Forward => (earlier, later),
                    Direction::Backward => (later, earlier),
                };
                self.push_connection(
                    None,
                    LaneEndpoint::new(source.id.clone(), source.direction.exit_end()),
                    LaneEndpoint::new(target.id.clone(), target.direction.entry_end()),
                )?;
            }
        }
        Ok(())
    }

    fn insert_road(&mut self, road: Road, lanes: Vec<Lane>) -> Result<(), BuildError> {
        let id = road.id.clone();
        self.map
            .roads
            .insert(id.clone(), road)
            .map_err(|duplicate| BuildError::DuplicateId(duplicate.0.to_string()))?;
        for lane in lanes {
            let lane_id = lane.id.clone();
            self.map
                .lanes
                .insert(lane_id, lane)
                .map_err(|duplicate| BuildError::DuplicateId(duplicate.0.to_string()))?;
        }
        Ok(())
    }

    fn build_connections(&mut self) -> Result<(), BuildError> {
        for junction in &self.builder.junctions {
            self.map
                .junctions
                .insert(junction.id.clone(), junction.clone())
                .map_err(|duplicate| BuildError::DuplicateId(duplicate.0.to_string()))?;
        }

        for operation in self.builder.operations.clone() {
            match operation {
                ConnectOp::Direct { from, to } => {
                    let from_lane = self.lane(&from)?.clone();
                    let to_lane = self.lane(&to)?.clone();
                    self.push_connection(
                        None,
                        LaneEndpoint::new(from_lane.id.clone(), from_lane.direction.exit_end()),
                        LaneEndpoint::new(to_lane.id.clone(), to_lane.direction.entry_end()),
                    )?;
                }
                ConnectOp::ViaJunction { junction, from, to } => {
                    self.build_connector(&junction, &from, &to)?
                }
                ConnectOp::Walkway { junction, from, to } => {
                    self.build_walkway(&junction, &from, &to)?
                }
            }
        }
        Ok(())
    }

    /// Lays a pavement round every corner of every junction.
    ///
    /// The arms of a junction are taken in the order they stand round it, and
    /// between each arm and the next the outermost sidewalk on the side facing the
    /// corner is joined to the next arm's on its facing side — the pavement a
    /// pedestrian walks round rather than the road they cross. An arm with no
    /// sidewalk on that side gets no pavement there, and a junction of one arm
    /// has no corners.
    fn pave_corners(&mut self) -> Result<(), BuildError> {
        // Every road end that arrives at each junction, with the heading it points
        // away from it.
        let mut arms_of: HashMap<JunctionId, Vec<(RoadId, RoadEnd, f64)>> = HashMap::new();
        for draft in &self.builder.roads {
            for end in [RoadEnd::Start, RoadEnd::End] {
                let Some(RoadLinkTarget::Junction(junction)) = draft.link.at(end) else {
                    continue;
                };
                let outward = -into_joint(&draft.spec.reference_line, end)?;
                arms_of.entry(junction.clone()).or_default().push((
                    draft.id.clone(),
                    end,
                    outward.y.atan2(outward.x),
                ));
            }
        }
        let mut walkways = Vec::new();
        for junction in &self.builder.junctions {
            let Some(arms) = arms_of.get_mut(&junction.id) else {
                continue;
            };
            // Only between arms the junction knows as its own. An arm is listed when
            // a movement was connected through it; a pavement between arms nothing
            // else joins would be a connection into a junction that does not list
            // its roads, which validation rightly refuses — and would blame the
            // pavement for whatever left the arm unconnected.
            arms.retain(|(road, _, _)| junction.incoming_roads.contains(road));
            if arms.len() < 2 {
                continue;
            }
            arms.sort_by(|a, b| a.2.total_cmp(&b.2));
            for index in 0..arms.len() {
                let (a_road, a_end, _) = &arms[index];
                let (b_road, b_end, _) = &arms[(index + 1) % arms.len()];
                // Looking out along `a`, the next arm anticlockwise is on its left;
                // looking out along `b`, `a` is on its right. In each road's own
                // terms that is its left when it starts at the junction and its
                // right when it ends there, and the other way round for `b`.
                let a_side = match a_end {
                    RoadEnd::Start => LateralSide::Left,
                    RoadEnd::End => LateralSide::Right,
                };
                let b_side = match b_end {
                    RoadEnd::Start => LateralSide::Right,
                    RoadEnd::End => LateralSide::Left,
                };
                let (Some(from), Some(to)) = (
                    self.outer_sidewalk(a_road, *a_end, a_side)?,
                    self.outer_sidewalk(b_road, *b_end, b_side)?,
                ) else {
                    continue;
                };
                walkways.push(ConnectOp::Walkway {
                    junction: junction.id.clone(),
                    from: (from, *a_end),
                    to: (to, *b_end),
                });
            }
        }
        self.builder.operations.extend(walkways);
        Ok(())
    }

    /// The outermost sidewalk lane at one end of a road, on one of its sides.
    fn outer_sidewalk(
        &self,
        road: &RoadId,
        end: RoadEnd,
        side: LateralSide,
    ) -> Result<Option<LaneRef>, BuildError> {
        let section = self.builder.section_at_end(road, end)?;
        let mut outermost: Option<(usize, LaneRef)> = None;
        for index in self.builder.lanes_of_section(road, section)? {
            let lane = LaneRef::new(road.clone(), index);
            if self.builder.lane_spec(&lane)?.lane_type != LaneType::Sidewalk
                || self.builder.lane_side(&lane)? != side
            {
                continue;
            }
            let ordinal = self.builder.lane_ordinal(&lane)?;
            if outermost.as_ref().is_none_or(|(rank, _)| ordinal > *rank) {
                outermost = Some((ordinal, lane));
            }
        }
        Ok(outermost.map(|(_, lane)| lane))
    }

    /// Draws the pavement round one corner of a junction: from the sidewalk `from`
    /// at its `from.1` end to the sidewalk `to` at its `to.1` end.
    fn build_walkway(
        &mut self,
        junction: &JunctionId,
        from: &(LaneRef, RoadEnd),
        to: &(LaneRef, RoadEnd),
    ) -> Result<(), BuildError> {
        let from_lane = self.lane(&from.0)?.clone();
        let to_lane = self.lane(&to.0)?.clone();
        self.build_junction_road(junction, &from_lane, from.1, &to_lane, to.1)
    }

    fn lane(&self, reference: &LaneRef) -> Result<&Lane, BuildError> {
        self.map
            .lanes
            .get(&reference.lane_id())
            .ok_or_else(|| BuildError::UnknownLane(reference.lane_id()))
    }

    fn push_connection(
        &mut self,
        junction: Option<&JunctionId>,
        from: LaneEndpoint,
        to: LaneEndpoint,
    ) -> Result<ConnectionId, BuildError> {
        let id = ConnectionId::between(junction, &from.lane, &to.lane);
        if self.map.connections.contains(&id) {
            return Ok(id);
        }
        self.map
            .connections
            .insert(
                id.clone(),
                LaneConnection {
                    id: id.clone(),
                    from,
                    to,
                    junction: junction.cloned(),
                },
            )
            .map_err(|duplicate| BuildError::DuplicateId(duplicate.0.to_string()))?;
        Ok(id)
    }

    /// Draws the road that carries one movement through a junction.
    fn build_connector(
        &mut self,
        junction: &JunctionId,
        from: &LaneRef,
        to: &LaneRef,
    ) -> Result<(), BuildError> {
        let from_lane = self.lane(from)?.clone();
        let to_lane = self.lane(to)?.clone();
        let from_end = road_end_of(from_lane.direction.exit_end());
        let to_end = road_end_of(to_lane.direction.entry_end());
        self.build_junction_road(junction, &from_lane, from_end, &to_lane, to_end)
    }

    /// Draws a road through a junction, out of the `from_end` end of one lane and
    /// into the `to_end` end of another.
    ///
    /// The new road is a Hermite curve between the two ends, leaving the first lane
    /// along it and arriving at the second along it, one lane wide, tapering from
    /// the first lane's width there to the second's, of the first lane's type. A
    /// traffic connector leaves a lane at its exit end and enters one at its entry
    /// end; a pavement round a corner has no such ends, and is given them.
    fn build_junction_road(
        &mut self,
        junction: &JunctionId,
        from_lane: &Lane,
        from_end: RoadEnd,
        to_lane: &Lane,
        to_end: RoadEnd,
    ) -> Result<(), BuildError> {
        let config = self.config();
        // Out through `from_end` and in through `to_end`, in the reference line's
        // own terms: a road's start is left against its direction and entered
        // along it. The sign says whether the new road runs with the lane's
        // reference line there (`+1`) or against it.
        let start = from_lane.endpoint(from_end.as_lane_end());
        let start_tangent = horizontal(into_joint(&from_lane.centerline, from_end)?)?;
        let end = to_lane.endpoint(to_end.as_lane_end());
        let end_tangent = horizontal(into_joint(&to_lane.centerline, to_end)?)?.reversed();
        let sign = |end: RoadEnd| match end {
            RoadEnd::End => 1.0,
            RoadEnd::Start => -1.0,
        };
        let (start_sign, end_sign) = (sign(from_end), -sign(to_end));
        let start_width = from_lane.width_at_end(from_end.as_lane_end());
        let end_width = to_lane.width_at_end(to_end.as_lane_end());

        // The connector is realised at the map's resolution, which is fixed into the
        // curve: a Bézier's length is its vertices walked end to end, so the
        // resolution has to be settled before anything asks how long it is.
        let reference_line = Curve3::Bezier(Bezier3::hermite(
            start,
            start_tangent,
            end,
            end_tangent,
            config,
        )?);

        let road_id = RoadId::new(format!(
            "{}/{}_{}",
            junction.local_name(),
            from_lane.id.local_name(),
            to_lane.id.local_name()
        ));

        let mut geometry =
            RoadGeometry::new(&reference_line, &Poly3Profile::default(), config, &[])?;
        // Adopt the lateral direction of each road it meets, so the connector's
        // boundary endpoints land exactly on theirs. The banked direction, not the
        // plan one: the endpoints have to land on the approach's, which a banked road
        // lifts off the horizontal.
        let start_lateral = self.geometry[&from_lane.road].banked_lateral_at(from_end) * start_sign;
        let end_lateral = self.geometry[&to_lane.road].banked_lateral_at(to_end) * end_sign;
        let last = geometry.laterals.len() - 1;
        geometry.laterals[0] = start_lateral;
        geometry.laterals[last] = end_lateral;
        self.geometry.insert(road_id.clone(), geometry);

        // The connector carries the source lane's width into the target's. Where the
        // two differ — a slip road feeding a wider carriageway — it tapers between
        // them rather than stopping short of one of its neighbours.
        let width = WidthProfile::tapered(
            0.0,
            reference_line.horizontal_length()?,
            PositiveWidth::new(start_width)?,
            PositiveWidth::new(end_width)?,
            Taper::Linear,
        )?;
        // The lane straddles the connector's reference line. It goes on the side a
        // forward lane takes under the map's handedness — right under right-hand
        // traffic, left under left-hand — because OpenDRIVE derives a lane's travel
        // direction from its side and the road's rule: a connector written on the
        // wrong side would read as running against its own reference line. The
        // cross-section origin then sits half a lane the other way, so that the lane
        // is centred on the reference line, and follows the taper.
        let side = self.map.metadata.handedness.side_for(Direction::Forward);
        let half_width = width.to_poly3(0.0).scaled(0.5);
        let lane_offset = match side {
            LateralSide::Right => half_width,
            LateralSide::Left => half_width.scaled(-1.0),
        };

        let spec = RoadSpec {
            name: None,
            reference_line: reference_line.clone(),
            cross_sections: vec![CrossSectionSpec {
                station: 0.0,
                lanes: vec![LaneSpec {
                    width,
                    direction: Direction::Forward,
                    lane_type: from_lane.lane_type,
                    speed_limit: from_lane.speed_limit,
                    side: Some(side),
                    left_marking: BoundaryMarking::new(
                        RoadMarking::None,
                        from_lane.left_marking.color,
                    ),
                    right_marking: BoundaryMarking::new(
                        RoadMarking::None,
                        from_lane.right_marking.color,
                    ),
                }],
            }],
            road_type: self
                .map
                .road(&from_lane.road)
                .map(|road| road.road_type)
                .unwrap_or(RoadType::Town),
            speed_limit: from_lane.speed_limit,
            superelevation: Poly3Profile::default(),
        };
        let (road, lanes) = self.build_road(
            &road_id,
            &spec,
            lane_offset,
            Some(junction.clone()),
            RoadLink {
                predecessor: Some(RoadLinkTarget::Road(RoadEndpoint::new(
                    from_lane.road.clone(),
                    from_end,
                ))),
                successor: Some(RoadLinkTarget::Road(RoadEndpoint::new(
                    to_lane.road.clone(),
                    to_end,
                ))),
            },
        )?;
        let connector_lane = lanes[0].id.clone();
        self.insert_road(road, lanes)?;

        if let Some(entry) = self.map.junctions.get_mut(junction) {
            entry.connecting_roads.push(road_id.clone());
        } else {
            return Err(BuildError::UnknownJunction(junction.clone()));
        }

        self.push_connection(
            Some(junction),
            LaneEndpoint::new(from_lane.id.clone(), from_end.as_lane_end()),
            LaneEndpoint::new(connector_lane.clone(), LaneEnd::Start),
        )?;
        self.push_connection(
            Some(junction),
            LaneEndpoint::new(connector_lane, LaneEnd::End),
            LaneEndpoint::new(to_lane.id.clone(), to_end.as_lane_end()),
        )?;
        Ok(())
    }

    fn build_objects(&mut self) -> Result<(), BuildError> {
        let config = self.config();
        for (id, spec) in self.builder.objects.clone() {
            let object = match spec {
                ObjectSpec::AcrossLane {
                    kind,
                    lane,
                    end,
                    height,
                    setback,
                    lanes,
                } => {
                    let lane = self.lane(&lane)?.clone();
                    let travel = lane.travel_geometry(config)?;
                    let at_exit = end == lane.direction.exit_end();
                    // `setback` metres back into the lane from the end, measured
                    // along each boundary; the lane's own polyline is the truth about
                    // where a point that far back is, curve or no curve.
                    let back = |curve: &Curve3| -> Result<Point3, BuildError> {
                        let polyline = curve.to_polyline(config)?;
                        Ok(point_back_from_end(&polyline, at_exit, setback))
                    };
                    let (left, right) = (back(&travel.left)?, back(&travel.right)?);
                    // Height is measured away from the road surface, not straight up:
                    // that is what "five metres above the road" means on a slope, and
                    // it is the quantity OpenDRIVE's `zOffset` carries.
                    let (start, finish) = lane.station_range;
                    let station = match end {
                        LaneEnd::End => (finish - setback).max(start),
                        LaneEnd::Start => (start + setback).min(finish),
                    };
                    let up = self
                        .map
                        .road(&lane.road)
                        .ok_or_else(|| BuildError::UnknownRoad(lane.road.clone()))
                        .and_then(|road| Ok(road.frame_at(station, config)?.up.scaled(height)))?;
                    let raise = |point: Point3| point + up;
                    MapObject {
                        id: id.clone(),
                        kind,
                        geometry: ObjectGeometry::Line(Curve3::polyline([
                            raise(left),
                            raise(right),
                        ])?),
                        lanes: self.lane_ids(&lanes)?,
                    }
                }
                ObjectSpec::AcrossRoad {
                    kind,
                    road,
                    station,
                    width,
                    lanes,
                } => {
                    // The station the caller asked for, not the nearest vertex to
                    // it: a straight road has only two, and rounding to one of them
                    // would put a crosswalk at the very end of it.
                    let entry = self
                        .map
                        .road(&road)
                        .ok_or_else(|| BuildError::UnknownRoad(road.clone()))?;
                    let length = entry.horizontal_length()?;
                    let half = (width / 2.0).min(length / 2.0);
                    let station = station.clamp(half, length - half);
                    let frame = entry.frame_at(station, config)?;
                    let lateral = frame.left.get();
                    // How far the road reaches at *this* station, which a tapering
                    // cross-section makes a different question at every one.
                    let extent = self.layouts[&road]
                        .iter()
                        .filter(|layout| {
                            station >= layout.station_range.0 && station <= layout.station_range.1
                        })
                        .map(|layout| layout.extent(station))
                        .fold(0.0_f64, f64::max);
                    let along = frame.tangent.scaled(width / 2.0);
                    let edge = |sign: f64| -> Result<Curve3, GeometryError> {
                        let center = frame.origin + along * sign;
                        Curve3::polyline([center + lateral * extent, center - lateral * extent])
                    };
                    MapObject {
                        id: id.clone(),
                        kind,
                        geometry: ObjectGeometry::Band {
                            left: edge(-1.0)?,
                            right: edge(1.0)?,
                        },
                        lanes: self.lane_ids(&lanes)?,
                    }
                }
            };
            self.map
                .objects
                .insert(id, object)
                .map_err(|duplicate| BuildError::DuplicateId(duplicate.0.to_string()))?;
        }
        Ok(())
    }

    fn lane_ids(&self, lanes: &[LaneRef]) -> Result<Vec<LaneId>, BuildError> {
        lanes
            .iter()
            .map(|lane| Ok(self.lane(lane)?.id.clone()))
            .collect()
    }

    fn build_rules(&mut self) -> Result<(), BuildError> {
        for spec in self.builder.rules.clone() {
            let rule = match spec {
                RuleSpec::TrafficLight {
                    lights,
                    stop_line,
                    lanes,
                } => TrafficRule::TrafficLight {
                    lights,
                    stop_line,
                    lanes: self.lane_ids(&lanes)?,
                },
                RuleSpec::RightOfWay {
                    right_of_way,
                    yielding,
                    stop_line,
                } => TrafficRule::RightOfWay {
                    right_of_way: self.lane_ids(&right_of_way)?,
                    yielding: self.lane_ids(&yielding)?,
                    stop_line,
                },
                RuleSpec::SpeedLimit { limit, lanes } => TrafficRule::SpeedLimit {
                    limit,
                    lanes: self.lane_ids(&lanes)?,
                },
            };
            self.map.rules.push(rule);
        }
        Ok(())
    }
}

fn road_end_of(end: LaneEnd) -> RoadEnd {
    match end {
        LaneEnd::Start => RoadEnd::Start,
        LaneEnd::End => RoadEnd::End,
    }
}

/// The radius of the inside edge of a rounded corner, metres.
///
/// Two roads that meet at an angle are joined by an arc rather than a kink (see
/// `Generator::round_corners`). The arc's radius is measured to the reference line
/// and is this plus however far the cross-section reaches on the inside of the turn,
/// so the kerb on the inside is this tight and no tighter, whatever the road's width.
pub const CORNER_INNER_RADIUS: f64 = 6.0;

/// Below this, two headings count as the same and the joint is left as it is.
const CORNER_TOLERANCE: f64 = 1e-3;

/// Above this, two roads meet too sharply to be rounded into one another.
const CORNER_MAX_ANGLE: f64 = 150.0 * std::f64::consts::PI / 180.0;

/// The direction along which traffic leaves a curve through `end`.
fn into_joint(curve: &Curve3, end: RoadEnd) -> Result<Vector3, GeometryError> {
    Ok(match end {
        RoadEnd::End => curve.end_tangent()?.get(),
        RoadEnd::Start => -curve.start_tangent()?.get(),
    })
}

/// The direction's shadow on the horizontal plane, as a unit vector.
fn horizontal(vector: Vector3) -> Result<UnitVector3, GeometryError> {
    Vector3::new(vector.x, vector.y, 0.0).normalize()
}

fn endpoint(curve: &Curve3, end: RoadEnd) -> Point3 {
    match end {
        RoadEnd::Start => curve.start_point(),
        RoadEnd::End => curve.end_point(),
    }
}

/// The straight piece a curve arrives at `end` on, as the two points of its last
/// segment with the end itself second, or `None` when it arrives on a curve.
fn last_segment(curve: &Curve3, end: RoadEnd) -> Option<[Point3; 2]> {
    match curve {
        Curve3::Line(line) => Some(match end {
            RoadEnd::End => [line.start(), line.end()],
            RoadEnd::Start => [line.end(), line.start()],
        }),
        Curve3::Polyline(polyline) => {
            let points = polyline.points();
            Some(match end {
                RoadEnd::End => [points[points.len() - 2], points[points.len() - 1]],
                RoadEnd::Start => [points[1], points[0]],
            })
        }
        Curve3::Composite(segments) => {
            let piece = match end {
                RoadEnd::End => segments.last()?,
                RoadEnd::Start => segments.first()?,
            };
            last_segment(piece, end)
        }
        Curve3::Arc(_) | Curve3::Clothoid(_) | Curve3::Bezier(_) => None,
    }
}

/// Horizontal length of the straight piece a curve arrives at `end` on, or `None`
/// when it arrives on a curve.
fn straight_run(curve: &Curve3, end: RoadEnd) -> Option<f64> {
    let [from, to] = last_segment(curve, end)?;
    Some(from.horizontal_distance_to(to))
}

/// The curve with `setback` metres taken off its `end`. Only ever asked of a curve
/// that `straight_run` said arrives straight, and for less than that run.
fn trim(curve: &Curve3, end: RoadEnd, setback: f64) -> Result<Curve3, GeometryError> {
    // The end point, moved back along the segment it ends.
    let shortened = |[from, to]: [Point3; 2]| -> Point3 {
        let run = from.horizontal_distance_to(to);
        from.lerp(to, (run - setback) / run)
    };
    match curve {
        Curve3::Line(_) | Curve3::Polyline(_) => {
            let mut points = match curve {
                Curve3::Line(line) => vec![line.start(), line.end()],
                Curve3::Polyline(polyline) => polyline.points().to_vec(),
                _ => unreachable!(),
            };
            let segment = last_segment(curve, end).ok_or(GeometryError::NoHorizontalExtent)?;
            let index = match end {
                RoadEnd::End => points.len() - 1,
                RoadEnd::Start => 0,
            };
            points[index] = shortened(segment);
            match curve {
                Curve3::Line(_) => Curve3::line(points[0], points[1]),
                _ => Curve3::polyline(points),
            }
        }
        Curve3::Composite(segments) => {
            let mut segments = segments.clone();
            let index = match end {
                RoadEnd::End => segments.len() - 1,
                RoadEnd::Start => 0,
            };
            segments[index] = trim(&segments[index], end, setback)?;
            Curve3::composite(segments)
        }
        Curve3::Arc(_) | Curve3::Clothoid(_) | Curve3::Bezier(_) => {
            Err(GeometryError::NoHorizontalExtent)
        }
    }
}

/// Where a station on a road is after `setback` metres at `end` were replaced by
/// an arc `half_arc` metres long, leaving the road `new_length` long.
///
/// Stations on the part of the road that was kept move rigidly; a station that was
/// on the part cut away is spread over the arc that replaced it, so it stays on the
/// road and keeps its order.
fn station_map(end: RoadEnd, setback: f64, half_arc: f64, new_length: f64) -> impl Fn(f64) -> f64 {
    move |station: f64| -> f64 {
        let moved = match end {
            RoadEnd::Start => {
                if station < setback {
                    station * half_arc / setback
                } else {
                    station - setback + half_arc
                }
            }
            RoadEnd::End => {
                let kept = new_length - half_arc;
                if station <= kept {
                    station
                } else {
                    kept + (station - kept) * half_arc / setback
                }
            }
        };
        moved.clamp(0.0, new_length)
    }
}

impl RoadSpec {
    /// Moves everything the spec places by station through `map`: its cross-section
    /// boundaries, its lanes' width knots and its roll profile.
    fn map_stations(&mut self, map: &impl Fn(f64) -> f64) {
        for section in &mut self.cross_sections {
            section.station = map(section.station);
            for lane in &mut section.lanes {
                let taper = lane.width.taper();
                let knots: Vec<(f64, PositiveWidth)> = lane
                    .width
                    .knots()
                    .iter()
                    .map(|(station, width)| (map(*station), *width))
                    .collect();
                if let Ok(width) = WidthProfile::new(knots, taper) {
                    lane.width = width;
                }
            }
        }
        let pieces: Vec<Poly3Piece> = self
            .superelevation
            .pieces()
            .iter()
            .map(|piece| Poly3Piece::new(map(piece.station), piece.a, piece.b, piece.c, piece.d))
            .collect();
        if let Ok(profile) = Poly3Profile::new(pieces) {
            self.superelevation = profile;
        }
    }
}

/// The point `distance` metres back along `polyline` from one of its ends: from
/// the last vertex when `from_last`, else from the first. Clamped to the other end.
fn point_back_from_end(polyline: &Polyline3, from_last: bool, distance: f64) -> Point3 {
    let points = polyline.points();
    let ordered: Vec<Point3> = if from_last {
        points.iter().rev().copied().collect()
    } else {
        points.to_vec()
    };
    let mut remaining = distance.max(0.0);
    for pair in ordered.windows(2) {
        let length = pair[0].distance_to(pair[1]);
        if remaining <= length {
            return if length > 0.0 {
                pair[0].lerp(pair[1], remaining / length)
            } else {
                pair[0]
            };
        }
        remaining -= length;
    }
    *ordered
        .last()
        .expect("a polyline has at least two vertices")
}

#[cfg(test)]
mod tests {
    use super::*;

    fn width(value: f64) -> PositiveWidth {
        PositiveWidth::new(value).unwrap()
    }

    fn two_way() -> Vec<LaneSpec> {
        vec![
            LaneSpec::new(width(3.5), Direction::Forward),
            LaneSpec::new(width(3.5), Direction::Backward),
        ]
    }

    /// A straight two-way road between two points, added to the builder.
    fn straight(builder: &mut MapBuilder, from: Point3, to: Point3, name: &str) -> RoadId {
        builder
            .add_road(RoadSpec::line(from, to, two_way()).unwrap().with_name(name))
            .unwrap()
    }

    #[test]
    fn right_hand_traffic_puts_the_forward_lane_on_the_right() {
        let mut builder = MapBuilder::default();
        let road = builder
            .add_road(
                RoadSpec::line(
                    Point3::new(0.0, 0.0, 0.0),
                    Point3::new(100.0, 0.0, 0.0),
                    two_way(),
                )
                .unwrap()
                .with_name("main"),
            )
            .unwrap();
        let map = builder.finish().unwrap().into_map();

        let lanes = map.lanes_of(&road);
        assert_eq!(lanes[0].side, LateralSide::Right);
        assert!((lanes[0].center_offset() + 1.75).abs() < 1e-9);
        assert_eq!(lanes[1].side, LateralSide::Left);
        assert!((lanes[1].center_offset() - 1.75).abs() < 1e-9);
    }

    #[test]
    fn a_bend_is_rounded_into_an_arc_tangent_to_both_roads() {
        let mut builder = MapBuilder::default();
        let a = straight(
            &mut builder,
            Point3::new(0.0, 0.0, 10.0),
            Point3::new(100.0, 0.0, 12.0),
            "a",
        );
        let b = straight(
            &mut builder,
            Point3::new(100.0, 0.0, 12.0),
            Point3::new(200.0, 50.0, 15.0),
            "b",
        );
        builder.connect(&a, &b).unwrap();
        let map = builder.finish().unwrap().into_map();

        let a = &map.road(&a).unwrap().reference_line;
        let b = &map.road(&b).unwrap().reference_line;
        // Both roads still start and end where they were asked to.
        assert!(a.start_point().is_close(Point3::new(0.0, 0.0, 10.0), 1e-9));
        assert!(b.end_point().is_close(Point3::new(200.0, 50.0, 15.0), 1e-9));
        // They meet along one tangent, so there is no kink left to mitre.
        assert!(a.end_point().is_close(b.start_point(), 1e-6));
        assert!(a.end_tangent().unwrap().dot(b.start_tangent().unwrap()) > 1.0 - 1e-9);
        // The setback is the corner's radius against half the turn, on each road.
        let angle = 50.0_f64.atan2(100.0);
        let radius = 3.5 + CORNER_INNER_RADIUS;
        let expected = 100.0 - radius * (angle / 2.0).tan() + radius * angle / 2.0;
        assert!((a.horizontal_length().unwrap() - expected).abs() < 1e-6);
        // And the height carries straight through the arc.
        let middle = a.end_point().z;
        assert!(
            middle > 11.9 && middle < 12.1,
            "the joint sits at z = {middle}"
        );
    }

    #[test]
    fn a_road_met_at_its_start_is_rounded_the_same_way() {
        // Road b is drawn towards the joint rather than away from it, so its
        // reference line runs against the flow through the corner.
        let mut builder = MapBuilder::default();
        let a = straight(
            &mut builder,
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(100.0, 0.0, 0.0),
            "a",
        );
        let b = straight(
            &mut builder,
            Point3::new(100.0, 100.0, 0.0),
            Point3::new(100.0, 0.0, 0.0),
            "b",
        );
        builder
            .connect_ends(&a, RoadEnd::End, &b, RoadEnd::End, None)
            .unwrap();
        let map = builder.finish().unwrap().into_map();

        let a = &map.road(&a).unwrap().reference_line;
        let b = &map.road(&b).unwrap().reference_line;
        assert!(a.end_point().is_close(b.end_point(), 1e-6));
        // Tangents oppose: one arrives where the other, run backwards, arrives too.
        assert!(a.end_tangent().unwrap().dot(b.end_tangent().unwrap()) < -1.0 + 1e-9);
        assert!(b
            .start_point()
            .is_close(Point3::new(100.0, 100.0, 0.0), 1e-9));
        let radius = 3.5 + CORNER_INNER_RADIUS;
        let expected = 100.0 - radius + radius * std::f64::consts::FRAC_PI_4;
        assert!((b.horizontal_length().unwrap() - expected).abs() < 1e-6);
    }

    #[test]
    fn what_a_road_places_by_station_moves_with_the_rounded_corner() {
        let mut builder = MapBuilder::default();
        let a = straight(
            &mut builder,
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(100.0, 0.0, 0.0),
            "a",
        );
        // A second cross-section 30 m in from the joint end of road b.
        let b = builder
            .add_road(
                RoadSpec::line(
                    Point3::new(100.0, 0.0, 0.0),
                    Point3::new(100.0, 100.0, 0.0),
                    two_way(),
                )
                .unwrap()
                .with_cross_section(30.0, two_way())
                .with_name("b"),
            )
            .unwrap();
        builder.connect(&a, &b).unwrap();
        let map = builder.finish().unwrap().into_map();

        let road = map.road(&b).unwrap();
        let radius = 3.5 + CORNER_INNER_RADIUS;
        let (setback, half_arc) = (radius, radius * std::f64::consts::FRAC_PI_4);
        assert!((road.sections[1].station - (30.0 - setback + half_arc)).abs() < 1e-9);
        // Which is the same place on the ground: 30 m short of the far end.
        let there = road
            .reference_line
            .sample_at(road.sections[1].station, SamplingConfig::default())
            .unwrap();
        assert!(
            there.point.is_close(Point3::new(100.0, 30.0, 0.0), 1e-6),
            "{:?}",
            there.point
        );
    }

    #[test]
    fn a_corner_too_tight_to_round_is_refused_with_a_reason() {
        let mut builder = MapBuilder::default();
        let a = straight(
            &mut builder,
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(100.0, 0.0, 0.0),
            "a",
        );
        // The arc shrinks to fit a short road, down to a corner as tight as the road
        // is wide — and four metres of road is not even that.
        let b = straight(
            &mut builder,
            Point3::new(100.0, 0.0, 0.0),
            Point3::new(100.0, 4.0, 0.0),
            "b",
        );
        builder.connect(&a, &b).unwrap();
        let error = match builder.finish() {
            Ok(_) => panic!("the corner should be refused"),
            Err(error) => error,
        };
        assert!(
            matches!(error, BuildError::CornerTooSharp { .. }),
            "{error}"
        );
        assert!(error.to_string().contains("90.0°"), "{error}");
    }

    #[test]
    fn only_a_lane_that_carries_traffic_goes_through_a_junction() {
        // A road with a driving lane, a parking lane and a shoulder either side
        // meets another such road through a junction. The driving lanes get their
        // connector each way; the parking lanes and shoulders stop at the arm,
        // because nothing moves along them to be carried through.
        let mut builder = MapBuilder::default();
        let street = || {
            vec![
                LaneSpec::new(width(2.0), Direction::Backward).with_type(LaneType::Shoulder),
                LaneSpec::new(width(2.5), Direction::Backward).with_type(LaneType::Parking),
                LaneSpec::new(width(3.5), Direction::Backward),
                LaneSpec::new(width(3.5), Direction::Forward),
                LaneSpec::new(width(2.5), Direction::Forward).with_type(LaneType::Parking),
                LaneSpec::new(width(2.0), Direction::Forward).with_type(LaneType::Shoulder),
            ]
        };
        let junction = builder.add_junction(Some("j"));
        let a = builder
            .add_road(
                RoadSpec::line(Point3::ORIGIN, Point3::new(100.0, 0.0, 0.0), street()).unwrap(),
            )
            .unwrap();
        let b = builder
            .add_road(
                RoadSpec::line(
                    Point3::new(120.0, 0.0, 0.0),
                    Point3::new(220.0, 0.0, 0.0),
                    street(),
                )
                .unwrap(),
            )
            .unwrap();
        builder.connect_via(&junction, &a, &b).unwrap();
        let map = builder.finish().unwrap().validate().unwrap();

        let connectors: Vec<&Road> = map
            .roads
            .iter()
            .filter(|road| road.is_connector())
            .collect();
        assert_eq!(connectors.len(), 2, "one connector per driving lane");
        for road in connectors {
            for lane in map.lanes_of_section(&road.id, 0) {
                assert_eq!(lane.lane_type, LaneType::Driving);
            }
        }
    }

    #[test]
    fn a_join_that_could_carry_traffic_but_pairs_no_lanes_is_refused() {
        // The lane list is read outwards from the reference line on each side. Write
        // it left to right instead — pavement, carriageway, carriageway, pavement —
        // and the pavement takes the first rank on its side, so slot for slot the
        // two arms do not correspond and nothing pairs. That used to pass in
        // silence and fail validation later, blaming the corner pavements.
        let mut builder = MapBuilder::default();
        let inside_out = || {
            vec![
                LaneSpec::new(width(2.0), Direction::Backward).with_type(LaneType::Sidewalk),
                LaneSpec::new(width(3.5), Direction::Backward),
                LaneSpec::new(width(3.5), Direction::Forward),
                LaneSpec::new(width(2.0), Direction::Forward).with_type(LaneType::Sidewalk),
            ]
        };
        let junction = builder.add_junction(Some("x"));
        let w = builder
            .add_road(
                RoadSpec::line(
                    Point3::new(-60.0, 0.0, 0.0),
                    Point3::new(-12.0, 0.0, 0.0),
                    inside_out(),
                )
                .unwrap(),
            )
            .unwrap();
        let s = builder
            .add_road(
                RoadSpec::line(
                    Point3::new(0.0, -60.0, 0.0),
                    Point3::new(0.0, -12.0, 0.0),
                    inside_out(),
                )
                .unwrap(),
            )
            .unwrap();
        let error = builder
            .connect_ends(&w, RoadEnd::End, &s, RoadEnd::End, Some(&junction))
            .unwrap_err();
        assert!(
            matches!(&error, BuildError::NoLanePairs { from, to } if from == &w && to == &s),
            "{error:?}"
        );
        assert!(error
            .to_string()
            .contains("outwards from the reference line"));

        // Two one-way arms that both feed the junction have no movement between
        // them, and joining them is not a mistake: adjacency, and nothing else.
        let mut builder = MapBuilder::default();
        let junction = builder.add_junction(Some("y"));
        let one_way = || vec![LaneSpec::new(width(3.5), Direction::Forward)];
        let a = builder
            .add_road(
                RoadSpec::line(
                    Point3::new(-60.0, 0.0, 0.0),
                    Point3::new(-12.0, 0.0, 0.0),
                    one_way(),
                )
                .unwrap(),
            )
            .unwrap();
        let b = builder
            .add_road(
                RoadSpec::line(
                    Point3::new(0.0, -60.0, 0.0),
                    Point3::new(0.0, -12.0, 0.0),
                    one_way(),
                )
                .unwrap(),
            )
            .unwrap();
        assert!(builder
            .connect_ends(&a, RoadEnd::End, &b, RoadEnd::End, Some(&junction))
            .unwrap()
            .is_empty());
    }

    #[test]
    fn a_junction_gets_a_pavement_round_each_corner_and_none_across_it() {
        // Four arms with a pavement on each side, joined through a junction. The
        // pavements do not go through the junction with the traffic: each goes
        // round the corner to the next arm's, so a pedestrian keeps to the kerb and
        // crosses only where a crosswalk is put.
        let mut builder = MapBuilder::default();
        let street = || {
            vec![
                LaneSpec::new(width(3.5), Direction::Backward),
                LaneSpec::new(width(2.0), Direction::Backward).with_type(LaneType::Sidewalk),
                LaneSpec::new(width(3.5), Direction::Forward),
                LaneSpec::new(width(2.0), Direction::Forward).with_type(LaneType::Sidewalk),
            ]
        };
        let junction = builder.add_junction(Some("x"));
        let mut arm = |name: &str, from: (f64, f64), to: (f64, f64)| {
            builder
                .add_road(
                    RoadSpec::line(
                        Point3::new(from.0, from.1, 0.0),
                        Point3::new(to.0, to.1, 0.0),
                        street(),
                    )
                    .unwrap()
                    .with_name(name),
                )
                .unwrap()
        };
        let w = arm("w", (-60.0, 0.0), (-12.0, 0.0));
        let e = arm("e", (12.0, 0.0), (60.0, 0.0));
        let n = arm("n", (0.0, 12.0), (0.0, 60.0));
        let s = arm("s", (0.0, -60.0), (0.0, -12.0));
        for (from, to) in [(&w, &e), (&s, &n), (&w, &n), (&s, &e)] {
            builder.connect_via(&junction, from, to).unwrap();
        }
        let map = builder.finish().unwrap().validate().unwrap();

        let pavements: Vec<&Road> = map
            .roads
            .iter()
            .filter(|road| {
                road.is_connector()
                    && map
                        .lanes_of_section(&road.id, 0)
                        .iter()
                        .all(|lane| lane.lane_type == LaneType::Sidewalk)
            })
            .collect();
        assert_eq!(pavements.len(), 4, "one pavement per corner");
        for road in &pavements {
            // Each hugs its corner: its middle is well off both arms' centrelines
            // and outside the carriageway, on the diagonal.
            let middle = road
                .reference_line
                .sample_at(
                    road.horizontal_length().unwrap() / 2.0,
                    SamplingConfig::default(),
                )
                .unwrap()
                .point;
            assert!(
                middle.x.abs() > 5.0 && middle.y.abs() > 5.0,
                "{} runs through the junction at {middle:?}",
                road.id
            );
        }
        // And the movements through the junction are the traffic's alone.
        let connectors = map.roads.iter().filter(|road| road.is_connector()).count();
        assert_eq!(
            connectors - pavements.len(),
            8,
            "two carriageways × four movements"
        );
    }

    #[test]
    fn a_bend_gets_a_mitred_joint_so_the_boundaries_meet() {
        let mut builder = MapBuilder::default();
        let a = builder
            .add_road(
                RoadSpec::line(
                    Point3::new(0.0, 0.0, 10.0),
                    Point3::new(100.0, 0.0, 12.0),
                    two_way(),
                )
                .unwrap()
                .with_name("a"),
            )
            .unwrap();
        let b = builder
            .add_road(
                RoadSpec::line(
                    Point3::new(100.0, 0.0, 12.0),
                    Point3::new(200.0, 50.0, 15.0),
                    two_way(),
                )
                .unwrap()
                .with_name("b"),
            )
            .unwrap();
        builder.connect(&a, &b).unwrap();
        let map = builder.finish().unwrap().into_map();

        let a0 = map.lane(&LaneId::of_road(&a, 0)).unwrap();
        let b0 = map.lane(&LaneId::of_road(&b, 0)).unwrap();
        // The shared boundary points are the same points, not merely nearby ones.
        assert!(a0
            .left_boundary
            .end_point()
            .is_close(b0.left_boundary.start_point(), 1e-9));
        assert!(a0
            .right_boundary
            .end_point()
            .is_close(b0.right_boundary.start_point(), 1e-9));
    }

    #[test]
    fn connecting_two_carriageways_produces_a_movement_each_way() {
        let mut builder = MapBuilder::default();
        let a = builder
            .add_road(
                RoadSpec::line(Point3::ORIGIN, Point3::new(100.0, 0.0, 0.0), two_way())
                    .unwrap()
                    .with_name("a"),
            )
            .unwrap();
        let b = builder
            .add_road(
                RoadSpec::line(
                    Point3::new(100.0, 0.0, 0.0),
                    Point3::new(200.0, 0.0, 0.0),
                    two_way(),
                )
                .unwrap()
                .with_name("b"),
            )
            .unwrap();
        let movements = builder.connect(&a, &b).unwrap();
        assert_eq!(movements.len(), 2);

        let map = builder.finish().unwrap().into_map();
        assert_eq!(
            map.successors(&LaneId::of_road(&a, 0)),
            vec![LaneId::of_road(&b, 0)]
        );
        // The opposing lane runs the other way.
        assert_eq!(
            map.successors(&LaneId::of_road(&b, 1)),
            vec![LaneId::of_road(&a, 1)]
        );
    }

    #[test]
    fn a_second_continuation_without_a_junction_is_refused() {
        let mut builder = MapBuilder::default();
        let a = builder
            .add_road(
                RoadSpec::line(Point3::ORIGIN, Point3::new(100.0, 0.0, 0.0), two_way())
                    .unwrap()
                    .with_name("a"),
            )
            .unwrap();
        let b = builder
            .add_road(
                RoadSpec::line(
                    Point3::new(100.0, 0.0, 0.0),
                    Point3::new(200.0, 0.0, 0.0),
                    two_way(),
                )
                .unwrap()
                .with_name("b"),
            )
            .unwrap();
        let c = builder
            .add_road(
                RoadSpec::line(
                    Point3::new(100.0, 0.0, 0.0),
                    Point3::new(200.0, 80.0, 0.0),
                    two_way(),
                )
                .unwrap()
                .with_name("c"),
            )
            .unwrap();
        builder.connect(&a, &b).unwrap();
        assert!(matches!(
            builder.connect(&a, &c),
            Err(BuildError::ConflictingRoadLink { .. })
        ));
    }

    #[test]
    fn a_junction_split_generates_one_connector_per_movement() {
        let mut builder = MapBuilder::default();
        let approach = builder
            .add_road(
                RoadSpec::line(Point3::ORIGIN, Point3::new(100.0, 0.0, 0.0), two_way())
                    .unwrap()
                    .with_name("approach"),
            )
            .unwrap();
        let straight = builder
            .add_road(
                RoadSpec::line(
                    Point3::new(120.0, 0.0, 0.0),
                    Point3::new(220.0, 0.0, 0.0),
                    two_way(),
                )
                .unwrap()
                .with_name("straight"),
            )
            .unwrap();
        let turn = builder
            .add_road(
                RoadSpec::line(
                    Point3::new(130.0, 20.0, 0.0),
                    Point3::new(200.0, 90.0, 0.0),
                    two_way(),
                )
                .unwrap()
                .with_name("turn"),
            )
            .unwrap();
        let junction = builder.add_junction(Some("j0"));
        builder
            .connect_via(&junction, &approach, &straight)
            .unwrap();
        builder.connect_via(&junction, &approach, &turn).unwrap();

        let map = builder.finish().unwrap().validate().unwrap();

        // Two movements each way through the junction: four connectors.
        let connectors: Vec<_> = map
            .roads
            .iter()
            .filter(|road| road.is_connector())
            .collect();
        assert_eq!(connectors.len(), 4);
        assert_eq!(map.junction(&junction).unwrap().connecting_roads.len(), 4);

        // The approach lane reaches both exits, through one connector each.
        let approach_lane = LaneId::of_road(&approach, 0);
        let first_hop = map.successors(&approach_lane);
        assert_eq!(first_hop.len(), 2);
        let reached: Vec<_> = first_hop
            .iter()
            .flat_map(|lane| map.successors(lane))
            .collect();
        assert!(reached.contains(&LaneId::of_road(&straight, 0)));
        assert!(reached.contains(&LaneId::of_road(&turn, 0)));
    }

    #[test]
    fn identifiers_do_not_move_when_the_map_is_built_again() {
        let build = || {
            let mut builder = MapBuilder::default();
            let a = builder
                .add_road(
                    RoadSpec::line(Point3::ORIGIN, Point3::new(50.0, 0.0, 0.0), two_way())
                        .unwrap()
                        .with_name("a"),
                )
                .unwrap();
            let b = builder
                .add_road(
                    RoadSpec::line(
                        Point3::new(50.0, 0.0, 0.0),
                        Point3::new(100.0, 0.0, 0.0),
                        two_way(),
                    )
                    .unwrap()
                    .with_name("b"),
                )
                .unwrap();
            builder.connect(&a, &b).unwrap();
            builder.finish().unwrap().into_map()
        };
        let first = build();
        let second = build();
        let ids = |map: &Map| {
            map.connections
                .iter()
                .map(|connection| connection.id.to_string())
                .collect::<Vec<_>>()
        };
        assert_eq!(ids(&first), ids(&second));
        assert_eq!(ids(&first), ["connection/a_0/b_0", "connection/b_1/a_1"]);
    }
}
