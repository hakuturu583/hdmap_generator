//! Generating a map: cross-sections, joints, junction connectors.
//!
//! The builder collects *topology* first — roads, their cross-sections and the
//! movements between them — and only turns that into coordinates in [`MapBuilder::finish`].
//! That order is what lets a joint be mitred (the boundaries of two roads that meet
//! have to agree on one lateral direction, which needs both roads) and what lets a
//! junction connector be drawn at all (it is shaped by the lanes it joins).

use std::collections::HashMap;

use crate::error::{BuildError, GeometryError};
use crate::geometry::{
    Bezier3, Curve3, Point3, Poly3Profile, Polyline3, Sample, SamplingConfig, Vector3,
};
use crate::id::{ConnectionId, JunctionId, LaneId, ObjectId, RoadId};
use crate::map::{Lane, Map, MapMetadata, Road, TravelGeometry};
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
    pub width: PositiveWidth,
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
    pub fn new(width: PositiveWidth, direction: Direction) -> Self {
        LaneSpec {
            width,
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

    pub fn with_markings(mut self, left: BoundaryMarking, right: BoundaryMarking) -> Self {
        self.left_marking = left;
        self.right_marking = right;
        self
    }
}

/// A road to generate.
#[derive(Debug, Clone, PartialEq)]
pub struct RoadSpec {
    pub name: Option<String>,
    pub reference_line: Curve3,
    pub lanes: Vec<LaneSpec>,
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
            lanes,
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
}

#[derive(Debug, Clone)]
enum ObjectSpec {
    /// A line across a lane at one of its ends, raised by `height` metres.
    AcrossLane {
        kind: MapObjectKind,
        lane: LaneRef,
        end: LaneEnd,
        height: f64,
        lanes: Vec<LaneRef>,
    },
    /// A band across a whole road at a fraction of its length.
    AcrossRoad {
        kind: MapObjectKind,
        road: RoadId,
        fraction: f64,
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
        if spec.lanes.is_empty() {
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
            .lanes
            .get(lane.index)
            .ok_or_else(|| BuildError::LaneIndexOutOfRange {
                road: lane.road.clone(),
                index: lane.index,
                lanes: draft.spec.lanes.len(),
            })
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
        let side = self.lane_side(lane)?;
        let mut ordinal = 0;
        for index in 0..=lane.index {
            let candidate = LaneRef::new(lane.road.clone(), index);
            if self.lane_side(&candidate)? == side {
                ordinal += 1;
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
        let from_draft_lanes = self.draft(from)?.spec.lanes.len();
        let to_draft_lanes = self.draft(to)?.spec.lanes.len();

        // A road end that points into the joint carries its reference direction
        // forwards through it; one that points away carries it backwards.
        let from_sense = if from_end == RoadEnd::End { 1.0 } else { -1.0 };
        let to_sense = if to_end == RoadEnd::Start { 1.0 } else { -1.0 };
        let sides_agree = from_sense * to_sense > 0.0;

        // Pair lanes by the side and rank they present to the joint.
        let mut to_by_slot: HashMap<(LateralSide, usize), usize> = HashMap::new();
        for index in 0..to_draft_lanes {
            let lane = LaneRef::new(to.clone(), index);
            let mut side = self.lane_side(&lane)?;
            if !sides_agree {
                side = side.opposite();
            }
            to_by_slot.insert((side, self.lane_ordinal(&lane)?), index);
        }

        // Record the adjacency first: two roads that meet are adjacent even when no
        // through movement pairs up, and both exporters need to know it.
        self.set_link(from, from_end, to, to_end, junction)?;

        let mut created = Vec::new();
        for index in 0..from_draft_lanes {
            let from_lane = LaneRef::new(from.clone(), index);
            let slot = (self.lane_side(&from_lane)?, self.lane_ordinal(&from_lane)?);
            let Some(&to_index) = to_by_slot.get(&slot) else {
                continue;
            };
            let to_lane = LaneRef::new(to.clone(), to_index);

            let from_flow = self.lane_spec(&from_lane)?.direction.sign() * from_sense;
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
        self.lane_spec(lane)?;
        self.push_object(
            format!("stopline/{}/{}", lane.lane_id().local_name(), end.as_str()),
            ObjectSpec::AcrossLane {
                kind: MapObjectKind::StopLine,
                lane: lane.clone(),
                end,
                height: 0.0,
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
        let lanes = (0..self.draft(road)?.spec.lanes.len())
            .map(|index| LaneRef::new(road.clone(), index))
            .collect();
        self.push_object(
            format!("crosswalk/{}/{fraction}", road.local_name()),
            ObjectSpec::AcrossRoad {
                kind: MapObjectKind::Crosswalk,
                road: road.clone(),
                fraction,
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
    ) -> Result<Self, GeometryError> {
        let samples = reference_line.samples(config)?;
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

    /// The curve `offset` metres to the left of the reference line.
    fn boundary(&self, offset: f64) -> Result<Curve3, GeometryError> {
        Ok(Curve3::Polyline(Polyline3::new(
            self.samples
                .iter()
                .enumerate()
                .map(|(index, sample)| sample.point + self.banked_lateral(index) * offset),
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
    map: Map,
}

impl Generator {
    fn new(builder: MapBuilder) -> Self {
        let map = Map::new(builder.metadata.clone());
        Generator {
            builder,
            geometry: HashMap::new(),
            map,
        }
    }

    fn config(&self) -> SamplingConfig {
        self.builder.metadata.sampling
    }

    fn run(mut self) -> Result<UnvalidatedMap, BuildError> {
        self.build_reference_geometry()?;
        self.mitre_joints()?;
        self.build_roads()?;
        self.build_connections()?;
        self.build_objects()?;
        self.build_rules()?;
        Ok(UnvalidatedMap::from_map(self.map))
    }

    fn build_reference_geometry(&mut self) -> Result<(), BuildError> {
        let config = self.config();
        for draft in &self.builder.roads {
            self.geometry.insert(
                draft.id.clone(),
                RoadGeometry::new(
                    &draft.spec.reference_line,
                    &draft.spec.superelevation,
                    config,
                )?,
            );
        }
        Ok(())
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
            let lanes = self.build_cross_section(&draft.id, &draft.spec, 0.0)?;
            let road = Road {
                id: draft.id.clone(),
                name: draft.spec.name.clone(),
                reference_line: draft.spec.reference_line.clone(),
                lane_offset: 0.0,
                lanes: lanes.iter().map(|lane| lane.id.clone()).collect(),
                junction: None,
                link: draft.link.clone(),
                road_type: draft.spec.road_type,
                speed_limit: draft.spec.speed_limit,
                superelevation: draft.spec.superelevation.clone(),
            };
            self.insert_road(road, lanes)?;
        }
        Ok(())
    }

    /// Lays the cross-section out against the road's generated geometry.
    fn build_cross_section(
        &self,
        road: &RoadId,
        spec: &RoadSpec,
        lane_offset: f64,
    ) -> Result<Vec<Lane>, BuildError> {
        let geometry = &self.geometry[road];
        let mut left_edge = lane_offset;
        let mut right_edge = lane_offset;
        let mut left_count = 0usize;
        let mut right_count = 0usize;
        let mut lanes = Vec::with_capacity(spec.lanes.len());

        for (index, lane_spec) in spec.lanes.iter().enumerate() {
            let side = lane_spec.side.unwrap_or_else(|| {
                self.builder
                    .metadata
                    .handedness
                    .side_for(lane_spec.direction)
            });
            let width = lane_spec.width.metres();
            let (left_offset, right_offset, ordinal) = match side {
                LateralSide::Left => {
                    let inner = left_edge;
                    left_edge += width;
                    left_count += 1;
                    (left_edge, inner, left_count)
                }
                LateralSide::Right => {
                    let inner = right_edge;
                    right_edge -= width;
                    right_count += 1;
                    (inner, right_edge, right_count)
                }
            };
            let center_offset = (left_offset + right_offset) / 2.0;
            lanes.push(Lane {
                id: LaneId::of_road(road, index),
                road: road.clone(),
                index,
                side,
                ordinal,
                direction: lane_spec.direction,
                lane_type: lane_spec.lane_type,
                width: lane_spec.width,
                speed_limit: lane_spec.speed_limit.or(spec.speed_limit),
                left_offset,
                right_offset,
                left_boundary: geometry.boundary(left_offset)?,
                right_boundary: geometry.boundary(right_offset)?,
                centerline: geometry.boundary(center_offset)?,
                left_marking: lane_spec.left_marking,
                right_marking: lane_spec.right_marking,
            });
        }
        Ok(lanes)
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
            }
        }
        Ok(())
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
        let config = self.config();
        let from_lane = self.lane(from)?.clone();
        let to_lane = self.lane(to)?.clone();
        let from_travel: TravelGeometry = from_lane.travel_geometry(config)?;
        let to_travel: TravelGeometry = to_lane.travel_geometry(config)?;

        let start = from_travel.centerline.end_point();
        let start_tangent = from_travel.centerline.end_tangent()?;
        let end = to_travel.centerline.start_point();
        let end_tangent = to_travel.centerline.start_tangent()?;
        let reference_line =
            Curve3::Bezier(Bezier3::hermite(start, start_tangent, end, end_tangent)?);

        let width = from_lane.width;
        let road_id = RoadId::new(format!(
            "{}/{}_{}",
            junction.local_name(),
            from_lane.id.local_name(),
            to_lane.id.local_name()
        ));

        // The connector's lane straddles its reference line, so the cross-section
        // origin sits half a lane to the left of it.
        let mut geometry = RoadGeometry::new(&reference_line, &Poly3Profile::default(), config)?;
        // Adopt the lateral direction of each road it meets, so the connector's
        // boundary endpoints land exactly on theirs.
        let from_end = road_end_of(from_lane.direction.exit_end());
        let to_end = road_end_of(to_lane.direction.entry_end());
        // The banked direction, not the plan one: the connector's boundary endpoints
        // have to land on the approach's, which a banked road lifts off the
        // horizontal.
        let start_lateral =
            self.geometry[&from_lane.road].banked_lateral_at(from_end) * from_lane.direction.sign();
        let end_lateral =
            self.geometry[&to_lane.road].banked_lateral_at(to_end) * to_lane.direction.sign();
        let last = geometry.laterals.len() - 1;
        geometry.laterals[0] = start_lateral;
        geometry.laterals[last] = end_lateral;
        self.geometry.insert(road_id.clone(), geometry);

        let spec = RoadSpec {
            name: None,
            reference_line: reference_line.clone(),
            lanes: vec![LaneSpec {
                width,
                direction: Direction::Forward,
                lane_type: from_lane.lane_type,
                speed_limit: from_lane.speed_limit,
                side: Some(LateralSide::Right),
                left_marking: BoundaryMarking::new(RoadMarking::None, from_lane.left_marking.color),
                right_marking: BoundaryMarking::new(
                    RoadMarking::None,
                    from_lane.right_marking.color,
                ),
            }],
            road_type: self
                .map
                .road(&from_lane.road)
                .map(|road| road.road_type)
                .unwrap_or(RoadType::Town),
            speed_limit: from_lane.speed_limit,
            superelevation: Poly3Profile::default(),
        };
        let lanes = self.build_cross_section(&road_id, &spec, width.half())?;
        let connector_lane = lanes[0].id.clone();
        let road = Road {
            id: road_id.clone(),
            name: None,
            reference_line,
            lane_offset: width.half(),
            lanes: lanes.iter().map(|lane| lane.id.clone()).collect(),
            junction: Some(junction.clone()),
            link: RoadLink {
                predecessor: Some(RoadLinkTarget::Road(RoadEndpoint::new(
                    from_lane.road.clone(),
                    from_end,
                ))),
                successor: Some(RoadLinkTarget::Road(RoadEndpoint::new(
                    to_lane.road.clone(),
                    to_end,
                ))),
            },
            road_type: spec.road_type,
            speed_limit: spec.speed_limit,
            superelevation: Poly3Profile::default(),
        };
        self.insert_road(road, lanes)?;

        if let Some(entry) = self.map.junctions.get_mut(junction) {
            entry.connecting_roads.push(road_id.clone());
        } else {
            return Err(BuildError::UnknownJunction(junction.clone()));
        }

        self.push_connection(
            Some(junction),
            LaneEndpoint::new(from_lane.id.clone(), from_lane.direction.exit_end()),
            LaneEndpoint::new(connector_lane.clone(), LaneEnd::Start),
        )?;
        self.push_connection(
            Some(junction),
            LaneEndpoint::new(connector_lane, LaneEnd::End),
            LaneEndpoint::new(to_lane.id.clone(), to_lane.direction.entry_end()),
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
                    lanes,
                } => {
                    let lane = self.lane(&lane)?.clone();
                    let travel = lane.travel_geometry(config)?;
                    let (left, right) = match end == lane.direction.exit_end() {
                        true => (travel.left.end_point(), travel.right.end_point()),
                        false => (travel.left.start_point(), travel.right.start_point()),
                    };
                    let raise = |point: Point3| Point3::new(point.x, point.y, point.z + height);
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
                    fraction,
                    width,
                    lanes,
                } => {
                    let geometry = &self.geometry[&road];
                    let count = geometry.samples.len();
                    let index = (((count - 1) as f64) * fraction.clamp(0.0, 1.0)).round() as usize;
                    let sample = geometry.samples[index];
                    let lateral = geometry.laterals[index];
                    let extent = self.map.lanes_of(&road).iter().fold(0.0_f64, |acc, lane| {
                        acc.max(lane.left_offset.abs()).max(lane.right_offset.abs())
                    });
                    let along = sample.tangent.scaled(width / 2.0);
                    let edge = |sign: f64| -> Result<Curve3, GeometryError> {
                        let center = sample.point + along * sign;
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
