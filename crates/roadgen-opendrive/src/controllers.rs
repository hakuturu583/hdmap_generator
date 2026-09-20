//! Which traffic lights switch together.
//!
//! OpenDRIVE has no phase plan. What it has is the `<controller>`: a list of the
//! signals that show the same state at the same time, and a `<junction>` naming the
//! controllers that belong to it. That is enough for a consumer to run a junction —
//! CARLA cycles a junction's controllers one after another, giving each its green in
//! turn — and a signal in no controller is, to such a consumer, a light with no
//! junction behind it.
//!
//! The IR says which lights switch together in exactly one place: a
//! [`TrafficRule::TrafficLight`] lists the lights that govern one set of lanes, so
//! the lights of one rule are one controller. A light no rule mentions still has to
//! go somewhere, and it goes with the other unmentioned lights on the same road at
//! the same junction — an approach is a phase, if nothing more is said.
//!
//! The junction a controller belongs to is the one its lights' lanes lead into: the
//! lane's own junction when it is a connector, otherwise whatever the road links to
//! at the end the lane exits by. A light on a lane that leads to no junction gets a
//! controller with no junction, which a consumer runs on its own.

use roadgen_core::id::{JunctionId, ObjectId, RoadId};
use roadgen_core::map::Map;
use roadgen_core::semantics::{MapObject, MapObjectKind, TrafficRule};
use roadgen_core::topology::{LaneEnd, RoadEnd, RoadLinkTarget};

/// One `<controller>`: the lights that switch together, and the junction they do
/// it at.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SignalGroup {
    /// The controller's OpenDRIVE id: its position in the list, as a string.
    pub id: String,
    /// The controller's name: the first light's own identifier, so that a
    /// controller can be traced back to the IR by eye.
    pub name: String,
    pub junction: Option<JunctionId>,
    /// The lights, in the map's own order.
    pub lights: Vec<ObjectId>,
}

/// The controllers a map's traffic lights fall into, in the order they are written.
///
/// Every traffic light is in exactly one. A light listed by more than one rule goes
/// with the first rule that names it.
pub fn signal_groups(map: &Map) -> Vec<SignalGroup> {
    let is_light = |id: &ObjectId| {
        map.objects
            .get(id)
            .is_some_and(|object| object.kind == MapObjectKind::TrafficLight)
    };
    let mut groups: Vec<(Option<JunctionId>, Vec<ObjectId>)> = Vec::new();
    let mut assigned: Vec<ObjectId> = Vec::new();

    // The rules first: they are the caller saying which lights are one phase.
    for rule in &map.rules {
        let TrafficRule::TrafficLight { lights, .. } = rule else {
            continue;
        };
        let mut members = Vec::new();
        for light in lights {
            if is_light(light) && !assigned.contains(light) && !members.contains(light) {
                members.push(light.clone());
            }
        }
        if members.is_empty() {
            continue;
        }
        let junction = map
            .objects
            .get(&members[0])
            .and_then(|object| junction_ahead(map, object));
        assigned.extend(members.iter().cloned());
        groups.push((junction, members));
    }

    // Then whatever is left, by approach: the same road into the same junction.
    type Approach = (Option<JunctionId>, Option<RoadId>);
    let mut approaches: Vec<(Approach, usize)> = Vec::new();
    for object in map.objects.iter() {
        if object.kind != MapObjectKind::TrafficLight || assigned.contains(&object.id) {
            continue;
        }
        let junction = junction_ahead(map, object);
        let road = owning_road(map, object);
        let key = (junction.clone(), road);
        let index = match approaches.iter().find(|(held, _)| *held == key) {
            Some((_, index)) => *index,
            None => {
                groups.push((junction, Vec::new()));
                approaches.push((key, groups.len() - 1));
                groups.len() - 1
            }
        };
        groups[index].1.push(object.id.clone());
        assigned.push(object.id.clone());
    }

    groups
        .into_iter()
        .enumerate()
        .map(|(index, (junction, lights))| SignalGroup {
            id: index.to_string(),
            name: lights[0].to_string(),
            junction,
            lights,
        })
        .collect()
}

/// The junction the traffic an object governs is about to enter, if any.
pub fn junction_ahead(map: &Map, object: &MapObject) -> Option<JunctionId> {
    let lane = map.lanes.get(object.lanes.first()?)?;
    let road = map.road(&lane.road)?;
    if let Some(junction) = &road.junction {
        return Some(junction.clone());
    }
    let end = match lane.direction.exit_end() {
        LaneEnd::Start => RoadEnd::Start,
        LaneEnd::End => RoadEnd::End,
    };
    match road.link.at(end)? {
        RoadLinkTarget::Junction(junction) => Some(junction.clone()),
        RoadLinkTarget::Road(_) => None,
    }
}

/// The road an object is written against: that of the first lane it governs.
pub fn owning_road(map: &Map, object: &MapObject) -> Option<RoadId> {
    let lane = map.lanes.get(object.lanes.first()?)?;
    Some(lane.road.clone())
}
