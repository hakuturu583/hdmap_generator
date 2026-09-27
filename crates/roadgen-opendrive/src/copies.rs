//! Connecting roads that more than one road runs into, or that run into more than
//! one.
//!
//! A junction connector of the IR is one road, and its lanes may be entered from
//! lanes of two roads — two approaches merging right at the junction mouth — or
//! leave into lanes of two. OpenDRIVE's `<connection>` can say the first, but a
//! road has one predecessor and one successor, and a consumer that follows the
//! connecting road's own `<link>` rather than the junction's lane links — CARLA
//! does — never enters it from the road the link does not name. A movement from
//! that road is lost.
//!
//! So the document writes such a connector once per pair of roads it joins: the
//! connector itself, linked as the IR links it, and a copy for every other road it
//! is entered from or leaves into, linked to that one. Each copy carries the lanes
//! that make a movement between its two roads, lies where the connector lies, and
//! is the connecting road of the junction's `<connection>` from its road.

use roadgen_core::id::RoadId;
use roadgen_core::map::{Lane, Map};
use roadgen_core::topology::{LaneEnd, RoadEnd, RoadLinkTarget};

use crate::splits::{road_end, RoadEndKey};

/// One connector written again, joining other roads than it is linked to.
#[derive(Debug, Clone)]
pub(crate) struct Copy {
    /// The connector it copies.
    pub road: RoadId,
    /// The road end each end of the copy links to: its start's, then its end's.
    pub ends: [Option<RoadEndKey>; 2],
}

/// Every connector copy of a map, in the map's order.
#[derive(Debug, Default)]
pub(crate) struct Copies {
    pub copies: Vec<Copy>,
}

fn end_index(end: RoadEnd) -> usize {
    match end {
        RoadEnd::Start => 0,
        RoadEnd::End => 1,
    }
}

fn road_end_of(end: LaneEnd) -> RoadEnd {
    match end {
        LaneEnd::Start => RoadEnd::Start,
        LaneEnd::End => RoadEnd::End,
    }
}

/// The ordinary road ends one lane of a connector meets at one of its ends.
pub(crate) fn met(map: &Map, lane: &Lane, end: RoadEnd) -> Vec<RoadEndKey> {
    let mut found = Vec::new();
    let ends = map
        .connections_from(&lane.id)
        .into_iter()
        .map(|connection| (connection.from.end, &connection.to))
        .chain(
            map.connections_to(&lane.id)
                .into_iter()
                .map(|connection| (connection.to.end, &connection.from)),
        );
    for (own, other) in ends {
        if road_end_of(own) != end {
            continue;
        }
        if let Some(key) = road_end(map, other) {
            if !found.contains(&key) {
                found.push(key);
            }
        }
    }
    found
}

impl Copies {
    pub fn new(map: &Map) -> Self {
        let mut copies = Vec::new();
        for road in map.roads.iter().filter(|road| road.is_connector()) {
            let lanes = map.lanes_of(&road.id);
            let linked = |end: RoadEnd| match road.link.at(end) {
                Some(RoadLinkTarget::Road(target)) => Some((target.road.clone(), target.end)),
                _ => None,
            };
            let primary = [linked(RoadEnd::Start), linked(RoadEnd::End)];
            // The road ends met at each end: the linked one first, as the connector
            // itself stands for it, then the rest in the order the lanes meet them.
            let candidates = [RoadEnd::Start, RoadEnd::End].map(|end| {
                let mut all: Vec<Option<RoadEndKey>> = vec![primary[end_index(end)].clone()];
                for lane in &lanes {
                    for key in met(map, lane, end) {
                        if !all.contains(&Some(key.clone())) {
                            all.push(Some(key));
                        }
                    }
                }
                all
            });
            for start in &candidates[0] {
                for finish in &candidates[1] {
                    let ends = [start.clone(), finish.clone()];
                    if ends == primary {
                        continue;
                    }
                    let copy = Copy {
                        road: road.id.clone(),
                        ends,
                    };
                    if lanes.iter().any(|lane| copy.carries(map, lane)) {
                        copies.push(copy);
                    }
                }
            }
        }
        Copies { copies }
    }

    /// The copies of one connector.
    pub fn of<'a>(&'a self, road: &'a RoadId) -> impl Iterator<Item = (usize, &'a Copy)> + 'a {
        self.copies
            .iter()
            .enumerate()
            .filter(move |(_, copy)| copy.road == *road)
    }
}

impl Copy {
    /// Whether a lane of the connector makes a movement between the copy's two
    /// roads: it meets the road the copy links to at each end, or meets nothing
    /// there.
    pub fn carries(&self, map: &Map, lane: &Lane) -> bool {
        [RoadEnd::Start, RoadEnd::End].into_iter().all(|end| {
            let met = met(map, lane, end);
            match &self.ends[end_index(end)] {
                Some(target) => met.contains(target),
                None => met.is_empty(),
            }
        })
    }

    /// The road end the copy links to at one end.
    pub fn at(&self, end: RoadEnd) -> Option<&RoadEndKey> {
        self.ends[end_index(end)].as_ref()
    }
}
