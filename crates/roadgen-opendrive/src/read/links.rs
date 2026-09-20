//! Connectivity, from `<link>` and `<junction>`.
//!
//! OpenDRIVE says which lane continues into which in two places: on the lane, as a
//! predecessor or successor id in the neighbouring section — of the same road, or of
//! the road the `<link>` names — and, for a movement into a junction, in the
//! junction's `<connection>` from an incoming lane to a lane of the connecting road.
//! Both become the one thing the IR has, a [`LaneConnection`] from the end a lane
//! is left by to the end the next is entered at; which lane is *from* follows from
//! the lanes' directions, not from which of the two the document happened to write
//! the link on.

use std::collections::HashSet;

use opendrive::junction::contact_point::ContactPoint;
use opendrive::road::element_type::ElementType;
use opendrive::road::Road as OdRoad;

use roadgen_core::id::{ConnectionId, JunctionId, LaneId, RoadId};
use roadgen_core::topology::{
    LaneConnection, LaneEnd, LaneEndpoint, RoadEnd, RoadEndpoint, RoadLink, RoadLinkTarget,
};

use super::{Approximations, Names, Reader};
use crate::error::ImportError;

/// The road's `<link>` as a [`RoadLink`].
pub fn road_link(
    road: &OdRoad,
    names: &Names,
    approximations: &mut Approximations,
) -> Result<RoadLink, ImportError> {
    let mut link = RoadLink::default();
    let Some(element) = &road.link else {
        return Ok(link);
    };
    for (end, entry) in [
        (RoadEnd::Start, &element.predecessor),
        (RoadEnd::End, &element.successor),
    ] {
        let Some(entry) = entry else {
            continue;
        };
        let is_junction = match entry.element_type {
            Some(ElementType::Junction) => true,
            Some(ElementType::Road) => false,
            // Left unsaid: whichever of the two the id names.
            None => {
                !names.roads.contains_key(&entry.element_id)
                    && names.junctions.contains_key(&entry.element_id)
            }
        };
        let target = if is_junction {
            let Some(junction) = names.junctions.get(&entry.element_id) else {
                return Err(ImportError::Inconsistent(format!(
                    "road {} links to junction {}, which the document does not describe",
                    road.id, entry.element_id
                )));
            };
            RoadLinkTarget::Junction(junction.clone())
        } else {
            let Some(other) = names.roads.get(&entry.element_id) else {
                return Err(ImportError::Inconsistent(format!(
                    "road {} links to road {}, which the document does not describe",
                    road.id, entry.element_id
                )));
            };
            let contact = match entry.contact_point {
                Some(ContactPoint::Start) => RoadEnd::Start,
                Some(ContactPoint::End) => RoadEnd::End,
                None => {
                    approximations.count(
                        "{n} road links name no contact point and are read as meeting the \
                         neighbour's nearer end in the direction of travel",
                    );
                    end.opposite()
                }
            };
            RoadLinkTarget::Road(RoadEndpoint::new(other.clone(), contact))
        };
        link.set(end, target);
    }
    Ok(link)
}

/// Every lane connection the document states, into the map.
pub fn connect(reader: &mut Reader<'_>) -> Result<(), ImportError> {
    let mut seen: HashSet<(LaneId, LaneId)> = HashSet::new();

    let document = reader.document;

    // The lane links first: within a road from section to section, and across a
    // road link at either end.
    for road in &document.road {
        let road_id = RoadId::new(&road.id);
        let last = road.lanes.lane_section.len() - 1;
        for (section, entry) in road.lanes.lane_section.iter().enumerate() {
            for (id, lane) in super::numbered_lanes(entry) {
                let Some(link) = &lane.link else {
                    continue;
                };
                let here = reader.lane_at(&road_id, section, id, || "a lane link".to_owned())?;
                for (end, targets) in [
                    (RoadEnd::Start, &link.predecessor),
                    (RoadEnd::End, &link.successor),
                ] {
                    let Some((other_road, other_section, other_end)) =
                        reader.neighbour_at(&road_id, section, last, end)?
                    else {
                        if !targets.is_empty() {
                            reader.approximations.count(
                                "{n} lane links point past a road end that leads to a junction \
                                 or to nothing, and are dropped: a movement into a junction is \
                                 the junction's to state",
                            );
                        }
                        continue;
                    };
                    for target in targets {
                        let there =
                            reader.lane_at(&other_road, other_section, target.id, || {
                                format!("the link of lane {id} of road {}", road.id)
                            })?;
                        reader.record(
                            &mut seen,
                            (here.clone(), end.as_lane_end()),
                            (there, other_end),
                        )?;
                    }
                }
            }
        }
    }

    // Then the junctions: an incoming lane into a lane of a connecting road.
    for junction in &document.junction {
        let junction_id = JunctionId::new(&junction.id);
        for connection in junction.connection.iter() {
            let (Some(incoming), Some(connecting)) =
                (&connection.incoming_road, &connection.connecting_road)
            else {
                reader.approximations.count(
                    "{n} junction connections name no incoming or connecting road and are \
                     dropped",
                );
                continue;
            };
            let incoming = RoadId::new(incoming);
            let connecting = RoadId::new(connecting);
            let contact = match connection.contact_point {
                Some(ContactPoint::End) => RoadEnd::End,
                _ => RoadEnd::Start,
            };
            let Some(connecting_section) = reader.section_at_end(&connecting, contact) else {
                return Err(ImportError::Inconsistent(format!(
                    "junction {} connects through road {connecting}, which the document does \
                     not describe",
                    junction.id
                )));
            };
            let incoming_end =
                reader.junction_end(&incoming, &junction_id, &connecting, contact)?;
            let incoming_section = reader
                .section_at_end(&incoming, incoming_end)
                .expect("the road was found a moment ago");
            let referrer = || format!("connection {} of junction {}", connection.id, junction.id);
            for lane_link in &connection.lane_link {
                let from = reader.lane_at(&incoming, incoming_section, lane_link.from, referrer)?;
                let to = reader.lane_at(&connecting, connecting_section, lane_link.to, referrer)?;
                reader.record(
                    &mut seen,
                    (from, incoming_end.as_lane_end()),
                    (to, contact.as_lane_end()),
                )?;
            }
            // The arm is one of the junction's, whichever way its traffic runs.
            reader.add_arm(&junction_id, incoming);
        }
    }

    // Every road that links to a junction is one of its arms, listed after the
    // ones the connections named, in the document's order.
    let arms: Vec<(JunctionId, RoadId)> = reader
        .map
        .roads
        .iter()
        .flat_map(|road| {
            [RoadEnd::Start, RoadEnd::End]
                .into_iter()
                .filter_map(|end| match road.link.at(end) {
                    Some(RoadLinkTarget::Junction(junction)) => {
                        Some((junction.clone(), road.id.clone()))
                    }
                    _ => None,
                })
                .collect::<Vec<_>>()
        })
        .collect();
    for (junction, road) in arms {
        reader.add_arm(&junction, road);
    }
    Ok(())
}

impl Reader<'_> {
    /// Where a lane's neighbour at one end of its section is: the next section of
    /// the same road, or the section at the far end of the road link. A link into
    /// a junction names no lane, so a lane there has nothing to point at and the
    /// junction's connections say it.
    fn neighbour_at(
        &self,
        road: &RoadId,
        section: usize,
        last: usize,
        end: RoadEnd,
    ) -> Result<Option<(RoadId, usize, LaneEnd)>, ImportError> {
        let at_road_end = match end {
            RoadEnd::Start => section == 0,
            RoadEnd::End => section == last,
        };
        if !at_road_end {
            return Ok(Some(match end {
                RoadEnd::Start => (road.clone(), section - 1, LaneEnd::End),
                RoadEnd::End => (road.clone(), section + 1, LaneEnd::Start),
            }));
        }
        let target = self
            .map
            .roads
            .get(road)
            .and_then(|entry| entry.link.at(end).cloned());
        let Some(RoadLinkTarget::Road(other)) = target else {
            return Ok(None);
        };
        let other_section = self.section_at_end(&other.road, other.end).ok_or_else(|| {
            ImportError::Inconsistent(format!(
                "road {road} links to road {}, which was not read",
                other.road
            ))
        })?;
        Ok(Some((other.road, other_section, other.end.as_lane_end())))
    }

    /// Lists a road among a junction's arms, once.
    fn add_arm(&mut self, junction: &JunctionId, road: RoadId) {
        if let Some(entry) = self.map.junctions.get_mut(junction) {
            if !entry.incoming_roads.contains(&road) {
                entry.incoming_roads.push(road);
            }
        }
    }

    /// Records the movement between two lane ends, whichever way round the document
    /// wrote it: the lane that is *left* by its end is the source.
    ///
    /// A pair the document writes twice — once on each lane, as it should — is
    /// recorded once. A pair whose directions do not make a movement, because both
    /// lanes are entered at those ends or both left, is reported and dropped.
    fn record(
        &mut self,
        seen: &mut HashSet<(LaneId, LaneId)>,
        (a, a_end): (LaneId, LaneEnd),
        (b, b_end): (LaneId, LaneEnd),
    ) -> Result<(), ImportError> {
        let (Some(lane_a), Some(lane_b)) = (self.map.lanes.get(&a), self.map.lanes.get(&b)) else {
            return Ok(());
        };
        let a_leaves = lane_a.direction.exit_end() == a_end;
        let b_leaves = lane_b.direction.exit_end() == b_end;
        let (from, from_end, to, to_end) = match (a_leaves, b_leaves) {
            (true, false) => (a, a_end, b, b_end),
            (false, true) => (b, b_end, a, a_end),
            _ => {
                self.approximations.count(
                    "{n} lane links join two lanes that both leave, or both enter, at the \
                     ends they meet, and are dropped: a movement runs one way",
                );
                return Ok(());
            }
        };
        if !seen.insert((from.clone(), to.clone())) {
            return Ok(());
        }
        // A movement is a junction's when either lane belongs to one of its
        // connecting roads.
        let junction = [&from, &to]
            .into_iter()
            .filter_map(|lane| self.map.lanes.get(lane))
            .filter_map(|lane| self.map.roads.get(&lane.road))
            .find_map(|road| road.junction.clone());
        let id = ConnectionId::between(junction.as_ref(), &from, &to);
        self.map
            .connections
            .insert(
                id.clone(),
                LaneConnection {
                    id,
                    from: LaneEndpoint::new(from, from_end),
                    to: LaneEndpoint::new(to, to_end),
                    junction,
                },
            )
            .ok();
        Ok(())
    }

    /// Which end of an incoming road faces a junction.
    ///
    /// The end whose link names the junction; if both do — a road that loops back
    /// to the same junction — the end nearer the connecting road's contact point.
    fn junction_end(
        &mut self,
        incoming: &RoadId,
        junction: &JunctionId,
        connecting: &RoadId,
        contact: RoadEnd,
    ) -> Result<RoadEnd, ImportError> {
        let Some(road) = self.map.roads.get(incoming) else {
            return Err(ImportError::Inconsistent(format!(
                "junction {junction} is entered from road {incoming}, which the document does \
                 not describe"
            )));
        };
        let faces: Vec<RoadEnd> = [RoadEnd::Start, RoadEnd::End]
            .into_iter()
            .filter(|end| road.link.at(*end) == Some(&RoadLinkTarget::Junction(junction.clone())))
            .collect();
        match faces.as_slice() {
            [only] => Ok(*only),
            _ => {
                if faces.is_empty() {
                    self.approximations.count(
                        "{n} junction connections come from a road whose links do not name \
                         the junction; the end nearer the connecting road is taken",
                    );
                }
                let Some(connector) = self.map.roads.get(connecting) else {
                    return Ok(RoadEnd::End);
                };
                let mouth = connector.endpoint(contact);
                let nearer = [RoadEnd::Start, RoadEnd::End]
                    .into_iter()
                    .min_by(|a, b| {
                        road.endpoint(*a)
                            .distance_to(mouth)
                            .total_cmp(&road.endpoint(*b).distance_to(mouth))
                    })
                    .expect("two ends");
                Ok(nearer)
            }
        }
    }
}
