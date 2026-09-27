//! Splits and merges outside a junction.
//!
//! The IR lets a lane run on into lanes of several roads, or lanes of several roads
//! run into one, with no junction there: a turn pocket opening beside a through
//! lane, a slip road leaving a carriageway, two carriageways becoming one.
//! OpenDRIVE does not. A road's end links to one road or to one junction, a lane
//! link names a lane of the road linked at that end, and a lane has more than one
//! continuation only inside a junction. Written as they are, such lanes carry links
//! into roads their own road does not reach, which a consumer either drops — losing
//! the movement — or trips over.
//!
//! So wherever road ends meet in any way but one to one, the document gets a
//! junction there. Every lane connection between those ends becomes a connecting
//! road of its own: a stub whose one lane runs from the cross-section the lane
//! traffic leaves ends with, as the document draws it, to the one the lane it
//! enters starts with — [`STUB_LENGTH`] long where those are the same, half of it
//! reaching past each. Every one of those road ends links to the junction. Where
//! one of them already runs into a junction of the IR's, the stubs join that
//! junction instead, since a road end links to one.
//!
//! Which road ends meet is read from the lane connections, not the road links: two
//! road ends meet when a lane of one runs into a lane of the other, and a place is
//! every road end reachable that way. A place of two road ends linked to each
//! other, whose lanes each run into at most one lane, is what OpenDRIVE's road and
//! lane links already say, and keeps them.

use std::collections::{BTreeSet, HashMap, HashSet};

use roadgen_core::id::{ConnectionId, JunctionId, RoadId};
use roadgen_core::map::Map;
use roadgen_core::topology::{LaneConnection, LaneEnd, LaneEndpoint, RoadEnd, RoadLinkTarget};

/// How long a stub is where the lanes it joins meet, metres. OpenDRIVE and its
/// consumers want a road to have some length; half of this each side of the
/// cross-section the lanes share is within the IR's own connection tolerance.
pub(crate) const STUB_LENGTH: f64 = 1e-3;

/// One end of one road.
type RoadEndKey = (RoadId, RoadEnd);

/// The junction a place's stubs belong to.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) enum Place {
    /// A junction of the IR's, which one of the place's road ends runs into.
    Existing(JunctionId),
    /// A junction of the document's own, the n-th made.
    New(usize),
}

/// One lane connection written as a connecting road.
#[derive(Debug, Clone)]
pub(crate) struct Stub<'a> {
    pub connection: &'a LaneConnection,
    /// The road ends traffic leaves and enters by.
    pub from: RoadEndKey,
    pub to: RoadEndKey,
    /// Index into [`Splits::places`].
    pub place: usize,
}

/// Every split and merge of a map, and how the document writes it.
#[derive(Debug, Default)]
pub(crate) struct Splits<'a> {
    pub places: Vec<Place>,
    /// Road ends that link to a place's junction.
    pub ends: HashMap<RoadEndKey, usize>,
    pub stubs: Vec<Stub<'a>>,
    /// The connections the stubs carry, which the lanes no longer link.
    pub diverted: HashSet<ConnectionId>,
    /// Places whose road ends already run into more than one junction, and so can
    /// link to none: their lanes keep the links OpenDRIVE does not allow.
    pub unresolved: usize,
}

/// The road end a lane end is at, for a lane of an ordinary road at the start of
/// its first section or the end of its last.
fn road_end(map: &Map, endpoint: &LaneEndpoint) -> Option<RoadEndKey> {
    let lane = map.lanes.get(&endpoint.lane)?;
    let road = map.road(&lane.road)?;
    if road.is_connector() {
        return None;
    }
    match endpoint.end {
        LaneEnd::Start if lane.section == 0 => Some((road.id.clone(), RoadEnd::Start)),
        LaneEnd::End if lane.section + 1 == road.sections.len() => {
            Some((road.id.clone(), RoadEnd::End))
        }
        _ => None,
    }
}

impl<'a> Splits<'a> {
    pub fn new(map: &'a Map) -> Self {
        // Lane connections between two ordinary road ends, outside any junction.
        let direct: Vec<(&LaneConnection, RoadEndKey, RoadEndKey)> = map
            .connections
            .iter()
            .filter(|connection| connection.junction.is_none())
            .filter_map(|connection| {
                let from = road_end(map, &connection.from)?;
                let to = road_end(map, &connection.to)?;
                // Within one road, lane links between its sections say it.
                (from.0 != to.0).then_some((connection, from, to))
            })
            .collect();

        // Places: road ends joined by those connections.
        let mut index: HashMap<RoadEndKey, usize> = HashMap::new();
        let mut ends: Vec<RoadEndKey> = Vec::new();
        let mut parent: Vec<usize> = Vec::new();
        let mut node = |end: &RoadEndKey, parent: &mut Vec<usize>| -> usize {
            *index.entry(end.clone()).or_insert_with(|| {
                ends.push(end.clone());
                parent.push(parent.len());
                parent.len() - 1
            })
        };
        fn root(parent: &mut [usize], mut at: usize) -> usize {
            while parent[at] != at {
                parent[at] = parent[parent[at]];
                at = parent[at];
            }
            at
        }
        let mut pairs = Vec::with_capacity(direct.len());
        for (_, from, to) in &direct {
            let (a, b) = (node(from, &mut parent), node(to, &mut parent));
            let (a_root, b_root) = (root(&mut parent, a), root(&mut parent, b));
            parent[a_root] = b_root;
            pairs.push(a);
        }
        let place_of: Vec<usize> = (0..ends.len()).map(|at| root(&mut parent, at)).collect();

        let mut splits = Splits::default();
        // Each place, by its root, in the order its first connection was met.
        let mut decided: HashMap<usize, Option<usize>> = HashMap::new();
        for (at, (connection, from, to)) in direct.iter().enumerate() {
            let key = place_of[pairs[at]];
            let place = *decided.entry(key).or_insert_with(|| {
                let members: Vec<&RoadEndKey> = ends
                    .iter()
                    .enumerate()
                    .filter(|(at, _)| place_of[*at] == key)
                    .map(|(_, end)| end)
                    .collect();
                let connections: Vec<&LaneConnection> = direct
                    .iter()
                    .enumerate()
                    .filter(|(at, _)| place_of[pairs[*at]] == key)
                    .map(|(_, (connection, _, _))| *connection)
                    .collect();
                if one_to_one(map, &members, &connections) {
                    return None;
                }
                let junctions: BTreeSet<&JunctionId> = members
                    .iter()
                    .filter_map(|(road, end)| match map.road(road)?.link.at(*end) {
                        Some(RoadLinkTarget::Junction(junction)) => Some(junction),
                        _ => None,
                    })
                    .collect();
                let place = match junctions.iter().collect::<Vec<_>>()[..] {
                    [] => Place::New(
                        splits
                            .places
                            .iter()
                            .filter(|place| matches!(place, Place::New(_)))
                            .count(),
                    ),
                    [only] => Place::Existing((*only).clone()),
                    _ => {
                        splits.unresolved += 1;
                        return None;
                    }
                };
                splits.places.push(place);
                let at = splits.places.len() - 1;
                for member in members {
                    splits.ends.insert(member.clone(), at);
                }
                Some(at)
            });
            if let Some(place) = place {
                splits.diverted.insert(connection.id.clone());
                splits.stubs.push(Stub {
                    connection,
                    from: from.clone(),
                    to: to.clone(),
                    place,
                });
            }
        }
        splits
    }

    /// How many junctions the document makes of its own.
    pub fn new_junctions(&self) -> usize {
        self.places
            .iter()
            .filter(|place| matches!(place, Place::New(_)))
            .count()
    }
}

/// Whether a place is two road ends linked to each other, whose lanes each run into
/// at most one lane of the other: what OpenDRIVE's links already say.
fn one_to_one(map: &Map, ends: &[&RoadEndKey], connections: &[&LaneConnection]) -> bool {
    let [(a, a_end), (b, b_end)] = ends else {
        return false;
    };
    let linked = |road: &RoadId, end: RoadEnd, other: &RoadId, other_end: RoadEnd| {
        map.road(road).is_some_and(|road| {
            matches!(road.link.at(end), Some(RoadLinkTarget::Road(target))
                if target.road == *other && target.end == other_end)
        })
    };
    if !(linked(a, *a_end, b, *b_end) && linked(b, *b_end, a, *a_end)) {
        return false;
    }
    let mut seen = HashSet::new();
    connections.iter().all(|connection| {
        seen.insert((&connection.from.lane, connection.from.end, true))
            && seen.insert((&connection.to.lane, connection.to.end, false))
    })
}
