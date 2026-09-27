//! The file, indexed the way the reconstruction asks questions of it.

use std::collections::{BTreeMap, HashMap};

use ll2_core::geometry::distance::{middle_point, signed_distance_2d};
use ll2_core::id::Id;
use ll2_io::osm::{Document, MemberType, Relation};
use ll2_projection::LocalCartesian;

use roadgen_core::geometry::Point3;

use super::{project, Approximations};
use crate::error::ImportError;

/// One lanelet, with its boundaries pointing the way it runs.
#[derive(Debug, Clone)]
pub(crate) struct Lanelet {
    pub id: Id,
    pub subtype: String,
    pub location: Option<String>,
    /// Autoware's mark for a lanelet that crosses an intersection.
    pub turn: bool,
    pub one_way: bool,
    pub speed_limit_kph: Option<f64>,
    pub left_way: Id,
    pub right_way: Id,
    /// Node ids, in the direction the lanelet runs.
    pub left: Vec<Id>,
    pub right: Vec<Id>,
    /// Regulatory elements the lanelet refers to.
    pub regulatory_elements: Vec<Id>,
}

impl Lanelet {
    /// The two points a lanelet starts at, which the one before it ends at.
    pub fn start_key(&self) -> (Id, Id) {
        (self.left[0], self.right[0])
    }

    pub fn end_key(&self) -> (Id, Id) {
        (
            *self.left.last().expect("a boundary has points"),
            *self.right.last().expect("a boundary has points"),
        )
    }
}

pub(crate) struct Source<'a> {
    pub document: &'a Document,
    pub points: HashMap<Id, Point3>,
    pub lanelets: BTreeMap<Id, Lanelet>,
    /// Lanelets that start where each lanelet ends.
    pub successors: HashMap<Id, Vec<Id>>,
    pub predecessors: HashMap<Id, Vec<Id>>,
}

impl<'a> Source<'a> {
    pub fn new(
        document: &'a Document,
        projector: &LocalCartesian,
        approximations: &mut Approximations,
    ) -> Result<Self, ImportError> {
        let mut points = HashMap::with_capacity(document.nodes.len());
        for node in document.nodes.values() {
            match project(projector, node) {
                Some(point) => {
                    points.insert(node.id, point);
                }
                None => approximations.count("{n} nodes could not be projected and are left out"),
            }
        }

        let mut lanelets = BTreeMap::new();
        for relation in document.relations.values() {
            if relation.tags.get("type").map(String::as_str) != Some("lanelet") {
                continue;
            }
            match lanelet(document, &points, relation) {
                Ok(lanelet) => {
                    lanelets.insert(lanelet.id, lanelet);
                }
                Err(reason) => {
                    approximations.note(format!("lanelet {} is left out: {reason}", relation.id))
                }
            }
        }

        let mut starting: HashMap<(Id, Id), Vec<Id>> = HashMap::new();
        for lanelet in lanelets.values() {
            starting
                .entry(lanelet.start_key())
                .or_default()
                .push(lanelet.id);
        }
        let mut successors: HashMap<Id, Vec<Id>> = HashMap::new();
        let mut predecessors: HashMap<Id, Vec<Id>> = HashMap::new();
        for lanelet in lanelets.values() {
            for next in starting.get(&lanelet.end_key()).into_iter().flatten() {
                if *next == lanelet.id {
                    continue;
                }
                successors.entry(lanelet.id).or_default().push(*next);
                predecessors.entry(*next).or_default().push(lanelet.id);
            }
        }

        Ok(Source {
            document,
            points,
            lanelets,
            successors,
            predecessors,
        })
    }

    pub fn successors_of(&self, lanelet: Id) -> &[Id] {
        self.successors
            .get(&lanelet)
            .map(Vec::as_slice)
            .unwrap_or(&[])
    }

    pub fn predecessors_of(&self, lanelet: Id) -> &[Id] {
        self.predecessors
            .get(&lanelet)
            .map(Vec::as_slice)
            .unwrap_or(&[])
    }

    pub fn point(&self, node: Id) -> Point3 {
        self.points[&node]
    }

    pub fn polyline(&self, nodes: &[Id]) -> Vec<Point3> {
        nodes.iter().map(|node| self.point(*node)).collect()
    }

    /// A way's points, in the way's own order, if every node is there.
    pub fn way_points(&self, way: Id) -> Option<Vec<Point3>> {
        let way = self.document.ways.get(&way)?;
        way.nodes
            .iter()
            .map(|node| self.points.get(node).copied())
            .collect()
    }

    pub fn way_tag(&self, way: Id, key: &str) -> Option<&str> {
        self.document
            .ways
            .get(&way)
            .and_then(|way| way.tags.get(key))
            .map(String::as_str)
    }
}

/// Members of a relation with a given role and kind.
pub(crate) fn members(relation: &Relation, kind: MemberType, role: &str) -> Vec<Id> {
    relation
        .members
        .iter()
        .filter(|member| member.kind == kind && member.role == role)
        .map(|member| member.reference)
        .collect()
}

/// Reads one lanelet relation.
///
/// Lanelet2 allows a boundary way to point either way; see [`align`] for how the
/// way the lanelet runs is decided.
fn lanelet(
    document: &Document,
    points: &HashMap<Id, Point3>,
    relation: &Relation,
) -> Result<Lanelet, String> {
    let bound = |role: &str| -> Result<(Id, Vec<Id>), String> {
        let ways = members(relation, MemberType::Way, role);
        let [way] = ways.as_slice() else {
            return Err(format!(
                "it has {} `{role}` boundaries, not one",
                ways.len()
            ));
        };
        let nodes = &document
            .ways
            .get(way)
            .ok_or_else(|| format!("its {role} boundary {way} is not in the file"))?
            .nodes;
        if nodes.len() < 2 {
            return Err(format!(
                "its {role} boundary {way} has fewer than two nodes"
            ));
        }
        if let Some(missing) = nodes.iter().find(|node| !points.contains_key(node)) {
            return Err(format!(
                "its {role} boundary names node {missing}, which is not in the file"
            ));
        }
        Ok((*way, nodes.clone()))
    };
    let (left_way, mut left) = bound("left")?;
    let (right_way, mut right) = bound("right")?;

    align(&mut left, &mut right, points);

    let tag = |key: &str| relation.tags.get(key).map(String::as_str);
    let one_way = tag("one_way") != Some("no");
    Ok(Lanelet {
        id: relation.id,
        subtype: tag("subtype").unwrap_or("road").to_owned(),
        location: tag("location").map(str::to_owned),
        turn: tag("turn_direction").is_some(),
        one_way,
        speed_limit_kph: tag("speed_limit").and_then(parse_speed),
        left_way,
        right_way,
        left,
        right,
        regulatory_elements: members(relation, MemberType::Relation, "regulatory_element"),
    })
}

/// Points each boundary the way the lanelet runs, the way Lanelet2 does on load.
///
/// The roles are what a lanelet is sure of — which boundary is its left — and a way
/// in the file may point either way. So the left boundary is turned round when the
/// right one's middle is not to its right, and the right boundary when the left
/// one's middle is not to its left: which way the lanelet runs follows from which
/// side each boundary is on.
///
/// Upstream: `geometry::align`, `impl/LineString.h`, as `ll2_io` ports it.
fn align(left: &mut [Id], right: &mut [Id], points: &HashMap<Id, Point3>) {
    let flat = |nodes: &[Id]| -> Vec<[f64; 2]> {
        nodes
            .iter()
            .map(|node| [points[node].x, points[node].y])
            .collect()
    };
    if let Some(middle) = middle_point(&flat(right)) {
        if signed_distance_2d(&flat(left), middle) >= 0.0 {
            left.reverse();
        }
    }
    if let Some(middle) = middle_point(&flat(left)) {
        if signed_distance_2d(&flat(right), middle) <= 0.0 {
            right.reverse();
        }
    }
}

/// A Lanelet2 speed, in km/h: a bare number is km/h, and `mph` and `m/s` suffixes
/// are what the specification also allows.
fn parse_speed(value: &str) -> Option<f64> {
    let value = value.trim();
    let (number, factor) = if let Some(number) = value.strip_suffix("mph") {
        (number, 1.609_344)
    } else if let Some(number) = value.strip_suffix("m/s") {
        (number, 3.6)
    } else if let Some(number) = value.strip_suffix("km/h") {
        (number, 1.0)
    } else if let Some(number) = value.strip_suffix("kmh") {
        (number, 1.0)
    } else {
        (value, 1.0)
    };
    number
        .trim()
        .parse::<f64>()
        .ok()
        .filter(|speed| speed.is_finite() && *speed > 0.0)
        .map(|speed| speed * factor)
}
