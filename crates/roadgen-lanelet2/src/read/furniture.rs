//! Rules and furniture: the regulatory elements, and crosswalks.
//!
//! What the exporter writes, read the other way. A `traffic_light` element's lights
//! and stop line become objects and a [`TrafficRule::TrafficLight`] over the lanes
//! that refer to it; `right_of_way` becomes [`TrafficRule::RightOfWay`]; a
//! `traffic_sign`'s signs and a `road_marking`'s stop lines become objects over the
//! lanes that refer to them; a crosswalk lanelet becomes a crosswalk. Each way
//! becomes one object however many elements name it, governing all their lanes.
//!
//! A traffic light keeps what Autoware draws of it besides its bottom edge: the
//! `height` of its housing, and the lamps of the `light_bulbs` way whose
//! `traffic_light_id` names it — each lamp's position, `color` and `arrow`.

use std::collections::{BTreeMap, HashMap};

use ll2_core::id::Id;
use ll2_io::osm::MemberType;

use roadgen_core::geometry::Curve3;
use roadgen_core::id::{LaneId, ObjectId};
use roadgen_core::map::Map;
use roadgen_core::semantics::{
    LightArrow, LightBulb, LightColor, LightHead, MapObject, MapObjectKind, ObjectGeometry,
    TrafficRule,
};

use super::roads::Built;
use super::source::{members, Source};
use super::Approximations;
use crate::error::ImportError;

pub(crate) fn build(
    source: &Source,
    built: &Built,
    map: &mut Map,
    approximations: &mut Approximations,
) -> Result<(), ImportError> {
    let mut objects = Objects::default();

    for lanelet in source.lanelets.values() {
        if lanelet.subtype != "crosswalk" {
            continue;
        }
        let band = Curve3::polyline(source.polyline(&lanelet.left)).and_then(|left| {
            Curve3::polyline(source.polyline(&lanelet.right)).map(|right| (left, right))
        });
        match band {
            Ok((left, right)) => {
                let id = ObjectId::new(format!("crosswalk/{}", lanelet.id));
                objects.held.insert(
                    id.clone(),
                    MapObject {
                        id,
                        kind: MapObjectKind::Crosswalk,
                        geometry: ObjectGeometry::Band { left, right },
                        lanes: Vec::new(),
                    },
                );
            }
            Err(_) => approximations.count("{n} crosswalks have no extent and are left out"),
        }
    }

    // The lanes that refer to each regulatory element.
    let mut referrers: HashMap<Id, Vec<LaneId>> = HashMap::new();
    for lanelet in source.lanelets.values() {
        let Some(lane) = built.lane_of.get(&lanelet.id) else {
            continue;
        };
        for element in &lanelet.regulatory_elements {
            referrers.entry(*element).or_default().push(lane.clone());
        }
    }
    // Lanelets read as one lane name it once.
    let lanes_of = |lanelets: Vec<Id>| -> Vec<LaneId> {
        let mut lanes: Vec<LaneId> = Vec::new();
        for lane in lanelets
            .iter()
            .filter_map(|lanelet| built.lane_of.get(&source.resolve(*lanelet)))
        {
            if !lanes.contains(lane) {
                lanes.push(lane.clone());
            }
        }
        lanes
    };

    for relation in source.document.relations.values() {
        if relation.tags.get("type").map(String::as_str) != Some("regulatory_element") {
            continue;
        }
        let lanes = referrers.get(&relation.id).cloned().unwrap_or_default();
        let ways = |role: &str| members(relation, MemberType::Way, role);
        let subtype = relation
            .tags
            .get("subtype")
            .map(String::as_str)
            .unwrap_or("");
        match subtype {
            "traffic_light" => {
                let refers = ways("refers");
                let bulbs = ways("light_bulbs");
                let lights: Vec<ObjectId> = refers
                    .iter()
                    .filter_map(|way| {
                        let head = light_head(source, *way, &refers, &bulbs, approximations);
                        objects.line(
                            source,
                            *way,
                            MapObjectKind::TrafficLight { head },
                            &lanes,
                            approximations,
                        )
                    })
                    .collect();
                let stop_line = stop_line(
                    source,
                    &mut objects,
                    &ways("ref_line"),
                    &lanes,
                    approximations,
                );
                map.rules.push(TrafficRule::TrafficLight {
                    lights,
                    stop_line,
                    lanes,
                });
            }
            "right_of_way" => {
                let right_of_way =
                    lanes_of(members(relation, MemberType::Relation, "right_of_way"));
                let yielding = lanes_of(members(relation, MemberType::Relation, "yield"));
                let stop_line = stop_line(
                    source,
                    &mut objects,
                    &ways("ref_line"),
                    &yielding,
                    approximations,
                );
                map.rules.push(TrafficRule::RightOfWay {
                    right_of_way,
                    yielding,
                    stop_line,
                });
            }
            "traffic_sign" => {
                for way in ways("refers") {
                    let code = source
                        .way_tag(way, "subtype")
                        .unwrap_or("unknown")
                        .to_owned();
                    objects.line(
                        source,
                        way,
                        MapObjectKind::TrafficSign { code },
                        &lanes,
                        approximations,
                    );
                }
                stop_line(
                    source,
                    &mut objects,
                    &ways("ref_line"),
                    &lanes,
                    approximations,
                );
            }
            "road_marking" => {
                for way in ways("refers") {
                    if source.way_tag(way, "type") == Some("stop_line") {
                        objects.line(source, way, MapObjectKind::StopLine, &lanes, approximations);
                    } else {
                        approximations.count(
                            "the IR has stop lines and crosswalks but no other road marking, so \
                             {n} road markings are not read",
                        );
                    }
                }
            }
            // The limit is on the lanelets as well, which is where Autoware reads it
            // and where the lanes already have it.
            "speed_limit" => {}
            other => approximations.count(format!(
                "the IR has no rule for a `{other}` regulatory element, so {{n}} of them are \
                 not read"
            )),
        }
    }

    for object in objects.held.into_values() {
        map.objects.insert(object.id.clone(), object).ok();
    }
    Ok(())
}

/// The stop line a rule names, as an object governing `lanes`.
fn stop_line(
    source: &Source,
    objects: &mut Objects,
    ways: &[Id],
    lanes: &[LaneId],
    approximations: &mut Approximations,
) -> Option<ObjectId> {
    if ways.len() > 1 {
        approximations.count("a rule has one stop line in the IR, so {n} rules keep their first");
    }
    let way = *ways.first()?;
    objects.line(source, way, MapObjectKind::StopLine, lanes, approximations)
}

/// The objects read so far, by id, so that a way named twice is one object.
#[derive(Default)]
struct Objects {
    held: BTreeMap<ObjectId, MapObject>,
}

impl Objects {
    /// The object a way becomes, made the first time it is asked for and given
    /// `lanes` in addition to whatever it governs already.
    fn line(
        &mut self,
        source: &Source,
        way: Id,
        kind: MapObjectKind,
        lanes: &[LaneId],
        approximations: &mut Approximations,
    ) -> Option<ObjectId> {
        let id = ObjectId::new(format!("{}/{way}", kind.as_str()));
        if !self.held.contains_key(&id) {
            let Some(curve) = source
                .way_points(way)
                .and_then(|points| Curve3::polyline(points).ok())
            else {
                approximations.count(
                    "{n} ways a regulatory element names are missing or have no extent, and are \
                     not read",
                );
                return None;
            };
            self.held.insert(
                id.clone(),
                MapObject {
                    id: id.clone(),
                    kind,
                    geometry: ObjectGeometry::Line(curve),
                    lanes: Vec::new(),
                },
            );
        }
        let object = self.held.get_mut(&id).expect("inserted above");
        for lane in lanes {
            if !object.lanes.contains(lane) {
                object.lanes.push(lane.clone());
            }
        }
        Some(id)
    }
}

/// A traffic light's housing height and lamps: the `height` of its own way, and
/// the lamps of every `light_bulbs` way whose `traffic_light_id` is that way — or,
/// where the bulbs name none of the element's lights, those of its one light or
/// those listed in the same place as it.
fn light_head(
    source: &Source,
    way: Id,
    refers: &[Id],
    bulbs: &[Id],
    approximations: &mut Approximations,
) -> LightHead {
    let height = source
        .way_tag(way, "height")
        .and_then(|value| value.trim().parse::<f64>().ok())
        .filter(|height| height.is_finite() && *height > 0.0);
    let mut head = LightHead {
        height,
        bulbs: Vec::new(),
    };
    let named = |bulbs_way: Id| {
        source
            .way_tag(bulbs_way, "traffic_light_id")
            .and_then(|value| value.trim().parse::<Id>().ok())
            .filter(|light| refers.contains(light))
    };
    // Bulbs that name none of the element's lights — a mistyped id — are paired
    // with the lights in the order both are listed, when there are as many of each.
    let by_order = bulbs.len() == refers.len();
    for (index, bulbs_way) in bulbs.iter().enumerate() {
        let ours = match named(*bulbs_way) {
            Some(light) => light == way,
            None if refers.len() == 1 => true,
            None if by_order => {
                let ours = refers[index] == way;
                if ours {
                    approximations.count(
                        "{n} times a `light_bulbs` way named no light of its element, and was \
                         paired with the light listed in the same place",
                    );
                }
                ours
            }
            None => false,
        };
        if !ours {
            continue;
        }
        let Some(nodes) = source.document.ways.get(bulbs_way).map(|way| &way.nodes) else {
            continue;
        };
        for node in nodes {
            let (Some(position), Some(tags)) = (
                source.points.get(node),
                source.document.nodes.get(node).map(|node| &node.tags),
            ) else {
                continue;
            };
            let Some(color) = tags.get("color").and_then(|value| LightColor::parse(value)) else {
                approximations.count(
                    "the IR knows red, yellow and green lamps, so {n} traffic light lamps of \
                     another colour, or none, are not read",
                );
                continue;
            };
            let arrow = match tags.get("arrow") {
                None => None,
                Some(value) => match LightArrow::parse(value) {
                    Some(arrow) => Some(arrow),
                    None => {
                        approximations.count(
                            "{n} traffic light lamps point an arrow the IR does not know, and \
                             are read as round lamps",
                        );
                        None
                    }
                },
            };
            head.bulbs.push(LightBulb {
                position: *position,
                color,
                arrow,
            });
        }
    }
    head
}
