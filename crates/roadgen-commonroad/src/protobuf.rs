//! The map alone, as a CommonRoad 2024 `CommonRoadMap` protocol buffer.
//!
//! The 2020a XML is a scenario: the map, the traffic on it and the planning
//! problems posed on it, in one document, and the schema will not have one
//! without a planning problem. CommonRoad's 2024 format splits that into three
//! files — the map, the dynamic part (traffic, light timing) and the scenario that
//! ties a map and a dynamic file to planning problems — and this module writes the
//! first: a map and nothing more, which `commonroad-io` reads with
//! `CommonRoadFileReader(filename_map=...).open_map()`.
//!
//! The schema is CommonRoad's own `.proto` files, carried in `proto/` (BSD
//! 3-Clause) and compiled at build time. The map is built from the same lanelet
//! network the XML scenario is ([`crate::network`]), so the two agree lanelet for
//! lanelet, and differs from it in four places, as the format does:
//!
//! - **Bounds and stop lines are elements of their own**, with ids, that lanelets
//!   refer to. Each lanelet gets its own two bounds in its own direction of
//!   travel: the format can share one bound between neighbours with a "reversed"
//!   flag, but `commonroad-io`'s reader ignores the flag and would read the bound
//!   of a lanelet running against it backwards. A stop line has its two ends;
//!   one across no line the map draws is drawn across the lanelet's end.
//! - **Every sign stands somewhere.** A virtual sign — a speed limit, a give-way —
//!   stands beside the first lanelet that refers to it, at its right bound: at
//!   the start for a limit, which holds from there on, else at the end.
//! - **Signs are named, not numbered**: `MAX_SPEED`, not `274`.
//! - **No light timing**: a light's cycle belongs to the dynamic file.
//!
//! And it holds something the scenario does not: every building part, as an
//! environment obstacle of type `BUILDING` with its outline in plan.

use std::path::{Path, PathBuf};

use prost::Message;
use roadgen_core::geometry::Point3;
use roadgen_core::{IrRef, Relation, Trace, ValidatedMap};

use crate::error::ExportError;
use crate::model::{self, Scenario};
use crate::{signs, ScenarioConfig, COUNTRY};

/// The generated message types: CommonRoad's `commonroad_map` and
/// `commonroad_common` packages.
pub mod pb {
    pub mod commonroad_common {
        include!(concat!(env!("OUT_DIR"), "/commonroad_common.rs"));
    }
    pub mod commonroad_map {
        include!(concat!(env!("OUT_DIR"), "/commonroad_map.rs"));
    }
}

use pb::commonroad_common as common;
use pb::commonroad_map as map_pb;

pub use map_pb::CommonRoadMap;

/// The format name a CommonRoad map's [`Trace`] is recorded under.
pub const TRACE_FORMAT: &str = "commonroad_map";

/// Builds the CommonRoad map of `map`. The configuration's benchmark id names
/// the map, and its date, author, affiliation, source and elevation are the
/// map's; a planning problem and a light cycle have no place in it.
pub fn to_map(map: &ValidatedMap, config: &ScenarioConfig) -> Result<CommonRoadMap, ExportError> {
    to_map_traced(map, config).map(|(written, _)| written)
}

/// [`to_map`], with the [`Trace`] of which IR elements each written element came
/// from: the scenario's names (`lanelet:<id>`, `traffic_sign:<id>`, …), and
/// besides them each lane's two bounds as `boundary:<id>` and each building part
/// as `environment_obstacle:<id>`.
pub fn to_map_traced(
    map: &ValidatedMap,
    config: &ScenarioConfig,
) -> Result<(CommonRoadMap, Trace), ExportError> {
    let config = ScenarioConfig {
        planning_problem: false,
        light_cycle: None,
        ..config.clone()
    };
    let (scenario, mut trace) = crate::to_scenario_traced(map, &config)?;
    trace.format = TRACE_FORMAT.to_owned();
    let written = Lowering::new(&scenario, config.elevation).run(map, &mut trace)?;
    Ok((written, trace))
}

/// The map as one serialized `CommonRoadMap` message.
pub fn to_map_bytes(map: &ValidatedMap, config: &ScenarioConfig) -> Result<Vec<u8>, ExportError> {
    to_map_bytes_traced(map, config).map(|(bytes, _)| bytes)
}

/// [`to_map_bytes`], with the [`Trace`] of the map it encodes.
pub fn to_map_bytes_traced(
    map: &ValidatedMap,
    config: &ScenarioConfig,
) -> Result<(Vec<u8>, Trace), ExportError> {
    let (written, trace) = to_map_traced(map, config)?;
    Ok((written.encode_to_vec(), trace))
}

/// Writes `map` to `path` as a CommonRoad 2024 map file: the bare message, the way
/// `commonroad-io` writes and reads one.
pub fn write_map(
    map: &ValidatedMap,
    path: impl AsRef<Path>,
    config: &ScenarioConfig,
) -> Result<(), ExportError> {
    write_map_traced(map, path, config).map(|_| ())
}

/// [`write_map`], with the [`Trace`] of the map it wrote. The trace's one file is
/// `path`.
pub fn write_map_traced(
    map: &ValidatedMap,
    path: impl AsRef<Path>,
    config: &ScenarioConfig,
) -> Result<Trace, ExportError> {
    let path = path.as_ref();
    let (bytes, mut trace) = to_map_bytes_traced(map, config)?;
    std::fs::write(path, bytes)
        .map_err(|error| ExportError::Io(format!("{}: {error}", path.display())))?;
    trace.files = vec![PathBuf::from(path)];
    Ok(trace)
}

/// A CommonRoad map file, the way a consumer reads one.
pub fn read_map(path: impl AsRef<Path>) -> Result<CommonRoadMap, ExportError> {
    let path = path.as_ref();
    let bytes = std::fs::read(path)
        .map_err(|error| ExportError::Io(format!("{}: {error}", path.display())))?;
    CommonRoadMap::decode(bytes.as_slice()).map_err(|error| {
        ExportError::Io(format!("{}: not a CommonRoad map: {error}", path.display()))
    })
}

/// The scenario's network, re-cut into the map's elements.
struct Lowering<'s> {
    scenario: &'s Scenario,
    elevation: bool,
    /// Ids for what the scenario has none for — bounds, stop lines, crossing
    /// groups, environment obstacles — from above every id it used.
    next_id: u64,
}

impl<'s> Lowering<'s> {
    fn new(scenario: &'s Scenario, elevation: bool) -> Self {
        let last = scenario
            .lanelets
            .iter()
            .map(|lanelet| lanelet.id)
            .chain(scenario.traffic_signs.iter().map(|sign| sign.id))
            .chain(scenario.traffic_lights.iter().map(|light| light.id))
            .chain(scenario.intersections.iter().flat_map(|intersection| {
                std::iter::once(intersection.id)
                    .chain(intersection.incomings.iter().map(|incoming| incoming.id))
            }))
            .max()
            .unwrap_or(0);
        Lowering {
            scenario,
            elevation,
            next_id: last + 1,
        }
    }

    fn take_id(&mut self) -> u64 {
        let id = self.next_id;
        self.next_id += 1;
        id
    }

    fn point(&self, point: Point3) -> common::Point {
        common::Point {
            x: point.x,
            y: point.y,
            z: self.elevation.then_some(point.z),
        }
    }

    fn run(mut self, map: &ValidatedMap, trace: &mut Trace) -> Result<CommonRoadMap, ExportError> {
        let scenario = self.scenario;
        let mut boundaries = Vec::new();
        let mut stop_lines = Vec::new();
        let mut lanelets = Vec::new();
        for lanelet in &scenario.lanelets {
            let (left, right) = (self.take_id(), self.take_id());
            for (id, bound, role) in [
                (left, &lanelet.left, "left_boundary"),
                (right, &lanelet.right, "right_boundary"),
            ] {
                boundaries.push(map_pb::Bound {
                    boundary_id: id as i32,
                    points: bound
                        .points
                        .iter()
                        .map(|point| self.point(*point))
                        .collect(),
                    // Always written: the reader takes a bound with none for
                    // dashed, the enumeration's first value.
                    line_marking: Some(line_marking(bound.line_marking)),
                });
                for link in written_from(trace, &format!("lanelet:{}", lanelet.id)) {
                    trace.link_as(link, format!("boundary:{id}"), Relation::Exact, role);
                }
            }
            let stop_line = match &lanelet.stop_line {
                None => None,
                Some(stop) => {
                    let [start, end] = stop.points.unwrap_or_else(|| {
                        [
                            *lanelet.left.points.last().expect("a bound has points"),
                            *lanelet.right.points.last().expect("a bound has points"),
                        ]
                    });
                    let id = self.take_id() as u32;
                    stop_lines.push(map_pb::StopLine {
                        stop_line_id: id,
                        start_point: self.point(start),
                        end_point: self.point(end),
                        line_marking: map_pb::line_marking_enum::LineMarking::Solid as i32,
                    });
                    Some(id)
                }
            };
            lanelets.push(map_pb::Lanelet {
                lanelet_id: lanelet.id as u32,
                left_bound: left as i32,
                right_bound: right as i32,
                left_bound_reverse: None,
                right_bound_reverse: None,
                predecessors: ids(&lanelet.predecessors),
                successors: ids(&lanelet.successors),
                adjacent_left: lanelet.adjacent_left.map(|a| a.lanelet as u32),
                adjacent_right: lanelet.adjacent_right.map(|a| a.lanelet as u32),
                adjacent_left_opposite_dir: lanelet.adjacent_left.map(|a| !a.same_direction),
                adjacent_right_opposite_dir: lanelet.adjacent_right.map(|a| !a.same_direction),
                stop_line,
                lanelet_types: lanelet
                    .lanelet_types
                    .iter()
                    .map(|kind| {
                        enumerated(kind, map_pb::lanelet_type_enum::LaneletType::from_str_name)
                    })
                    .collect::<Result<_, _>>()?,
                user_one_way: users(&lanelet.user_one_way)?,
                user_bidirectional: users(&lanelet.user_bidirectional)?,
                traffic_sign_refs: ids(&lanelet.traffic_signs),
                traffic_light_refs: ids(&lanelet.traffic_lights),
                adjacent_areas: Vec::new(),
            });
        }

        let traffic_signs = scenario
            .traffic_signs
            .iter()
            .map(|sign| self.traffic_sign(sign))
            .collect::<Result<_, _>>()?;
        let traffic_lights = scenario
            .traffic_lights
            .iter()
            .map(|light| map_pb::TrafficLight {
                traffic_light_id: light.id as u32,
                position: self.point(light.position),
                color: Vec::new(),
                direction: Some(
                    map_pb::traffic_light_direction_enum::TrafficLightDirection::All as i32,
                ),
            })
            .collect();
        let mut intersections = Vec::new();
        for intersection in &scenario.intersections {
            let crossings = if intersection.crossing.is_empty() {
                Vec::new()
            } else {
                vec![map_pb::CrossingGroup {
                    crossing_group_id: self.take_id() as u32,
                    crossing_lanelets: ids(&intersection.crossing),
                    incoming_group_id: None,
                    outgoing_group_id: None,
                }]
            };
            intersections.push(map_pb::Intersection {
                intersection_id: intersection.id as u32,
                incomings: intersection
                    .incomings
                    .iter()
                    .map(|incoming| map_pb::IncomingGroup {
                        incoming_group_id: incoming.id as u32,
                        incoming_lanelets: ids(&incoming.incoming_lanelets),
                        outgoing_group_id: None,
                        outgoing_right: ids(&incoming.successors_right),
                        outgoing_straight: ids(&incoming.successors_straight),
                        outgoing_left: ids(&incoming.successors_left),
                    })
                    .collect(),
                outgoings: Vec::new(),
                crossings,
            });
        }

        let mut environment_obstacles = Vec::new();
        for part in map.building_parts.iter() {
            let id = self.take_id();
            environment_obstacles.push(map_pb::EnvironmentObstacle {
                environment_obstacle_id: id as u32,
                obstacle_type:
                    map_pb::environment_obstacle_type_enum::EnvironmentObstacleType::Building
                        as i32,
                obstacle_shape: common::Occupancy {
                    shape: Some(common::occupancy::Shape::Polygon(common::PolygonOccupancy {
                        vertices: part
                            .solid
                            .footprint
                            .points()
                            .iter()
                            .map(|point| self.point(*point))
                            .collect(),
                    })),
                },
            });
            trace.link_as(
                IrRef::BuildingPart(part.id.clone()),
                format!("environment_obstacle:{id}"),
                Relation::Exact,
                "building",
            );
        }

        Ok(CommonRoadMap {
            map_meta_information: map_pb::MapInformation {
                map_id: map_id(&scenario.benchmark_id),
                file_information: common::FileInformation {
                    license_name: String::new(),
                    license_text: None,
                    date: time_stamp(&scenario.date),
                    author: scenario.author.clone(),
                    affiliation: scenario.affiliation.clone(),
                    source: scenario.source.clone(),
                },
            },
            location: map_pb::Location {
                geo_name_id: scenario.location.geo_name_id as i32,
                gps_latitude: scenario.location.latitude,
                gps_longitude: scenario.location.longitude,
                geo_transformation: Some(map_pb::GeoTransformation {
                    geo_reference: Some(scenario.location.geo_reference.clone()),
                    x_translation: Some(0.0),
                    y_translation: Some(0.0),
                    z_rotation: Some(0.0),
                    scaling: Some(1.0),
                }),
            },
            lanelets,
            stop_lines,
            boundaries,
            areas: Vec::new(),
            traffic_signs,
            traffic_lights,
            intersections,
            environment_obstacles,
        })
    }

    /// A sign, standing where the scenario put it or, virtual, beside the first
    /// lanelet that refers to it.
    fn traffic_sign(&self, sign: &model::TrafficSign) -> Result<map_pb::TrafficSign, ExportError> {
        let position = match sign.position {
            Some(position) => position,
            None => {
                let lanelet = self
                    .scenario
                    .lanelets
                    .iter()
                    .find(|lanelet| lanelet.traffic_signs.contains(&sign.id))
                    .expect("a sign no lanelet refers to is not written");
                let limit = sign
                    .elements
                    .iter()
                    .any(|element| element.sign_id == signs::MAX_SPEED);
                let points = &lanelet.right.points;
                if limit {
                    points[0]
                } else {
                    *points.last().expect("a bound has points")
                }
            }
        };
        Ok(map_pb::TrafficSign {
            traffic_sign_id: sign.id as u32,
            traffic_sign_elements: sign
                .elements
                .iter()
                .map(|element| {
                    let name = signs::name(element.sign_id).ok_or_else(|| {
                        ExportError::Config(format!("sign {} has no name", element.sign_id))
                    })?;
                    Ok(common::TrafficSignElement {
                        element_id: enumerated(
                            name,
                            common::traffic_sign_id_enum::TrafficSignId::from_str_name,
                        )?,
                        additional_values: element.additional_values.clone(),
                    })
                })
                .collect::<Result<_, ExportError>>()?,
            position: self.point(position),
            r#virtual: Some(sign.virtual_sign),
        })
    }
}

/// The IR elements a written element came from, by its local name.
fn written_from(trace: &Trace, local: &str) -> Vec<IrRef> {
    trace.links_to(local).map(|link| link.ir.clone()).collect()
}

fn ids(ids: &[u64]) -> Vec<u32> {
    ids.iter().map(|&id| id as u32).collect()
}

/// An enumeration's value by the schema's name for it, from the XML's: `bicycleLane`
/// is `BICYCLE_LANE`, `solid_dashed` is `SOLID_DASHED`.
fn enumerated<E: Into<i32>>(
    name: &str,
    from_str_name: fn(&str) -> Option<E>,
) -> Result<i32, ExportError> {
    let mut screaming = String::new();
    // A name already in the schema's spelling — a sign's `MAX_SPEED` — stays as it is.
    let camel = name.chars().any(|c| c.is_ascii_lowercase());
    for c in name.chars() {
        if camel && c.is_ascii_uppercase() && !screaming.is_empty() && !screaming.ends_with('_') {
            screaming.push('_');
        }
        screaming.push(c.to_ascii_uppercase());
    }
    from_str_name(&screaming)
        .map(Into::into)
        .ok_or_else(|| ExportError::Config(format!("the map schema has no {screaming}")))
}

fn line_marking(name: &str) -> i32 {
    enumerated(name, map_pb::line_marking_enum::LineMarking::from_str_name)
        .unwrap_or(map_pb::line_marking_enum::LineMarking::Unknown as i32)
}

fn users(names: &[&str]) -> Result<Vec<i32>, ExportError> {
    names
        .iter()
        .map(|name| enumerated(name, map_pb::road_user_enum::RoadUser::from_str_name))
        .collect()
}

/// The map's id, read off a benchmark id `ZAM_Town-1_…`: its country, name and
/// number, or Zamunda, the whole id's letters and digits, and 1 for one that does
/// not read that way.
fn map_id(benchmark_id: &str) -> common::MapId {
    let id = benchmark_id.strip_prefix("C-").unwrap_or(benchmark_id);
    let parsed = id.split_once('_').and_then(|(country, rest)| {
        let map = rest.split('_').next()?;
        let (name, number) = map.split_once('-')?;
        Some((country, name, number.parse::<u64>().ok()?))
    });
    match parsed {
        Some((country, name, number)) => common::MapId {
            country_id: country.to_owned(),
            map_name: name.to_owned(),
            map_id: number,
        },
        None => common::MapId {
            country_id: COUNTRY.to_owned(),
            map_name: benchmark_id
                .chars()
                .filter(char::is_ascii_alphanumeric)
                .collect(),
            map_id: 1,
        },
    }
}

/// A `YYYY-MM-DD` date as the schema's time stamp.
fn time_stamp(date: &str) -> common::TimeStamp {
    let mut parts = date.split('-').map(|part| part.parse::<u32>().ok());
    common::TimeStamp {
        year: parts.next().flatten(),
        month: parts.next().flatten(),
        day: parts.next().flatten(),
        hour: None,
        minute: None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn xml_names_become_the_schemas() {
        assert_eq!(
            enumerated(
                "bicycleLane",
                map_pb::lanelet_type_enum::LaneletType::from_str_name
            )
            .unwrap(),
            map_pb::lanelet_type_enum::LaneletType::BicycleLane as i32
        );
        assert_eq!(
            line_marking("solid_dashed"),
            map_pb::line_marking_enum::LineMarking::SolidDashed as i32
        );
        assert_eq!(
            enumerated(
                "mainCarriageWay",
                map_pb::lanelet_type_enum::LaneletType::from_str_name
            )
            .unwrap(),
            map_pb::lanelet_type_enum::LaneletType::MainCarriageWay as i32
        );
    }

    #[test]
    fn a_name_in_the_schemas_spelling_is_kept() {
        assert_eq!(
            enumerated(
                "MAX_SPEED",
                common::traffic_sign_id_enum::TrafficSignId::from_str_name
            )
            .unwrap(),
            common::traffic_sign_id_enum::TrafficSignId::MaxSpeed as i32
        );
    }

    #[test]
    fn a_benchmark_id_names_the_map() {
        let id = map_id("ZAM_StraightRoad-1_1_T-1");
        assert_eq!(
            (id.country_id.as_str(), id.map_name.as_str(), id.map_id),
            ("ZAM", "StraightRoad", 1)
        );
        let odd = map_id("town one");
        assert_eq!(
            (odd.country_id.as_str(), odd.map_name.as_str()),
            ("ZAM", "townone")
        );
    }
}
