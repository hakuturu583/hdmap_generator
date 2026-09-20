//! Signals, objects and the rules over them.
//!
//! Everything here is placed in a road's own coordinates — `(s, t, zOffset)` along,
//! across and above the reference line — and the IR holds positions in space, so
//! each is put through the road's frame at its station: the inverse of
//! [`crate::road_coordinates::locate`], which is what put it there.
//!
//! A `<signal>` is a traffic light when it is dynamic and a sign otherwise; a
//! `roadMark` object named as a stop line is one; a `crosswalk` object with an
//! outline is a band; a `building` object with outlines is a building of as many
//! parts. A `<controller>` is the lights that switch together, which is what a
//! [`TrafficRule::TrafficLight`] says; a junction's `<priority>` is one connecting
//! road over another, which the IR says as the approach lanes that feed them.

use std::collections::HashMap;

use opendrive::object::corner::Corner;
use opendrive::object::lane_validity::LaneValidity;
use opendrive::object::orientation::{ObjectType, Orientation};
use opendrive::object::outline::Outline;
use opendrive::object::Object;
use opendrive::signal::Signal;
use uom::si::angle::radian;
use uom::si::length::meter;

use roadgen_core::buildings::{Building, BuildingPart, Footprint, Frontage, Solid};
use roadgen_core::geometry::{Curve3, Frame3, Point3};
use roadgen_core::id::{BuildingId, BuildingPartId, LaneId, ObjectId, RoadId};
use roadgen_core::map::Road;
use roadgen_core::semantics::{MapObject, MapObjectKind, ObjectGeometry, TrafficRule};
use roadgen_core::topology::{Direction, LateralSide};

use super::Reader;
use crate::error::ImportError;
use crate::{lane_number, STOP_LINE_SUBTYPE, TRAFFIC_LIGHT_TYPE};

/// Storeys per metre of wall, for a building whose document says how tall it is
/// and not how many floors: three metres a floor, which is what the generator
/// assumes too.
const FLOOR_HEIGHT: f64 = 3.0;

/// Where something across the road stands: `(s, t, height above the surface)`, and
/// how far across it reaches when the document says.
type Placement = (f64, f64, f64, Option<f64>);

/// Reads every road's signals and objects, then the controllers and priorities
/// that refer to them.
pub fn read<'a>(reader: &mut Reader<'a>) -> Result<(), ImportError> {
    let document = reader.document;
    let mut signal_ids: HashMap<&str, ObjectId> = HashMap::new();
    let mut stop_lines: Vec<ObjectId> = Vec::new();

    for road in &document.road {
        // One copy per road, so that the road can be read while objects are added.
        let entry = reader
            .map
            .roads
            .get(&RoadId::new(&road.id))
            .cloned()
            .expect("roads are read first");
        for signal in road.signals.iter().flat_map(|s| s.signal.iter()) {
            if let Some(id) = reader.read_signal(&entry, signal)? {
                signal_ids.entry(&signal.id).or_insert(id);
            }
        }
        for object in road.objects.iter().flat_map(|o| o.object.iter()) {
            match object.r#type {
                Some(ObjectType::Building) => reader.read_building(&entry, object)?,
                Some(ObjectType::Crosswalk) => reader.read_crosswalk(&entry, object)?,
                Some(ObjectType::RoadMark) if is_stop_line(object) => {
                    if let Some(id) = reader.read_stop_line(&entry, object)? {
                        stop_lines.push(id);
                    }
                }
                _ => reader.approximations.count(format!(
                    "the IR has no object of type {:?}, so {{n}} of them are not read",
                    object.r#type
                )),
            }
        }
    }

    // A controller is the lights that switch together: one traffic-light rule,
    // over the lanes those lights govern, with the stop line across them if there
    // is one.
    for controller in &document.controller {
        let mut lights: Vec<ObjectId> = Vec::new();
        let mut lanes: Vec<LaneId> = Vec::new();
        for control in controller.control.iter() {
            let Some(light) = signal_ids
                .get(control.signal_id.as_str())
                .and_then(|id| reader.map.objects.get(id))
                .filter(|object| object.kind == MapObjectKind::TrafficLight)
            else {
                continue;
            };
            lights.push(light.id.clone());
            for lane in &light.lanes {
                if !lanes.contains(lane) {
                    lanes.push(lane.clone());
                }
            }
        }
        if lights.is_empty() {
            continue;
        }
        let stop_line = reader.stop_line_across(&stop_lines, &lanes);
        reader.map.rules.push(TrafficRule::TrafficLight {
            lights,
            stop_line,
            lanes,
        });
    }

    // A priority is one connecting road over another; the IR says it as the lanes
    // that approach each. The pairs a junction states over one road with priority
    // are one rule.
    for junction in &document.junction {
        // The approaches of each connecting road, worked out once however many
        // priorities name it.
        let mut approaches: HashMap<&str, Vec<LaneId>> = HashMap::new();
        let mut approaches_of = |road: &'a str| -> Vec<LaneId> {
            approaches
                .entry(road)
                .or_insert_with(|| reader.approaches(&RoadId::new(road)))
                .clone()
        };
        let mut grouped: Vec<(&str, Vec<LaneId>)> = Vec::new();
        for priority in &junction.priority {
            let (Some(high), Some(low)) = (&priority.high, &priority.low) else {
                continue;
            };
            let yielding = approaches_of(low);
            match grouped.iter_mut().find(|(held, _)| *held == high.as_str()) {
                Some((_, held)) => {
                    for lane in yielding {
                        if !held.contains(&lane) {
                            held.push(lane);
                        }
                    }
                }
                None => grouped.push((high, yielding)),
            }
        }
        let rules: Vec<_> = grouped
            .into_iter()
            .map(|(high, yielding)| (approaches_of(high), yielding))
            .collect();
        for (right_of_way, yielding) in rules {
            if right_of_way.is_empty() || yielding.is_empty() {
                reader.approximations.count(
                    "{n} junction priorities name a connecting road no lane approaches, and \
                     are dropped",
                );
                continue;
            }
            let stop_line = reader.stop_line_across(&stop_lines, &yielding);
            reader.map.rules.push(TrafficRule::RightOfWay {
                right_of_way,
                yielding,
                stop_line,
            });
        }
    }
    Ok(())
}

/// Whether a `roadMark` object is a stop line: the subtype the exporter writes,
/// or a name that says so in a document from elsewhere.
fn is_stop_line(object: &Object) -> bool {
    let says = |text: &Option<String>| {
        text.as_deref().is_some_and(|text| {
            text.eq_ignore_ascii_case(STOP_LINE_SUBTYPE) || text.eq_ignore_ascii_case("stop_line")
        })
    };
    says(&object.subtype) || says(&object.name)
}

/// A line across the road at `(t, height)` reaching `width` either side, or a
/// point there when the document gives it no width.
fn line_or_point(
    frame: &Frame3,
    (_, t, height, width): Placement,
) -> Result<ObjectGeometry, ImportError> {
    Ok(match width {
        Some(width) if width > 0.0 => ObjectGeometry::Line(Curve3::polyline([
            frame.to_global([0.0, t + width / 2.0, height]),
            frame.to_global([0.0, t - width / 2.0, height]),
        ])?),
        _ => ObjectGeometry::Point(frame.to_global([0.0, t, height])),
    })
}

/// The local name to give a thing from the document: its `name` when the exporter
/// wrote one of the IR's own identifiers there, else the document's id — made
/// unique either way, since a document from elsewhere may reuse a number.
fn unique_name(
    name: Option<&str>,
    prefix: &str,
    od_id: &str,
    taken: impl Fn(&str) -> bool,
) -> String {
    let wanted = name
        .and_then(|name| name.strip_prefix(prefix))
        .and_then(|rest| rest.strip_prefix('/'))
        .unwrap_or(od_id);
    let mut candidate = wanted.to_owned();
    let mut suffix = 1;
    while taken(&candidate) {
        candidate = format!("{wanted}-{suffix}");
        suffix += 1;
    }
    candidate
}

impl Reader<'_> {
    /// The lanes an object governs: the ones its validity names, in the section at
    /// its station, else every lane there that runs the way it faces.
    fn governed(
        &mut self,
        road: &Road,
        s: f64,
        validity: &[LaneValidity],
        orientation: Option<&Orientation>,
    ) -> Vec<LaneId> {
        let Some(section) = road.section_at(s) else {
            return Vec::new();
        };
        let lanes = self.map.lanes_of_section(&road.id, section);
        if !validity.is_empty() {
            return lanes
                .iter()
                .filter(|lane| {
                    let id = lane_number(lane.side, lane.ordinal);
                    validity.iter().any(|range| {
                        id >= range.from_lane.min(range.to_lane)
                            && id <= range.from_lane.max(range.to_lane)
                    })
                })
                .map(|lane| lane.id.clone())
                .collect();
        }
        self.approximations.count(
            "{n} signals or objects name no lanes, and are read as governing every lane of \
             their section that runs the way they face",
        );
        lanes
            .iter()
            .filter(|lane| match orientation {
                Some(Orientation::Plus) => lane.direction == Direction::Forward,
                Some(Orientation::Minus) => lane.direction == Direction::Backward,
                _ => true,
            })
            .map(|lane| lane.id.clone())
            .collect()
    }

    /// Adds a map object under a name of its own, and hands the name back.
    fn place(
        &mut self,
        kind: MapObjectKind,
        geometry: ObjectGeometry,
        lanes: Vec<LaneId>,
        name: Option<&str>,
        od_id: &str,
    ) -> ObjectId {
        let id = ObjectId::new(unique_name(name, ObjectId::PREFIX, od_id, |candidate| {
            self.map.objects.contains(&ObjectId::new(candidate))
        }));
        let object = MapObject {
            id: id.clone(),
            kind,
            geometry,
            lanes,
        };
        self.map.objects.insert(id.clone(), object).ok();
        id
    }

    /// A signal or a stop line: something across the road at one station, over
    /// the lanes it governs. `None` when it governs no lane, since the IR places
    /// furniture by the lanes it applies to.
    fn read_across(
        &mut self,
        road: &Road,
        kind: MapObjectKind,
        placement: Placement,
        validity: &[LaneValidity],
        orientation: Option<&Orientation>,
        (name, od_id): (Option<&str>, &str),
    ) -> Result<Option<ObjectId>, ImportError> {
        let s = placement.0;
        let frame = road.frame_at(s, self.sampling())?;
        let geometry = line_or_point(&frame, placement)?;
        let lanes = self.governed(road, s, validity, orientation);
        if lanes.is_empty() {
            self.approximations.count(format!(
                "{{n}} {}s govern no lane of the road they are on, and are not read",
                kind.as_str().replace('_', " ")
            ));
            return Ok(None);
        }
        Ok(Some(self.place(kind, geometry, lanes, name, od_id)))
    }

    fn read_signal(
        &mut self,
        road: &Road,
        signal: &Signal,
    ) -> Result<Option<ObjectId>, ImportError> {
        let kind = if signal.dynamic || signal.r#type == TRAFFIC_LIGHT_TYPE {
            MapObjectKind::TrafficLight
        } else {
            // The caller's own code is the `type`, with the subtype after a slash
            // when the document states one — a catalogue sign is `type/subtype`.
            let code = match signal.subtype.trim() {
                "" | "-1" => signal.r#type.clone(),
                subtype => format!("{}/{subtype}", signal.r#type),
            };
            MapObjectKind::TrafficSign { code }
        };
        self.read_across(
            road,
            kind,
            (
                signal.s.get::<meter>(),
                signal.t.get::<meter>(),
                signal.z_offset.get::<meter>(),
                signal.width.map(|width| width.get::<meter>()),
            ),
            &signal.validity,
            Some(&signal.orientation),
            (signal.name.as_deref(), &signal.id),
        )
    }

    fn read_stop_line(
        &mut self,
        road: &Road,
        object: &Object,
    ) -> Result<Option<ObjectId>, ImportError> {
        self.read_across(
            road,
            MapObjectKind::StopLine,
            (
                object.s.get::<meter>(),
                object.t.get::<meter>(),
                object.z_offset.get::<meter>(),
                object.width.map(|width| width.get::<meter>()),
            ),
            &object.validity,
            object.orientation.as_ref(),
            (object.name.as_deref(), &object.id),
        )
    }

    /// The corners of an outline, in space.
    ///
    /// A local corner is measured from the object's pivot in a frame turned `hdg`
    /// from the road's at the object's station; a road corner is its own `(s, t)`.
    fn corners(
        &self,
        road: &Road,
        object: &Object,
        outline: &Outline,
    ) -> Result<Vec<(Point3, f64)>, ImportError> {
        let s = object.s.get::<meter>();
        let t = object.t.get::<meter>();
        let z = object.z_offset.get::<meter>();
        let heading = object.hdg.map(|hdg| hdg.get::<radian>()).unwrap_or(0.0);
        // The plan frame, not the banked one: `u` and `v` lie in the xy-plane, which
        // is what the standard measures them in and how the exporter wrote them.
        let plan = road.reference_line.sample_at(s, self.sampling())?.frame()?;
        let (sin, cos) = heading.sin_cos();
        let mut corners = Vec::with_capacity(outline.choice.len());
        for corner in outline.choice.iter() {
            corners.push(match corner {
                Corner::Local(local) => {
                    let (u, v) = (local.u.get::<meter>(), local.v.get::<meter>());
                    (
                        plan.to_global([
                            u * cos - v * sin,
                            t + u * sin + v * cos,
                            z + local.z.get::<meter>(),
                        ]),
                        local.height.get::<meter>(),
                    )
                }
                Corner::Road(at) => {
                    let frame = road
                        .reference_line
                        .sample_at(at.s.get::<meter>(), self.sampling())?
                        .frame()?;
                    (
                        frame.to_global([0.0, at.t.get::<meter>(), at.dz.get::<meter>()]),
                        at.height.get::<meter>(),
                    )
                }
            });
        }
        Ok(corners)
    }

    fn read_crosswalk(&mut self, road: &Road, object: &Object) -> Result<(), ImportError> {
        let Some(outline) = &object.outline else {
            self.approximations
                .count("{n} crosswalks have no outline and are not read");
            return Ok(());
        };
        let mut ring: Vec<Point3> = self
            .corners(road, object, outline)?
            .into_iter()
            .map(|(point, _)| point)
            .collect();
        // The closing repeat, which the exporter writes because CARLA reads it.
        if ring.len() > 1 && ring[0].is_close(ring[ring.len() - 1], 1e-6) {
            ring.pop();
        }
        if ring.len() != 4 {
            self.approximations.count(
                "a crosswalk is a band with two edges in the IR, so {n} crosswalks whose \
                 outline is not four corners are not read",
            );
            return Ok(());
        }
        // Written as [left.start, left.end, right.end, right.start].
        let geometry = ObjectGeometry::Band {
            left: Curve3::polyline([ring[0], ring[1]])?,
            right: Curve3::polyline([ring[3], ring[2]])?,
        };
        let lanes = self.governed(road, object.s.get::<meter>(), &object.validity, None);
        if lanes.is_empty() {
            self.approximations
                .count("{n} crosswalks cross no lane and are not read");
            return Ok(());
        }
        self.place(
            MapObjectKind::Crosswalk,
            geometry,
            lanes,
            object.name.as_deref(),
            &object.id,
        );
        Ok(())
    }

    fn read_building(&mut self, road: &Road, object: &Object) -> Result<(), ImportError> {
        let outlines: Vec<&Outline> = match (&object.outlines, &object.outline) {
            (Some(outlines), _) => outlines.outline.iter().collect(),
            (None, Some(outline)) => vec![outline],
            (None, None) => {
                self.approximations
                    .count("{n} buildings have no outline and are not read");
                return Ok(());
            }
        };
        let id = BuildingId::new(unique_name(
            object.name.as_deref(),
            BuildingId::PREFIX,
            &object.id,
            |candidate| self.map.buildings.contains(&BuildingId::new(candidate)),
        ));

        let mut parts = Vec::with_capacity(outlines.len());
        for (index, outline) in outlines.iter().enumerate() {
            let corners = self.corners(road, object, outline)?;
            let wall_height = corners
                .iter()
                .map(|(_, height)| *height)
                .fold(0.0_f64, f64::max);
            let footprint = match Footprint::new(corners.iter().map(|(point, _)| *point)) {
                Ok(footprint) => footprint,
                Err(error) => {
                    self.approximations.note(format!(
                        "building {}: outline {index} is not a footprint ({error}) and is not \
                         read",
                        object.id
                    ));
                    continue;
                }
            };
            if wall_height <= 0.0 {
                self.approximations.note(format!(
                    "building {}: outline {index} has no height and is not read",
                    object.id
                ));
                continue;
            }
            parts.push(BuildingPart {
                id: BuildingPartId::of_building(&id, index),
                building: id.clone(),
                solid: Solid::prism(footprint, wall_height),
                kind: None,
                levels: ((wall_height / FLOOR_HEIGHT).round() as i64).max(1) as u32,
            });
        }
        if parts.is_empty() {
            return Ok(());
        }
        self.approximations.count(
            "an outline is a ring of corner heights and has no ridge in it, so {n} buildings \
             are read with flat roofs",
        );
        let t = object.t.get::<meter>();
        let building = Building {
            id: id.clone(),
            parts: parts.iter().map(|part| part.id.clone()).collect(),
            kind: object
                .subtype
                .clone()
                .unwrap_or_else(|| "building".to_owned()),
            frontage: Some(Frontage {
                road: road.id.clone(),
                side: if t >= 0.0 {
                    LateralSide::Left
                } else {
                    LateralSide::Right
                },
                station: object.s.get::<meter>(),
            }),
        };
        self.map.buildings.insert(id, building).ok();
        for part in parts {
            self.map.building_parts.insert(part.id.clone(), part).ok();
        }
        Ok(())
    }

    /// The first stop line that lies across any of `lanes`.
    fn stop_line_across(&self, stop_lines: &[ObjectId], lanes: &[LaneId]) -> Option<ObjectId> {
        stop_lines
            .iter()
            .find(|id| {
                self.map
                    .objects
                    .get(id)
                    .is_some_and(|object| object.lanes.iter().any(|lane| lanes.contains(lane)))
            })
            .cloned()
    }

    /// The lanes that feed a connecting road from outside its junction.
    fn approaches(&self, connector: &RoadId) -> Vec<LaneId> {
        let mut found: Vec<LaneId> = Vec::new();
        for lane in self.map.lanes_of(connector) {
            for connection in self.map.connections_to(&lane.id) {
                let from = &connection.from.lane;
                let outside = self
                    .map
                    .lanes
                    .get(from)
                    .and_then(|lane| self.map.roads.get(&lane.road))
                    .is_some_and(|road| !road.is_connector());
                if outside && !found.contains(from) {
                    found.push(from.clone());
                }
            }
        }
        found
    }
}
