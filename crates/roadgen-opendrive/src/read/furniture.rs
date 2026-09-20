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

use std::collections::HashSet;

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
use crate::TRAFFIC_LIGHT_TYPE;

/// Storeys per metre of wall, for a building whose document says how tall it is
/// and not how many floors: three metres a floor, which is what the generator
/// assumes too.
const FLOOR_HEIGHT: f64 = 3.0;

/// Reads every road's signals and objects, then the controllers and priorities
/// that refer to them.
pub fn read(reader: &mut Reader<'_>) -> Result<(), ImportError> {
    let document = reader.document;
    let mut signal_ids: Vec<(String, ObjectId)> = Vec::new();
    let mut stop_lines: Vec<ObjectId> = Vec::new();

    for road in &document.road {
        let road_id = RoadId::new(&road.id);
        for signal in road
            .signals
            .iter()
            .flat_map(|signals| signals.signal.iter())
        {
            if let Some(id) = reader.read_signal(&road_id, signal)? {
                signal_ids.push((signal.id.clone(), id));
            }
        }
        for object in road
            .objects
            .iter()
            .flat_map(|objects| objects.object.iter())
        {
            match object.r#type {
                Some(ObjectType::Building) => reader.read_building(&road_id, object)?,
                Some(ObjectType::Crosswalk) => {
                    reader.read_crosswalk(&road_id, object)?;
                }
                Some(ObjectType::RoadMark) if is_stop_line(object) => {
                    if let Some(id) = reader.read_stop_line(&road_id, object)? {
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
        let lights: Vec<ObjectId> = controller
            .control
            .iter()
            .filter_map(|control| {
                signal_ids
                    .iter()
                    .find(|(od, _)| *od == control.signal_id)
                    .map(|(_, id)| id.clone())
            })
            .filter(|id| {
                reader
                    .map
                    .objects
                    .get(id)
                    .is_some_and(|object| object.kind == MapObjectKind::TrafficLight)
            })
            .collect();
        if lights.is_empty() {
            continue;
        }
        let mut lanes: Vec<LaneId> = Vec::new();
        for light in &lights {
            for lane in &reader.map.objects.get(light).expect("found above").lanes {
                if !lanes.contains(lane) {
                    lanes.push(lane.clone());
                }
            }
        }
        let stop_line = stop_lines
            .iter()
            .find(|id| {
                reader
                    .map
                    .objects
                    .get(id)
                    .is_some_and(|object| object.lanes.iter().any(|lane| lanes.contains(lane)))
            })
            .cloned();
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
        let mut highs: Vec<String> = Vec::new();
        for priority in &junction.priority {
            if let Some(high) = &priority.high {
                if !highs.contains(high) {
                    highs.push(high.clone());
                }
            }
        }
        for high in highs {
            let right_of_way = reader.approaches(&RoadId::new(&high));
            let mut yielding: Vec<LaneId> = Vec::new();
            for priority in &junction.priority {
                if priority.high.as_deref() != Some(high.as_str()) {
                    continue;
                }
                if let Some(low) = &priority.low {
                    for lane in reader.approaches(&RoadId::new(low)) {
                        if !yielding.contains(&lane) {
                            yielding.push(lane);
                        }
                    }
                }
            }
            if right_of_way.is_empty() || yielding.is_empty() {
                reader.approximations.count(
                    "{n} junction priorities name a connecting road no lane approaches, and \
                     are dropped",
                );
                continue;
            }
            let stop_line = stop_lines
                .iter()
                .find(|id| {
                    reader.map.objects.get(id).is_some_and(|object| {
                        object.lanes.iter().any(|lane| yielding.contains(lane))
                    })
                })
                .cloned();
            reader.map.rules.push(TrafficRule::RightOfWay {
                right_of_way,
                yielding,
                stop_line,
            });
        }
    }
    Ok(())
}

/// Whether a `roadMark` object is the stop line the exporter writes.
fn is_stop_line(object: &Object) -> bool {
    let says = |text: &Option<String>| {
        text.as_deref().is_some_and(|text| {
            text.eq_ignore_ascii_case("stopline") || text.eq_ignore_ascii_case("stop_line")
        })
    };
    says(&object.subtype) || says(&object.name)
}

impl Reader<'_> {
    /// The road frame at a station: banked, since a signal's `zOffset` is above the
    /// road surface.
    fn frame(&self, road: &Road, s: f64) -> Result<Frame3, ImportError> {
        Ok(road.frame_at(s, self.sampling())?)
    }

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
        let opendrive_id = |side: LateralSide, ordinal: usize| match side {
            LateralSide::Left => ordinal as i64,
            LateralSide::Right => -(ordinal as i64),
        };
        if !validity.is_empty() {
            return lanes
                .iter()
                .filter(|lane| {
                    let id = opendrive_id(lane.side, lane.ordinal);
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

    /// An identifier for a signal or object: its `name` when the exporter wrote
    /// the IR's own identifier there, else its OpenDRIVE id — made unique either
    /// way, since signals and objects are numbered separately and a foreign file may
    /// reuse a number.
    fn object_id(&self, name: Option<&str>, id: &str) -> ObjectId {
        let wanted = match name {
            Some(name) if name.starts_with(&format!("{}/", ObjectId::PREFIX)) => {
                ObjectId::parse_printed(name)
            }
            _ => ObjectId::new(id),
        };
        let mut candidate = wanted.clone();
        let mut suffix = 1;
        while self.map.objects.contains(&candidate) {
            candidate = ObjectId::new(format!(
                "{}-{suffix}",
                wanted.as_str().trim_start_matches("object/")
            ));
            suffix += 1;
        }
        candidate
    }

    fn read_signal(
        &mut self,
        road_id: &RoadId,
        signal: &Signal,
    ) -> Result<Option<ObjectId>, ImportError> {
        let road = self
            .map
            .roads
            .get(road_id)
            .cloned()
            .expect("roads are read first");
        let s = signal.s.get::<meter>();
        let t = signal.t.get::<meter>();
        let height = signal.z_offset.get::<meter>();
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
        let frame = self.frame(&road, s)?;
        let geometry = match signal.width.map(|width| width.get::<meter>()) {
            Some(width) if width > 0.0 => ObjectGeometry::Line(Curve3::polyline([
                frame.to_global([0.0, t + width / 2.0, height]),
                frame.to_global([0.0, t - width / 2.0, height]),
            ])?),
            _ => ObjectGeometry::Point(frame.to_global([0.0, t, height])),
        };
        let lanes = self.governed(&road, s, &signal.validity, Some(&signal.orientation));
        if lanes.is_empty() {
            self.approximations.count(
                "{n} signals govern no lane of the road they are on, and are not read: the \
                 IR places furniture by the lanes it applies to",
            );
            return Ok(None);
        }
        let id = self.object_id(signal.name.as_deref(), &signal.id);
        self.map
            .objects
            .insert(
                id.clone(),
                MapObject {
                    id: id.clone(),
                    kind,
                    geometry,
                    lanes,
                },
            )
            .ok();
        Ok(Some(id))
    }

    fn read_stop_line(
        &mut self,
        road_id: &RoadId,
        object: &Object,
    ) -> Result<Option<ObjectId>, ImportError> {
        let road = self
            .map
            .roads
            .get(road_id)
            .cloned()
            .expect("roads are read first");
        let s = object.s.get::<meter>();
        let t = object.t.get::<meter>();
        let height = object.z_offset.get::<meter>();
        let frame = self.frame(&road, s)?;
        let width = object
            .width
            .map(|width| width.get::<meter>())
            .unwrap_or(0.0);
        let geometry = if width > 0.0 {
            ObjectGeometry::Line(Curve3::polyline([
                frame.to_global([0.0, t + width / 2.0, height]),
                frame.to_global([0.0, t - width / 2.0, height]),
            ])?)
        } else {
            ObjectGeometry::Point(frame.to_global([0.0, t, height]))
        };
        let lanes = self.governed(&road, s, &object.validity, object.orientation.as_ref());
        if lanes.is_empty() {
            self.approximations
                .count("{n} stop lines govern no lane of the road they are on, and are not read");
            return Ok(None);
        }
        let id = self.object_id(object.name.as_deref(), &object.id);
        self.map
            .objects
            .insert(
                id.clone(),
                MapObject {
                    id: id.clone(),
                    kind: MapObjectKind::StopLine,
                    geometry,
                    lanes,
                },
            )
            .ok();
        Ok(Some(id))
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
        let sample = road.reference_line.sample_at(s, self.sampling())?;
        let plan = Frame3::from_tangent(sample.point, sample.tangent)?;
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
                    let sample = road
                        .reference_line
                        .sample_at(at.s.get::<meter>(), self.sampling())?;
                    let frame = Frame3::from_tangent(sample.point, sample.tangent)?;
                    (
                        frame.to_global([0.0, at.t.get::<meter>(), at.dz.get::<meter>()]),
                        at.height.get::<meter>(),
                    )
                }
            });
        }
        Ok(corners)
    }

    fn read_crosswalk(&mut self, road_id: &RoadId, object: &Object) -> Result<(), ImportError> {
        let road = self
            .map
            .roads
            .get(road_id)
            .cloned()
            .expect("roads are read first");
        let Some(outline) = &object.outline else {
            self.approximations
                .count("{n} crosswalks have no outline and are not read");
            return Ok(());
        };
        let mut ring: Vec<Point3> = self
            .corners(&road, object, outline)?
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
        let s = object.s.get::<meter>();
        let lanes = self.governed(&road, s, &object.validity, None);
        if lanes.is_empty() {
            self.approximations
                .count("{n} crosswalks cross no lane and are not read");
            return Ok(());
        }
        let id = self.object_id(object.name.as_deref(), &object.id);
        self.map
            .objects
            .insert(
                id.clone(),
                MapObject {
                    id: id.clone(),
                    kind: MapObjectKind::Crosswalk,
                    geometry,
                    lanes,
                },
            )
            .ok();
        Ok(())
    }

    fn read_building(&mut self, road_id: &RoadId, object: &Object) -> Result<(), ImportError> {
        let road = self
            .map
            .roads
            .get(road_id)
            .cloned()
            .expect("roads are read first");
        let outlines: Vec<&Outline> = match (&object.outlines, &object.outline) {
            (Some(outlines), _) => outlines.outline.iter().collect(),
            (None, Some(outline)) => vec![outline],
            (None, None) => {
                self.approximations
                    .count("{n} buildings have no outline and are not read");
                return Ok(());
            }
        };
        let wanted = match object.name.as_deref() {
            Some(name) if name.starts_with(&format!("{}/", BuildingId::PREFIX)) => {
                BuildingId::parse_printed(name)
            }
            _ => BuildingId::new(&object.id),
        };
        let mut id = wanted.clone();
        let mut suffix = 1;
        while self.map.buildings.contains(&id) {
            id = BuildingId::new(format!("{}-{suffix}", wanted.local_name()));
            suffix += 1;
        }

        let mut parts = Vec::with_capacity(outlines.len());
        for (index, outline) in outlines.iter().enumerate() {
            let corners = self.corners(&road, object, outline)?;
            let wall_height = corners
                .iter()
                .map(|(_, height)| *height)
                .fold(0.0_f64, f64::max);
            let footprint = match Footprint::new(corners.iter().map(|(point, _)| *point)) {
                Ok(footprint) => footprint,
                Err(error) => {
                    self.approximations.note(format!(
                        "building {}: outline {index} is not a footprint ({error}) and is not read",
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
            let part_id = BuildingPartId::of_building(&id, index);
            parts.push(BuildingPart {
                id: part_id,
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

    /// The lanes that feed a connecting road from outside its junction.
    fn approaches(&self, connector: &RoadId) -> Vec<LaneId> {
        let mut found: Vec<LaneId> = Vec::new();
        let mut seen: HashSet<LaneId> = HashSet::new();
        for lane in self.map.lanes_of(connector) {
            for connection in self.map.connections_to(&lane.id) {
                let from = &connection.from.lane;
                let outside = self
                    .map
                    .lanes
                    .get(from)
                    .and_then(|lane| self.map.roads.get(&lane.road))
                    .is_some_and(|road| !road.is_connector());
                if outside && seen.insert(from.clone()) {
                    found.push(from.clone());
                }
            }
        }
        found
    }
}
