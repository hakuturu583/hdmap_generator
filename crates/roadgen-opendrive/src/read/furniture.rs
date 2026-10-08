//! Signals, objects and the rules over them.
//!
//! Everything here is placed in a road's own coordinates — `(s, t, zOffset)` along,
//! across and above the reference line — and the IR holds positions in space, so
//! each is put through the road's frame at its station: the inverse of
//! [`crate::road_coordinates::locate`], which is what put it there.
//!
//! A `<signal>` is a traffic light when it is dynamic and a sign otherwise. It
//! governs the lanes its own validity names on its road plus, for every
//! `<signalReference>` to it, the lanes that reference names on its road: a
//! junction's light is one post, placed once, that other roads refer to (CARLA's
//! Town maps stand theirs on a sidewalk with `fromLane="0" toLane="0"`, valid on no
//! lane of their own road, and govern the junction only by reference). A light
//! the document names over the lanes *inside* a junction — CARLA's references
//! stand on the connecting roads — is read as governing the approaches that feed
//! them, because the IR's light, like Autoware's, is about the lanes that stop for
//! it. A `roadMark` object named as a stop line is one; a `crosswalk` object with
//! an outline is a band; a `building` object with outlines is a building of as
//! many parts. A `<controller>` is the lights that switch together, which is what
//! a [`TrafficRule::TrafficLight`] says — one rule, stopping at a line for each
//! junction mouth its lanes enter by, drawn by the reader where the document
//! draws none; a junction's `<priority>` is one connecting road over another,
//! which the IR says as the approach lanes that feed them.

use std::collections::HashMap;

use opendrive::object::corner::Corner;
use opendrive::object::lane_validity::LaneValidity;
use opendrive::object::orientation::{ObjectType, Orientation};
use opendrive::object::outline::Outline;
use opendrive::object::Object;
use opendrive::signal::signal_reference::SignalReference;
use opendrive::signal::Signal;
use uom::si::angle::radian;
use uom::si::length::meter;

use roadgen_core::buildings::{Building, BuildingPart, Footprint, Frontage, Solid};
use roadgen_core::geometry::{Curve3, Frame3, Point3};
use roadgen_core::id::{BuildingId, BuildingPartId, LaneId, ObjectId, RoadId};
use roadgen_core::map::Road;
use roadgen_core::semantics::{LightHead, MapObject, MapObjectKind, ObjectGeometry, TrafficRule};
use roadgen_core::topology::{Direction, LateralSide, RoadEnd, RoadEndpoint};
use roadgen_core::trace::{IrRef, Relation};

use super::Reader;
use crate::bulbs::{self, BULB_CODE};
use crate::controllers::junction_mouth;
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

    // Every road's references to a signal, by the signal's id, gathered before any
    // signal is read: a reference may stand on a road that comes after the signal.
    let mut references: HashMap<&str, Vec<(Road, &SignalReference)>> = HashMap::new();
    for road in &document.road {
        for reference in road.signals.iter().flat_map(|s| s.signal_reference.iter()) {
            let entry = reader
                .map
                .roads
                .get(&RoadId::new(&road.id))
                .cloned()
                .expect("roads are read first");
            references
                .entry(reference.id.as_str())
                .or_default()
                .push((entry, reference));
        }
    }

    for road in &document.road {
        // One copy per road, so that the road can be read while objects are added.
        let entry = reader
            .map
            .roads
            .get(&RoadId::new(&road.id))
            .cloned()
            .expect("roads are read first");
        for signal in road.signals.iter().flat_map(|s| s.signal.iter()) {
            let referenced = references
                .get(signal.id.as_str())
                .map_or(&[][..], Vec::as_slice);
            if let Some(id) = reader.read_signal(&entry, signal, referenced)? {
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

    read_controllers(reader, &signal_ids, &stop_lines)?;

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
                        push_unique(held, lane);
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

/// Where one junction mouth of a controller's lanes stops traffic.
enum StopAt {
    /// At a stop line the document draws.
    Drawn(ObjectId),
    /// At a junction mouth the document draws no line across, where the reader
    /// draws one.
    Mouth(RoadEndpoint),
}

/// A traffic-light rule read from a controller, before the stop lines it needs
/// are drawn.
struct PendingRule {
    /// The controller it was read from, as the trace names it.
    controller: String,
    lights: Vec<ObjectId>,
    lanes: Vec<LaneId>,
    /// Where it stops traffic, one per mouth of its lanes that the map can say.
    stops: Vec<(StopAt, Vec<LaneId>)>,
}

/// A controller is the lights that switch together: one traffic-light rule over
/// the lanes those lights govern, with the stop lines across them.
///
/// The lanes of one controller can reach their junction by more than one mouth —
/// one light whose references fan out over two incoming roads, or a controller
/// of several lights — and each mouth stops traffic at its own line. So each
/// mouth is resolved on its own: at the stop line the document draws across its
/// lanes, as a map roadgen wrote comes back, or — where it draws none, as
/// CARLA's Town maps draw none at all — at one the reader draws across the mouth
/// (`mouth_line`). A light stops traffic *somewhere*, and Lanelet2's `ref_line`
/// is how Autoware knows where; the place is not in doubt, since by now every
/// lane a light governs is an approach (see `read_signal`), and it stops where it
/// enters the junction. Lanes that run into no junction stop at a document's line
/// if one crosses them, and nowhere otherwise.
///
/// The lines are drawn after every controller is read, one per mouth over every
/// lane that stops there, whichever controllers those lanes belong to: two
/// controllers over the lanes of one mouth — a turn arrow and the straight
/// ahead — share one painted line, as they do on the street.
fn read_controllers(
    reader: &mut Reader<'_>,
    signal_ids: &HashMap<&str, ObjectId>,
    stop_lines: &[ObjectId],
) -> Result<(), ImportError> {
    let mut pending: Vec<PendingRule> = Vec::new();
    for controller in &reader.document.controller {
        let mut lights: Vec<ObjectId> = Vec::new();
        let mut lanes: Vec<LaneId> = Vec::new();
        for control in controller.control.iter() {
            let Some(light) = signal_ids
                .get(control.signal_id.as_str())
                .and_then(|id| reader.map.objects.get(id))
                .filter(|object| object.kind.is_traffic_light())
            else {
                continue;
            };
            // CARLA lists one signal several times in one controller.
            push_unique(&mut lights, light.id.clone());
            for lane in &light.lanes {
                push_unique(&mut lanes, lane.clone());
            }
        }
        if lights.is_empty() {
            continue;
        }
        let controller = format!("controller:{}", controller.id);
        // Each light is a part of its controller, as the exporter records them.
        for light in &lights {
            reader.trace.link_as(
                light.clone(),
                controller.clone(),
                Relation::Merged,
                "controller",
            );
        }
        let stops = reader
            .mouths(&lanes)
            .into_iter()
            .filter_map(|(mouth, lanes)| {
                let stop = match reader.stop_line_across(stop_lines, &lanes) {
                    Some(line) => StopAt::Drawn(line),
                    None => StopAt::Mouth(mouth?),
                };
                Some((stop, lanes))
            })
            .collect();
        pending.push(PendingRule {
            controller,
            lights,
            lanes,
            stops,
        });
    }

    // One line across every mouth some rule stops at, over all the lanes that stop
    // there.
    let mut at_mouth: Vec<(RoadEndpoint, Vec<LaneId>)> = Vec::new();
    for (stop, lanes) in pending.iter().flat_map(|rule| &rule.stops) {
        let StopAt::Mouth(mouth) = stop else {
            continue;
        };
        let index = match at_mouth.iter().position(|(held, _)| held == mouth) {
            Some(index) => index,
            None => {
                at_mouth.push((mouth.clone(), Vec::new()));
                at_mouth.len() - 1
            }
        };
        for lane in lanes {
            push_unique(&mut at_mouth[index].1, lane.clone());
        }
    }
    let mut drawn: Vec<(RoadEndpoint, ObjectId)> = Vec::new();
    for (mouth, lanes) in at_mouth {
        if let Some(line) = reader.mouth_line(&mouth, lanes)? {
            drawn.push((mouth, line));
        }
    }

    for rule in pending {
        let mut lines: Vec<ObjectId> = Vec::new();
        for (stop, _) in &rule.stops {
            let line = match stop {
                StopAt::Drawn(line) => line.clone(),
                StopAt::Mouth(mouth) => {
                    let Some((_, line)) = drawn.iter().find(|(held, _)| held == mouth) else {
                        continue;
                    };
                    // A line the reader drew has no element of the document behind
                    // it; it is what the controller implies, so it is traced to the
                    // controller as something folded into it — the way a SUMO export
                    // traces a connector, which SUMO has no element for, to the
                    // connection it became.
                    reader.trace.link_as(
                        line.clone(),
                        rule.controller.clone(),
                        Relation::Collapsed,
                        "stop_line",
                    );
                    line.clone()
                }
            };
            push_unique(&mut lines, line);
        }
        reader.trace.link(
            IrRef::Rule(reader.map.rules.len()),
            rule.controller,
            Relation::Exact,
        );
        reader.map.rules.push(TrafficRule::TrafficLight {
            lights: rule.lights,
            stop_lines: lines,
            lanes: rule.lanes,
        });
    }
    Ok(())
}

/// Adds `item` to `items` unless it is there already, keeping the first order.
fn push_unique<T: PartialEq>(items: &mut Vec<T>, item: T) {
    if !items.contains(&item) {
        items.push(item);
    }
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

    /// Adds a map object read from the document's `od_id` under a name of its
    /// own, traced to that element, and hands the name back.
    fn place(
        &mut self,
        kind: MapObjectKind,
        geometry: ObjectGeometry,
        lanes: Vec<LaneId>,
        name: Option<&str>,
        od_id: &str,
    ) -> ObjectId {
        let id = self.unique_object_id(name, od_id);
        // The exporter writes a light or a sign as a `<signal>` and the rest as an
        // `<object>`, and so does this trace, in the same kinds.
        let source = match kind {
            MapObjectKind::TrafficLight { .. } | MapObjectKind::TrafficSign { .. } => "signal",
            _ => "object",
        };
        self.trace
            .link(id.clone(), format!("{source}:{od_id}"), Relation::Exact);
        self.insert_object(id.clone(), kind, geometry, lanes);
        id
    }

    /// The name for an object: its `name` when the exporter wrote one of the IR's
    /// own identifiers there, else `fallback`, made unique among the map's objects.
    fn unique_object_id(&self, name: Option<&str>, fallback: &str) -> ObjectId {
        ObjectId::new(unique_name(name, ObjectId::PREFIX, fallback, |candidate| {
            self.map.objects.contains(&ObjectId::new(candidate))
        }))
    }

    fn insert_object(
        &mut self,
        id: ObjectId,
        kind: MapObjectKind,
        geometry: ObjectGeometry,
        lanes: Vec<LaneId>,
    ) {
        let object = MapObject {
            id: id.clone(),
            kind,
            geometry,
            lanes,
        };
        self.map.objects.insert(id, object).ok();
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
        let lanes = self.governed(road, placement.0, validity, orientation);
        self.place_across(road, kind, placement, lanes, (name, od_id))
    }

    /// Something across the road at one station over the given lanes; `None`, and
    /// counted, when there are none.
    fn place_across(
        &mut self,
        road: &Road,
        kind: MapObjectKind,
        placement: Placement,
        lanes: Vec<LaneId>,
        (name, od_id): (Option<&str>, &str),
    ) -> Result<Option<ObjectId>, ImportError> {
        let frame = road.frame_at(placement.0, self.sampling())?;
        let geometry = line_or_point(&frame, placement)?;
        if lanes.is_empty() {
            self.approximations.count(format!(
                "{{n}} {}s govern no lane of the road they are on, and are not read",
                kind.as_str().replace('_', " ")
            ));
            return Ok(None);
        }
        Ok(Some(self.place(kind, geometry, lanes, name, od_id)))
    }

    /// A signal where it stands, over the lanes it governs: those its validity
    /// names on its own road and those each reference to it names on the
    /// reference's road. Without a validity the own road contributes every lane
    /// facing the signal, unless references say where it applies.
    fn read_signal(
        &mut self,
        road: &Road,
        signal: &Signal,
        references: &[(Road, &SignalReference)],
    ) -> Result<Option<ObjectId>, ImportError> {
        let kind = if signal.dynamic || signal.r#type == TRAFFIC_LIGHT_TYPE {
            let (bulbs, unreadable) = bulbs::from_user_data(&signal.additional_data.user_data);
            if unreadable > 0 {
                self.approximations.count(format!(
                    "{{n}} `<userData code=\"{BULB_CODE}\">` of traffic lights do not say a \
                     lamp's colour and position the way the writer does, and are not read"
                ));
            }
            MapObjectKind::TrafficLight {
                head: LightHead {
                    height: signal
                        .height
                        .map(|height| height.get::<meter>())
                        .filter(|height| height.is_finite() && *height > 0.0),
                    bulbs,
                },
            }
        } else {
            // The caller's own code is the `type`, with the subtype after a slash
            // when the document states one — a catalogue sign is `type/subtype`.
            let code = match signal.subtype.trim() {
                "" | "-1" => signal.r#type.clone(),
                subtype => format!("{}/{subtype}", signal.r#type),
            };
            MapObjectKind::TrafficSign { code }
        };
        let s = signal.s.get::<meter>();
        let mut lanes = if signal.validity.is_empty() && !references.is_empty() {
            Vec::new()
        } else {
            self.governed(road, s, &signal.validity, Some(&signal.orientation))
        };
        for (referring, reference) in references {
            let governed = self.governed(
                referring,
                reference.s.get::<meter>(),
                &reference.validity,
                Some(&reference.orientation),
            );
            for lane in governed {
                push_unique(&mut lanes, lane);
            }
        }
        // A light names the lanes it stops, which are the approaches to the
        // junction; what the document names inside the junction is read as the
        // approaches that feed it (see `onto_approaches`). A sign keeps the lanes
        // it names: what a sign means is its code, which the IR passes through
        // without reading, and a sign inside a junction may well mean the junction
        // — a speed limit on a connecting road applies on it — so nothing says the
        // lanes behind it are the ones it is about.
        if kind.is_traffic_light() {
            lanes = self.onto_approaches(lanes);
        }
        self.place_across(
            road,
            kind,
            (
                s,
                signal.t.get::<meter>(),
                signal.z_offset.get::<meter>(),
                signal.width.map(|width| width.get::<meter>()),
            ),
            lanes,
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
        self.trace
            .link(id.clone(), format!("object:{}", object.id), Relation::Exact);
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
            for from in self.feeding(&lane.id) {
                push_unique(&mut found, from);
            }
        }
        found
    }

    /// Whether `lane` is a lane of a junction's connecting road.
    fn is_inside_junction(&self, lane: &LaneId) -> bool {
        self.map
            .lanes
            .get(lane)
            .and_then(|entry| self.map.roads.get(&entry.road))
            .is_some_and(Road::is_connector)
    }

    /// The lanes the map's connections lead into `lane` from, worked out for
    /// every lane at once the first time one is asked for: the connections are
    /// all read by the time furniture is.
    fn predecessors(&self, lane: &LaneId) -> &[LaneId] {
        self.predecessors
            .get_or_init(|| {
                let mut index: HashMap<LaneId, Vec<LaneId>> = HashMap::new();
                for connection in self.map.connections.iter() {
                    index
                        .entry(connection.to.lane.clone())
                        .or_default()
                        .push(connection.from.lane.clone());
                }
                index
            })
            .get(lane)
            .map_or(&[], Vec::as_slice)
    }

    /// The lanes outside any junction that lead into `lane`, which is a lane of a
    /// connecting road: its predecessors, and theirs in turn while they are
    /// inside the junction too — a document may chain two connecting roads
    /// through one junction.
    fn feeding(&self, lane: &LaneId) -> Vec<LaneId> {
        let mut found: Vec<LaneId> = Vec::new();
        let mut seen: Vec<LaneId> = vec![lane.clone()];
        let mut queue: Vec<LaneId> = vec![lane.clone()];
        while let Some(inside) = queue.pop() {
            for from in self.predecessors(&inside) {
                if seen.contains(from) || !self.map.lanes.contains(from) {
                    continue;
                }
                seen.push(from.clone());
                if self.is_inside_junction(from) {
                    queue.push(from.clone());
                } else {
                    push_unique(&mut found, from.clone());
                }
            }
        }
        found
    }

    /// The lanes a traffic light governs, with every lane of a junction's
    /// connecting road among them replaced by the approaches that feed it.
    ///
    /// The IR's traffic light is a rule about the lanes that stop for it — the
    /// builder puts one at the end of an approach, the SUMO export signalises the
    /// junction *ahead* of each lane a light names, and Lanelet2 hangs the
    /// regulatory element on those lanelets, which is where Autoware looks for it
    /// before the junction. CARLA's Town maps say it the other way round: a
    /// junction's light is referred to from the connecting roads, the movements it
    /// switches, and names no lane outside the junction at all. Read as written,
    /// the light would govern the inside of the junction and a vehicle on the
    /// approach would see none. Every connecting lane has the approaches it is
    /// entered from, so the light is read as governing those: the same lanes,
    /// counted where they stop rather than where they go.
    ///
    /// A connecting lane nothing outside the junction leads into is kept, and
    /// counted: there is no approach to put the light on instead.
    fn onto_approaches(&mut self, lanes: Vec<LaneId>) -> Vec<LaneId> {
        let mut onto: Vec<LaneId> = Vec::new();
        let mut moved = false;
        for lane in lanes {
            if !self.is_inside_junction(&lane) {
                push_unique(&mut onto, lane);
                continue;
            }
            let instead = self.feeding(&lane);
            if instead.is_empty() {
                self.approximations.count(
                    "{n} traffic lights govern a lane inside a junction that no lane outside \
                     it leads into, and govern that lane as written",
                );
                push_unique(&mut onto, lane);
                continue;
            }
            moved = true;
            for approach in instead {
                push_unique(&mut onto, approach);
            }
        }
        if moved {
            self.approximations.count(
                "{n} traffic lights name the lanes inside a junction they switch, and are \
                 read as governing the approaches that enter it, where traffic stops for them",
            );
        }
        onto
    }

    /// `lanes` gathered by where they stop: one group for each junction mouth
    /// they enter by (see [`junction_mouth`]), over its lanes in the order they
    /// are first named, and one group with no mouth for the rest, which run into
    /// no junction and stop nowhere the map can say.
    fn mouths(&self, lanes: &[LaneId]) -> Vec<(Option<RoadEndpoint>, Vec<LaneId>)> {
        let mut groups: Vec<(Option<RoadEndpoint>, Vec<LaneId>)> = Vec::new();
        let mut elsewhere: Vec<LaneId> = Vec::new();
        for id in lanes {
            let mouth = self
                .map
                .lanes
                .get(id)
                .and_then(|lane| junction_mouth(&self.map, lane))
                .map(|(mouth, _)| mouth);
            let Some(mouth) = mouth else {
                elsewhere.push(id.clone());
                continue;
            };
            match groups
                .iter_mut()
                .find(|(held, _)| held.as_ref() == Some(&mouth))
            {
                Some((_, held)) => held.push(id.clone()),
                None => groups.push((Some(mouth), vec![id.clone()])),
            }
        }
        if !elsewhere.is_empty() {
            groups.push((None, elsewhere));
        }
        groups
    }

    /// Draws the stop line a junction mouth has no line of its own for: a line
    /// across `lanes` where they leave the road, from the outermost boundary on
    /// the driver's left to the outermost on the right — the way the builder draws
    /// a stop line across a lane — and named `stopline/<road>/<end>` after the
    /// mouth, so that it reads as what it is and stays the same from one reading
    /// to the next. `None` for lanes that are not there.
    ///
    /// It is drawn at the very end of the lanes, the junction's edge, because that
    /// is the one place the document does say: a CARLA junction has no painted
    /// line, and its vehicles stop at the mouth. The width of the line is the
    /// lanes', so a mouth whose lanes are not side by side is crossed as a whole.
    fn mouth_line(
        &mut self,
        mouth: &RoadEndpoint,
        lanes: Vec<LaneId>,
    ) -> Result<Option<ObjectId>, ImportError> {
        let Some(road) = self.map.roads.get(&mouth.road) else {
            return Ok(None);
        };
        let frame = match mouth.end {
            RoadEnd::Start => road.frame_at(0.0, self.sampling())?,
            RoadEnd::End => road.frame_at(road.horizontal_length()?, self.sampling())?,
        };
        // The lanes leave the road here, so in travel order each boundary's last
        // point is at the mouth, with the driver's left and right already sorted
        // out; the outermost of each is how far across the road it is.
        let mut lefts: Vec<(f64, Point3)> = Vec::new();
        let mut rights: Vec<(f64, Point3)> = Vec::new();
        for id in &lanes {
            let Some(lane) = self.map.lanes.get(id) else {
                continue;
            };
            let travel = lane.travel_geometry(self.sampling())?;
            for (boundary, ends) in [(&travel.left, &mut lefts), (&travel.right, &mut rights)] {
                let point = boundary.end_point();
                ends.push((frame.to_local(point)[1], point));
            }
        }
        // Lanes leaving at the start run against the road, so their driver's left
        // is the road's right: the outermost on the driver's left is the furthest
        // that way.
        let leftward = |a: &(f64, Point3), b: &(f64, Point3)| match mouth.end {
            RoadEnd::End => a.0.total_cmp(&b.0),
            RoadEnd::Start => b.0.total_cmp(&a.0),
        };
        let (Some(&(_, from)), Some(&(_, to))) = (
            lefts.iter().max_by(|a, b| leftward(a, b)),
            rights.iter().min_by(|a, b| leftward(a, b)),
        ) else {
            return Ok(None);
        };
        let name = format!(
            "stopline/{}/{}",
            mouth.road.local_name(),
            mouth.end.as_str()
        );
        self.approximations.count(
            "{n} junction mouths have traffic lights and no stop line, and are read with \
             one drawn across the end of the lanes that stop there",
        );
        let id = self.unique_object_id(None, &name);
        self.insert_object(
            id.clone(),
            MapObjectKind::StopLine,
            ObjectGeometry::Line(Curve3::polyline([from, to])?),
            lanes,
        );
        Ok(Some(id))
    }
}
