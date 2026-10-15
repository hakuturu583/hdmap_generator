//! Messages, as the IR.
//!
//! The encoder's table read the other way. Every value goes back through the IR's
//! own constructor, so a message from some other writer cannot put a value into
//! the map that the IR would not have built: a width of zero, a polyline of one
//! point, an origin off the globe. Identifiers are taken exactly as written.

use std::fmt::Display;
use std::hash::Hash;

use roadgen_core::arena::{Arena, DuplicateId};
use roadgen_core::buildings::{
    Building, BuildingPart, Footprint, Frontage, Roof, RoofShape, Solid,
};
use roadgen_core::error::GeometryError;
use roadgen_core::geometry::{
    Arc3, Bezier3, Clothoid3, Curve3, LaneHeight, Point3, Poly3Piece, Poly3Profile, Polyline3,
    SamplingConfig, Taper, WidthProfile,
};
use roadgen_core::id::{
    BuildingId, BuildingPartId, ConnectionId, GradeSeparationId, InterchangeId, JunctionId,
    LandCoverId, LaneId, ObjectId, RoadId, SiteId, StructureId, TinBreaklineId, TinSurfaceId,
};
use roadgen_core::land_cover::LandCover;
use roadgen_core::map::{
    CrossSection, Lane, Map, MapMetadata, Projection, Road, TrafficHandedness,
};
use roadgen_core::semantics::{
    BoundaryMarking, FormOfWay, LaneType, LightArrow, LightBulb, LightColor, LightHead, MapObject,
    MapObjectKind, MarkingColor, ObjectGeometry, PavedSurface, Pavement, PavementProfile,
    RoadMarking, RoadType, TrafficRule, UnpavedSurface,
};
use roadgen_core::site::Site;
use roadgen_core::structure::{
    Bridge, BridgeForm, GradeSeparation, Interchange, Portal, RoadSpan, Structure, StructureKind,
    StructureMaterial, StructureProfile, StructureSpan, Support, SupportKind, Tunnel, TunnelForm,
    Underneath,
};
use roadgen_core::terrain::{Tin, TinBreakline, TinSurface};
use roadgen_core::topology::{
    Direction, Junction, LaneConnection, LaneEnd, LaneEndpoint, LateralSide, RoadEnd, RoadEndpoint,
    RoadLink, RoadLinkTarget,
};
use roadgen_core::units::{GeoOrigin, PositiveWidth, SpeedLimit};

use crate::error::{invalid, Indexed, Within};
use crate::v1;
use crate::ReadError;

type Result<T> = std::result::Result<T, ReadError>;

pub(crate) fn map(message: &v1::Map) -> Result<Map> {
    let mut map = Map::new(metadata(required(&message.metadata)?).within("metadata")?);
    // Every arena but the rules, which have no identifiers: in the schema's
    // order, each refusing an identifier it has already seen.
    macro_rules! fill {
        ($($field:ident => $decode:ident),* $(,)?) => {$(
            fill(&mut map.$field, &message.$field, stringify!($field), $decode, |value| {
                value.id.clone()
            })?;
        )*};
    }
    fill! {
        roads => road,
        lanes => lane,
        junctions => junction,
        connections => connection,
        objects => object,
    }
    map.rules = message
        .rules
        .iter()
        .enumerate()
        .map(|(index, message)| rule(message).within(Indexed("rules", index)))
        .collect::<Result<_>>()?;
    fill! {
        buildings => building,
        building_parts => building_part,
        sites => site,
        tin_surfaces => tin_surface,
        tin_breaklines => tin_breakline,
        land_covers => land_cover,
        structures => structure,
        grade_separations => grade_separation,
        interchanges => interchange,
    }
    Ok(map)
}

/// Decodes every message of `field` into `arena`, in order, refusing an
/// identifier that comes twice.
fn fill<K: Eq + Hash + Clone + Display, V, M>(
    arena: &mut Arena<K, V>,
    messages: &[M],
    field: &str,
    decode: impl Fn(&M) -> Result<V>,
    key: impl Fn(&V) -> K,
) -> Result<()> {
    for (index, message) in messages.iter().enumerate() {
        let value = decode(message).within(Indexed(field, index))?;
        arena
            .insert(key(&value), value)
            .map_err(|DuplicateId(id)| {
                invalid(format!("{id} is the identifier of an earlier element"))
            })
            .within("id")
            .within(Indexed(field, index))?;
    }
    Ok(())
}

/// A sub-message the schema leaves optional but the IR does not.
fn required<T>(value: &Option<T>) -> Result<&T> {
    value.as_ref().ok_or_else(|| invalid("is not set"))
}

/// A sub-message, decoded, naming the field it is in when either fails.
fn field<T, U>(value: &Option<T>, name: &str, decode: impl FnOnce(&T) -> Result<U>) -> Result<U> {
    required(value).and_then(decode).within(name)
}

/// A sub-message whose absence the schema gives a meaning: the empty message,
/// which is what a writer that never touched the field means by it.
fn or_empty<T: Default, U>(
    value: &Option<T>,
    name: &str,
    decode: impl FnOnce(&T) -> Result<U>,
) -> Result<U> {
    match value {
        Some(value) => decode(value),
        None => decode(&T::default()),
    }
    .within(name)
}

/// An enum field's value. Unknown numbers are refused rather than guessed at.
fn enumerated<E: TryFrom<i32>>(raw: i32, name: &str) -> Result<E> {
    E::try_from(raw)
        .map_err(|_| invalid(format!("{raw} is not one of its values")))
        .within(name)
}

fn unspecified<T>(name: &str) -> Result<T> {
    Err(invalid("is not set")).within(name)
}

/// The identifier types, read exactly as written.
trait Raw {
    fn raw(text: &str) -> Self;
}

macro_rules! raw {
    ($($id:ty),*) => {$(
        impl Raw for $id {
            fn raw(text: &str) -> Self {
                <$id>::from_raw(text)
            }
        }
    )*};
}

raw!(
    BuildingId,
    BuildingPartId,
    ConnectionId,
    GradeSeparationId,
    InterchangeId,
    JunctionId,
    LandCoverId,
    LaneId,
    ObjectId,
    RoadId,
    SiteId,
    StructureId,
    TinBreaklineId,
    TinSurfaceId
);

fn ids<T: Raw>(raw: &[String]) -> Vec<T> {
    raw.iter().map(|id| T::raw(id)).collect()
}

fn metadata(message: &v1::MapMetadata) -> Result<MapMetadata> {
    let origin = field(&message.origin, "origin", |origin| {
        GeoOrigin::new(origin.latitude, origin.longitude, origin.altitude).map_err(invalid)
    })?;
    let sampling = field(&message.sampling, "sampling", |sampling| {
        SamplingConfig::new(sampling.max_segment_length).map_err(invalid)
    })?;
    Ok(MapMetadata {
        name: message.name.clone(),
        origin,
        projection: match enumerated(message.projection, "projection")? {
            v1::Projection::LocalCartesian => Projection::LocalCartesian,
            v1::Projection::Utm => Projection::Utm,
            v1::Projection::Mgrs => Projection::Mgrs,
            v1::Projection::Unspecified => return unspecified("projection"),
        },
        handedness: match enumerated(message.handedness, "handedness")? {
            v1::TrafficHandedness::RightHand => TrafficHandedness::RightHand,
            v1::TrafficHandedness::LeftHand => TrafficHandedness::LeftHand,
            v1::TrafficHandedness::Unspecified => return unspecified("handedness"),
        },
        sampling,
    })
}

// ---------------------------------------------------------------------------------
// Geometry
// ---------------------------------------------------------------------------------

fn point(message: &v1::Point3) -> Point3 {
    Point3::new(message.x, message.y, message.z)
}

/// A point standing on its own, which no constructor checks for it.
fn finite_point(message: &v1::Point3) -> Result<Point3> {
    let at = point(message);
    if at.is_finite() {
        Ok(at)
    } else {
        Err(invalid("is not finite"))
    }
}

fn point_at(message: &Option<v1::Point3>, name: &str) -> Result<Point3> {
    field(message, name, finite_point)
}

fn points(messages: &[v1::Point3]) -> Vec<Point3> {
    messages.iter().map(point).collect()
}

/// A curve. Its `samples` are the writer's convenience for other readers, and
/// are not read: the curve is what it says it is.
fn curve(message: &v1::Curve) -> Result<Curve3> {
    use v1::curve::Kind;
    let geometry = |error: GeometryError| invalid(error);
    match required(&message.kind).within("kind")? {
        Kind::Line(message) => line(message).within("line"),
        Kind::Arc(arc) => Ok(Curve3::Arc(
            Arc3::new(
                point_at(&arc.start, "start").within("arc")?,
                arc.heading,
                arc.curvature,
                arc.horizontal_length,
                arc.end_z,
            )
            .map_err(geometry)
            .within("arc")?,
        )),
        Kind::Clothoid(clothoid) => Ok(Curve3::Clothoid(
            Clothoid3::new(
                point_at(&clothoid.start, "start").within("clothoid")?,
                clothoid.heading,
                clothoid.curvature_start,
                clothoid.curvature_end,
                clothoid.horizontal_length,
                clothoid.end_z,
            )
            .map_err(geometry)
            .within("clothoid")?,
        )),
        Kind::Bezier(bezier) => {
            let control: [Point3; 4] = points(&bezier.control)
                .try_into()
                .map_err(|given: Vec<Point3>| {
                    invalid(format!("has {} control points, not four", given.len()))
                })
                .within("bezier.control")?;
            Ok(Curve3::Bezier(
                Bezier3::with_segments(control, bezier.segments as usize)
                    .map_err(geometry)
                    .within("bezier")?,
            ))
        }
        Kind::Polyline(polyline) => Ok(Curve3::Polyline(
            Polyline3::new(points(&polyline.points))
                .map_err(geometry)
                .within("polyline.points")?,
        )),
        Kind::Composite(composite) => {
            if composite.pieces.is_empty() {
                return Err(invalid("has no pieces")).within("composite.pieces");
            }
            let pieces = composite
                .pieces
                .iter()
                .enumerate()
                .map(|(index, piece)| {
                    curve(piece)
                        .within(Indexed("pieces", index))
                        .within("composite")
                })
                .collect::<Result<Vec<_>>>()?;
            // Checked as `Curve3::composite` checks, but kept as written rather than
            // built through it: that would unwrap a composite of one piece, and the
            // map said composite.
            for (index, pair) in pieces.windows(2).enumerate() {
                let gap = pair[0].end_point().distance_to(pair[1].start_point());
                if gap > Curve3::JOIN_TOLERANCE {
                    return Err(invalid(GeometryError::DisjointSegments { gap }))
                        .within(Indexed("pieces", index + 1))
                        .within("composite");
                }
            }
            Ok(Curve3::Composite(pieces))
        }
    }
}

fn line(message: &v1::Line) -> Result<Curve3> {
    Curve3::line(
        point_at(&message.start, "start")?,
        point_at(&message.end, "end")?,
    )
    .map_err(invalid)
}

fn curve_at(message: &Option<v1::Curve>, name: &str) -> Result<Curve3> {
    field(message, name, curve)
}

fn profile(message: &v1::Poly3Profile) -> Result<Poly3Profile> {
    Poly3Profile::new(
        message
            .pieces
            .iter()
            .map(|piece| Poly3Piece::new(piece.station, piece.a, piece.b, piece.c, piece.d)),
    )
    .map_err(invalid)
    .within("pieces")
}

fn width(message: &v1::WidthProfile) -> Result<WidthProfile> {
    let taper = match enumerated(message.taper, "taper")? {
        v1::Taper::Linear => Taper::Linear,
        v1::Taper::Smooth => Taper::Smooth,
        v1::Taper::Unspecified => return unspecified("taper"),
    };
    let knots = message
        .knots
        .iter()
        .enumerate()
        .map(|(index, knot)| {
            PositiveWidth::new(knot.width)
                .map(|width| (knot.station, width))
                .map_err(invalid)
                .within("width")
                .within(Indexed("knots", index))
        })
        .collect::<Result<Vec<_>>>()?;
    WidthProfile::new(knots, taper)
        .map_err(invalid)
        .within("knots")
}

fn height(message: &v1::LaneHeight) -> Result<LaneHeight> {
    LaneHeight::new(
        message
            .knots
            .iter()
            .map(|knot| (knot.station, knot.inner, knot.outer)),
    )
    .map_err(invalid)
    .within("knots")
}

fn footprint(message: &v1::Footprint) -> Result<Footprint> {
    Footprint::new(points(&message.points))
        .map_err(invalid)
        .within("points")
}

// ---------------------------------------------------------------------------------
// Topology
// ---------------------------------------------------------------------------------

fn direction(raw: i32) -> Result<Direction> {
    Ok(match enumerated(raw, "direction")? {
        v1::Direction::Forward => Direction::Forward,
        v1::Direction::Backward => Direction::Backward,
        v1::Direction::Unspecified => return unspecified("direction"),
    })
}

fn side(raw: i32) -> Result<LateralSide> {
    Ok(match enumerated(raw, "side")? {
        v1::LateralSide::Left => LateralSide::Left,
        v1::LateralSide::Right => LateralSide::Right,
        v1::LateralSide::Unspecified => return unspecified("side"),
    })
}

fn lane_end(raw: i32) -> Result<LaneEnd> {
    Ok(match enumerated(raw, "end")? {
        v1::End::Start => LaneEnd::Start,
        v1::End::End => LaneEnd::End,
        v1::End::Unspecified => return unspecified("end"),
    })
}

fn road_end(raw: i32) -> Result<RoadEnd> {
    lane_end(raw).map(LaneEnd::as_road_end)
}

fn link_target(message: &v1::RoadLinkTarget) -> Result<RoadLinkTarget> {
    use v1::road_link_target::Target;
    Ok(match required(&message.target).within("target")? {
        Target::Road(endpoint) => RoadLinkTarget::Road(RoadEndpoint::new(
            RoadId::from_raw(&endpoint.road),
            road_end(endpoint.end).within("road")?,
        )),
        Target::Junction(junction) => RoadLinkTarget::Junction(JunctionId::from_raw(junction)),
    })
}

fn optional_link(
    message: &Option<v1::RoadLinkTarget>,
    name: &str,
) -> Result<Option<RoadLinkTarget>> {
    message.as_ref().map(link_target).transpose().within(name)
}

fn connection(message: &v1::LaneConnection) -> Result<LaneConnection> {
    let endpoint = |endpoint: &v1::LaneEndpoint| {
        Ok(LaneEndpoint::new(
            LaneId::from_raw(&endpoint.lane),
            lane_end(endpoint.end)?,
        ))
    };
    Ok(LaneConnection {
        id: ConnectionId::from_raw(&message.id),
        from: field(&message.from, "from", endpoint)?,
        to: field(&message.to, "to", endpoint)?,
        junction: message.junction.as_deref().map(JunctionId::from_raw),
    })
}

fn junction(message: &v1::Junction) -> Result<Junction> {
    Ok(Junction {
        id: JunctionId::from_raw(&message.id),
        name: message.name.clone(),
        incoming_roads: ids(&message.incoming_roads),
        connecting_roads: ids(&message.connecting_roads),
        interchange: message.interchange.as_deref().map(InterchangeId::from_raw),
    })
}

// ---------------------------------------------------------------------------------
// Roads and lanes
// ---------------------------------------------------------------------------------

fn speed_limit(mps: Option<f64>, name: &str) -> Result<Option<SpeedLimit>> {
    mps.map(SpeedLimit::from_mps)
        .transpose()
        .map_err(invalid)
        .within(name)
}

fn surface(raw: i32) -> Result<Pavement> {
    Ok(match enumerated(raw, "surface")? {
        v1::Surface::Asphalt => Pavement::Paved(PavedSurface::Asphalt),
        v1::Surface::Concrete => Pavement::Paved(PavedSurface::Concrete),
        v1::Surface::Cobblestone => Pavement::Paved(PavedSurface::Cobblestone),
        v1::Surface::Gravel => Pavement::Unpaved(UnpavedSurface::Gravel),
        v1::Surface::Dirt => Pavement::Unpaved(UnpavedSurface::Dirt),
        v1::Surface::Sand => Pavement::Unpaved(UnpavedSurface::Sand),
        v1::Surface::Rock => Pavement::Unpaved(UnpavedSurface::Rock),
        v1::Surface::Unspecified => return unspecified("surface"),
    })
}

fn road(message: &v1::Road) -> Result<Road> {
    if message.sections.is_empty() {
        return Err(invalid("has none; a road has at least one cross-section")).within("sections");
    }
    let link = or_empty(&message.link, "link", |link| {
        Ok(RoadLink {
            predecessor: optional_link(&link.predecessor, "predecessor")?,
            successor: optional_link(&link.successor, "successor")?,
        })
    })?;
    let pavement = PavementProfile::new(
        message
            .pavement
            .iter()
            .enumerate()
            .map(|(index, span)| {
                Ok((
                    span.station,
                    surface(span.surface).within(Indexed("pavement", index))?,
                ))
            })
            .collect::<Result<Vec<_>>>()?,
    )
    .map_err(invalid)
    .within("pavement")?;
    let mut structure = StructureProfile::at_grade();
    for (index, span) in message.structure.iter().enumerate() {
        structure
            .insert(StructureSpan {
                from: span.from,
                to: span.to,
                structure: StructureId::from_raw(&span.structure),
            })
            .map_err(|_| invalid("overlaps a span before it"))
            .within(Indexed("structure", index))?;
    }
    Ok(Road {
        id: RoadId::from_raw(&message.id),
        name: message.name.clone(),
        reference_line: curve_at(&message.reference_line, "reference_line")?,
        lane_offset: or_empty(&message.lane_offset, "lane_offset", profile)?,
        lanes: ids(&message.lanes),
        sections: message
            .sections
            .iter()
            .map(|section| CrossSection {
                station: section.station,
                lanes: ids(&section.lanes),
            })
            .collect(),
        junction: message.junction.as_deref().map(JunctionId::from_raw),
        link,
        road_type: match enumerated(message.road_type, "road_type")? {
            v1::RoadType::Town => RoadType::Town,
            v1::RoadType::Rural => RoadType::Rural,
            v1::RoadType::Motorway => RoadType::Motorway,
            v1::RoadType::LowSpeed => RoadType::LowSpeed,
            v1::RoadType::Pedestrian => RoadType::Pedestrian,
            v1::RoadType::Unspecified => return unspecified("road_type"),
        },
        speed_limit: speed_limit(message.speed_limit_mps, "speed_limit_mps")?,
        superelevation: or_empty(&message.superelevation, "superelevation", profile)?,
        pavement,
        form: match enumerated(message.form, "form")? {
            v1::FormOfWay::Unspecified => FormOfWay::Unspecified,
            v1::FormOfWay::Motorway => FormOfWay::Motorway,
            v1::FormOfWay::MultipleCarriageway => FormOfWay::MultipleCarriageway,
            v1::FormOfWay::SingleCarriageway => FormOfWay::SingleCarriageway,
            v1::FormOfWay::SlipRoad => FormOfWay::SlipRoad,
            v1::FormOfWay::ParallelRoad => FormOfWay::ParallelRoad,
            v1::FormOfWay::ServiceRoad => FormOfWay::ServiceRoad,
            v1::FormOfWay::Roundabout => FormOfWay::Roundabout,
        },
        structure,
    })
}

fn marking(message: &v1::BoundaryMarking) -> Result<BoundaryMarking> {
    let marking = match enumerated(message.marking, "marking")? {
        v1::RoadMarking::None => RoadMarking::None,
        v1::RoadMarking::Solid => RoadMarking::Solid,
        v1::RoadMarking::Broken => RoadMarking::Broken,
        v1::RoadMarking::SolidSolid => RoadMarking::SolidSolid,
        v1::RoadMarking::BrokenSolid => RoadMarking::BrokenSolid,
        v1::RoadMarking::SolidBroken => RoadMarking::SolidBroken,
        v1::RoadMarking::Curbstone => RoadMarking::Curbstone,
        v1::RoadMarking::Unspecified => return unspecified("marking"),
    };
    let color = match enumerated(message.color, "color")? {
        v1::MarkingColor::White => MarkingColor::White,
        v1::MarkingColor::Yellow => MarkingColor::Yellow,
        v1::MarkingColor::Unspecified => return unspecified("color"),
    };
    Ok(BoundaryMarking::new(marking, color))
}

fn lane(message: &v1::Lane) -> Result<Lane> {
    Ok(Lane {
        id: LaneId::from_raw(&message.id),
        road: RoadId::from_raw(&message.road),
        index: message.index as usize,
        side: side(message.side)?,
        ordinal: message.ordinal as usize,
        direction: direction(message.direction)?,
        lane_type: match enumerated(message.lane_type, "lane_type")? {
            v1::LaneType::Driving => LaneType::Driving,
            v1::LaneType::Shoulder => LaneType::Shoulder,
            v1::LaneType::Border => LaneType::Border,
            v1::LaneType::Sidewalk => LaneType::Sidewalk,
            v1::LaneType::Biking => LaneType::Biking,
            v1::LaneType::Parking => LaneType::Parking,
            v1::LaneType::Restricted => LaneType::Restricted,
            v1::LaneType::None => LaneType::None,
            v1::LaneType::Unspecified => return unspecified("lane_type"),
        },
        width: field(&message.width, "width", width)?,
        height: or_empty(&message.height, "height", height)?,
        speed_limit: speed_limit(message.speed_limit_mps, "speed_limit_mps")?,
        section: message.section as usize,
        station_range: (message.start_station, message.end_station),
        left_edge: message.left_edge,
        right_edge: message.right_edge,
        left_offset: message.left_offset,
        right_offset: message.right_offset,
        left_boundary: curve_at(&message.left_boundary, "left_boundary")?,
        right_boundary: curve_at(&message.right_boundary, "right_boundary")?,
        centerline: curve_at(&message.centerline, "centerline")?,
        left_marking: field(&message.left_marking, "left_marking", marking)?,
        right_marking: field(&message.right_marking, "right_marking", marking)?,
    })
}

// ---------------------------------------------------------------------------------
// Objects and rules
// ---------------------------------------------------------------------------------

fn light_head(message: &v1::LightHead) -> Result<LightHead> {
    let bulbs = message
        .bulbs
        .iter()
        .enumerate()
        .map(|(index, bulb)| {
            Ok(LightBulb {
                position: point_at(&bulb.position, "position")?,
                color: match enumerated(bulb.color, "color")? {
                    v1::LightColor::Red => LightColor::Red,
                    v1::LightColor::Yellow => LightColor::Yellow,
                    v1::LightColor::Green => LightColor::Green,
                    v1::LightColor::Unspecified => return unspecified("color"),
                },
                arrow: match enumerated(bulb.arrow, "arrow")? {
                    v1::LightArrow::None => None,
                    v1::LightArrow::Up => Some(LightArrow::Up),
                    v1::LightArrow::Down => Some(LightArrow::Down),
                    v1::LightArrow::Left => Some(LightArrow::Left),
                    v1::LightArrow::Right => Some(LightArrow::Right),
                    v1::LightArrow::UpLeft => Some(LightArrow::UpLeft),
                    v1::LightArrow::UpRight => Some(LightArrow::UpRight),
                    v1::LightArrow::DownLeft => Some(LightArrow::DownLeft),
                    v1::LightArrow::DownRight => Some(LightArrow::DownRight),
                },
            })
            .within(Indexed("bulbs", index))
        })
        .collect::<Result<_>>()?;
    Ok(LightHead {
        height: message.height,
        bulbs,
    })
}

fn object(message: &v1::MapObject) -> Result<MapObject> {
    use v1::map_object::Kind;
    use v1::object_geometry::Kind as Geometry;
    let kind = match required(&message.kind).within("kind")? {
        Kind::TrafficLight(light) => MapObjectKind::TrafficLight {
            head: field(&light.head, "traffic_light.head", light_head)?,
        },
        Kind::TrafficSign(sign) => MapObjectKind::TrafficSign {
            code: sign.code.clone(),
        },
        Kind::StopLine(_) => MapObjectKind::StopLine,
        Kind::Crosswalk(_) => MapObjectKind::Crosswalk,
        Kind::Barrier(barrier) => MapObjectKind::Barrier {
            kind: barrier.kind.clone(),
            height: barrier.height,
            structure: barrier.structure.as_deref().map(StructureId::from_raw),
        },
    };
    let geometry = field(&message.geometry, "geometry", |geometry| {
        Ok(match required(&geometry.kind).within("kind")? {
            Geometry::Point(at) => ObjectGeometry::Point(finite_point(at).within("point")?),
            Geometry::Line(line) => ObjectGeometry::Line(curve(line).within("line")?),
            Geometry::Band(band) => ObjectGeometry::Band {
                left: curve_at(&band.left, "left").within("band")?,
                right: curve_at(&band.right, "right").within("band")?,
            },
        })
    })?;
    Ok(MapObject {
        id: ObjectId::from_raw(&message.id),
        kind,
        geometry,
        lanes: ids(&message.lanes),
    })
}

fn rule(message: &v1::TrafficRule) -> Result<TrafficRule> {
    use v1::traffic_rule::Kind;
    Ok(match required(&message.kind).within("kind")? {
        Kind::TrafficLight(rule) => TrafficRule::TrafficLight {
            lights: ids(&rule.lights),
            stop_lines: ids(&rule.stop_lines),
            lanes: ids(&rule.lanes),
        },
        Kind::RightOfWay(rule) => TrafficRule::RightOfWay {
            right_of_way: ids(&rule.right_of_way),
            yielding: ids(&rule.yielding),
            stop_line: rule.stop_line.as_deref().map(ObjectId::from_raw),
        },
        Kind::SpeedLimit(rule) => TrafficRule::SpeedLimit {
            limit: SpeedLimit::from_mps(rule.limit_mps)
                .map_err(invalid)
                .within("speed_limit.limit_mps")?,
            lanes: ids(&rule.lanes),
        },
    })
}

// ---------------------------------------------------------------------------------
// Buildings, terrain, land cover and sites
// ---------------------------------------------------------------------------------

fn roof(message: &v1::Roof) -> Result<Roof> {
    let shape = match enumerated(message.shape, "shape")? {
        v1::RoofShape::Flat => RoofShape::Flat,
        v1::RoofShape::Skillion => RoofShape::Skillion,
        v1::RoofShape::Gabled => RoofShape::Gabled,
        v1::RoofShape::Hipped => RoofShape::Hipped,
        v1::RoofShape::Pyramidal => RoofShape::Pyramidal,
        v1::RoofShape::Unspecified => return unspecified("shape"),
    };
    // Through `Roof::new`, which zeroes what the shape does not have: a flat
    // roof's height, a pyramid's direction.
    Ok(Roof::new(shape, message.height, message.direction))
}

fn building(message: &v1::Building) -> Result<Building> {
    let frontage = message
        .frontage
        .as_ref()
        .map(|frontage| {
            Ok::<_, ReadError>(Frontage {
                road: RoadId::from_raw(&frontage.road),
                side: side(frontage.side)?,
                station: frontage.station,
            })
        })
        .transpose()
        .within("frontage")?;
    Ok(Building {
        id: BuildingId::from_raw(&message.id),
        parts: ids(&message.parts),
        kind: message.kind.clone(),
        frontage,
    })
}

fn building_part(message: &v1::BuildingPart) -> Result<BuildingPart> {
    let solid = field(&message.solid, "solid", |solid| {
        Ok(Solid {
            footprint: field(&solid.footprint, "footprint", footprint)?,
            wall_height: solid.wall_height,
            roof: field(&solid.roof, "roof", roof)?,
        })
    })?;
    Ok(BuildingPart {
        id: BuildingPartId::from_raw(&message.id),
        building: BuildingId::from_raw(&message.building),
        solid,
        kind: message.kind.clone(),
        levels: message.levels,
    })
}

fn tin_surface(message: &v1::TinSurface) -> Result<TinSurface> {
    let tin = field(&message.tin, "tin", |tin| {
        if tin.triangles.len() % 3 != 0 {
            return Err(invalid(format!(
                "holds {} indices, which is not three per triangle",
                tin.triangles.len()
            )))
            .within("triangles");
        }
        Tin::new(
            points(&tin.nodes),
            tin.triangles
                .chunks_exact(3)
                .map(|corners| [corners[0], corners[1], corners[2]])
                .collect(),
        )
        .map_err(invalid)
    })?;
    Ok(TinSurface {
        id: TinSurfaceId::from_raw(&message.id),
        site: message.site.as_deref().map(SiteId::from_raw),
        breaklines: ids(&message.breaklines),
        tin,
    })
}

fn tin_breakline(message: &v1::TinBreakline) -> Result<TinBreakline> {
    Ok(TinBreakline {
        id: TinBreaklineId::from_raw(&message.id),
        surface: TinSurfaceId::from_raw(&message.surface),
        kind: message.kind.clone(),
        line: field(&message.line, "line", |line| {
            Polyline3::new(points(&line.points))
                .map_err(invalid)
                .within("points")
        })?,
    })
}

fn land_cover(message: &v1::LandCover) -> Result<LandCover> {
    Ok(LandCover {
        id: LandCoverId::from_raw(&message.id),
        site: message.site.as_deref().map(SiteId::from_raw),
        kind: message.kind.clone(),
        area: field(&message.area, "area", footprint)?,
        draped_on: message.draped_on.as_deref().map(TinSurfaceId::from_raw),
    })
}

fn site(message: &v1::Site) -> Result<Site> {
    Ok(Site {
        id: SiteId::from_raw(&message.id),
        name: message.name.clone(),
        kind: message.kind.clone(),
        land_covers: ids(&message.land_covers),
        surfaces: ids(&message.surfaces),
        roads: ids(&message.roads),
    })
}

// ---------------------------------------------------------------------------------
// Structures
// ---------------------------------------------------------------------------------

fn road_span(message: &v1::RoadSpan) -> RoadSpan {
    RoadSpan::new(RoadId::from_raw(&message.road), message.from, message.to)
}

fn bridge(message: &v1::Bridge) -> Result<Bridge> {
    let supports = message
        .supports
        .iter()
        .enumerate()
        .map(|(index, support)| {
            Ok(Support {
                kind: match enumerated(support.kind, "kind")? {
                    v1::SupportKind::Abutment => SupportKind::Abutment,
                    v1::SupportKind::Pier => SupportKind::Pier,
                    v1::SupportKind::Unspecified => return unspecified("kind"),
                },
                footprint: field(&support.footprint, "footprint", footprint)?,
                base: support.base,
            })
            .within(Indexed("supports", index))
        })
        .collect::<Result<_>>()?;
    Ok(Bridge {
        form: match enumerated(message.form, "form")? {
            v1::BridgeForm::Bridge => BridgeForm::Bridge,
            v1::BridgeForm::Viaduct => BridgeForm::Viaduct,
            v1::BridgeForm::Overpass => BridgeForm::Overpass,
            v1::BridgeForm::Unspecified => return unspecified("form"),
        },
        material: match enumerated(message.material, "material")? {
            v1::StructureMaterial::Concrete => StructureMaterial::Concrete,
            v1::StructureMaterial::Steel => StructureMaterial::Steel,
            v1::StructureMaterial::Masonry => StructureMaterial::Masonry,
            v1::StructureMaterial::Timber => StructureMaterial::Timber,
            v1::StructureMaterial::Unspecified => return unspecified("material"),
        },
        deck_depth: message.deck_depth,
        curb: message.curb,
        supports,
    })
}

fn tunnel(message: &v1::Tunnel) -> Result<Tunnel> {
    let portals: Vec<Portal> = message
        .portals
        .iter()
        .enumerate()
        .map(|(index, portal)| {
            Ok(Portal {
                road: RoadId::from_raw(&portal.road),
                station: portal.station,
                left: point_at(&portal.left, "left")?,
                right: point_at(&portal.right, "right")?,
            })
            .within(Indexed("portals", index))
        })
        .collect::<Result<_>>()?;
    let portals: [Portal; 2] = portals
        .try_into()
        .map_err(|given: Vec<Portal>| invalid(format!("has {} portals, not two", given.len())))
        .within("portals")?;
    Ok(Tunnel {
        form: match enumerated(message.form, "form")? {
            v1::TunnelForm::Bored => TunnelForm::Bored,
            v1::TunnelForm::CutAndCover => TunnelForm::CutAndCover,
            v1::TunnelForm::Underpass => TunnelForm::Underpass,
            v1::TunnelForm::Unspecified => return unspecified("form"),
        },
        clear_height: message.clear_height,
        lining: message.lining,
        lighting: message.lighting,
        walkway: message.walkway,
        portals,
    })
}

fn structure(message: &v1::Structure) -> Result<Structure> {
    use v1::structure::Kind;
    let kind = match required(&message.kind).within("kind")? {
        Kind::Bridge(message) => StructureKind::Bridge(bridge(message).within("bridge")?),
        Kind::Tunnel(message) => StructureKind::Tunnel(tunnel(message).within("tunnel")?),
    };
    Ok(Structure {
        id: StructureId::from_raw(&message.id),
        name: message.name.clone(),
        kind,
        carries: message.carries.iter().map(road_span).collect(),
        outline: field(&message.outline, "outline", footprint)?,
        site: message.site.as_deref().map(SiteId::from_raw),
    })
}

fn grade_separation(message: &v1::GradeSeparation) -> Result<GradeSeparation> {
    use v1::underneath::Kind;
    Ok(GradeSeparation {
        id: GradeSeparationId::from_raw(&message.id),
        over: field(&message.over, "over", |span| Ok(road_span(span)))?,
        under: field(&message.under, "under", |under| {
            Ok(match required(&under.kind).within("kind")? {
                Kind::Road(span) => Underneath::Road(road_span(span)),
                Kind::LandCover(cover) => Underneath::LandCover(LandCoverId::from_raw(cover)),
            })
        })?,
        via: StructureId::from_raw(&message.via),
    })
}

fn interchange(message: &v1::Interchange) -> Result<Interchange> {
    Ok(Interchange {
        id: InterchangeId::from_raw(&message.id),
        name: message.name.clone(),
        kind: message.kind.clone(),
        junctions: ids(&message.junctions),
        roads: ids(&message.roads),
        structures: ids(&message.structures),
    })
}
