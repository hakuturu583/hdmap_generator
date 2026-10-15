//! The IR, as messages.
//!
//! A table, field for field: nothing is derived here except a curve's `samples`,
//! which the schema marks as informative and the reader ignores.

use roadgen_core::buildings::{Building, BuildingPart, Footprint, Frontage, Roof, RoofShape};
use roadgen_core::geometry::{
    Curve3, LaneHeight, Point3, Poly3Profile, SamplingConfig, Taper, Vector3, WidthProfile,
};
use roadgen_core::land_cover::LandCover;
use roadgen_core::map::{Lane, Map, MapMetadata, Projection, Road, TrafficHandedness};
use roadgen_core::semantics::{
    BoundaryMarking, FormOfWay, LaneType, LightArrow, LightColor, LightHead, MapObject,
    MapObjectKind, MarkingColor, ObjectGeometry, PavedSurface, Pavement, RoadMarking, RoadType,
    TrafficRule, UnpavedSurface,
};
use roadgen_core::site::Site;
use roadgen_core::structure::{
    BridgeForm, GradeSeparation, Interchange, RoadSpan, Structure, StructureKind,
    StructureMaterial, SupportKind, TunnelForm, Underneath,
};
use roadgen_core::terrain::{TinBreakline, TinSurface};
use roadgen_core::topology::{
    Direction, Junction, LaneConnection, LaneEnd, LateralSide, RoadEnd, RoadLinkTarget,
};

use crate::error::{unwritable, Indexed, Within};
use crate::v1;
use crate::{WriteError, WriteOptions};

type Result<T> = std::result::Result<T, WriteError>;

/// What the writer carries down into every curve.
struct Writer {
    /// The resolution to sample curves at, when they are to carry samples.
    samples: Option<SamplingConfig>,
}

pub(crate) fn map(map: &Map, options: &WriteOptions) -> Result<v1::Map> {
    let writer = Writer {
        samples: options.curve_samples.then_some(map.metadata.sampling),
    };
    Ok(v1::Map {
        generator: concat!("roadgen ", env!("CARGO_PKG_VERSION")).to_owned(),
        metadata: Some(metadata(&map.metadata)),
        roads: each(map.roads.iter(), "roads", |road| writer.road(road))?,
        lanes: each(map.lanes.iter(), "lanes", |lane| writer.lane(lane))?,
        junctions: map.junctions.iter().map(junction).collect(),
        connections: map.connections.iter().map(connection).collect(),
        objects: each(map.objects.iter(), "objects", |object| {
            writer.object(object)
        })?,
        rules: map.rules.iter().map(rule).collect(),
        buildings: map.buildings.iter().map(building).collect(),
        building_parts: map.building_parts.iter().map(building_part).collect(),
        sites: map.sites.iter().map(site).collect(),
        tin_surfaces: map.tin_surfaces.iter().map(tin_surface).collect(),
        tin_breaklines: map.tin_breaklines.iter().map(tin_breakline).collect(),
        land_covers: map.land_covers.iter().map(land_cover).collect(),
        structures: map.structures.iter().map(structure).collect(),
        grade_separations: map.grade_separations.iter().map(grade_separation).collect(),
        interchanges: map.interchanges.iter().map(interchange).collect(),
    })
}

/// Encodes every item, naming the one that failed by its place in `field`.
fn each<'a, T: 'a, U>(
    items: impl Iterator<Item = &'a T>,
    field: &str,
    mut encode: impl FnMut(&'a T) -> Result<U>,
) -> Result<Vec<U>> {
    items
        .enumerate()
        .map(|(index, item)| encode(item).within(Indexed(field, index)))
        .collect()
}

fn count(value: usize) -> Result<u32> {
    u32::try_from(value).map_err(|_| unwritable(format!("{value} does not fit in 32 bits")))
}

fn metadata(metadata: &MapMetadata) -> v1::MapMetadata {
    v1::MapMetadata {
        name: metadata.name.clone(),
        origin: Some(v1::GeoOrigin {
            latitude: metadata.origin.latitude(),
            longitude: metadata.origin.longitude(),
            altitude: metadata.origin.altitude(),
        }),
        projection: match metadata.projection {
            Projection::LocalCartesian => v1::Projection::LocalCartesian,
            Projection::Utm => v1::Projection::Utm,
            Projection::Mgrs => v1::Projection::Mgrs,
        } as i32,
        handedness: match metadata.handedness {
            TrafficHandedness::RightHand => v1::TrafficHandedness::RightHand,
            TrafficHandedness::LeftHand => v1::TrafficHandedness::LeftHand,
        } as i32,
        sampling: Some(v1::SamplingConfig {
            max_segment_length: metadata.sampling.max_segment_length,
        }),
    }
}

// ---------------------------------------------------------------------------------
// Geometry
// ---------------------------------------------------------------------------------

fn point(point: Point3) -> v1::Point3 {
    v1::Point3 {
        x: point.x,
        y: point.y,
        z: point.z,
    }
}

fn vector(vector: Vector3) -> v1::Vector3 {
    v1::Vector3 {
        x: vector.x,
        y: vector.y,
        z: vector.z,
    }
}

fn points(points: &[Point3]) -> Vec<v1::Point3> {
    points.iter().copied().map(point).collect()
}

impl Writer {
    /// A curve, carrying its samples when the writer was asked for them.
    fn curve(&self, curve: &Curve3) -> Result<v1::Curve> {
        let samples = match self.samples {
            Some(config) => curve
                .samples(config)
                .map_err(|error| unwritable(format!("could not sample the curve: {error}")))?
                .into_iter()
                .map(|sample| v1::CurveSample {
                    station: sample.station,
                    point: Some(point(sample.point)),
                    tangent: Some(vector(sample.tangent.get())),
                })
                .collect(),
            None => Vec::new(),
        };
        Ok(v1::Curve {
            kind: Some(curve_kind(curve)?),
            samples,
        })
    }
}

/// The curve itself, without samples: what the pieces of a composite are.
fn curve_kind(curve: &Curve3) -> Result<v1::curve::Kind> {
    use v1::curve::Kind;
    Ok(match curve {
        Curve3::Line(line) => Kind::Line(v1::Line {
            start: Some(point(line.start())),
            end: Some(point(line.end())),
        }),
        Curve3::Arc(arc) => Kind::Arc(v1::Arc {
            start: Some(point(arc.start())),
            heading: arc.heading(),
            curvature: arc.curvature(),
            horizontal_length: arc.horizontal_length(),
            end_z: arc.end_z(),
        }),
        Curve3::Clothoid(clothoid) => Kind::Clothoid(v1::Clothoid {
            start: Some(point(clothoid.start())),
            heading: clothoid.heading(),
            curvature_start: clothoid.curvature_start(),
            curvature_end: clothoid.curvature_end(),
            horizontal_length: clothoid.horizontal_length(),
            end_z: clothoid.end_z(),
        }),
        Curve3::Bezier(bezier) => Kind::Bezier(v1::Bezier {
            control: points(bezier.control()),
            segments: count(bezier.segments()).within("bezier.segments")?,
        }),
        Curve3::Polyline(polyline) => Kind::Polyline(v1::Polyline {
            points: points(polyline.points()),
        }),
        Curve3::Composite(pieces) => Kind::Composite(v1::Composite {
            pieces: pieces
                .iter()
                .enumerate()
                .map(|(index, piece)| {
                    Ok(v1::Curve {
                        kind: Some(curve_kind(piece)?),
                        samples: Vec::new(),
                    })
                    .within(Indexed("pieces", index))
                    .within("composite")
                })
                .collect::<Result<_>>()?,
        }),
    })
}

fn profile(profile: &Poly3Profile) -> v1::Poly3Profile {
    v1::Poly3Profile {
        pieces: profile
            .pieces()
            .iter()
            .map(|piece| v1::Poly3Piece {
                station: piece.station,
                a: piece.a,
                b: piece.b,
                c: piece.c,
                d: piece.d,
            })
            .collect(),
    }
}

fn width(width: &WidthProfile) -> v1::WidthProfile {
    v1::WidthProfile {
        knots: width
            .knots()
            .iter()
            .map(|(station, width)| v1::WidthKnot {
                station: *station,
                width: width.metres(),
            })
            .collect(),
        taper: match width.taper() {
            Taper::Linear => v1::Taper::Linear,
            Taper::Smooth => v1::Taper::Smooth,
        } as i32,
    }
}

fn height(height: &LaneHeight) -> v1::LaneHeight {
    v1::LaneHeight {
        knots: height
            .knots()
            .iter()
            .map(|&(station, inner, outer)| v1::LaneHeightKnot {
                station,
                inner,
                outer,
            })
            .collect(),
    }
}

fn footprint(footprint: &Footprint) -> v1::Footprint {
    v1::Footprint {
        points: points(footprint.points()),
    }
}

// ---------------------------------------------------------------------------------
// Topology
// ---------------------------------------------------------------------------------

fn direction(direction: Direction) -> i32 {
    i32::from(match direction {
        Direction::Forward => v1::Direction::Forward,
        Direction::Backward => v1::Direction::Backward,
    })
}

fn side(side: LateralSide) -> i32 {
    i32::from(match side {
        LateralSide::Left => v1::LateralSide::Left,
        LateralSide::Right => v1::LateralSide::Right,
    })
}

fn lane_end(end: LaneEnd) -> i32 {
    i32::from(match end {
        LaneEnd::Start => v1::End::Start,
        LaneEnd::End => v1::End::End,
    })
}

fn road_end(end: RoadEnd) -> i32 {
    lane_end(end.as_lane_end())
}

fn link_target(target: &RoadLinkTarget) -> v1::RoadLinkTarget {
    use v1::road_link_target::Target;
    v1::RoadLinkTarget {
        target: Some(match target {
            RoadLinkTarget::Road(endpoint) => Target::Road(v1::RoadEndpoint {
                road: endpoint.road.to_string(),
                end: road_end(endpoint.end),
            }),
            RoadLinkTarget::Junction(junction) => Target::Junction(junction.to_string()),
        }),
    }
}

fn connection(connection: &LaneConnection) -> v1::LaneConnection {
    let endpoint = |endpoint: &roadgen_core::topology::LaneEndpoint| v1::LaneEndpoint {
        lane: endpoint.lane.to_string(),
        end: lane_end(endpoint.end),
    };
    v1::LaneConnection {
        id: connection.id.to_string(),
        from: Some(endpoint(&connection.from)),
        to: Some(endpoint(&connection.to)),
        junction: connection.junction.as_ref().map(ToString::to_string),
    }
}

fn junction(junction: &Junction) -> v1::Junction {
    v1::Junction {
        id: junction.id.to_string(),
        name: junction.name.clone(),
        incoming_roads: strings(&junction.incoming_roads),
        connecting_roads: strings(&junction.connecting_roads),
        interchange: junction.interchange.as_ref().map(ToString::to_string),
    }
}

fn strings<T: ToString>(ids: &[T]) -> Vec<String> {
    ids.iter().map(ToString::to_string).collect()
}

// ---------------------------------------------------------------------------------
// Roads and lanes
// ---------------------------------------------------------------------------------

fn road_type(road_type: RoadType) -> i32 {
    i32::from(match road_type {
        RoadType::Town => v1::RoadType::Town,
        RoadType::Rural => v1::RoadType::Rural,
        RoadType::Motorway => v1::RoadType::Motorway,
        RoadType::LowSpeed => v1::RoadType::LowSpeed,
        RoadType::Pedestrian => v1::RoadType::Pedestrian,
    })
}

fn form(form: FormOfWay) -> i32 {
    i32::from(match form {
        FormOfWay::Unspecified => v1::FormOfWay::Unspecified,
        FormOfWay::Motorway => v1::FormOfWay::Motorway,
        FormOfWay::MultipleCarriageway => v1::FormOfWay::MultipleCarriageway,
        FormOfWay::SingleCarriageway => v1::FormOfWay::SingleCarriageway,
        FormOfWay::SlipRoad => v1::FormOfWay::SlipRoad,
        FormOfWay::ParallelRoad => v1::FormOfWay::ParallelRoad,
        FormOfWay::ServiceRoad => v1::FormOfWay::ServiceRoad,
        FormOfWay::Roundabout => v1::FormOfWay::Roundabout,
    })
}

fn surface(pavement: Pavement) -> i32 {
    i32::from(match pavement {
        Pavement::Paved(PavedSurface::Asphalt) => v1::Surface::Asphalt,
        Pavement::Paved(PavedSurface::Concrete) => v1::Surface::Concrete,
        Pavement::Paved(PavedSurface::Cobblestone) => v1::Surface::Cobblestone,
        Pavement::Unpaved(UnpavedSurface::Gravel) => v1::Surface::Gravel,
        Pavement::Unpaved(UnpavedSurface::Dirt) => v1::Surface::Dirt,
        Pavement::Unpaved(UnpavedSurface::Sand) => v1::Surface::Sand,
        Pavement::Unpaved(UnpavedSurface::Rock) => v1::Surface::Rock,
    })
}

fn lane_type(lane_type: LaneType) -> i32 {
    i32::from(match lane_type {
        LaneType::Driving => v1::LaneType::Driving,
        LaneType::Shoulder => v1::LaneType::Shoulder,
        LaneType::Border => v1::LaneType::Border,
        LaneType::Sidewalk => v1::LaneType::Sidewalk,
        LaneType::Biking => v1::LaneType::Biking,
        LaneType::Parking => v1::LaneType::Parking,
        LaneType::Restricted => v1::LaneType::Restricted,
        LaneType::None => v1::LaneType::None,
    })
}

fn marking(marking: BoundaryMarking) -> v1::BoundaryMarking {
    v1::BoundaryMarking {
        marking: match marking.marking {
            RoadMarking::None => v1::RoadMarking::None,
            RoadMarking::Solid => v1::RoadMarking::Solid,
            RoadMarking::Broken => v1::RoadMarking::Broken,
            RoadMarking::SolidSolid => v1::RoadMarking::SolidSolid,
            RoadMarking::BrokenSolid => v1::RoadMarking::BrokenSolid,
            RoadMarking::SolidBroken => v1::RoadMarking::SolidBroken,
            RoadMarking::Curbstone => v1::RoadMarking::Curbstone,
        } as i32,
        color: match marking.color {
            MarkingColor::White => v1::MarkingColor::White,
            MarkingColor::Yellow => v1::MarkingColor::Yellow,
        } as i32,
    }
}

impl Writer {
    fn road(&self, road: &Road) -> Result<v1::Road> {
        Ok(v1::Road {
            id: road.id.to_string(),
            name: road.name.clone(),
            reference_line: Some(self.curve(&road.reference_line).within("reference_line")?),
            lane_offset: Some(profile(&road.lane_offset)),
            lanes: strings(&road.lanes),
            sections: road
                .sections
                .iter()
                .map(|section| v1::CrossSection {
                    station: section.station,
                    lanes: strings(&section.lanes),
                })
                .collect(),
            junction: road.junction.as_ref().map(ToString::to_string),
            link: Some(v1::RoadLink {
                predecessor: road.link.predecessor.as_ref().map(link_target),
                successor: road.link.successor.as_ref().map(link_target),
            }),
            road_type: road_type(road.road_type),
            speed_limit_mps: road.speed_limit.map(|limit| limit.mps()),
            superelevation: Some(profile(&road.superelevation)),
            pavement: road
                .pavement
                .spans()
                .iter()
                .map(|&(station, pavement)| v1::PavementSpan {
                    station,
                    surface: surface(pavement),
                })
                .collect(),
            form: form(road.form),
            structure: road
                .structure
                .spans()
                .iter()
                .map(|span| v1::StructureSpan {
                    from: span.from,
                    to: span.to,
                    structure: span.structure.to_string(),
                })
                .collect(),
        })
    }

    fn lane(&self, lane: &Lane) -> Result<v1::Lane> {
        Ok(v1::Lane {
            id: lane.id.to_string(),
            road: lane.road.to_string(),
            index: count(lane.index).within("index")?,
            side: side(lane.side),
            ordinal: count(lane.ordinal).within("ordinal")?,
            direction: direction(lane.direction),
            lane_type: lane_type(lane.lane_type),
            width: Some(width(&lane.width)),
            height: Some(height(&lane.height)),
            speed_limit_mps: lane.speed_limit.map(|limit| limit.mps()),
            section: count(lane.section).within("section")?,
            start_station: lane.station_range.0,
            end_station: lane.station_range.1,
            left_edge: lane.left_edge,
            right_edge: lane.right_edge,
            left_offset: lane.left_offset,
            right_offset: lane.right_offset,
            left_boundary: Some(self.curve(&lane.left_boundary).within("left_boundary")?),
            right_boundary: Some(self.curve(&lane.right_boundary).within("right_boundary")?),
            centerline: Some(self.curve(&lane.centerline).within("centerline")?),
            left_marking: Some(marking(lane.left_marking)),
            right_marking: Some(marking(lane.right_marking)),
        })
    }
}

// ---------------------------------------------------------------------------------
// Objects and rules
// ---------------------------------------------------------------------------------

fn light_color(color: LightColor) -> i32 {
    i32::from(match color {
        LightColor::Red => v1::LightColor::Red,
        LightColor::Yellow => v1::LightColor::Yellow,
        LightColor::Green => v1::LightColor::Green,
    })
}

fn light_arrow(arrow: Option<LightArrow>) -> i32 {
    i32::from(match arrow {
        None => v1::LightArrow::None,
        Some(LightArrow::Up) => v1::LightArrow::Up,
        Some(LightArrow::Down) => v1::LightArrow::Down,
        Some(LightArrow::Left) => v1::LightArrow::Left,
        Some(LightArrow::Right) => v1::LightArrow::Right,
        Some(LightArrow::UpLeft) => v1::LightArrow::UpLeft,
        Some(LightArrow::UpRight) => v1::LightArrow::UpRight,
        Some(LightArrow::DownLeft) => v1::LightArrow::DownLeft,
        Some(LightArrow::DownRight) => v1::LightArrow::DownRight,
    })
}

fn light_head(head: &LightHead) -> v1::LightHead {
    v1::LightHead {
        height: head.height,
        bulbs: head
            .bulbs
            .iter()
            .map(|bulb| v1::LightBulb {
                position: Some(point(bulb.position)),
                color: light_color(bulb.color),
                arrow: light_arrow(bulb.arrow),
            })
            .collect(),
    }
}

impl Writer {
    fn object(&self, object: &MapObject) -> Result<v1::MapObject> {
        use v1::map_object::Kind;
        use v1::object_geometry::Kind as Geometry;
        let kind = match &object.kind {
            MapObjectKind::TrafficLight { head } => Kind::TrafficLight(v1::TrafficLight {
                head: Some(light_head(head)),
            }),
            MapObjectKind::TrafficSign { code } => {
                Kind::TrafficSign(v1::TrafficSign { code: code.clone() })
            }
            MapObjectKind::StopLine => Kind::StopLine(v1::StopLine {}),
            MapObjectKind::Crosswalk => Kind::Crosswalk(v1::Crosswalk {}),
            MapObjectKind::Barrier {
                kind,
                height,
                structure,
            } => Kind::Barrier(v1::Barrier {
                kind: kind.clone(),
                height: *height,
                structure: structure.as_ref().map(ToString::to_string),
            }),
        };
        let geometry = match &object.geometry {
            ObjectGeometry::Point(at) => Geometry::Point(point(*at)),
            ObjectGeometry::Line(line) => Geometry::Line(self.curve(line).within("geometry.line")?),
            ObjectGeometry::Band { left, right } => Geometry::Band(v1::Band {
                left: Some(self.curve(left).within("geometry.band.left")?),
                right: Some(self.curve(right).within("geometry.band.right")?),
            }),
        };
        Ok(v1::MapObject {
            id: object.id.to_string(),
            kind: Some(kind),
            geometry: Some(v1::ObjectGeometry {
                kind: Some(geometry),
            }),
            lanes: strings(&object.lanes),
        })
    }
}

fn rule(rule: &TrafficRule) -> v1::TrafficRule {
    use v1::traffic_rule::Kind;
    v1::TrafficRule {
        kind: Some(match rule {
            TrafficRule::TrafficLight {
                lights,
                stop_lines,
                lanes,
            } => Kind::TrafficLight(v1::TrafficLightRule {
                lights: strings(lights),
                stop_lines: strings(stop_lines),
                lanes: strings(lanes),
            }),
            TrafficRule::RightOfWay {
                right_of_way,
                yielding,
                stop_line,
            } => Kind::RightOfWay(v1::RightOfWayRule {
                right_of_way: strings(right_of_way),
                yielding: strings(yielding),
                stop_line: stop_line.as_ref().map(ToString::to_string),
            }),
            TrafficRule::SpeedLimit { limit, lanes } => Kind::SpeedLimit(v1::SpeedLimitRule {
                limit_mps: limit.mps(),
                lanes: strings(lanes),
            }),
        }),
    }
}

// ---------------------------------------------------------------------------------
// Buildings, terrain, land cover and sites
// ---------------------------------------------------------------------------------

fn roof(roof: &Roof) -> v1::Roof {
    v1::Roof {
        shape: match roof.shape {
            RoofShape::Flat => v1::RoofShape::Flat,
            RoofShape::Skillion => v1::RoofShape::Skillion,
            RoofShape::Gabled => v1::RoofShape::Gabled,
            RoofShape::Hipped => v1::RoofShape::Hipped,
            RoofShape::Pyramidal => v1::RoofShape::Pyramidal,
        } as i32,
        height: roof.height,
        direction: roof.direction,
    }
}

fn frontage(frontage: &Frontage) -> v1::Frontage {
    v1::Frontage {
        road: frontage.road.to_string(),
        side: side(frontage.side),
        station: frontage.station,
    }
}

fn building(building: &Building) -> v1::Building {
    v1::Building {
        id: building.id.to_string(),
        parts: strings(&building.parts),
        kind: building.kind.clone(),
        frontage: building.frontage.as_ref().map(frontage),
    }
}

fn building_part(part: &BuildingPart) -> v1::BuildingPart {
    v1::BuildingPart {
        id: part.id.to_string(),
        building: part.building.to_string(),
        solid: Some(v1::Solid {
            footprint: Some(footprint(&part.solid.footprint)),
            wall_height: part.solid.wall_height,
            roof: Some(roof(&part.solid.roof)),
        }),
        kind: part.kind.clone(),
        levels: part.levels,
    }
}

fn tin_surface(surface: &TinSurface) -> v1::TinSurface {
    v1::TinSurface {
        id: surface.id.to_string(),
        site: surface.site.as_ref().map(ToString::to_string),
        breaklines: strings(&surface.breaklines),
        tin: Some(v1::Tin {
            nodes: points(surface.tin.nodes()),
            triangles: surface.tin.triangles().iter().flatten().copied().collect(),
        }),
    }
}

fn tin_breakline(breakline: &TinBreakline) -> v1::TinBreakline {
    v1::TinBreakline {
        id: breakline.id.to_string(),
        surface: breakline.surface.to_string(),
        kind: breakline.kind.clone(),
        line: Some(v1::Polyline {
            points: points(breakline.line.points()),
        }),
    }
}

fn land_cover(cover: &LandCover) -> v1::LandCover {
    v1::LandCover {
        id: cover.id.to_string(),
        site: cover.site.as_ref().map(ToString::to_string),
        kind: cover.kind.clone(),
        area: Some(footprint(&cover.area)),
        draped_on: cover.draped_on.as_ref().map(ToString::to_string),
    }
}

fn site(site: &Site) -> v1::Site {
    v1::Site {
        id: site.id.to_string(),
        name: site.name.clone(),
        kind: site.kind.clone(),
        land_covers: strings(&site.land_covers),
        surfaces: strings(&site.surfaces),
        roads: strings(&site.roads),
    }
}

// ---------------------------------------------------------------------------------
// Structures
// ---------------------------------------------------------------------------------

fn road_span(span: &RoadSpan) -> v1::RoadSpan {
    v1::RoadSpan {
        road: span.road.to_string(),
        from: span.from,
        to: span.to,
    }
}

fn structure(structure: &Structure) -> v1::Structure {
    use v1::structure::Kind;
    let kind = match &structure.kind {
        StructureKind::Bridge(bridge) => Kind::Bridge(v1::Bridge {
            form: match bridge.form {
                BridgeForm::Bridge => v1::BridgeForm::Bridge,
                BridgeForm::Viaduct => v1::BridgeForm::Viaduct,
                BridgeForm::Overpass => v1::BridgeForm::Overpass,
            } as i32,
            material: match bridge.material {
                StructureMaterial::Concrete => v1::StructureMaterial::Concrete,
                StructureMaterial::Steel => v1::StructureMaterial::Steel,
                StructureMaterial::Masonry => v1::StructureMaterial::Masonry,
                StructureMaterial::Timber => v1::StructureMaterial::Timber,
            } as i32,
            deck_depth: bridge.deck_depth,
            curb: bridge.curb,
            supports: bridge
                .supports
                .iter()
                .map(|support| v1::Support {
                    kind: match support.kind {
                        SupportKind::Abutment => v1::SupportKind::Abutment,
                        SupportKind::Pier => v1::SupportKind::Pier,
                    } as i32,
                    footprint: Some(footprint(&support.footprint)),
                    base: support.base,
                })
                .collect(),
        }),
        StructureKind::Tunnel(tunnel) => Kind::Tunnel(v1::Tunnel {
            form: match tunnel.form {
                TunnelForm::Bored => v1::TunnelForm::Bored,
                TunnelForm::CutAndCover => v1::TunnelForm::CutAndCover,
                TunnelForm::Underpass => v1::TunnelForm::Underpass,
            } as i32,
            clear_height: tunnel.clear_height,
            lining: tunnel.lining,
            lighting: tunnel.lighting,
            walkway: tunnel.walkway,
            portals: tunnel
                .portals
                .iter()
                .map(|portal| v1::Portal {
                    road: portal.road.to_string(),
                    station: portal.station,
                    left: Some(point(portal.left)),
                    right: Some(point(portal.right)),
                })
                .collect(),
        }),
    };
    v1::Structure {
        id: structure.id.to_string(),
        name: structure.name.clone(),
        kind: Some(kind),
        carries: structure.carries.iter().map(road_span).collect(),
        outline: Some(footprint(&structure.outline)),
        site: structure.site.as_ref().map(ToString::to_string),
    }
}

fn grade_separation(separation: &GradeSeparation) -> v1::GradeSeparation {
    use v1::underneath::Kind;
    v1::GradeSeparation {
        id: separation.id.to_string(),
        over: Some(road_span(&separation.over)),
        under: Some(v1::Underneath {
            kind: Some(match &separation.under {
                Underneath::Road(span) => Kind::Road(road_span(span)),
                Underneath::LandCover(cover) => Kind::LandCover(cover.to_string()),
            }),
        }),
        via: separation.via.to_string(),
    }
}

fn interchange(interchange: &Interchange) -> v1::Interchange {
    v1::Interchange {
        id: interchange.id.to_string(),
        name: interchange.name.clone(),
        kind: interchange.kind.clone(),
        junctions: strings(&interchange.junctions),
        roads: strings(&interchange.roads),
        structures: strings(&interchange.structures),
    }
}
