//! The plan view of a map, as plain data.

use roadgen_core::{
    Curve3, Direction, GeometryError, LateralSide, Map, MarkingColor, ObjectGeometry, Point3, Road,
    RoadMarking, SamplingConfig, StructureKind, SurfaceFeature, SurfaceFeatureKind,
};

/// A point in plan: x east, y north, metres.
pub type Xy = [f64; 2];

/// The axis-aligned box a scene occupies.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize))]
pub struct Bounds {
    pub min_x: f64,
    pub min_y: f64,
    pub max_x: f64,
    pub max_y: f64,
}

impl Bounds {
    fn of(points: impl IntoIterator<Item = Xy>) -> Option<Bounds> {
        let mut bounds: Option<Bounds> = None;
        for [x, y] in points {
            if !x.is_finite() || !y.is_finite() {
                continue;
            }
            bounds = Some(match bounds {
                None => Bounds {
                    min_x: x,
                    min_y: y,
                    max_x: x,
                    max_y: y,
                },
                Some(b) => Bounds {
                    min_x: b.min_x.min(x),
                    min_y: b.min_y.min(y),
                    max_x: b.max_x.max(x),
                    max_y: b.max_y.max(y),
                },
            });
        }
        bounds
    }

    pub fn width(&self) -> f64 {
        self.max_x - self.min_x
    }

    pub fn height(&self) -> f64 {
        self.max_y - self.min_y
    }
}

/// One end of a road's reference line.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize))]
pub struct EndView {
    /// Where the end is, with its height kept: an editor snapping one road onto
    /// another wants the z too.
    pub point: [f64; 3],
    /// Direction of travel along the reference line there, radians from east.
    pub heading: f64,
}

/// A road: its reference line and its two ends.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize))]
pub struct RoadView {
    /// The IR identifier, `road/<key>`.
    pub id: String,
    pub name: Option<String>,
    /// The junction the road carries traffic through; `None` for an ordinary road.
    pub junction: Option<String>,
    pub road_type: String,
    /// Plan length of the reference line, metres.
    pub length: f64,
    pub reference_line: Vec<Xy>,
    pub start: EndView,
    pub end: EndView,
    /// How many lanes the road has, over all its cross-sections.
    pub lane_count: usize,
}

/// One lane, as a closed outline.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize))]
pub struct LaneView {
    /// The IR identifier, `lane/<road key>/<index>`.
    pub id: String,
    pub road: String,
    /// Index of the cross-section the lane belongs to.
    pub section: usize,
    /// Position in the road's lane list, as the caller wrote it.
    pub index: usize,
    pub lane_type: String,
    /// `forward` or `backward`, against the reference line.
    pub direction: String,
    /// `left` or `right` of the reference line.
    pub side: String,
    pub drivable: bool,
    /// Whether the lane is on a road through a junction.
    pub connector: bool,
    /// Width at the lane's start, metres.
    pub width: f64,
    /// Left boundary followed by the right boundary reversed: a ring, not closed.
    pub outline: Vec<Xy>,
    /// The centreline in the order traffic drives it.
    pub centerline: Vec<Xy>,
}

/// A painted line or a kerb along a cross-section edge.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize))]
pub struct LineView {
    pub road: String,
    /// The IR's word for the marking: `solid`, `broken`, `solid solid`, `curb`…
    pub marking: String,
    /// `white` or `yellow`.
    pub color: String,
    pub points: Vec<Xy>,
}

/// A piece of road furniture.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize))]
pub struct ObjectView {
    pub id: String,
    /// `traffic_light`, `traffic_sign`, `stop_line`, `crosswalk` or `barrier`.
    pub kind: String,
    /// One point, a line, or — when `closed` — a ring.
    pub points: Vec<Xy>,
    pub closed: bool,
    /// The lanes the object governs.
    pub lanes: Vec<String>,
}

/// A junction, drawn as the hull of the connectors through it.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize))]
pub struct JunctionView {
    pub id: String,
    pub name: Option<String>,
    pub incoming_roads: Vec<String>,
    pub connecting_roads: Vec<String>,
    /// Convex hull of every connector lane; empty when the junction has none.
    pub outline: Vec<Xy>,
}

/// Something with an outline beside the road: a building part or a land cover.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize))]
pub struct AreaView {
    pub id: String,
    /// `building` or `land_cover`.
    pub category: String,
    /// The IR's own word for it: `house`, `pit`…
    pub kind: String,
    pub outline: Vec<Xy>,
}

/// A departure of a road's surface from its design: a rut, a pothole, a puddle, a
/// bump or a patch of roughness, drawn where it lies on the road.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize))]
pub struct SurfaceView {
    /// The IR identifier, `surface/<road>/<kind>/<n>`.
    pub id: String,
    pub road: String,
    /// `rut`, `pothole`, `puddle`, `bump` or `roughness`.
    pub kind: String,
    /// The feature in a few words: `pothole 8 cm deep`, `hump 7.5 cm, circular`.
    pub label: String,
    /// The footprint in the road's own coordinates: station range, and lateral
    /// range right edge first, metres to the left of the reference line.
    pub stations: [f64; 2],
    pub laterals: [f64; 2],
    /// The deepest below the design and the highest above it, metres.
    pub extremes: [f64; 2],
    /// Whether water stands on it.
    pub wet: bool,
    /// What the feature covers in plan: the ellipse of a pothole or a puddle, the
    /// rectangle of a bump or a rough patch, the footprint a rut runs in.
    pub outline: Vec<Xy>,
    /// For a rut, the middle of each trough; empty otherwise.
    pub troughs: Vec<Vec<Xy>>,
}

/// One stretch of road a structure carries.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize))]
pub struct SpanView {
    pub road: String,
    pub from: f64,
    pub to: f64,
}

/// A bridge or a tunnel, drawn as its outline in plan.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize))]
pub struct StructureView {
    /// The IR identifier, `structure/<…>`.
    pub id: String,
    /// `bridge` or `tunnel`.
    pub kind: String,
    /// `bridge`, `viaduct`, `overpass`, `bored`, `cut_and_cover` or `underpass`.
    pub form: String,
    pub name: Option<String>,
    /// The structure in a few words: `viaduct, steel, 120 m`.
    pub label: String,
    /// What the structure covers in plan: a bridge's deck, a tunnel's bore.
    pub outline: Vec<Xy>,
    pub carries: Vec<SpanView>,
    /// A bridge's supports — abutments at the ends, piers between — each as its
    /// footprint; empty for a tunnel.
    pub supports: Vec<Vec<Xy>>,
    /// A tunnel's two portals, each as the line across its mouth; empty for a
    /// bridge.
    pub portals: Vec<[Xy; 2]>,
}

/// One surface of the land: its triangles, by index into its nodes.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize))]
pub struct TerrainView {
    pub id: String,
    /// x, y and height, metres.
    pub nodes: Vec<[f64; 3]>,
    pub triangles: Vec<[u32; 3]>,
    /// The lowest and the highest node.
    pub heights: [f64; 2],
}

/// The land's contour lines at one height: segments, not joined into lines.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize))]
pub struct ContourView {
    pub height: f64,
    /// Every fifth line, drawn heavier.
    pub major: bool,
    pub segments: Vec<[Xy; 2]>,
}

/// Everything in a map that can be drawn in plan.
#[derive(Debug, Clone, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize))]
pub struct Scene {
    pub bounds: Option<Bounds>,
    pub roads: Vec<RoadView>,
    pub lanes: Vec<LaneView>,
    pub lines: Vec<LineView>,
    pub junctions: Vec<JunctionView>,
    pub objects: Vec<ObjectView>,
    pub areas: Vec<AreaView>,
    pub surfaces: Vec<SurfaceView>,
    pub structures: Vec<StructureView>,
    /// The land, where the map states any. It is left out of [`Scene::bounds`]: it
    /// reaches well past the roads, which are what the scene is about.
    pub terrain: Vec<TerrainView>,
    /// The land's contour lines, every [`Scene::contour_interval`] metres.
    pub contours: Vec<ContourView>,
    pub contour_interval: f64,
    /// What could not be drawn and why, one entry each. A scene is drawn from maps
    /// that failed validation too, so a broken element is skipped, not fatal.
    pub warnings: Vec<String>,
}

impl Scene {
    /// Draws a map, sampled as finely as the map itself asks.
    pub fn from_map(map: &Map) -> Scene {
        Scene::from_map_with(map, map.metadata.sampling)
    }

    /// Draws a map, sampling curves at `sampling` — coarser than the map's own for a
    /// large map on a small screen.
    pub fn from_map_with(map: &Map, sampling: SamplingConfig) -> Scene {
        let mut scene = Scene::default();
        let mut warnings = Vec::new();
        let warn = |what: &str, error: GeometryError| format!("{what}: {error}");

        for road in map.roads.iter() {
            let id = road.id.as_str().to_owned();
            let drawn = (|| -> Result<RoadView, GeometryError> {
                let reference_line = plan(&road.reference_line, sampling)?;
                Ok(RoadView {
                    id: id.clone(),
                    name: road.name.clone(),
                    junction: road.junction.as_ref().map(|j| j.as_str().to_owned()),
                    road_type: road.road_type.as_str().to_owned(),
                    length: road.horizontal_length()?,
                    reference_line,
                    start: EndView {
                        point: road.reference_line.start_point().as_array(),
                        heading: heading(road.reference_line.start_tangent()?.get()),
                    },
                    end: EndView {
                        point: road.reference_line.end_point().as_array(),
                        heading: heading(road.reference_line.end_tangent()?.get()),
                    },
                    lane_count: road.lanes.len(),
                })
            })();
            match drawn {
                Ok(view) => scene.roads.push(view),
                Err(error) => warnings.push(warn(&id, error)),
            }

            for (section, _) in road.sections.iter().enumerate() {
                for edge in map.section_edges(&road.id, section).values() {
                    let Some((curve, marking)) = edge.boundary() else {
                        continue;
                    };
                    if marking.marking == RoadMarking::None {
                        continue;
                    }
                    match plan(curve, sampling) {
                        Ok(points) => scene.lines.push(LineView {
                            road: id.clone(),
                            marking: marking.marking.as_str().to_owned(),
                            color: match marking.color {
                                MarkingColor::White => "white",
                                MarkingColor::Yellow => "yellow",
                            }
                            .to_owned(),
                            points,
                        }),
                        Err(error) => warnings.push(warn(&format!("{id} marking"), error)),
                    }
                }
            }
        }

        for lane in map.lanes.iter() {
            let id = lane.id.as_str().to_owned();
            let drawn = (|| -> Result<LaneView, GeometryError> {
                let mut outline = plan(&lane.left_boundary, sampling)?;
                let mut right = plan(&lane.right_boundary, sampling)?;
                right.reverse();
                outline.extend(right);
                let mut centerline = plan(&lane.centerline, sampling)?;
                if lane.direction == Direction::Backward {
                    centerline.reverse();
                }
                Ok(LaneView {
                    id: id.clone(),
                    road: lane.road.as_str().to_owned(),
                    section: lane.section,
                    index: lane.index,
                    lane_type: lane.lane_type.as_str().to_owned(),
                    direction: match lane.direction {
                        Direction::Forward => "forward",
                        Direction::Backward => "backward",
                    }
                    .to_owned(),
                    side: match lane.side {
                        LateralSide::Left => "left",
                        LateralSide::Right => "right",
                    }
                    .to_owned(),
                    drivable: lane.lane_type.is_drivable(),
                    connector: map.is_connector_lane(&lane.id),
                    width: lane.width_at(lane.station_range.0),
                    outline,
                    centerline,
                })
            })();
            match drawn {
                Ok(view) => scene.lanes.push(view),
                Err(error) => warnings.push(warn(&id, error)),
            }
        }

        for junction in map.junctions.iter() {
            let connectors: Vec<&str> = junction
                .connecting_roads
                .iter()
                .map(|road| road.as_str())
                .collect();
            let outline = convex_hull(
                scene
                    .lanes
                    .iter()
                    .filter(|lane| connectors.contains(&lane.road.as_str()))
                    .flat_map(|lane| lane.outline.iter().copied())
                    .collect(),
            );
            scene.junctions.push(JunctionView {
                id: junction.id.as_str().to_owned(),
                name: junction.name.clone(),
                incoming_roads: junction
                    .incoming_roads
                    .iter()
                    .map(|road| road.as_str().to_owned())
                    .collect(),
                connecting_roads: connectors.iter().map(|road| (*road).to_owned()).collect(),
                outline,
            });
        }

        for object in map.objects.iter() {
            let id = object.id.as_str().to_owned();
            let drawn = match &object.geometry {
                ObjectGeometry::Point(point) => Ok((vec![xy(*point)], false)),
                ObjectGeometry::Line(curve) => plan(curve, sampling).map(|points| (points, false)),
                ObjectGeometry::Band { left, right } => (|| {
                    let mut ring = plan(left, sampling)?;
                    let mut back = plan(right, sampling)?;
                    back.reverse();
                    ring.extend(back);
                    Ok((ring, true))
                })(),
            };
            match drawn {
                Ok((points, closed)) => scene.objects.push(ObjectView {
                    id,
                    kind: object.kind.as_str().to_owned(),
                    points,
                    closed,
                    lanes: object
                        .lanes
                        .iter()
                        .map(|lane| lane.as_str().to_owned())
                        .collect(),
                }),
                Err(error) => warnings.push(warn(&id, error)),
            }
        }

        for part in map.building_parts.iter() {
            let kind = part.kind.clone().unwrap_or_else(|| {
                map.building(&part.building)
                    .map(|building| building.kind.clone())
                    .unwrap_or_default()
            });
            scene.areas.push(AreaView {
                id: part.id.as_str().to_owned(),
                category: "building".to_owned(),
                kind,
                outline: part
                    .solid
                    .footprint
                    .points()
                    .iter()
                    .map(|p| xy(*p))
                    .collect(),
            });
        }
        for cover in map.land_covers.iter() {
            scene.areas.push(AreaView {
                id: cover.id.as_str().to_owned(),
                category: "land_cover".to_owned(),
                kind: cover.kind.clone(),
                outline: cover.area.points().iter().map(|p| xy(*p)).collect(),
            });
        }

        for feature in map.surface_features.iter() {
            let Some(road) = map.road(&feature.road) else {
                warnings.push(format!(
                    "{}: on {}, which the map does not have",
                    feature.id, feature.road
                ));
                continue;
            };
            match surface_view(map, road, feature, sampling) {
                Ok(view) => scene.surfaces.push(view),
                Err(error) => warnings.push(warn(feature.id.as_str(), error)),
            }
        }

        for structure in map.structures.iter() {
            let length: f64 = structure.carries.iter().map(|span| span.length()).sum();
            let (form, label, supports, portals) = match &structure.kind {
                StructureKind::Bridge(bridge) => (
                    bridge.form.as_str(),
                    format!(
                        "{}, {}, {:.0} m",
                        bridge.form.as_str(),
                        bridge.material.as_str(),
                        length
                    ),
                    bridge
                        .supports
                        .iter()
                        .map(|support| support.footprint.points().iter().map(|p| xy(*p)).collect())
                        .collect(),
                    Vec::new(),
                ),
                StructureKind::Tunnel(tunnel) => (
                    tunnel.form.as_str(),
                    format!(
                        "tunnel, {}, {:.0} m, {:.1} m clear",
                        tunnel.form.as_str().replace('_', " "),
                        length,
                        tunnel.clear_height
                    ),
                    Vec::new(),
                    tunnel
                        .portals
                        .iter()
                        .map(|portal| [xy(portal.left), xy(portal.right)])
                        .collect(),
                ),
            };
            scene.structures.push(StructureView {
                id: structure.id.as_str().to_owned(),
                kind: structure.kind.as_str().to_owned(),
                form: form.to_owned(),
                name: structure.name.clone(),
                label: match &structure.name {
                    Some(name) => format!("{name}: {label}"),
                    None => label,
                },
                outline: structure.outline.points().iter().map(|p| xy(*p)).collect(),
                carries: structure
                    .carries
                    .iter()
                    .map(|span| SpanView {
                        road: span.road.as_str().to_owned(),
                        from: span.from,
                        to: span.to,
                    })
                    .collect(),
                supports,
                portals,
            });
        }

        for surface in map.tin_surfaces.iter() {
            let nodes: Vec<[f64; 3]> = surface.tin.nodes().iter().map(|p| p.as_array()).collect();
            if nodes.is_empty() {
                continue;
            }
            let (low, high) = nodes
                .iter()
                .fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), p| {
                    (lo.min(p[2]), hi.max(p[2]))
                });
            scene.terrain.push(TerrainView {
                id: surface.id.as_str().to_owned(),
                nodes,
                triangles: surface.tin.triangles().to_vec(),
                heights: [low, high],
            });
        }
        let (contours, interval) = contours(&scene.terrain);
        scene.contours = contours;
        scene.contour_interval = interval;

        scene.warnings = warnings;
        scene.bounds = Bounds::of(
            scene
                .lanes
                .iter()
                .flat_map(|lane| lane.outline.iter().copied())
                .chain(
                    scene
                        .roads
                        .iter()
                        .flat_map(|road| road.reference_line.iter().copied()),
                )
                .chain(scene.objects.iter().flat_map(|o| o.points.iter().copied()))
                .chain(scene.areas.iter().flat_map(|a| a.outline.iter().copied())),
        );
        scene
    }

    /// The lane whose outline contains `point`, preferring a lane through a junction
    /// over the road it overlaps — it is the smaller target, and drawn on top.
    pub fn lane_at(&self, point: Xy) -> Option<&LaneView> {
        let mut found: Option<&LaneView> = None;
        for lane in &self.lanes {
            if contains(&lane.outline, point)
                && found.is_none_or(|current| lane.connector && !current.connector)
            {
                found = Some(lane);
            }
        }
        found
    }
}

/// Where a road's `(station, lateral)` is in plan: the lateral measured along the
/// horizontal left of the reference line, the way a surface feature's footprint
/// is stated.
fn road_point(
    road: &Road,
    sampling: SamplingConfig,
    station: f64,
    lateral: f64,
) -> Result<Xy, GeometryError> {
    let sample = road.reference_line.sample_at(station, sampling)?;
    let tangent = sample.tangent.get();
    let norm = tangent.x.hypot(tangent.y);
    if norm < 1e-12 {
        return Ok(xy(sample.point));
    }
    Ok([
        sample.point.x - tangent.y / norm * lateral,
        sample.point.y + tangent.x / norm * lateral,
    ])
}

/// Stations from `from` to `to`, both included, no further apart than a metre.
fn stations_along(from: f64, to: f64) -> Vec<f64> {
    let steps = ((to - from) / 1.0).ceil().max(1.0) as usize;
    (0..=steps)
        .map(|i| from + (to - from) * i as f64 / steps as f64)
        .collect()
}

fn surface_view(
    map: &Map,
    road: &Road,
    feature: &SurfaceFeature,
    sampling: SamplingConfig,
) -> Result<SurfaceView, GeometryError> {
    let (s0, s1) = feature.stations();
    let (t0, t1) = feature.laterals();
    let at = |s: f64, t: f64| road_point(road, sampling, s, t);
    let (low, high) = feature.extremes();
    let cm = |metres: f64| format!("{:.1} cm", metres * 100.0).replace(".0 cm", " cm");

    let mut troughs = Vec::new();
    let (label, outline) = match &feature.kind {
        SurfaceFeatureKind::Pothole { depth, .. } | SurfaceFeatureKind::Puddle { depth } => {
            // The ellipse inscribed in the footprint, which is what both fill.
            let (sc, tc) = ((s0 + s1) / 2.0, (t0 + t1) / 2.0);
            let (rs, rt) = ((s1 - s0) / 2.0, (t1 - t0) / 2.0);
            let ring = (0..32)
                .map(|i| {
                    let a = i as f64 / 32.0 * std::f64::consts::TAU;
                    at(sc + rs * a.cos(), tc + rt * a.sin())
                })
                .collect::<Result<Vec<_>, _>>()?;
            (
                format!("{} {} deep", feature.kind.as_str(), cm(*depth)),
                ring,
            )
        }
        kind => {
            let along = stations_along(s0, s1);
            let mut ring = along
                .iter()
                .map(|&s| at(s, t0))
                .collect::<Result<Vec<_>, _>>()?;
            for &s in along.iter().rev() {
                ring.push(at(s, t1)?);
            }
            let label = match kind {
                SurfaceFeatureKind::Rut {
                    depth, track, path, ..
                } => {
                    // The middle of each trough: either side of the footprint's
                    // middle, or of the path the rut was worn along.
                    let centre = |s: f64| -> f64 {
                        match path {
                            Some(path) => map
                                .lane_centre(&path.lane)
                                .map(|lane| lane(s) + path.offset_at(s).0)
                                .unwrap_or((t0 + t1) / 2.0),
                            None => (t0 + t1) / 2.0,
                        }
                    };
                    for side in [-0.5, 0.5] {
                        troughs.push(
                            along
                                .iter()
                                .map(|&s| at(s, centre(s) + side * track))
                                .collect::<Result<Vec<_>, _>>()?,
                        );
                    }
                    let along_path = if path.is_some() { ", along a path" } else { "" };
                    format!("rut {} deep{along_path}", cm(*depth))
                }
                SurfaceFeatureKind::Bump {
                    kind,
                    height,
                    profile,
                } => format!("{} {}, {}", kind.as_str(), cm(*height), profile.as_str()),
                SurfaceFeatureKind::Roughness { class, .. } => {
                    format!("roughness, ISO 8608 class {}", class.as_str())
                }
                _ => unreachable!("potholes and puddles are drawn above"),
            };
            (label, ring)
        }
    };
    Ok(SurfaceView {
        id: feature.id.as_str().to_owned(),
        road: feature.road.as_str().to_owned(),
        kind: feature.kind.as_str().to_owned(),
        label,
        stations: [s0, s1],
        laterals: [t0, t1],
        extremes: [low, high],
        wet: feature.holds_water(),
        outline,
        troughs,
    })
}

/// A contour interval for a height range: the first of 0.5, 1, 2, 5, 10, 20, 50
/// and 100 m that gives no more than thirty lines.
pub fn contour_interval(low: f64, high: f64) -> f64 {
    const STEPS: [f64; 8] = [0.5, 1.0, 2.0, 5.0, 10.0, 20.0, 50.0, 100.0];
    let range = (high - low).max(0.0);
    STEPS
        .into_iter()
        .find(|step| range / step <= 30.0)
        .unwrap_or(100.0)
}

/// The contour lines of every surface, at one interval for all of them: where
/// each level crosses each triangle, a segment.
fn contours(terrain: &[TerrainView]) -> (Vec<ContourView>, f64) {
    let (low, high) = terrain
        .iter()
        .fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), t| {
            (lo.min(t.heights[0]), hi.max(t.heights[1]))
        });
    if !(low.is_finite() && high.is_finite()) || high - low < 1e-6 {
        return (Vec::new(), 0.0);
    }
    let interval = contour_interval(low, high);
    let first = (low / interval).ceil() as i64;
    let last = (high / interval).floor() as i64;
    let mut levels: Vec<ContourView> = (first..=last)
        .map(|n| ContourView {
            height: n as f64 * interval,
            major: n % 5 == 0,
            segments: Vec::new(),
        })
        .collect();
    for surface in terrain {
        for triangle in &surface.triangles {
            let corners = triangle.map(|i| surface.nodes[i as usize]);
            let (lo, hi) = corners
                .iter()
                .fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), p| {
                    (lo.min(p[2]), hi.max(p[2]))
                });
            for level in levels.iter_mut() {
                let z = level.height;
                if z < lo || z > hi || hi - lo < 1e-9 {
                    continue;
                }
                // Where the level crosses each edge whose ends straddle it; a
                // corner exactly on the level counts on the side above.
                let mut cut: Vec<Xy> = Vec::with_capacity(2);
                for (a, b) in [(0, 1), (1, 2), (2, 0)] {
                    let (p, q) = (corners[a], corners[b]);
                    if (p[2] >= z) != (q[2] >= z) {
                        let t = (z - p[2]) / (q[2] - p[2]);
                        cut.push([p[0] + (q[0] - p[0]) * t, p[1] + (q[1] - p[1]) * t]);
                    }
                }
                if let [a, b] = cut[..] {
                    level.segments.push([a, b]);
                }
            }
        }
    }
    levels.retain(|level| !level.segments.is_empty());
    (levels, interval)
}

fn xy(point: Point3) -> Xy {
    [point.x, point.y]
}

fn heading(direction: roadgen_core::Vector3) -> f64 {
    direction.y.atan2(direction.x)
}

/// A curve's vertices in plan.
fn plan(curve: &Curve3, sampling: SamplingConfig) -> Result<Vec<Xy>, GeometryError> {
    Ok(curve
        .to_polyline(sampling)?
        .points()
        .iter()
        .map(|p| xy(*p))
        .collect())
}

/// Even-odd point-in-polygon.
fn contains(ring: &[Xy], [x, y]: Xy) -> bool {
    let mut inside = false;
    let mut previous = match ring.last() {
        Some(point) => *point,
        None => return false,
    };
    for &current in ring {
        let ([xi, yi], [xj, yj]) = (current, previous);
        if (yi > y) != (yj > y) && x < (xj - xi) * (y - yi) / (yj - yi) + xi {
            inside = !inside;
        }
        previous = current;
    }
    inside
}

/// Andrew's monotone chain, counter-clockwise, without the closing point.
fn convex_hull(mut points: Vec<Xy>) -> Vec<Xy> {
    points.retain(|[x, y]| x.is_finite() && y.is_finite());
    points.sort_by(|a, b| a[0].total_cmp(&b[0]).then(a[1].total_cmp(&b[1])));
    points.dedup();
    if points.len() < 3 {
        return points;
    }
    fn chain<'a>(points: impl Iterator<Item = &'a Xy>) -> Vec<Xy> {
        let cross =
            |o: Xy, a: Xy, b: Xy| (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]);
        let mut chain: Vec<Xy> = Vec::new();
        for &point in points {
            while chain.len() >= 2
                && cross(chain[chain.len() - 2], chain[chain.len() - 1], point) <= 0.0
            {
                chain.pop();
            }
            chain.push(point);
        }
        // The last point of each chain is the first of the other.
        chain.pop();
        chain
    }
    let mut hull = chain(points.iter());
    hull.extend(chain(points.iter().rev()));
    hull
}

#[cfg(test)]
mod tests {
    use super::*;
    use roadgen_core::prelude::*;

    fn two_lanes() -> Vec<LaneSpec> {
        vec![
            LaneSpec::new(PositiveWidth::new(3.5).unwrap(), Direction::Forward),
            LaneSpec::new(PositiveWidth::new(3.5).unwrap(), Direction::Backward),
        ]
    }

    fn crossroads() -> Map {
        let mut builder = MapBuilder::default();
        let arm = |builder: &mut MapBuilder, name: &str, from: (f64, f64), to: (f64, f64)| {
            builder
                .add_road(
                    RoadSpec::line(
                        Point3::new(from.0, from.1, 0.0),
                        Point3::new(to.0, to.1, 0.0),
                        two_lanes(),
                    )
                    .unwrap()
                    .with_name(name),
                )
                .unwrap()
        };
        let west = arm(&mut builder, "west", (-60.0, 0.0), (-10.0, 0.0));
        let east = arm(&mut builder, "east", (10.0, 0.0), (60.0, 0.0));
        let south = arm(&mut builder, "south", (0.0, -60.0), (0.0, -10.0));
        let junction = builder.add_junction(Some("x"));
        builder
            .connect_ends(&west, RoadEnd::End, &east, RoadEnd::Start, Some(&junction))
            .unwrap();
        builder
            .connect_ends(&west, RoadEnd::End, &south, RoadEnd::End, Some(&junction))
            .unwrap();
        builder.finish().unwrap().into_map()
    }

    #[test]
    fn every_lane_and_road_is_drawn() {
        let map = crossroads();
        let scene = Scene::from_map(&map);
        assert!(scene.warnings.is_empty(), "{:?}", scene.warnings);
        assert_eq!(scene.roads.len(), map.roads.len());
        assert_eq!(scene.lanes.len(), map.lanes.len());
        for lane in &scene.lanes {
            assert!(
                lane.outline.len() >= 4,
                "{} has {:?}",
                lane.id,
                lane.outline
            );
            assert!(lane.width > 3.0);
        }
        let bounds = scene.bounds.expect("a map with roads has bounds");
        assert!(bounds.min_x <= -60.0 && bounds.max_x >= 60.0);
        assert!(bounds.min_y <= -60.0);
    }

    #[test]
    fn a_junction_is_the_hull_of_its_connectors() {
        let scene = Scene::from_map(&crossroads());
        assert_eq!(scene.junctions.len(), 1);
        let junction = &scene.junctions[0];
        assert!(!junction.connecting_roads.is_empty());
        assert!(junction.outline.len() >= 3);
        // The middle of the crossing is inside the hull, the arms' far ends are not.
        assert!(contains(&junction.outline, [0.0, 0.0]));
        assert!(!contains(&junction.outline, [-50.0, 0.0]));
        assert!(scene.lanes.iter().any(|lane| lane.connector));
    }

    #[test]
    fn centrelines_run_the_way_traffic_does() {
        let scene = Scene::from_map(&crossroads());
        let west: Vec<&LaneView> = scene
            .lanes
            .iter()
            .filter(|lane| lane.road == "road/west")
            .collect();
        assert_eq!(west.len(), 2);
        for lane in west {
            let first = lane.centerline.first().unwrap();
            let last = lane.centerline.last().unwrap();
            match lane.direction.as_str() {
                "forward" => assert!(last[0] > first[0]),
                _ => assert!(last[0] < first[0]),
            }
            // Right-hand traffic: forward lanes are south of an eastbound road.
            let mid_y =
                lane.centerline.iter().map(|p| p[1]).sum::<f64>() / lane.centerline.len() as f64;
            match lane.direction.as_str() {
                "forward" => assert!(mid_y < 0.0, "{}", lane.id),
                _ => assert!(mid_y > 0.0, "{}", lane.id),
            }
        }
    }

    #[test]
    fn a_point_picks_the_lane_under_it() {
        let scene = Scene::from_map(&crossroads());
        let lane = scene.lane_at([-30.0, -1.75]).expect("a lane at -30, -1.75");
        assert_eq!(lane.road, "road/west");
        assert_eq!(lane.direction, "forward");
        assert!(scene.lane_at([-30.0, 40.0]).is_none());
        let middle = scene
            .lane_at([0.0, -1.75])
            .expect("something in the junction");
        assert!(middle.connector);
    }

    #[test]
    fn markings_follow_the_section_edges() {
        let scene = Scene::from_map(&crossroads());
        // Two lanes either side of a reference line: three edges per road arm.
        let west = scene.lines.iter().filter(|l| l.road == "road/west").count();
        assert_eq!(west, 3);
        assert!(scene.lines.iter().all(|line| line.points.len() >= 2));
    }

    #[test]
    fn hull_of_a_square_is_the_square() {
        let hull = convex_hull(vec![
            [0.0, 0.0],
            [1.0, 0.0],
            [0.5, 0.5],
            [1.0, 1.0],
            [0.0, 1.0],
        ]);
        assert_eq!(hull.len(), 4);
        assert!(!hull.contains(&[0.5, 0.5]));
    }

    #[test]
    fn surface_features_lie_where_the_road_says() {
        let mut builder = MapBuilder::default();
        let road = builder
            .add_road(
                RoadSpec::line(
                    Point3::new(0.0, 0.0, 0.0),
                    Point3::new(100.0, 0.0, 0.0),
                    two_lanes(),
                )
                .unwrap()
                .with_name("r"),
            )
            .unwrap();
        builder
            .add_surface_feature(
                &road,
                40.0,
                2.0,
                -1.75,
                1.0,
                SurfaceFeatureKind::Pothole {
                    depth: 0.08,
                    seed: 1,
                },
            )
            .unwrap();
        builder
            .add_surface_feature(
                &road,
                10.0,
                30.0,
                -1.75,
                2.2,
                SurfaceFeatureKind::Rut {
                    depth: 0.02,
                    track: 1.8,
                    trough: 0.4,
                    path: None,
                },
            )
            .unwrap();
        let map = builder.finish().unwrap().into_map();
        let scene = Scene::from_map(&map);
        assert!(scene.warnings.is_empty(), "{:?}", scene.warnings);
        assert_eq!(scene.surfaces.len(), 2);

        let pothole = scene.surfaces.iter().find(|s| s.kind == "pothole").unwrap();
        assert_eq!(pothole.label, "pothole 8 cm deep");
        assert_eq!(pothole.extremes, [-0.08, 0.0]);
        // An eastbound road: station is x, and the lateral is y.
        for [x, y] in &pothole.outline {
            assert!(
                (39.0..=42.0).contains(x) && (-2.3..=-1.2).contains(y),
                "{x}, {y}"
            );
        }
        assert!(contains(&pothole.outline, [41.0, -1.75]));

        let rut = scene.surfaces.iter().find(|s| s.kind == "rut").unwrap();
        assert_eq!(rut.troughs.len(), 2);
        let ys: Vec<f64> = rut.troughs.iter().map(|t| t[0][1]).collect();
        assert!(
            (ys[0] - (-2.65)).abs() < 1e-6 && (ys[1] - (-0.85)).abs() < 1e-6,
            "{ys:?}"
        );
        assert!(scene
            .to_svg(&Default::default())
            .contains("pothole 8 cm deep"));
    }

    #[test]
    fn a_bridge_and_a_tunnel_are_drawn_over_their_roads() {
        let mut builder = MapBuilder::default();
        let road = builder
            .add_road(
                RoadSpec::line(
                    Point3::new(0.0, 0.0, 0.0),
                    Point3::new(300.0, 0.0, 0.0),
                    two_lanes(),
                )
                .unwrap()
                .with_name("r"),
            )
            .unwrap();
        builder
            .add_bridge(
                vec![roadgen_core::RoadSpan::new(road.clone(), 20.0, 140.0)],
                roadgen_core::BridgeSpec::for_form(roadgen_core::BridgeForm::Viaduct),
            )
            .unwrap();
        builder
            .add_tunnel(
                vec![roadgen_core::RoadSpan::new(road, 180.0, 260.0)],
                roadgen_core::TunnelSpec::default(),
            )
            .unwrap();
        let map = builder.finish().unwrap().into_map();
        let scene = Scene::from_map(&map);
        assert_eq!(scene.structures.len(), 2);
        let bridge = scene
            .structures
            .iter()
            .find(|s| s.kind == "bridge")
            .unwrap();
        assert_eq!(bridge.form, "viaduct");
        assert!(
            bridge.label.starts_with("viaduct, concrete, 120 m"),
            "{}",
            bridge.label
        );
        assert!(contains(&bridge.outline, [80.0, 0.0]));
        assert!(!contains(&bridge.outline, [160.0, 0.0]));
        // Abutments at both ends and piers no further apart than 40 m.
        assert!(bridge.supports.len() >= 4, "{}", bridge.supports.len());
        let tunnel = scene
            .structures
            .iter()
            .find(|s| s.kind == "tunnel")
            .unwrap();
        assert_eq!(tunnel.portals.len(), 2);
        assert!((tunnel.portals[0][0][0] - 180.0).abs() < 1e-6);
        assert!(contains(&tunnel.outline, [220.0, 0.0]));
        assert_eq!(tunnel.carries[0].road, "road/r");
    }

    #[test]
    fn the_land_is_drawn_with_its_contours() {
        // A pyramid 10 m high on a 40 m square: four triangles round its top.
        let nodes = vec![
            Point3::new(-20.0, -20.0, 0.0),
            Point3::new(20.0, -20.0, 0.0),
            Point3::new(20.0, 20.0, 0.0),
            Point3::new(-20.0, 20.0, 0.0),
            Point3::new(0.0, 0.0, 10.0),
        ];
        let triangles = vec![[0, 1, 4], [1, 2, 4], [2, 3, 4], [3, 0, 4]];
        let mut map = Map::new(MapMetadata::default());
        let id = roadgen_core::TinSurfaceId::new("ground");
        map.tin_surfaces
            .insert(
                id.clone(),
                roadgen_core::TinSurface {
                    id,
                    site: None,
                    breaklines: Vec::new(),
                    tin: roadgen_core::Tin::new(nodes, triangles).unwrap(),
                },
            )
            .unwrap();
        let scene = Scene::from_map(&map);
        assert_eq!(scene.terrain.len(), 1);
        assert_eq!(scene.terrain[0].heights, [0.0, 10.0]);
        assert_eq!(scene.contour_interval, 0.5);
        // The land is not what the view is fitted to.
        assert!(scene.bounds.is_none());
        // The 5 m contour is a square half the base's size: four segments, one per
        // face, each of whose ends is 10 m out from the middle along one axis.
        let five = scene.contours.iter().find(|c| c.height == 5.0).unwrap();
        assert!(five.major);
        assert_eq!(five.segments.len(), 4);
        for segment in &five.segments {
            for [x, y] in segment {
                assert!((x.abs().max(y.abs()) - 10.0).abs() < 1e-9, "{x}, {y}");
            }
        }
        assert!(scene
            .contours
            .iter()
            .all(|c| c.height > 0.0 && c.height <= 10.0));
    }

    #[test]
    fn contour_intervals_keep_the_lines_few() {
        assert_eq!(contour_interval(0.0, 10.0), 0.5);
        assert_eq!(contour_interval(0.0, 25.0), 1.0);
        assert_eq!(contour_interval(100.0, 400.0), 10.0);
    }

    #[test]
    fn an_empty_map_draws_nothing() {
        let scene = Scene::from_map(&Map::new(MapMetadata::default()));
        assert!(scene.bounds.is_none());
        assert!(scene.lanes.is_empty());
    }
}
