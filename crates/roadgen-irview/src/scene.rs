//! The plan view of a map, as plain data.

use roadgen_core::{
    Curve3, Direction, GeometryError, LateralSide, Map, MarkingColor, ObjectGeometry, Point3,
    RoadMarking, SamplingConfig,
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
    fn an_empty_map_draws_nothing() {
        let scene = Scene::from_map(&Map::new(MapMetadata::default()));
        assert!(scene.bounds.is_none());
        assert!(scene.lanes.is_empty());
    }
}
