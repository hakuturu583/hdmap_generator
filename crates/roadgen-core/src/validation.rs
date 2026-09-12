//! The gate between a generated map and an exporter.
//!
//! A map exists in one of two states and the type says which: an [`UnvalidatedMap`]
//! is whatever the builder produced, and a [`ValidatedMap`] has been through every
//! check below. Exporters take the second, so "did anyone check this?" is not a
//! question they have to ask.
//!
//! Constraints that a type already rules out are not rechecked here: a lane width
//! cannot be negative because [`crate::units::PositiveWidth`] cannot hold a negative
//! number. What is checked is everything that only becomes wrong once objects are
//! put together — references, reciprocity, and geometry meeting up.

use std::collections::HashSet;
use std::ops::Deref;

use crate::error::{ValidationError, ValidationIssue};
use crate::map::{Map, Projection};
use crate::topology::{LaneEnd, RoadEnd, RoadLinkTarget};

/// How far apart two things may be before they count as disconnected.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ValidationConfig {
    /// Metres; applies to every position comparison.
    pub position_tolerance: f64,
}

impl Default for ValidationConfig {
    fn default() -> Self {
        ValidationConfig {
            position_tolerance: 1e-3,
        }
    }
}

/// A map straight out of the builder.
#[derive(Debug, Clone, PartialEq)]
pub struct UnvalidatedMap(Map);

impl UnvalidatedMap {
    pub fn from_map(map: Map) -> Self {
        UnvalidatedMap(map)
    }

    pub fn as_map(&self) -> &Map {
        &self.0
    }

    pub fn as_map_mut(&mut self) -> &mut Map {
        &mut self.0
    }

    /// Unwraps without checking. Useful in tests and when inspecting a map that
    /// failed validation; exporters do not accept the result.
    pub fn into_map(self) -> Map {
        self.0
    }

    /// Everything wrong with the map, in a stable order.
    pub fn issues(&self, config: ValidationConfig) -> Vec<ValidationIssue> {
        let mut issues = Vec::new();
        check_coordinate_metadata(&self.0, &mut issues);
        check_references(&self.0, &mut issues);
        check_cross_sections(&self.0, config, &mut issues);
        check_superelevation(&self.0, &mut issues);
        check_road_links(&self.0, config, &mut issues);
        check_connections(&self.0, config, &mut issues);
        check_junctions(&self.0, &mut issues);
        issues
    }

    pub fn validate(self) -> Result<ValidatedMap, ValidationError> {
        self.validate_with(ValidationConfig::default())
    }

    pub fn validate_with(self, config: ValidationConfig) -> Result<ValidatedMap, ValidationError> {
        let issues = self.issues(config);
        if issues.is_empty() {
            Ok(ValidatedMap(self.0))
        } else {
            Err(ValidationError::new(issues))
        }
    }
}

/// A map that passed [`UnvalidatedMap::validate`].
///
/// The only way to build one is to validate, so holding it is proof.
#[derive(Debug, Clone, PartialEq)]
pub struct ValidatedMap(Map);

impl ValidatedMap {
    pub fn as_map(&self) -> &Map {
        &self.0
    }

    pub fn into_map(self) -> Map {
        self.0
    }
}

impl Deref for ValidatedMap {
    type Target = Map;
    fn deref(&self) -> &Map {
        &self.0
    }
}

fn check_coordinate_metadata(map: &Map, issues: &mut Vec<ValidationIssue>) {
    let origin = map.metadata.origin;
    // Both UTM and MGRS are built on the transverse Mercator grid, which stops at 84
    // degrees; beyond it there is no zone to put the map in.
    if matches!(map.metadata.projection, Projection::Utm | Projection::Mgrs)
        && origin.latitude().abs() > 84.0
    {
        issues.push(ValidationIssue::InvalidCoordinateMetadata {
            detail: format!(
                "{} is undefined beyond 84 degrees of latitude; the origin is at {}",
                map.metadata.projection.as_str(),
                origin.latitude()
            ),
        });
    }
    if map.metadata.sampling.max_segment_length <= 0.0 {
        issues.push(ValidationIssue::InvalidCoordinateMetadata {
            detail: "the sampling step must be a positive number of metres".into(),
        });
    }
}

fn check_references(map: &Map, issues: &mut Vec<ValidationIssue>) {
    let mut seen_lanes: HashSet<&str> = HashSet::new();
    for road in map.roads.iter() {
        for lane_id in &road.lanes {
            if !seen_lanes.insert(lane_id.as_str()) {
                issues.push(ValidationIssue::DuplicateId(lane_id.to_string()));
            }
            match map.lanes.get(lane_id) {
                None => issues.push(ValidationIssue::DanglingLaneReference {
                    referrer: road.id.to_string(),
                    lane: lane_id.clone(),
                }),
                Some(lane) if lane.road != road.id => {
                    issues.push(ValidationIssue::LaneRoadMismatch {
                        lane: lane_id.clone(),
                        road: lane.road.clone(),
                    })
                }
                Some(_) => {}
            }
        }
        if let Some(junction) = &road.junction {
            if !map.junctions.contains(junction) {
                issues.push(ValidationIssue::DanglingJunctionReference {
                    referrer: road.id.to_string(),
                    junction: junction.clone(),
                });
            }
        }
    }

    for lane in map.lanes.iter() {
        match map.roads.get(&lane.road) {
            None => issues.push(ValidationIssue::DanglingRoadReference {
                referrer: lane.id.to_string(),
                road: lane.road.clone(),
            }),
            Some(road) if !road.lanes.contains(&lane.id) => {
                issues.push(ValidationIssue::LaneRoadMismatch {
                    lane: lane.id.clone(),
                    road: lane.road.clone(),
                })
            }
            Some(_) => {}
        }
    }

    for object in map.objects.iter() {
        for lane in &object.lanes {
            if !map.lanes.contains(lane) {
                issues.push(ValidationIssue::DanglingLaneReference {
                    referrer: object.id.to_string(),
                    lane: lane.clone(),
                });
            }
        }
    }

    for rule in &map.rules {
        for lane in rule.lanes() {
            if !map.lanes.contains(&lane) {
                issues.push(ValidationIssue::DanglingLaneReference {
                    referrer: format!("rule {}", rule.kind_str()),
                    lane,
                });
            }
        }
    }
}

fn check_cross_sections(map: &Map, config: ValidationConfig, issues: &mut Vec<ValidationIssue>) {
    for lane in map.lanes.iter() {
        // The width itself needs no check: `WidthProfile` cannot describe one that
        // reaches zero. What is worth checking is that the *generated boundaries*
        // agree with it, which catches a cross-section laid out wrongly.
        if lane.left_edge != lane.right_edge + 1 {
            issues.push(ValidationIssue::InvalidLaneBoundary {
                lane: lane.id.clone(),
                detail: format!(
                    "the lane spans cross-section edges {} and {}, which is not one slot",
                    lane.left_edge, lane.right_edge
                ),
            });
        }
        for (name, station, left, right) in [
            (
                "start",
                lane.station_range.0,
                lane.left_boundary.start_point(),
                lane.right_boundary.start_point(),
            ),
            (
                "end",
                lane.station_range.1,
                lane.left_boundary.end_point(),
                lane.right_boundary.end_point(),
            ),
        ] {
            // Measured *across the cross-section*, not as a straight-line distance.
            // Where two roads meet at an angle the boundary is mitred, so the two
            // points are further apart in space than the lane is wide — and that is
            // the mitre doing its job, not a cross-section laid out wrongly.
            let Some(lateral) = cross_section_direction(map, lane, station) else {
                continue;
            };
            let spanned = (left - right).dot(lateral.get());
            let expected = lane.width_at(station);
            // The cross-section direction at the end of a junction connector between curved
            // arms is itself sampled, so the span misses the width by micrometres there;
            // the position tolerance is the right yardstick, not machine precision.
            if (spanned - expected).abs() > config.position_tolerance {
                issues.push(ValidationIssue::InvalidLaneBoundary {
                    lane: lane.id.clone(),
                    detail: format!(
                        "at its {name} the boundaries span {spanned:.6} m across the \
                         cross-section but the lane is {expected:.6} m wide there"
                    ),
                });
            }
        }
        for (name, curve) in [
            ("left boundary", &lane.left_boundary),
            ("right boundary", &lane.right_boundary),
            ("centerline", &lane.centerline),
        ] {
            match curve.length(map.metadata.sampling) {
                Err(error) => issues.push(ValidationIssue::InvalidLaneBoundary {
                    lane: lane.id.clone(),
                    detail: format!("{name}: {error}"),
                }),
                Ok(length) if length <= 0.0 => issues.push(ValidationIssue::InvalidLaneBoundary {
                    lane: lane.id.clone(),
                    detail: format!("{name} has zero length"),
                }),
                Ok(_) => {}
            }
        }
    }
}

/// Beyond this roll the cross-section stops being a road surface: at a right angle
/// the lateral axis is vertical and a lane has no width in plan at all.
const MAX_SUPERELEVATION: f64 = std::f64::consts::FRAC_PI_4;

fn check_superelevation(map: &Map, issues: &mut Vec<ValidationIssue>) {
    for road in map.roads.iter() {
        if road.superelevation.is_zero() {
            continue;
        }
        // Sampled at the stations the geometry was generated at, which is where the
        // roll was actually applied.
        let Ok(samples) = road.reference_line.samples(map.metadata.sampling) else {
            continue;
        };
        let peak = road
            .superelevation
            .peak_over(samples.iter().map(|sample| sample.station));
        if peak >= MAX_SUPERELEVATION {
            issues.push(ValidationIssue::ImplausibleSuperelevation {
                road: road.id.clone(),
                radians: peak,
            });
        }
    }
}

/// The direction a cross-section is measured along at one station: the road local
/// frame's lateral axis, rolled by whatever superelevation the road carries there.
fn cross_section_direction(
    map: &Map,
    lane: &crate::map::Lane,
    station: f64,
) -> Option<crate::geometry::UnitVector3> {
    let road = map.roads.get(&lane.road)?;
    let sample = road
        .reference_line
        .sample_at(station, map.metadata.sampling)
        .ok()?;
    let frame = sample.frame().ok()?;
    Some(frame.banked(road.superelevation.evaluate(station)).left)
}

fn check_road_links(map: &Map, config: ValidationConfig, issues: &mut Vec<ValidationIssue>) {
    for road in map.roads.iter() {
        for end in [RoadEnd::Start, RoadEnd::End] {
            let Some(target) = road.link.at(end) else {
                continue;
            };
            match target {
                RoadLinkTarget::Junction(junction) => {
                    if !map.junctions.contains(junction) {
                        issues.push(ValidationIssue::DanglingJunctionReference {
                            referrer: road.id.to_string(),
                            junction: junction.clone(),
                        });
                    }
                }
                RoadLinkTarget::Road(other) => {
                    let Some(neighbour) = map.roads.get(&other.road) else {
                        issues.push(ValidationIssue::DanglingRoadReference {
                            referrer: road.id.to_string(),
                            road: other.road.clone(),
                        });
                        continue;
                    };
                    // Reciprocity: the neighbour has to point back at this road,
                    // either directly or through the junction they share.
                    let points_back = match neighbour.link.at(other.end) {
                        Some(RoadLinkTarget::Road(back)) => back.road == road.id && back.end == end,
                        Some(RoadLinkTarget::Junction(_)) => road.is_connector(),
                        None => false,
                    };
                    if !points_back {
                        issues.push(ValidationIssue::RoadEndpointGap {
                            detail: format!(
                                "road {} links its {} end to the {} end of road {}, which does \
                                 not link back",
                                road.id,
                                end.as_str(),
                                other.end.as_str(),
                                other.road
                            ),
                            gap: 0.0,
                        });
                    }
                    // A junction connector's reference line runs down the middle of
                    // its single lane, so it deliberately does *not* meet the
                    // approach's reference line. What has to meet there is the lane
                    // geometry, which `check_connections` compares.
                    if road.is_connector() || neighbour.is_connector() {
                        continue;
                    }
                    let here = road.endpoint(end);
                    let there = neighbour.endpoint(other.end);
                    let gap = here.distance_to(there);
                    if gap > config.position_tolerance {
                        issues.push(ValidationIssue::RoadEndpointGap {
                            detail: format!(
                                "road {} ({}) and road {} ({}) are linked but do not meet",
                                road.id,
                                end.as_str(),
                                other.road,
                                other.end.as_str()
                            ),
                            gap,
                        });
                    }
                }
            }
        }
    }
}

fn check_connections(map: &Map, config: ValidationConfig, issues: &mut Vec<ValidationIssue>) {
    for connection in map.connections.iter() {
        let from = map.lanes.get(&connection.from.lane);
        let to = map.lanes.get(&connection.to.lane);
        if from.is_none() {
            issues.push(ValidationIssue::DanglingLaneReference {
                referrer: connection.id.to_string(),
                lane: connection.from.lane.clone(),
            });
        }
        if to.is_none() {
            issues.push(ValidationIssue::DanglingLaneReference {
                referrer: connection.id.to_string(),
                lane: connection.to.lane.clone(),
            });
        }
        if let Some(junction) = &connection.junction {
            if !map.junctions.contains(junction) {
                issues.push(ValidationIssue::DanglingJunctionReference {
                    referrer: connection.id.to_string(),
                    junction: junction.clone(),
                });
            }
        }
        let (Some(from), Some(to)) = (from, to) else {
            continue;
        };

        // A connection has to leave by the end traffic actually leaves by.
        if connection.from.end != from.direction.exit_end() {
            issues.push(ValidationIssue::InconsistentTravelDirection {
                connection: connection.id.clone(),
                detail: format!(
                    "lane {} runs {} so traffic leaves by its {} end, not its {} end",
                    from.id,
                    from.direction.as_str(),
                    from.direction.exit_end().as_str(),
                    connection.from.end.as_str()
                ),
            });
        }
        if connection.to.end != to.direction.entry_end() {
            issues.push(ValidationIssue::InconsistentTravelDirection {
                connection: connection.id.clone(),
                detail: format!(
                    "lane {} runs {} so traffic enters by its {} end, not its {} end",
                    to.id,
                    to.direction.as_str(),
                    to.direction.entry_end().as_str(),
                    connection.to.end.as_str()
                ),
            });
        }

        let gap = from
            .endpoint(connection.from.end)
            .distance_to(to.endpoint(connection.to.end));
        if gap > config.position_tolerance {
            issues.push(ValidationIssue::ConnectionGap {
                connection: connection.id.clone(),
                gap,
            });
        }

        // The lane boundaries have to meet too, or the lanelet export cannot share
        // the points that make two lanelets continuous.
        let boundary_gap = boundary_gap(map, connection.from.end, from, connection.to.end, to);
        match boundary_gap {
            Err(detail) => issues.push(ValidationIssue::Geometry { detail }),
            Ok(gap) if gap > config.position_tolerance => {
                issues.push(ValidationIssue::GeometryDiscontinuity {
                    detail: format!(
                        "the lane boundaries of connection {} do not meet",
                        connection.id
                    ),
                    gap,
                })
            }
            Ok(_) => {}
        }
    }
}

/// How far apart the paired boundary endpoints of a connection are.
fn boundary_gap(
    map: &Map,
    from_end: LaneEnd,
    from: &crate::map::Lane,
    to_end: LaneEnd,
    to: &crate::map::Lane,
) -> Result<f64, String> {
    let config = map.metadata.sampling;
    let from_travel = from.travel_geometry(config).map_err(|e| e.to_string())?;
    let to_travel = to.travel_geometry(config).map_err(|e| e.to_string())?;
    let leaving = from_end == from.direction.exit_end();
    let entering = to_end == to.direction.entry_end();
    let (from_left, from_right) = if leaving {
        (from_travel.left.end_point(), from_travel.right.end_point())
    } else {
        (
            from_travel.left.start_point(),
            from_travel.right.start_point(),
        )
    };
    let (to_left, to_right) = if entering {
        (to_travel.left.start_point(), to_travel.right.start_point())
    } else {
        (to_travel.left.end_point(), to_travel.right.end_point())
    };
    // Lanes of different widths legitimately meet at a split or a merge; only the
    // narrower overlap has to line up, so compare the gap against the width
    // difference where they actually meet.
    let slack = (from.exit_width() - to.entry_width()).abs();
    let gap = from_left
        .distance_to(to_left)
        .max(from_right.distance_to(to_right));
    Ok((gap - slack).max(0.0))
}

fn check_junctions(map: &Map, issues: &mut Vec<ValidationIssue>) {
    for junction in map.junctions.iter() {
        for road in junction
            .incoming_roads
            .iter()
            .chain(&junction.connecting_roads)
        {
            if !map.roads.contains(road) {
                issues.push(ValidationIssue::DanglingRoadReference {
                    referrer: junction.id.to_string(),
                    road: road.clone(),
                });
            }
        }
        for road in &junction.connecting_roads {
            match map.roads.get(road) {
                Some(connector) if connector.junction.as_ref() != Some(&junction.id) => issues
                    .push(ValidationIssue::JunctionMembershipMismatch {
                        junction: junction.id.clone(),
                        detail: format!(
                            "road {road} is listed as a connector but does not belong to it"
                        ),
                    }),
                _ => {}
            }
        }
    }

    for connection in map.connections.iter() {
        let Some(junction) = &connection.junction else {
            continue;
        };
        let Some(entry) = map.junctions.get(junction) else {
            continue;
        };
        for lane in [&connection.from.lane, &connection.to.lane] {
            let Some(lane) = map.lanes.get(lane) else {
                continue;
            };
            let known = entry.incoming_roads.contains(&lane.road)
                || entry.connecting_roads.contains(&lane.road);
            if !known {
                issues.push(ValidationIssue::JunctionMembershipMismatch {
                    junction: junction.clone(),
                    detail: format!(
                        "connection {} uses road {}, which the junction does not list",
                        connection.id, lane.road
                    ),
                });
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::builder::{LaneSpec, MapBuilder, RoadSpec};
    use crate::geometry::Point3;
    use crate::topology::Direction;
    use crate::units::PositiveWidth;

    fn lanes() -> Vec<LaneSpec> {
        vec![
            LaneSpec::new(PositiveWidth::new(3.5).unwrap(), Direction::Forward),
            LaneSpec::new(PositiveWidth::new(3.5).unwrap(), Direction::Backward),
        ]
    }

    #[test]
    fn a_well_formed_map_validates() {
        let mut builder = MapBuilder::default();
        let a = builder
            .add_road(
                RoadSpec::line(
                    Point3::new(0.0, 0.0, 10.0),
                    Point3::new(100.0, 0.0, 12.0),
                    lanes(),
                )
                .unwrap()
                .with_name("a"),
            )
            .unwrap();
        let b = builder
            .add_road(
                RoadSpec::line(
                    Point3::new(100.0, 0.0, 12.0),
                    Point3::new(200.0, 50.0, 15.0),
                    lanes(),
                )
                .unwrap()
                .with_name("b"),
            )
            .unwrap();
        builder.connect(&a, &b).unwrap();
        builder.finish().unwrap().validate().unwrap();
    }

    #[test]
    fn junction_connectors_between_curved_arms_validate() {
        // Straight out of a real drive (Odaiba, left-hand traffic): a one-lane ramp joining
        // a curved, sloping main road through a junction. The connector's boundaries missed
        // the lane width by 7 µm at its start and failed validation at machine precision.
        use crate::geometry::Curve3;
        use crate::map::TrafficHandedness;
        let mut builder = MapBuilder::default();
        builder.metadata_mut().handedness = TrafficHandedness::LeftHand;
        let ramp = builder
            .add_road(
                RoadSpec::new(
                    Curve3::polyline([
                        Point3::new(0.0, 0.0, 0.0),
                        Point3::new(2.1, -3.5, -0.01),
                        Point3::new(5.4, -10.8, 0.09),
                        Point3::new(7.3, -21.5, 0.05),
                        Point3::new(6.2, -27.1, -0.08),
                        Point3::new(3.7, -32.4, -0.09),
                        Point3::new(-0.5, -37.5, -0.14),
                    ])
                    .unwrap(),
                    vec![LaneSpec::new(
                        PositiveWidth::new(3.5).unwrap(),
                        Direction::Forward,
                    )],
                )
                .with_name("ramp"),
            )
            .unwrap();
        let main = builder
            .add_road(
                RoadSpec::new(
                    Curve3::polyline([
                        Point3::new(-0.5, -37.5, -0.14),
                        Point3::new(-12.3, -46.5, -0.24),
                        Point3::new(-27.3, -56.4, -0.31),
                        Point3::new(-44.2, -67.8, -0.49),
                        Point3::new(-61.5, -79.5, -0.68),
                        Point3::new(-77.8, -91.1, -0.91),
                        Point3::new(-92.0, -101.5, -1.09),
                        Point3::new(-106.8, -111.5, -1.28),
                        Point3::new(-119.6, -120.0, -1.29),
                    ])
                    .unwrap(),
                    vec![
                        LaneSpec::new(PositiveWidth::new(3.3).unwrap(), Direction::Forward),
                        LaneSpec::new(PositiveWidth::new(3.3).unwrap(), Direction::Forward),
                        LaneSpec::new(PositiveWidth::new(3.4).unwrap(), Direction::Backward),
                    ],
                )
                .with_name("main"),
            )
            .unwrap();
        let junction = builder.add_junction(Some("j"));
        builder.connect_via(&junction, &ramp, &main).unwrap();
        builder.finish().unwrap().validate().unwrap();
    }

    #[test]
    fn roads_that_are_linked_but_do_not_meet_are_reported() {
        let mut builder = MapBuilder::default();
        let a = builder
            .add_road(
                RoadSpec::line(Point3::ORIGIN, Point3::new(100.0, 0.0, 0.0), lanes())
                    .unwrap()
                    .with_name("a"),
            )
            .unwrap();
        let b = builder
            .add_road(
                RoadSpec::line(
                    Point3::new(105.0, 0.0, 0.0),
                    Point3::new(200.0, 0.0, 0.0),
                    lanes(),
                )
                .unwrap()
                .with_name("b"),
            )
            .unwrap();
        builder.connect(&a, &b).unwrap();
        let error = builder.finish().unwrap().validate().unwrap_err();
        assert!(error.issues.iter().any(
            |issue| matches!(issue, ValidationIssue::RoadEndpointGap { gap, .. } if *gap > 4.0)
        ));
    }
}
