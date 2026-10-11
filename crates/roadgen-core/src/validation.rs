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
use crate::map::{Lane, Map, Projection};
use crate::semantics::{LaneType, MapObjectKind};
use crate::topology::{LaneEnd, RoadEnd, RoadLinkTarget};

/// How far apart two things may be before they count as disconnected.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ValidationConfig {
    /// Metres; applies to every position comparison.
    pub position_tolerance: f64,
    /// Metres; how far the span of a lane's generated boundaries may differ from its
    /// width. Junction connectors between curved arms miss it by a few millimetres at an
    /// end (the cross-section direction there is sampled), which no consumer can tell.
    pub width_tolerance: f64,
}

impl Default for ValidationConfig {
    fn default() -> Self {
        ValidationConfig {
            position_tolerance: 1e-3,
            width_tolerance: 2e-2,
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
        check_buildings(&self.0, &mut issues);
        check_traffic_lights(&self.0, &mut issues);
        check_barriers(&self.0, &mut issues);
        check_terrain(&self.0, config, &mut issues);
        check_land_covers(&self.0, &mut issues);
        check_sites(&self.0, &mut issues);
        crate::structure::check(&self.0, config.position_tolerance.max(1e-2), &mut issues);
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

/// Buildings are checked for what only becomes wrong once the pieces are put
/// together: the composition, the references, and the heights.
///
/// What a type already rules out is not rechecked. A
/// [`Footprint`](crate::buildings::Footprint) cannot be built from fewer than three
/// vertices, from a non-finite coordinate or from a ring that encloses no area, and a
/// [`Solid`](crate::buildings::Solid)'s shell closes by construction.
///
/// Whether a building stands clear of the road is deliberately *not* checked. A map
/// may perfectly well describe a building the road passes under, and the generator
/// that places buildings is the thing that owes the caller a setback — validation is
/// not where a generator's taste is enforced.
fn check_buildings(map: &Map, issues: &mut Vec<ValidationIssue>) {
    for building in map.buildings.iter() {
        // A building is what its parts add up to, so a building with none is not a
        // building — it is a name with nothing under it.
        if building.parts.is_empty() {
            issues.push(ValidationIssue::ImplausibleBuilding {
                building: building.id.clone(),
                detail: "no parts; a building is what its parts add up to".to_owned(),
            });
        }
        for part in &building.parts {
            match map.building_parts.get(part) {
                None => issues.push(ValidationIssue::DanglingBuildingPartReference {
                    referrer: building.id.to_string(),
                    part: part.clone(),
                }),
                // The composition is recorded from both ends, as a road and its lanes
                // record it, so the two have to agree.
                Some(entry) if entry.building != building.id => {
                    issues.push(ValidationIssue::ImplausibleBuilding {
                        building: building.id.clone(),
                        detail: format!("part {part} claims building {}", entry.building),
                    })
                }
                Some(_) => {}
            }
        }
        if let Some(frontage) = &building.frontage {
            if !map.roads.contains(&frontage.road) {
                issues.push(ValidationIssue::DanglingRoadReference {
                    referrer: building.id.to_string(),
                    road: frontage.road.clone(),
                });
            }
        }
    }

    for part in map.building_parts.iter() {
        // Named from both ends or not at all: every consumer reads a building's parts
        // through its own list, so a part the list leaves out is a part that is on the
        // map and in none of the files written from it.
        match map.buildings.get(&part.building) {
            None => issues.push(ValidationIssue::ImplausibleBuilding {
                building: part.building.clone(),
                detail: format!("part {} belongs to no building on this map", part.id),
            }),
            Some(building) if !building.parts.contains(&part.id) => {
                issues.push(ValidationIssue::ImplausibleBuilding {
                    building: part.building.clone(),
                    detail: format!(
                        "part {} names this building, which does not list it",
                        part.id
                    ),
                })
            }
            Some(_) => {}
        }
        let solid = &part.solid;
        if !solid.wall_height.is_finite() || solid.wall_height <= 0.0 {
            issues.push(ValidationIssue::ImplausibleBuildingPart {
                part: part.id.clone(),
                detail: format!(
                    "walls {} m high, and a wall rises above the outline it stands on",
                    solid.wall_height
                ),
            });
        }
        if !solid.roof.height.is_finite() || solid.roof.height < 0.0 {
            issues.push(ValidationIssue::ImplausibleBuildingPart {
                part: part.id.clone(),
                detail: format!("a roof cannot rise {} m", solid.roof.height),
            });
        }
        // The two are stated together, so they have to mean the same thing: a shape
        // with no rise is flat, and a flat roof has no rise.
        if (solid.roof.shape == crate::buildings::RoofShape::Flat) != (solid.roof.height == 0.0) {
            issues.push(ValidationIssue::ImplausibleBuildingPart {
                part: part.id.clone(),
                detail: format!(
                    "a {} roof rising {} m: a flat roof rises nothing and a roof that \
                     rises is not flat",
                    solid.roof.shape.as_str(),
                    solid.roof.height
                ),
            });
        }
        // A ridge that runs nowhere: `sin` and `cos` of one turn every vertex of the
        // roof into a non-finite number, and the exporters write what they are given.
        if solid.roof.shape.is_directed() && !solid.roof.direction.is_finite() {
            issues.push(ValidationIssue::ImplausibleBuildingPart {
                part: part.id.clone(),
                detail: format!(
                    "a {} roof pointing {}: a ridge runs in some direction",
                    solid.roof.shape.as_str(),
                    solid.roof.direction
                ),
            });
        }
        if part.levels == 0 {
            issues.push(ValidationIssue::ImplausibleBuildingPart {
                part: part.id.clone(),
                detail: "no storeys; a part of a building has at least one".to_owned(),
            });
        }
    }
}

/// A barrier's height is what a consumer builds it to, so it has to be one; and a
/// barrier that is part of a structure — a bridge's parapet — has to name one the
/// map has.
fn check_barriers(map: &Map, issues: &mut Vec<ValidationIssue>) {
    for object in map.objects.iter() {
        if let MapObjectKind::Barrier {
            height, structure, ..
        } = &object.kind
        {
            if !height.is_finite() || *height <= 0.0 {
                issues.push(ValidationIssue::ImplausibleFeature {
                    feature: object.id.to_string(),
                    detail: format!("a barrier {height} m high stands no height at all"),
                });
            }
            if let Some(structure) = structure {
                if !map.structures.contains(structure) {
                    issues.push(ValidationIssue::DanglingFeatureReference {
                        referrer: object.id.to_string(),
                        feature: structure.to_string(),
                    });
                }
            }
        }
    }
}

/// The terrain is checked for its composition and for the one invariant that ties
/// a breakline to its surface: it is an edge of the triangulation, vertex for vertex
/// and segment for segment.
///
/// What [`Tin::new`](crate::terrain::Tin::new) already rules out — an index past
/// the nodes, a triangle with no area, a clockwise triangle — is not rechecked.
/// Folds are: two triangles that share an edge and lie on the same side of it are
/// a surface folded over itself, which is two heights over one point of the plan.
fn check_terrain(map: &Map, config: ValidationConfig, issues: &mut Vec<ValidationIssue>) {
    for surface in map.tin_surfaces.iter() {
        let feature = surface.id.to_string();
        if surface.tin.is_empty() {
            issues.push(ValidationIssue::ImplausibleFeature {
                feature: feature.clone(),
                detail: "no triangles; a surface is what its triangles cover".into(),
            });
        }
        // Anticlockwise triangles that do not overlap use each directed edge once.
        let mut directed: HashSet<(u32, u32)> = HashSet::new();
        let mut folds = 0usize;
        for &[a, b, c] in surface.tin.triangles() {
            for edge in [(a, b), (b, c), (c, a)] {
                if !directed.insert(edge) {
                    folds += 1;
                }
            }
        }
        let overlaps = overlapping_triangles(&surface.tin);
        if overlaps > 0 {
            issues.push(ValidationIssue::ImplausibleFeature {
                feature: feature.clone(),
                detail: format!(
                    "{overlaps} triangles overlap another in plan: two heights over one \
                     point of the land"
                ),
            });
        }
        if folds > 0 {
            issues.push(ValidationIssue::ImplausibleFeature {
                feature: feature.clone(),
                detail: format!(
                    "{folds} edges are shared by two triangles on the same side of them: \
                     the surface folds over itself"
                ),
            });
        }
        if let Some(site) = &surface.site {
            check_site_membership(
                map,
                site,
                &feature,
                |entry| entry.surfaces.contains(&surface.id),
                issues,
            );
        }

        let nodes: std::collections::HashMap<(u64, u64), u32> = surface
            .tin
            .nodes()
            .iter()
            .enumerate()
            .map(|(index, node)| ((node.x.to_bits(), node.y.to_bits()), index as u32))
            .collect();
        let edges = surface.tin.edges();
        for id in &surface.breaklines {
            let Some(breakline) = map.tin_breaklines.get(id) else {
                issues.push(ValidationIssue::DanglingFeatureReference {
                    referrer: feature.clone(),
                    feature: id.to_string(),
                });
                continue;
            };
            if breakline.surface != surface.id {
                issues.push(ValidationIssue::ImplausibleFeature {
                    feature: id.to_string(),
                    detail: format!(
                        "listed by {feature} but names surface {}",
                        breakline.surface
                    ),
                });
                continue;
            }
            let mut indices = Vec::with_capacity(breakline.line.len());
            for point in breakline.line.points() {
                match nodes.get(&(point.x.to_bits(), point.y.to_bits())) {
                    Some(&index)
                        if (surface.tin.nodes()[index as usize].z - point.z).abs()
                            <= config.position_tolerance =>
                    {
                        indices.push(index)
                    }
                    _ => {
                        issues.push(ValidationIssue::ImplausibleFeature {
                            feature: id.to_string(),
                            detail: format!(
                                "vertex ({:.3}, {:.3}, {:.3}) is not a node of {feature}",
                                point.x, point.y, point.z
                            ),
                        });
                        indices.clear();
                        break;
                    }
                }
            }
            let crossings = indices
                .windows(2)
                .filter(|pair| !edges.contains(&(pair[0].min(pair[1]), pair[0].max(pair[1]))))
                .count();
            if crossings > 0 {
                issues.push(ValidationIssue::ImplausibleFeature {
                    feature: id.to_string(),
                    detail: format!(
                        "{crossings} segments are not edges of {feature}: the triangles \
                         cross the line rather than break along it"
                    ),
                });
            }
        }
    }
    for breakline in map.tin_breaklines.iter() {
        match map.tin_surfaces.get(&breakline.surface) {
            None => issues.push(ValidationIssue::DanglingFeatureReference {
                referrer: breakline.id.to_string(),
                feature: breakline.surface.to_string(),
            }),
            Some(surface) if !surface.breaklines.contains(&breakline.id) => {
                issues.push(ValidationIssue::ImplausibleFeature {
                    feature: breakline.id.to_string(),
                    detail: format!(
                        "names surface {}, which does not list it",
                        breakline.surface
                    ),
                })
            }
            Some(_) => {}
        }
    }
}

/// How many triangles of a TIN overlap another in plan without sharing the edge
/// between them: an edge of one crossing an edge of the other, or one lying inside
/// the other. A fold over a shared edge is counted where folds are.
fn overlapping_triangles(tin: &crate::terrain::Tin) -> usize {
    use crate::geometry::Grid;

    let nodes = tin.nodes();
    let plan = |index: u32| {
        let p = nodes[index as usize];
        [p.x, p.y]
    };
    let cross = |a: [f64; 2], b: [f64; 2], c: [f64; 2]| {
        (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
    };
    let edges: Vec<(u32, u32)> = tin.edges().into_iter().collect();
    let triangles = tin.triangles();
    let reach = |points: &[[f64; 2]]| {
        points.iter().fold(
            ([f64::INFINITY; 2], [f64::NEG_INFINITY; 2]),
            |(lo, hi), p| {
                (
                    [lo[0].min(p[0]), lo[1].min(p[1])],
                    [hi[0].max(p[0]), hi[1].max(p[1])],
                )
            },
        )
    };
    // Cells about a triangle across, so each is filed under a few.
    let cell = tin
        .bounds()
        .map(|b| (((b[2] - b[0]) * (b[3] - b[1])) / triangles.len().max(1) as f64).sqrt())
        .unwrap_or(1.0)
        .max(1.0);

    let mut bad = std::collections::HashSet::new();
    let mut by_edge: Grid<usize> = Grid::new(cell);
    for (index, &(a, b)) in edges.iter().enumerate() {
        let (lo, hi) = reach(&[plan(a), plan(b)]);
        by_edge.insert(lo, hi, index);
    }
    for (index, &(a, b)) in edges.iter().enumerate() {
        let (p, q) = (plan(a), plan(b));
        let (lo, hi) = reach(&[p, q]);
        for &other in by_edge.covering(lo, hi) {
            if other <= index {
                continue;
            }
            let (c, d) = edges[other];
            if [a, b].contains(&c) || [a, b].contains(&d) {
                continue;
            }
            let (r, t) = (plan(c), plan(d));
            // A proper crossing: each segment's ends strictly either side of the
            // other's line.
            let (d1, d2) = (cross(p, q, r), cross(p, q, t));
            let (d3, d4) = (cross(r, t, p), cross(r, t, q));
            if d1 * d2 < 0.0 && d3 * d4 < 0.0 {
                bad.insert((index, other));
            }
        }
    }
    let mut by_triangle: Grid<usize> = Grid::new(cell);
    for (index, triangle) in triangles.iter().enumerate() {
        let corners = triangle.map(plan);
        let (lo, hi) = reach(&corners);
        by_triangle.insert(lo, hi, index);
    }
    let mut inside = 0;
    for (index, triangle) in triangles.iter().enumerate() {
        let [a, b, c] = triangle.map(plan);
        let centre = [(a[0] + b[0] + c[0]) / 3.0, (a[1] + b[1] + c[1]) / 3.0];
        let mut seen = std::collections::HashSet::new();
        for &other in by_triangle.at(centre[0], centre[1]) {
            if other == index || !seen.insert(other) {
                continue;
            }
            let [p, q, r] = triangles[other].map(plan);
            let tolerance = 1e-9;
            if cross(p, q, centre) > tolerance
                && cross(q, r, centre) > tolerance
                && cross(r, p, centre) > tolerance
            {
                inside += 1;
                break;
            }
        }
    }
    bad.len() + inside
}

fn check_land_covers(map: &Map, issues: &mut Vec<ValidationIssue>) {
    for cover in map.land_covers.iter() {
        let feature = cover.id.to_string();
        if cover.kind.trim().is_empty() {
            issues.push(ValidationIssue::ImplausibleFeature {
                feature: feature.clone(),
                detail: "no kind; a land cover is the land being something".into(),
            });
        }
        if let Some(surface) = &cover.draped_on {
            if !map.tin_surfaces.contains(surface) {
                issues.push(ValidationIssue::DanglingFeatureReference {
                    referrer: feature.clone(),
                    feature: surface.to_string(),
                });
            }
        }
        if let Some(site) = &cover.site {
            check_site_membership(
                map,
                site,
                &feature,
                |entry| entry.land_covers.contains(&cover.id),
                issues,
            );
        }
    }
}

/// A feature that names a site has to be one the site lists: the composition is
/// recorded from both ends, as a building and its parts record it.
fn check_site_membership(
    map: &Map,
    site: &crate::id::SiteId,
    feature: &str,
    listed: impl Fn(&crate::site::Site) -> bool,
    issues: &mut Vec<ValidationIssue>,
) {
    match map.sites.get(site) {
        None => issues.push(ValidationIssue::DanglingFeatureReference {
            referrer: feature.to_owned(),
            feature: site.to_string(),
        }),
        Some(entry) if !listed(entry) => issues.push(ValidationIssue::ImplausibleFeature {
            feature: feature.to_owned(),
            detail: format!("names site {site}, which does not list it"),
        }),
        Some(_) => {}
    }
}

fn check_sites(map: &Map, issues: &mut Vec<ValidationIssue>) {
    for site in map.sites.iter() {
        let referrer = site.id.to_string();
        for cover in &site.land_covers {
            match map.land_covers.get(cover) {
                Some(entry) if entry.site.as_ref() == Some(&site.id) => {}
                Some(_) => issues.push(ValidationIssue::ImplausibleFeature {
                    feature: cover.to_string(),
                    detail: format!("listed by {} but does not name it", site.id),
                }),
                None => issues.push(ValidationIssue::DanglingFeatureReference {
                    referrer: referrer.clone(),
                    feature: cover.to_string(),
                }),
            }
        }
        for surface in &site.surfaces {
            // A surface no site owns is the whole map's land, which every site
            // on it lies on.
            match map.tin_surfaces.get(surface) {
                Some(entry) if entry.site.as_ref().is_none_or(|own| own == &site.id) => {}
                Some(_) => issues.push(ValidationIssue::ImplausibleFeature {
                    feature: surface.to_string(),
                    detail: format!("listed by {} but does not name it", site.id),
                }),
                None => issues.push(ValidationIssue::DanglingFeatureReference {
                    referrer: referrer.clone(),
                    feature: surface.to_string(),
                }),
            }
        }
        for road in &site.roads {
            if !map.roads.contains(road) {
                issues.push(ValidationIssue::DanglingRoadReference {
                    referrer: referrer.clone(),
                    road: road.clone(),
                });
            }
        }
    }
}

/// A traffic light's housing height and lamp positions are written verbatim, as a
/// Lanelet2 `height` tag, lamp points and OpenDRIVE `userData`, so a height that is
/// not a positive length or a lamp that is nowhere would be a malformed file.
fn check_traffic_lights(map: &Map, issues: &mut Vec<ValidationIssue>) {
    for object in map.objects.iter() {
        let Some(head) = object.kind.light_head() else {
            continue;
        };
        let mut report = |detail: String| {
            issues.push(ValidationIssue::ImplausibleTrafficLight {
                object: object.id.clone(),
                detail,
            })
        };
        if let Some(height) = head.height {
            if !height.is_finite() || height <= 0.0 {
                report(format!(
                    "a housing {height} m tall; a housing rises above its bottom edge"
                ));
            }
        }
        for (index, bulb) in head.bulbs.iter().enumerate() {
            let at = bulb.position;
            if !at.is_finite() {
                report(format!(
                    "lamp {index} is at ({}, {}, {}), which is not a place",
                    at.x, at.y, at.z
                ));
            }
        }
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
            if (spanned - expected).abs() > config.width_tolerance {
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
    lane: &Lane,
    station: f64,
) -> Option<crate::geometry::UnitVector3> {
    let road = map.roads.get(&lane.road)?;
    Some(road.frame_at(station, map.metadata.sampling).ok()?.left)
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

        // A connection has to leave by the end traffic actually leaves by — unless
        // it joins two pavements, which have no direction of travel to leave by.
        let footway = is_footway(from, to);
        if !footway && connection.from.end != from.direction.exit_end() {
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
        if !footway && connection.to.end != to.direction.entry_end() {
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
    from: &Lane,
    to_end: LaneEnd,
    to: &Lane,
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
    let straight = from_left
        .distance_to(to_left)
        .max(from_right.distance_to(to_right));
    let crossed = from_left
        .distance_to(to_right)
        .max(from_right.distance_to(to_left));
    // Two pavements meet whichever way round: a footway has no direction, so its
    // left may be the other's right.
    let gap = if is_footway(from, to) {
        straight.min(crossed)
    } else {
        straight
    };
    Ok((gap - slack).max(0.0))
}

/// Whether a connection joins two pavements, which have no direction of travel:
/// a pedestrian walks a pavement either way, so neither its ends nor its sides
/// are checked against a direction it does not have.
fn is_footway(from: &Lane, to: &Lane) -> bool {
    from.lane_type == LaneType::Sidewalk && to.lane_type == LaneType::Sidewalk
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
    use crate::builder::{LaneRef, LaneSpec, MapBuilder, RoadSpec};
    use crate::geometry::Point3;
    use crate::id::{BuildingId, BuildingPartId};
    use crate::semantics::MapObjectKind;
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
    fn a_junction_between_curved_arms_validates_at_any_sampling_resolution() {
        // A roundabout: two arcs of a ring joined through a junction, which is the
        // shape that found this. A connector's reference line is a Bézier, and a
        // Bézier used to be measured at the *default* sampling however finely the map
        // was actually sampled — so a map sampled at anything finer reported a length
        // shorter than its own last vertex's station, the lane was cut before that
        // vertex, and the connector stopped a sampling step short of the lane it
        // joined. The map was sound; the resolution decided whether it validated.
        use crate::geometry::{Alignment, SamplingConfig};
        use crate::map::MapMetadata;
        let radius = 22.25;
        let gap = 20.0_f64.to_radians();

        for step in [0.25, 0.5, 1.0, 2.0, 4.0] {
            let metadata = MapMetadata {
                sampling: SamplingConfig::new(step).unwrap(),
                ..MapMetadata::default()
            };
            let mut builder = MapBuilder::new(metadata);

            let arms: Vec<_> = (0..2)
                .map(|index| {
                    let start_angle = index as f64 * std::f64::consts::FRAC_PI_2 + gap / 2.0;
                    let start =
                        Point3::new(radius * start_angle.cos(), radius * start_angle.sin(), 0.0);
                    let alignment =
                        Alignment::new(start, start_angle + std::f64::consts::FRAC_PI_2)
                            .arc(
                                (std::f64::consts::FRAC_PI_2 - gap) * radius,
                                1.0 / radius,
                                0.0,
                            )
                            .unwrap();
                    builder
                        .add_road(
                            RoadSpec::new(
                                alignment.finish().unwrap(),
                                vec![LaneSpec::new(
                                    PositiveWidth::new(3.5).unwrap(),
                                    Direction::Forward,
                                )],
                            )
                            .with_name(format!("arc{index}")),
                        )
                        .unwrap()
                })
                .collect();

            let junction = builder.add_junction(Some("j"));
            builder
                .connect_ends(
                    &arms[0],
                    RoadEnd::End,
                    &arms[1],
                    RoadEnd::Start,
                    Some(&junction),
                )
                .unwrap();

            let issues = builder
                .finish()
                .unwrap()
                .issues(ValidationConfig::default());
            assert!(
                issues.is_empty(),
                "sampled at {step} m: {}",
                issues
                    .iter()
                    .map(ValidationIssue::to_string)
                    .collect::<Vec<_>>()
                    .join("; ")
            );
        }
    }

    #[test]
    fn connector_between_a_curved_arm_and_its_continuation_validates() {
        // The same drive, further on: a four-plus-one lane main road bends into a
        // three-plus-three lane road through a junction; the connectors' end boundaries span
        // 3.2950 m and 3.2987 m for 3.3 m lanes — millimetres, not micrometres.
        use crate::geometry::Curve3;
        use crate::map::TrafficHandedness;
        let f = || LaneSpec::new(PositiveWidth::new(3.3).unwrap(), Direction::Forward);
        let b = || LaneSpec::new(PositiveWidth::new(3.5).unwrap(), Direction::Backward);
        let mut builder = MapBuilder::default();
        builder.metadata_mut().handedness = TrafficHandedness::LeftHand;
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
                        Point3::new(-128.9, -126.2, -1.12),
                    ])
                    .unwrap(),
                    vec![f(), f(), f(), f(), b()],
                )
                .with_name("main"),
            )
            .unwrap();
        let south = builder
            .add_road(
                RoadSpec::new(
                    Curve3::polyline([
                        Point3::new(-138.7, -136.7, -0.78),
                        Point3::new(-139.2, -141.9, -0.68),
                        Point3::new(-134.7, -154.8, -0.54),
                        Point3::new(-127.1, -172.1, -0.39),
                        Point3::new(-118.8, -192.4, -0.21),
                        Point3::new(-110.2, -214.1, -0.03),
                    ])
                    .unwrap(),
                    vec![f(), f(), f(), b(), b(), b()],
                )
                .with_name("south"),
            )
            .unwrap();
        let junction = builder.add_junction(Some("j"));
        builder.connect_via(&junction, &main, &south).unwrap();
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

    /// A map with one road and one building of `parts` parts on it, before validation,
    /// so a test can break one thing about the building and see what is said.
    fn map_with_a_building() -> (UnvalidatedMap, BuildingId) {
        use crate::buildings::{Building, BuildingPart, Footprint, Solid};

        let mut builder = MapBuilder::default();
        builder
            .add_road(
                RoadSpec::line(
                    Point3::new(0.0, 0.0, 0.0),
                    Point3::new(100.0, 0.0, 0.0),
                    lanes(),
                )
                .unwrap()
                .with_name("a"),
            )
            .unwrap();
        let mut map = builder.finish().unwrap();

        let id = BuildingId::new("high_street/left/0");
        let part = BuildingPartId::of_building(&id, 0);
        let footprint = Footprint::new(vec![
            Point3::new(10.0, 10.0, 0.0),
            Point3::new(20.0, 10.0, 0.0),
            Point3::new(20.0, 18.0, 0.0),
            Point3::new(10.0, 18.0, 0.0),
        ])
        .unwrap();
        map.as_map_mut()
            .buildings
            .insert(
                id.clone(),
                Building {
                    id: id.clone(),
                    parts: vec![part.clone()],
                    kind: "house".to_owned(),
                    frontage: None,
                },
            )
            .unwrap();
        map.as_map_mut()
            .building_parts
            .insert(
                part.clone(),
                BuildingPart {
                    id: part,
                    building: id.clone(),
                    solid: Solid::prism(footprint, 6.0),
                    kind: None,
                    levels: 2,
                },
            )
            .unwrap();
        (map, id)
    }

    #[test]
    fn a_part_its_building_does_not_list_is_a_part_no_file_would_carry() {
        let (mut map, id) = map_with_a_building();
        map.as_map_mut()
            .buildings
            .get_mut(&id)
            .unwrap()
            .parts
            .clear();

        let error = map.validate().unwrap_err();
        assert!(
            error.issues.iter().any(|issue| matches!(
                issue,
                ValidationIssue::ImplausibleBuilding { detail, .. }
                    if detail.contains("does not list it")
            )),
            "{:?}",
            error.issues
        );
    }

    #[test]
    fn a_ridge_has_to_run_somewhere() {
        use crate::buildings::{Roof, RoofShape};

        let (mut map, id) = map_with_a_building();
        let part = BuildingPartId::of_building(&id, 0);
        let solid = &mut map
            .as_map_mut()
            .building_parts
            .get_mut(&part)
            .unwrap()
            .solid;
        solid.roof = Roof {
            shape: RoofShape::Gabled,
            height: 2.5,
            direction: f64::NAN,
        };

        let error = map.validate().unwrap_err();
        assert!(
            error.issues.iter().any(|issue| matches!(
                issue,
                ValidationIssue::ImplausibleBuildingPart { detail, .. }
                    if detail.contains("pointing NaN")
            )),
            "{:?}",
            error.issues
        );
    }
    #[test]
    fn a_traffic_light_has_to_be_a_light_someone_could_build() {
        use crate::semantics::{LightBulb, LightColor, LightHead};

        let mut builder = MapBuilder::default();
        let road = builder
            .add_road(
                RoadSpec::line(
                    Point3::new(0.0, 0.0, 0.0),
                    Point3::new(100.0, 0.0, 0.0),
                    lanes(),
                )
                .unwrap(),
            )
            .unwrap();
        let light = builder
            .add_traffic_light(&LaneRef::new(road, 0), LaneEnd::End, 5.0)
            .unwrap();
        let mut map = builder.finish().unwrap();
        map.as_map_mut().objects.get_mut(&light).unwrap().kind = MapObjectKind::TrafficLight {
            head: LightHead {
                height: Some(0.0),
                bulbs: vec![LightBulb {
                    position: Point3::new(1.0, f64::INFINITY, 5.0),
                    color: LightColor::Red,
                    arrow: None,
                }],
            },
        };

        let error = map.validate().unwrap_err();
        let details: Vec<_> = error
            .issues
            .iter()
            .filter_map(|issue| match issue {
                ValidationIssue::ImplausibleTrafficLight { object, detail } if *object == light => {
                    Some(detail.as_str())
                }
                _ => None,
            })
            .collect();
        assert_eq!(details.len(), 2, "{:?}", error.issues);
        assert!(details[0].contains("0 m tall"), "{details:?}");
        assert!(details[1].contains("lamp 0"), "{details:?}");
    }
}

#[cfg(test)]
mod terrain_tests {
    use crate::builder::MapBuilder;
    use crate::geometry::Point3;
    use crate::id::TinSurfaceId;
    use crate::terrain::{Tin, TinSurface};

    fn with(tin: Tin) -> Vec<String> {
        let mut map = MapBuilder::default().finish().unwrap();
        let id = TinSurfaceId::new("land");
        map.as_map_mut()
            .tin_surfaces
            .insert(
                id.clone(),
                TinSurface {
                    id,
                    site: None,
                    breaklines: Vec::new(),
                    tin,
                },
            )
            .unwrap();
        map.issues(Default::default())
            .iter()
            .map(ToString::to_string)
            .collect()
    }

    #[test]
    fn triangles_that_overlap_without_sharing_an_edge_are_reported() {
        let p = |x, y| Point3::new(x, y, 0.0);
        // Two triangles crossing each other, and a third inside the first.
        let crossing = Tin::new(
            vec![
                p(0.0, 0.0),
                p(10.0, 0.0),
                p(0.0, 10.0),
                p(5.0, -2.0),
                p(12.0, 5.0),
                p(5.0, 8.0),
            ],
            vec![[0, 1, 2], [3, 4, 5]],
        )
        .unwrap();
        assert!(with(crossing).iter().any(|issue| issue.contains("overlap")));
        let nested = Tin::new(
            vec![
                p(0.0, 0.0),
                p(10.0, 0.0),
                p(0.0, 10.0),
                p(1.0, 1.0),
                p(3.0, 1.0),
                p(1.0, 3.0),
            ],
            vec![[0, 1, 2], [3, 4, 5]],
        )
        .unwrap();
        assert!(with(nested).iter().any(|issue| issue.contains("overlap")));
        // Two triangles side by side over a shared edge are a surface.
        let square = Tin::new(
            vec![p(0.0, 0.0), p(10.0, 0.0), p(10.0, 10.0), p(0.0, 10.0)],
            vec![[0, 1, 2], [0, 2, 3]],
        )
        .unwrap();
        let issues = with(square);
        assert!(issues.is_empty(), "{issues:?}");
    }
}
