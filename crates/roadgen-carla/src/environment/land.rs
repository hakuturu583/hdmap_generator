//! The land, for any kind of place: one TIN, built round every road on the map.
//!
//! What differs between a town and a mine is what the land is *shaped* by — nothing
//! but its roads, or a pit and its heaps besides — and what height it sits at before
//! it rolls. How it is built is the same, so it is built here once:
//!
//! - every through road's two edges are breaklines, so the land under a road is the
//!   road's own plane, vertex for vertex, whatever kind of place it is in;
//! - a place's own [`Line`]s — a bench's crest, a heap's toe — are breaklines too,
//!   cut back where a road runs through them;
//! - the country is a grid of points at the [`Base`] height, rolling with seeded
//!   noise away from everything the place is made of and flat near it;
//! - [`Level`]s — a pit's floor, a heap's top — are filled with points at their own
//!   height, and [`Landforms::holes`] keep the country's grid out of what the
//!   place's own lines shape.
//!
//! The result goes into the IR as a [`TinSurface`] with its breaklines, so every
//! format reads the same land.

use roadgen_core::buildings::Footprint;
use roadgen_core::error::GeometryError;
use roadgen_core::geometry::plan::{polyline_distance, ring_contains, PlanPoint, Segments};
use roadgen_core::geometry::Point3;
use roadgen_core::id::{LandCoverId, SiteId, TinBreaklineId, TinSurfaceId};
use roadgen_core::land_cover::LandCover;
use roadgen_core::map::Map;
use roadgen_core::terrain::{TinBreakline, TinError, TinSurface};
use roadgen_core::validation::UnvalidatedMap;

use super::noise;
use super::tin::{self, Constraint};
use crate::ground::{with_reference_lines, Field};

/// Clearance past a road's edge inside which no line of the land runs, metres.
pub const ROAD_CLEARANCE: f64 = 3.0;
/// Within this of anything a place is made of the country does not roll, and past
/// the second distance it rolls fully, metres.
const QUIET: f64 = 20.0;
const ROLLING: f64 = 140.0;

/// The country round a place.
#[derive(Debug, Clone, PartialEq)]
pub struct GroundSpec {
    /// How far past everything the place is made of the land is written, metres.
    pub margin: f64,
    /// How far apart the country's own points are, metres.
    pub cell: f64,
    /// How far apart the vertices of a place's lines are, metres.
    pub spacing: f64,
    /// How much the country rolls, up or down, metres, and over how long a
    /// distance.
    pub relief: f64,
    pub wavelength: f64,
}

impl Default for GroundSpec {
    fn default() -> Self {
        GroundSpec {
            margin: 150.0,
            cell: 10.0,
            spacing: 5.0,
            relief: 6.0,
            wavelength: 180.0,
        }
    }
}

impl GroundSpec {
    pub fn check(&self) -> Result<(), String> {
        for (name, value) in [
            ("cell", self.cell),
            ("spacing", self.spacing),
            ("wavelength", self.wavelength),
        ] {
            if !(value.is_finite() && value > 0.0) {
                return Err(format!(
                    "ground.{name} must be a positive number, not {value}"
                ));
            }
        }
        if !(self.margin >= 0.0 && self.relief >= 0.0) {
            return Err("the ground's margin and relief cannot be negative".into());
        }
        Ok(())
    }
}

/// What height the country sits at before it rolls.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Base {
    /// One height everywhere: a mine's rim, which is the country's.
    Level(f64),
    /// The height of the roads nearest: the flattest ground that stays under every
    /// road, as a town's land is when nothing else is said about it.
    Roads,
}

/// A line a place's land breaks along.
#[derive(Debug, Clone, PartialEq)]
pub struct Line {
    /// The breakline's name within the place: `bench/3/crest`.
    pub key: String,
    /// What it is the edge of: `crest`, `toe`.
    pub kind: &'static str,
    pub points: Vec<Point3>,
    pub closed: bool,
}

/// A region filled with points at one height: a pit's floor, a heap's top.
#[derive(Debug, Clone, PartialEq)]
pub struct Level {
    pub ring: Vec<PlanPoint>,
    pub z: f64,
}

/// What a place shapes its land with, beyond its roads. A town shapes it with
/// nothing, and passes [`Landforms::default`].
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Landforms {
    pub lines: Vec<Line>,
    pub levels: Vec<Level>,
    /// Regions the country's grid stays out of, because the place's own lines and
    /// levels shape them.
    pub holes: Vec<Vec<PlanPoint>>,
    /// Regions that do not roll, and are not otherwise shaped: a yard.
    pub flats: Vec<Vec<PlanPoint>>,
}

/// Which land, for which place.
#[derive(Debug, Clone, PartialEq)]
pub struct Land {
    /// The surface's name: `tin/<name>`, and every breakline's prefix.
    pub name: String,
    pub site: Option<SiteId>,
    pub base: Base,
    pub ground: GroundSpec,
    pub seed: u64,
}

/// What the land came to.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Report {
    pub nodes: usize,
    pub triangles: usize,
    pub breaklines: usize,
}

/// Why the land could not be built.
#[derive(Debug, Clone, PartialEq)]
pub enum Error {
    Spec(String),
    Geometry(GeometryError),
    Tin(TinError),
    /// The map already holds a feature of this name.
    Duplicate(String),
}

impl std::fmt::Display for Error {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Error::Spec(detail) => write!(f, "land: {detail}"),
            Error::Geometry(error) => write!(f, "land: {error}"),
            Error::Tin(error) => write!(f, "land: {error}"),
            Error::Duplicate(id) => write!(f, "{id} is already on the map"),
        }
    }
}

impl std::error::Error for Error {}

impl From<GeometryError> for Error {
    fn from(error: GeometryError) -> Self {
        Error::Geometry(error)
    }
}

impl From<TinError> for Error {
    fn from(error: TinError) -> Self {
        Error::Tin(error)
    }
}

pub(crate) fn plan_of(points: &[Point3]) -> Vec<PlanPoint> {
    points.iter().map(|p| [p.x, p.y]).collect()
}

/// `0` below `edge0`, `1` above `edge1`, and a smooth step between.
pub fn smoothstep(edge0: f64, edge1: f64, x: f64) -> f64 {
    let t = ((x - edge0) / (edge1 - edge0)).clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

/// The outer edges of a road, left then right, from its generated lanes.
pub fn road_edges(map: &Map, road: &roadgen_core::id::RoadId) -> (Vec<Point3>, Vec<Point3>) {
    let sampling = map.metadata.sampling;
    let (mut left, mut right) = (Vec::new(), Vec::new());
    let Some(entry) = map.road(road) else {
        return (left, right);
    };
    for section in 0..entry.sections.len() {
        let lanes = map.lanes_of_section(road, section);
        let outer_left = lanes.iter().max_by_key(|lane| lane.left_edge);
        let outer_right = lanes.iter().min_by_key(|lane| lane.right_edge);
        if let (Some(l), Some(r)) = (outer_left, outer_right) {
            if let (Ok(l), Ok(r)) = (
                l.left_boundary.to_polyline(sampling),
                r.right_boundary.to_polyline(sampling),
            ) {
                left.extend_from_slice(l.points());
                right.extend_from_slice(r.points());
            }
        }
    }
    (left, right)
}

/// Every road on the map as the land sees it: through roads' edges as breaklines,
/// and every road's outline — a junction's movements included — as ground nothing
/// of the land is put on.
struct Roads {
    rails: Segments,
    outlines: Vec<([f64; 4], Vec<PlanPoint>)>,
    edges: Vec<Constraint>,
}

impl Roads {
    fn of(map: &Map) -> Roads {
        let mut roads = Roads {
            rails: Segments::new(20.0),
            outlines: Vec::new(),
            edges: Vec::new(),
        };
        for road in map.roads.iter() {
            let (left, right) = road_edges(map, &road.id);
            if left.len() < 2 || right.len() < 2 {
                continue;
            }
            let ring: Vec<PlanPoint> = plan_of(&left)
                .into_iter()
                .chain(plan_of(&right).into_iter().rev())
                .collect();
            let bounds = ring.iter().fold(
                [
                    f64::INFINITY,
                    f64::INFINITY,
                    f64::NEG_INFINITY,
                    f64::NEG_INFINITY,
                ],
                |b, p| {
                    [
                        b[0].min(p[0]),
                        b[1].min(p[1]),
                        b[2].max(p[0]),
                        b[3].max(p[1]),
                    ]
                },
            );
            roads.outlines.push((bounds, ring));
            if road.is_connector() {
                continue;
            }
            for (side, points) in [("left", left), ("right", right)] {
                roads.rails.insert_polyline(&plan_of(&points));
                roads.edges.push(Constraint {
                    kind: "road_edge".into(),
                    key: format!("road/{}/{side}", road.id.local_name()),
                    points,
                    closed: false,
                });
            }
        }
        roads
    }

    /// Within `reach` of a road's edge, or on a road.
    fn near(&self, p: PlanPoint, reach: f64) -> bool {
        self.rails.within(p, reach).is_some()
            || self.outlines.iter().any(|(b, ring)| {
                p[0] >= b[0]
                    && p[0] <= b[2]
                    && p[1] >= b[1]
                    && p[1] <= b[3]
                    && ring_contains(ring, p)
            })
    }
}

/// A place's line cut back where a road runs through it: the stretches clear of
/// every road, each an open line of its own, or the line whole.
fn clear_of(roads: &Roads, line: Line) -> Vec<Constraint> {
    let kept: Vec<bool> = line
        .points
        .iter()
        .map(|p| !roads.near([p.x, p.y], ROAD_CLEARANCE))
        .collect();
    if kept.iter().all(|&ok| ok) {
        return vec![Constraint {
            kind: line.kind.into(),
            key: line.key,
            points: line.points,
            closed: line.closed,
        }];
    }
    let count = line.points.len();
    let mut pieces = Vec::new();
    let mut run: Vec<Point3> = Vec::new();
    // A closed line starts just after a gap, so a stretch across its seam stays one.
    let start = if line.closed {
        kept.iter().position(|&ok| !ok).map_or(0, |gap| gap + 1)
    } else {
        0
    };
    for step in 0..count {
        let index = (start + step) % count;
        if kept[index] {
            run.push(line.points[index]);
        } else if !run.is_empty() {
            pieces.push(std::mem::take(&mut run));
        }
    }
    if !run.is_empty() {
        pieces.push(run);
    }
    let pieces: Vec<Vec<Point3>> = pieces
        .into_iter()
        .filter(|piece| piece.len() >= 2)
        .collect();
    let several = pieces.len() > 1;
    pieces
        .into_iter()
        .enumerate()
        .map(|(index, points)| Constraint {
            kind: line.kind.into(),
            key: if several {
                format!("{}/{index}", line.key)
            } else {
                line.key.clone()
            },
            points,
            closed: false,
        })
        .collect()
}

/// Builds the land round every road on `map` and `forms`, and puts it on the map as
/// one TIN surface with its breaklines.
pub fn generate(
    map: &mut UnvalidatedMap,
    land: &Land,
    forms: Landforms,
) -> Result<(TinSurfaceId, Report), Error> {
    land.ground.check().map_err(Error::Spec)?;
    let surface_id = TinSurfaceId::new(&land.name);
    if map.as_map().tin_surfaces.contains(&surface_id) {
        return Err(Error::Duplicate(surface_id.to_string()));
    }
    let ground = &land.ground;
    let source = map.as_map();
    let roads = Roads::of(source);

    let Landforms {
        lines,
        levels,
        holes,
        flats,
    } = forms;
    // The roads' edges first: where anything else would cross one, the road wins.
    let mut constraints: Vec<Constraint> = roads
        .edges
        .iter()
        .map(|edge| Constraint {
            kind: edge.kind.clone(),
            key: edge.key.clone(),
            points: edge.points.clone(),
            closed: false,
        })
        .collect();
    for line in lines {
        constraints.extend(clear_of(&roads, line));
    }

    // Every line the land breaks along, to keep the ground's own points off them,
    // and everything the place is made of — its lines, its roads, its buildings,
    // its flats — to keep the country's roll off it.
    let mut lines = Segments::new(ground.cell);
    let mut features = Segments::new(40.0);
    for constraint in &constraints {
        let plan = plan_of(&constraint.points);
        lines.insert_polyline(&plan);
        features.insert_polyline(&plan);
    }
    for ring in holes.iter().chain(&flats) {
        features.insert_ring(ring);
    }
    for part in source.building_parts.iter() {
        features.insert_ring(&plan_of(part.solid.footprint.points()));
    }

    // The extent: everything above, and the margin round it.
    let mut bounds = [
        f64::INFINITY,
        f64::INFINITY,
        f64::NEG_INFINITY,
        f64::NEG_INFINITY,
    ];
    let mut widen = |p: PlanPoint| {
        bounds = [
            bounds[0].min(p[0]),
            bounds[1].min(p[1]),
            bounds[2].max(p[0]),
            bounds[3].max(p[1]),
        ];
    };
    constraints
        .iter()
        .flat_map(|c| c.points.iter())
        .for_each(|p| widen([p.x, p.y]));
    flats.iter().flatten().copied().for_each(&mut widen);
    if !bounds[0].is_finite() {
        return Err(Error::Spec(
            "there is nothing on the map to build land round".into(),
        ));
    }

    // The height the country sits at, before it rolls.
    let field = match land.base {
        Base::Level(_) => None,
        Base::Roads => {
            // Every lane a vehicle drives on; a pavement stands above the road and
            // would bring the ground up through it.
            let samples: Vec<Point3> = source
                .lanes
                .iter()
                .filter(|lane| {
                    !matches!(lane.lane_type, roadgen_core::semantics::LaneType::Sidewalk)
                })
                .filter_map(|lane| lane.centerline.to_polyline(source.metadata.sampling).ok())
                .flat_map(|line| line.points().to_vec())
                .collect();
            Field::from_samples(
                with_reference_lines(source, samples),
                &[],
                ground.margin + ground.cell,
                ground.cell,
            )
        }
    };
    let base = |x: f64, y: f64| match (land.base, &field) {
        (Base::Level(z), _) => z,
        (Base::Roads, Some(field)) => field.height(x, y),
        (Base::Roads, None) => 0.0,
    };
    let height = |x: f64, y: f64| -> f64 {
        let flat = flats.iter().any(|ring| ring_contains(ring, [x, y]));
        let distance = features.within([x, y], ROLLING).unwrap_or(ROLLING);
        let weight = if flat {
            0.0
        } else {
            smoothstep(QUIET, ROLLING, distance)
        };
        base(x, y) + ground.relief * weight * noise::fractal(x, y, ground.wavelength, land.seed)
    };

    // The free points: the country on a grid, and the levels.
    let (cell, margin) = (ground.cell, ground.margin);
    let (x0, y0) = (bounds[0] - margin, bounds[1] - margin);
    let columns = ((bounds[2] + margin - x0) / cell).ceil() as usize;
    let rows = ((bounds[3] + margin - y0) / cell).ceil() as usize;
    let off_lines = |p: PlanPoint, reach: f64| lines.within(p, reach).is_none();
    let mut free = Vec::new();
    for j in 0..=rows {
        for i in 0..=columns {
            let p = [x0 + cell * i as f64, y0 + cell * j as f64];
            if holes.iter().any(|ring| ring_contains(ring, p))
                || roads.near(p, cell * 0.5)
                || !off_lines(p, cell * 0.45)
            {
                continue;
            }
            free.push(Point3::new(p[0], p[1], height(p[0], p[1])));
        }
    }
    let step = (cell / 2.0).max(ground.spacing);
    for level in &levels {
        let (lo, hi) = level.ring.iter().fold(
            ([f64::INFINITY; 2], [f64::NEG_INFINITY; 2]),
            |(lo, hi), p| {
                (
                    [lo[0].min(p[0]), lo[1].min(p[1])],
                    [hi[0].max(p[0]), hi[1].max(p[1])],
                )
            },
        );
        let mut closed = level.ring.clone();
        if let Some(&first) = level.ring.first() {
            closed.push(first);
        }
        let mut y = lo[1] + step / 2.0;
        while y < hi[1] {
            let mut x = lo[0] + step / 2.0;
            while x < hi[0] {
                let p = [x, y];
                if ring_contains(&level.ring, p)
                    && polyline_distance(p, &closed) > step * 0.45
                    && !roads.near(p, step * 0.5)
                    && off_lines(p, step * 0.45)
                {
                    free.push(Point3::new(x, y, level.z));
                }
                x += step;
            }
            y += step;
        }
    }

    let (tin, breaklines) = tin::build(free, constraints)?;
    let report = Report {
        nodes: tin.nodes().len(),
        triangles: tin.triangles().len(),
        breaklines: breaklines.len(),
    };
    let target = map.as_map_mut();
    let mut ids = Vec::with_capacity(breaklines.len());
    for breakline in breaklines {
        let id = TinBreaklineId::new(format!("{}/{}", land.name, breakline.key));
        target
            .tin_breaklines
            .insert(
                id.clone(),
                TinBreakline {
                    id: id.clone(),
                    surface: surface_id.clone(),
                    kind: breakline.kind,
                    line: breakline.line,
                },
            )
            .map_err(|duplicate| Error::Duplicate(duplicate.0.to_string()))?;
        ids.push(id);
    }
    target
        .tin_surfaces
        .insert(
            surface_id.clone(),
            TinSurface {
                id: surface_id.clone(),
                site: land.site.clone(),
                breaklines: ids,
                tin,
            },
        )
        .map_err(|duplicate| Error::Duplicate(duplicate.0.to_string()))?;
    Ok((surface_id, report))
}

/// Puts a land cover on the map: `key` within the place `name`, of `kind`, over
/// `ring`, lying on `surface`.
pub fn cover(
    map: &mut Map,
    name: &str,
    key: &str,
    kind: &str,
    ring: Vec<Point3>,
    site: Option<&SiteId>,
    surface: Option<&TinSurfaceId>,
) -> Result<LandCoverId, Error> {
    let id = LandCoverId::new(format!("{name}/{key}"));
    map.land_covers
        .insert(
            id.clone(),
            LandCover {
                id: id.clone(),
                site: site.cloned(),
                kind: kind.into(),
                area: Footprint::new(ring)?,
                draped_on: surface.cloned(),
            },
        )
        .map_err(|duplicate| Error::Duplicate(duplicate.0.to_string()))?;
    Ok(id)
}
