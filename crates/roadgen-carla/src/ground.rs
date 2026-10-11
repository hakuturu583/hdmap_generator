//! The height of the ground around the road network.
//!
//! A road network says nothing about the shape of the land it runs through, and
//! nothing here pretends otherwise: the ground is the flattest thing that stays
//! under every road at the road's own height, which is what a road network *does*
//! say about its land. It is a height field — a grid over the network's extent plus
//! a margin, whose height at each vertex is taken from the roads near it — that the
//! land in [`crate::terrain`] reads its heights off, away from the roads.
//!
//! Each vertex takes the *lowest* road vertex within a cell's reach of it, a little
//! under — never an average, because an average of a road climbing away from a
//! vertex sits above the road at that vertex, and the land would come up through
//! the road there.

use roadgen_core::geometry::{Curve3, Point3};
use roadgen_core::map::Map;

use crate::mesh::Mesh;
use crate::surfaces::SurfaceConfig;
use crate::tags::Role;

/// How far under the roads the ground sits, metres: more than a road can bend
/// under the chord between two ground vertices.
pub const DROP: f64 = 0.05;
/// How far apart the reference line is sampled for the ground, metres.
const SAMPLE_SPACING: f64 = 2.0;

/// `samples` with every road's reference line added, densely: a straight road's
/// surface has vertices at its two ends and nowhere between, and a ground that read
/// only those would take the road's far end's height all the way along it. A
/// polyline is already as dense as it is going to get, and its own samples are
/// cheaper than resampling it.
pub fn with_reference_lines(map: &Map, mut samples: Vec<Point3>) -> Vec<Point3> {
    let sampling = map.metadata.sampling;
    for road in map.roads.iter() {
        let line = &road.reference_line;
        if matches!(line, Curve3::Polyline(_) | Curve3::Bezier(_)) {
            if let Ok(points) = line.samples(sampling) {
                samples.extend(points.into_iter().map(|sample| sample.point));
            }
            continue;
        }
        let Ok(length) = line.horizontal_length() else {
            continue;
        };
        let steps = (length / SAMPLE_SPACING).ceil().max(1.0) as usize;
        for step in 0..=steps {
            let station = length * step as f64 / steps as f64;
            if let Ok(sample) = line.sample_at(station, sampling) {
                samples.push(sample.point);
            }
        }
    }
    samples
}

/// The ground as a height field: a grid of heights, read between its vertices.
pub struct Field {
    x0: f64,
    y0: f64,
    cell: f64,
    columns: usize,
    rows: usize,
    /// Row-major from the south-west corner, `(columns + 1) * (rows + 1)` of them.
    heights: Vec<f64>,
}

impl Field {
    /// Lays the field out under `surface` — the roads built so far — reaching
    /// `config.ground_extent` past them and past every building. `None` when there
    /// is nothing to lay it under or the configuration asks for none.
    pub fn under(map: &Map, surface: &[Mesh], config: &SurfaceConfig) -> Option<Field> {
        if config.ground_extent <= 0.0 || config.ground_cell <= 0.0 {
            return None;
        }
        // Every vertex a vehicle drives on: the roads and the gutters. Pavements
        // stand above the road beside them and are left out, or the ground under
        // a pavement would come up through the road.
        let samples: Vec<Point3> = surface
            .iter()
            .filter(|mesh| matches!(mesh.role, Role::Road | Role::Gutter))
            .flat_map(|mesh| mesh.positions.iter().copied())
            .collect();
        if samples.is_empty() {
            return None;
        }
        let buildings: Vec<Point3> = map
            .building_parts
            .iter()
            .flat_map(|part| part.solid.footprint.points().iter().copied())
            .collect();
        // The verge is what the ground is drawn up to, and it reaches past the road.
        Field::from_samples(
            with_reference_lines(map, samples),
            &buildings,
            config.ground_extent + config.verge_width,
            config.ground_cell,
        )
    }

    /// The field under `samples` — points of the roads a vehicle drives on — over
    /// their extent and `extent`'s, plus `margin` all round, in `cell`-metre cells.
    /// Shared by the export, which samples the surface it built, and by an
    /// environment's land, which samples the lanes.
    pub fn from_samples(
        samples: Vec<Point3>,
        extent: &[Point3],
        margin: f64,
        cell: f64,
    ) -> Option<Field> {
        if samples.is_empty() || cell <= 0.0 {
            return None;
        }
        let mut min = [f64::INFINITY; 2];
        let mut max = [f64::NEG_INFINITY; 2];
        let mut widen = |point: &Point3| {
            min[0] = min[0].min(point.x);
            min[1] = min[1].min(point.y);
            max[0] = max[0].max(point.x);
            max[1] = max[1].max(point.y);
        };
        samples.iter().for_each(&mut widen);
        extent.iter().for_each(&mut widen);
        let x0 = min[0] - margin;
        let y0 = min[1] - margin;
        let columns = ((max[0] + margin - x0) / cell).ceil().max(1.0) as usize;
        let rows = ((max[1] + margin - y0) / cell).ceil().max(1.0) as usize;
        let stride = columns + 1;

        // A ground vertex governs the triangles around it, which reach a cell's
        // diagonal; every road sample that could be over one of them is looked at,
        // and the lowest wins. Each sample reaches only the few vertices within
        // that diagonal of it, so the samples are scattered onto the grid rather
        // than the grid searched from every vertex.
        let reach = cell * std::f64::consts::SQRT_2;
        let mut heights = vec![f64::INFINITY; stride * (rows + 1)];
        let span = |centre: f64, count: usize| -> std::ops::RangeInclusive<usize> {
            let low = ((centre - reach) / cell).ceil().max(0.0) as usize;
            let high = ((centre + reach) / cell).floor().max(0.0) as usize;
            low..=high.min(count)
        };
        for point in &samples {
            for j in span(point.y - y0, rows) {
                for i in span(point.x - x0, columns) {
                    let x = x0 + cell * i as f64;
                    let y = y0 + cell * j as f64;
                    if (point.x - x).powi(2) + (point.y - y).powi(2) <= reach * reach {
                        let slot = &mut heights[j * stride + i];
                        *slot = slot.min(point.z);
                    }
                }
            }
        }
        // The rest of the grid — most of it, out in the margin — takes the height
        // of the nearest vertex that has one, ring by ring outwards: flat away from
        // the roads, and at the height of whichever road is closest.
        let mut queue: std::collections::VecDeque<usize> = (0..heights.len())
            .filter(|&index| heights[index].is_finite())
            .collect();
        while let Some(index) = queue.pop_front() {
            let (i, j) = (index % stride, index / stride);
            let neighbours = [
                (i > 0).then(|| index - 1),
                (i < columns).then(|| index + 1),
                (j > 0).then(|| index - stride),
                (j < rows).then(|| index + stride),
            ];
            for next in neighbours.into_iter().flatten() {
                if !heights[next].is_finite() {
                    heights[next] = heights[index];
                    queue.push_back(next);
                }
            }
        }
        for height in &mut heights {
            *height -= DROP;
        }
        Some(Field {
            x0,
            y0,
            cell,
            columns,
            rows,
            heights,
        })
    }

    fn vertex(&self, i: usize, j: usize) -> Point3 {
        Point3::new(
            self.x0 + self.cell * i as f64,
            self.y0 + self.cell * j as f64,
            self.heights[j * (self.columns + 1) + i],
        )
    }

    /// The ground's height at a position, interpolated within the grid's cells so
    /// that a point between vertices reads a height between theirs. Beyond the
    /// grid, the edge's height.
    pub fn height(&self, x: f64, y: f64) -> f64 {
        let fx = ((x - self.x0) / self.cell).clamp(0.0, self.columns as f64 - 1e-9);
        let fy = ((y - self.y0) / self.cell).clamp(0.0, self.rows as f64 - 1e-9);
        let (i, j) = (fx.floor() as usize, fy.floor() as usize);
        let (u, v) = (fx - i as f64, fy - j as f64);
        // The cell is split along the diagonal from its north-west corner to its
        // south-east one.
        let (sw, se, nw, ne) = (
            self.vertex(i, j).z,
            self.vertex(i + 1, j).z,
            self.vertex(i, j + 1).z,
            self.vertex(i + 1, j + 1).z,
        );
        if u + v <= 1.0 {
            // South-west of the diagonal: the triangle sw, se, nw.
            sw + (se - sw) * u + (nw - sw) * v
        } else {
            // North-east of it: the triangle ne, nw, se.
            ne + (nw - ne) * (1.0 - u) + (se - ne) * (1.0 - v)
        }
    }

    /// Every vertex of the grid, at its height, row by row from the south-west.
    pub fn vertices(&self) -> impl Iterator<Item = Point3> + '_ {
        (0..=self.rows).flat_map(move |j| (0..=self.columns).map(move |i| self.vertex(i, j)))
    }
}

/// How far out from a road the land is drawn down to the stated terrain, metres,
/// at most. A stated terrain already meets its roads — their edges are breaklines
/// of it — so there is no verge to slope down; only the seam between the road's
/// surface as this exporter samples it and as the generator did.
pub const STATED_VERGE: f64 = 1.0;

/// The land as the map states it: its TIN surfaces, the breaklines across them, and
/// what each stretch of land is.
pub struct Stated<'a> {
    locators: Vec<roadgen_core::terrain::TinLocator<'a>>,
    nodes: Vec<Point3>,
    /// Every breakline but the roads' edges, which the land takes from the roads
    /// themselves, and the outline of every land cover.
    breaklines: Vec<Vec<Point3>>,
    /// Each land cover's outline in plan, with the material it is surfaced with.
    covers: Vec<(Vec<[f64; 2]>, usize)>,
}

impl<'a> Stated<'a> {
    /// `None` for a map that states no terrain.
    pub fn of(map: &'a Map) -> Option<Stated<'a>> {
        if map.tin_surfaces.is_empty() {
            return None;
        }
        let covers = map
            .land_covers
            .iter()
            .map(|cover| {
                let material = match cover.kind.as_str() {
                    "pit" | "quarry" | "waste_dump" | "spoil_heap" | "stockpile" => {
                        crate::materials::ROCK
                    }
                    "pad" | "yard" => crate::materials::GRAVEL,
                    _ => crate::materials::GRASS,
                };
                let ring = cover.area.points().iter().map(|p| [p.x, p.y]).collect();
                (ring, material)
            })
            .collect();
        Some(Stated {
            locators: map
                .tin_surfaces
                .iter()
                .map(|surface| surface.tin.locator())
                .collect(),
            nodes: map
                .tin_surfaces
                .iter()
                .flat_map(|surface| surface.tin.nodes().iter().copied())
                .collect(),
            breaklines: map
                .tin_breaklines
                .iter()
                .filter(|line| line.kind != "road_edge")
                .map(|line| line.line.points().to_vec())
                .collect(),
            covers,
        })
        .map(|mut stated| {
            // Each land cover's outline too, draped on the land, so that where the
            // surface changes from grass to rock it changes along the outline
            // rather than along whichever triangles straddle it.
            let outlines: Vec<Vec<Point3>> = map
                .land_covers
                .iter()
                .map(|cover| stated.drape(cover.area.points()))
                .collect();
            stated.breaklines.extend(outlines);
            stated
        })
    }

    pub fn height(&self, x: f64, y: f64) -> Option<f64> {
        self.locators
            .iter()
            .find_map(|locator| locator.height_at(x, y))
    }

    /// A closed outline laid on the land: a vertex every few metres along it, each
    /// at the land's height there.
    fn drape(&self, ring: &[Point3]) -> Vec<Point3> {
        const STEP: f64 = 5.0;
        let mut out = Vec::new();
        for (index, &from) in ring.iter().enumerate() {
            let to = ring[(index + 1) % ring.len()];
            let steps = (from.horizontal_distance_to(to) / STEP).ceil().max(1.0) as usize;
            for step in 0..steps {
                let p = from.lerp(to, step as f64 / steps as f64);
                out.push(Point3::new(p.x, p.y, self.height(p.x, p.y).unwrap_or(p.z)));
            }
        }
        if let Some(&first) = out.first() {
            out.push(first);
        }
        out
    }

    /// What the land at `(x, y)` is surfaced with: the last land cover over it, or
    /// grass.
    pub fn material_at(&self, x: f64, y: f64) -> usize {
        self.covers
            .iter()
            .rev()
            .find(|(ring, _)| roadgen_core::geometry::plan::ring_contains(ring, [x, y]))
            .map_or(crate::materials::GRASS, |&(_, material)| material)
    }
}

/// Where the land's heights come from: the map's own terrain, or the flattest
/// ground under its roads.
pub enum Ground<'a> {
    Field(Field),
    Stated(Stated<'a>),
}

impl Ground<'_> {
    /// The ground's own points, to triangulate the land through.
    pub fn vertices(&self) -> Box<dyn Iterator<Item = Point3> + '_> {
        match self {
            Ground::Field(field) => Box::new(field.vertices()),
            Ground::Stated(stated) => Box::new(stated.nodes.iter().copied()),
        }
    }

    /// The height at `(x, y)`, or `fallback` off a stated terrain.
    pub fn height(&self, x: f64, y: f64, fallback: f64) -> f64 {
        match self {
            Ground::Field(field) => field.height(x, y),
            Ground::Stated(stated) => stated.height(x, y).unwrap_or(fallback),
        }
    }

    /// The lines the land breaks along, which it is triangulated to keep.
    pub fn breaklines(&self) -> &[Vec<Point3>] {
        match self {
            Ground::Field(_) => &[],
            Ground::Stated(stated) => &stated.breaklines,
        }
    }

    /// What the land at `(x, y)` is surfaced with.
    pub fn material_at(&self, x: f64, y: f64) -> usize {
        match self {
            Ground::Field(_) => crate::materials::GRASS,
            Ground::Stated(stated) => stated.material_at(x, y),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use roadgen_core::prelude::*;

    fn map() -> ValidatedMap {
        let mut builder = MapBuilder::new(MapMetadata {
            name: Some("g".into()),
            ..MapMetadata::default()
        });
        let lanes = vec![LaneSpec::new(
            PositiveWidth::new(3.5).unwrap(),
            Direction::Forward,
        )];
        builder
            .add_road(
                RoadSpec::line(
                    Point3::new(0.0, 0.0, 10.0),
                    Point3::new(200.0, 0.0, 20.0),
                    lanes,
                )
                .unwrap(),
            )
            .unwrap();
        builder.finish().unwrap().validate().unwrap()
    }

    fn config() -> SurfaceConfig {
        SurfaceConfig {
            ground_extent: 100.0,
            ground_cell: 10.0,
            ..SurfaceConfig::default()
        }
    }

    fn build(map: &ValidatedMap, config: &SurfaceConfig) -> Option<Mesh> {
        crate::surfaces::build(map, "g", config)
            .into_iter()
            .find(|mesh| mesh.role == Role::Terrain)
    }

    #[test]
    fn the_land_reaches_the_margin_past_the_network() {
        let map = map();
        let mesh = build(&map, &config()).expect("a terrain");
        let span = |pick: fn(&Point3) -> f64| {
            let values: Vec<f64> = mesh.positions.iter().map(pick).collect();
            (
                values.iter().cloned().fold(f64::MAX, f64::min),
                values.iter().cloned().fold(f64::MIN, f64::max),
            )
        };
        let verge = config().verge_width;
        let (x_min, x_max) = span(|p| p.x);
        let (y_min, y_max) = span(|p| p.y);
        // The one lane is on the right of the reference line: y from 0 to -3.5.
        assert!((x_min - (-100.0 - verge)).abs() < 1e-9 && x_max >= 300.0 + verge);
        assert!((y_min - (-100.0 - 3.5 - verge)).abs() < 1e-9 && y_max >= 100.0 + verge);
        assert!(mesh.name.starts_with("g_Terrain_Ground_"));
        assert_eq!(crate::tags::label_of(&mesh.name), crate::Label::Terrain);
        assert!(
            mesh.normals().iter().all(|n| n.z > 0.5),
            "the ground faces up"
        );
    }

    #[test]
    fn the_field_sits_just_under_the_road_and_holds_its_height_far_out() {
        let map = map();
        let config = config();
        let roads = crate::surfaces::build(
            &map,
            "g",
            &SurfaceConfig {
                ground_extent: 0.0,
                ..config
            },
        );
        let field = Field::under(&map, &roads, &config).expect("a field");
        // Under the road at x = 100 the road is at z = 15; the field a little under
        // the lowest road vertex within a cell's reach, which on a 5 % grade is up
        // to 14 m back down the road.
        let under = field.height(100.0, -1.75);
        assert!(
            (15.0 - DROP - 1.0..=15.0 - DROP + 0.5).contains(&under),
            "ground under the road at {under}"
        );
        // Far from it the field holds the nearest road's height rather than
        // dropping to zero or drifting off.
        let far = field.height(100.0, 100.0);
        assert!(far > 13.0 && far < 17.0, "ground far out at {far}");
    }

    #[test]
    fn the_field_reads_back_the_height_its_vertices_have() {
        let map = map();
        let config = config();
        let roads = crate::surfaces::build(
            &map,
            "g",
            &SurfaceConfig {
                ground_extent: 0.0,
                ..config
            },
        );
        let field = Field::under(&map, &roads, &config).expect("a field");
        let vertices: Vec<Point3> = field.vertices().collect();
        for point in vertices.iter().step_by(7) {
            assert!((field.height(point.x, point.y) - point.z).abs() < 1e-9);
        }
        // And between two vertices, a height between theirs.
        let (a, b) = (vertices[0], vertices[1]);
        let between = field.height(a.x * 0.7 + b.x * 0.3, a.y);
        assert!(between >= a.z.min(b.z) - 1e-9 && between <= a.z.max(b.z) + 1e-9);
    }

    #[test]
    fn no_extent_means_no_field_but_still_a_verge() {
        let config = SurfaceConfig {
            ground_extent: 0.0,
            ..SurfaceConfig::default()
        };
        let map = map();
        assert!(Field::under(&map, &[], &config).is_none());
        // The land is then the verges alone, level with the road they are beside.
        let mesh = build(&map, &config).expect("a verge");
        let (x_min, x_max) = mesh
            .positions
            .iter()
            .fold((f64::MAX, f64::MIN), |(lo, hi), p| {
                (lo.min(p.x), hi.max(p.x))
            });
        assert!(
            x_min >= -1e-9 && x_max <= 200.0 + 1e-9,
            "a verge past the road's ends"
        );
    }
}
