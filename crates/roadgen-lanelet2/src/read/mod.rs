//! Reading a Lanelet2 map back into the IR.
//!
//! The exporter lowers a road onto lanelets; this goes the other way, and it is a
//! different kind of job from reading OpenDRIVE. A Lanelet2 file holds every lane
//! exactly — both boundaries, in 3D, and the points two lanes share — but it holds
//! no road and no junction at all. Those are reconstructed:
//!
//! 1. **Lanes.** Every lanelet a vehicle, a bicycle or a pedestrian moves along
//!    becomes one IR lane, with the lanelet's boundaries as its boundaries. Nothing is
//!    regenerated, so the lanes come back exactly as the file drew them.
//! 2. **Roads.** Lanelets that share a boundary linestring are side by side, and a
//!    run of them side by side is one road: a single cross-section whose reference
//!    line is the outermost boundary on the side the lanes are laid out from. The
//!    widths are measured across it. See [`roads`].
//! 3. **Junctions.** A lanelet tagged `turn_direction` is how Autoware marks the
//!    lanes that cross an intersection, so each of those becomes a junction
//!    connector of its own, and connectors that belong together — one follows the
//!    other, they leave or reach the same road, they share a boundary, or they cross
//!    at the same level — are one junction. A turn that branches into several
//!    turns is the road it is up to the branch. See [`junctions`].
//! 4. **Rules and furniture.** Traffic lights, right of way, signs, stop lines and
//!    crosswalks come back as the IR's objects and rules. See [`furniture`].
//!
//! Continuity is Lanelet2's own: a lanelet follows another when its boundaries start
//! at the points the other's end at. Every such pair becomes a
//! [`LaneConnection`](roadgen_core::topology::LaneConnection).
//!
//! # What does not come back
//!
//! Whatever the file cannot say. Road ids and names are made up from the lanelet
//! ids; the reference line is a polyline, because a lanelet's boundary is one;
//! opposite directions are separate roads, because nothing in the file pairs them;
//! and a lanelet the IR has no lane for is left out. Each of those is reported in
//! [`Imported::approximations`] rather than failing the read.

mod furniture;
mod junctions;
mod roads;
mod source;

use std::collections::BTreeMap;
use std::path::Path;

use ll2_projection::{GpsPoint, LocalCartesian, Origin, Projector};

use roadgen_core::geometry::{Point3, SamplingConfig};
use roadgen_core::map::{Map, MapMetadata, Projection, TrafficHandedness};
use roadgen_core::units::GeoOrigin;
use roadgen_core::validation::UnvalidatedMap;

use crate::error::ImportError;
use crate::grid::MgrsGrid;

use source::Source;

/// What a caller can tell the reader that the file does not.
#[derive(Debug, Clone, PartialEq)]
pub struct ReadOptions {
    /// Which side of the road traffic keeps to. Lanelet2 does not record it: a
    /// lanelet runs one way and knows nothing of the one beside it. It decides only
    /// which side of each reconstructed road's reference line its lanes are laid out
    /// on.
    pub handedness: TrafficHandedness,
    /// Where the map's `(0, 0)` stands. By default, the middle of the file's nodes.
    pub origin: Option<GeoOrigin>,
    /// How finely curves are sampled when the map is written out again.
    pub sampling: SamplingConfig,
    /// How far along the lane the seam between two lanelets may run, metres, before
    /// the two are read as one lane; `None` never reads them so.
    ///
    /// OpenDRIVE ends a lane square to its road, so a seam drawn slantwise is met
    /// by turning the road's reference line until its normal lies along the seam.
    /// A seam a few metres long can be met that way. One drawn nearly along the
    /// lane — twenty metres of diagonal between two lanes three wide, as a map may
    /// draw where a lane changes shape — cannot: the lanes laid out square to the
    /// reference line come out metres from the file's on one side of the seam or
    /// the other. Read as one lane, named `lanelet a+b`, the two keep their shape
    /// and lose only where the seam lay. Only a lanelet that runs into exactly one
    /// other, which runs out of exactly it, is joined, and only when both are
    /// ordinary lanes of one kind, going one way at one speed.
    ///
    /// By default [`DEFAULT_MAX_SEAM_ALONG`]. At zero every such pair with a seam
    /// not square to the lane is joined, and whole runs of lanelets become one lane.
    pub max_seam_along: Option<f64>,
    /// How far a road's reference line may turn at an end to meet the end square,
    /// degrees, below 90.
    ///
    /// OpenDRIVE ends every lane on the reference line's normal, and a lanelet's
    /// end is rarely drawn square to the lane, so the reference line turns near
    /// each end until its normal lies along the end. The further it turns, the
    /// more steeply drawn an end it meets; turned too hard, the normals fan so fast
    /// near the end that the lanes laid out along them fold. A turn that would take
    /// more than this is not made, and the lanes end on the reference line's
    /// normal where the file draws them slantwise. By default
    /// [`DEFAULT_MAX_END_LEAN_DEGREES`].
    pub max_end_lean_degrees: f64,
    /// How far OpenDRIVE may draw a lane edge from where the file draws it, metres,
    /// before the reader tries harder: lanes side by side are split into roads of
    /// their own, and a lone lane is laid out from its middle or its far side. By
    /// default [`DEFAULT_MAX_EDGE_MISS`].
    pub max_edge_miss: f64,
    /// How far a lane's height off the road's tilted surface may stray from the
    /// file's before another knot is kept, metres. By default
    /// [`DEFAULT_LIFT_TOLERANCE`].
    pub lift_tolerance: f64,
    /// How close the ends of two `turn_direction` lanelets have to come for them to
    /// be one junction, metres. By default [`DEFAULT_JUNCTION_END_DISTANCE`]: about
    /// a lane's width, how far apart the two movements round one corner of a
    /// crossroads start and finish — they neither cross nor share a lane.
    pub junction_end_distance: f64,
    /// How far apart in height two `turn_direction` lanelets that cross in plan, or
    /// whose ends meet, may be and still be one junction, metres: further apart,
    /// one passes over the other. By default [`DEFAULT_GRADE_SEPARATION`].
    pub grade_separation: f64,
}

/// [`ReadOptions::max_seam_along`] by default, metres. On Autoware's
/// Nishi-Shinjuku map two seams run 19 and 22 m along the lane and the next
/// longest 3.5 m.
pub const DEFAULT_MAX_SEAM_ALONG: f64 = 10.0;

/// [`ReadOptions::max_end_lean_degrees`] by default. On Autoware's Nishi-Shinjuku
/// map a lane rounding a corner whose ends are drawn at 64° and 75° across it
/// needs this much; at 45° its outer edge swung 1.2 m past its ends.
pub const DEFAULT_MAX_END_LEAN_DEGREES: f64 = 80.0;

/// [`ReadOptions::max_edge_miss`] by default, metres.
pub const DEFAULT_MAX_EDGE_MISS: f64 = 0.1;

/// [`ReadOptions::lift_tolerance`] by default, metres.
pub const DEFAULT_LIFT_TOLERANCE: f64 = 0.005;

/// [`ReadOptions::junction_end_distance`] by default, metres.
pub const DEFAULT_JUNCTION_END_DISTANCE: f64 = 4.0;

/// [`ReadOptions::grade_separation`] by default, metres.
pub const DEFAULT_GRADE_SEPARATION: f64 = 3.0;

impl Default for ReadOptions {
    fn default() -> Self {
        ReadOptions {
            handedness: TrafficHandedness::RightHand,
            origin: None,
            sampling: SamplingConfig::default(),
            max_seam_along: Some(DEFAULT_MAX_SEAM_ALONG),
            max_end_lean_degrees: DEFAULT_MAX_END_LEAN_DEGREES,
            max_edge_miss: DEFAULT_MAX_EDGE_MISS,
            lift_tolerance: DEFAULT_LIFT_TOLERANCE,
            junction_end_distance: DEFAULT_JUNCTION_END_DISTANCE,
            grade_separation: DEFAULT_GRADE_SEPARATION,
        }
    }
}

impl ReadOptions {
    /// Whether every option is one the reader can use: each length finite and not
    /// negative, the edge miss above zero, and the end lean below a right angle.
    pub fn check(&self) -> Result<(), ImportError> {
        let length = |name: &str, value: f64| {
            if value.is_finite() && value >= 0.0 {
                Ok(())
            } else {
                Err(ImportError::Option(format!(
                    "{name} must be a length of zero or more, not {value}"
                )))
            }
        };
        if let Some(limit) = self.max_seam_along {
            length("max_seam_along", limit)?;
        }
        length("lift_tolerance", self.lift_tolerance)?;
        length("junction_end_distance", self.junction_end_distance)?;
        length("grade_separation", self.grade_separation)?;
        if !(self.max_edge_miss.is_finite() && self.max_edge_miss > 0.0) {
            return Err(ImportError::Option(format!(
                "max_edge_miss must be a length above zero, not {}",
                self.max_edge_miss
            )));
        }
        if !(0.0..90.0).contains(&self.max_end_lean_degrees) {
            return Err(ImportError::Option(format!(
                "max_end_lean_degrees must be at least 0 and below 90, not {}",
                self.max_end_lean_degrees
            )));
        }
        Ok(())
    }
}

/// A map read from a Lanelet2 file, and what the reading could not keep.
#[derive(Debug, Clone)]
pub struct Imported {
    /// The map, not yet validated.
    pub map: UnvalidatedMap,
    /// Everything the map says less exactly than the file did, one entry each.
    pub approximations: Vec<String>,
}

/// Reads a Lanelet2 map from OSM XML already in memory.
pub fn from_osm_str(xml: &str, options: &ReadOptions) -> Result<Imported, ImportError> {
    options.check()?;
    let (document, errors) = ll2_io::osm::parse(xml).map_err(ImportError::Parse)?;
    let mut approximations = Approximations::default();
    if !errors.is_empty() {
        approximations.note(format!(
            "the OSM parser skipped {} elements it could not read: {}",
            errors.len(),
            errors.join("; ")
        ));
    }

    let metadata = metadata(&document, options, &mut approximations)?;
    let projector = LocalCartesian::new(Origin::new(GpsPoint::new(
        metadata.origin.latitude(),
        metadata.origin.longitude(),
        metadata.origin.altitude(),
    )));
    let source = Source::new(&document, &projector, options, &mut approximations)?;

    let mut map = Map::new(metadata);
    let built = roads::build(&source, &mut map, &mut approximations)?;
    junctions::build(&source, &built, &mut map, &mut approximations)?;
    furniture::build(&source, &built, &mut map, &mut approximations)?;

    Ok(Imported {
        map: UnvalidatedMap::from_map(map),
        approximations: approximations.finish(),
    })
}

/// Reads a Lanelet2 `.osm` file.
pub fn read(path: impl AsRef<Path>) -> Result<Imported, ImportError> {
    read_with(path, &ReadOptions::default())
}

/// Reads a Lanelet2 `.osm` file, with the caller's options.
pub fn read_with(path: impl AsRef<Path>, options: &ReadOptions) -> Result<Imported, ImportError> {
    let path = path.as_ref();
    let xml = std::fs::read_to_string(path)
        .map_err(|error| ImportError::Io(format!("{}: {error}", path.display())))?;
    from_osm_str(&xml, options)
}

/// What the reading could not keep, collected as it goes.
///
/// A kind of thing that happens forty times is one line that says forty, which is
/// what most of these are.
#[derive(Default)]
pub(crate) struct Approximations {
    lines: Vec<String>,
    counted: BTreeMap<String, usize>,
}

impl Approximations {
    pub(crate) fn note(&mut self, line: impl Into<String>) {
        self.lines.push(line.into());
    }

    /// Notes one occurrence of a kind of thing; `{n}` in the key becomes the count.
    pub(crate) fn count(&mut self, key: impl Into<String>) {
        *self.counted.entry(key.into()).or_default() += 1;
    }

    fn finish(self) -> Vec<String> {
        let mut lines = self.lines;
        for (key, count) in self.counted {
            lines.push(key.replace("{n}", &count.to_string()));
        }
        lines
    }
}

/// The map's origin, projection and handedness.
///
/// The origin is the caller's, or the middle of the file's nodes. The projection is
/// MGRS when the nodes carry an `mgrs_code` — which is how an Autoware map made with
/// the MGRS projector says so — and local Cartesian otherwise. Either way the map's
/// own metres are east/north/up about the origin, which is what the exporter
/// projects back through.
fn metadata(
    document: &ll2_io::osm::Document,
    options: &ReadOptions,
    approximations: &mut Approximations,
) -> Result<MapMetadata, ImportError> {
    let origin = match options.origin {
        Some(origin) => origin,
        None => {
            if document.nodes.is_empty() {
                return Err(ImportError::Inconsistent("the file has no nodes".into()));
            }
            let count = document.nodes.len() as f64;
            let (lat, lon) = document
                .nodes
                .values()
                .fold((0.0, 0.0), |(lat, lon), node| {
                    (lat + node.lat, lon + node.lon)
                });
            GeoOrigin::new(lat / count, lon / count, 0.0)?
        }
    };

    let mgrs = document
        .nodes
        .values()
        .any(|node| node.tags.contains_key("mgrs_code"));
    let projection = if mgrs {
        check_grid_positions(document, origin, approximations);
        Projection::Mgrs
    } else {
        Projection::LocalCartesian
    };

    Ok(MapMetadata {
        name: None,
        origin,
        projection,
        handedness: options.handedness,
        sampling: options.sampling,
    })
}

/// Beyond this, a node's `local_x`/`local_y` and its latitude and longitude are two
/// different places, metres.
const GRID_AGREEMENT: f64 = 0.05;

/// Whether each node's `local_x`/`local_y` is where its latitude and longitude put
/// it in the MGRS square.
///
/// The reader places every node by its latitude and longitude. Autoware places it by
/// `local_x`/`local_y`. A file whose two disagree is one the two will read
/// differently, which is worth saying.
fn check_grid_positions(
    document: &ll2_io::osm::Document,
    origin: GeoOrigin,
    approximations: &mut Approximations,
) {
    let Ok(grid) = MgrsGrid::containing(origin) else {
        approximations
            .note("the map's origin has no MGRS square, so its projection is MGRS in name only");
        return;
    };
    let mut worst: f64 = 0.0;
    let mut outside = 0;
    for node in document.nodes.values() {
        let number = |key: &str| {
            node.tags
                .get(key)
                .and_then(|value| value.parse::<f64>().ok())
        };
        let (Some(x), Some(y)) = (number("local_x"), number("local_y")) else {
            continue;
        };
        match grid.locate(GpsPoint::new(node.lat, node.lon, node.ele)) {
            Ok((gx, gy)) => worst = worst.max((gx - x).hypot(gy - y)),
            Err(_) => outside += 1,
        }
    }
    if worst > GRID_AGREEMENT {
        approximations.note(format!(
            "the nodes' local_x/local_y disagree with their latitude and longitude by up to \
             {worst:.3} m; the map is placed by latitude and longitude"
        ));
    }
    if outside > 0 {
        approximations.note(format!(
            "{outside} nodes lie outside the MGRS square {} the origin falls in",
            grid.code()
        ));
    }
}

/// A node's position in the map's metres: east/north about the origin, and its own
/// elevation as the height.
pub(crate) fn project(projector: &LocalCartesian, node: &ll2_io::osm::Node) -> Option<Point3> {
    let ele = if node.ele != 0.0 {
        node.ele
    } else {
        node.tags
            .get("ele")
            .and_then(|value| value.parse::<f64>().ok())
            .unwrap_or(0.0)
    };
    let [x, y, _] = projector
        .forward(GpsPoint::new(node.lat, node.lon, ele))
        .ok()?;
    Some(Point3::new(x, y, ele))
}
