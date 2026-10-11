//! The road, as a surface.
//!
//! Every other exporter here writes what the road *is*: a reference line and a width,
//! a boundary and a lanelet, a polyline and a type. CARLA wants what it *looks like* —
//! triangles, in six named classes, because the class is the semantic ground truth a
//! segmentation camera reports. So this is the one lowering in the workspace that has
//! to decide where the kerb stops and the gutter begins.
//!
//! # The cross-section, cut into bands
//!
//! At each station a road has an ordered list of lateral **cuts**: the edges of its
//! lanes, from the leftmost to the rightmost, read off the generated lanes rather than
//! off the spec so that a road that tapers or gains a lane is measured where it
//! actually is. Between two cuts is a **band**, and a band is one strip of one class:
//!
//! ```text
//!    grass        pavement          road surface         pavement       grass
//!   ┌──────┐┌───────────────┐┌─────┬─────────────┬─────┐┌────────────┐┌──────┐
//!   │verge ││   sidewalk    ││gut- │   driving   │gut- ││  sidewalk  ││verge │
//!   │      ││               ││ ter │             │ ter ││            ││      │
//!   └──────┘└───────────────┘└─────┴─────────────┴─────┘└────────────┘└──────┘
//!             ▲             ╲                           ╱             ▲
//!             │              ╲ curb                curb ╱             │
//!             └─ raised by the kerb height ──────────────┘            └─ Terrain
//! ```
//!
//! A lane whose type is `sidewalk` is raised by the kerb height, which puts a vertical
//! face between it and the surface beside it — that face is the **curb**, and the
//! strip of road at the foot of it is the **gutter**, split off the outermost part of
//! the driving surface the way a real one is. Neither is a lane in the IR and neither
//! should be: they are what a *surface* has and a road network does not.
//!
//! # Where the road stops
//!
//! Beyond the outermost band is the **land**, which is [`crate::terrain`]'s: one
//! surface from every road's edge outwards, sloping down over a verge's width to the
//! ground's height and carrying on at that height to where a lidar reaches. What
//! this module gives it is each section's two edges — the rails the land meets,
//! vertex for vertex — and it is one surface rather than a strip per road so that
//! there is no seam for a wheel or a lidar to find.
//!
//! There is no landscape in it, and there should not be. A road network says nothing
//! about the shape of the country it runs through, and a generator that produced
//! hills here would be inventing a terrain model rather than deriving one.
//! [`crate::check`] says so rather than leaving it to be discovered in the editor.

use std::collections::HashMap;

use roadgen_core::geometry::{Frame3, Point3, Vector3};
use roadgen_core::map::{Map, Road};
use roadgen_core::semantics::{LaneType, MarkingColor, RoadMarking};
use roadgen_core::topology::LateralSide;

use crate::facades;
use crate::materials;
use crate::mesh::Mesh;
use crate::tags::{mesh_name, Role};

/// How a road network is turned into a surface. Every distance is in metres.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SurfaceConfig {
    /// How far a pavement stands above the road beside it. The face between them is
    /// the kerb.
    pub kerb_height: f64,
    /// How much of the road surface at the foot of a kerb is gutter rather than
    /// carriageway. Zero leaves the carriageway whole and writes no gutter mesh.
    pub gutter_width: f64,
    /// How wide a painted line is.
    pub marking_width: f64,
    /// How far above the road surface the paint sits, so that it does not fight the
    /// surface for the same depth value.
    pub marking_rise: f64,
    /// A broken line's painted length, and then the gap after it.
    pub dash_on: f64,
    pub dash_off: f64,
    /// How far out from a road's edge the land slopes down to the ground's height:
    /// the verge. Zero puts the ground's own vertices right up against the road.
    pub verge_width: f64,
    /// How far past the road network the land is written, metres, at the roads'
    /// own heights, for a lidar to reach and a vehicle to land on beyond the verge.
    /// Zero writes no land past the verges; see [`crate::terrain`].
    pub ground_extent: f64,
    /// How far apart the land's vertices are past the verges.
    pub ground_cell: f64,
}

impl Default for SurfaceConfig {
    fn default() -> Self {
        SurfaceConfig {
            kerb_height: 0.15,
            gutter_width: 0.3,
            marking_width: 0.15,
            // A centimetre. Enough to win the depth test at any range a simulated
            // camera looks from, little enough that a wheel does not climb it.
            marking_rise: 0.01,
            dash_on: 3.0,
            dash_off: 6.0,
            verge_width: 8.0,
            // A long-range lidar sees about 200 m; a hundred and fifty of ground
            // past the last road keeps every return it makes on something.
            ground_extent: 150.0,
            ground_cell: 10.0,
        }
    }
}

/// Every mesh a map's surface is made of, in the order they are written.
///
/// `map_name` is the name every mesh's own name begins with, and it is the caller's
/// job to have run it through [`crate::tags::sanitize`] first: an unsanitised name is
/// renamed by Unreal on import, and a renamed mesh is one the classifier no longer
/// agrees with this crate about.
pub fn build(map: &Map, map_name: &str, config: &SurfaceConfig) -> Vec<Mesh> {
    let mut meshes = Vec::new();
    let mut ordinal = Ordinals::default();
    // The roads first, then the land round them: the land is one surface built
    // out from every road's edge, so it needs every edge before it can start.
    let mut land = crate::terrain::Land::default();
    let spacing = ground_spacing(map, config);
    // A land the map states meets its roads already — their edges are breaklines
    // of it — so a road has no verge to slope down to it, and no toe: a line of
    // points a verge out, at the land's heights, would only stand among the land's
    // own nodes, a hand's breadth from its toes and crests, every triangle between
    // them a sliver.
    let layout_config = SurfaceConfig {
        verge_width: if map.tin_surfaces.is_empty() {
            config.verge_width
        } else {
            0.0
        },
        ..*config
    };
    let all_rungs: Vec<Option<Vec<Rung>>> = map
        .roads
        .iter()
        .map(|road| rungs(map, road, spacing))
        .collect();
    let centres = junction_centres(map, &all_rungs);
    for (road, rungs) in map.roads.iter().zip(&all_rungs) {
        let Some(rungs) = rungs else {
            continue;
        };
        let facing = road
            .junction
            .as_ref()
            .and_then(|junction| centres.get(junction).copied());
        for section in 0..road.sections.len() {
            let Some(layout) = Layout::of(map, road, section, &layout_config) else {
                continue;
            };
            // Every rung of the section, with its cuts worked out once: every band,
            // line and flank of the section reads them, and there are a dozen or so
            // of those per lane.
            let within: Vec<Cross> = rungs
                .iter()
                .filter(|rung| layout.covers(rung.station))
                .map(|rung| Cross {
                    rung,
                    cuts: layout.cuts(rung.station),
                })
                .collect();
            if within.len() < 2 {
                continue;
            }
            layout.surfaces(&within, map_name, &mut ordinal, &mut meshes);
            layout.markings(&within, map_name, config, &mut ordinal, &mut meshes);
            // The land meets the road only where the road is on the ground: a
            // stretch on a bridge has the land under it, one in a tunnel the land
            // over it, and neither has a verge.
            for run in at_grade(map, road, &within) {
                if let Some([left, right]) =
                    layout.flanks(run, facing, map_name, &mut ordinal, &mut meshes)
                {
                    land.section(left, right);
                }
            }
        }
    }
    crate::crosswalks::paint(map, map_name, config, &mut ordinal, &mut meshes);
    // The land the map states, where it states one; otherwise the flattest land
    // under the roads, which is all a road network says about it.
    let (ground, verge) = match crate::ground::Stated::of(map) {
        Some(stated) => (
            Some(crate::ground::Ground::Stated(stated)),
            config.verge_width.min(crate::ground::STATED_VERGE),
        ),
        None => (
            crate::ground::Field::under(map, &meshes, config).map(crate::ground::Ground::Field),
            config.verge_width,
        ),
    };
    if let Some(terrain) = land.mesh(ground.as_ref(), verge, map_name, &mut ordinal) {
        meshes.push(terrain);
    }
    meshes.extend(crate::water::meshes(map, map_name, &mut ordinal));
    meshes.extend(crate::structures::meshes(
        map,
        ground.as_ref(),
        map_name,
        &mut ordinal,
    ));
    meshes.extend(crate::barriers::berms(map, map_name, &mut ordinal));
    meshes.retain(|mesh| !mesh.is_empty());
    meshes
}

/// The runs of `within` that are on the ground, each with at least two rungs: the
/// whole of it for a road on no structure, the stretches between its bridges and
/// tunnels otherwise. A rung on a span's boundary is in the run on either side.
fn at_grade<'c, 'r>(map: &Map, road: &Road, within: &'c [Cross<'r>]) -> Vec<&'c [Cross<'r>]> {
    if roadgen_core::structure::connector_carried(map, road) {
        return Vec::new();
    }
    if road.structure.is_empty() {
        return vec![within];
    }
    let mut runs = Vec::new();
    let mut start: Option<usize> = None;
    for (index, cross) in within.iter().enumerate() {
        let station = cross.rung.station;
        let carried = road.structure.carries_strictly(station);
        if !carried {
            start.get_or_insert(index);
        }
        // A rung on a span's start ends the run before it; one inside a span ends
        // whatever run there was.
        if carried || road.structure.begins_at(station) {
            if let Some(from) = start.take() {
                runs.push(&within[from..index + usize::from(!carried)]);
            }
        }
    }
    if let Some(from) = start {
        runs.push(&within[from..]);
    }
    runs.into_iter().filter(|run| run.len() >= 2).collect()
}

/// Where each junction is: the mean of the midpoints of the roads through it.
///
/// A road through a junction has other roads, not land, on the side that faces the
/// junction, and this is what a verge is checked against to find that side.
fn junction_centres(
    map: &Map,
    all_rungs: &[Option<Vec<Rung>>],
) -> HashMap<roadgen_core::id::JunctionId, Point3> {
    let mut sums: HashMap<roadgen_core::id::JunctionId, (Vector3, f64)> = HashMap::new();
    for (road, rungs) in map.roads.iter().zip(all_rungs) {
        let (Some(junction), Some(rungs)) = (&road.junction, rungs) else {
            continue;
        };
        let middle = rungs[rungs.len() / 2].frame.origin;
        let (sum, count) = sums.entry(junction.clone()).or_insert((Vector3::ZERO, 0.0));
        *sum = *sum + middle.to_vector();
        *count += 1.0;
    }
    sums.into_iter()
        .map(|(junction, (sum, count))| (junction, Point3::ORIGIN + sum * (1.0 / count)))
        .collect()
}

/// The ordinal each role's next mesh takes, so that no two meshes in a map share a
/// name. Unreal renames a duplicate on import, and a renamed mesh is one nothing can
/// account for afterwards.
#[derive(Debug, Default)]
pub struct Ordinals {
    next: Vec<(Role, usize)>,
}

impl Ordinals {
    pub fn take(&mut self, role: Role) -> usize {
        match self.next.iter_mut().find(|(held, _)| *held == role) {
            Some((_, ordinal)) => {
                *ordinal += 1;
                *ordinal - 1
            }
            None => {
                self.next.push((role, 1));
                0
            }
        }
    }
}

/// One station of a road, with its banked frame.
struct Rung {
    station: f64,
    frame: Frame3,
}

impl Rung {
    /// The point `lateral` metres to the left of the reference line and `rise` metres
    /// above the road surface there.
    ///
    /// Both are measured in the *banked* frame, so a superelevated road's kerb stands
    /// perpendicular to its surface rather than to the horizon.
    fn at(&self, lateral: f64, rise: f64) -> Point3 {
        self.frame.to_global([0.0, lateral, rise])
    }
}

/// One rung of one section, with the lateral cuts of its cross-section there.
struct Cross<'a> {
    rung: &'a Rung,
    /// From the leftmost cut to the rightmost: one more than there are lanes.
    cuts: Vec<f64>,
}

/// How far apart the ground's own points are beside the roads, metres: the
/// spacing the land wants a road's edge met at. `None` when there is no ground past
/// the verges to meet it.
///
/// A generated ground is a grid of `ground_cell`. A stated one is whatever the map
/// says, and what it says beside a road is its road-edge breaklines, which the
/// generator splits to its own grid's spacing.
fn ground_spacing(map: &Map, config: &SurfaceConfig) -> Option<f64> {
    if !map.tin_surfaces.is_empty() {
        let stated = map
            .tin_breaklines
            .iter()
            .filter(|line| line.kind == "road_edge")
            .flat_map(|line| line.line.points().windows(2))
            .map(|pair| pair[0].horizontal_distance_to(pair[1]))
            .fold(0.0, f64::max);
        let spacing = if stated > 1e-6 {
            stated
        } else {
            config.ground_cell
        };
        return (spacing > 0.0).then_some(spacing);
    }
    (config.ground_extent > 0.0 && config.ground_cell > 0.0).then_some(config.ground_cell)
}

/// How many of the ground's cells apart two rungs are along a straight stretch. The
/// land's triangles beside the road then reach a cell out to the ground's points
/// and two along, which is no sliver; a rung every cell is half as many rungs
/// again on every road, for land triangles that are shorter but no fewer of them
/// thin.
const CELLS_PER_RUNG: f64 = 2.0;

/// Every station a road is sampled at, with its banked frame.
///
/// The frame's own lateral is the right one at every station, the ends included:
/// two roads that meet are tangent-continuous by the time they are built — a corner
/// between them is rounded into an arc — so a cut perpendicular to one is
/// perpendicular to the other, and the two surfaces end on the same line.
///
/// `spacing` is the ground's, from [`ground_spacing`].
fn rungs(map: &Map, road: &Road, spacing: Option<f64>) -> Option<Vec<Rung>> {
    let mut samples = map.vertex_samples(&road.id).ok()?;
    // A rung wherever the pavement changes, too, so that the surface can change
    // material there rather than at the next vertex, which on a straight road is
    // its far end. A bridge's or a tunnel's ends are vertices of the road already.
    let (first, last) = (samples.first()?.station, samples.last()?.station);
    for &(station, _) in road.pavement.spans() {
        if station > first + 1e-6
            && station < last - 1e-6
            && !samples
                .iter()
                .any(|sample| (sample.station - station).abs() < 1e-6)
        {
            if let Ok(sample) = road
                .reference_line
                .sample_at(station, map.metadata.sampling)
            {
                samples.push(sample);
            }
        }
    }
    samples.sort_by(|a, b| a.station.total_cmp(&b.station));
    // And a rung every `CELLS_PER_RUNG` of the ground's cells along a long
    // straight stretch, which has vertices only at its ends: the land meets the
    // road rung for rung, and a street hundreds of metres long met at its two
    // ends alone is a fan of triangles that long beside it, out to the ground's
    // points. Only where the land does meet it, though — every rung is a dozen
    // triangles across the road, and a deck, a tunnel, a movement through a
    // junction on a viaduct and a map with no ground past its verges have no
    // ground beside them to fan out to.
    let spacing = spacing.filter(|_| !roadgen_core::structure::connector_carried(map, road));
    let mut between = Vec::new();
    for pair in samples.windows(2) {
        let Some(spacing) = spacing else {
            break;
        };
        // A structure's ends are vertices already, so a stretch is on one
        // throughout or not at all.
        let (from, to) = (pair[0].station, pair[1].station);
        if road.structure.carries_strictly(0.5 * (from + to)) {
            continue;
        }
        let extra = ((to - from) / (CELLS_PER_RUNG * spacing)).ceil() as usize;
        for k in 1..extra {
            let station = from + (to - from) * k as f64 / extra as f64;
            if let Ok(sample) = road
                .reference_line
                .sample_at(station, map.metadata.sampling)
            {
                between.push(sample);
            }
        }
    }
    if !between.is_empty() {
        samples.extend(between);
        samples.sort_by(|a, b| a.station.total_cmp(&b.station));
    }
    // Two rungs nearer each other than the land keeps two vertices of one line are
    // one: the curve's own sample a hand's breadth from one a width's taper asked
    // for is a strip of road too short to change across, and every land triangle
    // on its edge a sliver. Not where something does change, though — a section,
    // a structure's end, the pavement, a lane's width or height — and not within that of either end of the
    // road, where its outline meets the next road's: moved there, the two cross
    // so nearly along each other that the land's triangulation cannot split them.
    let fixed: Vec<f64> = road
        .sections
        .iter()
        .map(|section| section.station)
        .chain(
            road.structure
                .spans()
                .iter()
                .flat_map(|span| [span.from, span.to]),
        )
        .chain(road.pavement.spans().iter().map(|&(station, _)| station))
        .chain(map.lanes_of(&road.id).into_iter().flat_map(|lane| {
            lane.width
                .knots()
                .iter()
                .map(|knot| knot.0)
                .chain(lane.height.knots().iter().map(|knot| knot.0))
                .collect::<Vec<_>>()
        }))
        .collect();
    let is_fixed = |station: f64| fixed.iter().any(|&at| (at - station).abs() < 1e-6);
    let closest = crate::environment::tin::CLOSEST;
    let mut kept: Vec<roadgen_core::geometry::Sample> = Vec::with_capacity(samples.len());
    for sample in samples {
        let station = sample.station;
        match kept.last_mut() {
            Some(previous)
                if station - previous.station < closest
                    && station > first + closest
                    && station < last - closest
                    && !(is_fixed(previous.station) && is_fixed(station)) =>
            {
                if is_fixed(station) {
                    *previous = sample;
                }
            }
            _ => kept.push(sample),
        }
    }
    let samples = kept;
    let mut rungs = Vec::with_capacity(samples.len());
    for sample in samples {
        let Ok(frame) = sample.frame() else {
            continue;
        };
        rungs.push(Rung {
            station: sample.station,
            frame: frame.banked(road.superelevation.evaluate(sample.station)),
        });
    }
    (rungs.len() >= 2).then_some(rungs)
}

/// How far a rail stands off the road plane, by station.
type Lift<'a> = Box<dyn Fn(f64) -> f64 + 'a>;

/// What one cross-section of one road is made of, left to right.
pub(crate) struct Layout<'a> {
    range: (f64, f64),
    /// The road's lateral offset profile, which the cuts are measured from.
    lane_offset: &'a roadgen_core::geometry::Poly3Profile,
    /// The lanes, from the leftmost to the rightmost.
    lanes: Vec<&'a roadgen_core::map::Lane>,
    config: SurfaceConfig,
    /// What the road is surfaced with along its length.
    pavement: &'a roadgen_core::semantics::PavementProfile,
}

impl<'a> Layout<'a> {
    pub(crate) fn of(
        map: &'a Map,
        road: &'a Road,
        section: usize,
        config: &SurfaceConfig,
    ) -> Option<Layout<'a>> {
        let range = road.section_range(section).ok()?;
        let lanes = map.lanes_of_section(&road.id, section);
        // Left to right: the outermost left lane first, down to the reference line,
        // then out to the right. That is the order the cuts run in, and every band
        // below is the gap between two neighbours in it.
        let mut left: Vec<&roadgen_core::map::Lane> = lanes
            .iter()
            .copied()
            .filter(|lane| lane.side == LateralSide::Left)
            .collect();
        left.sort_by_key(|lane| std::cmp::Reverse(lane.ordinal));
        let mut right: Vec<&roadgen_core::map::Lane> = lanes
            .iter()
            .copied()
            .filter(|lane| lane.side == LateralSide::Right)
            .collect();
        right.sort_by_key(|lane| lane.ordinal);
        left.extend(right);
        if left.is_empty() {
            return None;
        }
        Some(Layout {
            range,
            lane_offset: &road.lane_offset,
            lanes: left,
            config: *config,
            pavement: &road.pavement,
        })
    }

    /// The lateral extent of the carriageway at `station`: the outer edges of the
    /// outermost lanes that carry vehicles, left then right. `None` when there is
    /// no such lane.
    pub(crate) fn carriageway(&self, station: f64) -> Option<(f64, f64)> {
        let cuts = self.cuts(station);
        let first = self
            .lanes
            .iter()
            .position(|lane| matches!(Self::role_of(lane), Some((Role::Road | Role::Gutter, _))))?;
        let last = self.lanes.iter().rposition(|lane| {
            matches!(Self::role_of(lane), Some((Role::Road | Role::Gutter, _)))
        })?;
        Some((cuts[first], cuts[last + 1]))
    }

    /// Where a post stands beside the carriageway on `side`: its lateral offset
    /// from the reference line and how far above the road plane its foot is.
    ///
    /// On the pavement, `setback` in from the kerb face, when there is a pavement
    /// on that side — no further in than the pavement's middle, so a narrow one
    /// still has the post on it. On the verge, `setback` out from the carriageway's
    /// edge, when there is not. `None` when the cross-section carries no vehicles
    /// at all, since there is then no road for a post to stand beside.
    ///
    /// `ledge` is where a post stands on a road with no pavement whose edge has
    /// no verge either — a deck's parapet, a tunnel's walkway: how far out from
    /// the road's outer edge, and how high.
    pub(crate) fn post_position(
        &self,
        station: f64,
        side: LateralSide,
        setback: f64,
        ledge: Option<(f64, f64)>,
    ) -> Option<(f64, f64)> {
        let cuts = self.cuts(station);
        let (left_edge, right_edge) = self.carriageway(station)?;
        // The cuts run left to right and the lateral offset falls with them, so
        // "outwards" on the right is down the list and down in value, and on the
        // left the reverse.
        let (edge, outward): (f64, f64) = match side {
            LateralSide::Left => (left_edge, 1.0),
            LateralSide::Right => (right_edge, -1.0),
        };
        let pavement = self.lanes.iter().enumerate().find(|(index, lane)| {
            let (inner, outer) = match side {
                LateralSide::Left => (cuts[index + 1], cuts[*index]),
                LateralSide::Right => (cuts[*index], cuts[index + 1]),
            };
            let beyond = (inner - edge) * outward >= -1e-9;
            matches!(Self::role_of(lane), Some((Role::Sidewalk, _)))
                && beyond
                && (outer - inner).abs() > 1e-9
        });
        match pavement {
            Some((index, lane)) => {
                let (inner, outer) = match side {
                    LateralSide::Left => (cuts[index + 1], cuts[index]),
                    LateralSide::Right => (cuts[index], cuts[index + 1]),
                };
                let width = (outer - inner).abs();
                Some((
                    inner + outward * setback.min(width / 2.0),
                    self.rise_of(lane),
                ))
            }
            None => match ledge {
                Some((out, rise)) => {
                    let outer = match side {
                        LateralSide::Left => cuts[0],
                        LateralSide::Right => cuts[cuts.len() - 1],
                    };
                    Some((outer + outward * out, rise))
                }
                None => Some((edge + outward * setback, 0.0)),
            },
        }
    }

    /// The lateral offset of the middle of `lane` at `station`, if the lane is one
    /// of this cross-section's.
    pub(crate) fn lane_centre(&self, station: f64, lane: &roadgen_core::id::LaneId) -> Option<f64> {
        let index = self.lanes.iter().position(|held| held.id == *lane)?;
        let cuts = self.cuts(station);
        Some((cuts[index] + cuts[index + 1]) / 2.0)
    }

    fn covers(&self, station: f64) -> bool {
        station >= self.range.0 - 1e-9 && station <= self.range.1 + 1e-9
    }

    /// The lateral offset of every cut at `station`, from the leftmost to the
    /// rightmost. One more of them than there are lanes.
    fn cuts(&self, station: f64) -> Vec<f64> {
        let centre = self.lane_offset.evaluate(station);
        let mut cuts = Vec::with_capacity(self.lanes.len() + 1);
        // Start at the outer edge of the leftmost lane and work in: every lane to the
        // left of the reference line adds its width to the offset of the one inside
        // it, so the sum of them all is where the cross-section ends.
        let leftmost: f64 = self
            .lanes
            .iter()
            .filter(|lane| lane.side == LateralSide::Left)
            .map(|lane| lane.width_at(station))
            .sum();
        let mut offset = centre + leftmost;
        cuts.push(offset);
        for lane in &self.lanes {
            offset -= lane.width_at(station);
            cuts.push(offset);
        }
        cuts
    }

    /// What each lane's surface is, and how high it sits above the road plane.
    fn role_of(lane: &roadgen_core::map::Lane) -> Option<(Role, usize)> {
        match lane.lane_type {
            LaneType::Driving
            | LaneType::Biking
            | LaneType::Parking
            | LaneType::Restricted
            | LaneType::Shoulder => Some((Role::Road, materials::ASPHALT)),
            // OpenDRIVE's `border` is the made strip at the edge of the carriageway,
            // which is what a gutter is.
            LaneType::Border => Some((Role::Gutter, materials::ASPHALT)),
            LaneType::Sidewalk => Some((Role::Sidewalk, materials::SIDEWALK)),
            LaneType::None => None,
        }
    }

    /// How far a lane's surface stands off the road plane, as one number: the
    /// lift the IR gives it halfway along the section, or for a pavement the IR
    /// left flat, the configured kerb. What decides whether there is a kerb, and
    /// which side the land is on; the surfaces themselves follow [`Self::edge_rise`].
    fn rise_of(&self, lane: &roadgen_core::map::Lane) -> f64 {
        if !lane.height.is_flat() {
            let (inner, outer) = lane.height.evaluate((self.range.0 + self.range.1) / 2.0);
            return (inner + outer) / 2.0;
        }
        match lane.lane_type {
            LaneType::Sidewalk => self.config.kerb_height,
            _ => 0.0,
        }
    }

    /// How far lane `index`'s surface stands off the road plane at cut `cut` — one
    /// of its own two edges — at `station`: the IR's lift there, inner or outer
    /// whichever that edge is, or for a lane the IR left flat, [`Self::rise_of`].
    fn edge_rise(&self, index: usize, cut: usize, station: f64) -> f64 {
        let lane = self.lanes[index];
        if lane.height.is_flat() {
            return self.rise_of(lane);
        }
        let (inner, outer) = lane.height.evaluate(station);
        // The cuts run left to right: a left lane's inner edge is its right one,
        // a right lane's its left one.
        let is_left_edge = cut == index;
        match (lane.side, is_left_edge) {
            (LateralSide::Left, true) | (LateralSide::Right, false) => outer,
            _ => inner,
        }
    }

    /// The drivable surface, the pavements, the kerbs and the gutters.
    fn surfaces(
        &self,
        rungs: &[Cross],
        map_name: &str,
        ordinals: &mut Ordinals,
        out: &mut Vec<Mesh>,
    ) {
        for (index, lane) in self.lanes.iter().enumerate() {
            let Some((role, material)) = Self::role_of(lane) else {
                continue;
            };
            let left_rise = move |station: f64| self.edge_rise(index, index, station);
            let right_rise = move |station: f64| self.edge_rise(index, index + 1, station);

            // A driving surface beside a kerb gives up its outermost strip to the
            // gutter. Which side that is depends on which neighbour is raised, and a
            // lane between two kerbs gives up both.
            let (inner_gutter, outer_gutter) = match role {
                Role::Road => (
                    self.gutter_against(index, Neighbour::Left, rungs),
                    self.gutter_against(index, Neighbour::Right, rungs),
                ),
                _ => (0.0, 0.0),
            };
            // Both gutters come out of one lane, so a lane narrow enough for either to
            // be clamped has to be checked against their sum rather than against each.
            let (inner_gutter, outer_gutter) = share(inner_gutter, outer_gutter);

            if inner_gutter > 0.0 {
                self.band(
                    rungs,
                    |cuts| cuts[index],
                    move |cuts| cuts[index] - inner_gutter,
                    left_rise,
                    left_rise,
                    Role::Gutter,
                    materials::ASPHALT,
                    map_name,
                    ordinals,
                    out,
                );
            }
            if outer_gutter > 0.0 {
                self.band(
                    rungs,
                    move |cuts| cuts[index + 1] + outer_gutter,
                    |cuts| cuts[index + 1],
                    right_rise,
                    right_rise,
                    Role::Gutter,
                    materials::ASPHALT,
                    map_name,
                    ordinals,
                    out,
                );
            }
            // A road's surface is what its pavement says, stretch by stretch: a
            // change of pavement starts a band of its own at the rung nearest it.
            for (stretch, surface) in self.stretches(rungs, material) {
                self.band(
                    stretch,
                    move |cuts| cuts[index] - inner_gutter,
                    move |cuts| cuts[index + 1] + outer_gutter,
                    left_rise,
                    right_rise,
                    role,
                    surface,
                    map_name,
                    ordinals,
                    out,
                );
            }

            // The kerb: a vertical face at the cut this lane shares with a neighbour
            // that sits lower. Written by the raised lane so that it is written once.
            for side in [Neighbour::Left, Neighbour::Right] {
                if self.kerb_against(index, side).is_none() {
                    continue;
                }
                let (cut, neighbour) = match side {
                    Neighbour::Left => (index, index - 1),
                    Neighbour::Right => (index + 1, index + 1),
                };
                // From this lane's edge down to its neighbour's, both as the IR
                // lifts them at each station.
                let high = move |station: f64| self.edge_rise(index, cut, station);
                let low = move |station: f64| self.edge_rise(neighbour, cut, station);
                // Anticlockwise seen from the road: the low rail on the outside of
                // the face, so the kerb is visible from the carriageway rather than
                // from inside the pavement.
                let (first, second): (Lift, Lift) = match side {
                    Neighbour::Left => (Box::new(low), Box::new(high)),
                    Neighbour::Right => (Box::new(high), Box::new(low)),
                };
                self.band(
                    rungs,
                    move |cuts| cuts[cut],
                    move |cuts| cuts[cut],
                    first,
                    second,
                    Role::Curb,
                    materials::CURB,
                    map_name,
                    ordinals,
                    out,
                );
            }
        }
    }

    /// The rungs split where the road's pavement changes, each stretch with the
    /// material it is surfaced in. A lane whose material is not the road's — a
    /// pavement, a kerb — keeps `material` and is one stretch.
    fn stretches<'r>(
        &self,
        rungs: &'r [Cross<'r>],
        material: usize,
    ) -> Vec<(&'r [Cross<'r>], usize)> {
        if material != materials::ASPHALT {
            return vec![(rungs, material)];
        }
        let surface = |cross: &Cross| materials::road(self.pavement.at(cross.rung.station));
        let mut stretches = Vec::new();
        let mut start = 0;
        for index in 1..rungs.len() {
            if surface(&rungs[index]) != surface(&rungs[start]) {
                // The rung where the change is shared, so the two bands meet.
                stretches.push((&rungs[start..=index], surface(&rungs[start])));
                start = index;
            }
        }
        if rungs.len() - start >= 2 {
            stretches.push((&rungs[start..], surface(&rungs[start])));
        }
        stretches
    }

    /// How wide a gutter this lane gives up on one side: the configured width, if the
    /// neighbour on that side is raised and there is room for it.
    ///
    /// Measured at every station the lane is sampled at rather than at its two ends,
    /// because a lane's width profile is not monotone over a section — a lane can
    /// widen out of a junction and narrow again into the next one, and its narrowest
    /// point is then in the middle. A gutter wider than the lane it came out of turns
    /// the carriageway band inside out, which is a road surface with its triangles
    /// facing down.
    fn gutter_against(&self, index: usize, side: Neighbour, rungs: &[Cross]) -> f64 {
        if self.kerb_against_neighbour(index, side).is_none() {
            return 0.0;
        }
        let narrowest = rungs
            .iter()
            .map(|cross| self.lanes[index].width_at(cross.rung.station))
            .fold(f64::INFINITY, f64::min);
        self.config.gutter_width.min(narrowest / 2.0).max(0.0)
    }

    /// How far a kerb written by lane `index` on `side` drops, if there is one.
    fn kerb_against(&self, index: usize, side: Neighbour) -> Option<f64> {
        let drop = self.kerb_against_neighbour(index, side)?;
        // Only the raised side writes the face, so it is written once.
        (drop > 0.0).then_some(drop)
    }

    /// The height difference between lane `index` and its neighbour on `side`,
    /// positive when this lane is the higher one. `None` when there is no neighbour
    /// or the two are level.
    fn kerb_against_neighbour(&self, index: usize, side: Neighbour) -> Option<f64> {
        let neighbour = match side {
            Neighbour::Left => index.checked_sub(1)?,
            Neighbour::Right => index + 1,
        };
        let other = self.lanes.get(neighbour)?;
        // A neighbour with no surface is not a neighbour: the strip beyond the
        // outermost boundary is where the verge starts, not where a kerb does.
        Self::role_of(other)?;
        let difference = self.rise_of(self.lanes[index]) - self.rise_of(other);
        (difference.abs() > 1e-9).then_some(difference)
    }

    /// Adds one band as a mesh of its own.
    ///
    /// A mesh per band rather than one per road: CARLA classifies, moves and tags a
    /// whole static mesh at a time, so two classes in one mesh is two classes CARLA
    /// cannot tell apart.
    #[allow(clippy::too_many_arguments)]
    fn band(
        &self,
        rungs: &[Cross],
        left: impl Fn(&[f64]) -> f64,
        right: impl Fn(&[f64]) -> f64,
        left_rise: impl Fn(f64) -> f64,
        right_rise: impl Fn(f64) -> f64,
        role: Role,
        material: usize,
        map_name: &str,
        ordinals: &mut Ordinals,
        out: &mut Vec<Mesh>,
    ) {
        let mut left_rail = Vec::with_capacity(rungs.len());
        let mut right_rail = Vec::with_capacity(rungs.len());
        for cross in rungs {
            let station = cross.rung.station;
            left_rail.push(cross.rung.at(left(&cross.cuts), left_rise(station)));
            right_rail.push(cross.rung.at(right(&cross.cuts), right_rise(station)));
        }
        let mut mesh = Mesh::new(
            mesh_name(map_name, role, ordinals.take(role)),
            role,
            material,
        );
        mesh.strip(&left_rail, &right_rail, 0);
        out.push(mesh);
    }

    /// The paint.
    ///
    /// One mesh per section rather than one per line: a marking mesh may carry both a
    /// white and a yellow slot, and CARLA's material pass wants to find them on the
    /// same mesh.
    fn markings(
        &self,
        rungs: &[Cross],
        map_name: &str,
        config: &SurfaceConfig,
        ordinals: &mut Ordinals,
        out: &mut Vec<Mesh>,
    ) {
        let mut mesh = Mesh::new(
            mesh_name(map_name, Role::Marking, ordinals.take(Role::Marking)),
            Role::Marking,
            materials::MARKING_WHITE,
        );
        for cut in 0..=self.lanes.len() {
            // The boundary between two lanes is one boundary. It is read off the lane
            // to its right, whose `left_marking` it is, and off the last lane's own
            // right-hand marking at the far edge.
            let boundary = match self.lanes.get(cut) {
                Some(lane) => lane.left_marking,
                None => self.lanes[cut - 1].right_marking,
            };
            let lines: &[f64] = match boundary.marking {
                RoadMarking::None | RoadMarking::Curbstone => continue,
                RoadMarking::Solid | RoadMarking::Broken => &[0.0],
                // A double line: two lines, a line's width apart.
                RoadMarking::SolidSolid | RoadMarking::BrokenSolid | RoadMarking::SolidBroken => {
                    &[1.0, -1.0]
                }
            };
            let slot = mesh.slot_for(match boundary.color {
                MarkingColor::White => materials::MARKING_WHITE,
                MarkingColor::Yellow => materials::MARKING_YELLOW,
            });
            for (index, side) in lines.iter().enumerate() {
                let shift = side * config.marking_width;
                // On the surface the line is painted on: the higher of the two lanes
                // either side of it, as the IR lifts them, at each station.
                let lift = |station: f64| {
                    let left = cut
                        .checked_sub(1)
                        .map(|index| self.edge_rise(index, cut, station));
                    let right = (cut < self.lanes.len()).then(|| self.edge_rise(cut, cut, station));
                    config.marking_rise + left.into_iter().chain(right).fold(0.0_f64, f64::max)
                };
                let rail = self.rail(rungs, cut, lift, shift);
                // In `broken solid` the left-hand line is the broken one, which is
                // the order OpenDRIVE and the IR both read a double line in.
                let broken = match boundary.marking {
                    RoadMarking::Broken => true,
                    RoadMarking::BrokenSolid => index == 0,
                    RoadMarking::SolidBroken => index == 1,
                    _ => false,
                };
                paint(&mut mesh, &rail, config, broken, slot);
            }
        }
        out.push(mesh);
    }

    /// The road's two edges as the land meets them, and the kerb faces the land
    /// does not.
    ///
    /// Each side's flank is the outermost surfaced band's edge — a `none` lane
    /// carries no surface, so the land starts where the surface does — with the
    /// verge's toe `verge_width` out from it on a side that has land, and no toe
    /// for a verge of no width. `facing` is
    /// where the junction this road runs through is, if it runs through one. A road
    /// in a junction has land on one side of it at most, the side that faces away
    /// from the junction; the other side has the junction's other roads, and a
    /// raised band there gets a kerb face down to the road level instead of a
    /// verge, which the land then meets at the foot. `None` when nothing on the
    /// section is surfaced.
    fn flanks(
        &self,
        rungs: &[Cross],
        facing: Option<Point3>,
        map_name: &str,
        ordinals: &mut Ordinals,
        out: &mut Vec<Mesh>,
    ) -> Option<[crate::terrain::Flank; 2]> {
        let first = self
            .lanes
            .iter()
            .position(|lane| Self::role_of(lane).is_some())?;
        let last = self
            .lanes
            .iter()
            .rposition(|lane| Self::role_of(lane).is_some())?;
        let edges = [
            (first, self.rise_of(self.lanes[first])),
            (last + 1, self.rise_of(self.lanes[last])),
        ];
        // Each edge's lift station by station, as the lane's surface has it.
        let edge_rise = |side: usize, station: f64| match side {
            0 => self.edge_rise(first, first, station),
            _ => self.edge_rise(last, last + 1, station),
        };
        // Which side the land is on, when only one is: the side whose edge, halfway
        // along, is farther from the junction.
        let land = match facing {
            None => [true, true],
            Some(centre) => {
                let middle = &rungs[rungs.len() / 2];
                let [left, right] = edges.map(|(cut, rise)| {
                    middle
                        .rung
                        .at(middle.cuts[cut], rise)
                        .horizontal_distance_to(centre)
                });
                [left >= right, right > left]
            }
        };
        let width = self.config.verge_width;

        let flanks = [0, 1].map(|side| {
            let (cut, rise) = edges[side];
            // Outwards: to the left on the left side, the right on the right.
            let sign = if side == 0 { 1.0 } else { -1.0 };
            let mut rail = Vec::with_capacity(rungs.len());
            let mut toe = Vec::with_capacity(if land[side] { rungs.len() } else { 0 });
            for cross in rungs {
                let edge = cross.cuts[cut];
                let rail_rise = if land[side] {
                    edge_rise(side, cross.rung.station)
                } else {
                    0.0
                };
                rail.push(cross.rung.at(edge, rail_rise));
                if land[side] && width > 0.0 {
                    toe.push(cross.rung.at(edge + sign * width, 0.0));
                }
            }
            if !land[side] && rise > 0.0 {
                // The kerb face, anticlockwise seen from the junction: the low
                // rail on the outside of the face.
                let lift = move |station: f64| edge_rise(side, station);
                let ground = |_: f64| 0.0;
                let (first, second): (Lift, Lift) = if side == 0 {
                    (Box::new(ground), Box::new(lift))
                } else {
                    (Box::new(lift), Box::new(ground))
                };
                self.band(
                    rungs,
                    move |cuts| cuts[cut],
                    move |cuts| cuts[cut],
                    first,
                    second,
                    Role::Curb,
                    materials::CURB,
                    map_name,
                    ordinals,
                    out,
                );
            }
            crate::terrain::Flank { rail, toe }
        });
        Some(flanks)
    }

    /// The line a cut traces through space, with the lateral direction at each point.
    ///
    /// The direction comes along so that a dash can be cut anywhere between two
    /// stations: a painted line has to be given a width perpendicular to itself, and
    /// halfway through a rung there is no frame to ask.
    fn rail(
        &self,
        rungs: &[Cross],
        cut: usize,
        rise: impl Fn(f64) -> f64,
        shift: f64,
    ) -> Vec<(Point3, Vector3)> {
        rungs
            .iter()
            .map(|cross| {
                (
                    cross
                        .rung
                        .at(cross.cuts[cut] + shift, rise(cross.rung.station)),
                    cross.rung.frame.left.get(),
                )
            })
            .collect()
    }
}

/// Two gutters out of one lane, cut back until they leave some lane behind them.
///
/// Each is already at most half the narrowest the lane gets, so their sum is at most
/// the whole of it — which is a carriageway of nothing between two gutters. Halving
/// both when they would meet leaves a quarter of the lane as carriageway, which is
/// little but is still a road rather than an edge case.
fn share(inner: f64, outer: f64) -> (f64, f64) {
    if inner > 0.0 && outer > 0.0 {
        (inner / 2.0, outer / 2.0)
    } else {
        (inner, outer)
    }
}

/// Which way along the cross-section a neighbour lies.
#[derive(Debug, Clone, Copy, PartialEq)]
enum Neighbour {
    Left,
    Right,
}

/// Lays a painted line along `rail`, whole or in dashes.
fn paint(
    mesh: &mut Mesh,
    rail: &[(Point3, Vector3)],
    config: &SurfaceConfig,
    broken: bool,
    slot: usize,
) {
    let half = config.marking_width / 2.0;
    let widen = |span: &[(Point3, Vector3)]| {
        let left: Vec<Point3> = span
            .iter()
            .map(|(point, across)| *point + *across * half)
            .collect();
        let right: Vec<Point3> = span
            .iter()
            .map(|(point, across)| *point - *across * half)
            .collect();
        (left, right)
    };

    if !broken {
        let (left, right) = widen(rail);
        mesh.strip(&left, &right, slot);
        return;
    }
    for span in dashes(rail, config.dash_on, config.dash_off) {
        let (left, right) = widen(&span);
        mesh.strip(&left, &right, slot);
    }
}

/// Cuts a line into painted spans of `on` metres every `on + off`.
///
/// The spans are cut by arc length rather than by vertex, because the vertices are
/// wherever the curve sampler put them and a dash that started at one would be a dash
/// whose length depended on the road's curvature. Both the position and the lateral
/// direction are interpolated at a cut, so a dash ending mid-curve still has a width
/// perpendicular to the line it is part of.
fn dashes(rail: &[(Point3, Vector3)], on: f64, off: f64) -> Vec<Vec<(Point3, Vector3)>> {
    let pitch = on + off;
    if rail.len() < 2 || on <= 0.0 || pitch <= 0.0 {
        return Vec::new();
    }
    // Where each vertex falls along the line.
    let mut lengths = Vec::with_capacity(rail.len());
    let mut total = 0.0;
    lengths.push(0.0);
    for pair in rail.windows(2) {
        total += pair[0].0.distance_to(pair[1].0);
        lengths.push(total);
    }
    if total <= 0.0 {
        return Vec::new();
    }

    let at = |distance: f64| -> (Point3, Vector3) {
        let index = match lengths.iter().rposition(|&length| length <= distance) {
            Some(index) if index + 1 < rail.len() => index,
            Some(_) => rail.len() - 2,
            None => 0,
        };
        let (from, to) = (lengths[index], lengths[index + 1]);
        let t = if to > from {
            ((distance - from) / (to - from)).clamp(0.0, 1.0)
        } else {
            0.0
        };
        let point = rail[index].0.lerp(rail[index + 1].0, t);
        let across = rail[index].1 + (rail[index + 1].1 - rail[index].1) * t;
        (
            point,
            across
                .normalize()
                .map(|unit| unit.get())
                .unwrap_or(rail[index].1),
        )
    };

    let mut spans = Vec::new();
    // Centred on the line, so that a road reads the same from either end and neither
    // end begins with a stub of paint.
    let count = (total / pitch).floor().max(1.0);
    let start = (total - (count * pitch - off)).max(0.0) / 2.0;
    let mut from = start;
    while from < total - 1e-9 {
        let to = (from + on).min(total);
        let mut span = vec![at(from)];
        for (index, &length) in lengths.iter().enumerate() {
            if length > from + 1e-9 && length < to - 1e-9 {
                span.push(rail[index]);
            }
        }
        span.push(at(to));
        if span.len() >= 2 {
            spans.push(span);
        }
        from += pitch;
    }
    spans
}

/// The town, as meshes.
///
/// One mesh per building rather than one per part: a building is what a viewer of the
/// map points at, and CARLA moves and tags a whole static mesh at a time anyway.
///
/// Walls take brick or plaster by what the building is for, and a pitched roof takes
/// tiles. A flat roof does not: it is the top of the volume rather than a covering,
/// and tiles laid flat read as a mistake.
pub fn buildings(map: &Map, map_name: &str, ordinals: &mut Ordinals) -> Vec<Mesh> {
    let mut meshes = Vec::new();
    for building in map.buildings.iter() {
        let mut mesh = Mesh::new(
            mesh_name(map_name, Role::Building, ordinals.take(Role::Building)),
            Role::Building,
            wall_material(&building.kind),
        );
        let walls = mesh.materials[0];
        let parts = map.parts_of(&building.id);
        // The door goes on the part that stands on the ground, which is the lowest
        // one; a tower on a podium is entered through the podium.
        let ground = parts
            .iter()
            .map(|part| part.solid.base_height())
            .fold(f64::INFINITY, f64::min);
        for part in parts {
            let solid = &part.solid;
            let faces = solid.shell();
            // `Solid::shell` returns the base, then one quad per wall, then the roof:
            // the split is the footprint's own vertex count, which is exact where
            // reading it back off the normals would guess at a steep pitch.
            let sides = solid.footprint.len();
            let roof_material = if solid.roof.height > 0.0 {
                materials::ROOF_TILES
            } else {
                walls
            };
            let street = if solid.base_height() <= ground + 1e-6 {
                facades::street_wall(map, building, part)
            } else {
                None
            };
            for (index, face) in faces.iter().enumerate() {
                let material = if index > sides { roof_material } else { walls };
                let slot = mesh.slot_for(material);
                mesh.face(face, slot);
                // Walls are faces 1..=sides; each gets its windows, and the one that
                // faces the street its door.
                if (1..=sides).contains(&index) {
                    let kind = part.kind.as_deref().unwrap_or(&building.kind);
                    let door = street == Some(index - 1);
                    for opening in facades::openings(face, part.levels, kind, door) {
                        let slot = mesh.slot_for(opening.material);
                        mesh.face(&opening.ring, slot);
                    }
                }
            }
        }
        if !mesh.is_empty() {
            meshes.push(mesh);
        }
    }
    meshes
}

/// What a building of this kind is built of.
///
/// Two materials, chosen by the words the shape grammar's presets emit. A kind the
/// grammar made up gets brick, because a town of unrecognised buildings should look
/// like a town rather than like an error.
fn wall_material(kind: &str) -> usize {
    match kind {
        "retail" | "apartments" | "office" | "industrial" | "warehouse" => materials::PLASTER,
        _ => materials::BRICK,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use roadgen_core::geometry::Vector3;

    fn rail(count: usize, spacing: f64) -> Vec<(Point3, Vector3)> {
        (0..count)
            .map(|index| {
                (
                    Point3::new(index as f64 * spacing, 0.0, 0.0),
                    Vector3::new(0.0, 1.0, 0.0),
                )
            })
            .collect()
    }

    /// Four arms with a pavement each side, joined through a junction, so that each
    /// corner gets a pavement round it.
    fn crossroads() -> roadgen_core::validation::ValidatedMap {
        use roadgen_core::prelude::*;
        let mut builder = MapBuilder::new(MapMetadata {
            name: Some("x".into()),
            ..MapMetadata::default()
        });
        let width = |w| PositiveWidth::new(w).unwrap();
        let street = || {
            vec![
                LaneSpec::new(width(3.5), Direction::Backward),
                LaneSpec::new(width(2.0), Direction::Backward).with_type(LaneType::Sidewalk),
                LaneSpec::new(width(3.5), Direction::Forward),
                LaneSpec::new(width(2.0), Direction::Forward).with_type(LaneType::Sidewalk),
            ]
        };
        let junction = builder.add_junction(Some("x"));
        let mut arm = |from: (f64, f64), to: (f64, f64)| {
            builder
                .add_road(
                    RoadSpec::line(
                        Point3::new(from.0, from.1, 0.0),
                        Point3::new(to.0, to.1, 0.0),
                        street(),
                    )
                    .unwrap(),
                )
                .unwrap()
        };
        let w = arm((-60.0, 0.0), (-12.0, 0.0));
        let e = arm((12.0, 0.0), (60.0, 0.0));
        let n = arm((0.0, 12.0), (0.0, 60.0));
        let s = arm((0.0, -60.0), (0.0, -12.0));
        for (from, to) in [(&w, &e), (&s, &n), (&w, &n), (&s, &e)] {
            builder.connect_via(&junction, from, to).unwrap();
        }
        builder.finish().unwrap().validate().unwrap()
    }

    /// A straight road 300 m long, on a bridge from 100 m to 200 m if `bridged`.
    fn straight(bridged: bool) -> roadgen_core::validation::ValidatedMap {
        use roadgen_core::prelude::*;
        use roadgen_core::structure::BridgeSpec;
        let mut builder = MapBuilder::default();
        let road = builder
            .add_road(
                RoadSpec::line(
                    Point3::new(0.0, 0.0, 10.0),
                    Point3::new(300.0, 0.0, 10.0),
                    vec![LaneSpec::new(
                        PositiveWidth::new(3.5).unwrap(),
                        Direction::Forward,
                    )],
                )
                .unwrap(),
            )
            .unwrap();
        if bridged {
            builder
                .add_bridge(
                    vec![roadgen_core::structure::RoadSpan::new(road, 100.0, 200.0)],
                    BridgeSpec::default(),
                )
                .unwrap();
        }
        builder.finish().unwrap().validate().unwrap()
    }

    /// Where along the road its surface has vertices.
    fn stations(meshes: &[Mesh]) -> Vec<f64> {
        let mut stations: Vec<f64> = meshes
            .iter()
            .filter(|mesh| mesh.role == Role::Road)
            .flat_map(|mesh| mesh.positions.iter().map(|point| point.x))
            .collect();
        stations.sort_by(f64::total_cmp);
        stations.dedup_by(|a, b| (*a - *b).abs() < 1e-6);
        stations
    }

    #[test]
    fn a_taper_shorter_than_the_land_keeps_two_vertices_apart_still_ends_where_it_says() {
        // Rungs a hand's breadth apart are one, but not the two ends of a lane's
        // width changing: the road would widen to the next rung instead.
        use roadgen_core::prelude::*;
        let taper = WidthProfile::tapered(
            100.0,
            0.6,
            PositiveWidth::new(3.5).unwrap(),
            PositiveWidth::new(4.0).unwrap(),
            Taper::Linear,
        )
        .unwrap();
        let mut builder = MapBuilder::default();
        builder
            .add_road(
                RoadSpec::line(
                    Point3::new(0.0, 0.0, 10.0),
                    Point3::new(300.0, 0.0, 10.0),
                    vec![
                        LaneSpec::new(PositiveWidth::new(3.5).unwrap(), Direction::Forward)
                            .with_width_profile(taper),
                    ],
                )
                .unwrap(),
            )
            .unwrap();
        let map = builder.finish().unwrap().validate().unwrap();
        let at = stations(&build(&map, "m", &SurfaceConfig::default()));
        for knot in [100.0, 100.6] {
            assert!(
                at.iter().any(|x| (x - knot).abs() < 1e-6),
                "no rung at {knot}: {at:?}"
            );
        }
    }

    #[test]
    fn a_road_gets_rungs_only_where_the_ground_meets_its_edge() {
        // A rung is a dozen triangles across the road, there for the land beside
        // it to meet. On the ground it comes every two of the ground's cells.
        let config = SurfaceConfig::default();
        let gaps = |stations: &[f64]| {
            stations
                .windows(2)
                .map(|pair| pair[1] - pair[0])
                .fold(0.0, f64::max)
        };
        let at_grade = stations(&build(&straight(false), "m", &config));
        assert!(
            gaps(&at_grade) <= 2.0 * config.ground_cell + 1e-6,
            "{at_grade:?}"
        );
        // On a bridge there is no land beside it: the deck's only rungs are its ends.
        let bridged = stations(&build(&straight(true), "m", &config));
        let on_deck: Vec<f64> = bridged
            .iter()
            .copied()
            .filter(|x| *x > 100.0 + 1e-6 && *x < 200.0 - 1e-6)
            .collect();
        assert!(on_deck.is_empty(), "rungs on the deck at {on_deck:?}");
        assert!(bridged.iter().any(|x| *x > 1e-6 && *x < 100.0 - 1e-6));
        // And with no ground past the verges there is nothing to fan out to.
        let bare = SurfaceConfig {
            ground_extent: 0.0,
            ..SurfaceConfig::default()
        };
        assert_eq!(stations(&build(&straight(false), "m", &bare)), [0.0, 300.0]);
    }

    #[test]
    fn no_land_lies_over_the_roads_through_a_junction() {
        // A corner pavement is raised, and the land that slopes down from it goes
        // on the land side of it: on the junction side it would lie across the
        // junction's roads. There the pavement gets a kerb face down to the road
        // instead, and the land meets its foot.
        let map = crossroads();
        let config = SurfaceConfig {
            ground_extent: 20.0,
            ground_cell: 5.0,
            ..SurfaceConfig::default()
        };
        let meshes = build(&map, "x", &config);
        let land = meshes
            .iter()
            .find(|mesh| mesh.role == Role::Terrain)
            .expect("the land");
        let mut over = 0;
        for point in &land.positions {
            if point.x.abs() < 6.0 && point.y.abs() < 6.0 && point.z > 1e-9 {
                over += 1;
            }
        }
        assert_eq!(over, 0, "{over} land vertices above the junction's roads");
        // And nothing of it on the arms' carriageways either: no land triangle is
        // centred on a lane.
        for triangle in &land.triangles {
            let [a, b, c] = triangle.map(|index| land.positions[index as usize]);
            let (x, y) = ((a.x + b.x + c.x) / 3.0, (a.y + b.y + c.y) / 3.0);
            let arm = 12.0..60.0;
            let on_an_arm = (arm.contains(&x.abs()) && y.abs() < 5.5 - 1e-9)
                || (arm.contains(&y.abs()) && x.abs() < 5.5 - 1e-9);
            assert!(!on_an_arm, "land at ({x}, {y}) on an arm");
        }
        // The pavements do get their kerb face, so they are not left floating.
        assert!(meshes.iter().any(|mesh| mesh.role == Role::Curb
            && mesh
                .positions
                .iter()
                .any(|point| point.x.abs() < 10.0 && point.y.abs() < 10.0)));
    }

    #[test]
    fn the_land_has_no_ledge_in_it() {
        // Every slope in the land is a verge's: a kerb's height over a verge's
        // width, more or less. A land triangle steeper than that is a step — the
        // kind a shortened toe leaves round a corner pavement, where the land
        // dropped a kerb's height in a hand's width.
        let map = crossroads();
        let meshes = build(&map, "x", &SurfaceConfig::default());
        let land = meshes
            .iter()
            .find(|mesh| mesh.role == Role::Terrain)
            .expect("the land");
        let mut steepest = 0.0_f64;
        for triangle in &land.triangles {
            let [a, b, c] = triangle.map(|index| land.positions[index as usize]);
            let normal = (b - a).cross(c - a);
            let length = normal.norm();
            if length < 1e-9 {
                continue;
            }
            // The gradient is the tangent of the angle the normal makes with up.
            let gradient = (normal.x * normal.x + normal.y * normal.y).sqrt() / normal.z;
            steepest = steepest.max(gradient);
        }
        assert!(
            steepest < 0.15,
            "the land has a slope of {:.0} % in it",
            steepest * 100.0
        );
    }

    #[test]
    fn the_land_is_one_surface_that_faces_up_everywhere() {
        // Round a corner pavement the land is on the inside of the bend, and the
        // bend is tighter than the verge is wide. Still one mesh, and every
        // triangle of it faces up: nothing has folded over.
        let map = crossroads();
        let meshes = build(&map, "x", &SurfaceConfig::default());
        let land: Vec<&Mesh> = meshes
            .iter()
            .filter(|mesh| mesh.role == Role::Terrain)
            .collect();
        assert_eq!(land.len(), 1, "the land is in more than one piece");
        for triangle in &land[0].triangles {
            let [a, b, c] = triangle.map(|index| land[0].positions[index as usize]);
            let normal = (b - a).cross(c - a);
            assert!(
                normal.z > 0.0,
                "a land triangle {a:?} {b:?} {c:?} faces down"
            );
        }
    }

    #[test]
    fn a_dash_is_as_long_as_it_was_asked_to_be() {
        // Cut by arc length, not by vertex: a three-metre dash is three metres
        // however far apart the curve sampler put its vertices.
        let spans = dashes(&rail(101, 1.0), 3.0, 6.0);
        assert!(!spans.is_empty());
        for span in &spans {
            let length: f64 = span
                .windows(2)
                .map(|pair| pair[0].0.distance_to(pair[1].0))
                .sum();
            assert!(
                (length - 3.0).abs() < 1e-6,
                "a dash {length} metres long, not 3"
            );
        }
    }

    #[test]
    fn dashes_are_centred_so_a_line_reads_the_same_from_either_end() {
        let total = 100.0;
        let spans = dashes(&rail(101, 1.0), 3.0, 6.0);
        let first = spans.first().unwrap().first().unwrap().0.x;
        let last = total - spans.last().unwrap().last().unwrap().0.x;
        assert!((first - last).abs() < 1e-6, "{first} in, {last} out");
    }

    #[test]
    fn a_line_shorter_than_one_dash_still_gets_paint() {
        // A connector through a junction can be two metres long. Emitting nothing
        // there leaves a gap in a line that runs through the junction.
        let spans = dashes(&rail(3, 1.0), 3.0, 6.0);
        assert_eq!(spans.len(), 1);
    }

    #[test]
    fn a_line_of_no_length_is_not_painted() {
        let flat = vec![
            (Point3::ORIGIN, Vector3::new(0.0, 1.0, 0.0)),
            (Point3::ORIGIN, Vector3::new(0.0, 1.0, 0.0)),
        ];
        assert!(dashes(&flat, 3.0, 6.0).is_empty());
    }

    #[test]
    fn two_gutters_out_of_one_lane_leave_some_lane_behind_them() {
        // Each is already at most half the lane, so together they would be all of it.
        assert_eq!(share(0.3, 0.3), (0.15, 0.15));
        // One gutter is not shared with anything.
        assert_eq!(share(0.3, 0.0), (0.3, 0.0));
        assert_eq!(share(0.0, 0.0), (0.0, 0.0));
    }

    #[test]
    fn an_ordinal_counts_once_per_role() {
        let mut ordinals = Ordinals::default();
        assert_eq!(ordinals.take(Role::Road), 0);
        assert_eq!(ordinals.take(Role::Marking), 0);
        assert_eq!(ordinals.take(Role::Road), 1);
        assert_eq!(ordinals.take(Role::Road), 2);
    }
}
