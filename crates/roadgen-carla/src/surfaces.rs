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
//! # Where the ground stops
//!
//! Beside the outermost band is a **verge**: grass, for as far as the road network can
//! honestly say anything about the land. It follows the road's own elevation, so a
//! road on a grade has ground beside it at the right height.
//!
//! There is no landscape past the verge, and there should not be. A road network says
//! nothing about the shape of the country it runs through, and a generator that
//! produced hills here would be inventing a terrain model rather than deriving one.
//! [`crate::check`] says so rather than leaving it to be discovered in the editor.

use roadgen_core::geometry::{Frame3, Point3, Vector3};
use roadgen_core::map::{Map, Road};
use roadgen_core::semantics::{LaneType, MarkingColor, RoadMarking};
use roadgen_core::topology::LateralSide;

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
    /// How far the grass beside a road reaches. Zero writes no verge.
    pub verge_width: f64,
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
    for road in map.roads.iter() {
        let Some(rungs) = rungs(map, road) else {
            continue;
        };
        for section in 0..road.sections.len() {
            let Some(layout) = Layout::of(map, road, section, config) else {
                continue;
            };
            let within: Vec<&Rung> = rungs
                .iter()
                .filter(|rung| layout.covers(rung.station))
                .collect();
            if within.len() < 2 {
                continue;
            }
            layout.surfaces(&within, map_name, &mut ordinal, &mut meshes);
            layout.markings(&within, map_name, config, &mut ordinal, &mut meshes);
            layout.verges(&within, map_name, config, &mut ordinal, &mut meshes);
        }
    }
    meshes.retain(|mesh| !mesh.is_empty());
    meshes
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

/// One station of a road, with its frame and the lateral cuts of its cross-section.
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

/// Every station a road is sampled at, with its banked frame.
fn rungs(map: &Map, road: &Road) -> Option<Vec<Rung>> {
    let stations = map.vertex_stations(&road.id).ok()?;
    let mut rungs = Vec::with_capacity(stations.len());
    for station in stations {
        let Ok(sample) = road
            .reference_line
            .sample_at(station, map.metadata.sampling)
        else {
            continue;
        };
        let Ok(frame) = sample.frame() else {
            continue;
        };
        rungs.push(Rung {
            station,
            frame: frame.banked(road.superelevation.evaluate(station)),
        });
    }
    (rungs.len() >= 2).then_some(rungs)
}

/// What one cross-section of one road is made of, left to right.
struct Layout<'a> {
    range: (f64, f64),
    /// The road's lateral offset profile, which the cuts are measured from.
    lane_offset: &'a roadgen_core::geometry::Poly3Profile,
    /// The lanes, from the leftmost to the rightmost.
    lanes: Vec<&'a roadgen_core::map::Lane>,
    config: SurfaceConfig,
}

impl<'a> Layout<'a> {
    fn of(
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
        })
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

    fn rise_of(&self, lane: &roadgen_core::map::Lane) -> f64 {
        match lane.lane_type {
            LaneType::Sidewalk => self.config.kerb_height,
            _ => 0.0,
        }
    }

    /// The drivable surface, the pavements, the kerbs and the gutters.
    fn surfaces(
        &self,
        rungs: &[&Rung],
        map_name: &str,
        ordinals: &mut Ordinals,
        out: &mut Vec<Mesh>,
    ) {
        for (index, lane) in self.lanes.iter().enumerate() {
            let Some((role, material)) = Self::role_of(lane) else {
                continue;
            };
            let rise = self.rise_of(lane);

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
                    rise,
                    rise,
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
                    rise,
                    rise,
                    Role::Gutter,
                    materials::ASPHALT,
                    map_name,
                    ordinals,
                    out,
                );
            }
            self.band(
                rungs,
                move |cuts| cuts[index] - inner_gutter,
                move |cuts| cuts[index + 1] + outer_gutter,
                rise,
                rise,
                role,
                material,
                map_name,
                ordinals,
                out,
            );

            // The kerb: a vertical face at the cut this lane shares with a neighbour
            // that sits lower. Written by the raised lane so that it is written once.
            for side in [Neighbour::Left, Neighbour::Right] {
                let Some(drop) = self.kerb_against(index, side) else {
                    continue;
                };
                let cut = match side {
                    Neighbour::Left => index,
                    Neighbour::Right => index + 1,
                };
                // Anticlockwise seen from the road: the low rail on the outside of
                // the face, so the kerb is visible from the carriageway rather than
                // from inside the pavement.
                let (low, high) = match side {
                    Neighbour::Left => (rise - drop, rise),
                    Neighbour::Right => (rise, rise - drop),
                };
                self.band(
                    rungs,
                    move |cuts| cuts[cut],
                    move |cuts| cuts[cut],
                    low,
                    high,
                    Role::Curb,
                    materials::CURB,
                    map_name,
                    ordinals,
                    out,
                );
            }
        }
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
    fn gutter_against(&self, index: usize, side: Neighbour, rungs: &[&Rung]) -> f64 {
        if self.kerb_against_neighbour(index, side).is_none() {
            return 0.0;
        }
        let narrowest = rungs
            .iter()
            .map(|rung| self.lanes[index].width_at(rung.station))
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
        rungs: &[&Rung],
        left: impl Fn(&[f64]) -> f64,
        right: impl Fn(&[f64]) -> f64,
        left_rise: f64,
        right_rise: f64,
        role: Role,
        material: usize,
        map_name: &str,
        ordinals: &mut Ordinals,
        out: &mut Vec<Mesh>,
    ) {
        let mut left_rail = Vec::with_capacity(rungs.len());
        let mut right_rail = Vec::with_capacity(rungs.len());
        for rung in rungs {
            let cuts = self.cuts(rung.station);
            left_rail.push(rung.at(left(&cuts), left_rise));
            right_rail.push(rung.at(right(&cuts), right_rise));
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
        rungs: &[&Rung],
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
                let rail = self.rail(rungs, cut, config.marking_rise, shift);
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

    /// The grass beside the road, on each side that has an outermost band to put it
    /// against.
    fn verges(
        &self,
        rungs: &[&Rung],
        map_name: &str,
        config: &SurfaceConfig,
        ordinals: &mut Ordinals,
        out: &mut Vec<Mesh>,
    ) {
        if config.verge_width <= 0.0 {
            return;
        }
        // Outermost *surfaced* band on each side: a `none` lane carries no surface,
        // so the grass starts where it starts rather than beyond it.
        let first = self
            .lanes
            .iter()
            .position(|lane| Self::role_of(lane).is_some());
        let last = self
            .lanes
            .iter()
            .rposition(|lane| Self::role_of(lane).is_some());
        let (Some(first), Some(last)) = (first, last) else {
            return;
        };
        let width = config.verge_width;
        let left_rise = self.rise_of(self.lanes[first]);
        let right_rise = self.rise_of(self.lanes[last]);

        self.band(
            rungs,
            move |cuts| cuts[first] + width,
            move |cuts| cuts[first],
            left_rise,
            left_rise,
            Role::Terrain,
            materials::GRASS,
            map_name,
            ordinals,
            out,
        );
        self.band(
            rungs,
            move |cuts| cuts[last + 1],
            move |cuts| cuts[last + 1] - width,
            right_rise,
            right_rise,
            Role::Terrain,
            materials::GRASS,
            map_name,
            ordinals,
            out,
        );
    }

    /// The line a cut traces through space, with the lateral direction at each point.
    ///
    /// The direction comes along so that a dash can be cut anywhere between two
    /// stations: a painted line has to be given a width perpendicular to itself, and
    /// halfway through a rung there is no frame to ask.
    fn rail(&self, rungs: &[&Rung], cut: usize, rise: f64, shift: f64) -> Vec<(Point3, Vector3)> {
        rungs
            .iter()
            .map(|rung| {
                let cuts = self.cuts(rung.station);
                (rung.at(cuts[cut] + shift, rise), rung.frame.left.get())
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
        for part in building
            .parts
            .iter()
            .filter_map(|id| map.building_parts.get(id))
        {
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
            for (index, face) in faces.iter().enumerate() {
                let material = if index > sides { roof_material } else { walls };
                let slot = mesh.slot_for(material);
                mesh.face(face, slot);
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
