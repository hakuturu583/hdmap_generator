//! The condition of a road's surface: what a wheel feels that the road's design
//! does not say.
//!
//! A road's reference line, its elevation and its superelevation describe the
//! surface it was *built* to. A surface as it is driven departs from that by a few
//! millimetres to a few centimetres, and those departures are what a vehicle
//! dynamics model, a suspension test or a perception stack looking for a puddle
//! needs: the troughs traffic wears into the wheel paths, a hole where the
//! pavement has broken up, standing water, a hump laid on purpose to slow traffic,
//! and the unevenness of the whole surface.
//!
//! Each is a [`SurfaceFeature`]: a kind, a road, and a footprint in that road's
//! own `(s, t)` coordinates — station along the reference line, and metres to the
//! left of it, the same `t` OpenDRIVE uses. What a feature *is* is a function of
//! the footprint: [`SurfaceFeature::displacement`] says how far the surface stands
//! above (positive) or below (negative) the designed surface at any `(s, t)`, and
//! [`SurfaceFeature::water_depth`] how deep water stands on it. Every exporter
//! that writes the surface reads those two functions and nothing else, so an
//! OpenCRG grid and a triangle mesh of the same feature agree by construction.
//!
//! # What the vocabulary is borrowed from
//!
//! - **ASAM OpenCRG 1.2** is the format the IR is meant to be lowered onto: a
//!   regular grid of elevations in a road's `(u, v)` coordinates, attached to an
//!   OpenDRIVE road by **ASAM OpenDRIVE 1.7**'s `<surface>/<CRG>`, whose
//!   `purpose` is either `elevation` or `friction`. That is why a feature is
//!   stated as a displacement *from* the road rather than as an absolute height —
//!   OpenDRIVE's `attached` mode adds the grid to the road it is attached to — and
//!   why standing water is also a change of friction.
//! - **ISO 8608** (road surface profiles) classifies unevenness by the
//!   displacement power spectral density of the profile, `Gd(n) = Gd(n0) ·
//!   (n / n0)^-w` with `n0 = 0.1` cycles/m and `w = 2`, in eight classes from A
//!   (`Gd(n0) = 16·10⁻⁶ m³`) to H, each four times the one before.
//!   [`RoughnessClass`] is that table, and [`SurfaceFeatureKind::Roughness`]
//!   synthesises a surface whose longitudinal profile follows it.
//! - **ASTM E1703** measures a rut as its depth below a straightedge laid across
//!   the wheel path, which is what [`SurfaceFeatureKind::Rut`]'s `depth` is: the
//!   deepest point of each trough below the surface either side of it.
//! - **ASTM D6433** (the Pavement Condition Index) rates a pothole by its depth and
//!   its mean diameter, which is how [`SurfaceFeatureKind::Pothole`] is stated:
//!   a depth and the footprint's two diameters.
//! - **OpenStreetMap**'s `traffic_calming` tag names the devices a road is given on
//!   purpose — `bump`, `hump`, `table`, `cushion` — which [`BumpKind`] follows, and
//!   the profiles a hump is built to (a circular arc, the "Watts" profile; a
//!   parabola; sinusoidal ramps; a flat top between ramps) are [`BumpProfile`].
//!   A frost heave is the same shape raised by nobody's design.
//!
//! None of these is implemented to the letter: there is no ISO 8608 report, no
//! PCI rating. What is taken is the quantities each standard says a feature is
//! measured by, so that a feature written from a survey is stated in the survey's
//! own terms.
//!
//! # A feature's footprint
//!
//! The footprint is a rectangle in `(s, t)`: from `station` for `length` metres
//! along the road, and `width` metres across, centred `offset` metres to the left
//! of the reference line. A pothole and a puddle fill the ellipse inscribed in it;
//! a rut runs its length in two troughs a wheel track apart; a bump spans its
//! width across the road; roughness covers all of it. Nothing outside the
//! footprint is displaced, and every kind meets the designed surface at the edge of
//! its footprint without a step, so features can sit side by side or overlap —
//! where two overlap, their displacements add.

use std::f64::consts::PI;

use crate::id::{LaneId, RoadId, SurfaceFeatureId};

/// A departure of one road's surface from its design, over a footprint in the
/// road's own coordinates. See the [module documentation](self).
#[derive(Debug, Clone, PartialEq)]
pub struct SurfaceFeature {
    pub id: SurfaceFeatureId,
    /// The road whose `(s, t)` the footprint is in.
    pub road: RoadId,
    /// Where the footprint starts along the road, metres of horizontal station.
    pub station: f64,
    /// How far along the road the footprint reaches, metres.
    pub length: f64,
    /// Where the middle of the footprint is across the road, metres to the left of
    /// the reference line.
    pub offset: f64,
    /// How wide the footprint is across the road, metres.
    pub width: f64,
    pub kind: SurfaceFeatureKind,
}

/// What a surface feature is.
#[derive(Debug, Clone, PartialEq)]
pub enum SurfaceFeatureKind {
    /// Wheel-path rutting: two troughs, `track` apart centre to centre, each
    /// `trough` wide and `depth` deep at its deepest, running the footprint's
    /// length and easing in and out over its first and last few metres.
    ///
    /// With no `path`, the troughs run straight along the road down the middle of
    /// the footprint, which is as wide as the two of them, `track + trough`. With
    /// one, they follow it: a line in a lane's own coordinates that may wander
    /// across the lane and out of it — a truck cutting the inside of a bend, a
    /// lane change worn into the surface — and the footprint is whatever bounds
    /// it, which the builder works out once the lane exists. Where the path runs
    /// at an angle to the road, the troughs keep `track` and `trough` across the
    /// *path*, so they stand wider apart across the road.
    Rut {
        depth: f64,
        track: f64,
        trough: f64,
        path: Option<LanePath>,
    },
    /// A hole where the pavement has broken out: an irregular bowl inside the
    /// footprint's ellipse, steep-sided and flat-bottomed, `depth` deep. `seed`
    /// decides the irregularity of its rim, so that two potholes of one size do not
    /// look stamped from one die.
    Pothole { depth: f64, seed: u64 },
    /// Water standing in a shallow depression that fills the footprint's ellipse:
    /// the surface dips `depth` below the design at the middle, and the water
    /// stands level with the design, so `depth` is also the deepest the water gets.
    Puddle { depth: f64 },
    /// A raised hump across the footprint, `height` above the design at its crest,
    /// with the longitudinal profile `profile`. `kind` says what it is for.
    Bump {
        kind: BumpKind,
        height: f64,
        profile: BumpProfile,
    },
    /// Unevenness over the whole footprint, synthesised so that its longitudinal
    /// profile has ISO 8608's spectrum for `class`, reproducibly from `seed`.
    Roughness { class: RoughnessClass, seed: u64 },
}

/// A line along a lane, in the lane's own coordinates: knots of station along
/// the road and offset from the lane's middle, metres to the left, joined by a
/// monotone cubic Hermite curve (PCHIP) — through every knot, smooth, never past
/// one, level between two knots of one offset, and straight between two knots
/// alone.
///
/// Stated against the lane rather than the road so that it stays where it was put
/// when the lanes beside it change width, and so that "a metre in from the lane's
/// edge" is a number a caller can write without laying the road out first.
#[derive(Debug, Clone, PartialEq)]
pub struct LanePath {
    pub lane: LaneId,
    knots: Vec<(f64, f64)>,
}

impl LanePath {
    /// Fails unless there are at least two knots, every number is finite and the
    /// stations strictly ascend.
    pub fn new(lane: LaneId, knots: Vec<(f64, f64)>) -> Result<LanePath, String> {
        if knots.len() < 2 {
            return Err(format!(
                "a path needs two knots at least, and has {}",
                knots.len()
            ));
        }
        for pair in knots.windows(2) {
            if pair[1].0 <= pair[0].0 {
                return Err(format!(
                    "a path's stations must ascend, and {} follows {}",
                    pair[1].0, pair[0].0
                ));
            }
        }
        if let Some(bad) = knots
            .iter()
            .find(|(s, t)| !(s.is_finite() && t.is_finite()))
        {
            return Err(format!("a path has a knot at {bad:?}"));
        }
        Ok(LanePath { lane, knots })
    }

    pub fn knots(&self) -> &[(f64, f64)] {
        &self.knots
    }

    /// The first station and the last.
    pub fn stations(&self) -> (f64, f64) {
        (self.knots[0].0, self.knots[self.knots.len() - 1].0)
    }

    /// The slope the curve has at knot `i`: PCHIP's (Fritsch and Butland's
    /// weighted harmonic mean of the chords either side, and level where the
    /// path turns back or stops drifting), so that a stretch between two knots
    /// of one offset stays at it, and the curve never overshoots a knot.
    fn tangent(&self, i: usize) -> f64 {
        let k = &self.knots;
        let chord = |j: usize| (k[j + 1].1 - k[j].1) / (k[j + 1].0 - k[j].0);
        let width = |j: usize| k[j + 1].0 - k[j].0;
        if i == 0 {
            return chord(0);
        }
        if i == k.len() - 1 {
            return chord(i - 1);
        }
        let (before, after) = (chord(i - 1), chord(i));
        if before * after <= 0.0 {
            return 0.0;
        }
        let (h0, h1) = (width(i - 1), width(i));
        let (w0, w1) = (2.0 * h1 + h0, h1 + 2.0 * h0);
        (w0 + w1) / (w0 / before + w1 / after)
    }

    /// The offset from the lane's middle at `station`, and its rate of change
    /// along the road; held level past either end.
    pub fn offset_at(&self, station: f64) -> (f64, f64) {
        let k = &self.knots;
        let (first, last) = self.stations();
        if station <= first {
            return (k[0].1, 0.0);
        }
        if station >= last {
            return (k[k.len() - 1].1, 0.0);
        }
        let i = k
            .partition_point(|knot| knot.0 <= station)
            .saturating_sub(1);
        let (s0, y0) = k[i];
        let (s1, y1) = k[i + 1];
        let h = s1 - s0;
        let (m0, m1) = (self.tangent(i) * h, self.tangent(i + 1) * h);
        let x = (station - s0) / h;
        let (x2, x3) = (x * x, x * x * x);
        let value = (2.0 * x3 - 3.0 * x2 + 1.0) * y0
            + (x3 - 2.0 * x2 + x) * m0
            + (-2.0 * x3 + 3.0 * x2) * y1
            + (x3 - x2) * m1;
        let slope = ((6.0 * x2 - 6.0 * x) * y0
            + (3.0 * x2 - 4.0 * x + 1.0) * m0
            + (-6.0 * x2 + 6.0 * x) * y1
            + (3.0 * x2 - 2.0 * x) * m1)
            / h;
        (value, slope)
    }
}

/// A path laid out on its road: where its middle is, metres to the left of the
/// reference line, and how fast that changes, sampled along it.
#[derive(Debug, Clone)]
pub struct PathCentre {
    start: f64,
    step: f64,
    samples: Vec<(f64, f64)>,
}

impl PathCentre {
    /// How finely a path is laid out along its road, metres.
    const STEP: f64 = 0.1;

    /// Lays `path` out against its lane on `map`; `None` when the map has no such
    /// lane.
    pub fn of(map: &crate::map::Map, path: &LanePath) -> Option<PathCentre> {
        let (first, last) = path.stations();
        let count = (((last - first) / Self::STEP).ceil() as usize).max(1);
        let step = (last - first) / count as f64;
        let centre = |s: f64| -> Option<f64> {
            Some(map.lane_centre_offset(&path.lane, s)? + path.offset_at(s).0)
        };
        let mut samples = Vec::with_capacity(count + 1);
        for k in 0..=count {
            let s = first + step * k as f64;
            // The lane's own drift — a taper — counts as much as the path's.
            let h = (step / 2.0).max(1e-3);
            let slope = (centre(s + h)? - centre(s - h)?) / (2.0 * h);
            samples.push((centre(s)?, slope));
        }
        Some(PathCentre {
            start: first,
            step,
            samples,
        })
    }

    /// The middle of the path and its slope across the road at `station`.
    pub fn at(&self, station: f64) -> (f64, f64) {
        let x = ((station - self.start) / self.step).clamp(0.0, (self.samples.len() - 1) as f64);
        let i = (x.floor() as usize).min(self.samples.len().saturating_sub(2));
        let f = x - i as f64;
        let (a, b) = (
            self.samples[i],
            self.samples[(i + 1).min(self.samples.len() - 1)],
        );
        (a.0 + (b.0 - a.0) * f, a.1 + (b.1 - a.1) * f)
    }

    /// The lateral range the path's middle sweeps, right then left, and the
    /// steepest it runs across the road.
    fn bounds(&self) -> (f64, f64, f64) {
        self.samples.iter().fold(
            (f64::INFINITY, f64::NEG_INFINITY, 0.0_f64),
            |(lo, hi, steep), &(t, k)| (lo.min(t), hi.max(t), steep.max(k.abs())),
        )
    }
}

/// What a raised feature is, in OpenStreetMap's `traffic_calming` words where it
/// was built on purpose.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum BumpKind {
    /// A short, sharp bump, under a metre long, for car parks and private roads.
    SpeedBump,
    /// A hump a few metres long, taken at 25–30 km/h.
    SpeedHump,
    /// A hump with a flat top long enough for a car's wheelbase.
    SpeedTable,
    /// A hump that leaves gaps for wide-tracked vehicles; the footprint is one
    /// cushion.
    SpeedCushion,
    /// Pavement pushed up from beneath — by frost, by a tree's roots — that nobody
    /// laid.
    Heave,
}

impl BumpKind {
    pub const ALL: [BumpKind; 5] = [
        BumpKind::SpeedBump,
        BumpKind::SpeedHump,
        BumpKind::SpeedTable,
        BumpKind::SpeedCushion,
        BumpKind::Heave,
    ];

    pub fn as_str(self) -> &'static str {
        match self {
            BumpKind::SpeedBump => "bump",
            BumpKind::SpeedHump => "hump",
            BumpKind::SpeedTable => "table",
            BumpKind::SpeedCushion => "cushion",
            BumpKind::Heave => "heave",
        }
    }

    pub fn parse(value: &str) -> Option<BumpKind> {
        let value = value.trim().to_ascii_lowercase().replace([' ', '-'], "_");
        Some(match value.as_str() {
            "bump" | "speed_bump" => BumpKind::SpeedBump,
            "hump" | "speed_hump" => BumpKind::SpeedHump,
            "table" | "speed_table" => BumpKind::SpeedTable,
            "cushion" | "speed_cushion" => BumpKind::SpeedCushion,
            "heave" | "frost_heave" => BumpKind::Heave,
            _ => return None,
        })
    }

    /// The profile a bump of this kind is built to when nobody says otherwise.
    pub fn default_profile(self) -> BumpProfile {
        match self {
            BumpKind::SpeedBump => BumpProfile::Parabolic,
            BumpKind::SpeedHump | BumpKind::SpeedCushion => BumpProfile::Circular,
            BumpKind::SpeedTable => BumpProfile::Flat { ramp: 1.0 },
            BumpKind::Heave => BumpProfile::Sinusoidal,
        }
    }
}

/// The longitudinal shape of a raised feature, from where it leaves the design
/// surface to where it meets it again.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum BumpProfile {
    /// An arc of a circle through both ends and the crest: the "Watts" profile.
    Circular,
    /// A parabola through both ends and the crest.
    Parabolic,
    /// One period of a raised cosine: no kink where it meets the road.
    Sinusoidal,
    /// Ramps `ramp` metres long either end, each a half-period of a raised cosine,
    /// with a flat top between them.
    Flat { ramp: f64 },
}

impl BumpProfile {
    pub fn as_str(self) -> &'static str {
        match self {
            BumpProfile::Circular => "circular",
            BumpProfile::Parabolic => "parabolic",
            BumpProfile::Sinusoidal => "sinusoidal",
            BumpProfile::Flat { .. } => "flat",
        }
    }

    /// How high the profile stands at `along` metres from its start, as a share of
    /// its height, for a profile `length` long.
    fn share(self, along: f64, length: f64, height: f64) -> f64 {
        if !(0.0..=length).contains(&along) || length <= 0.0 {
            return 0.0;
        }
        let x = along / length;
        match self {
            BumpProfile::Circular => {
                // The circle through (0, 0), (L, 0) and (L/2, h).
                let half = length / 2.0;
                let radius = (half * half + height * height) / (2.0 * height);
                let dx = along - half;
                (((radius * radius - dx * dx).max(0.0)).sqrt() - (radius - height)) / height
            }
            BumpProfile::Parabolic => 4.0 * x * (1.0 - x),
            BumpProfile::Sinusoidal => 0.5 * (1.0 - (2.0 * PI * x).cos()),
            BumpProfile::Flat { ramp } => {
                let ramp = ramp.clamp(0.0, length / 2.0);
                let from_end = along.min(length - along);
                if ramp <= 0.0 || from_end >= ramp {
                    1.0
                } else {
                    0.5 * (1.0 - (PI * from_end / ramp).cos())
                }
            }
        }
    }
}

/// ISO 8608's road classes, by the displacement PSD at the reference spatial
/// frequency.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum RoughnessClass {
    A,
    B,
    C,
    D,
    E,
    F,
    G,
    H,
}

impl RoughnessClass {
    pub const ALL: [RoughnessClass; 8] = [
        RoughnessClass::A,
        RoughnessClass::B,
        RoughnessClass::C,
        RoughnessClass::D,
        RoughnessClass::E,
        RoughnessClass::F,
        RoughnessClass::G,
        RoughnessClass::H,
    ];

    /// The reference spatial frequency `n0`, cycles per metre.
    pub const REFERENCE_FREQUENCY: f64 = 0.1;

    /// The waviness `w`: the PSD falls as `n^-w`.
    pub const WAVINESS: f64 = 2.0;

    /// The class's geometric-mean `Gd(n0)`, m³: 16·10⁻⁶ for A, and four times the
    /// class before for each after it.
    pub fn psd_at_reference(self) -> f64 {
        16e-6 * 4f64.powi(self as i32)
    }

    /// The displacement PSD at spatial frequency `n`, m³.
    pub fn psd(self, n: f64) -> f64 {
        self.psd_at_reference() * (n / Self::REFERENCE_FREQUENCY).powf(-Self::WAVINESS)
    }

    pub fn as_str(self) -> &'static str {
        match self {
            RoughnessClass::A => "A",
            RoughnessClass::B => "B",
            RoughnessClass::C => "C",
            RoughnessClass::D => "D",
            RoughnessClass::E => "E",
            RoughnessClass::F => "F",
            RoughnessClass::G => "G",
            RoughnessClass::H => "H",
        }
    }

    pub fn parse(value: &str) -> Option<RoughnessClass> {
        let value = value.trim();
        Self::ALL
            .into_iter()
            .find(|class| class.as_str().eq_ignore_ascii_case(value))
    }
}

impl SurfaceFeatureKind {
    pub fn as_str(&self) -> &'static str {
        match self {
            SurfaceFeatureKind::Rut { .. } => "rut",
            SurfaceFeatureKind::Pothole { .. } => "pothole",
            SurfaceFeatureKind::Puddle { .. } => "puddle",
            SurfaceFeatureKind::Bump { .. } => "bump",
            SurfaceFeatureKind::Roughness { .. } => "roughness",
        }
    }

    /// The parameters that must be finite and positive, by name, for a report.
    pub fn magnitudes(&self) -> Vec<(&'static str, f64)> {
        match *self {
            SurfaceFeatureKind::Rut {
                depth,
                track,
                trough,
                ..
            } => vec![("depth", depth), ("track", track), ("trough", trough)],
            SurfaceFeatureKind::Pothole { depth, .. } => vec![("depth", depth)],
            SurfaceFeatureKind::Puddle { depth } => vec![("depth", depth)],
            SurfaceFeatureKind::Bump {
                height, profile, ..
            } => match profile {
                BumpProfile::Flat { ramp } => vec![("height", height), ("ramp", ramp)],
                _ => vec![("height", height)],
            },
            SurfaceFeatureKind::Roughness { .. } => Vec::new(),
        }
    }
}

/// How far the edge of a rut eases in from nothing, at most, metres.
const RUT_EASE: f64 = 2.0;
/// How far roughness and the side of a bump ease in from the footprint's edge, at
/// most, metres.
const EDGE_EASE: f64 = 0.5;
/// How much of a pothole's radius is its flat bottom.
const POTHOLE_FLOOR: f64 = 0.55;
/// How far a pothole's rim wanders in from the footprint's ellipse, as a share of
/// its radius.
const POTHOLE_RAGGEDNESS: f64 = 0.18;
/// How many plane waves a rough surface is the sum of.
const ROUGHNESS_WAVES: usize = 48;
/// The band of spatial frequencies roughness is synthesised over, cycles per metre:
/// wavelengths from 50 m down to 0.35 m, ISO 8608's band short of its longest
/// waves, which are the road's own elevation rather than its unevenness.
const ROUGHNESS_BAND: (f64, f64) = (0.02, 2.83);

impl SurfaceFeature {
    /// A footprint for a feature centred at `(station, offset)`, `length` along the
    /// road and `width` across it.
    pub fn centred(
        id: SurfaceFeatureId,
        road: RoadId,
        station: f64,
        offset: f64,
        length: f64,
        width: f64,
        kind: SurfaceFeatureKind,
    ) -> SurfaceFeature {
        SurfaceFeature {
            id,
            road,
            station: station - length / 2.0,
            length,
            offset,
            width,
            kind,
        }
    }

    /// Wheel-path ruts from `from` to `to` along a lane whose middle is `offset`
    /// to the left of the reference line.
    pub fn rut(
        id: SurfaceFeatureId,
        road: RoadId,
        (from, to): (f64, f64),
        offset: f64,
        depth: f64,
        track: f64,
        trough: f64,
    ) -> SurfaceFeature {
        SurfaceFeature {
            id,
            road,
            station: from.min(to),
            length: (to - from).abs(),
            offset,
            width: track + trough,
            kind: SurfaceFeatureKind::Rut {
                depth,
                track,
                trough,
                path: None,
            },
        }
    }

    /// Wheel-path ruts along `path`, which wanders as it likes within and out of
    /// its lane. The footprint is left empty: [`Self::fit_to_path`] fits it once
    /// the lane exists, which the builder does.
    pub fn rut_along(
        id: SurfaceFeatureId,
        road: RoadId,
        path: LanePath,
        depth: f64,
        track: f64,
        trough: f64,
    ) -> SurfaceFeature {
        let (from, to) = path.stations();
        SurfaceFeature {
            id,
            road,
            station: from,
            length: to - from,
            offset: 0.0,
            width: track + trough,
            kind: SurfaceFeatureKind::Rut {
                depth,
                track,
                trough,
                path: Some(path),
            },
        }
    }

    /// The path the feature follows, if it follows one.
    pub fn path(&self) -> Option<&LanePath> {
        match &self.kind {
            SurfaceFeatureKind::Rut { path, .. } => path.as_ref(),
            _ => None,
        }
    }

    /// The footprint a rut along a path needs on `map`: from the path's first
    /// station to its last, and across everything its troughs sweep. `None` for a
    /// feature that follows no path, or one whose lane `map` does not hold.
    pub fn footprint_on(&self, map: &crate::map::Map) -> Option<(f64, f64, f64, f64)> {
        let SurfaceFeatureKind::Rut {
            track,
            trough,
            path: Some(path),
            ..
        } = &self.kind
        else {
            return None;
        };
        let centre = PathCentre::of(map, path)?;
        let (lo, hi, steep) = centre.bounds();
        let reach = (track + trough) / 2.0 * (1.0 + steep * steep).sqrt();
        let (from, to) = path.stations();
        Some((from, to - from, (lo + hi) / 2.0, hi - lo + 2.0 * reach))
    }

    /// Sets the footprint of a rut along a path to [`Self::footprint_on`] `map`.
    pub fn fit_to_path(&mut self, map: &crate::map::Map) -> Result<(), String> {
        if self.path().is_none() {
            return Ok(());
        }
        let lane = self.path().map(|path| path.lane.clone());
        let (station, length, offset, width) = self.footprint_on(map).ok_or_else(|| {
            format!(
                "{} follows a path along {}, which the map does not have",
                self.id,
                lane.map_or_else(String::new, |lane| lane.to_string())
            )
        })?;
        self.station = station;
        self.length = length;
        self.offset = offset;
        self.width = width;
        Ok(())
    }

    /// The station range the footprint covers.
    pub fn stations(&self) -> (f64, f64) {
        (self.station, self.station + self.length)
    }

    /// The lateral range the footprint covers, right edge then left, metres to the
    /// left of the reference line.
    pub fn laterals(&self) -> (f64, f64) {
        (
            self.offset - self.width / 2.0,
            self.offset + self.width / 2.0,
        )
    }

    /// Whether `(station, lateral)` is inside the footprint.
    pub fn covers(&self, station: f64, lateral: f64) -> bool {
        let (s0, s1) = self.stations();
        let (t0, t1) = self.laterals();
        station >= s0 && station <= s1 && lateral >= t0 && lateral <= t1
    }

    /// The deepest the feature goes below the design, and the highest it stands
    /// above it, metres: the bounds of [`Self::displacement`].
    pub fn extremes(&self) -> (f64, f64) {
        match self.kind {
            SurfaceFeatureKind::Rut { depth, .. }
            | SurfaceFeatureKind::Pothole { depth, .. }
            | SurfaceFeatureKind::Puddle { depth } => (-depth, 0.0),
            SurfaceFeatureKind::Bump { height, .. } => (0.0, height),
            SurfaceFeatureKind::Roughness { class, .. } => {
                // The waves' amplitudes summed: a bound no surface reaches, but
                // the one that is certain.
                let bound: f64 = roughness_waves(class, 0).iter().map(|w| w.amplitude).sum();
                (-bound, bound)
            }
        }
    }

    /// How far the surface stands above the design at `(station, lateral)`,
    /// metres; negative below it. Zero outside the footprint.
    ///
    /// A rut along a path needs to know where its lane is, which only the map
    /// does: [`Relief`] lays the path out and asks [`Self::displacement_on`].
    /// Asked here, the path is read as if its lane's middle were the middle of
    /// the footprint and ran straight.
    pub fn displacement(&self, station: f64, lateral: f64) -> f64 {
        let centre = self.path().map(|path| {
            let (offset, slope) = path.offset_at(station);
            (self.offset + offset, slope)
        });
        self.displacement_on(station, lateral, centre)
    }

    /// [`Self::displacement`], with the middle of a rut's path at `station` and its
    /// slope across the road there given, as [`PathCentre::at`] gives them. Ignored
    /// by every other kind, and by a rut that runs straight.
    pub fn displacement_on(&self, station: f64, lateral: f64, centre: Option<(f64, f64)>) -> f64 {
        if !self.covers(station, lateral) {
            return 0.0;
        }
        let along = station - self.station;
        let across = lateral - self.offset;
        match self.kind {
            SurfaceFeatureKind::Rut {
                depth,
                track,
                trough,
                ..
            } => {
                let ease = RUT_EASE.min(self.length / 4.0);
                let ends = ease_in(along.min(self.length - along), ease);
                // Across the path rather than across the road: at an angle, a
                // trough's section along the road's lateral is wider by 1 / cos.
                let (middle, stretch) = match centre {
                    Some((middle, slope)) => (middle, (1.0 + slope * slope).sqrt()),
                    None => (self.offset, 1.0),
                };
                let across = lateral - middle;
                let half = trough / 2.0 * stretch;
                let apart = track / 2.0 * stretch;
                let profile = |centre: f64| {
                    let from = (across - centre).abs();
                    if from >= half {
                        0.0
                    } else {
                        0.5 * (1.0 + (PI * from / half).cos())
                    }
                };
                -depth * ends * (profile(apart) + profile(-apart)).min(1.0)
            }
            SurfaceFeatureKind::Pothole { depth, seed } => {
                let (r, angle) = self.elliptic(along, across);
                let rim = 1.0 - POTHOLE_RAGGEDNESS * raggedness(seed, angle);
                if r >= rim {
                    return 0.0;
                }
                let r = r / rim;
                let share = if r <= POTHOLE_FLOOR {
                    1.0
                } else {
                    0.5 * (1.0 + (PI * (r - POTHOLE_FLOOR) / (1.0 - POTHOLE_FLOOR)).cos())
                };
                -depth * share
            }
            SurfaceFeatureKind::Puddle { depth } => {
                let (r, _) = self.elliptic(along, across);
                if r >= 1.0 {
                    0.0
                } else {
                    -depth * 0.5 * (1.0 + (PI * r).cos())
                }
            }
            SurfaceFeatureKind::Bump {
                height, profile, ..
            } => {
                let side = ease_in(
                    self.width / 2.0 - across.abs(),
                    EDGE_EASE.min(self.width / 4.0),
                );
                height * profile.share(along, self.length, height) * side
            }
            SurfaceFeatureKind::Roughness { class, seed } => {
                let ease = EDGE_EASE.min(self.length / 4.0).min(self.width / 4.0);
                let edge = ease_in(along.min(self.length - along), ease)
                    * ease_in(self.width / 2.0 - across.abs(), ease);
                if edge <= 0.0 {
                    return 0.0;
                }
                edge * roughness_waves(class, seed)
                    .iter()
                    .map(|wave| wave.at(along, across))
                    .sum::<f64>()
            }
        }
    }

    /// How deep water stands on the surface at `(station, lateral)`, metres: the
    /// depth of a puddle's depression there, and nothing for any other kind.
    pub fn water_depth(&self, station: f64, lateral: f64) -> f64 {
        match self.kind {
            SurfaceFeatureKind::Puddle { .. } => (-self.displacement(station, lateral)).max(0.0),
            _ => 0.0,
        }
    }

    /// Whether the feature holds water.
    pub fn holds_water(&self) -> bool {
        matches!(self.kind, SurfaceFeatureKind::Puddle { .. })
    }

    /// How finely the feature has to be sampled to keep its shape, metres: a
    /// fifth of the narrowest thing in it.
    pub fn resolution(&self) -> f64 {
        let smallest = match self.kind {
            SurfaceFeatureKind::Rut { trough, .. } => trough.min(self.length),
            SurfaceFeatureKind::Pothole { .. } | SurfaceFeatureKind::Puddle { .. } => {
                self.length.min(self.width)
            }
            SurfaceFeatureKind::Bump { profile, .. } => match profile {
                BumpProfile::Flat { ramp } => ramp.min(self.length),
                _ => self.length,
            },
            // The shortest wave of the band.
            SurfaceFeatureKind::Roughness { .. } => 1.0 / ROUGHNESS_BAND.1,
        };
        (smallest / 5.0).max(0.01)
    }

    /// The radial distance of a point from the middle of the footprint, as a share
    /// of the inscribed ellipse's radius in that direction, and its angle.
    fn elliptic(&self, along: f64, across: f64) -> (f64, f64) {
        let du = (along - self.length / 2.0) / (self.length / 2.0);
        let dw = across / (self.width / 2.0);
        (du.hypot(dw), dw.atan2(du))
    }
}

/// A raised cosine from 0 at `x = 0` to 1 at `x = ease`; 1 beyond, 0 before.
fn ease_in(x: f64, ease: f64) -> f64 {
    if x <= 0.0 {
        0.0
    } else if ease <= 0.0 || x >= ease {
        1.0
    } else {
        0.5 * (1.0 - (PI * x / ease).cos())
    }
}

/// How far a pothole's rim stands in from its ellipse at `angle`, between 0 and 1:
/// three harmonics with phases drawn from `seed`.
fn raggedness(seed: u64, angle: f64) -> f64 {
    let mut random = SplitMix64::new(seed ^ 0x706f_7468_6f6c_6521);
    let mut sum = 0.0;
    let mut weight = 0.0;
    for harmonic in [2.0, 3.0, 5.0] {
        let phase = random.unit() * 2.0 * PI;
        let amplitude = 1.0 / harmonic;
        sum += amplitude * (0.5 + 0.5 * (harmonic * angle + phase).sin());
        weight += amplitude;
    }
    sum / weight
}

/// One plane wave of a rough surface.
#[derive(Debug, Clone, Copy)]
struct Wave {
    amplitude: f64,
    /// Cycles per metre along the road and across it.
    along: f64,
    across: f64,
    phase: f64,
}

impl Wave {
    fn at(&self, along: f64, across: f64) -> f64 {
        self.amplitude * (2.0 * PI * (self.along * along + self.across * across) + self.phase).cos()
    }
}

/// The waves of a rough surface of `class`, drawn from `seed`.
///
/// The band is cut into [`ROUGHNESS_WAVES`] bands of equal width in log frequency;
/// each contributes one wave at a frequency drawn within it, of amplitude
/// `sqrt(2 · Gd(n) · Δn)`, which is the amplitude a sinusoid needs to carry the
/// band's share of the variance. Each wave runs at an angle to the road drawn from
/// ±30°, so that the surface varies across the road as well as along it; the
/// longitudinal profile is then the ISO spectrum to within the `cos` of that angle.
fn roughness_waves(class: RoughnessClass, seed: u64) -> Vec<Wave> {
    let mut random = SplitMix64::new(seed ^ 0x726f_7567_686e_6573);
    let (low, high) = ROUGHNESS_BAND;
    let step = (high / low).ln() / ROUGHNESS_WAVES as f64;
    (0..ROUGHNESS_WAVES)
        .map(|band| {
            let from = low * (step * band as f64).exp();
            let to = low * (step * (band + 1) as f64).exp();
            let n = from + (to - from) * random.unit();
            let amplitude = (2.0 * class.psd(n) * (to - from)).sqrt();
            let angle = (random.unit() - 0.5) * (PI / 3.0);
            Wave {
                amplitude,
                along: n * angle.cos(),
                across: n * angle.sin(),
                phase: random.unit() * 2.0 * PI,
            }
        })
        .collect()
}

/// A small, fixed pseudo-random generator, so that a seed means the same surface
/// on every platform and in every version that keeps this algorithm.
struct SplitMix64(u64);

impl SplitMix64 {
    fn new(seed: u64) -> Self {
        SplitMix64(seed)
    }

    fn next(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9e37_79b9_7f4a_7c15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
        z ^ (z >> 31)
    }

    /// Uniform in `[0, 1)`.
    fn unit(&mut self) -> f64 {
        (self.next() >> 11) as f64 / (1u64 << 53) as f64
    }
}

/// Every feature on one road, read as one surface.
#[derive(Debug, Clone, Default)]
pub struct Relief<'a> {
    pub features: Vec<&'a SurfaceFeature>,
    /// Each feature's path laid out on the road, beside it; `None` for one that
    /// follows no path.
    paths: Vec<Option<PathCentre>>,
}

impl<'a> Relief<'a> {
    pub fn of(map: &'a crate::map::Map, road: &RoadId) -> Relief<'a> {
        let features: Vec<&SurfaceFeature> = map
            .surface_features
            .iter()
            .filter(|feature| feature.road == *road)
            .collect();
        let paths = features
            .iter()
            .map(|feature| feature.path().and_then(|path| PathCentre::of(map, path)))
            .collect();
        Relief { features, paths }
    }

    /// One feature's displacement, its path laid out on the road.
    fn displacement_of(&self, index: usize, station: f64, lateral: f64) -> f64 {
        let feature = self.features[index];
        match &self.paths[index] {
            Some(centre) => {
                if !feature.covers(station, lateral) {
                    return 0.0;
                }
                feature.displacement_on(station, lateral, Some(centre.at(station)))
            }
            None => feature.displacement(station, lateral),
        }
    }

    pub fn is_empty(&self) -> bool {
        self.features.is_empty()
    }

    /// The sum of every feature's displacement at `(station, lateral)`.
    pub fn displacement(&self, station: f64, lateral: f64) -> f64 {
        (0..self.features.len())
            .map(|index| self.displacement_of(index, station, lateral))
            .sum()
    }

    /// The deepest water stands at `(station, lateral)`.
    pub fn water_depth(&self, station: f64, lateral: f64) -> f64 {
        self.features
            .iter()
            .map(|feature| feature.water_depth(station, lateral))
            .fold(0.0, f64::max)
    }

    /// Whether any feature's footprint reaches into the station range `[from, to]`.
    pub fn touches(&self, from: f64, to: f64) -> bool {
        self.features.iter().any(|feature| {
            let (s0, s1) = feature.stations();
            s1 >= from && s0 <= to
        })
    }

    /// Whether any feature's footprint reaches into the rectangle `[from, to]` by
    /// `[right, left]`.
    pub fn touches_area(&self, (from, to): (f64, f64), (right, left): (f64, f64)) -> bool {
        self.features.iter().any(|feature| {
            let (s0, s1) = feature.stations();
            let (t0, t1) = feature.laterals();
            s1 >= from && s0 <= to && t1 >= right && t0 <= left
        })
    }

    /// Stations to sample the surface at between `from` and `to` so that every
    /// feature there keeps its shape at `cell` metres or finer: the ends of each
    /// footprint, and the stations between them `cell` apart, or the feature's own
    /// resolution where that is finer.
    pub fn stations(&self, from: f64, to: f64, cell: f64) -> Vec<f64> {
        let mut out = Vec::new();
        for feature in &self.features {
            let (s0, s1) = feature.stations();
            let (a, b) = (s0.max(from), s1.min(to));
            if b <= a {
                continue;
            }
            let step = cell.min(feature.resolution()).max(1e-3);
            let count = ((b - a) / step).ceil() as usize;
            for k in 0..=count {
                out.push(a + (b - a) * k as f64 / count.max(1) as f64);
            }
        }
        out.sort_by(f64::total_cmp);
        out.dedup_by(|a, b| (*a - *b).abs() < 1e-6);
        out
    }

    /// The finest lateral step any feature touching `[from, to]` needs, capped at
    /// `cell`.
    pub fn lateral_step(&self, from: f64, to: f64, cell: f64) -> f64 {
        self.features
            .iter()
            .filter(|feature| {
                let (s0, s1) = feature.stations();
                s1 >= from && s0 <= to
            })
            .map(|feature| feature.resolution())
            .fold(cell, f64::min)
            .max(1e-3)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn feature(kind: SurfaceFeatureKind, length: f64, width: f64) -> SurfaceFeature {
        SurfaceFeature::centred(
            SurfaceFeatureId::new("main/0"),
            RoadId::new("main"),
            50.0,
            1.75,
            length,
            width,
            kind,
        )
    }

    #[test]
    fn iso_8608_classes_quadruple_from_a() {
        assert!((RoughnessClass::A.psd_at_reference() - 16e-6).abs() < 1e-12);
        assert!((RoughnessClass::C.psd_at_reference() - 256e-6).abs() < 1e-12);
        assert!((RoughnessClass::H.psd_at_reference() - 262_144e-6).abs() < 1e-9);
        // n^-2: a tenth of the frequency, a hundred times the density.
        let class = RoughnessClass::B;
        assert!((class.psd(0.01) / class.psd(0.1) - 100.0).abs() < 1e-9);
    }

    #[test]
    fn a_pothole_is_deepest_in_the_middle_and_flush_outside() {
        let hole = feature(
            SurfaceFeatureKind::Pothole {
                depth: 0.08,
                seed: 3,
            },
            0.8,
            0.6,
        );
        assert!((hole.displacement(50.0, 1.75) + 0.08).abs() < 1e-9);
        assert_eq!(hole.displacement(50.5, 1.75), 0.0);
        assert_eq!(hole.displacement(50.0, 2.2), 0.0);
        // And nothing on the ellipse's own rim.
        assert!(hole.displacement(50.4, 1.75).abs() < 1e-9);
    }

    #[test]
    fn a_puddle_holds_water_as_deep_as_its_dip() {
        let puddle = feature(SurfaceFeatureKind::Puddle { depth: 0.03 }, 2.0, 1.2);
        assert!((puddle.water_depth(50.0, 1.75) - 0.03).abs() < 1e-9);
        assert!(puddle.water_depth(50.0, 1.75 + 0.3) > 0.0);
        assert_eq!(puddle.water_depth(52.0, 1.75), 0.0);
    }

    #[test]
    fn ruts_are_two_troughs_a_track_apart() {
        let rut = SurfaceFeature::rut(
            SurfaceFeatureId::new("main/rut"),
            RoadId::new("main"),
            (10.0, 60.0),
            -1.75,
            0.025,
            1.8,
            0.6,
        );
        assert!((rut.width - 2.4).abs() < 1e-12);
        let middle = 35.0;
        assert!((rut.displacement(middle, -1.75 + 0.9) + 0.025).abs() < 1e-9);
        assert!((rut.displacement(middle, -1.75 - 0.9) + 0.025).abs() < 1e-9);
        assert!(rut.displacement(middle, -1.75).abs() < 1e-9);
        // Eased in: shallower a metre in than in the middle, nothing at the end.
        assert!(rut.displacement(11.0, -1.75 + 0.9) > -0.025);
        assert!(rut.displacement(10.0, -1.75 + 0.9).abs() < 1e-9);
    }

    #[test]
    fn every_hump_profile_reaches_its_height_at_the_crest_and_nothing_at_its_ends() {
        for profile in [
            BumpProfile::Circular,
            BumpProfile::Parabolic,
            BumpProfile::Sinusoidal,
            BumpProfile::Flat { ramp: 1.0 },
        ] {
            let hump = feature(
                SurfaceFeatureKind::Bump {
                    kind: BumpKind::SpeedHump,
                    height: 0.1,
                    profile,
                },
                3.7,
                6.0,
            );
            assert!(
                (hump.displacement(50.0, 1.75) - 0.1).abs() < 1e-9,
                "{profile:?} at its crest"
            );
            assert!(hump.displacement(50.0 - 1.85, 1.75).abs() < 1e-9);
            assert!(hump.displacement(50.0 + 1.85, 1.75).abs() < 1e-9);
        }
    }

    #[test]
    fn roughness_is_reproducible_bounded_and_eased_to_nothing_at_its_edge() {
        let rough = |seed| {
            feature(
                SurfaceFeatureKind::Roughness {
                    class: RoughnessClass::C,
                    seed,
                },
                40.0,
                7.0,
            )
        };
        let a = rough(7);
        let b = rough(7);
        let c = rough(8);
        let (low, high) = a.extremes();
        let mut differs = false;
        let mut sum_sq = 0.0;
        let mut count = 0.0;
        for i in 0..400 {
            let s = 30.5 + i as f64 * 0.1;
            let z = a.displacement(s, 1.75);
            assert_eq!(z, b.displacement(s, 1.75));
            assert!(z >= low && z <= high);
            differs |= (z - c.displacement(s, 1.75)).abs() > 1e-6;
            sum_sq += z * z;
            count += 1.0;
        }
        assert!(differs, "another seed is another surface");
        // Class C over this band is about a centimetre RMS.
        let rms = (sum_sq / count).sqrt();
        assert!(rms > 0.002 && rms < 0.03, "rms {rms}");
        assert_eq!(a.displacement(30.0, 1.75), 0.0);
        assert_eq!(a.displacement(50.0, 1.75 + 3.5), 0.0);
    }

    #[test]
    fn a_lane_path_passes_through_its_knots_smoothly() {
        let path = LanePath::new(
            LaneId::new("main/0"),
            vec![(0.0, 0.0), (10.0, 1.0), (20.0, -0.5), (30.0, 0.0)],
        )
        .unwrap();
        for &(s, t) in path.knots() {
            assert!((path.offset_at(s).0 - t).abs() < 1e-12);
        }
        // Continuous in value and slope either side of an inner knot.
        let (left, right) = (path.offset_at(10.0 - 1e-6), path.offset_at(10.0 + 1e-6));
        assert!((left.0 - right.0).abs() < 1e-5);
        assert!((left.1 - right.1).abs() < 1e-4);
        // Two knots are a straight line.
        let straight = LanePath::new(LaneId::new("main/0"), vec![(0.0, 0.0), (10.0, 2.0)]).unwrap();
        assert!((straight.offset_at(2.5).0 - 0.5).abs() < 1e-12);
        assert!((straight.offset_at(2.5).1 - 0.2).abs() < 1e-12);
        assert!(LanePath::new(LaneId::new("x"), vec![(0.0, 0.0)]).is_err());
        assert!(LanePath::new(LaneId::new("x"), vec![(1.0, 0.0), (1.0, 1.0)]).is_err());
    }

    #[test]
    fn a_rut_along_a_path_follows_it_across_the_lane_and_into_the_next() {
        use crate::prelude::*;
        let mut builder = MapBuilder::default();
        let road = builder
            .add_road(
                RoadSpec::line(
                    Point3::new(0.0, 0.0, 0.0),
                    Point3::new(100.0, 0.0, 0.0),
                    vec![
                        LaneSpec::new(PositiveWidth::new(3.5).unwrap(), Direction::Forward),
                        LaneSpec::new(PositiveWidth::new(3.5).unwrap(), Direction::Backward),
                    ],
                )
                .unwrap()
                .with_name("main"),
            )
            .unwrap();
        // The forward lane is to the right, its middle at t = -1.75. The path
        // starts in its middle and drifts 3.5 m left, into the other lane.
        let id = builder
            .add_rut_along(
                &LaneRef::new(road.clone(), 0),
                vec![(10.0, 0.0), (30.0, 0.0), (60.0, 3.5), (80.0, 3.5)],
                0.04,
                1.8,
                0.6,
            )
            .unwrap();
        let map = builder.finish().unwrap().validate().unwrap();
        let rut = map.surface_features.get(&id).unwrap();
        // The footprint was fitted: it spans the path and both ends of its drift.
        let (t0, t1) = rut.laterals();
        assert!(t0 <= -1.75 - 1.2 && t1 >= 1.75 + 1.2, "{t0}..{t1}");
        assert_eq!(rut.stations(), (10.0, 80.0));
        let relief = Relief::of(&map, &road);
        // In the middle stretch, the troughs straddle the forward lane's middle;
        // at the far end, the backward lane's.
        assert!((relief.displacement(20.0, -1.75 + 0.9) + 0.04).abs() < 1e-6);
        assert!(relief.displacement(20.0, -1.75).abs() < 1e-6);
        assert!((relief.displacement(70.0, 1.75 - 0.9) + 0.04).abs() < 1e-6);
        assert!(relief.displacement(70.0, -1.75 + 0.9).abs() < 1e-6);
        // On the slant, the troughs stand wider apart across the road.
        let (centre, slope) = PathCentre::of(&map, rut.path().unwrap()).unwrap().at(45.0);
        assert!(slope > 0.05);
        let apart = 0.9 * (1.0 + slope * slope).sqrt();
        assert!((relief.displacement(45.0, centre + apart) + 0.04).abs() < 1e-6);
        // A path that leaves its lane's stations does not validate.
        let mut builder = MapBuilder::default();
        let road = builder
            .add_road(
                RoadSpec::line(
                    Point3::new(0.0, 0.0, 0.0),
                    Point3::new(20.0, 0.0, 0.0),
                    vec![LaneSpec::new(
                        PositiveWidth::new(3.5).unwrap(),
                        Direction::Forward,
                    )],
                )
                .unwrap(),
            )
            .unwrap();
        builder
            .add_rut_along(
                &LaneRef::new(road, 0),
                vec![(5.0, 0.0), (25.0, 0.0)],
                0.03,
                1.8,
                0.6,
            )
            .unwrap();
        assert!(builder.finish().unwrap().validate().is_err());
    }

    #[test]
    fn names_round_trip() {
        for kind in BumpKind::ALL {
            assert_eq!(BumpKind::parse(kind.as_str()), Some(kind));
        }
        for class in RoughnessClass::ALL {
            assert_eq!(RoughnessClass::parse(class.as_str()), Some(class));
        }
    }
}
