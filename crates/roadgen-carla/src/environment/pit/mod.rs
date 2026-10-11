//! An open pit: its floor, and the benches stepped up from it to the rim.
//!
//! A pit is a shape of the land and nothing else. It lays out no road: the roads
//! into it are the caller's, drawn with `MapBuilder::add_road` like any other —
//! a spiral down the wall, a switchback, a straight cut — and the land is built
//! round them wherever they go. [`PitSpec::ramp`] is there to help draw the usual
//! one, and is only a list of points.
//!
//! # How a pit is made
//!
//! A pit is designed outwards from its floor. The floor is an ellipse; each bench
//! is the ring of the floor pushed out along its normal — a face rising
//! `bench_height` at `face_angle`, from its toe to its crest, then a bench
//! `bench_width` wide, then the next face. Parallel curves of a convex outline never
//! fold, so the rings stack without having to be checked against each other. Every
//! toe and crest is a breakline of the land, cut back where a road runs through it.
//!
//! The pit is designed in its own frame — the floor's centre at the origin, its
//! long axis along `x` — and placed in the map's by one turn and one shift.

pub mod geometry;

use std::f64::consts::TAU;

use roadgen_core::geometry::Point3;

use self::geometry::{Ellipse, Frame};
use super::land::{plan_of, smoothstep, Landforms, Level, Line};

/// An open pit.
#[derive(Debug, Clone, PartialEq)]
pub struct PitSpec {
    /// What the pit's features are named after.
    pub name: String,
    /// The centre of the floor in plan, at the height of the rim — which is the
    /// height of the country round it.
    pub centre: Point3,
    /// Radians anticlockwise from `+x` to the pit's long axis.
    pub heading: f64,
    /// The floor's length along the long axis, and its width across it.
    pub floor_length: f64,
    pub floor_width: f64,
    /// Benches from the floor to the rim. The pit is this many bench heights deep.
    pub benches: usize,
    pub bench_height: f64,
    /// How wide each bench is, from the crest of the face below to the toe of the
    /// face above.
    pub bench_width: f64,
    /// How steep each face is, degrees from the horizontal.
    pub face_angle: f64,
    /// The site the pit belongs to: its own name's, when none is said.
    pub site: Option<String>,
}

impl Default for PitSpec {
    fn default() -> Self {
        PitSpec {
            name: "pit".into(),
            centre: Point3::ORIGIN,
            heading: 0.0,
            floor_length: 220.0,
            floor_width: 140.0,
            benches: 6,
            bench_height: 12.0,
            bench_width: 10.0,
            face_angle: 70.0,
            site: None,
        }
    }
}

/// The usual ramp into a pit: a road winding down its wall at one grade.
#[derive(Debug, Clone, PartialEq)]
pub struct RampSpec {
    /// Where on the rim it starts, degrees anticlockwise round the pit from the end
    /// of its long axis.
    pub start: f64,
    /// Rise over run: a haul ramp is usually between 0.08 and 0.10.
    pub grade: f64,
    /// Which way round it winds on the way down, seen from above.
    pub clockwise: bool,
    /// The full width of the road that will be drawn along it, metres: its outer
    /// edge, not its middle, is what runs along the wall.
    pub width: f64,
    /// How far back along the rim, metres, a curve runs in to the ramp from a
    /// point clear of the pit — where the caller's other roads can meet it. Zero
    /// starts the ramp at the rim.
    pub approach: f64,
}

impl Default for RampSpec {
    fn default() -> Self {
        RampSpec {
            start: -90.0,
            grade: 0.1,
            clockwise: true,
            width: 21.0,
            approach: 80.0,
        }
    }
}

/// How far round the rim the ramp eases from the rim into the wall, metres.
const EASE: f64 = 40.0;
/// How far out from the rim, past the road's own half width, the approach starts.
const APPROACH_OUT: f64 = 35.0;
/// The longest approach, metres: it runs along the rim, so a longer one would wind
/// round the pit rather than lead into it.
const MAX_APPROACH: f64 = 1000.0;
/// The tightest the ramp's inner edge turns, metres.
const INNER_RADIUS: f64 = 4.0;
/// How far apart the ramp's points are, metres.
const RAMP_SPACING: f64 = 4.0;

impl PitSpec {
    /// What is wrong with the spec, if anything.
    pub fn check(&self) -> Result<(), String> {
        for (name, value) in [
            ("floor_length", self.floor_length),
            ("floor_width", self.floor_width),
            ("bench_height", self.bench_height),
            ("bench_width", self.bench_width),
        ] {
            if !(value.is_finite() && value > 0.0) {
                return Err(format!(
                    "{}: {name} must be a positive number, not {value}",
                    self.name
                ));
            }
        }
        if self.benches == 0 {
            return Err(format!("{}: a pit has at least one bench", self.name));
        }
        if !(10.0..89.0).contains(&self.face_angle) {
            return Err(format!(
                "{}: a face at {} degrees is not one a bench stands at",
                self.name, self.face_angle
            ));
        }
        if !self.centre.is_finite() || !self.heading.is_finite() {
            return Err(format!(
                "{}: the centre and heading must be finite",
                self.name
            ));
        }
        Ok(())
    }

    fn frame(&self) -> Frame {
        Frame {
            centre: self.centre,
            heading: self.heading,
        }
    }

    fn floor(&self) -> Ellipse {
        Ellipse {
            a: self.floor_length / 2.0,
            b: self.floor_width / 2.0,
        }
    }

    /// The rim's height, which is the country's.
    pub fn rim_height(&self) -> f64 {
        self.centre.z
    }

    /// The floor's height.
    pub fn floor_height(&self) -> f64 {
        self.centre.z - self.depth()
    }

    pub fn depth(&self) -> f64 {
        self.benches as f64 * self.bench_height
    }

    /// How far each face reaches in plan.
    fn run(&self) -> f64 {
        self.bench_height / self.face_angle.to_radians().tan()
    }

    /// How far the rim's crest stands out from the floor's outline.
    fn top(&self) -> f64 {
        self.benches as f64 * (self.run() + self.bench_width) - self.bench_width
    }

    /// A ring of the pit, in the map's frame, at height `z`.
    fn ring(&self, offset: f64, z: f64, spacing: f64) -> Vec<Point3> {
        let frame = self.frame();
        self.floor()
            .ring(offset, spacing)
            .into_iter()
            .map(|p| frame.place(p[0], p[1], z))
            .collect()
    }

    /// The rim's crest: the pit's outline at the country's height.
    pub fn rim(&self, spacing: f64) -> Vec<Point3> {
        self.ring(self.top(), self.rim_height(), spacing)
    }

    /// What the pit shapes the land with: every bench's toe and crest as lines, the
    /// floor as a level, and the rim as a hole the country stays out of, with the
    /// country's height at it.
    pub fn landforms(&self, key: &str, spacing: f64) -> Landforms {
        let mut forms = Landforms::default();
        let rim = self.rim(spacing);
        let (run, bottom) = (self.run(), self.floor_height());
        for bench in 0..self.benches {
            let toe = bench as f64 * (run + self.bench_width);
            let z = bottom + bench as f64 * self.bench_height;
            forms.lines.push(Line {
                key: format!("{key}/bench/{bench}/toe"),
                kind: "toe",
                points: self.ring(toe, z, spacing),
                closed: true,
            });
            forms.lines.push(Line {
                key: format!("{key}/bench/{bench}/crest"),
                kind: "crest",
                points: self.ring(toe + run, z + self.bench_height, spacing),
                closed: true,
            });
        }
        forms.levels.push(Level {
            ring: plan_of(&self.ring(0.0, bottom, spacing)),
            z: bottom,
        });
        forms.holes.push(plan_of(&rim));
        forms.anchors.extend(rim);
        forms
    }

    /// The usual ramp into the pit, as the points of a road's reference line: an
    /// approach curving in from clear of the rim, then a spiral down the wall at the
    /// ramp's grade to the floor. Hand them to `add_road` with the cross-section the
    /// ramp is meant to have — `ramp.width` wide.
    ///
    /// At each height the ramp's outer edge runs where the wall, on average, is at
    /// that height, so it is cut into the wall above and stands out over the bench
    /// below — except where the floor turns too tightly for a road that wide, where
    /// the ramp keeps out from it so that its inner edge does not fold. A grade too gentle to reach the floor in four turns of the pit is
    /// refused.
    pub fn ramp(&self, ramp: &RampSpec) -> Result<Vec<Point3>, String> {
        self.check()?;
        if !(ramp.grade > 0.0 && ramp.grade <= 0.25) {
            return Err(format!(
                "a ramp at grade {} is not one a haul truck climbs",
                ramp.grade
            ));
        }
        if !(ramp.width.is_finite() && ramp.width > 0.0) {
            return Err(format!("a ramp {} m wide", ramp.width));
        }
        if !(ramp.approach.is_finite() && (0.0..=MAX_APPROACH).contains(&ramp.approach)) {
            return Err(format!(
                "an approach of {} m: it runs along the rim, between 0 and {MAX_APPROACH} m",
                ramp.approach
            ));
        }
        if !ramp.start.is_finite() {
            return Err(format!("a ramp starting at {} degrees", ramp.start));
        }
        let (frame, floor) = (self.frame(), self.floor());
        let (rim, bottom, depth, top) = (
            self.rim_height(),
            self.floor_height(),
            self.depth(),
            self.top(),
        );
        let half = ramp.width / 2.0;
        let wall = |z: f64| ((z - bottom) / depth).clamp(0.0, 1.0) * top - half;
        let sense = if ramp.clockwise { -1.0 } else { 1.0 };
        let theta0 = ramp.start.to_radians();
        // The ramp's path at any offset from the floor is a parallel curve of it,
        // turning at the floor's radius plus the offset; the road's inner edge turns
        // `half` tighter still. One bound for the whole ramp, from the floor's
        // tightest turn, keeps it a parallel curve — no kink where it applies.
        let tightest = floor
            .radius(0.0)
            .min(floor.radius(std::f64::consts::FRAC_PI_2));
        let innermost = half + INNER_RADIUS - tightest;

        // Down the wall in small steps of the ellipse's parameter, keeping the
        // horizontal distance travelled, which is what the grade is over.
        let mut spiral: Vec<[f64; 3]> = Vec::new();
        let mut theta = theta0;
        let mut travelled = 0.0;
        let mut last = floor.at(theta, top - half);
        spiral.push([last[0], last[1], rim]);
        let mut since = 0.0;
        let mut reached = false;
        while (theta - theta0).abs() <= 4.0 * TAU {
            theta += sense * 0.002;
            let z_guess = rim - ramp.grade * travelled;
            // Ease from the rim into the wall over the first stretch, so the road
            // turns into the pit rather than kinking into it.
            let ease = smoothstep(0.0, EASE, travelled);
            let offset = top - half + (wall(z_guess) - (top - half)) * ease;
            // Never so far in that the road's inner edge would fold: where the
            // floor's outline turns tighter than the road is wide — the ends of a
            // long, narrow pit — the road keeps out, cut into the wall.
            let offset = offset.max(innermost);
            let here = floor.at(theta, offset);
            let length = (here[0] - last[0]).hypot(here[1] - last[1]);
            travelled += length;
            since += length;
            last = here;
            let z = rim - ramp.grade * travelled;
            if z <= bottom {
                // The floor, in place of a vertex just before it: a chord much
                // shorter than the rest leaves a corner no room to be rounded in.
                if since < RAMP_SPACING / 2.0 && spiral.len() > 1 {
                    spiral.pop();
                }
                spiral.push([here[0], here[1], bottom]);
                reached = true;
                break;
            }
            if since >= RAMP_SPACING {
                spiral.push([here[0], here[1], z]);
                since = 0.0;
            }
        }
        if !reached {
            return Err(format!(
                "a ramp at grade {} cannot reach a floor {depth} m down in four turns of \
                 the pit; make it steeper or the pit shallower",
                ramp.grade
            ));
        }

        // The approach: a curve that leaves a point clear of the rim heading in and
        // meets the rim along the ramp's own direction there.
        let mut points = Vec::new();
        if ramp.approach > 0.0 {
            let tangent = floor.tangent(theta0).map(|value| value * sense);
            let normal = floor.normal(theta0);
            let at = spiral[0];
            let mouth = [
                at[0] - tangent[0] * ramp.approach + normal[0] * (half + APPROACH_OUT),
                at[1] - tangent[1] * ramp.approach + normal[1] * (half + APPROACH_OUT),
            ];
            let control = [
                at[0] - tangent[0] * ramp.approach * 0.5,
                at[1] - tangent[1] * ramp.approach * 0.5,
            ];
            let steps = (ramp.approach / 5.0).ceil().max(2.0) as usize;
            for index in 0..steps {
                let t = index as f64 / steps as f64;
                let (a, b, c) = ((1.0 - t) * (1.0 - t), 2.0 * t * (1.0 - t), t * t);
                points.push(frame.place(
                    a * mouth[0] + b * control[0] + c * at[0],
                    a * mouth[1] + b * control[1] + c * at[1],
                    rim,
                ));
            }
        }
        points.extend(spiral.iter().map(|p| frame.place(p[0], p[1], p[2])));
        Ok(points)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_ramp_reaches_the_floor_at_its_grade() {
        let pit = PitSpec::default();
        let ramp = RampSpec::default();
        let points = pit.ramp(&ramp).unwrap();
        let last = points.last().unwrap();
        assert!((last.z - pit.floor_height()).abs() < 1e-9);
        // Level on the approach, then falling at the grade all the way down, but
        // for the last step, which stops on the floor.
        let falling: Vec<_> = points
            .windows(2)
            .filter(|pair| pair[1].z < pit.rim_height())
            .collect();
        assert!(falling.len() > 10);
        for pair in &falling[..falling.len() - 1] {
            let grade = (pair[0].z - pair[1].z) / pair[0].horizontal_distance_to(pair[1]);
            assert!((grade - ramp.grade).abs() < 1e-3, "{grade}");
        }
    }

    #[test]
    fn a_ramp_into_a_narrow_pit_does_not_fold() {
        let pit = PitSpec {
            floor_length: 200.0,
            floor_width: 20.0,
            ..PitSpec::default()
        };
        let points = pit.ramp(&RampSpec::default()).unwrap();
        // No vertex turns back on the one before it.
        for window in points.windows(3) {
            let (a, b, c) = (window[0], window[1], window[2]);
            let (u, v) = ([b.x - a.x, b.y - a.y], [c.x - b.x, c.y - b.y]);
            assert!(u[0] * v[0] + u[1] * v[1] > 0.0, "{a:?} {b:?} {c:?}");
        }
    }

    #[test]
    fn a_ramp_with_no_end_to_its_approach_is_refused() {
        for ramp in [
            RampSpec {
                approach: f64::INFINITY,
                ..RampSpec::default()
            },
            RampSpec {
                approach: 1e12,
                ..RampSpec::default()
            },
            RampSpec {
                start: f64::NAN,
                ..RampSpec::default()
            },
        ] {
            assert!(PitSpec::default().ramp(&ramp).is_err());
        }
    }

    #[test]
    fn a_ramp_too_gentle_to_reach_the_floor_is_refused() {
        let ramp = RampSpec {
            grade: 0.01,
            ..RampSpec::default()
        };
        let error = PitSpec::default().ramp(&ramp).unwrap_err();
        assert!(error.contains("cannot reach"), "{error}");
    }
}
