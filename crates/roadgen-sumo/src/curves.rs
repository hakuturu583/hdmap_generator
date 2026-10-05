//! Curve speeds: where along an edge its geometry holds traffic below the speed
//! limit, and the pieces the edge is cut into so that SUMO knows.
//!
//! # Why the export does this
//!
//! A SUMO vehicle drives at the speed limit of its lane times its own speed factor,
//! whatever the lane's shape: the car-following models read no curvature. On a
//! network drawn from a real road that is a car taking a 20 m radius bend at 14 m/s
//! — some 10 m/s² sideways, which no tyre gives — and a vehicle driven by physics
//! from SUMO's trajectory (a co-simulation with CARLA, say) leaves the road there.
//! netconvert's `junctions.limit-turn-speed` does something like it for the
//! internal lanes of a junction, but from the mean radius of the whole turn; nothing
//! does it for the edges between junctions.
//!
//! A SUMO lane has one speed from end to end, so the export does what netconvert's
//! own `<split>` would: it cuts an edge where its curvature changes, and gives each
//! lane of each piece the speed `√(a / κ)` its sharpest point allows for the lateral
//! acceleration `a` asked for, where that is below the lane's limit. Each lane is
//! measured on its own line, so the inside of a bend is slower than the outside.
//! SUMO's vehicles look ahead at the speeds of the lanes they are about to enter,
//! so they brake for a piece before they reach it.
//!
//! The cut is the export's rather than netconvert's because the export knows what
//! every piece is: it names them, traces each back to the IR lane it is part of,
//! and keeps the connections, signals, stop offsets and trip weights of the edge on
//! the piece that has to carry them — which a `<split>` netconvert applies after
//! reading the files would not.
//!
//! The paths across junctions get the same treatment without a cut: the IR's
//! connector, written as a connection's `shape`, is measured the same way and its
//! speed written as the connection's `speed` — the sharpest point of a turn, where
//! netconvert would take its mean. `junctions.limit-turn-speed` is still set, for the
//! movements the IR draws no path for.
//!
//! # How an edge is cut
//!
//! The curvature is measured every [`STEP`] metres, as that of the circle through
//! the points [`HALF_CHORD`] metres either side: long enough that the polyline's
//! sampling does not read as curvature, short enough to find a bend. Where the
//! carriageway's centre line is slower than the edge's speed by more than
//! [`MARGIN`], it is curved; the runs of curved and straight line are the pieces.
//! A run shorter than [`MIN_PIECE`] is folded into its neighbour — a straight one
//! into the bends either side of it — so that no piece is too short to drive. The
//! last piece reaches back past every stop line on the edge, since a stop offset
//! only works on the lane that runs into the junction. An edge curved or straight
//! from end to end is not cut, though a curved one still has its lanes slowed.

use roadgen_core::geometry::{Point3, Polyline3};

use crate::ExportError;

/// How far apart the curvature is measured, metres.
pub const STEP: f64 = 1.0;
/// Half the chord the curvature at a point is measured over, metres.
pub const HALF_CHORD: f64 = 4.0;
/// The shortest piece an edge is cut into, metres.
pub const MIN_PIECE: f64 = 15.0;
/// How far below the edge's speed the curve speed has to be for a stretch to count
/// as curved, m/s.
pub const MARGIN: f64 = 0.5;

/// A polyline measured along in plan: the station of every vertex, so that a point
/// at a station is a binary search away.
pub struct Plan<'a> {
    line: &'a Polyline3,
    stations: Vec<f64>,
}

impl<'a> Plan<'a> {
    pub fn new(line: &'a Polyline3) -> Self {
        let mut stations = Vec::with_capacity(line.len());
        let mut travelled = 0.0;
        stations.push(travelled);
        for pair in line.points().windows(2) {
            travelled += pair[0].horizontal_distance_to(pair[1]);
            stations.push(travelled);
        }
        Plan { line, stations }
    }

    /// The length in plan.
    pub fn length(&self) -> f64 {
        *self.stations.last().expect("a polyline has vertices")
    }

    /// The point `station` metres along in plan, clamped to the ends.
    pub fn point_at(&self, station: f64) -> Point3 {
        let points = self.line.points();
        let after = self.stations.partition_point(|&s| s < station);
        if after == 0 {
            return points[0];
        }
        if after == points.len() {
            return points[points.len() - 1];
        }
        let (s0, s1) = (self.stations[after - 1], self.stations[after]);
        let t = if s1 > s0 {
            (station - s0) / (s1 - s0)
        } else {
            0.0
        };
        points[after - 1].lerp(points[after], t)
    }

    /// The part between two stations, with the points at both ends interpolated.
    pub fn slice(&self, start: f64, end: f64) -> Result<Polyline3, ExportError> {
        let inside = self
            .stations
            .iter()
            .zip(self.line.points())
            .filter(|(&s, _)| s > start && s < end)
            .map(|(_, &point)| point);
        let points: Vec<Point3> = std::iter::once(self.point_at(start))
            .chain(inside)
            .chain(std::iter::once(self.point_at(end)))
            .collect();
        Ok(Polyline3::new(points)?)
    }

    /// The curvature at `station`, 1/m: that of the circle through the point and
    /// the points `half_chord` either side of it, in plan. Zero where the line is
    /// too short to hold the chord.
    fn curvature_at(&self, station: f64, half_chord: f64) -> f64 {
        let before = (station - half_chord).max(0.0);
        let after = (station + half_chord).min(self.length());
        if after - before < half_chord {
            return 0.0;
        }
        let (a, b, c) = (
            self.point_at(before),
            self.point_at(station),
            self.point_at(after),
        );
        let denominator =
            a.horizontal_distance_to(b) * b.horizontal_distance_to(c) * a.horizontal_distance_to(c);
        if denominator < 1e-12 {
            return 0.0;
        }
        let cross = (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
        2.0 * cross.abs() / denominator
    }

    /// The speed the curvature allows at `lateral_acceleration` m/s², every
    /// [`STEP`] metres from the start: sample `i` is at station `i · STEP`.
    /// Infinite where the line is straight.
    pub fn curve_speeds(&self, lateral_acceleration: f64) -> Vec<f64> {
        let length = self.length();
        (0..=(length / STEP).floor() as usize)
            .map(|index| {
                let curvature = self.curvature_at(index as f64 * STEP, HALF_CHORD);
                if curvature > 1e-9 {
                    (lateral_acceleration / curvature).sqrt()
                } else {
                    f64::INFINITY
                }
            })
            .collect()
    }
}

/// The slowest of `speeds` (as [`Plan::curve_speeds`] samples them) from `start` up
/// to, not including, `end`, rounded down to a tenth of a metre per second, where
/// it is more than [`MARGIN`] below `limit`.
///
/// Up to and not including: a piece ends where the next begins, at the first
/// sample of the next one's bend, which is not this piece's to slow for.
pub fn cap(speeds: &[f64], start: f64, end: f64, limit: f64) -> Option<f64> {
    let first = ((start / STEP).ceil() as usize).min(speeds.len() - 1);
    let last = ((end / STEP).ceil() as usize)
        .saturating_sub(1)
        .clamp(first, speeds.len() - 1);
    let slowest = speeds[first..=last]
        .iter()
        .copied()
        .fold(f64::INFINITY, f64::min);
    Some(round_down(slowest)).filter(|&speed| speed < limit - MARGIN)
}

/// A stretch of an edge's centre line, by station.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Stretch {
    pub start: f64,
    pub end: f64,
}

/// A run of samples, `start..end`, that are all curved or all straight.
#[derive(Debug, Clone, Copy)]
struct Run {
    start: usize,
    end: usize,
    curved: bool,
}

impl Run {
    fn length(&self) -> f64 {
        (self.end - self.start) as f64 * STEP
    }
}

/// Joins neighbouring runs that agree.
fn coalesce(runs: &mut Vec<Run>) {
    runs.dedup_by(|next, run| {
        let same = run.curved == next.curved;
        if same {
            run.end = next.end;
        }
        same
    });
}

/// The pieces a centre line is cut into, from its curve speeds `speeds` (see
/// [`Plan::curve_speeds`]) for traffic at `speed` m/s, cut nowhere after station
/// `latest_cut`.
///
/// Always at least one, covering `0..length`.
pub fn stretches(speeds: &[f64], length: f64, speed: f64, latest_cut: f64) -> Vec<Stretch> {
    let mut runs: Vec<Run> = speeds
        .iter()
        .enumerate()
        .map(|(index, &sample)| Run {
            start: index,
            end: index + 1,
            curved: sample < speed - MARGIN,
        })
        .collect();
    coalesce(&mut runs);
    // Fold the shortest run that is too short into a neighbour until none is: a
    // straight one becomes part of the bend beside it, a curved one widens into
    // the straight beside it, and either way the result is curved.
    while runs.len() > 1 {
        let Some((index, _)) = runs
            .iter()
            .enumerate()
            .filter(|(_, run)| run.length() < MIN_PIECE)
            .min_by(|a, b| a.1.length().total_cmp(&b.1.length()))
        else {
            break;
        };
        let neighbour = match (index.checked_sub(1), runs.get(index + 1)) {
            (Some(before), Some(after)) if runs[before].length() > after.length() => index + 1,
            (Some(before), _) => before,
            (None, _) => index + 1,
        };
        let (low, high) = (index.min(neighbour), index.max(neighbour));
        runs[low] = Run {
            start: runs[low].start,
            end: runs[high].end,
            curved: true,
        };
        runs.remove(high);
        coalesce(&mut runs);
    }
    // The last piece carries the stop offsets; fold pieces into it until it does.
    while runs.len() > 1 && runs[runs.len() - 1].start as f64 * STEP > latest_cut {
        let last = runs.pop().expect("more than one");
        runs.last_mut().expect("more than one").end = last.end;
    }

    let count = runs.len();
    runs.iter()
        .enumerate()
        .map(|(index, run)| Stretch {
            start: run.start as f64 * STEP,
            end: if index + 1 == count {
                length
            } else {
                run.end as f64 * STEP
            },
        })
        .collect()
}

/// The speed the sharpest point of `line` allows at `lateral_acceleration` m/s², or
/// `None` for a line with no bend to speak of.
///
/// For a path across a junction, which is often shorter than the chord the edges are
/// measured with: the chord shrinks to a third of the line so that a tight turn still
/// reads as one.
pub fn slowest(line: &Polyline3, lateral_acceleration: f64) -> Option<f64> {
    let plan = Plan::new(line);
    let length = plan.length();
    let half_chord = HALF_CHORD.min(length / 3.0);
    if half_chord < STEP {
        return None;
    }
    let mut station = half_chord;
    let mut sharpest: f64 = 0.0;
    while station <= length - half_chord {
        sharpest = sharpest.max(plan.curvature_at(station, half_chord));
        station += STEP;
    }
    (sharpest > 1e-9).then(|| round_down((lateral_acceleration / sharpest).sqrt()))
}

/// A speed to a tenth of a metre per second, rounded down so that it never exceeds
/// what the curve allows.
fn round_down(speed: f64) -> f64 {
    (speed * 10.0).floor() / 10.0
}

#[cfg(test)]
mod tests {
    use super::*;

    fn arc(radius: f64, degrees: f64) -> Vec<Point3> {
        let steps = (degrees / 2.0).ceil() as usize;
        (0..=steps)
            .map(|step| {
                let angle = (degrees * step as f64 / steps as f64).to_radians();
                Point3::new(radius * angle.sin(), radius * (1.0 - angle.cos()), 0.0)
            })
            .collect()
    }

    fn line(points: Vec<Point3>) -> Polyline3 {
        Polyline3::new(points).unwrap()
    }

    /// Straight, then a bend of `radius` turning `degrees` left, then straight.
    fn bend_between(before: f64, radius: f64, degrees: f64, after: f64) -> Polyline3 {
        let mut points = vec![Point3::new(-before, 0.0, 0.0)];
        points.extend(arc(radius, degrees));
        let end = *points.last().unwrap();
        let heading = degrees.to_radians();
        points.push(Point3::new(
            end.x + after * heading.cos(),
            end.y + after * heading.sin(),
            0.0,
        ));
        line(points)
    }

    fn cut(line: &Polyline3, speed: f64, latest_cut: f64) -> (Vec<Stretch>, Vec<f64>) {
        let plan = Plan::new(line);
        let speeds = plan.curve_speeds(3.0);
        (stretches(&speeds, plan.length(), speed, latest_cut), speeds)
    }

    #[test]
    fn a_straight_line_is_one_uncapped_piece() {
        let straight = line(vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(100.0, 0.0, 0.0),
        ]);
        let (pieces, speeds) = cut(&straight, 13.9, f64::INFINITY);
        assert_eq!(
            pieces,
            [Stretch {
                start: 0.0,
                end: 100.0
            }]
        );
        assert_eq!(cap(&speeds, 0.0, 100.0, 13.9), None);
    }

    #[test]
    fn an_arc_from_end_to_end_is_one_capped_piece() {
        // 30 m radius at 3 m/s²: √90 ≈ 9.49 m/s.
        let bend = line(arc(30.0, 90.0));
        let (pieces, speeds) = cut(&bend, 13.9, f64::INFINITY);
        assert_eq!(pieces.len(), 1);
        let speed = cap(&speeds, pieces[0].start, pieces[0].end, 13.9).unwrap();
        assert!((9.0..=9.5).contains(&speed), "{speed}");
    }

    #[test]
    fn a_bend_between_straights_is_cut_out() {
        let road = bend_between(50.0, 30.0, 90.0, 50.0);
        let (pieces, speeds) = cut(&road, 13.9, f64::INFINITY);
        assert_eq!(pieces.len(), 3, "{pieces:?}");
        let capped: Vec<_> = pieces
            .iter()
            .map(|p| cap(&speeds, p.start, p.end, 13.9))
            .collect();
        assert_eq!(capped[0], None);
        assert!(capped[1].unwrap() < 9.5);
        assert_eq!(capped[2], None);
        // The bend starts 50 m in and is about 47 m long; the chord blurs its ends.
        assert!(
            (pieces[1].start - 50.0).abs() <= HALF_CHORD + STEP,
            "{pieces:?}"
        );
        assert!(
            (pieces[2].start - 97.1).abs() <= HALF_CHORD + STEP,
            "{pieces:?}"
        );
        assert!((pieces[2].end - Plan::new(&road).length()).abs() < 1e-9);
    }

    #[test]
    fn a_short_bend_is_widened_to_a_drivable_piece() {
        // 3 m of arc at 15 m radius: a kink no longer than the chord.
        let road = bend_between(50.0, 15.0, 3.0_f64.to_degrees() / 15.0, 50.0);
        let (pieces, _) = cut(&road, 13.9, f64::INFINITY);
        assert!(
            pieces.iter().all(|p| p.end - p.start >= MIN_PIECE),
            "{pieces:?}"
        );
    }

    #[test]
    fn the_last_piece_reaches_back_past_the_latest_cut() {
        let road = bend_between(50.0, 30.0, 90.0, 50.0);
        let length = Plan::new(&road).length();
        // A stop line 60 m before the end: the last piece has to hold it.
        let (pieces, _) = cut(&road, 13.9, length - 60.0);
        assert!(pieces.last().unwrap().start <= length - 60.0, "{pieces:?}");
        assert_eq!(pieces.len(), 2, "{pieces:?}");
    }

    #[test]
    fn the_inside_of_a_bend_is_slower_than_the_outside() {
        let inside = Plan::new(&line(arc(20.0, 90.0))).curve_speeds(3.0);
        let outside = Plan::new(&line(arc(27.0, 90.0))).curve_speeds(3.0);
        let speed = |speeds: &[f64]| cap(speeds, 0.0, 1e9, 20.0).unwrap();
        assert!(speed(&inside) < speed(&outside));
    }

    #[test]
    fn a_tight_turn_shorter_than_the_chord_still_has_a_speed() {
        // A right-angle turn of 8 m radius is 12.6 m long; at 3 m/s², √24 ≈ 4.9 m/s.
        let turn = line(arc(8.0, 90.0));
        let speed = slowest(&turn, 3.0).unwrap();
        assert!((4.5..=4.9).contains(&speed), "{speed}");
        let straight = line(vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(20.0, 0.0, 0.0),
        ]);
        assert_eq!(slowest(&straight, 3.0), None);
    }

    #[test]
    fn slice_keeps_the_vertices_between_its_ends() {
        let road = line(vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(10.0, 0.0, 0.0),
            Point3::new(10.0, 10.0, 0.0),
        ]);
        let part = Plan::new(&road).slice(5.0, 15.0).unwrap();
        assert_eq!(
            part.points(),
            &[
                Point3::new(5.0, 0.0, 0.0),
                Point3::new(10.0, 0.0, 0.0),
                Point3::new(10.0, 5.0, 0.0)
            ]
        );
    }
}
