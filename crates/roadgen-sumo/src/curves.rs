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
//! piece the speed `√(a / κ)` its sharpest point allows for the lateral acceleration
//! `a` asked for, capped by the limit the edge already had. SUMO's vehicles look
//! ahead at the speeds of the lanes they are about to enter, so they brake for a
//! piece before they reach it.
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
//! The curvature is measured on the carriageway's centre line every
//! [`STEP`] metres, as that of the circle through the points [`HALF_CHORD`] metres
//! either side: long enough that the polyline's sampling does not read as
//! curvature, short enough to find a bend. Where the curve speed is below the
//! edge's speed by more than [`MARGIN`], the line is curved; the runs of curved and
//! straight line are the pieces, and a run shorter than [`MIN_PIECE`] metres is
//! taken as curved, so that a bend is never broken up by a stretch too short to
//! drive. A piece is cut no closer than [`MIN_PIECE`] to the edge's end where a stop
//! offset has to fit on the last piece. An edge curved or straight from end to end
//! is not cut, though a curved one still has its speed lowered.

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

/// A stretch of an edge's centre line, by station, and the speed its curvature
/// holds traffic to — `None` where that is no lower than the edge's own.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Stretch {
    pub start: f64,
    pub end: f64,
    pub speed: Option<f64>,
}

/// The pieces `line` is cut into for traffic at `speed` m/s and a lateral
/// acceleration of `lateral_acceleration` m/s², keeping the last at least `tail`
/// metres long.
///
/// Always at least one; one stretch over the whole line where it is not cut.
pub fn stretches(
    line: &Polyline3,
    speed: f64,
    lateral_acceleration: f64,
    tail: f64,
) -> Vec<Stretch> {
    let length = horizontal_length(line);
    let count = (length / STEP).floor() as usize;
    let samples: Vec<f64> = (0..=count)
        .map(|index| {
            let station = (index as f64 * STEP).min(length);
            let curvature = curvature_at(line, station, length, HALF_CHORD);
            if curvature > 1e-9 {
                (lateral_acceleration / curvature).sqrt()
            } else {
                f64::INFINITY
            }
        })
        .collect();

    // Runs of curved and straight line, as half-open sample ranges.
    let curved = |sample: f64| sample < speed - MARGIN;
    let mut runs: Vec<(usize, usize, bool)> = Vec::new();
    for (index, &sample) in samples.iter().enumerate() {
        match runs.last_mut() {
            Some(run) if run.2 == curved(sample) => run.1 = index + 1,
            _ => runs.push((index, index + 1, curved(sample))),
        }
    }
    let run_length = |run: &(usize, usize, bool)| (run.1 - run.0) as f64 * STEP;
    for run in &mut runs {
        if run_length(run) < MIN_PIECE {
            run.2 = true;
        }
    }
    let mut merged: Vec<(usize, usize, bool)> = Vec::new();
    for run in runs {
        match merged.last_mut() {
            Some(last) if last.2 == run.2 => last.1 = run.1,
            _ => merged.push(run),
        }
    }
    // The last piece carries the stop offset; fold pieces into it until it fits.
    while merged.len() > 1 && run_length(merged.last().expect("not empty")) < tail.max(MIN_PIECE) {
        let last = merged.pop().expect("not empty");
        let before = merged.last_mut().expect("more than one");
        before.1 = last.1;
        before.2 |= last.2;
    }

    let slowest = |run: &(usize, usize, bool)| {
        samples[run.0..run.1]
            .iter()
            .copied()
            .fold(f64::INFINITY, f64::min)
    };
    let pieces = merged.len();
    merged
        .iter()
        .enumerate()
        .map(|(index, run)| Stretch {
            start: if index == 0 { 0.0 } else { run.0 as f64 * STEP },
            end: if index + 1 == pieces {
                length
            } else {
                run.1 as f64 * STEP
            },
            speed: Some(round_down(slowest(run))).filter(|&limit| limit < speed - MARGIN),
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
    let length = horizontal_length(line);
    let half_chord = HALF_CHORD.min(length / 3.0);
    if half_chord < STEP {
        return None;
    }
    let mut station = half_chord;
    let mut sharpest: f64 = 0.0;
    while station <= length - half_chord {
        sharpest = sharpest.max(curvature_at(line, station, length, half_chord));
        station += STEP;
    }
    (sharpest > 1e-9).then(|| round_down((lateral_acceleration / sharpest).sqrt()))
}

/// A speed to a tenth of a metre per second, rounded down so that it never exceeds
/// what the curve allows.
fn round_down(speed: f64) -> f64 {
    (speed * 10.0).floor() / 10.0
}

/// The curvature of `line` at `station`, 1/m: that of the circle through the points
/// `half_chord` either side of it and the point itself, in plan.
fn curvature_at(line: &Polyline3, station: f64, length: f64, half_chord: f64) -> f64 {
    let before = (station - half_chord).max(0.0);
    let after = (station + half_chord).min(length);
    if after - before < half_chord {
        return 0.0;
    }
    let (a, b, c) = (
        point_at(line, before),
        point_at(line, station),
        point_at(line, after),
    );
    let (ab, bc, ac) = (
        (b.x - a.x).hypot(b.y - a.y),
        (c.x - b.x).hypot(c.y - b.y),
        (c.x - a.x).hypot(c.y - a.y),
    );
    let cross = (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
    let denominator = ab * bc * ac;
    if denominator < 1e-12 {
        return 0.0;
    }
    2.0 * cross.abs() / denominator
}

/// The length of `line` in plan.
pub fn horizontal_length(line: &Polyline3) -> f64 {
    line.points()
        .windows(2)
        .map(|segment| (segment[1].x - segment[0].x).hypot(segment[1].y - segment[0].y))
        .sum()
}

/// The point `station` metres along `line` in plan, clamped to its ends.
pub fn point_at(line: &Polyline3, station: f64) -> Point3 {
    let mut travelled = 0.0;
    for segment in line.points().windows(2) {
        let (a, b) = (segment[0], segment[1]);
        let length = (b.x - a.x).hypot(b.y - a.y);
        if travelled + length >= station && length > 0.0 {
            return a.lerp(b, ((station - travelled) / length).clamp(0.0, 1.0));
        }
        travelled += length;
    }
    if station <= 0.0 {
        line.first()
    } else {
        line.last()
    }
}

/// The part of `line` between two stations in plan, with the points at both ends
/// interpolated.
pub fn slice(line: &Polyline3, start: f64, end: f64) -> Result<Polyline3, ExportError> {
    let mut points = vec![point_at(line, start)];
    let mut travelled = 0.0;
    for segment in line.points().windows(2) {
        travelled += (segment[1].x - segment[0].x).hypot(segment[1].y - segment[0].y);
        if travelled > start && travelled < end {
            points.push(segment[1]);
        }
    }
    points.push(point_at(line, end));
    Ok(Polyline3::new(points)?)
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

    #[test]
    fn a_straight_line_is_one_uncapped_piece() {
        let straight = line(vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(100.0, 0.0, 0.0),
        ]);
        let pieces = stretches(&straight, 13.9, 3.0, 0.0);
        assert_eq!(pieces.len(), 1);
        assert_eq!(pieces[0].speed, None);
        assert!((pieces[0].end - 100.0).abs() < 1e-9);
    }

    #[test]
    fn an_arc_from_end_to_end_is_one_capped_piece() {
        // 30 m radius at 3 m/s²: √90 ≈ 9.49 m/s.
        let bend = line(arc(30.0, 90.0));
        let pieces = stretches(&bend, 13.9, 3.0, 0.0);
        assert_eq!(pieces.len(), 1);
        let speed = pieces[0].speed.unwrap();
        assert!((9.0..=9.5).contains(&speed), "{speed}");
    }

    #[test]
    fn a_bend_between_straights_is_cut_out() {
        let mut points = vec![Point3::new(-50.0, 0.0, 0.0)];
        points.extend(arc(30.0, 90.0));
        let end = *points.last().unwrap();
        points.push(Point3::new(end.x, end.y + 50.0, 0.0));
        let road = line(points);
        let pieces = stretches(&road, 13.9, 3.0, 0.0);
        assert_eq!(pieces.len(), 3, "{pieces:?}");
        assert_eq!(pieces[0].speed, None);
        assert!(pieces[1].speed.unwrap() < 9.5);
        assert_eq!(pieces[2].speed, None);
        // The bend starts 50 m in and is about 47 m long; the chord blurs its ends.
        assert!(
            (pieces[1].start - 50.0).abs() <= HALF_CHORD + STEP,
            "{pieces:?}"
        );
        assert!(
            (pieces[2].start - 97.1).abs() <= HALF_CHORD + STEP,
            "{pieces:?}"
        );
        assert!((pieces[2].end - horizontal_length(&road)).abs() < 1e-9);
    }

    #[test]
    fn a_short_last_piece_is_folded_back_to_fit_the_tail() {
        let mut points = vec![Point3::new(-50.0, 0.0, 0.0)];
        points.extend(arc(30.0, 90.0));
        let end = *points.last().unwrap();
        points.push(Point3::new(end.x, end.y + 20.0, 0.0));
        let road = line(points);
        let pieces = stretches(&road, 13.9, 3.0, 25.0);
        assert_eq!(pieces.len(), 2, "{pieces:?}");
        assert!(pieces[1].speed.is_some());
        assert!(pieces[1].end - pieces[1].start >= 25.0);
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
        let part = slice(&road, 5.0, 15.0).unwrap();
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
