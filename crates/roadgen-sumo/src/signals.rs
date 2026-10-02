//! The fixed-time program a signalised junction is written with.
//!
//! # Why the export writes one at all
//!
//! The IR says *that* a junction is signalised — a light stands on an approach, a
//! [`TrafficRule::TrafficLight`](roadgen_core::semantics::TrafficRule) names the lanes
//! it governs — and nothing about timing: no cycle, no phase, no green split. Left to
//! itself netconvert makes a program up, and a perfectly good one, but its link
//! numbering and its phases are netconvert's: they move when the netconvert version
//! does, when an option changes, or when netconvert reads the geometry of the junction
//! slightly differently. Nothing the export could put in its trace would then say
//! which slot of the program a light of the map controls.
//!
//! So the program is decided here, from the IR, and written down in a `.tll.xml`: the
//! phases (so netconvert has nothing left to generate), and every controlled connection
//! with the `linkIndex` it is given (so the position of each movement in a state string
//! is the export's). The result is the same network every
//! time, whatever netconvert's defaults — with one addition of netconvert's own: at a
//! node with pedestrian crossings it appends the crossings' links after the vehicle
//! links and splits each green to end it with a pedestrian clearance. The vehicle
//! links, their indices and the order of green, yellow and red each goes through stay
//! the export's.
//!
//! # The heuristic
//!
//! The program is the static one Autoware's `lanelet2_to_sumo` writes, which is to say
//! the program a traffic engineer would sketch on the back of an envelope for a
//! junction they know nothing about:
//!
//! - The approaches are grouped by their **axis**, so that an approach and the one
//!   facing it across the junction fall into one group: they share a phase only if
//!   they arrive on headings opposite to within [`AXIS_TOLERANCE_DEGREES`]. Arms that
//!   merely lie near one axis without facing each other — those of two roads crossing
//!   at an acute angle — are kept apart, since their straight movements cross.
//! - Each group gets a green phase, followed by a yellow and an all-red: the two
//!   approaches of one road run together, and the crossing road waits.
//! - A green is [`GREEN_FEW_SECONDS`] long where there are at most two groups — the
//!   ordinary crossroads or tee — and [`GREEN_MANY_SECONDS`] where there are more, so
//!   that the cycle of a five-arm junction does not grow without limit.
//! - In a green, a movement is **permissive** (`g`, it goes when it can) rather than
//!   protected (`G`) if it turns across the oncoming traffic released with it, or if
//!   another movement released with it runs into the same lane. Everything else
//!   released is protected; everything not released is red.
//!
//! "Across the oncoming traffic" is a left turn where traffic keeps right and a right
//! turn where it keeps left, and a U-turn (a swing past [`U_TURN_DEGREES`]) either
//! way. It is read off the geometry here — the angle between the
//! lane a movement leaves and the lane it joins, and the map's handedness — rather
//! than off the `dir` netconvert later writes, so that the program does not depend on
//! netconvert being told which side the map drives on. One refinement over the
//! reference: a turn is only made permissive when its phase does release an oncoming
//! approach. A turn off a road with nothing facing it, as on the stem of a tee, has
//! nothing to give way to, and calling it permissive would only make SUMO slow every
//! vehicle down for a conflict that cannot happen.

use std::f64::consts::PI;

use roadgen_core::TrafficHandedness;

/// The program every signalised junction is written with. One program per junction,
/// so the number says nothing beyond "the one this export wrote" — and `0` is what
/// netconvert calls its own, which a loaded program of the same id replaces.
pub(crate) const PROGRAM_ID: &str = "0";

/// How far from exactly opposite the headings of two approaches may be for them to
/// run in one phase.
pub const AXIS_TOLERANCE_DEGREES: f64 = 35.0;

/// How far a movement must swing from the heading it arrived on to count as a turn
/// rather than as carrying straight on through a junction drawn slightly askew.
pub const TURN_DEGREES: f64 = 30.0;

/// How far a movement must swing, either way, to count as a U-turn, which crosses
/// the oncoming traffic whichever side the map drives on.
pub const U_TURN_DEGREES: f64 = 150.0;

/// The green of each phase where a junction has at most two groups of approaches.
pub const GREEN_FEW_SECONDS: u32 = 35;
/// The green of each phase where a junction has more than two.
pub const GREEN_MANY_SECONDS: u32 = 25;
/// The yellow after every green.
pub const YELLOW_SECONDS: u32 = 3;
/// The all-red after every yellow, which clears the junction before the next group
/// is released.
pub const ALL_RED_SECONDS: u32 = 2;

/// One controlled movement, as much of it as the program is decided from.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct Link {
    /// Which approach the movement arrives on: any number that is the same for the
    /// movements sharing an approach edge, and different otherwise.
    pub approach: usize,
    /// The heading of that approach where it meets the junction, radians
    /// anticlockwise from +x.
    pub heading: f64,
    /// How far the movement swings from the lane it leaves to the lane it joins,
    /// radians, anticlockwise positive: a left turn is positive, a right turn
    /// negative.
    pub turn: f64,
    /// The lane it runs into, as any value two movements into the same lane share.
    pub target: (usize, usize),
}

/// One phase of a program: how long it lasts, and one SUMO signal state per link,
/// in link-index order.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct Phase {
    pub duration: u32,
    pub state: String,
}

/// The approaches of a junction, grouped by axis, in the order each group's first
/// approach appears among `links`.
///
/// An approach joins the first group every member of which it *faces*: arrives on a
/// heading within the tolerance of opposite to theirs. Sharing an axis is not enough.
/// Two roads crossing at an acute angle have arms whose axes lie within the tolerance
/// of each other, but an arm and the arm of the other road beside it do not face each
/// other — they cross — and releasing them together would give two crossing straight
/// movements one green. Requiring every member rather than any also keeps a sequence
/// of slightly skewed arms from chaining the whole junction into one phase.
pub(crate) fn groups(links: &[Link]) -> Vec<Vec<usize>> {
    let mut groups: Vec<Vec<(usize, f64)>> = Vec::new();
    for link in links {
        if groups.iter().any(|members| {
            members
                .iter()
                .any(|(approach, _)| *approach == link.approach)
        }) {
            continue;
        }
        let member = (link.approach, link.heading);
        match groups.iter_mut().find(|members| {
            members
                .iter()
                .all(|(_, heading)| facing(*heading, link.heading))
        }) {
            Some(members) => members.push(member),
            None => groups.push(vec![member]),
        }
    }
    groups
        .into_iter()
        .map(|members| members.into_iter().map(|(approach, _)| approach).collect())
        .collect()
}

/// The program for one junction whose controlled movements are `links`, in the order
/// their link indices are given: a green, a yellow and an all-red for each group of
/// approaches, in the order [`groups`] finds them.
pub(crate) fn program(links: &[Link], handedness: TrafficHandedness) -> Vec<Phase> {
    let groups = groups(links);
    let green = if groups.len() <= 2 {
        GREEN_FEW_SECONDS
    } else {
        GREEN_MANY_SECONDS
    };
    let mut phases = Vec::with_capacity(groups.len() * 3);
    for group in &groups {
        let released: Vec<bool> = links
            .iter()
            .map(|link| group.contains(&link.approach))
            .collect();
        let state: String =
            links
                .iter()
                .enumerate()
                .map(|(index, link)| {
                    if !released[index] {
                        return 'r';
                    }
                    let oncoming = links
                        .iter()
                        .zip(&released)
                        .any(|(other, &on)| on && other.approach != link.approach);
                    let shares_target = links.iter().zip(&released).enumerate().any(
                        |(other_index, (other, &on))| {
                            on && other_index != index && other.target == link.target
                        },
                    );
                    if (oncoming && crosses_oncoming(link.turn, handedness)) || shares_target {
                        'g'
                    } else {
                        'G'
                    }
                })
                .collect();
        let yellow: String = released
            .iter()
            .map(|&on| if on { 'y' } else { 'r' })
            .collect();
        phases.push(Phase {
            duration: green,
            state,
        });
        phases.push(Phase {
            duration: YELLOW_SECONDS,
            state: yellow,
        });
        phases.push(Phase {
            duration: ALL_RED_SECONDS,
            state: "r".repeat(links.len()),
        });
    }
    phases
}

/// Whether a movement that swings by `turn` crosses the path of the traffic coming
/// the other way: a left turn where traffic keeps right, a right turn where it keeps
/// left — and a U-turn either way.
///
/// A swing of more than [`U_TURN_DEGREES`] is a U-turn whatever its sign: the lane it
/// joins runs back beside the oncoming approach, so it cuts across that traffic even
/// where the drawn geometry has it bend a few degrees past straight back.
pub(crate) fn crosses_oncoming(turn: f64, handedness: TrafficHandedness) -> bool {
    if turn.abs() > U_TURN_DEGREES.to_radians() {
        return true;
    }
    let threshold = TURN_DEGREES.to_radians();
    match handedness {
        TrafficHandedness::RightHand => turn > threshold,
        TrafficHandedness::LeftHand => turn < -threshold,
    }
}

/// The signed angle from heading `from` to heading `to`, in (-π, π].
pub(crate) fn swing(from: f64, to: f64) -> f64 {
    let mut angle = (to - from) % (2.0 * PI);
    if angle <= -PI {
        angle += 2.0 * PI;
    } else if angle > PI {
        angle -= 2.0 * PI;
    }
    angle
}

/// Whether two approaches arriving on headings `a` and `b` face each other across
/// the junction: their headings are opposite, within the tolerance — the same axis,
/// travelled the other way.
fn facing(a: f64, b: f64) -> bool {
    PI - swing(a, b).abs() <= AXIS_TOLERANCE_DEGREES.to_radians()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn degrees(value: f64) -> f64 {
        value.to_radians()
    }

    /// The four approaches of a crossroads, each with a left, a straight and a right
    /// turn, numbered as `approach * 3 + movement`. Exits are numbered by the
    /// approach they lie along, and each has two lanes: a left turn joins the inner
    /// one and the rest the outer, so that the left turn from one approach and the
    /// right turn from the one facing it do not run into the same lane.
    fn crossroads() -> Vec<Link> {
        // Arriving from the north heads south, and so on round.
        let headings = [-90.0, 180.0, 90.0, 0.0];
        let mut links = Vec::new();
        for (approach, heading) in headings.into_iter().enumerate() {
            for (turn, exit, lane) in [(90.0, 1, 1), (0.0, 2, 0), (-90.0, 3, 0)] {
                links.push(Link {
                    approach,
                    heading: degrees(heading),
                    turn: degrees(turn),
                    target: ((approach + exit) % 4, lane),
                });
            }
        }
        links
    }

    #[test]
    fn opposing_approaches_share_a_group() {
        assert_eq!(groups(&crossroads()), vec![vec![0, 2], vec![1, 3]]);
    }

    #[test]
    fn an_approach_slightly_askew_still_runs_with_the_one_facing_it() {
        let mut links = crossroads();
        for link in links.iter_mut().filter(|link| link.approach == 2) {
            link.heading = degrees(90.0 + 30.0);
        }
        assert_eq!(groups(&links), vec![vec![0, 2], vec![1, 3]]);
        for link in links.iter_mut().filter(|link| link.approach == 2) {
            link.heading = degrees(90.0 + 40.0);
        }
        assert_eq!(groups(&links).len(), 3);
    }

    #[test]
    fn the_arms_of_roads_crossing_at_an_acute_angle_do_not_share_a_green() {
        // Two straight roads crossing at 30°: one runs along 0°/180°, the other along
        // 30°/210°. Every axis lies within the tolerance of every other, but only the
        // two arms of one road face each other.
        let headings = [0.0, 30.0, 180.0, 210.0];
        let links: Vec<Link> = headings
            .into_iter()
            .enumerate()
            .map(|(approach, heading)| Link {
                approach,
                heading: degrees(heading),
                turn: 0.0,
                target: (approach, 0),
            })
            .collect();
        assert_eq!(groups(&links), vec![vec![0, 2], vec![1, 3]]);
        let phases = program(&links, TrafficHandedness::RightHand);
        let greens: Vec<&str> = phases
            .iter()
            .step_by(3)
            .map(|phase| phase.state.as_str())
            .collect();
        assert_eq!(greens, ["GrGr", "rGrG"]);
    }

    #[test]
    fn approaches_arriving_the_same_way_do_not_share_a_green() {
        // Two arms arriving side by side, 20° apart, neither facing the other.
        let links: Vec<Link> = [0.0, 20.0]
            .into_iter()
            .enumerate()
            .map(|(approach, heading)| Link {
                approach,
                heading: degrees(heading),
                turn: 0.0,
                target: (approach, 0),
            })
            .collect();
        assert_eq!(groups(&links), vec![vec![0], vec![1]]);
    }

    #[test]
    fn a_crossroads_gets_two_greens_with_the_turns_across_traffic_permissive() {
        let phases = program(&crossroads(), TrafficHandedness::RightHand);
        let summary: Vec<(u32, &str)> = phases
            .iter()
            .map(|phase| (phase.duration, phase.state.as_str()))
            .collect();
        // Left turns (the first of each approach's three) give way to the oncoming
        // straight movement; straights and right turns are protected.
        assert_eq!(
            summary,
            vec![
                (35, "gGGrrrgGGrrr"),
                (3, "yyyrrryyyrrr"),
                (2, "rrrrrrrrrrrr"),
                (35, "rrrgGGrrrgGG"),
                (3, "rrryyyrrryyy"),
                (2, "rrrrrrrrrrrr"),
            ]
        );
    }

    #[test]
    fn under_left_hand_traffic_it_is_the_right_turn_that_waits() {
        let phases = program(&crossroads(), TrafficHandedness::LeftHand);
        assert_eq!(phases[0].state, "GGgrrrGGgrrr");
    }

    #[test]
    fn a_turn_with_nothing_facing_it_is_protected() {
        // Two arms at right angles: each its own group, nothing oncoming.
        let links = vec![
            Link {
                approach: 0,
                heading: degrees(-90.0),
                turn: degrees(90.0),
                target: (1, 0),
            },
            Link {
                approach: 1,
                heading: degrees(180.0),
                turn: degrees(-90.0),
                target: (0, 0),
            },
        ];
        let phases = program(&links, TrafficHandedness::RightHand);
        let states: Vec<&str> = phases.iter().map(|phase| phase.state.as_str()).collect();
        assert_eq!(states, ["Gr", "yr", "rr", "rG", "ry", "rr"]);
    }

    #[test]
    fn movements_into_one_lane_are_both_permissive() {
        let links = vec![
            Link {
                approach: 0,
                heading: 0.0,
                turn: 0.0,
                target: (5, 0),
            },
            Link {
                approach: 1,
                heading: PI,
                turn: degrees(-90.0),
                target: (5, 0),
            },
        ];
        assert_eq!(program(&links, TrafficHandedness::RightHand)[0].state, "gg");
    }

    #[test]
    fn more_than_two_groups_shorten_the_green() {
        let links: Vec<Link> = [0.0, 60.0, 120.0]
            .into_iter()
            .enumerate()
            .map(|(approach, heading)| Link {
                approach,
                heading: degrees(heading),
                turn: 0.0,
                target: (approach, 0),
            })
            .collect();
        let phases = program(&links, TrafficHandedness::RightHand);
        assert_eq!(phases.len(), 9);
        assert_eq!(phases[0].duration, GREEN_MANY_SECONDS);
    }

    #[test]
    fn a_u_turn_crosses_oncoming_traffic_whichever_way_it_bends() {
        for handedness in [TrafficHandedness::RightHand, TrafficHandedness::LeftHand] {
            assert!(crosses_oncoming(degrees(175.0), handedness));
            assert!(crosses_oncoming(degrees(-175.0), handedness));
            assert!(crosses_oncoming(PI, handedness));
        }
        assert!(!crosses_oncoming(
            degrees(-90.0),
            TrafficHandedness::RightHand
        ));
        assert!(!crosses_oncoming(
            degrees(90.0),
            TrafficHandedness::LeftHand
        ));
    }

    #[test]
    fn a_swing_is_the_shorter_way_round() {
        assert!((swing(degrees(170.0), degrees(-170.0)) - degrees(20.0)).abs() < 1e-9);
        assert!((swing(degrees(-170.0), degrees(170.0)) + degrees(20.0)).abs() < 1e-9);
        assert!((swing(degrees(-90.0), 0.0) - degrees(90.0)).abs() < 1e-9);
    }
}
