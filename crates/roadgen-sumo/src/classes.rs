//! What a SUMO edge and lane say about a road.
//!
//! SUMO describes a road by what may drive on it and how fast, not by what is
//! painted on it. So the lowering is four small tables: which vehicle classes a
//! lane type admits, whether the paint between two lanes lets a vehicle cross it,
//! how fast a road type is when the map gives no limit, and where a road type sits
//! in the priority order netconvert uses to work out who yields.

use roadgen_core::semantics::{LaneType, RoadMarking, RoadType};
use roadgen_core::topology::LateralSide;

/// What a SUMO lane says about who may use it.
///
/// SUMO's two attributes are complementary — `allow` names the only classes
/// admitted, `disallow` names the ones kept out of an otherwise open lane — and
/// which of the two is right depends on the lane. A driving lane is open to
/// everything on wheels, so it is written as a lane pedestrians are kept out of; a
/// footway admits pedestrians and nothing else.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Permission {
    /// `allow`: these classes and no others.
    Allow(&'static str),
    /// `disallow`: everything except these.
    Disallow(&'static str),
}

impl Permission {
    pub fn attribute(self) -> (&'static str, &'static str) {
        match self {
            Permission::Allow(classes) => ("allow", classes),
            Permission::Disallow(classes) => ("disallow", classes),
        }
    }

    /// Whether a vehicle of SUMO class `class` — `passenger`, `pedestrian` — may use
    /// a lane with this permission.
    ///
    /// Read off the same space-separated list the attribute is written with, so the
    /// answer is SUMO's for the file as written and cannot drift from it.
    pub fn admits(self, class: &str) -> bool {
        match self {
            Permission::Allow(classes) => classes.split_whitespace().any(|named| named == class),
            Permission::Disallow(classes) => {
                !classes.split_whitespace().any(|named| named == class)
            }
        }
    }
}

/// Who may use a lane of this type, or `None` for a lane SUMO has no place for.
///
/// A SUMO lane is a strip traffic of some class runs along. A border, a painted
/// island or a parking bay is none of those: it is part of the road surface with no
/// movement on it, and SUMO models it — when it models it at all — as a shape in a
/// separate additional file rather than as a lane. Such lanes are dropped, and
/// [`crate::check`] names them.
pub fn permission(lane_type: LaneType) -> Option<Permission> {
    Some(match lane_type {
        LaneType::Driving => Permission::Disallow("pedestrian"),
        LaneType::Biking => Permission::Allow("bicycle"),
        LaneType::Sidewalk => Permission::Allow("pedestrian"),
        // A hard shoulder is a lane in SUMO's sense — it is where a broken-down
        // vehicle stops and an ambulance passes — but not one open traffic uses.
        LaneType::Shoulder => Permission::Allow("emergency"),
        LaneType::Border | LaneType::Parking | LaneType::Restricted | LaneType::None => {
            return None
        }
    })
}

/// The vehicle classes a lane change across a line no one may cross is still open
/// to.
///
/// SUMO's `changeLeft` and `changeRight` name the classes *allowed* to change, not
/// the ones kept from it, so a line that forbids crossing is written as the short
/// list of who may cross it anyway. `emergency` is that list: an ambulance on a
/// call may cross a solid line, as the traffic codes that paint one generally let
/// it, and nobody else may.
pub const CHANGE_ACROSS_SOLID: &str = "emergency";

/// Whether a vehicle on the `from` side of a boundary painted `marking` may cross
/// it to change lanes.
///
/// `from` is read in the reference-line frame the IR keeps markings in — the side
/// of the reference line's direction the vehicle is on, *not* the side of its own
/// direction of travel — because that is the frame a double line's two words are
/// written in: in `solid broken` the solid line is the left one looking along the
/// reference line, as OpenDRIVE and the IR both read it. The line nearer the
/// vehicle is the one that binds it, so in `solid broken` a vehicle on the right
/// sees the broken line and may cross, and one on the left sees the solid line and
/// may not. That is the asymmetry the marking exists to state: overtaking is
/// allowed from one side and returning from the other.
///
/// A broken line and an unpainted boundary may be crossed. A solid line, a double
/// solid line and a kerb may not.
pub fn may_cross(marking: RoadMarking, from: LateralSide) -> bool {
    match marking {
        RoadMarking::None | RoadMarking::Broken => true,
        RoadMarking::Solid | RoadMarking::SolidSolid | RoadMarking::Curbstone => false,
        RoadMarking::SolidBroken => from == LateralSide::Right,
        RoadMarking::BrokenSolid => from == LateralSide::Left,
    }
}

/// How fast a road of this type is when the map states no limit, metres per second.
///
/// SUMO requires a speed on every edge — it is what the car-following model works
/// from — so there is no writing "unknown". These are SUMO's own defaults for the
/// OSM `highway` values each road type maps to, which is what a network built from
/// OSM data would carry.
pub fn default_speed(road_type: RoadType) -> f64 {
    match road_type {
        RoadType::Motorway => 36.11,
        RoadType::Rural => 22.22,
        RoadType::Town => 13.89,
        RoadType::LowSpeed => 8.33,
        RoadType::Pedestrian => 2.78,
    }
}

/// Where a road of this type sits in SUMO's priority order.
///
/// netconvert reads this to decide who yields at an uncontrolled junction: the arm
/// with the higher priority keeps right of way and the lower one gets a give-way
/// line. The ladder is the same one the OpenStreetMap export climbs, so the two
/// formats rank a map's roads identically.
pub fn priority(road_type: RoadType) -> i32 {
    match road_type {
        RoadType::Motorway => 13,
        RoadType::Rural => 9,
        RoadType::Town => 4,
        RoadType::LowSpeed => 3,
        RoadType::Pedestrian => 1,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn only_lanes_traffic_runs_along_become_sumo_lanes() {
        assert_eq!(
            permission(LaneType::Driving),
            Some(Permission::Disallow("pedestrian"))
        );
        assert_eq!(
            permission(LaneType::Sidewalk),
            Some(Permission::Allow("pedestrian"))
        );
        assert_eq!(permission(LaneType::Border), None);
        assert_eq!(permission(LaneType::None), None);
    }

    #[test]
    fn only_a_driving_lane_admits_a_passenger_car() {
        let admits = |lane_type| permission(lane_type).unwrap().admits("passenger");
        assert!(admits(LaneType::Driving));
        assert!(!admits(LaneType::Sidewalk));
        assert!(!admits(LaneType::Biking));
        assert!(!admits(LaneType::Shoulder));
        assert!(permission(LaneType::Sidewalk).unwrap().admits("pedestrian"));
    }

    #[test]
    fn the_line_nearer_the_vehicle_decides_whether_it_may_cross() {
        for from in [LateralSide::Left, LateralSide::Right] {
            assert!(may_cross(RoadMarking::None, from));
            assert!(may_cross(RoadMarking::Broken, from));
            assert!(!may_cross(RoadMarking::Solid, from));
            assert!(!may_cross(RoadMarking::SolidSolid, from));
            assert!(!may_cross(RoadMarking::Curbstone, from));
        }
        // Solid on the left, broken on the right.
        assert!(!may_cross(RoadMarking::SolidBroken, LateralSide::Left));
        assert!(may_cross(RoadMarking::SolidBroken, LateralSide::Right));
        assert!(may_cross(RoadMarking::BrokenSolid, LateralSide::Left));
        assert!(!may_cross(RoadMarking::BrokenSolid, LateralSide::Right));
    }

    #[test]
    fn the_priority_ladder_puts_a_motorway_above_a_living_street() {
        assert!(priority(RoadType::Motorway) > priority(RoadType::Rural));
        assert!(priority(RoadType::Rural) > priority(RoadType::Town));
        assert!(priority(RoadType::Town) > priority(RoadType::LowSpeed));
        assert!(priority(RoadType::LowSpeed) > priority(RoadType::Pedestrian));
    }

    #[test]
    fn every_road_type_has_a_speed_to_fall_back_on() {
        for road_type in [
            RoadType::Motorway,
            RoadType::Rural,
            RoadType::Town,
            RoadType::LowSpeed,
            RoadType::Pedestrian,
        ] {
            assert!(default_speed(road_type) > 0.0);
        }
    }

    /// The two lane types with a class of their own rather than the open road's: a
    /// cycle lane is for bicycles and nothing else, and a hard shoulder is for the
    /// emergency services. Both are `allow`, never `disallow`, because what they
    /// admit is the short list.
    #[test]
    fn a_cycle_lane_admits_bicycles_and_a_shoulder_the_emergency_services() {
        assert_eq!(
            permission(LaneType::Biking),
            Some(Permission::Allow("bicycle"))
        );
        assert_eq!(
            permission(LaneType::Shoulder),
            Some(Permission::Allow("emergency"))
        );
        assert_eq!(
            Permission::Allow("bicycle").attribute(),
            ("allow", "bicycle")
        );
        assert_eq!(
            Permission::Disallow("pedestrian").attribute(),
            ("disallow", "pedestrian")
        );
    }

    /// Every lane type with no movement along it is dropped — not just the border and
    /// the strip beyond it, but a parking bay and a restricted lane too.
    #[test]
    fn a_parking_bay_and_a_restricted_lane_are_not_sumo_lanes() {
        assert_eq!(permission(LaneType::Parking), None);
        assert_eq!(permission(LaneType::Restricted), None);
    }

    /// The fallback speeds fall down the same ladder the priorities do: a road that
    /// outranks another at a junction is also the faster of the two when neither
    /// states a limit.
    #[test]
    fn the_fallback_speeds_fall_from_a_motorway_to_a_footway() {
        let ladder = [
            RoadType::Motorway,
            RoadType::Rural,
            RoadType::Town,
            RoadType::LowSpeed,
            RoadType::Pedestrian,
        ];
        for pair in ladder.windows(2) {
            assert!(default_speed(pair[0]) > default_speed(pair[1]), "{pair:?}");
        }
    }
}
