//! The road networks the test suite is built on.
//!
//! Each one is the smallest map that exercises one thing: a straight road, a
//! carriageway each way, several lanes, two roads joined, a split, a merge, a
//! crossroads, a climb. Every test in this crate takes its map from here, so a
//! scenario is described once and checked from several angles.

use roadgen_core::prelude::*;
use roadgen_core::units::GeoOrigin;

/// A Tokyo anchor, so the exported latitudes and longitudes are somewhere real.
pub fn metadata(name: &str) -> MapMetadata {
    MapMetadata {
        name: Some(name.to_owned()),
        origin: GeoOrigin::new(35.68, 139.76, 0.0).unwrap(),
        ..MapMetadata::default()
    }
}

pub fn lane(width: f64, direction: Direction) -> LaneSpec {
    LaneSpec::new(PositiveWidth::new(width).unwrap(), direction)
}

pub fn one_way(count: usize) -> Vec<LaneSpec> {
    (0..count).map(|_| lane(3.5, Direction::Forward)).collect()
}

pub fn two_way() -> Vec<LaneSpec> {
    vec![
        lane(3.5, Direction::Forward),
        lane(3.5, Direction::Backward),
    ]
}

fn finish(builder: MapBuilder) -> ValidatedMap {
    builder
        .finish()
        .expect("the scenario should build")
        .validate()
        .expect("the scenario should validate")
}

/// 1. A single straight road in three dimensions, one lane.
pub fn straight_road() -> ValidatedMap {
    let mut builder = MapBuilder::new(metadata("straight"));
    builder
        .add_road(
            RoadSpec::line(
                Point3::new(0.0, 0.0, 5.0),
                Point3::new(120.0, 0.0, 5.0),
                one_way(1),
            )
            .unwrap()
            .with_name("main"),
        )
        .unwrap();
    finish(builder)
}

/// 2. One carriageway each way.
pub fn bidirectional_road() -> ValidatedMap {
    let mut builder = MapBuilder::new(metadata("bidirectional"));
    builder
        .add_road(
            RoadSpec::line(
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(150.0, 0.0, 0.0),
                two_way(),
            )
            .unwrap()
            .with_name("main"),
        )
        .unwrap();
    finish(builder)
}

/// 3. Three lanes one way and one the other, with the markings a real road has.
pub fn multi_lane_road() -> ValidatedMap {
    let mut builder = MapBuilder::new(metadata("multi-lane"));
    builder
        .add_road(
            RoadSpec::line(
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(200.0, 0.0, 0.0),
                vec![
                    lane(3.5, Direction::Forward).with_markings(
                        BoundaryMarking::new(RoadMarking::Solid, MarkingColor::Yellow),
                        BoundaryMarking::new(RoadMarking::Broken, MarkingColor::White),
                    ),
                    lane(3.5, Direction::Forward).with_markings(
                        BoundaryMarking::new(RoadMarking::Broken, MarkingColor::White),
                        BoundaryMarking::new(RoadMarking::Broken, MarkingColor::White),
                    ),
                    lane(3.25, Direction::Forward).with_markings(
                        BoundaryMarking::new(RoadMarking::Broken, MarkingColor::White),
                        BoundaryMarking::new(RoadMarking::Solid, MarkingColor::White),
                    ),
                    lane(3.5, Direction::Backward),
                ],
            )
            .unwrap()
            .with_name("wide")
            .with_speed_limit(SpeedLimit::from_kph(60.0).unwrap()),
        )
        .unwrap();
    finish(builder)
}

/// 4. Two roads joined end to start, with a bend and a climb — the map the design
///    notes use as their worked example.
pub fn two_roads_joined() -> ValidatedMap {
    let mut builder = MapBuilder::new(metadata("joined"));
    let a = builder
        .add_road(
            RoadSpec::line(
                Point3::new(0.0, 0.0, 10.0),
                Point3::new(100.0, 0.0, 12.0),
                two_way(),
            )
            .unwrap()
            .with_name("a"),
        )
        .unwrap();
    let b = builder
        .add_road(
            RoadSpec::line(
                Point3::new(100.0, 0.0, 12.0),
                Point3::new(200.0, 50.0, 15.0),
                two_way(),
            )
            .unwrap()
            .with_name("b"),
        )
        .unwrap();
    builder.connect(&a, &b).unwrap();
    finish(builder)
}

/// Two roads joined with no change of heading, so every vertex of the two formats
/// has to land in the same place.
pub fn two_roads_in_line() -> ValidatedMap {
    let mut builder = MapBuilder::new(metadata("in-line"));
    let a = builder
        .add_road(
            RoadSpec::line(
                Point3::new(0.0, 0.0, 10.0),
                Point3::new(100.0, 0.0, 12.0),
                two_way(),
            )
            .unwrap()
            .with_name("a"),
        )
        .unwrap();
    let b = builder
        .add_road(
            RoadSpec::line(
                Point3::new(100.0, 0.0, 12.0),
                Point3::new(200.0, 0.0, 14.0),
                two_way(),
            )
            .unwrap()
            .with_name("b"),
        )
        .unwrap();
    builder.connect(&a, &b).unwrap();
    finish(builder)
}

/// 5. One road fanning out into two through a junction.
pub fn split() -> ValidatedMap {
    let mut builder = MapBuilder::new(metadata("split"));
    let trunk = builder
        .add_road(
            RoadSpec::line(
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(100.0, 0.0, 0.0),
                one_way(1),
            )
            .unwrap()
            .with_name("trunk"),
        )
        .unwrap();
    let straight_on = builder
        .add_road(
            RoadSpec::line(
                Point3::new(130.0, 0.0, 0.0),
                Point3::new(230.0, 0.0, 0.0),
                one_way(1),
            )
            .unwrap()
            .with_name("straight_on"),
        )
        .unwrap();
    let slip = builder
        .add_road(
            RoadSpec::line(
                Point3::new(130.0, -25.0, 0.0),
                Point3::new(200.0, -80.0, 0.0),
                one_way(1),
            )
            .unwrap()
            .with_name("slip"),
        )
        .unwrap();
    let junction = builder.add_junction(Some("fork"));
    builder
        .connect_via(&junction, &trunk, &straight_on)
        .unwrap();
    builder.connect_via(&junction, &trunk, &slip).unwrap();
    finish(builder)
}

/// 6. Two roads coming together into one through a junction.
pub fn merge() -> ValidatedMap {
    let mut builder = MapBuilder::new(metadata("merge"));
    let main = builder
        .add_road(
            RoadSpec::line(
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(100.0, 0.0, 0.0),
                one_way(1),
            )
            .unwrap()
            .with_name("main"),
        )
        .unwrap();
    let on_ramp = builder
        .add_road(
            RoadSpec::line(
                Point3::new(20.0, -60.0, 0.0),
                Point3::new(100.0, -20.0, 0.0),
                one_way(1),
            )
            .unwrap()
            .with_name("on_ramp"),
        )
        .unwrap();
    let onward = builder
        .add_road(
            RoadSpec::line(
                Point3::new(130.0, 0.0, 0.0),
                Point3::new(230.0, 0.0, 0.0),
                one_way(1),
            )
            .unwrap()
            .with_name("onward"),
        )
        .unwrap();
    let junction = builder.add_junction(Some("merge"));
    builder.connect_via(&junction, &main, &onward).unwrap();
    builder.connect_via(&junction, &on_ramp, &onward).unwrap();
    finish(builder)
}

/// 7. A four-way crossroads: four two-way approaches, every turn enumerated.
pub fn crossroads() -> ValidatedMap {
    finish(crossroads_builder("crossroads", 70.0))
}

/// The same crossroads, with arms `reach` metres long, before it is finished.
///
/// Long arms are what give a test frontages worth generating on; taking the builder
/// back is what lets a caller add to the map before it is validated.
pub fn crossroads_builder(name: &str, reach: f64) -> MapBuilder {
    let mut builder = MapBuilder::new(metadata(name));
    // Each approach points at the centre, so all four meet at their `End`.
    let arms = [
        (
            "north",
            Point3::new(0.0, reach, 0.0),
            Point3::new(0.0, 14.0, 0.0),
        ),
        (
            "east",
            Point3::new(reach, 0.0, 0.0),
            Point3::new(14.0, 0.0, 0.0),
        ),
        (
            "south",
            Point3::new(0.0, -reach, 0.0),
            Point3::new(0.0, -14.0, 0.0),
        ),
        (
            "west",
            Point3::new(-reach, 0.0, 0.0),
            Point3::new(-14.0, 0.0, 0.0),
        ),
    ];
    let mut roads = Vec::new();
    for (name, start, end) in arms {
        roads.push(
            builder
                .add_road(
                    RoadSpec::line(start, end, two_way())
                        .unwrap()
                        .with_name(name),
                )
                .unwrap(),
        );
    }
    let junction = builder.add_junction(Some("x"));
    for (index, from) in roads.iter().enumerate() {
        for to in roads.iter().skip(index + 1) {
            builder
                .connect_ends(from, RoadEnd::End, to, RoadEnd::End, Some(&junction))
                .unwrap();
        }
    }
    builder
}

/// 8. A road that climbs, then levels off, then climbs again.
pub fn graded_road() -> ValidatedMap {
    let mut builder = MapBuilder::new(metadata("graded"));
    builder
        .add_road(
            RoadSpec::new(
                Curve3::polyline([
                    Point3::new(0.0, 0.0, 0.0),
                    Point3::new(100.0, 0.0, 6.0),
                    Point3::new(200.0, 0.0, 6.0),
                    Point3::new(300.0, 0.0, 15.0),
                ])
                .unwrap(),
                two_way(),
            )
            .with_name("hill"),
        )
        .unwrap();
    finish(builder)
}

/// A road given as a polyline that actually bends — 37° left, 37° right, 45°
/// right — so its vertices are corners. Each is rounded into an arc the way a joint
/// between two roads is, which is what lets every format agree on it.
pub fn bent_polyline() -> ValidatedMap {
    let mut builder = MapBuilder::new(metadata("bent"));
    builder
        .add_road(
            RoadSpec::new(
                Curve3::polyline([
                    Point3::new(0.0, 0.0, 0.0),
                    Point3::new(60.0, 0.0, 1.0),
                    Point3::new(100.0, 30.0, 2.0),
                    Point3::new(160.0, 30.0, 3.0),
                    Point3::new(200.0, -10.0, 4.0),
                ])
                .unwrap(),
                two_way(),
            )
            .with_name("wiggle"),
        )
        .unwrap();
    finish(builder)
}

/// A proper alignment: straight, transition, bend, transition, straight.
///
/// The transitions are clothoids, which is how a road actually enters a bend — and
/// the reason OpenDRIVE has a `<spiral>` element at all.
pub fn spiral_transition_road() -> ValidatedMap {
    let radius = 120.0;
    let alignment = Alignment::new(Point3::new(0.0, 0.0, 4.0), 0.0)
        .line(80.0, 1.0)
        .unwrap()
        .spiral(60.0, 1.0 / radius, 1.0)
        .unwrap()
        .arc(140.0, 1.0 / radius, 2.0)
        .unwrap()
        .spiral(60.0, 0.0, 1.0)
        .unwrap()
        .line(80.0, 1.0)
        .unwrap()
        .finish()
        .unwrap();

    let mut builder = MapBuilder::new(metadata("spiral"));
    builder
        .add_road(
            RoadSpec::new(alignment, two_way())
                .with_name("sweep")
                .with_speed_limit(SpeedLimit::from_kph(80.0).unwrap()),
        )
        .unwrap();
    finish(builder)
}

/// A bend banked the way a fast one is: level on the approach, rolled through the
/// curve, level again on the way out.
pub fn banked_curve() -> ValidatedMap {
    let radius = 150.0;
    let alignment = Alignment::new(Point3::new(0.0, 0.0, 0.0), 0.0)
        .line(50.0, 0.0)
        .unwrap()
        .spiral(50.0, 1.0 / radius, 0.0)
        .unwrap()
        .arc(120.0, 1.0 / radius, 0.0)
        .unwrap()
        .finish()
        .unwrap();
    // Flat to the start of the transition, rolled by the time the bend proper
    // begins, and held through it. The left side rises, which for a left-hand bend
    // is the outside going down — a right-hand bend would use the opposite sign.
    let superelevation =
        Poly3Profile::piecewise_linear([(0.0, 0.0), (50.0, 0.0), (100.0, -0.06), (220.0, -0.06)])
            .unwrap();

    let mut builder = MapBuilder::new(metadata("banked"));
    builder
        .add_road(
            RoadSpec::new(alignment, two_way())
                .with_name("bend")
                .with_superelevation(superelevation),
        )
        .unwrap();
    finish(builder)
}

/// A lane drop: three lanes one way, the outer one tapering away and then ending.
///
/// This is both halves of a changing cross-section at once. The taper is a width
/// profile — the lane stays one lane while it narrows — and the drop is a second
/// cross-section, because past it there is one lane fewer.
pub fn lane_drop() -> ValidatedMap {
    let taper_start = 120.0;
    let drop = 200.0;
    let outer = lane(3.5, Direction::Forward).with_width_profile(
        WidthProfile::tapered(
            taper_start,
            drop - taper_start,
            PositiveWidth::new(3.5).unwrap(),
            PositiveWidth::new(0.4).unwrap(),
            Taper::Smooth,
        )
        .unwrap(),
    );

    let mut builder = MapBuilder::new(metadata("lane-drop"));
    builder
        .add_road(
            RoadSpec::line(
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(320.0, 0.0, 0.0),
                vec![
                    lane(3.5, Direction::Forward),
                    lane(3.5, Direction::Forward),
                    outer,
                ],
            )
            .unwrap()
            .with_name("wide")
            // Past the drop the outer lane is gone and two remain.
            .with_cross_section(
                drop,
                vec![lane(3.5, Direction::Forward), lane(3.5, Direction::Forward)],
            ),
        )
        .unwrap();
    finish(builder)
}

/// A carriageway that widens out into a lay-by and narrows back again, without ever
/// changing how many lanes it has.
pub fn widening_road() -> ValidatedMap {
    let shoulder = lane(2.0, Direction::Forward)
        .with_type(LaneType::Shoulder)
        .with_width_profile(
            WidthProfile::new(
                [
                    (0.0, PositiveWidth::new(2.0).unwrap()),
                    (60.0, PositiveWidth::new(2.0).unwrap()),
                    (100.0, PositiveWidth::new(5.0).unwrap()),
                    (160.0, PositiveWidth::new(5.0).unwrap()),
                    (200.0, PositiveWidth::new(2.0).unwrap()),
                ],
                Taper::Linear,
            )
            .unwrap(),
        );

    let mut builder = MapBuilder::new(metadata("widening"));
    builder
        .add_road(
            RoadSpec::line(
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(260.0, 0.0, 0.0),
                vec![lane(3.5, Direction::Forward), shoulder],
            )
            .unwrap()
            .with_name("layby"),
        )
        .unwrap();
    finish(builder)
}

/// A crossroads with the traffic control a real one has: stop lines, lights and a
/// right of way. Used to check that semantics reach the Lanelet2 map.
pub fn controlled_crossroads() -> ValidatedMap {
    let mut builder = MapBuilder::new(metadata("controlled"));
    let arms = [
        (
            "north",
            Point3::new(0.0, 70.0, 0.0),
            Point3::new(0.0, 14.0, 0.0),
        ),
        (
            "east",
            Point3::new(70.0, 0.0, 0.0),
            Point3::new(14.0, 0.0, 0.0),
        ),
    ];
    let mut roads = Vec::new();
    for (name, start, end) in arms {
        roads.push(
            builder
                .add_road(
                    RoadSpec::line(start, end, two_way())
                        .unwrap()
                        .with_name(name),
                )
                .unwrap(),
        );
    }
    let junction = builder.add_junction(Some("x"));
    builder
        .connect_ends(
            &roads[0],
            RoadEnd::End,
            &roads[1],
            RoadEnd::End,
            Some(&junction),
        )
        .unwrap();

    let north_approach = LaneRef::new(roads[0].clone(), 0);
    let east_approach = LaneRef::new(roads[1].clone(), 0);
    let stop_line = builder
        .add_stop_line(&north_approach, LaneEnd::End)
        .unwrap();
    let light = builder
        .add_traffic_light(&north_approach, LaneEnd::End, 5.0)
        .unwrap();
    builder.add_traffic_light_rule(
        vec![light],
        Some(stop_line.clone()),
        vec![north_approach.clone()],
    );
    builder.add_right_of_way(vec![east_approach], vec![north_approach], Some(stop_line));
    builder.add_crosswalk(&roads[0], 0.8, 4.0).unwrap();
    finish(builder)
}
