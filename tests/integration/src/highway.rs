//! A motorway, put together the way a caller does it: a river it bridges, a ridge
//! it tunnels through, a diamond interchange onto a town's crossroad, and its far
//! end becoming an ordinary road — every structure chosen by the rules, from the
//! land.
//!
//! Written once so every test of what a motorway comes to starts from the same one.

use roadgen_buildings::Rules;
use roadgen_carla::environment::{
    Control, CorridorPlan, CorridorSpec, HillSpec, InterchangeKind, InterchangeSite, Places,
    RiverSpec,
};
use roadgen_core::prelude::*;

/// The crossroad's cross-section: a lane each way and a pavement each side.
pub fn street_lanes() -> Vec<LaneSpec> {
    let lane = |width: f64, direction, lane_type| {
        LaneSpec::new(PositiveWidth::new(width).unwrap(), direction).with_type(lane_type)
    };
    vec![
        lane(3.5, Direction::Backward, LaneType::Driving),
        lane(2.5, Direction::Backward, LaneType::Sidewalk),
        lane(3.5, Direction::Forward, LaneType::Driving),
        lane(2.5, Direction::Forward, LaneType::Sidewalk),
    ]
}

/// The corridor: west to east along y = 0, then bending north-east.
pub fn corridor() -> CorridorSpec {
    let mut spec = CorridorSpec::new(
        "E1",
        vec![
            Point3::new(-2000.0, 0.0, 0.0),
            Point3::new(1000.0, 0.0, 0.0),
            Point3::new(3000.0, 700.0, 0.0),
        ],
        100.0,
    )
    .unwrap();
    // Up onto the river crossing, level through the interchange, down at the end.
    spec.profile = vec![
        (0.0, 0.0),
        (500.0, 10.0),
        (800.0, 10.0),
        (1250.0, 1.0),
        (4300.0, 4.0),
        (4900.0, 0.0),
    ];
    spec.interchanges.push(InterchangeSite {
        name: "Kita IC".into(),
        kind: InterchangeKind::Diamond,
        at: 1700.0,
        crossroad: vec![
            Point3::new(-300.0, -450.0, 0.0),
            Point3::new(-300.0, 450.0, 0.0),
        ],
        crossroad_lanes: street_lanes(),
        over: true,
        control: Control::Signal,
        terminal: 80.0,
        reach: 320.0,
    });
    spec.end_transition = Some(400.0);
    spec
}

/// What is generated onto the motorway once its roads are built.
pub fn places() -> Places {
    Places {
        hills: vec![HillSpec {
            name: "Kita Ridge".into(),
            crest: vec![
                Point3::new(1650.0, 1350.0, 0.0),
                Point3::new(2350.0, -650.0, 0.0),
            ],
            width: 700.0,
            height: 50.0,
        }],
        rivers: vec![RiverSpec {
            name: "Ara River".into(),
            centre: vec![
                Point3::new(-1450.0, -600.0, 0.0),
                Point3::new(-1350.0, 600.0, 0.0),
            ],
            width: 80.0,
            depth: 6.0,
            bank_slope: 2.0,
            site: None,
        }],
        corridors: vec![corridor()],
        town: Some(Rules::preset("town").unwrap()),
        ..Places::default()
    }
}

/// The motorway's plan, for `handedness`.
pub fn plan(handedness: TrafficHandedness) -> CorridorPlan {
    corridor().plan(handedness).expect("the corridor plans")
}

/// The motorway, built and generated but not validated.
pub fn build(
    handedness: TrafficHandedness,
) -> (UnvalidatedMap, roadgen_carla::environment::Report) {
    let mut builder = MapBuilder::new(MapMetadata {
        name: Some("expressway".into()),
        handedness,
        ..MapMetadata::default()
    });
    let mut places = places();
    plan(handedness)
        .apply(&mut builder, &mut places)
        .expect("the plan goes on the map");
    let mut map = builder.finish().expect("the roads build");
    let report = roadgen_carla::environment::generate(&mut map, &places).expect("generates");
    (map, report)
}

/// The reference motorway, validated.
pub fn highway() -> ValidatedMap {
    let (map, _) = build(TrafficHandedness::RightHand);
    map.validate().expect("the motorway validates")
}
