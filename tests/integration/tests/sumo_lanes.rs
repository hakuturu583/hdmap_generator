//! What SUMO makes of each lane: who may use it, how wide it is and how fast.
//!
//! The lane-level counterpart to `sumo.rs`. Everything the export knows about a lane
//! beyond its shape is three attributes — the classes it admits, its width and its
//! speed — and the edge it belongs to carries two more, its speed and its priority.
//! Each of them is checked here where it ends up: in the `.net.xml` netconvert
//! builds from the export, read back independently, so a value netconvert rewrote,
//! dropped or defaulted shows up as what SUMO will actually simulate.
//!
//! When SUMO is not installed these skip, saying so; CI sets `ROADGEN_REQUIRE_SUMO`,
//! which turns the skip into a failure.

use roadgen_core::prelude::*;
use roadgen_integration_tests::scenarios;
use roadgen_integration_tests::sumo_build;

/// A map of the one road, built and validated.
fn one_road(name: &str, spec: RoadSpec) -> ValidatedMap {
    let mut builder = MapBuilder::new(scenarios::metadata(name));
    builder.add_road(spec).unwrap();
    builder.finish().unwrap().validate().unwrap()
}

/// A cycle lane is for bicycles and a hard shoulder for the emergency services, and
/// SUMO is told so with an `allow` naming the one class — which netconvert keeps as
/// it is rather than widening or rewriting it.
#[test]
fn a_cycle_lane_admits_bicycles_and_a_shoulder_the_emergency_services() {
    if !sumo_build::sumo_available() {
        return;
    }
    let map = one_road(
        "classes",
        RoadSpec::line(
            Point3::ORIGIN,
            Point3::new(150.0, 0.0, 0.0),
            vec![
                scenarios::lane(3.5, Direction::Forward),
                scenarios::lane(1.5, Direction::Forward).with_type(LaneType::Biking),
                scenarios::lane(2.5, Direction::Forward).with_type(LaneType::Shoulder),
            ],
        )
        .unwrap()
        .with_name("street"),
    );
    let (_directory, network) = sumo_build::build(&map);

    let street = network.edge("street.fwd");
    assert_eq!(street.lanes.len(), 3);
    // Outermost first: the shoulder at the kerb, the cycle lane inside it.
    let shoulder = street.lane(0);
    assert_eq!(shoulder.allow.as_deref(), Some("emergency"));
    assert_eq!(shoulder.disallow, None);
    let cycle = street.lane(1);
    assert_eq!(cycle.allow.as_deref(), Some("bicycle"));
    assert_eq!(cycle.disallow, None);
    assert_eq!(street.lane(2).disallow.as_deref(), Some("pedestrian"));
}

/// A parking bay, a restricted lane and a border are not lanes in SUMO's sense, and
/// dropping them leaves the network's numbering whole: the edge has as many lanes as
/// were written, numbered from 0 with none skipped, and each is the lane the lane
/// map says it is — even though a dropped lane sat between two of them.
#[test]
fn a_dropped_lane_leaves_no_gap_in_the_built_network() {
    if !sumo_build::sumo_available() {
        return;
    }
    let map = one_road(
        "dropped",
        RoadSpec::line(
            Point3::ORIGIN,
            Point3::new(150.0, 0.0, 0.0),
            vec![
                scenarios::lane(3.5, Direction::Forward),
                scenarios::lane(2.5, Direction::Forward).with_type(LaneType::Restricted),
                scenarios::lane(3.5, Direction::Forward),
                scenarios::lane(2.5, Direction::Forward).with_type(LaneType::Parking),
                scenarios::lane(0.5, Direction::Forward).with_type(LaneType::Border),
                scenarios::lane(2.0, Direction::Forward).with_type(LaneType::Sidewalk),
            ],
        )
        .unwrap()
        .with_name("street"),
    );
    let lane_map = roadgen_sumo::to_plain_xml(&map).unwrap().lanes;
    let (_directory, network) = sumo_build::build(&map);

    assert_eq!(network.roads().len(), 1);
    let street = network.edge("street.fwd");
    let numbering: Vec<(usize, &str)> = street
        .lanes
        .iter()
        .map(|lane| (lane.index, lane.id.as_str()))
        .collect();
    assert_eq!(
        numbering,
        [
            (0, "street.fwd_0"),
            (1, "street.fwd_1"),
            (2, "street.fwd_2")
        ]
    );
    assert_eq!(street.lane(0).allow.as_deref(), Some("pedestrian"));

    // Every lane the export says it wrote is in the network under that id, and
    // every lane of a type SUMO has no place for is absent from the lane map.
    for lane in map.lanes.iter() {
        match roadgen_sumo::classes::permission(lane.lane_type) {
            Some(_) => {
                let id = &lane_map[&lane.id];
                assert!(
                    street.lanes.iter().any(|built| &built.id == id),
                    "{} was written as {id}, which netconvert did not build",
                    lane.id
                );
            }
            None => assert!(
                !lane_map.contains_key(&lane.id),
                "{} is a {} lane, which SUMO has no place for",
                lane.id,
                lane.lane_type.as_str()
            ),
        }
    }
}

/// A tapering lane reaches the simulator at its mean width along, which for the
/// widening scenario's lay-by can be worked out by hand: a 260 m road whose shoulder
/// runs at 2 m, ramps to 5 m between 60 m and 100 m, holds it to 160 m and ramps back
/// to 2 m by 200 m. That is 2 m over 120 m, 5 m over 60 m and 3.5 m on average over
/// the two 40 m ramps — 820 m² over 260 m. netconvert writes widths to the
/// centimetre, which is the whole of the tolerance.
#[test]
fn a_tapering_lane_is_built_at_its_mean_width_along() {
    if !sumo_build::sumo_available() {
        return;
    }
    let map = scenarios::widening_road();
    let (_directory, network) = sumo_build::build(&map);

    let analytic = (2.0 * 120.0 + 5.0 * 60.0 + 3.5 * 80.0) / 260.0;
    let layby = network.edge("layby.fwd");
    // The shoulder is the outer lane, so the kerbside lane 0.
    let shoulder = layby.lane(0);
    assert_eq!(shoulder.allow.as_deref(), Some("emergency"));
    assert!(
        (shoulder.width - analytic).abs() < 0.006,
        "the shoulder was built {} m wide, but its mean along is {analytic} m",
        shoulder.width
    );
    assert!((layby.lane(1).width - 3.5).abs() < 0.006);
}

/// A lane's own limit is the speed SUMO gives the lane, and a lane on a road that
/// states no limit runs at its road type's fallback.
#[test]
fn a_lane_runs_at_its_own_limit_or_its_road_types_fallback() {
    if !sumo_build::sumo_available() {
        return;
    }
    let mut builder = MapBuilder::new(scenarios::metadata("speeds"));
    builder
        .add_road(
            RoadSpec::line(
                Point3::ORIGIN,
                Point3::new(150.0, 0.0, 0.0),
                vec![
                    scenarios::lane(3.5, Direction::Forward)
                        .with_speed_limit(SpeedLimit::from_kph(30.0).unwrap()),
                    scenarios::lane(3.5, Direction::Forward)
                        .with_speed_limit(SpeedLimit::from_kph(60.0).unwrap()),
                ],
            )
            .unwrap()
            .with_name("limited")
            .with_type(RoadType::Rural),
        )
        .unwrap();
    builder
        .add_road(
            RoadSpec::line(
                Point3::new(0.0, 50.0, 0.0),
                Point3::new(150.0, 50.0, 0.0),
                vec![scenarios::lane(3.5, Direction::Forward)],
            )
            .unwrap()
            .with_name("unlimited")
            .with_type(RoadType::Rural),
        )
        .unwrap();
    let map = builder.finish().unwrap().validate().unwrap();
    let lane_map = roadgen_sumo::to_plain_xml(&map).unwrap().lanes;
    let (_directory, network) = sumo_build::build(&map);

    // netconvert writes speeds to the centimetre per second.
    let close = |found: f64, wanted: f64| (found - wanted).abs() < 0.006;
    let limited = network.edge("limited.fwd");
    for lane in map
        .lanes
        .iter()
        .filter(|lane| lane.road.local_name() == "limited")
    {
        let id = &lane_map[&lane.id];
        let built = limited.lanes.iter().find(|built| &built.id == id).unwrap();
        let wanted = lane.speed_limit.unwrap().mps();
        assert!(
            close(built.speed, wanted),
            "{id} runs at {} m/s rather than its own {wanted} m/s",
            built.speed
        );
    }
    let fallback = roadgen_sumo::classes::default_speed(RoadType::Rural);
    let unlimited = network.edge("unlimited.fwd").lane(0);
    assert!(
        close(unlimited.speed, fallback),
        "a rural road with no limit runs at {} m/s rather than {fallback} m/s",
        unlimited.speed
    );
}

/// The road types rank in the built network as they do on the road: a motorway above
/// a rural road above a town street above a living street above a footway, by the
/// priority netconvert reads to decide who yields, and in the same order by speed.
/// The numbers themselves are not checked — only the order is what netconvert acts
/// on, and the numbers may move as long as it holds.
#[test]
fn the_road_types_rank_in_the_built_network_as_they_do_on_the_road() {
    if !sumo_build::sumo_available() {
        return;
    }
    let ladder = [
        RoadType::Motorway,
        RoadType::Rural,
        RoadType::Town,
        RoadType::LowSpeed,
        RoadType::Pedestrian,
    ];
    let mut builder = MapBuilder::new(scenarios::metadata("ladder"));
    for (row, road_type) in ladder.into_iter().enumerate() {
        let y = 50.0 * row as f64;
        builder
            .add_road(
                RoadSpec::line(
                    Point3::new(0.0, y, 0.0),
                    Point3::new(150.0, y, 0.0),
                    vec![scenarios::lane(3.5, Direction::Forward)],
                )
                .unwrap()
                .with_name(road_type.as_str())
                .with_type(road_type),
            )
            .unwrap();
    }
    let map = builder.finish().unwrap().validate().unwrap();
    let (_directory, network) = sumo_build::build(&map);

    let edge = |road_type: RoadType| network.edge(&format!("{}.fwd", road_type.as_str()));
    for road_type in ladder {
        let wanted = roadgen_sumo::classes::default_speed(road_type);
        let found = edge(road_type).lane(0).speed;
        assert!(
            (found - wanted).abs() < 0.006,
            "a {road_type:?} road with no limit runs at {found} m/s rather than {wanted} m/s"
        );
    }
    for pair in ladder.windows(2) {
        let (higher, lower) = (edge(pair[0]), edge(pair[1]));
        assert!(
            higher.priority.unwrap() > lower.priority.unwrap(),
            "a {:?} road should outrank a {:?} one",
            pair[0],
            pair[1]
        );
        assert!(
            higher.lane(0).speed > lower.lane(0).speed,
            "a {:?} road should be faster than a {:?} one",
            pair[0],
            pair[1]
        );
    }
}

/// A road named with characters SUMO cannot take in an id — a space, a colon — is
/// built under the id the export made of its name, with the characters SUMO can take
/// left alone: `#` is one of them, being what SUMO's own OpenStreetMap import uses to
/// number the pieces of an edge it splits. netconvert accepting the network without
/// complaint is half of the check; finding every edge under the expected id is the
/// other.
#[test]
fn a_road_name_with_characters_sumo_reserves_still_builds() {
    if !sumo_build::sumo_available() {
        return;
    }
    let mut builder = MapBuilder::new(scenarios::metadata("names"));
    for (row, name) in ["north arm", "ring:inner", "route#2"]
        .into_iter()
        .enumerate()
    {
        let y = 50.0 * row as f64;
        builder
            .add_road(
                RoadSpec::line(
                    Point3::new(0.0, y, 0.0),
                    Point3::new(150.0, y, 0.0),
                    scenarios::two_way(),
                )
                .unwrap()
                .with_name(name),
            )
            .unwrap();
    }
    let map = builder.finish().unwrap().validate().unwrap();
    let (_directory, network) = sumo_build::build(&map);

    for (id, name) in [
        ("north_arm", "north arm"),
        ("ring_inner", "ring:inner"),
        ("route#2", "route#2"),
    ] {
        for sense in ["fwd", "bwd"] {
            let edge = network.edge(&format!("{id}.{sense}"));
            // The name is a label, not an id, so it keeps what the map called it.
            assert_eq!(edge.name.as_deref(), Some(name));
            assert_eq!(edge.lane(0).id, format!("{id}.{sense}_0"));
        }
    }
    assert_eq!(network.roads().len(), 6);
}
