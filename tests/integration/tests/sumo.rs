//! Exporting to SUMO.
//!
//! Every one of these tests goes through SUMO itself. The plain XML is handed to
//! `netconvert`, which is the program that defines what the format means, and the
//! `.net.xml` that comes back is read independently — so what is being checked is
//! never the exporter's own opinion of what it wrote, but what SUMO made of it. The
//! first test goes one step further and loads the built network into the simulator.
//!
//! When SUMO is not installed these skip, saying so; CI sets `ROADGEN_REQUIRE_SUMO`,
//! which turns the skip into a failure.

use roadgen_core::prelude::*;
use roadgen_integration_tests::scenarios;
use roadgen_integration_tests::sumo_build::{self, SumoNetwork};

/// Every scenario, built and loaded — the check that the export is a network SUMO
/// will actually run, rather than a file that merely parses.
#[test]
fn every_scenario_builds_with_netconvert_and_loads_in_sumo() {
    if !sumo_build::sumo_available() {
        return;
    }
    let scenarios: Vec<(&str, ValidatedMap)> = vec![
        ("straight", scenarios::straight_road()),
        ("bidirectional", scenarios::bidirectional_road()),
        ("multi-lane", scenarios::multi_lane_road()),
        ("joined", scenarios::two_roads_joined()),
        ("in-line", scenarios::two_roads_in_line()),
        ("split", scenarios::split()),
        ("merge", scenarios::merge()),
        ("crossroads", scenarios::crossroads()),
        ("controlled", scenarios::controlled_crossroads()),
        ("graded", scenarios::graded_road()),
        ("spiral", scenarios::spiral_transition_road()),
        ("banked", scenarios::banked_curve()),
        ("lane drop", scenarios::lane_drop()),
        ("widening", scenarios::widening_road()),
    ];

    for (name, map) in scenarios {
        let prefix = roadgen_sumo::network_name(&map);
        let (directory, network) = sumo_build::build(&map);
        assert!(
            !network.roads().is_empty(),
            "{name} built into a network with no edges"
        );
        sumo_build::simulate(directory.path(), &prefix);
    }
}

/// A road carrying traffic both ways is two edges, because a SUMO edge is one way.
#[test]
fn a_two_way_road_is_an_edge_in_each_direction() {
    if !sumo_build::sumo_available() {
        return;
    }
    let map = scenarios::bidirectional_road();
    let (_directory, network) = sumo_build::build(&map);

    let roads = network.roads();
    assert_eq!(roads.len(), 2, "one edge per direction of travel");
    let forward = network.edge("main.fwd");
    let backward = network.edge("main.bwd");
    assert_eq!(forward.lanes.len(), 1);
    assert_eq!(backward.lanes.len(), 1);
    // They are the same road, so they run between the same two nodes the other way
    // round.
    assert_eq!(forward.from, backward.to);
    assert_eq!(forward.to, backward.from);
    assert_eq!(forward.name.as_deref(), Some("main"));

    // And each is drawn along its own side: the forward carriageway is to the right
    // of the reference line where traffic drives on the right.
    let forward_y = forward.lane(0).shape[0].y;
    let backward_y = backward.lane(0).shape[0].y;
    assert!(
        forward_y < 0.0 && backward_y > 0.0,
        "forward at {forward_y}, backward at {backward_y}"
    );
}

/// SUMO numbers an edge's lanes from the right of the direction of travel. The IR
/// counts outwards from the reference line, which for the opposing carriageway is the
/// other way round, so getting this wrong mirrors one side of every road.
#[test]
fn lanes_are_numbered_from_the_right_of_travel() {
    if !sumo_build::sumo_available() {
        return;
    }
    let mut builder = MapBuilder::new(scenarios::metadata("numbering"));
    builder
        .add_road(
            RoadSpec::line(
                Point3::ORIGIN,
                Point3::new(200.0, 0.0, 0.0),
                vec![
                    scenarios::lane(3.5, Direction::Forward),
                    scenarios::lane(3.5, Direction::Forward),
                    scenarios::lane(3.5, Direction::Backward),
                    scenarios::lane(3.5, Direction::Backward),
                ],
            )
            .unwrap()
            .with_name("dual"),
        )
        .unwrap();
    let map = builder.finish().unwrap().validate().unwrap();
    let (_directory, network) = sumo_build::build(&map);

    // Travelling along +x, the driver's right is -y, so lane 0 is the one with the
    // most negative offset.
    let forward = network.edge("dual.fwd");
    assert!(
        forward.lane(0).shape[0].y < forward.lane(1).shape[0].y,
        "lane 0 of the forward carriageway should be its rightmost"
    );
    // Travelling along -x, the driver's right is +y — the opposite sense, on the
    // opposite side of the reference line.
    let backward = network.edge("dual.bwd");
    assert!(
        backward.lane(0).shape[0].y > backward.lane(1).shape[0].y,
        "lane 0 of the backward carriageway should be its rightmost"
    );
    // Whichever carriageway, lane 0 is the outside of the road.
    assert!(forward.lane(0).shape[0].y < -3.5);
    assert!(backward.lane(0).shape[0].y > 3.5);
}

/// Every lane is written with its own shape, so what SUMO has is the geometry the
/// generator computed and not a centreline with a width laid out from it.
#[test]
fn a_lane_keeps_the_geometry_the_generator_computed() {
    if !sumo_build::sumo_available() {
        return;
    }
    for map in [
        scenarios::multi_lane_road(),
        scenarios::spiral_transition_road(),
        scenarios::graded_road(),
    ] {
        let network_lanes = roadgen_sumo::to_plain_xml(&map).unwrap().lanes;
        let (_directory, network) = sumo_build::build(&map);

        for lane in map.lanes.iter() {
            let Some(written) = network_lanes.get(&lane.id) else {
                continue;
            };
            let (edge, index) = written.rsplit_once('_').expect("an edge and an index");
            let built = network.edge(edge).lane(index.parse().unwrap());
            let expected = lane
                .travel_geometry(map.metadata.sampling)
                .unwrap()
                .centerline
                .to_polyline(map.metadata.sampling)
                .unwrap();

            assert_eq!(
                built.shape.len(),
                expected.len(),
                "{} kept a different number of vertices",
                lane.id
            );
            for (found, wanted) in built.shape.iter().zip(expected.points()) {
                // netconvert writes its output to the centimetre, which is the whole
                // of the difference: nothing is resampled or re-laid-out.
                assert!(
                    found.distance_to(*wanted) < 0.02,
                    "{} landed at {found:?} rather than {wanted:?}",
                    lane.id
                );
            }
        }
    }
}

/// Two roads that continue into each other meet at one node, which is what makes the
/// pair one road as far as a route is concerned.
#[test]
fn roads_that_continue_into_each_other_share_a_node() {
    if !sumo_build::sumo_available() {
        return;
    }
    let map = scenarios::two_roads_in_line();
    let (_directory, network) = sumo_build::build(&map);

    let first = network.edge("a.fwd");
    let second = network.edge("b.fwd");
    assert_eq!(
        first.to, second.from,
        "the joint should be one node, not two on top of each other"
    );
    // Both carriageways use it, in opposite senses.
    assert_eq!(network.edge("b.bwd").to, first.to);
    assert_eq!(network.edge("a.bwd").from, first.to);

    let movements = network.movements();
    assert!(movements.contains(&("a.fwd", 0, "b.fwd", 0)));
    assert!(movements.contains(&("b.bwd", 0, "a.bwd", 0)));
    assert!(
        !movements.contains(&("a.fwd", 0, "a.bwd", 0)),
        "nothing turns round in the middle of a road: {movements:?}"
    );
}

/// The gap the IR leaves between a junction's arms *is* the junction, and SUMO fills
/// it with an internal lane per movement.
#[test]
fn a_junction_is_a_node_the_arms_stop_short_of() {
    if !sumo_build::sumo_available() {
        return;
    }
    let map = scenarios::crossroads();
    let (_directory, network) = sumo_build::build(&map);

    let junction = map.junctions.iter().next().unwrap();
    let centre = map.junction_centre(&junction.id).unwrap();
    let node = network.junction(&format!("j_{}", junction.id.local_name()));
    assert!(
        node.position.horizontal_distance_to(centre) < 0.02,
        "the node sits at {:?} rather than at the arms' crossing {centre:?}",
        node.position
    );

    // The connectors are not edges: what crosses the junction is SUMO's own internal
    // lanes, one per movement.
    assert!(
        network
            .roads()
            .iter()
            .all(|edge| !edge.id.contains("connector")),
        "a connector road was written as an edge"
    );
    assert!(
        network
            .edges
            .iter()
            .any(|edge| edge.function.as_deref() == Some("internal")),
        "netconvert should have drawn the movements across the junction"
    );

    // And every movement across it is carried by one of them.
    let across: Vec<_> = network
        .connections
        .iter()
        .filter(|connection| connection.via.is_some())
        .collect();
    assert!(!across.is_empty(), "no movement crosses the junction");
}

/// SUMO is asked for exactly the movements the IR permits. Its own guess would join
/// every arm to every other, which is the whole reason the connections are written.
#[test]
fn only_the_movements_the_map_permits_are_written() {
    if !sumo_build::sumo_available() {
        return;
    }
    // A tee whose two side arms are joined to the stem but not to each other.
    let mut builder = MapBuilder::new(scenarios::metadata("tee"));
    let arms: Vec<_> = [
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
        (
            "west",
            Point3::new(-70.0, 0.0, 0.0),
            Point3::new(-14.0, 0.0, 0.0),
        ),
    ]
    .into_iter()
    .map(|(name, start, end)| {
        builder
            .add_road(
                RoadSpec::line(start, end, scenarios::two_way())
                    .unwrap()
                    .with_name(name),
            )
            .unwrap()
    })
    .collect();
    let junction = builder.add_junction(Some("t"));
    for other in &arms[1..] {
        builder
            .connect_ends(&arms[0], RoadEnd::End, other, RoadEnd::End, Some(&junction))
            .unwrap();
    }
    let map = builder.finish().unwrap().validate().unwrap();

    let (_directory, network) = sumo_build::build(&map);
    let movements = network.movements();
    let joins = |from: &str, to: &str| {
        movements
            .iter()
            .any(|(source, _, target, _)| *source == from && *target == to)
    };

    assert!(joins("north.fwd", "east.bwd"), "north into east");
    assert!(joins("north.fwd", "west.bwd"), "north into west");
    assert!(joins("east.fwd", "north.bwd"), "east into north");
    assert!(
        !joins("east.fwd", "west.bwd") && !joins("west.fwd", "east.bwd"),
        "the movement across the top of the tee is not in the map: {movements:?}"
    );
}

/// A traffic light in the IR has no timing, so what it can say is *which* junction is
/// signalised. netconvert generates the phases from that.
#[test]
fn a_traffic_light_makes_its_junction_a_signalised_node() {
    if !sumo_build::sumo_available() {
        return;
    }
    let map = scenarios::controlled_crossroads();
    let (_directory, network) = sumo_build::build(&map);

    let junction = map.junctions.iter().next().unwrap();
    let node = network.junction(&format!("j_{}", junction.id.local_name()));
    assert_eq!(node.kind, "traffic_light");
    assert!(
        network
            .connections
            .iter()
            .any(|connection| connection.tl.is_some()),
        "the movements across it should be signal-controlled"
    );
}

/// A right-of-way rule is the one thing the IR says about who waits at an
/// uncontrolled junction, and an edge's priority is the one thing netconvert reads.
#[test]
fn the_arm_that_yields_is_written_below_the_one_it_yields_to() {
    if !sumo_build::sumo_available() {
        return;
    }
    let map = scenarios::controlled_crossroads();
    let (_directory, network) = sumo_build::build(&map);

    // `controlled_crossroads` gives the east arm right of way over the north one.
    let east = network.edge("east.fwd").priority.unwrap();
    let north = network.edge("north.fwd").priority.unwrap();
    assert!(
        east > north,
        "east should outrank north, but they are {east} and {north}"
    );
}

/// A SUMO edge has one lane count from end to end, so a road that drops a lane is a
/// chain of edges with a node between them.
#[test]
fn a_changing_cross_section_becomes_a_chain_of_edges() {
    if !sumo_build::sumo_available() {
        return;
    }
    let map = scenarios::lane_drop();
    let (_directory, network) = sumo_build::build(&map);

    let before = network.edge("wide.0.fwd");
    let after = network.edge("wide.1.fwd");
    assert_eq!(before.lanes.len(), 3);
    assert_eq!(after.lanes.len(), 2);
    assert_eq!(
        before.to, after.from,
        "the two sections should meet at one node"
    );
    // The lanes that carry on do carry on: a lane count that changes is not a break
    // in the road.
    assert!(
        network
            .movements()
            .iter()
            .any(|(from, _, to, _)| *from == "wide.0.fwd" && *to == "wide.1.fwd"),
        "traffic should be able to get from one section to the next"
    );
}

/// What may use a lane is the whole of what SUMO knows about lane type, so it is what
/// the lane types are lowered onto.
#[test]
fn a_footway_admits_pedestrians_and_a_driving_lane_keeps_them_out() {
    if !sumo_build::sumo_available() {
        return;
    }
    let mut builder = MapBuilder::new(scenarios::metadata("footways"));
    builder
        .add_road(
            RoadSpec::line(
                Point3::ORIGIN,
                Point3::new(150.0, 0.0, 0.0),
                vec![
                    scenarios::lane(3.5, Direction::Forward),
                    scenarios::lane(2.0, Direction::Forward).with_type(LaneType::Sidewalk),
                ],
            )
            .unwrap()
            .with_name("street"),
        )
        .unwrap();
    let map = builder.finish().unwrap().validate().unwrap();
    let (_directory, network) = sumo_build::build(&map);

    let street = network.edge("street.fwd");
    assert_eq!(street.lanes.len(), 2);
    // The footway is on the outside, which is lane 0 — SUMO's own convention for a
    // sidewalk, and here it falls out of where the lane actually is.
    assert_eq!(street.lane(0).allow.as_deref(), Some("pedestrian"));
    assert_eq!(street.lane(1).disallow.as_deref(), Some("pedestrian"));
}

/// The heights the generator computed reach SUMO, which is the one thing plain
/// OpenStreetMap could not carry.
#[test]
fn the_network_is_three_dimensional() {
    if !sumo_build::sumo_available() {
        return;
    }
    let map = scenarios::graded_road();
    let (_directory, network) = sumo_build::build(&map);

    let climb = |network: &SumoNetwork| {
        network
            .roads()
            .iter()
            .flat_map(|edge| edge.lanes.iter())
            .flat_map(|lane| lane.shape.iter())
            .fold((f64::MAX, f64::MIN), |(low, high), point| {
                (low.min(point.z), high.max(point.z))
            })
    };
    let (low, high) = climb(&network);
    assert!(
        high - low > 1.0,
        "the road climbs, but the network is flat between {low} and {high}"
    );
}

/// A lane SUMO has no place for is dropped rather than written as something it is
/// not, and the export says which.
#[test]
fn what_the_format_cannot_carry_is_reported() {
    let report = roadgen_sumo::check(&scenarios::widening_road()).join("\n");
    assert!(report.contains("markings"), "{report}");
    assert!(report.contains("mean width along"), "{report}");

    let crossroads = roadgen_sumo::check(&scenarios::controlled_crossroads()).join("\n");
    assert!(
        crossroads.contains("netconvert generates the phases"),
        "{crossroads}"
    );
    assert!(crossroads.contains("crosswalk"), "{crossroads}");
}

/// The four files are named after the map and refer to each other, so that building
/// the network is one command over one configuration.
#[test]
fn the_export_is_a_netconvert_run_ready_to_go() {
    let map = scenarios::crossroads();
    let directory = tempfile::tempdir().unwrap();
    let prefix = roadgen_sumo::write(&map, directory.path()).unwrap();
    assert_eq!(prefix, "crossroads");

    for suffix in [".nod.xml", ".edg.xml", ".con.xml", ".netccfg"] {
        let path = directory.path().join(format!("{prefix}{suffix}"));
        assert!(path.is_file(), "{} was not written", path.display());
    }
    let config =
        std::fs::read_to_string(directory.path().join(format!("{prefix}.netccfg"))).unwrap();
    for suffix in [".nod.xml", ".edg.xml", ".con.xml", ".net.xml"] {
        assert!(
            config.contains(&format!("{prefix}{suffix}")),
            "the configuration should name {prefix}{suffix}:\n{config}"
        );
    }
}
