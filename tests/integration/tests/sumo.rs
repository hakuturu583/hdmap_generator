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

/// SUMO numbers an edge's lanes from the outside, which on a right-hand map is the
/// right of the direction of travel. The IR counts outwards from the reference line,
/// which for the opposing carriageway is the other way round, so getting this wrong
/// mirrors one side of every road.
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

/// The same road, driving on the left. SUMO still numbers from the outside of the
/// carriageway, and the outside is now the driver's left — which is what netconvert
/// itself lays out when it spreads an edge's lanes with `lefthand` set. Numbered from
/// the right instead, the kerb lane would be SUMO's overtaking lane.
#[test]
fn lanes_of_a_left_hand_map_are_numbered_from_the_left_of_travel() {
    if !sumo_build::sumo_available() {
        return;
    }
    let mut builder = MapBuilder::new(scenarios::metadata("numbering"));
    builder.metadata_mut().handedness = TrafficHandedness::LeftHand;
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
    assert!(
        network.lefthand,
        "netconvert should know the map drives on the left"
    );

    // Travelling along +x on the left of the road, the driver's left is +y, so lane 0
    // is the one with the largest offset.
    let forward = network.edge("dual.fwd");
    assert!(
        forward.lane(0).shape[0].y > forward.lane(1).shape[0].y,
        "lane 0 of the forward carriageway should be its leftmost"
    );
    let backward = network.edge("dual.bwd");
    assert!(
        backward.lane(0).shape[0].y < backward.lane(1).shape[0].y,
        "lane 0 of the backward carriageway should be its leftmost"
    );
    // Whichever carriageway, lane 0 is still the outside of the road — on the other
    // side of it from where a right-hand map puts it.
    assert!(forward.lane(0).shape[0].y > 3.5);
    assert!(backward.lane(0).shape[0].y < -3.5);
}

/// netconvert assumes right-hand traffic unless it is told otherwise, and what it
/// assumes decides who gives way. Driving on the left, the turn that crosses the
/// oncoming carriageway is the *right* turn, so on a major approach that is the one
/// that must yield, and the left turn — which stays on the kerb side — keeps right of
/// way. A left-hand map built as a right-hand network gets exactly the opposite.
#[test]
fn a_left_hand_map_is_built_as_a_left_hand_network() {
    if !sumo_build::sumo_available() {
        return;
    }
    let (_directory, right) = sumo_build::build(&scenarios::crossroads());
    assert!(!right.lefthand, "a right-hand map is netconvert's default");

    let mut builder = scenarios::crossroads_builder("left-crossroads", 70.0);
    builder.metadata_mut().handedness = TrafficHandedness::LeftHand;
    let map = builder.finish().unwrap().validate().unwrap();
    let prefix = roadgen_sumo::network_name(&map);
    let (directory, left) = sumo_build::build(&map);
    assert!(
        left.lefthand,
        "netconvert should know the map drives on the left"
    );
    sumo_build::simulate(directory.path(), &prefix);

    // Which pair of arms netconvert makes the major road is its own call, so the
    // check is made on whichever approaches it gave the straight-on movement to.
    let state = |network: &SumoNetwork, from: &str, direction: &str| {
        network
            .connections
            .iter()
            .find(|connection| {
                connection.from == from && connection.direction.as_deref() == Some(direction)
            })
            .and_then(|connection| connection.state.clone())
            .unwrap_or_else(|| panic!("no {direction:?} movement from {from}"))
    };
    for (network, crossing, kerbside) in [(&right, "l", "r"), (&left, "r", "l")] {
        let major: Vec<&str> = ["north.fwd", "east.fwd", "south.fwd", "west.fwd"]
            .into_iter()
            .filter(|from| state(network, from, "s") == "M")
            .collect();
        assert_eq!(major.len(), 2, "netconvert should pick one major road");
        for from in major {
            assert_eq!(
                state(network, from, crossing),
                "m",
                "{from}: the turn across the oncoming carriageway should give way \
                 (lefthand {})",
                network.lefthand
            );
            assert_eq!(
                state(network, from, kerbside),
                "M",
                "{from}: the kerb-side turn should keep right of way (lefthand {})",
                network.lefthand
            );
        }
    }
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

/// A right-of-way rule has to reach the network as *right of way*, not as a hint
/// netconvert is free to weigh against its own reading of the geometry.
///
/// So this reads what netconvert decided — the `state` on each movement, `M` for one
/// that keeps right of way and `m` for one that must give way — rather than the
/// priority the export asked for. The junction is deliberately unsignalised: at a
/// `traffic_light` node the phases decide and the priorities barely matter, which
/// would make the check say nothing.
#[test]
fn a_right_of_way_rule_decides_who_gives_way() {
    if !sumo_build::sumo_available() {
        return;
    }
    let mut builder = MapBuilder::new(scenarios::metadata("priority"));
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
    // The stem of the tee keeps right of way over the road across the top of it —
    // the opposite of what the geometry alone suggests, so nothing but the rule can
    // produce this answer.
    builder.add_right_of_way(
        vec![LaneRef::new(arms[0].clone(), 0)],
        vec![
            LaneRef::new(arms[1].clone(), 0),
            LaneRef::new(arms[2].clone(), 0),
        ],
        None,
    );
    let map = builder.finish().unwrap().validate().unwrap();
    let (_directory, network) = sumo_build::build(&map);

    let gives_way = |from: &str, to: &str| {
        network
            .movement(from, to)
            .state
            .as_deref()
            .unwrap_or_default()
            == "m"
    };
    assert!(
        !gives_way("north.fwd", "east.bwd") && !gives_way("north.fwd", "west.bwd"),
        "the stem was given right of way, so nothing it does should yield: {:?}",
        network.connections
    );
    assert!(
        gives_way("east.fwd", "north.bwd") || gives_way("west.fwd", "north.bwd"),
        "the arms that yield to it should give way: {:?}",
        network.connections
    );

    // And the rule named the approach lanes, so it moved the approach edges and left
    // the opposing carriageways where their road type put them.
    let base = network.edge("north.bwd").priority.unwrap();
    assert_eq!(network.edge("east.bwd").priority.unwrap(), base);
    assert_eq!(network.edge("west.bwd").priority.unwrap(), base);
    assert!(
        network.edge("north.fwd").priority.unwrap() > network.edge("east.fwd").priority.unwrap(),
        "the approach that keeps right of way should outrank the one that yields"
    );
}

/// A SUMO edge has one lane count from end to end, so a road that drops a lane is a
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
    // The IR has no U-turns, so netconvert is not to invent one at every dead end.
    assert!(
        config.contains(r#"<no-turnarounds value="true"/>"#),
        "{config}"
    );
    // A right-hand map is netconvert's default, so nothing is said about handedness.
    assert!(!config.contains("lefthand"), "{config}");
}

/// The trace names each connection by its two lanes as the built network names them,
/// so every movement netconvert built from the export's connections is found in it —
/// and with it the internal lane netconvert generated in place of the IR's connector.
/// There is nothing left over: the configuration keeps netconvert from adding the
/// turnarounds the IR never stated, so every movement is one of the export's.
#[test]
fn every_movement_netconvert_built_is_in_the_trace() {
    if !sumo_build::sumo_available() {
        return;
    }
    let map = scenarios::crossroads();
    let (_directory, network) = sumo_build::build(&map);
    let trace = roadgen_sumo::to_plain_xml(&map)
        .expect("the map should export as SUMO")
        .trace;
    let traced_lanes: std::collections::HashSet<&str> = trace
        .links
        .iter()
        .filter_map(|link| link.local.strip_prefix("lane:"))
        .collect();

    let mut matched = 0;
    for connection in &network.connections {
        if connection.from.starts_with(':') || connection.via.is_none() {
            continue;
        }
        let from = format!("{}_{}", connection.from, connection.from_lane);
        let to = format!("{}_{}", connection.to, connection.to_lane);
        if !traced_lanes.contains(from.as_str()) || !traced_lanes.contains(to.as_str()) {
            continue;
        }
        let local = format!("connection:{from}>{to}");
        let links: Vec<_> = trace.links_to(&local).collect();
        assert_ne!(
            connection.direction.as_deref(),
            Some("t"),
            "{local} is a turnaround, which the IR has none of"
        );
        assert!(!links.is_empty(), "{local} is not in the trace");
        assert!(
            links
                .iter()
                .any(|link| link.relation == roadgen_core::Relation::Collapsed),
            "{local} crosses the junction, so a connector lane should be collapsed into it"
        );
        matched += 1;
    }
    // Every pair of the four arms, both ways.
    assert_eq!(matched, 12);
}

/// A crossroads with arms 2 km long, tied to the globe at `origin` through
/// `projection`.
fn located(origin: GeoOrigin, projection: Projection) -> ValidatedMap {
    let mut builder = scenarios::crossroads_builder("located", 2_000.0);
    builder.metadata_mut().origin = origin;
    builder.metadata_mut().projection = projection;
    builder.finish().unwrap().validate().unwrap()
}

/// The built network carries the map's geo-reference, and it is the right one: SUMO,
/// undoing the network's `<location>` itself, puts every junction at the latitude and
/// longitude the IR puts it at — the ones the Lanelet2 export writes — while the
/// network's coordinates stay the IR's own metres.
///
/// Through each of the map's projections, and on both sides of the equator, because
/// a UTM network south of it is the one with a false northing to get right.
#[test]
fn the_built_network_is_where_the_map_is_on_the_globe() {
    if !sumo_build::sumo_available() {
        return;
    }
    let tokyo = GeoOrigin::new(35.68, 139.76, 0.0).unwrap();
    let sydney = GeoOrigin::new(-33.87, 151.21, 0.0).unwrap();
    for (origin, projection) in [
        (tokyo, Projection::LocalCartesian),
        (sydney, Projection::LocalCartesian),
        (tokyo, Projection::Utm),
        (sydney, Projection::Utm),
        (tokyo, Projection::Mgrs),
    ] {
        let case = format!("{} at {origin:?}", projection.as_str());
        let map = located(origin, projection);
        assert_on_the_globe(&map, &case, 5);
    }
}

/// The same, from an origin 2000 m up — where a transverse Mercator at unit scale
/// would put the far end of a 2 km arm 0.63 m from where the Lanelet2 export's
/// east/north/up frame does — and with the far ends of three of the arms above or
/// below the origin's plane, which a 2D `<location>` cannot say.
#[test]
fn a_high_origin_and_raised_junctions_are_where_the_map_is_on_the_globe() {
    if !sumo_build::sumo_available() {
        return;
    }
    let reach = 2_000.0;
    for projection in [
        Projection::LocalCartesian,
        Projection::Mgrs,
        Projection::Utm,
    ] {
        let mut builder = MapBuilder::new(scenarios::metadata("located"));
        builder.metadata_mut().origin = GeoOrigin::new(35.68, 139.76, 2_000.0).unwrap();
        builder.metadata_mut().projection = projection;
        // Each arm climbs or falls from its far end to the level centre. The west one
        // stays on the plane, so there the scale alone is under test.
        let arms = [
            (
                "north",
                Point3::new(0.0, reach, 60.0),
                Point3::new(0.0, 14.0, 0.0),
            ),
            (
                "east",
                Point3::new(reach, 0.0, -40.0),
                Point3::new(14.0, 0.0, 0.0),
            ),
            (
                "south",
                Point3::new(0.0, -reach, 100.0),
                Point3::new(0.0, -14.0, 0.0),
            ),
            (
                "west",
                Point3::new(-reach, 0.0, 0.0),
                Point3::new(-14.0, 0.0, 0.0),
            ),
        ];
        let roads: Vec<_> = arms
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
        let junction = builder.add_junction(Some("x"));
        for (index, from) in roads.iter().enumerate() {
            for to in roads.iter().skip(index + 1) {
                builder
                    .connect_ends(from, RoadEnd::End, to, RoadEnd::End, Some(&junction))
                    .unwrap();
            }
        }
        let map = builder.finish().unwrap().validate().unwrap();
        let case = format!("{} 2000 m up with raised arms", projection.as_str());
        let raised = assert_on_the_globe(&map, &case, 5);
        // The raised ends are where SUMO lands off the IR by more than its own
        // centimetre, so the bound is what the test leant on.
        if projection != Projection::Utm {
            assert!(raised > 0.01, "{case}: worst height error only {raised}");
        }
    }
}

/// Builds `map` with netconvert and checks its `<location>` against the map: what the
/// network states, and where SUMO, undoing it, puts each of the `junctions`
/// non-internal junctions. Returns the largest error from height allowed for.
fn assert_on_the_globe(map: &ValidatedMap, case: &str, junctions: usize) -> f64 {
    let projection = map.metadata.projection;
    let (directory, network) = sumo_build::build(map);
    let location = &network.location;

    // What the export asked for is what the network says.
    let reference = roadgen_sumo::geo_reference(map).unwrap();
    assert_eq!(location.proj_parameter, reference.proj_parameter, "{case}");
    // netconvert writes the offset to the centimetre, as it does every length.
    assert!(
        (location.net_offset.0 - reference.net_offset.0).abs() <= 0.005
            && (location.net_offset.1 - reference.net_offset.1).abs() <= 0.005,
        "{case}: the network's offset is {:?}, not {:?}",
        location.net_offset,
        reference.net_offset
    );
    if projection != Projection::Utm {
        assert_eq!(location.net_offset, (0.0, 0.0), "{case}");
        // A local map at sea level is in the frame the OpenDRIVE export describes,
        // in the same words. Above it SUMO's transverse Mercator is scaled to the
        // origin's plane and OpenDRIVE's is not, so the two differ by the `+k`.
        let opendrive = roadgen_opendrive::origin_proj_string(map);
        if map.metadata.origin.altitude() == 0.0 {
            assert_eq!(location.proj_parameter, opendrive, "{case}");
        } else {
            assert_ne!(location.proj_parameter, opendrive, "{case}");
            assert!(opendrive.contains("+k=1 "), "{opendrive}");
        }
    }

    // The coordinates are still the IR's: the junction is at the origin.
    let centre = network.junction("j_x").position;
    assert!(centre.x.abs() < 0.02 && centre.y.abs() < 0.02, "{case}");

    // And SUMO's own reading of every junction's position is the IR's.
    let projector = roadgen_lanelet2::projector_for(map).unwrap();
    let on_the_globe =
        sumo_build::junctions_on_the_globe(&directory.path().join("located.net.xml"));
    let [west, south, east, north] = location.orig_boundary;
    let mut checked = 0;
    let mut worst_height = 0.0f64;
    for junction in network.junctions.iter().filter(|j| j.kind != "internal") {
        let (lon, lat) = on_the_globe[&junction.id];
        let p = junction.position;
        let wanted = projector.reverse([p.x, p.y, p.z]).unwrap();
        // Degrees to metres, near enough for a tolerance.
        let metres_north = (lat - wanted.lat) * 111_320.0;
        let metres_east = (lon - wanted.lon) * 111_320.0 * wanted.lat.to_radians().cos();
        let error = metres_north.hypot(metres_east);
        // The centimetre is netconvert's rounding of the offset and of the
        // positions. Then 2 mm for a scaled transverse Mercator standing in for the
        // east/north/up plane 2 km out from an origin 2000 m up (1.5 mm measured:
        // one scale cannot match both radii of curvature), and the height a 2D
        // `<location>` cannot carry, which the export states as a bound.
        let height = roadgen_sumo::height_error(map, &p);
        worst_height = worst_height.max(height);
        let tolerance = 0.01 + 0.002 + height;
        assert!(
            error < tolerance,
            "{case}: SUMO puts {} at ({lat:.9}, {lon:.9}), {error:.4} m from the \
             IR's ({:.9}, {:.9}), more than {tolerance:.4} m",
            junction.id,
            wanted.lat,
            wanted.lon
        );
        // And the boundary the network states holds it — to the millionth of a
        // degree netconvert rounds a boundary to, since the far end of an arm is
        // where the boundary is.
        let slack = 1e-6;
        assert!(
            (west - slack..=east + slack).contains(&lon)
                && (south - slack..=north + slack).contains(&lat),
            "{case}: {} at ({lat}, {lon}) is outside the origBoundary {:?}",
            junction.id,
            location.orig_boundary
        );
        checked += 1;
    }
    // The centre and the far end of each arm.
    assert_eq!(checked, junctions, "{case}");
    worst_height
}

/// A movement across a junction follows the path the IR drew for it. The connector
/// lane is not an edge, but its centreline is written as the connection's shape, and
/// netconvert lays the internal lane along that — splitting it in two where a turn
/// must wait inside the junction — instead of inventing a curve of its own that the
/// OpenDRIVE and the Lanelet2 map written from the same IR would not share.
#[test]
fn a_movement_across_a_junction_follows_the_connector_the_ir_drew() {
    if !sumo_build::sumo_available() {
        return;
    }
    let map = scenarios::crossroads();
    let sampling = map.metadata.sampling;
    let written = roadgen_sumo::to_plain_xml(&map).unwrap().lanes;
    let (_directory, network) = sumo_build::build(&map);
    let internal_shape = |id: &str| -> Vec<Point3> {
        let (edge, index) = id.rsplit_once('_').expect("an edge and an index");
        network
            .edges
            .iter()
            .find(|candidate| candidate.id == edge)
            .unwrap_or_else(|| panic!("no internal edge {edge}"))
            .lane(index.parse().unwrap())
            .shape
            .clone()
    };

    let mut turns = 0;
    for connector in map.lanes.iter() {
        let Some(road) = map.road(&connector.road) else {
            continue;
        };
        if road.junction.is_none() {
            continue;
        }
        // The lanes the connector joins, as the network names them.
        let into = map
            .connections
            .iter()
            .find(|connection| connection.to.lane == connector.id)
            .and_then(|connection| written.get(&connection.from.lane))
            .expect("a connector is entered from a written lane");
        let out = map
            .connections
            .iter()
            .find(|connection| connection.from.lane == connector.id)
            .and_then(|connection| written.get(&connection.to.lane))
            .expect("a connector leads to a written lane");
        let (from, from_lane) = into.rsplit_once('_').unwrap();
        let (to, to_lane) = out.rsplit_once('_').unwrap();
        let entry = network
            .connections
            .iter()
            .find(|connection| {
                connection.from == from
                    && connection.to == to
                    && connection.from_lane.to_string() == from_lane
                    && connection.to_lane.to_string() == to_lane
            })
            .unwrap_or_else(|| panic!("no movement {into} > {out}"));
        if entry.direction.as_deref() == Some("s") {
            continue;
        }
        turns += 1;

        // The internal lanes of the movement, in order: the first is the entry's
        // `via`, and each after it is the `via` of the connection leaving the last.
        let mut built: Vec<Point3> = Vec::new();
        let mut lane = entry.via.clone().expect("a turn crosses the junction");
        loop {
            for point in internal_shape(&lane) {
                if built
                    .last()
                    .is_none_or(|last| last.distance_to(point) > 0.02)
                {
                    built.push(point);
                }
            }
            let next = network
                .connections
                .iter()
                .find(|connection| format!("{}_{}", connection.from, connection.from_lane) == lane);
            match next.and_then(|connection| connection.via.clone()) {
                Some(via) => lane = via,
                None => break,
            }
        }

        let drawn = connector
            .travel_geometry(sampling)
            .unwrap()
            .centerline
            .to_polyline(sampling)
            .unwrap();
        // Every point netconvert put the internal lanes through lies on the
        // connector's centreline, to the centimetre it writes its output to.
        for point in &built {
            let off = distance_to_polyline(*point, drawn.points());
            assert!(
                off < 0.05,
                "{into} > {out}: the internal lane passes {off:.3} m off the connector at {point:?}"
            );
        }
        // And they run the whole of it, end to end.
        let length = |points: &[Point3]| -> f64 {
            points
                .windows(2)
                .map(|pair| pair[0].distance_to(pair[1]))
                .sum()
        };
        let (built_length, drawn_length) = (length(&built), length(drawn.points()));
        assert!(
            (built_length - drawn_length).abs() < 0.05,
            "{into} > {out}: the internal lanes are {built_length:.3} m long, the connector \
             {drawn_length:.3} m"
        );
    }
    // A left and a right turn from each of the four arms.
    assert_eq!(turns, 8);
}

/// How far `point` is from the nearest segment of `line`.
fn distance_to_polyline(point: Point3, line: &[Point3]) -> f64 {
    line.windows(2)
        .map(|pair| {
            let (a, b) = (pair[0], pair[1]);
            let (dx, dy, dz) = (b.x - a.x, b.y - a.y, b.z - a.z);
            let squared = dx * dx + dy * dy + dz * dz;
            let t = if squared == 0.0 {
                0.0
            } else {
                (((point.x - a.x) * dx + (point.y - a.y) * dy + (point.z - a.z) * dz) / squared)
                    .clamp(0.0, 1.0)
            };
            point.distance_to(Point3::new(a.x + t * dx, a.y + t * dy, a.z + t * dz))
        })
        .fold(f64::INFINITY, f64::min)
}
