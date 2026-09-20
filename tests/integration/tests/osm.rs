//! Exporting to plain OpenStreetMap.
//!
//! OSM is the furthest of the four formats from the IR: a road is one line with a
//! lane *count* on it, junctions are shared nodes rather than enumerated movements,
//! and what is forbidden is said instead of what is allowed. So these tests are
//! mostly about the translation being the right way round — and they read the file
//! back through an OSM parser rather than trusting the exporter's own structures.

use roadgen_core::prelude::*;
use roadgen_integration_tests::{reload_osm, scenarios};

/// A three-arm junction that permits everything except turning between two of them.
fn tee_with_a_missing_turn() -> ValidatedMap {
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
    // North joins both of the others, but east and west do not join each other: the
    // straight-through movement across the top of the tee is missing.
    for other in &arms[1..] {
        builder
            .connect_ends(&arms[0], RoadEnd::End, other, RoadEnd::End, Some(&junction))
            .unwrap();
    }
    builder.finish().unwrap().validate().unwrap()
}

#[test]
fn a_road_becomes_one_way_down_its_own_centreline() {
    let map = scenarios::straight_road();
    let osm = reload_osm(&map);

    let ways = osm.ways_tagged("highway", "residential");
    assert_eq!(ways.len(), 1, "one way per road, not one per lane");
    let way = ways[0];

    // The node order follows the reference line, which is what makes `oneway` mean
    // anything: `yes` is "with the node order".
    let road = map.roads.iter().next().unwrap();
    let line = road
        .reference_line
        .to_polyline(map.metadata.sampling)
        .unwrap();
    assert_eq!(way.nodes.len(), line.len());
    for (id, expected) in way.nodes.iter().zip(line.points()) {
        assert!(
            osm.at(*id).distance_to(*expected) < 1e-3,
            "{:?} should be {expected:?}",
            osm.at(*id)
        );
    }
}

#[test]
fn oneway_says_which_directions_the_lanes_run() {
    let mut builder = MapBuilder::new(scenarios::metadata("directions"));
    builder
        .add_road(
            RoadSpec::line(
                Point3::ORIGIN,
                Point3::new(100.0, 0.0, 0.0),
                scenarios::two_way(),
            )
            .unwrap()
            .with_name("both"),
        )
        .unwrap();
    builder
        .add_road(
            RoadSpec::line(
                Point3::new(0.0, 50.0, 0.0),
                Point3::new(100.0, 50.0, 0.0),
                scenarios::one_way(2),
            )
            .unwrap()
            .with_name("with"),
        )
        .unwrap();
    builder
        .add_road(
            RoadSpec::line(
                Point3::new(0.0, 100.0, 0.0),
                Point3::new(100.0, 100.0, 0.0),
                vec![scenarios::lane(3.5, Direction::Backward)],
            )
            .unwrap()
            .with_name("against"),
        )
        .unwrap();
    let osm = reload_osm(&builder.finish().unwrap().validate().unwrap());

    let both = osm.way_named("both");
    assert_eq!(both.tags["oneway"], "no");
    assert_eq!(both.tags["lanes"], "2");
    assert_eq!(both.tags["lanes:forward"], "1");
    assert_eq!(both.tags["lanes:backward"], "1");

    let with = osm.way_named("with");
    assert_eq!(with.tags["oneway"], "yes");
    assert_eq!(with.tags["lanes"], "2");
    assert!(
        !with.tags.contains_key("lanes:forward"),
        "a one-way road's lanes all run one way"
    );

    // Every lane against the reference line, which OSM spells as a negative rather
    // than by reversing the way — so the node order can stay the IR's.
    assert_eq!(osm.way_named("against").tags["oneway"], "-1");
}

#[test]
fn roads_that_join_share_the_node_they_meet_at() {
    // Sharing a node *is* connectivity in OSM; there is nothing else to say it with.
    let map = scenarios::two_roads_joined();
    let osm = reload_osm(&map);
    let (a, b) = (osm.way_named("a"), osm.way_named("b"));

    let shared: Vec<i64> = a
        .nodes
        .iter()
        .filter(|id| b.nodes.contains(id))
        .copied()
        .collect();
    assert_eq!(shared.len(), 1, "they meet at one point, so at one node");
    assert_eq!(*a.nodes.last().unwrap(), shared[0]);
    assert_eq!(*b.nodes.first().unwrap(), shared[0]);
}

#[test]
fn a_junction_becomes_one_node_that_every_arm_reaches() {
    let map = scenarios::crossroads();
    let osm = reload_osm(&map);

    let arms: Vec<_> = ["north", "east", "south", "west"]
        .map(|name| osm.way_named(name))
        .into_iter()
        .collect();
    let shared: Vec<i64> = arms[0]
        .nodes
        .iter()
        .filter(|id| arms[1..].iter().all(|way| way.nodes.contains(id)))
        .copied()
        .collect();
    assert_eq!(shared.len(), 1, "all four arms meet at one node");

    // At the centre of the crossroads — not at the average of the arm ends, which
    // for arms that stop 14 m short would be somewhere else entirely.
    let centre = osm.at(shared[0]);
    assert!(
        centre.x.abs() < 1e-6 && centre.y.abs() < 1e-6,
        "the junction node should be at the crossing: {centre:?}"
    );

    // And the connectors that carried the movements in the IR are not ways: OSM has
    // no such thing, and drawing twelve of them across the junction would be twelve
    // roads that do not exist.
    assert_eq!(osm.ways_tagged("highway", "residential").len(), 4);
    assert_eq!(map.roads.len(), 16, "the IR still has its connectors");
}

#[test]
fn a_turn_the_map_does_not_permit_becomes_a_restriction() {
    let map = tee_with_a_missing_turn();
    let osm = reload_osm(&map);

    // OSM assumes every turn is legal, so the file has to say what is not. East and
    // west are the pair with no movement between them, in both directions.
    let restrictions: Vec<_> = osm
        .document
        .relations
        .values()
        .filter(|relation| {
            relation
                .tags
                .get("type")
                .is_some_and(|kind| kind == "restriction")
        })
        .collect();
    assert_eq!(restrictions.len(), 2, "{restrictions:#?}");

    let (east, west) = (osm.way_named("east").id, osm.way_named("west").id);
    for relation in &restrictions {
        // Straight across the top of the tee, and named as such.
        assert_eq!(relation.tags["restriction"], "no_straight_on");
        let member = |role: &str| {
            relation
                .members
                .iter()
                .find(|member| member.role == role)
                .unwrap()
        };
        let (from, to) = (member("from").reference, member("to").reference);
        assert!(
            (from == east && to == west) || (from == west && to == east),
            "from {from} to {to}"
        );
        // The via member is the junction node, which is what makes it a turn at all.
        let via = member("via");
        assert_eq!(via.kind, ll2_io::osm::MemberType::Node);
        assert!(osm.at(via.reference).x.abs() < 1e-6);
    }

    // A junction that permits everything says nothing.
    assert!(reload_osm(&scenarios::crossroads())
        .document
        .relations
        .is_empty());
}

#[test]
fn furniture_lands_on_the_way_where_the_ir_put_it() {
    let map = scenarios::controlled_crossroads();
    let osm = reload_osm(&map);

    let signals = osm.nodes_tagged("highway", "traffic_signals");
    assert_eq!(signals.len(), 1);
    let light = map
        .objects
        .iter()
        .find(|object| object.kind == MapObjectKind::TrafficLight)
        .unwrap();
    let ObjectGeometry::Line(bar) = &light.geometry else {
        panic!("a light is a bar across the lane")
    };
    // The node is on the road's centreline, abeam of the bar over the lane.
    let at = osm.at(signals[0].id);
    let centre = bar.start_point().lerp(bar.end_point(), 0.5);
    assert!((at.y - centre.y).abs() < 1e-3, "{at:?} vs {centre:?}");

    // It is a node *of* the road's way, not a loose one beside it.
    let north = osm.way_named("north");
    assert!(north.nodes.contains(&signals[0].id));

    // The stop line is at the same point, and a signalised stop is signals rather
    // than a stop sign: one `highway` tag, and the stronger control keeps it.
    assert!(osm.nodes_tagged("highway", "stop").is_empty());

    // The crossing is a node on the road and a footway across it — through that
    // node, so the footway and the road share a vertex and a router can step from
    // one onto the other.
    let crossings = osm.nodes_tagged("highway", "crossing");
    assert_eq!(crossings.len(), 1);
    assert!(north.nodes.contains(&crossings[0].id));
    let footways = osm.ways_tagged("footway", "crossing");
    assert_eq!(footways.len(), 1);
    assert!(footways[0].nodes.len() >= 3);
    assert!(footways[0].nodes.contains(&crossings[0].id));

    // And the check says which controls will share a node, and which tag wins.
    let problems = roadgen_osm::check(&map);
    let merged = problems
        .iter()
        .find(|problem| problem.contains("share one node"))
        .expect("the stop line under the light is reported");
    assert!(merged.contains("highway=traffic_signals"), "{merged}");
}

#[test]
fn a_crossing_beside_a_stop_line_keeps_its_own_node() {
    // The ordinary layout at a signalised mouth: the stop line set back a few
    // metres, the crosswalk between it and the junction. The two land 0.4 m apart
    // — within the spacing that folds two controls onto one node — and a crossing
    // is not a control, so it must not be folded away.
    let mut builder = MapBuilder::new(scenarios::metadata("crossing"));
    let road = builder
        .add_road(
            RoadSpec::line(
                Point3::ORIGIN,
                Point3::new(100.0, 0.0, 0.0),
                scenarios::two_way(),
            )
            .unwrap()
            .with_name("main"),
        )
        .unwrap();
    let approach = LaneRef::new(road.clone(), 0);
    builder
        .add_stop_line_at(&approach, LaneEnd::End, 6.0)
        .unwrap();
    builder.add_crosswalk(&road, 0.936, 4.0).unwrap();
    let map = builder.finish().unwrap().validate().unwrap();
    let osm = reload_osm(&map);

    let stops = osm.nodes_tagged("highway", "stop");
    let crossings = osm.nodes_tagged("highway", "crossing");
    assert_eq!(stops.len(), 1);
    assert_eq!(crossings.len(), 1);
    assert_ne!(stops[0].id, crossings[0].id);
    let main = osm.way_named("main");
    assert!(main.nodes.contains(&stops[0].id));
    assert!(main.nodes.contains(&crossings[0].id));
    assert!((osm.at(stops[0].id).x - 94.0).abs() < 1e-3);
    assert!((osm.at(crossings[0].id).x - 93.6).abs() < 1e-3);

    // The footway goes through the road's crossing node.
    let footways = osm.ways_tagged("footway", "crossing");
    assert_eq!(footways.len(), 1);
    assert!(footways[0].nodes.contains(&crossings[0].id));
    // In order: kerb, road, kerb — not tacked on at an end.
    let index = footways[0]
        .nodes
        .iter()
        .position(|node| *node == crossings[0].id)
        .unwrap();
    assert!(index > 0 && index + 1 < footways[0].nodes.len());

    // Nothing was merged, so nothing is reported as merged.
    assert!(!roadgen_osm::check(&map)
        .iter()
        .any(|problem| problem.contains("share one node")));
}

#[test]
fn a_sign_keeps_the_callers_own_code() {
    let mut builder = MapBuilder::new(scenarios::metadata("signed"));
    let road = builder
        .add_road(
            RoadSpec::line(
                Point3::ORIGIN,
                Point3::new(100.0, 0.0, 0.0),
                scenarios::two_way(),
            )
            .unwrap()
            .with_name("main"),
        )
        .unwrap();
    builder
        .add_traffic_sign(&LaneRef::new(road, 0), LaneEnd::End, "de274-60", 2.5)
        .unwrap();
    let osm = reload_osm(&builder.finish().unwrap().validate().unwrap());

    // OSM has no catalogue of its own here either, so the code passes through.
    let signs = osm.nodes_tagged("traffic_sign", "de274-60");
    assert_eq!(signs.len(), 1);
    assert!(osm.way_named("main").nodes.contains(&signs[0].id));
}

#[test]
fn heights_survive_only_as_ele_tags() {
    // An OSM node is a latitude and a longitude. This is the whole of what the format
    // can say about the third dimension, and the test is here so that it keeps
    // saying it.
    let map = scenarios::graded_road();
    let osm = reload_osm(&map);

    let line = map
        .roads
        .iter()
        .next()
        .unwrap()
        .reference_line
        .to_polyline(map.metadata.sampling)
        .unwrap();
    let climb = line.last().z - line.first().z;
    assert!(climb.abs() > 1.0, "the fixture should climb");

    assert!(osm.xml.contains("<tag k=\"ele\""), "heights are ele tags");
    let way = osm.ways_tagged("highway", "residential")[0];
    for (id, expected) in way.nodes.iter().zip(line.points()) {
        assert!((osm.at(*id).z - expected.z).abs() < 1e-6);
    }
}

#[test]
fn the_file_reads_as_data_that_was_never_uploaded() {
    let osm = reload_osm(&scenarios::two_roads_joined());
    assert!(
        osm.xml.contains("generator=\"roadgen\""),
        "{}",
        &osm.xml[..120]
    );
    assert!(
        !osm.xml.contains("lanelet2"),
        "the writer is shared with the Lanelet2 export; the file is not a Lanelet2 map"
    );

    // Negative identifiers, and no `version`: this is new data, and saying otherwise
    // would claim it came from the OSM database.
    assert!(osm.document.nodes.keys().all(|id| *id < 0));
    assert!(osm.document.ways.keys().all(|id| *id < 0));
    assert!(!osm.xml.contains("version=\"1\""));
}

#[test]
fn a_sidewalk_is_tagged_on_the_side_it_is_on() {
    let mut builder = MapBuilder::new(scenarios::metadata("footways"));
    builder
        .add_road(
            RoadSpec::line(
                Point3::ORIGIN,
                Point3::new(100.0, 0.0, 0.0),
                vec![
                    scenarios::lane(3.5, Direction::Forward),
                    scenarios::lane(3.5, Direction::Backward),
                    scenarios::lane(2.0, Direction::Forward).with_type(LaneType::Sidewalk),
                ],
            )
            .unwrap()
            .with_name("kerbed"),
        )
        .unwrap();
    let osm = reload_osm(&builder.finish().unwrap().validate().unwrap());

    let way = osm.way_named("kerbed");
    assert_eq!(way.tags["sidewalk"], "right", "{:?}", way.tags);
    // A footway is not a driving lane, so it is not counted as one.
    assert_eq!(way.tags["lanes"], "2");
}

#[test]
fn the_same_map_exports_identically_twice() {
    let map = scenarios::controlled_crossroads();
    assert_eq!(
        roadgen_osm::to_xml(&map).unwrap(),
        roadgen_osm::to_xml(&map).unwrap()
    );
}

#[test]
fn what_the_format_cannot_carry_is_reported() {
    let problems = roadgen_osm::check(&scenarios::banked_curve());
    assert!(
        problems
            .iter()
            .any(|problem| problem.contains("lane geometry")),
        "{problems:?}"
    );
    assert!(
        problems
            .iter()
            .any(|problem| problem.contains("superelevation")),
        "{problems:?}"
    );
    // A flat, unbanked, single-section map still reports the lane geometry, because
    // that is a property of OSM rather than of the map.
    let plain = roadgen_osm::check(&scenarios::straight_road());
    assert_eq!(plain.len(), 1, "{plain:?}");

    // And a changing cross-section is named, since one way carries one lane count.
    let dropped = roadgen_osm::check(&scenarios::lane_drop());
    assert!(
        dropped
            .iter()
            .any(|problem| problem.contains("cross-section")),
        "{dropped:?}"
    );
}

#[test]
fn a_light_over_a_graded_road_shares_the_stop_lines_node() {
    // A light five metres up a road that falls away is a quarter of a metre up-road
    // of the stop line underneath it, because the IR raises furniture along the road
    // normal rather than straight up. That is right in three dimensions and noise in
    // OSM: one control point, so one node.
    let mut builder = MapBuilder::new(scenarios::metadata("graded-control"));
    let road = builder
        .add_road(
            RoadSpec::line(
                Point3::new(0.0, 0.0, 3.0),
                Point3::new(60.0, 0.0, 0.0),
                scenarios::two_way(),
            )
            .unwrap()
            .with_name("down"),
        )
        .unwrap();
    let lane = LaneRef::new(road, 0);
    builder.add_stop_line(&lane, LaneEnd::End).unwrap();
    builder.add_traffic_light(&lane, LaneEnd::End, 5.0).unwrap();
    let map = builder.finish().unwrap().validate().unwrap();

    let osm = reload_osm(&map);
    let signals = osm.nodes_tagged("highway", "traffic_signals");
    assert_eq!(signals.len(), 1);
    assert!(
        osm.nodes_tagged("highway", "stop").is_empty(),
        "the stronger control keeps the node's one `highway` tag"
    );

    // And the way did not gain a second node a hand's breadth from the first.
    let way = osm.way_named("down");
    assert_eq!(
        way.nodes.len(),
        2,
        "a straight road, with the light on its end"
    );
    assert_eq!(*way.nodes.last().unwrap(), signals[0].id);
}
