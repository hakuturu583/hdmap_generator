//! The trace says where each element of the IR went, so it is checked against the
//! file rather than against the exporter: every element it names is read back out
//! of the written document, and every road and lane of the IR is named exactly once.

use std::collections::BTreeSet;

use opendrive::core::OpenDrive;
use roadgen_core::prelude::*;
use roadgen_core::semantics::LaneType;
use roadgen_core::trace::{IrRef, Relation, Trace};
use roadgen_opendrive::Options;

/// A signalised T with pavements, a crosswalk and a town beside it: a junction, its
/// controllers, every kind of signal and object, and buildings of several parts.
fn signalised_town() -> ValidatedMap {
    let mut builder = MapBuilder::new(MapMetadata {
        name: Some("traced".to_owned()),
        ..MapMetadata::default()
    });
    let width = |metres: f64| PositiveWidth::new(metres).expect("a positive width");
    let lanes = || {
        vec![
            LaneSpec::new(width(3.5), Direction::Backward),
            LaneSpec::new(width(2.0), Direction::Backward).with_type(LaneType::Sidewalk),
            LaneSpec::new(width(3.5), Direction::Forward),
            LaneSpec::new(width(2.0), Direction::Forward).with_type(LaneType::Sidewalk),
        ]
    };
    let north = builder
        .add_road(
            RoadSpec::line(
                Point3::new(0.0, 120.0, 0.0),
                Point3::new(0.0, 14.0, 0.0),
                lanes(),
            )
            .unwrap()
            .with_name("north"),
        )
        .unwrap();
    let east = builder
        .add_road(
            RoadSpec::line(
                Point3::new(120.0, 0.0, 0.0),
                Point3::new(14.0, 0.0, 0.0),
                lanes(),
            )
            .unwrap()
            .with_name("east"),
        )
        .unwrap();
    let junction = builder.add_junction(Some("x"));
    builder
        .connect_ends(&north, RoadEnd::End, &east, RoadEnd::End, Some(&junction))
        .unwrap();
    let north_in = LaneRef::new(north.clone(), 2);
    let east_in = LaneRef::new(east.clone(), 2);
    let stop = builder
        .add_stop_line_at(&north_in, LaneEnd::End, 8.0)
        .unwrap();
    let north_light = builder
        .add_traffic_light(&north_in, LaneEnd::End, 5.5)
        .unwrap();
    let east_light = builder
        .add_traffic_light(&east_in, LaneEnd::End, 5.5)
        .unwrap();
    builder.add_traffic_light_rule(vec![north_light], Some(stop), vec![north_in.clone()]);
    builder.add_traffic_light_rule(vec![east_light], None, vec![east_in]);
    builder
        .add_traffic_sign(&north_in, LaneEnd::End, "stop", 2.4)
        .unwrap();
    builder.add_crosswalk(&east, 0.5, 4.0).unwrap();
    let mut map = builder.finish().unwrap();
    roadgen_buildings::generate(&mut map, &roadgen_buildings::Rules::default())
        .expect("a town should generate");
    map.validate().unwrap()
}

/// Every element the document holds, named the way the trace names them.
fn written_elements(drive: &OpenDrive) -> BTreeSet<String> {
    let mut found = BTreeSet::new();
    for road in &drive.road {
        found.insert(format!("road:{}", road.id));
        for (index, section) in road.lanes.lane_section.iter().enumerate() {
            let left = section.left.iter().flat_map(|left| left.lane.iter());
            let right = section.right.iter().flat_map(|right| right.lane.iter());
            for id in left.map(|lane| lane.id).chain(right.map(|lane| lane.id)) {
                found.insert(format!("lane:{}/{index}/{id}", road.id));
            }
        }
        for signal in road.signals.iter().flat_map(|signals| &signals.signal) {
            found.insert(format!("signal:{}", signal.id));
        }
        for object in road.objects.iter().flat_map(|objects| &objects.object) {
            found.insert(format!("object:{}", object.id));
            for outline in object
                .outlines
                .iter()
                .flat_map(|outlines| &outlines.outline)
            {
                found.insert(format!(
                    "outline:{}/{}",
                    object.id,
                    outline.id.expect("a building outline carries its id")
                ));
            }
        }
    }
    for junction in &drive.junction {
        found.insert(format!("junction:{}", junction.id));
        for connection in junction.connection.iter() {
            found.insert(format!("connection:{}/{}", junction.id, connection.id));
        }
    }
    for controller in &drive.controller {
        found.insert(format!("controller:{}", controller.id));
    }
    found
}

fn exact_links(trace: &Trace, ir: &IrRef) -> usize {
    trace
        .links_of(ir)
        .filter(|link| link.relation == Relation::Exact)
        .count()
}

#[test]
fn every_traced_element_is_in_the_written_file() {
    let map = signalised_town();
    let directory = tempfile::tempdir().unwrap();
    let path = directory.path().join("traced.xodr");
    let trace = roadgen_opendrive::write_traced(&map, &path).unwrap();
    assert_eq!(trace.format, "opendrive");
    assert_eq!(trace.files, vec![path.clone()]);

    let drive = OpenDrive::from_xml_str(&std::fs::read_to_string(&path).unwrap()).unwrap();
    let written = written_elements(&drive);
    for link in &trace.links {
        assert!(
            written.contains(&link.local),
            "{} is traced to {}, which the file does not hold",
            link.ir,
            link.local
        );
    }
    // And the other way: nothing the scenario asks for goes untraced.
    let traced: BTreeSet<&str> = trace.links.iter().map(|link| link.local.as_str()).collect();
    for element in &written {
        assert!(traced.contains(element.as_str()), "{element} is not traced");
    }
    for kind in [
        "road:",
        "lane:",
        "junction:",
        "connection:",
        "signal:",
        "object:",
        "outline:",
        "controller:",
    ] {
        assert!(
            traced.iter().any(|local| local.starts_with(kind)),
            "the scenario writes no {kind}"
        );
    }
}

#[test]
fn every_road_and_lane_is_traced_exactly_once() {
    let map = signalised_town();
    let trace = roadgen_opendrive::trace(&map, &Options::default()).unwrap();
    assert!(trace.files.is_empty());
    for road in map.roads.iter() {
        assert_eq!(
            exact_links(&trace, &IrRef::Road(road.id.clone())),
            1,
            "{}",
            road.id
        );
    }
    for lane in map.lanes.iter() {
        let ir = IrRef::Lane(lane.id.clone());
        assert_eq!(exact_links(&trace, &ir), 1, "{}", lane.id);
        // The same numbers the public lookup gives, since it is the same numbering.
        let (road, number) = roadgen_opendrive::lane_id(&map, &lane.id).unwrap();
        let local = &trace.links_of(&ir).next().unwrap().local;
        assert!(local.starts_with(&format!("lane:{road}/")), "{local}");
        assert!(local.ends_with(&format!("/{number}")), "{local}");
    }
    // Every movement into a connector is one of the lane links of a `<connection>`.
    for connection in map.connections.iter() {
        let Some(junction) = &connection.junction else {
            continue;
        };
        let from_connector = map
            .road(&map.lanes.get(&connection.from.lane).unwrap().road)
            .unwrap()
            .is_connector();
        if from_connector {
            continue;
        }
        let ir = IrRef::Connection(connection.id.clone());
        let links: Vec<_> = trace
            .links_of(&ir)
            .filter(|link| link.relation == Relation::Merged)
            .collect();
        assert_eq!(links.len(), 1, "{}", connection.id);
        let id = roadgen_opendrive::junction_id(&map, junction).unwrap();
        assert!(links[0].local.starts_with(&format!("connection:{id}/")));
    }
    // Every connection is written somewhere — as a `<connection>`'s lane link, or as
    // the successor of one lane and the predecessor of the other — and a lane it is
    // collapsed into is one of its two ends.
    for connection in map.connections.iter() {
        let ir = IrRef::Connection(connection.id.clone());
        let links: Vec<_> = trace.links_of(&ir).collect();
        assert!(!links.is_empty(), "{} is not traced", connection.id);
        let ends: Vec<String> = [&connection.from.lane, &connection.to.lane]
            .into_iter()
            .map(|lane| {
                trace
                    .links_of(&IrRef::Lane(lane.clone()))
                    .next()
                    .unwrap()
                    .local
                    .clone()
            })
            .collect();
        for link in links
            .iter()
            .filter(|link| link.relation == Relation::Collapsed)
        {
            assert!(ends.contains(&link.local), "{link:?} is not an end of it");
            assert!(matches!(
                link.role.as_deref(),
                Some("successor" | "predecessor")
            ));
        }
    }
    // Every light is switched by exactly one controller.
    for object in map.objects.iter() {
        let lights = trace
            .links_of(&IrRef::Object(object.id.clone()))
            .filter(|link| link.role.as_deref() == Some("controller"))
            .count();
        let expected = usize::from(object.kind == MapObjectKind::TrafficLight);
        assert_eq!(lights, expected, "{}", object.id);
    }
}

#[test]
fn the_same_map_traces_the_same_twice() {
    let map = signalised_town();
    let one = roadgen_opendrive::trace(&map, &Options::default()).unwrap();
    let two = roadgen_opendrive::trace(&map, &Options::default()).unwrap();
    assert!(!one.links.is_empty());
    assert_eq!(one, two);
}

#[test]
fn a_connection_between_two_roads_is_collapsed_into_the_lanes_it_links() {
    let mut builder = MapBuilder::default();
    let lanes = || {
        vec![
            LaneSpec::new(PositiveWidth::new(3.5).unwrap(), Direction::Forward),
            LaneSpec::new(PositiveWidth::new(3.5).unwrap(), Direction::Backward),
        ]
    };
    let a = builder
        .add_road(
            RoadSpec::line(
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(100.0, 0.0, 0.0),
                lanes(),
            )
            .unwrap()
            .with_name("a"),
        )
        .unwrap();
    let b = builder
        .add_road(
            RoadSpec::line(
                Point3::new(100.0, 0.0, 0.0),
                Point3::new(200.0, 0.0, 0.0),
                lanes(),
            )
            .unwrap()
            .with_name("b"),
        )
        .unwrap();
    builder.connect(&a, &b).unwrap();
    let map = builder.finish().unwrap().validate().unwrap();
    let trace = roadgen_opendrive::trace(&map, &Options::default()).unwrap();

    assert!(!map.connections.is_empty());
    for connection in map.connections.iter() {
        let ir = IrRef::Connection(connection.id.clone());
        let links: Vec<_> = trace.links_of(&ir).collect();
        // The successor of one lane and the predecessor of the other.
        assert_eq!(links.len(), 2, "{}: {links:?}", connection.id);
        assert!(links
            .iter()
            .all(|link| link.relation == Relation::Collapsed && link.local.starts_with("lane:")));
        let roles: BTreeSet<_> = links
            .iter()
            .filter_map(|link| link.role.as_deref())
            .collect();
        assert_eq!(roles, BTreeSet::from(["predecessor", "successor"]));
    }
}
