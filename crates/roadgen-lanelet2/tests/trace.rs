//! A trace is only worth anything if it names what is really in the file. So the
//! file is read back with `simple_lanelet2`'s loader, which knows nothing about the
//! trace, and every element the trace names has to be there, as the kind of
//! primitive it says.

use std::collections::HashMap;

use ll2_core::map::LaneletMap;
use roadgen_core::prelude::*;
use roadgen_core::trace::{IrRef, Relation, Trace, TraceLink};
use roadgen_core::validation::ValidatedMap;

fn metadata() -> MapMetadata {
    MapMetadata {
        name: Some("trace".into()),
        origin: GeoOrigin::new(35.68, 139.76, 0.0).unwrap(),
        ..MapMetadata::default()
    }
}

fn width() -> PositiveWidth {
    PositiveWidth::new(3.5).unwrap()
}

/// One road with everything the exporter traces: lanes sharing a boundary, a
/// shoulder that becomes no lanelet, a crosswalk, a signal with its stop line, and
/// one rule of each kind — plus a signal rule with no signal, which writes nothing.
fn street() -> ValidatedMap {
    let mut builder = MapBuilder::new(metadata());
    let road = builder
        .add_road(
            RoadSpec::line(
                Point3::ORIGIN,
                Point3::new(120.0, 0.0, 0.0),
                vec![
                    LaneSpec::new(width(), Direction::Forward),
                    LaneSpec::new(width(), Direction::Forward),
                    LaneSpec::new(width(), Direction::Backward),
                    LaneSpec::new(width(), Direction::Forward).with_type(LaneType::Shoulder),
                ],
            )
            .unwrap()
            .with_name("main"),
        )
        .unwrap();
    let lane = |index| LaneRef::new(road.clone(), index);
    builder.add_crosswalk(&road, 0.5, 4.0).unwrap();
    let light = builder
        .add_traffic_light(&lane(0), LaneEnd::End, 5.0)
        .unwrap();
    let stop = builder.add_stop_line(&lane(0), LaneEnd::End).unwrap();
    builder.add_traffic_light_rule(vec![light], Some(stop), vec![lane(0)]);
    builder.add_traffic_light_rule(Vec::new(), None, vec![lane(1)]);
    builder.add_speed_limit_rule(
        SpeedLimit::from_kph(40.0).unwrap(),
        vec![lane(0), lane(1), lane(3)],
    );
    builder.add_right_of_way(vec![lane(0)], vec![lane(2)], None);
    builder.finish().unwrap().validate().unwrap()
}

fn load(xml: &str, map: &ValidatedMap) -> std::sync::Arc<LaneletMap> {
    let projector = roadgen_lanelet2::projector_for(map).unwrap();
    ll2_io::load_str(xml, projector.as_ref()).expect("a Lanelet2 loader should accept this")
}

fn lane_links<'a>(trace: &'a Trace, lane: &LaneId) -> Vec<&'a TraceLink> {
    trace.links_of(&IrRef::Lane(lane.clone())).collect()
}

#[test]
fn every_traced_element_is_in_the_written_file() {
    let map = street();
    let (xml, trace) = roadgen_lanelet2::to_osm_xml_traced(&map).unwrap();
    let loaded = load(&xml, &map);
    assert_eq!(trace.format, "lanelet2");
    assert!(trace.files.is_empty());
    assert!(!trace.links.is_empty());

    for link in &trace.links {
        let (kind, local) = link.local.split_once(':').expect(&link.local);
        let id: i64 = local.parse().expect(&link.local);
        let layer = match kind {
            "lanelet" => &loaded.lanelets,
            "linestring" => &loaded.line_strings,
            "regulatory_element" => &loaded.regulatory_elements,
            other => panic!("{other} is not a kind the Lanelet2 trace writes"),
        };
        assert!(layer.exists(id), "{} is not in the file", link.local);
    }
}

#[test]
fn every_lane_with_a_lanelet_has_exactly_one() {
    let map = street();
    let (_, trace) = roadgen_lanelet2::to_lanelet_map_traced(&map).unwrap();
    for lane in map.lanes.iter() {
        let lanelets = lane_links(&trace, &lane.id)
            .into_iter()
            .filter(|link| link.role.as_deref() == Some("lanelet"))
            .collect::<Vec<_>>();
        if lane.lane_type == LaneType::Shoulder {
            assert!(lane_links(&trace, &lane.id).is_empty(), "{}", lane.id);
            continue;
        }
        assert_eq!(lanelets.len(), 1, "{}", lane.id);
        assert_eq!(lanelets[0].relation, Relation::Exact);
        assert!(lanelets[0].local.starts_with("lanelet:"));

        let roles: Vec<(&str, Relation)> = lane_links(&trace, &lane.id)
            .iter()
            .map(|link| (link.role.as_deref().unwrap(), link.relation))
            .collect();
        assert!(
            roles.contains(&("centerline", Relation::Exact)),
            "{}",
            lane.id
        );
        assert!(roles.iter().any(|(role, _)| *role == "left_boundary"));
        assert!(roles.iter().any(|(role, _)| *role == "right_boundary"));
    }
}

#[test]
fn a_boundary_between_two_lanelets_is_merged_and_one_at_the_edge_is_exact() {
    let map = street();
    let (_, trace) = roadgen_lanelet2::to_lanelet_map_traced(&map).unwrap();
    let mut users: HashMap<&str, Vec<&IrRef>> = HashMap::new();
    for link in &trace.links {
        if let Some("left_boundary" | "right_boundary") = link.role.as_deref() {
            users.entry(&link.local).or_default().push(&link.ir);
        }
    }
    for link in &trace.links {
        if let Some("left_boundary" | "right_boundary") = link.role.as_deref() {
            let shared = users[link.local.as_str()].len() > 1;
            let expected = if shared {
                Relation::Merged
            } else {
                Relation::Exact
            };
            assert_eq!(link.relation, expected, "{}", link.local);
        }
    }
    // Three lanelets side by side: two boundaries between them, two at the edges.
    let merged = users.values().filter(|lanes| lanes.len() == 2).count();
    assert_eq!(merged, 2);
    assert_eq!(users.len(), 4);
}

#[test]
fn objects_and_rules_are_traced_to_what_they_became() {
    let map = street();
    let (_, trace) = roadgen_lanelet2::to_lanelet_map_traced(&map).unwrap();

    let crosswalk = map
        .objects
        .iter()
        .find(|object| object.kind == MapObjectKind::Crosswalk)
        .unwrap();
    let links: Vec<_> = trace
        .links_of(&IrRef::Object(crosswalk.id.clone()))
        .collect();
    let lines = links
        .iter()
        .filter(|link| link.local.starts_with("linestring:"))
        .collect::<Vec<_>>();
    assert_eq!(lines.len(), 2);
    assert!(lines.iter().all(|link| link.relation == Relation::Part));
    assert_eq!(
        links
            .iter()
            .filter(|link| link.role.as_deref() == Some("crosswalk")
                && link.relation == Relation::Exact
                && link.local.starts_with("lanelet:"))
            .count(),
        1
    );

    // A signal is a single line, so it is its linestring exactly; a builder's
    // light has a standard head, whose lamps are the whole light again.
    let light = map
        .objects
        .iter()
        .find(|object| object.kind.is_traffic_light())
        .unwrap();
    let links: Vec<_> = trace.links_of(&IrRef::Object(light.id.clone())).collect();
    assert_eq!(links.len(), 2);
    assert!(links.iter().all(|link| link.relation == Relation::Exact));
    assert_eq!(
        links
            .iter()
            .filter(|link| link.role.as_deref() == Some("light_bulbs"))
            .count(),
        1
    );

    let rule = |index| trace.links_of(&IrRef::Rule(index)).collect::<Vec<_>>();
    for index in [0, 3] {
        let links = rule(index);
        assert_eq!(links.len(), 1, "rule {index}");
        assert!(links[0].local.starts_with("regulatory_element:"));
        assert_eq!(links[0].relation, Relation::Exact);
    }
    // A signal rule with no signal writes nothing, so it links to nothing.
    assert!(rule(1).is_empty());

    // The limit is a tag on the two driving lanelets it covers; the shoulder it also
    // names has no lanelet to carry it.
    let limit = rule(2);
    assert_eq!(limit.len(), 2);
    for link in limit {
        assert_eq!(link.relation, Relation::Part);
        assert_eq!(link.role.as_deref(), Some("speed_limit"));
        assert!(trace
            .links_to(&link.local)
            .any(|lane| lane.role.as_deref() == Some("lanelet")));
    }
}

#[test]
fn the_same_map_traces_identically_twice() {
    let (_, first) = roadgen_lanelet2::to_lanelet_map_traced(&street()).unwrap();
    let (_, second) = roadgen_lanelet2::to_lanelet_map_traced(&street()).unwrap();
    assert_eq!(first, second);
}

#[test]
fn writing_records_the_file_and_the_plain_entry_points_agree() {
    let map = street();
    let dir = std::env::temp_dir().join(format!("roadgen-lanelet2-trace-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let path = dir.join("lanelet2_map.osm");

    let trace = roadgen_lanelet2::write_traced(&map, &path).unwrap();
    assert_eq!(trace.files, vec![path.clone()]);
    let written = std::fs::read_to_string(&path).unwrap();
    assert_eq!(written, roadgen_lanelet2::to_osm_xml(&map).unwrap());

    let (xml, in_memory) = roadgen_lanelet2::to_osm_xml_traced(&map).unwrap();
    assert_eq!(xml, written);
    assert_eq!(in_memory.links, trace.links);
    std::fs::remove_dir_all(&dir).unwrap();
}
