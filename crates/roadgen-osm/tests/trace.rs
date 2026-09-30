//! The trace an OSM export keeps of where each IR element went.
//!
//! Read against the file itself rather than the exporter's own tables: a trace that
//! names an element the parser cannot find is worse than none, because it is
//! believed.

use std::collections::HashSet;

use roadgen_core::buildings::{Building, BuildingPart, Footprint, Solid};
use roadgen_core::prelude::*;
use roadgen_core::units::GeoOrigin;
use roadgen_core::{IrRef, Relation, Trace, TraceLink};

fn lane(direction: Direction) -> LaneSpec {
    LaneSpec::new(PositiveWidth::new(3.5).unwrap(), direction)
}

/// A tee whose east and west arms do not join, so there is a restriction to write,
/// with a signalised stop line and a crossing on the north arm and two buildings —
/// one of a single part and one of two stacked parts — well clear of the road.
fn town() -> ValidatedMap {
    let mut builder = MapBuilder::new(MapMetadata {
        name: Some("trace".to_owned()),
        origin: GeoOrigin::new(35.68, 139.76, 0.0).unwrap(),
        ..MapMetadata::default()
    });
    let arms: Vec<RoadId> = [
        ("north", (0.0, 70.0), (0.0, 14.0)),
        ("east", (70.0, 0.0), (14.0, 0.0)),
        ("west", (-70.0, 0.0), (-14.0, 0.0)),
    ]
    .into_iter()
    .map(|(name, start, end)| {
        let spec = RoadSpec::line(
            Point3::new(start.0, start.1, 0.0),
            Point3::new(end.0, end.1, 0.0),
            vec![lane(Direction::Forward), lane(Direction::Backward)],
        )
        .unwrap()
        .with_name(name);
        builder.add_road(spec).unwrap()
    })
    .collect();
    let junction = builder.add_junction(Some("t"));
    for other in &arms[1..] {
        builder
            .connect_ends(&arms[0], RoadEnd::End, other, RoadEnd::End, Some(&junction))
            .unwrap();
    }

    let approach = LaneRef::new(arms[0].clone(), 0);
    builder.add_stop_line(&approach, LaneEnd::End).unwrap();
    builder
        .add_traffic_light(&approach, LaneEnd::End, 5.0)
        .unwrap();
    builder
        .add_traffic_sign(&approach, LaneEnd::Start, "speed_limit", 2.0)
        .unwrap();
    builder.add_crosswalk(&arms[0], 0.5, 4.0).unwrap();

    let mut map = builder.finish().unwrap();
    let square = |x: f64, z: f64| {
        Footprint::new([
            Point3::new(x, 200.0, z),
            Point3::new(x + 10.0, 200.0, z),
            Point3::new(x + 10.0, 210.0, z),
            Point3::new(x, 210.0, z),
        ])
        .unwrap()
    };
    let buildings = map.as_map_mut();
    for (name, stacked) in [("single", false), ("stacked", true)] {
        let id = BuildingId::new(name);
        let x = if stacked { 50.0 } else { 0.0 };
        let mut solids = vec![Solid::prism(square(x, 0.0), 6.0)];
        if stacked {
            solids.push(Solid::prism(square(x, 6.0), 9.0));
        }
        let mut parts = Vec::new();
        for (index, solid) in solids.into_iter().enumerate() {
            let part = BuildingPartId::of_building(&id, index);
            buildings
                .building_parts
                .insert(
                    part.clone(),
                    BuildingPart {
                        id: part.clone(),
                        building: id.clone(),
                        solid,
                        kind: None,
                        levels: 2,
                    },
                )
                .unwrap();
            parts.push(part);
        }
        buildings
            .buildings
            .insert(
                id.clone(),
                Building {
                    id,
                    parts,
                    kind: "house".to_owned(),
                    frontage: None,
                },
            )
            .unwrap();
    }
    map.validate().unwrap()
}

fn export(map: &ValidatedMap) -> (ll2_io::osm::Document, Trace) {
    let (xml, trace) = roadgen_osm::to_xml_traced(map).unwrap();
    let (document, errors) = ll2_io::osm::parse(&xml).unwrap();
    assert!(errors.is_empty(), "{errors:?}");
    (document, trace)
}

#[test]
fn every_traced_element_is_in_the_file_as_what_it_says_it_is() {
    let (document, trace) = export(&town());
    assert_eq!(trace.format, "osm");
    assert!(trace.files.is_empty(), "nothing was written to disk");
    for link in &trace.links {
        let (kind, local) = link.local.split_once(':').expect(&link.local);
        let id: i64 = local.parse().expect(&link.local);
        let found = match kind {
            "node" => document.nodes.contains_key(&id),
            "way" => document.ways.contains_key(&id),
            "relation" => document.relations.contains_key(&id),
            _ => panic!("unknown kind in {}", link.local),
        };
        assert!(found, "{} is not in the file", link.local);
    }
}

#[test]
fn every_road_but_a_connector_is_exactly_one_way() {
    let map = town();
    let (document, trace) = export(&map);
    for road in map.roads.iter() {
        let links: Vec<_> = trace.links_of(&IrRef::Road(road.id.clone())).collect();
        if road.is_connector() {
            assert!(links.is_empty(), "{} is a connector: {links:?}", road.id);
            continue;
        }
        assert_eq!(links.len(), 1, "{}: {links:?}", road.id);
        assert_eq!(links[0].relation, Relation::Exact);
        let id: i64 = links[0]
            .local
            .strip_prefix("way:")
            .unwrap()
            .parse()
            .unwrap();
        assert!(document.ways[&id].tags.contains_key("highway"));
    }
}

#[test]
fn junctions_furniture_restrictions_and_buildings_are_traced() {
    let map = town();
    let (document, trace) = export(&map);
    let id_of = |local: &str| -> i64 { local.split_once(':').unwrap().1.parse().unwrap() };

    for junction in map.junctions.iter() {
        let links: Vec<_> = trace
            .links_of(&IrRef::Junction(junction.id.clone()))
            .collect();
        let node: Vec<_> = links
            .iter()
            .filter(|l| l.local.starts_with("node:"))
            .collect();
        assert_eq!(node.len(), 1);
        assert_eq!(node[0].relation, Relation::Exact);
        // East and west do not join, so one restriction each way, via this node.
        let restrictions: Vec<_> = links
            .iter()
            .filter(|l| l.role.as_deref() == Some("restriction"))
            .collect();
        assert_eq!(restrictions.len(), 2, "{links:?}");
        for link in restrictions {
            assert_eq!(link.relation, Relation::Part);
            let relation = &document.relations[&id_of(&link.local)];
            assert!(relation
                .members
                .iter()
                .any(|m| m.role == "via" && node_ref(m.reference) == node[0].local));
        }
    }

    // The stop line and the light share one node, and both are traced to it though
    // only the light's tag survives there.
    let furniture = |prefix: &str| -> Vec<&TraceLink> {
        let object = map
            .objects
            .iter()
            .find(|object| object.id.local_name().starts_with(prefix))
            .unwrap();
        trace.links_of(&IrRef::Object(object.id.clone())).collect()
    };
    let stop = furniture("stopline");
    let light = furniture("trafficlight");
    assert_eq!(stop.len(), 1);
    assert_eq!(light.len(), 1);
    assert_eq!(stop[0].local, light[0].local);
    assert_eq!(stop[0].relation, Relation::Merged);
    assert_eq!(stop[0].role.as_deref(), Some("stop"));
    assert_eq!(light[0].role.as_deref(), Some("traffic_signals"));
    let node = &document.nodes[&id_of(&light[0].local)];
    assert_eq!(
        node.tags.get("highway").map(String::as_str),
        Some("traffic_signals")
    );

    let sign = furniture("trafficsign");
    assert_eq!(sign.len(), 1);
    assert_eq!(sign[0].role.as_deref(), Some("traffic_sign"));

    let crossing = furniture("crosswalk");
    assert_eq!(crossing.len(), 2, "{crossing:?}");
    assert!(crossing
        .iter()
        .any(|l| l.relation == Relation::Merged && l.local.starts_with("node:")));
    let footway = crossing
        .iter()
        .find(|l| l.local.starts_with("way:"))
        .unwrap();
    assert_eq!(footway.relation, Relation::Exact);
    assert_eq!(footway.role.as_deref(), Some("crossing"));
    let way = &document.ways[&id_of(&footway.local)];
    assert_eq!(
        way.tags.get("footway").map(String::as_str),
        Some("crossing")
    );

    for building in map.buildings.iter() {
        let outline: Vec<_> = trace
            .links_of(&IrRef::Building(building.id.clone()))
            .collect();
        assert_eq!(outline.len(), 1);
        assert_eq!(outline[0].relation, Relation::Exact);
        assert_eq!(outline[0].role.as_deref(), Some("outline"));
        let mut part_ways = HashSet::new();
        for part in &building.parts {
            let links: Vec<_> = trace.links_of(&IrRef::BuildingPart(part.clone())).collect();
            assert_eq!(links.len(), 1, "{part}");
            assert_eq!(links[0].relation, Relation::Exact);
            part_ways.insert(links[0].local.clone());
        }
        if building.parts.len() == 1 {
            // A simple building is one way, which is its part as well.
            assert!(part_ways.contains(&outline[0].local));
        } else {
            assert_eq!(part_ways.len(), building.parts.len());
            assert!(!part_ways.contains(&outline[0].local));
            for local in &part_ways {
                let tags = &document.ways[&id_of(local)].tags;
                assert_eq!(tags.get("building:part").map(String::as_str), Some("yes"));
            }
        }
    }
}

fn node_ref(id: i64) -> String {
    format!("node:{id}")
}

#[test]
fn the_trace_is_the_same_every_time_and_names_the_file_it_wrote() {
    let map = town();
    let (xml, trace) = roadgen_osm::to_xml_traced(&map).unwrap();
    let (again_xml, again) = roadgen_osm::to_xml_traced(&map).unwrap();
    assert_eq!(xml, again_xml);
    assert_eq!(trace, again);
    assert_eq!(xml, roadgen_osm::to_xml(&map).unwrap());

    let path = std::env::temp_dir().join(format!("roadgen-osm-trace-{}.osm", std::process::id()));
    let written = roadgen_osm::write_traced(&map, &path).unwrap();
    let on_disk = std::fs::read_to_string(&path).unwrap();
    std::fs::remove_file(&path).ok();
    assert_eq!(on_disk, xml);
    assert_eq!(written.files, vec![path]);
    assert_eq!(written.links, trace.links);
}
