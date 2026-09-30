//! The trace a scene is written with: every element it names is one the scene holds,
//! under the id the scene gives it, and the same map traces the same way twice.

use std::collections::{BTreeMap, HashSet};

use roadgen_core::prelude::*;
use roadgen_core::{IrRef, Relation, Trace};
use roadgen_gpudrive::{Agent, ObjectKind, RoadKind, Scene, SceneConfig};

fn lane(width: f64, direction: Direction) -> LaneSpec {
    LaneSpec::new(PositiveWidth::new(width).unwrap(), direction)
}

/// Two arms meeting at a junction, each with a pavement that is not driven on, a
/// crossing and a stop sign.
fn crossroads() -> ValidatedMap {
    let mut builder = MapBuilder::new(MapMetadata {
        name: Some("traced".into()),
        ..MapMetadata::default()
    });
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
        let lanes = vec![
            lane(3.5, Direction::Forward),
            lane(3.5, Direction::Backward),
            lane(2.0, Direction::Forward).with_type(LaneType::Sidewalk),
        ];
        roads.push(
            builder
                .add_road(RoadSpec::line(start, end, lanes).unwrap().with_name(name))
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
    let approach = LaneRef::new(roads[0].clone(), 0);
    builder
        .add_traffic_sign(&approach, LaneEnd::End, "stop", 2.0)
        .unwrap();
    builder.add_crosswalk(&roads[0], 0.8, 4.0).unwrap();
    builder.finish().unwrap().validate().unwrap()
}

fn config() -> SceneConfig {
    SceneConfig::new("traced").with_agents(vec![
        Agent::new(ObjectKind::Vehicle),
        Agent::new(ObjectKind::Vehicle).with_speed(5.0),
    ])
}

fn split(local: &str) -> (&str, u32) {
    let (kind, id) = local.split_once(':').expect("<kind>:<id>");
    (kind, id.parse().expect("an id"))
}

/// Every link names an element the scene holds, and every road element is named.
fn check_against(trace: &Trace, scene: &Scene) {
    assert_eq!(trace.format, "gpudrive");
    let roads: HashSet<u32> = scene.roads.iter().map(|road| road.id).collect();
    let agents: HashSet<u32> = scene.objects.iter().map(|object| object.id).collect();
    for (index, road) in scene.roads.iter().enumerate() {
        assert_eq!(road.id as usize, index, "a road's id is its row");
    }
    for link in &trace.links {
        let (kind, id) = split(&link.local);
        match kind {
            "road" => assert!((id as usize) < scene.roads.len() && roads.contains(&id)),
            "agent" => assert!(agents.contains(&id), "{}", link.local),
            other => panic!("{other} is not a kind this trace writes"),
        }
    }
    let named: HashSet<&str> = trace.links.iter().map(|link| link.local.as_str()).collect();
    for road in &scene.roads {
        assert!(named.contains(format!("road:{}", road.id).as_str()));
    }
    for object in &scene.objects {
        assert!(named.contains(format!("agent:{}", object.id).as_str()));
    }
}

#[test]
fn the_trace_names_the_elements_of_the_json_it_came_with() {
    let map = crossroads();
    let (json, trace) = roadgen_gpudrive::to_json_traced(&map, &config()).unwrap();
    assert!(trace.files.is_empty());
    let scene: Scene = serde_json::from_str(&json).unwrap();
    assert_eq!(scene.objects.len(), 2);
    check_against(&trace, &scene);
}

#[test]
fn the_trace_names_the_elements_of_the_file_it_wrote() {
    let map = crossroads();
    let path = std::env::temp_dir().join(format!(
        "roadgen-gpudrive-trace-{}.json",
        std::process::id()
    ));
    let trace = roadgen_gpudrive::write_traced(&map, &path, &config()).unwrap();
    assert_eq!(trace.files, vec![path.clone()]);
    let scene: Scene = serde_json::from_str(&std::fs::read_to_string(&path).unwrap()).unwrap();
    check_against(&trace, &scene);
    std::fs::remove_file(&path).unwrap();
}

#[test]
fn every_drivable_lane_is_exactly_one_centreline_and_nothing_else_is() {
    let map = crossroads();
    let (scene, trace) = roadgen_gpudrive::to_scene_traced(&map, &config()).unwrap();
    for lane in map.lanes.iter() {
        let ir = IrRef::Lane(lane.id.clone());
        let centrelines: Vec<_> = trace
            .links_of(&ir)
            .filter(|link| {
                let (kind, id) = split(&link.local);
                kind == "road" && scene.roads[id as usize].kind == RoadKind::Lane
            })
            .collect();
        if lane.lane_type.is_drivable() {
            assert_eq!(centrelines.len(), 1, "{}", lane.id);
            assert_eq!(centrelines[0].relation, Relation::Exact);
            assert_eq!(centrelines[0].role, None);
        } else {
            assert!(centrelines.is_empty(), "{} is not driven on", lane.id);
        }
    }
}

#[test]
fn edges_are_merged_when_shared_and_agents_are_parts_of_their_routes() {
    let map = crossroads();
    let (scene, trace) = roadgen_gpudrive::to_scene_traced(&map, &config()).unwrap();

    let mut by_element: BTreeMap<u32, Vec<_>> = BTreeMap::new();
    for link in &trace.links {
        let (kind, id) = split(&link.local);
        if kind == "road" {
            by_element.entry(id).or_default().push(link);
        }
    }
    let mut merged = 0;
    for (id, links) in &by_element {
        let road = &scene.roads[*id as usize];
        let role = match road.kind {
            RoadKind::RoadEdge => Some("road_edge"),
            RoadKind::RoadLine => Some("road_line"),
            _ => None,
        };
        if role.is_none() {
            assert_eq!(links.len(), 1);
            continue;
        }
        let expected = if links.len() > 1 {
            merged += 1;
            Relation::Merged
        } else {
            Relation::Exact
        };
        for link in links {
            assert_eq!(link.relation, expected);
            assert_eq!(link.role.as_deref(), role);
        }
    }
    assert!(merged > 0, "the line between the two directions is shared");

    for object in map.objects.iter() {
        let ir = IrRef::Object(object.id.clone());
        let links: Vec<_> = trace.links_of(&ir).collect();
        assert_eq!(links.len(), 1, "{}", object.id);
        assert_eq!(links[0].relation, Relation::Exact);
    }

    for object in &scene.objects {
        let local = format!("agent:{}", object.id);
        let links: Vec<_> = trace.links_to(&local).collect();
        assert!(!links.is_empty());
        for link in links {
            assert_eq!(link.relation, Relation::Part);
            assert!(
                matches!(link.role.as_deref(), Some("route" | "goal")),
                "{link:?}"
            );
            assert!(matches!(&link.ir, IrRef::Lane(id) if map.lane(id).is_some()));
        }
    }
}

/// The lanes linked to the one agent of `agent`'s scene, by role.
fn agent_lanes(map: &ValidatedMap, agent: Agent) -> (Vec<LaneId>, Vec<LaneId>) {
    let config = SceneConfig::new("traced").with_agents(vec![agent]);
    let (_, trace) = roadgen_gpudrive::to_scene_traced(map, &config).unwrap();
    let of_role = |role: &str| {
        trace
            .links_to("agent:1")
            .filter(|link| link.role.as_deref() == Some(role))
            .map(|link| match &link.ir {
                IrRef::Lane(id) => id.clone(),
                other => panic!("an agent is made of lanes, not {other}"),
            })
            .collect::<Vec<_>>()
    };
    (of_role("route"), of_role("goal"))
}

#[test]
fn an_agent_is_made_of_the_lanes_its_track_reaches_and_its_goal() {
    let map = crossroads();
    let route = map.route_from(&map.default_start().unwrap());
    assert!(route.len() > 2, "{route:?}");

    // Fast enough to reach the end of the route inside the scene: every lane of it.
    let (driven, goal) = agent_lanes(&map, Agent::new(ObjectKind::Vehicle).with_speed(50.0));
    assert_eq!(driven, route);
    assert!(goal.is_empty());

    // Standing still: the lane it stands on, and the one its goal is on — nothing in
    // between, since nothing written is there.
    let (driven, goal) = agent_lanes(&map, Agent::new(ObjectKind::Vehicle).with_speed(0.0));
    assert_eq!(driven, route[..1]);
    assert_eq!(goal, route[route.len() - 1..]);

    // A route of one lane, standing still: the goal is on the lane it stands on, and
    // is still ahead of it.
    let mut still = Agent::new(ObjectKind::Vehicle).with_speed(0.0);
    still.route = Some(Route::Lanes(route[..1].to_vec()));
    let (driven, goal) = agent_lanes(&map, still);
    assert_eq!(driven, route[..1]);
    assert_eq!(goal, route[..1]);

    // 9 s at 5 m/s is 45 m, short of the end of a 56 m approach.
    let (driven, goal) = agent_lanes(&map, Agent::new(ObjectKind::Vehicle).with_speed(5.0));
    assert_eq!(driven, route[..1]);
    assert_eq!(goal, route[route.len() - 1..]);
}

#[test]
fn the_same_map_traces_the_same_way() {
    let map = crossroads();
    let (first_json, first) = roadgen_gpudrive::to_json_traced(&map, &config()).unwrap();
    let (second_json, second) = roadgen_gpudrive::to_json_traced(&map, &config()).unwrap();
    assert_eq!(first, second);
    assert_eq!(first_json, second_json);
    assert_eq!(
        first_json,
        roadgen_gpudrive::to_json(&map, &config()).unwrap()
    );
}
