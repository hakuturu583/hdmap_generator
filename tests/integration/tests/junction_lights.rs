//! A junction's lights as CARLA's Town maps draw them, read and handed to Autoware.
//!
//! CARLA stands a junction's light on a post off the road — valid on no lane of
//! its own road (`fromLane="0" toLane="0"`) — and names the lanes it switches by
//! `<signalReference>`s on the junction's *connecting* roads, with no stop line
//! anywhere. The IR's traffic light, and Autoware's, is about the lanes that stop
//! for it: the approaches, with the stop line where they enter the junction. So the
//! reader moves the light onto the approaches that feed what the document names,
//! and draws the stop line at their mouth; these check that it does, and that the
//! Lanelet2 export then hangs the regulatory element, with a `ref_line`, on the
//! approach lanelets.

use std::collections::{BTreeSet, HashMap};

use ll2_core::map::as_lanelet;
use ll2_core::regelem::RuleParameter;
use roadgen_core::prelude::*;
use roadgen_core::trace::{IrRef, Relation, Trace};
use roadgen_integration_tests::reload_lanelet2_traced;
use roadgen_opendrive::{from_xml, Imported};

/// Two roads into one junction and one road out of it: road 1 from the west with
/// two lanes straight across, road 3 from the south with one lane turning right
/// into road 2, east. The light stands on road 4, a pavement beside the junction,
/// and the connecting roads 100 and 101 refer to it; one controller switches it.
const CARLA_SHAPED: &str = r#"<?xml version="1.0" standalone="yes"?>
<OpenDRIVE>
  <header revMajor="1" revMinor="4" name="carla_shaped" version="1.00" date="2026-10-08" north="0" south="0" east="0" west="0"/>
  <road name="west" length="100.0" id="1" junction="-1">
    <link><successor elementType="junction" elementId="10"/></link>
    <planView>
      <geometry s="0.0" x="0.0" y="0.0" hdg="0.0" length="100.0"><line/></geometry>
    </planView>
    <lanes>
      <laneSection s="0.0">
        <center><lane id="0" type="none" level="false"/></center>
        <right>
          <lane id="-1" type="driving" level="false"><width sOffset="0.0" a="3.5" b="0.0" c="0.0" d="0.0"/></lane>
          <lane id="-2" type="driving" level="false"><width sOffset="0.0" a="3.5" b="0.0" c="0.0" d="0.0"/></lane>
        </right>
      </laneSection>
    </lanes>
  </road>
  <road name="east" length="100.0" id="2" junction="-1">
    <link><predecessor elementType="junction" elementId="10"/></link>
    <planView>
      <geometry s="0.0" x="120.0" y="0.0" hdg="0.0" length="100.0"><line/></geometry>
    </planView>
    <lanes>
      <laneSection s="0.0">
        <center><lane id="0" type="none" level="false"/></center>
        <right>
          <lane id="-1" type="driving" level="false"><width sOffset="0.0" a="3.5" b="0.0" c="0.0" d="0.0"/></lane>
          <lane id="-2" type="driving" level="false"><width sOffset="0.0" a="3.5" b="0.0" c="0.0" d="0.0"/></lane>
        </right>
      </laneSection>
    </lanes>
  </road>
  <road name="south" length="100.0" id="3" junction="-1">
    <link><successor elementType="junction" elementId="10"/></link>
    <planView>
      <geometry s="0.0" x="110.0" y="-110.0" hdg="1.5707963267948966" length="100.0"><line/></geometry>
    </planView>
    <lanes>
      <laneSection s="0.0">
        <center><lane id="0" type="none" level="false"/></center>
        <right>
          <lane id="-1" type="driving" level="false"><width sOffset="0.0" a="3.5" b="0.0" c="0.0" d="0.0"/></lane>
        </right>
      </laneSection>
    </lanes>
  </road>
  <road name="post" length="10.0" id="4" junction="-1">
    <planView>
      <geometry s="0.0" x="90.0" y="-12.0" hdg="0.0" length="10.0"><line/></geometry>
    </planView>
    <lanes>
      <laneSection s="0.0">
        <center><lane id="0" type="none" level="false"/></center>
        <right>
          <lane id="-1" type="sidewalk" level="false"><width sOffset="0.0" a="2.0" b="0.0" c="0.0" d="0.0"/></lane>
        </right>
      </laneSection>
    </lanes>
    <signals>
      <signal s="9.0" t="-1.0" id="42" name="Signal_3Light_Post01" dynamic="yes" orientation="-" zOffset="0.0" type="1000001" subtype="-1" country="OpenDRIVE" height="5.0">
        <validity fromLane="0" toLane="0"/>
      </signal>
    </signals>
  </road>
  <road name="" length="20.0" id="100" junction="10">
    <link>
      <predecessor elementType="road" elementId="1" contactPoint="end"/>
      <successor elementType="road" elementId="2" contactPoint="start"/>
    </link>
    <planView>
      <geometry s="0.0" x="100.0" y="0.0" hdg="0.0" length="20.0"><line/></geometry>
    </planView>
    <lanes>
      <laneSection s="0.0">
        <center><lane id="0" type="none" level="false"/></center>
        <right>
          <lane id="-1" type="driving" level="false">
            <link><predecessor id="-1"/><successor id="-1"/></link>
            <width sOffset="0.0" a="3.5" b="0.0" c="0.0" d="0.0"/>
          </lane>
          <lane id="-2" type="driving" level="false">
            <link><predecessor id="-2"/><successor id="-2"/></link>
            <width sOffset="0.0" a="3.5" b="0.0" c="0.0" d="0.0"/>
          </lane>
        </right>
      </laneSection>
    </lanes>
    <signals>
      <signalReference s="0.0" t="0.0" id="42" orientation="+"><validity fromLane="-2" toLane="-1"/></signalReference>
    </signals>
  </road>
  <road name="" length="15.707963267948966" id="101" junction="10">
    <link>
      <predecessor elementType="road" elementId="3" contactPoint="end"/>
      <successor elementType="road" elementId="2" contactPoint="start"/>
    </link>
    <planView>
      <geometry s="0.0" x="110.0" y="-10.0" hdg="1.5707963267948966" length="15.707963267948966"><arc curvature="-0.1"/></geometry>
    </planView>
    <lanes>
      <laneSection s="0.0">
        <center><lane id="0" type="none" level="false"/></center>
        <right>
          <lane id="-1" type="driving" level="false">
            <link><predecessor id="-1"/><successor id="-1"/></link>
            <width sOffset="0.0" a="3.5" b="0.0" c="0.0" d="0.0"/>
          </lane>
        </right>
      </laneSection>
    </lanes>
    <signals>
      <signalReference s="0.0" t="0.0" id="42" orientation="+"><validity fromLane="-1" toLane="-1"/></signalReference>
    </signals>
  </road>
  <controller name="ctrl7" id="7" sequence="0">
    <control signalId="42" type=""/>
    <control signalId="42" type=""/>
  </controller>
  <junction id="10" name="junction">
    <connection id="0" incomingRoad="1" connectingRoad="100" contactPoint="start">
      <laneLink from="-1" to="-1"/>
      <laneLink from="-2" to="-2"/>
    </connection>
    <connection id="1" incomingRoad="3" connectingRoad="101" contactPoint="start">
      <laneLink from="-1" to="-1"/>
    </connection>
    <controller id="7"/>
  </junction>
</OpenDRIVE>"#;

fn lane_of(map: &ValidatedMap, road: &str, ordinal: usize) -> LaneId {
    map.lanes_of(&RoadId::new(road))
        .into_iter()
        .find(|lane| lane.side == LateralSide::Right && lane.ordinal == ordinal)
        .unwrap_or_else(|| panic!("{road} has a right lane {ordinal}"))
        .id
        .clone()
}

fn sorted(lanes: &[LaneId]) -> Vec<LaneId> {
    let mut lanes = lanes.to_vec();
    lanes.sort();
    lanes
}

/// How each traffic-light rule of a map reached Lanelet2: the elements it was
/// written as, the lanelets of its lanes that carry one of them, whether that one
/// has a `ref_line` which is a `stop_line` way, and how many elements refer to
/// fewer of the rule's lights than all of them.
#[derive(Debug, PartialEq, Eq)]
struct Reached {
    rules: usize,
    elements: usize,
    lanes: usize,
    lanes_carrying: usize,
    with_ref_line: usize,
    narrowed: usize,
}

/// Exports `map` as Lanelet2, reads the file back with `simple_lanelet2`'s loader
/// — which knows nothing of the IR — and follows each traffic-light rule through
/// the export's trace to the lanelets of its lanes, asking each whether it holds
/// one of the rule's regulatory elements and whether that names its stop line.
fn reach_lanelet2(map: &ValidatedMap) -> Reached {
    let (loaded, trace) = reload_lanelet2_traced(map);
    let written = |ir: IrRef, kind: &str| -> Vec<i64> {
        trace
            .links_of(&ir)
            .filter(|link| link.role.as_deref() != Some("light_bulbs"))
            .filter_map(|link| link.local.strip_prefix(&format!("{kind}:"))?.parse().ok())
            .collect()
    };
    let lanelets: HashMap<i64, _> = loaded
        .lanelets
        .all()
        .iter()
        .filter_map(as_lanelet)
        .map(|lanelet| (lanelet.id(), lanelet.clone()))
        .collect();

    let mut reached = Reached {
        rules: 0,
        elements: 0,
        lanes: 0,
        lanes_carrying: 0,
        with_ref_line: 0,
        narrowed: 0,
    };
    for (index, rule) in map.rules.iter().enumerate() {
        let TrafficRule::TrafficLight { lights, lanes, .. } = rule else {
            continue;
        };
        reached.rules += 1;
        let elements = written(IrRef::Rule(index), "regulatory_element");
        reached.elements += elements.len();
        let every_light: BTreeSet<i64> = lights
            .iter()
            .flat_map(|light| written(IrRef::Object(light.clone()), "linestring"))
            .collect();
        let mut seen: BTreeSet<i64> = BTreeSet::new();
        for lane in lanes {
            reached.lanes += 1;
            let Some(lanelet) = written(IrRef::Lane(lane.clone()), "lanelet")
                .first()
                .and_then(|id| lanelets.get(id))
            else {
                continue;
            };
            let held: Vec<_> = lanelet
                .regulatory_elements()
                .into_iter()
                .filter(|held| elements.contains(&held.id()))
                .collect();
            assert!(held.len() <= 1, "a lane stops at one of its rule's lines");
            let Some(held) = held.first() else {
                continue;
            };
            assert_eq!(held.attributes().read()["subtype"].value(), "traffic_light");
            reached.lanes_carrying += 1;
            let stops = held.parameters_for("ref_line").iter().any(|parameter| {
                matches!(parameter, RuleParameter::LineString(line)
                    if line.attributes().read().get("type").map(|v| v.value()) == Some("stop_line"))
            });
            if stops {
                reached.with_ref_line += 1;
            }
            if seen.insert(held.id()) {
                let refers: BTreeSet<i64> = held
                    .parameters_for("refers")
                    .iter()
                    .filter_map(|parameter| match parameter {
                        RuleParameter::LineString(line) => Some(line.id()),
                        _ => None,
                    })
                    .collect();
                assert!(refers.is_subset(&every_light), "{refers:?} {every_light:?}");
                reached.narrowed += usize::from(refers != every_light);
            }
        }
    }
    reached
}

/// The traffic-light rules of a map, as `(lights, stop lines, lanes)`.
fn light_rules(map: &ValidatedMap) -> Vec<(&Vec<ObjectId>, &Vec<ObjectId>, &Vec<LaneId>)> {
    map.rules
        .iter()
        .filter_map(|rule| match rule {
            TrafficRule::TrafficLight {
                lights,
                stop_lines,
                lanes,
            } => Some((lights, stop_lines, lanes)),
            _ => None,
        })
        .collect()
}

#[test]
fn a_carla_junction_light_governs_the_approaches_and_stops_them_at_the_mouth() {
    let Imported {
        map,
        approximations: notes,
        trace,
    } = from_xml(CARLA_SHAPED).expect("the document reads");
    let map = map.validate().unwrap_or_else(|error| panic!("{error}"));

    // The light stands where the post is, and governs the lanes that stop for it:
    // both of road 1's and road 3's, not the connecting roads the references are on.
    let light = map
        .objects
        .iter()
        .find(|object| object.kind.is_traffic_light())
        .unwrap_or_else(|| panic!("the light is read: {notes:#?}"));
    let west = vec![lane_of(&map, "1", 1), lane_of(&map, "1", 2)];
    let south = vec![lane_of(&map, "3", 1)];
    let approaches: Vec<LaneId> = west.iter().chain(&south).cloned().collect();
    assert_eq!(sorted(&light.lanes), sorted(&approaches), "{notes:#?}");
    assert!(
        notes
            .iter()
            .any(|note| note.contains("read as governing the approaches")),
        "{notes:#?}"
    );

    // One controller is one rule, over both mouths, stopping at a line the reader
    // drew at each.
    let rules = light_rules(&map);
    assert_eq!(rules.len(), 1, "{:#?}", map.rules);
    let (lights, stop_lines, lanes) = rules[0];
    assert_eq!(lights, &vec![light.id.clone()]);
    assert_eq!(sorted(lanes), sorted(&approaches));
    assert_eq!(
        stop_lines,
        &vec![
            ObjectId::new("stopline/1/end"),
            ObjectId::new("stopline/3/end")
        ]
    );
    for (id, (expected, road)) in stop_lines.iter().zip([(&west, "1"), (&south, "3")]) {
        let line = map.objects.get(id).unwrap();
        assert_eq!(line.kind, MapObjectKind::StopLine);
        assert_eq!(sorted(&line.lanes), sorted(expected));
        // Across the lanes where they leave the road: at its end, from the
        // driver's left to the right, as wide as the lanes are.
        let ObjectGeometry::Line(curve) = &line.geometry else {
            panic!("a stop line is a line");
        };
        let entry = map.road(&RoadId::new(road)).unwrap();
        let end = entry.reference_line.end_point();
        let (from, to) = (curve.start_point(), curve.end_point());
        let width = 3.5 * expected.len() as f64;
        assert!(
            (from.distance_to(to) - width).abs() < 1e-6,
            "{from:?} {to:?}"
        );
        assert!(
            from.distance_to(end) < 1e-6,
            "from the centre line: {from:?}"
        );
        let frame = entry
            .frame_at(
                entry.horizontal_length().unwrap(),
                SamplingConfig::default(),
            )
            .unwrap();
        assert!((frame.to_local(to)[1] + width).abs() < 1e-6, "{to:?}");
    }

    // The trace says where they came from: the rule is the controller, the drawn
    // lines what it implied and the document has no element for, and the light a
    // part of it.
    let links: Vec<_> = trace.links_of(&IrRef::Rule(0)).collect();
    assert_eq!(links.len(), 1);
    assert_eq!(links[0].local, "controller:7");
    assert_eq!(links[0].relation, Relation::Exact);
    for id in stop_lines {
        let links: Vec<_> = trace.links_of(&IrRef::Object(id.clone())).collect();
        assert_eq!(links.len(), 1, "{links:?}");
        assert_eq!(links[0].local, "controller:7");
        assert_eq!(links[0].relation, Relation::Collapsed);
        assert_eq!(links[0].role.as_deref(), Some("stop_line"));
    }
    assert_eq!(
        trace
            .links_of(&IrRef::Object(light.id.clone()))
            .filter(|link| link.role.as_deref() == Some("controller"))
            .count(),
        1,
        "the light is traced once to its controller, though it is listed twice"
    );

    // And Autoware gets it the way it looks for it: every approach lanelet holds
    // a regulatory element, one per mouth, with that mouth's stop line as its
    // ref_line — and the light over both mouths is referred to by both.
    assert_eq!(
        reach_lanelet2(&map),
        Reached {
            rules: 1,
            elements: 2,
            lanes: 3,
            lanes_carrying: 3,
            with_ref_line: 3,
            narrowed: 0,
        }
    );
    assert!(roadgen_lanelet2::check(&map).is_empty());

    // The other formats take the map as they take any other.
    let document = roadgen_opendrive::to_xml(&map).expect("OpenDRIVE writes");
    let again = opendrive::core::OpenDrive::from_xml_str(&document).unwrap();
    assert_eq!(again.controller.len(), 1);
    let written_lines = again
        .road
        .iter()
        .filter_map(|road| road.objects.as_ref())
        .flat_map(|objects| objects.object.iter())
        .count();
    assert_eq!(written_lines, 2);
    let reread = from_xml(&document).unwrap().map.validate().unwrap();
    let rules = light_rules(&reread);
    assert_eq!(rules.len(), 1, "read back as one rule");
    // The writer puts a light's validity on the road of its first lane and writes
    // no `<signalReference>`, so the light comes back over road 1's lanes only —
    // and its rule, stopping them at the line the document now draws there.
    assert_eq!(rules[0].1, &vec![ObjectId::new("stopline/1/end")]);
    let sumo = roadgen_sumo::to_plain_xml(&map).expect("SUMO writes");
    assert!(sumo.traffic_lights.is_some(), "the junction is signalised");
    every_other_export(&map, true);
}

/// Two lights in one controller, one over each mouth: the rule still names both,
/// and each mouth's regulatory element refers to the light over it.
#[test]
fn each_mouth_of_a_controller_refers_to_the_lights_over_it() {
    let document = CARLA_SHAPED
        .replace(
            r#"<signalReference s="0.0" t="0.0" id="42" orientation="+"><validity fromLane="-1" toLane="-1"/>"#,
            r#"<signalReference s="0.0" t="0.0" id="43" orientation="+"><validity fromLane="-1" toLane="-1"/>"#,
        )
        .replace(
            "    </signals>\n  </road>\n  <road name=\"\" length=\"20.0\" id=\"100\"",
            "      <signal s=\"5.0\" t=\"-1.0\" id=\"43\" name=\"Signal_3Light_Post02\" dynamic=\"yes\" \
             orientation=\"-\" zOffset=\"0.0\" type=\"1000001\" subtype=\"-1\" country=\"OpenDRIVE\" \
             height=\"5.0\">\n        <validity fromLane=\"0\" toLane=\"0\"/>\n      </signal>\n    \
             </signals>\n  </road>\n  <road name=\"\" length=\"20.0\" id=\"100\"",
        )
        .replace(
            "<control signalId=\"42\" type=\"\"/>\n  </controller>",
            "<control signalId=\"43\" type=\"\"/>\n  </controller>",
        );
    let map = from_xml(&document)
        .expect("the document reads")
        .map
        .validate()
        .unwrap_or_else(|error| panic!("{error}"));
    let lights = map
        .objects
        .iter()
        .filter(|object| object.kind.is_traffic_light())
        .count();
    assert_eq!(lights, 2);
    let rules = light_rules(&map);
    assert_eq!(rules.len(), 1, "{:#?}", map.rules);
    assert_eq!(rules[0].0.len(), 2);
    assert_eq!(rules[0].1.len(), 2);
    assert_eq!(
        reach_lanelet2(&map),
        Reached {
            rules: 1,
            elements: 2,
            lanes: 3,
            lanes_carrying: 3,
            with_ref_line: 3,
            narrowed: 2,
        }
    );
}

/// Every export but Lanelet2's, OpenDRIVE's and SUMO's, which have checks of
/// their own: each takes the map as it takes any other. CARLA's meshes too, when
/// `carla`.
fn every_other_export(map: &ValidatedMap, carla: bool) {
    roadgen_osm::to_xml(map).expect("OpenStreetMap writes");
    let clip = roadgen_clipgt::ClipConfig::default();
    roadgen_clipgt::to_layers(map, &clip).expect("ClipGT writes");
    let scene = roadgen_gpudrive::SceneConfig::default();
    roadgen_gpudrive::to_scene(map, &scene).expect("GPUDrive writes");
    if carla {
        let package = roadgen_carla::PackageConfig::default();
        assert!(
            !roadgen_carla::to_meshes(map, &package).is_empty(),
            "CARLA meshes"
        );
    }
}

/// The document's ids of the signals an IR object was read from.
fn imported_trace_names(trace: &Trace, object: &ObjectId) -> Vec<String> {
    trace
        .links_of(&IrRef::Object(object.clone()))
        .filter_map(|link| link.local.strip_prefix("signal:").map(str::to_owned))
        .collect()
}

/// The same question of a real CARLA town. Not run by default: it needs the
/// town's OpenDRIVE, dumped from CARLA and put through the sanitising CARLA's own
/// reader needs no help with but a strict one does (carla_driver_interface's
/// `sanitize_opendrive`). Point `ROADGEN_CARLA_TOWN` at it and run
/// `cargo test -p roadgen-integration-tests --test junction_lights -- --ignored --nocapture`.
#[test]
#[ignore = "needs a CARLA town's OpenDRIVE in ROADGEN_CARLA_TOWN"]
fn a_carla_town_hands_autoware_every_light_on_its_approaches() {
    let path = std::env::var("ROADGEN_CARLA_TOWN").expect("ROADGEN_CARLA_TOWN names the town");
    let Imported {
        map,
        approximations,
        trace,
    } = roadgen_opendrive::read(&path).expect("the town reads");
    for note in &approximations {
        println!("note: {note}");
    }
    let map = map.validate().expect("the town validates");

    let lights: Vec<&roadgen_core::semantics::MapObject> = map
        .objects
        .iter()
        .filter(|object| object.kind.is_traffic_light())
        .collect();
    let on_connectors = |lanes: &[LaneId]| {
        lanes
            .iter()
            .filter(|lane| {
                map.lane(lane)
                    .and_then(|lane| map.road(&lane.road))
                    .is_some_and(|road| road.is_connector())
            })
            .count()
    };
    let light_lanes: usize = lights.iter().map(|light| light.lanes.len()).sum();
    let light_inside: usize = lights.iter().map(|light| on_connectors(&light.lanes)).sum();
    let mut ruled: BTreeSet<&ObjectId> = BTreeSet::new();
    let mut rules_inside = 0;
    let mut without_stop_line = 0;
    let mut stop_lines = 0;
    for (lights, lines, lanes) in light_rules(&map) {
        ruled.extend(lights);
        rules_inside += on_connectors(lanes);
        without_stop_line += usize::from(lines.is_empty());
        stop_lines += lines.len();
    }
    let reached = reach_lanelet2(&map);
    println!(
        "lights {} (in a rule {}), lanes they govern {light_lanes} (inside a junction \
         {light_inside})",
        lights.len(),
        ruled.len()
    );
    println!(
        "traffic-light rules {} (without a stop line {without_stop_line}, stop lines \
         {stop_lines}), rule lanes {} (inside a junction {rules_inside}); regulatory elements \
         {} (referring to fewer than all their rule's lights {}), lanelets holding one {}, of \
         which with a stop_line ref_line {}",
        reached.rules,
        reached.lanes,
        reached.elements,
        reached.narrowed,
        reached.lanes_carrying,
        reached.with_ref_line
    );
    assert_eq!(light_inside, 0);
    assert_eq!(rules_inside, 0);
    assert_eq!(without_stop_line, 0);
    // A light no `<controller>` names is in no rule — the document says nothing
    // of what it switches with — and so has no regulatory element; Town10 has two,
    // halfway along a connecting road. Every light a controller names is ruled.
    let document = std::fs::read_to_string(&path).unwrap();
    let document = opendrive::core::OpenDrive::from_xml_str(&document).unwrap();
    let controlled: BTreeSet<String> = document
        .controller
        .iter()
        .flat_map(|controller| controller.control.iter())
        .map(|control| control.signal_id.clone())
        .collect();
    let named = |light: &&&roadgen_core::semantics::MapObject| {
        imported_trace_names(&trace, &light.id)
            .iter()
            .any(|signal| controlled.contains(signal))
    };
    let controlled_lights: Vec<_> = lights.iter().filter(named).collect();
    println!(
        "lights a controller names {}, and none names {}",
        controlled_lights.len(),
        lights.len() - controlled_lights.len()
    );
    assert!(controlled_lights
        .iter()
        .all(|light| ruled.contains(&light.id)));
    assert_eq!(reached.lanes_carrying, reached.lanes);
    assert_eq!(reached.with_ref_line, reached.lanes);
    assert_eq!(
        roadgen_opendrive::check(&map),
        Vec::<String>::new(),
        "OpenDRIVE writes it"
    );
    roadgen_opendrive::to_xml(&map).expect("OpenDRIVE writes");
    // Not CARLA's meshes: the terrain triangulation fails on Town10 as it did
    // before lights were read this way, which is a question for the surface.
    every_other_export(&map, false);
}
