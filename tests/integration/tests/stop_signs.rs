//! A stop sign, handed to Autoware.
//!
//! Autoware stops for a sign at a line when a lanelet carries a `traffic_sign`
//! regulatory element that refers to a `stop_sign` and names the line as its
//! `ref_line`. The IR has a sign — a position, the lanes it is for, and its
//! catalogue code — and a stop line across the lanes that stop, but no rule joining
//! them, so the Lanelet2 export makes one for each stop sign and hangs it where a
//! junction's light hangs: on the lanelets through the junction the stopping lanes
//! feed. These check it does, both for a sign the caller stood at an approach and
//! for one as CARLA's towns draw theirs, on a post and named from the connecting
//! roads with the line painted across the mouth.

use std::collections::BTreeSet;

use ll2_core::map::{as_lanelet, as_regulatory_element, LaneletMap};
use ll2_core::regelem::{RegulatoryElement, RuleParameter};
use roadgen_core::prelude::*;
use roadgen_core::trace::{IrRef, Trace};
use roadgen_integration_tests::reload_lanelet2_traced;
use roadgen_integration_tests::scenarios::{metadata, two_way};
use roadgen_opendrive::from_xml;

/// Road 1 from the west into junction 10, whose connectors go straight on to road 2
/// (100), left to road 3 (101) and right to road 4 (102). The stop sign stands on a
/// post beside the junction (road 5), valid on none of its lanes, and the three
/// connectors refer to it; its line is painted on road 100 across road 1's mouth.
const STOP_SIGNED: &str = r#"<?xml version="1.0" standalone="yes"?>
<OpenDRIVE>
  <header revMajor="1" revMinor="4" name="stop_signed" version="1.00" date="2026-10-20" north="0" south="0" east="0" west="0"/>
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
          <lane id="-2" type="sidewalk" level="false"><width sOffset="0.0" a="2.0" b="0.0" c="0.0" d="0.0"/></lane>
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
          <lane id="-2" type="sidewalk" level="false"><width sOffset="0.0" a="2.0" b="0.0" c="0.0" d="0.0"/></lane>
        </right>
      </laneSection>
    </lanes>
  </road>
  <road name="north" length="100.0" id="3" junction="-1">
    <link><predecessor elementType="junction" elementId="10"/></link>
    <planView>
      <geometry s="0.0" x="110.0" y="10.0" hdg="1.5707963267948966" length="100.0"><line/></geometry>
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
  <road name="south" length="100.0" id="4" junction="-1">
    <link><predecessor elementType="junction" elementId="10"/></link>
    <planView>
      <geometry s="0.0" x="110.0" y="-10.0" hdg="-1.5707963267948966" length="100.0"><line/></geometry>
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
  <road name="post" length="10.0" id="5" junction="-1">
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
      <signal s="9.0" t="-1.0" id="70" name="Sign_Stop" dynamic="no" orientation="-" zOffset="0.0" type="206" subtype="-1" country="DE" height="0.8">
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
          <lane id="-2" type="sidewalk" level="false">
            <link><predecessor id="-2"/><successor id="-2"/></link>
            <width sOffset="0.0" a="2.0" b="0.0" c="0.0" d="0.0"/>
          </lane>
        </right>
      </laneSection>
    </lanes>
    <objects>
      <object id="900" name="StopLine" type="-1" s="1.5" t="-1.75" zOffset="0.0" hdg="1.5707963267948966" pitch="0.0" roll="0.0" orientation="+" length="3.5" width="0.0">
        <outline>
          <cornerLocal u="-1.75" v="0.0" z="0.0"/>
          <cornerLocal u="1.75" v="0.0" z="0.0"/>
        </outline>
      </object>
    </objects>
    <signals>
      <signalReference s="0.0" t="0.0" id="70" orientation="+"><validity fromLane="-1" toLane="-1"/></signalReference>
    </signals>
  </road>
  <road name="" length="15.707963267948966" id="101" junction="10">
    <link>
      <predecessor elementType="road" elementId="1" contactPoint="end"/>
      <successor elementType="road" elementId="3" contactPoint="start"/>
    </link>
    <planView>
      <geometry s="0.0" x="100.0" y="0.0" hdg="0.0" length="15.707963267948966"><arc curvature="0.1"/></geometry>
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
      <signalReference s="0.0" t="0.0" id="70" orientation="+"><validity fromLane="-1" toLane="-1"/></signalReference>
    </signals>
  </road>
  <road name="" length="15.707963267948966" id="102" junction="10">
    <link>
      <predecessor elementType="road" elementId="1" contactPoint="end"/>
      <successor elementType="road" elementId="4" contactPoint="start"/>
    </link>
    <planView>
      <geometry s="0.0" x="100.0" y="0.0" hdg="0.0" length="15.707963267948966"><arc curvature="-0.1"/></geometry>
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
      <signalReference s="0.0" t="0.0" id="70" orientation="+"><validity fromLane="-1" toLane="-1"/></signalReference>
    </signals>
  </road>
  <junction id="10" name="junction">
    <connection id="0" incomingRoad="1" connectingRoad="100" contactPoint="start">
      <laneLink from="-1" to="-1"/>
      <laneLink from="-2" to="-2"/>
    </connection>
    <connection id="1" incomingRoad="1" connectingRoad="101" contactPoint="start">
      <laneLink from="-1" to="-1"/>
    </connection>
    <connection id="2" incomingRoad="1" connectingRoad="102" contactPoint="start">
      <laneLink from="-1" to="-1"/>
    </connection>
  </junction>
</OpenDRIVE>"#;

/// The ids of the elements a written IR element became, of one `kind`.
fn written(trace: &Trace, ir: IrRef, kind: &str) -> BTreeSet<i64> {
    trace
        .links_of(&ir)
        .filter_map(|link| link.local.strip_prefix(&format!("{kind}:"))?.parse().ok())
        .collect()
}

/// Every `traffic_sign` element in the file, and the lanelets holding each.
fn sign_elements(loaded: &LaneletMap) -> Vec<(RegulatoryElement, BTreeSet<i64>)> {
    loaded
        .regulatory_elements
        .all()
        .iter()
        .filter_map(as_regulatory_element)
        .filter(|element| element.attributes().read()["subtype"].value() == "traffic_sign")
        .map(|element| {
            let holders = loaded
                .lanelets
                .all()
                .iter()
                .filter_map(as_lanelet)
                .filter(|lanelet| {
                    lanelet
                        .regulatory_elements()
                        .iter()
                        .any(|held| held.id() == element.id())
                })
                .map(|lanelet| lanelet.id())
                .collect();
            (element.clone(), holders)
        })
        .collect()
}

fn line_ids(element: &RegulatoryElement, role: &str) -> BTreeSet<i64> {
    element
        .parameters_for(role)
        .iter()
        .filter_map(|parameter| match parameter {
            RuleParameter::LineString(line) => Some(line.id()),
            _ => None,
        })
        .collect()
}

/// The lanelets of the driving lanes through the junction that `lane` feeds.
fn junction_lanelets(map: &ValidatedMap, trace: &Trace, lane: &LaneId) -> BTreeSet<i64> {
    map.successors(lane)
        .into_iter()
        .filter(|next| {
            map.lane(next)
                .is_some_and(|next| next.lane_type == LaneType::Driving)
                && map
                    .lane(next)
                    .and_then(|next| map.road(&next.road))
                    .is_some_and(|road| road.is_connector())
        })
        .flat_map(|next| written(trace, IrRef::Lane(next), "lanelet"))
        .collect()
}

/// What a stop sign became: one element referring to the sign's way, which says it
/// is a `stop_sign`; its stop line as the `ref_line`; held by the lanelets through
/// the junction that the stopping lane feeds, and by nothing else.
fn assert_stop_sign_reaches_autoware(
    map: &ValidatedMap,
    sign: &ObjectId,
    stop_line: &ObjectId,
    approach: &LaneId,
) {
    let (loaded, trace) = reload_lanelet2_traced(map);
    let elements = sign_elements(&loaded);
    assert_eq!(elements.len(), 1, "one stop sign, one element");
    let (element, holders) = &elements[0];

    let sign_lines = written(&trace, IrRef::Object(sign.clone()), "linestring");
    assert_eq!(
        line_ids(element, "refers"),
        sign_lines,
        "it refers to the sign"
    );
    for parameter in element.parameters_for("refers") {
        let RuleParameter::LineString(line) = parameter else {
            panic!("a sign is a way")
        };
        assert_eq!(line.attributes().read()["subtype"].value(), "stop_sign");
    }
    assert_eq!(
        written(&trace, IrRef::Object(sign.clone()), "regulatory_element"),
        BTreeSet::from([element.id()]),
        "the trace names the element the sign became"
    );
    assert_eq!(
        line_ids(element, "ref_line"),
        written(&trace, IrRef::Object(stop_line.clone()), "linestring"),
        "it stops at the line across the approach"
    );

    let through = junction_lanelets(map, &trace, approach);
    assert!(!through.is_empty());
    assert_eq!(
        holders, &through,
        "held by the lanelets through the junction"
    );
    let own = written(&trace, IrRef::Lane(approach.clone()), "lanelet");
    assert!(holders.is_disjoint(&own), "not by the approach itself");
}

#[test]
fn a_carla_stop_sign_stops_its_connectors_at_the_painted_line() {
    let map = from_xml(STOP_SIGNED)
        .expect("the document reads")
        .map
        .validate()
        .unwrap_or_else(|error| panic!("{error}"));
    let only = |kind: fn(&MapObjectKind) -> bool| {
        let found: Vec<_> = map
            .objects
            .iter()
            .filter(|object| kind(&object.kind))
            .collect();
        assert_eq!(found.len(), 1, "{found:?}");
        found[0].id.clone()
    };
    let sign = only(|kind| matches!(kind, MapObjectKind::TrafficSign { .. }));
    let stop_line = only(|kind| *kind == MapObjectKind::StopLine);
    let approach = map
        .lanes_of(&RoadId::new("1"))
        .into_iter()
        .find(|lane| lane.lane_type == LaneType::Driving)
        .expect("road 1 drives into the junction")
        .id
        .clone();
    assert_stop_sign_reaches_autoware(&map, &sign, &stop_line, &approach);
}

/// West and north arms into one junction and an east arm out of it, every pair of
/// ends joined, with a stop line and a stop sign at the end of the west arm's
/// approach and a speed limit sign at its start.
#[test]
fn a_stop_sign_at_an_approach_stops_the_lanes_through_the_junction() {
    let mut builder = MapBuilder::new(metadata("stop_sign"));
    let mut arm = |name: &str, from: (f64, f64), to: (f64, f64)| {
        builder
            .add_road(
                RoadSpec::line(
                    Point3::new(from.0, from.1, 0.0),
                    Point3::new(to.0, to.1, 0.0),
                    two_way(),
                )
                .unwrap()
                .with_name(name),
            )
            .unwrap()
    };
    let west = arm("west", (-70.0, 0.0), (-14.0, 0.0));
    let east = arm("east", (14.0, 0.0), (70.0, 0.0));
    let north = arm("north", (0.0, 70.0), (0.0, 14.0));
    let junction = builder.add_junction(Some("x"));
    let ends = [
        (&west, RoadEnd::End),
        (&east, RoadEnd::Start),
        (&north, RoadEnd::End),
    ];
    for (index, (from, from_end)) in ends.iter().enumerate() {
        for (to, to_end) in ends.iter().skip(index + 1) {
            builder
                .connect_ends(from, *from_end, to, *to_end, Some(&junction))
                .unwrap();
        }
    }
    let approach = LaneRef::new(west.clone(), 0);
    let stop_line = builder.add_stop_line(&approach, LaneEnd::End).unwrap();
    let sign = builder
        .add_traffic_sign(&approach, LaneEnd::End, "stop", 2.0)
        .unwrap();
    builder
        .add_traffic_sign(&approach, LaneEnd::Start, "de274-50", 2.0)
        .unwrap();
    let map = builder.finish().unwrap().validate().unwrap();
    // The lane the line was put across, as the IR names it.
    let approach = map.objects.get(&stop_line).unwrap().lanes[0].clone();
    assert_stop_sign_reaches_autoware(&map, &sign, &stop_line, &approach);
}
