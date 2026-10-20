//! A lanelet through a junction says which way it turns.
//!
//! Autoware reads a lanelet with a `turn_direction` tag as one inside an
//! intersection, and the value as the movement it makes there — what its planners
//! and a scenario's "the lane before a junction" both go by. The OpenDRIVE says the
//! same thing by a road's `junction` attribute and the connector's shape, so the
//! Lanelet2 export writes the tag on every junction connector's `road` lanelet, and
//! on nothing else: not an arm's, not a pavement's through the junction, and not a
//! motorway junction's slip road (`highway`), which is no intersection.

use std::collections::BTreeMap;

use ll2_core::map::as_lanelet;
use roadgen_core::trace::IrRef;
use roadgen_integration_tests::reload_lanelet2_traced;
use roadgen_opendrive::from_xml;

/// Road 1 from the west into junction 10, whose connectors go straight on to road 2
/// (east), turn left to road 3 (north) and right to road 4 (south). A pavement runs
/// along road 1's right and through the junction to road 2's, as CARLA's do.
const CROSSROADS: &str = r#"<?xml version="1.0" standalone="yes"?>
<OpenDRIVE>
  <header revMajor="1" revMinor="4" name="crossroads" version="1.00" date="2026-10-20" north="0" south="0" east="0" west="0"/>
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

/// Each lane's lanelet's `turn_direction`, keyed by its road and lane type, read
/// back from the exported file through the export's trace.
fn turn_directions() -> BTreeMap<(String, String), Option<String>> {
    let map = from_xml(CROSSROADS)
        .expect("the document reads")
        .map
        .validate()
        .unwrap_or_else(|error| panic!("{error}"));
    let (loaded, trace) = reload_lanelet2_traced(&map);
    let lanelets: BTreeMap<i64, _> = loaded
        .lanelets
        .all()
        .iter()
        .filter_map(as_lanelet)
        .map(|lanelet| (lanelet.id(), lanelet.clone()))
        .collect();
    map.lanes
        .iter()
        .filter_map(|lane| {
            let id: i64 = trace
                .links_of(&IrRef::Lane(lane.id.clone()))
                .find_map(|link| link.local.strip_prefix("lanelet:")?.parse().ok())?;
            let attributes = lanelets[&id].attributes().read();
            let road = lane.road.to_string();
            let kind = attributes["subtype"].value().to_owned();
            let turn = attributes
                .get("turn_direction")
                .map(|value| value.value().to_owned());
            Some(((road, kind), turn))
        })
        .collect()
}

#[test]
fn a_junction_connectors_driving_lane_says_which_way_it_turns() {
    let turns = turn_directions();
    let of = |road: &str| turns[&(format!("road/{road}"), "road".to_owned())].as_deref();
    assert_eq!(of("100"), Some("straight"), "{turns:?}");
    assert_eq!(of("101"), Some("left"), "{turns:?}");
    assert_eq!(of("102"), Some("right"), "{turns:?}");
}

#[test]
fn only_a_junction_connectors_driving_lane_says_so() {
    let turns = turn_directions();
    let tagged: Vec<_> = turns
        .iter()
        .filter(|(_, turn)| turn.is_some())
        .map(|(key, _)| key)
        .collect();
    assert_eq!(
        tagged,
        [
            &("road/100".to_owned(), "road".to_owned()),
            &("road/101".to_owned(), "road".to_owned()),
            &("road/102".to_owned(), "road".to_owned()),
        ],
        "neither an arm's lanes nor the pavement through the junction: {turns:?}"
    );
}
