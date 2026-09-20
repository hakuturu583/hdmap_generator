//! A document roadgen did not write: a connecting road of two lanes, a width that
//! is a free cubic and one that tapers to nothing, an elevation that curves, a lane
//! type the IR does not have, a signal that names no lanes, and no geo-reference.
//! Each is read the way the reader says it is, the map validates, and what was
//! approximated is reported.

use roadgen_core::prelude::*;
use roadgen_core::topology::Direction;
use roadgen_opendrive::{check, from_xml, to_xml};

const FOREIGN: &str = r#"<?xml version="1.0" standalone="yes"?>
<OpenDRIVE>
  <header revMajor="1" revMinor="7" name="foreign" version="1.00" date="2026-09-20" north="0" south="0" east="0" west="0"/>
  <road name="approach" length="100.0" id="1" junction="-1">
    <link><successor elementType="junction" elementId="10"/></link>
    <type s="0.0" type="rural"><speed max="80" unit="km/h"/></type>
    <planView>
      <geometry s="0.0" x="0.0" y="0.0" hdg="0.0" length="100.0"><line/></geometry>
    </planView>
    <elevationProfile>
      <elevation s="0.0" a="0.0" b="0.0" c="0.001" d="0.0"/>
    </elevationProfile>
    <lanes>
      <laneSection s="0.0">
        <left>
          <lane id="1" type="driving" level="false">
            <width sOffset="0.0" a="3.5" b="0.0" c="0.0005" d="-0.000004"/>
            <roadMark sOffset="0.0" type="solid" weight="standard" color="standard" width="0.13"/>
          </lane>
        </left>
        <center>
          <lane id="0" type="none" level="false">
            <roadMark sOffset="0.0" type="solid solid" weight="standard" color="yellow" width="0.13"/>
          </lane>
        </center>
        <right>
          <lane id="-1" type="driving" level="false">
            <width sOffset="0.0" a="3.5" b="0.0" c="0.0" d="0.0"/>
            <roadMark sOffset="0.0" type="broken" weight="standard" color="standard" width="0.13"/>
            <speed sOffset="0.0" max="50" unit="km/h"/>
          </lane>
          <lane id="-2" type="driving" level="false">
            <width sOffset="0.0" a="3.5" b="0.0" c="0.0" d="0.0"/>
            <roadMark sOffset="0.0" type="solid" weight="standard" color="standard" width="0.13"/>
          </lane>
          <lane id="-3" type="stop" level="false">
            <width sOffset="0.0" a="2.5" b="-0.025" c="0.0" d="0.0"/>
            <roadMark sOffset="0.0" type="none" weight="standard" color="standard" width="0.13"/>
          </lane>
        </right>
      </laneSection>
    </lanes>
    <objects>
      <object type="roadMark" subtype="stopLine" id="7" name="line" s="95.0" t="-3.5" zOffset="0.0" orientation="+" length="0.4" width="7.0">
        <validity fromLane="-2" toLane="-1"/>
      </object>
    </objects>
    <signals>
      <signal s="95.0" t="-8.0" id="42" name="light" dynamic="yes" orientation="+" zOffset="5.0" type="1000001" subtype="-1"/>
    </signals>
  </road>
  <road name="exit" length="100.0" id="2" junction="-1">
    <link><predecessor elementType="junction" elementId="10"/></link>
    <planView>
      <geometry s="0.0" x="120.0" y="0.0" hdg="0.0" length="100.0"><line/></geometry>
    </planView>
    <elevationProfile>
      <elevation s="0.0" a="14.0" b="0.2" c="0.0" d="0.0"/>
    </elevationProfile>
    <lanes>
      <laneSection s="0.0">
        <left>
          <lane id="1" type="driving" level="false">
            <width sOffset="0.0" a="3.5" b="0.0" c="0.0" d="0.0"/>
          </lane>
        </left>
        <center><lane id="0" type="none" level="false"/></center>
        <right>
          <lane id="-1" type="driving" level="false">
            <width sOffset="0.0" a="3.5" b="0.0" c="0.0" d="0.0"/>
          </lane>
          <lane id="-2" type="driving" level="false">
            <width sOffset="0.0" a="3.5" b="0.0" c="0.0" d="0.0"/>
          </lane>
        </right>
      </laneSection>
    </lanes>
  </road>
  <road name="" length="20.0" id="3" junction="10">
    <link>
      <predecessor elementType="road" elementId="1" contactPoint="end"/>
      <successor elementType="road" elementId="2" contactPoint="start"/>
    </link>
    <planView>
      <geometry s="0.0" x="100.0" y="0.0" hdg="0.0" length="20.0"><line/></geometry>
    </planView>
    <elevationProfile>
      <elevation s="0.0" a="10.0" b="0.2" c="0.0" d="0.0"/>
    </elevationProfile>
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
  </road>
  <controller id="c1" name="approach"><control signalId="42"/></controller>
  <junction id="10" name="fork">
    <connection id="0" incomingRoad="1" connectingRoad="3" contactPoint="start">
      <laneLink from="-1" to="-1"/>
      <laneLink from="-2" to="-2"/>
    </connection>
    <controller id="c1"/>
  </junction>
</OpenDRIVE>"#;

fn lane_of(map: &ValidatedMap, road: &str, side: LateralSide, ordinal: usize) -> LaneId {
    map.lanes_of(&RoadId::new(road))
        .into_iter()
        .find(|lane| lane.side == side && lane.ordinal == ordinal)
        .unwrap_or_else(|| panic!("{road} has a lane {side:?} {ordinal}"))
        .id
        .clone()
}

fn says(notes: &[String], fragment: &str) -> bool {
    notes.iter().any(|note| note.contains(fragment))
}

#[test]
fn a_foreign_document_is_read_approximated_and_reported() {
    let imported = from_xml(FOREIGN).expect("the document should read");
    let notes = imported.approximations.clone();
    let map = imported
        .map
        .validate()
        .unwrap_or_else(|error| panic!("{error}"));

    // What the document says, the map says.
    assert_eq!(map.roads.len(), 3);
    assert_eq!(map.junctions.len(), 1);
    let approach = map.road(&RoadId::new("1")).unwrap();
    assert_eq!(approach.name.as_deref(), Some("approach"));
    assert_eq!(approach.road_type, RoadType::Rural);
    assert!((approach.speed_limit.unwrap().kph() - 80.0).abs() < 1e-9);
    let connector = map.road(&RoadId::new("3")).unwrap();
    assert!(connector.is_connector());
    assert_eq!(connector.lanes.len(), 2);
    assert_eq!(
        map.junction(&JunctionId::new("10"))
            .unwrap()
            .connecting_roads,
        vec![RoadId::new("3")]
    );

    // The movements: both lanes through the connector, and on into the exit.
    for ordinal in [1, 2] {
        let from = lane_of(&map, "1", LateralSide::Right, ordinal);
        let via = lane_of(&map, "3", LateralSide::Right, ordinal);
        let to = lane_of(&map, "2", LateralSide::Right, ordinal);
        assert_eq!(map.successors(&from), vec![via.clone()]);
        assert_eq!(map.successors(&via), vec![to]);
        assert!(map
            .connections_from(&from)
            .iter()
            .all(|connection| connection.junction == Some(JunctionId::new("10"))));
    }
    let inner = map
        .lane(&lane_of(&map, "1", LateralSide::Right, 1))
        .unwrap();
    assert_eq!(inner.direction, Direction::Forward);
    assert!((inner.speed_limit.unwrap().kph() - 50.0).abs() < 1e-9);
    assert_eq!(inner.right_marking.marking, RoadMarking::Broken);
    assert_eq!(inner.left_marking.marking, RoadMarking::SolidSolid);
    assert_eq!(inner.left_marking.color, MarkingColor::Yellow);
    let opposite = map.lane(&lane_of(&map, "1", LateralSide::Left, 1)).unwrap();
    assert_eq!(opposite.direction, Direction::Backward);
    assert_eq!(opposite.right_marking.marking, RoadMarking::SolidSolid);

    // The elevation: a parabola, read as straight grades that meet it at the ends.
    assert!((approach.reference_line.end_point().z - 10.0).abs() < 1e-9);
    assert!((connector.reference_line.end_point().z - 14.0).abs() < 1e-9);

    // The lane that tapers to nothing is held at the floor, not taken through it.
    let stop = map
        .lane(&lane_of(&map, "1", LateralSide::Right, 3))
        .unwrap();
    assert_eq!(stop.lane_type, LaneType::Shoulder);
    assert!(stop.width_at(100.0) > 0.0 && stop.width_at(100.0) < 0.02);
    assert!((stop.width_at(0.0) - 2.5).abs() < 1e-9);

    // The furniture: a light over the forward lanes, a stop line across the two the
    // document names, and a rule that ties them together.
    assert_eq!(map.objects.len(), 2);
    let light = map
        .objects
        .iter()
        .find(|object| object.kind == MapObjectKind::TrafficLight)
        .unwrap();
    assert_eq!(
        light.lanes.len(),
        3,
        "every forward lane, since none was named"
    );
    assert!(matches!(light.geometry, ObjectGeometry::Point(_)));
    let stop_line = map
        .objects
        .iter()
        .find(|object| object.kind == MapObjectKind::StopLine)
        .unwrap();
    assert_eq!(stop_line.lanes.len(), 2);
    assert_eq!(map.rules.len(), 1);
    let TrafficRule::TrafficLight {
        lights,
        stop_line: across,
        ..
    } = &map.rules[0]
    else {
        panic!("a traffic light rule");
    };
    assert_eq!(lights, &vec![light.id.clone()]);
    assert_eq!(across.as_ref(), Some(&stop_line.id));

    // What could not be kept is said.
    assert!(says(&notes, "no `<geoReference>`"), "{notes:#?}");
    assert!(says(&notes, "free cubic"), "{notes:#?}");
    assert!(says(&notes, "reaches zero"), "{notes:#?}");
    assert!(says(&notes, "eight lane types"), "{notes:#?}");
    assert!(says(&notes, "curved elevation"), "{notes:#?}");
    assert!(says(&notes, "name no lanes"), "{notes:#?}");

    // And the map exports again, with both lanes of the connector linked.
    assert!(check(&map).is_empty(), "{:?}", check(&map));
    let again = to_xml(&map).unwrap();
    let document = opendrive::core::OpenDrive::from_xml_str(&again).unwrap();
    let junction = &document.junction[0];
    assert_eq!(junction.connection.len(), 1);
    assert_eq!(junction.connection[0].lane_link.len(), 2);
    let reread = from_xml(&again).unwrap().map.validate().unwrap();
    assert_eq!(reread.connections.len(), map.connections.len());
}

#[test]
fn a_gap_in_the_reference_line_is_an_error() {
    let broken = FOREIGN.replace(
        r#"<geometry s="0.0" x="120.0" y="0.0" hdg="0.0" length="100.0"><line/></geometry>"#,
        r#"<geometry s="0.0" x="120.0" y="0.0" hdg="0.0" length="50.0"><line/></geometry>
      <geometry s="50.0" x="171.0" y="0.0" hdg="0.0" length="50.0"><line/></geometry>"#,
    );
    let error = from_xml(&broken).expect_err("a metre-wide gap is not a road");
    assert!(error.to_string().contains("starts 1.000 m from"), "{error}");
}
