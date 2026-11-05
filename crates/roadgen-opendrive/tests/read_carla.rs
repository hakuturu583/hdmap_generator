//! What CARLA 0.10 writes (RoadRunner-authored, OpenDRIVE 1.4), cut down to the
//! parts the reader used to refuse: a `<userData>` with elements in it and no
//! `code`, a `<roadMark>` with no `color`, an outline `<cornerLocal>` with no
//! `height`, an `<object>` of type `-1`; a junction's connecting road that opens
//! with a short section of `none` lanes and a `<laneOffset>` that cancels them up
//! to where the next section starts; and roads and lanes that miss the ones they
//! are linked to by a millimetre. Each is read, the map validates, and what was
//! changed to read it is reported.

use roadgen_core::map::Lane;
use roadgen_core::prelude::*;
use roadgen_opendrive::from_xml;

/// Road 1 runs into road 2, which starts 1.3 mm past road 1's end; road 2 runs
/// into junction 10, whose connecting road 3 starts a millimetre to the side of
/// road 2's end and carries its one lane into road 4. Road 3's first two metres
/// are a section of three lanes, the inner two `none`, under a lane offset of 7 m
/// that puts its driving lane where the next section's is.
const CARLA: &str = r#"<?xml version="1.0" standalone="yes"?>
<OpenDRIVE>
  <header revMajor="1" revMinor="4" name="" version="1" vendor="VectorZero" north="0" south="0" east="0" west="0">
    <userData>
      <vectorScene program="RoadRunner" version="2019.2.12 (build 5161c15)"/>
    </userData>
  </header>
  <road name="Road 1" length="50.0" id="1" junction="-1">
    <link><successor elementType="road" elementId="2" contactPoint="start"/></link>
    <planView>
      <geometry s="0.0" x="0.0" y="0.0" hdg="0.0" length="50.0"><line/></geometry>
    </planView>
    <lanes>
      <laneSection s="0.0">
        <center>
          <lane id="0" type="none" level="false">
            <roadMark sOffset="0" type="none" material="standard" laneChange="none"/>
            <userData/>
          </lane>
        </center>
        <right>
          <lane id="-1" type="driving" level="false">
            <link><successor id="-1"/></link>
            <width sOffset="0.0" a="3.5" b="0.0" c="0.0" d="0.0"/>
            <roadMark sOffset="0" type="curb" material="standard" laneChange="none" width="0.1524"/>
            <userData/>
          </lane>
        </right>
      </laneSection>
    </lanes>
  </road>
  <road name="Road 2" length="50.0" id="2" junction="-1">
    <link>
      <predecessor elementType="road" elementId="1" contactPoint="end"/>
      <successor elementType="junction" elementId="10"/>
    </link>
    <planView>
      <geometry s="0.0" x="50.0013" y="0.0" hdg="0.0" length="50.0"><line/></geometry>
    </planView>
    <lanes>
      <laneSection s="0.0">
        <center><lane id="0" type="none" level="false"/></center>
        <right>
          <lane id="-1" type="driving" level="false">
            <link><predecessor id="-1"/></link>
            <width sOffset="0.0" a="3.5" b="0.0" c="0.0" d="0.0"/>
          </lane>
        </right>
      </laneSection>
    </lanes>
  </road>
  <road name="Road 3" length="20.0" id="3" junction="10">
    <link>
      <predecessor elementType="road" elementId="2" contactPoint="end"/>
      <successor elementType="road" elementId="4" contactPoint="start"/>
    </link>
    <planView>
      <geometry s="0.0" x="100.0" y="0.001" hdg="0.0" length="20.0"><line/></geometry>
    </planView>
    <lanes>
      <laneOffset s="0.0" a="7.0" b="0.0" c="0.0" d="0.0"/>
      <laneOffset s="2.0" a="0.0" b="0.0" c="0.0" d="0.0"/>
      <laneSection s="0.0">
        <center><lane id="0" type="none" level="false"/></center>
        <right>
          <lane id="-1" type="none" level="false">
            <width sOffset="0.0" a="3.5" b="0.0" c="0.0" d="0.0"/>
          </lane>
          <lane id="-2" type="none" level="false">
            <width sOffset="0.0" a="3.5" b="0.0" c="0.0" d="0.0"/>
          </lane>
          <lane id="-3" type="driving" level="false">
            <link><predecessor id="-1"/><successor id="-1"/></link>
            <width sOffset="0.0" a="3.5" b="0.0" c="0.0" d="0.0"/>
          </lane>
        </right>
      </laneSection>
      <laneSection s="2.0">
        <center><lane id="0" type="none" level="false"/></center>
        <right>
          <lane id="-1" type="driving" level="false">
            <link><predecessor id="-3"/><successor id="-1"/></link>
            <width sOffset="0.0" a="3.5" b="0.0" c="0.0" d="0.0"/>
          </lane>
        </right>
      </laneSection>
    </lanes>
  </road>
  <road name="Road 4" length="50.0" id="4" junction="-1">
    <link><predecessor elementType="junction" elementId="10"/></link>
    <planView>
      <geometry s="0.0" x="120.0" y="0.001" hdg="0.0" length="50.0"><line/></geometry>
    </planView>
    <lanes>
      <laneSection s="0.0">
        <center><lane id="0" type="none" level="false"/></center>
        <right>
          <lane id="-1" type="driving" level="false">
            <width sOffset="0.0" a="3.5" b="0.0" c="0.0" d="0.0"/>
          </lane>
        </right>
      </laneSection>
    </lanes>
    <objects>
      <object id="5" name="SolidSingleWhite" type="-1" s="10.0" t="-3.5" zOffset="0.0" hdg="0.0" pitch="0.0" roll="0.0" orientation="+" length="10.0" width="0.1">
        <outline>
          <cornerLocal u="0.0" v="-0.05" z="0.0"/>
          <cornerLocal u="10.0" v="-0.05" z="0.0"/>
          <cornerLocal u="10.0" v="0.05" z="0.0"/>
          <cornerLocal u="0.0" v="0.05" z="0.0"/>
        </outline>
        <userData/>
      </object>
    </objects>
  </road>
  <junction id="10" name="">
    <connection id="0" incomingRoad="2" connectingRoad="3" contactPoint="start">
      <laneLink from="-1" to="-3"/>
    </connection>
  </junction>
</OpenDRIVE>"#;

fn says(notes: &[String], fragment: &str) -> bool {
    notes.iter().any(|note| note.contains(fragment))
}

fn only_lane(map: &ValidatedMap, road: &str, section: usize) -> Lane {
    let lanes: Vec<_> = map
        .lanes_of(&RoadId::new(road))
        .into_iter()
        .filter(|lane| lane.section == section && lane.lane_type == LaneType::Driving)
        .collect();
    assert_eq!(lanes.len(), 1, "road {road} section {section}");
    lanes[0].clone()
}

#[test]
fn what_carla_leaves_out_is_filled_in_and_said() {
    let imported = from_xml(CARLA).unwrap_or_else(|error| panic!("{error}"));
    let notes = imported.approximations.clone();
    imported
        .map
        .validate()
        .unwrap_or_else(|error| panic!("{error}"));

    assert!(
        says(&notes, "2 `<roadMark>`s give no `color`"),
        "{notes:#?}"
    );
    assert!(
        says(&notes, "4 outline `<cornerLocal>`s give no `height`"),
        "{notes:#?}"
    );
    assert!(
        says(
            &notes,
            r#"1 `<object>`s have a `type` OpenDRIVE does not define ("-1")"#
        ),
        "{notes:#?}"
    );
    // Ancillary data is passed over whatever is in it, and that loses nothing.
    assert!(!says(&notes, "userData"), "{notes:#?}");
}

#[test]
fn a_lane_offset_that_ends_with_its_section_stays_with_it() {
    let map = from_xml(CARLA)
        .expect("the document should read")
        .map
        .validate()
        .unwrap_or_else(|error| panic!("{error}"));

    // The driving lane of the short first section ends where the next section's
    // begins: on the lane offset of its own section (7 m, against two `none`
    // lanes 7 m wide), not the next's (none at all).
    let first = only_lane(&map, "3", 0);
    let second = only_lane(&map, "3", 1);
    let gap = first
        .centerline
        .end_point()
        .distance_to(second.centerline.start_point());
    assert!(gap < 1e-9, "the sections' driving lanes are {gap} m apart");
    assert!(
        (first.centerline.end_point().y - (0.001 - 1.75)).abs() < 1e-9,
        "{:?}",
        first.centerline.end_point()
    );
    assert_eq!(map.successors(&first.id), vec![second.id.clone()]);
}

#[test]
fn a_millimetre_between_linked_roads_and_lanes_is_closed_and_said() {
    let imported = from_xml(CARLA).expect("the document should read");
    let notes = imported.approximations.clone();
    let map = imported
        .map
        .validate()
        .unwrap_or_else(|error| panic!("{error}"));

    // Road 2 is moved, whole, onto the end of road 1: its other end is in a
    // junction, so nothing else holds it where it was.
    let road_1 = map.road(&RoadId::new("1")).unwrap();
    let road_2 = map.road(&RoadId::new("2")).unwrap();
    assert!(
        road_1
            .reference_line
            .end_point()
            .distance_to(road_2.reference_line.start_point())
            < 1e-9
    );
    assert!(says(&notes, "1 roads end up to 0.0013 m"), "{notes:#?}");

    // And the connecting road's lane, a millimetre to the side of road 2's, is
    // brought onto it, edges and all, so a Lanelet2 export shares the points.
    let incoming = only_lane(&map, "2", 0);
    let connector = only_lane(&map, "3", 0);
    assert_eq!(map.successors(&incoming.id), vec![connector.id.clone()]);
    for (leaving, entering) in [
        (&incoming.left_boundary, &connector.left_boundary),
        (&incoming.right_boundary, &connector.right_boundary),
        (&incoming.centerline, &connector.centerline),
    ] {
        let gap = leaving.end_point().distance_to(entering.start_point());
        assert!(gap < 1e-9, "{gap}");
    }
    assert!(
        says(&notes, "lane connections join lane ends up to"),
        "{notes:#?}"
    );
}

#[test]
fn more_than_a_hairline_is_not_closed() {
    // Twenty centimetres to the side, past the 10 cm the reader closes, is not
    // rounding: the reader leaves it, and validation says the lanes do not meet.
    let apart = CARLA.replace(
        r#"<geometry s="0.0" x="100.0" y="0.001" hdg="0.0" length="20.0"><line/></geometry>"#,
        r#"<geometry s="0.0" x="100.0" y="0.2" hdg="0.0" length="20.0"><line/></geometry>"#,
    );
    let imported = from_xml(&apart).expect("the document should read");
    let error = imported
        .map
        .validate()
        .expect_err("the lanes are 20 cm apart");
    assert!(error.to_string().contains("do not meet"), "{error}");
}
