//! Names SUMO will not take as ids.
//!
//! A road, a junction or the map itself is named by the caller, and an ordinary name
//! — "Smith's Lane", "A; B" — can hold a character SUMO refuses in an id. netconvert
//! stops at the first such id with "Invalid edge id", so the export has to have
//! replaced every one of them before the files are written. These tests build a map
//! out of nothing but such names, hand it to netconvert and load the result in the
//! simulator: the check is SUMO's own, not the exporter's opinion of what SUMO wants.
//!
//! When SUMO is not installed these skip, saying so; CI sets `ROADGEN_REQUIRE_SUMO`,
//! which turns the skip into a failure.

use roadgen_core::prelude::*;
use roadgen_integration_tests::scenarios::{metadata, two_way};
use roadgen_integration_tests::sumo_build;

/// Every character netconvert 1.26 rejects in a node or an edge id.
const REFUSED: [char; 9] = [';', ',', '|', '\'', '"', '&', '<', '>', '\\'];

/// A map whose every name holds something SUMO refuses: two roads joined end to
/// start (so a node is named after both), three arms meeting at a junction, and a
/// road that ends in nothing.
fn awkwardly_named_map() -> ValidatedMap {
    let mut builder = MapBuilder::new(metadata("Main St; \"phase\" 2"));

    let first = builder
        .add_road(
            RoadSpec::line(
                Point3::new(-200.0, 0.0, 0.0),
                Point3::new(-100.0, 0.0, 0.0),
                two_way(),
            )
            .unwrap()
            .with_name("A; B"),
        )
        .unwrap();
    let second = builder
        .add_road(
            RoadSpec::line(
                Point3::new(-100.0, 0.0, 0.0),
                Point3::new(-14.0, 0.0, 0.0),
                two_way(),
            )
            .unwrap()
            .with_name("Smith's Lane"),
        )
        .unwrap();
    builder.connect(&first, &second).unwrap();

    let north = builder
        .add_road(
            RoadSpec::line(
                Point3::new(0.0, 100.0, 0.0),
                Point3::new(0.0, 14.0, 0.0),
                two_way(),
            )
            .unwrap()
            .with_name("x|y,z"),
        )
        .unwrap();
    let east = builder
        .add_road(
            RoadSpec::line(
                Point3::new(100.0, 0.0, 0.0),
                Point3::new(14.0, 0.0, 0.0),
                two_way(),
            )
            .unwrap()
            .with_name(r#"<Q> & "R" \ S"#),
        )
        .unwrap();

    let junction = builder.add_junction(Some("Fish & Chips <corner>"));
    let arms = [&second, &north, &east];
    for (index, from) in arms.iter().enumerate() {
        for to in arms.iter().skip(index + 1) {
            builder
                .connect_ends(from, RoadEnd::End, to, RoadEnd::End, Some(&junction))
                .unwrap();
        }
    }

    builder
        .finish()
        .expect("the map should build")
        .validate()
        .expect("the map should validate")
}

/// netconvert builds the network and the simulator loads it, however the roads were
/// named.
#[test]
fn names_sumo_refuses_still_build_with_netconvert_and_load_in_sumo() {
    if !sumo_build::sumo_available() {
        return;
    }
    let map = awkwardly_named_map();
    let prefix = roadgen_sumo::network_name(&map);
    let (directory, network) = sumo_build::build(&map);
    sumo_build::simulate(directory.path(), &prefix);

    // Every road arrived, each way, under an id that is its name with the refused
    // characters replaced.
    for (name, expected) in [
        ("A; B", "A__B"),
        ("Smith's Lane", "Smith_s_Lane"),
        ("x|y,z", "x_y_z"),
        (r#"<Q> & "R" \ S"#, "_Q_____R____S"),
    ] {
        for sense in ["fwd", "bwd"] {
            let id = format!("{expected}.{sense}");
            assert_eq!(network.edge(&id).id, id, "{name} should be written as {id}");
        }
    }
}

/// Nothing the export writes — the file prefix, node and edge ids, the ends of a
/// connection — keeps a character SUMO would refuse.
#[test]
fn no_written_id_holds_a_character_sumo_refuses() {
    let map = awkwardly_named_map();
    let prefix = roadgen_sumo::network_name(&map);
    assert!(
        !prefix.contains(REFUSED) && !prefix.contains(char::is_whitespace),
        "the prefix {prefix:?} holds a character SUMO refuses"
    );

    let network = roadgen_sumo::to_plain_xml(&map).expect("the map should export as SUMO");
    for (file, contents) in network.files() {
        if file.ends_with(".netccfg") {
            continue;
        }
        let mut reader = quick_xml::Reader::from_str(contents);
        loop {
            match reader.read_event().expect("the export should be XML") {
                quick_xml::events::Event::Eof => break,
                quick_xml::events::Event::Start(element)
                | quick_xml::events::Event::Empty(element) => {
                    let tag = String::from_utf8_lossy(element.name().as_ref()).into_owned();
                    for attribute in element.attributes() {
                        let attribute = attribute.expect("a well-formed attribute");
                        let key = String::from_utf8_lossy(attribute.key.as_ref()).into_owned();
                        if !matches!(key.as_str(), "id" | "from" | "to") {
                            continue;
                        }
                        let value = attribute
                            .normalized_value(quick_xml::XmlVersion::Implicit1_0)
                            .expect("an attribute value")
                            .into_owned();
                        assert!(
                            !value.contains(REFUSED) && !value.contains(char::is_whitespace),
                            "{file}: <{tag} {key}={value:?}> holds a character SUMO refuses"
                        );
                    }
                }
                _ => {}
            }
        }
    }
}
