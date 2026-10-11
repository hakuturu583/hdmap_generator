//! Bridges, tunnels and motorways in OSM: ways cut where a structure begins and
//! ends, tagged for it with a `layer` counted from the ground up, a deck outline of
//! its own, and `highway` values chosen by the road's form of way.

use ll2_io::osm::{Document, Way};
use roadgen_core::buildings::Footprint;
use roadgen_core::id::LandCoverId;
use roadgen_core::land_cover::LandCover;
use roadgen_core::prelude::*;
use roadgen_core::structure::{BridgeForm, BridgeSpec, RoadSpan, TunnelForm, TunnelSpec};
use roadgen_core::{FormOfWay, IrRef, Relation, Trace};

fn two_way() -> Vec<LaneSpec> {
    vec![
        LaneSpec::new(PositiveWidth::new(3.5).unwrap(), Direction::Forward),
        LaneSpec::new(PositiveWidth::new(3.5).unwrap(), Direction::Backward),
    ]
}

fn line(name: &str, from: (f64, f64, f64), to: (f64, f64, f64)) -> RoadSpec {
    RoadSpec::line(
        Point3::new(from.0, from.1, from.2),
        Point3::new(to.0, to.1, to.2),
        two_way(),
    )
    .unwrap()
    .with_name(name)
}

/// A motorway on a viaduct over a street, a road on a bridge over the viaduct, a
/// road through an underpass, and a pond.
fn levels() -> ValidatedMap {
    let mut builder = MapBuilder::default();
    let motorway = builder
        .add_road(
            line("motorway", (-300.0, 0.0, 9.0), (300.0, 0.0, 9.0)).with_form(FormOfWay::Motorway),
        )
        .unwrap();
    builder
        .add_road(line("street", (0.0, -200.0, 0.0), (0.0, 200.0, 0.0)))
        .unwrap();
    let upper = builder
        .add_road(line("upper", (50.0, -200.0, 20.0), (50.0, 200.0, 20.0)))
        .unwrap();
    let through = builder
        .add_road(line("through", (0.0, 500.0, 0.0), (400.0, 500.0, 0.0)))
        .unwrap();
    builder
        .add_bridge(
            vec![RoadSpan::new(motorway, 200.0, 400.0)],
            BridgeSpec {
                name: Some("Street Viaduct".into()),
                form: BridgeForm::Viaduct,
                ..BridgeSpec::default()
            },
        )
        .unwrap();
    builder
        .add_bridge(
            vec![RoadSpan::new(upper, 150.0, 250.0)],
            BridgeSpec::default(),
        )
        .unwrap();
    builder
        .add_tunnel(
            vec![RoadSpan::new(through, 150.0, 250.0)],
            TunnelSpec {
                form: TunnelForm::Underpass,
                ..TunnelSpec::default()
            },
        )
        .unwrap();
    let mut map = builder.finish().unwrap();
    let pond = LandCoverId::new("pond");
    map.as_map_mut()
        .land_covers
        .insert(
            pond.clone(),
            LandCover {
                id: pond,
                site: None,
                kind: "water".into(),
                area: Footprint::new([
                    Point3::new(-200.0, -300.0, 0.0),
                    Point3::new(-150.0, -300.0, 0.0),
                    Point3::new(-150.0, -250.0, 0.0),
                    Point3::new(-200.0, -250.0, 0.0),
                ])
                .unwrap(),
                draped_on: None,
            },
        )
        .unwrap();
    map.validate().unwrap()
}

fn export(map: &ValidatedMap) -> Document {
    export_traced(map).0
}

fn export_traced(map: &ValidatedMap) -> (Document, Trace) {
    let (xml, trace) = roadgen_osm::to_xml_traced(map).unwrap();
    let (document, errors) = ll2_io::osm::parse(&xml).unwrap();
    assert!(errors.is_empty(), "{errors:?}");
    (document, trace)
}

/// How the trace names a way of the file.
fn way_ref(way: &Way) -> String {
    format!("way:{}", way.id)
}

/// The ways named `name`, in the order the road runs.
fn ways_of<'d>(document: &'d Document, name: &str) -> Vec<&'d Way> {
    let mut ways: Vec<&Way> = document
        .ways
        .values()
        .filter(|way| way.tags.get("name").map(String::as_str) == Some(name))
        .filter(|way| way.tags.contains_key("highway"))
        .collect();
    // Ids count down in the order the pieces were written.
    ways.sort_by_key(|way| std::cmp::Reverse(way.id));
    ways
}

fn tag<'w>(way: &'w Way, key: &str) -> Option<&'w str> {
    way.tags.get(key).map(String::as_str)
}

#[test]
fn a_road_is_cut_where_its_bridge_begins_and_ends() {
    let document = export(&levels());
    let motorway = ways_of(&document, "motorway");
    assert_eq!(motorway.len(), 3, "before, on and after the viaduct");
    // The pieces share the node at each cut, so the road is still one line.
    for pair in motorway.windows(2) {
        assert_eq!(pair[0].nodes.last(), pair[1].nodes.first());
    }
    assert_eq!(tag(motorway[0], "bridge"), None);
    assert_eq!(tag(motorway[1], "bridge"), Some("viaduct"));
    assert_eq!(tag(motorway[1], "bridge:name"), Some("Street Viaduct"));
    assert_eq!(tag(motorway[1], "layer"), Some("1"));
    assert_eq!(tag(motorway[2], "layer"), None);
    for way in &motorway {
        assert_eq!(tag(way, "highway"), Some("motorway"), "by its form of way");
    }

    let street = ways_of(&document, "street");
    assert_eq!(street.len(), 1);
    assert_eq!(tag(street[0], "layer"), None, "the ground is layer 0");
}

#[test]
fn a_bridge_over_a_bridge_is_a_level_higher_and_a_tunnel_is_below_ground() {
    let document = export(&levels());
    let upper = ways_of(&document, "upper");
    assert_eq!(upper.len(), 3);
    assert_eq!(tag(upper[1], "bridge"), Some("yes"));
    assert_eq!(tag(upper[1], "layer"), Some("2"));

    let through = ways_of(&document, "through");
    assert_eq!(through.len(), 3);
    assert_eq!(tag(through[1], "tunnel"), Some("yes"), "an underpass too");
    assert_eq!(tag(through[1], "layer"), Some("-1"));
}

#[test]
fn a_deck_is_an_outline_and_water_is_natural() {
    let document = export(&levels());
    let decks: Vec<&Way> = document
        .ways
        .values()
        .filter(|way| tag(way, "man_made") == Some("bridge"))
        .collect();
    assert_eq!(decks.len(), 2);
    for deck in &decks {
        assert_eq!(deck.nodes.first(), deck.nodes.last(), "a closed way");
    }
    assert!(
        decks
            .iter()
            .any(|deck| tag(deck, "name") == Some("Street Viaduct")
                && tag(deck, "layer") == Some("1"))
    );
    assert!(document
        .ways
        .values()
        .any(|way| tag(way, "natural") == Some("water")));
}

#[test]
fn check_says_what_a_structure_loses() {
    let notes = roadgen_osm::check(&levels());
    assert!(notes.iter().any(|note| note.contains("piers")), "{notes:?}");
}

/// A motorway with a slip road leaving it at an exit of the Kita interchange, and a
/// slip road off on its own.
fn kita_exit() -> ValidatedMap {
    let mut builder = MapBuilder::default();
    let main = builder
        .add_road(line("main", (0.0, 0.0, 0.0), (300.0, 0.0, 0.0)).with_form(FormOfWay::Motorway))
        .unwrap();
    let ramp = builder
        .add_road(line("ramp", (340.0, 0.0, 0.0), (500.0, 0.0, 0.0)).with_form(FormOfWay::SlipRoad))
        .unwrap();
    builder
        .add_road(
            line("lone ramp", (0.0, 300.0, 0.0), (200.0, 300.0, 0.0))
                .with_form(FormOfWay::SlipRoad),
        )
        .unwrap();
    let junction = builder.add_junction(Some("exit"));
    builder.connect_via(&junction, &main, &ramp).unwrap();
    builder
        .add_interchange("Kita IC", "diamond", vec![junction], vec![main])
        .unwrap();
    builder.finish().unwrap().validate().unwrap()
}

#[test]
fn a_slip_road_off_a_motorway_is_a_motorway_link() {
    let document = export(&kita_exit());
    assert_eq!(
        tag(ways_of(&document, "ramp")[0], "highway"),
        Some("motorway_link")
    );
    assert_eq!(
        tag(ways_of(&document, "lone ramp")[0], "highway"),
        Some("trunk_link"),
        "a slip road that joins no motorway"
    );
    let exit = document
        .nodes
        .values()
        .find(|node| node.tags.get("highway").map(String::as_str) == Some("motorway_junction"))
        .expect("the node where the ramp leaves");
    assert_eq!(exit.tags.get("name").map(String::as_str), Some("Kita IC"));
}

#[test]
fn a_structure_is_traced_to_the_ways_it_carries_and_its_deck() {
    let map = levels();
    let (document, trace) = export_traced(&map);
    let structure_of = |road: &str| {
        let road = map
            .roads
            .iter()
            .find(|r| r.name.as_deref() == Some(road))
            .unwrap();
        road.structure.spans()[0].structure.clone()
    };

    let viaduct = structure_of("motorway");
    let links: Vec<_> = trace.links_of(&IrRef::Structure(viaduct)).collect();
    assert_eq!(links.len(), 2, "{links:?}");
    let carried = links
        .iter()
        .find(|link| link.role.as_deref() == Some("bridge"))
        .expect("the way on the viaduct");
    assert_eq!(
        carried.relation,
        Relation::Merged,
        "it is the road's way too"
    );
    assert_eq!(carried.local, way_ref(ways_of(&document, "motorway")[1]));
    let deck = links
        .iter()
        .find(|link| link.role.as_deref() == Some("outline"))
        .expect("the deck's outline");
    assert_eq!(deck.relation, Relation::Exact);
    let deck = document
        .ways
        .values()
        .find(|way| way_ref(way) == deck.local)
        .unwrap();
    assert_eq!(tag(deck, "man_made"), Some("bridge"));
    assert_eq!(tag(deck, "name"), Some("Street Viaduct"));

    let underpass = structure_of("through");
    let links: Vec<_> = trace.links_of(&IrRef::Structure(underpass)).collect();
    assert_eq!(links.len(), 1, "a tunnel has no outline: {links:?}");
    assert_eq!(links[0].relation, Relation::Merged);
    assert_eq!(links[0].role.as_deref(), Some("tunnel"));
    assert_eq!(links[0].local, way_ref(ways_of(&document, "through")[1]));
}

#[test]
fn a_grade_separation_is_collapsed_into_the_layer_of_the_bridge_it_raises() {
    let map = levels();
    let (document, trace) = export_traced(&map);
    assert_eq!(
        map.grade_separations.len(),
        2,
        "street and motorway are crossed"
    );
    for separation in map.grade_separations.iter() {
        let over = map.road(&separation.over.road).unwrap();
        let links: Vec<_> = trace
            .links_of(&IrRef::GradeSeparation(separation.id.clone()))
            .collect();
        assert_eq!(links.len(), 1, "{}: {links:?}", separation.id);
        assert_eq!(links[0].relation, Relation::Collapsed);
        assert_eq!(links[0].role.as_deref(), Some("layer"));
        let on_bridge = ways_of(&document, over.name.as_deref().unwrap())[1];
        assert_eq!(links[0].local, way_ref(on_bridge));
        assert!(tag(on_bridge, "layer").is_some());
    }
}

#[test]
fn an_interchange_is_merged_into_the_exit_node_it_names() {
    let map = kita_exit();
    let (document, trace) = export_traced(&map);
    let interchange = map.interchanges.iter().next().unwrap();
    let links: Vec<_> = trace
        .links_of(&IrRef::Interchange(interchange.id.clone()))
        .collect();
    assert_eq!(links.len(), 1, "{links:?}");
    assert_eq!(links[0].relation, Relation::Merged);
    assert_eq!(links[0].role.as_deref(), Some("motorway_junction"));
    let exit = document
        .nodes
        .values()
        .find(|node| format!("node:{}", node.id) == links[0].local)
        .expect("a node of the file");
    assert_eq!(exit.tags.get("name").map(String::as_str), Some("Kita IC"));
}
