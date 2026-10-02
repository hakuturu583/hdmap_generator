//! Exporting traffic lights to SUMO.
//!
//! The export decides each signalised junction's program itself and writes it down —
//! the phases as a `<tlLogic>`, and the link index of every movement it controls — so
//! that the state strings mean what the export says they mean, and the trace can say
//! which slot of which program a light of the map controls. That only holds if
//! netconvert keeps what it was given rather than generating a program of its own, so
//! these tests read the built `.net.xml` and compare it with the files that went in:
//! the same programs, the same phases, the same link index on every movement. Then
//! they drive traffic through the junction in the simulator, which is the check that
//! the program is one SUMO will actually run, and that every movement is released.
//!
//! When SUMO is not installed these skip, saying so; CI sets `ROADGEN_REQUIRE_SUMO`,
//! which turns the skip into a failure.

use std::collections::{BTreeMap, BTreeSet, HashMap};
use std::path::Path;
use std::process::Command;

use quick_xml::events::Event;
use quick_xml::Reader;

use roadgen_core::prelude::*;
use roadgen_core::trace::Relation;
use roadgen_integration_tests::scenarios;
use roadgen_integration_tests::sumo_build;
use roadgen_trace::{directory_sidecar, write_ir, write_trace, TraceIndex};

/// How long the simulations run, seconds: long enough for two full cycles of a
/// two-group program, so that every group is released and every movement served.
const STEPS: u32 = 200;

/// A traffic-light program, as a `.tll.xml` or a `.net.xml` states it.
#[derive(Debug, Clone, PartialEq)]
struct Program {
    id: String,
    kind: String,
    program: String,
    offset: f64,
    /// Each phase's duration, seconds, and state.
    phases: Vec<(f64, String)>,
}

/// A movement between two of the export's lanes: from edge, from lane, to edge, to
/// lane.
type Movement = (String, usize, String, usize);

/// What a file says about traffic lights: its programs, by id, and the light and
/// link index of every movement one controls.
#[derive(Debug, Default)]
struct Lights {
    programs: BTreeMap<String, Program>,
    links: BTreeMap<Movement, (String, usize)>,
}

impl Lights {
    fn read(path: &Path) -> Lights {
        let xml = std::fs::read_to_string(path)
            .unwrap_or_else(|error| panic!("{}: {error}", path.display()));
        let mut lights = Lights::default();
        let mut current: Option<Program> = None;
        let mut reader = Reader::from_str(&xml);
        loop {
            let event = reader.read_event().expect("valid XML");
            match &event {
                Event::Eof => break,
                Event::End(end) if end.name().as_ref() == b"tlLogic" => {
                    let program = current.take().expect("a program being read");
                    lights.programs.insert(program.id.clone(), program);
                }
                Event::Start(element) | Event::Empty(element) => {
                    let attributes = attributes(element);
                    match element.name().as_ref() {
                        b"tlLogic" => {
                            let program = Program {
                                id: attributes["id"].clone(),
                                kind: attributes["type"].clone(),
                                program: attributes["programID"].clone(),
                                offset: attributes["offset"].parse().unwrap(),
                                phases: Vec::new(),
                            };
                            if matches!(event, Event::Empty(_)) {
                                lights.programs.insert(program.id.clone(), program);
                            } else {
                                current = Some(program);
                            }
                        }
                        b"phase" => current
                            .as_mut()
                            .expect("a phase in a program")
                            .phases
                            .push((
                                attributes["duration"].parse().unwrap(),
                                attributes["state"].clone(),
                            )),
                        // A connection out of one of netconvert's internal lanes is
                        // the second half of a movement and carries no signal.
                        b"connection" if !attributes["from"].starts_with(':') => {
                            if let (Some(light), Some(index)) =
                                (attributes.get("tl"), attributes.get("linkIndex"))
                            {
                                lights.links.insert(
                                    (
                                        attributes["from"].clone(),
                                        attributes["fromLane"].parse().unwrap(),
                                        attributes["to"].clone(),
                                        attributes["toLane"].parse().unwrap(),
                                    ),
                                    (light.clone(), index.parse().unwrap()),
                                );
                            }
                        }
                        _ => {}
                    }
                }
                _ => {}
            }
        }
        lights
    }

    /// The state of `movement` in each phase of the program controlling it.
    fn states(&self, from: &str, to: &str) -> String {
        let ((_, _, _, _), (light, index)) = self
            .links
            .iter()
            .find(|((source, _, target, _), _)| source == from && target == to)
            .unwrap_or_else(|| panic!("no controlled movement {from} → {to}"));
        self.programs[light]
            .phases
            .iter()
            .map(|(_, state)| state.as_bytes()[*index] as char)
            .collect()
    }
}

fn attributes(element: &quick_xml::events::BytesStart) -> HashMap<String, String> {
    element
        .attributes()
        .map(|attribute| {
            let attribute = attribute.unwrap();
            (
                String::from_utf8_lossy(attribute.key.as_ref()).into_owned(),
                String::from_utf8_lossy(&attribute.value).into_owned(),
            )
        })
        .collect()
}

/// Builds `map` with netconvert and checks that the network it built runs on exactly
/// the programs the export wrote, with every movement into a signalised node on the
/// link index the export gave it. Returns what the export wrote, and the directory.
fn build_and_compare(map: &ValidatedMap) -> (tempfile::TempDir, String, Lights) {
    let directory = tempfile::tempdir().unwrap();
    let prefix = roadgen_sumo::write(map, directory.path()).unwrap();
    let written = Lights::read(&directory.path().join(format!("{prefix}.tll.xml")));
    let net_path = sumo_build::netconvert(directory.path(), &prefix);
    let built = Lights::read(&net_path);

    assert!(!written.programs.is_empty(), "the export wrote no program");
    assert_eq!(
        built.programs, written.programs,
        "netconvert built a different program from the one it was given"
    );
    assert_eq!(
        built.links, written.links,
        "netconvert numbered the links differently from the export"
    );

    // And nothing into a signalised node is left uncontrolled.
    let network = sumo_build::SumoNetwork::read(&net_path);
    for connection in network
        .connections
        .iter()
        .filter(|connection| !connection.from.starts_with(':'))
    {
        let edge = network.edge(&connection.from);
        let Some(node) = &edge.to else { continue };
        if written.programs.contains_key(node) {
            assert_eq!(
                connection.tl.as_deref(),
                Some(node.as_str()),
                "{} → {} enters {node} uncontrolled",
                connection.from,
                connection.to
            );
        }
    }
    (directory, prefix, written)
}

/// Runs the network in the simulator for [`STEPS`] seconds with a steady flow of
/// vehicles along every controlled movement, and checks that SUMO said nothing and
/// that every movement got vehicles through.
fn drive(directory: &Path, prefix: &str, lights: &Lights) {
    let movements: BTreeSet<(&str, &str)> = lights
        .links
        .keys()
        .map(|(from, _, to, _)| (from.as_str(), to.as_str()))
        .collect();
    let mut routes = String::from("<routes>\n");
    for (index, (from, to)) in movements.iter().enumerate() {
        routes.push_str(&format!(
            "    <flow id=\"m{index}\" begin=\"0\" end=\"{}\" period=\"9\" from=\"{from}\" \
             to=\"{to}\"/>\n",
            STEPS / 2
        ));
    }
    routes.push_str("</routes>\n");
    std::fs::write(directory.join("demand.rou.xml"), routes).unwrap();

    let output = Command::new(sumo_build::tool("sumo").expect("sumo"))
        .args(["-n", &format!("{prefix}.net.xml")])
        .args(["-r", "demand.rou.xml"])
        .args(["--end", &STEPS.to_string()])
        .args(["--no-step-log", "true"])
        .args(["--tripinfo-output", "trips.xml"])
        // Collisions inside the junction are what a wrong program causes, and SUMO
        // only looks for them there when asked; the statistics then say whether any
        // happened, and whether a vehicle had to be teleported out of a deadlock.
        .args(["--collision.check-junctions", "true"])
        .args(["--statistic-output", "statistics.xml"])
        .current_dir(directory)
        .output()
        .expect("sumo should run");
    let stderr = String::from_utf8_lossy(&output.stderr).into_owned();
    let complaints: Vec<&str> = stderr
        .lines()
        .filter(|line| !line.trim().is_empty())
        .filter(|line| !sumo_build::is_about_the_machine_or_the_map(line))
        .collect();
    assert!(
        output.status.success(),
        "sumo could not run the network:\n{stderr}"
    );
    assert!(
        complaints.is_empty(),
        "sumo ran the network but complained about it:\n{}",
        complaints.join("\n")
    );

    let statistics = std::fs::read_to_string(directory.join("statistics.xml")).unwrap();
    let mut counted = BTreeMap::new();
    let mut reader = Reader::from_str(&statistics);
    loop {
        match reader.read_event().unwrap() {
            Event::Eof => break,
            Event::Start(element) | Event::Empty(element) => {
                let name = String::from_utf8_lossy(element.name().as_ref()).into_owned();
                let key = match name.as_str() {
                    "safety" => "collisions",
                    "teleports" => "total",
                    _ => continue,
                };
                counted.insert(name, attributes(&element)[key].clone());
            }
            _ => {}
        }
    }
    assert_eq!(
        counted.get("safety").map(String::as_str),
        Some("0"),
        "vehicles collided under the program:\n{statistics}"
    );
    assert_eq!(
        counted.get("teleports").map(String::as_str),
        Some("0"),
        "vehicles had to be teleported under the program:\n{statistics}"
    );

    // A vehicle of flow `m3` is called `m3.<n>`.
    let trips = std::fs::read_to_string(directory.join("trips.xml")).unwrap();
    let mut arrived: BTreeMap<String, usize> = BTreeMap::new();
    let mut reader = Reader::from_str(&trips);
    loop {
        match reader.read_event().unwrap() {
            Event::Eof => break,
            Event::Start(element) | Event::Empty(element)
                if element.name().as_ref() == b"tripinfo" =>
            {
                let id = attributes(&element)["id"].clone();
                let flow = id.split_once('.').map_or(id.as_str(), |(flow, _)| flow);
                *arrived.entry(flow.to_owned()).or_default() += 1;
            }
            _ => {}
        }
    }
    for (index, (from, to)) in movements.iter().enumerate() {
        assert!(
            arrived.get(&format!("m{index}")).copied().unwrap_or(0) > 0,
            "no vehicle got from {from} to {to} in {STEPS} s: the signal never let it go"
        );
    }
}

/// Four arms, each turning into the other three, with a light on every approach and
/// a rule saying which lane each one governs.
fn signalised_crossroads() -> ValidatedMap {
    let mut builder = MapBuilder::new(scenarios::metadata("signalised"));
    let arms: Vec<RoadId> = [
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
        (
            "south",
            Point3::new(0.0, -70.0, 0.0),
            Point3::new(0.0, -14.0, 0.0),
        ),
        (
            "west",
            Point3::new(-70.0, 0.0, 0.0),
            Point3::new(-14.0, 0.0, 0.0),
        ),
    ]
    .into_iter()
    .map(|(name, start, end)| {
        builder
            .add_road(
                RoadSpec::line(start, end, scenarios::two_way())
                    .unwrap()
                    .with_name(name),
            )
            .unwrap()
    })
    .collect();
    let junction = builder.add_junction(Some("x"));
    for (index, from) in arms.iter().enumerate() {
        for to in arms.iter().skip(index + 1) {
            builder
                .connect_ends(from, RoadEnd::End, to, RoadEnd::End, Some(&junction))
                .unwrap();
        }
    }
    for arm in &arms {
        let approach = LaneRef::new(arm.clone(), 0);
        let stop_line = builder.add_stop_line(&approach, LaneEnd::End).unwrap();
        let light = builder
            .add_traffic_light(&approach, LaneEnd::End, 5.0)
            .unwrap();
        builder.add_traffic_light_rule(vec![light], Some(stop_line), vec![approach]);
    }
    builder.finish().unwrap().validate().unwrap()
}

/// The scenario every format is tested with: two arms at right angles, so two groups
/// of one approach each, and a turn that crosses nothing.
#[test]
fn the_controlled_crossroads_runs_on_the_program_the_export_wrote() {
    if !sumo_build::sumo_available() {
        return;
    }
    let (directory, prefix, lights) = build_and_compare(&scenarios::controlled_crossroads());

    let program = &lights.programs["j_x"];
    assert_eq!(program.kind, "static");
    assert_eq!(program.program, "0");
    assert_eq!(program.offset, 0.0);
    let phases: Vec<(f64, &str)> = program
        .phases
        .iter()
        .map(|(duration, state)| (*duration, state.as_str()))
        .collect();
    assert_eq!(
        phases,
        [
            (35.0, "Gr"),
            (3.0, "yr"),
            (2.0, "rr"),
            (35.0, "rG"),
            (3.0, "ry"),
            (2.0, "rr"),
        ]
    );
    assert_eq!(lights.states("north.fwd", "east.bwd"), "Gyrrrr");
    assert_eq!(lights.states("east.fwd", "north.bwd"), "rrrGyr");

    drive(directory.path(), &prefix, &lights);
}

/// A full crossroads: the two approaches of each road run together, straight on is
/// protected, and every turn yields — the left across the oncoming stream, and both
/// turns because each runs into a one-lane exit the opposing approach also turns
/// into.
#[test]
fn a_signalised_crossroads_releases_each_road_in_turn() {
    if !sumo_build::sumo_available() {
        return;
    }
    let (directory, prefix, lights) = build_and_compare(&signalised_crossroads());

    let program = &lights.programs["j_x"];
    let durations: Vec<f64> = program
        .phases
        .iter()
        .map(|(duration, _)| *duration)
        .collect();
    assert_eq!(durations, [35.0, 3.0, 2.0, 35.0, 3.0, 2.0]);
    assert_eq!(lights.links.len(), 12);
    let indices: BTreeSet<usize> = lights.links.values().map(|(_, index)| *index).collect();
    assert_eq!(indices, (0..12).collect());
    assert!(program.phases.iter().all(|(_, state)| state.len() == 12));

    for (from, straight, left, right) in [
        ("north.fwd", "south.bwd", "east.bwd", "west.bwd"),
        ("south.fwd", "north.bwd", "west.bwd", "east.bwd"),
    ] {
        assert_eq!(lights.states(from, straight), "Gyrrrr");
        assert_eq!(lights.states(from, left), "gyrrrr");
        assert_eq!(lights.states(from, right), "gyrrrr");
    }
    for (from, straight) in [("east.fwd", "west.bwd"), ("west.fwd", "east.bwd")] {
        assert_eq!(lights.states(from, straight), "rrrGyr");
    }

    drive(directory.path(), &prefix, &lights);
}

/// A light of the map is traced to the program of its junction and to the slot of
/// each movement off the lane it governs, and both are in the network netconvert
/// built — which the trace index checks for itself when it is handed the network.
#[test]
fn a_light_traces_to_its_program_and_its_slots_in_the_built_network() {
    if !sumo_build::sumo_available() {
        return;
    }
    let map = signalised_crossroads();
    let directory = tempfile::tempdir().unwrap();
    write_ir(&map, directory.path().join("map.ir.json")).unwrap();
    let (prefix, trace) = roadgen_sumo::write_traced(&map, directory.path()).unwrap();
    assert!(trace
        .files
        .iter()
        .any(|file| file.to_string_lossy().ends_with(".tll.xml")));
    let sidecar = directory_sidecar(directory.path(), &prefix, &trace.format);
    write_trace(&trace, &map, &sidecar).unwrap();
    let net = sumo_build::netconvert(directory.path(), &prefix);
    let built = Lights::read(&net);

    let mut index = TraceIndex::new();
    index.load(directory.path().join("map.ir.json")).unwrap();
    index.load(&sidecar).unwrap();
    index.load_sumo_net(&net).unwrap();

    let north = map
        .roads
        .iter()
        .find(|road| road.name.as_deref() == Some("north"))
        .unwrap();
    let light = map
        .objects
        .iter()
        .find(|object| {
            object.kind.is_traffic_light()
                && object
                    .lanes
                    .iter()
                    .any(|lane| map.lane(lane).is_some_and(|lane| lane.road == north.id))
        })
        .unwrap();
    let locals: BTreeSet<String> = index
        .from_ir(light.id.as_str(), "sumo")
        .unwrap()
        .into_iter()
        .map(|link| {
            assert_eq!(link.relation, Relation::Merged, "{}", link.local);
            link.local.clone()
        })
        .collect();

    // The program, and the slot of each of the three movements off the northern
    // approach as the built network numbers them.
    let mut expected: BTreeSet<String> = built
        .links
        .iter()
        .filter(|((from, ..), _)| from == "north.fwd")
        .map(|(_, (light, index))| format!("tls:{light}/{index}"))
        .collect();
    assert_eq!(expected.len(), 3);
    expected.insert("tls:j_x".to_owned());
    assert_eq!(locals, expected);

    // And back: a slot is the light that governs it.
    let slot = expected.iter().find(|local| local.contains('/')).unwrap();
    let lights: Vec<String> = index
        .to_ir("sumo", slot)
        .unwrap()
        .into_iter()
        .map(|link| link.ir.clone())
        .collect();
    assert_eq!(lights, [light.id.as_str().to_owned()]);
}

/// A network whose program numbers its links differently from the export is not the
/// network the trace describes, and the index refuses it.
#[test]
fn a_network_numbering_the_links_otherwise_is_refused() {
    if !sumo_build::sumo_available() {
        return;
    }
    let map = signalised_crossroads();
    let directory = tempfile::tempdir().unwrap();
    write_ir(&map, directory.path().join("map.ir.json")).unwrap();
    let (prefix, trace) = roadgen_sumo::write_traced(&map, directory.path()).unwrap();
    let sidecar = directory_sidecar(directory.path(), &prefix, &trace.format);
    write_trace(&trace, &map, &sidecar).unwrap();

    // The same network with netconvert's own program in place of the export's.
    let config = directory.path().join(format!("{prefix}.netccfg"));
    let text = std::fs::read_to_string(&config).unwrap();
    let without = text
        .lines()
        .filter(|line| !line.contains("tllogic-files"))
        .collect::<Vec<_>>()
        .join("\n");
    std::fs::write(&config, without).unwrap();
    let net = sumo_build::netconvert(directory.path(), &prefix);
    // The light is still called what the node says, and has as many links: only the
    // numbering and the phases are netconvert's.
    let built = Lights::read(&net);
    assert!(built.programs.contains_key("j_x"));
    assert_eq!(built.links.len(), 12);

    // The configuration was edited after the trace recorded it, which is the point.
    let mut index = TraceIndex::new().check_files(false);
    index.load(directory.path().join("map.ir.json")).unwrap();
    index.load(&sidecar).unwrap();
    let error = index.load_sumo_net(&net).unwrap_err();
    let refused = match &error {
        roadgen_trace::TraceError::Foreign { reason, .. } => {
            reason.contains("is not controlled as link")
        }
        _ => false,
    };
    assert!(refused, "{error}");
}
