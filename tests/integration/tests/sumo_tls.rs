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
    /// The controlled connections between netconvert's internal lanes, which are the
    /// way onto a built network's pedestrian crossings: by the internal edge they
    /// lead to, its light and link index. A `.tll.xml` the export wrote has none.
    internal: BTreeMap<String, (String, usize)>,
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
                        // the second half of a movement and carries no signal —
                        // except the one from a walking area onto a pedestrian
                        // crossing, which a light at a node with crossings controls.
                        b"connection" if attributes["from"].starts_with(':') => {
                            if let (Some(light), Some(index)) =
                                (attributes.get("tl"), attributes.get("linkIndex"))
                            {
                                lights.internal.insert(
                                    attributes["to"].clone(),
                                    (light.clone(), index.parse().unwrap()),
                                );
                            }
                        }
                        b"connection" => {
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
    build_and_compare_warned(map, &[])
}

/// [`build_and_compare`], for a map netconvert is right to warn about: it must say
/// exactly `expected`, one line each, and nothing else.
fn build_and_compare_warned(
    map: &ValidatedMap,
    expected: &[&str],
) -> (tempfile::TempDir, String, Lights) {
    let directory = tempfile::tempdir().unwrap();
    let prefix = roadgen_sumo::write(map, directory.path()).unwrap();
    let written = Lights::read(&directory.path().join(format!("{prefix}.tll.xml")));
    let (net_path, warnings) = sumo_build::netconvert_with_warnings(directory.path(), &prefix);
    assert_eq!(warnings, expected, "netconvert said something unexpected");
    let built = Lights::read(&net_path);

    assert!(!written.programs.is_empty(), "the export wrote no program");
    let plain =
        std::fs::read_to_string(directory.path().join(format!("{prefix}.con.xml"))).unwrap();
    if plain.contains("<crossing ") {
        // At a node with pedestrian crossings netconvert extends the program it was
        // given: the crossings' links go after the vehicle links, and each green is
        // split to end in a pedestrian clearance. What must hold is that the vehicle
        // links are the export's, on the same indices, each going through the same
        // signals in the same order for the same time.
        compare_vehicle_links(&written, &built);
    } else {
        assert_eq!(
            built.programs, written.programs,
            "netconvert built a different program from the one it was given"
        );
    }
    assert_eq!(
        built.links, written.links,
        "netconvert numbered the links differently from the export"
    );

    // And nothing into a signalised node is left uncontrolled. A pavement's
    // connection into a walking area (`:j_x_w0`) is not a movement across the
    // junction, and is uncontrolled in any network: pedestrians meet the light at the
    // crossing.
    let network = sumo_build::SumoNetwork::read(&net_path);
    for connection in network
        .connections
        .iter()
        .filter(|connection| !connection.from.starts_with(':') && !connection.to.starts_with(':'))
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

/// Checks that each program `built` holds the vehicle links of the one `written`:
/// the same light, type, programme id and offset; every state starting with as many
/// signals as the export wrote, and longer only by netconvert's crossing links; and
/// each vehicle link going through the same signals — green, yellow, red — in the
/// same order and for the same time, however netconvert split the phases.
fn compare_vehicle_links(written: &Lights, built: &Lights) {
    assert_eq!(
        built.programs.keys().collect::<Vec<_>>(),
        written.programs.keys().collect::<Vec<_>>(),
        "netconvert built different lights from the ones it was given"
    );
    for (id, program) in &written.programs {
        let other = &built.programs[id];
        assert_eq!(
            (&other.kind, &other.program, other.offset),
            (&program.kind, &program.program, program.offset),
            "{id}: netconvert changed the program's type, id or offset"
        );
        let vehicles = program.phases[0].1.len();
        assert!(
            other
                .phases
                .iter()
                .all(|(_, state)| state.len() >= vehicles),
            "{id}: netconvert dropped vehicle links: {other:?}"
        );
        // Each link's signal over the cycle, a phase at a time, with a phase netconvert
        // split in two read as one: consecutive phases showing the link the same signal
        // are merged, adding their durations.
        let signals = |program: &Program, index: usize| {
            let mut merged: Vec<(char, f64)> = Vec::new();
            for (duration, state) in &program.phases {
                let signal = state.as_bytes()[index] as char;
                match merged.last_mut() {
                    Some((last, total)) if *last == signal => *total += duration,
                    _ => merged.push((signal, *duration)),
                }
            }
            merged
        };
        for index in 0..vehicles {
            assert_eq!(
                signals(other, index),
                signals(program, index),
                "{id}: vehicle link {index} does not run as the export wrote it"
            );
        }
    }
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

/// The crossroads of [`signalised_crossroads`], lit and ruled on every arm, but with
/// no connector out of the northern approach: the IR states no movement onward from
/// `north.fwd`, while every other approach turns into the other three arms,
/// `north.bwd` among them.
fn crossroads_with_a_dead_approach() -> ValidatedMap {
    let mut builder = MapBuilder::new(scenarios::metadata("dead_approach"));
    let arms: Vec<RoadId> = [
        ("north", (0.0, 70.0), (0.0, 14.0)),
        ("east", (70.0, 0.0), (14.0, 0.0)),
        ("south", (0.0, -70.0), (0.0, -14.0)),
        ("west", (-70.0, 0.0), (-14.0, 0.0)),
    ]
    .into_iter()
    .map(|(name, (x0, y0), (x1, y1))| {
        builder
            .add_road(
                RoadSpec::line(
                    Point3::new(x0, y0, 0.0),
                    Point3::new(x1, y1, 0.0),
                    scenarios::two_way(),
                )
                .unwrap()
                .with_name(name),
            )
            .unwrap()
    })
    .collect();
    let junction = builder.add_junction(Some("x"));
    // Lane 0 of each arm runs into the junction, lane 1 out of it.
    for from in arms.iter().skip(1) {
        for to in arms.iter().filter(|to| *to != from) {
            builder
                .connect_lanes(
                    &LaneRef::new(from.clone(), 0),
                    &LaneRef::new(to.clone(), 1),
                    Some(&junction),
                )
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

/// An approach into a signalised junction that the IR carries nowhere is not left
/// for netconvert to fill in. Were it, netconvert would guess movements off it that
/// no program controls — state `m`, no `tl` — and vehicles on it would drive through
/// the junction ignoring the signal. The export deletes every movement off it
/// instead, so the network has none, every movement that is there is controlled, and
/// the rest of the junction runs on the program without incident.
#[test]
fn an_approach_with_nowhere_to_go_enters_the_signal_not_uncontrolled() {
    if !sumo_build::sumo_available() {
        return;
    }
    let map = crossroads_with_a_dead_approach();
    // netconvert says the edge goes nowhere, which is what the IR says.
    let (directory, prefix, lights) = build_and_compare_warned(
        &map,
        &["Warning: Edge 'north.fwd' is not connected to outgoing edges at junction 'j_x'."],
    );

    let net = directory.path().join(format!("{prefix}.net.xml"));
    let network = sumo_build::SumoNetwork::read(&net);
    let guessed: Vec<String> = network
        .connections
        .iter()
        .filter(|connection| connection.from == "north.fwd")
        .map(|connection| format!("{} → {}", connection.from, connection.to))
        .collect();
    assert!(
        guessed.is_empty(),
        "netconvert guessed movements the IR does not state: {guessed:?}"
    );

    // The other three approaches each turn into the other three arms.
    assert_eq!(lights.links.len(), 9);
    assert!(lights.links.keys().all(|(from, ..)| from != "north.fwd"));
    assert!(lights.links.keys().any(|(_, _, to, _)| to == "north.bwd"));
    let indices: BTreeSet<usize> = lights.links.values().map(|(_, index)| *index).collect();
    assert_eq!(indices, (0..9).collect());

    drive(directory.path(), &prefix, &lights);
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

/// A signalised crossroads with a pedestrian crossing across every arm, in a
/// right-hand and a left-hand version. netconvert takes the export's program and
/// extends it with the crossings: it must say nothing about it, keep every vehicle
/// link on the index the export gave it and running as the export wrote it, and put
/// each crossing on the same light, after the vehicle links. Then cars on every
/// movement get through in the simulator without a collision or a teleport, and a
/// pedestrian crosses every arm both ways on its crossing.
#[test]
fn a_signalised_crossroads_with_crossings_keeps_the_vehicle_links() {
    if !sumo_build::sumo_available() {
        return;
    }
    for handedness in [TrafficHandedness::RightHand, TrafficHandedness::LeftHand] {
        let (map, _) = scenarios::signalised_crosswalk_crossroads(handedness);
        let (directory, prefix, lights) = build_and_compare(&map);
        let net = directory.path().join(format!("{prefix}.net.xml"));
        let built = Lights::read(&net);

        // Each of the four arms turns into the other three.
        assert_eq!(lights.links.len(), 12, "{handedness:?}");
        let program = &lights.programs["j_x"];
        assert!(program.phases.iter().all(|(_, state)| state.len() == 12));

        // Four crossings, each controlled by the junction's light, on the indices
        // after the vehicle links.
        let network = sumo_build::SumoNetwork::read(&net);
        let crossings: BTreeSet<&str> = network
            .edges
            .iter()
            .filter(|edge| edge.function.as_deref() == Some("crossing"))
            .map(|edge| edge.id.as_str())
            .collect();
        assert_eq!(crossings.len(), 4, "{handedness:?}: {crossings:?}");
        let crossing_slots: BTreeSet<usize> = crossings
            .iter()
            .map(|crossing| {
                let (light, index) = built
                    .internal
                    .get(*crossing)
                    .unwrap_or_else(|| panic!("{handedness:?}: {crossing} is not controlled"));
                assert_eq!(light, "j_x", "{handedness:?}: {crossing}");
                *index
            })
            .collect();
        assert_eq!(
            crossing_slots,
            (12..16).collect(),
            "{handedness:?}: the crossings are not after the vehicle links"
        );
        let built_program = &built.programs["j_x"];
        assert!(built_program
            .phases
            .iter()
            .all(|(_, state)| state.len() == 16));
        // And each crossing gets a green of its own at some point in the cycle.
        for &crossing in &crossing_slots {
            assert!(
                built_program
                    .phases
                    .iter()
                    .any(|(_, state)| state.as_bytes()[crossing] == b'G'),
                "{handedness:?}: crossing link {crossing} is never green: {built_program:?}"
            );
        }

        // Cars on every movement get through under the program, without a
        // collision or a teleport.
        drive(directory.path(), &prefix, &lights);

        // And a pedestrian crosses every arm at the junction, both ways, on the
        // crossing and under the light: from a metre short of the junction on one
        // pavement to a metre short of it on the other, a walk of a few metres.
        let near_junction = |edge: &str| {
            let edge = network.edge(edge);
            let pavement = edge
                .lanes
                .iter()
                .find(|lane| lane.allow.as_deref() == Some("pedestrian"))
                .unwrap_or_else(|| panic!("{} has no pavement", edge.id));
            let arriving = edge.to.as_deref() == Some("j_x");
            let position = if arriving { pavement.length - 1.0 } else { 1.0 };
            (edge.id.clone(), position)
        };
        let mut walks = Vec::new();
        let mut persons = String::new();
        for arm in ["west", "east", "north", "south"] {
            let (fwd, bwd) = (format!("{arm}.fwd"), format!("{arm}.bwd"));
            for (from, to) in [(&fwd, &bwd), (&bwd, &fwd)] {
                let (from_edge, depart) = near_junction(from);
                let (to_edge, arrive) = near_junction(to);
                persons.push_str(&format!(
                    "    <person id=\"p{}\" depart=\"0\" departPos=\"{depart:.2}\">\n        \
                     <walk from=\"{from_edge}\" to=\"{to_edge}\" arrivalPos=\"{arrive:.2}\"/>\n    \
                     </person>\n",
                    walks.len()
                ));
                walks.push((from.clone(), to.clone()));
            }
        }
        std::fs::write(
            directory.path().join("walks.rou.xml"),
            format!("<routes>\n{persons}</routes>\n"),
        )
        .unwrap();
        let walked = sumo_build::walk(directory.path(), &prefix);
        assert_eq!(
            walked.len(),
            walks.len(),
            "{handedness:?}: every pedestrian should arrive: {walked:?}"
        );
        for (person, length) in &walked {
            let (from, to) = &walks[person.trim_start_matches('p').parse::<usize>().unwrap()];
            assert!(
                *length < 25.0,
                "{handedness:?}: {from} to {to} took {length} m, so it did not use the \
                 crossing"
            );
        }
    }
}

/// The trace index takes the network netconvert built for a signalised junction with
/// crossings: the vehicle slots it checks are the export's, and the crossing slots
/// netconvert appended after them are its own, which the trace neither names nor
/// refuses the network for. Each light still traces to the vehicle slots only.
#[test]
fn the_trace_index_takes_a_signalised_network_with_crossings() {
    if !sumo_build::sumo_available() {
        return;
    }
    for handedness in [TrafficHandedness::RightHand, TrafficHandedness::LeftHand] {
        let (map, _) = scenarios::signalised_crosswalk_crossroads(handedness);
        let directory = tempfile::tempdir().unwrap();
        write_ir(&map, directory.path().join("map.ir.json")).unwrap();
        let (prefix, trace) = roadgen_sumo::write_traced(&map, directory.path()).unwrap();
        let sidecar = directory_sidecar(directory.path(), &prefix, &trace.format);
        write_trace(&trace, &map, &sidecar).unwrap();
        let net = sumo_build::netconvert(directory.path(), &prefix);
        let built = Lights::read(&net);
        assert_eq!(built.internal.len(), 4, "{handedness:?}: {built:?}");

        let mut index = TraceIndex::new();
        index.load(directory.path().join("map.ir.json")).unwrap();
        index.load(&sidecar).unwrap();
        index
            .load_sumo_net(&net)
            .unwrap_or_else(|error| panic!("{handedness:?}: {error}"));

        let slots: BTreeSet<usize> = map
            .objects
            .iter()
            .filter(|object| object.kind.is_traffic_light())
            .flat_map(|light| index.from_ir(light.id.as_str(), "sumo").unwrap())
            .filter_map(|link| link.local.strip_prefix("tls:j_x/")?.parse().ok())
            .collect();
        assert_eq!(slots, (0..12).collect(), "{handedness:?}");
    }
}
