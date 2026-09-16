//! Building an exported network with SUMO, and reading back what SUMO made of it.
//!
//! The point of these helpers is that nothing here trusts the exporter's own view of
//! what it wrote. The plain XML goes to `netconvert`, which is the program that owns
//! the format, and what comes back is the `.net.xml` a simulation would run on —
//! parsed independently. If SUMO disagrees with the exporter, the disagreement shows
//! up here rather than in a user's simulation.
//!
//! `netconvert` and `sumo` are external programs. When they are not installed the
//! helpers say so and the test skips, unless `ROADGEN_REQUIRE_SUMO` is set — which CI
//! does set, so the checks that matter never silently stop running.

use std::collections::HashMap;
use std::path::{Path, PathBuf};
use std::process::Command;

use quick_xml::events::Event;
use quick_xml::Reader;

use roadgen_core::geometry::Point3;
use roadgen_core::validation::ValidatedMap;

/// Where SUMO's binaries are, if they are anywhere.
///
/// A package manager puts them on `PATH`; a source or pip install puts them in
/// `$SUMO_HOME/bin` and may not.
fn tool(name: &str) -> Option<PathBuf> {
    if let Ok(home) = std::env::var("SUMO_HOME") {
        let candidate = Path::new(&home).join("bin").join(name);
        if candidate.is_file() {
            return Some(candidate);
        }
    }
    let path = std::env::var_os("PATH")?;
    std::env::split_paths(&path)
        .map(|directory| directory.join(name))
        .find(|candidate| candidate.is_file())
}

/// Whether the SUMO tools are here, and what to do when they are not.
///
/// Returns `false` for "skip this test". With `ROADGEN_REQUIRE_SUMO` set it panics
/// instead, so a CI machine that lost its SUMO install fails loudly rather than
/// quietly stopping checking anything.
pub fn sumo_available() -> bool {
    let missing: Vec<&str> = ["netconvert", "sumo"]
        .into_iter()
        .filter(|name| tool(name).is_none())
        .collect();
    if missing.is_empty() {
        return true;
    }
    assert!(
        std::env::var_os("ROADGEN_REQUIRE_SUMO").is_none(),
        "ROADGEN_REQUIRE_SUMO is set but {} could not be found; install SUMO or unset it",
        missing.join(" and ")
    );
    eprintln!(
        "skipped: {} not installed (set ROADGEN_REQUIRE_SUMO to make this a failure)",
        missing.join(" and ")
    );
    false
}

/// Whether a line SUMO printed is about something other than the file it was given.
///
/// Two kinds are not the export's business. One is about the machine: SUMO says so
/// when `SUMO_HOME` is unset and it cannot find its schemas. The other is about the
/// *map*: where two roads meet at an angle, SUMO works out the turning radius of the
/// link between them and lowers the speed a vehicle may take it at. That is SUMO
/// reading the geometry correctly and saying what it found — a scenario like
/// `two_roads_joined`, which bends by 26 degrees at the joint, should produce it.
///
/// Everything else counts as a complaint and fails the test.
fn is_about_the_machine_or_the_map(line: &str) -> bool {
    line.contains("SUMO_HOME")
        || (line.contains("Speed of") && line.contains("due to turning radius"))
}

/// Exports `map` as a plain-XML network, hands it to `netconvert`, and reads back the
/// `.net.xml` that comes out.
///
/// The directory is returned alongside so that it outlives the test, and so that a
/// test can look at the plain files that went in as well as the network that came
/// out.
pub fn build(map: &ValidatedMap) -> (tempfile::TempDir, SumoNetwork) {
    let directory = tempfile::tempdir().expect("a temporary directory");
    let prefix = roadgen_sumo::write(map, directory.path()).expect("the map should export as SUMO");

    // Through the generated configuration, which is how a user runs it: if the
    // `.netccfg` names the wrong files or asks for the wrong processing, that is a
    // fault in the export and the test should see it.
    let output = Command::new(tool("netconvert").expect("netconvert"))
        .arg("-c")
        .arg(format!("{prefix}.netccfg"))
        .current_dir(directory.path())
        .output()
        .expect("netconvert should run");

    let stderr = String::from_utf8_lossy(&output.stderr).into_owned();
    let complaints: Vec<String> = stderr
        .lines()
        .filter(|line| !line.trim().is_empty())
        .filter(|line| !is_about_the_machine_or_the_map(line))
        .map(str::to_owned)
        .collect();
    assert!(
        output.status.success(),
        "netconvert rejected the export:\n{stderr}"
    );
    assert!(
        complaints.is_empty(),
        "netconvert accepted the export but complained about it:\n{}",
        complaints.join("\n")
    );

    let path = directory.path().join(format!("{prefix}.net.xml"));
    let xml = std::fs::read_to_string(&path).expect("netconvert should have written a network");
    let network = SumoNetwork::parse(&xml);
    (directory, network)
}

/// Loads a built network into the simulator itself.
///
/// netconvert accepting the plain XML says the *input* was well formed. This says the
/// network is one SUMO will actually run: it is the same load a simulation does,
/// stopped after a single step because there is no demand to simulate.
pub fn simulate(directory: &Path, prefix: &str) {
    let output = Command::new(tool("sumo").expect("sumo"))
        .args(["-n", &format!("{prefix}.net.xml")])
        .args(["--no-step-log", "true"])
        .args(["--end", "1"])
        .current_dir(directory)
        .output()
        .expect("sumo should run");

    let stderr = String::from_utf8_lossy(&output.stderr).into_owned();
    let complaints: Vec<&str> = stderr
        .lines()
        .filter(|line| !line.trim().is_empty())
        .filter(|line| !is_about_the_machine_or_the_map(line))
        .collect();
    assert!(
        output.status.success(),
        "sumo could not load the network:\n{stderr}"
    );
    assert!(
        complaints.is_empty(),
        "sumo loaded the network but complained about it:\n{}",
        complaints.join("\n")
    );
}

// --------------------------------------------------------------------------- //
// The network netconvert produced
// --------------------------------------------------------------------------- //

#[derive(Debug, Clone)]
pub struct SumoNetwork {
    pub edges: Vec<SumoEdge>,
    pub junctions: Vec<SumoJunction>,
    pub connections: Vec<SumoConnection>,
}

#[derive(Debug, Clone)]
pub struct SumoEdge {
    pub id: String,
    /// `internal` for the lanes netconvert generated inside a junction; absent for a
    /// road the export asked for.
    pub function: Option<String>,
    pub from: Option<String>,
    pub to: Option<String>,
    pub name: Option<String>,
    pub priority: Option<i32>,
    pub lanes: Vec<SumoLane>,
}

#[derive(Debug, Clone)]
pub struct SumoLane {
    pub id: String,
    pub index: usize,
    pub speed: f64,
    pub width: f64,
    pub length: f64,
    pub allow: Option<String>,
    pub disallow: Option<String>,
    pub shape: Vec<Point3>,
}

#[derive(Debug, Clone)]
pub struct SumoJunction {
    pub id: String,
    pub kind: String,
    pub position: Point3,
    pub incoming: Vec<String>,
}

#[derive(Debug, Clone)]
pub struct SumoConnection {
    pub from: String,
    pub to: String,
    pub from_lane: usize,
    pub to_lane: usize,
    /// The internal lane the movement runs over, when there is a junction to cross.
    pub via: Option<String>,
    /// The traffic light controlling the movement, if any.
    pub tl: Option<String>,
}

impl SumoNetwork {
    fn parse(xml: &str) -> SumoNetwork {
        let mut reader = Reader::from_str(xml);
        let mut network = SumoNetwork {
            edges: Vec::new(),
            junctions: Vec::new(),
            connections: Vec::new(),
        };
        let mut buffer = Vec::new();
        loop {
            let event = reader.read_event_into(&mut buffer).expect("valid net XML");
            match &event {
                Event::Eof => break,
                Event::Start(element) | Event::Empty(element) => {
                    let name = String::from_utf8_lossy(element.name().as_ref()).into_owned();
                    let attributes = attributes(element);
                    match name.as_str() {
                        "edge" => network.edges.push(SumoEdge {
                            id: attributes["id"].clone(),
                            function: attributes.get("function").cloned(),
                            from: attributes.get("from").cloned(),
                            to: attributes.get("to").cloned(),
                            name: attributes.get("name").cloned(),
                            priority: attributes
                                .get("priority")
                                .and_then(|value| value.parse().ok()),
                            lanes: Vec::new(),
                        }),
                        "lane" => {
                            let lane = SumoLane {
                                id: attributes["id"].clone(),
                                index: number(&attributes, "index") as usize,
                                speed: number(&attributes, "speed"),
                                // SUMO leaves the width out when it is the default.
                                width: attributes
                                    .get("width")
                                    .and_then(|value| value.parse().ok())
                                    .unwrap_or(3.2),
                                length: number(&attributes, "length"),
                                allow: attributes.get("allow").cloned(),
                                disallow: attributes.get("disallow").cloned(),
                                shape: points(attributes.get("shape").map(String::as_str)),
                            };
                            network
                                .edges
                                .last_mut()
                                .expect("a lane inside an edge")
                                .lanes
                                .push(lane);
                        }
                        "junction" => network.junctions.push(SumoJunction {
                            id: attributes["id"].clone(),
                            kind: attributes.get("type").cloned().unwrap_or_default(),
                            position: Point3::new(
                                number(&attributes, "x"),
                                number(&attributes, "y"),
                                attributes
                                    .get("z")
                                    .and_then(|value| value.parse().ok())
                                    .unwrap_or(0.0),
                            ),
                            incoming: attributes
                                .get("incLanes")
                                .map(|value| value.split_whitespace().map(str::to_owned).collect())
                                .unwrap_or_default(),
                        }),
                        "connection" => network.connections.push(SumoConnection {
                            from: attributes["from"].clone(),
                            to: attributes["to"].clone(),
                            from_lane: number(&attributes, "fromLane") as usize,
                            to_lane: number(&attributes, "toLane") as usize,
                            via: attributes.get("via").cloned(),
                            tl: attributes.get("tl").cloned(),
                        }),
                        _ => {}
                    }
                }
                _ => {}
            }
            buffer.clear();
        }
        network
    }

    /// The edges the export asked for, without the ones netconvert generated inside
    /// junctions.
    pub fn roads(&self) -> Vec<&SumoEdge> {
        self.edges
            .iter()
            .filter(|edge| edge.function.is_none())
            .collect()
    }

    pub fn edge(&self, id: &str) -> &SumoEdge {
        self.edges
            .iter()
            .find(|edge| edge.id == id)
            .unwrap_or_else(|| {
                panic!(
                    "no edge {id:?}; the network has {:?}",
                    self.roads().iter().map(|edge| &edge.id).collect::<Vec<_>>()
                )
            })
    }

    pub fn junction(&self, id: &str) -> &SumoJunction {
        self.junctions
            .iter()
            .find(|junction| junction.id == id)
            .unwrap_or_else(|| panic!("no junction {id:?}"))
    }

    /// The movements between the export's own edges, ignoring the ones that merely
    /// continue onto an internal lane.
    pub fn movements(&self) -> Vec<(&str, usize, &str, usize)> {
        let internal = |id: &str| id.starts_with(':');
        self.connections
            .iter()
            .filter(|connection| !internal(&connection.from) && !internal(&connection.to))
            .map(|connection| {
                (
                    connection.from.as_str(),
                    connection.from_lane,
                    connection.to.as_str(),
                    connection.to_lane,
                )
            })
            .collect()
    }
}

impl SumoEdge {
    pub fn lane(&self, index: usize) -> &SumoLane {
        self.lanes
            .iter()
            .find(|lane| lane.index == index)
            .unwrap_or_else(|| panic!("edge {} has no lane {index}", self.id))
    }
}

fn attributes(element: &quick_xml::events::BytesStart) -> HashMap<String, String> {
    element
        .attributes()
        .filter_map(Result::ok)
        .map(|attribute| {
            (
                String::from_utf8_lossy(attribute.key.as_ref()).into_owned(),
                quick_xml::escape::unescape(&String::from_utf8_lossy(&attribute.value))
                    .expect("an escaped attribute")
                    .into_owned(),
            )
        })
        .collect()
}

fn number(attributes: &HashMap<String, String>, key: &str) -> f64 {
    attributes
        .get(key)
        .and_then(|value| value.parse().ok())
        .unwrap_or_default()
}

/// A SUMO shape: `x,y` or `x,y,z` triples separated by spaces.
fn points(shape: Option<&str>) -> Vec<Point3> {
    shape
        .unwrap_or_default()
        .split_whitespace()
        .map(|point| {
            let parts: Vec<f64> = point
                .split(',')
                .map(|value| value.parse().expect("a number in a shape"))
                .collect();
            Point3::new(parts[0], parts[1], parts.get(2).copied().unwrap_or(0.0))
        })
        .collect()
}
