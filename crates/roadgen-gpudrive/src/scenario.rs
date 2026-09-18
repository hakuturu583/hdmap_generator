//! The scenario file: who drives the map, and where.
//!
//! A generated road network is one thing; the traffic on it is another. The same map
//! should be drivable by several sets of agents, at several speeds, without editing the
//! map or recompiling — so the agents come from a file.
//!
//! ```yaml
//! name: town
//! scenario_id: town-0001
//! steps: 91            # timesteps; the simulator reads at most 91
//! time_step: 0.1       # seconds between them
//!
//! agents:
//!   - type: vehicle
//!     speed: 12.0                  # metres per second
//!     route:
//!       start: lane/north/0        # follow successors from here
//!       # lanes: [lane/north/0, …] # or drive exactly these, in order
//!   - type: cyclist
//!     speed: 4.0
//!     route:
//!       start: lane/north/1
//!     mark_as_expert: true         # replayed rather than controlled
//! ```
//!
//! Unknown keys are an error rather than being ignored. A scenario file is a thing
//! people edit by hand, and a silently dropped `speed_kph` that should have been
//! `speed` is a worse outcome than a message saying so.

use std::path::Path;

use serde::Deserialize;

use roadgen_core::LaneId;

use crate::error::ExportError;
use crate::scene::ObjectKind;
use crate::{Agent, Route, SceneConfig};

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Document {
    name: Option<String>,
    scenario_id: Option<String>,
    steps: Option<usize>,
    time_step: Option<f64>,
    agents: Option<Vec<AgentDocument>>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct AgentDocument {
    #[serde(rename = "type")]
    kind: Option<String>,
    speed: Option<f64>,
    route: Option<RouteDocument>,
    length: Option<f64>,
    width: Option<f64>,
    height: Option<f64>,
    mark_as_expert: Option<bool>,
    track_to_predict: Option<bool>,
    difficulty: Option<i32>,
    of_interest: Option<bool>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct RouteDocument {
    start: Option<String>,
    lanes: Option<Vec<String>>,
}

/// Reads a scenario from YAML text, starting from `base`.
///
/// Anything the file does not mention keeps the value it has in `base`, so a scenario
/// can say only what it wants to change.
pub fn from_yaml_str(text: &str, base: SceneConfig) -> Result<SceneConfig, ExportError> {
    let document: Document =
        serde_norway::from_str(text).map_err(|error| ExportError::Scenario(error.to_string()))?;
    apply(document, base)
}

/// Reads a scenario from a file, starting from `base`.
pub fn from_yaml_file(
    path: impl AsRef<Path>,
    base: SceneConfig,
) -> Result<SceneConfig, ExportError> {
    let path = path.as_ref();
    let text = std::fs::read_to_string(path)
        .map_err(|error| ExportError::Io(format!("{}: {error}", path.display())))?;
    from_yaml_str(&text, base).map_err(|error| match error {
        // A parse failure names a line and column, which is only useful with the file
        // it belongs to.
        ExportError::Scenario(detail) => {
            ExportError::Scenario(format!("{}: {detail}", path.display()))
        }
        other => other,
    })
}

fn apply(document: Document, base: SceneConfig) -> Result<SceneConfig, ExportError> {
    let mut config = base;
    if let Some(name) = document.name {
        config.name = name;
    }
    if let Some(scenario_id) = document.scenario_id {
        config.scenario_id = scenario_id;
    }
    if let Some(steps) = document.steps {
        config.steps = steps;
    }
    if let Some(time_step) = document.time_step {
        config.time_step = time_step;
    }
    if let Some(agents) = document.agents {
        // An explicit empty list is a scene of no agents, which is a thing a caller
        // may want; `agents` left out keeps whatever the base had.
        config.agents = agents.into_iter().map(agent_of).collect::<Result<_, _>>()?;
    }
    Ok(config)
}

fn agent_of(document: AgentDocument) -> Result<Agent, ExportError> {
    let kind = match document.kind.as_deref() {
        Some(text) => object_kind(text)?,
        None => ObjectKind::Vehicle,
    };
    // `Agent::new` has already sized the agent for its kind; the file only says where
    // it differs.
    let mut agent = Agent::new(kind);
    if let Some(length) = document.length {
        agent.length = length;
    }
    if let Some(width) = document.width {
        agent.width = width;
    }
    if let Some(height) = document.height {
        agent.height = height;
    }
    if let Some(speed) = document.speed {
        agent.speed = speed;
    }
    if let Some(route) = document.route {
        agent.route = Some(route_of(route)?);
    }
    if let Some(expert) = document.mark_as_expert {
        agent.mark_as_expert = expert;
    }
    if let Some(predict) = document.track_to_predict {
        agent.track_to_predict = predict;
    }
    if let Some(difficulty) = document.difficulty {
        agent.difficulty = difficulty;
    }
    if let Some(interest) = document.of_interest {
        agent.of_interest = interest;
    }
    Ok(agent)
}

/// The agent types the scenario file spells, including the aliases a caller is likely
/// to write. The three kinds themselves are the reader's, and live with its model.
fn object_kind(text: &str) -> Result<ObjectKind, ExportError> {
    Ok(match text.to_ascii_lowercase().as_str() {
        "vehicle" | "car" => ObjectKind::Vehicle,
        "pedestrian" => ObjectKind::Pedestrian,
        "cyclist" | "bicycle" => ObjectKind::Cyclist,
        _ => {
            return Err(ExportError::Scenario(format!(
                "{text:?} is not an agent type: GPUDrive knows vehicle, pedestrian and \
                 cyclist"
            )))
        }
    })
}

fn route_of(document: RouteDocument) -> Result<Route, ExportError> {
    match (document.start, document.lanes) {
        (Some(start), None) => Ok(Route::From(LaneId::parse_printed(start))),
        (None, Some(lanes)) => {
            if lanes.is_empty() {
                return Err(ExportError::Scenario(
                    "route.lanes is empty; leave the whole `route` out to have one \
                     found automatically"
                        .into(),
                ));
            }
            Ok(Route::Lanes(
                lanes.iter().map(LaneId::parse_printed).collect(),
            ))
        }
        (Some(_), Some(_)) => Err(ExportError::Scenario(
            "route has both `start` and `lanes`: either follow successors from one \
             lane or list them all, not both"
                .into(),
        )),
        (None, None) => Err(ExportError::Scenario(
            "route has neither `start` nor `lanes`".into(),
        )),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_scenario_says_only_what_it_changes() {
        let config = from_yaml_str("speed: 3.0\n", SceneConfig::default());
        // `speed` is a property of an agent, not of the scene, so this is the kind of
        // near-miss `deny_unknown_fields` is here to catch.
        assert!(matches!(config, Err(ExportError::Scenario(_))));

        let config = from_yaml_str("steps: 40\n", SceneConfig::new("town")).unwrap();
        assert_eq!(config.steps, 40);
        assert_eq!(config.name, "town");
        assert_eq!(config.agents.len(), 1);
    }

    #[test]
    fn an_agent_keeps_the_size_of_its_kind_unless_it_says_otherwise() {
        use crate::objects::default_size;

        let config = from_yaml_str(
            "agents:\n  - type: cyclist\n  - type: vehicle\n    length: 12.0\n",
            SceneConfig::default(),
        )
        .unwrap();
        assert_eq!(config.agents[0].kind, ObjectKind::Cyclist);
        assert_eq!(config.agents[0].length, default_size(ObjectKind::Cyclist).0);
        assert_eq!(config.agents[1].length, 12.0);
        assert_eq!(config.agents[1].width, default_size(ObjectKind::Vehicle).1);
    }

    #[test]
    fn a_route_is_either_a_start_or_a_list() {
        let config = from_yaml_str(
            "agents:\n  - route:\n      start: north/0\n",
            SceneConfig::default(),
        )
        .unwrap();
        assert_eq!(
            config.agents[0].route,
            Some(Route::From(LaneId::parse_printed("lane/north/0")))
        );

        for text in [
            "agents:\n  - route:\n      start: north/0\n      lanes: [north/0]\n",
            "agents:\n  - route: {}\n",
            "agents:\n  - route:\n      lanes: []\n",
        ] {
            assert!(
                matches!(
                    from_yaml_str(text, SceneConfig::default()),
                    Err(ExportError::Scenario(_))
                ),
                "{text:?}"
            );
        }
    }

    #[test]
    fn an_agent_type_the_simulator_does_not_know_is_refused() {
        assert!(matches!(
            from_yaml_str("agents:\n  - type: tram\n", SceneConfig::default()),
            Err(ExportError::Scenario(_))
        ));
    }
}
