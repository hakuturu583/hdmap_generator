//! What a CARLA map is a map *of*: a town, a mine.
//!
//! The package is written from the IR whatever the map is; what differs between one
//! kind of place and another is what is generated into the IR first. A town is
//! streets with buildings along them. A mine is a pit, the ramp down it, the haul
//! roads out of it and the land round all of it. So the choice is one value,
//! [`Environment`], and each kind of place is a branch of it:
//!
//! ```text
//!   Environment ──▶ lay_out(&mut MapBuilder)    roads the place needs, if any
//!                          │
//!                   MapBuilder::finish()
//!                          │
//!                   generate(&mut UnvalidatedMap) what stands on and under them
//!                          │
//!                   validate() ──▶ write()       the same package for every kind
//! ```
//!
//! Every branch fills the IR and nothing else, so what it generates reaches every
//! format, not only CARLA's: a mine's gravel is in the OpenDRIVE, its quarry in the
//! OpenStreetMap.
//!
//! Adding a kind of place is adding a branch here: a variant of [`Environment`], a
//! module beside [`mine`], and an arm in [`lay_out`] and [`generate`].

pub mod land;
pub mod mine;
pub mod noise;
pub mod tin;

use std::fmt;

use roadgen_core::builder::MapBuilder;
use roadgen_core::validation::UnvalidatedMap;

pub use land::GroundSpec;
pub use mine::MineSpec;

/// A town: buildings along the caller's streets, and, when asked for, the land
/// round them.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct UrbanSpec {
    /// What its land is named after: `tin/<name>`.
    pub name: String,
    /// The shape grammar its buildings come from, or none.
    pub buildings: Option<roadgen_buildings::Rules>,
    /// The country round it, built the way a mine's is but shaped by nothing
    /// except its roads and sitting at their height; or no land in the IR, which
    /// leaves the CARLA export to lay its own ground under the roads.
    pub ground: Option<GroundSpec>,
    pub seed: u64,
}

/// The kind of place a map is.
#[derive(Debug, Clone, PartialEq)]
pub enum Environment {
    /// Streets, as the caller laid them out, with buildings along them from a CGA
    /// shape grammar, and the land round them.
    Urban(UrbanSpec),
    /// An open-pit mine: its roads, its benches and heaps, the yard's buildings
    /// from the same grammar, and the land as a TIN.
    Mine(Box<MineSpec>),
}

/// What [`lay_out`] did, and what [`generate`] needs to finish.
#[derive(Debug, Clone, PartialEq)]
pub enum Plan {
    Urban(UrbanSpec),
    Mine(Box<mine::Plan>),
}

/// What [`generate`] came to.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Report {
    Urban {
        buildings: Option<roadgen_buildings::Report>,
        land: Option<land::Report>,
    },
    Mine(mine::Report),
}

/// Why a place could not be generated.
#[derive(Debug, Clone, PartialEq)]
pub enum Error {
    Urban(roadgen_buildings::Error),
    Land(land::Error),
    Mine(mine::Error),
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Error::Urban(error) => write!(f, "{error}"),
            Error::Land(error) => write!(f, "{error}"),
            Error::Mine(error) => write!(f, "{error}"),
        }
    }
}

impl std::error::Error for Error {}

/// Puts on `builder` the roads the place needs. A town's are the caller's own, so
/// it adds none; a mine adds its ramp and haul roads.
pub fn lay_out(builder: &mut MapBuilder, environment: &Environment) -> Result<Plan, Error> {
    match environment {
        Environment::Urban(spec) => Ok(Plan::Urban(spec.clone())),
        Environment::Mine(spec) => mine::lay_out(builder, spec)
            .map(|plan| Plan::Mine(Box::new(plan)))
            .map_err(Error::Mine),
    }
}

/// Generates what stands on and under the roads, once they are built and before
/// the map is validated.
///
/// Both kinds of place go through the same two stages, with what differs passed
/// in: the buildings, from one shape grammar on lots along roads — every street
/// that is no other place's for a town, the yard beside the haul road for a mine
/// — and then the land, from one TIN builder round every road on the map, shaped
/// by nothing for a town and by the pit and heaps for a mine.
pub fn generate(map: &mut UnvalidatedMap, plan: &Plan) -> Result<Report, Error> {
    match plan {
        Plan::Urban(spec) => urban(map, spec),
        Plan::Mine(plan) => mine::generate(map, plan)
            .map(Report::Mine)
            .map_err(Error::Mine),
    }
}

fn urban(map: &mut UnvalidatedMap, spec: &UrbanSpec) -> Result<Report, Error> {
    let buildings = match &spec.buildings {
        Some(rules) => {
            // Every street but another place's: a mine's ramp is not a high street.
            let taken: Vec<_> = map
                .as_map()
                .sites
                .iter()
                .flat_map(|site| site.roads.iter().cloned())
                .collect();
            let roads = map
                .as_map()
                .roads
                .iter()
                .filter(|road| !taken.contains(&road.id))
                .map(|road| road.id.clone())
                .collect();
            let placement = roadgen_buildings::Placement {
                roads: Some(roads),
                within: Vec::new(),
                keep: true,
            };
            Some(roadgen_buildings::generate_with(map, rules, &placement).map_err(Error::Urban)?)
        }
        None => None,
    };
    let land = match &spec.ground {
        Some(ground) => {
            if !map.as_map().tin_surfaces.is_empty() {
                return Err(Error::Land(land::Error::Spec(
                    "the map already states its land — a mine's — and a town's would \
                     lie over it"
                        .into(),
                )));
            }
            let name = if spec.name.trim().is_empty() {
                "town".to_owned()
            } else {
                spec.name.clone()
            };
            let land = land::Land {
                name,
                site: None,
                base: land::Base::Roads,
                ground: ground.clone(),
                seed: spec.seed,
            };
            Some(
                land::generate(map, &land, land::Landforms::default())
                    .map_err(Error::Land)?
                    .1,
            )
        }
        None => None,
    };
    Ok(Report::Urban { buildings, land })
}
