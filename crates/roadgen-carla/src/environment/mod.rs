//! What stands on and under a CARLA map's roads: its buildings, its land, and the
//! places — pits, heaps, yards — the land is shaped into.
//!
//! The package is written from the IR whatever the map is a map of; what differs
//! between a town and a mine is only what is put into the IR first, and that is
//! made of parts the caller places one by one:
//!
//! ```text
//!   MapBuilder: add_road, add_junction, add_barrier   the roads, any shape — a
//!        │                                            ramp, a haul road, a street
//!   MapBuilder::finish()
//!        │
//!   generate(&mut UnvalidatedMap, &Places)            pits, heaps, pads, buildings,
//!        │                                            and one land round all of it
//!   validate() ──▶ write()                            the same package for every map
//! ```
//!
//! There is no "mine" and no "town" here: a mine is a map with a pit on it and
//! roads into the pit, a town a map with buildings along its streets, and a map
//! may have both, or two pits. Every part fills the IR and nothing else, so what it
//! generates reaches every format, not only CARLA's: a mine's gravel is in the
//! OpenDRIVE, its quarry in the OpenStreetMap.
//!
//! # One land
//!
//! The land is one TIN, built once round every road and shaped by every part, so
//! two pits beside each other are two holes in the same country, never two TINs
//! over one plan. It sits at the height of the roads and of the parts' rims, and
//! rolls away from all of them.
//!
//! # Sites
//!
//! Parts and roads that name the same site are gathered into one [`Site`]: its land
//! covers, its roads, and the land they all lie on. A site with a pit in it is an
//! `open_pit_mine`.

pub mod heap;
pub mod land;
pub mod noise;
pub mod pad;
pub mod pit;
pub mod tin;

use std::collections::BTreeMap;
use std::fmt;

use roadgen_core::geometry::plan::{ring_contains, PlanPoint};
use roadgen_core::id::{LandCoverId, RoadId, SiteId};
use roadgen_core::site::Site;
use roadgen_core::validation::UnvalidatedMap;

pub use heap::HeapSpec;
pub use land::GroundSpec;
pub use pad::PadSpec;
pub use pit::{PitSpec, RampSpec};

use land::{plan_of, Base, Land, Landforms};

/// Everything generated onto a map once its roads are built.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Places {
    pub pits: Vec<PitSpec>,
    pub heaps: Vec<HeapSpec>,
    pub pads: Vec<PadSpec>,
    /// Roads that belong to a site, by the site's name.
    pub site_roads: Vec<(RoadId, String)>,
    /// Buildings along every road that is no site's, from a CGA shape grammar —
    /// a town — kept off the pits, heaps and pads.
    pub town: Option<roadgen_buildings::Rules>,
    /// The land. Pits and heaps are shapes of it, so with any of them on the map it
    /// is built with [`GroundSpec::default`] when nothing else is said; with none,
    /// and nothing said, the map states no land and the CARLA export lays its own
    /// ground under the roads.
    pub ground: Option<GroundSpec>,
    pub seed: u64,
}

/// What [`generate`] came to.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Report {
    /// Buildings put on the map, along the streets and on the pads.
    pub buildings: usize,
    pub land: Option<land::Report>,
    pub land_covers: usize,
    pub sites: usize,
}

/// Why the places could not be generated.
#[derive(Debug, Clone, PartialEq)]
pub enum Error {
    /// A part asks for something it could not be, or overlaps another.
    Spec(String),
    Buildings(roadgen_buildings::Error),
    Land(land::Error),
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Error::Spec(detail) => write!(f, "{detail}"),
            Error::Buildings(error) => write!(f, "buildings: {error}"),
            Error::Land(error) => write!(f, "{error}"),
        }
    }
}

impl std::error::Error for Error {}

impl From<land::Error> for Error {
    fn from(error: land::Error) -> Self {
        Error::Land(error)
    }
}

/// The name made safe to put in an identifier: lower case, words joined by `_`.
pub fn slug(name: &str) -> String {
    let mut out = String::new();
    for character in name.trim().chars() {
        if character.is_ascii_alphanumeric() {
            out.push(character.to_ascii_lowercase());
        } else if !out.ends_with('_') {
            out.push('_');
        }
    }
    out.trim_matches('_').to_owned()
}

/// What the land is named: `tin/land`.
pub const LAND: &str = "land";

/// One part, ready to be put on the land.
struct Part {
    key: String,
    /// Its land cover's kind.
    kind: String,
    site: Option<String>,
    /// Its outline on the ground: what its land cover covers, and what nothing
    /// else may overlap.
    outline: Vec<roadgen_core::geometry::Point3>,
    forms: Landforms,
}

impl Places {
    fn parts(&self, spacing: f64) -> Result<Vec<Part>, Error> {
        let mut parts = Vec::new();
        let named = |name: &str, what: &str| {
            let key = slug(name);
            if key.is_empty() {
                Err(Error::Spec(format!(
                    "a {what} named {name:?} has nothing to name features after"
                )))
            } else {
                Ok(key)
            }
        };
        for pit in &self.pits {
            pit.check().map_err(Error::Spec)?;
            let key = named(&pit.name, "pit")?;
            parts.push(Part {
                forms: pit.landforms(&key, spacing),
                outline: pit.rim(spacing),
                kind: "pit".into(),
                site: Some(pit.site.clone().unwrap_or_else(|| pit.name.clone())),
                key,
            });
        }
        for heap in &self.heaps {
            heap.check().map_err(Error::Spec)?;
            let key = named(&heap.name, "heap")?;
            parts.push(Part {
                forms: heap.landforms(&key, spacing),
                outline: heap.toe(spacing),
                kind: heap.kind.clone(),
                site: heap.site.clone(),
                key,
            });
        }
        for pad in &self.pads {
            pad.check().map_err(Error::Spec)?;
            let key = named(&pad.name, "pad")?;
            parts.push(Part {
                forms: pad.landforms(),
                outline: pad.outline.clone(),
                kind: "pad".into(),
                site: pad.site.clone(),
                key,
            });
        }
        // Two parts in one place would be two shapes of one piece of land.
        for (index, a) in parts.iter().enumerate() {
            if parts[..index].iter().any(|b| b.key == a.key) {
                return Err(Error::Spec(format!(
                    "two parts are named {:?}; every pit, heap and pad needs a name of its own",
                    a.key
                )));
            }
            let ring_a = plan_of(&a.outline);
            for b in &parts[..index] {
                let ring_b = plan_of(&b.outline);
                if overlap(&ring_a, &ring_b) {
                    return Err(Error::Spec(format!(
                        "{} and {} overlap; a part has to stand clear of the others",
                        a.key, b.key
                    )));
                }
            }
        }
        Ok(parts)
    }
}

fn overlap(a: &[PlanPoint], b: &[PlanPoint]) -> bool {
    a.iter().any(|&p| ring_contains(b, p)) || b.iter().any(|&p| ring_contains(a, p))
}

/// Generates the places onto `map`, once its roads are built and before it is
/// validated: the buildings first — the land keeps its roll off them — then one
/// land round every road and shaped by every part, then what each part's land is,
/// then the sites.
pub fn generate(map: &mut UnvalidatedMap, places: &Places) -> Result<Report, Error> {
    let ground = places.ground.clone().unwrap_or_default();
    ground.check().map_err(Error::Spec)?;
    let parts = places.parts(ground.spacing)?;
    let mut report = Report::default();

    // The buildings: the pads' kept to their pads, the town's along every road no
    // site has, kept off every part.
    let before = map.as_map().buildings.len();
    for pad in &places.pads {
        if let Some(rules) = &pad.buildings {
            let placement = roadgen_buildings::Placement {
                roads: None,
                within: vec![plan_of(&pad.outline)],
                outside: Vec::new(),
                keep: true,
            };
            roadgen_buildings::generate_with(map, rules, &placement).map_err(Error::Buildings)?;
        }
    }
    if let Some(rules) = &places.town {
        let roads = map
            .as_map()
            .roads
            .iter()
            .filter(|road| !places.site_roads.iter().any(|(id, _)| id == &road.id))
            .map(|road| road.id.clone())
            .collect();
        let placement = roadgen_buildings::Placement {
            roads: Some(roads),
            within: Vec::new(),
            outside: parts.iter().map(|part| plan_of(&part.outline)).collect(),
            keep: true,
        };
        roadgen_buildings::generate_with(map, rules, &placement).map_err(Error::Buildings)?;
    }
    report.buildings = map.as_map().buildings.len() - before;

    // The land: one, round everything.
    if parts.is_empty() && places.ground.is_none() {
        return Ok(report);
    }
    let mut forms = Landforms::default();
    for part in &parts {
        forms.extend(part.forms.clone());
    }
    let land = Land {
        name: LAND.into(),
        site: None,
        base: Base::Roads,
        ground,
        seed: places.seed,
    };
    let (surface, built) = land::generate(map, &land, forms)?;
    report.land = Some(built);

    // What each part's land is, and which site it belongs to.
    let mut sites: BTreeMap<String, (String, Vec<LandCoverId>, Vec<RoadId>, bool)> =
        BTreeMap::new();
    let site_of = |name: &str| -> Result<SiteId, Error> {
        let key = slug(name);
        if key.is_empty() {
            return Err(Error::Spec(format!(
                "a site named {name:?} has nothing to name it after"
            )));
        }
        Ok(SiteId::new(key))
    };
    let target = map.as_map_mut();
    for part in &parts {
        let site = part.site.as_deref().map(site_of).transpose()?;
        let id = land::cover(
            target,
            LAND,
            &part.key,
            &part.kind,
            part.outline.clone(),
            site.as_ref(),
            Some(&surface),
        )?;
        report.land_covers += 1;
        if let (Some(site), Some(name)) = (site, &part.site) {
            let entry = sites
                .entry(site.local_name().to_owned())
                .or_insert_with(|| (name.clone(), Vec::new(), Vec::new(), false));
            entry.1.push(id);
            entry.3 |= part.kind == "pit";
        }
    }
    for (road, name) in &places.site_roads {
        if !target.roads.contains(road) {
            return Err(Error::Spec(format!("{road} is no road on the map")));
        }
        let site = site_of(name)?;
        let entry = sites
            .entry(site.local_name().to_owned())
            .or_insert_with(|| (name.clone(), Vec::new(), Vec::new(), false));
        entry.2.push(road.clone());
    }
    // A junction every one of whose roads is a site's is the site's, and so is
    // every road through it.
    for junction in target.junctions.iter() {
        let owner = junction
            .incoming_roads
            .iter()
            .map(|road| places.site_roads.iter().find(|(id, _)| id == road))
            .collect::<Option<Vec<_>>>()
            .and_then(|owners| {
                let first = slug(&owners.first()?.1);
                owners
                    .iter()
                    .all(|(_, name)| slug(name) == first)
                    .then_some(first)
            });
        if let Some(entry) = owner.and_then(|key| sites.get_mut(&key)) {
            entry.2.extend(junction.connecting_roads.iter().cloned());
        }
    }
    for (key, (name, land_covers, roads, pit)) in sites {
        let id = SiteId::new(key);
        target
            .sites
            .insert(
                id.clone(),
                Site {
                    id,
                    name: Some(name),
                    kind: if pit { "open_pit_mine" } else { "works" }.into(),
                    land_covers,
                    surfaces: vec![surface.clone()],
                    roads,
                },
            )
            .map_err(|duplicate| land::Error::Duplicate(duplicate.0.to_string()))?;
        report.sites += 1;
    }
    Ok(report)
}
