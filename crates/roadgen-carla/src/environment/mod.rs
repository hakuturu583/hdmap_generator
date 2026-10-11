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

pub mod corridor;
pub mod foundation;
pub mod heap;
pub(crate) mod highway;
pub mod hill;
pub mod land;
pub mod noise;
pub mod pad;
pub mod pit;
pub mod river;
mod structures;
pub mod tin;

use std::collections::BTreeMap;
use std::fmt;

use roadgen_core::geometry::plan::rings_overlap;
use roadgen_core::id::{LandCoverId, RoadId, SiteId};
use roadgen_core::site::Site;
use roadgen_core::validation::UnvalidatedMap;

pub use corridor::{
    Control, CorridorPlan, CorridorSpec, DesignStandard, InterchangeKind, InterchangeSite,
    RampSite, Side, StructureMode,
};
pub use heap::HeapSpec;
pub use hill::HillSpec;
pub use land::GroundSpec;
pub use pad::PadSpec;
pub use pit::{PitSpec, RampSpec};
pub use river::RiverSpec;

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
    /// The land. Pits, heaps and pads are shapes of it, so with any of them on the
    /// map it is built — with [`GroundSpec::default`] when nothing else is said;
    /// with none, and nothing said, the map states no land and the CARLA export
    /// lays its own ground under the roads.
    pub ground: Option<GroundSpec>,
    pub seed: u64,
    /// Hills and ridges: swells of the country, for a tunnel to go through.
    pub hills: Vec<HillSpec>,
    /// Rivers: channels in the land, for a bridge to cross.
    pub rivers: Vec<RiverSpec>,
    /// Motorway corridors: the rules their roads are fitted to the land by.
    pub corridors: Vec<CorridorSpec>,
    /// Roads that belong to a corridor, by the corridor's name.
    pub corridor_roads: Vec<(RoadId, String)>,
    /// Roads the land is cut and filled to meet, rather than read its height from,
    /// that are no corridor's: a crossroad lifted over a motorway.
    pub fitted_roads: Vec<RoadId>,
    /// Heights the land meets where they are known: the ground a crossroad was
    /// drawn on, before it was lifted.
    pub anchors: Vec<roadgen_core::geometry::Point3>,
}

/// What [`generate`] came to.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct Report {
    /// Buildings put on the map, along the streets and on the pads.
    pub buildings: usize,
    pub land: Option<land::Report>,
    pub land_covers: usize,
    pub sites: usize,
    /// Bridges and tunnels on the map once the rules have run.
    pub structures: usize,
    /// Buildings given a foundation down to land that falls away under them.
    pub foundations: usize,
    /// Where a rule could not be kept, or was kept only just.
    pub notes: Vec<String>,
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

pub use roadgen_core::structure::slug;

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
                forms: pad.landforms(&key, spacing),
                // At the height it is levelled to, which is the land's under it.
                outline: pad
                    .outline
                    .iter()
                    .map(|p| roadgen_core::geometry::Point3::new(p.x, p.y, pad.height()))
                    .collect(),
                kind: "pad".into(),
                site: pad.site.clone(),
                key,
            });
        }
        for river in &self.rivers {
            river.check().map_err(Error::Spec)?;
            let key = named(&river.name, "river")?;
            parts.push(Part {
                forms: river.landforms(&key, spacing),
                outline: river.outline(spacing),
                kind: "water".into(),
                site: river.site.clone(),
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
                if rings_overlap(&ring_a, &ring_b) {
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

/// The roads through every junction whose roads all belong to one site, by that
/// site's key: they are the site's too.
fn site_junctions(map: &roadgen_core::map::Map, places: &Places) -> Vec<(String, Vec<RoadId>)> {
    map.junctions
        .iter()
        .filter_map(|junction| {
            let owners = junction
                .incoming_roads
                .iter()
                .map(|road| places.site_roads.iter().find(|(id, _)| id == road))
                .collect::<Option<Vec<_>>>()?;
            let first = slug(&owners.first()?.1);
            owners
                .iter()
                .all(|(_, name)| slug(name) == first)
                .then(|| (first, junction.connecting_roads.clone()))
        })
        .collect()
}

/// The roads through every junction whose roads all belong to one corridor: they
/// are the corridor's too.
fn corridor_junctions(map: &roadgen_core::map::Map, places: &Places) -> Vec<(String, Vec<RoadId>)> {
    map.junctions
        .iter()
        .filter_map(|junction| {
            let owners = junction
                .incoming_roads
                .iter()
                .map(|road| places.corridor_roads.iter().find(|(id, _)| id == road))
                .collect::<Option<Vec<_>>>()?;
            let first = owners.first()?.1.clone();
            owners
                .iter()
                .all(|(_, name)| *name == first)
                .then(|| (first, junction.connecting_roads.clone()))
        })
        .collect()
}

/// Generates the places onto `map`, once its roads are built and before it is
/// validated:
///
/// 1. the natural land — the country, the hills, the rivers and the parts —
///    before anything is fitted to it;
/// 2. the structures a corridor's roads need to cross it: a bridge where a road is
///    far above it or over water, a tunnel where it is far below;
/// 3. an overpass wherever one road crosses over another with nothing carrying it,
///    and every grade separation;
/// 4. the cuttings and embankments that fit the rest of the corridor to the land;
/// 5. the buildings — off the corridors, the structures and the parts;
/// 6. one land round all of it;
/// 7. what each part's land is, and the sites.
pub fn generate(map: &mut UnvalidatedMap, places: &Places) -> Result<Report, Error> {
    let ground = places.ground.clone().unwrap_or_default();
    ground.check().map_err(Error::Spec)?;
    let parts = places.parts(ground.spacing)?;
    let mut report = Report::default();
    for hill in &places.hills {
        hill.check().map_err(Error::Spec)?;
    }
    for corridor in &places.corridors {
        corridor.check().map_err(Error::Spec)?;
    }

    // Everything that shapes the land, and the roads the land does not take its
    // height from: a corridor's, and the movements through its junctions.
    let mut forms = Landforms::default();
    for part in &parts {
        forms.extend(part.forms.clone());
    }
    for hill in &places.hills {
        forms.extend(hill.landforms());
    }
    let mut corridor_roads = places.corridor_roads.clone();
    for (corridor, roads) in corridor_junctions(map.as_map(), places) {
        corridor_roads.extend(roads.into_iter().map(|road| (road, corridor.clone())));
    }
    for (road, _) in &corridor_roads {
        if !map.as_map().roads.contains(road) {
            return Err(Error::Spec(format!("{road} is no road on the map")));
        }
    }
    forms.floating = corridor_roads
        .iter()
        .map(|(road, _)| road.clone())
        .collect();
    forms.floating.extend(places.fitted_roads.iter().cloned());
    // And the movements through a junction whose roads are all fitted to the land:
    // a ramp terminal on a lifted crossroad is as far off the ground as they are.
    let through: Vec<RoadId> = map
        .as_map()
        .junctions
        .iter()
        .filter(|junction| {
            junction
                .incoming_roads
                .iter()
                .all(|road| forms.floating.contains(road))
        })
        .flat_map(|junction| junction.connecting_roads.iter().cloned())
        .collect();
    forms.floating.extend(through);
    forms.anchors.extend(places.anchors.iter().copied());

    // The structures and the separations: what the corridors ask for, and what any
    // road crossing another does. A map with none of either — no corridor, no
    // structure, no motorway — is left exactly as it was.
    let separates = !places.corridors.is_empty()
        || !map.as_map().structures.is_empty()
        || map
            .as_map()
            .roads
            .iter()
            .any(|road| road.form.is_controlled_access());
    let water: Vec<Vec<roadgen_core::geometry::plan::PlanPoint>> = parts
        .iter()
        .filter(|part| part.kind == "water")
        .map(|part| plan_of(&part.outline))
        .collect();
    let mut earthworks = None;
    // The stretches of a fitted road that stand well off the land — a crossroad's
    // embankment up to its overpass — which nothing has a frontage on.
    let mut embanked: Vec<Vec<roadgen_core::geometry::plan::PlanPoint>> = Vec::new();
    if separates {
        let natural = highway::Natural {
            field: land::base_field(
                map.as_map(),
                &ground,
                &forms.holes,
                forms.anchors.clone(),
                &forms.floating,
            ),
            rises: &forms.rises,
        };
        let modes: Vec<(String, StructureMode)> = places
            .corridors
            .iter()
            .map(|corridor| (corridor.name.clone(), corridor.structures))
            .collect();
        let target = map.as_map_mut();
        report.notes.extend(highway::choose_structures(
            target,
            &places.corridor_roads,
            &modes,
            &natural,
            &water,
        ));
        report.notes.extend(highway::separate(target));
        // Every road the land is fitted to; the movements among them are skipped,
        // as the land does not take its slopes from a junction.
        if !forms.floating.is_empty() {
            earthworks = Some(highway::earthworks(target, &forms.floating, &natural));
        }
        embanked = highway::off_the_land(target, &places.fitted_roads, &natural);
    }
    if let Some(works) = &earthworks {
        forms.lines.extend(works.lines.clone());
        forms.holes.extend(works.holes.clone());
    }
    report.structures = map.as_map().structures.len();

    // The buildings: the pads' kept to their pads, the town's along every road no
    // site and no corridor has, and none that is a motorway or a slip road — they
    // have no frontage — kept off every part, every corridor and the ground under
    // every bridge.
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
        let taken = site_junctions(map.as_map(), places);
        let source = map.as_map();
        let roads = source
            .roads
            .iter()
            .filter(|road| !places.site_roads.iter().any(|(id, _)| id == &road.id))
            .filter(|road| !taken.iter().any(|(_, roads)| roads.contains(&road.id)))
            .filter(|road| !corridor_roads.iter().any(|(id, _)| id == &road.id))
            .filter(|road| !road.form.is_controlled_access())
            .map(|road| road.id.clone())
            .collect();
        let mut outside: Vec<Vec<roadgen_core::geometry::plan::PlanPoint>> =
            parts.iter().map(|part| plan_of(&part.outline)).collect();
        outside.extend(embanked.iter().cloned());
        for (road, _) in &corridor_roads {
            let Some(entry) = source.road(road) else {
                continue;
            };
            let Ok(length) = entry.horizontal_length() else {
                continue;
            };
            // The road, its slopes and its right of way.
            if let Ok(ring) = roadgen_core::structure::outline(
                source,
                &[roadgen_core::structure::RoadSpan::new(
                    road.clone(),
                    0.0,
                    length,
                )],
                25.0,
            ) {
                outside.push(plan_of(&ring));
            }
        }
        for structure in source.structures.iter() {
            let margin = match &structure.kind {
                roadgen_core::structure::StructureKind::Bridge(_) => 5.0,
                roadgen_core::structure::StructureKind::Tunnel(tunnel) => {
                    if tunnel.form == roadgen_core::structure::TunnelForm::Bored {
                        continue;
                    }
                    2.0
                }
            };
            if let Ok(ring) = roadgen_core::structure::outline(source, &structure.carries, margin) {
                outside.push(plan_of(&ring));
            }
        }
        let placement = roadgen_buildings::Placement {
            roads: Some(roads),
            within: Vec::new(),
            outside,
            keep: true,
        };
        roadgen_buildings::generate_with(map, rules, &placement).map_err(Error::Buildings)?;
    }
    report.buildings = map.as_map().buildings.len() - before;

    // The land: one, round everything — when there is anything to shape it, or it
    // was asked for.
    let shaped = !parts.is_empty()
        || !places.hills.is_empty()
        || !places.corridors.is_empty()
        || !map.as_map().structures.is_empty();
    let surface = if !shaped && places.ground.is_none() {
        None
    } else {
        let land = Land {
            name: LAND.into(),
            site: None,
            base: Base::Roads,
            ground,
            seed: places.seed,
        };
        let (surface, built) = land::generate(map, &land, forms)?;
        report.land = Some(built);
        Some(surface)
    };

    // Each pier stands on the land, where there is some.
    if let Some(surface) = &surface {
        let target = map.as_map_mut();
        let heights: Vec<(roadgen_core::id::StructureId, Vec<Option<f64>>)> = {
            let tin = target.tin_surfaces.get(surface).expect("just built");
            let locator = tin.tin.locator();
            target
                .structures
                .iter()
                .filter_map(|structure| {
                    let bridge = structure.kind.bridge()?;
                    Some((
                        structure.id.clone(),
                        bridge
                            .supports
                            .iter()
                            .map(|support| {
                                let centre = support.footprint.centroid();
                                let top = support.footprint.lowest();
                                locator.height_at(centre.x, centre.y).map(|z| z.min(top))
                            })
                            .collect(),
                    ))
                })
                .collect()
        };
        for (id, bases) in heights {
            if let Some(roadgen_core::structure::StructureKind::Bridge(bridge)) =
                target.structures.get_mut(&id).map(|s| &mut s.kind)
            {
                for (support, base) in bridge.supports.iter_mut().zip(bases) {
                    support.base = base;
                }
            }
        }
    }

    // Every building stands on the land: where it falls away under one, the
    // building gets a foundation down to it.
    if let Some(surface) = &surface {
        report.foundations = foundation::lay(map.as_map_mut(), surface)?;
    }

    // What each part's land is, and which site it belongs to.
    // By key: the site's name, its land covers, its roads and its kind.
    type Gathered = (String, Vec<LandCoverId>, Vec<RoadId>, &'static str);
    let mut sites: BTreeMap<String, Gathered> = BTreeMap::new();
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
            surface.as_ref(),
        )?;
        report.land_covers += 1;
        if let (Some(site), Some(name)) = (site, &part.site) {
            let entry = sites
                .entry(site.local_name().to_owned())
                .or_insert_with(|| (name.clone(), Vec::new(), Vec::new(), "works"));
            entry.1.push(id);
            if part.kind == "pit" {
                entry.3 = "open_pit_mine";
            }
        }
    }
    for (road, name) in &places.site_roads {
        if !target.roads.contains(road) {
            return Err(Error::Spec(format!("{road} is no road on the map")));
        }
        let site = site_of(name)?;
        let entry = sites
            .entry(site.local_name().to_owned())
            .or_insert_with(|| (name.clone(), Vec::new(), Vec::new(), "works"));
        entry.2.push(road.clone());
    }
    for (key, roads) in site_junctions(target, places) {
        if let Some(entry) = sites.get_mut(&key) {
            entry.2.extend(roads);
        }
    }
    // A corridor is a site of its own: a motorway, its roads, and the cuttings and
    // embankments it is fitted to the land with.
    for corridor in &places.corridors {
        let site = site_of(&corridor.name)?;
        let key = site.local_name().to_owned();
        let entry = sites
            .entry(key.clone())
            .or_insert_with(|| (corridor.name.clone(), Vec::new(), Vec::new(), "motorway"));
        entry.3 = "motorway";
        entry.2.extend(
            corridor_roads
                .iter()
                .filter(|(_, name)| *name == corridor.name)
                .map(|(road, _)| road.clone()),
        );
    }
    if let Some(works) = earthworks {
        let mut counts: BTreeMap<String, usize> = BTreeMap::new();
        // Each slope is its road's corridor's: a corridor road's own, or — for a
        // crossroad the corridor fitted to the land — the corridor whose slip
        // roads meet it at a junction.
        let corridor_of = |road: &RoadId| -> Option<&String> {
            if let Some((_, name)) = corridor_roads.iter().find(|(id, _)| id == road) {
                return Some(name);
            }
            let entry = target.road(road)?;
            [&entry.link.predecessor, &entry.link.successor]
                .into_iter()
                .find_map(|link| match link {
                    Some(roadgen_core::topology::RoadLinkTarget::Junction(junction)) => {
                        corridor_roads.iter().find_map(|(id, name)| {
                            (target.road(id)?.junction.as_ref() == Some(junction)).then_some(name)
                        })
                    }
                    _ => None,
                })
        };
        let owners: Vec<Option<SiteId>> = works
            .owners
            .iter()
            .map(|road| {
                corridor_of(road)
                    .or_else(|| places.corridors.first().map(|corridor| &corridor.name))
                    .map(|name| site_of(name))
                    .transpose()
            })
            .collect::<Result<_, _>>()?;
        for ((kind, ring), site) in works.covers.into_iter().zip(owners) {
            // Draped on the land just built, so the outline sits on it.
            let count = counts.entry(kind.clone()).or_default();
            let cover_key = format!("{kind}/{count}");
            *count += 1;
            let id = match land::cover(
                target,
                LAND,
                &cover_key,
                &kind,
                ring,
                site.as_ref(),
                surface.as_ref(),
            ) {
                Ok(id) => id,
                // A slope too thin to have an outline is no land cover.
                Err(land::Error::Geometry(_)) => continue,
                Err(error) => return Err(error.into()),
            };
            report.land_covers += 1;
            if let Some(site) = &site {
                if let Some(entry) = sites.get_mut(site.local_name()) {
                    entry.1.push(id);
                }
            }
        }
    }
    for (key, (name, land_covers, roads, kind)) in sites {
        let id = SiteId::new(key);
        target
            .sites
            .insert(
                id.clone(),
                Site {
                    id,
                    name: Some(name),
                    kind: kind.into(),
                    land_covers,
                    surfaces: surface.iter().cloned().collect(),
                    roads,
                },
            )
            .map_err(|duplicate| land::Error::Duplicate(duplicate.0.to_string()))?;
        report.sites += 1;
    }
    // The separations once more, now the land covers are on the map: a bridge over
    // a river passes over water.
    if separates {
        roadgen_core::structure::derive_separations(target);
        roadgen_core::structure::refresh_interchanges(target);
    }
    Ok(report)
}
