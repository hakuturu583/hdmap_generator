//! `roadgen-carla` — writes the canonical IR as a CARLA UE5 asset package.
//!
//! CARLA is a driving simulator built on Unreal Engine. A map in it is two files that
//! have to agree: an **`.fbx`** holding the surface the sensors see, and an
//! **`.xodr`** holding the road network the traffic drives. This writes both, plus
//! the descriptor CARLA's importer reads and the manifest that fetches the textures.
//!
//! Like the other exporters this is a thin lowering layer: it takes a [`ValidatedMap`]
//! and pushes nothing back into the IR.
//!
//! ```text
//!         ValidatedMap
//!              │
//!      ┌───────┴────────┐
//!      ▼                ▼
//!   surfaces       roadgen-opendrive
//!      │                │
//!    meshes           .xodr
//!      │                │
//!     fbx               │
//!      └───────┬────────┘
//!              ▼
//!     <Package>.json + <Map>/
//! ```
//!
//! # A tag is a mesh name
//!
//! This is the thing worth knowing about CARLA before writing anything for it, and it
//! is not in the documentation — it is in `MoveAssetsCommandlet.cpp`.
//!
//! CARLA's semantic segmentation does not read a material, a property or a mesh. It
//! reads the **folder** the imported asset ended up in, and the import puts an asset
//! in a folder by matching its **name** against six substrings. So the name of a mesh
//! inside the FBX is its semantic ground truth, spelled at one remove, and
//! [`tags`] is that whole chain written out where it can be tested.
//!
//! Get it wrong and nothing fails. The mesh imports, the map loads, the camera
//! renders — and the pavements come back labelled as ground. [`check`] exists mostly
//! for this.
//!
//! # What the surface is made of
//!
//! Six classes, from the cross-section the generator already produced: the
//! carriageway, the paint on it, the pavements beside it, the kerb faces between them,
//! the gutters at the foot of those, and the grass beyond. See [`surfaces`].
//!
//! # What stands beside it
//!
//! Traffic lights and signs, built to the cross-section — a pole on the pavement, an
//! arm long enough to reach over the lanes it governs — and written as props with a
//! placement manifest and a `map_logic.json`, so that CARLA adopts them as its own
//! lights rather than spawning its blueprints over the middle of the road. See
//! [`furniture`], which is also where the reasons are.
//!
//! # Example
//!
//! ```no_run
//! # fn main() -> Result<(), Box<dyn std::error::Error>> {
//! # let map: roadgen_core::ValidatedMap = unimplemented!();
//! let config = roadgen_carla::PackageConfig::new("Town01");
//! let package = roadgen_carla::write(&map, "Import/", &config)?;
//! println!("{} meshes, {} textures to fetch", package.meshes, package.textures.len());
//! # Ok(())
//! # }
//! ```

pub mod crosswalks;
pub mod error;
pub mod facades;
pub mod fbx;
pub mod furniture;
pub mod ground;
pub mod materials;
pub mod mesh;
pub mod obj;
pub mod package;
pub mod script;
pub mod surfaces;
pub mod tags;
pub mod terrain;

use std::collections::BTreeMap;
use std::fs;
use std::path::{Path, PathBuf};

use roadgen_core::semantics::MapObjectKind;
use roadgen_core::ValidatedMap;

pub use error::ExportError;
pub use furniture::{FurnitureConfig, SignKind};
pub use materials::Material;
pub use mesh::Mesh;
pub use package::{Descriptor, MapEntry, PropEntry, TextureManifest};
pub use surfaces::SurfaceConfig;
pub use tags::{Folder, Label, Role};

/// Where the town goes.
///
/// The two answers are both wrong in different ways, and which is less wrong depends
/// on what the map is for — so it is a choice rather than a default nobody was told
/// about.
///
/// The reason there is a choice at all is that CARLA's map import has exactly six
/// names in it. `UMoveAssetsCommandlet` sorts a map's meshes into `Road`, `RoadLine`,
/// `SideWalk` and `Terrain`, **and nothing else**: its final `else` is `Terrain`. Then
/// `UPrepareAssetsForCookingCommandlet` places into the world exactly what it finds in
/// those four folders. A building can be in the map or correctly tagged, and the
/// import pipeline will not do both.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BuildingPlacement {
    /// In the map's own FBX: the town stands in the level, and every building
    /// segments as `Terrain` (10) rather than `Buildings` (3).
    ///
    /// The default, because a CARLA map with no town in it is not a town, and a wrong
    /// label on a building is recoverable in the editor while a missing town is a
    /// re-export.
    InMap,
    /// In an FBX of their own, declared in the descriptor's `props` with
    /// `tag: "Building"`: every building segments as `Buildings` (3), and none of them
    /// is placed in the level until someone drags them in.
    Props,
    /// Not written at all.
    Omitted,
}

/// How a map is turned into a package.
#[derive(Debug, Clone, PartialEq)]
pub struct PackageConfig {
    /// The package's name, which is the descriptor's file name and the folder CARLA
    /// imports into under `/Game/`.
    pub package: String,
    /// The map's name, which is the `.fbx`, the `.xodr`, the level and the prefix of
    /// every mesh name in the file.
    pub map: String,
    /// Whether CARLA replaces the package's materials with its own.
    ///
    /// On by default, because CARLA's road materials are built for its lighting and
    /// its sensors, and because a map that wears them looks like the maps it will be
    /// benchmarked against. Turn it off to see the textures this package ships.
    pub use_carla_materials: bool,
    pub buildings: BuildingPlacement,
    pub surfaces: SurfaceConfig,
    /// How the traffic lights and signs are built and placed, or `None` to leave
    /// them to CARLA — which spawns its own from the `.xodr`, over the lanes.
    pub furniture: Option<FurnitureConfig>,
    /// The CARLA checkout the package's script imports into, when the caller said.
    /// Otherwise the script takes it from its arguments or the environment.
    pub carla_root: Option<String>,
    /// The Unreal Engine that checkout was built against, likewise.
    pub engine: Option<String>,
    /// Where the script puts the sun, in degrees, the way CARLA's weather spells it.
    pub sun_altitude: f64,
    pub sun_azimuth: f64,
}

impl Default for PackageConfig {
    fn default() -> Self {
        PackageConfig::new("roadgen")
    }
}

impl PackageConfig {
    pub fn new(name: impl AsRef<str>) -> Self {
        let name = tags::sanitize(name.as_ref());
        PackageConfig {
            package: format!("{name}Package"),
            map: name,
            use_carla_materials: true,
            buildings: BuildingPlacement::InMap,
            surfaces: SurfaceConfig::default(),
            furniture: Some(FurnitureConfig::default()),
            carla_root: None,
            engine: None,
            sun_altitude: 45.0,
            sun_azimuth: -50.0,
        }
    }

    /// The configuration a map exports under when the caller names none: the map's
    /// own name, or `roadgen` for a map that has none either.
    pub fn for_map(map: &ValidatedMap) -> Self {
        match map.metadata.name.as_deref() {
            Some(name) if !name.trim().is_empty() => PackageConfig::new(name),
            _ => PackageConfig::default(),
        }
    }

    pub fn with_package(mut self, package: impl AsRef<str>) -> Self {
        self.package = tags::sanitize(package.as_ref());
        self
    }

    pub fn with_buildings(mut self, buildings: BuildingPlacement) -> Self {
        self.buildings = buildings;
        self
    }

    pub fn with_carla_materials(mut self, use_carla_materials: bool) -> Self {
        self.use_carla_materials = use_carla_materials;
        self
    }

    pub fn with_furniture(mut self, furniture: Option<FurnitureConfig>) -> Self {
        self.furniture = furniture;
        self
    }

    /// Bakes the CARLA checkout and its engine into the package's script.
    pub fn with_carla(mut self, carla_root: impl Into<String>, engine: impl Into<String>) -> Self {
        self.carla_root = Some(carla_root.into());
        self.engine = Some(engine.into());
        self
    }
}

/// What a package turned out to be.
#[derive(Debug, Clone, PartialEq)]
pub struct Package {
    /// The descriptor, which is the file CARLA's importer is pointed at.
    pub descriptor: PathBuf,
    pub fbx: PathBuf,
    pub xodr: PathBuf,
    /// The props FBX, when the town went into one.
    pub props: Option<PathBuf>,
    /// The lights' and the signs' FBX, when there were any.
    pub lights_fbx: Option<PathBuf>,
    pub signs_fbx: Option<PathBuf>,
    /// The placement manifest the package's script reads, when there was any
    /// furniture to place.
    pub furniture: Option<PathBuf>,
    /// CARLA's `map_logic.json`, written beside the `.xodr` when there are lights.
    pub map_logic: Option<PathBuf>,
    /// How many traffic lights and signs were built.
    pub lights: usize,
    pub signs: usize,
    /// How many meshes the map's FBX holds.
    pub meshes: usize,
    pub triangles: usize,
    /// How many meshes will carry each semantic class, worked out the way CARLA works
    /// it out — by running the mesh names back through its own classifier.
    pub labels: BTreeMap<Label, usize>,
    /// The texture files the package expects and does not yet have.
    pub textures: Vec<materials::Entry>,
    /// The script that imports the package into CARLA and gives it a sky.
    pub script: PathBuf,
    /// The `.obj` CARLA's navigation builder makes the pedestrians' mesh from.
    pub obj: PathBuf,
}

/// Builds the meshes a map's FBX is made of, without writing anything.
pub fn to_meshes(map: &ValidatedMap, config: &PackageConfig) -> Vec<Mesh> {
    let mut meshes = surfaces::build(map, &config.map, &config.surfaces);
    if config.buildings == BuildingPlacement::InMap {
        let mut ordinals = surfaces::Ordinals::default();
        meshes.extend(surfaces::buildings(map, &config.map, &mut ordinals));
    }
    meshes
}

/// Writes the package into `directory`, creating it if it is not there.
pub fn write(
    map: &ValidatedMap,
    directory: impl AsRef<Path>,
    config: &PackageConfig,
) -> Result<Package, ExportError> {
    if config.map.is_empty() {
        return Err(ExportError::Name("the map has no name".into()));
    }
    let root = directory.as_ref();
    let folder = root.join(&config.map);
    let textures = folder.join(materials::TEXTURE_DIRECTORY);
    fs::create_dir_all(&textures).map_err(|error| ExportError::Io(error.to_string()))?;

    let meshes = to_meshes(map, config);
    let fbx_path = folder.join(format!("{}.fbx", config.map));
    write_text(&fbx_path, &fbx::document(&meshes, creator()))?;

    // The furniture before the OpenDRIVE, because the OpenDRIVE is told where it
    // stands: a signal written anywhere but at the foot of its pole is a signal
    // CARLA puts a pole of its own under.
    let furniture = config
        .furniture
        .as_ref()
        .map(|settings| furniture::build(map, &config.map, &config.surfaces, settings))
        .unwrap_or_default();
    let mut options = roadgen_opendrive::Options::default();
    for placed in furniture.iter() {
        options
            .signals
            .insert(placed.object.clone(), placed.signal.clone());
    }

    // The OpenDRIVE is written by the OpenDRIVE exporter, not by this one. A CARLA
    // map is a mesh and a road network and they have to be the same road network;
    // writing a second one here would be two chances to be wrong about it.
    let xodr_path = folder.join(format!("{}.xodr", config.map));
    roadgen_opendrive::write_with(map, &xodr_path, &options)
        .map_err(|error| ExportError::OpenDrive(error.to_string()))?;

    let mut props_path = None;
    let mut props = Vec::new();
    let mut town = Vec::new();
    if config.buildings == BuildingPlacement::Props {
        let mut ordinals = surfaces::Ordinals::default();
        town = surfaces::buildings(map, &config.map, &mut ordinals);
        if !town.is_empty() {
            // Named after the prop, because the import names every mesh in the
            // file `<file>_<node>` and the manifest has to say where each landed.
            let prop = format!("{}_Buildings", config.map);
            let file = format!("{prop}.fbx");
            let path = folder.join(&file);
            write_text(&path, &fbx::document(&town, creator()))?;
            props.push(PropEntry {
                name: prop,
                source: relative(&config.map, &file),
                size: "huge".to_owned(),
                tag: Folder::Building.as_str().to_owned(),
            });
            props_path = Some(path);
        }
    }
    let mut lights_fbx = None;
    let mut signs_fbx = None;
    let mut furniture_path = None;
    let mut map_logic_path = None;
    if !furniture.is_empty() || !town.is_empty() {
        let written = write_furniture(map, &folder, config, &furniture, &town, &mut props)?;
        lights_fbx = written.lights_fbx;
        signs_fbx = written.signs_fbx;
        furniture_path = Some(written.manifest);
        map_logic_path = written.map_logic;
    }

    // The same surfaces once more, for the pedestrians' navigation mesh — with the
    // town, wherever it went, since a pedestrian walks round a building whether it
    // is a prop or part of the map.
    let obj_path = folder.join(format!("{}.obj", config.map));
    let walkable: Vec<Mesh> = meshes.iter().chain(town.iter()).cloned().collect();
    write_text(&obj_path, &obj::document(&walkable, map))?;

    let descriptor = Descriptor {
        maps: vec![MapEntry {
            name: config.map.clone(),
            source: relative(&config.map, &format!("{}.fbx", config.map)),
            use_carla_materials: config.use_carla_materials,
            xodr: relative(&config.map, &format!("{}.xodr", config.map)),
        }],
        props,
    };
    let descriptor_path = root.join(format!("{}.json", config.package));
    write_text(
        &descriptor_path,
        &descriptor
            .to_json()
            .map_err(|error| ExportError::Json(error.to_string()))?,
    )?;
    let script_path = root.join(format!("{}.py", config.package));
    write_text(
        &script_path,
        &script::script(config, !furniture.is_empty() || !town.is_empty()),
    )?;
    make_executable(&script_path)?;

    let wanted: Vec<usize> = {
        let mut wanted: Vec<usize> = meshes
            .iter()
            .flat_map(|mesh| mesh.materials.iter().copied())
            .collect();
        wanted.sort_unstable();
        wanted.dedup();
        wanted
    };
    let entries = materials::manifest(&wanted);
    let manifest = TextureManifest::new(entries.clone());
    write_text(
        &textures.join("polyhaven.manifest"),
        &manifest
            .to_json()
            .map_err(|error| ExportError::Json(error.to_string()))?,
    )?;
    write_text(&textures.join("CREDITS.md"), &materials::credits(&entries))?;

    let mut labels: BTreeMap<Label, usize> = BTreeMap::new();
    for mesh in &meshes {
        *labels.entry(tags::label_of(&mesh.name)).or_default() += 1;
    }

    for placed in furniture.iter() {
        *labels
            .entry(placed.role.intended_folder().label())
            .or_default() += 1;
    }

    Ok(Package {
        script: script_path,
        obj: obj_path,
        descriptor: descriptor_path,
        fbx: fbx_path,
        xodr: xodr_path,
        props: props_path,
        lights_fbx,
        signs_fbx,
        furniture: furniture_path,
        map_logic: map_logic_path,
        lights: furniture.lights.len(),
        signs: furniture.signs.len(),
        meshes: meshes.len(),
        triangles: meshes.iter().map(Mesh::triangle_count).sum(),
        labels,
        // Only the ones that are not there. A package exported twice into the same
        // directory keeps the pictures it already fetched.
        textures: entries
            .into_iter()
            .filter(|entry| !folder.join(&entry.path).exists())
            .collect(),
    })
}

/// What writing the furniture produced.
struct WrittenFurniture {
    lights_fbx: Option<PathBuf>,
    signs_fbx: Option<PathBuf>,
    manifest: PathBuf,
    map_logic: Option<PathBuf>,
}

/// Writes the lights and the signs as props, the manifest that places them and
/// the `map_logic.json` that makes CARLA adopt the lights.
fn write_furniture(
    map: &ValidatedMap,
    folder: &Path,
    config: &PackageConfig,
    furniture: &furniture::Furniture,
    town: &[Mesh],
    props: &mut Vec<PropEntry>,
) -> Result<WrittenFurniture, ExportError> {
    let settings = config.furniture.unwrap_or_default();
    let mut placements: [Vec<package::Placement>; 2] = [Vec::new(), Vec::new()];
    let mut paths = [None, None];
    for (index, (placed, role)) in [
        (&furniture.lights, Role::TrafficLight),
        (&furniture.signs, Role::TrafficSign),
    ]
    .into_iter()
    .enumerate()
    {
        if placed.is_empty() {
            continue;
        }
        let tag = role.intended_folder();
        let prop = format!("{}_{}s", config.map, tag.as_str());
        let file = format!("{prop}.fbx");
        let meshes: Vec<Mesh> = placed
            .iter()
            .flat_map(|placed| std::iter::once(&placed.mesh).chain(placed.lamps.as_ref()))
            .cloned()
            .collect();
        let path = folder.join(&file);
        write_text(&path, &fbx::document(&meshes, creator()))?;
        props.push(PropEntry {
            name: prop.clone(),
            source: relative(&config.map, &file),
            size: "medium".to_owned(),
            tag: tag.as_str().to_owned(),
        });
        for placed in placed {
            placements[index].push(package::Placement {
                name: placed.mesh.name.clone(),
                asset: package::prop_asset_path(
                    &config.package,
                    tag.as_str(),
                    &prop,
                    &placed.mesh.name,
                ),
                lamps_asset: placed.lamps.as_ref().map(|lamps| {
                    package::prop_asset_path(&config.package, tag.as_str(), &prop, &lamps.name)
                }),
                object: placed.object.to_string(),
                signal: roadgen_opendrive::signal_id(map, &placed.object).unwrap_or_default(),
                position: [placed.position.x, placed.position.y, placed.position.z],
                heading: placed.heading,
                carla_state: match placed.role {
                    Role::TrafficSign => placed.kind.carla_state().map(str::to_owned),
                    _ => None,
                },
                arm_length: placed.arm_length,
            });
        }
        paths[index] = Some(path);
    }
    let [lights, signs] = placements;
    let [lights_fbx, signs_fbx] = paths;

    let manifest = package::FurnitureManifest {
        package: config.package.clone(),
        map: config.map.clone(),
        lamps: package::LampSlots {
            red: materials::MATERIALS[materials::LAMP_RED].name.to_owned(),
            amber: materials::MATERIALS[materials::LAMP_AMBER].name.to_owned(),
            green: materials::MATERIALS[materials::LAMP_GREEN].name.to_owned(),
        },
        lights,
        signs,
        buildings: town
            .iter()
            .map(|mesh| package::BuildingProp {
                name: mesh.name.clone(),
                asset: package::prop_asset_path(
                    &config.package,
                    Folder::Building.as_str(),
                    &format!("{}_Buildings", config.map),
                    &mesh.name,
                ),
            })
            .collect(),
    };
    // Neither of these can end in `.json`: `Import.py` takes every `.json` under
    // `Import/` for a package descriptor and runs the import commandlets on it.
    // CARLA wants the second as `map_logic.json`, but beside the `.xodr` in its
    // content tree, which is where `roadgen.carla_furniture` copies it to.
    let manifest_path = folder.join(package::FURNITURE_MANIFEST);
    write_text(
        &manifest_path,
        &manifest
            .to_json()
            .map_err(|error| ExportError::Json(error.to_string()))?,
    )?;

    let mut map_logic = None;
    if !furniture.lights.is_empty() {
        let groups = roadgen_opendrive::signal_groups(map.as_map());
        let mut entries = Vec::with_capacity(furniture.lights.len());
        for placed in &furniture.lights {
            let group = groups
                .iter()
                .find(|group| group.lights.contains(&placed.object));
            let junction_id = group
                .and_then(|group| group.junction.as_ref())
                .and_then(|junction| roadgen_opendrive::junction_id(map, junction))
                .and_then(|id| id.parse::<i64>().ok())
                .unwrap_or(-1);
            let lane_ids = placed
                .lanes
                .iter()
                .filter_map(|lane| roadgen_opendrive::lane_id(map, lane))
                .map(|(_, lane)| lane)
                .collect();
            entries.push(package::MapLogicLight {
                actor_name: placed.mesh.name.clone(),
                signal_id: roadgen_opendrive::signal_id(map, &placed.object).unwrap_or_default(),
                junction_id,
                group_id: group.map(|group| group.id.clone()).unwrap_or_default(),
                timing: package::MapLogicTiming {
                    red: settings.timing.red,
                    green: settings.timing.green,
                    amber: settings.timing.amber,
                    amber_blink_interval: 0.25,
                },
                modules: vec![package::MapLogicModule { lane_ids }],
            });
        }
        let path = folder.join(package::MAP_LOGIC);
        write_text(
            &path,
            &package::MapLogic {
                traffic_lights: entries,
            }
            .to_json()
            .map_err(|error| ExportError::Json(error.to_string()))?,
        )?;
        map_logic = Some(path);
    }

    Ok(WrittenFurniture {
        lights_fbx,
        signs_fbx,
        manifest: manifest_path,
        map_logic,
    })
}

fn creator() -> &'static str {
    concat!("roadgen ", env!("CARGO_PKG_VERSION"))
}

/// A path inside the package, the way the descriptor spells one.
fn relative(map: &str, file: &str) -> String {
    format!("./{map}/{file}")
}

/// Marks a written script runnable, where the file system has such a thing.
fn make_executable(path: &Path) -> Result<(), ExportError> {
    #[cfg(unix)]
    {
        use std::os::unix::fs::PermissionsExt;
        let mut permissions = fs::metadata(path)
            .map_err(|error| ExportError::Io(error.to_string()))?
            .permissions();
        permissions.set_mode(permissions.mode() | 0o111);
        fs::set_permissions(path, permissions)
            .map_err(|error| ExportError::Io(error.to_string()))?;
    }
    #[cfg(not(unix))]
    let _ = path;
    Ok(())
}

fn write_text(path: &Path, text: &str) -> Result<(), ExportError> {
    fs::write(path, text).map_err(|error| ExportError::Io(format!("{}: {error}", path.display())))
}

/// What a CARLA package cannot carry, and what it will get wrong quietly.
///
/// Every other exporter's `check` lists what its format drops. This one lists that
/// too, but most of it is about the *import*: CARLA's pipeline classifies by name, and
/// the ways that goes wrong are silent by construction. A map whose pavements come
/// back labelled as ground renders correctly, drives correctly, and is only wrong in
/// the one output anybody wanted semantic classes for.
pub fn check(map: &ValidatedMap, config: &PackageConfig) -> Vec<String> {
    let mut warnings = Vec::new();

    if let Some(token) = tags::is_safe_map_name(&config.map) {
        warnings.push(format!(
            "the map is called `{}`, which holds `{token}`: every mesh in the FBX is \
             named after the map and CARLA classifies a mesh by looking for that word \
             in its name, so the whole map would be tagged as though it were one \
             thing. Rename the map",
            config.map
        ));
    }

    let meshes = to_meshes(map, config);
    if meshes.is_empty() {
        warnings.push(
            "the map produced no surface at all: a CARLA package needs a mesh, and \
             an .xodr on its own imports as an empty level"
                .into(),
        );
    }

    let mut rejected = 0;
    let mut mistagged: BTreeMap<(Role, Label), usize> = BTreeMap::new();
    for mesh in &meshes {
        if tags::is_rejected_by_import(&mesh.name).is_some() {
            rejected += 1;
        }
        let landed = tags::label_of(&mesh.name);
        if landed != mesh.role.intended_folder().label() {
            *mistagged.entry((mesh.role, landed)).or_default() += 1;
        }
    }
    if rejected > 0 {
        warnings.push(format!(
            "{rejected} mesh names hold `light` or `sign`; CARLA's ValidateStaticMesh \
             drops any mesh whose name or material holds either word, before it is \
             placed, without logging it. Those meshes will not be in the map"
        ));
    }
    for ((role, landed), count) in mistagged {
        if role == Role::Building {
            continue; // Said properly, with its remedy, below.
        }
        warnings.push(format!(
            "{count} {} meshes will be tagged `{landed}` ({}) rather than `{}`",
            role.as_str(),
            landed.stencil(),
            role.intended_folder().label()
        ));
    }

    let buildings = map.buildings.len();
    if buildings > 0 {
        match config.buildings {
            BuildingPlacement::InMap => warnings.push(format!(
                "{buildings} buildings are in the map's FBX, so they will stand in the \
                 level — and CARLA will tag every one of them `Terrain` ({}) rather \
                 than `Buildings` ({}). Its MoveAssets commandlet knows six mesh names, \
                 none of them a building, and sorts everything else into Terrain. \
                 Export them as props to be tagged correctly and stood in the level \
                 by the package's script instead",
                Label::Terrain.stencil(),
                Label::Buildings.stencil()
            )),
            BuildingPlacement::Props => warnings.push(format!(
                "{buildings} buildings are props, tagged `Buildings` ({}) and stood in \
                 the level by the package's script rather than by CARLA's importer, \
                 which places what it finds in a map's four semantic folders and \
                 nothing else: until `{}.py` has run `roadgen.carla_furniture`, the \
                 town waits in the content browser",
                Label::Buildings.stencil(),
                config.package
            )),
            BuildingPlacement::Omitted => warnings.push(format!(
                "{buildings} buildings were not written: the package has roads and no \
                 town"
            )),
        }
    }

    if config.surfaces.ground_extent > 0.0 {
        warnings.push(format!(
            "the land past the {:.0} m verge is at the roads' own heights, out to \
             {:.0} m beyond the network: flat where the roads are flat and sloping \
             between roads at different heights, because a road network says nothing \
             more about the land it runs through. Hills are CARLA's editor's to add",
            config.surfaces.verge_width, config.surfaces.ground_extent
        ));
    } else if config.surfaces.verge_width > 0.0 {
        warnings.push(format!(
            "the land is a {:.0} m verge either side of each road and nothing \
             further: a road network says nothing about the shape of the land it runs \
             through, so there is no landscape here to invent one from. CARLA's \
             editor is where a map gets terrain",
            config.surfaces.verge_width
        ));
    } else {
        warnings.push(
            "there is no ground at all past the roads: the verge and the ground are \
             both switched off, so the land stops at the network's outermost edges \
             and a vehicle that leaves a road there drives off the map"
                .into(),
        );
    }

    let connectors = map.roads.iter().filter(|road| road.is_connector()).count();
    if connectors > 0 {
        warnings.push(format!(
            "{connectors} junction connectors each carry their own surface, so the \
             carriageway inside a junction is written once per movement through it. \
             The overlap is coplanar and will z-fight; CARLA's own maps have a single \
             junction surface, which a road network has no way to describe"
        ));
    }

    let lights = map
        .objects
        .iter()
        .filter(|object| object.kind == MapObjectKind::TrafficLight)
        .count();
    let signs = map
        .objects
        .iter()
        .filter(|object| matches!(object.kind, MapObjectKind::TrafficSign { .. }))
        .count();
    match &config.furniture {
        None if lights + signs > 0 => warnings.push(format!(
            "{lights} traffic lights and {signs} signs are in the .xodr and not in the \
             FBX, so CARLA spawns its own: a BP_TLOpenDrive at every light's signal \
             and a blueprint at every stop, yield and speed-limit sign's, each standing \
             where the signal is — over the middle of the lane, for a signal written \
             where the IR put it. Export the furniture to put poles on the pavement \
             instead"
        )),
        Some(settings) if lights + signs > 0 => {
            let built = furniture::build(map, &config.map, &config.surfaces, settings);
            if built.lights.len() + built.signs.len() < lights + signs {
                warnings.push(format!(
                    "{} of the {} lights and signs could not be placed — each governs \
                     no lane of a road with a carriageway to stand beside — and stay \
                     as signals CARLA spawns its own furniture for",
                    lights + signs - built.lights.len() - built.signs.len(),
                    lights + signs
                ));
            }
            if !built.is_empty() {
                warnings.push(format!(
                    "{} lights and {} signs are props, placed by the package's script \
                     rather than by CARLA's importer: until `{}.py` has run \
                     `roadgen.carla_furniture`, the level has their signals and none \
                     of their meshes",
                    built.lights.len(),
                    built.signs.len(),
                    config.package
                ));
            }
            let groups = roadgen_opendrive::signal_groups(map);
            let loose = groups
                .iter()
                .filter(|group| group.junction.is_none())
                .map(|group| group.lights.len())
                .sum::<usize>();
            if loose > 0 {
                warnings.push(format!(
                    "{loose} lights govern lanes that lead into no junction, so their \
                     controllers belong to none: CARLA runs each on its own and logs an \
                     error for the timing in map_logic.json it cannot apply"
                ));
            }
            let unknown = built
                .signs
                .iter()
                .filter(|placed| placed.kind == SignKind::Other)
                .count();
            if unknown > 0 {
                warnings.push(format!(
                    "{unknown} signs have codes CARLA has no meaning for and are written \
                     with them verbatim: they stand, segment as signs, and govern \
                     nothing. CARLA knows stop, yield and a speed limit — `stop`, \
                     `de205`, `de274-50`, `speed_limit_50`"
                ));
            }
            let doubled = built
                .signs
                .iter()
                .filter(|placed| placed.kind.doubled_by_carla())
                .count();
            if doubled > 0 {
                warnings.push(format!(
                    "{doubled} 110 km/h signs will get CARLA's own plate spawned beside \
                     them: it has a model for that limit and no sign state to match a \
                     placed one against"
                ));
            }
            let long = built
                .lights
                .iter()
                .filter(|placed| placed.arm_length > 12.0)
                .count();
            if long > 0 {
                warnings.push(format!(
                    "{long} lights have mast arms longer than 12 m, because the lane they \
                     govern is that far from the nearest pavement; a real one would be \
                     hung from a gantry or a second pole"
                ));
            }
        }
        _ => {}
    }

    if materials::manifest(&[materials::ASPHALT])
        .iter()
        .any(|entry| entry.map == "Rough")
    {
        warnings.push(
            "roughness maps are fetched into the package but not wired into the FBX: \
             the format's material model is Phong, which has a shininess exponent and \
             no roughness map. They are there for a material rebuilt in the editor"
                .into(),
        );
    }

    warnings
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_package_is_named_after_the_map_it_holds() {
        let config = PackageConfig::new("High Street");
        assert_eq!(config.map, "High_Street");
        assert_eq!(config.package, "High_StreetPackage");
    }

    #[test]
    fn a_map_name_that_would_ruin_the_tagging_is_reported_rather_than_rewritten() {
        // Rewriting it would change the name of the level, which is the name a script
        // loads the map by. Saying so is the caller's decision to make.
        let config = PackageConfig::new("Terrain Town");
        assert_eq!(config.map, "Terrain_Town");
        assert!(tags::is_safe_map_name(&config.map).is_some());
    }

    #[test]
    fn a_path_in_the_descriptor_is_relative_to_it() {
        // An absolute path would be this machine's, and a package is a thing that
        // gets copied into someone else's `Import` folder.
        assert_eq!(relative("Town01", "Town01.fbx"), "./Town01/Town01.fbx");
    }
}
