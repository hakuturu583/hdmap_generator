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

pub mod error;
pub mod facades;
pub mod fbx;
pub mod ground;
pub mod materials;
pub mod mesh;
pub mod package;
pub mod script;
pub mod surfaces;
pub mod tags;

use std::collections::BTreeMap;
use std::fs;
use std::path::{Path, PathBuf};

use roadgen_core::semantics::MapObjectKind;
use roadgen_core::ValidatedMap;

pub use error::ExportError;
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

    // The OpenDRIVE is written by the OpenDRIVE exporter, not by this one. A CARLA
    // map is a mesh and a road network and they have to be the same road network;
    // writing a second one here would be two chances to be wrong about it.
    let xodr_path = folder.join(format!("{}.xodr", config.map));
    roadgen_opendrive::write(map, &xodr_path)
        .map_err(|error| ExportError::OpenDrive(error.to_string()))?;

    let mut props_path = None;
    let mut props = Vec::new();
    if config.buildings == BuildingPlacement::Props {
        let mut ordinals = surfaces::Ordinals::default();
        let town = surfaces::buildings(map, &config.map, &mut ordinals);
        if !town.is_empty() {
            let path = folder.join(format!("{}_Props.fbx", config.map));
            write_text(&path, &fbx::document(&town, creator()))?;
            props.push(PropEntry {
                name: format!("{}_Buildings", config.map),
                source: relative(&config.map, &format!("{}_Props.fbx", config.map)),
                size: "huge".to_owned(),
                tag: Folder::Building.as_str().to_owned(),
            });
            props_path = Some(path);
        }
    }

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
    write_text(&script_path, &script::script(config))?;
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
        &textures.join("polyhaven.json"),
        &manifest
            .to_json()
            .map_err(|error| ExportError::Json(error.to_string()))?,
    )?;
    write_text(&textures.join("CREDITS.md"), &materials::credits(&entries))?;

    let mut labels: BTreeMap<Label, usize> = BTreeMap::new();
    for mesh in &meshes {
        *labels.entry(tags::label_of(&mesh.name)).or_default() += 1;
    }

    Ok(Package {
        script: script_path,
        descriptor: descriptor_path,
        fbx: fbx_path,
        xodr: xodr_path,
        props: props_path,
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
                 Export them as props to be tagged correctly, at the cost of their \
                 not being placed",
                Label::Terrain.stencil(),
                Label::Buildings.stencil()
            )),
            BuildingPlacement::Props => warnings.push(format!(
                "{buildings} buildings are props, so they will be tagged `Buildings` \
                 ({}) — and none of them will be in the level. CARLA places what it \
                 finds in a map's four semantic folders and nothing else; a prop is \
                 imported and waits in the content browser",
                Label::Buildings.stencil()
            )),
            BuildingPlacement::Omitted => warnings.push(format!(
                "{buildings} buildings were not written: the package has roads and no \
                 town"
            )),
        }
    }

    if config.surfaces.ground_extent > 0.0 {
        warnings.push(format!(
            "the ground past the {:.0} m verge is a grid at the roads' own heights, \
             out to {:.0} m beyond the network: flat where the roads are flat and \
             sloping between roads at different heights, because a road network says \
             nothing more about the land it runs through. Hills are CARLA's editor's \
             to add",
            config.surfaces.verge_width, config.surfaces.ground_extent
        ));
    } else if config.surfaces.verge_width > 0.0 {
        warnings.push(format!(
            "the ground is a {:.0} m verge either side of each road and nothing \
             further: a road network says nothing about the shape of the land it runs \
             through, so there is no landscape here to invent one from. CARLA's \
             editor is where a map gets terrain",
            config.surfaces.verge_width
        ));
    } else {
        warnings.push(
            "there is no ground at all beside the roads: the verge is switched off, so \
             the map's roads stand on nothing"
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

    let furniture = map
        .objects
        .iter()
        .filter(|object| {
            matches!(
                object.kind,
                MapObjectKind::TrafficLight | MapObjectKind::TrafficSign { .. }
            )
        })
        .count();
    if furniture > 0 {
        warnings.push(format!(
            "{furniture} traffic lights and signs are in the .xodr and not in the FBX. \
             That is the right way round — CARLA spawns its own from the OpenDRIVE, \
             and a mesh named for one would be dropped by ValidateStaticMesh anyway — \
             but nothing in the mesh marks where they stand"
        ));
    }

    let crossings = map
        .objects
        .iter()
        .filter(|object| matches!(object.kind, MapObjectKind::Crosswalk))
        .count();
    if crossings > 0 {
        warnings.push(format!(
            "{crossings} crosswalks are in the .xodr but are not painted on the \
             surface: a crossing is an object in the IR rather than a lane boundary, \
             and this exporter paints boundaries"
        ));
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
