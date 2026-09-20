//! The folder CARLA imports, and the file that describes it.
//!
//! CARLA's `Util/Tools/Import.py` walks the `Import` directory looking for a `.json`
//! that names maps and props, and hands each one to Unreal's `ImportAssets`
//! commandlet. This is that JSON, and the folder around it.
//!
//! ```text
//!   <package>/
//!   ├── <Package>.json          what Import.py reads
//!   ├── <Package>.py            the script that takes the package the rest of the way
//!   └── <Map>/
//!       ├── <Map>.fbx           the surface
//!       ├── <Map>.xodr          the road network — same name, which CARLA insists on
//!       ├── <Map>_Buildings.fbx the town, when it is exported as props
//!       ├── <Map>_TrafficLights.fbx   the lights, as props tagged `TrafficLight`
//!       ├── <Map>_TrafficSigns.fbx    the signs, as props tagged `TrafficSign`
//!       ├── furniture.manifest  where each light and sign stands
//!       ├── map_logic.carla     CARLA's map_logic.json, copied beside the .xodr to adopt the lights
//!       └── Textures/
//!           ├── polyhaven.manifest  what to fetch, and where each file goes
//!           ├── CREDITS.md
//!           └── …               the pictures, once they have been fetched
//! ```
//!
//! The `.fbx` and the `.xodr` share a name because CARLA requires it in three
//! separate places: `Import.py` pairs them by name when it has to generate a
//! descriptor itself, it copies the `.xodr` to `Content/<Package>/Maps/<name>/
//! OpenDrive/<name>.xodr` so the level and its road network agree, and
//! `UOpenDrive::LoadXODR` finds the file by the level's own name at runtime.

use serde::Serialize;

use crate::materials;

/// The descriptor `Import.py` reads.
///
/// Both arrays are always written, empty or not: `ULoadAssetMaterialsCommandlet`
/// calls `GetArrayField` on each without checking, and a descriptor missing one is a
/// descriptor that stops the import rather than skipping a step.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct Descriptor {
    pub maps: Vec<MapEntry>,
    pub props: Vec<PropEntry>,
}

/// One map: a mesh, the road network under it, and whose materials it wears.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct MapEntry {
    /// The map's name, which is also the name of the `.fbx`, of the `.xodr`, and of
    /// the level CARLA builds.
    pub name: String,
    /// Path to the `.fbx`, relative to the descriptor.
    pub source: String,
    /// Whether CARLA replaces the materials in the FBX with its own.
    ///
    /// With this on, `UPrepareAssetsForCookingCommandlet` assigns CARLA's asphalt,
    /// lane paint, kerb, gutter, pavement and ground materials by reading each mesh's
    /// *name* — so the textures in this package are what the map looks like with it
    /// off, and what it falls back to for anything CARLA has no material for.
    pub use_carla_materials: bool,
    /// Path to the `.xodr`, relative to the descriptor.
    pub xodr: String,
}

/// One prop: a mesh imported into a tagged folder, and not placed in any map.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct PropEntry {
    pub name: String,
    pub source: String,
    /// CARLA's own size vocabulary, which is what the blueprint library reports.
    pub size: String,
    /// The folder under `/Game/<Package>/Static/` the prop is imported into — which
    /// is, through `ATagger::GetLabelByPath`, its semantic class. This is the one
    /// place in the whole pipeline where a tag can be *stated* rather than spelled
    /// into a mesh name.
    pub tag: String,
}

impl Descriptor {
    pub fn to_json(&self) -> Result<String, serde_json::Error> {
        // Three spaces, which is what `Import.py` writes when it generates one of
        // these itself. Matching it means a descriptor this crate wrote and one CARLA
        // wrote diff cleanly against each other.
        let mut text = serde_json::to_string_pretty(self)?;
        text.push('\n');
        Ok(text)
    }
}

/// The path an imported prop's mesh has inside Unreal's content tree, which is what
/// the placement script loads it by.
///
/// `Import.py` imports each prop's FBX into `/Game/<Package>/Static/<tag>/<prop>/`
/// with `bCombineMeshes` off, so every mesh node in the file becomes an asset of
/// its own — named, by `FFbxImporter::MakeNameForMesh`, after the file *and* the
/// node: `<file>_<node>`. The file is named after the prop.
pub fn prop_asset_path(package: &str, tag: &str, prop: &str, mesh: &str) -> String {
    format!("/Game/{package}/Static/{tag}/{prop}/{prop}_{mesh}")
}

/// The file name the placement manifest is written under: JSON, but not `.json`,
/// because `Import.py` takes every `.json` under `Import/` for a package.
pub const FURNITURE_MANIFEST: &str = "furniture.manifest";

/// The file name CARLA's `map_logic.json` is written under in the package, for the
/// same reason; `roadgen.carla_furniture` copies it beside the `.xodr` under the
/// name CARLA looks for.
pub const MAP_LOGIC: &str = "map_logic.carla";

/// Where each piece of furniture stands: what the package's script reads to place
/// the props in the level, since CARLA's importer places only a map's own meshes.
///
/// Coordinates are the map's own — metres, x east, y north, z up — and the
/// heading is radians anticlockwise from east. The script does the one flip
/// Unreal needs.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct FurnitureManifest {
    pub package: String,
    pub map: String,
    /// The material slots of a light's three lamps, by name, for the script to
    /// replace with materials CARLA can switch on and off.
    pub lamps: LampSlots,
    pub lights: Vec<Placement>,
    pub signs: Vec<Placement>,
    /// The town, when it was exported as props: meshes in the map's own
    /// coordinates already, so each stands at the origin, unturned.
    pub buildings: Vec<BuildingProp>,
}

/// One building, as a prop for the script to stand in the level.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct BuildingProp {
    pub name: String,
    pub asset: String,
}

#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct LampSlots {
    pub red: String,
    pub amber: String,
    pub green: String,
}

/// One placed prop.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct Placement {
    /// The mesh's name, which is the asset's and the actor's.
    pub name: String,
    /// Where the imported asset is, from [`prop_asset_path`].
    pub asset: String,
    /// A light's lamps, as an asset of their own, which is the actor CARLA turns
    /// into the light that switches; the post above stays put and tagged.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub lamps_asset: Option<String>,
    /// The IR object this stands for.
    pub object: String,
    /// The `<signal id>` it is written as in the `.xodr`.
    pub signal: String,
    /// The foot of the post.
    pub position: [f64; 3],
    pub heading: f64,
    /// The `ETrafficSignState` a marker beside a sign is put in, spelled as
    /// Unreal's Python spells the enumerator, when CARLA has one for it.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub carla_state: Option<String>,
    /// How far a light's arm reaches, for a report; zero for a sign.
    pub arm_length: f64,
}

impl FurnitureManifest {
    pub fn to_json(&self) -> Result<String, serde_json::Error> {
        let mut text = serde_json::to_string_pretty(self)?;
        text.push('\n');
        Ok(text)
    }
}

/// CARLA's `map_logic.json`, read by `UMapLogicParser` from beside the `.xodr`.
///
/// Its presence is what matters most: `ATrafficLightManager::InitializeTrafficLights`
/// spawns its own traffic lights only when there is no such file. With one, every
/// entry's `SignalID` is looked up in the `.xodr`, the actor within fifty
/// centimetres of that signal becomes the light, and `TrafficLightGroupID` names the
/// `<controller>` whose timing the entry sets.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct MapLogic {
    #[serde(rename = "TrafficLights")]
    pub traffic_lights: Vec<MapLogicLight>,
}

#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct MapLogicLight {
    #[serde(rename = "ActorName")]
    pub actor_name: String,
    #[serde(rename = "SignalID")]
    pub signal_id: String,
    /// The junction's OpenDRIVE id as a number, or -1 for a light at none.
    #[serde(rename = "JunctionID")]
    pub junction_id: i64,
    #[serde(rename = "TrafficLightGroupID")]
    pub group_id: String,
    #[serde(rename = "Timing")]
    pub timing: MapLogicTiming,
    #[serde(rename = "Modules")]
    pub modules: Vec<MapLogicModule>,
}

/// Seconds in each state. The field names are `FTrafficLightTiming`'s.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct MapLogicTiming {
    #[serde(rename = "RedDuration")]
    pub red: f64,
    #[serde(rename = "GreenDuration")]
    pub green: f64,
    #[serde(rename = "AmberDuration")]
    pub amber: f64,
    #[serde(rename = "AmberBlinkInterval")]
    pub amber_blink_interval: f64,
}

/// The OpenDRIVE lane ids one head governs.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct MapLogicModule {
    #[serde(rename = "LaneIds")]
    pub lane_ids: Vec<i64>,
}

impl MapLogic {
    pub fn to_json(&self) -> Result<String, serde_json::Error> {
        let mut text = serde_json::to_string_pretty(self)?;
        text.push('\n');
        Ok(text)
    }
}

/// The manifest `roadgen.fetch_textures()` reads.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct TextureManifest {
    /// Where the fetcher gets them from. Written into the file rather than compiled
    /// into the fetcher so that a package carries its own source.
    pub api: String,
    /// The licence every asset listed here is published under.
    pub license: String,
    pub files: Vec<materials::Entry>,
}

impl TextureManifest {
    pub fn new(files: Vec<materials::Entry>) -> TextureManifest {
        TextureManifest {
            api: "https://api.polyhaven.com".to_owned(),
            license: "CC0-1.0".to_owned(),
            files,
        }
    }

    pub fn to_json(&self) -> Result<String, serde_json::Error> {
        let mut text = serde_json::to_string_pretty(self)?;
        text.push('\n');
        Ok(text)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_descriptor_names_the_fbx_and_the_xodr_the_same() {
        // CARLA pairs them by name, copies the xodr under the level's name and loads
        // it back by that name. Three places, one rule.
        let descriptor = Descriptor {
            maps: vec![MapEntry {
                name: "Town01".into(),
                source: "./Town01/Town01.fbx".into(),
                use_carla_materials: true,
                xodr: "./Town01/Town01.xodr".into(),
            }],
            props: Vec::new(),
        };
        let json = descriptor.to_json().unwrap();
        assert!(json.contains("\"name\": \"Town01\""));
        assert!(json.contains("Town01.fbx"));
        assert!(json.contains("Town01.xodr"));
    }

    #[test]
    fn a_descriptor_always_writes_both_arrays() {
        // `GetArrayField("props")` is called without checking, so a descriptor
        // missing the key stops the import rather than skipping the props.
        let json = Descriptor {
            maps: Vec::new(),
            props: Vec::new(),
        }
        .to_json()
        .unwrap();
        assert!(json.contains("\"maps\""));
        assert!(json.contains("\"props\""));
    }

    #[test]
    fn map_logic_is_spelled_the_way_carlas_parser_reads_it() {
        // `UMapLogicParser` reads these keys by name, and a key it does not find is
        // a default it applies silently.
        let logic = MapLogic {
            traffic_lights: vec![MapLogicLight {
                actor_name: "Town01_TrafficLight_Post_0".into(),
                signal_id: "3".into(),
                junction_id: 2,
                group_id: "0".into(),
                timing: MapLogicTiming {
                    red: 2.0,
                    green: 10.0,
                    amber: 3.0,
                    amber_blink_interval: 0.25,
                },
                modules: vec![MapLogicModule { lane_ids: vec![-1] }],
            }],
        };
        let json = logic.to_json().unwrap();
        for key in [
            "\"TrafficLights\"",
            "\"ActorName\"",
            "\"SignalID\": \"3\"",
            "\"JunctionID\": 2",
            "\"TrafficLightGroupID\": \"0\"",
            "\"RedDuration\"",
            "\"GreenDuration\"",
            "\"AmberDuration\"",
            "\"AmberBlinkInterval\"",
            "\"LaneIds\"",
        ] {
            assert!(json.contains(key), "{key} missing from {json}");
        }
    }

    #[test]
    fn a_prop_asset_is_found_where_import_py_puts_it() {
        assert_eq!(
            prop_asset_path("Pkg", "TrafficLight", "Town01_TrafficLights", "Town01_TrafficLight_Post_0"),
            "/Game/Pkg/Static/TrafficLight/Town01_TrafficLights/Town01_TrafficLights_Town01_TrafficLight_Post_0"
        );
    }

    #[test]
    fn a_prop_states_its_tag_rather_than_spelling_it_into_a_name() {
        let entry = PropEntry {
            name: "Town01_Buildings".into(),
            source: "./Town01/Town01_Buildings.fbx".into(),
            size: "huge".into(),
            tag: "Building".into(),
        };
        let json = serde_json::to_string(&entry).unwrap();
        assert!(json.contains("\"tag\":\"Building\""));
    }
}
