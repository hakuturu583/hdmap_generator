//! What the meshes are made of, and where the pictures come from.
//!
//! A procedurally generated surface has no texture of its own. It has a *role* — this
//! is asphalt, this is a kerb, this is a pitched roof — and something has to turn that
//! role into pixels. The pixels here come from [Poly Haven](https://polyhaven.com),
//! which publishes scanned PBR materials under CC0, so a generated town can ship with
//! photographic surfaces and no licence attached to them.
//!
//! # Nothing is fetched while exporting
//!
//! An exporter that reached for the network would be an exporter that cannot run
//! offline, cannot run in CI and cannot run in the browser — and the demo page runs
//! this crate compiled to WebAssembly. So the split is:
//!
//! - **this crate** names the material each surface is made of, writes the FBX that
//!   references it by file name, and writes [`manifest`] — a list of exactly which
//!   Poly Haven asset, at which resolution, belongs at which path;
//! - **`roadgen.fetch_textures()`**, in Python, reads that manifest and downloads
//!   them.
//!
//! A package with no textures in it is still a complete, importable CARLA package:
//! every material carries a flat colour that stands in for its texture, and
//! [`crate::check`] counts the files that are not there yet rather than failing.
//!
//! # The slugs are defaults, not a promise
//!
//! Poly Haven's catalogue is theirs and it moves. Each entry below names the asset it
//! was written against; the fetcher reports any that the API no longer knows, and the
//! manifest it reads is JSON in the package, so swapping one is editing a file rather
//! than rebuilding this crate.
//!
//! # Paint is not a photograph
//!
//! Lane markings have no texture and are not meant to. A painted line is flat white or
//! flat yellow, its edge comes from the geometry rather than from an alpha channel,
//! and scanned paint would tile visibly down a straight line. The two marking
//! materials are colours, and CARLA replaces them outright when `use_carla_materials`
//! is on — which is the other reason not to photograph them.

use std::fmt;

/// One of Poly Haven's texture maps, as the file suffix it is published under.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Map {
    /// Base colour.
    Diffuse,
    /// Tangent-space normals, OpenGL convention — which is the one Unreal reads once
    /// its importer has flipped the green channel, and the one Poly Haven publishes
    /// as `nor_gl`.
    Normal,
    Roughness,
}

impl Map {
    /// The suffix Poly Haven puts in the file name, and the key its API lists the
    /// map under.
    pub fn suffix(self) -> &'static str {
        match self {
            Map::Diffuse => "diff",
            Map::Normal => "nor_gl",
            Map::Roughness => "rough",
        }
    }

    /// The key the Poly Haven files API lists this map under, which is not always
    /// the file suffix: the base colour is published as `diff` and listed as
    /// `Diffuse`.
    pub fn api_key(self) -> &'static str {
        match self {
            Map::Diffuse => "Diffuse",
            Map::Normal => "nor_gl",
            Map::Roughness => "Rough",
        }
    }
}

/// A Poly Haven texture, at the resolution and format a package takes it in.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Texture {
    /// The asset's identifier on polyhaven.com — the last part of its page URL.
    pub slug: &'static str,
    /// `1k`, `2k`, `4k`. 2k is the default: a road surface is seen from a metre away
    /// by a simulated camera, and 1k asphalt shows its pixels at that distance.
    pub resolution: &'static str,
    /// `jpg` or `png`. JPEG for everything, because these are photographs and a
    /// 4k PNG normal map is forty megabytes of a package.
    pub format: &'static str,
    pub maps: &'static [Map],
    /// How many metres of surface one repeat covers: the size the asset was scanned
    /// at, which Poly Haven publishes as its `dimensions`. Projected at that scale a
    /// brick is brick-sized whatever wall it is on; at any other, a wall reads as a
    /// wall of the wrong bricks — four times too big, in the first packages this
    /// wrote. Aerial ground textures are scanned over tens of metres and tile at
    /// that; up close they are the ground seen from a drone, which is what a verge
    /// is from a car.
    pub scale: f64,
}

impl Texture {
    /// The file name Poly Haven publishes one of this texture's maps under, which is
    /// also the name it takes inside the package.
    pub fn file_name(&self, map: Map) -> String {
        format!(
            "{}_{}_{}.{}",
            self.slug,
            map.suffix(),
            self.resolution,
            self.format
        )
    }

    /// Where that file sits in the package, relative to the map's own folder — which
    /// is what the FBX references, so that moving the package moves the textures with
    /// it.
    pub fn relative_path(&self, map: Map) -> String {
        format!("{}/{}", TEXTURE_DIRECTORY, self.file_name(map))
    }
}

/// Where textures live inside a map's folder.
pub const TEXTURE_DIRECTORY: &str = "Textures";

/// A material, as the FBX carries it and as CARLA reads its name.
#[derive(Debug, Clone, PartialEq)]
pub struct Material {
    /// The name of the material and of the slot it occupies.
    ///
    /// CARLA reads this, twice over. `ValidateStaticMesh` throws away any mesh
    /// carrying a material whose name holds `light` or `sign`, and
    /// `PrepareAssetsForCooking` picks its yellow lane instance for the slot whose
    /// name holds `Yellow`. Both are why these are spelled out here rather than
    /// derived from a texture's file name.
    pub name: &'static str,
    /// What the surface is, when there is no picture of it: linear RGB, 0 to 1.
    pub color: [f64; 3],
    /// How rough it is, 0 to 1. Written as the FBX's shininess, which is the only
    /// thing that format has to say it with.
    pub roughness: f64,
    /// The Poly Haven asset this material is a stand-in for, if any.
    pub texture: Option<Texture>,
}

impl fmt::Display for Material {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.name)
    }
}

/// Every map Poly Haven publishes that this exporter asks for.
const FULL: &[Map] = &[Map::Diffuse, Map::Normal, Map::Roughness];

/// The materials a package uses, in the order the rest of the crate indexes them by.
///
/// An index into this table is what a [`Mesh`](crate::mesh::Mesh) carries, and the
/// constants below name the ones the surface builder reaches for. A table rather than
/// an enum because it is data — swapping the asphalt is editing one line.
pub const MATERIALS: &[Material] = &[
    Material {
        name: "M_Road_Asphalt",
        color: [0.21, 0.21, 0.22],
        roughness: 0.85,
        texture: Some(Texture {
            slug: "asphalt_02",
            resolution: "2k",
            format: "jpg",
            maps: FULL,
            scale: 3.0,
        }),
    },
    Material {
        // No `Yellow` in the name, so CARLA's marking pass takes it for the white one.
        name: "M_RoadMarking_White",
        color: [0.90, 0.90, 0.88],
        roughness: 0.6,
        texture: None,
    },
    Material {
        // `Yellow` is load-bearing: `PrepareAssetsForCookingCommandlet` matches the
        // slot name against it to decide which of CARLA's two lane materials to use.
        name: "M_RoadMarking_Yellow",
        color: [0.85, 0.68, 0.11],
        roughness: 0.6,
        texture: None,
    },
    Material {
        name: "M_Sidewalk_Concrete",
        color: [0.55, 0.54, 0.52],
        roughness: 0.8,
        texture: Some(Texture {
            slug: "concrete_floor_worn_001",
            resolution: "2k",
            format: "jpg",
            maps: FULL,
            scale: 3.0,
        }),
    },
    Material {
        name: "M_Curb_Concrete",
        color: [0.62, 0.61, 0.59],
        roughness: 0.75,
        texture: Some(Texture {
            slug: "concrete_wall_008",
            resolution: "2k",
            format: "jpg",
            maps: FULL,
            scale: 2.71,
        }),
    },
    Material {
        name: "M_Terrain_Grass",
        color: [0.24, 0.35, 0.16],
        roughness: 0.95,
        texture: Some(Texture {
            slug: "aerial_grass_rock",
            resolution: "2k",
            format: "jpg",
            maps: FULL,
            scale: 15.0,
        }),
    },
    Material {
        name: "M_Building_Brick",
        color: [0.46, 0.25, 0.20],
        roughness: 0.85,
        texture: Some(Texture {
            slug: "red_brick_03",
            resolution: "2k",
            format: "jpg",
            maps: FULL,
            scale: 1.0,
        }),
    },
    Material {
        name: "M_Building_Plaster",
        color: [0.72, 0.70, 0.65],
        roughness: 0.8,
        texture: Some(Texture {
            slug: "painted_plaster_wall",
            resolution: "2k",
            format: "jpg",
            maps: FULL,
            scale: 2.0,
        }),
    },
    Material {
        name: "M_Building_RoofTiles",
        color: [0.38, 0.20, 0.16],
        roughness: 0.8,
        texture: Some(Texture {
            slug: "roof_tiles_14",
            resolution: "2k",
            format: "jpg",
            maps: FULL,
            scale: 1.50,
        }),
    },
    Material {
        // Glazing: a dark, slightly blue surface with the low roughness that makes it
        // read as glass under a sky. No texture — a window is a reflection, not a
        // picture.
        name: "M_Building_Glazing",
        color: [0.10, 0.13, 0.17],
        roughness: 0.15,
        texture: None,
    },
    Material {
        name: "M_Building_Frame",
        color: [0.88, 0.87, 0.84],
        roughness: 0.5,
        texture: None,
    },
    Material {
        name: "M_Building_Door",
        color: [0.28, 0.17, 0.10],
        roughness: 0.6,
        texture: None,
    },
    // Street furniture. None of it is photographed: a galvanised pole is a colour
    // and a roughness, and a lamp is a colour that the editor script turns into an
    // emissive CARLA material once the package is imported — see
    // `_unreal_furniture.py`, which finds the three lamp slots by these names.
    Material {
        name: "M_Furniture_Steel",
        color: [0.45, 0.46, 0.47],
        roughness: 0.45,
        texture: None,
    },
    Material {
        name: "M_Furniture_Housing",
        color: [0.06, 0.06, 0.06],
        roughness: 0.7,
        texture: None,
    },
    Material {
        name: "M_Lamp_Red",
        color: [0.90, 0.05, 0.05],
        roughness: 0.2,
        texture: None,
    },
    Material {
        name: "M_Lamp_Amber",
        color: [0.95, 0.55, 0.05],
        roughness: 0.2,
        texture: None,
    },
    Material {
        name: "M_Lamp_Green",
        color: [0.05, 0.80, 0.25],
        roughness: 0.2,
        texture: None,
    },
    Material {
        name: "M_Plate_Red",
        color: [0.75, 0.05, 0.05],
        roughness: 0.4,
        texture: None,
    },
    Material {
        name: "M_Plate_White",
        color: [0.92, 0.92, 0.90],
        roughness: 0.4,
        texture: None,
    },
    Material {
        name: "M_Plate_Blue",
        color: [0.05, 0.25, 0.70],
        roughness: 0.4,
        texture: None,
    },
];

pub const ASPHALT: usize = 0;
pub const MARKING_WHITE: usize = 1;
pub const MARKING_YELLOW: usize = 2;
pub const SIDEWALK: usize = 3;
pub const CURB: usize = 4;
pub const GRASS: usize = 5;
pub const BRICK: usize = 6;
pub const PLASTER: usize = 7;
pub const ROOF_TILES: usize = 8;
pub const GLAZING: usize = 9;
pub const FRAME: usize = 10;
pub const DOOR: usize = 11;
pub const STEEL: usize = 12;
pub const HOUSING: usize = 13;
pub const LAMP_RED: usize = 14;
pub const LAMP_AMBER: usize = 15;
pub const LAMP_GREEN: usize = 16;
pub const PLATE_RED: usize = 17;
pub const PLATE_WHITE: usize = 18;
pub const PLATE_BLUE: usize = 19;

/// One texture file the package expects, and where it comes from.
#[derive(Debug, Clone, PartialEq, serde::Serialize)]
pub struct Entry {
    /// Where the file goes, relative to the map's folder.
    pub path: String,
    /// The Poly Haven asset it comes from.
    pub slug: String,
    /// The key its files API lists this map under.
    pub map: String,
    pub resolution: String,
    pub format: String,
    /// Which material wants it, so a report can say what is missing a texture.
    pub material: String,
}

/// Every texture file a package made of these materials expects, once each.
///
/// Once each matters: two materials may share an asset, and a fetcher that took this
/// list literally would download it twice and a report would count it twice.
pub fn manifest(materials: &[usize]) -> Vec<Entry> {
    let mut entries: Vec<Entry> = Vec::new();
    for &index in materials {
        let Some(material) = MATERIALS.get(index) else {
            continue;
        };
        let Some(texture) = material.texture else {
            continue;
        };
        for &map in texture.maps {
            let path = texture.relative_path(map);
            if entries.iter().any(|entry| entry.path == path) {
                continue;
            }
            entries.push(Entry {
                path,
                slug: texture.slug.to_owned(),
                map: map.api_key().to_owned(),
                resolution: texture.resolution.to_owned(),
                format: texture.format.to_owned(),
                material: material.name.to_owned(),
            });
        }
    }
    entries
}

/// The credit note written beside the textures.
///
/// Poly Haven's assets are CC0, so this is a courtesy rather than a condition — but a
/// package that travels without saying where its pictures came from is a package
/// nobody can go back to for the 4k version.
pub fn credits(entries: &[Entry]) -> String {
    let mut slugs: Vec<&str> = entries.iter().map(|entry| entry.slug.as_str()).collect();
    slugs.sort_unstable();
    slugs.dedup();

    let mut text = String::from(
        "# Textures\n\n\
         These are scanned materials from [Poly Haven](https://polyhaven.com), \
         published under [CC0](https://creativecommons.org/publicdomain/zero/1.0/): \
         public domain, no attribution required. This note is here so that the \
         package says where its surfaces came from, not because it has to.\n\n\
         They are fetched rather than shipped. `roadgen.fetch_textures()` reads \
         `polyhaven.manifest` beside this file and downloads exactly what is listed \
         there; until it has run, the materials fall back to the flat colours \
         written into the FBX.\n\n",
    );
    for slug in slugs {
        text.push_str(&format!("- `{slug}` — <https://polyhaven.com/a/{slug}>\n"));
    }
    text
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_yellow_marking_is_the_only_material_carla_will_call_yellow() {
        // `PrepareAssetsForCookingCommandlet` walks a marking mesh's slots and gives
        // the yellow instance to every one whose name holds `Yellow`. Exactly one
        // material may answer to that, or white lines come out yellow.
        let yellow: Vec<&str> = MATERIALS
            .iter()
            .filter(|material| material.name.contains("Yellow"))
            .map(|material| material.name)
            .collect();
        assert_eq!(yellow, vec!["M_RoadMarking_Yellow"]);
    }

    #[test]
    fn no_material_is_named_in_a_way_that_makes_carla_drop_the_mesh() {
        // `ValidateStaticMesh` throws away a mesh whose *material* holds `light` or
        // `sign`, so a material called `M_Road_SignPaint` would silently cost the
        // road it is painted on.
        for material in MATERIALS {
            assert_eq!(
                crate::tags::is_rejected_by_import(material.name),
                None,
                "{material} would take its mesh down with it"
            );
        }
    }

    #[test]
    fn a_texture_is_asked_for_under_the_name_poly_haven_publishes_it() {
        let texture = MATERIALS[ASPHALT].texture.unwrap();
        assert_eq!(texture.file_name(Map::Diffuse), "asphalt_02_diff_2k.jpg");
        assert_eq!(texture.file_name(Map::Normal), "asphalt_02_nor_gl_2k.jpg");
        assert_eq!(
            texture.relative_path(Map::Diffuse),
            "Textures/asphalt_02_diff_2k.jpg"
        );
    }

    #[test]
    fn the_manifest_lists_each_file_once_however_many_materials_want_it() {
        let entries = manifest(&[ASPHALT, ASPHALT, GRASS]);
        let asphalt = entries
            .iter()
            .filter(|entry| entry.slug == "asphalt_02")
            .count();
        assert_eq!(asphalt, FULL.len());
        assert!(entries
            .iter()
            .any(|entry| entry.slug == "aerial_grass_rock"));
    }

    #[test]
    fn paint_asks_for_no_pictures() {
        assert!(manifest(&[MARKING_WHITE, MARKING_YELLOW]).is_empty());
    }

    #[test]
    fn the_credits_name_every_asset_once() {
        let text = credits(&manifest(&[ASPHALT, GRASS, ASPHALT]));
        assert_eq!(text.matches("polyhaven.com/a/asphalt_02").count(), 1);
        assert!(text.contains("aerial_grass_rock"));
    }
}
