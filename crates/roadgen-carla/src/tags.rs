//! What CARLA calls the things in a map, and how it works out which is which.
//!
//! CARLA's semantic segmentation camera does not read a mesh, a material or a
//! property. It reads a **stencil value**, and that value comes from the *folder the
//! asset sits in* inside Unreal's content tree. `ATagger::GetLabelByPath` splits an
//! asset's path and reads one component out of it:
//!
//! ```text
//!   /Game/<Package>/Static/<Folder>/<Map>/<Mesh>
//!    0     1         2       3        4      5
//!                            ▲
//!                            └── the tag, via GetLabelByFolderName
//! ```
//!
//! Nothing about the file we write reaches that path directly. What reaches it is
//! the **mesh name**: `UMoveAssetsCommandlet` walks every static mesh imported from a
//! map's FBX, matches its name against a short list of substrings, and moves it into
//! the matching folder. So the name of a mesh inside the FBX *is* its semantic class,
//! spelled indirectly, and getting a character wrong silently changes what a
//! segmentation camera reports.
//!
//! This module is that classifier, written out where it can be read, tested and
//! reported on. Everything in it is derived from CARLA's own source — `SSTags.h`,
//! `MoveAssetsCommandlet.cpp`, `Tagger.cpp` and `ObjectLabel.h` on the `ue5-dev`
//! branch — and none of it is reproduced from that source: what is here is the names
//! two programs must agree on to exchange a map.
//!
//! # The name grammar
//!
//! RoadRunner's, which is what CARLA was built against:
//!
//! ```text
//!   <mapName>_<meshType>_<meshSubtype>_<layer>
//!      │          │            │          │
//!      │          │            │          └── an ordinal, to keep names unique
//!      │          │            └── Road, Marking, Sidewalk, Curb, Gutter, Grass
//!      │          └── Road or Terrain
//!      └── the map's own name
//! ```
//!
//! # Three traps
//!
//! Reading the commandlets rather than the documentation turns up three things that
//! will quietly ruin a map, and [`check`](crate::check) reports all three.
//!
//! **The match is `Contains`, in a fixed order.** `Terrain` is tested before
//! `Road_Sidewalk`, and it is tested bare — so a *map* called `TerrainTown` puts
//! every one of its meshes in the Terrain folder, pavements and all, because every
//! mesh name starts with the map's name. See [`is_safe_map_name`].
//!
//! **Anything unrecognised becomes Terrain.** The final `else` of the classifier is
//! not "leave it alone", it is `Terrain`. A mesh whose name has a typo in it does not
//! fail to import; it segments as ground.
//!
//! **`light` and `sign` are rejected outright.** `ValidateStaticMesh` drops any mesh
//! whose name — or whose *material's* name — contains either word, case-insensitively,
//! before it is ever placed in the world. A street called `Sign Street` takes its
//! whole road surface with it. See [`is_rejected_by_import`].

use std::fmt;

/// A CARLA semantic class, with the stencil value its segmentation camera writes.
///
/// Only the classes this exporter can actually produce are listed. CARLA's own
/// enumeration has thirty; a generated road network is not going to emit a rider or
/// a rail track, and an enum that could name one would be claiming otherwise.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum Label {
    /// Nothing recognised it. Never produced by a folder CARLA knows.
    None,
    Roads,
    Sidewalks,
    Buildings,
    Fences,
    Vegetation,
    Terrain,
    RoadLines,
    Ground,
}

impl Label {
    /// The stencil value `carla::rpc::CityObjectLabel` gives this class, which is
    /// what a segmentation image's red channel holds.
    pub fn stencil(self) -> u8 {
        match self {
            Label::None => 0,
            Label::Roads => 1,
            Label::Sidewalks => 2,
            Label::Buildings => 3,
            Label::Fences => 5,
            Label::Vegetation => 9,
            Label::Terrain => 10,
            Label::RoadLines => 24,
            Label::Ground => 25,
        }
    }

    /// The name `ATagger::GetTagAsString` gives this class, which is what ends up in
    /// an actor's component tags.
    pub fn as_str(self) -> &'static str {
        match self {
            Label::None => "None",
            Label::Roads => "Roads",
            Label::Sidewalks => "Sidewalks",
            Label::Buildings => "Buildings",
            Label::Fences => "Fences",
            Label::Vegetation => "Vegetation",
            Label::Terrain => "Terrain",
            Label::RoadLines => "RoadLines",
            Label::Ground => "Ground",
        }
    }
}

impl fmt::Display for Label {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.as_str())
    }
}

/// A folder under `/Game/<Package>/Static/`, which is where a tag comes from.
///
/// These are the spellings `ATagger::GetLabelByFolderName` compares against, and they
/// are not the spellings [`Label`] uses: the folder is `SideWalk`, the tag is
/// `Sidewalks`, and CARLA is the only reason either is what it is.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum Folder {
    Road,
    RoadLine,
    SideWalk,
    Terrain,
    Building,
}

impl Folder {
    pub fn as_str(self) -> &'static str {
        match self {
            Folder::Road => "Road",
            Folder::RoadLine => "RoadLine",
            Folder::SideWalk => "SideWalk",
            Folder::Terrain => "Terrain",
            Folder::Building => "Building",
        }
    }

    /// What `GetLabelByFolderName` makes of this folder.
    pub fn label(self) -> Label {
        match self {
            Folder::Road => Label::Roads,
            Folder::RoadLine => Label::RoadLines,
            Folder::SideWalk => Label::Sidewalks,
            Folder::Terrain => Label::Terrain,
            Folder::Building => Label::Buildings,
        }
    }

    /// The four folders `UMoveAssetsCommandlet` will move a *map's* meshes into, and
    /// therefore the four `UPrepareAssetsForCookingCommandlet` spawns into the world.
    ///
    /// [`Folder::Building`] is not among them, which is the whole of the difficulty
    /// described on [`crate::BuildingPlacement`].
    pub fn from_a_map() -> [Folder; 4] {
        [
            Folder::Road,
            Folder::RoadLine,
            Folder::SideWalk,
            Folder::Terrain,
        ]
    }
}

impl fmt::Display for Folder {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.as_str())
    }
}

/// What a mesh is, in this exporter's own terms.
///
/// A role is not a folder and not a tag. It is the thing the generator made — a lane
/// surface, a painted line, a kerb — and the two CARLA words for it fall out of the
/// name it is given.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum Role {
    /// The drivable surface: every lane of a cross-section, as one ribbon.
    Road,
    /// A painted lane boundary, laid just above the road surface.
    Marking,
    /// A pavement: a sidewalk lane's surface, raised by the kerb height.
    Sidewalk,
    /// The vertical face between the road surface and the pavement above it.
    Curb,
    /// The strip of road surface at the foot of a kerb.
    Gutter,
    /// The land: the verges beside the roads and the ground past them, out to
    /// where a lidar reaches, as one surface.
    Terrain,
    /// A part of a building.
    Building,
}

impl Role {
    /// The `meshType` and `meshSubtype` halves of the RoadRunner name.
    ///
    /// `Terrain` is the one role whose type and subtype are the same word, because
    /// the classifier tests it bare rather than as a pair.
    pub fn name_parts(self) -> (&'static str, &'static str) {
        match self {
            Role::Road => ("Road", "Road"),
            Role::Marking => ("Road", "Marking"),
            Role::Sidewalk => ("Road", "Sidewalk"),
            Role::Curb => ("Road", "Curb"),
            Role::Gutter => ("Road", "Gutter"),
            Role::Terrain => ("Terrain", "Ground"),
            Role::Building => ("Building", "Part"),
        }
    }

    /// Where a mesh of this role is meant to end up.
    ///
    /// What it *will* end up as, once CARLA has classified the name, is
    /// [`folder_of`] — and the two differ for exactly one role, which is why they
    /// are two functions.
    pub fn intended_folder(self) -> Folder {
        match self {
            Role::Road => Folder::Road,
            Role::Marking => Folder::RoadLine,
            Role::Sidewalk | Role::Curb | Role::Gutter => Folder::SideWalk,
            Role::Terrain => Folder::Terrain,
            Role::Building => Folder::Building,
        }
    }

    pub fn as_str(self) -> &'static str {
        match self {
            Role::Road => "road",
            Role::Marking => "marking",
            Role::Sidewalk => "sidewalk",
            Role::Curb => "curb",
            Role::Gutter => "gutter",
            Role::Terrain => "terrain",
            Role::Building => "building",
        }
    }
}

/// The name a mesh of this role takes inside a map's FBX.
///
/// `<map>_<type>_<subtype>_<ordinal>`, which is the RoadRunner grammar CARLA's
/// classifier was written against. The ordinal is what keeps two lane markings on the
/// same road from colliding; Unreal would rename the second one on import, and a
/// renamed mesh is one the map's own report can no longer account for.
pub fn mesh_name(map: &str, role: Role, ordinal: usize) -> String {
    let (kind, subtype) = role.name_parts();
    format!("{map}_{kind}_{subtype}_{ordinal}")
}

/// Which folder CARLA will move a mesh of this name into.
///
/// This is `UMoveAssetsCommandlet::MoveAssetsFromMapForSemanticSegmentation`, tested
/// in its own order with its own `Contains`. The order is the specification: `Terrain`
/// is tested third and bare, so it wins over `Road_Sidewalk`, `Road_Curb` and
/// `Road_Gutter` for any name that happens to hold the word.
///
/// The final arm is not a failure. A name CARLA does not recognise becomes ground.
pub fn folder_of(mesh_name: &str) -> Folder {
    // CARLA's `FString::Contains` is case-insensitive by default, and every call in
    // the classifier leaves the default in place.
    let name = mesh_name.to_ascii_lowercase();
    let has = |needle: &str| name.contains(&needle.to_ascii_lowercase());

    if has("Road_Road") || has("Roads_Road") {
        Folder::Road
    } else if has("Road_Marking") || has("Roads_Marking") {
        Folder::RoadLine
    } else if has("Terrain") {
        Folder::Terrain
    } else if has("Road_Sidewalk")
        || has("Roads_Sidewalk")
        || has("Road_Curb")
        || has("Roads_Curb")
        || has("Road_Gutter")
        || has("Roads_Gutter")
    {
        // Three separate arms in CARLA — SIDEWALK, CURB and GUTTER — and all three of
        // those constants are the string `SideWalk`. A pavement, a kerb and a gutter
        // are one semantic class; only their materials differ.
        Folder::SideWalk
    } else {
        Folder::Terrain
    }
}

/// What a segmentation camera will report for a mesh of this name, once it is in the
/// map. The classifier and the tagger, run end to end.
pub fn label_of(mesh_name: &str) -> Label {
    folder_of(mesh_name).label()
}

/// Whether CARLA will drop this mesh before placing it.
///
/// `ValidateStaticMesh` refuses any mesh whose name contains `light` or `sign`,
/// case-insensitively, and does the same for the name of every material on it. The
/// intent is to keep RoadRunner's own traffic lights and signs out of the map, since
/// CARLA spawns its own; the effect is that a road called `Sign Street` loses its
/// surface without a word being logged.
///
/// Returns the offending word, so a report can quote it.
pub fn is_rejected_by_import(name: &str) -> Option<&'static str> {
    let lowered = name.to_ascii_lowercase();
    ["light", "sign"]
        .into_iter()
        .find(|word| lowered.contains(word))
}

/// Whether a map may be called this.
///
/// Every mesh name in a map's FBX begins with the map's name, and the classifier
/// matches on substrings — so a map name that itself holds one of the tokens decides
/// the class of every mesh in the map before the mesh's own role is ever looked at.
///
/// Returns the token that would take over, or `None` when the name is safe.
pub fn is_safe_map_name(map: &str) -> Option<&'static str> {
    // In classifier order, since the first match is the one that wins.
    const TOKENS: [&str; 7] = [
        "Road_Road",
        "Roads_Road",
        "Road_Marking",
        "Roads_Marking",
        "Terrain",
        "light",
        "sign",
    ];
    let lowered = map.to_ascii_lowercase();
    TOKENS
        .into_iter()
        .find(|token| lowered.contains(&token.to_ascii_lowercase()))
}

/// The map's name, reduced to something Unreal will accept as an asset name.
///
/// Unreal's object names allow a narrow set of characters and a name that leaves it
/// is renamed on import, which breaks the agreement between the mesh names in the FBX
/// and the classifier that reads them. Letters, digits and underscores survive;
/// everything else becomes an underscore, and a name that is left empty becomes
/// `Map`, because an unnamed asset is not importable at all.
pub fn sanitize(name: &str) -> String {
    let mut out: String = name
        .chars()
        .map(|character| {
            if character.is_ascii_alphanumeric() {
                character
            } else {
                '_'
            }
        })
        .collect();
    while out.starts_with('_') {
        out.remove(0);
    }
    while out.ends_with('_') {
        out.pop();
    }
    if out.is_empty() {
        out.push_str("Map");
    }
    // An asset name cannot begin with a digit and remain a legal identifier in every
    // place CARLA spells one, the package JSON's paths among them.
    if out.starts_with(|character: char| character.is_ascii_digit()) {
        out.insert(0, 'M');
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_mesh_name_is_classified_the_way_carla_classifies_it() {
        assert_eq!(folder_of("Town01_Road_Road_0"), Folder::Road);
        assert_eq!(folder_of("Town01_Road_Marking_3"), Folder::RoadLine);
        assert_eq!(folder_of("Town01_Road_Sidewalk_1"), Folder::SideWalk);
        assert_eq!(folder_of("Town01_Road_Curb_1"), Folder::SideWalk);
        assert_eq!(folder_of("Town01_Road_Gutter_1"), Folder::SideWalk);
        assert_eq!(folder_of("Town01_Terrain_Ground_0"), Folder::Terrain);
    }

    #[test]
    fn a_name_carla_does_not_recognise_becomes_ground() {
        // Not "it fails to import" and not "it keeps its own name": the classifier's
        // last arm is Terrain, so a typo segments as ground.
        assert_eq!(folder_of("Town01_Road_Sidwalk_1"), Folder::Terrain);
        assert_eq!(label_of("Town01_Road_Sidwalk_1"), Label::Terrain);
    }

    #[test]
    fn terrain_is_tested_before_the_pavement_and_wins() {
        // The one that costs a whole map: `Terrain` is tested bare and third, so a
        // map named for it drags every pavement, kerb and gutter into the ground.
        assert_eq!(folder_of("TerrainTown_Road_Sidewalk_1"), Folder::Terrain);
        assert_eq!(is_safe_map_name("TerrainTown"), Some("Terrain"));
        assert_eq!(is_safe_map_name("Town01"), None);
    }

    #[test]
    fn the_road_surface_still_wins_over_terrain() {
        // Road_Road is tested first, so the same map name does not cost the road —
        // which is exactly what makes the failure hard to see.
        assert_eq!(folder_of("TerrainTown_Road_Road_0"), Folder::Road);
    }

    #[test]
    fn a_mesh_named_for_a_light_or_a_sign_never_arrives() {
        assert_eq!(
            is_rejected_by_import("Sign_Street_Road_Road_0"),
            Some("sign")
        );
        assert_eq!(is_rejected_by_import("Town01_Road_Road_0"), None);
        // Case-insensitive, as `ESearchCase::IgnoreCase` makes it.
        assert_eq!(
            is_rejected_by_import("Highlight_Road_Road_0"),
            Some("light")
        );
    }

    #[test]
    fn every_label_keeps_the_stencil_value_carla_gives_it() {
        assert_eq!(Label::Roads.stencil(), 1);
        assert_eq!(Label::Sidewalks.stencil(), 2);
        assert_eq!(Label::Buildings.stencil(), 3);
        assert_eq!(Label::Terrain.stencil(), 10);
        assert_eq!(Label::RoadLines.stencil(), 24);
    }

    #[test]
    fn a_role_is_named_so_that_it_lands_where_it_meant_to() {
        // The point of the whole module: for every role but one, the name a mesh is
        // given is classified back into the folder the role asked for.
        for role in [
            Role::Road,
            Role::Marking,
            Role::Sidewalk,
            Role::Curb,
            Role::Gutter,
            Role::Terrain,
        ] {
            let name = mesh_name("Town01", role, 0);
            assert_eq!(
                folder_of(&name),
                role.intended_folder(),
                "{name} did not land in {}",
                role.intended_folder()
            );
        }
    }

    #[test]
    fn a_building_in_a_map_is_the_one_role_that_cannot_land_where_it_meant_to() {
        // `MoveAssetsCommandlet` has six names and none of them is a building, so a
        // building mesh inside a map's FBX is ground as far as CARLA is concerned.
        // `BuildingPlacement::Props` is the way out, and `check` says so.
        let name = mesh_name("Town01", Role::Building, 0);
        assert_eq!(Role::Building.intended_folder(), Folder::Building);
        assert_eq!(folder_of(&name), Folder::Terrain);
        // And it is the only role whose folder is not one a map's meshes can reach,
        // which is why it is the only one with a placement to choose.
        assert!(!Folder::from_a_map().contains(&Folder::Building));
        for role in [Role::Road, Role::Marking, Role::Sidewalk, Role::Terrain] {
            assert!(Folder::from_a_map().contains(&role.intended_folder()));
        }
    }

    #[test]
    fn a_map_name_is_reduced_to_something_unreal_will_keep() {
        assert_eq!(sanitize("high street"), "high_street");
        assert_eq!(sanitize("Town-01"), "Town_01");
        assert_eq!(sanitize("  "), "Map");
        assert_eq!(sanitize("01Town"), "M01Town");
    }
}
