//! Writing the meshes as FBX.
//!
//! FBX is Autodesk's interchange format and Unreal reads it through Autodesk's own
//! SDK, which accepts two encodings of the same node tree: a binary one and a text
//! one. This writes the **text** one, FBX 7.4.
//!
//! That is a deliberate trade and worth stating. Binary FBX is smaller and imports
//! faster; it also needs DEFLATE for its array records and ends in an undocumented
//! footer block whose contents the SDK checks. Text FBX needs neither, so this crate
//! has no compression dependency, no magic constants, and produces a file that can be
//! opened in an editor and diffed when a map comes out wrong — which, for a generator
//! whose output nobody can eyeball in a simulator, is worth more than the bytes.
//!
//! # The frame, which is the part that can silently be wrong
//!
//! Three conventions have to line up, and only two of them are ours.
//!
//! **roadgen** is right-handed: x east, y north, z up, metres.
//!
//! **CARLA and Unreal** are left-handed. `carla::geom::RightHandedVector3D` says so in
//! as many words — it negates Y at the boundary and negates it back — and
//! `MapBuilder.cpp` does the same to every OpenDRIVE point it parses, under a comment
//! reading `Unreal Y axis hack`.
//!
//! **Unreal's FBX importer** closes the gap. It is told which frame the file is in by
//! the file's own `GlobalSettings`, and with `bConvertScene` set — which CARLA's
//! import settings do set — it converts that frame to Unreal's, negating Y on the way.
//!
//! So the mesh is written in roadgen's own frame, unflipped, and declared Z-up
//! right-handed. The importer flips it; CARLA flips the `.xodr` the same way when it
//! parses it; and the two land on top of each other. Flipping Y here as well would
//! flip it twice, and a map mirrored about its own centreline is a map whose roads all
//! turn the wrong way — which looks plausible enough in a screenshot to survive a
//! review.
//!
//! Units are metres, declared as such: `UnitScaleFactor` is 100, meaning a hundred
//! centimetres to the unit. CARLA's importer sets `bConvertSceneUnit`, so it scales
//! that to Unreal's centimetres itself, and the numbers in this file stay the numbers
//! in the `.xodr` beside it.

use std::fmt::Write as _;

use roadgen_core::geometry::Point3;

use crate::materials::{self, Map, Material};
use crate::mesh::Mesh;

/// The first object identifier handed out.
///
/// FBX identifiers have to be unique and non-zero within a document; zero is the
/// scene root, which every model is connected to. The value itself means nothing —
/// this one is high enough to be obvious in a diff.
const FIRST_ID: i64 = 1_000_000;

/// How many decimals a coordinate is written with.
///
/// Six, in metres, is a micrometre — the tolerance the workspace's cross-format tests
/// hold geometry to. Writing more would be writing float noise into a diff.
const PLACES: usize = 6;

/// Writes `meshes` as one FBX document.
///
/// `materials` is indexed by the indices the meshes carry, and `creator` goes in the
/// header where a reader looks to find out what wrote a file it cannot import.
pub fn document(meshes: &[Mesh], creator: &str) -> String {
    let mut used: Vec<usize> = meshes
        .iter()
        .flat_map(|mesh| mesh.materials.iter().copied())
        .collect();
    used.sort_unstable();
    used.dedup();

    let mut ids = Ids::new();
    // Every object's identifier is settled before anything is written, because the
    // connections at the end of the file refer to objects declared at the top of it.
    let geometries: Vec<i64> = meshes.iter().map(|_| ids.next()).collect();
    let models: Vec<i64> = meshes.iter().map(|_| ids.next()).collect();
    let material_ids: Vec<i64> = used.iter().map(|_| ids.next()).collect();
    let textures: Vec<Picture> = pictures(&used, &mut ids);

    let mut out = String::new();
    header(&mut out, creator);
    definitions(&mut out, meshes.len(), material_ids.len(), textures.len());

    out.push_str("\n; Object properties\n");
    out.push_str(";------------------------------------------------------------------\n\n");
    out.push_str("Objects:  {\n");
    for (index, mesh) in meshes.iter().enumerate() {
        geometry(&mut out, geometries[index], mesh);
        model(&mut out, models[index], mesh);
    }
    for (slot, &index) in used.iter().enumerate() {
        material(&mut out, material_ids[slot], &materials::MATERIALS[index]);
    }
    for picture in &textures {
        texture(&mut out, picture);
    }
    out.push_str("}\n");

    out.push_str("\n; Object connections\n");
    out.push_str(";------------------------------------------------------------------\n\n");
    out.push_str("Connections:  {\n");
    for (index, mesh) in meshes.iter().enumerate() {
        // The model hangs off the scene root, the geometry off the model, and each of
        // the mesh's materials off the model in the order of its own slots — which is
        // the order `LayerElementMaterial` indexes them in.
        let _ = writeln!(out, "\tC: \"OO\",{},0", models[index]);
        let _ = writeln!(out, "\tC: \"OO\",{},{}", geometries[index], models[index]);
        for &material in &mesh.materials {
            let slot = used.iter().position(|&held| held == material).unwrap_or(0);
            let _ = writeln!(out, "\tC: \"OO\",{},{}", material_ids[slot], models[index]);
        }
    }
    for picture in &textures {
        let slot = used
            .iter()
            .position(|&held| held == picture.material)
            .unwrap_or(0);
        let _ = writeln!(
            out,
            "\tC: \"OP\",{},{}, \"{}\"",
            picture.texture, material_ids[slot], picture.channel
        );
        let _ = writeln!(out, "\tC: \"OO\",{},{}", picture.video, picture.texture);
    }
    out.push_str("}\n");
    out
}

/// Hands out object identifiers.
struct Ids(i64);

impl Ids {
    fn new() -> Ids {
        Ids(FIRST_ID)
    }

    fn next(&mut self) -> i64 {
        self.0 += 1;
        self.0
    }
}

/// One texture file, as the pair of objects FBX needs to reference it.
///
/// Two objects for one picture is FBX's own doing: a `Video` is the file on disk and
/// a `Texture` is one use of it, with its own UV set and wrapping. Nothing here shares
/// a `Video` between textures, because nothing here uses one file twice with different
/// settings.
struct Picture {
    texture: i64,
    video: i64,
    material: usize,
    name: String,
    path: String,
    /// The material property the texture drives: the FBX property name Unreal looks
    /// for when deciding what to plug a texture into.
    channel: &'static str,
}

fn pictures(used: &[usize], ids: &mut Ids) -> Vec<Picture> {
    let mut pictures = Vec::new();
    for &index in used {
        let material = &materials::MATERIALS[index];
        let Some(texture) = material.texture else {
            continue;
        };
        for &map in texture.maps {
            let Some(channel) = channel_of(map) else {
                continue;
            };
            pictures.push(Picture {
                texture: ids.next(),
                video: ids.next(),
                material: index,
                name: texture.file_name(map).replace('.', "_"),
                path: texture.relative_path(map),
                channel,
            });
        }
    }
    pictures
}

/// Which material property a texture map drives.
///
/// Roughness has none. FBX's material model is Phong, which has a shininess exponent
/// and no roughness map, and Unreal's importer will not invent a channel for one.
/// The file is still written into the package — a roughness map is the first thing
/// wanted when the material is rebuilt by hand in the editor — it is just not wired
/// up here, and `check` says so.
fn channel_of(map: Map) -> Option<&'static str> {
    match map {
        Map::Diffuse => Some("DiffuseColor"),
        Map::Normal => Some("NormalMap"),
        Map::Roughness => None,
    }
}

fn header(out: &mut String, creator: &str) {
    out.push_str("; FBX 7.4.0 project file\n");
    out.push_str(";------------------------------------------------------------------\n\n");
    out.push_str("FBXHeaderExtension:  {\n");
    out.push_str("\tFBXHeaderVersion: 1003\n");
    out.push_str("\tFBXVersion: 7400\n");
    // A fixed timestamp, so that the same map written twice is the same bytes. A
    // generator whose output changed with the clock could not be diffed, cached or
    // checked into anything.
    out.push_str("\tCreationTimeStamp:  {\n");
    out.push_str("\t\tVersion: 1000\n\t\tYear: 1970\n\t\tMonth: 1\n\t\tDay: 1\n");
    out.push_str("\t\tHour: 0\n\t\tMinute: 0\n\t\tSecond: 0\n\t\tMillisecond: 0\n");
    out.push_str("\t}\n");
    let _ = writeln!(out, "\tCreator: \"{}\"", escape(creator));
    out.push_str("}\n");

    out.push_str("GlobalSettings:  {\n\tVersion: 1000\n\tProperties70:  {\n");
    // Z-up, right-handed, x east and y north: roadgen's own frame, declared so that
    // the importer knows what to convert *from*. See this module's own documentation
    // for why nothing is flipped before it gets here.
    for (name, value) in [
        ("UpAxis", 2),
        ("UpAxisSign", 1),
        ("FrontAxis", 1),
        ("FrontAxisSign", -1),
        ("CoordAxis", 0),
        ("CoordAxisSign", 1),
        ("OriginalUpAxis", 2),
        ("OriginalUpAxisSign", 1),
    ] {
        let _ = writeln!(out, "\t\tP: \"{name}\", \"int\", \"Integer\", \"\",{value}");
    }
    // A hundred centimetres to the unit: the file is in metres, and CARLA's importer
    // is told to convert scene units, so it does the scaling into Unreal's
    // centimetres itself.
    for name in ["UnitScaleFactor", "OriginalUnitScaleFactor"] {
        let _ = writeln!(out, "\t\tP: \"{name}\", \"double\", \"Number\", \"\",100");
    }
    out.push_str("\t}\n}\n");
}

fn definitions(out: &mut String, meshes: usize, materials: usize, textures: usize) {
    out.push_str("\n; Object definitions\n");
    out.push_str(";------------------------------------------------------------------\n\n");
    out.push_str("Definitions:  {\n\tVersion: 100\n");
    // The count the reader preallocates against: every object in the file, plus the
    // global settings, which are declared here and not in `Objects`.
    let total = 1 + meshes * 2 + materials + textures * 2;
    let _ = writeln!(out, "\tCount: {total}");
    for (kind, count) in [
        ("GlobalSettings", 1),
        ("Geometry", meshes),
        ("Model", meshes),
        ("Material", materials),
        ("Texture", textures),
        ("Video", textures),
    ] {
        if count == 0 {
            continue;
        }
        let _ = writeln!(out, "\tObjectType: \"{kind}\" {{\n\t\tCount: {count}\n\t}}");
    }
    out.push_str("}\n");
}

fn geometry(out: &mut String, id: i64, mesh: &Mesh) {
    let _ = writeln!(
        out,
        "\tGeometry: {id}, \"Geometry::{}\", \"Mesh\" {{",
        escape(&mesh.name)
    );

    let mut vertices = Vec::with_capacity(mesh.positions.len() * 3);
    for point in &mesh.positions {
        vertices.extend(point.as_array());
    }
    array(out, 2, "Vertices", &vertices);

    // FBX marks the last index of a polygon by flipping its sign and subtracting one,
    // which is how a flat list of indices says where one polygon ends and the next
    // begins. Everything here is triangles, so it is every third entry.
    let indices: Vec<i64> = mesh
        .triangles
        .iter()
        .flat_map(|triangle| {
            [
                triangle[0] as i64,
                triangle[1] as i64,
                !(triangle[2] as i64),
            ]
        })
        .collect();
    integers(out, 2, "PolygonVertexIndex", &indices);
    out.push_str("\t\tGeometryVersion: 124\n");

    // Normals per polygon vertex rather than per vertex, because that is the only
    // mapping every reader of this format agrees on. The values are the mesh's own
    // vertex normals, so a vertex shared along a strip still gets one smooth normal.
    let normals = mesh.normals();
    let mut flat = Vec::with_capacity(mesh.triangles.len() * 9);
    for triangle in &mesh.triangles {
        for index in triangle {
            let normal = normals[*index as usize];
            flat.extend([normal.x, normal.y, normal.z]);
        }
    }
    out.push_str("\t\tLayerElementNormal: 0 {\n\t\t\tVersion: 101\n\t\t\tName: \"\"\n");
    out.push_str("\t\t\tMappingInformationType: \"ByPolygonVertex\"\n");
    out.push_str("\t\t\tReferenceInformationType: \"Direct\"\n");
    array(out, 3, "Normals", &flat);
    out.push_str("\t\t}\n");

    let mut uvs = Vec::with_capacity(mesh.uvs.len() * 2);
    for uv in &mesh.uvs {
        uvs.extend(uv);
    }
    let uv_indices: Vec<i64> = mesh
        .triangles
        .iter()
        .flat_map(|triangle| triangle.map(|index| index as i64))
        .collect();
    out.push_str("\t\tLayerElementUV: 0 {\n\t\t\tVersion: 101\n\t\t\tName: \"UVMap\"\n");
    out.push_str("\t\t\tMappingInformationType: \"ByPolygonVertex\"\n");
    out.push_str("\t\t\tReferenceInformationType: \"IndexToDirect\"\n");
    array(out, 3, "UV", &uvs);
    integers(out, 3, "UVIndex", &uv_indices);
    out.push_str("\t\t}\n");

    // The material layer indexes the mesh's *own* slots, in the order its materials
    // are connected to its model — not the document's material table. A mesh of one
    // material says so once, which is what `AllSame` is for.
    out.push_str("\t\tLayerElementMaterial: 0 {\n\t\t\tVersion: 101\n\t\t\tName: \"\"\n");
    if mesh.is_one_material() {
        out.push_str("\t\t\tMappingInformationType: \"AllSame\"\n");
        out.push_str("\t\t\tReferenceInformationType: \"IndexToDirect\"\n");
        integers(out, 3, "Materials", &[0]);
    } else {
        out.push_str("\t\t\tMappingInformationType: \"ByPolygon\"\n");
        out.push_str("\t\t\tReferenceInformationType: \"IndexToDirect\"\n");
        let slots: Vec<i64> = mesh.slots.iter().map(|&slot| slot as i64).collect();
        integers(out, 3, "Materials", &slots);
    }
    out.push_str("\t\t}\n");

    out.push_str("\t\tLayer: 0 {\n\t\t\tVersion: 100\n");
    for kind in [
        "LayerElementNormal",
        "LayerElementUV",
        "LayerElementMaterial",
    ] {
        let _ = writeln!(
            out,
            "\t\t\tLayerElement:  {{\n\t\t\t\tType: \"{kind}\"\n\t\t\t\tTypedIndex: 0\n\t\t\t}}"
        );
    }
    out.push_str("\t\t}\n\t}\n");
}

fn model(out: &mut String, id: i64, mesh: &Mesh) {
    let _ = writeln!(
        out,
        "\tModel: {id}, \"Model::{}\", \"Mesh\" {{",
        escape(&mesh.name)
    );
    out.push_str("\t\tVersion: 232\n\t\tProperties70:  {\n");
    // At the origin, unrotated and unscaled. A map's geometry is already in the map's
    // own coordinates; putting any of it in a node transform would mean the vertices
    // no longer say where the road is.
    out.push_str("\t\t\tP: \"Lcl Translation\", \"Lcl Translation\", \"\", \"A\",0,0,0\n");
    out.push_str("\t\t\tP: \"Lcl Rotation\", \"Lcl Rotation\", \"\", \"A\",0,0,0\n");
    out.push_str("\t\t\tP: \"Lcl Scaling\", \"Lcl Scaling\", \"\", \"A\",1,1,1\n");
    out.push_str("\t\t\tP: \"DefaultAttributeIndex\", \"int\", \"Integer\", \"\",0\n");
    out.push_str("\t\t}\n\t\tShading: T\n\t\tCulling: \"CullingOff\"\n\t}\n");
}

fn material(out: &mut String, id: i64, material: &Material) {
    let _ = writeln!(
        out,
        "\tMaterial: {id}, \"Material::{}\", \"\" {{",
        escape(material.name)
    );
    out.push_str("\t\tVersion: 102\n\t\tShadingModel: \"phong\"\n\t\tMultiLayer: 0\n");
    out.push_str("\t\tProperties70:  {\n");
    let [r, g, b] = material.color;
    let _ = writeln!(
        out,
        "\t\t\tP: \"DiffuseColor\", \"Color\", \"\", \"A\",{},{},{}",
        number(r),
        number(g),
        number(b)
    );
    // Phong's shininess is an exponent rather than a roughness, and the two run in
    // opposite directions. This is the usual mapping — a rough surface is a low
    // exponent — and it is only a stand-in: CARLA replaces these outright when
    // `use_carla_materials` is on, and a PBR material rebuilt in the editor reads the
    // roughness map in the package instead.
    let shininess = ((1.0 - material.roughness.clamp(0.0, 1.0)) * 100.0).max(2.0);
    let _ = writeln!(
        out,
        "\t\t\tP: \"ShininessExponent\", \"Number\", \"\", \"A\",{}",
        number(shininess)
    );
    out.push_str("\t\t\tP: \"SpecularColor\", \"Color\", \"\", \"A\",0.1,0.1,0.1\n");
    out.push_str("\t\t}\n\t}\n");
}

fn texture(out: &mut String, picture: &Picture) {
    let path = escape(&picture.path);
    let name = escape(&picture.name);
    let _ = writeln!(
        out,
        "\tTexture: {}, \"Texture::{name}\", \"\" {{",
        picture.texture
    );
    out.push_str("\t\tType: \"TextureVideoClip\"\n\t\tVersion: 202\n");
    let _ = writeln!(out, "\t\tTextureName: \"Texture::{name}\"");
    out.push_str("\t\tProperties70:  {\n");
    out.push_str("\t\t\tP: \"UVSet\", \"KString\", \"\", \"\", \"UVMap\"\n");
    out.push_str("\t\t\tP: \"UseMaterial\", \"bool\", \"\", \"\",1\n");
    out.push_str("\t\t}\n");
    let _ = writeln!(out, "\t\tMedia: \"Video::{name}\"");
    // Relative, and only relative. An absolute path would be this machine's, and a
    // package is a thing that gets copied to another one.
    let _ = writeln!(out, "\t\tFileName: \"{path}\"");
    let _ = writeln!(out, "\t\tRelativeFilename: \"{path}\"");
    out.push_str("\t\tModelUVTranslation: 0,0\n\t\tModelUVScaling: 1,1\n");
    out.push_str("\t\tTexture_Alpha_Source: \"None\"\n\t\tCropping: 0,0,0,0\n\t}\n");

    let _ = writeln!(
        out,
        "\tVideo: {}, \"Video::{name}\", \"Clip\" {{",
        picture.video
    );
    out.push_str("\t\tType: \"Clip\"\n\t\tProperties70:  {\n");
    let _ = writeln!(
        out,
        "\t\t\tP: \"Path\", \"KString\", \"XRefUrl\", \"\", \"{path}\""
    );
    out.push_str("\t\t}\n\t\tUseMipMap: 0\n");
    let _ = writeln!(out, "\t\tFilename: \"{path}\"");
    let _ = writeln!(out, "\t\tRelativeFilename: \"{path}\"");
    out.push_str("\t}\n");
}

/// One `*count { a: … }` array of numbers, wrapped so no line runs away.
fn array(out: &mut String, depth: usize, name: &str, values: &[f64]) {
    let tabs = "\t".repeat(depth);
    let _ = writeln!(out, "{tabs}{name}: *{} {{", values.len());
    let _ = write!(out, "{tabs}\ta: ");
    for (index, value) in values.iter().enumerate() {
        if index > 0 {
            out.push(',');
            // The SDK reads a line of any length, but nothing else does: an editor, a
            // diff and a terminal all want a file with lines in it.
            if index % 12 == 0 {
                let _ = write!(out, "\n{tabs}\t");
            }
        }
        out.push_str(&number(*value));
    }
    let _ = writeln!(out, "\n{tabs}}}");
}

fn integers(out: &mut String, depth: usize, name: &str, values: &[i64]) {
    let tabs = "\t".repeat(depth);
    let _ = writeln!(out, "{tabs}{name}: *{} {{", values.len());
    let _ = write!(out, "{tabs}\ta: ");
    for (index, value) in values.iter().enumerate() {
        if index > 0 {
            out.push(',');
            if index % 24 == 0 {
                let _ = write!(out, "\n{tabs}\t");
            }
        }
        let _ = write!(out, "{value}");
    }
    let _ = writeln!(out, "\n{tabs}}}");
}

/// A number, at a fixed precision with its trailing zeros taken off.
///
/// Fixed rather than shortest-round-trip so that the same geometry always writes the
/// same characters; trimmed so that a file of zeros is not three times the size it
/// needs to be.
fn number(value: f64) -> String {
    if !value.is_finite() {
        return "0".to_owned();
    }
    let mut text = format!("{value:.PLACES$}");
    if text.contains('.') {
        while text.ends_with('0') {
            text.pop();
        }
        if text.ends_with('.') {
            text.pop();
        }
    }
    // `-0` is a number no reader is wrong about, but it is noise in a diff.
    if text == "-0" {
        text = "0".to_owned();
    }
    text
}

/// A string as FBX can hold one.
///
/// The format has no escape for a double quote inside a quoted string, so the only
/// safe thing to do with one is not write it. Names reaching here have been through
/// [`crate::tags::sanitize`] and hold none; a path might, and a path with a quote in
/// it is not a path this will invent a syntax for.
fn escape(value: &str) -> String {
    value.replace(['"', '\n', '\r'], "_")
}

/// Where the FBX puts a point, which is where the IR put it.
///
/// Here so that the claim in this module's documentation — that nothing is flipped or
/// scaled on the way out — is a function with a test under it rather than a sentence.
pub fn place(point: Point3) -> [f64; 3] {
    point.as_array()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::tags::Role;

    fn one_triangle() -> Mesh {
        let mut mesh = Mesh::new("Town01_Road_Road_0", Role::Road, materials::ASPHALT);
        mesh.face(
            &[
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(10.0, 0.0, 0.0),
                Point3::new(10.0, 5.0, 0.0),
            ],
            0,
        );
        mesh
    }

    #[test]
    fn the_document_is_an_fbx_7_4_file() {
        let text = document(&[one_triangle()], "roadgen");
        assert!(text.starts_with("; FBX 7.4.0 project file"));
        assert!(text.contains("FBXVersion: 7400"));
        assert!(text.contains("Objects:  {"));
        assert!(text.contains("Connections:  {"));
    }

    #[test]
    fn nothing_is_flipped_or_scaled_on_the_way_out() {
        // The whole of the coordinate agreement with CARLA rests on this: the file
        // holds roadgen's own metres in roadgen's own frame, and says which frame that
        // is, so that Unreal's importer is the one place the handedness changes.
        let point = Point3::new(1.0, -2.0, 3.0);
        assert_eq!(place(point), [1.0, -2.0, 3.0]);

        let text = document(&[one_triangle()], "roadgen");
        assert!(text.contains("P: \"UpAxis\", \"int\", \"Integer\", \"\",2"));
        assert!(text.contains("P: \"UnitScaleFactor\", \"double\", \"Number\", \"\",100"));
    }

    #[test]
    fn a_polygon_ends_with_a_flipped_index() {
        let text = document(&[one_triangle()], "roadgen");
        // 0, 1, then ~2 — which is -3 — to say the triangle stops there.
        assert!(
            text.contains("a: 0,1,-3"),
            "the polygon list does not close a triangle:\n{text}"
        );
    }

    #[test]
    fn a_mesh_of_one_material_says_so_once() {
        let text = document(&[one_triangle()], "roadgen");
        assert!(text.contains("MappingInformationType: \"AllSame\""));
    }

    #[test]
    fn a_mesh_of_two_materials_names_one_per_polygon() {
        let mut mesh = one_triangle();
        let slot = mesh.slot_for(materials::MARKING_YELLOW);
        mesh.face(
            &[
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(1.0, 1.0, 0.0),
            ],
            slot,
        );
        let text = document(&[mesh], "roadgen");
        assert!(text.contains("MappingInformationType: \"ByPolygon\""));
        assert!(text.contains("Materials: *2 {"));
    }

    #[test]
    fn every_model_hangs_off_the_scene_root() {
        // A model connected to nothing is a model Unreal imports and then cannot find.
        let text = document(&[one_triangle()], "roadgen");
        let roots = text
            .lines()
            .filter(|line| line.trim_start().starts_with("C: \"OO\","))
            .filter(|line| line.ends_with(",0"))
            .count();
        assert_eq!(roots, 1);
    }

    #[test]
    fn a_textured_material_carries_its_pictures_by_relative_path() {
        let text = document(&[one_triangle()], "roadgen");
        assert!(text.contains("RelativeFilename: \"Textures/asphalt_02_diff_2k.jpg\""));
        // Wired to the material it belongs to, by the property Unreal reads.
        assert!(text.contains("\"DiffuseColor\""));
        assert!(text.contains("\"NormalMap\""));
        // And nothing absolute: a package is a thing that gets copied elsewhere.
        assert!(!text.contains("FileName: \"/"));
    }

    #[test]
    fn paint_asks_for_no_pictures_and_gets_none() {
        let mut mesh = Mesh::new(
            "Town01_Road_Marking_0",
            Role::Marking,
            materials::MARKING_WHITE,
        );
        mesh.face(
            &[
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(10.0, 0.0, 0.0),
                Point3::new(10.0, 0.2, 0.0),
            ],
            0,
        );
        let text = document(&[mesh], "roadgen");
        assert!(!text.contains("Video:"));
        assert!(!text.contains("ObjectType: \"Texture\""));
    }

    #[test]
    fn the_same_map_written_twice_is_the_same_bytes() {
        // A timestamp from the clock would make every export a new file, which is the
        // end of diffing one, caching one or checking one in.
        assert_eq!(
            document(&[one_triangle()], "roadgen"),
            document(&[one_triangle()], "roadgen")
        );
    }

    #[test]
    fn a_number_is_written_the_short_way() {
        assert_eq!(number(0.0), "0");
        assert_eq!(number(-0.0), "0");
        assert_eq!(number(1.5), "1.5");
        assert_eq!(number(1.0 / 3.0), "0.333333");
        assert_eq!(number(f64::NAN), "0");
    }
}
