//! `roadgen-editor-wasm` — the road editor's engine, for a browser.
//!
//! The page keeps the [`Document`] as JSON and owns every interaction: dragging a
//! point, adding a lane, undo. Each time the document changes it hands it here, and
//! gets back what the generator made of it, drawn by `roadgen-irview`, with every
//! problem pinned to the road or link it is about. Writing a format, and reading a
//! map back in, happen here too — with the same exporters the Python package uses,
//! so a file written from the editor is the file `roadgen` would have written.
//!
//! ```text
//!   page (JS)          this crate (wasm)
//!   ─────────          ─────────────────
//!   document.json ──▶  build ──▶ roadgen-irview::Scene ──▶ JSON ──▶ canvas
//!   document.json ──▶  export(format) ──▶ files ──▶ download
//!   a file        ──▶  import ──▶ IR ──▶ roadgen_editor::from_map ──▶ document.json
//! ```
//!
//! Every function is plain Rust underneath ([`build_json`], [`export_files`],
//! [`import_file`]), so that what the page relies on is tested natively; the
//! `#[wasm_bindgen]` layer only converts errors and byte buffers.

use roadgen_editor::{build, from_map, Built, Document, Problem};
use roadgen_irview::{Scene, SvgOptions};
use serde::Serialize;
use wasm_bindgen::prelude::*;

/// A format the editor can write.
#[derive(Debug, Clone, Copy, Serialize)]
pub struct Format {
    pub id: &'static str,
    pub label: &'static str,
    /// Whether the map has to validate first. The document itself, the picture and
    /// the IR can be saved from a map that is still being drawn.
    pub needs_valid_map: bool,
}

/// Every format, in the order the page lists them.
pub const FORMATS: &[Format] = &[
    Format {
        id: "document",
        label: "Editor document (.json)",
        needs_valid_map: false,
    },
    Format {
        id: "opendrive",
        label: "OpenDRIVE (.xodr, + OpenCRG for the surface)",
        needs_valid_map: true,
    },
    Format {
        id: "lanelet2",
        label: "Lanelet2 for Autoware (.osm)",
        needs_valid_map: true,
    },
    Format {
        id: "osm",
        label: "OpenStreetMap (.osm)",
        needs_valid_map: true,
    },
    Format {
        id: "sumo",
        label: "SUMO plain XML (several files)",
        needs_valid_map: true,
    },
    Format {
        id: "gpudrive",
        label: "GPUDrive scene (.json)",
        needs_valid_map: true,
    },
    Format {
        id: "commonroad",
        label: "CommonRoad scenario (.xml)",
        needs_valid_map: true,
    },
    Format {
        id: "ir",
        label: "roadgen IR (.roadgen.pb)",
        needs_valid_map: false,
    },
    Format {
        id: "svg",
        label: "Picture (.svg)",
        needs_valid_map: false,
    },
];

/// What the page draws after every change.
#[derive(Debug, Serialize)]
pub struct BuildResult {
    /// The map drawn in plan; empty when nothing could be built.
    pub scene: Scene,
    pub problems: Vec<Problem>,
    /// Whether every format can be written.
    pub valid: bool,
    pub stats: Stats,
}

/// How big the generated map is.
#[derive(Debug, Default, Serialize)]
pub struct Stats {
    pub roads: usize,
    pub connectors: usize,
    pub lanes: usize,
    pub junctions: usize,
    pub connections: usize,
}

/// One file of an export.
#[derive(Debug, Clone, PartialEq)]
pub struct File {
    pub name: String,
    pub bytes: Vec<u8>,
}

/// What an import gave back.
#[derive(Debug, Serialize)]
pub struct Imported {
    pub document: Document,
    /// What the editable document leaves out of the file, one line each.
    pub notes: Vec<String>,
}

pub fn parse_document(json: &str) -> Result<Document, String> {
    serde_json::from_str(json).map_err(|error| format!("the document is not readable: {error}"))
}

/// Builds and draws a document: the JSON of a [`BuildResult`].
pub fn build_json(document: &str) -> Result<String, String> {
    let built = build(&parse_document(document)?);
    let result = build_result(&built);
    serde_json::to_string(&result).map_err(|error| error.to_string())
}

fn build_result(built: &Built) -> BuildResult {
    let (scene, stats) = match &built.map {
        Some(map) => {
            let map = map.as_map();
            let connectors = map.roads.iter().filter(|road| road.is_connector()).count();
            (
                Scene::from_map(map),
                Stats {
                    roads: map.roads.len() - connectors,
                    connectors,
                    lanes: map.lanes.len(),
                    junctions: map.junctions.len(),
                    connections: map.connections.len(),
                },
            )
        }
        None => (Scene::default(), Stats::default()),
    };
    BuildResult {
        scene,
        problems: built.problems.clone(),
        valid: built.is_valid(),
        stats,
    }
}

/// Options an export takes from the page.
#[derive(Debug, Clone, Default)]
pub struct ExportOptions {
    /// Today, `YYYY-MM-DD`. CommonRoad dates its scenarios, and a WebAssembly
    /// module without WASI has no clock to read — so the page, which has one, says.
    pub date: Option<String>,
}

/// Writes a document in one format.
pub fn export_files(
    document: &str,
    format: &str,
    options: &ExportOptions,
) -> Result<Vec<File>, String> {
    let document = parse_document(document)?;
    let stem = file_stem(&document.name);
    let format = FORMATS
        .iter()
        .find(|candidate| candidate.id == format)
        .ok_or_else(|| format!("'{format}' is not a format the editor writes"))?;

    if format.id == "document" {
        let json = serde_json::to_string_pretty(&document).map_err(|e| e.to_string())?;
        return Ok(vec![file(format!("{stem}.roadgen.json"), json)]);
    }

    let built = build(&document);
    if !format.needs_valid_map {
        let map = built
            .map
            .as_ref()
            .ok_or_else(|| "nothing could be built from the document".to_owned())?
            .as_map();
        return Ok(match format.id {
            "ir" => vec![File {
                name: format!("{stem}.roadgen.pb"),
                bytes: roadgen_proto::to_bytes(map).map_err(|e| e.to_string())?,
            }],
            "svg" => vec![file(
                format!("{stem}.svg"),
                Scene::from_map(map).to_svg(&SvgOptions::default()),
            )],
            other => unreachable!("{other} is listed as needing no valid map"),
        });
    }

    let map = built
        .validated()
        .map_err(|error| format!("the map cannot be written yet: {error}"))?;
    let error = |e: &dyn std::fmt::Display| format!("{}: {e}", format.label);
    Ok(match format.id {
        "opendrive" => {
            // OpenDRIVE has no element for a rut or a pothole: they go in OpenCRG
            // grids beside the document, which its roads' <surface> refers to by
            // name — so they are part of the export, not an option of it.
            let options = roadgen_opendrive::Options {
                crg: (!map.surface_features.is_empty())
                    .then(roadgen_opendrive::CrgOptions::default),
                ..roadgen_opendrive::Options::default()
            };
            let (xml, _, grids) =
                roadgen_opendrive::to_xml_with_crg(&map, &options).map_err(|e| error(&e))?;
            let mut files = vec![file(format!("{stem}.xodr"), xml)];
            for grid in grids {
                files.push(file(
                    grid.file_name.clone(),
                    grid.text(map.as_map()).map_err(|e| error(&e))?,
                ));
            }
            files
        }
        "lanelet2" => vec![file(
            "lanelet2_map.osm".to_owned(),
            roadgen_lanelet2::to_osm_xml(&map).map_err(|e| error(&e))?,
        )],
        "osm" => vec![file(
            format!("{stem}.osm"),
            roadgen_osm::to_xml(&map).map_err(|e| error(&e))?,
        )],
        "sumo" => roadgen_sumo::to_plain_xml(&map)
            .map_err(|e| error(&e))?
            .files()
            .into_iter()
            .map(|(name, text)| file(name, text.to_owned()))
            .collect(),
        "gpudrive" => vec![file(
            format!("{stem}.json"),
            roadgen_gpudrive::to_json(&map, &roadgen_gpudrive::SceneConfig::default())
                .map_err(|e| error(&e))?,
        )],
        "commonroad" => {
            let config = roadgen_commonroad::ScenarioConfig {
                date: options.date.clone(),
                ..roadgen_commonroad::ScenarioConfig::default()
            };
            vec![file(
                format!("{stem}.xml"),
                roadgen_commonroad::to_xml(&map, &config).map_err(|e| error(&e))?,
            )]
        }
        other => unreachable!("{other} has no writer"),
    })
}

/// Reads a file into an editable document: an editor document, an OpenDRIVE file,
/// or the IR as Protocol Buffers, told apart by the name.
pub fn import_file(name: &str, bytes: &[u8]) -> Result<Imported, String> {
    let lower = name.to_ascii_lowercase();
    let text = || std::str::from_utf8(bytes).map_err(|_| format!("{name} is not text"));
    if lower.ends_with(".json") {
        return Ok(Imported {
            document: parse_document(text()?)?,
            notes: Vec::new(),
        });
    }
    let (map, mut notes) = if lower.ends_with(".xodr") || lower.ends_with(".xml") {
        let imported = roadgen_opendrive::from_xml(text()?).map_err(|e| e.to_string())?;
        (imported.map, imported.approximations)
    } else if lower.ends_with(".pb") {
        (
            roadgen_proto::from_bytes(bytes).map_err(|e| e.to_string())?,
            Vec::new(),
        )
    } else {
        return Err(format!(
            "{name}: the editor reads .roadgen.json, .xodr and .roadgen.pb files"
        ));
    };
    let (mut document, dropped) = from_map(map.as_map());
    if document.name == "imported" {
        document.name = file_stem(name.split('.').next().unwrap_or("imported"));
    }
    notes.extend(dropped);
    Ok(Imported { document, notes })
}

fn file(name: String, text: String) -> File {
    File {
        name,
        bytes: text.into_bytes(),
    }
}

/// A name that is safe as a file name on every system.
fn file_stem(name: &str) -> String {
    let stem: String = name
        .chars()
        .map(|c| {
            if c.is_ascii_alphanumeric() || c == '-' || c == '_' {
                c
            } else {
                '_'
            }
        })
        .collect();
    if stem.trim_matches('_').is_empty() {
        "map".to_owned()
    } else {
        stem
    }
}

// --- The JavaScript surface -------------------------------------------------------

/// The example document, as JSON.
#[wasm_bindgen(js_name = exampleDocument)]
pub fn example_document() -> String {
    serde_json::to_string_pretty(&Document::example()).expect("a document always serializes")
}

/// The formats the editor writes, as JSON.
#[wasm_bindgen]
pub fn formats() -> String {
    serde_json::to_string(FORMATS).expect("formats always serialize")
}

/// Builds and draws a document. Returns the JSON of the scene, the problems and
/// whether the map is valid.
#[wasm_bindgen(js_name = buildDocument)]
pub fn build_document(document: &str) -> Result<String, JsError> {
    build_json(document).map_err(|e| JsError::new(&e))
}

/// The files of one export.
#[wasm_bindgen]
pub struct Export {
    files: Vec<File>,
}

#[wasm_bindgen]
impl Export {
    pub fn count(&self) -> usize {
        self.files.len()
    }

    pub fn name(&self, index: usize) -> String {
        self.files
            .get(index)
            .map(|file| file.name.clone())
            .unwrap_or_default()
    }

    pub fn bytes(&self, index: usize) -> Vec<u8> {
        self.files
            .get(index)
            .map(|file| file.bytes.clone())
            .unwrap_or_default()
    }
}

/// Writes a document in `format`; `date` is today, `YYYY-MM-DD`.
#[wasm_bindgen(js_name = exportDocument)]
pub fn export_document(
    document: &str,
    format: &str,
    date: Option<String>,
) -> Result<Export, JsError> {
    export_files(document, format, &ExportOptions { date })
        .map(|files| Export { files })
        .map_err(|e| JsError::new(&e))
}

/// Reads a file into a document. Returns the JSON of the document and the notes.
#[wasm_bindgen(js_name = importFile)]
pub fn import(name: &str, bytes: &[u8]) -> Result<String, JsError> {
    import_file(name, bytes)
        .and_then(|imported| serde_json::to_string(&imported).map_err(|e| e.to_string()))
        .map_err(|e| JsError::new(&e))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn example() -> String {
        example_document()
    }

    #[test]
    fn the_example_builds_into_a_scene() {
        let json: serde_json::Value =
            serde_json::from_str(&build_json(&example()).unwrap()).unwrap();
        assert_eq!(json["valid"], true, "{}", json["problems"]);
        assert!(json["scene"]["lanes"].as_array().unwrap().len() > 10);
        assert_eq!(json["stats"]["roads"], 5);
        assert!(json["scene"]["bounds"]["max_x"].as_f64().unwrap() > 100.0);
    }

    #[test]
    fn a_broken_document_still_draws() {
        let mut document = Document::example();
        document.roads[0].points.truncate(1);
        let json = serde_json::to_string(&document).unwrap();
        let result: serde_json::Value = serde_json::from_str(&build_json(&json).unwrap()).unwrap();
        assert_eq!(result["valid"], false);
        assert!(!result["scene"]["lanes"].as_array().unwrap().is_empty());
        assert!(result["problems"]
            .as_array()
            .unwrap()
            .iter()
            .any(|p| p["road"] == "west"));
    }

    #[test]
    fn every_format_writes_the_example() {
        let options = ExportOptions {
            date: Some("2026-01-01".to_owned()),
        };
        for format in FORMATS {
            let files = export_files(&example(), format.id, &options)
                .unwrap_or_else(|e| panic!("{}: {e}", format.id));
            assert!(!files.is_empty(), "{}", format.id);
            for file in files {
                assert!(!file.bytes.is_empty(), "{} {}", format.id, file.name);
            }
        }
    }

    #[test]
    fn the_surface_goes_out_as_opencrg_beside_the_opendrive() {
        let files = export_files(&example(), "opendrive", &ExportOptions::default()).unwrap();
        let xodr = std::str::from_utf8(&files[0].bytes).unwrap();
        let grids: Vec<&File> = files.iter().filter(|f| f.name.ends_with(".crg")).collect();
        assert!(
            !grids.is_empty(),
            "{:?}",
            files.iter().map(|f| &f.name).collect::<Vec<_>>()
        );
        for grid in grids {
            assert!(
                xodr.contains(&grid.name),
                "{} is not referred to",
                grid.name
            );
        }
        // Without features, the document alone, as before.
        let mut plain = Document::example();
        for road in &mut plain.roads {
            road.surface.clear();
        }
        let json = serde_json::to_string(&plain).unwrap();
        let files = export_files(&json, "opendrive", &ExportOptions::default()).unwrap();
        assert_eq!(files.len(), 1);
    }

    #[test]
    fn an_invalid_map_is_not_written() {
        let mut document = Document::example();
        document.roads[0].lanes[0].width = 0.0;
        let json = serde_json::to_string(&document).unwrap();
        assert!(export_files(&json, "opendrive", &ExportOptions::default()).is_err());
        // The document and the picture can still be saved.
        assert!(export_files(&json, "document", &ExportOptions::default()).is_ok());
        assert!(export_files(&json, "svg", &ExportOptions::default()).is_ok());
    }

    #[test]
    fn what_is_written_reads_back() {
        let options = ExportOptions::default();
        let xodr = &export_files(&example(), "opendrive", &options).unwrap()[0];
        let imported = import_file(&xodr.name, &xodr.bytes).unwrap();
        assert_eq!(imported.document.roads.len(), 5, "{:?}", imported.notes);
        assert!(build(&imported.document).map.is_some());

        let pb = &export_files(&example(), "ir", &options).unwrap()[0];
        let imported = import_file(&pb.name, &pb.bytes).unwrap();
        assert_eq!(imported.document.roads.len(), 5);
        assert!(
            build(&imported.document).is_valid(),
            "{:?}",
            build(&imported.document).problems
        );

        let saved = &export_files(&example(), "document", &options).unwrap()[0];
        let imported = import_file(&saved.name, &saved.bytes).unwrap();
        assert_eq!(imported.document, Document::example());
    }

    #[test]
    fn unknown_formats_and_files_are_refused() {
        assert!(export_files(&example(), "dwg", &ExportOptions::default()).is_err());
        assert!(import_file("map.dwg", b"").is_err());
        assert!(build_json("{not json").is_err());
    }

    #[test]
    fn file_stems_are_safe() {
        assert_eq!(file_stem("my map/1"), "my_map_1");
        assert_eq!(file_stem(""), "map");
    }
}
