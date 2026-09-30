//! Trace files: one exporter's [`Trace`] on disk, beside what it describes.
//!
//! A trace file names the files it describes by a path relative to itself, with a
//! digest of each, and the fingerprint of the IR it was written from. The digests
//! are what make a trace safe to keep: Lanelet2, OpenStreetMap, ClipGT and GPUDrive
//! number their output by counting, so a file written again from a changed map
//! reuses the same numbers for different things, and an old trace read against it
//! would give wrong answers without any sign of being wrong.

use std::fs;
use std::path::{Component, Path, PathBuf};

use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};

use roadgen_core::map::Map;
use roadgen_core::trace::{Relation, Trace};

use crate::error::TraceError;
use crate::ir::{hex, Generator, IrCatalog, IrDocument};

/// The schema a trace file is written with.
pub const TRACE_SCHEMA: &str = "roadgen-trace/1";

/// Which way a trace runs: from the IR to a file an exporter wrote, or from a file a
/// reader read to the IR it made.
pub const EXPORT: &str = "export";
pub const IMPORT: &str = "import";

/// A trace file as it is written to disk.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct TraceFile {
    pub schema: String,
    pub generator: Generator,
    pub format: String,
    pub direction: String,
    pub ir_fingerprint: String,
    pub files: Vec<FileDigest>,
    pub links: Vec<LinkRecord>,
    /// `sha256:<hex>` of everything above, serialised compactly, so that a link
    /// edited in the file — an `ir` or a `ref` changed to another plausible value —
    /// is refused rather than answered with. The digests of the files it describes
    /// say nothing about the mapping itself.
    pub digest: String,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct FileDigest {
    /// Relative to the trace file's own directory, with `/` between components.
    pub path: String,
    /// `sha256:<hex>`.
    pub sha256: String,
}

/// One link, with the IR element and the written element as strings.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct LinkRecord {
    pub ir: String,
    #[serde(rename = "ref")]
    pub local: String,
    pub rel: String,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub role: Option<String>,
}

impl TraceFile {
    /// The file for `trace`, to be written at `path`, of the map `ir_fingerprint`
    /// identifies. The files the trace lists are read here, for their digests.
    pub fn new(trace: &Trace, ir_fingerprint: String, path: &Path) -> Result<Self, TraceError> {
        let base = parent_of(path);
        let files = trace
            .files
            .iter()
            .map(|file| {
                Ok(FileDigest {
                    path: relative_path(&base, file),
                    sha256: digest_file(file)?,
                })
            })
            .collect::<Result<Vec<_>, TraceError>>()?;
        let mut file = TraceFile {
            schema: TRACE_SCHEMA.into(),
            generator: Generator::this(),
            format: trace.format.clone(),
            direction: EXPORT.into(),
            ir_fingerprint,
            files,
            links: trace
                .links
                .iter()
                .map(|link| LinkRecord {
                    ir: link.ir.to_string(),
                    local: link.local.clone(),
                    rel: link.relation.as_str().into(),
                    role: link.role.clone(),
                })
                .collect(),
            digest: String::new(),
        };
        file.digest = file.contents_digest();
        Ok(file)
    }

    /// The digest of the file with its own [`digest`](Self::digest) left empty.
    fn contents_digest(&self) -> String {
        let unsigned = TraceFile {
            digest: String::new(),
            ..self.clone()
        };
        let bytes = serde_json::to_vec(&unsigned).expect("a trace is plain data");
        format!("sha256:{}", hex(&Sha256::digest(bytes)))
    }

    pub fn read(path: &Path) -> Result<Self, TraceError> {
        let text = fs::read_to_string(path).map_err(|error| TraceError::io(path, error))?;
        let file: TraceFile = serde_json::from_str(&text)
            .map_err(|error| TraceError::Parse(path.display().to_string(), error.to_string()))?;
        if file.schema != TRACE_SCHEMA {
            return Err(TraceError::Schema {
                path: path.display().to_string(),
                found: file.schema,
                expected: TRACE_SCHEMA,
            });
        }
        let actual = file.contents_digest();
        if actual != file.digest {
            return Err(TraceError::Altered {
                path: path.display().to_string(),
                stated: file.digest,
                actual,
            });
        }
        for link in &file.links {
            if Relation::parse(&link.rel).is_none() {
                return Err(TraceError::Parse(
                    path.display().to_string(),
                    format!("unknown relation `{}`", link.rel),
                ));
            }
        }
        Ok(file)
    }

    pub fn write(&self, path: &Path) -> Result<(), TraceError> {
        write_json(path, self)
    }

    /// Whether every file listed is where it was and unchanged.
    ///
    /// Returns the first one that is not, as a readable complaint.
    pub fn verify_files(&self, path: &Path) -> Result<(), TraceError> {
        let base = parent_of(path);
        for file in &self.files {
            let on_disk = base.join(&file.path);
            let actual = digest_file(&on_disk)?;
            if actual != file.sha256 {
                return Err(TraceError::Stale {
                    trace: path.display().to_string(),
                    file: on_disk.display().to_string(),
                });
            }
        }
        Ok(())
    }
}

/// Writes `trace` to `path`, recording the fingerprint of `map` and a digest of every
/// file the trace lists.
pub fn write_trace(trace: &Trace, map: &Map, path: impl AsRef<Path>) -> Result<(), TraceError> {
    let path = path.as_ref();
    TraceFile::new(trace, IrCatalog::of(map).fingerprint(), path)?.write(path)
}

/// Writes the IR dump of `map` to `path`.
pub fn write_ir(map: &Map, path: impl AsRef<Path>) -> Result<(), TraceError> {
    write_json(path.as_ref(), &IrDocument::of(map))
}

/// Reads an IR dump, refusing one whose body does not hash to its fingerprint.
pub fn read_ir(path: impl AsRef<Path>) -> Result<IrDocument, TraceError> {
    let path = path.as_ref();
    let text = fs::read_to_string(path).map_err(|error| TraceError::io(path, error))?;
    let document: IrDocument = serde_json::from_str(&text)
        .map_err(|error| TraceError::Parse(path.display().to_string(), error.to_string()))?;
    if document.schema != crate::ir::IR_SCHEMA {
        return Err(TraceError::Schema {
            path: path.display().to_string(),
            found: document.schema,
            expected: crate::ir::IR_SCHEMA,
        });
    }
    document.check(&path.display().to_string())?;
    Ok(document)
}

/// Where the trace of an export that writes one file belongs: beside it, the file's
/// name with `.trace.json` added — `lanelet2_map.osm.trace.json` — so the two sort
/// together.
pub fn sidecar_path(output: impl AsRef<Path>) -> PathBuf {
    let mut name = output.as_ref().as_os_str().to_owned();
    name.push(".trace.json");
    PathBuf::from(name)
}

/// Where the trace of an export that writes a directory belongs: inside it, named
/// after what the export named its files — a SUMO network's prefix, a clip's id —
/// and the format, `demo_town.sumo.trace.json`.
///
/// Not a fixed name: two networks, or two clips, can share a directory under their
/// own prefixes, and a trace named only by format would be overwritten by the second
/// and leave the first untraceable.
pub fn directory_sidecar(directory: impl AsRef<Path>, name: &str, format: &str) -> PathBuf {
    directory
        .as_ref()
        .join(format!("{name}.{format}.trace.json"))
}

fn write_json(path: &Path, value: &impl Serialize) -> Result<(), TraceError> {
    let mut text = serde_json::to_string_pretty(value).expect("a trace is plain data");
    text.push('\n');
    fs::write(path, text).map_err(|error| TraceError::io(path, error))
}

fn digest_file(path: &Path) -> Result<String, TraceError> {
    let bytes = fs::read(path).map_err(|error| TraceError::io(path, error))?;
    Ok(format!("sha256:{}", hex(&Sha256::digest(bytes))))
}

fn parent_of(path: &Path) -> PathBuf {
    match path.parent() {
        Some(parent) if !parent.as_os_str().is_empty() => parent.to_path_buf(),
        _ => PathBuf::from("."),
    }
}

/// `file` relative to `base`, when it is under it; otherwise as given.
///
/// Both are made absolute first, so a trace written with relative paths from one
/// working directory reads back from another.
fn relative_path(base: &Path, file: &Path) -> String {
    let absolute = |path: &Path| fs::canonicalize(path).unwrap_or_else(|_| path.to_path_buf());
    let (base, file) = (absolute(base), absolute(file));
    let relative = file.strip_prefix(&base).unwrap_or(&file);
    relative
        .components()
        .filter_map(|component| match component {
            Component::Normal(part) => Some(part.to_string_lossy().into_owned()),
            Component::RootDir => Some(String::new()),
            _ => None,
        })
        .collect::<Vec<_>>()
        .join("/")
}
