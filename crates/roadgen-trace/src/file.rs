//! Trace files: one exporter's [`Trace`] on disk, beside what it describes.
//!
//! A trace file names the files it describes by a path relative to itself, with a
//! digest of each, and the fingerprint of the IR it was written from. The digests
//! are what make a trace safe to keep: Lanelet2, OpenStreetMap, ClipGT and GPUDrive
//! number their output by counting, so a file written again from a changed map
//! reuses the same numbers for different things, and an old trace read against it
//! would give wrong answers without any sign of being wrong.

use std::fs;
use std::io;
use std::path::{Component, Path, PathBuf};

use serde::de::DeserializeOwned;
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};

use roadgen_core::map::Map;
use roadgen_core::trace::{Relation, Trace};

use crate::error::TraceError;
use crate::ir::{sha256_tag, Generator, IrCatalog, IrDocument, IR_SCHEMA};

/// The schema a trace file is written with.
pub const TRACE_SCHEMA: &str = "roadgen-trace/1";

/// Which way a trace runs: from the IR to a file an exporter wrote, or from a file a
/// reader read to the IR it made.
pub const EXPORT: &str = "export";

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
    ///
    /// Taken over a borrowed copy of the file rather than a clone of it, since the
    /// links are most of a trace. The copy has to serialise to exactly the bytes the
    /// file itself would with an empty digest, or every trace already written would
    /// read as altered: same fields, same names, same order.
    fn contents_digest(&self) -> String {
        #[derive(Serialize)]
        struct Unsigned<'a> {
            schema: &'a str,
            generator: &'a Generator,
            format: &'a str,
            direction: &'a str,
            ir_fingerprint: &'a str,
            files: &'a [FileDigest],
            links: &'a [LinkRecord],
            digest: &'a str,
        }
        let unsigned = Unsigned {
            schema: &self.schema,
            generator: &self.generator,
            format: &self.format,
            direction: &self.direction,
            ir_fingerprint: &self.ir_fingerprint,
            files: &self.files,
            links: &self.links,
            digest: "",
        };
        let bytes = serde_json::to_vec(&unsigned).expect("a trace is plain data");
        sha256_tag(&Sha256::digest(bytes))
    }

    pub fn read(path: &Path) -> Result<Self, TraceError> {
        TraceFile::parse(&read_text(path)?, &path.display().to_string())
    }

    /// A trace file from its text, `name` saying where it came from in any complaint.
    pub fn parse(text: &str, name: &str) -> Result<Self, TraceError> {
        let file = parse_json(text, name, TRACE_SCHEMA, |file: &TraceFile| &file.schema)?;
        let actual = file.contents_digest();
        if actual != file.digest {
            return Err(TraceError::Altered {
                path: name.to_owned(),
                stated: file.digest,
                actual,
            });
        }
        for link in &file.links {
            if Relation::parse(&link.rel).is_none() {
                return Err(TraceError::Parse(
                    name.to_owned(),
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
    write_trace_with(trace, &IrCatalog::of(map).fingerprint(), path)
}

/// Writes `trace` to `path` as [`write_trace`] does, for the map `ir_fingerprint`
/// identifies.
///
/// For a caller that writes several traces of one map: the fingerprint means
/// cataloguing the whole map and hashing its geometry, which is worth doing once
/// rather than once per export.
pub fn write_trace_with(
    trace: &Trace,
    ir_fingerprint: &str,
    path: impl AsRef<Path>,
) -> Result<(), TraceError> {
    let path = path.as_ref();
    TraceFile::new(trace, ir_fingerprint.to_owned(), path)?.write(path)
}

/// Writes the IR dump of `map` to `path`, and returns its fingerprint: what every
/// trace of the same map records, for a caller that writes those too.
pub fn write_ir(map: &Map, path: impl AsRef<Path>) -> Result<String, TraceError> {
    let document = IrDocument::of(map);
    write_json(path.as_ref(), &document)?;
    Ok(document.fingerprint)
}

/// Reads an IR dump, refusing one whose body does not hash to its fingerprint.
pub fn read_ir(path: impl AsRef<Path>) -> Result<IrDocument, TraceError> {
    let path = path.as_ref();
    read_ir_str(&read_text(path)?, &path.display().to_string())
}

/// An IR dump from its text, as [`read_ir`] reads one, `name` saying where it came
/// from in any complaint.
pub fn read_ir_str(text: &str, name: &str) -> Result<IrDocument, TraceError> {
    let document = parse_json(text, name, IR_SCHEMA, |document: &IrDocument| {
        &document.schema
    })?;
    document.check(name)?;
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

pub(crate) fn read_text(path: &Path) -> Result<String, TraceError> {
    fs::read_to_string(path).map_err(|error| TraceError::io(path, error))
}

/// `text` as a `T` written with `expected` as its schema, which `schema` reads off
/// it. Shared by both kinds of file, so they complain alike.
fn parse_json<T: DeserializeOwned>(
    text: &str,
    name: &str,
    expected: &'static str,
    schema: fn(&T) -> &str,
) -> Result<T, TraceError> {
    let value: T = serde_json::from_str(text)
        .map_err(|error| TraceError::Parse(name.to_owned(), error.to_string()))?;
    let found = schema(&value);
    if found != expected {
        return Err(TraceError::Schema {
            path: name.to_owned(),
            found: found.to_owned(),
            expected,
        });
    }
    Ok(value)
}

/// Streamed, since the files a trace describes — a point cloud, a CARLA mesh — can be
/// far larger than is worth holding in memory to hash.
fn digest_file(path: &Path) -> Result<String, TraceError> {
    let mut hasher = Sha256::new();
    fs::File::open(path)
        .and_then(|mut file| io::copy(&mut file, &mut hasher))
        .map_err(|error| TraceError::io(path, error))?;
    Ok(sha256_tag(&hasher.finalize()))
}

/// The directory `path` is in, `.` for a bare file name.
pub(crate) fn parent_of(path: &Path) -> PathBuf {
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
