//! Finding the files of an export on disk.
//!
//! Two of the six formats are a directory rather than a file, and in both the names
//! inside it are the writer's to choose: `export_sumo` returns the prefix it used and
//! `export_clipgt` the clip id. A caller who kept the return value can name every
//! file; a caller who did not — which includes anyone looking at a directory someone
//! else wrote — needs the directory read instead. That is what this is.
//!
//! Nothing here decides what a file *means* beyond its name. A `.nod.xml` is the node
//! file because SUMO says so, and `{clip}.{layer}.parquet` is a layer because ClipGT
//! does.

use std::fs;
use std::path::Path;

use crate::clipgt::Clip;
use crate::error::ViewError;
use crate::sumo::Network;

/// The three files of a SUMO plain-XML network, found by their suffixes.
///
/// The connection file is optional in the format and optional here. The node and edge
/// files are not: a network without them is not a network, and saying which one is
/// missing is more use than an empty picture.
pub fn sumo(directory: &Path) -> Result<Network, ViewError> {
    let mut network = Network::default();
    let mut nodes = false;
    let mut edges = false;

    for entry in entries(directory)? {
        let name = entry.file_name().to_string_lossy().into_owned();
        let text = || text(&entry.path());
        if name.ends_with(".nod.xml") {
            network.nodes = text()?;
            nodes = true;
        } else if name.ends_with(".edg.xml") {
            network.edges = text()?;
            edges = true;
        } else if name.ends_with(".con.xml") {
            network.connections = Some(text()?);
        }
    }

    if !nodes {
        return Err(ViewError::Missing(format!(
            "a .nod.xml in {}",
            directory.display()
        )));
    }
    if !edges {
        return Err(ViewError::Missing(format!(
            "a .edg.xml in {}",
            directory.display()
        )));
    }
    Ok(network)
}

/// The layer tables of a ClipGT clip, keyed by layer.
///
/// A directory holding more than one clip is read as one: the clip id is dropped and
/// the layer kept, so two clips in a directory would overwrite each other layer by
/// layer. ClipGT is a clip per directory, and a caller with two of them has already
/// gone somewhere this cannot follow.
pub fn clipgt(directory: &Path) -> Result<Clip, ViewError> {
    let mut clip = Clip::new();
    for entry in entries(directory)? {
        let name = entry.file_name().to_string_lossy().into_owned();
        let Some(stem) = name.strip_suffix(".parquet") else {
            continue;
        };
        // `{clip_id}.{layer}` — and a clip id may hold dots of its own, so the layer
        // is what follows the last one.
        let layer = stem.rsplit('.').next().unwrap_or(stem);
        clip.insert(
            layer.to_owned(),
            fs::read(entry.path())
                .map_err(|error| ViewError::Io(format!("{}: {error}", entry.path().display())))?,
        );
    }
    if clip.is_empty() {
        return Err(ViewError::Missing(format!(
            "a .parquet layer in {}",
            directory.display()
        )));
    }
    Ok(clip)
}

/// A file's text.
pub fn text(path: &Path) -> Result<String, ViewError> {
    fs::read_to_string(path).map_err(|error| ViewError::Io(format!("{}: {error}", path.display())))
}

fn entries(directory: &Path) -> Result<Vec<fs::DirEntry>, ViewError> {
    let read = fs::read_dir(directory)
        .map_err(|error| ViewError::Io(format!("{}: {error}", directory.display())))?;
    let mut entries = Vec::new();
    for entry in read {
        let entry =
            entry.map_err(|error| ViewError::Io(format!("{}: {error}", directory.display())))?;
        if entry.path().is_file() {
            entries.push(entry);
        }
    }
    // Directory order is the filesystem's, and a picture should not depend on it.
    entries.sort_by_key(|entry| entry.file_name());
    Ok(entries)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn write(directory: &Path, name: &str, body: &[u8]) {
        fs::write(directory.join(name), body).unwrap();
    }

    #[test]
    fn a_sumo_network_is_found_by_the_suffixes_sumo_gives_it() {
        let directory = tempdir();
        write(&directory, "demo_town.nod.xml", b"<nodes/>");
        write(&directory, "demo_town.edg.xml", b"<edges/>");
        write(&directory, "demo_town.con.xml", b"<connections/>");
        write(&directory, "demo_town.netccfg", b"<configuration/>");
        let network = sumo(&directory).unwrap();
        assert_eq!(network.nodes, "<nodes/>");
        assert_eq!(network.edges, "<edges/>");
        assert_eq!(network.connections.as_deref(), Some("<connections/>"));
    }

    #[test]
    fn a_network_with_no_connection_file_is_still_a_network() {
        let directory = tempdir();
        write(&directory, "t.nod.xml", b"<nodes/>");
        write(&directory, "t.edg.xml", b"<edges/>");
        assert!(sumo(&directory).unwrap().connections.is_none());
    }

    #[test]
    fn a_missing_node_file_says_which_file_is_missing() {
        let directory = tempdir();
        write(&directory, "t.edg.xml", b"<edges/>");
        let error = sumo(&directory).unwrap_err();
        assert!(error.to_string().contains(".nod.xml"), "{error}");
    }

    #[test]
    fn a_clip_is_keyed_by_layer_whatever_the_clip_is_called() {
        let directory = tempdir();
        write(&directory, "my.town.lane.parquet", b"one");
        write(&directory, "my.town.lane_line.parquet", b"two");
        write(&directory, "my.town.camera_front.json", b"not a layer");
        let clip = clipgt(&directory).unwrap();
        assert_eq!(clip.get("lane").map(Vec::as_slice), Some(&b"one"[..]));
        assert_eq!(clip.get("lane_line").map(Vec::as_slice), Some(&b"two"[..]));
        assert_eq!(clip.len(), 2);
    }

    #[test]
    fn a_directory_with_no_layers_in_it_is_not_a_clip() {
        let directory = tempdir();
        write(&directory, "readme.txt", b"nothing here");
        assert!(matches!(clipgt(&directory), Err(ViewError::Missing(_))));
    }

    /// A directory of this test's own, removed by the process exiting. `tempfile` is
    /// a dev-dependency the workspace already carries, but the viewer is built for
    /// the browser as well and a test helper is not worth another entry in its
    /// manifest.
    fn tempdir() -> std::path::PathBuf {
        use std::sync::atomic::{AtomicUsize, Ordering};
        static COUNTER: AtomicUsize = AtomicUsize::new(0);
        let path = std::env::temp_dir().join(format!(
            "roadgen-viewer-{}-{}",
            std::process::id(),
            COUNTER.fetch_add(1, Ordering::Relaxed)
        ));
        fs::create_dir_all(&path).unwrap();
        path
    }
}
