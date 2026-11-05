//! An independent reader for RDS-HQ clips, for the tests to check against.
//!
//! It opens the tars the way a WebDataset reader does — a member is
//! `{key}.{entry}`, the key everything before the first dot — and decodes `.npy`
//! from its own header rather than trusting that the writer's shape is the one it
//! meant. Going through the files is the point: a test that only looks at the
//! exporter's own structs cannot tell whether the archive says what it should.

use std::collections::BTreeMap;
use std::io::Read;
use std::path::Path;

use roadgen_core::geometry::Point3;

/// One sample: its key, and its entries by name.
pub struct Sample {
    pub key: String,
    pub entries: BTreeMap<String, Vec<u8>>,
}

impl Sample {
    pub fn json(&self, entry: &str) -> serde_json::Value {
        let bytes = self
            .entries
            .get(entry)
            .unwrap_or_else(|| panic!("no {entry} in {:?}", self.entries.keys()));
        serde_json::from_slice(bytes).expect("an entry ending in .json is JSON")
    }

    pub fn array(&self, entry: &str) -> (Vec<usize>, Vec<f64>) {
        npy(&self.entries[entry])
    }

    /// The entries whose names end in `suffix`, in name order — which, with
    /// zero-padded frame numbers, is frame order.
    pub fn ending_in<'a>(&'a self, suffix: &'a str) -> impl Iterator<Item = (&'a str, &'a [u8])> {
        self.entries
            .iter()
            .filter(move |(name, _)| name.ends_with(suffix))
            .map(|(name, bytes)| (name.as_str(), bytes.as_slice()))
    }
}

/// Reads `{root}/{folder}/{clip}.tar`, insisting that every member has one key.
pub fn sample(root: &Path, folder: &str, clip: &str) -> Sample {
    let path = root.join(folder).join(format!("{clip}.tar"));
    let file =
        std::fs::File::open(&path).unwrap_or_else(|error| panic!("{}: {error}", path.display()));
    let mut archive = tar::Archive::new(file);
    let mut keys = std::collections::BTreeSet::new();
    let mut entries = BTreeMap::new();
    for member in archive.entries().expect("a tar") {
        let mut member = member.expect("a member");
        let name = member.path().expect("a name").display().to_string();
        let (key, entry) = name
            .split_once('.')
            .unwrap_or_else(|| panic!("{name} has no extension"));
        keys.insert(key.to_owned());
        let mut bytes = Vec::new();
        member.read_to_end(&mut bytes).expect("readable");
        entries.insert(entry.to_owned(), bytes);
    }
    assert_eq!(
        keys.len(),
        1,
        "{} holds one sample, not {keys:?}",
        path.display()
    );
    Sample {
        key: keys.into_iter().next().expect("one"),
        entries,
    }
}

/// A version 1.0 `.npy` of little-endian `f64`, C order: its shape and values.
pub fn npy(bytes: &[u8]) -> (Vec<usize>, Vec<f64>) {
    assert_eq!(&bytes[..6], b"\x93NUMPY", "not an npy file");
    let header_len = u16::from_le_bytes([bytes[8], bytes[9]]) as usize;
    let header = std::str::from_utf8(&bytes[10..10 + header_len]).expect("an ASCII header");
    assert!(header.contains("'descr': '<f8'"), "{header}");
    assert!(header.contains("'fortran_order': False"), "{header}");
    let shape_text = header
        .split("'shape': (")
        .nth(1)
        .and_then(|rest| rest.split(')').next())
        .expect("a shape");
    let shape: Vec<usize> = shape_text
        .split(',')
        .map(str::trim)
        .filter(|part| !part.is_empty())
        .map(|part| part.parse().expect("a dimension"))
        .collect();
    let values: Vec<f64> = bytes[10 + header_len..]
        .chunks_exact(8)
        .map(|chunk| f64::from_le_bytes(chunk.try_into().expect("eight bytes")))
        .collect();
    assert_eq!(values.len(), shape.iter().product::<usize>(), "{header}");
    (shape, values)
}

/// A shape's points, found the way the toolkit finds them: under `vertices`.
pub fn vertices(shape: &serde_json::Value) -> Vec<Point3> {
    shape["vertices"]
        .as_array()
        .expect("vertices")
        .iter()
        .map(|point| {
            let xyz: Vec<f64> = point
                .as_array()
                .expect("[x, y, z]")
                .iter()
                .map(|value| value.as_f64().expect("a number"))
                .collect();
            assert_eq!(xyz.len(), 3);
            Point3::new(xyz[0], xyz[1], xyz[2])
        })
        .collect()
}
