//! The surface features of a road, as ASAM OpenCRG grids beside the OpenDRIVE file.
//!
//! OpenDRIVE describes the surface a road was designed to: a reference line, an
//! elevation, a superelevation and a lateral shape. What a wheel meets on top of
//! that — a rut, a pothole, a puddle, a hump, the roughness of the whole surface —
//! is what **ASAM OpenCRG** is for, and OpenDRIVE 1.7 refers to it from a road's
//! `<surface>` with one `<CRG>` per file.
//!
//! # What is written
//!
//! One elevation file per road that has [surface features](roadgen_core::surface)
//! on it, covering the station range of all of them and the lateral range of all
//! of them, with a margin, on a regular grid of [`CrgOptions::increment_u`] by
//! [`CrgOptions::increment_v`] metres. Each grid value is
//! [`Relief::displacement`] there: how far the surface stands above the design.
//! Where a road holds water, a second file of the same grid gives the friction
//! coefficient — the pavement's nominal one, reduced where water stands — and is
//! referred to with `purpose="friction"`.
//!
//! The `<CRG>` is written in `attached` mode with `orientation="same"`: the CRG
//! reference line is replaced by the road's own, and its values are *added* to the
//! road's surface, which is exactly what a displacement from the design is. `u` is
//! measured from the `<CRG>`'s `sStart` (`u = s − sStart`, `sOffset` and
//! `tOffset` zero) and `v` is the road's own `t`, positive to the left.
//!
//! # The file
//!
//! An OpenCRG 1.2 ASCII file: a `$CT` comment block naming the features, a
//! `$ROAD_CRG` block with the grid's extent (`REFERENCE_LINE_START_U`, `_END_U`,
//! `_INCREMENT`, `LONG_SECTION_V_RIGHT`, `_LEFT`, `_INCREMENT`), a
//! `$KD_Definition` block in format `LRFI` with one `D:long section N,m` channel
//! per long section from right to left, and after the `$$$$` separator one row per
//! cross section from the start, each value ten characters wide, eight to an
//! 80-character record. The grid is full: there is no missing value in it.
//!
//! # Why the `<CRG>` element is patched after serialisation
//!
//! The `opendrive` crate types a road's `<CRG>` with the junction surface's mode,
//! which has `global` only, and with an object's orientation (`+`, `-`), where the
//! schema's road CRG wants `attached`/`attached0`/`genuine`/`global` and
//! `same`/`opposite`. The element is built with the crate's values and the two
//! attributes are rewritten in the serialised XML ([`patch_xml`]); the reader
//! undoes it before parsing, in `read::repair`. A caller holding the
//! [`OpenDrive`](opendrive::core::OpenDrive) value rather than the XML gets the
//! crate's spelling.

use std::collections::BTreeSet;

use roadgen_core::id::{RoadId, SurfaceFeatureId};
use roadgen_core::map::{Map, Road};
use roadgen_core::surface::Relief;

use crate::error::ExportError;
use crate::materials;

/// How the OpenCRG files are written. See the [module documentation](self).
#[derive(Debug, Clone, PartialEq)]
pub struct CrgOptions {
    /// Metres between cross sections, along the road.
    pub increment_u: f64,
    /// Metres between long sections, across the road.
    pub increment_v: f64,
    /// How far past the features' footprints the grid reaches, metres, so that the
    /// grid's edge, where a consumer stops reading it, is flat.
    pub margin: f64,
    /// What every file name begins with. A file is named
    /// `<prefix><road>.<purpose>.crg`, `<road>` being the road's IR name with
    /// anything but letters, digits, `-` and `_` turned into `_`.
    pub file_prefix: String,
    /// Whether a road that holds water gets a friction file beside its elevation
    /// file.
    pub friction: bool,
}

impl Default for CrgOptions {
    fn default() -> Self {
        CrgOptions {
            increment_u: 0.05,
            increment_v: 0.05,
            margin: 0.5,
            file_prefix: String::new(),
            friction: true,
        }
    }
}

/// What an OpenCRG file's values mean, as OpenDRIVE's `<CRG purpose>` says it.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum CrgPurpose {
    /// Metres above the road's designed surface.
    Elevation,
    /// A tyre-on-surface friction coefficient.
    Friction,
}

impl CrgPurpose {
    pub fn as_str(self) -> &'static str {
        match self {
            CrgPurpose::Elevation => "elevation",
            CrgPurpose::Friction => "friction",
        }
    }
}

/// One OpenCRG file, and where the OpenDRIVE road refers to it.
#[derive(Debug, Clone, PartialEq)]
pub struct CrgDocument {
    /// The file name, with no directory: the `<CRG file>` refers to it relative to
    /// the OpenDRIVE file, so it is written beside it.
    pub file_name: String,
    pub road: RoadId,
    pub purpose: CrgPurpose,
    /// The station range the grid covers, which is the `<CRG>`'s `sStart` and
    /// `sEnd`.
    pub s_start: f64,
    pub s_end: f64,
    /// The lateral range the grid covers, right edge then left, metres to the left
    /// of the reference line.
    pub v_right: f64,
    pub v_left: f64,
    /// The features the grid describes.
    pub features: Vec<SurfaceFeatureId>,
    /// How many cross sections and long sections the grid has.
    pub rows: usize,
    pub columns: usize,
    /// The file's contents.
    pub text: String,
}

/// Every OpenCRG file the map's surface features call for, road by road in the
/// map's order: an elevation file for every road with features, and a friction
/// file after it for every one that holds water, when [`CrgOptions::friction`].
pub fn documents(map: &Map, options: &CrgOptions) -> Result<Vec<CrgDocument>, ExportError> {
    if !(options.increment_u > 0.0 && options.increment_v > 0.0) {
        return Err(ExportError::Unsupported(format!(
            "an OpenCRG grid whose increments are not positive ({} by {} m)",
            options.increment_u, options.increment_v
        )));
    }
    let mut out = Vec::new();
    let mut names = BTreeSet::new();
    for road in map.roads.iter() {
        let relief = Relief::of(map, &road.id);
        if relief.is_empty() {
            continue;
        }
        let grid = Grid::of(road, &relief, options)?;
        let stem = unique_stem(
            &format!("{}{}", options.file_prefix, sanitize(road.id.local_name())),
            &mut names,
        );
        let features: Vec<SurfaceFeatureId> =
            relief.features.iter().map(|f| f.id.clone()).collect();
        out.push(grid.document(
            map,
            road,
            &relief,
            &stem,
            CrgPurpose::Elevation,
            features.clone(),
        ));
        if options.friction && relief.features.iter().any(|f| f.holds_water()) {
            out.push(
                grid.document(
                    map,
                    road,
                    &relief,
                    &stem,
                    CrgPurpose::Friction,
                    relief
                        .features
                        .iter()
                        .filter(|f| f.holds_water())
                        .map(|f| f.id.clone())
                        .collect(),
                ),
            );
        }
    }
    Ok(out)
}

/// The grid one road's files share.
struct Grid {
    s_start: f64,
    increment_u: f64,
    rows: usize,
    v_right: f64,
    increment_v: f64,
    columns: usize,
}

impl Grid {
    fn of(road: &Road, relief: &Relief, options: &CrgOptions) -> Result<Grid, ExportError> {
        let length = road.horizontal_length()?;
        let (mut s0, mut s1, mut t0, mut t1) = (
            f64::INFINITY,
            f64::NEG_INFINITY,
            f64::INFINITY,
            f64::NEG_INFINITY,
        );
        for feature in &relief.features {
            let (from, to) = feature.stations();
            let (right, left) = feature.laterals();
            s0 = s0.min(from);
            s1 = s1.max(to);
            t0 = t0.min(right);
            t1 = t1.max(left);
        }
        let s0 = (s0 - options.margin).max(0.0);
        let s1 = (s1 + options.margin).min(length);
        let (du, dv) = (options.increment_u, options.increment_v);
        // Whole increments, so the last cross section is where the header says the
        // grid ends; the grid grows past the features rather than short of them,
        // and is moved back onto the road if that takes it past an end.
        let rows = ((s1 - s0) / du - 1e-9).ceil().max(1.0) as usize;
        let mut start = s0;
        if start + rows as f64 * du > length {
            start = (length - rows as f64 * du).max(0.0);
        }
        let right = ((t0 - options.margin) / dv).floor() * dv;
        let left = ((t1 + options.margin) / dv).ceil() * dv;
        let columns = ((left - right) / dv).round().max(1.0) as usize;
        Ok(Grid {
            s_start: start,
            increment_u: du,
            rows: rows + 1,
            v_right: right,
            increment_v: dv,
            columns: columns + 1,
        })
    }

    fn s_end(&self) -> f64 {
        self.s_start + (self.rows - 1) as f64 * self.increment_u
    }

    fn v_left(&self) -> f64 {
        self.v_right + (self.columns - 1) as f64 * self.increment_v
    }

    fn document(
        &self,
        map: &Map,
        road: &Road,
        relief: &Relief,
        stem: &str,
        purpose: CrgPurpose,
        features: Vec<SurfaceFeatureId>,
    ) -> CrgDocument {
        let file_name = format!("{stem}.{}.crg", purpose.as_str());
        let value = |s: f64, t: f64| match purpose {
            CrgPurpose::Elevation => relief.displacement(s, t),
            CrgPurpose::Friction => {
                let dry = road
                    .pavement
                    .at(s)
                    .map_or(materials::DEFAULT_FRICTION, |p| materials::nominal(p).0);
                dry * materials::wet_share(relief.water_depth(s, t))
            }
        };
        let mut text = String::new();
        self.header(map, road, relief, purpose, &features, &mut text);
        for row in 0..self.rows {
            let s = self.s_start + row as f64 * self.increment_u;
            let values: Vec<f64> = (0..self.columns)
                .map(|column| value(s, self.v_right + column as f64 * self.increment_v))
                .collect();
            for record in values.chunks(VALUES_PER_RECORD) {
                for value in record {
                    text.push_str(&field(*value));
                }
                text.push('\n');
            }
        }
        CrgDocument {
            file_name,
            road: road.id.clone(),
            purpose,
            s_start: self.s_start,
            s_end: self.s_end(),
            v_right: self.v_right,
            v_left: self.v_left(),
            features,
            rows: self.rows,
            columns: self.columns,
            text,
        }
    }

    fn header(
        &self,
        map: &Map,
        road: &Road,
        relief: &Relief,
        purpose: CrgPurpose,
        features: &[SurfaceFeatureId],
        out: &mut String,
    ) {
        let mut line = |text: String| {
            // A header record is at most 72 characters.
            let mut text = text;
            if text.len() > HEADER_RECORD {
                let mut cut = HEADER_RECORD;
                while !text.is_char_boundary(cut) {
                    cut -= 1;
                }
                text.truncate(cut);
            }
            out.push_str(&text);
            out.push('\n');
        };
        line("$CT".into());
        line(format!(
            "{} of {}, written by roadgen {}",
            match purpose {
                CrgPurpose::Elevation => "Surface elevation",
                CrgPurpose::Friction => "Surface friction",
            },
            road.id,
            env!("CARGO_PKG_VERSION")
        ));
        if let Some(name) = &map.metadata.name {
            line(format!("map: {name}"));
        }
        line(match purpose {
            CrgPurpose::Elevation => {
                "values: metres above the road's designed surface (attached mode)".into()
            }
            CrgPurpose::Friction => "values: tyre-on-surface friction coefficient".into(),
        });
        line("u = s - sStart along the road, v = t (left positive), metres".into());
        for id in features {
            if let Some(feature) = relief.features.iter().find(|f| f.id == *id) {
                line(format!(
                    "{} {} s={:.2}+{:.2} t={:.2}+-{:.2}",
                    feature.id,
                    feature.kind.as_str(),
                    feature.station,
                    feature.length,
                    feature.offset,
                    feature.width / 2.0
                ));
            }
        }
        line("$".into());
        line("$ROAD_CRG".into());
        line(format!("REFERENCE_LINE_START_U   = {:.6}", 0.0));
        line(format!(
            "REFERENCE_LINE_END_U     = {:.6}",
            self.s_end() - self.s_start
        ));
        line(format!(
            "REFERENCE_LINE_INCREMENT = {:.6}",
            self.increment_u
        ));
        line(format!("LONG_SECTION_V_RIGHT     = {:.6}", self.v_right));
        line(format!("LONG_SECTION_V_LEFT      = {:.6}", self.v_left()));
        line(format!(
            "LONG_SECTION_V_INCREMENT = {:.6}",
            self.increment_v
        ));
        line("$".into());
        line("$KD_Definition".into());
        line("#:LRFI".into());
        line(format!("U:reference line u,m,0,{:.6}", self.increment_u));
        for column in 0..self.columns {
            line(format!("D:long section {},m", column + 1));
        }
        line("$".into());
        out.push_str(SEPARATOR);
        out.push('\n');
    }
}

/// The longest a header record may be.
const HEADER_RECORD: usize = 72;
/// How many ten-character values fit an 80-character record.
const VALUES_PER_RECORD: usize = 8;
/// The line that ends the header and starts the data, with its ruler.
const SEPARATOR: &str =
    "$$$$$$$$10$$$$$$$$20$$$$$$$$30$$$$$$$$40$$$$$$$$50$$$$$$$$60$$$$$$$$70$$$$$$$$80";

/// One value, right-justified in ten characters: fixed point while it fits,
/// scientific when it does not.
fn field(value: f64) -> String {
    let fixed = format!("{value:10.7}");
    if fixed.len() <= 10 {
        return fixed;
    }
    for precision in (0..7).rev() {
        let fixed = format!("{value:10.precision$}");
        if fixed.len() <= 10 {
            return fixed;
        }
    }
    format!("{value:10.3e}")
}

/// A road's IR name as a file name.
fn sanitize(name: &str) -> String {
    name.chars()
        .map(|c| {
            if c.is_ascii_alphanumeric() || c == '-' || c == '_' {
                c
            } else {
                '_'
            }
        })
        .collect()
}

/// `stem`, or `stem_2`, `stem_3`… if two roads sanitise to the same name.
fn unique_stem(stem: &str, taken: &mut BTreeSet<String>) -> String {
    let mut candidate = stem.to_owned();
    let mut n = 2;
    while !taken.insert(candidate.clone()) {
        candidate = format!("{stem}_{n}");
        n += 1;
    }
    candidate
}

/// Rewrites every `<CRG>` the `opendrive` crate serialised into the spelling
/// OpenDRIVE 1.7 has for a road's CRG: `mode="attached"` for the crate's only
/// mode, `global`, and `orientation="same"`/`"opposite"` for `+`/`-`. See the
/// [module documentation](self).
pub(crate) fn patch_xml(xml: &str) -> String {
    let mut out = String::with_capacity(xml.len());
    let mut rest = xml;
    while let Some(start) = rest.find("<CRG ") {
        out.push_str(&rest[..start]);
        let tail = &rest[start..];
        let end = tail.find('>').map_or(tail.len(), |end| end + 1);
        let element = tail[..end]
            .replace("mode=\"global\"", "mode=\"attached\"")
            .replace("orientation=\"+\"", "orientation=\"same\"")
            .replace("orientation=\"-\"", "orientation=\"opposite\"");
        out.push_str(&element);
        rest = &tail[end..];
    }
    out.push_str(rest);
    out
}

/// Reads the values of an OpenCRG file this module wrote, as one row per cross
/// section: for tests, and for a caller that wants to check a file against the
/// IR without an OpenCRG library.
pub fn parse_values(text: &str, columns: usize) -> Option<Vec<Vec<f64>>> {
    let data = text.split_once(SEPARATOR)?.1;
    let mut values = Vec::new();
    for record in data.lines().filter(|line| !line.trim().is_empty()) {
        let bytes = record.as_bytes();
        for chunk in bytes.chunks(10) {
            let field = std::str::from_utf8(chunk).ok()?.trim();
            values.push(field.parse::<f64>().ok()?);
        }
    }
    if columns == 0 || values.len() % columns != 0 {
        return None;
    }
    Some(values.chunks(columns).map(<[f64]>::to_vec).collect())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_field_is_ten_characters_whatever_the_value() {
        for value in [
            0.0,
            -0.025,
            0.1234567891,
            -1.5,
            12.25,
            -123.456,
            98_765.4,
            1e12,
        ] {
            let text = field(value);
            assert_eq!(text.len(), 10, "{value} → {text:?}");
            let back: f64 = text.trim().parse().unwrap();
            assert!(
                (back - value).abs() <= value.abs() * 1e-3 + 1e-7,
                "{value} → {text:?}"
            );
        }
    }

    #[test]
    fn the_crates_crg_attributes_are_rewritten_to_a_roads() {
        let xml = r#"<surface><CRG file="a.crg" mode="global" orientation="+" sEnd="1" sStart="0"/></surface><object orientation="+"/>"#;
        let patched = patch_xml(xml);
        assert!(patched.contains(r#"mode="attached""#));
        assert!(patched.contains(r#"<CRG file="a.crg" mode="attached" orientation="same""#));
        // Nothing outside a <CRG> is touched.
        assert!(patched.ends_with(r#"<object orientation="+"/>"#));
    }

    #[test]
    fn names_are_made_safe_and_unique() {
        let mut taken = BTreeSet::new();
        assert_eq!(unique_stem(&sanitize("j0/a.b"), &mut taken), "j0_a_b");
        assert_eq!(unique_stem(&sanitize("j0_a/b"), &mut taken), "j0_a_b_2");
    }
}
