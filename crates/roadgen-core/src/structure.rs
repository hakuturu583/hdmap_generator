//! What carries a road clear of the ground: bridges, viaducts and tunnels — and
//! the places where one road passes over another.
//!
//! # The shape of the model
//!
//! GDF's Structures theme, taken the way the rest of this IR takes GDF: the ideas,
//! not the catalogue. GDF and the formats built on it say a road is on a bridge
//! twice over, and so does this IR:
//!
//! - a [`Structure`] is a feature of its own — the bridge, with its deck, its piers
//!   and its abutments; the tunnel, with its portals — and lists the stretches of
//!   road it carries;
//! - a [`Road`](crate::map::Road) says, along its length, which structure each
//!   stretch of it is on or in ([`StructureProfile`]), the way GDF's road element
//!   carries a "part of structure" attribute.
//!
//! The two refer to each other, the way a road and its lanes do, and validation
//! checks that they agree.
//!
//! A [`GradeSeparation`] is the relationship GDF writes as "X over Y through
//! structure": this road passes over that one — or over a river — by way of this
//! bridge, or under it through this tunnel. It is an object rather than something
//! re-derived from the geometry each time, so a consumer asks "where does this
//! road pass under something?" and gets an answer without intersecting anything.
//!
//! What the IR does *not* keep is a relative level — GDF's z-level, OSM's `layer`.
//! Every point here has its real height, and a level kept beside it would be a
//! second answer that could disagree with the first. A format that wants a level
//! derives it.
//!
//! An [`Interchange`] is the complex feature that gathers the junctions, slip roads
//! and structures of one grade-separated junction, the way GDF's Level 2
//! intersection gathers its road elements.

use std::collections::HashMap;
use std::fmt;

use crate::buildings::Footprint;
use crate::error::GeometryError;
use crate::error::ValidationIssue;
use crate::geometry::plan::{rings_overlap, PlanPoint};
use crate::geometry::{Frame3, Point3};
use crate::id::{
    GradeSeparationId, InterchangeId, JunctionId, LandCoverId, RoadId, SiteId, StructureId,
};
use crate::map::Map;
use crate::semantics::LaneType;
use crate::topology::LateralSide;

/// A stretch of one road, by horizontal station.
#[derive(Debug, Clone, PartialEq)]
pub struct RoadSpan {
    pub road: RoadId,
    pub from: f64,
    pub to: f64,
}

impl RoadSpan {
    pub fn new(road: RoadId, from: f64, to: f64) -> Self {
        RoadSpan { road, from, to }
    }

    pub fn length(&self) -> f64 {
        self.to - self.from
    }

    pub fn contains(&self, station: f64) -> bool {
        station >= self.from - 1e-6 && station <= self.to + 1e-6
    }
}

/// What a bridge is, by what it spans.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum BridgeForm {
    /// A bridge over a river, a valley, a railway.
    Bridge,
    /// A long elevated road on a row of piers.
    Viaduct,
    /// A short bridge that only carries one road over another.
    Overpass,
}

impl BridgeForm {
    pub fn as_str(self) -> &'static str {
        match self {
            BridgeForm::Bridge => "bridge",
            BridgeForm::Viaduct => "viaduct",
            BridgeForm::Overpass => "overpass",
        }
    }

    pub fn parse(text: &str) -> Option<Self> {
        match text {
            "bridge" => Some(BridgeForm::Bridge),
            "viaduct" => Some(BridgeForm::Viaduct),
            "overpass" => Some(BridgeForm::Overpass),
            _ => None,
        }
    }
}

/// How a tunnel was made, which decides how deep it lies.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum TunnelForm {
    /// Bored through a hill.
    Bored,
    /// Dug from the surface and covered over, under a street or a park.
    CutAndCover,
    /// A short passage under another road.
    Underpass,
}

impl TunnelForm {
    pub fn as_str(self) -> &'static str {
        match self {
            TunnelForm::Bored => "bored",
            TunnelForm::CutAndCover => "cut_and_cover",
            TunnelForm::Underpass => "underpass",
        }
    }

    pub fn parse(text: &str) -> Option<Self> {
        match text {
            "bored" => Some(TunnelForm::Bored),
            "cut_and_cover" => Some(TunnelForm::CutAndCover),
            "underpass" => Some(TunnelForm::Underpass),
            _ => None,
        }
    }
}

/// What a structure is built of.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum StructureMaterial {
    Concrete,
    Steel,
    Masonry,
    Timber,
}

impl StructureMaterial {
    pub fn as_str(self) -> &'static str {
        match self {
            StructureMaterial::Concrete => "concrete",
            StructureMaterial::Steel => "steel",
            StructureMaterial::Masonry => "masonry",
            StructureMaterial::Timber => "timber",
        }
    }

    pub fn parse(text: &str) -> Option<Self> {
        match text {
            "concrete" => Some(StructureMaterial::Concrete),
            "steel" => Some(StructureMaterial::Steel),
            "masonry" | "brick" => Some(StructureMaterial::Masonry),
            "timber" | "wood" => Some(StructureMaterial::Timber),
            _ => None,
        }
    }
}

/// What holds a deck up.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum SupportKind {
    /// Where the deck meets the ground, at each end.
    Abutment,
    /// A column between them.
    Pier,
}

impl SupportKind {
    pub fn as_str(self) -> &'static str {
        match self {
            SupportKind::Abutment => "abutment",
            SupportKind::Pier => "pier",
        }
    }
}

/// One abutment or pier column.
///
/// Part of the bridge's geometry rather than a feature of its own, the way a TIN's
/// triangles are part of its surface: a pier is where the generator put it, and has
/// nothing to be called that a regenerated map would call it again besides its
/// place in the row.
#[derive(Debug, Clone, PartialEq)]
pub struct Support {
    pub kind: SupportKind,
    /// The column's plan, at the height of its top — the deck's underside.
    pub footprint: Footprint,
    /// The height its foot stands at: the land's, where the map states land; `None`
    /// where it does not, and the column goes down to whatever ground a consumer
    /// lays.
    pub base: Option<f64>,
}

/// A bridge: a deck on supports.
#[derive(Debug, Clone, PartialEq)]
pub struct Bridge {
    pub form: BridgeForm,
    pub material: StructureMaterial,
    /// From the road surface to the underside of the deck, metres.
    pub deck_depth: f64,
    pub supports: Vec<Support>,
}

/// One mouth of a tunnel: where its road goes in or comes out.
#[derive(Debug, Clone, PartialEq)]
pub struct Portal {
    pub road: RoadId,
    pub station: f64,
    /// The opening's two sides at road level, to the left of the road and to its
    /// right.
    pub left: Point3,
    pub right: Point3,
}

/// A tunnel: a bore with a portal at each end.
#[derive(Debug, Clone, PartialEq)]
pub struct Tunnel {
    pub form: TunnelForm,
    /// From the road surface to the ceiling, metres.
    pub clear_height: f64,
    /// How thick the lining round the bore is, metres: the land over a tunnel has
    /// to clear its ceiling by this much.
    pub lining: f64,
    pub lighting: bool,
    pub portals: [Portal; 2],
}

/// Which kind of structure, with what only that kind has.
#[derive(Debug, Clone, PartialEq)]
pub enum StructureKind {
    Bridge(Bridge),
    Tunnel(Tunnel),
}

impl StructureKind {
    pub fn as_str(&self) -> &'static str {
        match self {
            StructureKind::Bridge(_) => "bridge",
            StructureKind::Tunnel(_) => "tunnel",
        }
    }

    pub fn bridge(&self) -> Option<&Bridge> {
        match self {
            StructureKind::Bridge(bridge) => Some(bridge),
            StructureKind::Tunnel(_) => None,
        }
    }

    pub fn tunnel(&self) -> Option<&Tunnel> {
        match self {
            StructureKind::Tunnel(tunnel) => Some(tunnel),
            StructureKind::Bridge(_) => None,
        }
    }

    /// Its form's name: `viaduct`, `bored`.
    pub fn form(&self) -> &'static str {
        match self {
            StructureKind::Bridge(bridge) => bridge.form.as_str(),
            StructureKind::Tunnel(tunnel) => tunnel.form.as_str(),
        }
    }
}

/// A bridge or a tunnel, and the stretches of road it carries.
#[derive(Debug, Clone, PartialEq)]
pub struct Structure {
    pub id: StructureId,
    pub name: Option<String>,
    pub kind: StructureKind,
    /// The roads it carries, and over which stations. A viaduct carrying both
    /// carriageways of a motorway lists two; twin bores are two tunnels.
    pub carries: Vec<RoadSpan>,
    /// Its plan: a bridge's deck, a tunnel's bore seen from above. At the height of
    /// the road it carries.
    pub outline: Footprint,
    pub site: Option<SiteId>,
}

/// One entry of a [`StructureProfile`].
#[derive(Debug, Clone, PartialEq)]
pub struct StructureSpan {
    pub from: f64,
    pub to: f64,
    pub structure: StructureId,
}

/// Which stretches of a road are on, or in, which structure. A station no span
/// covers is at grade — on the ground, in a cutting or on an embankment.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct StructureProfile {
    spans: Vec<StructureSpan>,
}

impl StructureProfile {
    /// A road wholly at grade.
    pub fn at_grade() -> Self {
        StructureProfile::default()
    }

    pub fn spans(&self) -> &[StructureSpan] {
        &self.spans
    }

    pub fn is_empty(&self) -> bool {
        self.spans.is_empty()
    }

    /// The structure at `station`, if any.
    pub fn at(&self, station: f64) -> Option<&StructureId> {
        self.spans
            .iter()
            .find(|span| station >= span.from - 1e-9 && station <= span.to + 1e-9)
            .map(|span| &span.structure)
    }

    /// Adds a span, keeping them in station order. Fails, handing the span back,
    /// when it overlaps one already there.
    pub fn insert(&mut self, span: StructureSpan) -> Result<(), StructureSpan> {
        if self
            .spans
            .iter()
            .any(|other| span.from < other.to - 1e-9 && other.from < span.to - 1e-9)
        {
            return Err(span);
        }
        let at = self
            .spans
            .iter()
            .position(|other| other.from > span.from)
            .unwrap_or(self.spans.len());
        self.spans.insert(at, span);
        Ok(())
    }
}

/// What a road passes over.
#[derive(Debug, Clone, PartialEq)]
pub enum Underneath {
    Road(RoadSpan),
    /// A river, a canal: what a land cover says the ground is.
    LandCover(LandCoverId),
}

/// "`over` passes over `under` by way of `via`": a crossing that is not a junction.
#[derive(Debug, Clone, PartialEq)]
pub struct GradeSeparation {
    pub id: GradeSeparationId,
    pub over: RoadSpan,
    pub under: Underneath,
    /// The bridge `over` is on, or the tunnel `under` is in.
    pub via: StructureId,
}

/// A grade-separated junction as one feature: its junctions, its slip roads and the
/// stretches of the roads it joins, and the structures it stands on.
#[derive(Debug, Clone, PartialEq)]
pub struct Interchange {
    pub id: InterchangeId,
    pub name: Option<String>,
    /// What it is, in the generator's words: `diamond`, `trumpet`, `parclo`.
    pub kind: String,
    pub junctions: Vec<JunctionId>,
    pub roads: Vec<RoadId>,
    pub structures: Vec<StructureId>,
}

// ---------------------------------------------------------------------------------
// Laying a structure out
// ---------------------------------------------------------------------------------

/// A bridge to put under some stretches of road.
#[derive(Debug, Clone, PartialEq)]
pub struct BridgeSpec {
    pub name: Option<String>,
    pub form: BridgeForm,
    pub material: StructureMaterial,
    pub deck_depth: f64,
    /// The longest span between two supports, metres.
    pub pier_spacing: f64,
    /// How far the deck reaches past the road's outer edge, metres: the kerb a
    /// parapet stands on.
    pub curb: f64,
    pub site: Option<SiteId>,
}

impl Default for BridgeSpec {
    fn default() -> Self {
        BridgeSpec {
            name: None,
            form: BridgeForm::Bridge,
            material: StructureMaterial::Concrete,
            deck_depth: 2.5,
            pier_spacing: 40.0,
            curb: 0.6,
            site: None,
        }
    }
}

/// A tunnel to put round some stretches of road.
#[derive(Debug, Clone, PartialEq)]
pub struct TunnelSpec {
    pub name: Option<String>,
    pub form: TunnelForm,
    pub clear_height: f64,
    pub lining: f64,
    pub lighting: bool,
    /// The inspection walkway beside the road, inside the bore, metres.
    pub walkway: f64,
    pub site: Option<SiteId>,
}

impl Default for TunnelSpec {
    fn default() -> Self {
        TunnelSpec {
            name: None,
            form: TunnelForm::Bored,
            clear_height: 5.0,
            lining: 0.6,
            lighting: true,
            walkway: 0.75,
            site: None,
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub enum StructureSpec {
    Bridge(BridgeSpec),
    Tunnel(TunnelSpec),
}

impl StructureSpec {
    fn name(&self) -> Option<&str> {
        match self {
            StructureSpec::Bridge(spec) => spec.name.as_deref(),
            StructureSpec::Tunnel(spec) => spec.name.as_deref(),
        }
    }

    fn form(&self) -> &'static str {
        match self {
            StructureSpec::Bridge(spec) => spec.form.as_str(),
            StructureSpec::Tunnel(spec) => match spec.form {
                TunnelForm::Underpass => "underpass",
                _ => "tunnel",
            },
        }
    }

    fn check(&self) -> Result<(), String> {
        let positive = |name: &str, value: f64| {
            if value.is_finite() && value > 0.0 {
                Ok(())
            } else {
                Err(format!(
                    "a structure's {name} must be a positive number, not {value}"
                ))
            }
        };
        match self {
            StructureSpec::Bridge(spec) => {
                positive("deck depth", spec.deck_depth)?;
                positive("pier spacing", spec.pier_spacing)?;
                if !(spec.curb.is_finite() && spec.curb >= 0.0) {
                    return Err(format!("a bridge's curb cannot be {}", spec.curb));
                }
            }
            StructureSpec::Tunnel(spec) => {
                positive("clear height", spec.clear_height)?;
                positive("lining", spec.lining)?;
                if !(spec.walkway.is_finite() && spec.walkway >= 0.0) {
                    return Err(format!("a tunnel's walkway cannot be {}", spec.walkway));
                }
            }
        }
        Ok(())
    }
}

/// Why a structure could not be put on the map.
#[derive(Debug, Clone, PartialEq)]
pub enum StructureError {
    Spec(String),
    NoSuchRoad(RoadId),
    /// The span is not within its road, or overlaps a structure already there.
    Span(String),
    Duplicate(String),
    Geometry(GeometryError),
}

impl fmt::Display for StructureError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            StructureError::Spec(detail) | StructureError::Span(detail) => f.write_str(detail),
            StructureError::NoSuchRoad(road) => write!(f, "{road} is no road on the map"),
            StructureError::Duplicate(id) => write!(f, "{id} is already on the map"),
            StructureError::Geometry(error) => write!(f, "{error}"),
        }
    }
}

impl std::error::Error for StructureError {}

impl From<GeometryError> for StructureError {
    fn from(error: GeometryError) -> Self {
        StructureError::Geometry(error)
    }
}

/// How far a road reaches to each side of its reference line at a station, along
/// its banked lateral axis: the left edge's offset (positive) and the right edge's
/// (negative), from the lanes of the section there.
pub fn road_extent(map: &Map, road: &RoadId, station: f64) -> Option<(f64, f64)> {
    let entry = map.road(road)?;
    let section = entry.section_at(station)?;
    let origin = entry.lane_offset.evaluate(station);
    let (mut left, mut right) = (origin, origin);
    for lane in map.lanes_of_section(road, section) {
        match lane.side {
            LateralSide::Left => left += lane.width_at(station),
            LateralSide::Right => right -= lane.width_at(station),
        }
    }
    Some((left, right))
}

/// The same as [`road_extent`], counting only the lanes traffic drives on and their
/// shoulders — where a pier must not stand.
pub fn carriageway_extent(map: &Map, road: &RoadId, station: f64) -> Option<(f64, f64)> {
    let entry = map.road(road)?;
    let section = entry.section_at(station)?;
    let origin = entry.lane_offset.evaluate(station);
    let mut lanes = map.lanes_of_section(road, section);
    lanes.sort_by_key(|lane| lane.ordinal);
    let (mut left, mut right) = (origin, origin);
    let (mut outer_left, mut outer_right) = (origin, origin);
    for lane in lanes {
        let width = lane.width_at(station);
        let traffic = matches!(
            lane.lane_type,
            LaneType::Driving | LaneType::Shoulder | LaneType::Biking | LaneType::Parking
        );
        match lane.side {
            LateralSide::Left => {
                left += width;
                if traffic {
                    outer_left = left;
                }
            }
            LateralSide::Right => {
                right -= width;
                if traffic {
                    outer_right = right;
                }
            }
        }
    }
    Some((outer_left, outer_right))
}

/// A road sampled along a span: the station, the banked frame and the extent there.
struct Strip {
    rows: Vec<(f64, Frame3, f64, f64)>,
}

impl Strip {
    fn of(map: &Map, span: &RoadSpan) -> Result<Strip, StructureError> {
        let road = map
            .road(&span.road)
            .ok_or_else(|| StructureError::NoSuchRoad(span.road.clone()))?;
        let step = map.metadata.sampling.max_segment_length.max(0.5);
        let count = ((span.to - span.from) / step).ceil().max(1.0) as usize;
        let mut rows = Vec::with_capacity(count + 1);
        for index in 0..=count {
            let station = span.from + (span.to - span.from) * index as f64 / count as f64;
            let frame = road.frame_at(station, map.metadata.sampling)?;
            let (left, right) = road_extent(map, &span.road, station).unwrap_or((0.0, 0.0));
            rows.push((station, frame, left, right));
        }
        Ok(Strip { rows })
    }

    /// The row nearest `point` in plan, and the point's lateral offset there.
    fn locate(&self, point: Point3) -> (usize, f64) {
        let mut best = (0, f64::INFINITY);
        for (index, (_, frame, _, _)) in self.rows.iter().enumerate() {
            let distance = frame.origin.horizontal_distance_to(point);
            if distance < best.1 {
                best = (index, distance);
            }
        }
        let frame = self.rows[best.0].1;
        let (dx, dy) = (point.x - frame.origin.x, point.y - frame.origin.y);
        let left = frame.left.get();
        let horizontal = (left.x * left.x + left.y * left.y).sqrt().max(1e-9);
        (best.0, (dx * left.x + dy * left.y) / horizontal)
    }
}

/// The plan every carried stretch adds up to, with `curb` added round it: sampled
/// along the first stretch, widened at each sample to take in the others.
pub fn outline(map: &Map, spans: &[RoadSpan], curb: f64) -> Result<Vec<Point3>, StructureError> {
    if spans.is_empty() {
        return Err(StructureError::Span("an outline of nothing".into()));
    }
    outline_of(map, spans, curb)
}

fn outline_of(map: &Map, spans: &[RoadSpan], curb: f64) -> Result<Vec<Point3>, StructureError> {
    let primary = Strip::of(map, &spans[0])?;
    let mut bounds: Vec<(f64, f64)> = primary.rows.iter().map(|row| (row.2, row.3)).collect();
    for span in &spans[1..] {
        let other = Strip::of(map, span)?;
        for (_, frame, left, right) in &other.rows {
            for offset in [*left, *right] {
                let point = frame.offset(offset);
                let (index, lateral) = primary.locate(point);
                bounds[index].0 = bounds[index].0.max(lateral);
                bounds[index].1 = bounds[index].1.min(lateral);
            }
        }
    }
    let mut ring = Vec::with_capacity(bounds.len() * 2);
    for ((_, frame, _, _), (left, _)) in primary.rows.iter().zip(&bounds) {
        ring.push(frame.offset(left + curb));
    }
    for ((_, frame, _, _), (_, right)) in primary.rows.iter().zip(&bounds).rev() {
        ring.push(frame.offset(right - curb));
    }
    Ok(ring)
}

/// A rectangle in plan at height `z`: centred on `centre`, `along` long in the
/// direction `heading` and `across` wide.
fn rectangle(centre: Point3, heading: f64, along: f64, across: f64, z: f64) -> Vec<Point3> {
    let (c, s) = (heading.cos(), heading.sin());
    let corner =
        |a: f64, b: f64| Point3::new(centre.x + c * a - s * b, centre.y + s * a + c * b, z);
    let (a, b) = (along / 2.0, across / 2.0);
    vec![corner(-a, -b), corner(a, -b), corner(a, b), corner(-a, b)]
}

/// Every road's carriageway as a ring in plan, with the road's lowest height, to
/// keep piers off: everything but `except`.
fn carriageways(map: &Map, except: &[RoadId]) -> Vec<(Vec<PlanPoint>, f64)> {
    let mut rings = Vec::new();
    for road in map.roads.iter() {
        if except.contains(&road.id) {
            continue;
        }
        let Ok(length) = road.horizontal_length() else {
            continue;
        };
        let step = 4.0_f64;
        let count = (length / step).ceil().max(1.0) as usize;
        let (mut lefts, mut rights) = (Vec::new(), Vec::new());
        let mut low = f64::INFINITY;
        for index in 0..=count {
            let station = length * index as f64 / count as f64;
            let Ok(frame) = road.frame_at(station, map.metadata.sampling) else {
                continue;
            };
            let Some((left, right)) = carriageway_extent(map, &road.id, station) else {
                continue;
            };
            let (l, r) = (frame.offset(left), frame.offset(right));
            low = low.min(frame.origin.z);
            lefts.push([l.x, l.y]);
            rights.push([r.x, r.y]);
        }
        if lefts.len() < 2 {
            continue;
        }
        rights.reverse();
        lefts.extend(rights);
        rings.push((lefts, low));
    }
    rings
}

/// The id a structure gets: its name's, or the next free `<road>/<form>/<n>`.
fn structure_id(map: &Map, spans: &[RoadSpan], spec: &StructureSpec) -> StructureId {
    if let Some(name) = spec.name() {
        let key = slug(name);
        if !key.is_empty() {
            return StructureId::new(key);
        }
    }
    let road = spans[0].road.local_name();
    let form = spec.form();
    let mut index = 0;
    loop {
        let id = StructureId::new(format!("{road}/{form}/{index}"));
        if !map.structures.contains(&id) {
            return id;
        }
        index += 1;
    }
}

/// The name made safe to put in an identifier: lower case, words joined by `_`.
pub fn slug(name: &str) -> String {
    let mut out = String::new();
    for character in name.trim().chars() {
        if character.is_ascii_alphanumeric() {
            out.push(character.to_ascii_lowercase());
        } else if !out.ends_with('_') {
            out.push('_');
        }
    }
    out.trim_matches('_').to_owned()
}

/// Puts a structure under (or round) `spans`, lays out its geometry — its outline,
/// and a bridge's abutments and piers or a tunnel's portals — and records it on
/// each road it carries.
///
/// Piers are spaced evenly at no more than the spec's spacing and kept off the
/// carriageway of every road below; one that has nowhere to stand within half a
/// span of where it belongs is left out.
pub fn place(
    map: &mut Map,
    spans: &[RoadSpan],
    spec: &StructureSpec,
) -> Result<StructureId, StructureError> {
    place_with(map, spans, spec, true)
}

/// [`place`], for a structure whose ends the roads were built with vertices at:
/// the builder's own, which it was told of before it built the lanes.
pub(crate) fn place_built(
    map: &mut Map,
    spans: &[RoadSpan],
    spec: &StructureSpec,
) -> Result<StructureId, StructureError> {
    place_with(map, spans, spec, false)
}

fn place_with(
    map: &mut Map,
    spans: &[RoadSpan],
    spec: &StructureSpec,
    snapping: bool,
) -> Result<StructureId, StructureError> {
    spec.check().map_err(StructureError::Spec)?;
    if spans.is_empty() {
        return Err(StructureError::Span(
            "a structure has to carry some road".into(),
        ));
    }
    for span in spans {
        let road = map
            .road(&span.road)
            .ok_or_else(|| StructureError::NoSuchRoad(span.road.clone()))?;
        let length = road.horizontal_length()?;
        if !(span.from.is_finite() && span.to.is_finite())
            || span.from < -1e-6
            || span.to > length + 1e-6
            || span.to - span.from < 1.0
        {
            return Err(StructureError::Span(format!(
                "{} {:.1}–{:.1} is not a stretch of a road {length:.1} m long",
                span.road, span.from, span.to
            )));
        }
        if let Some(other) = road
            .structure
            .spans()
            .iter()
            .find(|other| span.from < other.to - 1e-9 && other.from < span.to - 1e-9)
        {
            return Err(StructureError::Span(format!(
                "{} {:.1}–{:.1} is already carried by {}",
                span.road, span.from, span.to, other.structure
            )));
        }
    }
    // Each end moved onto the nearest vertex the road already has, so the road's
    // lanes, built before the structure was known, have a vertex exactly where it
    // begins and ends — which is where the land stops meeting them. A structure the
    // builder was told of before the roads were built has its ends among the
    // vertices already, and does not move.
    let spans: Vec<RoadSpan> = spans
        .iter()
        .map(|span| {
            let length = map
                .road(&span.road)
                .and_then(|road| road.horizontal_length().ok())
                .unwrap_or(span.to);
            let stations = map.vertex_stations(&span.road).unwrap_or_default();
            let snap = |station: f64| {
                if !snapping {
                    return station;
                }
                stations
                    .iter()
                    .copied()
                    .min_by(|a, b| (a - station).abs().total_cmp(&(b - station).abs()))
                    .filter(|nearest| {
                        (nearest - station).abs() <= map.metadata.sampling.max_segment_length
                    })
                    .unwrap_or(station)
            };
            RoadSpan::new(
                span.road.clone(),
                snap(span.from.max(0.0)),
                snap(span.to.min(length)),
            )
        })
        .collect();
    if let Some(span) = spans.iter().find(|span| span.to - span.from < 1.0) {
        return Err(StructureError::Span(format!(
            "{} {:.1}–{:.1} is too short to carry",
            span.road, span.from, span.to
        )));
    }
    let id = structure_id(map, &spans, spec);
    if map.structures.contains(&id) {
        return Err(StructureError::Duplicate(id.to_string()));
    }

    let (curb, site) = match spec {
        StructureSpec::Bridge(bridge) => (bridge.curb, bridge.site.clone()),
        StructureSpec::Tunnel(tunnel) => (tunnel.walkway + tunnel.lining, tunnel.site.clone()),
    };
    let outline = Footprint::new(outline_of(map, &spans, curb)?)?;
    let primary = Strip::of(map, &spans[0])?;
    let carried: Vec<RoadId> = spans.iter().map(|span| span.road.clone()).collect();

    let kind = match spec {
        StructureSpec::Bridge(bridge) => {
            let width_at = |row: &(f64, Frame3, f64, f64)| {
                let (_, frame, _, _) = row;
                // The deck's width across the primary road at this row, from the
                // outline: the widest of every carried road.
                let mut low = f64::INFINITY;
                let mut high = f64::NEG_INFINITY;
                for point in outline.points() {
                    let d = frame.origin.horizontal_distance_to(*point);
                    if d > 60.0 {
                        continue;
                    }
                    let (dx, dy) = (point.x - frame.origin.x, point.y - frame.origin.y);
                    let along = dx * frame.tangent.x() + dy * frame.tangent.y();
                    if along.abs() > 3.0 {
                        continue;
                    }
                    let lateral = dx * frame.left.x() + dy * frame.left.y();
                    low = low.min(lateral);
                    high = high.max(lateral);
                }
                if low.is_finite() {
                    (low, high)
                } else {
                    (row.3 - bridge.curb, row.2 + bridge.curb)
                }
            };
            let underside = |station: f64| -> f64 {
                spans
                    .iter()
                    .filter_map(|span| {
                        let road = map.road(&span.road)?;
                        let s = if span.road == spans[0].road {
                            station
                        } else {
                            // The other roads at the same proportion along.
                            span.from
                                + (station - spans[0].from) / spans[0].length().max(1e-9)
                                    * span.length()
                        };
                        road.frame_at(s.clamp(span.from, span.to), map.metadata.sampling)
                            .ok()
                            .map(|frame| frame.origin.z)
                    })
                    .fold(f64::INFINITY, f64::min)
                    - bridge.deck_depth
            };
            let mut supports = Vec::new();
            let rows = &primary.rows;
            let heading_at = |frame: &Frame3| frame.tangent.y().atan2(frame.tangent.x());
            // The abutments: the deck's full width, six metres deep, inside each end.
            for (row, inward) in [(&rows[0], 1.0), (&rows[rows.len() - 1], -1.0)] {
                let (low, high) = width_at(row);
                let frame = row.1;
                let across = high - low;
                let middle = frame.offset((high + low) / 2.0);
                let depth = 6.0_f64.min(spans[0].length() / 3.0);
                let heading = heading_at(&frame);
                let centre = Point3::new(
                    middle.x + frame.tangent.x() * inward * depth / 2.0,
                    middle.y + frame.tangent.y() * inward * depth / 2.0,
                    middle.z,
                );
                supports.push(Support {
                    kind: SupportKind::Abutment,
                    footprint: Footprint::new(rectangle(
                        centre,
                        heading,
                        depth,
                        across,
                        underside(row.0),
                    ))?,
                    base: None,
                });
            }
            // The piers, evenly between the abutments, off every road below.
            let length = spans[0].length();
            let bays = (length / bridge.pier_spacing).ceil() as usize;
            if bays >= 2 {
                let below = carriageways(map, &carried);
                let bay = length / bays as f64;
                for index in 1..bays {
                    let wanted = spans[0].from + bay * index as f64;
                    let mut placed = None;
                    let mut nudge = 0.0;
                    while nudge <= bay * 0.5 {
                        for candidate in [wanted + nudge, wanted - nudge] {
                            if placed.is_some()
                                || candidate <= spans[0].from + 6.0
                                || candidate >= spans[0].to - 6.0
                            {
                                continue;
                            }
                            let road = map.road(&spans[0].road).expect("checked");
                            let frame = road.frame_at(candidate, map.metadata.sampling)?;
                            let (left, right) =
                                road_extent(map, &spans[0].road, candidate).unwrap_or((0.0, 0.0));
                            let (low, high) = width_at(&(candidate, frame, left, right));
                            let top = underside(candidate);
                            let heading = heading_at(&frame);
                            let width = high - low;
                            let columns: Vec<f64> = if width <= 15.0 {
                                vec![(high + low) / 2.0]
                            } else {
                                vec![low + 3.0, high - 3.0]
                            };
                            let rings: Vec<Vec<Point3>> = columns
                                .iter()
                                .map(|lateral| {
                                    rectangle(frame.offset(*lateral), heading, 2.0, 2.0, top)
                                })
                                .collect();
                            let clear = rings.iter().all(|ring| {
                                let plan: Vec<PlanPoint> =
                                    ring.iter().map(|p| [p.x, p.y]).collect();
                                below.iter().all(|(carriageway, low)| {
                                    *low >= top || !rings_overlap(&plan, carriageway)
                                })
                            });
                            if clear {
                                placed = Some(rings);
                            }
                        }
                        if placed.is_some() {
                            break;
                        }
                        nudge += 1.0;
                    }
                    for ring in placed.into_iter().flatten() {
                        supports.push(Support {
                            kind: SupportKind::Pier,
                            footprint: Footprint::new(ring)?,
                            base: None,
                        });
                    }
                }
            }
            StructureKind::Bridge(Bridge {
                form: bridge.form,
                material: bridge.material,
                deck_depth: bridge.deck_depth,
                supports,
            })
        }
        StructureSpec::Tunnel(tunnel) => {
            let portal = |row: &(f64, Frame3, f64, f64)| Portal {
                road: spans[0].road.clone(),
                station: row.0,
                left: row.1.offset(row.2),
                right: row.1.offset(row.3),
            };
            StructureKind::Tunnel(Tunnel {
                form: tunnel.form,
                clear_height: tunnel.clear_height,
                lining: tunnel.lining,
                lighting: tunnel.lighting,
                portals: [
                    portal(&primary.rows[0]),
                    portal(&primary.rows[primary.rows.len() - 1]),
                ],
            })
        }
    };

    for span in &spans {
        let road = map.roads.get_mut(&span.road).expect("checked");
        road.structure
            .insert(StructureSpan {
                from: span.from,
                to: span.to,
                structure: id.clone(),
            })
            .map_err(|clash| {
                StructureError::Span(format!(
                    "{} {:.1}–{:.1} is carried twice",
                    span.road, clash.from, clash.to
                ))
            })?;
    }
    map.structures
        .insert(
            id.clone(),
            Structure {
                id: id.clone(),
                name: spec.name().map(str::to_owned),
                kind,
                carries: spans,
                outline,
                site,
            },
        )
        .map_err(|duplicate| StructureError::Duplicate(duplicate.0.to_string()))?;
    Ok(id)
}

// ---------------------------------------------------------------------------------
// Where roads cross
// ---------------------------------------------------------------------------------

/// Where two roads' reference lines cross in plan, away from where they are joined.
#[derive(Debug, Clone, PartialEq)]
pub struct Crossing {
    pub a: RoadId,
    pub a_station: f64,
    pub a_z: f64,
    pub b: RoadId,
    pub b_station: f64,
    pub b_z: f64,
}

impl Crossing {
    /// The upper road and its station, then the lower.
    pub fn ordered(&self) -> ((&RoadId, f64, f64), (&RoadId, f64, f64)) {
        let a = (&self.a, self.a_station, self.a_z);
        let b = (&self.b, self.b_station, self.b_z);
        if self.a_z >= self.b_z {
            (a, b)
        } else {
            (b, a)
        }
    }
}

/// A road's reference line in plan, with each vertex's station and height.
fn track(map: &Map, road: &RoadId) -> Vec<(f64, Point3)> {
    map.road(road)
        .and_then(|entry| entry.reference_line.samples(map.metadata.sampling).ok())
        .map(|samples| {
            samples
                .into_iter()
                .map(|sample| (sample.station, sample.point))
                .collect()
        })
        .unwrap_or_default()
}

/// Every crossing between two roads that are not junction movements, more than a
/// metre from either road's ends — a road that ends on another is joined to it,
/// not crossing it.
pub fn crossings(map: &Map) -> Vec<Crossing> {
    let roads: Vec<&crate::map::Road> = map.roads.iter().filter(|r| !r.is_connector()).collect();
    let tracks: Vec<Vec<(f64, Point3)>> = roads.iter().map(|r| track(map, &r.id)).collect();
    let boxes: Vec<[f64; 4]> = tracks
        .iter()
        .map(|t| {
            t.iter().fold(
                [
                    f64::INFINITY,
                    f64::INFINITY,
                    f64::NEG_INFINITY,
                    f64::NEG_INFINITY,
                ],
                |b, (_, p)| [b[0].min(p.x), b[1].min(p.y), b[2].max(p.x), b[3].max(p.y)],
            )
        })
        .collect();
    let mut out = Vec::new();
    for i in 0..roads.len() {
        for j in (i + 1)..roads.len() {
            let (a, b) = (boxes[i], boxes[j]);
            if a[0] > b[2] || b[0] > a[2] || a[1] > b[3] || b[1] > a[3] {
                continue;
            }
            let (ta, tb) = (&tracks[i], &tracks[j]);
            let (la, lb) = (
                ta.last().map_or(0.0, |v| v.0),
                tb.last().map_or(0.0, |v| v.0),
            );
            for pa in ta.windows(2) {
                for pb in tb.windows(2) {
                    let Some((u, v)) = intersect(pa[0].1, pa[1].1, pb[0].1, pb[1].1) else {
                        continue;
                    };
                    let sa = pa[0].0 + (pa[1].0 - pa[0].0) * u;
                    let sb = pb[0].0 + (pb[1].0 - pb[0].0) * v;
                    if sa < 1.0 || sa > la - 1.0 || sb < 1.0 || sb > lb - 1.0 {
                        continue;
                    }
                    out.push(Crossing {
                        a: roads[i].id.clone(),
                        a_station: sa,
                        a_z: pa[0].1.z + (pa[1].1.z - pa[0].1.z) * u,
                        b: roads[j].id.clone(),
                        b_station: sb,
                        b_z: pb[0].1.z + (pb[1].1.z - pb[0].1.z) * v,
                    });
                }
            }
        }
    }
    out
}

/// Where segments `p–q` and `r–s` cross in plan, as fractions along each.
fn intersect(p: Point3, q: Point3, r: Point3, s: Point3) -> Option<(f64, f64)> {
    let (dx1, dy1) = (q.x - p.x, q.y - p.y);
    let (dx2, dy2) = (s.x - r.x, s.y - r.y);
    let denominator = dx1 * dy2 - dy1 * dx2;
    if denominator.abs() < 1e-12 {
        return None;
    }
    let (ex, ey) = (r.x - p.x, r.y - p.y);
    let u = (ex * dy2 - ey * dx2) / denominator;
    let v = (ex * dy1 - ey * dx1) / denominator;
    ((0.0..1.0).contains(&u) && (0.0..1.0).contains(&v)).then_some((u, v))
}

/// The stretch of `road` within half of `other`'s width (and `margin` more) of
/// where it crosses `other` at `station`.
pub fn span_across(
    map: &Map,
    road: &RoadId,
    station: f64,
    other: &RoadId,
    other_station: f64,
) -> RoadSpan {
    let half = road_extent(map, other, other_station)
        .map_or(5.0, |(left, right)| (left - right) / 2.0)
        + 1.0;
    let length = map
        .road(road)
        .and_then(|r| r.horizontal_length().ok())
        .unwrap_or(station + half);
    RoadSpan::new(
        road.clone(),
        (station - half).max(0.0),
        (station + half).min(length),
    )
}

/// Derives every grade separation the map's structures make: a road on a bridge
/// over another road, a road over a tunnel, a bridge over a land cover (a river).
/// Replaces whatever separations the map held.
pub fn derive_separations(map: &mut Map) {
    let mut found: Vec<(RoadSpan, Underneath, StructureId)> = Vec::new();
    for crossing in crossings(map) {
        let ((upper, us, _), (lower, ls, _)) = crossing.ordered();
        let upper_on = map
            .road(upper)
            .and_then(|r| r.structure.at(us))
            .filter(|id| {
                map.structures
                    .get(id)
                    .is_some_and(|s| s.kind.bridge().is_some())
            })
            .cloned();
        let lower_in = map
            .road(lower)
            .and_then(|r| r.structure.at(ls))
            .filter(|id| {
                map.structures
                    .get(id)
                    .is_some_and(|s| s.kind.tunnel().is_some())
            })
            .cloned();
        if let Some(via) = upper_on.or(lower_in) {
            found.push((
                span_across(map, upper, us, lower, ls),
                Underneath::Road(span_across(map, lower, ls, upper, us)),
                via,
            ));
        }
    }
    // Bridges over land covers that are water.
    for structure in map.structures.iter() {
        if structure.kind.bridge().is_none() {
            continue;
        }
        let deck: Vec<PlanPoint> = structure
            .outline
            .points()
            .iter()
            .map(|p| [p.x, p.y])
            .collect();
        for cover in map.land_covers.iter() {
            if !matches!(cover.kind.as_str(), "water" | "river" | "canal") {
                continue;
            }
            let area: Vec<PlanPoint> = cover.area.points().iter().map(|p| [p.x, p.y]).collect();
            if rings_overlap(&deck, &area) {
                for span in &structure.carries {
                    found.push((
                        span.clone(),
                        Underneath::LandCover(cover.id.clone()),
                        structure.id.clone(),
                    ));
                }
            }
        }
    }
    map.grade_separations.clear();
    let mut counts: HashMap<String, usize> = HashMap::new();
    for (over, under, via) in found {
        let under_name = match &under {
            Underneath::Road(span) => span.road.local_name().to_owned(),
            Underneath::LandCover(cover) => cover
                .as_str()
                .strip_prefix("landcover/")
                .unwrap_or(cover.as_str())
                .replace('/', "_"),
        };
        let key = format!("{}/{}", over.road.local_name(), under_name);
        let count = counts.entry(key.clone()).or_default();
        let id = GradeSeparationId::new(format!("{key}/{count}"));
        *count += 1;
        let _ = map.grade_separations.insert(
            id.clone(),
            GradeSeparation {
                id,
                over,
                under,
                via,
            },
        );
    }
}

/// Fills in what each interchange gathers beyond what its caller named — the
/// movements through its junctions, the structures its roads are on — and points
/// each of its junctions back at it.
pub fn refresh_interchanges(map: &mut Map) {
    let ids: Vec<InterchangeId> = map.interchanges.iter().map(|i| i.id.clone()).collect();
    for id in ids {
        let interchange = map.interchanges.get(&id).expect("listed").clone();
        let mut roads = interchange.roads.clone();
        for junction in &interchange.junctions {
            if let Some(entry) = map.junctions.get(junction) {
                for road in &entry.connecting_roads {
                    if !roads.contains(road) {
                        roads.push(road.clone());
                    }
                }
            }
        }
        let mut structures: Vec<StructureId> = Vec::new();
        for structure in map.structures.iter() {
            if structure
                .carries
                .iter()
                .any(|span| roads.contains(&span.road))
                && !structures.contains(&structure.id)
            {
                structures.push(structure.id.clone());
            }
        }
        for junction in &interchange.junctions {
            if let Some(entry) = map.junctions.get_mut(junction) {
                entry.interchange = Some(id.clone());
            }
        }
        let entry = map.interchanges.get_mut(&id).expect("listed");
        entry.roads = roads;
        entry.structures = structures;
    }
}

/// Whether a movement through a junction is carried too: both the roads it joins
/// are on a structure where it joins them — a junction on a viaduct.
pub fn connector_carried(map: &Map, road: &crate::map::Road) -> bool {
    if !road.is_connector() {
        return false;
    }
    let carried_end = |target: &Option<crate::topology::RoadLinkTarget>| {
        let Some(crate::topology::RoadLinkTarget::Road(endpoint)) = target else {
            return false;
        };
        let Some(other) = map.road(&endpoint.road) else {
            return false;
        };
        let station = match endpoint.end {
            crate::topology::RoadEnd::Start => 0.0,
            crate::topology::RoadEnd::End => other.horizontal_length().unwrap_or(0.0),
        };
        other.structure.at(station).is_some()
    };
    carried_end(&road.link.predecessor) && carried_end(&road.link.successor)
}

/// How far below the road surface the structure at a crossing reaches: a bridge's
/// deck depth, a tunnel's lining, nothing for a road at grade.
pub fn depth_at(map: &Map, road: &RoadId, station: f64) -> f64 {
    map.road(road)
        .and_then(|r| r.structure.at(station))
        .and_then(|id| map.structures.get(id))
        .map_or(0.0, |s| match &s.kind {
            StructureKind::Bridge(bridge) => bridge.deck_depth,
            StructureKind::Tunnel(tunnel) => tunnel.lining,
        })
}

// ---------------------------------------------------------------------------------
// Validation
// ---------------------------------------------------------------------------------

/// How far a road on a structure may climb or fall per metre before it is not a
/// road a vehicle could drive: well past any design standard, so only what is
/// physically wrong is refused.
const STEEPEST: f64 = 0.12;

/// Everything about the map's structures, separations and interchanges that only
/// becomes wrong once they are put together.
pub(crate) fn check(map: &Map, tolerance: f64, issues: &mut Vec<ValidationIssue>) {
    let implausible =
        |feature: String, detail: String| ValidationIssue::ImplausibleFeature { feature, detail };
    let dangling = |referrer: String, feature: String| ValidationIssue::DanglingFeatureReference {
        referrer,
        feature,
    };
    let tin = map
        .tin_surfaces
        .iter()
        .next()
        .map(|surface| surface.tin.locator());

    for structure in map.structures.iter() {
        let name = structure.id.to_string();
        if structure.carries.is_empty() {
            issues.push(implausible(name.clone(), "carries no road".into()));
        }
        for span in &structure.carries {
            let Some(road) = map.road(&span.road) else {
                issues.push(ValidationIssue::DanglingRoadReference {
                    referrer: name.clone(),
                    road: span.road.clone(),
                });
                continue;
            };
            let length = road.horizontal_length().unwrap_or(0.0);
            if span.from < -tolerance || span.to > length + tolerance || span.to <= span.from {
                issues.push(implausible(
                    name.clone(),
                    format!(
                        "carries {} {:.3}–{:.3}, which is not a stretch of it",
                        span.road, span.from, span.to
                    ),
                ));
            }
            let recorded = road.structure.spans().iter().any(|entry| {
                entry.structure == structure.id
                    && (entry.from - span.from).abs() <= tolerance
                    && (entry.to - span.to).abs() <= tolerance
            });
            if !recorded {
                issues.push(implausible(
                    name.clone(),
                    format!(
                        "carries {} {:.1}–{:.1}, which does not say it is on it",
                        span.road, span.from, span.to
                    ),
                ));
            }
            // What the structure stands on, or under, against the land.
            if let Some(tin) = &tin {
                check_against_land(map, structure, span, tin, issues);
            }
            // A grade no road could have.
            let step = 5.0;
            let mut station = span.from;
            while station + step <= span.to {
                if let (Ok(a), Ok(b)) = (
                    road.reference_line
                        .sample_at(station, map.metadata.sampling),
                    road.reference_line
                        .sample_at(station + step, map.metadata.sampling),
                ) {
                    let grade = (b.point.z - a.point.z) / step;
                    if grade.abs() > STEEPEST {
                        issues.push(implausible(
                            name.clone(),
                            format!(
                                "{} climbs {:.0} % at {station:.0} m on it; no road is that steep",
                                span.road,
                                grade.abs() * 100.0
                            ),
                        ));
                        break;
                    }
                }
                station += step;
            }
        }
        match &structure.kind {
            StructureKind::Bridge(bridge) => {
                if !(bridge.deck_depth.is_finite() && bridge.deck_depth > 0.0) {
                    issues.push(implausible(name.clone(), "a deck with no depth".into()));
                }
                for support in &bridge.supports {
                    let top = support.footprint.points()[0].z;
                    if let Some(base) = support.base {
                        if !(base.is_finite() && base <= top + tolerance) {
                            issues.push(implausible(
                                name.clone(),
                                format!(
                                    "a {} stands from {base:.2} m up to {top:.2} m",
                                    support.kind.as_str()
                                ),
                            ));
                        }
                    }
                }
            }
            StructureKind::Tunnel(tunnel) => {
                if !(tunnel.clear_height > 0.0 && tunnel.lining > 0.0) {
                    issues.push(implausible(
                        name.clone(),
                        "a tunnel needs a positive clear height and lining".into(),
                    ));
                }
                for portal in &tunnel.portals {
                    if !structure.carries.iter().any(|span| {
                        span.road == portal.road
                            && ((portal.station - span.from).abs() <= tolerance
                                || (portal.station - span.to).abs() <= tolerance)
                    }) {
                        issues.push(implausible(
                            name.clone(),
                            format!(
                                "has a portal at {} {:.1}, which is not an end of what it carries",
                                portal.road, portal.station
                            ),
                        ));
                    }
                }
            }
        }
    }

    for road in map.roads.iter() {
        for span in road.structure.spans() {
            match map.structures.get(&span.structure) {
                None => issues.push(dangling(road.id.to_string(), span.structure.to_string())),
                Some(structure) => {
                    if !structure.carries.iter().any(|carried| {
                        carried.road == road.id
                            && (carried.from - span.from).abs() <= tolerance
                            && (carried.to - span.to).abs() <= tolerance
                    }) {
                        issues.push(implausible(
                            road.id.to_string(),
                            format!(
                                "says {:.1}–{:.1} is on {}, which does not carry it",
                                span.from, span.to, span.structure
                            ),
                        ));
                    }
                }
            }
        }
    }

    for separation in map.grade_separations.iter() {
        let name = separation.id.to_string();
        if !map.structures.contains(&separation.via) {
            issues.push(dangling(name.clone(), separation.via.to_string()));
        }
        if !map.roads.contains(&separation.over.road) {
            issues.push(ValidationIssue::DanglingRoadReference {
                referrer: name.clone(),
                road: separation.over.road.clone(),
            });
        }
        match &separation.under {
            Underneath::Road(span) if !map.roads.contains(&span.road) => {
                issues.push(ValidationIssue::DanglingRoadReference {
                    referrer: name.clone(),
                    road: span.road.clone(),
                })
            }
            Underneath::LandCover(cover) if !map.land_covers.contains(cover) => {
                issues.push(dangling(name.clone(), cover.to_string()))
            }
            _ => {}
        }
    }

    for interchange in map.interchanges.iter() {
        let name = interchange.id.to_string();
        for junction in &interchange.junctions {
            match map.junctions.get(junction) {
                None => issues.push(ValidationIssue::DanglingJunctionReference {
                    referrer: name.clone(),
                    junction: junction.clone(),
                }),
                Some(entry) if entry.interchange.as_ref() != Some(&interchange.id) => {
                    issues.push(implausible(
                        junction.to_string(),
                        format!("listed by {} but does not name it", interchange.id),
                    ))
                }
                Some(_) => {}
            }
        }
        for road in &interchange.roads {
            if !map.roads.contains(road) {
                issues.push(ValidationIssue::DanglingRoadReference {
                    referrer: name.clone(),
                    road: road.clone(),
                });
            }
        }
        for structure in &interchange.structures {
            if !map.structures.contains(structure) {
                issues.push(dangling(name.clone(), structure.to_string()));
            }
        }
    }
    for junction in map.junctions.iter() {
        if let Some(interchange) = &junction.interchange {
            if !map
                .interchanges
                .get(interchange)
                .is_some_and(|entry| entry.junctions.contains(&junction.id))
            {
                issues.push(implausible(
                    junction.id.to_string(),
                    format!("names {interchange}, which does not list it"),
                ));
            }
        }
    }

    check_crossings(map, issues);
}

/// A bridge's deck clears the land under it, and the land over a tunnel clears its
/// lining — away from the ends, where the one meets the other.
fn check_against_land(
    map: &Map,
    structure: &Structure,
    span: &RoadSpan,
    tin: &crate::terrain::TinLocator<'_>,
    issues: &mut Vec<ValidationIssue>,
) {
    let Some(road) = map.road(&span.road) else {
        return;
    };
    let margin = 15.0_f64.min(span.length() / 3.0);
    let step = 5.0;
    let mut station = span.from + margin;
    while station <= span.to - margin {
        let Ok(sample) = road
            .reference_line
            .sample_at(station, map.metadata.sampling)
        else {
            break;
        };
        let p = sample.point;
        if let Some(ground) = tin.height_at(p.x, p.y) {
            match &structure.kind {
                StructureKind::Bridge(bridge) => {
                    if ground > p.z - bridge.deck_depth + 1e-3 {
                        issues.push(ValidationIssue::ImplausibleFeature {
                            feature: structure.id.to_string(),
                            detail: format!(
                                "the land at {:.1} m stands {:.2} m above the underside of the \
                                 deck carrying {} at {station:.0} m",
                                ground,
                                ground - (p.z - bridge.deck_depth),
                                span.road
                            ),
                        });
                        return;
                    }
                }
                StructureKind::Tunnel(tunnel) => {
                    let crown = p.z + tunnel.clear_height + tunnel.lining;
                    if ground < crown - 1e-3 {
                        issues.push(ValidationIssue::ImplausibleFeature {
                            feature: structure.id.to_string(),
                            detail: format!(
                                "the land at {ground:.1} m lies {:.2} m below the top of the \
                                 tunnel round {} at {station:.0} m",
                                crown - ground,
                                span.road
                            ),
                        });
                        return;
                    }
                }
            }
        }
        station += step;
    }
}

/// A motorway, a slip road and a road on a structure cross nothing on the level:
/// wherever one of them crosses another road away from a junction, one of the two
/// has to be carried over the other by a structure, and a grade separation has to
/// say so.
///
/// A crossing between two ordinary roads at grade is left alone. A map may draw
/// streets that cross without a junction, and always could.
fn check_crossings(map: &Map, issues: &mut Vec<ValidationIssue>) {
    for crossing in crossings(map) {
        let ((upper, us, uz), (lower, ls, lz)) = crossing.ordered();
        let (Some(upper_road), Some(lower_road)) = (map.road(upper), map.road(lower)) else {
            continue;
        };
        let strict = upper_road.form.is_controlled_access()
            || lower_road.form.is_controlled_access()
            || upper_road.structure.at(us).is_some()
            || lower_road.structure.at(ls).is_some();
        if !strict {
            continue;
        }
        let needed = depth_at(map, upper, us).max(depth_at(map, lower, ls));
        let recorded = map.grade_separations.iter().any(|separation| {
            separation.over.road == *upper
                && separation.over.contains(us)
                && matches!(&separation.under, Underneath::Road(span)
                    if span.road == *lower && span.contains(ls))
        });
        if uz - lz < needed.max(0.5) || !recorded {
            issues.push(ValidationIssue::UnseparatedCrossing {
                over: upper.clone(),
                under: lower.clone(),
                detail: if uz - lz < needed.max(0.5) {
                    format!(
                        "{upper} at {us:.1} m is only {:.2} m above {lower} at {ls:.1} m",
                        uz - lz
                    )
                } else {
                    format!(
                        "{upper} at {us:.1} m crosses {lower} at {ls:.1} m with nothing \
                         carrying one over the other"
                    )
                },
            });
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_profile_keeps_its_spans_apart_and_in_order() {
        let mut profile = StructureProfile::at_grade();
        let id = StructureId::new("b");
        profile
            .insert(StructureSpan {
                from: 50.0,
                to: 80.0,
                structure: id.clone(),
            })
            .unwrap();
        profile
            .insert(StructureSpan {
                from: 10.0,
                to: 20.0,
                structure: id.clone(),
            })
            .unwrap();
        assert!(profile
            .insert(StructureSpan {
                from: 70.0,
                to: 90.0,
                structure: id.clone()
            })
            .is_err());
        assert_eq!(profile.spans()[0].from, 10.0);
        assert_eq!(profile.at(60.0), Some(&id));
        assert_eq!(profile.at(30.0), None);
    }
}
