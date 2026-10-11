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
use crate::geometry::plan::{bounds, ring_contains, rings_overlap, segments_cross, PlanPoint};
use crate::geometry::{Curve3, Frame3, Point3};
use crate::id::{
    GradeSeparationId, InterchangeId, JunctionId, LandCoverId, LaneId, ObjectId, RoadId, SiteId,
    StructureId,
};
use crate::map::Map;
use crate::semantics::{LaneType, MapObject, MapObjectKind, ObjectGeometry};
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
    /// How far the deck reaches past the road's outer edge: the kerb its parapet
    /// stands on, metres.
    pub curb: f64,
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
    /// The inspection walkway each side of the road, inside the bore, metres.
    pub walkway: f64,
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

    /// Whether `station` is strictly inside a span: on a structure and not at
    /// either end of it, where the road meets the ground.
    pub fn carries_strictly(&self, station: f64) -> bool {
        self.spans
            .iter()
            .any(|span| station > span.from + 1e-6 && station < span.to - 1e-6)
    }

    /// Whether a span begins at `station`.
    pub fn begins_at(&self, station: f64) -> bool {
        self.spans
            .iter()
            .any(|span| (station - span.from).abs() <= 1e-6)
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

/// How tall a bridge's parapet stands above its deck, metres.
pub const PARAPET: f64 = 1.1;
/// What a bridge's parapets are called, as a [`MapObjectKind::Barrier`]'s kind.
pub const PARAPET_KIND: &str = "parapet";
/// The widest gap between two roads one deck carries side by side, metres: a
/// median. Wider than this, the two are on the same structure but each at its own
/// edge of it.
pub const MEDIAN: f64 = 15.0;

/// The clear height a road needs under anything that crosses over it, metres.
pub const CLEARANCE: f64 = 4.5;
/// An overpass's deck: a short span, so a shallower one than a viaduct's, metres.
pub const OVERPASS_DECK: f64 = 1.8;

impl BridgeSpec {
    /// The defaults for a bridge of `form`: an overpass's deck is shallower.
    pub fn for_form(form: BridgeForm) -> Self {
        BridgeSpec {
            form,
            deck_depth: if form == BridgeForm::Overpass {
                OVERPASS_DECK
            } else {
                BridgeSpec::default().deck_depth
            },
            ..BridgeSpec::default()
        }
    }
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
    pub fn name(&self) -> Option<&str> {
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
    extent(map, road, station, |_| true)
}

/// The same as [`road_extent`], counting only the lanes traffic drives on and their
/// shoulders — where a pier must not stand.
pub fn carriageway_extent(map: &Map, road: &RoadId, station: f64) -> Option<(f64, f64)> {
    extent(map, road, station, |lane| {
        matches!(
            lane.lane_type,
            LaneType::Driving | LaneType::Shoulder | LaneType::Biking | LaneType::Parking
        )
    })
}

/// How far out each side the outermost lane `keep` takes reaches, the lanes inside
/// it counted whatever they are.
fn extent(
    map: &Map,
    road: &RoadId,
    station: f64,
    keep: impl Fn(&crate::map::Lane) -> bool,
) -> Option<(f64, f64)> {
    let entry = map.road(road)?;
    let section = entry.section_at(station)?;
    let origin = entry.lane_offset.evaluate(station);
    let mut lanes = map.lanes_of_section(road, section);
    lanes.sort_by_key(|lane| lane.ordinal);
    let (mut left, mut right) = (origin, origin);
    let (mut outer_left, mut outer_right) = (origin, origin);
    for lane in lanes {
        let width = lane.width_at(station);
        match lane.side {
            LateralSide::Left => {
                left += width;
                if keep(lane) {
                    outer_left = left;
                }
            }
            LateralSide::Right => {
                right -= width;
                if keep(lane) {
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
/// each road it carries. A bridge's parapets go on the map with it, as barriers
/// that belong to it (see [`shared_sides`] for where there is none).
///
/// Piers are spaced evenly at no more than the spec's spacing and kept off the
/// carriageway of every road below; one that has nowhere to stand within half a
/// span of where it belongs is left out. Where the deck carries roads side by
/// side, no more than a [`MEDIAN`] apart, they stand in a single row down the
/// middle of the gap between them rather than under each road.
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
                // A straight road has a vertex only at each end, so the nearest
                // may be far: then the end stays where it was asked for.
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
            // The other roads where they pass nearest the first: two carriageways
            // on one deck run opposite ways, so the same proportion along each is
            // opposite ends of the deck.
            let others = spans[1..]
                .iter()
                .map(|span| Strip::of(map, span))
                .collect::<Result<Vec<_>, _>>()?;
            let underside = |station: f64| -> f64 {
                let Some(here) = map
                    .road(&spans[0].road)
                    .and_then(|road| {
                        road.frame_at(
                            station.clamp(spans[0].from, spans[0].to),
                            map.metadata.sampling,
                        )
                        .ok()
                    })
                    .map(|frame| frame.origin)
                else {
                    return f64::INFINITY;
                };
                others
                    .iter()
                    .map(|other| other.rows[other.locate(here).0].1.origin.z)
                    .fold(here.z, f64::min)
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
            // The other carried roads running beside the first at a frame of it:
            // `None` where there are none, or else the middle of each gap between
            // two neighbours across the deck — the median — as offsets to its left.
            let beside_here = |frame: &Frame3, left: f64, right: f64| -> Option<Vec<f64>> {
                let mut bands = vec![(right, left)];
                for (span, other) in spans[1..].iter().zip(&others) {
                    // Another stretch of the same road is further along it.
                    if span.road == spans[0].road {
                        continue;
                    }
                    let (row, _) = other.locate(frame.origin);
                    let (_, near, l, r) = other.rows[row];
                    let (dx, dy) = (
                        near.origin.x - frame.origin.x,
                        near.origin.y - frame.origin.y,
                    );
                    // Only where it passes alongside, not off past its end.
                    if (dx * frame.tangent.x() + dy * frame.tangent.y()).abs() > 3.0 {
                        continue;
                    }
                    let across = frame.left.get();
                    let norm = (across.x * across.x + across.y * across.y).sqrt().max(1e-9);
                    let lateral = |p: Point3| {
                        ((p.x - frame.origin.x) * across.x + (p.y - frame.origin.y) * across.y)
                            / norm
                    };
                    let (a, b) = (lateral(near.offset(l)), lateral(near.offset(r)));
                    bands.push((a.min(b), a.max(b)));
                }
                if bands.len() < 2 {
                    return None;
                }
                bands.sort_by(|a, b| a.0.total_cmp(&b.0));
                // Two roads further apart than a median are each at their own edge
                // of the deck, not side by side.
                let gaps: Vec<(f64, f64)> = bands
                    .windows(2)
                    .map(|pair| (pair[0].1, pair[1].0))
                    .filter(|(from, to)| to - from <= MEDIAN)
                    .collect();
                if gaps.is_empty() {
                    return None;
                }
                Some(
                    gaps.into_iter()
                        .filter(|(from, to)| to - from > 1e-6)
                        .map(|(from, to)| (from + to) / 2.0)
                        .collect(),
                )
            };
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
                            let columns: Vec<f64> = match beside_here(&frame, left, right) {
                                // Roads side by side on one deck: a single row of
                                // columns down the median, one in each gap between
                                // them, or under the deck's middle where they touch.
                                Some(gaps) if gaps.is_empty() => vec![(high + low) / 2.0],
                                Some(gaps) => gaps,
                                None if width <= 15.0 => vec![(high + low) / 2.0],
                                None => vec![low + 3.0, high - 3.0],
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
                curb: bridge.curb,
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
                walkway: tunnel.walkway,
                portals: [
                    portal(&primary.rows[0]),
                    portal(&primary.rows[primary.rows.len() - 1]),
                ],
            })
        }
    };

    // A bridge's parapets, worked out before anything is changed, so a clash leaves
    // the map as it was.
    let parapets = match spec {
        StructureSpec::Bridge(bridge) => parapets(map, &id, &spans, bridge.curb)?,
        StructureSpec::Tunnel(_) => Vec::new(),
    };
    if let Some(clash) = parapets
        .iter()
        .find(|parapet| map.objects.contains(&parapet.id))
    {
        return Err(StructureError::Duplicate(clash.id.to_string()));
    }

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
    for parapet in parapets {
        map.objects
            .insert(parapet.id.clone(), parapet)
            .map_err(|duplicate| StructureError::Duplicate(duplicate.0.to_string()))?;
    }
    Ok(id)
}

/// Which sides of the road `spans[index]` carries have another of the carried
/// roads running beside them on the same deck, left then right: the side of a
/// carriageway that faces the other across the median.
///
/// Decided once, in the middle of the stretch, where the roads beside it are
/// nearest: two carriageways on one deck run side by side the length of it. A road
/// is beside a side when its nearer edge is no further out from that side's edge
/// than [`MEDIAN`].
pub fn shared_sides(map: &Map, spans: &[RoadSpan], index: usize) -> [bool; 2] {
    let Some(span) = spans.get(index) else {
        return [false, false];
    };
    let Some(road) = map.road(&span.road) else {
        return [false, false];
    };
    let middle = (span.from + span.to) / 2.0;
    let (Ok(frame), Some((left, right))) = (
        road.frame_at(middle, map.metadata.sampling),
        road_extent(map, &span.road, middle),
    ) else {
        return [false, false];
    };
    let across = frame.left.get();
    let norm = (across.x * across.x + across.y * across.y).sqrt().max(1e-9);
    // The other roads' edges where they pass nearest, as offsets to this one's left.
    let beside: Vec<f64> = spans
        .iter()
        .enumerate()
        // Another stretch of the same road is further along it, not beside it.
        .filter(|(other, stretch)| *other != index && stretch.road != span.road)
        .filter_map(|(_, other)| Strip::of(map, other).ok())
        .filter_map(|strip| {
            let (row, _) = strip.locate(frame.origin);
            strip.rows.get(row).copied()
        })
        .flat_map(|(_, near, l, r)| [near.offset(l), near.offset(r)])
        .map(|p| ((p.x - frame.origin.x) * across.x + (p.y - frame.origin.y) * across.y) / norm)
        .collect();
    [(left, 1.0), (right, -1.0)].map(|(edge, side)| {
        beside
            .iter()
            .map(|lateral| (lateral - edge) * side)
            .any(|gap| gap > -1e-6 && gap <= MEDIAN)
    })
}

/// The lane nearest one side's edge of a road in a section: its outermost lane on
/// that side, or, where the road has none on that side — a one-way carriageway's
/// left — its innermost on the other.
fn edge_lane(map: &Map, road: &RoadId, section: usize, side: LateralSide) -> Option<LaneId> {
    let lanes = map.lanes_of_section(road, section);
    lanes
        .iter()
        .filter(|lane| lane.side == side)
        .max_by_key(|lane| lane.ordinal)
        .or_else(|| {
            lanes
                .iter()
                .filter(|lane| lane.side != side)
                .min_by_key(|lane| lane.ordinal)
        })
        .map(|lane| lane.id.clone())
}

/// A bridge's parapets: one along each outer side of each road its deck carries,
/// standing on the curb from the road's edge to the deck's, except down a side that
/// faces another of the carried roads across the median.
///
/// Each is a [`MapObjectKind::Barrier`] of kind [`PARAPET_KIND`] belonging to the
/// bridge, as GDF holds a safety barrier: road furniture, related to the road it
/// lines — through the lanes along it, one per cross-section — and to the
/// structure it is part of.
fn parapets(
    map: &Map,
    structure: &StructureId,
    spans: &[RoadSpan],
    curb: f64,
) -> Result<Vec<MapObject>, StructureError> {
    let key = structure
        .as_str()
        .strip_prefix("structure/")
        .unwrap_or(structure.as_str());
    let mut out = Vec::new();
    for (index, span) in spans.iter().enumerate() {
        let shared = shared_sides(map, spans, index);
        let strip = Strip::of(map, span)?;
        let road = map
            .road(&span.road)
            .ok_or_else(|| StructureError::NoSuchRoad(span.road.clone()))?;
        let sections = road.section_at(span.from).unwrap_or(0)
            ..=road
                .section_at(span.to - 1e-6)
                .unwrap_or(road.sections.len().saturating_sub(1));
        for (shared, side, name) in [
            (shared[0], LateralSide::Left, "left"),
            (shared[1], LateralSide::Right, "right"),
        ] {
            if shared {
                continue;
            }
            // Its band, left edge and right edge looking along the road: on the left
            // the deck's edge is the band's left; on the right, the road's is.
            let (inner, outer): (Vec<Point3>, Vec<Point3>) = strip
                .rows
                .iter()
                .map(|(_, frame, left, right)| match side {
                    LateralSide::Left => (frame.offset(*left), frame.offset(left + curb)),
                    LateralSide::Right => (frame.offset(*right), frame.offset(right - curb)),
                })
                .unzip();
            let (left, right) = match side {
                LateralSide::Left => (outer, inner),
                LateralSide::Right => (inner, outer),
            };
            let lanes = sections
                .clone()
                .filter_map(|section| edge_lane(map, &span.road, section, side))
                .collect();
            // A road the structure carries more than one stretch of has a parapet
            // per stretch, told apart by the stretch's place among them.
            let repeats = spans.iter().filter(|other| other.road == span.road).count() > 1;
            let stretch = if repeats {
                format!(
                    "/{}",
                    spans[..index]
                        .iter()
                        .filter(|other| other.road == span.road)
                        .count()
                )
            } else {
                String::new()
            };
            out.push(MapObject {
                id: ObjectId::new(format!(
                    "{PARAPET_KIND}/{key}/{}{stretch}/{name}",
                    span.road.local_name()
                )),
                kind: MapObjectKind::Barrier {
                    kind: PARAPET_KIND.to_owned(),
                    height: PARAPET,
                    structure: Some(structure.clone()),
                },
                geometry: ObjectGeometry::Band {
                    left: Curve3::polyline(left)?,
                    right: Curve3::polyline(right)?,
                },
                lanes,
            });
        }
    }
    Ok(out)
}

// ---------------------------------------------------------------------------------
// Where roads cross
// ---------------------------------------------------------------------------------

/// Where two roads cross in plan, away from where they are joined: where their
/// reference lines cross, or where the outline of one reaches into the other's.
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

    /// The structure that already carries one road over the other here: a bridge
    /// the upper is on, or a tunnel the lower is in.
    pub fn carried_by(&self, map: &Map) -> Option<StructureId> {
        let ((upper, us, _), (lower, ls, _)) = self.ordered();
        let on = |road: &RoadId, station: f64, bridge: bool| {
            map.road(road)
                .and_then(|r| r.structure.at(station))
                .filter(|id| {
                    map.structures.get(id).is_some_and(|s| {
                        if bridge {
                            s.kind.bridge().is_some()
                        } else {
                            s.kind.tunnel().is_some()
                        }
                    })
                })
                .cloned()
        };
        on(upper, us, true).or_else(|| on(lower, ls, false))
    }

    /// Whether the crossing has to be separated: either road is a motorway or a
    /// slip road, or is on a structure here.
    pub fn strict(&self, map: &Map) -> bool {
        [(&self.a, self.a_station), (&self.b, self.b_station)]
            .iter()
            .any(|(road, station)| {
                map.road(road).is_some_and(|r| {
                    r.form.is_controlled_access() || r.structure.at(*station).is_some()
                })
            })
    }
}

/// Whether anything on the map can make a crossing matter: a structure, or a road
/// that crosses nothing on the level. A map with neither — every town written
/// before this module — has no crossings worth looking for.
pub fn separates(map: &Map) -> bool {
    !map.structures.is_empty()
        || map
            .roads
            .iter()
            .any(|road| road.form.is_controlled_access())
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

/// How far inside its outer edges a road's outline is taken when looking for
/// outlines that overlap, metres: two outlines have to reach twice this far into
/// each other to cross. Carriageways drawn side by side, edge to edge, touch
/// without crossing, and the chords of two sampled curves along one line do not
/// make a crossing either.
const OUTLINE_INSET: f64 = 0.25;

/// A road in plan for finding where it crosses others: its reference line, and its
/// outline as a closed ring — the left edge out, the far end, the right edge back,
/// the near end — each vertex with its station.
struct RoadPlan {
    track: Vec<(f64, Point3)>,
    rim: Vec<(f64, Point3)>,
    ring: Vec<PlanPoint>,
    bounds: [f64; 4],
}

impl RoadPlan {
    fn of(map: &Map, road: &crate::map::Road) -> RoadPlan {
        let track = track(map, &road.id);
        let (mut lefts, mut rights) = (Vec::new(), Vec::new());
        for station in rim_stations(map, road, &track) {
            if let Some((left, right)) = inset_edges(map, road, station) {
                lefts.push((station, left));
                rights.push((station, right));
            }
        }
        rights.reverse();
        let mut rim = lefts;
        rim.extend(rights);
        let ring: Vec<PlanPoint> = rim.iter().map(|(_, p)| [p.x, p.y]).collect();
        let bounds = bounds(&ring);
        RoadPlan {
            track,
            rim,
            ring,
            bounds,
        }
    }

    fn meets(&self, other: &RoadPlan) -> bool {
        boxes_meet(&self.bounds, &other.bounds)
    }
}

/// Where a road's outline turns: every vertex the road is drawn with — the
/// reference line's, where a cross-section takes over, where a lane's width
/// changes — and a hair before each cross-section, so that where lanes appear or
/// end the outline steps out or in there rather than slanting from the vertex
/// before. A straight road's reference line alone has only its two ends.
fn rim_stations(map: &Map, road: &crate::map::Road, track: &[(f64, Point3)]) -> Vec<f64> {
    let mut stations: Vec<f64> = track.iter().map(|(station, _)| *station).collect();
    if let Ok(samples) = map.vertex_samples(&road.id) {
        stations.extend(samples.iter().map(|sample| sample.station));
    }
    let length = track.last().map_or(0.0, |(station, _)| *station);
    stations.extend(
        road.sections
            .iter()
            .map(|section| section.station - 1e-6)
            .filter(|station| *station > 0.0 && *station < length),
    );
    stations.sort_by(f64::total_cmp);
    stations.dedup_by(|a, b| (*a - *b).abs() < 1e-9);
    stations
}

/// A road's outer edges at a station, each [`OUTLINE_INSET`] in from the edge.
fn inset_edges(map: &Map, road: &crate::map::Road, station: f64) -> Option<(Point3, Point3)> {
    let frame = road.frame_at(station, map.metadata.sampling).ok()?;
    let (left, right) = road_extent(map, &road.id, station)?;
    let inset = OUTLINE_INSET.min((left - right) / 4.0).max(0.0);
    Some((frame.offset(left - inset), frame.offset(right + inset)))
}

/// Whether two roads are joined, so that their outlines meeting is where one runs
/// into the other and no crossing: one follows on from the other, or both come to
/// the same junction.
fn joined(map: &Map, a: &crate::map::Road, b: &crate::map::Road) -> bool {
    use crate::topology::RoadLinkTarget;
    let follows = |from: &crate::map::Road, to: &RoadId| {
        [&from.link.predecessor, &from.link.successor]
            .into_iter()
            .any(|target| matches!(target, Some(RoadLinkTarget::Road(end)) if end.road == *to))
    };
    follows(a, &b.id)
        || follows(b, &a.id)
        || map.junctions.iter().any(|junction| {
            junction.incoming_roads.contains(&a.id) && junction.incoming_roads.contains(&b.id)
        })
}

/// Every crossing between two roads that are not junction movements.
///
/// Where the reference lines cross, more than a metre from either road's ends — a
/// road that ends on another is joined to it, not crossing it. Where they do not
/// but the outlines overlap, and the two roads are not joined to each other
/// ([`joined`]), the overlap is a crossing too, one for each stretch the two
/// share: a road that runs into the edge of another, or ends under it.
pub fn crossings(map: &Map) -> Vec<Crossing> {
    let roads: Vec<&crate::map::Road> = map.roads.iter().filter(|r| !r.is_connector()).collect();
    let plans: Vec<RoadPlan> = roads.iter().map(|r| RoadPlan::of(map, r)).collect();
    let mut out = Vec::new();
    for i in 0..roads.len() {
        for j in (i + 1)..roads.len() {
            if !plans[i].meets(&plans[j]) {
                continue;
            }
            let before = out.len();
            cross_tracks(&roads[i].id, &plans[i], &roads[j].id, &plans[j], &mut out);
            if out.len() == before && !joined(map, roads[i], roads[j]) {
                overlap_outlines(map, roads[i], &plans[i], roads[j], &plans[j], &mut out);
            }
        }
    }
    out
}

/// Where two reference lines cross, more than a metre from either's ends.
fn cross_tracks(a: &RoadId, pa: &RoadPlan, b: &RoadId, pb: &RoadPlan, out: &mut Vec<Crossing>) {
    let (ta, tb) = (&pa.track, &pb.track);
    let (la, lb) = (
        ta.last().map_or(0.0, |v| v.0),
        tb.last().map_or(0.0, |v| v.0),
    );
    for sa in ta.windows(2) {
        for sb in tb.windows(2) {
            let Some((u, v)) = intersect(sa[0].1, sa[1].1, sb[0].1, sb[1].1) else {
                continue;
            };
            let (station_a, station_b) = (
                sa[0].0 + (sa[1].0 - sa[0].0) * u,
                sb[0].0 + (sb[1].0 - sb[0].0) * v,
            );
            if station_a < 1.0 || station_a > la - 1.0 || station_b < 1.0 || station_b > lb - 1.0 {
                continue;
            }
            out.push(Crossing {
                a: a.clone(),
                a_station: station_a,
                a_z: sa[0].1.z + (sa[1].1.z - sa[0].1.z) * u,
                b: b.clone(),
                b_station: station_b,
                b_z: sb[0].1.z + (sb[1].1.z - sb[0].1.z) * v,
            });
        }
    }
}

/// Where two outlines overlap: every place an edge of one crosses an edge of the
/// other, gathered into the stretches the two share — two places along `a` are one
/// stretch when `a`, half way between them, still reaches into `b`. Each stretch is
/// one crossing, at the middle of where its edges cross and at the heights of the
/// two surfaces there.
fn overlap_outlines(
    map: &Map,
    a: &crate::map::Road,
    pa: &RoadPlan,
    b: &crate::map::Road,
    pb: &RoadPlan,
    out: &mut Vec<Crossing>,
) {
    let edges = |rim: &[(f64, Point3)]| {
        (0..rim.len())
            .map(|k| (rim[k], rim[(k + 1) % rim.len()]))
            .collect::<Vec<_>>()
    };
    let (ea, eb) = (edges(&pa.rim), edges(&pb.rim));
    // (station on a, station on b, height of a, height of b)
    let mut hits: Vec<(f64, f64, f64, f64)> = Vec::new();
    for (p, q) in &ea {
        if !boxes_meet(&pb.bounds, &bounds(&[[p.1.x, p.1.y], [q.1.x, q.1.y]])) {
            continue;
        }
        for (r, s) in &eb {
            let Some((u, v)) = intersect(p.1, q.1, r.1, s.1) else {
                continue;
            };
            hits.push((
                p.0 + (q.0 - p.0) * u,
                r.0 + (s.0 - r.0) * v,
                p.1.z + (q.1.z - p.1.z) * u,
                r.1.z + (s.1.z - r.1.z) * v,
            ));
        }
    }
    if hits.is_empty() {
        // No edge crosses an edge, yet one outline may lie inside the other — the
        // two run side by side end to end, or one is drawn wholly within the
        // other. Where any point of either outline is inside the other's, the two
        // share ground: one crossing, at the middle of those points.
        if let Some(inside) = overlap_inside(pa, pb) {
            out.push(Crossing {
                a: a.id.clone(),
                a_station: inside.0,
                a_z: inside.2,
                b: b.id.clone(),
                b_station: inside.1,
                b_z: inside.3,
            });
        }
        return;
    }
    hits.sort_by(|x, y| x.0.total_cmp(&y.0));
    let mut stretches: Vec<Vec<(f64, f64, f64, f64)>> = Vec::new();
    for hit in hits {
        let same = stretches.last().and_then(|s| s.last()).is_some_and(|last| {
            let middle = (last.0 + hit.0) / 2.0;
            (hit.0 - last.0).abs() < 1e-9 || reaches_into(map, a, middle, &pb.ring)
        });
        match stretches.last_mut() {
            Some(stretch) if same => stretch.push(hit),
            _ => stretches.push(vec![hit]),
        }
    }
    for stretch in stretches {
        let n = stretch.len() as f64;
        let mean =
            |pick: fn(&(f64, f64, f64, f64)) -> f64| stretch.iter().map(pick).sum::<f64>() / n;
        out.push(Crossing {
            a: a.id.clone(),
            a_station: mean(|h| h.0),
            a_z: mean(|h| h.2),
            b: b.id.clone(),
            b_station: mean(|h| h.1),
            b_z: mean(|h| h.3),
        });
    }
}

/// Where two outlines whose edges never cross overlap: the points of each outline
/// that are inside the other's — its vertices, and the middle of each edge, so a
/// straight road drawn with only its ends is asked along its length too — as
/// (station on a, station on b, height of a, height of b), averaged. Nothing when
/// no point of either is inside the other.
fn overlap_inside(pa: &RoadPlan, pb: &RoadPlan) -> Option<(f64, f64, f64, f64)> {
    let mut inside: Vec<(f64, f64, f64, f64)> = Vec::new();
    for (own, other, flip) in [(pa, pb, false), (pb, pa, true)] {
        let count = own.rim.len();
        for k in 0..count {
            let (p, q) = (own.rim[k], own.rim[(k + 1) % count]);
            let middle = (
                (p.0 + q.0) / 2.0,
                Point3::new(
                    (p.1.x + q.1.x) / 2.0,
                    (p.1.y + q.1.y) / 2.0,
                    (p.1.z + q.1.z) / 2.0,
                ),
            );
            for (station, point) in [p, middle] {
                if !ring_contains(&other.ring, [point.x, point.y]) {
                    continue;
                }
                let Some((there, height)) = project(&other.track, point) else {
                    continue;
                };
                inside.push(if flip {
                    (there, station, height, point.z)
                } else {
                    (station, there, point.z, height)
                });
            }
        }
    }
    if inside.is_empty() {
        return None;
    }
    let n = inside.len() as f64;
    let mean = |pick: fn(&(f64, f64, f64, f64)) -> f64| inside.iter().map(pick).sum::<f64>() / n;
    Some((mean(|h| h.0), mean(|h| h.1), mean(|h| h.2), mean(|h| h.3)))
}

/// The station on a reference line nearest `point` in plan, and the line's height
/// there.
fn project(track: &[(f64, Point3)], point: Point3) -> Option<(f64, f64)> {
    let mut best: Option<(f64, f64, f64)> = None;
    for pair in track.windows(2) {
        let ((s0, a), (s1, b)) = (pair[0], pair[1]);
        let (dx, dy) = (b.x - a.x, b.y - a.y);
        let length = dx * dx + dy * dy;
        let t = if length < 1e-18 {
            0.0
        } else {
            (((point.x - a.x) * dx + (point.y - a.y) * dy) / length).clamp(0.0, 1.0)
        };
        let distance = (point.x - a.x - t * dx).hypot(point.y - a.y - t * dy);
        if best.is_none_or(|(d, _, _)| distance < d) {
            best = Some((distance, s0 + (s1 - s0) * t, a.z + (b.z - a.z) * t));
        }
    }
    best.map(|(_, station, height)| (station, height))
}

/// Whether two plan bounds from [`bounds`] share any ground.
fn boxes_meet(a: &[f64; 4], b: &[f64; 4]) -> bool {
    a[0] <= b[2] && b[0] <= a[2] && a[1] <= b[3] && b[1] <= a[3]
}

/// Whether `road`, across its width at `station`, reaches into `ring`.
fn reaches_into(map: &Map, road: &crate::map::Road, station: f64, ring: &[PlanPoint]) -> bool {
    let Some((left, right)) = inset_edges(map, road, station) else {
        return false;
    };
    let (l, r) = ([left.x, left.y], [right.x, right.y]);
    ring_contains(ring, l)
        || ring_contains(ring, r)
        || (0..ring.len()).any(|k| segments_cross(l, r, ring[k], ring[(k + 1) % ring.len()]))
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
    if map.structures.is_empty() {
        map.grade_separations.clear();
        return;
    }
    for crossing in crossings(map) {
        if let Some(via) = crossing.carried_by(map) {
            let ((upper, us, _), (lower, ls, _)) = crossing.ordered();
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
/// wherever one of them crosses another road away from a junction — their
/// reference lines cross, or their outlines overlap ([`crossings`]) — one of the
/// two has to be carried over the other by a structure, and a grade separation has
/// to say so.
///
/// A crossing between two ordinary roads at grade is left alone. A map may draw
/// streets that cross without a junction, and always could.
fn check_crossings(map: &Map, issues: &mut Vec<ValidationIssue>) {
    if !separates(map) {
        return;
    }
    for crossing in crossings(map) {
        if !crossing.strict(map) {
            continue;
        }
        let ((upper, us, uz), (lower, ls, lz)) = crossing.ordered();
        let needed = depth_at(map, upper, us).max(depth_at(map, lower, ls));
        let recorded = map.grade_separations.iter().any(|separation| {
            separation.over.road == *upper
                && separation.over.contains(us)
                && matches!(&separation.under, Underneath::Road(span)
                    if span.road == *lower && span.contains(ls))
        });
        let too_close = uz - lz < needed.max(0.5);
        if too_close || !recorded {
            issues.push(ValidationIssue::UnseparatedCrossing {
                over: upper.clone(),
                under: lower.clone(),
                detail: if too_close {
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
