//! Where each element of the IR ended up in a written file.
//!
//! Every exporter numbers its output its own way — OpenDRIVE by the IR's order,
//! Lanelet2 and OpenStreetMap with counters, ClipGT and GPUDrive by row — and none of
//! those numbers flow back into the IR. A [`Trace`] is the record an exporter keeps of
//! what it did instead: one [`TraceLink`] per (IR element, written element) pair.
//!
//! Traces of different formats meet at the IR's identifiers. Going from a lanelet to
//! a SUMO lane is the Lanelet2 trace read backwards and the SUMO trace read forwards;
//! no format is ever mapped onto another directly, so adding an exporter adds one
//! trace and changes none.
//!
//! Nothing here knows a file format. The written element is named by a string of the
//! form `<kind>:<local>` — `lanelet:1000123`, `lane:0/0/-1`, `lane:north.fwd_0` —
//! whose meaning is the exporter's to define, and serialising a trace is left to
//! `roadgen-trace`, so this crate stays free of a JSON dependency.

use std::fmt;
use std::path::PathBuf;

use crate::id::{BuildingId, BuildingPartId, ConnectionId, JunctionId, LaneId, ObjectId, RoadId};

/// Any element of the IR, by its identifier.
///
/// A [`TrafficRule`](crate::semantics::TrafficRule) has no identifier of its own, so
/// it is named by its position in [`Map::rules`](crate::map::Map::rules).
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum IrRef {
    Road(RoadId),
    Lane(LaneId),
    Junction(JunctionId),
    Connection(ConnectionId),
    Object(ObjectId),
    Building(BuildingId),
    BuildingPart(BuildingPartId),
    Rule(usize),
}

impl IrRef {
    /// The prefix a rule is printed with. The other kinds print as their identifier,
    /// which already carries its own.
    pub const RULE_PREFIX: &'static str = "rule";

    /// Reads a reference the way [`Display`](fmt::Display) prints it.
    pub fn parse(text: &str) -> Option<IrRef> {
        let (prefix, rest) = text.split_once('/')?;
        if rest.is_empty() {
            return None;
        }
        Some(match prefix {
            RoadId::PREFIX => IrRef::Road(RoadId::from_raw(text)),
            LaneId::PREFIX => IrRef::Lane(LaneId::from_raw(text)),
            JunctionId::PREFIX => IrRef::Junction(JunctionId::from_raw(text)),
            ConnectionId::PREFIX => IrRef::Connection(ConnectionId::from_raw(text)),
            ObjectId::PREFIX => IrRef::Object(ObjectId::from_raw(text)),
            BuildingId::PREFIX => IrRef::Building(BuildingId::from_raw(text)),
            BuildingPartId::PREFIX => IrRef::BuildingPart(BuildingPartId::from_raw(text)),
            IrRef::RULE_PREFIX => IrRef::Rule(rest.parse().ok()?),
            _ => return None,
        })
    }
}

impl fmt::Display for IrRef {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            IrRef::Road(id) => f.write_str(id.as_str()),
            IrRef::Lane(id) => f.write_str(id.as_str()),
            IrRef::Junction(id) => f.write_str(id.as_str()),
            IrRef::Connection(id) => f.write_str(id.as_str()),
            IrRef::Object(id) => f.write_str(id.as_str()),
            IrRef::Building(id) => f.write_str(id.as_str()),
            IrRef::BuildingPart(id) => f.write_str(id.as_str()),
            IrRef::Rule(index) => write!(f, "{}/{index}", IrRef::RULE_PREFIX),
        }
    }
}

impl From<RoadId> for IrRef {
    fn from(id: RoadId) -> Self {
        IrRef::Road(id)
    }
}
impl From<LaneId> for IrRef {
    fn from(id: LaneId) -> Self {
        IrRef::Lane(id)
    }
}
impl From<JunctionId> for IrRef {
    fn from(id: JunctionId) -> Self {
        IrRef::Junction(id)
    }
}
impl From<ConnectionId> for IrRef {
    fn from(id: ConnectionId) -> Self {
        IrRef::Connection(id)
    }
}
impl From<ObjectId> for IrRef {
    fn from(id: ObjectId) -> Self {
        IrRef::Object(id)
    }
}
impl From<BuildingId> for IrRef {
    fn from(id: BuildingId) -> Self {
        IrRef::Building(id)
    }
}
impl From<BuildingPartId> for IrRef {
    fn from(id: BuildingPartId) -> Self {
        IrRef::BuildingPart(id)
    }
}

/// How many IR elements and written elements a link joins.
///
/// A format rarely draws the IR one element for one element, and a trace that
/// pretended it did would answer a reverse lookup with half the truth.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum Relation {
    /// One IR element is one written element.
    Exact,
    /// The IR element was split, and this written element is one of the pieces —
    /// a road written as one SUMO edge per direction.
    Part,
    /// Several IR elements were written as one, and this IR element is one of
    /// them — a boundary two lanelets share.
    Merged,
    /// The IR element has no counterpart of its own, and was absorbed into this
    /// written element — a junction connector, which SUMO writes as a connection.
    Collapsed,
}

impl Relation {
    /// How one of `sources` IR elements relates to a written element they all
    /// became: exactly it when it is the only one, else one of those merged into it.
    pub fn shared_by(sources: usize) -> Self {
        if sources > 1 {
            Relation::Merged
        } else {
            Relation::Exact
        }
    }

    pub fn as_str(self) -> &'static str {
        match self {
            Relation::Exact => "exact",
            Relation::Part => "part",
            Relation::Merged => "merged",
            Relation::Collapsed => "collapsed",
        }
    }

    pub fn parse(value: &str) -> Option<Relation> {
        Some(match value {
            "exact" => Relation::Exact,
            "part" => Relation::Part,
            "merged" => Relation::Merged,
            "collapsed" => Relation::Collapsed,
            _ => return None,
        })
    }
}

/// One IR element and one element of the written file.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct TraceLink {
    pub ir: IrRef,
    /// The written element, as `<kind>:<local>`.
    pub local: String,
    pub relation: Relation,
    /// Which of several written elements this is, when one IR element becomes more
    /// than one — `lanelet`, `centerline`, `left_boundary`.
    pub role: Option<String>,
}

/// What an exporter wrote for each element of the IR.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct Trace {
    /// The format's name: `opendrive`, `lanelet2`, `sumo`, ….
    pub format: String,
    /// The files the export wrote, when it wrote any.
    pub files: Vec<PathBuf>,
    /// In the order the exporter wrote them.
    pub links: Vec<TraceLink>,
}

impl Trace {
    pub fn new(format: impl Into<String>) -> Self {
        Trace {
            format: format.into(),
            ..Trace::default()
        }
    }

    /// Records that `ir` was written as `local`, which is `<kind>:<local>`.
    pub fn link(&mut self, ir: impl Into<IrRef>, local: impl Into<String>, relation: Relation) {
        self.push(ir.into(), local.into(), relation, None);
    }

    /// Records a link with the role the written element plays for `ir`.
    pub fn link_as(
        &mut self,
        ir: impl Into<IrRef>,
        local: impl Into<String>,
        relation: Relation,
        role: impl Into<String>,
    ) {
        self.push(ir.into(), local.into(), relation, Some(role.into()));
    }

    fn push(&mut self, ir: IrRef, local: String, relation: Relation, role: Option<String>) {
        self.links.push(TraceLink {
            ir,
            local,
            relation,
            role,
        });
    }

    /// Every link to one IR element.
    pub fn links_of(&self, ir: &IrRef) -> impl Iterator<Item = &TraceLink> + '_ {
        let ir = ir.clone();
        self.links.iter().filter(move |link| link.ir == ir)
    }

    /// Every link to one written element.
    pub fn links_to(&self, local: &str) -> impl Iterator<Item = &TraceLink> + '_ {
        let local = local.to_owned();
        self.links.iter().filter(move |link| link.local == local)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_reference_reads_back_the_way_it_prints() {
        for text in [
            "road/north",
            "lane/north/0",
            "junction/j0",
            "connection/j0/north_0/east_0",
            "object/tl_0",
            "building/north/left/3/0",
            "part/north/left/3/0/1",
            "rule/4",
        ] {
            let parsed = IrRef::parse(text).expect(text);
            assert_eq!(parsed.to_string(), text);
        }
        assert_eq!(IrRef::parse("road/"), None);
        assert_eq!(IrRef::parse("rule/x"), None);
        assert_eq!(IrRef::parse("lanelet/3"), None);
    }

    #[test]
    fn links_are_found_from_either_side() {
        let mut trace = Trace::new("test");
        let lane = IrRef::Lane(LaneId::new("a/0"));
        trace.link(lane.clone(), "lanelet:1", Relation::Exact);
        trace.link_as(
            lane.clone(),
            "linestring:2",
            Relation::Merged,
            "left_boundary",
        );
        trace.link(LaneId::new("a/1"), "linestring:2", Relation::Merged);
        assert_eq!(trace.links_of(&lane).count(), 2);
        assert_eq!(trace.links_to("linestring:2").count(), 2);
        assert_eq!(
            Relation::parse(Relation::Collapsed.as_str()),
            Some(Relation::Collapsed)
        );
    }
}
