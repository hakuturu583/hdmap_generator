//! Stable, human-readable identifiers.
//!
//! Each kind of identifier is its own type, so passing a [`LaneId`] where a
//! [`RoadId`] belongs does not compile. Values are derived from what the caller asked
//! for rather than from a counter, so the same input map always produces the same
//! identifiers — regenerating a map and diffing the result is meaningful.
//!
//! Identifiers here are the IR's own. OpenDRIVE ids and Lanelet2 ids are assigned by
//! the exporters, from these, and never flow back.

use std::fmt;
use std::sync::Arc;

macro_rules! define_id {
    ($(#[$doc:meta])* $name:ident, $prefix:literal) => {
        $(#[$doc])*
        #[derive(Clone, PartialEq, Eq, PartialOrd, Ord, Hash)]
        pub struct $name(Arc<str>);

        impl $name {
            /// The prefix every identifier of this kind carries.
            pub const PREFIX: &'static str = $prefix;

            /// Builds the identifier for `key`, prefixing it with the kind.
            pub fn new(key: impl AsRef<str>) -> Self {
                $name(Arc::from(format!("{}/{}", $prefix, key.as_ref()).as_str()))
            }

            /// Wraps an already-prefixed string, for round-tripping.
            pub fn from_raw(raw: impl AsRef<str>) -> Self {
                $name(Arc::from(raw.as_ref()))
            }

            /// Reads an identifier the way it is printed, with the prefix or without
            /// it: both `lane/north/0` and `north/0` name the same lane.
            ///
            /// This is what a file written by hand needs. `new` would turn the
            /// printed form into `lane/lane/north/0`, and `from_raw` would take the
            /// bare form as if it were already prefixed.
            pub fn parse_printed(text: impl AsRef<str>) -> Self {
                let text = text.as_ref();
                match text.strip_prefix($prefix) {
                    Some(rest) if rest.starts_with('/') => $name::from_raw(text),
                    _ => $name::new(text),
                }
            }

            pub fn as_str(&self) -> &str {
                &self.0
            }
        }

        impl fmt::Display for $name {
            fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
                f.write_str(&self.0)
            }
        }

        impl fmt::Debug for $name {
            fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
                write!(f, "{}({})", stringify!($name), &self.0)
            }
        }
    };
}

define_id!(
    /// Identifies a road, e.g. `road/north`.
    RoadId, "road");
define_id!(
    /// Identifies a lane, e.g. `lane/north/0`.
    LaneId, "lane");
define_id!(
    /// Identifies a junction, e.g. `junction/j0`.
    JunctionId, "junction");
define_id!(
    /// Identifies a lane-to-lane connection, e.g.
    /// `connection/j0/north_0/east_0`.
    ConnectionId, "connection");
define_id!(
    /// Identifies a map object such as a traffic light or a stop line.
    ObjectId, "object");

impl LaneId {
    /// The identifier of lane number `index` of `road`.
    ///
    /// `index` is the lane's position in the cross-section as the caller wrote it,
    /// so inserting a lane renames only the lanes after it.
    pub fn of_road(road: &RoadId, index: usize) -> Self {
        LaneId::new(format!("{}/{}", road.local_name(), index))
    }
}

impl RoadId {
    /// The part after the `road/` prefix.
    pub fn local_name(&self) -> &str {
        self.as_str()
            .strip_prefix("road/")
            .unwrap_or_else(|| self.as_str())
    }
}

impl LaneId {
    /// The part after the `lane/` prefix, with `/` replaced so it can be embedded in
    /// a composite identifier without looking like another path segment.
    pub fn local_name(&self) -> String {
        self.as_str()
            .strip_prefix("lane/")
            .unwrap_or_else(|| self.as_str())
            .replace('/', "_")
    }
}

impl JunctionId {
    pub fn local_name(&self) -> &str {
        self.as_str()
            .strip_prefix("junction/")
            .unwrap_or_else(|| self.as_str())
    }
}

impl ConnectionId {
    /// The identifier of a connection, named after what it joins.
    ///
    /// Two calls with the same lanes and the same junction produce the same value,
    /// which is what makes a regenerated map comparable to its predecessor.
    pub fn between(junction: Option<&JunctionId>, from: &LaneId, to: &LaneId) -> Self {
        match junction {
            Some(junction) => ConnectionId::new(format!(
                "{}/{}/{}",
                junction.local_name(),
                from.local_name(),
                to.local_name()
            )),
            None => ConnectionId::new(format!("{}/{}", from.local_name(), to.local_name())),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn identifiers_read_the_way_the_design_notes_write_them() {
        let north = RoadId::new("north");
        assert_eq!(north.to_string(), "road/north");
        assert_eq!(LaneId::of_road(&north, 1).to_string(), "lane/north/1");
        assert_eq!(JunctionId::new("j0").to_string(), "junction/j0");

        let junction = JunctionId::new("j0");
        let from = LaneId::of_road(&RoadId::new("north"), 0);
        let to = LaneId::of_road(&RoadId::new("east"), 0);
        assert_eq!(
            ConnectionId::between(Some(&junction), &from, &to).to_string(),
            "connection/j0/north_0/east_0"
        );
    }

    #[test]
    fn an_identifier_reads_the_same_with_or_without_its_prefix() {
        assert_eq!(
            LaneId::parse_printed("lane/north/0"),
            LaneId::new("north/0")
        );
        assert_eq!(
            LaneId::parse_printed("lane/north/0").as_str(),
            "lane/north/0"
        );
        // A name that merely starts with the prefix's letters is not prefixed.
        assert_eq!(
            RoadId::parse_printed("roadworks").as_str(),
            "road/roadworks"
        );
    }

    #[test]
    fn the_same_input_names_the_same_identifier() {
        assert_eq!(RoadId::new("north"), RoadId::new("north"));
        assert_ne!(RoadId::new("north"), RoadId::new("south"));
    }
}
