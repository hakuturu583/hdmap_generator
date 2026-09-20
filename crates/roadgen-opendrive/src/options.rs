//! What a caller can tell the exporter that the IR does not.
//!
//! The IR knows where a traffic light's *bar* is and which lanes it governs. It does
//! not know where the pole stands, which way the lamps face, or what a country's
//! catalogue calls the sign — those are physical facts about street furniture, and
//! the exporter that builds the furniture is the one that knows them. This is how it
//! says so: a placement per object, keyed by the object's id, that the signal is
//! written from in place of the object's own geometry.
//!
//! A map exported without any of this is exported the way it always was: every
//! signal at the middle of its geometry, with the caller's own catalogue code.

use std::collections::HashMap;

use roadgen_core::geometry::Point3;
use roadgen_core::id::ObjectId;

/// Everything the exporter can be told beyond the map itself.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Options {
    /// Where a signal physically stands, by the object it is written for.
    pub signals: HashMap<ObjectId, SignalPlacement>,
}

/// Where one signal stands and what it is called.
#[derive(Debug, Clone, PartialEq)]
pub struct SignalPlacement {
    /// The foot of the post, in the map's own coordinates. This becomes the
    /// signal's `t`, `zOffset` and `<positionInertial>`; its `s` is where the
    /// signal *applies* — [`SignalPlacement::applies_at`], else the object's own
    /// geometry — because a consumer builds its stop boxes from `s`.
    pub position: Point3,
    /// Where the signal applies, when that is not where its bar is: the stop line
    /// of the rule it belongs to. Only its station along the road is used.
    pub applies_at: Option<Point3>,
    /// The direction the signal faces, radians anticlockwise from east, or `None`
    /// to leave the heading unwritten.
    pub heading: Option<f64>,
    /// The catalogue entry to write, or `None` to write the object's own code.
    pub catalogue: Option<SignalCatalogue>,
}

/// A signal's identity in some country's catalogue.
#[derive(Debug, Clone, PartialEq)]
pub struct SignalCatalogue {
    /// ISO 3166-1 alpha-2, such as `DE`.
    pub country: Option<String>,
    /// The catalogue's `type`, such as `206` for a German stop sign.
    pub kind: String,
    /// The catalogue's `subtype`, `-1` when it has none.
    pub subtype: String,
    /// The signal's value, when it carries one: a speed limit's limit, km/h.
    pub speed_kph: Option<f64>,
}
