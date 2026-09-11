//! An independent reader of the OpenDRIVE geometry this project writes.
//!
//! Checking that the two exports agree needs a way to ask the OpenDRIVE document
//! where a lane actually is, and asking `roadgen-opendrive` would only prove it
//! agrees with itself. So this walks `<planView>`, `<elevationProfile>`,
//! `<laneOffset>` and the lane widths the way the specification says to, using only
//! the parsed document.

use opendrive::core::OpenDrive;
use opendrive::lane::lane_choice::LaneChoice;
use opendrive::road::geometry::geometry_type::GeometryType;
use opendrive::road::Road as OdRoad;

/// A position in the document's coordinates.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Position {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Position {
    pub fn distance_to(self, other: Position) -> f64 {
        ((self.x - other.x).powi(2) + (self.y - other.y).powi(2) + (self.z - other.z).powi(2))
            .sqrt()
    }
}

/// Evaluates one `<road>` of a parsed document.
pub struct RoadEvaluator<'a> {
    road: &'a OdRoad,
}

impl<'a> RoadEvaluator<'a> {
    pub fn new(road: &'a OdRoad) -> Self {
        RoadEvaluator { road }
    }

    /// Finds the road with the given id.
    pub fn find(document: &'a OpenDrive, id: &str) -> Option<Self> {
        document
            .road
            .iter()
            .find(|road| road.id == id)
            .map(RoadEvaluator::new)
    }

    /// The reference line's position and heading at plan-view station `s`.
    fn reference_at(&self, s: f64) -> (f64, f64, f64) {
        let geometry = self
            .road
            .plan_view
            .geometry
            .iter()
            .rfind(|geometry| geometry.s.value <= s + 1e-9)
            .unwrap_or_else(|| self.road.plan_view.geometry.first());
        let ds = s - geometry.s.value;
        let hdg = geometry.hdg.value;
        match &geometry.r#type {
            GeometryType::Arc(arc) => {
                let curvature = arc.curvature.value;
                let heading = hdg + curvature * ds;
                if curvature.abs() < 1e-12 {
                    (
                        geometry.x.value + ds * hdg.cos(),
                        geometry.y.value + ds * hdg.sin(),
                        heading,
                    )
                } else {
                    (
                        geometry.x.value + (heading.sin() - hdg.sin()) / curvature,
                        geometry.y.value - (heading.cos() - hdg.cos()) / curvature,
                        heading,
                    )
                }
            }
            // Everything this project writes other than an arc is a straight
            // segment; a document using paramPoly3 would need more here.
            _ => (
                geometry.x.value + ds * hdg.cos(),
                geometry.y.value + ds * hdg.sin(),
                hdg,
            ),
        }
    }

    fn elevation_at(&self, s: f64) -> f64 {
        let Some(profile) = &self.road.elevation_profile else {
            return 0.0;
        };
        let Some(entry) = profile
            .elevation
            .iter()
            .rfind(|entry| entry.s <= s + 1e-9)
            .or_else(|| profile.elevation.first())
        else {
            return 0.0;
        };
        let ds = s - entry.s;
        entry.a + entry.b * ds + entry.c * ds * ds + entry.d * ds * ds * ds
    }

    fn lane_offset_at(&self, s: f64) -> f64 {
        let Some(entry) = self
            .road
            .lanes
            .lane_offset
            .iter()
            .rfind(|entry| entry.s <= s + 1e-9)
        else {
            return 0.0;
        };
        let ds = s - entry.s;
        entry.a + entry.b * ds + entry.c * ds * ds + entry.d * ds * ds * ds
    }

    /// Widths of the lanes on one side, ordered outwards from the centre.
    fn widths(&self, left: bool) -> Vec<(i64, f64)> {
        let section = self.road.lanes.lane_section.first();
        let width_of = |lane: &opendrive::lane::Lane| {
            lane.choice
                .iter()
                .find_map(|choice| match choice {
                    LaneChoice::Width(width) => Some(width.a),
                    LaneChoice::Border(_) => None,
                })
                .unwrap_or(0.0)
        };
        let mut widths: Vec<(i64, f64)> = if left {
            section
                .left
                .as_ref()
                .map(|side| {
                    side.lane
                        .iter()
                        .map(|lane| (lane.id, width_of(&lane.base)))
                        .collect()
                })
                .unwrap_or_default()
        } else {
            section
                .right
                .as_ref()
                .map(|side| {
                    side.lane
                        .iter()
                        .map(|lane| (lane.id, width_of(&lane.base)))
                        .collect()
                })
                .unwrap_or_default()
        };
        widths.sort_by_key(|(id, _)| id.abs());
        widths
    }

    /// The lateral offset of the middle of lane `id` at station `s`.
    pub fn lane_center_offset(&self, id: i64, s: f64) -> Option<f64> {
        let mut offset = self.lane_offset_at(s);
        for (lane_id, width) in self.widths(id > 0) {
            let half = width / 2.0;
            offset += if id > 0 { half } else { -half };
            if lane_id == id {
                return Some(offset);
            }
            offset += if id > 0 { half } else { -half };
        }
        None
    }

    /// The centre of lane `id` at station `s`.
    pub fn lane_center(&self, id: i64, s: f64) -> Option<Position> {
        let (x, y, heading) = self.reference_at(s);
        let t = self.lane_center_offset(id, s)?;
        Some(Position {
            // `t` is positive to the left of the reference line.
            x: x - t * heading.sin(),
            y: y + t * heading.cos(),
            z: self.elevation_at(s),
        })
    }

    pub fn length(&self) -> f64 {
        self.road.length.value
    }
}
