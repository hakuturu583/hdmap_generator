//! The land as a TIN: a constrained Delaunay triangulation of the ground's points,
//! with every edge the land breaks along kept as an edge of it.
//!
//! The breaklines go in last, after every free point, and the triangulation is then
//! asked which of their segments it actually kept. A segment can fail to become an
//! edge — it would cross another breakline, or a point lies exactly on it — and
//! such a segment is left out of the breakline rather than left in it untrue: the
//! invariant the IR states is that every segment of a breakline is an edge of its
//! surface, and this is the one place that can make it so.

use roadgen_core::geometry::{Point3, Polyline3};
use roadgen_core::terrain::{Tin, TinError};
use spade::handles::FixedVertexHandle;
use spade::{ConstrainedDelaunayTriangulation, HasPosition, Point2, Triangulation};

/// A line the land breaks along, before it is triangulated.
pub struct Constraint {
    /// What it is the edge of: `crest`, `toe`, `road_edge`.
    pub kind: String,
    /// The identifier it is published under, before any split; see [`Breakline`].
    pub key: String,
    pub points: Vec<Point3>,
    /// Whether its last vertex joins back to its first.
    pub closed: bool,
}

/// A breakline as triangulated: the stretch of a constraint the TIN kept whole.
pub struct Breakline {
    pub kind: String,
    /// The constraint's key, with `/<n>` after it when the constraint came out in
    /// several pieces.
    pub key: String,
    pub line: Polyline3,
}

struct Vertex {
    at: Point3,
}

impl HasPosition for Vertex {
    type Scalar = f64;

    fn position(&self) -> Point2<f64> {
        Point2::new(self.at.x, self.at.y)
    }
}

/// Triangulates `free` and `constraints` into one TIN, and says which stretches of
/// each constraint are edges of it.
pub fn build(
    free: Vec<Point3>,
    constraints: Vec<Constraint>,
) -> Result<(Tin, Vec<Breakline>), TinError> {
    let mut cdt: ConstrainedDelaunayTriangulation<Vertex> =
        ConstrainedDelaunayTriangulation::bulk_load(
            free.into_iter().map(|at| Vertex { at }).collect(),
        )
        .unwrap_or_default();

    // Every constraint's vertices first, then its segments: a vertex that lands on
    // a segment already constrained would split it.
    let mut handles: Vec<Vec<FixedVertexHandle>> = Vec::with_capacity(constraints.len());
    for constraint in &constraints {
        let mut own = Vec::with_capacity(constraint.points.len());
        for &at in &constraint.points {
            if let Ok(handle) = cdt.insert(Vertex { at }) {
                if own.last() != Some(&handle) {
                    own.push(handle);
                }
            }
        }
        handles.push(own);
    }

    // Each constraint's segments, and whether each became a constraint edge.
    let mut kept: Vec<Vec<bool>> = Vec::with_capacity(constraints.len());
    for (constraint, own) in constraints.iter().zip(&handles) {
        let mut segments = Vec::new();
        let count = own.len();
        let pairs = if constraint.closed && count > 2 {
            count
        } else {
            count.saturating_sub(1)
        };
        for index in 0..pairs {
            let (from, to) = (own[index], own[(index + 1) % count]);
            let ok = from != to && cdt.can_add_constraint(from, to) && cdt.add_constraint(from, to);
            segments.push(ok);
        }
        kept.push(segments);
    }
    // A segment that went in may still not be one edge — a vertex lying on it splits
    // it into two — and only one edge is a breakline segment.
    for (own, segments) in handles.iter().zip(kept.iter_mut()) {
        let count = own.len();
        for (index, ok) in segments.iter_mut().enumerate() {
            if *ok {
                let (from, to) = (own[index], own[(index + 1) % count]);
                *ok = cdt.get_edge_from_neighbors(from, to).is_some();
            }
        }
    }

    // The nodes in the triangulation's own order, so a handle's index is its node.
    let nodes: Vec<Point3> = cdt.vertices().map(|vertex| vertex.data().at).collect();
    let triangles: Vec<[u32; 3]> = cdt
        .inner_faces()
        .map(|face| face.vertices().map(|vertex| vertex.fix().index() as u32))
        .filter(|&[a, b, c]| {
            let (a, b, c) = (nodes[a as usize], nodes[b as usize], nodes[c as usize]);
            ((b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x)).abs() / 2.0 >= Tin::MIN_AREA
        })
        .collect();

    let mut breaklines = Vec::new();
    for ((constraint, own), segments) in constraints.iter().zip(&handles).zip(&kept) {
        let pieces = pieces(own, segments, constraint.closed);
        let several = pieces.len() > 1;
        for (index, piece) in pieces.into_iter().enumerate() {
            let points = piece.iter().map(|handle| nodes[handle.index()]);
            let Ok(line) = Polyline3::new(points) else {
                continue;
            };
            breaklines.push(Breakline {
                kind: constraint.kind.clone(),
                key: if several {
                    format!("{}/{index}", constraint.key)
                } else {
                    constraint.key.clone()
                },
                line,
            });
        }
    }
    Ok((Tin::new(nodes, triangles)?, breaklines))
}

/// The runs of consecutive kept segments, as the vertices along each. A closed
/// constraint kept whole comes back as one run whose last vertex is its first.
fn pieces(
    own: &[FixedVertexHandle],
    segments: &[bool],
    closed: bool,
) -> Vec<Vec<FixedVertexHandle>> {
    let count = own.len();
    if segments.is_empty() {
        return Vec::new();
    }
    if closed && segments.iter().all(|&ok| ok) {
        let mut ring = own.to_vec();
        ring.push(own[0]);
        return vec![ring];
    }
    // Start just after a broken segment, so that a closed constraint's run across
    // its seam stays one run.
    let start = if closed {
        segments
            .iter()
            .position(|&ok| !ok)
            .map_or(0, |broken| broken + 1)
    } else {
        0
    };
    let mut runs = Vec::new();
    let mut current: Vec<FixedVertexHandle> = Vec::new();
    for step in 0..segments.len() {
        let index = (start + step) % segments.len();
        if segments[index] {
            if current.is_empty() {
                current.push(own[index]);
            }
            current.push(own[(index + 1) % count]);
        } else if current.len() >= 2 {
            runs.push(std::mem::take(&mut current));
        } else {
            current.clear();
        }
    }
    if current.len() >= 2 {
        runs.push(current);
    }
    runs
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_breakline_crossing_another_is_cut_where_it_crosses() {
        let free = vec![
            Point3::new(-20.0, -20.0, 0.0),
            Point3::new(20.0, -20.0, 0.0),
            Point3::new(20.0, 20.0, 0.0),
            Point3::new(-20.0, 20.0, 0.0),
        ];
        let constraints = vec![
            Constraint {
                kind: "crest".into(),
                key: "a".into(),
                points: vec![Point3::new(-10.0, 0.0, 1.0), Point3::new(10.0, 0.0, 1.0)],
                closed: false,
            },
            Constraint {
                kind: "toe".into(),
                key: "b".into(),
                points: vec![
                    Point3::new(0.0, -10.0, 2.0),
                    Point3::new(0.0, -5.0, 2.0),
                    Point3::new(0.0, 5.0, 2.0),
                    Point3::new(0.0, 10.0, 2.0),
                ],
                closed: false,
            },
        ];
        let (tin, breaklines) = build(free, constraints).unwrap();
        assert!(!tin.is_empty());
        let edges = tin.edges();
        // `b` crosses `a` between its second and third vertices, so it comes back
        // as the two stretches either side.
        let keys: Vec<&str> = breaklines.iter().map(|line| line.key.as_str()).collect();
        assert_eq!(keys, ["a", "b/0", "b/1"]);
        let index = |point: Point3| {
            tin.nodes()
                .iter()
                .position(|node| node.x == point.x && node.y == point.y)
                .unwrap() as u32
        };
        for breakline in &breaklines {
            for pair in breakline.line.points().windows(2) {
                let (a, b) = (index(pair[0]), index(pair[1]));
                assert!(edges.contains(&(a.min(b), a.max(b))));
            }
        }
    }
}
