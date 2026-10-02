//! Edge weights for `randomTrips.py`: where a random car trip may begin, end and pass
//! through.
//!
//! # Why the network needs them
//!
//! `randomTrips.py` draws a trip's first and last edge at random from the built
//! network, and a drawn pair is only a trip if a route runs from the one to the other.
//! A generated network is short of routes in two ways a network imported from
//! OpenStreetMap usually is not:
//!
//! - Every road end the IR does not link anywhere is a `dead_end` node, and the
//!   `.netccfg` turns turnarounds off because the IR has none. So the edge running
//!   *into* a dead end cannot be left, and the edge running *out of* one cannot be
//!   reached: a trip that starts on the first or ends on the second has no route.
//! - A road's footway is a SUMO edge in its own right when the road has nothing else
//!   (and a lane of a mixed edge otherwise), and `randomTrips.py` does not look at
//!   permissions unless it is asked to with `--vclass`. A car trip drawn onto a
//!   pedestrian-only edge has no route either.
//!
//! `randomTrips.py` reads a weight per edge from three `edgedata` files given with
//! `--weights-prefix <prefix>` — `<prefix>.src.xml` for where trips depart,
//! `<prefix>.dst.xml` for where they arrive and `<prefix>.via.xml` for the
//! intermediate points of `--intermediate` — and an edge it draws with weight 0 is
//! never drawn. Those files replace its own choice of edges entirely, so what is
//! written here is the whole of what decides where trips go. They are written as
//! `<name>.safe.src.xml`, `<name>.safe.dst.xml` and `<name>.safe.via.xml`, the names
//! `autowarefoundation/lanelet2_to_sumo` gives the same files, and used as
//! `--weights-prefix <name>.safe`.
//!
//! # What gets a weight
//!
//! The weights are for **passenger cars**, `randomTrips.py`'s default vehicle class.
//! The graph they are read from has an edge for every SUMO edge with at least one lane
//! a passenger car may use, and a link from one edge to the next wherever a written
//! connection joins two such lanes. Every other edge — a footway, a cycle track, a hard
//! shoulder — is written with weight 0, so that the files name every edge of the
//! network and say outright that a car trip has no business there.
//!
//! Over that graph:
//!
//! | file | an edge is weighted when |
//! | --- | --- |
//! | `src` | it has a way out: some connection leaves it |
//! | `dst` | it has a way in: some connection arrives on it |
//! | `via` | it has both |
//!
//! and in each case only within the largest weakly connected part of the graph.
//! A weighted edge carries its length — the longest of its lanes, and never less than a
//! millimetre — so that, as with `randomTrips.py`'s own `--length`, a long road is
//! drawn more often than a short one, and zero means "never" rather than "short".
//!
//! # Why that rule, and not another
//!
//! `lanelet2_to_sumo` gives an edge its weight in all three files only when it has
//! both a way in and a way out. That is safe for a network that is mostly through
//! roads, and wrong for this one: with turnarounds off, every arm of a junction that
//! runs out to a dead end is an edge with only a way in (the carriageway leaving the
//! junction) and an edge with only a way out (the one approaching it). The rule would
//! give a crossroads — or any of the tree-shaped networks the generator draws — no
//! weighted edge at all, and `randomTrips.py` would refuse to run. Asking each file
//! only for what *it* needs keeps the arms: a trip may depart on the approach to a
//! junction and arrive on the carriageway leaving it, which is exactly the trip a
//! dead-end arm is for. Arriving on an edge that leads into a dead end is perfectly
//! good — the car simply arrives — and only departing there is not.
//!
//! The strongest guarantee would come from the strongly connected components: weight
//! only edges that can reach, and be reached from, the largest set of edges that can
//! all reach one another, and every drawn pair would have a route. But a strongly
//! connected set needs loops, and without turnarounds a generated network without a
//! ring road has none: every component is a single edge, and the guarantee is bought
//! by weighting almost nothing. So the components used here are the *weakly*
//! connected ones — edges joined by a connection in either direction — and only the
//! largest of them, by total length, is weighted. That removes the pairs that can
//! never be joined however a router tries, such as the two carriageways of a lone road
//! whose ends link nowhere, which are two separate one-way strips once U-turns are
//! gone. It does not remove every unroutable pair: within one part of the network a
//! car still cannot turn back onto the arm it came from. Run `randomTrips.py` with
//! `--validate`, which has `duarouter` drop the trips that have no route; the weights
//! make those the exception rather than half of the demand.
//!
//! A network with no connection at all — a single road with nothing at either end —
//! has no edge that can be left, so every weight is 0 and `randomTrips.py` says it
//! found no valid edges. That is the truth about such a network.
//!
//! # Connections the graph knows about
//!
//! The written connections, and one kind netconvert adds itself: where a road changes
//! cross-section, its two edges meet at a node the IR states no movement across, and
//! netconvert's lane matching continues the one into the other (see
//! [`crate::check`]). Anything else netconvert might guess — at a junction approach the
//! IR gave no movement, say — is left out, so the graph can only be short of the built
//! network's connections, never ahead of it: an edge weighted here has its way in or
//! out in the network SUMO runs.

use std::collections::{BTreeSet, HashMap};

use roadgen_core::RoadId;

use crate::xml;
use crate::Exporter;

/// The vehicle class the weights are for: `randomTrips.py`'s default.
const VEHICLE_CLASS: &str = "passenger";

/// The shortest length a weighted edge is given, so that a very short edge is still
/// drawn now and again rather than written as the 0 that means "never".
const MINIMUM_WEIGHT: f64 = 0.001;

/// The three `edgedata` files `randomTrips.py --weights-prefix` reads, rendered.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct TripWeights {
    /// Where a trip may depart: `<name>.safe.src.xml`.
    pub src: String,
    /// Where a trip may arrive: `<name>.safe.dst.xml`.
    pub dst: String,
    /// Where a trip may pass through as an intermediate point: `<name>.safe.via.xml`.
    pub via: String,
}

impl TripWeights {
    /// The file name suffixes, after the network's prefix, in the order `src`, `dst`,
    /// `via`.
    pub const SUFFIXES: [&'static str; 3] = ["safe.src.xml", "safe.dst.xml", "safe.via.xml"];
}

/// What each edge may be used for, by edge index.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
struct Use {
    depart: bool,
    arrive: bool,
    length: f64,
}

impl Exporter<'_> {
    pub(crate) fn render_weights(&self) -> TripWeights {
        let uses = self.trip_uses();
        let render = |interval: &str, weighted: fn(&Use) -> bool| {
            let mut document =
                xml::Document::new("edgedata", "http://sumo.dlr.de/xsd/meandata_file.xsd");
            document.open(
                "interval",
                &[
                    ("id", interval.to_owned()),
                    ("begin", "0".to_owned()),
                    ("end", "1".to_owned()),
                ],
            );
            for (edge, used) in self.edges.iter().zip(&uses) {
                let value = if weighted(used) {
                    used.length.max(MINIMUM_WEIGHT)
                } else {
                    0.0
                };
                document.leaf(
                    "edge",
                    &[("id", edge.id.clone()), ("value", crate::metres(value))],
                );
            }
            document.close("interval");
            document.finish()
        };
        TripWeights {
            src: render("src", |used| used.depart),
            dst: render("dst", |used| used.arrive),
            via: render("via", |used| used.depart && used.arrive),
        }
    }

    /// Which edges a passenger-car trip may depart from and arrive on. See the module
    /// documentation for the rule.
    fn trip_uses(&self) -> Vec<Use> {
        let count = self.edges.len();
        let admits = |edge: usize, lane: usize| {
            self.edges[edge].lanes[lane]
                .permission
                .is_none_or(|permission| permission.admits(VEHICLE_CLASS))
        };
        let drivable: Vec<bool> = (0..count)
            .map(|edge| (0..self.edges[edge].lanes.len()).any(|lane| admits(edge, lane)))
            .collect();

        // The links of the graph: the written connections between two lanes a car may
        // use, and the continuations netconvert draws across a change of
        // cross-section.
        let mut links: BTreeSet<(usize, usize)> = self
            .movements
            .keys()
            .filter(|(from, from_lane, to, to_lane)| {
                admits(*from, *from_lane) && admits(*to, *to_lane)
            })
            .map(|(from, _, to, _)| (*from, *to))
            .collect();
        let mut by_road: HashMap<&RoadId, Vec<usize>> = HashMap::new();
        for (index, edge) in self.edges.iter().enumerate() {
            by_road.entry(&edge.road).or_default().push(index);
        }
        for edges in by_road.values() {
            for &from in edges {
                for &to in edges {
                    let (a, b) = (&self.edges[from], &self.edges[to]);
                    if a.direction == b.direction
                        && a.to == b.from
                        && a.section.abs_diff(b.section) == 1
                        && drivable[from]
                        && drivable[to]
                    {
                        links.insert((from, to));
                    }
                }
            }
        }

        let mut has_exit = vec![false; count];
        let mut has_entry = vec![false; count];
        let mut parts = Partition::new(count);
        for &(from, to) in &links {
            has_exit[from] = true;
            has_entry[to] = true;
            parts.join(from, to);
        }

        let length = |edge: usize| {
            self.edges[edge]
                .lanes
                .iter()
                .map(|lane| lane.shape.length())
                .fold(0.0, f64::max)
        };

        // The largest weakly connected part, by length, among those with a link in
        // them; the lowest edge index settles a tie, so the choice is the same every
        // time.
        let mut totals = vec![0.0; count];
        for edge in 0..count {
            if has_exit[edge] || has_entry[edge] {
                totals[parts.root(edge)] += length(edge);
            }
        }
        let mut chosen: Option<usize> = None;
        for edge in 0..count {
            let root = parts.root(edge);
            if !(has_exit[edge] || has_entry[edge]) {
                continue;
            }
            if chosen.is_none_or(|best| totals[root] > totals[best]) {
                chosen = Some(root);
            }
        }

        (0..count)
            .map(|edge| {
                let inside = chosen == Some(parts.root(edge));
                Use {
                    depart: inside && has_exit[edge],
                    arrive: inside && has_entry[edge],
                    length: length(edge),
                }
            })
            .collect()
    }
}

/// Edges grouped into the weakly connected parts of the graph, by union–find.
struct Partition {
    parent: Vec<usize>,
}

impl Partition {
    fn new(count: usize) -> Self {
        Partition {
            parent: (0..count).collect(),
        }
    }

    fn root(&mut self, mut edge: usize) -> usize {
        while self.parent[edge] != edge {
            self.parent[edge] = self.parent[self.parent[edge]];
            edge = self.parent[edge];
        }
        edge
    }

    /// Puts two edges in one part. The part keeps the lower of the two roots, so a
    /// part's root is the lowest edge index in it.
    fn join(&mut self, a: usize, b: usize) {
        let (a, b) = (self.root(a), self.root(b));
        let (low, high) = if a <= b { (a, b) } else { (b, a) };
        self.parent[high] = low;
    }
}
