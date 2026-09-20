# Reading a map back into the IR

*An investigation, September 2026. Nothing here is implemented; this is what would be
involved, per format, and which ones are worth it.*

The README says it in the second paragraph: roadgen is a generator, not a converter,
and nothing parses an existing map. That is a design stance and not an accident, so
the first thing to say is that adding an importer changes it. The rest of this note
takes the question as asked — of the seven formats we write, which could be read back
into the Canonical Road IR — and answers it format by format.

## The short answer

| Format | Verdict | What the file holds | What a reader has to invent | Parser already in the tree |
| --- | --- | --- | --- | --- |
| **OpenDRIVE** `.xodr` | **◎ best candidate** | the IR's own model, almost element for element: reference line, sections, width polynomials, lane offset, superelevation, links, junctions, connections, signals, objects, buildings, geo-reference, handedness | nothing structural; only the approximations listed below | yes — `opendrive` crate (`OpenDrive::from_xml_str`), used by `roadgen-viewer` and `tests/integration/src/opendrive_eval.rs` |
| **Lanelet2** `.osm` | **○ feasible, lossy** | every lane's 3D boundaries and centreline exactly, lane subtype, speed limit, markings, all the regulatory elements, all the furniture, lat/lon + `local_x`/`local_y` | roads (which lanelets share a reference line), the reference line itself, junctions, which lanelets are connectors, road ids and names, road type beyond urban/non-urban | yes — `ll2_io::load_str` plus `ll2-routing` for successors and neighbours, both used by `crates/roadgen-lanelet2/tests/roundtrip.rs` |
| **SUMO** plain XML | **○ feasible, regenerated** | per-lane 3D shapes, one width per lane, speed, vehicle classes, lane-to-lane connections, node kinds, edge priorities | the pairing of the two one-way edges of a road, the junction connectors (netconvert's, never written), markings, tapers, superelevation, signal positions, geo-reference | yes for the three input files — `roadgen-viewer/src/sumo.rs` (quick-xml). No reader for a built `.net.xml` |
| **CARLA** package | **◎ = OpenDRIVE** | the package's `.xodr` is the OpenDRIVE export; `furniture.manifest` adds where each light and sign stands | the FBX is derived from the IR and carries meshes, not lanes — reading it is a reconstruction problem, not a parse | `.xodr` as above; the FBX reader in `roadgen-viewer/src/fbx.rs` recovers mesh names and outlines only |
| **OpenStreetMap** `.osm` | **△ a generator from OSM, not a round trip** | one centreline per road, lane counts, `oneway`, `maxspeed`, `highway`, `sidewalk`, `ele`, restriction relations; buildings almost completely (Simple 3D Buildings) | lane widths, markings, tapers, superelevation, sections, every junction's connectors, how far each arm is cut back from the shared node | yes — `ll2_io::osm::parse` gives nodes, ways and relations; the exporter writes through the same document model |
| **ClipGT** parquet | **△ geometry only** | every lane as two 3D rails, lane lines, road boundaries, crosswalks, wait lines, lights, signs, intersection hulls | topology (no successor, no shared vertex — endpoints have to be welded by coordinate), roads, junctions, ids, markings beyond line type, rules | yes — `roadgen-viewer/src/clipgt.rs` and `tests/integration/src/clipgt_read.rs` read the layers back with `parquet`/`arrow` |
| **GPUDrive** `.json` | **✕** | 2D centrelines, painted lines, road edges, crosswalks, stop signs — each an unlinked polyline | z, widths (a lane has none; they would have to be inferred from the distance to the nearest `road_line`), topology, roads, junctions, everything regulatory | yes — `roadgen-viewer/src/gpudrive.rs` deserialises the scene through `roadgen-gpudrive`'s own data model |

OpenDRIVE is the one that is worth doing: it is the format whose model the IR was
shaped against, its parser is already a dependency, and the tests already contain an
independent evaluator of its geometry. Lanelet2 is second, and is a different kind of
job — a *reconstruction* of roads and junctions from lanelets, rather than a lowering
in reverse. SUMO and OSM are feasible as **generators from a coarser description**,
which is a useful feature in its own right (a town from real OSM data) but is not a
round trip and should not be sold as one. ClipGT and GPUDrive are pictures of a map;
GPUDrive in particular holds less than OSM does.

## Two ways in

The IR is populated by exactly one thing today: `MapBuilder::finish`, which takes
`RoadSpec`s and `connect` operations and *generates* everything else — it rounds
polyline vertices and corners, mitres joints, lays the cross-section out from the
reference line, draws a Hermite cubic for every junction movement, and paves a
pavement round every junction corner (`builder.rs`, `Generator::run`). An importer
can go in through that door or around it, and the choice is the main design decision.

**A. Through the builder.** Read the file into `RoadSpec`s plus `connect` calls and
let the generator do what it does. The geometry that comes out is roadgen's, not the
file's: connectors are regenerated as cubics, kinks are rounded, and a road that
arrives at a joint on a curve is refused. This is right for a format that does not
carry generated geometry anyway — SUMO's plain XML (netconvert regenerates junctions
regardless), OSM (there is no lane geometry to keep) — and wrong for OpenDRIVE and
Lanelet2, where the file *is* the generated geometry and the point of reading it is to
keep it.

**B. Straight into `Map`.** Every field of `Map`, `Road` and `Lane` is `pub`, and
`UnvalidatedMap::from_map(map).validate()` is the same gate the builder's output goes
through. An importer that builds a `Map` directly keeps the file's geometry and has
to satisfy `validation.rs` on its own: boundaries spanning the lane's width across the
road frame within 2 cm at both ends, `left_edge == right_edge + 1`, reciprocal road
links, connection endpoints within 1 mm, junction membership consistent. Nothing in
`roadgen-core` helps with the one hard part — laying a cross-section out from a
reference line and width profiles — because that code is private to the builder
(`Generator::lay_out_section`, `RoadGeometry`). An OpenDRIVE importer needs exactly
that function, so the first step of building one is to lift it out into a public
`geometry`-level operation the builder and the importer share. That is a refactor of
`builder.rs`, not new maths.

Path B is the one for OpenDRIVE and Lanelet2; path A for SUMO and OSM.

## What the IR cannot hold, whichever way in

These bite a *foreign* file more than one roadgen wrote, because roadgen's exports
were lowered from the IR and so already fit it. They are worth listing because every
one of them is a place the importer has to choose an approximation and say so —
through a `check()`-style report, the way the exporters do.

1. **No zero width.** `PositiveWidth` cannot hold zero and `WidthProfile` is monotone
   between knots. An OpenDRIVE lane that tapers to nothing — the ordinary way to end
   a lane — has to become a lane that *stops* at a new `CrossSection` where its width
   falls below some floor. The IR supports the lane ending; what is lost is the last
   few metres of the wedge.
2. **A width is a taper, not a cubic.** `<width>` is `a + b·ds + c·ds² + d·ds³`. Linear
   (`c = d = 0`) is exact; the `3t² − 2t³` smooth step is exact; anything else is
   sampled into `Linear` knots. Roadgen's own files are always one of the first two.
3. **Constant grade per analytic piece.** `Line3`, `Arc3` and `Clothoid3` carry a
   start height and an end height and climb linearly between them. An
   `<elevationProfile>` is piecewise cubic on its own stations, unaligned with the
   plan-view pieces. Roadgen writes it piecewise *linear* at the sample stations, so
   its own arcs come back as a `Composite` of arcs split at those stations — same
   shape, more pieces. A foreign cubic elevation is approximated the same way, or the
   piece falls back to a `Polyline3`.
4. **A connector is one lane.** `roadgen-opendrive` reads `connector.lanes.first()`
   and the builder writes one connecting road per movement, with `lane_offset` half
   a lane wide. A foreign junction's connecting road with three lanes and two
   directions has to be split into one IR connector per lane, each with that lane's
   centreline as its reference line — which is a polyline, so the connecting road's
   analytic geometry is lost.
5. **Eight lane types, five road types.** OpenDRIVE has about twenty lane types;
   `stop`, `median`, `curb`, `entry`, `exit`, the ramps, `bus`, `tram` and `rail` all
   need a mapping and most of them land on `Restricted` or `None`. Lanelet2 holds only
   `road`, `bicycle_lane` and `walkway`, and only urban/non-urban for the road.
6. **Shoulders, borders, parking and `None` lanes are gone from Lanelet2 and SUMO.**
   Both exporters drop them (`tags::lanelet_subtype`, `classes::permission`); a map
   read back from either has narrower roads than the one that was written.
7. **Identifiers are not the file's.** OpenDRIVE and Lanelet2 ids are assigned in
   export order and never flow back. An importer has to synthesise `road/…` names; a
   round trip through the IR renumbers everything, so *diffing a re-imported map
   against its source is not meaningful* the way diffing two generated maps is.
8. **A polyline reference line gets its vertices rounded** by the builder
   (`round_vertices`). Path B avoids it; path A cannot, and a coarsely digitised OSM
   way can fail `CornerTooSharp` at a vertex the exporter never saw.

## OpenDRIVE, in detail

The lowering in `roadgen-opendrive/src/lib.rs` is a table with an inverse for every
row:

| OpenDRIVE | IR | Notes |
| --- | --- | --- |
| `<header>` name, `<geoReference>` | `MapMetadata.name`, `origin`, `projection` | roadgen writes its own PROJ string and, for CARLA, `origin_proj_string`; both carry `+lat_0`/`+lon_0`. A foreign string may be any CRS — accept `+proj=tmerc`/`utm` with an origin, refuse the rest |
| `<road rule="RHT/LHT">` | `TrafficHandedness` | `parse` exists |
| `<planView>` line / arc / spiral | `Line3` / `Arc3` / `Clothoid3` | exact; `Curve3::composite` chains them, and it checks the pieces meet |
| `paramPoly3` (normalized) | `Bezier3` | the exporter's formula inverted, in the start frame: `u1 = b/3`, `u2 = (c + 2b)/3`, `u3 = b + c + d`, the same in `v`; `arcLength` p-range needs a reparameterisation |
| `poly3` | `Polyline3` | sampled; the IR has no such piece |
| `<elevationProfile>` | heights on the pieces | see constraint 3 |
| `<lateralProfile><superelevation>` | `Road.superelevation` | rename; `<shape>` dropped |
| `<lanes><laneOffset>` | `Road.lane_offset` | rename |
| `<laneSection s=…>` | `CrossSection` | direct |
| `<lane id type level>` | `Lane` side, ordinal, type | sign of id is the side; magnitude the ordinal; `Direction` from side and handedness (or `direction=` in 1.8) |
| `<width>` | `WidthProfile` | constraints 1–2 |
| `<roadMark type color>` | `BoundaryMarking` | `RoadMarking::parse`; one per boundary, so a lane with several along `s` keeps the first |
| `<speed>` | `Lane.speed_limit` / `Road.speed_limit` | unit conversion |
| `<link>` predecessor/successor + contactPoint | `RoadLink` | direct |
| lane `<link>` | `LaneConnection` outside a junction | direct; direction decides `from`/`to` |
| `<junction>` `<connection>` `<laneLink>` | `Junction`, connector `Road.junction`, `LaneConnection { junction }` | constraint 4 |
| `<junction><priority>` | `TrafficRule::RightOfWay` | direct |
| `<signal>` + `<validity>`, `<controller>` | `MapObject::TrafficLight/TrafficSign`, `TrafficRule::TrafficLight` | `(s, t, zOffset)` back to a point by evaluating the road frame — the inverse of `road_coordinates::locate`; a foreign signal's bar geometry has to be invented from its lane validity |
| `<object type="crosswalk">`, stop-line objects | `MapObject::Crosswalk/StopLine` | outlines and `(s, t)` back through the road frame |
| `<object type="building">` + `<outlines>` | `Building`, `BuildingPart` | corners are `cornerLocal`, one frame per building; the roof is already flattened at export |

Everything the IR has is in the file, and the evaluator in
`tests/integration/src/opendrive_eval.rs` is already most of the geometry half of a
reader. An importer would live in `roadgen-opendrive` beside the writer (the crate
already depends on the parser), take path B, and reuse the lifted lane layout so the
boundaries it computes are the ones the exporter would have computed — which is what
makes a round-trip test meaningful: *export → import → export* should reproduce the
file to within the approximations above, and the 22 scenario maps in
`tests/integration/src/scenarios.rs` are the corpus to run it over.

## Lanelet2, in detail

The file holds every lane precisely and no road at all. Reading it is therefore a
reconstruction in three steps, the first two of which `simple_lanelet2` does already:

1. **Load** (`ll2_io::load_str` with a projector). The origin is not in the file; it
   is where `local_x = local_y = 0`, and any node's lat/lon less its local offset gives
   it. Which projection was used is not tagged either — a heuristic (do `local_x`
   values look like a position within an MGRS square?) or a caller argument.
2. **Topology** (`ll2-routing`'s `RoutingGraph`): successors from shared endpoints,
   `lefts`/`rights` from shared linestrings. This is exact for a file roadgen wrote,
   because the exporter interns vertices and shares one linestring per cross-section
   edge; it is the Lanelet2 convention, so most Autoware maps do the same.
3. **Roads and junctions**, which is the new work. Laterally adjacent lanelets that run
   the same span form one cross-section; a chain of such cross-sections that share
   their lateral structure forms one road with several sections. The reference line is
   a chosen cross-section edge (the boundary between the two directions, when there
   is one), as a `Polyline3` through the shared vertices; widths are the perpendicular
   distances at each vertex, as `Linear` knots. Superelevation is *in* the vertex
   heights — `atan(Δz / width)` across the section — and can either be recovered into
   `Road.superelevation` or left as it is, since a polyline boundary with the right
   heights validates either way. Junctions have no marker at all: connectors are found
   by the shape of the graph — a lanelet whose predecessor has other successors that
   diverge from it, or whose polygon overlaps another lanelet's — and every such
   cluster becomes a `Junction`. That is a heuristic, and on a foreign map it will
   sometimes be wrong.

Regulatory elements come back cleanly: `traffic_light` (with `ref_line` → stop line),
`right_of_way`, `speed_limit` are `TrafficRule`s one for one, and the furniture
linestrings are `MapObject`s by `type`. What does not come back is in constraint 6
and 7 above, plus buildings, road names, and the analytic geometry (every curve is a
polyline). For an *Autoware map somebody else drew* that is the honest result: the
lanes and the rules, on polylines, with the roads and junctions inferred.

## SUMO, in detail

The three input files are close to a `RoadSpec` per edge: a shape (the edge's
centre — the road's reference line, which `roadgen-sumo` writes as the reference
line), a lane count, a width and a speed per lane, and vehicle classes that
`classes::permission` inverts to a `LaneType`. What makes it path A rather than B is
that a road with two directions is two edges pointing at each other, and pairing them
back into one road (same two nodes, reversed, shapes coincident) is the one
reconstruction step; connections then become `connect_lanes` calls and the builder
draws the connectors netconvert would have drawn. Traffic lights survive as a node
type with no position, right of way as edge priorities. Nothing is geo-referenced.

A **built** `.net.xml` is a different and larger job — internal lanes, junction
shapes, request matrices — and is the format an OSM-imported SUMO network arrives in.
It is not proposed.

## OpenStreetMap, in detail

Everything the exporter reports in `check()` as lost is what an importer would have
to make up: a way is a centreline with a lane *count*, so widths come from a table by
`highway` class, and there are no sections, no markings and no tapers. Two things
are specific to this format. The arms of a junction are *extended to the shared
node* on export, and the builder requires them cut back, so the importer has to
shorten every arm at a node of degree three or more by a distance it chooses — the
IR records nothing that would say how far. And the builder rounds polyline vertices
(constraint 8), which a real OSM way may not survive.

What it holds well is buildings: Simple 3D Buildings is the IR's own massing model,
and the exporter's own doc says so. A `building` outline with `building:part` ways
and `height`/`min_height`/`roof:*` comes back as `Building` and `BuildingPart`
nearly whole — which makes an OSM reader most useful as a *town* source rather than a
road source, and hints that the right shape for it is a generator beside
`roadgen-buildings` rather than an importer beside the exporters.

## ClipGT and GPUDrive

Both are read already, for the viewer, and neither reading is worth extending into an
import. ClipGT's `lane` layer does hold two 3D rails per lane, so lanes could be
rebuilt with exact geometry, but with no shared vertices every join has to be welded
by coordinate and every road and junction inferred — the Lanelet2 job with less to go
on and no rules at the end of it. GPUDrive is flat, has no widths, and links nothing
to anything; a reader would be guessing lane widths from the spacing of painted lines.

## If one is built

1. Lift the cross-section layout out of `Generator` into a public function in
   `roadgen-core` (reference line, superelevation, lane offset, sections, width
   profiles → lane boundaries and centrelines). The builder calls it; so does the
   importer.
2. `roadgen-opendrive::read` / `from_xml`, path B, returning `UnvalidatedMap` and a
   `Vec<String>` of what was approximated — the same shape as `check()`, from the
   other side.
3. A round-trip test over the scenario corpus: export, import, export, compare with
   the independent evaluator.
4. Lanelet2 second, in `roadgen-lanelet2`, with the road/junction reconstruction as
   its own module and its own tests on roadgen-written files before any foreign one.
5. Rewrite the README's second paragraph, because it will no longer be true.
