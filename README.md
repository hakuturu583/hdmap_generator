# roadgen

Generate 3D road networks — with a town beside them, if you want one — and write the
same network out as **OpenDRIVE**, as an **Autoware-ready Lanelet2** map, as plain
**OpenStreetMap**, as a **SUMO** network, as a **CARLA UE5 asset package**, as a
**ClipGT** clip for NVIDIA Cosmos, and as a **GPUDrive** scene.

This is a generator, not a converter. You describe roads, lanes, junctions and the
movements between them, and the library builds the geometry and writes the files.
The one file it reads is its own kind: an **OpenDRIVE** document can be
[read back](#reading-opendrive-back) into the same model, whether roadgen wrote it or
someone else did, and written out again in every other format.

**[Try it in a browser](https://hakuturu583.github.io/hdmap_generator/)** — the demo
page runs this package, compiled for WebAssembly. Write a script, press Run, and flip
between the formats it wrote. Nothing is uploaded and nothing is installed; see
[the demo page](#the-demo-page).

```python
import roadgen

m = roadgen.Map()
a = m.add_road(
    start=(0.0, 0.0, 10.0),
    end=(100.0, 0.0, 12.0),
    lanes=[
        roadgen.Lane(width=3.5, direction="forward"),
        roadgen.Lane(width=3.5, direction="backward"),
    ],
)
b = m.add_road(
    start=(100.0, 0.0, 12.0),
    end=(200.0, 50.0, 15.0),
    lanes=[
        roadgen.Lane(width=3.5, direction="forward"),
        roadgen.Lane(width=3.5, direction="backward"),
    ],
)
m.connect(a, b)
m.generate_buildings()          # a town along every frontage, from a shape grammar
m.export_opendrive("map.xodr")
m.export_lanelet2("map.osm")
m.export_osm("openstreetmap.osm")
m.export_sumo("sumo/")
m.export_carla("Import/")       # .fbx + .xodr + the descriptor CARLA imports
m.export_clipgt("clip/")
m.export_gpudrive("scene.json")
```

## How it is put together

```
                 Python API
                    PyO3
                     │
                     ▼
                 Rust core
                     │
       ┌─────────────┼─────────────┐
       │             │             │
   Topology       Geometry      Semantics
       │             │             │
       └─────────────┼─────────────┘
                     ▼
              Canonical Road IR ◀──── roadgen-buildings ◀── CGA shape grammar
                     │                                        `symbios-shape`
                 validate()
                     ▼
                ValidatedMap
                     │
                     ├──▶ OpenDRIVE      map.xodr            `opendrive`
                     ├──▶ Lanelet2       map.osm             `simple_lanelet2`
                     ├──▶ OpenStreetMap  plain .osm          `ll2-io`
                     ├──▶ SUMO           .nod/.edg/.con.xml  `quick-xml` + netconvert
                     ├──▶ CARLA          .fbx + .xodr        `roadgen-opendrive`
                     ├──▶ ClipGT         .parquet layers     `arrow`/`parquet`
                     └──▶ GPUDrive       scene .json         `serde_json`

              the written files ──▶ roadgen-viewer ──▶ SVG

                map.xodr ──▶ roadgen-opendrive::read ──▶ Canonical Road IR
```

Every exporter reads a `ValidatedMap` and writes nothing back into it: what a format
cannot hold comes back to the caller through that exporter's `check()`, never as a
field in the IR.

`roadgen-buildings` is the one arrow pointing *into* the IR, and it is a generator
rather than an exporter — it reads roads and writes [buildings](#buildings), the
same way the builder reads a `RoadSpec` and writes lanes. It runs after generation
and before validation, because a lot is measured against geometry that does not exist
until the first and buildings are part of what the second checks.

The two arrows at the bottom go the other way. `roadgen-viewer` reads the **written
files** — never the IR — so a picture it draws is a picture of what a consumer would
receive. It is what the [demo page](#the-demo-page) shows. And
`roadgen-opendrive::read` is the one route from a file *into* the IR: OpenDRIVE is
the format the IR was shaped against, so a document comes back as roads, lanes,
movements, furniture and rules rather than as a picture — see
[Reading OpenDRIVE back](#reading-opendrive-back). No other format is read, and
[`docs/import-feasibility.md`](docs/import-feasibility.md) says why.

Four separations are load-bearing, and each is a module of `roadgen-core`:

- **Topology** (`topology`) is connectivity and nothing else. A `LaneConnection` is
  two lane endpoints; it mentions no coordinate. Topology can be built before any
  geometry exists, and that is the order the builder works in.
- **Geometry** (`geometry`) is three-dimensional from the start. There is no 2D point
  type. Where a planar computation is genuinely needed — an OpenDRIVE `s`
  coordinate, a lane offset — it goes through `Frame3::to_local` or a method whose
  name says `horizontal`, so the projection is visible at the call site. Lines, arcs,
  clothoids, Béziers and polylines are all `Curve3`, and a road's cross-section can be
  banked by a superelevation profile.
- **Semantics** (`semantics`) is lane types, speed limits, markings, traffic lights,
  stop lines, crosswalks and right of way, as IR concepts. No OpenDRIVE `<signal>`
  and no Lanelet2 `RegulatoryElement` appears in the IR. A **building** is not one of
  these — it means nothing to traffic — so it lives in `buildings` with an arena and
  an identifier of its own.
- **Physical encoding** lives in the exporter crates, which depend on the core and
  are never depended upon by it.

### Design lineage

The separation of features from topology, of geometry from semantics, relationships
as first-class objects, stable identifiers, and the split between a conceptual model
and its physical encoding are ideas taken from **GDF 5.1**. Only the ideas: there is
no GDF feature catalogue, no feature or attribute codes, no GDF XML. **This is not a
GDF-conformant implementation and does not claim to be.**

### What the types guarantee

Constraints that can be made unrepresentable are, rather than left to validation:

| Type | What it rules out |
| --- | --- |
| `PositiveWidth` | a lane width of zero or less |
| `SpeedLimit` | a non-positive speed |
| `UnitVector3` | a direction derived from a zero-length vector |
| `Polyline3` | a polyline with fewer than two distinct vertices |
| `GeoOrigin` | a latitude or longitude off the globe |
| `RoadId` / `LaneId` / `JunctionId` / `ConnectionId` / `ObjectId` | passing one kind of identifier where another belongs — it will not compile |
| `ValidatedMap` | handing an unchecked map to an exporter |

A map moves through `MapBuilder → UnvalidatedMap → validate() → ValidatedMap`, and
the exporters take only the last of those.

### Identifiers

Identifiers are derived from what the caller asked for, never from a counter, so
regenerating a map and diffing the result is meaningful:

```
road/north
lane/north/0
lane/north/1
junction/j0
connection/j0/north_0/east_0
```

OpenDRIVE and Lanelet2 ids are assigned by the exporters, in the IR's own iteration
order, and never flow back into the IR.

## Installing

### Python

```bash
pip install maturin
maturin build --release          # or: maturin develop
pip install target/wheels/roadgen-*.whl
```

### Rust

```toml
[dependencies]
roadgen-core = { git = "https://github.com/hakuturu583/hdmap_generator" }
roadgen-opendrive = { git = "https://github.com/hakuturu583/hdmap_generator" }
roadgen-lanelet2 = { git = "https://github.com/hakuturu583/hdmap_generator" }
roadgen-clipgt = { git = "https://github.com/hakuturu583/hdmap_generator" }
roadgen-osm = { git = "https://github.com/hakuturu583/hdmap_generator" }
roadgen-buildings = { git = "https://github.com/hakuturu583/hdmap_generator" }
```

```rust
use roadgen_core::prelude::*;

let mut builder = MapBuilder::default();
let lanes = || vec![
    LaneSpec::new(PositiveWidth::new(3.5)?, Direction::Forward),
    LaneSpec::new(PositiveWidth::new(3.5)?, Direction::Backward),
];
let a = builder.add_road(
    RoadSpec::line(Point3::new(0.0, 0.0, 10.0), Point3::new(100.0, 0.0, 12.0), lanes())?
        .with_name("a"),
)?;
let b = builder.add_road(
    RoadSpec::line(Point3::new(100.0, 0.0, 12.0), Point3::new(200.0, 50.0, 15.0), lanes())?
        .with_name("b"),
)?;
builder.connect(&a, &b)?;

let mut map = builder.finish()?;
roadgen_buildings::generate(&mut map, &roadgen_buildings::Rules::default())?;

let map = map.validate()?;
roadgen_opendrive::write(&map, "map.xodr")?;
roadgen_lanelet2::write(&map, "map.osm")?;
roadgen_osm::write(&map, "openstreetmap.osm")?;

let clip = roadgen_clipgt::ClipConfig::new("clip")
    .with_scenario_file("scenario.yaml")?;
roadgen_clipgt::write(&map, "clips/", &clip)?;
```

## The Python API

`roadgen.Map` is a handle on a Rust `MapBuilder`. `Road`, `Junction` and `LaneRef`
carry identifiers, not state: there is one model of the map and it is in Rust.

| Call | What it does |
| --- | --- |
| `Map(name, origin, projection, handedness, sampling)` | a new network; `origin` is `(lat, lon, alt)` |
| `add_road(lanes, start=, end=)`, `add_road(lanes, points=)` or `add_road(lanes, alignment=)` | a straight road, one along a polyline, or one following an alignment |
| `Alignment(start, heading)` then `.line()`, `.arc()`, `.spiral()` | a reference line built piece by piece |
| `add_road(..., superelevation=[(station, radians), ...])` | banks the cross-section |
| `Lane(width_profile=[(station, metres), ...], taper=)` | a lane that narrows or widens |
| `add_road(..., cross_sections=[(station, [lanes]), ...])` | where the *number* of lanes changes |
| `add_junction(name)` | a junction to route movements through |
| `connect(a, b, junction=None, ends=("end", "start"))` | joins two roads by the named ends, pairing lanes |
| `connect_lanes(from_lane, to_lane, junction=None)` | one specific movement |
| `add_stop_line`, `add_traffic_light`, `add_traffic_sign`, `add_crosswalk` | road furniture; the first three take `setback=` metres back from the lane end |
| `generate_buildings(enabled=True, rules=None, seed=0)` | a town beside the roads, on or off, by the rules you give it |
| `building_ids()`, `building_kind(id)`, `building_frontage(id)`, `building_parts(id)` | what it generated |
| `building_footprint(part)`, `building_part_shape(part)`, `building_part_kind(part)`, `building_shell(part)` | one part's outline, its heights and roof, what it is, and the faces that bound it |
| `add_traffic_light_rule`, `add_right_of_way`, `add_speed_limit` | rules over lanes |
| `validate()` / `issues()` / `format_warnings()` | check before exporting |
| `clipgt_warnings(scenario=None)` | what a ClipGT export would lose, and what its scenario gets wrong |
| `gpudrive_warnings(scenario=None)` | what a GPUDrive export would lose, and what its scene runs up against |
| `carla_warnings(name=, package=, buildings=, ...)` | what a CARLA package cannot carry, and what its import will get wrong quietly |
| `osm_warnings()` | what a plain OpenStreetMap export would lose |
| `sumo_warnings()` | what a SUMO export would lose |
| `mgrs_grid()` | the grid square an MGRS map is reported in |
| `export_opendrive(path)` / `export_lanelet2(path)` / `export_osm(path)` | write the files |
| `export_sumo(directory)` | write a SUMO plain-XML network and its netconvert configuration; returns the prefix |
| `sumo_lane_ids()` | where each lane of the map landed in the SUMO network |
| `export_clipgt(directory, scenario=, clip_id=, frame_rate=, speed=, route=)` | write a ClipGT clip; returns the clip id |
| `export_gpudrive(path, scenario=, name=, scenario_id=, steps=, time_step=, speed=, route=)` | write a GPUDrive scene |
| `export_carla(directory, name=, package=, buildings=, use_carla_materials=, kerb_height=, verge_width=, ground_extent=, furniture=, carla_root=, engine=, sun_altitude=, sun_azimuth=)` | write a CARLA UE5 package and the script that imports it; returns what was written and what each mesh will be tagged |
| `fetch_textures(package, overwrite=, resolution=)` | download the Poly Haven textures a written package asks for |
| `carla_sky(carla_root, package, map_name, engine=, sun_altitude=, sun_azimuth=)` | after `Import.py`: give the imported level a daylight sky, with the editor's own scripting |
| `carla_furniture(carla_root, package, map_name, engine=, manifest=)` | after `Import.py`: stand the package's traffic lights, signs and props town in the level, and make CARLA adopt the lights and signs |
| `to_opendrive_xml()` / `to_lanelet2_osm()` / `to_osm_xml()` / `to_gpudrive_json()` | the same, as strings |
| `road_ids()`, `lane_ids()`, `connections()`, `successors(lane)`, `lane_centerline(lane)` | inspect the built map |
| `render_opendrive(path)` / `render_sumo(directory)` / `render_carla(path)` / `render_clipgt(directory)` / `render_gpudrive(path)` | read a written export back and draw it, as an SVG document |
| `read_opendrive(path, sampling=, origin=, projection=)` | read an OpenDRIVE file as a `Map` — one that exports like any other and cannot be added to |
| `read_warnings()` | what the file a map was read from said that the map could not keep |
| `building_presets()` / `building_rules(name=None)` | the built-in rule sets, and one of them as text to edit |

The `render_*` functions are module-level rather than methods on `Map`, because they
read the **file** and not the map. Drawing the IR would agree with the IR by
construction and so would say nothing about whether the export is right; drawing what
was written says quite a lot. They are what the
[demo page](#the-demo-page) puts on the screen, and in a notebook they are
`IPython.display.SVG(...)`.

Handedness decides which side of the reference line a `forward` lane lands on:
`"rht"` (the default) puts it on the right, `"lht"` on the left. A lane can override
it with `side=`.

### The order of the lane list

The `lanes=` list is **not** read left to right across the road. Each lane goes to
its side by its direction, and within a side the lanes are stacked **outwards from
the reference line in list order** — the first `forward` lane is the one against the
centre, the next is outside it, and so on; the same for the `backward` lanes on the
other side. So a two-way street with pavements is written carriageway first:

```python
lanes=[
    roadgen.Lane(width=3.5, direction="backward"),                    # against the centre
    roadgen.Lane(width=2.0, direction="backward", type_="sidewalk"),  # outside it
    roadgen.Lane(width=3.5, direction="forward"),                     # against the centre
    roadgen.Lane(width=2.0, direction="forward", type_="sidewalk"),   # outside it
]
```

Written left to right instead — pavement, carriageway, carriageway, pavement — the
backward pavement takes the rank against the centre and the road has a footway down
its middle. Nothing about a single road can tell that was not meant; the first
`connect()` can, because lanes are paired across a joint by side and rank, and two
such roads pair nothing. It raises rather than returning an empty list.

## Alignments

A road's reference line can be a straight, a polyline, or a chain of lines, bends and
transition curves. `Alignment` carries the state between pieces — where it has got to,
which way it is pointing, how hard it is turning — so each piece begins exactly where
the last one ended:

```python
al = roadgen.Alignment(start=(0.0, 0.0, 4.0), heading=0.0)
al.line(80.0, rise=1.0)
al.spiral(60.0, curvature_end=1 / 120, rise=1.0)   # straight into the bend
al.arc(140.0, curvature=1 / 120, rise=2.0)
al.spiral(60.0, curvature_end=0.0, rise=1.0)       # and back out of it
al.line(80.0, rise=1.0)

m.add_road(lanes=[...], alignment=al, name="sweep")
```

The transitions are **clothoids** — curvature ramping linearly, which is what a
vehicle traces while the steering wheel turns at a constant rate. Entering a bend
straight from a straight is a step change in lateral acceleration; a spiral is what
spreads it out. They reach OpenDRIVE as `<spiral>` elements, not as a chain of line
segments.

A road can also be **banked**, by a roll profile against station:

```python
m.add_road(
    lanes=[...],
    alignment=al,
    # Flat to 50 m, rolled to -0.06 rad by 100 m, held from there.
    superelevation=[(0.0, 0.0), (50.0, 0.0), (100.0, -0.06), (170.0, -0.06)],
)
```

Banking rotates the road local frame about its tangent, so a lane offset gains height
and keeps its full width across the tilted surface. OpenDRIVE gets the same piecewise
polynomial in `<lateralProfile><superelevation>`; Lanelet2 has no such concept and
does not need one, because the roll is already in the heights of its vertices.

### Corners

Two straight roads that meet at an angle — the map at the top of this file — are a
kink, and a kink is something only one of the formats can hold. Lanelet2 is a list of
points and can be cut on the slant; OpenDRIVE derives every lane from the reference
line and a width measured *perpendicular* to it, so however it is written the two
cross-sections end on different lines: a wedge of nothing on the outside of the turn,
an overlap on the inside, in a file every consumer takes at face value. CARLA drives
its traffic on that file.

So the corner is not written. When `connect()` joins two roads whose headings differ,
each is cut back a little and a circular arc, tangent to both, takes the corner's
place — half on each road, so no road is added and every link stays where it was.
The arc's radius is 6 m plus however far the cross-section reaches on the inside of
the turn, which keeps the inside kerb a curve a vehicle can follow whatever the road's
width; on a road too short for that it shrinks to fit, down to a corner as tight as the
road is wide, and past that the build fails and says so. A road that *arrives* at the
joint on a curve is not bent either: give it an alignment that is tangent-continuous
with the road it meets, or join the two through a junction.

The result is the same in every format, because it is in the IR: `roadgen-viewer`'s
OpenDRIVE panel draws a line, an arc and a line, Lanelet2 gets the arc's vertices, and
the CARLA surface goes round the corner too.

A road given as a polyline — `add_road(points=[...])` — bends at each of its vertices,
and a bend inside one road is the same kink as a joint between two. So each interior
vertex is rounded the same way, with the same radius rule, the chords either side cut
back to meet the arc; a vertex the road has no room to round shrinks its arc to fit,
and past a corner as tight as the road is wide the build fails and names the vertex.
What comes out is a chain of lines and arcs, and every format gets that chain.

## A cross-section that changes

Two different things, with two different answers:

**A lane that narrows or widens** stays one lane. Give it a width profile and the
boundaries follow:

```python
shoulder = roadgen.Lane(
    width=2.0, type_="shoulder",
    width_profile=[(0.0, 2.0), (60.0, 2.0), (100.0, 5.0), (160.0, 5.0), (200.0, 2.0)],
    taper="smooth",       # or "linear"
)
```

A `WidthProfile` **cannot describe a width that reaches zero**: every knot is a
`PositiveWidth`, and both tapers are monotone between knots, so the value between two
positive knots stays between them. That is why there is no free cubic here — an
OpenDRIVE `<width>` is a cubic in general, and a general cubic can dip below zero
between its ends. The smooth taper is exactly the cubic that cannot, and it lowers to
`<width>` with no loss.

**A lane that ends** changes how many lanes the road has, so a new cross-section
begins there:

```python
m.add_road(
    lanes=[left, middle, outer],      # three lanes to begin with
    cross_sections=[(200.0, [left, middle])],   # two from 200 m on
    start=..., end=...,
)
```

Each cross-section becomes its own `<laneSection>` in OpenDRIVE and its own set of
lanelets in Lanelet2 — which is what both formats need anyway, since a lanelet has
exactly two boundaries and cannot gain a neighbour halfway along. The lanes that carry
on across the boundary are connected automatically; the one that stops is not, so a
routing graph will not drive off the end of it.

The same machinery lets a junction connector **taper between lanes of different
widths**, instead of picking one and missing the other.

## Junctions

A junction is not a bare list of permitted turns: each movement gets a generated
**connector road**, drawn as a cubic curve whose end tangents are pinned to the lanes
it joins, so traffic leaves and arrives without a kink.

```python
junction = m.add_junction("fork")
m.connect(trunk, straight_on, junction=junction)
m.connect(trunk, slip, junction=junction)
```

That one model lowers cleanly both ways: the connectors become OpenDRIVE connecting
roads inside a `<junction>`, and lanelets that a Lanelet2 routing graph walks through.
The connecting road's reference line is the cubic itself, a `<paramPoly3>`, not a
chain of chords through its vertices: a chain would put a heading step at every
vertex and end some degrees off the tangent it shares with the arm, and OpenDRIVE lays
lane edges perpendicular to the heading, so the connector's cross-section at the
mouth would be turned against the arm's. The vertices Lanelet2 gets are on the cubic,
at its own arc length, so the two files put every lane edge in the same place.

`connect` joins the end of the first road to the start of the second, which is the
shape of a road carrying on into the next one. A crossroads is the other shape: its
approaches all point *at* the centre, so they meet the joint end to end, and `ends`
says so.

```python
junction = m.add_junction("x")
for a, b in itertools.combinations(arms, 2):
    m.connect(a, b, junction=junction, ends=("end", "end"))
```

One call per *pair* of arms, not per movement: joining two two-way roads already
generates the connectors for both directions.

Get this wrong and nothing complains — the map validates and both files are written —
but the connectors run to the far tips of the approaches rather than across the
junction, so it is worth reading the connector lengths once.

### Pavements go round the corner, not across

Only a lane that carries a movement — `driving` or `biking` — is connected *through*
a junction; a shoulder, a parking lane or a border stops at the arm, because nothing
moves along it to be carried across. A `sidewalk` lane is never connected through
either: a pedestrian does not walk across a crossroads the way a car drives across
it. Instead, at every junction, each
pair of arms that are neighbours round the junction gets a connector between their
outer sidewalks — a **pavement round the corner**, a single sidewalk lane drawn as
the same tangent-pinned curve a traffic connector is. A crossroads with pavements on
all four arms comes out with eight traffic connectors and four corner pavements, and
the only ways across the carriageway are the crosswalks the caller put down. The
corner pavements are ordinary roads in the junction — connecting roads in OpenDRIVE,
lanelets in Lanelet2, a surface in CARLA — so a pedestrian network is walkable in
every format without any of them having to know it was made differently.

## Traffic control

Lights, signs, stop lines and crosswalks are IR objects — a position and the lanes they
govern — and both formats get them:

| IR | OpenDRIVE | Lanelet2 |
| --- | --- | --- |
| Traffic light | `<signal dynamic="true">` with `<validity>` | `traffic_light` way + `traffic_light` regulatory element |
| Traffic sign | `<signal>` carrying the caller's catalogue code | `traffic_sign` way, code as its subtype |
| Stop line | `<object type="roadMark" name="stopLine">` | `stop_line` way, the rule's `ref_line` |
| Crosswalk | `<object type="crosswalk">` with its outline as `<cornerLocal>` corners | a lanelet of subtype `crosswalk` |
| Right of way | `<junction><priority high low>` | `right_of_way` regulatory element |

A light, a sign or a stop line goes at one end of a lane, or `setback` metres back
along the lane from it — `m.add_stop_line(lane, setback=8.0)` — which is where a
stop line belongs when a crosswalk lies between it and the junction, and where a
stop sign stands beside it.

OpenDRIVE places an object at `(s, t, zOffset)` in one road's own coordinates, so the
exporter projects the IR's position through the road local frame — a nearest-point
search for the station, then `Frame3::to_local`. Height is measured away from the road
surface rather than straight up, which is what "five metres above the road" means on a
slope and what `zOffset` carries.

A crosswalk's outline is written as `<cornerLocal>` corners in the object's own frame
— the object lying across the road, its `hdg` a quarter turn from the road's, the
first corner repeated to close the ring — rather than as `<cornerRoad>` corners in
road coordinates, which the standard also allows. CARLA reads only the first kind
into its pedestrian navigation; a crossing it cannot read is one nobody crosses at.

A signal's `type` is a code from a *country's* catalogue rather than a name of its own,
so a traffic sign passes the caller's code straight through (the CARLA package is the
one exception, and says which codes it recognises), and a traffic light is
written as the German catalogue's three-colour light (`1000001`) — the value OpenDRIVE
tooling expects in a generated map, and one a caller with another catalogue can rewrite
after export. Every light is also in a `<controller>`: the lights of one traffic-light
rule are one controller, a light no rule names shares one with the other unnamed
lights on its approach, and each junction names the controllers that lead into it.
That is OpenDRIVE's only way of saying which lights switch together, and what a
consumer running the junction looks for.

## Buildings

Off by default. Switched on, every road grows two frontages of lots and a **CGA shape
grammar** — the CityEngine formalism, derived by
[`symbios-shape`](https://crates.io/crates/symbios-shape) — decides what stands on
each one.

```python
m.generate_buildings()                        # on, a mixed town street
m.generate_buildings(rules="downtown")        # on, one of the built-in sets
m.generate_buildings(rules=text, seed=7)      # on, rules of your own
m.generate_buildings(False)                   # off
```

That is the whole of the Python surface: whether to, and by what rules. Everything
else — how wide a lot is, how far back from the kerb, how many storeys — is *in* the
rules, because the alternative is a dozen keyword arguments that mean nothing without
the grammar they go with.

### The division of labour

Two questions have to be answered to put a building somewhere, and they are different
questions.

**Where may a building stand?** That one is about the road network, so roadgen answers
it. The road surface is taken from the generated cross-section — the real one, lane by
lane, sampled along the reference line, so a road that tapers or gains a lane is
measured where it actually is — widened by the setback, and cut into lots along each
frontage. Nothing is placed in the middle of a block: the middle of a block is gardens,
yards and the backs of the buildings on the next street, and inventing masses there
would be inventing a land-use map rather than deriving one.

**What stands there?** That one is architecture, so the grammar answers it. Each lot
arrives as a rectangle on the ground — local **X** along the frontage, **Y** up, **Z**
away from the road, so the face `Comp(Faces)` calls `Front` is the one looking at the
street — and what comes back is a set of oriented boxes.

Then roadgen has the last word: a lot whose buildings would reach into a road or onto
a neighbour is dropped whole, so a gap in a street is a gap and never half a house.

### The rules

One piece of text. Derivation starts at the rule called `Lot`, and the word a mass is
emitted with becomes the building's kind:

```text
attr Setback = 5             // kerb to lot, metres
attr LotWidth = 15           // frontage per lot
attr LotDepth = 18           // how far back a lot reaches
attr LotGap = 4              // left clear between neighbours
attr CornerClearance = 12    // left clear at each end of a frontage
attr FloorHeight = 3.2       // what a storey is, for the storey count

Lot     --> Size(scope.x - 2, 0, scope.z - 2) Center(XZ) Plot
Plot    --> 45% House | 30% Terrace | 15% Shop | else: Block

// A pitched roof is the top of the mass, split off and given a shape.
House   --> Extrude(rand(6, 9)) Split(Y) { ~1: Walls | 2.5: Gable }
Walls   --> I("house")
Gable   --> Roof(Gable, height=scope.y) { Slope: Tiles | GableEnd: Wall }
Terrace --> Split(X) { ~1: Unit | ~1: Unit }
Unit    --> Extrude(rand(6.5, 8.5)) Split(Y) { ~1: Party | 2: Gable }
Party   --> I("terrace")
Shop    --> Extrude(FloorHeight * 2) I("retail")
Block   --> Extrude(FloorHeight * rand(3, 5)) I("apartments")
```

`//` is CGA's line comment, and `#` is accepted beside it because rules are usually
written inside a Python string and `#` is what the hand reaches for there.

Those six `attr` declarations are the numbers roadgen needs, read back out of the
grammar rather than passed beside it — so there is one thing to edit, and a rule can
read `LotDepth` and set a footprint against it. Undeclared ones keep their defaults,
and every one of them is readable from the grammar either way: a knob that only
worked once you had declared it would be a knob with a trap in it.

Everything else is ordinary CGA: `Extrude`, `Split`, `Repeat`, `Comp(Faces)`,
`ShapeL`, `Roof`, weighted and guarded variants, `rand()` and `scope.x` in any
numeric position. [`symbios-shape`'s
README](https://github.com/TheJanusStream/symbios-shape) is the reference.

One idiom is worth knowing, because a lot arrives flat: `Size(scope.x - 2, 0,
scope.z - 2) Center(XZ)` is how you leave a margin round the edge of a plot.
`Offset(-2)` is CGA's inset, and it wants a scope with height — on a footprint,
whose `Y` extent is zero until `Extrude` runs, it refuses.

`building_presets()` lists the four built-in sets — `town`, `suburban`, `downtown`,
`industrial` — and `building_rules(name)` hands one back as text, so rules of your own
start as a preset with a line changed rather than as a blank page.

### The model

A building is not a shape. It is a **complex feature**: a set of **parts**, what they
are for, and what they stand beside. That is the same relationship a road has to its
lanes, and it is kept the same way — the building lists its parts, each part names its
building, and both live in arenas keyed by identifier.

```
Building        parts, kind, frontage
  └── Part      solid, kind, levels
        └── Solid    footprint, wall_height, roof
              └── Roof   shape, height, direction
```

Each part is a **solid**, not an outline with a number bolted on:

- its **footprint** is a ring whose vertices carry their own heights, so a part that
  starts partway up a building says so with nothing extra, and one on sloping ground
  is on sloping ground;
- its **walls** rise from that ring;
- its **roof** is flat, skillion, gabled, hipped or pyramidal, with a rise and — where
  the shape has a ridge — the direction it runs in.

A part carries a `kind` of its own where it has one: a tower of offices standing on a
shopping podium is one building, and neither word describes both halves of it.

`Solid::shell()` hands back the faces that bound it: the base, one quad per wall, and
the roof. Every edge of that surface is used by exactly two faces, which is what makes
it a solid rather than a pile of panels, and `Solid::is_closed()` is that property
written down where it can be checked. The compact form is what the IR stores because it
is what every format asks for; the shell is what it means. In Python,
`building_shell(part)` returns it.

**Frontage** is the third piece, and it is a *relationship* rather than a coordinate:
which road the building faces, which side of it, and how far along. It says nothing
about where the building is — the geometry does that, completely — and everything about
what it belongs to. The OpenDRIVE export reads it instead of searching for the nearest
road, and a caller who wants the buildings of one street has a way to ask.

### What counts as one building

A derivation is a flat list of boxes and panels, and most grammars produce more of them
than there are buildings. Three rules turn one into the other, and each is about what a
box *is* rather than what it was called:

- a box with extent on all three axes is a **mass**: one on the lot's ground starts a
  building, one standing on another mass is a further **part** of that building. So a
  stack of floors, a tower on a podium and a wing behind a house come out as one
  building of several parts — and two masses side by side on the ground come out as
  two buildings, which is what a terrace is;
- a box with no thickness is a **panel**. Panels sitting on a part's eaves are its
  **roof**;
- anything standing over nothing is dropped, because there is no part to give it.

A facade's windows fall out of this on their own: `Comp(Faces)` produces scopes with no
thickness, and a window stands against a wall rather than on the eaves, so it is
neither a part nor a roof.

### Reading a roof

The grammar knows fifteen roof types; the IR has words for five, because those are the
five a consumer can be told about — they are OSM's `roof:shape` values. What is read
back is **geometry**, not the grammar's vocabulary: the ridge is where the roof is
highest, and the shape follows from where that ridge lies over the part it covers.

| The highest points of the roof | The shape |
| --- | --- |
| one point | `pyramidal` |
| a line down the middle, spanning the part | `gabled` |
| a line down the middle, stopping short of the ends | `hipped` |
| a line along one side | `skillion` |

So `Roof(Pyramid, …)` over an oblong reads back as **hipped**, because what the engine
draws there has a ridge and what has a ridge is a hip. That is the point of reading
geometry: the word follows the shape rather than the other way round.

A roof that is none of the five — an M-shape has two ridges, a mansard a flat top —
is not called the nearest one. It becomes the flat top of the volume that contains it:
the massing stays right, and only the word for the shape is missing.

### A town is a pure function

The rules, the map and the seed decide the town, every time. Each lot is seeded from
its **own identity** rather than from a running counter, so adding a street changes
that street's buildings and leaves every other one exactly as it was — the same
property the [identifiers](#identifiers) have, for the same reason:

```
building/high_street/left/3/0
part/high_street/left/3/0/0        ← the part that meets the ground
part/high_street/left/3/0/1        ← the one standing on it
```

### Where they end up

| Format | What it does with a building |
| --- | --- |
| **OpenStreetMap** | [Simple 3D Buildings](https://wiki.openstreetmap.org/wiki/Simple_3D_Buildings): the outline as a closed `building=<kind>` way, a `building:part=yes` way per part where there is more than one, with `height`, `min_height`, `building:levels`, `roof:shape`, `roof:height` and `roof:direction`. The solid survives; the frontage, a sloping base and the union of several parts' outlines do not, and `check` says so. |
| **OpenDRIVE** | one `<object type="building">` carrying an `<outline>` per part, corners as `<cornerLocal>`. The massing survives; the roof shape does not, and `format_warnings()` says how many were flattened. |
| **CARLA** | actual walls and roofs, from `Solid::shell()`. The whole solid survives — it is the only export that draws the roof rather than describing it — but CARLA's import will tag it `Terrain` unless the town is exported as [props](#where-the-town-goes-and-why-it-is-a-choice), in which case it is tagged and the package's script places it. |
| Lanelet2, SUMO, ClipGT, GPUDrive | nothing; each one's warnings say how many were dropped. |

OSM measures a roof's direction clockwise from north, as a bearing; the IR measures it
anticlockwise from east, as an angle. That conversion is the whole of the difference
between them, and it happens in the exporter where it belongs.

The OpenDRIVE outline is `cornerLocal` rather than `cornerRoad` on purpose. Both would
place the corners; only the first keeps them rigid. A `cornerRoad` corner is its own
`(s, t)` pair, so beside a bend a straight wall is written as a curved one and a
consumer evaluating the file back gets a banana. `cornerLocal` measures every corner in
one frame — the road's, at the building's own station — so the shape that comes back is
the shape that went in. The [viewer](#which-viewer-draws-what) draws every part from
those outlines, which is why a town shows up in the OpenDRIVE picture and in no other.

## Coordinates

A map's own x and y are always metres about its origin — that is what makes a generated
map easy to write. `projection` says what frame those metres are read against, and
therefore what a consumer reconstructs:

| `projection` | Lanelet2 `local_x`/`local_y` |
| --- | --- |
| `"local_cartesian"` (default) | the map's own metres |
| `"utm"` | the same, with latitudes and longitudes taken through UTM |
| `"mgrs"` | metres within the 100 km MGRS square the origin falls in |

MGRS is the coordinate system an Autoware map built with the MGRS projector uses.
Each node's grid position is worked out **from that node's own latitude and
longitude**, not by shifting the whole map by the origin's grid position: a metre of
local east is not a metre of UTM easting, and over a few kilometres the difference
shows. `Map.mgrs_grid()` gives the square's reference for `map_projector_info`.

Heights are the map's own under every projection: a Lanelet2 node's `ele` is its `z`,
the same number OpenDRIVE, SUMO, ClipGT, CARLA and the plain OSM export carry — and the
number Autoware's MGRS, UTM and local projectors use as `z` when they load the map. It
is not the ellipsoidal height of the point (which a local Cartesian frame puts d²/2R
above the tangent plane, already 3 m at 6 km from the origin); a consumer that wants
that has the latitude, the longitude and the origin's altitude to compute it from.

A map with MGRS coordinates has to fit inside one square. `ll2`'s MGRS projector takes
the easting and northing modulo 100 km, so a map running over the edge would silently
come back on the other side; `format_warnings()` reports it instead, and exporting
fails rather than writing it.

The OpenDRIVE file's coordinates are the map's own metres under every projection, and
its `<geoReference>` describes *those* metres: a transverse Mercator about the origin
for `local_cartesian` and `mgrs`, and for `utm` the zone's transverse Mercator with the
origin's easting and northing folded into the false origin, so that a PROJ consumer
reading the file lands on the latitudes the Lanelet2 export wrote. The CARLA package's
`.xodr` is the one exception: CARLA's GNSS sensor reads only `+lat_0`/`+lon_0` from
the string, as where `(0, 0)` stands, so that file names the origin there whatever the
projection.

## OpenStreetMap

`export_lanelet2` and `export_osm` both write `.osm`, and they are **not
interchangeable**. Lanelet2 uses the OSM container for something else: its ways are
lane boundaries, its relations are lanelets, and nothing in it carries a `highway`
tag, so a router or a renderer sees no roads at all. `export_osm` writes OSM as OSM
means it.

```python
m.export_osm("map.osm")
```

OSM describes a road as **one way down the centreline** with the lanes as a *count*:

```xml
<way id="-4">
  <nd ref="-2" /><nd ref="-3" /><nd ref="-1" />
  <tag k="highway" v="residential" />
  <tag k="lanes" v="2" />
  <tag k="lanes:forward" v="1" />
  <tag k="lanes:backward" v="1" />
  <tag k="name" v="north" />
  <tag k="oneway" v="no" />
</way>
```

`highway` comes from the road type, `maxspeed` from the speed limit in km/h, and
`sidewalk` from where the footways are in the cross-section. `oneway` is `yes`, `no`
or **`-1`** — the last for a road whose every lane runs against the reference line,
which is why the way's node order follows that line rather than being reversed for
convenience.

[Buildings](#buildings) go out as **Simple 3D Buildings**: the outline as a closed
`building=<kind>` way, a `building:part=yes` way per part where there is more than one,
and `height`, `min_height`, `building:levels`, `roof:shape`, `roof:height` and
`roof:direction` saying what each occupies. This is the one part of the export that
loses nothing, because OSM has a model for a building of several parts and the IR has
the same one. Terraced buildings share their corner nodes, which is welding doing for a
terrace what it does for two roads meeting at a junction.

Furniture goes on the way's own nodes, which is where OSM puts it:
`highway=traffic_signals`, `highway=stop`, `highway=crossing` and `traffic_sign=<the
caller's code>`. A node is inserted at the object's own position rather than snapped
to the nearest vertex — a straight road has two of those, and everything on it would
otherwise pile up on one end. Where several controls share a point, the stronger
keeps the node's single `highway` tag: a signalised stop is signals, not a stop sign —
and `osm_warnings()` says which pair was folded together. A crossing is a place rather
than a control, so it always has a node of its own, however close the stop line
before it stands, and it also gets a `highway=footway` + `footway=crossing` way across
the road, through that node, so the footway and the road share a vertex.

### Junctions, which is where the two models disagree

OSM has no connector roads. Arms meet at **one shared node**, and every turn is legal
unless a `type=restriction` relation says otherwise. The IR is the other way round: it
enumerates the movements that *are* permitted and draws a connector for each.

So a junction becomes a single node where the arms' centrelines would actually cross,
every arm's way is extended to it, and a pair of arms with **no** movement between
them becomes a `no_left_turn` / `no_right_turn` / `no_straight_on` relation. The
connectors are not written: twelve ways across a crossroads would be twelve roads
that do not exist.

Extending the arms is the one place this export moves geometry, and it is what makes
the result routable — four ways stopping 14 m short of each other are four dead ends.
The node is the least-squares intersection of the arm centrelines rather than the
average of their ends, so two arms meeting at a corner land where the roads cross and
not out in the middle of the bend.

U-turns are deliberately never written. The IR enumerates movements between
*different* arms, so it never says anything about turning back the way you came, and
a `no_u_turn` on every arm would be inventing a rule the map does not hold.

### What it cannot carry

- **Height.** An OSM node is a latitude and a longitude; elevation is the `ele` *tag*
  convention rather than part of the geometry. The heights survive as those tags and
  nothing else.
- **Lane geometry.** A lane is a number. Widths, boundaries, markings, tapers and
  cross-section changes have nowhere to go — a road whose lane count changes is
  written with its first section's, and `osm_warnings()` names it.
- **Superelevation**, and any per-lane rule.

Identifiers are negative and the file carries no `version`, which is how OSM data
says it was never uploaded. That is what a generated map is.

## SUMO

SUMO's simulator reads a `.net.xml`, and a `.net.xml` is a **build product**: it
carries the shape of every junction, the internal lane through every movement, and
the right-of-way matrix that decides who waits for whom — all computed by
`netconvert`. Writing one directly would mean reimplementing netconvert, and getting
it subtly wrong.

So what `export_sumo` writes is netconvert's own input, the **plain XML** network,
which is the format a generator is meant to produce:

```python
prefix = m.export_sumo("network/")     # returns the name the files were given
```

| file | what is in it |
| --- | --- |
| `<name>.nod.xml` | junctions and road ends, as points |
| `<name>.edg.xml` | one edge per direction of travel, with its lanes |
| `<name>.con.xml` | which lane may be left for which lane |
| `<name>.netccfg` | the netconvert run that turns the three into a `.net.xml` |

```bash
netconvert -c network/demo_town.netccfg
```

### One road, two edges

A SUMO edge is a one-way bundle of lanes, so a road carrying traffic both ways is two
of them pointing at each other:

```xml
<edge id="north.fwd" from="n_north_start" to="j_x" priority="4" numLanes="1"
      speed="13.890" spreadType="center" name="north"
      shape="-1.750,70.000,0.000 -1.750,14.000,0.000">
  <lane index="0" width="3.500" disallow="pedestrian"
        shape="-1.750,70.000,0.000 -1.750,14.000,0.000"/>
</edge>
```

Every lane is written with **its own shape**, so what SUMO gets is the geometry the
generator computed — not a centreline with a width laid out from it, which is what a
network imported from OpenStreetMap has to make do with. The heights go with it: a
SUMO shape is `x,y,z`, so the relief that plain OSM could only put in `ele` tags is
part of the geometry here.

Lanes are numbered from the **right in the direction of travel**, index 0 outwards.
The IR counts outwards from the reference line instead, which for the opposing
carriageway is the other way round — so the same physical lane has different numbers
in the two directions. `sumo_lane_ids()` is the way back:

```python
dict(m.sumo_lane_ids())["lane/north/1"]     # 'north.bwd_0'
```

`speed` is the road's limit in m/s, or SUMO's own default for the OSM `highway` value
the road type maps to. What may use a lane is the whole of what SUMO knows about lane
type: a driving lane is written as one pedestrians are kept out of, a footway as one
that admits only them, a bike lane only bicycles and a hard shoulder only emergency
vehicles.

### Right of way

SUMO's right of way is a **matrix over pairs of movements** — which stream gives way
to which other stream — and a plain XML file has no way to state one: `<request>` is
something netconvert computes and writes into the `.net.xml`. What the format does
have is the edge `priority` ladder, the same one the OpenStreetMap export climbs.

So a `RightOfWay` rule moves the **approach edges it names** one rung apart, and the
junction is marked `rightOfWay="edgePriority"`:

```xml
<node id="j_t" x="0.000" y="0.000" z="0.000" rightOfWay="edgePriority"/>
```

That attribute is what keeps this from being a hint. Without it netconvert weighs the
priorities against its own reading of the geometry, and a map that gives the stem of a
tee right of way over the road across the top of it gets the road across the top
anyway. With it, the numbers decide.

The rule names *lanes*, so the priority goes on the edges that carry them and not on
the road: a rule about the northbound approach leaves the southbound carriageway
where its road type put it.

What survives, then, is **which approach holds right of way** — and the tests check
that by reading the `state` netconvert wrote on each movement, not the priority the
export asked for. What does not survive is anything finer: whether a particular
movement of the priority arm must still give way to an oncoming one is netconvert's
decision, and `sumo_warnings()` says so.

A road whose **cross-section changes** becomes a chain of edges with a node between
them, because an edge has one lane count from end to end.

### Junctions, which SUMO models the same way round as the IR

The IR draws a junction as a set of connector roads, one per movement. SUMO draws it
as a **node**: the arms stop at its edge, and netconvert generates an internal lane
for every connection across it. The two models agree about the thing that matters —
the movements are enumerated, not guessed — so the connectors are not written as
edges. Each becomes the `<connection>` saying its approach lane may be left for its
exit lane:

```xml
<connection from="north.fwd" to="west.bwd" fromLane="0" toLane="0"/>
```

This is why the arms are left exactly where the IR puts them, 14 m short of the
centre: **the gap is the junction**, and netconvert fills it. Nothing here moves
geometry, which is the one thing the OpenStreetMap export has to do.

A junction with a traffic light on an approach becomes a `traffic_light` node.
netconvert generates the phases, because the IR holds no signal timing to write.

### What it cannot carry

- **Lane markings.** Which line is painted between two lanes, and in what colour, has
  nowhere to go.
- **One width per lane.** A tapering lane is written at its mean width along — the
  area of the lane divided by its length — and `sumo_warnings()` names it.
- **Superelevation.** A SUMO lane is flat across. The heights along it survive.
- **Lanes traffic does not run along** — borders, painted islands, parking bays — are
  dropped rather than written as something they are not.
- **Crosswalks and signs.** A SUMO crossing belongs to a node and a sign is an
  additional file, not part of the network.
- **A pairwise right-of-way matrix**, as above: the IR can say which approach holds
  right of way and no more.
- **The geo-reference.** The network is in the map's own metres about its origin, and
  the generated configuration turns off netconvert's offset normalisation so that it
  stays that way — the same coordinates as the other four exports.

## CARLA

[CARLA](https://carla.org/) is a driving simulator built on Unreal Engine. A map in it
is **two files that have to agree**: an `.fbx` holding the surface the sensors see, and
an `.xodr` holding the road network the traffic drives. This writes both, plus the
descriptor CARLA's importer reads and a manifest naming the textures.

```python
m.export_carla("Import/", name="Town01")
for warning in m.carla_warnings(name="Town01"):
    print(warning)
roadgen.fetch_textures("Import/Town01")     # optional; see Textures below
```

```text
Import/
├── Town01Package.json          what CARLA's Import.py reads
├── Town01Package.py            the build script: everything below, in order
└── Town01/
    ├── Town01.fbx              the surface
    ├── Town01.xodr             the road network — same name, which CARLA insists on
    ├── Town01.obj              the surface again, for the pedestrian navigation mesh
    ├── Town01_TrafficLights.fbx  the lights, as props tagged TrafficLight
    ├── Town01_TrafficSigns.fbx   the signs, as props tagged TrafficSign
    ├── furniture.manifest      where each light and sign stands (JSON)
    ├── map_logic.carla         CARLA's map_logic.json, to be copied beside the .xodr
    └── Textures/
        ├── polyhaven.manifest  what to fetch, and where each file goes (JSON)
        └── CREDITS.md
```

Getting from that folder to a level that runs is half a dozen steps in the right
order with the right environment — fetch the textures, copy the package into CARLA's
`Import/`, stage the pedestrian navigation builder, keep every other package's
descriptor there out of `Import.py`'s way (it imports every `.json` it finds), run
`Import.py`, give the level a sky, stand the lights and signs in it — so the exporter
writes them down as a Python script beside the descriptor, with what it knows baked in:

```python
m.export_carla("out/", name="Town01",
               carla_root="/opt/carla", engine="/opt/UE_5.5")   # or CARLA_ROOT / CARLA_UNREAL_ENGINE_PATH
```

```sh
python out/Town01Package.py            # imports it, adds the sky, says how to run it
python out/Town01Package.py --launch   # and starts the server on it
```

It removes what an earlier import of the same package left under
`Content/<Package>/` first: Unreal will not create a map's mesh assets in a folder
that already holds a level of the map's name, so a second `Import.py` over the first
fails the whole map group — logged, in the middle of a few thousand lines — while the
props, the `.xodr` and the level are all replaced, and what runs is the old road under
new everything else.

Run it with the interpreter that has `roadgen` and CARLA's own `carla` module, which
`Import.py` needs. `--carla` and `--engine` override what was baked in, `--no-sky`,
`--no-furniture` and `--no-textures` skip those steps.

The `.fbx` and the `.xodr` share a name because CARLA requires it in three separate
places: `Import.py` pairs them by name when it generates a descriptor itself, it copies
the `.xodr` to `Content/<Package>/Maps/<name>/OpenDrive/<name>.xodr` so that the level
and its road network agree, and `UOpenDrive::LoadXODR` finds the file by the level's own
name at runtime. The `.xodr` is written by the OpenDRIVE exporter rather than by this
one — two writers for one format would be two chances to disagree about the map CARLA
drives and the map it draws.

### A tag is a mesh name

This is the thing worth knowing about CARLA before writing anything for it, and it is
not in the documentation — it is in `MoveAssetsCommandlet.cpp`.

**CARLA's semantic segmentation does not read a mesh, a material or a property.** It
reads the *folder* the imported asset ended up in:

```text
  /Game/<Package>/Static/<Folder>/<Map>/<Mesh>
   0     1         2       3        4      5
                           ▲
                           └── ATagger::GetLabelByPath reads this, and only this
```

and the import puts an asset in a folder by matching its **name** against six
substrings. So the name of a mesh inside the FBX *is* its semantic ground truth,
spelled at one remove. The grammar is RoadRunner's, which is what CARLA was built
against:

```text
  <mapName>_<meshType>_<meshSubtype>_<ordinal>
```

| What roadgen writes | Folder | Tag | Stencil |
| --- | --- | --- | --- |
| `Town01_Road_Road_0` | `Road` | `Roads` | 1 |
| `Town01_Road_Marking_0` | `RoadLine` | `RoadLines` | 24 |
| `Town01_Road_Sidewalk_0` | `SideWalk` | `Sidewalks` | 2 |
| `Town01_Road_Curb_0` | `SideWalk` | `Sidewalks` | 2 |
| `Town01_Road_Gutter_0` | `SideWalk` | `Sidewalks` | 2 |
| `Town01_Terrain_Ground_0` | `Terrain` | `Terrain` | 10 |

Get it wrong and nothing fails. The mesh imports, the map loads, the camera renders —
and the pavements come back labelled as ground. `Map.carla_warnings()` exists mostly
for this, and so does the [picture](#which-viewer-draws-what): the CARLA panel is
coloured by running each mesh name back through CARLA's own classifier, so a mesh drawn
in the wrong colour is one that will segment wrongly in the simulator.

Three things found by reading the commandlets rather than the documentation, each of
which will quietly ruin a map, and each of which `carla_warnings()` reports:

- **The match is `Contains`, in a fixed order, and `Terrain` is tested bare and third.**
  Every mesh name begins with the map's name — so a *map* called `TerrainTown` puts its
  pavements, kerbs and gutters in the Terrain folder, and keeps its road surface,
  because `Road_Road` is tested first. It is exactly the kind of failure that survives
  a screenshot.
- **Anything unrecognised becomes Terrain.** The classifier's last arm is not "leave it
  alone", it is `Terrain`. A typo in a mesh name does not fail to import; it segments as
  ground.
- **`light` and `sign` are rejected outright.** `ValidateStaticMesh` drops any mesh
  whose name — or whose *material's* name — contains either word, case-insensitively,
  before it is placed. A road called `Sign Street` takes its whole surface with it, and
  nothing is logged.

### The surface

Six classes, from the cross-section the generator already produced — the real one, lane
by lane, sampled along the reference line, so a road that tapers or gains a lane is
measured where it actually is:

```text
   grass        pavement          road surface         pavement       grass
  ┌──────┐┌───────────────┐┌─────┬─────────────┬─────┐┌────────────┐┌──────┐
  │verge ││   sidewalk    ││gut- │   driving   │gut- ││  sidewalk  ││verge │
  │      ││               ││ ter │             │ ter ││            ││      │
  └──────┘└───────────────┘└─────┴─────────────┴─────┘└────────────┘└──────┘
            ▲             ╲                           ╱             ▲
            │              ╲ curb                curb ╱             │
            └─ raised by the kerb height ──────────────┘            └─ Terrain
```

A lane whose type is `sidewalk` is raised by the kerb height, which puts a vertical face
between it and the surface beside it — that face is the **curb**, and the strip of road
at the foot of it is the **gutter**, split off the outermost part of the carriageway the
way a real one is. Neither is a lane in the IR and neither should be: they are what a
*surface* has and a road network does not. Painted lines are geometry rather than
texture — a broken line is a strip per dash, cut by arc length so that a three-metre
dash is three metres however the sampler laid its vertices.

Beyond the outermost band is the **land**: one mesh, from every road's edge out to
150 m past the network (`ground_extent`), for a lidar to reach and a vehicle that
leaves the road to land on. It is a constrained Delaunay triangulation, not a grid,
because a grid cannot meet a road's edge and the land has to: its constraints are
the roads' own outer edges, vertex for vertex the points the surfaces end on, so no
triangle crosses a road and what is inside a road's outline is cut away. Next to each
edge the land slopes down over the **verge** (`verge_width`, 8 m) to the ground's
height, and past the verges its vertices are a height field (`ground_cell` apart)
whose height at each vertex is the lowest nearby road's, a little under, so it is
flat where the roads are flat and slopes gently between roads at different heights.
It is not terrain, and it is not pretending to be — a road network says nothing about
the shape of the country it runs through, and generating hills here would be
inventing a terrain model rather than deriving one. CARLA's editor is still where a
map gets hills.

One mesh, meeting every edge exactly, is the point. A grass strip per road laid over
a grid under all of them is two surfaces wherever they overlap and a step wherever
they do not quite meet: the strips of neighbouring roads cross at a junction, a strip
round a tight corner folds over itself, and a lidar return or a wheel finds every
seam. A road through a junction has land on one side of it at most — the side that
faces away from the junction — and a raised pavement facing the junction gets a kerb
face down to the road level instead, which the land meets at the foot. And a vertex
of the land goes in only where it really is a verge's width from every road's edge
— for a toe, every edge but the two chords of its own rail it stands on — which is
the one rule that keeps ledges out of it: the toe of a verge on the inside of a bend
tighter than the verge is wide has folded back over the rest of its rail, and one
pushed out from a road in a junction lands beside the pavement round the corner;
either is nearer than a verge to some edge and is left out. The land is tagged
`Terrain` and is the one mesh of that class.

### Pedestrians

CARLA's walkers go where its navigation mesh says they can, and that mesh is not
built from the level: `Import.py` runs `RecastBuilder` over an `.obj` of the map, in
which every triangle is labelled by the material it wears — `road`, `sidewalk`,
`crosswalk`, `grass`, or a block — and a walker spawns on a sidewalk, crosses at a
crosswalk, and keeps off the rest. So the package carries the surface twice: the
`.fbx` the level is built from, and a `<name>.obj` in CARLA's frame (y up, the
roadgen y negated) whose materials are those labels. The tool that reads it is one the
UE5 branch builds but does not put where `Import.py` looks, so the build script
finds it under `Build/` and stages it, with the `.obj`, into `Util/DockerUtils/dist/`;
the navigation comes out as `Content/<Package>/Maps/<name>/Nav/<name>.bin`, which the
server hands to clients and `world.get_random_location_from_navigation()` draws on.

Two things about that `.obj` are not obvious and both were found the hard way. The
loader labels triangles in file order and the last label wins, so the meshes are
written in order of precedence — buildings and kerbs first, then the land, the
roads, the sidewalks and last the markings — so that where two meet along an edge
the one a walker should be on wins the voxels there. And the crosswalk bands in it are not the
painted stripes: they are one quad per crossing, `crosswalk` from kerb to kerb,
because the stripes are what a camera sees and the band is what a walker needs.

Crosswalks *are* painted, too — a bar and a gap of half a metre each, across the
carriageway, from the crossing objects in the IR — and so are stop lines, as a bar
across the lane on the traffic's side of the line. Both are tagged `RoadLines` like
every other marking, so the walkers cross, and the vehicles halt, on something the
cameras can see.

### Where the town goes, and why it is a choice

CARLA's map import has exactly six names in it. `UMoveAssetsCommandlet` sorts a map's
meshes into `Road`, `RoadLine`, `SideWalk` and `Terrain` and nothing else, and then
`UPrepareAssetsForCookingCommandlet` places into the world exactly what it finds in
those four folders. **A building can be in the map or correctly tagged, and the import
pipeline will not do both** — so the package's script does the placing itself.

```python
m.export_carla("Import/", buildings="in_map")   # the default
m.export_carla("Import/", buildings="props")
m.export_carla("Import/", buildings="omitted")
```

| | Placed by | Tagged |
| --- | --- | --- |
| `in_map` | `Import.py` | `Terrain` (10) |
| `props` | `roadgen.carla_furniture` | `Buildings` (3) |

`in_map` is the default, because a wrong label on a building is recoverable in the
editor while a town that depends on one more step is a town that is sometimes not
there. `props` writes a second `.fbx` and declares it in the descriptor's `props`
array with `tag: "Building"` — which is the one place in the whole pipeline where a tag
is *stated* rather than spelled into a mesh name — and lists every building in
`furniture.manifest`, so the same editor run that stands the traffic lights stands
the town, tagged as a town. The pedestrians' navigation mesh carries the buildings
either way.

### Facades

The IR's building is a massing model — an outline, walls, a roof, a storey count and
the street it faces — and by design it has no windows: a facade's openings are
neither parts nor roofs, and no format the IR is written to has a word for them. A
camera does not care. To a camera a wall with no windows is a warehouse, and a street
of them is a street of warehouses whatever the grammar called them.

So the CARLA surface puts them in. Each wall is divided into bays at the pitch its
building kind uses — a house every 3 m, a shed every 4.5 — every storey of every bay
gets a window sized for that kind, a shop's ground storey is a shopfront, and the
ground storey of the wall that faces the building's street gets a door in its middle
bay. Each opening is two quads a couple of centimetres proud of the wall, a frame and
the glazing inside it, so the wall's own quad and the building's semantic tag are
untouched. The rhythms are in `roadgen-carla/src/facades.rs`, one per kind the
presets emit; a kind the grammar made up is built like a house.

### The sky, which the import does not give you

CARLA's importer builds every map on its `BaseMap`, and in the UE5 branch that level
has a sun, a sky light and nothing for either to light: no atmosphere, no fog, no
exposure. A map imported through `Import.py` renders its surfaces and, above them,
black. CARLA's towns get their sky from a `BP_Carla_Sky` actor the weather system
drives — but that actor is not in the `BaseMap`, and dropping one into an imported
level does not give it the daylight the towns have, because those are lit by per-map
weather sublevels with baked lighting (`Town10HD_Opt/Weathers/T10HD_Day`) that an
imported map has not got.

```python
roadgen.carla_sky("/opt/carla", "Town01Package", "Town01")   # after Import.py
```

runs the editor once more, on the level `Import.py` made, and gives it a plain
Unreal daylight: a sky atmosphere and height fog, a physical sun of 75 000 lux in
place of `BaseMap`'s dim one, a sky light that captures the sky it now has, and an
unbound post-process volume that turns auto exposure on — the project leaves it off,
and without it a physical sun is a white frame. Running it twice changes nothing.
`python -m roadgen sky …` is the same thing from a shell. What it does not do is
hook the sky into `set_weather()`, which will store its parameters and move nothing:
the only sky that API knows how to move is the one the towns have. The sun's position
is an argument here instead.

### Traffic lights and signs, which CARLA would otherwise put in the road

CARLA spawns traffic lights of its own. Its `ATrafficLightManager` reads every
`<signal>` in the `.xodr` and, for each light, puts a `BP_TLOpenDrive` blueprint at
the signal's position — and a signal written where the IR holds the light is five
metres over the middle of the lane, so what appears is a pole in the carriageway. The
stop, yield and speed-limit signs get the same treatment with `BP_Stop01` and its
kin. And a light mesh of one's own in the map's FBX never arrives: `ValidateStaticMesh`
drops any mesh whose name holds `light` or `sign` before it is placed.

So the furniture is built, and it is built to the road: a pole on the pavement,
`kerb_setback` in from the kerb on the side the governed traffic keeps to; a mast arm
from there, perpendicular to the road, long enough to reach `arm_overhang` past the
middle of the farthest lane the light governs, at the height the IR gave the bar; a
three-lamp head hung from it over the middle of every governed lane, facing the
traffic. A sign is a post on the same pavement, `sign_setback_along` before the line
it applies to, with an octagon, a triangle, a disc or a square on it for what the
code says it is. A road with no pavement stands both on the verge instead.

Then three files have to agree, and the exporter writes all three:

- **The props.** `Town01_TrafficLights.fbx` and `Town01_TrafficSigns.fbx`, declared in
  the descriptor with the tags `TrafficLight` and `TrafficSign`. That is what a
  segmentation camera reports them as (7 and 8), and it is also why they are props:
  the import's four map folders have no such class. A light is two meshes in that
  file — the post, with the arm and the heads' housings, and the three lamps of each
  head on their own — for a reason given below.
- **The `.xodr`.** Each signal is written at the foot of its post — `t`, `zOffset`
  and a `<positionInertial>` — with its `s` where it *applies*: at the stop line of
  the rule that names it, else at the bar. CARLA builds a light's stop boxes three
  metres before `s`, so a light whose rule has a stop line set back from the mouth
  halts traffic at that line rather than under its own heads. Every light is in a
  `<controller>`, one per traffic-light rule (the lights of a rule switch together)
  and, for a light no rule names, one per approach; each junction names the
  controllers that are its own. A stop, yield or speed-limit sign is written in the
  German catalogue CARLA reads (`206`, `205`, `274` with the limit as its subtype and
  value), whatever spelling it was added with — `stop`, `de205`, `de274-50`,
  `speed_limit_50`, `R1-1`. Any other code goes through verbatim.
- **`map_logic.carla`.** CARLA's `map_logic.json`, which its `InitializeTrafficLights`
  looks for beside the `.xodr`: with it there, CARLA spawns no lights of its own and
  instead turns the actor within fifty centimetres of each listed signal into a
  working `ADigitalTwinsTrafficLight` — the signal's id, the junction's controller,
  the stop boxes, and lamps driven through any material named `TrafficLight` that
  has an `Emissive Intensity`. It is carried under another name because `Import.py`
  takes every `.json` under `Import/` for a package descriptor.

`Import.py` imports the props and places nothing, so the package's script runs the
editor once more:

```python
roadgen.carla_furniture("/opt/carla", "Town01Package", "Town01")   # after Import.py
```

reads `furniture.manifest`, spawns each prop at the foot of its post facing its
traffic, copies `map_logic.carla` beside the `.xodr` as `map_logic.json`, gives the
lights' lamp slots instances of CARLA's own `M_TrafficLights` so they switch, sets
each prop to collide as its triangles rather than as the convex hull the importer
generated (the hull of a pole and an arm is a wedge across the lane), and — for each
stop, yield and speed-limit sign — stands a bare `ATrafficSignBase` in the matching
state at the post, which is what CARLA's `SpawnSignals` looks for before spawning a
plate of its own. Running it twice places once. `python -m roadgen furniture …` is
the same from a shell.

Two details of CARLA's adoption decide how the props are placed. It rebuilds the
actor it adopts by copying each mesh component's *relative* transform onto a new
actor at the old one's location, so a `StaticMeshActor` — whose mesh is its root —
would be placed twice over; every prop is therefore a bare actor with a scene root
and the mesh as a child. And the copy is made after CARLA has tagged the level, so
the adopted mesh segments as nothing: hence the split, with the lamps standing
exactly at the signal as the actor CARLA finds, and the post a metre outside its
search, untouched and tagged. Only the lamps go unlabelled.

`export_carla(furniture=False)` leaves all of it out: the signals stay where the IR
put them and CARLA improvises, which `carla_warnings()` says. Two things it also says
that are CARLA's rather than this exporter's: a light on a lane that leads into no
junction gets a controller of its own that CARLA runs alone, logging that it cannot
apply the timing; and a 110 km/h sign gets CARLA's plate spawned beside ours, since
CARLA has a model for that limit and no sign state to match a placed one against.

### Textures

The surfaces are scanned PBR materials from [Poly Haven](https://polyhaven.com),
published under CC0, so a generated town can ship with photographic surfaces and no
licence attached to them.

**Nothing is fetched while exporting.** An exporter that reached for the network could
not run offline, could not run in CI and could not run in the browser — and the
[demo page](#the-demo-page) runs this crate compiled to WebAssembly. So the package
*references* its textures and carries a manifest saying exactly which Poly Haven asset,
at which resolution, belongs at which path; `roadgen.fetch_textures(folder)` reads that
manifest and downloads them.

A package with no textures in it is still a complete, importable CARLA package: every
material carries a flat colour that stands in for its texture. And CARLA replaces them
all anyway when `use_carla_materials` is on, which it is by default — its road materials
are built for its lighting and its sensors, and a map that wears them looks like the
maps it will be benchmarked against. Pass `use_carla_materials=False` to see the
package's own.

Lane markings have no texture and are not meant to: a painted line is flat white or flat
yellow, its edge comes from the geometry, and scanned paint would tile visibly down a
straight line. The two marking materials are colours — and `M_RoadMarking_Yellow` is
named that way on purpose, because `PrepareAssetsForCooking` picks CARLA's yellow lane
material for the slot whose name holds `Yellow`.

The slugs in the manifest are the ones roadgen was written against, and Poly Haven's
catalogue is theirs to change. A slug that has gone is reported by name rather than
guessed at, and the manifest is JSON in the package: swapping one is editing a file.

### Coordinates, which is the part that can silently be wrong

Three conventions have to line up and only two of them are roadgen's.

**roadgen** is right-handed: x east, y north, z up, metres. **CARLA and Unreal** are
left-handed — `carla::geom::RightHandedVector3D` says so in as many words, and
`MapBuilder.cpp` negates the y of every OpenDRIVE point it parses under a comment
reading `Unreal Y axis hack`. **Unreal's FBX importer** closes the gap: with
`bConvertScene` set, which CARLA's import settings do set, it converts the frame the
file declares into Unreal's, negating Y on the way.

So the mesh is written in roadgen's own frame, unflipped, and declared Z-up
right-handed; the importer flips it, CARLA flips the `.xodr` the same way, and the two
land on top of each other. Flipping Y here as well would flip it twice, and a map
mirrored about its own centreline is a map whose roads all turn the wrong way — which
looks plausible enough in a screenshot to survive a review. Units are metres, declared
as such, and CARLA's importer scales them into Unreal's centimetres itself.

The FBX is written in **text** rather than binary. Binary FBX is smaller and imports
faster; it also needs DEFLATE for its array records and ends in an undocumented footer
block the SDK checks. Text needs neither, so this crate has no compression dependency,
no magic constants, and produces a file that can be opened in an editor and diffed when
a map comes out wrong.

### What it cannot carry

`Map.carla_warnings()` reports all of it. Beyond the tagging traps above and whichever
trade the town was exported under:

- **A junction is written once per movement through it.** A junction in the IR is a
  connector road per turn, and each carries its own carriageway, so the middle of a
  junction is coplanar with itself and will z-fight. CARLA's own maps have a single
  junction surface, which a road network has no way to describe.
- **The furniture is placed by the script, not by the import.** Until
  `carla_furniture` has run, the level has the signals and none of the meshes; and
  the sign codes CARLA has no meaning for stand, segment as signs and govern nothing.
- **Roughness maps are fetched but not wired up.** FBX's material model is Phong, which
  has a shininess exponent and no roughness map. The files are in the package for a
  material rebuilt in the editor.

## ClipGT

ClipGT is the scene format NVIDIA's Cosmos world-scenario tooling reads: a directory
of Parquet files named `{clip_id}.{layer}.parquet`, one row per element.

```python
clip_id = m.export_clipgt("clips/", scenario="scenario.yaml")
```

| Layer | What goes in it |
| --- | --- |
| `lane` | each drivable lane's two rails, in travel order |
| `lane_line` | one row per painted cross-section edge, with its colour and style |
| `road_boundary` | the outermost edge on each side of every cross-section |
| `crosswalk`, `wait_line` | crossings and stop lines |
| `traffic_light`, `traffic_sign` | position, orientation and category |
| `intersection_area` | a junction's outline |
| `egomotion_estimate`, `calibration_estimate` | see below |

Coordinates are **FLU** — x forward, y left, z up — which is the same shape of frame
as the IR's east-north-up, so they are written through unchanged, heights and all. A
rail on a graded, banked road carries the elevation and the cross-fall the generator
computed for it; nothing here is projected to the horizontal plane.

**It is a scene format, not a map format.** A reader will not look at a directory at
all unless it holds an egomotion table, so a map on its own cannot be written as a
clip — something has to drive through it, and something has to be looking.

### The scenario file

Where the vehicle drives and what is bolted to it are not properties of the map: the
same network should be drivable several ways, with several rigs, without editing it.
So both come from a YAML file.

```yaml
clip_id: town
frame_rate: 30.0
speed: 12.0                    # metres per second

route:
  start: lane/north/0          # set off here and follow successors
  # lanes: [lane/north/0, ...] # or drive exactly these, in this order

sensors:
  - name: camera:front_wide_120fov
    position: [1.7, 0.0, 1.45]       # metres, in the rig's forward-left-up frame
    roll_pitch_yaw: [0.0, 0.0, 0.0]  # degrees
    width: 1920
    height: 1080
    fov_degrees: 120.0               # an ideal equidistant f-theta lens
  - name: camera:rear_tele_30fov
    position: [-0.9, 0.0, 1.3]
    roll_pitch_yaw: [0.0, -1.5, 180.0]
    width: 1920
    height: 1080
    cx: 955.0                        # defaults to the middle of the frame
    polynomial: [0.0, 1830.0, 0.0, 0.0, 0.0, 0.0]
    polynomial_type: angle-to-pixeldistance   # the default
    linear: [1.0, 0.0, 0.0]                   # the affine term, identity by default
```

[`examples/clipgt-scenario.yaml`](examples/clipgt-scenario.yaml) is this file, kept in
the repository and loaded by a test so that it cannot quietly stop working.

Everything is optional and anything left out keeps the value it already had, so a
scenario can say only what it wants to change. Arguments passed to `export_clipgt`
override the file; arguments left out keep it. **An unknown key is an error**, not a
silence: a scenario is written by hand, and a `speed_kph` that should have been
`speed` is worth being told about.

`fov_degrees` is a convenience, not a calibration: it produces the textbook
equidistant fisheye `r = f·θ` with `f` chosen so the horizontal half-angle lands on
the edge of the frame. A measured lens goes in `polynomial` instead.

A sensor's name is prefixed with `camera:` if it has no prefix, because a reader
discovers cameras by that prefix and skips anything else. Each one also gets a
`{clip_id}.{camera}.json` of frame timestamps, so a reader that syncs poses to frames
finds the frames the track was actually generated at rather than resampling.

With no `sensors`, the calibration table still holds a rig — an empty one, which is
what the IR knows about cameras on its own. The clip loads; there is just nothing to
render from, and `clipgt_warnings()` says so.

**What it cannot carry.** ClipGT has no topology: a lane is two rails, with no
successor, no predecessor and no junction movement, so a clip is a picture of the
roads rather than a network you can route on. Element ids are the Parquet row index,
so the IR's stable identifiers do not survive either. `Map.clipgt_warnings(scenario)`
reports that rather than letting a caller assume a round trip, and checks the
scenario's route against the map while it is there. The `pole`, `road_island` and
`road_marking` layers have no counterpart in the IR and are not written.

## GPUDrive

[GPUDrive](https://github.com/Emerge-Lab/gpudrive) is a GPU-accelerated driving
simulator that loads scenes from JSON: one file holding a map as polylines and the
agents driving it as logged tracks.

```python
m.export_gpudrive("scene.json", scenario="examples/gpudrive-scenario.yaml")
```

There is no published schema for that file. What there is is the simulator's reader,
[`src/json_serialization.hpp`](https://github.com/Emerge-Lab/gpudrive/blob/main/src/json_serialization.hpp),
and the data model in `roadgen-gpudrive` is shaped by it field for field: every key it
insists on, every key it will do without, and the strings it compares types against.

| Element | What goes in it |
| --- | --- |
| `lane` | each drivable lane's centreline, in travel order |
| `road_line` | a painted boundary between lanes, with its Waymo line code |
| `road_edge` | the edge of the drivable surface, and a kerbed median |
| `crosswalk` | the outline of a crossing |
| `stop_sign` | a single point, where a stop sign applies |
| `objects` | one track per agent: position, heading, velocity and validity per timestep |

`map_element_id` is the Waymo Open Motion feature code the simulator's `MapType` is
built from, so a solid single white line goes out as 7 and a median kerb as 16. The
markings the IR carries map onto it directly; a broken-and-solid pair painted white
has no code there and goes out as `ROAD_LINE_UNKNOWN` rather than as a colour it is
not.

**GPUDrive is a plane.** A position is an `(x, y)` in metres and the document has no z
anywhere, so the elevation, grade and superelevation the generator computed are
dropped at the boundary. Distances along an agent's route are measured in plan view
for the same reason: an agent asked for 10 m/s covers ten metres of ground a second,
not ten metres of a climbing road.

Two decisions do not fall out of the IR on their own. **An edge is one element**:
either the boundary of the drivable surface or a painted line, never both, because the
IR has one curve there and writing it twice would put two elements on top of each
other. And **a junction connector has no edges** — its lane centreline is written,
because that is the path through the intersection, but its boundaries are not, since a
`road_edge` is something an agent collides with and there is no wall down the middle of
a junction.

### The scenario file

A scene is a map *and the agents driving it*; a generated map has no logged traffic, so
the agents come from a file.

```yaml
name: town
scenario_id: town-0001
steps: 91                  # timesteps; 91 is GPUDrive's episode length and its maximum
time_step: 0.1             # seconds between them

agents:
  # The first agent is the scene's self-driving car: `sdc_track_index` is 0.
  - type: vehicle          # vehicle, pedestrian or cyclist
    speed: 12.0            # metres per second, held for the whole route
    route:
      start: lane/north/0          # follow successors from here
      # lanes: [lane/north/0, ...] # or drive exactly these, in order
    length: 4.6            # metres; left out, an agent is the ordinary size of its kind
    width: 2.0
    height: 1.6
    track_to_predict: true # listed in the scene's tracks_to_predict; the default
    difficulty: 0
  - type: cyclist
    speed: 4.5
    mark_as_expert: true   # replayed from the track rather than handed to a policy
    of_interest: true      # listed in the scene's objects_of_interest
```

[`examples/gpudrive-scenario.yaml`](examples/gpudrive-scenario.yaml) is this file, kept
in the repository and loaded by a test so that it cannot quietly stop working.

Everything is optional and anything left out keeps the value it already had. Arguments
passed to `export_gpudrive` override the file; `speed` and `route` apply to the first
agent, which is the scene's own vehicle. **An unknown key is an error**, not a silence.

An agent drives its route at a constant speed and stands at the end of it for whatever
is left of the episode, rather than vanishing — every timestep is `valid`, because an
agent that is there for the whole scene is what the others have to deal with. Where its
track ends is its `goalPosition`, which is what the simulator scores a policy against.

**What it cannot carry.** A scene holds no lane topology: a lane does not say what it
leads to, so an agent's route is baked into its track rather than routed at load time.
There is no traffic-light element and no stop-line element, so signals and the phases
they govern are dropped — and of signs, only the stop sign has anywhere to go. Lane
widths go with them: a GPUDrive lane is a centreline. Road element ids are the row
index in the written file, so the IR's stable identifiers do not survive either.
`Map.gpudrive_warnings(scenario)` reports all of that rather than letting a caller
assume a round trip, and checks the scene against the reader's fixed buffers while it
is there — 515 objects, 956 road elements, 1746 vertices an element and 91 timesteps,
past which the simulator silently drops what does not fit. The `speed_bump` and
`driveway` elements go the other way: the IR has nothing that means either, so they
are never written.

## The demo page

<https://hakuturu583.github.io/hdmap_generator/>

An editor on the left, a viewer on the right. Write roadgen, press Run, and the viewer
fills with what the script wrote; the switch above it flips between the formats — in
the same frame, so what changes is the format and nothing else. That is the argument
the rest of this README makes, made in one screen: one description of a road network,
seven exports, and what each format could and could not carry written under each
picture.

Whichever format is showing, its files are listed under the switch with their sizes, a
`source` toggle and a `save` button, so the bytes behind the picture are a click away.

**It is the package, not a demonstration of the package.** The page loads
[Pyodide](https://pyodide.org/) — CPython built for WebAssembly — and installs
roadgen's own wheel into it, cross-compiled to `wasm32-unknown-emscripten`. The
`import roadgen` on the page is the `import roadgen` everywhere else, so there is no
second implementation to keep in step and nothing that can be right in the demo and
wrong in the library. The generation runs in the tab: no file is uploaded, no server
does the work, and the page keeps working with the network unplugged once it has
loaded.

The script is run in a directory of its own and then the directory is *looked at*.
The tabs are not a fixed set — they are whatever turned up — so a script that exports
one format gets one tab. Neither the page's code nor its stylesheet names a format:
recognising a *new* one is a line in `driver.py`, the file that already knows which
reader to hand a written export to. The format you were looking at survives a re-run,
which is what makes it possible to change one line and watch one format.

### Which viewer draws what

| Export | Drawn by | Why |
| --- | --- | --- |
| Lanelet2, OpenStreetMap | [Leaflet](https://leafletjs.com/) and [osmtogeojson](https://github.com/tyrasd/osmtogeojson) | Both are OSM XML in latitudes and longitudes. A map library already knows what to do with that, and puts the result on the Earth rather than on a blank sheet. |
| OpenDRIVE, SUMO, CARLA, ClipGT, GPUDrive | `roadgen-viewer`, in this repository | There is no off-the-shelf browser viewer for these that an Apache-2.0 project can ship. |

`roadgen-viewer` reads the **written file** — with the same libraries that wrote it,
and never the IR — and draws it as SVG. That is what makes the pictures worth looking
at: a panel can only be right if the file is. It is not browser-only, either; it is
in the wheel, so a notebook can draw an export too:

```python
from IPython.display import SVG
m.export_opendrive("map.xodr")
SVG(roadgen.render_opendrive("map.xodr"))       # or render_sumo, render_carla, render_clipgt, render_gpudrive
```

OpenDRIVE is the interesting one to draw. It does not hold a road as a shape: it
holds a reference line as a chain of lines, arcs and clothoids, and every lane as a
polynomial width measured sideways from it. So the picture is *evaluated* rather than
read, which means it goes wrong in exactly the ways the geometry can — and a lane
that lands in the wrong place on the page is a lane in the wrong place in the file.
It is also the only one of the four that draws [buildings](#buildings), because it is
the only one of the four whose format has anywhere to put them. Every *part* is drawn,
not just the outline: a wing behind a house and a tower set back on a podium are both
things you would see from above. An object with no outline is not drawn at all —
OpenDRIVE lets one say "a building, this wide and this long", and a box inferred from
two numbers is a guess rather than a footprint.

Everything is a plan view. The heights, the grades and the superelevation are in the
files and not on the screen, and every drawing says so in its own description — one
clause, written once where the SVG is written — rather than leaving a reader to assume
the map is flat.

### Building it

```bash
# The wheel, for Pyodide rather than for this machine.
pip install pyodide-build==0.39.0
pyodide xbuildenv install --url \
  https://github.com/pyodide/pyodide/releases/download/0.28.3/xbuildenv-0.28.3.tar.bz2
source /path/to/emsdk/emsdk_env.sh      # Emscripten 4.0.9
USE_LEGACY_PLATFORM=1 \
  CARGO_TARGET_WASM32_UNKNOWN_EMSCRIPTEN_RUSTFLAGS="-C link-arg=-sSIDE_MODULE=2" \
  RUSTUP_TOOLCHAIN=nightly-2026-09-17 \
  pyodide build -o dist

# The page, which is the wheel plus Pyodide plus three JavaScript libraries.
cd web && npm ci && node build.mjs --wheel ../dist/*.whl
node smoke.mjs                          # opens site/ in headless Chromium
```

Four versions are pinned to each other and none of them floats: Pyodide fixes the
Python version and the ABI tag the wheel must carry, its cross-build environment fixes
the Emscripten version, and the Rust toolchain has to satisfy both — stable cannot
link this target, because `emcc` rejects the mangled names it emits. The same numbers
appear once each in [`.github/workflows/pages.yml`](.github/workflows/pages.yml) and
[`web/build.mjs`](web/build.mjs), with a note beside each saying what breaks if it
moves.

The published page is self-contained. Pyodide, Leaflet, osmtogeojson and CodeMirror
are all copied in at build time — from npm and from a GitHub release, pinned by
`web/package-lock.json` — so the page makes no request to a CDN and what ships is what
the lockfile says. The one thing it does fetch at runtime is OpenStreetMap's map
tiles, and only for the two panels that have a place on the Earth to show.

Publishing needs Pages enabled for the repository with **GitHub Actions** as the
source. A pull request builds the page and runs it in a browser; only `main` deploys.

## Validation

`validate()` reports everything wrong at once, rather than failing on the first
problem. It checks identifier uniqueness, dangling references, lane/road agreement,
road-link reciprocity, that linked roads actually meet in space, that a connection
leaves and enters by the ends traffic really uses, that connected lanes' centrelines
and boundaries are continuous in XYZ, that a lane's boundaries are the right way
round and span its width, junction membership, and the map's coordinate metadata.

Each exporter adds the constraints its own format imposes, through
`roadgen_opendrive::check` and `roadgen_lanelet2::check` (`Map.format_warnings()`
from Python). Those constraints stay on the exporter's side of the boundary and are
never pushed back into the IR.

## Reading OpenDRIVE back

```python
m = roadgen.read_opendrive("map.xodr")
for note in m.read_warnings():
    print(note)
m.export_lanelet2("map.osm")
m.export_sumo("sumo/")
```

```rust
let imported = roadgen_opendrive::read("map.xodr")?;
for note in &imported.approximations {
    eprintln!("{note}");
}
let map = imported.map.validate()?;
roadgen_lanelet2::write(&map, "map.osm")?;
```

The exporter's table — road to `<road>`, lane to `<lane>`, connection to
`<laneLink>` — read the other way. Nothing is regenerated: the reference line is
rebuilt piece for piece from `<planView>` (a line, an arc and a spiral as
themselves, a `<paramPoly3>` as the cubic it is), the widths from `<width>`, the
movements from the lane links and the junctions, the lights, signs, stop lines,
crosswalks and buildings from the signals and objects, and the controllers and
priorities as rules. The lane boundaries are then laid out by the **same code the
builder uses** for a road it generated, so a file roadgen wrote comes back with the
boundaries it was written from, and a file from somewhere else gets the boundaries
roadgen would have given it. The map that comes back is an `UnvalidatedMap`, and
`validate()` is the same gate every generated map goes through.

The IR is narrower than OpenDRIVE in a few places, and where it is, the reader
approximates and says so — `Imported::approximations` in Rust, `read_warnings()` in
Python, one line per thing the map now says less exactly than the file did:

- a `<width>` that is a straight run or the IR's own smooth ease is read exactly;
  any other cubic is sampled into straight runs;
- a width that reaches zero is held at a centimetre, because a `WidthProfile`
  cannot reach zero;
- an `<elevation>` that curves inside one plan-view piece is cut into straight
  grades at the sampling length, since every piece of the IR climbs at one grade;
  a `<paramPoly3>` cannot be cut and takes one grade end to end;
- a lane type the IR does not have — `stop`, `median`, `bus`, the ramps — lands on
  the nearest it does, and a road type likewise;
- a building's roof is flat, because an outline has one height per corner;
- a signal or object that names no lanes is read as governing every lane of its
  section that runs the way it faces.

What is an error rather than a note is a document that is not a road network: a
reference line with a gap in it wider than 10 cm (a narrower one is closed and
reported), a link to a road that is not there, a lane link to a lane the neighbour
does not have. A `<geoReference>` the reader does not recognise places the map at
latitude 0, longitude 0 and says so; `origin=` overrides it.

Roads and junctions are named after the file's ids, so the file's road 12 is
`road/12`; a signal, object or building whose `name` is one of the IR's own
identifiers gets it back, which is what makes a map that goes out and comes in keep
its object names. A map read from a file cannot be added to — `add_road` and the
rest refuse — because it has no builder behind it: roadgen generates maps and reads
them, it does not edit them.

`tests/integration/tests/reimport.rs` writes every scenario, reads it back and
writes it again, and asks the independent evaluator whether every lane edge of the
second document is where the first put it. It is.

## Agreement between OpenDRIVE and Lanelet2

The test suite exports each scenario, reads the OpenDRIVE back with the `opendrive`
crate's parser and the Lanelet2 map back with `simple_lanelet2`'s loader, and
compares:

- **Topology.** Lanelet2 has no successor tag — a routing graph rediscovers
  connectivity from shared boundary points — so `tests/integration` asserts that the
  graph finds exactly the movements the IR holds, and that OpenDRIVE's lane links and
  junction connections account for the same ones.
- **Geometry.** Every lane centreline is evaluated out of the OpenDRIVE document
  independently (`<planView>`, `<elevationProfile>`, `<laneOffset>`, widths) and
  compared with the IR's vertices, which are also the Lanelet2 map's vertices.

Where a map's reference lines are tangent-continuous the two agree to within a
micrometre — and the generator makes them so: **where two roads meet at an angle** the
corner is [rounded into an arc](#corners) before anything is written, so the joint
that used to be the one place the formats disagreed is now a place they agree.
`two_roads_that_meet_at_an_angle_are_rounded_so_the_formats_agree` pins that down.
The mitre the IR applies where road ends meet is still there for the joints that
remain — a junction's arms, and roads that meet along one tangent — and is what makes
their boundary points the *same* points rather than merely nearby ones.

## Layout

```
roadgen/
├── crates/
│   ├── roadgen-core/        canonical IR, generation, validation
│   │   ├── topology/        connectivity
│   │   ├── geometry/        3D curves, alignments, frames, profiles, sampling
│   │   ├── layout/          a cross-section laid out against a reference line
│   │   ├── semantics/       lane types, rules, markings, objects
│   │   ├── buildings/       footprints, heights, storeys
│   │   ├── id/              typed identifiers
│   │   └── validation/      UnvalidatedMap → ValidatedMap
│   ├── roadgen-buildings/   a town beside the roads, from a CGA shape grammar
│   ├── roadgen-carla/       meshes, FBX and the package CARLA UE5 imports
│   ├── roadgen-opendrive/   lowering onto the `opendrive` crate, and reading it back
│   ├── roadgen-lanelet2/    lowering onto `simple_lanelet2`
│   ├── roadgen-clipgt/      lowering onto ClipGT's parquet layers
│   ├── roadgen-gpudrive/    lowering onto GPUDrive's scene JSON
│   ├── roadgen-osm/         lowering onto plain OpenStreetMap XML
│   ├── roadgen-sumo/        lowering onto SUMO's plain-XML network
│   ├── roadgen-viewer/      reading the exports back, and drawing them as SVG
│   └── roadgen-python/      PyO3 bindings
├── examples/                ClipGT and GPUDrive scenario files
├── web/                     the demo page: roadgen in a browser, through Pyodide
├── python/roadgen/          the Python package
├── tests/
│   ├── integration/         scenarios and cross-format checks
│   └── python/              the Python front end
├── Cargo.toml
├── pyproject.toml
└── LICENSE
```

## Building and testing

```bash
cargo test --workspace          # Rust: unit, export round-trip, scenarios
cargo clippy --workspace --all-targets
cargo fmt --all --check

maturin develop                 # build and install the Python extension
python -m pytest tests/python

cd web && npm ci && node smoke.mjs   # the demo page, in a browser
```

The SUMO tests run SUMO. `netconvert` builds the exported network and `sumo` loads
it, so what they check is not the exporter's opinion of what it wrote. Install it
with `apt install sumo` (and `pip install sumolib` for the Python side); without it
those tests **skip**, saying so. Setting `ROADGEN_REQUIRE_SUMO=1` turns the skip into
a failure, which is what CI does.

## Licence

Apache-2.0. See [`LICENSE`](LICENSE) and [`NOTICE`](NOTICE).

Dependencies are kept to licences an Apache-2.0 project may redistribute — the
OpenDRIVE data model and writer (`opendrive`, MIT), the Lanelet2 model, OSM I/O and
projections (`simple_lanelet2`, BSD-3-Clause), and Arrow and Parquet
(`arrow`/`parquet`, Apache-2.0), plus MIT/Apache-2.0 transitive crates.

The demo page ships three more, copied in at build time and pinned by
`web/package-lock.json`: [Leaflet](https://leafletjs.com/) (BSD-2-Clause),
[osmtogeojson](https://github.com/tyrasd/osmtogeojson) (MIT) and
[CodeMirror](https://codemirror.net/5/) (MIT). Their licence texts are copied in
beside them, into `site/vendor/`. [Pyodide](https://pyodide.org/) (MPL-2.0) goes in
whole, from its own release.

The ClipGT layer names and field names were read off the public
[`clipgt_loader.py`](https://github.com/nvidia-cosmos/cosmos-transfer2.5/blob/main/cosmos_transfer2/_src/imaginaire/auxiliary/world_scenario/dataloaders/clipgt_loader.py).
That file is NVIDIA's and carries a proprietary header; none of it is reproduced here,
only the names two programs have to agree on to exchange data.

The GPUDrive scene fields were read the same way, off the simulator's own reader,
[`src/json_serialization.hpp`](https://github.com/Emerge-Lab/gpudrive/blob/main/src/json_serialization.hpp),
and its `init.hpp` and `types.hpp` beside it. Nothing from those files is reproduced
either — what is written down here is the shape of the document they accept.
Check licence compatibility before adding a dependency.
