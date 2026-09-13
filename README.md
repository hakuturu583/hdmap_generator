# roadgen

Generate 3D road networks, and write the same network out as **OpenDRIVE**, as an
**Autoware-ready Lanelet2** map, as plain **OpenStreetMap**, and as a **ClipGT** clip
for NVIDIA Cosmos.

This is a generator, not a converter. Nothing here parses an existing HD map: you
describe roads, lanes, junctions and the movements between them, and the library
builds the geometry and writes the files.

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
m.export_opendrive("map.xodr")
m.export_lanelet2("map.osm")
m.export_osm("openstreetmap.osm")
m.export_clipgt("clip/")
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
              Canonical Road IR
                     │
                 Validation
                     │
    ┌────────┬────────┴─────┬──────────┐
    ▼        ▼              ▼          ▼
OpenDRIVE  Lanelet2  OpenStreetMap  ClipGT
 exporter  exporter    exporter    exporter
    │         │            │           │
`opendrive`   `simple_lanelet2`   arrow/parquet
```

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
  and no Lanelet2 `RegulatoryElement` appears in the IR.
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

let map = builder.finish()?.validate()?;
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
| `add_stop_line`, `add_traffic_light`, `add_traffic_sign`, `add_crosswalk` | road furniture |
| `add_traffic_light_rule`, `add_right_of_way`, `add_speed_limit` | rules over lanes |
| `validate()` / `issues()` / `format_warnings()` | check before exporting |
| `clipgt_warnings(scenario=None)` | what a ClipGT export would lose, and what its scenario gets wrong |
| `osm_warnings()` | what a plain OpenStreetMap export would lose |
| `mgrs_grid()` | the grid square an MGRS map is reported in |
| `export_opendrive(path)` / `export_lanelet2(path)` / `export_osm(path)` | write the files |
| `export_clipgt(directory, scenario=, clip_id=, frame_rate=, speed=, route=)` | write a ClipGT clip; returns the clip id |
| `to_opendrive_xml()` / `to_lanelet2_osm()` / `to_osm_xml()` | the same, as strings |
| `road_ids()`, `lane_ids()`, `connections()`, `successors(lane)`, `lane_centerline(lane)` | inspect the built map |

Handedness decides which side of the reference line a `forward` lane lands on:
`"rht"` (the default) puts it on the right, `"lht"` on the left. A lane can override
it with `side=`.

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

## Traffic control

Lights, signs, stop lines and crosswalks are IR objects — a position and the lanes they
govern — and both formats get them:

| IR | OpenDRIVE | Lanelet2 |
| --- | --- | --- |
| Traffic light | `<signal dynamic="true">` with `<validity>` | `traffic_light` way + `traffic_light` regulatory element |
| Traffic sign | `<signal>` carrying the caller's catalogue code | `traffic_sign` way, code as its subtype |
| Stop line | `<object type="roadMark" name="stopLine">` | `stop_line` way, the rule's `ref_line` |
| Crosswalk | `<object type="crosswalk">` with its four corners | a lanelet of subtype `crosswalk` |
| Right of way | `<junction><priority high low>` | `right_of_way` regulatory element |

OpenDRIVE places an object at `(s, t, zOffset)` in one road's own coordinates, so the
exporter projects the IR's position through the road local frame — a nearest-point
search for the station, then `Frame3::to_local`. Height is measured away from the road
surface rather than straight up, which is what "five metres above the road" means on a
slope and what `zOffset` carries.

A signal's `type` is a code from a *country's* catalogue rather than a name of its own,
so a traffic sign passes the caller's code straight through, and a traffic light is
written as the German catalogue's three-colour light (`1000001`) — the value OpenDRIVE
tooling expects in a generated map, and one a caller with another catalogue can rewrite
after export.

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

A map with MGRS coordinates has to fit inside one square. `ll2`'s MGRS projector takes
the easting and northing modulo 100 km, so a map running over the edge would silently
come back on the other side; `format_warnings()` reports it instead, and exporting
fails rather than writing it.

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

Furniture goes on the way's own nodes, which is where OSM puts it:
`highway=traffic_signals`, `highway=stop`, `highway=crossing` and `traffic_sign=<the
caller's code>`. A node is inserted at the object's own position rather than snapped
to the nearest vertex — a straight road has two of those, and everything on it would
otherwise pile up on one end. Where several controls share a point, the stronger
keeps the node's single `highway` tag: a signalised stop is signals, not a stop sign.
A crossing also gets a `highway=footway` + `footway=crossing` way across the road.

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
micrometre. There is one place they cannot: **where two roads meet at an angle**,
their lane boundaries can either meet or follow the reference lines, not both. The IR
mitres such a joint so the boundaries meet, because Lanelet2 expresses continuity
through shared points and would otherwise lose the connection; OpenDRIVE derives lane
boundaries from the reference line and a width and has no way to say the same thing.
`a_kink_between_two_roads_is_the_one_place_the_formats_differ` pins that down.

## Layout

```
roadgen/
├── crates/
│   ├── roadgen-core/        canonical IR, generation, validation
│   │   ├── topology/        connectivity
│   │   ├── geometry/        3D curves, alignments, frames, profiles, sampling
│   │   ├── semantics/       lane types, rules, markings, objects
│   │   ├── id/              typed identifiers
│   │   └── validation/      UnvalidatedMap → ValidatedMap
│   ├── roadgen-opendrive/   lowering onto the `opendrive` crate
│   ├── roadgen-lanelet2/    lowering onto `simple_lanelet2`
│   ├── roadgen-clipgt/      lowering onto ClipGT's parquet layers
│   ├── roadgen-osm/         lowering onto plain OpenStreetMap XML
│   └── roadgen-python/      PyO3 bindings
├── examples/                a ClipGT scenario file
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
```

## Licence

Apache-2.0. See [`LICENSE`](LICENSE) and [`NOTICE`](NOTICE).

Dependencies are kept to licences an Apache-2.0 project may redistribute — the
OpenDRIVE data model and writer (`opendrive`, MIT), the Lanelet2 model, OSM I/O and
projections (`simple_lanelet2`, BSD-3-Clause), and Arrow and Parquet
(`arrow`/`parquet`, Apache-2.0), plus MIT/Apache-2.0 transitive crates.

The ClipGT layer names and field names were read off the public
[`clipgt_loader.py`](https://github.com/nvidia-cosmos/cosmos-transfer2.5/blob/main/cosmos_transfer2/_src/imaginaire/auxiliary/world_scenario/dataloaders/clipgt_loader.py).
That file is NVIDIA's and carries a proprietary header; none of it is reproduced here,
only the names two programs have to agree on to exchange data.
Check licence compatibility before adding a dependency.
