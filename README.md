# roadgen

Generate 3D road networks, and write the same network out as **OpenDRIVE** and as an
**Autoware-ready Lanelet2** map.

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
          ┌──────────┴───────────┐
          ▼                      ▼
      OpenDRIVE           Autoware Lanelet2
       exporter                exporter
          │                      │
     `opendrive` crate     `simple_lanelet2`
```

Four separations are load-bearing, and each is a module of `roadgen-core`:

- **Topology** (`topology`) is connectivity and nothing else. A `LaneConnection` is
  two lane endpoints; it mentions no coordinate. Topology can be built before any
  geometry exists, and that is the order the builder works in.
- **Geometry** (`geometry`) is three-dimensional from the start. There is no 2D point
  type. Where a planar computation is genuinely needed — an OpenDRIVE `s`
  coordinate, a lane offset — it goes through `Frame3::to_local` or a method whose
  name says `horizontal`, so the projection is visible at the call site.
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
```

## The Python API

`roadgen.Map` is a handle on a Rust `MapBuilder`. `Road`, `Junction` and `LaneRef`
carry identifiers, not state: there is one model of the map and it is in Rust.

| Call | What it does |
| --- | --- |
| `Map(name, origin, projection, handedness, sampling)` | a new network; `origin` is `(lat, lon, alt)` |
| `add_road(lanes, start=, end=)` or `add_road(lanes, points=)` | a straight road, or one along a polyline |
| `add_junction(name)` | a junction to route movements through |
| `connect(a, b, junction=None)` | joins the end of `a` to the start of `b`, pairing lanes |
| `connect_lanes(from_lane, to_lane, junction=None)` | one specific movement |
| `add_stop_line`, `add_traffic_light`, `add_traffic_sign`, `add_crosswalk` | road furniture |
| `add_traffic_light_rule`, `add_right_of_way`, `add_speed_limit` | rules over lanes |
| `validate()` / `issues()` / `format_warnings()` | check before exporting |
| `export_opendrive(path)` / `export_lanelet2(path)` | write the files |
| `to_opendrive_xml()` / `to_lanelet2_osm()` | the same, as strings |
| `road_ids()`, `lane_ids()`, `connections()`, `successors(lane)`, `lane_centerline(lane)` | inspect the built map |

Handedness decides which side of the reference line a `forward` lane lands on:
`"rht"` (the default) puts it on the right, `"lht"` on the left. A lane can override
it with `side=`.

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

## Agreement between the two formats

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
│   │   ├── geometry/        3D curves, frames, sampling
│   │   ├── semantics/       lane types, rules, markings, objects
│   │   ├── id/              typed identifiers
│   │   └── validation/      UnvalidatedMap → ValidatedMap
│   ├── roadgen-opendrive/   lowering onto the `opendrive` crate
│   ├── roadgen-lanelet2/    lowering onto `simple_lanelet2`
│   └── roadgen-python/      PyO3 bindings
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

## Limitations

- `Curve3` carries `Line`, `Arc`, `Bezier` and `Polyline`. Clothoids, splines,
  superelevation and banking are not implemented; the geometry module is arranged so
  each is an added variant and an added `samples` arm, with nothing outside it to
  change.
- A road has one lane section: lane widths are constant along a road, and a change of
  cross-section is a new road.
- Map objects — traffic lights, signs, stop lines, crosswalks — are lowered to
  Lanelet2 only. OpenDRIVE `<signal>` and `<object>` elements are not written yet.
- Lanelet2 output uses a local-Cartesian or UTM projection about the map's origin.
  MGRS is not offered, because writing MGRS coordinates needs a grid the generated
  map does not have; Autoware's `local` and `local_cartesian_utm` projectors read the
  `local_x`/`local_y` tags this writes.

## Licence

Apache-2.0. See [`LICENSE`](LICENSE) and [`NOTICE`](NOTICE).

Dependencies are kept to licences an Apache-2.0 project may redistribute — the
OpenDRIVE data model and writer (`opendrive`, MIT) and the Lanelet2 model, OSM I/O and
projections (`simple_lanelet2`, BSD-3-Clause), plus MIT/Apache-2.0 transitive crates.
Check licence compatibility before adding a dependency.
