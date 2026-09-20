"""Generate 3D road networks and write them as OpenDRIVE, Lanelet2, OSM, SUMO, ClipGT, GPUDrive or a CARLA package.

The road network itself — topology, geometry, semantics, validation — lives in
Rust. This package is the front end: every object here is a handle on something
the Rust core owns, so there is one model of the map and not two.

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
    m.export_sumo("sumo/")
    m.export_clipgt("clip/")
    m.export_gpudrive("scene.json")
    m.export_carla("Import/")                  # a CARLA UE5 asset package

A town can be generated beside the roads, and the only two things to say about
it are whether to and by what rules:

    m.generate_buildings()                     # on, a mixed town street
    m.generate_buildings(rules="downtown")     # on, one of the built-in rule sets
    m.generate_buildings(rules=text, seed=7)   # on, a CGA grammar of your own

`roadgen.building_presets()` lists the built-in sets and `roadgen.building_rules(name)`
hands one back as text, so rules of your own start as a preset with a line changed.

What comes out is solid rather than flat: a building is made of parts, each with an
outline, walls that rise from it and a roof, and `building_shell(part)` hands back the
faces that bound one. They go out with every export whose format has somewhere to put
them — OpenStreetMap ways and OpenDRIVE objects — and the others say what they
dropped.

Every export can be drawn back:

    svg = roadgen.render_opendrive("map.xodr")

And an OpenDRIVE file can be read back as a map — one that exports like any
other and cannot be added to, since it came from a file rather than from
`add_road`:

    m = roadgen.read_opendrive("map.xodr")
    for note in m.read_warnings():        # what the file said that the map cannot
        print(note)
    m.export_lanelet2("map.osm")

The `render_*` functions read the file rather than the map, so what they draw is
what a consumer would receive. They return an SVG document as text — in a notebook,
`IPython.display.SVG(svg)`; anywhere else, a string to write out or put in a page.

The CARLA export is a folder: the descriptor CARLA's importer reads, the map's `.fbx`,
the `.xodr` of the same name beside it, and a manifest naming the textures. CARLA
decides what a mesh *means* by matching its name, so the one thing worth reading before
importing one is `m.carla_warnings()`. The textures are listed rather than shipped, and
`roadgen.fetch_textures(folder)` downloads them:

    m.export_carla("Import/", name="Town01")
    for warning in m.carla_warnings(name="Town01"):
        print(warning)
    roadgen.fetch_textures("Import/Town01")
    svg = roadgen.render_carla("Import/Town01/Town01.fbx")

CARLA's importer builds the map on a level with no sky. After `Import.py` has run,
`roadgen.carla_sky(carla_root, package, name)` opens the level it made and gives it
one — a sun, an atmosphere and exposure — using the editor's own scripting.

The traffic lights and signs are built to the road — a pole on the pavement, an arm
long enough to reach over the lanes it governs — and written as props, which
`Import.py` imports but does not place. `roadgen.carla_furniture(carla_root, package,
name)` places them, and does the bookkeeping that makes CARLA adopt them as its own
lights and signs rather than spawn a second set over the middle of the road. The
script written beside the package does both.
"""

from ._roadgen import (
    Alignment,
    Junction,
    Lane,
    LaneRef,
    Map,
    Road,
    __version__,
    building_presets,
    building_rules,
    read_opendrive,
    render_carla,
    render_clipgt,
    render_gpudrive,
    render_opendrive,
    render_sumo,
)
from .furniture import CarlaFurnitureError, carla_furniture
from .sky import CarlaSkyError, carla_sky
from .textures import TextureError, fetch_textures, texture_manifest



__all__ = [
    "Alignment",
    "Junction",
    "Lane",
    "LaneRef",
    "Map",
    "Road",
    "TextureError",
    "__version__",
    "building_presets",
    "building_rules",
    "fetch_textures",
    "carla_furniture",
    "CarlaFurnitureError",
    "carla_sky",
    "CarlaSkyError",
    "render_carla",
    "render_clipgt",
    "render_gpudrive",
    "read_opendrive",
    "render_opendrive",
    "render_sumo",
    "texture_manifest",
]
