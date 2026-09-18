"""Generate 3D road networks and write them as OpenDRIVE, Lanelet2, OSM, SUMO, ClipGT or GPUDrive.

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

The `render_*` functions read the file rather than the map, so what they draw is
what a consumer would receive. They return an SVG document as text — in a notebook,
`IPython.display.SVG(svg)`; anywhere else, a string to write out or put in a page.
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
    render_clipgt,
    render_gpudrive,
    render_opendrive,
    render_sumo,
)

__all__ = [
    "Alignment",
    "Junction",
    "Lane",
    "LaneRef",
    "Map",
    "Road",
    "__version__",
    "building_presets",
    "building_rules",
    "render_clipgt",
    "render_gpudrive",
    "render_opendrive",
    "render_sumo",
]
