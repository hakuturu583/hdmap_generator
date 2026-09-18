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
    "render_clipgt",
    "render_gpudrive",
    "render_opendrive",
    "render_sumo",
]
