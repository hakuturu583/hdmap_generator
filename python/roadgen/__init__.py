"""Generate 3D road networks and write them as OpenDRIVE and Autoware Lanelet2.

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
"""

from ._roadgen import Junction, Lane, LaneRef, Map, Road, __version__

__all__ = [
    "Junction",
    "Lane",
    "LaneRef",
    "Map",
    "Road",
    "__version__",
]
