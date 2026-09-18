"""The Python front end.

These tests check the surface a user actually touches, and that it is a front end:
every result comes from the Rust core, and there is no second model of the map on
this side.
"""

import itertools
import json
import math
import os
import pathlib
import shutil
import subprocess
import textwrap
import xml.etree.ElementTree as ET

import pytest

import roadgen


def two_way():
    return [
        roadgen.Lane(width=3.5, direction="forward"),
        roadgen.Lane(width=3.5, direction="backward"),
    ]


@pytest.fixture
def joined(tmp_path):
    """The map from the design notes: two roads, a bend and a climb."""
    m = roadgen.Map()
    a = m.add_road(
        start=(0.0, 0.0, 10.0),
        end=(100.0, 0.0, 12.0),
        lanes=two_way(),
    )
    b = m.add_road(
        start=(100.0, 0.0, 12.0),
        end=(200.0, 50.0, 15.0),
        lanes=two_way(),
    )
    m.connect(a, b)
    return m


def test_the_worked_example_runs(tmp_path):
    """Exactly the script the design notes give as the completion condition."""
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
    m.export_opendrive(str(tmp_path / "test.xodr"))
    m.export_lanelet2(str(tmp_path / "test.osm"))

    assert (tmp_path / "test.xodr").stat().st_size > 0
    assert (tmp_path / "test.osm").stat().st_size > 0


def test_the_opendrive_file_is_opendrive(joined, tmp_path):
    path = tmp_path / "map.xodr"
    joined.export_opendrive(str(path))
    root = ET.parse(path).getroot()

    assert root.tag == "OpenDRIVE"
    assert root.find("header").get("revMajor") == "1"
    assert root.find("header").get("revMinor") == "7"
    roads = root.findall("road")
    assert len(roads) == 2
    # The climb is in the elevation profile, not thrown away.
    elevation = roads[0].find("elevationProfile/elevation")
    assert float(elevation.get("a")) == pytest.approx(10.0)
    assert float(elevation.get("b")) == pytest.approx(0.02)


def test_the_lanelet2_file_is_an_osm_map(joined, tmp_path):
    path = tmp_path / "map.osm"
    joined.export_lanelet2(str(path))
    root = ET.parse(path).getroot()

    assert root.tag == "osm"
    lanelets = [
        relation
        for relation in root.findall("relation")
        if any(
            tag.get("k") == "type" and tag.get("v") == "lanelet"
            for tag in relation.findall("tag")
        )
    ]
    assert len(lanelets) == 4
    for lanelet in lanelets:
        roles = {member.get("role") for member in lanelet.findall("member")}
        assert {"left", "right"} <= roles

    # Every node carries the metric position Autoware's parsers read, and its height.
    nodes = root.findall("node")
    assert nodes
    for node in nodes:
        keys = {tag.get("k") for tag in node.findall("tag")}
        assert "local_x" in keys and "local_y" in keys


def test_topology_is_readable_from_python(joined):
    connections = joined.connections()
    assert len(connections) == 2
    assert ("lane/r0/0", "lane/r1/0") in connections
    # The opposing carriageway runs the other way.
    assert ("lane/r1/1", "lane/r0/1") in connections
    assert joined.successors("lane/r0/0") == ["lane/r1/0"]


def test_roads_can_be_named_and_lanes_addressed():
    m = roadgen.Map(name="named")
    road = m.add_road(
        start=(0.0, 0.0, 0.0),
        end=(50.0, 0.0, 0.0),
        lanes=two_way(),
        name="main",
    )
    assert road.id == "road/main"
    assert road.lane_count == 2
    assert road.lane(0).id == "lane/main/0"
    assert [lane.index for lane in road.lanes()] == [0, 1]
    assert m.lane_ids() == ["lane/main/0", "lane/main/1"]


def test_a_junction_generates_connectors():
    m = roadgen.Map()
    trunk = m.add_road(
        start=(0.0, 0.0, 0.0), end=(100.0, 0.0, 0.0),
        lanes=[roadgen.Lane(width=3.5)], name="trunk",
    )
    straight_on = m.add_road(
        start=(130.0, 0.0, 0.0), end=(230.0, 0.0, 0.0),
        lanes=[roadgen.Lane(width=3.5)], name="straight_on",
    )
    slip = m.add_road(
        start=(130.0, -25.0, 0.0), end=(200.0, -80.0, 0.0),
        lanes=[roadgen.Lane(width=3.5)], name="slip",
    )
    junction = m.add_junction("fork")
    m.connect(trunk, straight_on, junction=junction)
    m.connect(trunk, slip, junction=junction)

    # Two generated connector roads on top of the three the caller added.
    assert len(m.road_ids()) == 5
    first_hop = m.successors("lane/trunk/0")
    assert len(first_hop) == 2
    reached = [lane for hop in first_hop for lane in m.successors(hop)]
    assert sorted(reached) == ["lane/slip/0", "lane/straight_on/0"]
    assert m.format_warnings() == []


def crossroads(ends):
    """Four approaches pointing at a shared centre, joined by the given ends."""
    m = roadgen.Map()
    arms = {}
    for name, start, end in (
        ("north", (0.0, 70.0, 0.0), (0.0, 14.0, 0.0)),
        ("east", (70.0, 0.0, 0.0), (14.0, 0.0, 0.0)),
        ("south", (0.0, -70.0, 0.0), (0.0, -14.0, 0.0)),
        ("west", (-70.0, 0.0, 0.0), (-14.0, 0.0, 0.0)),
    ):
        arms[name] = m.add_road(start=start, end=end, lanes=two_way(), name=name)
    junction = m.add_junction("x")
    for a, b in itertools.combinations(arms, 2):
        # `ends=None` means "don't pass it", so the default can be tested too.
        if ends is None:
            m.connect(arms[a], arms[b], junction=junction)
        else:
            m.connect(arms[a], arms[b], junction=junction, ends=ends)
    return m, arms


def connector_reach(m, arms):
    """How far the generated connectors stray from the junction's centre."""
    connectors = [
        lane
        for lane in m.lane_ids()
        if not any(lane.startswith(f"lane/{name}/") for name in arms)
    ]
    assert len(connectors) == 12, "one connector per movement"
    return max(
        max(max(abs(x), abs(y)) for x, y, _ in m.lane_centerline(lane))
        for lane in connectors
    )


def test_approaches_that_meet_end_to_end_can_say_so():
    # Every arm points at the centre, so every arm meets the joint at its end.
    m, arms = crossroads(("end", "end"))
    assert len(m.road_ids()) == 16, "four arms and twelve connectors"
    # The approaches stop 14 m out, so the connectors belong inside that circle.
    assert connector_reach(m, arms) <= 14.0 + 1e-6
    assert m.format_warnings() == []


def test_the_default_still_joins_one_road_into_the_next():
    # `connect` without `ends` is unchanged: the end of one road, the start of the
    # next. On approaches that all point inwards that is the wrong pairing, and it
    # shows up as connectors running out to the arms' far tips rather than as an
    # error — which is exactly why `ends` has to be sayable.
    m, arms = crossroads(("end", "start"))
    assert connector_reach(m, arms) > 70.0

    default, _ = crossroads(None)
    assert default.connections() == m.connections()


def test_an_unknown_road_end_is_refused():
    m = roadgen.Map()
    a = m.add_road(
        start=(0.0, 0.0, 0.0), end=(100.0, 0.0, 0.0), lanes=two_way(), name="a"
    )
    b = m.add_road(
        start=(100.0, 0.0, 0.0), end=(200.0, 0.0, 0.0), lanes=two_way(), name="b"
    )
    with pytest.raises(ValueError, match="'start' or 'end'"):
        m.connect(a, b, ends=("end", "middle"))


def test_a_polyline_road_keeps_its_gradient():
    m = roadgen.Map()
    m.add_road(
        points=[(0.0, 0.0, 0.0), (100.0, 0.0, 6.0), (200.0, 0.0, 6.0)],
        lanes=[roadgen.Lane(width=3.5)],
        name="hill",
    )
    heights = [point[2] for point in m.lane_centerline("lane/hill/0")]
    assert heights[0] == pytest.approx(0.0)
    assert heights[-1] == pytest.approx(6.0)


def test_left_hand_traffic_puts_the_forward_lane_on_the_left():
    left = roadgen.Map(handedness="lht")
    left.add_road(
        start=(0.0, 0.0, 0.0), end=(100.0, 0.0, 0.0),
        lanes=two_way(), name="main",
    )
    right = roadgen.Map(handedness="rht")
    right.add_road(
        start=(0.0, 0.0, 0.0), end=(100.0, 0.0, 0.0),
        lanes=two_way(), name="main",
    )
    # Same input, mirrored cross-section.
    y_left = left.lane_centerline("lane/main/0")[0][1]
    y_right = right.lane_centerline("lane/main/0")[0][1]
    assert y_left == pytest.approx(1.75)
    assert y_right == pytest.approx(-1.75)


def test_regenerating_gives_byte_identical_files():
    def build():
        m = roadgen.Map(name="stable")
        a = m.add_road(
            start=(0.0, 0.0, 0.0), end=(100.0, 0.0, 0.0),
            lanes=two_way(), name="a",
        )
        b = m.add_road(
            start=(100.0, 0.0, 0.0), end=(200.0, 0.0, 0.0),
            lanes=two_way(), name="b",
        )
        m.connect(a, b)
        return m

    first, second = build(), build()
    assert first.lane_ids() == second.lane_ids()
    assert first.to_opendrive_xml() == second.to_opendrive_xml()
    assert first.to_lanelet2_osm() == second.to_lanelet2_osm()


def test_traffic_control_can_be_added():
    m = roadgen.Map()
    north = m.add_road(
        start=(0.0, 70.0, 0.0), end=(0.0, 14.0, 0.0), lanes=two_way(), name="north",
    )
    east = m.add_road(
        start=(70.0, 0.0, 0.0), end=(14.0, 0.0, 0.0), lanes=two_way(), name="east",
    )
    junction = m.add_junction("x")
    m.connect_lanes(north.lane(0), east.lane(1), junction=junction)

    stop_line = m.add_stop_line(north.lane(0))
    light = m.add_traffic_light(north.lane(0), height=5.0)
    m.add_traffic_light_rule([light], [north.lane(0)], stop_line=stop_line)
    m.add_right_of_way([east.lane(0)], [north.lane(0)], stop_line=stop_line)
    m.add_crosswalk(north, fraction=0.8, width=4.0)
    # A Lanelet2 traffic sign says which sign it is in its subtype.
    m.add_traffic_sign(north.lane(0), code="de205", height=2.5)

    osm = m.to_lanelet2_osm()
    assert 'v="traffic_light"' in osm
    assert 'v="right_of_way"' in osm
    assert 'v="stop_line"' in osm
    assert 'v="crosswalk"' in osm
    assert 'v="traffic_sign"' in osm
    assert 'v="de205"' in osm


def test_a_bad_lane_width_is_refused_at_the_boundary():
    with pytest.raises(ValueError, match="greater than zero"):
        roadgen.Lane(width=0.0)
    with pytest.raises(ValueError, match="greater than zero"):
        roadgen.Lane(width=-3.5)


def test_a_bad_direction_is_refused():
    with pytest.raises(ValueError, match="forward"):
        roadgen.Lane(width=3.5, direction="sideways")


def test_roads_that_do_not_meet_are_reported_not_exported(tmp_path):
    m = roadgen.Map()
    a = m.add_road(
        start=(0.0, 0.0, 0.0), end=(100.0, 0.0, 0.0), lanes=two_way(), name="a",
    )
    b = m.add_road(
        start=(140.0, 0.0, 0.0), end=(200.0, 0.0, 0.0), lanes=two_way(), name="b",
    )
    m.connect(a, b)

    issues = m.issues()
    assert any("do not meet" in issue for issue in issues)
    with pytest.raises(ValueError):
        m.export_opendrive(str(tmp_path / "broken.xodr"))


def test_a_split_without_a_junction_is_refused():
    m = roadgen.Map()
    a = m.add_road(
        start=(0.0, 0.0, 0.0), end=(100.0, 0.0, 0.0), lanes=two_way(), name="a",
    )
    b = m.add_road(
        start=(100.0, 0.0, 0.0), end=(200.0, 0.0, 0.0), lanes=two_way(), name="b",
    )
    c = m.add_road(
        start=(100.0, 0.0, 0.0), end=(200.0, 80.0, 0.0), lanes=two_way(), name="c",
    )
    m.connect(a, b)
    with pytest.raises(ValueError, match="junction"):
        m.connect(a, c)


def test_an_alignment_chains_lines_bends_and_transitions():
    m = roadgen.Map()
    radius = 120.0
    al = roadgen.Alignment(start=(0.0, 0.0, 4.0), heading=0.0)
    al.line(80.0, rise=1.0)
    al.spiral(60.0, curvature_end=1 / radius, rise=1.0)
    al.arc(140.0, curvature=1 / radius, rise=2.0)
    al.spiral(60.0, curvature_end=0.0, rise=1.0)
    al.line(80.0, rise=1.0)

    # The alignment tracks where it has got to, so the caller never restates it.
    assert al.curvature == pytest.approx(0.0)
    # Each transition turns by half its length times the bend's curvature, and the
    # bend by all of its: (60/2 + 140 + 60/2) / radius.
    assert al.heading == pytest.approx(200.0 / radius, abs=1e-6)
    assert al.point[2] == pytest.approx(10.0)

    m.add_road(lanes=two_way(), alignment=al, name="sweep")
    assert m.format_warnings() == []

    # The transitions reach OpenDRIVE as spirals, not as chains of line segments.
    root = ET.fromstring(m.to_opendrive_xml())
    kinds = [
        child.tag
        for geometry in root.findall("road/planView/geometry")
        for child in geometry
    ]
    assert kinds == ["line", "spiral", "arc", "spiral", "line"]

    # And the lane centreline really follows the bend.
    centre = m.lane_centerline("lane/sweep/0")
    assert centre[0][2] == pytest.approx(4.0, abs=1e-6)
    assert centre[-1][1] > 20.0


def test_a_banked_road_lifts_one_edge():
    m = roadgen.Map()
    al = roadgen.Alignment(start=(0.0, 0.0, 0.0), heading=0.0)
    al.line(50.0)
    al.arc(120.0, curvature=1 / 150)
    m.add_road(
        lanes=two_way(),
        alignment=al,
        name="bend",
        # Flat to 50 m, rolled to -0.06 rad by 100 m, held from there.
        superelevation=[(0.0, 0.0), (50.0, 0.0), (100.0, -0.06), (170.0, -0.06)],
    )

    root = ET.fromstring(m.to_opendrive_xml())
    rolls = root.findall("road/lateralProfile/superelevation")
    assert len(rolls) == 4
    assert float(rolls[2].get("a")) == pytest.approx(-0.06)

    # The two carriageways are no longer at the same height through the bend: the
    # right-hand one rides higher on the banked surface.
    forward_end = m.lane_centerline("lane/bend/0")[-1]
    backward_start = m.lane_centerline("lane/bend/1")[0]
    assert forward_end[2] > backward_start[2]


def test_a_road_cannot_take_two_geometries_at_once():
    m = roadgen.Map()
    al = roadgen.Alignment(start=(0.0, 0.0, 0.0))
    al.line(50.0)
    with pytest.raises(ValueError, match="exactly one"):
        m.add_road(lanes=two_way(), start=(0.0, 0.0, 0.0), end=(1.0, 0.0, 0.0), alignment=al)


def test_an_absurd_bank_is_refused():
    m = roadgen.Map()
    m.add_road(
        lanes=two_way(),
        start=(0.0, 0.0, 0.0),
        end=(100.0, 0.0, 0.0),
        name="wall",
        superelevation=[(0.0, 1.05)],
    )
    assert any("banked" in issue for issue in m.issues())
    with pytest.raises(ValueError):
        m.validate()


def test_a_lane_can_taper_without_becoming_two_lanes():
    m = roadgen.Map()
    m.add_road(
        start=(0.0, 0.0, 0.0),
        end=(260.0, 0.0, 0.0),
        name="layby",
        lanes=[
            roadgen.Lane(width=3.5),
            roadgen.Lane(
                width=2.0,
                type_="shoulder",
                width_profile=[(0.0, 2.0), (60.0, 2.0), (100.0, 5.0), (200.0, 2.0)],
            ),
        ],
    )
    assert m.format_warnings() == []

    # Still two lanes, and one lane section: a taper is a width, not a new section.
    assert m.lane_ids() == ["lane/layby/0", "lane/layby/1"]
    root = ET.fromstring(m.to_opendrive_xml())
    assert len(root.findall("road/lanes/laneSection")) == 1

    # The shoulder's centreline swings out as it widens and comes back.
    centre = m.lane_centerline("lane/layby/1")
    offsets = [abs(point[1]) for point in centre]
    assert max(offsets) == pytest.approx(6.0, abs=1e-6)
    assert offsets[0] == pytest.approx(4.5, abs=1e-6)
    assert offsets[-1] == pytest.approx(4.5, abs=1e-6)


def test_a_lane_that_ends_needs_a_new_cross_section():
    m = roadgen.Map()
    lanes = lambda n: [roadgen.Lane(width=3.5) for _ in range(n)]
    road = m.add_road(
        start=(0.0, 0.0, 0.0),
        end=(320.0, 0.0, 0.0),
        name="wide",
        lanes=lanes(3),
        cross_sections=[(200.0, lanes(2))],
    )
    assert road.lane_count == 5
    assert m.format_warnings() == []

    # Two lane sections in the document, at the stations asked for.
    root = ET.fromstring(m.to_opendrive_xml())
    sections = root.findall("road/lanes/laneSection")
    assert [float(section.get("s")) for section in sections] == [
        pytest.approx(0.0),
        pytest.approx(200.0),
    ]
    assert len(sections[0].findall("right/lane")) == 3
    assert len(sections[1].findall("right/lane")) == 2

    # The lanes that carry on are connected; the one that ends is not.
    assert m.successors("lane/wide/0") == ["lane/wide/3"]
    assert m.successors("lane/wide/1") == ["lane/wide/4"]
    assert m.successors("lane/wide/2") == []


def test_a_width_of_zero_is_refused_wherever_it_is_written():
    with pytest.raises(ValueError, match="greater than zero"):
        roadgen.Lane(width=3.5, width_profile=[(0.0, 3.5), (100.0, 0.0)])
    with pytest.raises(ValueError, match="taper"):
        roadgen.Lane(width=3.5, width_profile=[(0.0, 3.5)], taper="wobbly")


def test_traffic_control_reaches_opendrive_too():
    m = roadgen.Map()
    north = m.add_road(
        start=(0.0, 70.0, 0.0), end=(0.0, 14.0, 0.0), lanes=two_way(), name="north",
    )
    east = m.add_road(
        start=(70.0, 0.0, 0.0), end=(14.0, 0.0, 0.0), lanes=two_way(), name="east",
    )
    junction = m.add_junction("x")
    m.connect_lanes(north.lane(0), east.lane(1), junction=junction)
    m.connect_lanes(east.lane(0), north.lane(1), junction=junction)

    stop_line = m.add_stop_line(north.lane(0))
    light = m.add_traffic_light(north.lane(0), height=5.0)
    m.add_traffic_light_rule([light], [north.lane(0)], stop_line=stop_line)
    m.add_right_of_way([east.lane(0)], [north.lane(0)], stop_line=stop_line)
    m.add_crosswalk(north, fraction=0.6, width=4.0)
    m.add_traffic_sign(north.lane(0), code="de206")

    root = ET.fromstring(m.to_opendrive_xml())

    # The light and the sign are signals on the road they govern, and the light is
    # five metres above the surface, over the middle of its lane.
    signals = root.findall("road/signals/signal")
    assert len(signals) == 2
    by_type = {signal.get("type"): signal for signal in signals}
    assert "de206" in by_type
    light_element = by_type["1000001"]
    # OpenDRIVE spells this one "yes"/"no", not "true"/"false".
    assert light_element.get("dynamic") == "yes"
    assert float(light_element.get("zOffset")) == pytest.approx(5.0, abs=1e-6)
    assert float(light_element.get("t")) == pytest.approx(-1.75, abs=1e-6)
    validity = light_element.find("validity")
    assert validity.get("fromLane") == "-1" and validity.get("toLane") == "-1"

    # The stop line and the crosswalk are objects.
    objects = root.findall("road/objects/object")
    kinds = {obj.get("type") for obj in objects}
    assert kinds == {"roadMark", "crosswalk"}
    crosswalk = next(obj for obj in objects if obj.get("type") == "crosswalk")
    assert len(crosswalk.findall("outline/cornerRoad")) == 4

    # And the right-of-way rule becomes a junction priority.
    assert root.findall("junction/priority")


def test_an_mgrs_map_reports_grid_coordinates():
    m = roadgen.Map(origin=(35.68, 139.76, 0.0), projection="mgrs")
    m.add_road(
        start=(0.0, 0.0, 0.0), end=(200.0, 0.0, 2.0), lanes=two_way(), name="main",
    )
    assert m.format_warnings() == []

    # The square's reference, for Autoware's map_projector_info.
    grid = m.mgrs_grid()
    assert grid is not None and grid.startswith("54S")

    # The map is built about (0, 0), but its nodes say where they are in the square.
    root = ET.fromstring(m.to_lanelet2_osm())
    for node in root.findall("node"):
        local = {tag.get("k"): float(tag.get("v")) for tag in node.findall("tag")}
        assert 0.0 <= local["local_x"] < 100_000.0
        assert 0.0 <= local["local_y"] < 100_000.0
        assert local["local_x"] > 1000.0

    # A map that does not use MGRS has no grid, and reports its own metres.
    plain = roadgen.Map(origin=(35.68, 139.76, 0.0))
    plain.add_road(
        start=(0.0, 0.0, 0.0), end=(200.0, 0.0, 2.0), lanes=two_way(), name="main",
    )
    assert plain.mgrs_grid() is None
    root = ET.fromstring(plain.to_lanelet2_osm())
    values = [
        float(tag.get("v"))
        for node in root.findall("node")
        for tag in node.findall("tag")
        if tag.get("k") == "local_x"
    ]
    assert max(values) < 1000.0


def test_a_map_that_leaves_its_mgrs_square_is_reported():
    m = roadgen.Map(origin=(35.68, 139.76, 0.0), projection="mgrs")
    m.add_road(
        start=(0.0, 0.0, 0.0), end=(150_000.0, 0.0, 0.0), lanes=two_way(), name="long",
    )
    assert any("MGRS square" in warning for warning in m.format_warnings())
    with pytest.raises(RuntimeError, match="MGRS square"):
        m.to_lanelet2_osm()


# --------------------------------------------------------------------------- #
# ClipGT
#
# These read the written directory the way ClipGTLoader reads one — the same file
# names, the same columns, the same keys reached through — so that a test failing
# here means a Cosmos reader would have failed too.
# --------------------------------------------------------------------------- #


def clipgt_map():
    """A graded, signalised crossroads: something with every layer in it."""
    m = roadgen.Map(name="demo town")
    arms = {}
    for name, start, end in (
        ("north", (0.0, 70.0, 4.0), (0.0, 14.0, 1.0)),
        ("east", (70.0, 0.0, 0.0), (14.0, 0.0, 1.0)),
        ("south", (0.0, -70.0, 0.0), (0.0, -14.0, 1.0)),
        ("west", (-70.0, 0.0, 2.0), (-14.0, 0.0, 1.0)),
    ):
        arms[name] = m.add_road(start=start, end=end, lanes=two_way(), name=name)
    junction = m.add_junction("x")
    for a, b in itertools.combinations(arms, 2):
        m.connect(arms[a], arms[b], junction=junction, ends=("end", "end"))

    approach = arms["north"].lane(0)
    stop_line = m.add_stop_line(approach)
    light = m.add_traffic_light(approach, height=5.0)
    m.add_traffic_light_rule([light], [approach], stop_line=stop_line)
    m.add_traffic_sign(approach, code="STOP", height=2.4)
    m.add_crosswalk(arms["north"], fraction=0.82, width=4.0)
    return m


def read_layer(directory, clip_id, layer):
    pd = pytest.importorskip("pandas")
    pytest.importorskip("pyarrow")
    return pd.read_parquet(directory / f"{clip_id}.{layer}.parquet")


def test_a_clipgt_directory_is_one_a_reader_would_accept(tmp_path):
    pytest.importorskip("pyarrow")
    clip_id = clipgt_map().export_clipgt(tmp_path)
    # The name comes from the map's, reduced to something a file name can hold.
    assert clip_id == "demo_town"

    # `can_load` looks for exactly these two before it looks at anything else.
    for required in ("calibration_estimate", "egomotion_estimate"):
        assert (tmp_path / f"{clip_id}.{required}.parquet").is_file()

    # And the calibration's first row parses as a rig, which is how it is read.
    calibration = read_layer(tmp_path, clip_id, "calibration_estimate")
    rig = json.loads(str(calibration.iloc[0]["calibration_estimate"]["rig_json"]))
    assert rig["rig"]["sensors"] == [], "the IR knows nothing about cameras"


def test_clipgt_lanes_are_read_as_two_rails_with_heights(tmp_path):
    pytest.importorskip("pyarrow")
    m = clipgt_map()
    clip_id = m.export_clipgt(tmp_path)
    lanes = read_layer(tmp_path, clip_id, "lane")

    heights = []
    for _, row in lanes.iterrows():
        lane = row["lane"]
        for side in ("left_rail", "right_rail"):
            assert side in lane and lane[side] is not None
            points = [(pt["x"], pt["y"], pt["z"]) for pt in lane[side]]
            assert len(points) >= 2
            heights.extend(z for _, _, z in points)

    # The arms run downhill into the junction from 4 m, 2 m and 0 m, so a rail that
    # came out flat would mean the third dimension had been dropped somewhere.
    assert max(heights) - min(heights) > 3.0


def test_a_clipgt_ego_track_drives_the_map(tmp_path):
    pytest.importorskip("pyarrow")
    clip_id = clipgt_map().export_clipgt(tmp_path, speed=15.0, frame_rate=10.0)
    ego = read_layer(tmp_path, clip_id, "egomotion_estimate")

    stamps, positions = [], []
    for _, row in ego.iterrows():
        motion, key = row["egomotion_estimate"], row["key"]
        location, orientation = motion["location"], motion["orientation"]
        positions.append((location["x"], location["y"], location["z"]))
        stamps.append(key["timestamp_micros"])
        norm = sum(orientation[axis] ** 2 for axis in ("x", "y", "z", "w")) ** 0.5
        assert abs(norm - 1.0) < 1e-9, "orientations are unit quaternions"

    assert len(stamps) > 2
    assert all(b - a == 100_000 for a, b in zip(stamps, stamps[1:]))
    # It goes somewhere, and it goes downhill: the route starts on a graded arm.
    start, finish = positions[0], positions[-1]
    assert (start[0] - finish[0]) ** 2 + (start[1] - finish[1]) ** 2 > 100.0
    assert abs(start[2] - finish[2]) > 0.5


def test_clipgt_traffic_control_reaches_its_own_layers(tmp_path):
    pytest.importorskip("pyarrow")
    clip_id = clipgt_map().export_clipgt(tmp_path)

    lights = read_layer(tmp_path, clip_id, "traffic_light")
    assert len(lights) == 1
    centre = lights.iloc[0]["traffic_light"]["center"]
    assert centre["z"] > 4.9, "a light hangs above the road"

    signs = read_layer(tmp_path, clip_id, "traffic_sign")
    assert signs.iloc[0]["traffic_sign"]["category"] == "STOP"

    assert len(read_layer(tmp_path, clip_id, "wait_line")) == 1
    crossing = read_layer(tmp_path, clip_id, "crosswalk").iloc[0]["crosswalk"]
    assert len(crossing["location"]) >= 4

    area = read_layer(tmp_path, clip_id, "intersection_area").iloc[0]
    assert len(area["intersection_area"]["location"]) >= 3


def test_clipgt_says_what_it_cannot_carry(tmp_path):
    m = clipgt_map()
    warnings = m.clipgt_warnings()
    assert any("topology" in warning for warning in warnings)
    # And it stays out of the general warnings, which are about the other two
    # formats and would otherwise be noise for a caller who never writes a clip.
    assert m.format_warnings() == []


def test_a_clip_id_that_would_escape_the_directory_is_refused(tmp_path):
    with pytest.raises(RuntimeError, match="clip id"):
        clipgt_map().export_clipgt(tmp_path, clip_id="../escape")


def write_scenario(tmp_path, text):
    path = tmp_path / "scenario.yaml"
    path.write_text(textwrap.dedent(text))
    return path


def test_a_scenario_file_sets_the_route_and_the_rig(tmp_path):
    pytest.importorskip("pyarrow")
    m = clipgt_map()
    scenario = write_scenario(
        tmp_path,
        """\
        clip_id: from_yaml
        frame_rate: 10.0
        speed: 9.0

        route:
          start: lane/east/0

        sensors:
          - name: camera:front_wide_120fov
            position: [1.7, 0.0, 1.45]
            width: 1920
            height: 1080
            fov_degrees: 120.0
        """,
    )
    out = tmp_path / "clip"
    assert m.export_clipgt(out, scenario=scenario) == "from_yaml"

    # The rig reached the calibration table.
    calibration = read_layer(out, "from_yaml", "calibration_estimate")
    rig = json.loads(str(calibration.iloc[0]["calibration_estimate"]["rig_json"]))
    sensors = rig["rig"]["sensors"]
    assert len(sensors) == 1
    assert sensors[0]["name"] == "camera:front_wide_120fov"
    assert sensors[0]["nominalSensor2Rig_FLU"]["t"] == [1.7, 0.0, 1.45]
    assert sensors[0]["properties"]["width"] == "1920"

    # The route is the one the file named: the east arm points at the junction from
    # (70, 0), which is nowhere near where a default route would have started.
    ego = read_layer(out, "from_yaml", "egomotion_estimate")
    first = ego.iloc[0]["egomotion_estimate"]["location"]
    assert abs(first["x"] - 70.0) < 1.0 and abs(first["y"] - 1.75) < 1.0

    # At 10 Hz, as asked.
    stamps = [row["key"]["timestamp_micros"] for _, row in ego.iterrows()]
    assert all(b - a == 100_000 for a, b in zip(stamps, stamps[1:]))

    # And the camera got a frame per pose.
    frames = json.loads((out / "from_yaml.camera_front_wide_120fov.json").read_text())
    assert [frame["timestamp"] for frame in frames] == stamps


def test_arguments_override_the_scenario_file(tmp_path):
    pytest.importorskip("pyarrow")
    m = clipgt_map()
    scenario = write_scenario(tmp_path, "clip_id: from_yaml\nspeed: 9.0\n")
    out = tmp_path / "clip"
    # What is passed wins; what is not keeps the file's value.
    assert m.export_clipgt(out, scenario=scenario, clip_id="override") == "override"
    assert (out / "override.egomotion_estimate.parquet").is_file()


def test_a_scenario_with_a_mistyped_key_is_refused(tmp_path):
    scenario = write_scenario(tmp_path, "speed_kph: 40.0\n")
    with pytest.raises(ValueError, match="speed_kph"):
        clipgt_map().export_clipgt(tmp_path / "clip", scenario=scenario)


def test_a_scenario_route_is_checked_against_the_map(tmp_path):
    m = clipgt_map()
    scenario = write_scenario(tmp_path, "route:\n  start: lane/nowhere/0\n")
    warnings = m.clipgt_warnings(scenario=scenario)
    assert any("lane/nowhere/0" in warning for warning in warnings)
    with pytest.raises(RuntimeError, match="lane/nowhere/0"):
        m.export_clipgt(tmp_path / "clip", scenario=scenario)


# --------------------------------------------------------------------------- #
# Plain OpenStreetMap
# --------------------------------------------------------------------------- #


def test_the_osm_export_is_osm_not_lanelet2(tmp_path):
    m = clipgt_map()
    path = tmp_path / "town.osm"
    m.export_osm(path)
    root = ET.fromstring(path.read_text())

    assert root.get("generator") == "roadgen"
    ways = {
        way.get("id"): {tag.get("k"): tag.get("v") for tag in way.findall("tag")}
        for way in root.findall("way")
    }
    # One `highway` way per arm, which is the thing a router looks for and the thing
    # the Lanelet2 file does not have.
    roads = [tags for tags in ways.values() if "highway" in tags and tags["highway"] != "footway"]
    assert len(roads) == 4
    for tags in roads:
        assert tags["highway"] == "residential"
        assert tags["lanes"] == "2"
        assert tags["oneway"] == "no"

    # The Lanelet2 file of the same map has no `highway` tag anywhere.
    assert "highway" not in m.to_lanelet2_osm()


def test_the_osm_arms_meet_at_one_node():
    # Its own map, with a real origin, so that the junction node's latitude and
    # longitude are worth asserting rather than being the zero the default gives.
    m = roadgen.Map(name="tokyo", origin=(35.68, 139.76, 0.0))
    arms = {}
    for name, start, end in (
        ("north", (0.0, 70.0, 0.0), (0.0, 14.0, 0.0)),
        ("east", (70.0, 0.0, 0.0), (14.0, 0.0, 0.0)),
        ("south", (0.0, -70.0, 0.0), (0.0, -14.0, 0.0)),
        ("west", (-70.0, 0.0, 0.0), (-14.0, 0.0, 0.0)),
    ):
        arms[name] = m.add_road(start=start, end=end, lanes=two_way(), name=name)
    junction = m.add_junction("x")
    for a, b in itertools.combinations(arms, 2):
        m.connect(arms[a], arms[b], junction=junction, ends=("end", "end"))
    root = ET.fromstring(m.to_osm_xml())

    arms = [
        [nd.get("ref") for nd in way.findall("nd")]
        for way in root.findall("way")
        if any(tag.get("k") == "name" for tag in way.findall("tag"))
    ]
    assert len(arms) == 4
    shared = set(arms[0]).intersection(*(set(arm) for arm in arms[1:]))
    assert len(shared) == 1, "every arm reaches the junction's node"

    # Which is at the crossing, not at the average of the arms' ends.
    junction = shared.pop()
    node = next(n for n in root.findall("node") if n.get("id") == junction)
    assert abs(float(node.get("lat")) - 35.68) < 1e-7
    assert abs(float(node.get("lon")) - 139.76) < 1e-7


def test_osm_says_what_it_cannot_carry():
    m = clipgt_map()
    warnings = m.osm_warnings()
    assert any("lane geometry" in warning for warning in warnings)
    assert any("ele" in warning for warning in warnings), "the arms are at different heights"
    # And it stays out of the general warnings, which are about the two XML formats.
    assert m.format_warnings() == []


# --------------------------------------------------------------------------- #
# SUMO
# --------------------------------------------------------------------------- #


def sumo_tools():
    """netconvert, and SUMO's own Python library for reading what it builds.

    A test that says "SUMO can read this" has to be SUMO reading it. When the tools
    are not installed the test skips, unless ``ROADGEN_REQUIRE_SUMO`` is set — which
    CI does set, so the check never quietly stops running.
    """
    netconvert = shutil.which("netconvert")
    if netconvert is None and os.environ.get("SUMO_HOME"):
        candidate = pathlib.Path(os.environ["SUMO_HOME"]) / "bin" / "netconvert"
        netconvert = str(candidate) if candidate.is_file() else None
    try:
        import sumolib
    except ImportError:
        sumolib = None

    if netconvert is None or sumolib is None:
        missing = ", ".join(
            name
            for name, found in (("netconvert", netconvert), ("sumolib", sumolib))
            if found is None
        )
        if os.environ.get("ROADGEN_REQUIRE_SUMO"):
            raise AssertionError(f"ROADGEN_REQUIRE_SUMO is set but {missing} is not installed")
        pytest.skip(f"{missing} not installed")
    return netconvert, sumolib


def build_with_netconvert(m, directory):
    """Exports, builds with netconvert, and hands back the network SUMO read."""
    netconvert, sumolib = sumo_tools()
    prefix = m.export_sumo(str(directory))
    result = subprocess.run(
        [netconvert, "-c", f"{prefix}.netccfg"],
        cwd=directory,
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0, f"netconvert rejected the export:\n{result.stderr}"
    built = directory / f"{prefix}.net.xml"
    assert built.is_file()
    # sumolib is SUMO's own reader: if it accepts the file, SUMO accepts the file.
    return prefix, sumolib.net.readNet(str(built), withInternal=True)


def test_a_sumo_export_is_a_network_netconvert_builds(tmp_path):
    m = clipgt_map()
    prefix, net = build_with_netconvert(m, tmp_path)
    assert prefix == "demo_town"

    # Four arms, each carrying traffic both ways, and nothing else: the connectors
    # are movements across the junction rather than edges of their own.
    roads = sorted(edge.getID() for edge in net.getEdges() if edge.getFunction() != "internal")
    assert roads == [
        "east.bwd",
        "east.fwd",
        "north.bwd",
        "north.fwd",
        "south.bwd",
        "south.fwd",
        "west.bwd",
        "west.fwd",
    ]

    # The junction is where the arms would cross, and it is signalised because the
    # map puts a traffic light on the approach to it.
    junction = net.getNode("j_x")
    assert junction.getType() == "traffic_light"
    assert abs(junction.getCoord()[0]) < 0.05 and abs(junction.getCoord()[1]) < 0.05

    # And SUMO found a route through it, which is the thing a network is for.
    north = net.getEdge("north.fwd")
    reachable = {edge.getID() for edge in north.getOutgoing()}
    assert {"east.bwd", "south.bwd", "west.bwd"} <= reachable


def test_a_sumo_lane_is_where_the_export_says_it_is(tmp_path):
    m = clipgt_map()
    _prefix, net = build_with_netconvert(m, tmp_path)

    written = dict(m.sumo_lane_ids())
    assert written, "every drivable lane should reach the network"
    for lane_id, sumo_id in written.items():
        lane = net.getLane(sumo_id)
        shape = [(x, y) for x, y, *_ in lane.getShape3D()]
        centerline = [(x, y) for x, y, *_ in m.lane_centerline(lane_id)]
        # SUMO orders a lane the way traffic drives it, which for a lane running
        # against its road's reference line is the other way round.
        assert len(shape) == len(centerline)
        if shape[0] != pytest.approx(centerline[0], abs=0.02):
            centerline.reverse()
        for found, wanted in zip(shape, centerline):
            assert found == pytest.approx(wanted, abs=0.02)


def test_sumo_says_what_it_cannot_carry():
    warnings = clipgt_map().sumo_warnings()
    assert any("markings" in warning for warning in warnings)
    assert any("netconvert generates the phases" in warning for warning in warnings)
    # And, as with the OpenStreetMap export, it stays out of the general warnings.
    assert clipgt_map().format_warnings() == []


# --------------------------------------------------------------------------- #
# GPUDrive
#
# A GPUDrive scene is JSON with no schema: what makes a file a scene is that the
# simulator's reader accepts it. So these read the written file the way that reader
# does — the keys it insists on, the arrays it steps through in parallel, the type
# strings it compares against — rather than asserting against the exporter's opinion
# of what it wrote. The map is the same signalised crossroads the ClipGT tests use.
# --------------------------------------------------------------------------- #


def export_scene(m, tmp_path, **arguments):
    path = tmp_path / "scene.json"
    m.export_gpudrive(path, **arguments)
    return json.loads(path.read_text())


def test_a_gpudrive_scene_holds_every_key_the_reader_insists_on(tmp_path):
    scene = export_scene(clipgt_map(), tmp_path)

    assert set(scene) >= {"name", "scenario_id", "objects", "roads", "metadata"}
    assert scene["name"] == "demo town"
    assert set(scene["metadata"]) >= {
        "sdc_track_index",
        "tracks_to_predict",
        "objects_of_interest",
    }
    assert scene["metadata"]["sdc_track_index"] == 0

    for obj in scene["objects"]:
        assert set(obj) >= {
            "position",
            "width",
            "length",
            "height",
            "id",
            "heading",
            "velocity",
            "valid",
            "goalPosition",
            "type",
        }
        assert obj["type"] in ("vehicle", "pedestrian", "cyclist")
        steps = len(obj["position"])
        assert steps == len(obj["heading"]) == len(obj["velocity"]) == len(obj["valid"])

    kinds = {road["type"] for road in scene["roads"]}
    assert kinds <= {
        "lane",
        "road_line",
        "road_edge",
        "crosswalk",
        "speed_bump",
        "stop_sign",
    }
    assert {"lane", "road_edge", "crosswalk", "stop_sign"} <= kinds
    for road in scene["roads"]:
        assert road["geometry"], "an element with no points is one the reader misreads"
        # The Waymo map-feature codes, gaps and all: 4 is not one of them.
        assert road["map_element_id"] in set(range(0, 21)) - {4} | {-1}
        assert all(set(point) == {"x", "y"} for point in road["geometry"])


def test_a_gpudrive_agent_drives_the_map(tmp_path):
    scene = export_scene(
        clipgt_map(), tmp_path, steps=30, time_step=0.1, speed=15.0, name="driven"
    )
    assert scene["name"] == "driven"

    agent = scene["objects"][0]
    assert len(agent["position"]) == 30
    assert all(agent["valid"])

    # 15 m/s at a tenth of a second is 1.5 m a step, measured on the ground: the arms
    # of this map are graded, and a track paced along the slope would fall short.
    steps = [
        ((b["x"] - a["x"]) ** 2 + (b["y"] - a["y"]) ** 2) ** 0.5
        for a, b in zip(agent["position"], agent["position"][1:])
    ]
    assert all(step == pytest.approx(1.5, abs=1e-2) for step in steps)
    assert sum(steps) == pytest.approx(29 * 1.5, abs=1e-2)

    # Thirty steps is 43.5 m of a route that is longer than that, so the goal — where
    # the route ends — is somewhere the agent has not reached yet.
    last = agent["position"][-1]
    assert set(agent["goalPosition"]) == {"x", "y"}
    assert agent["goalPosition"] != pytest.approx(last, abs=1.0)


def test_a_gpudrive_scenario_file_says_who_drives_and_where(tmp_path):
    m = clipgt_map()
    scenario = write_scenario(
        tmp_path,
        """\
        name: from_yaml
        scenario_id: from_yaml-1
        steps: 25

        agents:
          - type: vehicle
            speed: 9.0
            route:
              start: lane/east/0
          - type: cyclist
            speed: 4.0
            mark_as_expert: true
            of_interest: true
        """,
    )
    scene = export_scene(m, tmp_path, scenario=scenario)

    assert scene["name"] == "from_yaml"
    assert scene["scenario_id"] == "from_yaml-1"
    assert [obj["type"] for obj in scene["objects"]] == ["vehicle", "cyclist"]
    assert scene["objects"][1]["mark_as_expert"] is True
    assert len(scene["objects"][0]["position"]) == 25
    assert scene["metadata"]["objects_of_interest"] == [scene["objects"][1]["id"]]

    # The route reached the track: the east arm is driven inwards from (70, 0).
    start = scene["objects"][0]["position"][0]
    assert start["x"] == pytest.approx(70.0, abs=2.0)

    # And the same scene comes back as a string, for a caller who does not want a file.
    assert json.loads(m.to_gpudrive_json(scenario=scenario)) == scene


def test_a_gpudrive_scenario_with_an_unknown_key_is_refused(tmp_path):
    scenario = write_scenario(tmp_path, "speed: 3.0\n")
    with pytest.raises(ValueError, match="scenario"):
        clipgt_map().export_gpudrive(tmp_path / "scene.json", scenario=scenario)


def test_gpudrive_says_what_it_cannot_carry():
    m = clipgt_map()
    warnings = m.gpudrive_warnings()
    assert any("no z" in warning for warning in warnings)
    assert any("topology" in warning for warning in warnings)
    assert any("traffic-light element" in warning for warning in warnings)
    # And it stays out of the general warnings, which are about OpenDRIVE and
    # Lanelet2 and would otherwise be noise for a caller who never writes a scene.
    assert m.format_warnings() == []


# --------------------------------------------------------------------------- #
# Drawing what was written
# --------------------------------------------------------------------------- #


def test_every_export_can_be_drawn_back(tmp_path):
    """The `render_*` functions read the files, so they are a check on the files.

    Nothing here passes the map to the renderer: each picture is made from the bytes
    on disk, which is what makes an empty one worth failing over.
    """
    m = clipgt_map()
    m.export_opendrive(tmp_path / "map.xodr")
    m.export_sumo(tmp_path / "sumo")
    m.export_clipgt(tmp_path / "clip")
    m.export_gpudrive(tmp_path / "scene.json")

    pictures = {
        "OpenDRIVE": roadgen.render_opendrive(str(tmp_path / "map.xodr")),
        "SUMO": roadgen.render_sumo(str(tmp_path / "sumo")),
        "ClipGT": roadgen.render_clipgt(str(tmp_path / "clip")),
        "GPUDrive": roadgen.render_gpudrive(str(tmp_path / "scene.json")),
    }

    for name, svg in pictures.items():
        assert svg.startswith("<svg"), name
        assert svg.rstrip().endswith("</svg>"), name
        assert f"<title>{name}</title>" in svg
        # A document a browser will not draw is not a picture.
        ET.fromstring(svg)
        # Something was found. An empty picture parses and says nothing.
        assert "<polyline" in svg or "<polygon" in svg, name
        assert "nothing to draw" not in svg, name


def test_a_picture_explains_what_the_format_could_not_carry(tmp_path):
    m = clipgt_map()
    m.export_gpudrive(tmp_path / "scene.json")
    svg = roadgen.render_gpudrive(str(tmp_path / "scene.json"))
    # The same thing `gpudrive_warnings()` says, said to whoever is looking at the
    # picture rather than to whoever wrote the export.
    assert "no z" in svg


def test_drawing_a_file_that_is_not_there_says_so(tmp_path):
    with pytest.raises(RuntimeError):
        roadgen.render_opendrive(str(tmp_path / "nothing.xodr"))
    with pytest.raises(RuntimeError):
        roadgen.render_sumo(str(tmp_path / "no-such-directory"))


def test_drawing_a_file_that_is_not_the_format_says_so(tmp_path):
    path = tmp_path / "map.xodr"
    path.write_text("<not-opendrive/>")
    with pytest.raises(RuntimeError):
        roadgen.render_opendrive(str(path))


# --------------------------------------------------------------------------- #
# Buildings
# --------------------------------------------------------------------------- #


def street(name="high", length=400.0):
    m = roadgen.Map(name="town", origin=(35.6586, 139.7454, 0.0))
    m.add_road(
        start=(0.0, 0.0, 0.0), end=(length, 0.0, 0.0), lanes=two_way(), name=name,
    )
    return m


def test_buildings_are_off_until_they_are_asked_for():
    m = street()
    assert m.building_ids() == []
    m.generate_buildings()
    assert len(m.building_ids()) > 0


def test_buildings_can_be_switched_back_off():
    m = street()
    m.generate_buildings()
    assert m.building_ids()
    m.generate_buildings(False)
    assert m.building_ids() == []


def test_every_preset_can_be_named_and_its_text_read_back():
    presets = roadgen.building_presets()
    assert "town" in presets and "downtown" in presets
    for name in presets:
        text = roadgen.building_rules(name)
        assert "Lot" in text and "-->" in text
        m = street()
        m.generate_buildings(rules=name)
        assert m.building_ids(), name
        # The text a preset hands back is the preset: generating from either
        # names the same buildings.
        by_text = street()
        by_text.generate_buildings(rules=text)
        assert by_text.building_ids() == m.building_ids()
    # With no name, the set `generate_buildings()` uses when none is given.
    assert roadgen.building_rules() == roadgen.building_rules("town")


def test_rules_of_your_own_decide_what_gets_built():
    m = street()
    m.generate_buildings(rules="""
attr LotWidth = 20
attr LotDepth = 12
attr LotGap = 5
Lot --> Extrude(FloorHeight * 4) I("library")
""")
    ids = m.building_ids()
    assert ids
    for identifier in ids:
        kind, height, levels = m.building_shape(identifier)
        assert kind == "library"
        # FloorHeight was not declared, so it is the default 3.2.
        assert height == pytest.approx(12.8)
        assert levels == 4
        # A 20 x 12 lot, whichever way round the frontage runs.
        footprint = m.building_footprint(identifier)
        assert len(footprint) == 4
        sides = sorted(
            round(math.dist(footprint[i][:2], footprint[(i + 1) % 4][:2]), 3)
            for i in range(4)
        )
        assert sides == pytest.approx([12.0, 12.0, 20.0, 20.0])


def test_a_grammar_that_does_not_parse_is_refused_where_it_is_written():
    m = street()
    with pytest.raises(ValueError) as raised:
        m.generate_buildings(rules="Lot --> Extrude(")
    assert "line 1" in str(raised.value)
    # And a misspelt preset is not silently taken for an empty grammar.
    with pytest.raises(ValueError) as raised:
        m.generate_buildings(rules="dowtnown")
    assert "downtown" in str(raised.value)
    # Nothing was switched on by either attempt.
    assert m.building_ids() == []


def test_the_same_seed_builds_the_same_town():
    def build(seed):
        m = street()
        m.generate_buildings(seed=seed)
        return [
            (i, m.building_shape(i), m.building_footprint(i)) for i in m.building_ids()
        ]

    assert build(5) == build(5)
    assert build(5) != build(6)


def test_no_building_stands_in_the_road():
    m = street()
    m.generate_buildings()
    # Two 3.5 m lanes and the default 5 m setback: nothing may come nearer the
    # reference line than 8.5 m.
    for identifier in m.building_ids():
        for x, y, z in m.building_footprint(identifier):
            assert abs(y) >= 8.5 - 1e-6


def test_buildings_reach_openstreetmap_and_opendrive(tmp_path):
    m = street()
    m.generate_buildings()
    count = len(m.building_ids())

    osm = ET.fromstring(m.to_osm_xml())
    ways = [
        way
        for way in osm.findall("way")
        if any(tag.get("k") == "building" for tag in way.findall("tag"))
    ]
    assert len(ways) == count
    for way in ways:
        refs = [node.get("ref") for node in way.findall("nd")]
        # A closed way: the first node is also the last.
        assert refs[0] == refs[-1]

    xodr = ET.fromstring(m.to_opendrive_xml())
    objects = [
        obj
        for road in xodr.findall("road")
        for obj in road.findall("objects/object")
        if obj.get("type") == "building"
    ]
    assert len(objects) == count
    for obj in objects:
        assert obj.findall("outline/cornerLocal")


def test_a_format_that_cannot_hold_a_building_says_so():
    m = street()
    m.generate_buildings()
    for warnings in (m.sumo_warnings(), m.clipgt_warnings(), m.gpudrive_warnings(),
                     m.format_warnings()):
        assert any("buildings" in warning for warning in warnings)
    # And a map without them is not told they were dropped.
    plain = street()
    assert not any("buildings" in warning for warning in plain.sumo_warnings())


def test_the_opendrive_picture_shows_the_town(tmp_path):
    m = street()
    m.generate_buildings()
    m.export_opendrive(tmp_path / "map.xodr")
    svg = roadgen.render_opendrive(str(tmp_path / "map.xodr"))
    assert "rg-building" in svg
    assert "buildings, drawn from the outline" in svg
