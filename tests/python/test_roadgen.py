"""The Python front end.

These tests check the surface a user actually touches, and that it is a front end:
every result comes from the Rust core, and there is no second model of the map on
this side.
"""

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
