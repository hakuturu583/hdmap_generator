"""The map as a CommonRoad scenario, from Python.

A CommonRoad scenario is one XML document. These tests read it with the standard
library, and — when it is installed — validate it against the 2020a schema that
`commonroad-io` ships and open it with `commonroad-io`'s own `CommonRoadFileReader`,
so what is checked is what a CommonRoad planner would be handed.
"""

import itertools
import os
import xml.etree.ElementTree as ElementTree

import pytest

import roadgen


def crossroads():
    """Four two-way arms, a junction joining every pair, a light, a limit."""
    m = roadgen.Map(name="commonroad town")
    arms = []
    for name, start, end in (
        ("north", (0.0, 70.0, 0.0), (0.0, 14.0, 1.0)),
        ("east", (70.0, 0.0, 0.0), (14.0, 0.0, 1.0)),
        ("south", (0.0, -70.0, 0.0), (0.0, -14.0, 1.0)),
        ("west", (-70.0, 0.0, 0.0), (-14.0, 0.0, 1.0)),
    ):
        arms.append(
            m.add_road(
                start=start,
                end=end,
                name=name,
                speed_limit_kph=36.0,
                lanes=[
                    roadgen.Lane(width=3.5, direction="forward"),
                    roadgen.Lane(width=3.5, direction="backward"),
                ],
            )
        )
    junction = m.add_junction("x")
    for a, b in itertools.combinations(arms, 2):
        m.connect(a, b, junction=junction)
    north = arms[0].lane(0)
    stop = m.add_stop_line(north)
    light = m.add_traffic_light(north)
    m.add_traffic_light_rule([light], [north], stop_line=stop)
    return m


def test_the_file_is_the_document(tmp_path):
    m = crossroads()
    path = tmp_path / "scenario.xml"
    m.export_commonroad(str(path), date="2026-01-01")
    assert path.read_text() == m.to_commonroad_xml(date="2026-01-01")
    assert (tmp_path / "scenario.xml.trace.json").exists()

    root = ElementTree.parse(path).getroot()
    assert root.tag == "commonRoad"
    assert root.get("commonRoadVersion") == "2020a"
    assert root.get("benchmarkID") == "ZAM_CommonroadTown-1_1_T-1"
    assert root.get("timeStepSize") == "0.1"
    assert root.get("date") == "2026-01-01"


def test_every_reference_lands():
    root = ElementTree.fromstring(crossroads().to_commonroad_xml())
    ids = [int(e.get("id")) for e in root.iter() if e.get("id") is not None]
    assert len(ids) == len(set(ids)), "one id space, every id once"
    refs = [int(e.get("ref")) for e in root.iter() if e.get("ref") is not None]
    assert refs and set(refs) <= set(ids)

    lanelets = root.findall("lanelet")
    assert len(lanelets) > 8, "the eight arm lanes and the ways through the junction"
    for lanelet in lanelets:
        left = lanelet.find("leftBound").findall("point")
        right = lanelet.find("rightBound").findall("point")
        assert len(left) == len(right) >= 2
        assert left[0].find("z") is None, "in the plane unless asked"
    assert len(root.findall("intersection")) == 1
    assert len(root.findall("trafficLight")) == 1
    assert len(root.findall("planningProblem")) == 1


def test_the_arguments_reach_the_scenario():
    xml = crossroads().to_commonroad_xml(
        benchmark_id="DEU_Town-2_1_T-1",
        author="someone",
        time_step=0.05,
        goal_steps=40,
        elevation=True,
    )
    root = ElementTree.fromstring(xml)
    assert root.get("benchmarkID") == "DEU_Town-2_1_T-1"
    assert root.get("author") == "someone"
    assert root.get("timeStepSize") == "0.05"
    assert root.find("planningProblem/goalState/time/intervalEnd").text == "40"
    heights = [float(z.text) for lanelet in root.iter("lanelet") for z in lanelet.iter("z")]
    assert max(heights) == pytest.approx(1.0), "the arms climb a metre"
    (light,) = root.iter("trafficLight")
    assert float(light.find("position/point/z").text) == pytest.approx(6.0, abs=0.01)

    no_problem = ElementTree.fromstring(
        crossroads().to_commonroad_xml(planning_problem=False)
    )
    assert no_problem.find("planningProblem") is None


def test_a_bad_configuration_is_refused():
    with pytest.raises(RuntimeError):
        crossroads().to_commonroad_xml(time_step=0.0)
    with pytest.raises(RuntimeError):
        crossroads().to_commonroad_xml(date="tomorrow")


def test_the_warnings_say_what_the_format_cannot_hold():
    m = crossroads()
    warnings = m.commonroad_warnings()
    assert any("inactive phase" in warning for warning in warnings)
    assert any("heights are not written" in warning for warning in warnings)
    assert not any("heights are not written" in w for w in m.commonroad_warnings(elevation=True))
    assert any("benchmark id" in w for w in m.commonroad_warnings(benchmark_id="town"))


def test_the_schema_accepts_it(tmp_path):
    etree = pytest.importorskip("lxml.etree")
    commonroad = pytest.importorskip("commonroad")
    schema = os.path.join(
        os.path.dirname(commonroad.__file__),
        "common",
        "xml_definition_files",
        "XML_commonRoad_XSD.xsd",
    )
    if not os.path.exists(schema):
        pytest.skip("this commonroad-io ships no 2020a schema")
    validator = etree.XMLSchema(etree.parse(schema))
    path = tmp_path / "scenario.xml"
    crossroads().export_commonroad(str(path))
    assert validator.validate(etree.parse(str(path))), validator.error_log


def test_commonroad_io_reads_it(tmp_path):
    reader = pytest.importorskip("commonroad.common.file_reader")
    path = tmp_path / "scenario.xml"
    crossroads().export_commonroad(str(path))
    scenario, problems = reader.CommonRoadFileReader(str(path)).open()

    network = scenario.lanelet_network
    assert len(network.lanelets) > 8
    assert len(network.intersections) == 1
    assert len(network.traffic_lights) == 1
    # One virtual speed-limit sign for every lanelet held to 36 km/h.
    (limit,) = network.traffic_signs
    assert float(limit.traffic_sign_elements[0].additional_values[0]) == pytest.approx(10.0)

    (problem,) = problems.planning_problem_dict.values()
    start = network.find_lanelet_by_position([problem.initial_state.position])[0]
    assert start, "the problem starts on a lanelet"
    for lanelet in network.lanelets:
        for successor in lanelet.successor:
            assert lanelet.lanelet_id in network.find_lanelet_by_id(successor).predecessor


def test_a_light_cycle_is_written_when_asked_for():
    m = crossroads()
    dark = ElementTree.fromstring(m.to_commonroad_xml())
    assert [c.text for c in dark.iter("color")] == ["inactive"]

    lit = ElementTree.fromstring(m.to_commonroad_xml(light_cycle="default"))
    assert [c.text for c in lit.iter("color")] == ["red", "redYellow", "green", "yellow"]
    assert [d.text for d in lit.iter("duration")] == ["60", "10", "60", "10"]

    custom = ElementTree.fromstring(
        m.to_commonroad_xml(light_cycle=[("red", 3.0), ("green", 2.5), ("yellow", 0.5)])
    )
    assert [d.text for d in custom.iter("duration")] == ["30", "25", "5"]
    assert not any("inactive" in w for w in m.commonroad_warnings(light_cycle=True))

    with pytest.raises(ValueError):
        m.to_commonroad_xml(light_cycle=[("purple", 3.0)])
    with pytest.raises(ValueError):
        m.to_commonroad_xml(light_cycle="sometimes")


def test_commonroad_io_sees_crossing_traffic_take_turns(tmp_path):
    reader = pytest.importorskip("commonroad.common.file_reader")
    # The demo's crossroads: north and west run into the junction, south and east
    # out of it, so the approaches are the forward lanes of the first two and the
    # backward lanes of the others.
    two_way = lambda: [
        roadgen.Lane(width=3.5, direction="forward"),
        roadgen.Lane(width=3.5, direction="backward"),
    ]
    m = roadgen.Map(name="lights")
    north = m.add_road(start=(0.0, 70.0, 0.0), end=(0.0, 14.0, 0.0), lanes=two_way())
    south = m.add_road(start=(0.0, -14.0, 0.0), end=(0.0, -70.0, 0.0), lanes=two_way())
    east = m.add_road(start=(14.0, 0.0, 0.0), end=(70.0, 0.0, 0.0), lanes=two_way())
    west = m.add_road(start=(-70.0, 0.0, 0.0), end=(-14.0, 0.0, 0.0), lanes=two_way())
    x = m.add_junction("x")
    for a, b in ((north, south), (north, east), (west, east), (west, south)):
        m.connect(a, b, junction=x)
    for approach in (north.lane(0), west.lane(0), south.lane(1), east.lane(1)):
        light = m.add_traffic_light(approach, end="end" if approach.index == 0 else "start")
        m.add_traffic_light_rule([light], [approach])
    path = tmp_path / "lights.xml"
    m.export_commonroad(str(path), light_cycle="default")
    scenario, _ = reader.CommonRoadFileReader(str(path)).open()
    network = scenario.lanelet_network

    def axis(light):
        lanelet = next(l for l in network.lanelets if light.traffic_light_id in l.traffic_lights)
        dx, dy = lanelet.center_vertices[-1] - lanelet.center_vertices[-2]
        return "east-west" if abs(dx) > abs(dy) else "north-south"

    turns = set()
    for step in range(280):
        green = {
            axis(light)
            for light in network.traffic_lights
            if light.get_state_at_time_step(step).value == "green"
        }
        assert len(green) < 2, f"both ways green at step {step}"
        turns |= green
    assert turns == {"east-west", "north-south"}



def test_the_map_alone_is_a_commonroad_2024_map(tmp_path):
    m = crossroads()
    path = tmp_path / "map.pb"
    m.export_commonroad_map(str(path), date="2026-01-01")
    assert path.read_bytes() == m.to_commonroad_map_bytes(date="2026-01-01")
    assert (tmp_path / "map.pb.trace.json").exists()
    assert any("no cycle" in w for w in m.commonroad_map_warnings())


def test_commonroad_io_reads_the_map_as_the_scenarios_network(tmp_path):
    reader = pytest.importorskip("commonroad.common.file_reader")
    np = pytest.importorskip("numpy")
    m = crossroads()
    m.export_commonroad_map(str(tmp_path / "map.pb"))
    m.export_commonroad(str(tmp_path / "scenario.xml"))
    network, environment = reader.CommonRoadFileReader(
        filename_map=str(tmp_path / "map.pb")
    ).open_map()
    scenario, _ = reader.CommonRoadFileReader(str(tmp_path / "scenario.xml")).open()
    expected = scenario.lanelet_network

    assert {l.lanelet_id for l in network.lanelets} == {l.lanelet_id for l in expected.lanelets}
    for lanelet in expected.lanelets:
        read = network.find_lanelet_by_id(lanelet.lanelet_id)
        assert np.allclose(read.left_vertices, lanelet.left_vertices)
        assert np.allclose(read.right_vertices, lanelet.right_vertices)
        assert sorted(read.successor) == sorted(lanelet.successor)
        assert read.adj_left == lanelet.adj_left
        assert read.adj_left_same_direction == lanelet.adj_left_same_direction
        assert read.traffic_lights == lanelet.traffic_lights
    (limit,) = network.traffic_signs
    assert limit.traffic_sign_elements[0].traffic_sign_element_id.name == "MAX_SPEED"
    assert float(limit.traffic_sign_elements[0].additional_values[0]) == pytest.approx(10.0)
    assert len(network.traffic_lights) == 1
    assert len(network.intersections) == 1
    assert environment == []
