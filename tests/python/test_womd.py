"""The map as a Waymo Open Motion Dataset scenario, from Python.

A WOMD file is a TFRecord of `waymo.open_dataset.Scenario` messages. These tests
unframe the file the way TensorFlow does — length, checksums, data — and decode the
scenario with protobuf's own runtime from Waymo's schema, the one the crate is
compiled from, so what is checked is what any WOMD reader would see.
"""

import itertools
import pathlib
import struct
import sys

import pytest

import roadgen

SCHEMA = pathlib.Path(__file__).resolve().parents[2] / "crates" / "roadgen-womd" / "proto"


def crossroads():
    """Four two-way arms a metre up, a junction joining every pair, one light."""
    m = roadgen.Map(name="womd")
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
                speed_limit_kph=40.0,
                lanes=[
                    roadgen.Lane(width=3.5, direction="forward"),
                    roadgen.Lane(width=3.5, direction="backward"),
                ],
            )
        )
    junction = m.add_junction("x")
    for a, b in itertools.combinations(arms, 2):
        m.connect(a, b, junction=junction)
    return m


def crc32c(data):
    crc = 0xFFFFFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0x82F63B78 if crc & 1 else crc >> 1
    return crc ^ 0xFFFFFFFF


def masked(data):
    crc = crc32c(data)
    return ((((crc >> 15) | (crc << 17)) & 0xFFFFFFFF) + 0xA282EAD8) & 0xFFFFFFFF


def records(blob):
    """The records of a TFRecord file, checked the way TensorFlow checks them."""
    out = []
    while blob:
        header, blob = blob[:12], blob[12:]
        (length,) = struct.unpack("<Q", header[:8])
        assert struct.unpack("<I", header[8:])[0] == masked(header[:8])
        data, crc, blob = blob[:length], blob[length : length + 4], blob[length + 4 :]
        assert struct.unpack("<I", crc)[0] == masked(data)
        out.append(data)
    return out


@pytest.fixture(scope="module")
def scenario_pb2(tmp_path_factory):
    protoc = pytest.importorskip("grpc_tools.protoc")
    pytest.importorskip("google.protobuf")
    generated = tmp_path_factory.mktemp("generated")
    status = protoc.main(
        [
            "protoc",
            f"-I{SCHEMA}",
            f"--python_out={generated}",
            str(SCHEMA / "waymo_open_dataset" / "protos" / "map.proto"),
            str(SCHEMA / "waymo_open_dataset" / "protos" / "scenario.proto"),
        ]
    )
    assert status == 0
    sys.path.insert(0, str(generated))
    try:
        from waymo_open_dataset.protos import scenario_pb2
    finally:
        sys.path.remove(str(generated))
    return scenario_pb2


def test_the_file_is_one_framed_scenario(tmp_path, scenario_pb2):
    m = crossroads()
    path = tmp_path / "map.tfrecord"
    m.export_womd(str(path))
    (record,) = records(path.read_bytes())
    assert record == m.to_womd_bytes()

    scenario = scenario_pb2.Scenario.FromString(record)
    assert scenario.scenario_id == "womd"
    assert len(scenario.timestamps_seconds) == 91
    assert len(scenario.dynamic_map_states) == 91
    assert scenario.current_time_index == 10
    assert len(scenario.tracks) == 0
    assert (tmp_path / "map.tfrecord.trace.json").exists()


def test_the_lanes_are_a_graph_with_heights(scenario_pb2):
    scenario = scenario_pb2.Scenario.FromString(crossroads().to_womd_bytes())
    lanes = {f.id: f.lane for f in scenario.map_features if f.HasField("lane")}
    assert len(lanes) > 8, "the eight arm lanes and the ways through the junction"

    for lane in lanes.values():
        assert lane.speed_limit_mph == pytest.approx(40.0 / 1.609344)
        for exit_id in lane.exit_lanes:
            assert exit_id in lanes
        if lane.interpolating:
            assert lane.entry_lanes and lane.exit_lanes
    # The arms climb a metre towards the junction.
    assert max(p.z for lane in lanes.values() for p in lane.polyline) == pytest.approx(1.0)


def test_the_arguments_reach_the_scenario(scenario_pb2):
    record = crossroads().to_womd_bytes(scenario_id="x", steps=20, current_time_index=4)
    scenario = scenario_pb2.Scenario.FromString(record)
    assert scenario.scenario_id == "x"
    assert len(scenario.timestamps_seconds) == 20
    assert scenario.current_time_index == 4


def test_a_current_step_past_the_end_is_refused():
    with pytest.raises(RuntimeError):
        crossroads().to_womd_bytes(steps=5, current_time_index=5)


def test_the_warnings_say_what_the_format_cannot_hold():
    warnings = crossroads().womd_warnings()
    assert any("no tracks" in warning for warning in warnings)
