"""The IR as Protocol Buffers, from Python.

Two promises: a map written with `export_proto` and read with `read_proto` is the
same map, and a program that knows nothing of roadgen — only the schema — can read
the document. The second is checked by compiling `proto_schema()` with the stock
compiler and running `examples/proto_importer.py` on what came out.
"""

import importlib.util
import itertools
import math
import pathlib
import sys

import pytest

import roadgen

EXAMPLES = pathlib.Path(__file__).resolve().parents[2] / "examples"


def crossroads():
    """Four two-way arms and a junction joining every pair of them."""
    m = roadgen.Map(name="proto")
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


def test_a_map_comes_back_as_it_was_written(tmp_path):
    m = crossroads()
    m.export_proto(str(tmp_path / "map.roadgen.pb"))
    back = roadgen.read_proto(str(tmp_path / "map.roadgen.pb"))

    assert back.lane_ids() == m.lane_ids()
    m.export_lanelet2(str(tmp_path / "written.osm"), trace=False)
    back.export_lanelet2(str(tmp_path / "read.osm"), trace=False)
    assert (tmp_path / "read.osm").read_text() == (tmp_path / "written.osm").read_text()


def test_samples_can_be_left_out(tmp_path):
    m = crossroads()
    m.export_proto(str(tmp_path / "full.pb"))
    m.export_proto(str(tmp_path / "lean.pb"), curve_samples=False)
    assert (tmp_path / "lean.pb").stat().st_size < (tmp_path / "full.pb").stat().st_size
    assert roadgen.read_proto(str(tmp_path / "lean.pb")).lane_ids() == m.lane_ids()


def test_a_file_that_is_not_a_document_is_refused(tmp_path):
    (tmp_path / "junk.pb").write_bytes(b"\xff\xff\xff")
    with pytest.raises(ValueError):
        roadgen.read_proto(str(tmp_path / "junk.pb"))


def test_the_schema_is_shipped_with_the_package():
    schema = roadgen.proto_schema()
    assert "roadgen_ir/v1/map.proto" in schema
    assert all("package roadgen_ir.v1;" in text for text in schema.values())


def test_a_reader_that_knows_only_the_schema_reads_the_network(tmp_path, monkeypatch):
    protoc = pytest.importorskip("grpc_tools.protoc")
    pytest.importorskip("google.protobuf")

    # The schema, compiled the way any user of it would.
    source = tmp_path / "proto"
    for name, text in roadgen.proto_schema().items():
        (source / name).parent.mkdir(parents=True, exist_ok=True)
        (source / name).write_text(text)
    generated = tmp_path / "generated"
    generated.mkdir()
    status = protoc.main(
        ["protoc", f"-I{source}", f"--python_out={generated}"]
        + [str(source / name) for name in roadgen.proto_schema()]
    )
    assert status == 0

    monkeypatch.syspath_prepend(str(generated))
    spec = importlib.util.spec_from_file_location(
        "proto_importer", EXAMPLES / "proto_importer.py"
    )
    importer = importlib.util.module_from_spec(spec)
    monkeypatch.setitem(sys.modules, "proto_importer", importer)
    spec.loader.exec_module(importer)

    m = crossroads()
    m.export_proto(str(tmp_path / "map.roadgen.pb"))
    network = importer.read(str(tmp_path / "map.roadgen.pb"))

    assert network.name == "proto"
    assert sorted(network.lanes) == sorted(m.lane_ids())
    # Every lane is driven in the order the importer put it in: where one lane
    # leaves off, each of its successors takes over.
    joins = 0
    for lane in network.lanes.values():
        assert lane.type == "driving"
        assert lane.speed_limit == pytest.approx(40.0 / 3.6)
        for successor in lane.successors:
            gap = math.dist(lane.centreline[-1], network.lanes[successor].centreline[0])
            assert gap < 1e-6, f"{lane.id} -> {successor} is {gap} m apart"
            joins += 1
    # Twelve movements through the junction, each joined at both ends.
    assert joins == 24
