"""A roadgen IR importer written against the Protocol Buffers schema alone.

Nothing here imports roadgen: this is what a simulator, a planner or another map
library would write to read a document `Map.export_proto` produced. Generate the
message classes from the schema first, with the stock compiler:

    pip install protobuf grpcio-tools
    python -m grpc_tools.protoc -I crates/roadgen-proto/proto --python_out=generated \\
        crates/roadgen-proto/proto/roadgen_ir/v1/*.proto
    PYTHONPATH=generated python examples/proto_importer.py map.roadgen.pb

(Without the repository, `roadgen.proto_schema()` hands back the same files.)

The whole of the work is three things the schema's comments say: resolve
references through a dictionary per kind, read each curve's `samples` as its
vertices, and turn a backward lane round to get it in travel order.
"""

from __future__ import annotations

import dataclasses
import sys

from roadgen_ir.v1 import geometry_pb2, map_pb2, semantics_pb2, topology_pb2

Point = tuple[float, float, float]


@dataclasses.dataclass
class Lane:
    id: str
    road: str
    type: str
    # Metres per second: a speed-limit rule's over the lane's over its road's, or
    # None where none of them says.
    speed_limit: float | None
    # In the order traffic drives them, the left one on the driver's left.
    centreline: list[Point]
    left: list[Point]
    right: list[Point]
    successors: list[str] = dataclasses.field(default_factory=list)
    predecessors: list[str] = dataclasses.field(default_factory=list)


@dataclasses.dataclass
class Network:
    name: str
    origin: tuple[float, float, float]
    lanes: dict[str, Lane]


def vertices(curve: geometry_pb2.Curve) -> list[Point]:
    """The curve as a polyline: its samples, in reference-line order."""
    if not curve.samples:
        raise ValueError("written without curve samples; evaluate `curve.kind` instead")
    return [(s.point.x, s.point.y, s.point.z) for s in curve.samples]


def read(path: str) -> Network:
    document = map_pb2.Map()
    with open(path, "rb") as file:
        document.ParseFromString(file.read())

    roads = {road.id: road for road in document.roads}
    lanes: dict[str, Lane] = {}
    for lane in document.lanes:
        left, right, centre = (
            vertices(lane.left_boundary),
            vertices(lane.right_boundary),
            vertices(lane.centerline),
        )
        # Stored in reference-line order. A backward lane is driven the other way,
        # so its curves reverse and its two sides swap.
        if lane.direction == topology_pb2.DIRECTION_BACKWARD:
            left, right, centre = right[::-1], left[::-1], centre[::-1]
        road = roads[lane.road]
        limit = lane.speed_limit_mps if lane.HasField("speed_limit_mps") else None
        if limit is None and road.HasField("speed_limit_mps"):
            limit = road.speed_limit_mps
        lanes[lane.id] = Lane(
            id=lane.id,
            road=lane.road,
            type=semantics_pb2.LaneType.Name(lane.lane_type)
            .removeprefix("LANE_TYPE_")
            .lower(),
            speed_limit=limit,
            centreline=centre,
            left=left,
            right=right,
        )

    for rule in document.rules:
        if rule.WhichOneof("kind") == "speed_limit":
            for lane in rule.speed_limit.lanes:
                lanes[lane].speed_limit = rule.speed_limit.limit_mps

    for connection in document.connections:
        lanes[getattr(connection, "from").lane].successors.append(connection.to.lane)
        lanes[connection.to.lane].predecessors.append(getattr(connection, "from").lane)

    origin = document.metadata.origin
    return Network(
        name=document.metadata.name,
        origin=(origin.latitude, origin.longitude, origin.altitude),
        lanes=lanes,
    )


if __name__ == "__main__":
    network = read(sys.argv[1])
    print(f"{network.name or '(unnamed)'} at {network.origin}")
    for lane in network.lanes.values():
        start, end = lane.centreline[0], lane.centreline[-1]
        print(
            f"{lane.id}: {lane.type}, {len(lane.centreline)} vertices "
            f"from ({start[0]:.1f}, {start[1]:.1f}) to ({end[0]:.1f}, {end[1]:.1f})"
            f" -> {', '.join(lane.successors) or 'nothing'}"
        )
