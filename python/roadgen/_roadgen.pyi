"""Type stubs for the Rust extension module."""

from typing import Any, Iterable, Optional, Sequence

__version__: str

Point = tuple[float, float, float]

class Lane:
    """One lane of a road's cross-section."""

    def __init__(
        self,
        width: float,
        direction: str = "forward",
        type_: str = "driving",
        speed_limit_kph: Optional[float] = None,
        side: Optional[str] = None,
        left_marking: str = "solid",
        right_marking: str = "solid",
        marking_color: str = "white",
        width_profile: Optional[Sequence[tuple[float, float]]] = None,
        taper: str = "linear",
        height: Optional[tuple[float, float]] = None,
        height_profile: Optional[Sequence[tuple[float, float, float]]] = None,
    ) -> None:
        """`height` is `(inner, outer)` metres off the road surface — a pavement a
        kerb above the carriageway, say — with inner the edge nearer the reference
        line; `height_profile` is `(station, inner, outer)` knots for a lift that
        changes along the road."""
    @property
    def width(self) -> float: ...
    @property
    def width_profile(self) -> list[tuple[float, float]]: ...
    @property
    def height_profile(self) -> list[tuple[float, float, float]]: ...
    @property
    def direction(self) -> str: ...
    @property
    def type_(self) -> str: ...

class LaneRef:
    """A reference to one lane of one road."""

    @property
    def road(self) -> str: ...
    @property
    def index(self) -> int: ...
    @property
    def id(self) -> str: ...

class Road:
    """A handle on a road that has been added to a map."""

    @property
    def id(self) -> str: ...
    @property
    def lane_count(self) -> int: ...
    def lane(self, index: int) -> LaneRef: ...
    def lanes(self) -> list[LaneRef]: ...

class Junction:
    """A handle on a junction."""

    @property
    def id(self) -> str: ...

class Alignment:
    """A road alignment built one piece at a time."""

    def __init__(self, start: Point, heading: float = 0.0) -> None: ...
    def line(self, length: float, rise: float = 0.0) -> None: ...
    def arc(self, length: float, curvature: float, rise: float = 0.0) -> None: ...
    def spiral(
        self, length: float, curvature_end: float, rise: float = 0.0
    ) -> None: ...
    @property
    def point(self) -> Point: ...
    @property
    def heading(self) -> float: ...
    @property
    def curvature(self) -> float: ...

class Pit:
    """An open pit on the map, for drawing the roads into it."""

    @property
    def name(self) -> str: ...
    @property
    def land_cover(self) -> str: ...
    @property
    def rim_height(self) -> float: ...
    @property
    def floor_height(self) -> float: ...
    def ramp(
        self,
        start: float = -90.0,
        grade: float = 0.1,
        clockwise: bool = True,
        width: float = 21.0,
        approach: float = 80.0,
    ) -> list[Point]:
        """The points of the usual ramp: an approach from clear of the rim, then a
        spiral down the wall at `grade` to the floor. Pass them to
        `add_road(points=...)`."""
    def rim(self, spacing: float = 5.0) -> list[Point]: ...

Span = tuple[str, float, float]

class Corridor:
    """A motorway corridor. It puts nothing on the map; `plan()` draws its roads
    and `Map.add_plan()` puts them there."""
    @property
    def name(self) -> str: ...
    @property
    def standard(self) -> dict[str, float]: ...
    def add_interchange(
        self,
        name: str,
        at: float,
        crossroad: Sequence[Point],
        crossroad_lanes: Sequence[Lane],
        kind: str = "diamond",
        side: Optional[str] = None,
        over: bool = True,
        control: str = "signal",
        terminal: float = 80.0,
        reach: float = 320.0,
    ) -> None:
        """`kind` is `diamond`, `half_diamond` (with `side`), `parclo` — a diamond
        whose turns on across the crossroad are loops — or `trumpet`, whose
        crossroad stops beyond the motorway on `side`. A loop is 30 m round
        whatever the terminal; it needs the terminal about 76 m out or more (79 m
        beside three lanes each way), and the default 80 m will do."""
    def add_ramp(
        self, name: str, at: float, end: Point, side: str = "right", on: bool = False
    ) -> None: ...
    def plan(self) -> CorridorPlan: ...

class CorridorPlan:
    """A first draft of a corridor's roads, to look at, edit and add."""
    @property
    def roads(self) -> list[dict[str, Any]]: ...
    @property
    def notes(self) -> list[str]: ...
    @property
    def junctions(self) -> list[str]: ...
    @property
    def structures(self) -> list[tuple[str, list[Span]]]: ...
    def set_points(self, name: str, points: Sequence[Point]) -> None: ...

class Map:
    """A road network under construction."""

    def __init__(
        self,
        name: Optional[str] = None,
        origin: Optional[Point] = None,
        projection: str = "local_cartesian",
        handedness: str = "rht",
        sampling: float = 2.0,
    ) -> None: ...
    def add_road(
        self,
        lanes: Sequence[Lane],
        start: Optional[Point] = None,
        end: Optional[Point] = None,
        points: Optional[Iterable[Point]] = None,
        alignment: Optional[Alignment] = None,
        name: Optional[str] = None,
        type_: str = "town",
        speed_limit_kph: Optional[float] = None,
        superelevation: Optional[Sequence[tuple[float, float]]] = None,
        cross_sections: Optional[Sequence[tuple[float, Sequence[Lane]]]] = None,
        pavement: Optional[Sequence[tuple[float, str]]] = None,
        site: Optional[str] = None,
        form: Optional[str] = None,
        corridor: Optional[str] = None,
        structures: Optional[str] = None,
    ) -> Road:
        """`form` is the road's form of way: `motorway`, `slip_road`,
        `multiple_carriageway`, `single_carriageway`, `parallel_road`,
        `service_road`, `roundabout`. A motorway or slip road crosses nothing on
        the level and has no frontage for a building. `corridor` names the
        corridor the road belongs to: the land is cut and filled to meet it, and
        its structures are chosen from the land. `structures="auto"` has an
        ordinary road's structures chosen from the land the same way: a stretch
        far above it or over water goes on a bridge, one far below it through a
        bored tunnel, and the rest is cut and filled to meet it."""
        ...
    def add_junction(self, name: Optional[str] = None) -> Junction: ...
    def connect(
        self,
        a: Road,
        b: Road,
        junction: Optional[Junction] = None,
        ends: tuple[str, str] = ("end", "start"),
    ) -> list[tuple[str, str]]: ...
    def connect_lanes(
        self, from_lane: LaneRef, to_lane: LaneRef, junction: Optional[Junction] = None
    ) -> str: ...
    def add_stop_line(self, lane: LaneRef, end: str = "end", setback: float = 0.0) -> str: ...
    def add_traffic_light(
        self, lane: LaneRef, end: str = "end", height: float = 5.0, setback: float = 0.0
    ) -> str: ...
    def add_traffic_sign(
        self, lane: LaneRef, code: str, end: str = "end", height: float = 2.5, setback: float = 0.0
    ) -> str: ...
    def add_crosswalk(
        self, road: Road, fraction: float = 0.5, width: float = 4.0
    ) -> str: ...
    def add_traffic_light_rule(
        self,
        lights: Sequence[str],
        lanes: Sequence[LaneRef],
        stop_line: Optional[str] = None,
    ) -> None: ...
    def add_right_of_way(
        self,
        right_of_way: Sequence[LaneRef],
        yielding: Sequence[LaneRef],
        stop_line: Optional[str] = None,
    ) -> None: ...
    def add_speed_limit(self, lanes: Sequence[LaneRef], kph: float) -> None: ...
    def validate(self) -> None: ...
    def issues(self) -> list[str]: ...
    def mgrs_grid(self) -> Optional[str]: ...
    def read_warnings(self) -> list[str]: ...
    def write_read_trace(self, path: Optional[str] = None) -> str: ...
    def format_warnings(self) -> list[str]: ...
    def clipgt_warnings(self, scenario: Optional[str] = None) -> list[str]: ...
    def gpudrive_warnings(self, scenario: Optional[str] = None) -> list[str]: ...
    def carla_warnings(
        self,
        name: Optional[str] = None,
        package: Optional[str] = None,
        buildings: Optional[str] = None,
        use_carla_materials: Optional[bool] = None,
        kerb_height: Optional[float] = None,
        verge_width: Optional[float] = None,
        ground_extent: Optional[float] = None,
    ) -> list[str]: ...
    def osm_warnings(self) -> list[str]: ...
    def sumo_warnings(self) -> list[str]: ...
    def sumo_lane_ids(self) -> list[tuple[str, str]]: ...
    def export_ir(self, path: str) -> None: ...
    def export_opendrive(self, path: str, trace: bool = True) -> None: ...
    def export_lanelet2(self, path: str, trace: bool = True) -> None: ...
    def export_osm(self, path: str, trace: bool = True) -> None: ...
    def export_sumo(
        self,
        directory: str,
        trace: bool = True,
        curve_lateral_acceleration: float | None = None,
    ) -> str: ...
    def export_clipgt(
        self,
        directory: str,
        scenario: Optional[str] = None,
        clip_id: Optional[str] = None,
        frame_rate: Optional[float] = None,
        speed: Optional[float] = None,
        route: Optional[Sequence[str]] = None,
        trace: bool = True,
    ) -> str: ...
    def export_gpudrive(
        self,
        path: str,
        scenario: Optional[str] = None,
        name: Optional[str] = None,
        scenario_id: Optional[str] = None,
        steps: Optional[int] = None,
        time_step: Optional[float] = None,
        speed: Optional[float] = None,
        route: Optional[Sequence[str]] = None,
        trace: bool = True,
    ) -> None: ...
    def export_carla(
        self,
        directory: str,
        name: Optional[str] = None,
        package: Optional[str] = None,
        buildings: Optional[str] = None,
        use_carla_materials: Optional[bool] = None,
        kerb_height: Optional[float] = None,
        verge_width: Optional[float] = None,
        ground_extent: Optional[float] = None,
        carla_root: Optional[str] = None,
        engine: Optional[str] = None,
        sun_altitude: Optional[float] = None,
        sun_azimuth: Optional[float] = None,
        trace: bool = True,
    ) -> dict[str, Any]: ...
    def export_pointcloud_map(
        self,
        directory: str,
        name: Optional[str] = None,
        buildings: Optional[str] = None,
        kerb_height: Optional[float] = None,
        verge_width: Optional[float] = None,
        ground_extent: Optional[float] = None,
        spacing: Optional[float] = None,
        cell_size: Optional[float] = None,
    ) -> dict[str, Any]: ...
    def to_gpudrive_json(
        self,
        scenario: Optional[str] = None,
        name: Optional[str] = None,
        scenario_id: Optional[str] = None,
        steps: Optional[int] = None,
        time_step: Optional[float] = None,
        speed: Optional[float] = None,
        route: Optional[Sequence[str]] = None,
    ) -> str: ...
    def to_opendrive_xml(self) -> str: ...
    def to_lanelet2_osm(self) -> str: ...
    def to_osm_xml(self) -> str: ...
    def generate_buildings(
        self,
        enabled: bool = True,
        rules: Optional[str] = None,
        seed: int = 0,
    ) -> None: ...
    def add_pit(
        self,
        name: str = "pit",
        centre: Point = (0.0, 0.0, 0.0),
        heading: float = 0.0,
        floor: tuple[float, float] = (220.0, 140.0),
        benches: int = 6,
        bench_height: float = 12.0,
        bench_width: float = 10.0,
        face_angle: float = 70.0,
        site: Optional[str] = None,
    ) -> Pit:
        """Puts an open pit on the map: a floor and benches stepped up from it to
        the rim at `centre`'s height. Lays out no road: draw the roads into it with
        `add_road`; `Pit.ramp()` gives the points of the usual spiral ramp."""
    def add_heap(
        self,
        kind: str,
        centre: Point,
        radius: float,
        height: float,
        repose: float = 37.0,
        name: Optional[str] = None,
        site: Optional[str] = None,
    ) -> str:
        """Puts a heap of `kind` (`waste_dump`, `stockpile`) on the map; returns
        its land cover's id. `name` defaults to `kind`, so two heaps of one kind
        need names of their own."""
    def add_pad(
        self,
        outline: Sequence[Point],
        buildings: Optional[str] = None,
        seed: int = 0,
        name: str = "pad",
        site: Optional[str] = None,
    ) -> str:
        """Puts a levelled yard on the map, with `buildings` (a preset or a grammar)
        on lots along the roads beside it; returns its land cover's id."""
    def add_barrier(self, lane: LaneRef, kind: str = "berm", height: float = 1.5) -> str:
        """Stands a barrier along the whole of `lane`; returns the object's id."""
    def generate_land(
        self,
        enabled: bool = True,
        relief: float = 6.0,
        cell: float = 10.0,
        margin: float = 150.0,
        wavelength: float = 180.0,
        seed: int = 0,
    ) -> None:
        """States the land round the map's roads as a TIN, shaped by every pit,
        heap and pad on the map. A pit, heap or pad turns it on with these defaults,
        and `enabled=False` does not turn it off then: such a map always has land."""
    def add_bridge(
        self,
        spans: Sequence[Span],
        form: str = "bridge",
        material: str = "concrete",
        deck_depth: Optional[float] = None,
        pier_spacing: float = 40.0,
        name: Optional[str] = None,
    ) -> None:
        """A bridge under `[(road, from, to), ...]`; `form` is `bridge`, `viaduct`
        or `overpass`."""
        ...
    def add_tunnel(
        self,
        spans: Sequence[Span],
        form: str = "bored",
        clear_height: float = 5.0,
        lining: float = 0.6,
        lighting: bool = True,
        name: Optional[str] = None,
    ) -> None:
        """A tunnel round `[(road, from, to), ...]`; `form` is `bored`,
        `cut_and_cover` or `underpass`."""
        ...
    def add_interchange(
        self, name: str, kind: str, junctions: Sequence[str], roads: Sequence[str] = ...
    ) -> str: ...
    def add_hill(
        self, crest: Sequence[Point], width: float, height: float, name: str = "hill"
    ) -> None: ...
    def add_river(
        self,
        centre: Sequence[Point],
        width: float,
        depth: float,
        bank_slope: float = 2.0,
        name: str = "river",
        site: Optional[str] = None,
    ) -> str: ...
    def add_corridor(
        self,
        name: str,
        centre: Sequence[Point],
        design_speed: float = 100.0,
        profile: Optional[Sequence[tuple[float, float]]] = None,
        lanes: int = 2,
        structures: str = "auto",
        elevated: Optional[Sequence[tuple[float, float]]] = None,
        elevated_height: float = 9.0,
        sunken: Optional[Sequence[tuple[float, float]]] = None,
        sunken_depth: float = 10.0,
        end_transition: Optional[float] = None,
        shared_deck: bool = False,
    ) -> Corridor: ...
    def add_plan(self, plan: CorridorPlan) -> list[str]: ...
    def generation_notes(self) -> list[str]: ...
    def structure_ids(self) -> list[str]: ...
    def structure(self, structure: str) -> dict[str, Any]: ...
    def grade_separations(self) -> list[dict[str, Any]]: ...
    def interchange_ids(self) -> list[str]: ...
    def interchange(self, interchange: str) -> dict[str, Any]: ...
    def road_form(self, road: str) -> str: ...
    def road_structures(self, road: str) -> list[tuple[float, float, str]]: ...
    def site_ids(self) -> list[str]: ...
    def site(self, site: str) -> dict[str, Any]: ...
    def tin_surface_ids(self) -> list[str]: ...
    def tin_mesh(self, surface: str) -> tuple[list[Point], list[tuple[int, int, int]]]: ...
    def tin_breaklines(self, surface: str) -> list[tuple[str, str, list[Point]]]: ...
    def terrain_height(self, x: float, y: float) -> Optional[float]: ...
    def land_cover_ids(self) -> list[str]: ...
    def land_cover(self, cover: str) -> dict[str, Any]: ...
    def road_pavement(self, road: str) -> list[tuple[float, str]]: ...
    def building_ids(self) -> list[str]: ...
    def building_kind(self, building: str) -> str: ...
    def building_frontage(self, building: str) -> Optional[tuple[str, str, float]]: ...
    def building_parts(self, building: str) -> list[str]: ...
    def building_footprint(self, part: str) -> list[Point]: ...
    def building_part_shape(
        self, part: str
    ) -> tuple[float, float, str, float, float, int]: ...
    def building_part_kind(self, part: str) -> Optional[str]: ...
    def building_shell(self, part: str) -> list[list[Point]]: ...
    def road_ids(self) -> list[str]: ...
    def lane_ids(self) -> list[str]: ...
    def connections(self) -> list[tuple[str, str]]: ...
    def successors(self, lane: str) -> list[str]: ...
    def lane_centerline(self, lane: str) -> list[Point]: ...

class Trace:
    """The IR dump and trace files of one map, joined.

    Every file has to come from the same map, and every file a trace describes has
    to be unchanged since the trace was written.
    """

    def __init__(self, check_files: bool = True) -> None: ...
    @staticmethod
    def load(*paths: str, check_files: bool = True) -> "Trace":
        """IR dumps, trace files and a SUMO `.net.xml` (after the SUMO trace)."""
    def add(self, path: str) -> None: ...
    def add_sumo_net(self, path: str) -> tuple[int, int]:
        """Traces netconvert's internal lanes; returns (traced, untraced)."""
    def formats(self) -> list[str]: ...
    def to_ir(self, format: str, element: int | str) -> list[dict[str, Any]]:
        """`{"ir", "ref", "rel", "role"}` for each IR element `element` came from."""
    def from_ir(self, ir: str, format: str) -> list[dict[str, Any]]: ...
    def translate(
        self, format: str, element: int | str, to: str
    ) -> list[dict[str, Any]]:
        """`{"ref", "ir", "rel", "role", "via"}` for each element of `to`."""
    def neighbours(self, ir: str) -> list[str]: ...

def building_presets() -> list[str]:
    """The names of the built-in building rule sets."""

def building_rules(name: Optional[str] = None) -> str:
    """A built-in rule set, as the CGA grammar it is; the default one with no name."""

def read_opendrive(
    path: str,
    sampling: float = 2.0,
    origin: Optional[Point] = None,
    projection: str = "local_cartesian",
) -> Map:
    """The OpenDRIVE file at `path`, read as a map that exports like any other and
    cannot be added to. `Map.read_warnings()` says what the file stated that the map
    could not keep."""

def render_opendrive(path: str) -> str:
    """The OpenDRIVE file at `path`, drawn as an SVG document."""

def render_sumo(directory: str) -> str:
    """The SUMO plain-XML network in `directory`, drawn as an SVG document."""

def render_clipgt(directory: str) -> str:
    """The ClipGT clip in `directory`, drawn as an SVG document."""

def render_gpudrive(path: str) -> str:
    """The GPUDrive scene at `path`, drawn as an SVG document."""

def render_carla(path: str) -> str:
    """The FBX at `path`, drawn as an SVG document.

    Coloured by the semantic class CARLA will give each mesh, worked out from the
    mesh's name the way CARLA works it out.
    """
