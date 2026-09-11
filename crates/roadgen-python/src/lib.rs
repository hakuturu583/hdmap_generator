//! PyO3 bindings for `roadgen`.
//!
//! This crate is a front end and nothing else. It holds no geometry, no topology and
//! no second copy of the IR: a `roadgen.Map` is a handle on a Rust
//! [`MapBuilder`](roadgen_core::MapBuilder), and everything Python asks it to do
//! happens in Rust. The Python objects that look like data — `Road`, `Junction`,
//! `LaneRef` — carry identifiers, not state.

use std::path::PathBuf;

use pyo3::exceptions::{PyRuntimeError, PyValueError};
use pyo3::prelude::*;

use roadgen_core::builder::{LaneRef, LaneSpec, MapBuilder, RoadSpec};
use roadgen_core::geometry::{Alignment, Curve3, Point3, Poly3Profile, SamplingConfig};
use roadgen_core::id::{JunctionId, LaneId, ObjectId, RoadId};
use roadgen_core::map::{MapMetadata, Projection, TrafficHandedness};
use roadgen_core::semantics::{BoundaryMarking, LaneType, MarkingColor, RoadMarking, RoadType};
use roadgen_core::topology::{Direction, LaneEnd, LateralSide};
use roadgen_core::units::{GeoOrigin, PositiveWidth, SpeedLimit};
use roadgen_core::validation::{UnvalidatedMap, ValidatedMap};

fn value_error<E: std::fmt::Display>(error: E) -> PyErr {
    PyValueError::new_err(error.to_string())
}

fn runtime_error<E: std::fmt::Display>(error: E) -> PyErr {
    PyRuntimeError::new_err(error.to_string())
}

fn parse_direction(value: &str) -> PyResult<Direction> {
    match value.to_ascii_lowercase().as_str() {
        "forward" => Ok(Direction::Forward),
        "backward" => Ok(Direction::Backward),
        other => Err(PyValueError::new_err(format!(
            "direction must be 'forward' or 'backward', got {other:?}"
        ))),
    }
}

fn parse_lane_type(value: &str) -> PyResult<LaneType> {
    LaneType::parse(value)
        .ok_or_else(|| PyValueError::new_err(format!("unknown lane type {value:?}")))
}

fn parse_side(value: &str) -> PyResult<LateralSide> {
    match value.to_ascii_lowercase().as_str() {
        "left" => Ok(LateralSide::Left),
        "right" => Ok(LateralSide::Right),
        other => Err(PyValueError::new_err(format!(
            "side must be 'left' or 'right', got {other:?}"
        ))),
    }
}

fn parse_end(value: &str) -> PyResult<LaneEnd> {
    match value.to_ascii_lowercase().as_str() {
        "start" => Ok(LaneEnd::Start),
        "end" => Ok(LaneEnd::End),
        other => Err(PyValueError::new_err(format!(
            "end must be 'start' or 'end', got {other:?}"
        ))),
    }
}

fn parse_marking(value: &str, color: &str) -> PyResult<BoundaryMarking> {
    let marking = RoadMarking::parse(value)
        .ok_or_else(|| PyValueError::new_err(format!("unknown road marking {value:?}")))?;
    let color = match color.to_ascii_lowercase().as_str() {
        "white" => MarkingColor::White,
        "yellow" => MarkingColor::Yellow,
        other => {
            return Err(PyValueError::new_err(format!(
                "marking colour must be 'white' or 'yellow', got {other:?}"
            )))
        }
    };
    Ok(BoundaryMarking::new(marking, color))
}

fn point(value: (f64, f64, f64)) -> Point3 {
    Point3::new(value.0, value.1, value.2)
}

/// One lane of a road's cross-section.
// `from_py_object` because a list of these is passed into `add_road`.
#[pyclass(name = "Lane", module = "roadgen", from_py_object)]
#[derive(Clone)]
pub struct PyLane {
    spec: LaneSpec,
}

#[pymethods]
impl PyLane {
    /// `width` is in metres and must be greater than zero; `direction` is relative
    /// to the road's reference line.
    #[new]
    #[pyo3(signature = (
        width,
        direction = "forward",
        type_ = "driving",
        speed_limit_kph = None,
        side = None,
        left_marking = "solid",
        right_marking = "solid",
        marking_color = "white",
    ))]
    #[allow(clippy::too_many_arguments)]
    fn new(
        width: f64,
        direction: &str,
        type_: &str,
        speed_limit_kph: Option<f64>,
        side: Option<&str>,
        left_marking: &str,
        right_marking: &str,
        marking_color: &str,
    ) -> PyResult<Self> {
        let mut spec = LaneSpec::new(
            PositiveWidth::new(width).map_err(value_error)?,
            parse_direction(direction)?,
        )
        .with_type(parse_lane_type(type_)?)
        .with_markings(
            parse_marking(left_marking, marking_color)?,
            parse_marking(right_marking, marking_color)?,
        );
        if let Some(limit) = speed_limit_kph {
            spec = spec.with_speed_limit(SpeedLimit::from_kph(limit).map_err(value_error)?);
        }
        if let Some(side) = side {
            spec = spec.with_side(parse_side(side)?);
        }
        Ok(PyLane { spec })
    }

    #[getter]
    fn width(&self) -> f64 {
        self.spec.width.metres()
    }

    #[getter]
    fn direction(&self) -> &'static str {
        self.spec.direction.as_str()
    }

    #[getter]
    fn type_(&self) -> &'static str {
        self.spec.lane_type.as_str()
    }

    fn __repr__(&self) -> String {
        format!(
            "Lane(width={}, direction={:?}, type_={:?})",
            self.spec.width.metres(),
            self.spec.direction.as_str(),
            self.spec.lane_type.as_str()
        )
    }
}

/// A handle on a road that has been added to a map.
#[pyclass(name = "Road", module = "roadgen", skip_from_py_object)]
#[derive(Clone)]
pub struct PyRoad {
    id: RoadId,
    lanes: usize,
}

#[pymethods]
impl PyRoad {
    #[getter]
    fn id(&self) -> String {
        self.id.to_string()
    }

    #[getter]
    fn lane_count(&self) -> usize {
        self.lanes
    }

    /// The lane at `index` in this road's cross-section.
    fn lane(&self, index: usize) -> PyResult<PyLaneRef> {
        if index >= self.lanes {
            return Err(PyValueError::new_err(format!(
                "road {} has {} lane(s); there is no lane at index {index}",
                self.id, self.lanes
            )));
        }
        Ok(PyLaneRef {
            reference: LaneRef::new(self.id.clone(), index),
        })
    }

    /// Every lane of this road, in cross-section order.
    fn lanes(&self) -> Vec<PyLaneRef> {
        (0..self.lanes)
            .map(|index| PyLaneRef {
                reference: LaneRef::new(self.id.clone(), index),
            })
            .collect()
    }

    fn __repr__(&self) -> String {
        format!("Road({:?}, lanes={})", self.id.to_string(), self.lanes)
    }
}

/// A handle on a junction.
#[pyclass(name = "Junction", module = "roadgen", skip_from_py_object)]
#[derive(Clone)]
pub struct PyJunction {
    id: JunctionId,
}

#[pymethods]
impl PyJunction {
    #[getter]
    fn id(&self) -> String {
        self.id.to_string()
    }

    fn __repr__(&self) -> String {
        format!("Junction({:?})", self.id.to_string())
    }
}

/// A reference to one lane of one road.
// `from_py_object` because lists of these are passed into the rule methods.
#[pyclass(name = "LaneRef", module = "roadgen", from_py_object)]
#[derive(Clone)]
pub struct PyLaneRef {
    reference: LaneRef,
}

#[pymethods]
impl PyLaneRef {
    #[getter]
    fn road(&self) -> String {
        self.reference.road.to_string()
    }

    #[getter]
    fn index(&self) -> usize {
        self.reference.index
    }

    #[getter]
    fn id(&self) -> String {
        LaneId::of_road(&self.reference.road, self.reference.index).to_string()
    }

    fn __repr__(&self) -> String {
        format!("LaneRef({:?})", self.id())
    }
}

/// A road alignment built one piece at a time.
///
/// Each call appends a piece starting where the last one ended, pointing and curving
/// the way it was, so the chain is continuous and smooth without the caller
/// restating any of it:
///
/// ```python
/// al = roadgen.Alignment(start=(0.0, 0.0, 0.0), heading=0.0)
/// al.line(80.0, rise=1.0)
/// al.spiral(60.0, curvature_end=1 / 120, rise=1.0)
/// al.arc(140.0, curvature=1 / 120, rise=2.0)
/// al.spiral(60.0, curvature_end=0.0, rise=1.0)
/// m.add_road(lanes=[...], alignment=al)
/// ```
#[pyclass(name = "Alignment", module = "roadgen", skip_from_py_object)]
pub struct PyAlignment {
    inner: Alignment,
}

#[pymethods]
impl PyAlignment {
    /// Starts at `start`, pointing `heading` radians counter-clockwise from +x.
    #[new]
    #[pyo3(signature = (start, heading = 0.0))]
    fn new(start: (f64, f64, f64), heading: f64) -> Self {
        PyAlignment {
            inner: Alignment::new(point(start), heading),
        }
    }

    /// Appends `length` metres of straight, climbing `rise` metres.
    #[pyo3(signature = (length, rise = 0.0))]
    fn line(&mut self, length: f64, rise: f64) -> PyResult<()> {
        self.step(|alignment| alignment.line(length, rise))
    }

    /// Appends `length` metres of bend at `curvature` per metre, positive turning
    /// left, climbing `rise` metres.
    #[pyo3(signature = (length, curvature, rise = 0.0))]
    fn arc(&mut self, length: f64, curvature: f64, rise: f64) -> PyResult<()> {
        self.step(|alignment| alignment.arc(length, curvature, rise))
    }

    /// Appends `length` metres of transition, curving from wherever the alignment
    /// currently curves to `curvature_end`, and climbing `rise` metres.
    #[pyo3(signature = (length, curvature_end, rise = 0.0))]
    fn spiral(&mut self, length: f64, curvature_end: f64, rise: f64) -> PyResult<()> {
        self.step(|alignment| alignment.spiral(length, curvature_end, rise))
    }

    /// Where the alignment has reached.
    #[getter]
    fn point(&self) -> (f64, f64, f64) {
        let point = self.inner.point();
        (point.x, point.y, point.z)
    }

    /// The heading it is pointing in, radians.
    #[getter]
    fn heading(&self) -> f64 {
        self.inner.heading()
    }

    /// The curvature it is turning at, per metre.
    #[getter]
    fn curvature(&self) -> f64 {
        self.inner.curvature()
    }

    fn __repr__(&self) -> String {
        let point = self.inner.point();
        format!(
            "Alignment(at=({:.3}, {:.3}, {:.3}), heading={:.4}, curvature={:.6})",
            point.x,
            point.y,
            point.z,
            self.inner.heading(),
            self.inner.curvature()
        )
    }
}

impl PyAlignment {
    /// Applies one of the Rust builder's steps, which consume and return the
    /// alignment, to the one held here.
    fn step(
        &mut self,
        advance: impl FnOnce(Alignment) -> Result<Alignment, roadgen_core::GeometryError>,
    ) -> PyResult<()> {
        self.inner = advance(self.inner.clone()).map_err(value_error)?;
        Ok(())
    }

    /// The curve so far. Cloned, so the alignment can be extended afterwards and
    /// used for another road.
    fn curve(&self) -> PyResult<Curve3> {
        self.inner.clone().finish().map_err(value_error)
    }
}

/// A road network under construction.
///
/// The map builds itself the first time something asks for a result — exporting,
/// validating or inspecting — and rebuilds after any further change.
#[pyclass(name = "Map", module = "roadgen")]
pub struct PyMap {
    builder: MapBuilder,
    built: Option<ValidatedMap>,
}

#[pymethods]
impl PyMap {
    /// `origin` is `(latitude, longitude, altitude)` and anchors the map's metric
    /// coordinates; `handedness` decides which side of the reference line a
    /// `forward` lane sits on.
    #[new]
    #[pyo3(signature = (
        name = None,
        origin = None,
        projection = "local_cartesian",
        handedness = "rht",
        sampling = 2.0,
    ))]
    fn new(
        name: Option<String>,
        origin: Option<(f64, f64, f64)>,
        projection: &str,
        handedness: &str,
        sampling: f64,
    ) -> PyResult<Self> {
        let metadata = MapMetadata {
            name,
            origin: match origin {
                Some((latitude, longitude, altitude)) => {
                    GeoOrigin::new(latitude, longitude, altitude).map_err(value_error)?
                }
                None => GeoOrigin::default(),
            },
            projection: Projection::parse(projection).ok_or_else(|| {
                PyValueError::new_err(format!("unknown projection {projection:?}"))
            })?,
            handedness: TrafficHandedness::parse(handedness).ok_or_else(|| {
                PyValueError::new_err(format!("unknown traffic handedness {handedness:?}"))
            })?,
            sampling: SamplingConfig::new(sampling).map_err(value_error)?,
        };
        Ok(PyMap {
            builder: MapBuilder::new(metadata),
            built: None,
        })
    }

    /// Adds a road: straight (`start` and `end`), along `points`, or following an
    /// `alignment` of lines, bends and transitions.
    #[pyo3(signature = (
        lanes,
        start = None,
        end = None,
        points = None,
        alignment = None,
        name = None,
        type_ = "town",
        speed_limit_kph = None,
        superelevation = None,
    ))]
    #[allow(clippy::too_many_arguments)]
    fn add_road(
        &mut self,
        lanes: Vec<PyLane>,
        start: Option<(f64, f64, f64)>,
        end: Option<(f64, f64, f64)>,
        points: Option<Vec<(f64, f64, f64)>>,
        alignment: Option<&PyAlignment>,
        name: Option<String>,
        type_: &str,
        speed_limit_kph: Option<f64>,
        superelevation: Option<Vec<(f64, f64)>>,
    ) -> PyResult<PyRoad> {
        let reference_line = match (start, end, points, alignment) {
            (Some(start), Some(end), None, None) => {
                Curve3::line(point(start), point(end)).map_err(value_error)?
            }
            (None, None, Some(points), None) => {
                Curve3::polyline(points.into_iter().map(point)).map_err(value_error)?
            }
            (None, None, None, Some(alignment)) => alignment.curve()?,
            _ => {
                return Err(PyValueError::new_err(
                    "pass exactly one of start= with end=, points=, or alignment=",
                ))
            }
        };
        let mut spec = RoadSpec::new(
            reference_line,
            lanes.into_iter().map(|lane| lane.spec).collect(),
        )
        .with_type(
            RoadType::parse(type_)
                .ok_or_else(|| PyValueError::new_err(format!("unknown road type {type_:?}")))?,
        );
        if let Some(name) = name {
            spec = spec.with_name(name);
        }
        if let Some(limit) = speed_limit_kph {
            spec = spec.with_speed_limit(SpeedLimit::from_kph(limit).map_err(value_error)?);
        }
        if let Some(points) = superelevation {
            // `(station, radians)` pairs, straight between them and flat outside —
            // the shape a caller describes a bank in.
            spec = spec
                .with_superelevation(Poly3Profile::piecewise_linear(points).map_err(value_error)?);
        }
        let lane_count = spec.lanes.len();
        let id = self.builder.add_road(spec).map_err(value_error)?;
        self.invalidate();
        Ok(PyRoad {
            id,
            lanes: lane_count,
        })
    }

    #[pyo3(signature = (name = None))]
    fn add_junction(&mut self, name: Option<&str>) -> PyJunction {
        let id = self.builder.add_junction(name);
        self.invalidate();
        PyJunction { id }
    }

    /// Joins the end of `a` to the start of `b`, pairing lanes across the joint.
    ///
    /// Returns the movements it created, as `(from_lane_id, to_lane_id)` pairs.
    #[pyo3(signature = (a, b, junction = None))]
    fn connect(
        &mut self,
        a: &PyRoad,
        b: &PyRoad,
        junction: Option<&PyJunction>,
    ) -> PyResult<Vec<(String, String)>> {
        let movements = match junction {
            Some(junction) => self.builder.connect_via(&junction.id, &a.id, &b.id),
            None => self.builder.connect(&a.id, &b.id),
        }
        .map_err(value_error)?;
        self.invalidate();
        Ok(movements
            .into_iter()
            .map(|(from, to)| (from.to_string(), to.to_string()))
            .collect())
    }

    /// Connects one specific lane to one specific lane.
    #[pyo3(signature = (from_lane, to_lane, junction = None))]
    fn connect_lanes(
        &mut self,
        from_lane: &PyLaneRef,
        to_lane: &PyLaneRef,
        junction: Option<&PyJunction>,
    ) -> PyResult<String> {
        let id = self
            .builder
            .connect_lanes(
                &from_lane.reference,
                &to_lane.reference,
                junction.map(|junction| &junction.id),
            )
            .map_err(value_error)?;
        self.invalidate();
        Ok(id.to_string())
    }

    /// Adds a stop line across a lane.
    #[pyo3(signature = (lane, end = "end"))]
    fn add_stop_line(&mut self, lane: &PyLaneRef, end: &str) -> PyResult<String> {
        let id = self
            .builder
            .add_stop_line(&lane.reference, parse_end(end)?)
            .map_err(value_error)?;
        self.invalidate();
        Ok(id.to_string())
    }

    /// Adds a traffic light bar above a lane.
    #[pyo3(signature = (lane, end = "end", height = 5.0))]
    fn add_traffic_light(&mut self, lane: &PyLaneRef, end: &str, height: f64) -> PyResult<String> {
        let id = self
            .builder
            .add_traffic_light(&lane.reference, parse_end(end)?, height)
            .map_err(value_error)?;
        self.invalidate();
        Ok(id.to_string())
    }

    /// Adds a traffic sign beside a lane. `code` becomes the sign's subtype.
    #[pyo3(signature = (lane, code, end = "end", height = 2.5))]
    fn add_traffic_sign(
        &mut self,
        lane: &PyLaneRef,
        code: String,
        end: &str,
        height: f64,
    ) -> PyResult<String> {
        let id = self
            .builder
            .add_traffic_sign(&lane.reference, parse_end(end)?, code, height)
            .map_err(value_error)?;
        self.invalidate();
        Ok(id.to_string())
    }

    /// Adds a crosswalk across a road, `fraction` of the way along it.
    #[pyo3(signature = (road, fraction = 0.5, width = 4.0))]
    fn add_crosswalk(&mut self, road: &PyRoad, fraction: f64, width: f64) -> PyResult<String> {
        let id = self
            .builder
            .add_crosswalk(&road.id, fraction, width)
            .map_err(value_error)?;
        self.invalidate();
        Ok(id.to_string())
    }

    /// Records that `lanes` are controlled by the given traffic lights.
    #[pyo3(signature = (lights, lanes, stop_line = None))]
    fn add_traffic_light_rule(
        &mut self,
        lights: Vec<String>,
        lanes: Vec<PyLaneRef>,
        stop_line: Option<String>,
    ) {
        self.builder.add_traffic_light_rule(
            lights.into_iter().map(ObjectId::from_raw).collect(),
            stop_line.map(ObjectId::from_raw),
            lanes.into_iter().map(|lane| lane.reference).collect(),
        );
        self.invalidate();
    }

    /// Records that `yielding` gives way to `right_of_way`.
    #[pyo3(signature = (right_of_way, yielding, stop_line = None))]
    fn add_right_of_way(
        &mut self,
        right_of_way: Vec<PyLaneRef>,
        yielding: Vec<PyLaneRef>,
        stop_line: Option<String>,
    ) {
        self.builder.add_right_of_way(
            right_of_way
                .into_iter()
                .map(|lane| lane.reference)
                .collect(),
            yielding.into_iter().map(|lane| lane.reference).collect(),
            stop_line.map(ObjectId::from_raw),
        );
        self.invalidate();
    }

    /// Applies a speed limit to a set of lanes.
    fn add_speed_limit(&mut self, lanes: Vec<PyLaneRef>, kph: f64) -> PyResult<()> {
        self.builder.add_speed_limit_rule(
            SpeedLimit::from_kph(kph).map_err(value_error)?,
            lanes.into_iter().map(|lane| lane.reference).collect(),
        );
        self.invalidate();
        Ok(())
    }

    /// Builds and validates the map, raising if anything is wrong.
    fn validate(&mut self) -> PyResult<()> {
        self.ensure_built()
    }

    /// Everything wrong with the map, as text. Empty when the map is valid.
    fn issues(&self) -> PyResult<Vec<String>> {
        let unvalidated = self.build()?;
        Ok(unvalidated
            .issues(Default::default())
            .into_iter()
            .map(|issue| issue.to_string())
            .collect())
    }

    /// Constraints the two output formats impose beyond validation.
    fn format_warnings(&mut self) -> PyResult<Vec<String>> {
        self.ensure_built()?;
        let map = self.built.as_ref().expect("just built");
        let mut warnings = roadgen_opendrive::check(map);
        warnings.extend(roadgen_lanelet2::check(map));
        Ok(warnings)
    }

    /// Writes the map as OpenDRIVE.
    fn export_opendrive(&mut self, path: PathBuf) -> PyResult<()> {
        self.ensure_built()?;
        roadgen_opendrive::write(self.built.as_ref().expect("just built"), path)
            .map_err(runtime_error)
    }

    /// Writes the map as a Lanelet2 OSM file.
    fn export_lanelet2(&mut self, path: PathBuf) -> PyResult<()> {
        self.ensure_built()?;
        roadgen_lanelet2::write(self.built.as_ref().expect("just built"), path)
            .map_err(runtime_error)
    }

    // `&mut self` because the map builds itself on demand; the name is the one the
    // Python API exposes, so it stays as it reads from Python.
    /// The OpenDRIVE document as a string.
    #[allow(clippy::wrong_self_convention)]
    fn to_opendrive_xml(&mut self) -> PyResult<String> {
        self.ensure_built()?;
        roadgen_opendrive::to_xml(self.built.as_ref().expect("just built")).map_err(runtime_error)
    }

    /// The Lanelet2 map as OSM XML.
    #[allow(clippy::wrong_self_convention)]
    fn to_lanelet2_osm(&mut self) -> PyResult<String> {
        self.ensure_built()?;
        roadgen_lanelet2::to_osm_xml(self.built.as_ref().expect("just built"))
            .map_err(runtime_error)
    }

    /// Identifiers of every road, including generated junction connectors.
    fn road_ids(&mut self) -> PyResult<Vec<String>> {
        self.ensure_built()?;
        Ok(self
            .built
            .as_ref()
            .expect("just built")
            .roads
            .iter()
            .map(|road| road.id.to_string())
            .collect())
    }

    /// Identifiers of every lane.
    fn lane_ids(&mut self) -> PyResult<Vec<String>> {
        self.ensure_built()?;
        Ok(self
            .built
            .as_ref()
            .expect("just built")
            .lanes
            .iter()
            .map(|lane| lane.id.to_string())
            .collect())
    }

    /// Every movement in the map, as `(from_lane_id, to_lane_id)` pairs.
    fn connections(&mut self) -> PyResult<Vec<(String, String)>> {
        self.ensure_built()?;
        Ok(self
            .built
            .as_ref()
            .expect("just built")
            .connections
            .iter()
            .map(|connection| {
                (
                    connection.from.lane.to_string(),
                    connection.to.lane.to_string(),
                )
            })
            .collect())
    }

    /// Lanes reachable in one step from the given lane.
    fn successors(&mut self, lane: String) -> PyResult<Vec<String>> {
        self.ensure_built()?;
        Ok(self
            .built
            .as_ref()
            .expect("just built")
            .successors(&LaneId::from_raw(lane))
            .into_iter()
            .map(|lane| lane.to_string())
            .collect())
    }

    /// The vertices of a lane's centreline, in travel order.
    fn lane_centerline(&mut self, lane: String) -> PyResult<Vec<(f64, f64, f64)>> {
        self.ensure_built()?;
        let map = self.built.as_ref().expect("just built");
        let lane = map
            .lane(&LaneId::from_raw(lane.clone()))
            .ok_or_else(|| PyValueError::new_err(format!("no such lane: {lane}")))?;
        let travel = lane
            .travel_geometry(map.metadata.sampling)
            .map_err(value_error)?;
        Ok(travel
            .centerline
            .to_polyline(map.metadata.sampling)
            .map_err(value_error)?
            .points()
            .iter()
            .map(|point| (point.x, point.y, point.z))
            .collect())
    }

    fn __repr__(&self) -> String {
        format!(
            "Map(name={:?}, projection={:?}, handedness={:?})",
            self.builder.metadata().name,
            self.builder.metadata().projection.as_str(),
            self.builder.metadata().handedness.as_str()
        )
    }
}

impl PyMap {
    fn invalidate(&mut self) {
        self.built = None;
    }

    fn build(&self) -> PyResult<UnvalidatedMap> {
        self.builder.clone().finish().map_err(value_error)
    }

    fn ensure_built(&mut self) -> PyResult<()> {
        if self.built.is_none() {
            self.built = Some(self.build()?.validate().map_err(value_error)?);
        }
        Ok(())
    }
}

#[pymodule]
fn _roadgen(module: &Bound<'_, PyModule>) -> PyResult<()> {
    module.add(
        "__doc__",
        "Rust core of the roadgen road-network generator.",
    )?;
    module.add("__version__", env!("CARGO_PKG_VERSION"))?;
    module.add_class::<PyMap>()?;
    module.add_class::<PyLane>()?;
    module.add_class::<PyRoad>()?;
    module.add_class::<PyJunction>()?;
    module.add_class::<PyLaneRef>()?;
    module.add_class::<PyAlignment>()?;
    Ok(())
}
