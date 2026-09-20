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
use pyo3::types::PyDict;

use roadgen_core::builder::CrossSectionSpec;
use roadgen_core::builder::{LaneRef, LaneSpec, MapBuilder, RoadSpec};
use roadgen_core::geometry::{
    Alignment, Curve3, Point3, Poly3Profile, SamplingConfig, Taper, WidthProfile,
};
use roadgen_core::id::{BuildingId, BuildingPartId, JunctionId, LaneId, ObjectId, RoadId};
use roadgen_core::map::{MapMetadata, Projection, TrafficHandedness};
use roadgen_core::semantics::{BoundaryMarking, LaneType, MarkingColor, RoadMarking, RoadType};
use roadgen_core::topology::{Direction, LaneEnd, LateralSide, RoadEnd};
use roadgen_core::units::{GeoOrigin, PositiveWidth, SpeedLimit};
use roadgen_core::validation::{UnvalidatedMap, ValidatedMap};

use roadgen_buildings::Rules as BuildingRules;

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

fn parse_road_end(value: &str) -> PyResult<RoadEnd> {
    match value.to_ascii_lowercase().as_str() {
        "start" => Ok(RoadEnd::Start),
        "end" => Ok(RoadEnd::End),
        other => Err(PyValueError::new_err(format!(
            "a road end must be 'start' or 'end', got {other:?}"
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

fn parse_taper(value: &str) -> PyResult<Taper> {
    match value.to_ascii_lowercase().as_str() {
        "linear" => Ok(Taper::Linear),
        "smooth" | "ease" => Ok(Taper::Smooth),
        other => Err(PyValueError::new_err(format!(
            "taper must be 'linear' or 'smooth', got {other:?}"
        ))),
    }
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
        width_profile = None,
        taper = "linear",
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
        width_profile: Option<Vec<(f64, f64)>>,
        taper: &str,
    ) -> PyResult<Self> {
        // `width_profile` is `(station, metres)` pairs measured along the road, for a
        // lane that narrows or widens; `width` alone is the same width throughout.
        let profile = match width_profile {
            None => WidthProfile::constant(PositiveWidth::new(width).map_err(value_error)?),
            Some(knots) => WidthProfile::new(
                knots
                    .into_iter()
                    .map(|(station, metres)| {
                        Ok((station, PositiveWidth::new(metres).map_err(value_error)?))
                    })
                    .collect::<PyResult<Vec<_>>>()?,
                parse_taper(taper)?,
            )
            .map_err(value_error)?,
        };
        let mut spec = LaneSpec::new(profile, parse_direction(direction)?)
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

    /// The lane's width where it starts. A tapering lane is narrower or wider
    /// elsewhere; `width_profile` gives the whole of it.
    #[getter]
    fn width(&self) -> f64 {
        self.spec
            .width
            .evaluate(self.spec.width.knots()[0].0)
            .metres()
    }

    /// The lane's width as `(station, metres)` pairs.
    #[getter]
    fn width_profile(&self) -> Vec<(f64, f64)> {
        self.spec
            .width
            .knots()
            .iter()
            .map(|(station, width)| (*station, width.metres()))
            .collect()
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
            self.width(),
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

    /// Every lane of this road, section by section.
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
    /// A map that was read from a file rather than built: the map itself, and what
    /// the reading could not keep. A map has one source, so while this is set the
    /// builder is empty and the methods that would add to it refuse.
    imported: Option<(UnvalidatedMap, Vec<String>)>,
    /// The rules buildings are generated by, or `None` when they are switched off.
    /// Held rather than applied, because a lot is cut out of generated geometry and
    /// there is none until the map is built.
    buildings: Option<BuildingRules>,
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
            imported: None,
            buildings: None,
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
        cross_sections = None,
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
        cross_sections: Option<Vec<(f64, Vec<PyLane>)>>,
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
        // Further cross-sections, each taking over at its station. Use these where
        // the *number* of lanes changes; a lane that only tapers stays in one
        // cross-section and carries a width profile instead.
        for (station, section_lanes) in cross_sections.unwrap_or_default() {
            if section_lanes.is_empty() {
                return Err(PyValueError::new_err(format!(
                    "the cross-section at station {station} has no lanes"
                )));
            }
            spec.cross_sections.push(CrossSectionSpec {
                station,
                lanes: section_lanes.into_iter().map(|lane| lane.spec).collect(),
            });
        }
        if let Some(points) = superelevation {
            // `(station, radians)` pairs, straight between them and flat outside —
            // the shape a caller describes a bank in.
            spec = spec
                .with_superelevation(Poly3Profile::piecewise_linear(points).map_err(value_error)?);
        }
        let lane_count = spec.all_lanes().count();
        let id = self.builder_mut()?.add_road(spec).map_err(value_error)?;
        self.invalidate();
        Ok(PyRoad {
            id,
            lanes: lane_count,
        })
    }

    #[pyo3(signature = (name = None))]
    fn add_junction(&mut self, name: Option<&str>) -> PyResult<PyJunction> {
        let id = self.builder_mut()?.add_junction(name);
        self.invalidate();
        Ok(PyJunction { id })
    }

    /// Joins two roads, pairing lanes across the joint.
    ///
    /// `ends` says which end of each road meets, and defaults to the end of `a`
    /// meeting the start of `b` — one road carrying on into the next. A junction
    /// whose approaches all point at the centre meets end to end instead, which
    /// only `ends` can say.
    ///
    /// Returns the movements it created, as `(from_lane_id, to_lane_id)` pairs.
    #[pyo3(signature = (a, b, junction = None, ends = ("end", "start")))]
    fn connect(
        &mut self,
        a: &PyRoad,
        b: &PyRoad,
        junction: Option<&PyJunction>,
        ends: (&str, &str),
    ) -> PyResult<Vec<(String, String)>> {
        let movements = self
            .builder_mut()?
            .connect_ends(
                &a.id,
                parse_road_end(ends.0)?,
                &b.id,
                parse_road_end(ends.1)?,
                junction.map(|junction| &junction.id),
            )
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
            .builder_mut()?
            .connect_lanes(
                &from_lane.reference,
                &to_lane.reference,
                junction.map(|junction| &junction.id),
            )
            .map_err(value_error)?;
        self.invalidate();
        Ok(id.to_string())
    }

    /// Adds a stop line across a lane, `setback` metres back from the end named —
    /// before the crosswalk at a junction mouth, say.
    #[pyo3(signature = (lane, end = "end", setback = 0.0))]
    fn add_stop_line(&mut self, lane: &PyLaneRef, end: &str, setback: f64) -> PyResult<String> {
        let id = self
            .builder_mut()?
            .add_stop_line_at(&lane.reference, parse_end(end)?, setback)
            .map_err(value_error)?;
        self.invalidate();
        Ok(id.to_string())
    }

    /// Adds a traffic light bar above a lane, `setback` metres back from the end
    /// named.
    #[pyo3(signature = (lane, end = "end", height = 5.0, setback = 0.0))]
    fn add_traffic_light(
        &mut self,
        lane: &PyLaneRef,
        end: &str,
        height: f64,
        setback: f64,
    ) -> PyResult<String> {
        let id = self
            .builder_mut()?
            .add_traffic_light_at(&lane.reference, parse_end(end)?, height, setback)
            .map_err(value_error)?;
        self.invalidate();
        Ok(id.to_string())
    }

    /// Adds a traffic sign beside a lane, `setback` metres back from the end named.
    /// `code` becomes the sign's subtype.
    #[pyo3(signature = (lane, code, end = "end", height = 2.5, setback = 0.0))]
    fn add_traffic_sign(
        &mut self,
        lane: &PyLaneRef,
        code: String,
        end: &str,
        height: f64,
        setback: f64,
    ) -> PyResult<String> {
        let id = self
            .builder_mut()?
            .add_traffic_sign_at(&lane.reference, parse_end(end)?, code, height, setback)
            .map_err(value_error)?;
        self.invalidate();
        Ok(id.to_string())
    }

    /// Adds a crosswalk across a road, `fraction` of the way along it.
    #[pyo3(signature = (road, fraction = 0.5, width = 4.0))]
    fn add_crosswalk(&mut self, road: &PyRoad, fraction: f64, width: f64) -> PyResult<String> {
        let id = self
            .builder_mut()?
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
    ) -> PyResult<()> {
        self.builder_mut()?.add_traffic_light_rule(
            lights.into_iter().map(ObjectId::from_raw).collect(),
            stop_line.map(ObjectId::from_raw),
            lanes.into_iter().map(|lane| lane.reference).collect(),
        );
        self.invalidate();
        Ok(())
    }

    /// Records that `yielding` gives way to `right_of_way`.
    #[pyo3(signature = (right_of_way, yielding, stop_line = None))]
    fn add_right_of_way(
        &mut self,
        right_of_way: Vec<PyLaneRef>,
        yielding: Vec<PyLaneRef>,
        stop_line: Option<String>,
    ) -> PyResult<()> {
        self.builder_mut()?.add_right_of_way(
            right_of_way
                .into_iter()
                .map(|lane| lane.reference)
                .collect(),
            yielding.into_iter().map(|lane| lane.reference).collect(),
            stop_line.map(ObjectId::from_raw),
        );
        self.invalidate();
        Ok(())
    }

    /// Applies a speed limit to a set of lanes.
    fn add_speed_limit(&mut self, lanes: Vec<PyLaneRef>, kph: f64) -> PyResult<()> {
        self.builder_mut()?.add_speed_limit_rule(
            SpeedLimit::from_kph(kph).map_err(value_error)?,
            lanes.into_iter().map(|lane| lane.reference).collect(),
        );
        self.invalidate();
        Ok(())
    }

    /// Switches automatic building generation on or off, and says by what rules.
    ///
    /// Off by default. Switched on, every road gets two frontages of lots and a CGA
    /// shape grammar decides what stands on each — so the same map, the same rules
    /// and the same seed always produce the same town, and no building ever reaches
    /// into a road or onto a neighbour.
    ///
    /// `rules` is either the name of a built-in set — `roadgen.building_presets()`
    /// lists them — or a grammar written out in full, which
    /// `roadgen.building_rules(name)` hands back so it can be edited rather than
    /// written from nothing. The grammar is checked here rather than at export, so a
    /// typo in it raises on this line.
    ///
    /// ```text
    /// m.generate_buildings()                        # on, mixed town street
    /// m.generate_buildings(rules="downtown")        # on, a built-in set
    /// m.generate_buildings(rules=text, seed=7)      # on, rules of your own
    /// m.generate_buildings(False)                   # off
    /// ```
    #[pyo3(signature = (enabled = true, rules = None, seed = 0))]
    fn generate_buildings(
        &mut self,
        enabled: bool,
        rules: Option<&str>,
        seed: u64,
    ) -> PyResult<()> {
        self.invalidate();
        if !enabled {
            self.buildings = None;
            return Ok(());
        }
        let rules = match rules {
            Some(text) => BuildingRules::parse(text).map_err(value_error)?,
            None => BuildingRules::default(),
        }
        .with_seed(seed);
        // Compiled now so that a grammar error lands on the call that set it rather
        // than on whichever export happened to build the map first.
        rules.compile().map_err(value_error)?;
        self.buildings = Some(rules);
        Ok(())
    }

    /// The identifier of every building, in the order they were generated.
    ///
    /// Empty when generation is switched off, which is the default.
    fn building_ids(&mut self) -> PyResult<Vec<String>> {
        self.ensure_built()?;
        Ok(self
            .built
            .as_ref()
            .expect("just built")
            .buildings
            .iter()
            .map(|building| building.id.to_string())
            .collect())
    }

    /// Which road a building faces, as `(road, side, station_metres)`.
    ///
    /// `None` for a building that was put on the map without one, which a generated
    /// town never is.
    fn building_frontage(&mut self, building: String) -> PyResult<Option<(String, String, f64)>> {
        let entry = self.find_building(&building)?;
        Ok(entry.frontage.as_ref().map(|frontage| {
            (
                frontage.road.to_string(),
                frontage.side.as_str().to_owned(),
                frontage.station,
            )
        }))
    }

    /// What a building is, as the word the rules emitted it with.
    fn building_kind(&mut self, building: String) -> PyResult<String> {
        Ok(self.find_building(&building)?.kind.clone())
    }

    /// The parts a building is made of, lowest first.
    ///
    /// A building is not a shape; its parts are. One with a single part is the common
    /// case and not a special one.
    fn building_parts(&mut self, building: String) -> PyResult<Vec<String>> {
        Ok(self
            .find_building(&building)?
            .parts
            .iter()
            .map(|part| part.to_string())
            .collect())
    }

    /// One part's outline, as `(x, y, z)` metres, anticlockwise and not closed.
    ///
    /// The vertices carry their own heights: a part that starts partway up a building
    /// has an outline that starts there, and one on sloping ground is not level.
    fn building_footprint(&mut self, part: String) -> PyResult<Vec<(f64, f64, f64)>> {
        Ok(self
            .find_part(&part)?
            .solid
            .footprint
            .points()
            .iter()
            .map(|point| (point.x, point.y, point.z))
            .collect())
    }

    /// One part's `(base_height, wall_height, roof_shape, roof_height,
    /// roof_direction_radians, levels)`. Every length is in metres.
    fn building_part_shape(&mut self, part: String) -> PyResult<(f64, f64, String, f64, f64, u32)> {
        let entry = self.find_part(&part)?;
        Ok((
            entry.solid.base_height(),
            entry.solid.wall_height,
            entry.solid.roof.shape.as_str().to_owned(),
            entry.solid.roof.height,
            entry.solid.roof.direction,
            entry.levels,
        ))
    }

    /// What one part is, when it is something its building as a whole is not — the
    /// offices above a shopping podium. `None` when it is simply part of it.
    fn building_part_kind(&mut self, part: String) -> PyResult<Option<String>> {
        Ok(self.find_part(&part)?.kind.clone())
    }

    /// The faces bounding one part, each a list of `(x, y, z)` metres anticlockwise
    /// seen from outside: the base, one per wall, and the roof.
    ///
    /// This is the solid itself rather than a description of it — the outline and the
    /// heights are the compact form, and this is what they mean.
    fn building_shell(&mut self, part: String) -> PyResult<Vec<Vec<(f64, f64, f64)>>> {
        Ok(self
            .find_part(&part)?
            .solid
            .shell()
            .into_iter()
            .map(|face| {
                face.into_iter()
                    .map(|point| (point.x, point.y, point.z))
                    .collect()
            })
            .collect())
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

    /// The MGRS grid square the map's coordinates are reported in, or `None` if it
    /// does not use the MGRS projection.
    ///
    /// This is the reference Autoware's `map_projector_info` needs alongside the map.
    fn mgrs_grid(&mut self) -> PyResult<Option<String>> {
        self.ensure_built()?;
        Ok(
            roadgen_lanelet2::grid_for(self.built.as_ref().expect("just built"))
                .map_err(runtime_error)?
                .map(|grid| grid.code().to_owned()),
        )
    }

    /// What the file this map was read from said more exactly than the map does.
    ///
    /// Empty for a map that was built here, and for one read from a file roadgen
    /// wrote from a map the IR could hold. Everything else — a width that was a free
    /// cubic, a lane type the IR does not have, a roof flattened to its height — is
    /// one line each, the way `format_warnings` reports what an export loses.
    fn read_warnings(&self) -> Vec<String> {
        self.imported
            .as_ref()
            .map(|(_, notes)| notes.clone())
            .unwrap_or_default()
    }

    /// Constraints OpenDRIVE and Lanelet2 impose beyond validation.
    fn format_warnings(&mut self) -> PyResult<Vec<String>> {
        self.ensure_built()?;
        let map = self.built.as_ref().expect("just built");
        let mut warnings = roadgen_opendrive::check(map);
        warnings.extend(roadgen_lanelet2::check(map));
        Ok(warnings)
    }

    /// What a ClipGT export loses, and what its scenario leaves out.
    ///
    /// Kept apart from `format_warnings` because these are properties of the format
    /// rather than of the map: ClipGT carries no topology, so any map with a
    /// connection in it reports one, and folding that into the general warnings would
    /// make them noise for a caller who never touches ClipGT. Pass the scenario to
    /// have its route and rig checked too.
    #[pyo3(signature = (scenario = None))]
    fn clipgt_warnings(&mut self, scenario: Option<PathBuf>) -> PyResult<Vec<String>> {
        let config = match scenario {
            Some(path) => Some(self.clipgt_config(Some(path), None, None, None, None)?),
            None => {
                self.ensure_built()?;
                None
            }
        };
        let map = self.built.as_ref().expect("just built");
        Ok(roadgen_clipgt::check(map, config.as_ref()))
    }

    /// What a GPUDrive export loses, and what its scene runs up against.
    ///
    /// Kept apart from `format_warnings` for the same reason as the ClipGT ones:
    /// these are properties of the format — GPUDrive is a plane, and carries no
    /// topology — so every map reports them, and folding them into the general
    /// warnings would make them noise for a caller who never touches GPUDrive. Pass
    /// the scenario to have the agents, their routes and the simulator's own limits
    /// checked too.
    #[pyo3(signature = (scenario = None))]
    fn gpudrive_warnings(&mut self, scenario: Option<PathBuf>) -> PyResult<Vec<String>> {
        let config = match scenario {
            Some(path) => {
                Some(self.gpudrive_config(Some(path), None, None, None, None, None, None)?)
            }
            None => {
                self.ensure_built()?;
                None
            }
        };
        let map = self.built.as_ref().expect("just built");
        Ok(roadgen_gpudrive::check(map, config.as_ref()))
    }

    /// Writes the map as OpenDRIVE.
    fn export_opendrive(&mut self, path: PathBuf) -> PyResult<()> {
        self.ensure_built()?;
        roadgen_opendrive::write(self.built.as_ref().expect("just built"), path)
            .map_err(runtime_error)
    }

    /// Writes the map as a Lanelet2 OSM file.
    ///
    /// This is Lanelet2's *use* of the OSM container — ways are lane boundaries and
    /// relations are lanelets. For a file an OSM router or renderer understands, use
    /// `export_osm`.
    fn export_lanelet2(&mut self, path: PathBuf) -> PyResult<()> {
        self.ensure_built()?;
        roadgen_lanelet2::write(self.built.as_ref().expect("just built"), path)
            .map_err(runtime_error)
    }

    /// Writes the map as a plain OpenStreetMap file.
    ///
    /// One `highway` way per road, junctions as shared nodes, and turn restrictions
    /// for the movements the map does not permit. Both this and `export_lanelet2`
    /// write `.osm`; they are not interchangeable.
    fn export_osm(&mut self, path: PathBuf) -> PyResult<()> {
        self.ensure_built()?;
        roadgen_osm::write(self.built.as_ref().expect("just built"), path).map_err(runtime_error)
    }

    /// What a plain OpenStreetMap export loses.
    fn osm_warnings(&mut self) -> PyResult<Vec<String>> {
        self.ensure_built()?;
        Ok(roadgen_osm::check(self.built.as_ref().expect("just built")))
    }

    /// Writes the map as a SUMO network: a directory of plain-XML files, plus the
    /// `netconvert` configuration that builds them into a `.net.xml`.
    ///
    /// Returns the prefix the files were named with, which is the map's own name
    /// reduced to something a file name and a SUMO identifier can both hold.
    ///
    /// ```text
    /// prefix = m.export_sumo("network/")
    /// subprocess.run(["netconvert", "-c", f"network/{prefix}.netccfg"])
    /// ```
    ///
    /// The `.net.xml` is deliberately not written here: building it is netconvert's
    /// job, it carries the shape of every junction and the right-of-way matrix, and
    /// producing one without netconvert would mean reimplementing it.
    fn export_sumo(&mut self, directory: PathBuf) -> PyResult<String> {
        self.ensure_built()?;
        roadgen_sumo::write(self.built.as_ref().expect("just built"), directory)
            .map_err(runtime_error)
    }

    /// What a SUMO export loses.
    fn sumo_warnings(&mut self) -> PyResult<Vec<String>> {
        self.ensure_built()?;
        Ok(roadgen_sumo::check(
            self.built.as_ref().expect("just built"),
        ))
    }

    /// Where each lane of the map ended up in the SUMO network, as the
    /// `<edge>_<index>` identifier the built network gives it.
    ///
    /// A SUMO lane has no name of its own — it is the n-th lane of an edge — and the
    /// numbering is not the IR's: SUMO counts from the right of the direction of
    /// travel, and a two-way road is two edges. So this is the only way back from a
    /// lane of the map to a lane of the network.
    fn sumo_lane_ids(&mut self) -> PyResult<Vec<(String, String)>> {
        self.ensure_built()?;
        let network = roadgen_sumo::to_plain_xml(self.built.as_ref().expect("just built"))
            .map_err(runtime_error)?;
        Ok(network
            .lanes
            .into_iter()
            .map(|(lane, written)| (lane.as_str().to_owned(), written))
            .collect())
    }

    /// Writes the map as a ClipGT clip: a directory of per-layer parquet files.
    ///
    /// Returns the clip id the files were named with, which is `clip_id` when given
    /// and otherwise the map's own name reduced to something a file name can hold.
    ///
    /// `scenario` is the path to a YAML file naming the ego route and the camera rig
    /// — the two parts of a clip that are not the map. Everything after it overrides
    /// what the file says, and everything left as `None` keeps it, so a scenario can
    /// be used as written or nudged in one place.
    #[pyo3(signature = (
        directory,
        scenario = None,
        clip_id = None,
        frame_rate = None,
        speed = None,
        route = None,
    ))]
    fn export_clipgt(
        &mut self,
        directory: PathBuf,
        scenario: Option<PathBuf>,
        clip_id: Option<&str>,
        frame_rate: Option<f64>,
        speed: Option<f64>,
        route: Option<Vec<String>>,
    ) -> PyResult<String> {
        let config = self.clipgt_config(scenario, clip_id, frame_rate, speed, route)?;
        let map = self.built.as_ref().expect("just built");
        roadgen_clipgt::write(map, directory, &config).map_err(runtime_error)
    }

    /// Writes the map as a GPUDrive scene: one JSON file holding the map as
    /// polylines and the agents driving it as tracks.
    ///
    /// `scenario` is the path to a YAML file naming the agents — the part of a scene
    /// that is not the map. Everything after it overrides what the file says, and
    /// everything left as `None` keeps it, so a scenario can be used as written or
    /// nudged in one place. `speed` and `route` apply to the first agent, which is
    /// the scene's self-driving car.
    #[pyo3(signature = (
        path,
        scenario = None,
        name = None,
        scenario_id = None,
        steps = None,
        time_step = None,
        speed = None,
        route = None,
    ))]
    #[allow(clippy::too_many_arguments)]
    fn export_gpudrive(
        &mut self,
        path: PathBuf,
        scenario: Option<PathBuf>,
        name: Option<&str>,
        scenario_id: Option<&str>,
        steps: Option<usize>,
        time_step: Option<f64>,
        speed: Option<f64>,
        route: Option<Vec<String>>,
    ) -> PyResult<()> {
        let config =
            self.gpudrive_config(scenario, name, scenario_id, steps, time_step, speed, route)?;
        let map = self.built.as_ref().expect("just built");
        roadgen_gpudrive::write(map, path, &config).map_err(runtime_error)
    }

    /// Writes the map as a CARLA UE5 asset package.
    ///
    /// A folder holding the descriptor CARLA's `Import.py` reads, and under it the
    /// map's `.fbx`, the `.xodr` of the same name, and the manifest that fetches the
    /// textures. Point CARLA's importer at the folder.
    ///
    /// Returns a dict: the paths written, how many meshes and triangles, how many
    /// meshes will carry each CARLA semantic class, and the textures still to fetch.
    ///
    /// `name` is the map's name, which is the `.fbx`, the `.xodr`, the level CARLA
    /// builds and the prefix of every mesh name in the file — CARLA works out a
    /// mesh's semantic class by matching its *name*, so this one string decides how
    /// the whole map segments. `carla_warnings()` says when it would decide wrongly.
    ///
    /// `buildings` is `"in_map"` (they stand in the level and CARLA tags them
    /// Terrain), `"props"` (tagged Buildings, and not placed) or `"omitted"`. The
    /// choice exists because CARLA's import pipeline will not do both; see
    /// `carla_warnings()`.
    ///
    /// `use_carla_materials` lets CARLA replace this package's materials with its own,
    /// which is what its documentation recommends and what its own maps do. Turn it
    /// off to see the Poly Haven textures the package ships.
    ///
    /// `furniture` builds the traffic lights and signs — a pole on the pavement, a
    /// mast arm reaching over the lanes it governs, a head over each — as props,
    /// with the manifest the package's script places them from and the
    /// `map_logic` that makes CARLA adopt them as its own lights. On by default;
    /// off, CARLA spawns its own blueprints at the signals, which the `.xodr` then
    /// places where the IR put the bar: over the middle of the lane.
    #[pyo3(signature = (
        directory,
        name = None,
        package = None,
        buildings = None,
        use_carla_materials = None,
        kerb_height = None,
        verge_width = None,
        ground_extent = None,
        furniture = None,
        carla_root = None,
        engine = None,
        sun_altitude = None,
        sun_azimuth = None,
    ))]
    #[allow(clippy::too_many_arguments)]
    fn export_carla(
        &mut self,
        py: Python<'_>,
        directory: PathBuf,
        name: Option<&str>,
        package: Option<&str>,
        buildings: Option<&str>,
        use_carla_materials: Option<bool>,
        kerb_height: Option<f64>,
        verge_width: Option<f64>,
        ground_extent: Option<f64>,
        furniture: Option<bool>,
        carla_root: Option<String>,
        engine: Option<String>,
        sun_altitude: Option<f64>,
        sun_azimuth: Option<f64>,
    ) -> PyResult<Py<PyDict>> {
        let mut config = self.carla_config(
            name,
            package,
            buildings,
            use_carla_materials,
            kerb_height,
            verge_width,
            ground_extent,
            furniture,
        )?;
        // What the package's script imports into, when the caller says now rather
        // than when running it.
        config.carla_root = carla_root.or_else(|| std::env::var("CARLA_ROOT").ok());
        config.engine = engine.or_else(|| std::env::var("CARLA_UNREAL_ENGINE_PATH").ok());
        if let Some(sun_altitude) = sun_altitude {
            config.sun_altitude = sun_altitude;
        }
        if let Some(sun_azimuth) = sun_azimuth {
            config.sun_azimuth = sun_azimuth;
        }
        let map = self.built.as_ref().expect("just built");
        let written = roadgen_carla::write(map, directory, &config).map_err(runtime_error)?;

        let report = PyDict::new(py);
        report.set_item("descriptor", written.descriptor.to_string_lossy())?;
        report.set_item("script", written.script.to_string_lossy())?;
        report.set_item("obj", written.obj.to_string_lossy())?;
        report.set_item("fbx", written.fbx.to_string_lossy())?;
        report.set_item("xodr", written.xodr.to_string_lossy())?;
        report.set_item(
            "props",
            written.props.as_ref().map(|path| path.to_string_lossy()),
        )?;
        report.set_item(
            "furniture",
            written
                .furniture
                .as_ref()
                .map(|path| path.to_string_lossy()),
        )?;
        report.set_item(
            "map_logic",
            written
                .map_logic
                .as_ref()
                .map(|path| path.to_string_lossy()),
        )?;
        report.set_item("lights", written.lights)?;
        report.set_item("signs", written.signs)?;
        report.set_item("meshes", written.meshes)?;
        report.set_item("triangles", written.triangles)?;
        // Keyed by the tag a segmentation camera reports, because that is the name
        // anybody checking this will be looking for.
        let labels = PyDict::new(py);
        for (label, count) in &written.labels {
            labels.set_item(label.as_str(), count)?;
        }
        report.set_item("labels", labels)?;
        report.set_item(
            "textures",
            written
                .textures
                .iter()
                .map(|entry| entry.path.clone())
                .collect::<Vec<_>>(),
        )?;
        Ok(report.into())
    }

    /// What a CARLA package cannot carry, and what its import will get wrong quietly.
    ///
    /// Kept apart from `format_warnings` for the same reason ClipGT's and GPUDrive's
    /// are: most of these are properties of CARLA's *import*, which classifies a mesh
    /// by its name and has no way to report having classified one wrongly. Takes the
    /// same arguments as `export_carla`, so a package can be checked before it is
    /// written.
    #[pyo3(signature = (
        name = None,
        package = None,
        buildings = None,
        use_carla_materials = None,
        kerb_height = None,
        verge_width = None,
        ground_extent = None,
        furniture = None,
    ))]
    #[allow(clippy::too_many_arguments)]
    fn carla_warnings(
        &mut self,
        name: Option<&str>,
        package: Option<&str>,
        buildings: Option<&str>,
        use_carla_materials: Option<bool>,
        kerb_height: Option<f64>,
        verge_width: Option<f64>,
        ground_extent: Option<f64>,
        furniture: Option<bool>,
    ) -> PyResult<Vec<String>> {
        let config = self.carla_config(
            name,
            package,
            buildings,
            use_carla_materials,
            kerb_height,
            verge_width,
            ground_extent,
            furniture,
        )?;
        let map = self.built.as_ref().expect("just built");
        Ok(roadgen_carla::check(map, &config))
    }

    /// The GPUDrive scene as a JSON string, with the same arguments as
    /// `export_gpudrive`.
    #[pyo3(signature = (
        scenario = None,
        name = None,
        scenario_id = None,
        steps = None,
        time_step = None,
        speed = None,
        route = None,
    ))]
    #[allow(clippy::wrong_self_convention, clippy::too_many_arguments)]
    fn to_gpudrive_json(
        &mut self,
        scenario: Option<PathBuf>,
        name: Option<&str>,
        scenario_id: Option<&str>,
        steps: Option<usize>,
        time_step: Option<f64>,
        speed: Option<f64>,
        route: Option<Vec<String>>,
    ) -> PyResult<String> {
        let config =
            self.gpudrive_config(scenario, name, scenario_id, steps, time_step, speed, route)?;
        let map = self.built.as_ref().expect("just built");
        roadgen_gpudrive::to_json(map, &config).map_err(runtime_error)
    }

    // `&mut self` because the map builds itself on demand; the name is the one the
    // Python API exposes, so it stays as it reads from Python.
    /// The OpenDRIVE document as a string.
    #[allow(clippy::wrong_self_convention)]
    fn to_opendrive_xml(&mut self) -> PyResult<String> {
        self.ensure_built()?;
        roadgen_opendrive::to_xml(self.built.as_ref().expect("just built")).map_err(runtime_error)
    }

    /// The plain OpenStreetMap document as a string.
    #[allow(clippy::wrong_self_convention)]
    fn to_osm_xml(&mut self) -> PyResult<String> {
        self.ensure_built()?;
        roadgen_osm::to_xml(self.built.as_ref().expect("just built")).map_err(runtime_error)
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

    /// The builder, for a method that would add to the map — which a map read from
    /// a file has no builder to add to.
    fn builder_mut(&mut self) -> PyResult<&mut MapBuilder> {
        if self.imported.is_some() {
            return Err(PyValueError::new_err(
                "this map was read from a file and cannot be added to: roadgen generates \
                 maps and reads them, it does not edit them",
            ));
        }
        Ok(&mut self.builder)
    }

    fn find_building(&mut self, building: &str) -> PyResult<&roadgen_core::Building> {
        self.ensure_built()?;
        let id = BuildingId::parse_printed(building);
        self.built
            .as_ref()
            .expect("just built")
            .building(&id)
            .ok_or_else(|| PyValueError::new_err(format!("no such building: {building}")))
    }

    fn find_part(&mut self, part: &str) -> PyResult<&roadgen_core::BuildingPart> {
        self.ensure_built()?;
        let id = BuildingPartId::parse_printed(part);
        self.built
            .as_ref()
            .expect("just built")
            .building_part(&id)
            .ok_or_else(|| PyValueError::new_err(format!("no such building part: {part}")))
    }

    fn build(&self) -> PyResult<UnvalidatedMap> {
        let mut map = match &self.imported {
            Some((map, _)) => map.clone(),
            None => self.builder.clone().finish().map_err(value_error)?,
        };
        // After the roads and before validation, which is the only moment buildings
        // can be generated: a lot is measured against generated geometry, and
        // validation has something to say about what comes out.
        if let Some(rules) = &self.buildings {
            roadgen_buildings::generate(&mut map, rules).map_err(value_error)?;
        }
        Ok(map)
    }

    fn ensure_built(&mut self) -> PyResult<()> {
        if self.built.is_none() {
            self.built = Some(self.build()?.validate().map_err(value_error)?);
        }
        Ok(())
    }

    /// The clip configuration the ClipGT methods share: the map's own defaults, the
    /// scenario file over them, and the arguments over that.
    fn clipgt_config(
        &mut self,
        scenario: Option<PathBuf>,
        clip_id: Option<&str>,
        frame_rate: Option<f64>,
        speed: Option<f64>,
        route: Option<Vec<String>>,
    ) -> PyResult<roadgen_clipgt::ClipConfig> {
        self.ensure_built()?;
        let map = self.built.as_ref().expect("just built");
        let mut config = roadgen_clipgt::ClipConfig::for_map(map);
        if let Some(path) = scenario {
            config = config.with_scenario_file(path).map_err(value_error)?;
        }
        if let Some(id) = clip_id {
            config.clip_id = id.to_owned();
        }
        if let Some(frame_rate) = frame_rate {
            config.frame_rate = frame_rate;
        }
        if let Some(speed) = speed {
            config.speed = speed;
        }
        if let Some(route) = route {
            config = config.with_route(route.iter().map(LaneId::parse_printed).collect());
        }
        Ok(config)
    }

    /// The CARLA package configuration, with everything the caller left out taken
    /// from the map.
    #[allow(clippy::too_many_arguments)]
    fn carla_config(
        &mut self,
        name: Option<&str>,
        package: Option<&str>,
        buildings: Option<&str>,
        use_carla_materials: Option<bool>,
        kerb_height: Option<f64>,
        verge_width: Option<f64>,
        ground_extent: Option<f64>,
        furniture: Option<bool>,
    ) -> PyResult<roadgen_carla::PackageConfig> {
        self.ensure_built()?;
        let map = self.built.as_ref().expect("just built");
        let mut config = match name {
            Some(name) => roadgen_carla::PackageConfig::new(name),
            None => roadgen_carla::PackageConfig::for_map(map),
        };
        if let Some(package) = package {
            config = config.with_package(package);
        }
        if let Some(buildings) = buildings {
            config.buildings = match buildings.to_ascii_lowercase().as_str() {
                "in_map" | "map" | "in-map" => roadgen_carla::BuildingPlacement::InMap,
                "props" | "prop" => roadgen_carla::BuildingPlacement::Props,
                "omitted" | "none" | "off" => roadgen_carla::BuildingPlacement::Omitted,
                other => {
                    return Err(PyValueError::new_err(format!(
                        "`{other}` is not a place to put buildings: `in_map` stands \
                         them in the level and lets CARLA tag them Terrain, `props` \
                         has them tagged Buildings and not placed, `omitted` leaves \
                         them out"
                    )))
                }
            };
        }
        if let Some(use_carla_materials) = use_carla_materials {
            config.use_carla_materials = use_carla_materials;
        }
        if let Some(kerb_height) = kerb_height {
            config.surfaces.kerb_height = kerb_height;
        }
        if let Some(verge_width) = verge_width {
            config.surfaces.verge_width = verge_width;
        }
        if let Some(ground_extent) = ground_extent {
            config.surfaces.ground_extent = ground_extent;
        }
        if furniture == Some(false) {
            config.furniture = None;
        }
        Ok(config)
    }

    /// The scene configuration the GPUDrive methods share: the map's own defaults, the
    /// scenario file over them, and the arguments over that.
    #[allow(clippy::too_many_arguments)]
    fn gpudrive_config(
        &mut self,
        scenario: Option<PathBuf>,
        name: Option<&str>,
        scenario_id: Option<&str>,
        steps: Option<usize>,
        time_step: Option<f64>,
        speed: Option<f64>,
        route: Option<Vec<String>>,
    ) -> PyResult<roadgen_gpudrive::SceneConfig> {
        self.ensure_built()?;
        let map = self.built.as_ref().expect("just built");
        let mut config = roadgen_gpudrive::SceneConfig::for_map(map);
        if let Some(path) = scenario {
            config = config.with_scenario_file(path).map_err(value_error)?;
        }
        if let Some(name) = name {
            config.name = name.to_owned();
        }
        if let Some(scenario_id) = scenario_id {
            config.scenario_id = scenario_id.to_owned();
        }
        if let Some(steps) = steps {
            config.steps = steps;
        }
        if let Some(time_step) = time_step {
            config.time_step = time_step;
        }
        // Both nudge the scene's own vehicle, which the exporter knows how to find.
        if let Some(speed) = speed {
            config.sdc_mut().speed = speed;
        }
        if let Some(route) = route {
            config.sdc_mut().route = Some(roadgen_gpudrive::Route::Lanes(
                route.iter().map(LaneId::parse_printed).collect(),
            ));
        }
        Ok(config)
    }
}

// --------------------------------------------------------------------------- //
// Drawing what was written
// --------------------------------------------------------------------------- //

/// The module-level `render_*` functions, which are the only things here that read a
/// file rather than write one.
///
/// They take a path because what they draw is the export, not the map: a picture made
/// from the IR would agree with the IR by construction and so would say nothing about
/// whether the file is right. Reading the file back is the point.
///
/// Every one returns an SVG document as text. In a notebook that is
/// `IPython.display.SVG(...)`; on a page it goes straight into the DOM; written to a
/// file it opens in anything.
/// Reads an OpenDRIVE file as a map.
///
/// The one way into a `Map` that is not `add_road`: the file's reference lines,
/// lanes, movements, furniture and rules become the map's, and its lane boundaries
/// are laid out from them the way the builder lays out a road it generated. What
/// comes back exports like any other map and cannot be added to. `sampling` is how
/// finely the boundaries are sampled, metres, since the file does not say; `origin`
/// is `(latitude, longitude, altitude)` for a file whose `<geoReference>` is missing
/// or not understood, with `projection` naming which of the map's projections it
/// is in.
///
/// `Map.read_warnings()` says what the file stated that the map could not keep.
#[pyfunction]
#[pyo3(name = "read_opendrive")]
#[pyo3(signature = (path, sampling = 2.0, origin = None, projection = "local_cartesian"))]
fn read_opendrive(
    path: PathBuf,
    sampling: f64,
    origin: Option<(f64, f64, f64)>,
    projection: &str,
) -> PyResult<PyMap> {
    let options = roadgen_opendrive::ReadOptions {
        sampling: SamplingConfig::new(sampling).map_err(value_error)?,
        origin: match origin {
            Some((latitude, longitude, altitude)) => Some((
                GeoOrigin::new(latitude, longitude, altitude).map_err(value_error)?,
                Projection::parse(projection).ok_or_else(|| {
                    PyValueError::new_err(format!("unknown projection {projection:?}"))
                })?,
            )),
            None => None,
        },
    };
    let imported = roadgen_opendrive::read_with(&path, &options).map_err(value_error)?;
    let metadata = imported.map.as_map().metadata.clone();
    Ok(PyMap {
        builder: MapBuilder::new(metadata),
        built: None,
        imported: Some((imported.map, imported.approximations)),
        buildings: None,
    })
}

#[pyfunction]
#[pyo3(name = "render_opendrive")]
fn render_opendrive(path: PathBuf) -> PyResult<String> {
    Ok(roadgen_viewer::opendrive_file(&path)
        .map_err(runtime_error)?
        .to_svg())
}

/// Draws the SUMO plain-XML network in `directory` — the one `export_sumo` wrote.
#[pyfunction]
#[pyo3(name = "render_sumo")]
fn render_sumo(directory: PathBuf) -> PyResult<String> {
    Ok(roadgen_viewer::sumo_directory(&directory)
        .map_err(runtime_error)?
        .to_svg())
}

/// Draws the ClipGT clip in `directory` — the one `export_clipgt` wrote.
#[pyfunction]
#[pyo3(name = "render_clipgt")]
fn render_clipgt(directory: PathBuf) -> PyResult<String> {
    Ok(roadgen_viewer::clipgt_directory(&directory)
        .map_err(runtime_error)?
        .to_svg())
}

/// Draws the GPUDrive scene at `path` — the one `export_gpudrive` wrote.
#[pyfunction]
#[pyo3(name = "render_gpudrive")]
fn render_gpudrive(path: PathBuf) -> PyResult<String> {
    Ok(roadgen_viewer::gpudrive_file(&path)
        .map_err(runtime_error)?
        .to_svg())
}

/// Draws the FBX at `path` — the one `export_carla` wrote.
///
/// Coloured by the semantic class CARLA will give each mesh, worked out from the
/// mesh's *name* by the same substring match CARLA's own import uses. So a mesh drawn
/// in the wrong colour is one that will segment wrongly in the simulator, and the
/// picture is the only place that is visible before the map is loaded.
#[pyfunction]
#[pyo3(name = "render_carla")]
fn render_carla(path: PathBuf) -> PyResult<String> {
    Ok(roadgen_viewer::fbx_file(&path)
        .map_err(runtime_error)?
        .to_svg())
}

/// The names of the built-in building rule sets.
#[pyfunction]
fn building_presets() -> Vec<String> {
    roadgen_buildings::PRESETS
        .iter()
        .map(|(name, _)| (*name).to_owned())
        .collect()
}

/// A built-in rule set, as the CGA grammar it is.
///
/// With no argument, the one `generate_buildings()` uses when none is named. The
/// point of handing the text back is that rules of your own start as a preset with a
/// line changed rather than as a blank page.
#[pyfunction]
#[pyo3(signature = (name = None))]
fn building_rules(name: Option<&str>) -> PyResult<String> {
    match name {
        None => Ok(BuildingRules::default().source().to_owned()),
        // The same error the generator raises for the same mistake, said the same
        // way: a second wording of "no preset is called that" would drift from the
        // first the moment either changed.
        Some(name) => roadgen_buildings::preset(name)
            .map(str::to_owned)
            .ok_or_else(|| {
                value_error(roadgen_buildings::Error::UnknownPreset {
                    name: name.to_owned(),
                    known: roadgen_buildings::PRESETS.iter().map(|(n, _)| *n).collect(),
                })
            }),
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
    module.add_function(wrap_pyfunction!(read_opendrive, module)?)?;
    module.add_function(wrap_pyfunction!(render_opendrive, module)?)?;
    module.add_function(wrap_pyfunction!(render_sumo, module)?)?;
    module.add_function(wrap_pyfunction!(render_clipgt, module)?)?;
    module.add_function(wrap_pyfunction!(render_gpudrive, module)?)?;
    module.add_function(wrap_pyfunction!(render_carla, module)?)?;
    module.add_function(wrap_pyfunction!(building_presets, module)?)?;
    module.add_function(wrap_pyfunction!(building_rules, module)?)?;
    Ok(())
}
