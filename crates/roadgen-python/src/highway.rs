//! The motorway side of the API: corridors and the plans they draw, and reading
//! back the bridges, tunnels, grade separations and interchanges a map carries.

use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use pyo3::types::PyDict;

use roadgen_carla::environment::{
    Control, CorridorPlan, CorridorSpec, InterchangeKind, InterchangeSite, RampSite, Side,
    StructureMode,
};
use roadgen_core::map::{Map, TrafficHandedness};
use roadgen_core::semantics::FormOfWay;
use roadgen_core::structure::{
    BridgeForm, BridgeSpec, RoadSpan, Structure, StructureKind, StructureMaterial, TunnelForm,
    TunnelSpec, Underneath,
};

use crate::{point, PyLane};

fn triple(p: roadgen_core::geometry::Point3) -> (f64, f64, f64) {
    (p.x, p.y, p.z)
}

pub(crate) fn side(text: &str) -> PyResult<Side> {
    Side::parse(text)
        .ok_or_else(|| PyValueError::new_err(format!("a side is left or right, not {text:?}")))
}

pub(crate) fn form_of_way(text: &str) -> PyResult<FormOfWay> {
    FormOfWay::parse(text)
        .ok_or_else(|| PyValueError::new_err(format!("unknown form of way {text:?}")))
}

/// `[(road, from, to), ...]` as spans.
pub(crate) fn spans(spans: Vec<(String, f64, f64)>) -> Vec<RoadSpan> {
    spans
        .into_iter()
        .map(|(road, from, to)| {
            RoadSpan::new(roadgen_core::id::RoadId::parse_printed(road), from, to)
        })
        .collect()
}

pub(crate) fn bridge_spec(
    form: &str,
    material: &str,
    deck_depth: Option<f64>,
    pier_spacing: f64,
    name: Option<String>,
) -> PyResult<BridgeSpec> {
    let form = BridgeForm::parse(form)
        .ok_or_else(|| PyValueError::new_err(format!("unknown bridge form {form:?}")))?;
    let defaults = BridgeSpec::for_form(form);
    Ok(BridgeSpec {
        name,
        material: StructureMaterial::parse(material)
            .ok_or_else(|| PyValueError::new_err(format!("unknown material {material:?}")))?,
        deck_depth: deck_depth.unwrap_or(defaults.deck_depth),
        pier_spacing,
        ..defaults
    })
}

pub(crate) fn tunnel_spec(
    form: &str,
    clear_height: f64,
    lining: f64,
    lighting: bool,
    name: Option<String>,
) -> PyResult<TunnelSpec> {
    Ok(TunnelSpec {
        name,
        form: TunnelForm::parse(form)
            .ok_or_else(|| PyValueError::new_err(format!("unknown tunnel form {form:?}")))?,
        clear_height,
        lining,
        lighting,
        ..TunnelSpec::default()
    })
}

/// A stretch of road as Python sees it: `(road, from, to)`.
type Span = (String, f64, f64);

/// A motorway corridor: its line, its design speed and profile, and the
/// interchanges and slip roads along it. It puts nothing on the map itself;
/// `plan()` draws the roads, and `Map.add_plan()` puts them there.
#[pyclass(name = "Corridor", module = "roadgen", skip_from_py_object)]
#[derive(Clone)]
pub struct PyCorridor {
    pub(crate) spec: CorridorSpec,
    pub(crate) handedness: TrafficHandedness,
}

#[pymethods]
impl PyCorridor {
    #[getter]
    fn name(&self) -> String {
        self.spec.name.clone()
    }

    /// The design standard the corridor is drawn to, as a dict.
    #[getter]
    fn standard<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyDict>> {
        let s = &self.spec.standard;
        let out = PyDict::new(py);
        out.set_item("design_speed", s.design_speed)?;
        out.set_item("min_radius", s.min_radius)?;
        out.set_item("max_grade", s.max_grade)?;
        out.set_item("crest_radius", s.crest_radius)?;
        out.set_item("sag_radius", s.sag_radius)?;
        out.set_item("lane_width", s.lane_width)?;
        out.set_item("outer_shoulder", s.outer_shoulder)?;
        out.set_item("inner_shoulder", s.inner_shoulder)?;
        out.set_item("median", s.median)?;
        out.set_item("deceleration", s.deceleration)?;
        out.set_item("acceleration", s.acceleration)?;
        out.set_item("taper", s.taper)?;
        Ok(out)
    }

    /// Adds an interchange at centre-line station `at`, onto the crossroad drawn
    /// by `crossroad` at ground level. `kind` is `diamond`, or `half_diamond` with
    /// `side` saying which carriageway's slip roads it has.
    #[pyo3(signature = (
        name,
        at,
        crossroad,
        crossroad_lanes,
        kind = "diamond",
        side = None,
        over = true,
        control = "signal",
        terminal = 80.0,
        reach = 320.0,
    ))]
    #[allow(clippy::too_many_arguments)]
    fn add_interchange(
        &mut self,
        name: String,
        at: f64,
        crossroad: Vec<(f64, f64, f64)>,
        crossroad_lanes: Vec<PyLane>,
        kind: &str,
        side: Option<&str>,
        over: bool,
        control: &str,
        terminal: f64,
        reach: f64,
    ) -> PyResult<()> {
        let kind = match (kind, side) {
            ("diamond", _) => InterchangeKind::Diamond,
            ("half_diamond", Some(text)) => InterchangeKind::HalfDiamond {
                side: self::side(text)?,
            },
            ("half_diamond", None) => {
                return Err(PyValueError::new_err(
                    "a half diamond needs side= to say whose slip roads it has",
                ))
            }
            (other, _) => {
                return Err(PyValueError::new_err(format!(
                    "unknown interchange {other:?}; diamond or half_diamond"
                )))
            }
        };
        let control = match control {
            "signal" => Control::Signal,
            "yield" => Control::Yield,
            other => {
                return Err(PyValueError::new_err(format!(
                    "control is signal or yield, not {other:?}"
                )))
            }
        };
        self.spec.interchanges.push(InterchangeSite {
            name,
            kind,
            at,
            crossroad: crossroad.into_iter().map(point).collect(),
            crossroad_lanes: crossroad_lanes.into_iter().map(|lane| lane.spec).collect(),
            over,
            control,
            terminal,
            reach,
        });
        Ok(())
    }

    /// Adds a lone slip road off (`on=False`) or onto a carriageway, its nose at
    /// centre-line station `at`, its open end at `end` for a street to be joined to.
    #[pyo3(signature = (name, at, end, side = "right", on = false))]
    fn add_ramp(
        &mut self,
        name: String,
        at: f64,
        end: (f64, f64, f64),
        side: &str,
        on: bool,
    ) -> PyResult<()> {
        self.spec.ramps.push(RampSite {
            name,
            on,
            side: self::side(side)?,
            at,
            end: point(end),
        });
        Ok(())
    }

    /// The roads, junctions, movements and structures the corridor asks for.
    fn plan(&self) -> PyResult<PyCorridorPlan> {
        let plan = self
            .spec
            .plan(self.handedness)
            .map_err(PyValueError::new_err)?;
        Ok(PyCorridorPlan {
            plan,
            spec: self.spec.clone(),
        })
    }
}

/// A first draft of a corridor's roads. Look at it, move its points with
/// `set_points`, and put it on the map with `Map.add_plan`.
#[pyclass(name = "CorridorPlan", module = "roadgen", skip_from_py_object)]
#[derive(Clone)]
pub struct PyCorridorPlan {
    pub(crate) plan: CorridorPlan,
    pub(crate) spec: CorridorSpec,
}

#[pymethods]
impl PyCorridorPlan {
    /// Every road as `{"name", "points", "form", "type", "lanes", "corridor"}`.
    #[getter]
    fn roads<'py>(&self, py: Python<'py>) -> PyResult<Vec<Bound<'py, PyDict>>> {
        self.plan
            .roads
            .iter()
            .map(|road| {
                let out = PyDict::new(py);
                out.set_item("name", road.name.clone())?;
                out.set_item(
                    "points",
                    road.points.iter().copied().map(triple).collect::<Vec<_>>(),
                )?;
                out.set_item("form", road.form.as_str())?;
                out.set_item("type", road.road_type.as_str())?;
                out.set_item("lanes", road.lanes.len())?;
                out.set_item("speed_limit_kph", road.speed_limit)?;
                out.set_item("corridor", road.corridor)?;
                Ok(out)
            })
            .collect()
    }

    /// Where the plan departs from its design standard.
    #[getter]
    fn notes(&self) -> Vec<String> {
        self.plan.notes.clone()
    }

    #[getter]
    fn junctions(&self) -> Vec<String> {
        self.plan.junctions.clone()
    }

    /// The structures it asks for outright, as `(form, [(road, from, to), ...])`;
    /// the rest are chosen from the land when the map is built.
    #[getter]
    fn structures(&self) -> Vec<(String, Vec<Span>)> {
        self.plan
            .structures
            .iter()
            .map(|s| {
                let form = s
                    .bridge
                    .map(|f| f.as_str())
                    .or(s.tunnel.map(|f| f.as_str()))
                    .unwrap_or("")
                    .to_owned();
                (form, s.spans.clone())
            })
            .collect()
    }

    /// Replaces one road's points, keeping its lanes and connections.
    fn set_points(&mut self, name: &str, points: Vec<(f64, f64, f64)>) -> PyResult<()> {
        let road = self
            .plan
            .roads
            .iter_mut()
            .find(|road| road.name == name)
            .ok_or_else(|| PyValueError::new_err(format!("the plan has no road {name}")))?;
        if points.len() < 2 {
            return Err(PyValueError::new_err("a road needs two points at least"));
        }
        road.points = points.into_iter().map(point).collect();
        Ok(())
    }
}

#[allow(clippy::too_many_arguments)]
pub(crate) fn corridor(
    name: String,
    centre: Vec<(f64, f64, f64)>,
    design_speed: f64,
    profile: Option<Vec<(f64, f64)>>,
    lanes: usize,
    structures: &str,
    elevated: Option<Vec<(f64, f64)>>,
    elevated_height: f64,
    sunken: Option<Vec<(f64, f64)>>,
    sunken_depth: f64,
    end_transition: Option<f64>,
) -> PyResult<CorridorSpec> {
    let mut spec = CorridorSpec::new(name, centre.into_iter().map(point).collect(), design_speed)
        .map_err(PyValueError::new_err)?;
    spec.profile = profile.unwrap_or_default();
    spec.lanes = lanes;
    spec.structures = match structures {
        "auto" => StructureMode::Auto,
        "explicit" => StructureMode::Explicit,
        other => {
            return Err(PyValueError::new_err(format!(
                "structures is auto or explicit, not {other:?}"
            )))
        }
    };
    spec.elevated = elevated.unwrap_or_default();
    spec.elevated_height = elevated_height;
    spec.sunken = sunken.unwrap_or_default();
    spec.sunken_depth = sunken_depth;
    spec.end_transition = end_transition;
    spec.check().map_err(PyValueError::new_err)?;
    Ok(spec)
}

/// A structure as a dict.
pub(crate) fn structure<'py>(py: Python<'py>, entry: &Structure) -> PyResult<Bound<'py, PyDict>> {
    let out = PyDict::new(py);
    out.set_item("name", entry.name.clone())?;
    out.set_item("kind", entry.kind.as_str())?;
    out.set_item("form", entry.kind.form())?;
    out.set_item(
        "carries",
        entry
            .carries
            .iter()
            .map(|span| (span.road.to_string(), span.from, span.to))
            .collect::<Vec<_>>(),
    )?;
    out.set_item(
        "outline",
        entry
            .outline
            .points()
            .iter()
            .copied()
            .map(triple)
            .collect::<Vec<_>>(),
    )?;
    match &entry.kind {
        StructureKind::Bridge(bridge) => {
            out.set_item("material", bridge.material.as_str())?;
            out.set_item("deck_depth", bridge.deck_depth)?;
            let supports = bridge
                .supports
                .iter()
                .map(|support| {
                    let item = PyDict::new(py);
                    item.set_item("kind", support.kind.as_str())?;
                    item.set_item(
                        "footprint",
                        support
                            .footprint
                            .points()
                            .iter()
                            .copied()
                            .map(triple)
                            .collect::<Vec<_>>(),
                    )?;
                    item.set_item("base", support.base)?;
                    Ok(item)
                })
                .collect::<PyResult<Vec<_>>>()?;
            out.set_item("supports", supports)?;
        }
        StructureKind::Tunnel(tunnel) => {
            out.set_item("clear_height", tunnel.clear_height)?;
            out.set_item("lining", tunnel.lining)?;
            out.set_item("lighting", tunnel.lighting)?;
            out.set_item(
                "portals",
                tunnel
                    .portals
                    .iter()
                    .map(|portal| {
                        (
                            portal.road.to_string(),
                            portal.station,
                            triple(portal.left),
                            triple(portal.right),
                        )
                    })
                    .collect::<Vec<_>>(),
            )?;
        }
    }
    Ok(out)
}

/// Every grade separation as `{"id", "over": (road, from, to), "under", "via"}`;
/// `under` is a road's stretch or a land cover's identifier.
pub(crate) fn separations<'py>(py: Python<'py>, map: &Map) -> PyResult<Vec<Bound<'py, PyDict>>> {
    map.grade_separations
        .iter()
        .map(|separation| {
            let out = PyDict::new(py);
            out.set_item("id", separation.id.to_string())?;
            let span = &separation.over;
            out.set_item("over", (span.road.to_string(), span.from, span.to))?;
            match &separation.under {
                Underneath::Road(span) => {
                    out.set_item("under", (span.road.to_string(), span.from, span.to))?
                }
                Underneath::LandCover(cover) => out.set_item("under", cover.to_string())?,
            }
            out.set_item("via", separation.via.to_string())?;
            Ok(out)
        })
        .collect()
}
