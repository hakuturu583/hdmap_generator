//! `roadgen.Trace`: lookups across the trace files the exports write.

use std::path::PathBuf;

use pyo3::prelude::*;
use pyo3::types::PyDict;

use roadgen_trace::{Link, TraceIndex, Translation};

use crate::{runtime_error, value_error};

/// An element named from Python: a lanelet id is an `int`, a SUMO lane a `str`.
fn element_text(element: &Bound<'_, PyAny>) -> PyResult<String> {
    if let Ok(number) = element.extract::<i64>() {
        return Ok(number.to_string());
    }
    element.extract::<String>()
}

fn link_dict<'py>(py: Python<'py>, link: &Link) -> PyResult<Bound<'py, PyDict>> {
    let dict = PyDict::new(py);
    dict.set_item("ir", &link.ir)?;
    dict.set_item("ref", &link.local)?;
    dict.set_item("rel", link.relation.as_str())?;
    dict.set_item("role", &link.role)?;
    Ok(dict)
}

fn translation_dict<'py>(py: Python<'py>, answer: &Translation) -> PyResult<Bound<'py, PyDict>> {
    let dict = PyDict::new(py);
    dict.set_item("ref", &answer.local)?;
    dict.set_item("ir", &answer.ir)?;
    dict.set_item("rel", answer.relation.as_str())?;
    dict.set_item("role", &answer.role)?;
    dict.set_item("via", &answer.via)?;
    Ok(dict)
}

/// The IR dump and trace files of one map, joined.
///
/// ```text
/// t = roadgen.Trace.load("out/map.ir.json",
///                        "out/lanelet2_map.osm.trace.json",
///                        "out/sumo/sumo.trace.json")
/// t.translate("lanelet2", 1000123, to="sumo")
/// ```
///
/// Every file has to come from the same map, and every file a trace describes has
/// to be unchanged since it was written; loading says so otherwise, rather than
/// answering with numbers that now mean something else.
#[pyclass(name = "Trace", module = "roadgen")]
pub struct PyTrace {
    index: TraceIndex,
}

#[pymethods]
impl PyTrace {
    #[new]
    #[pyo3(signature = (check_files = true))]
    fn new(check_files: bool) -> Self {
        PyTrace {
            index: TraceIndex::new().check_files(check_files),
        }
    }

    /// A trace of the files at `paths`: IR dumps, trace files, and a SUMO `.net.xml`
    /// built from an export — which has to come after the SUMO trace.
    #[staticmethod]
    #[pyo3(signature = (*paths, check_files = true))]
    fn load(paths: Vec<PathBuf>, check_files: bool) -> PyResult<Self> {
        let mut trace = PyTrace::new(check_files);
        for path in paths {
            trace.add(path)?;
        }
        Ok(trace)
    }

    /// Adds one more file.
    fn add(&mut self, path: PathBuf) -> PyResult<()> {
        self.index.load(&path).map_err(value_error)
    }

    /// Adds the network netconvert built from a SUMO export, so that the internal
    /// lanes it drew across each junction trace back too. Returns how many were
    /// traced, and how many connections netconvert added of its own.
    fn add_sumo_net(&mut self, path: PathBuf) -> PyResult<(usize, usize)> {
        let report = self.index.load_sumo_net(&path).map_err(value_error)?;
        Ok((report.internal_lanes, report.untraced))
    }

    /// The formats loaded.
    fn formats(&self) -> Vec<String> {
        self.index.formats().into_iter().map(str::to_owned).collect()
    }

    /// The IR elements `element` of `format` was written from.
    ///
    /// `element` may be bare — `1000123` in Lanelet2 is a lanelet, `"north.fwd_0"` in
    /// SUMO a lane — or carry its kind, `"linestring:1000124"`.
    fn to_ir<'py>(
        &self,
        py: Python<'py>,
        format: &str,
        element: &Bound<'py, PyAny>,
    ) -> PyResult<Vec<Bound<'py, PyDict>>> {
        let element = element_text(element)?;
        self.index
            .to_ir(format, &element)
            .map_err(runtime_error)?
            .into_iter()
            .map(|link| link_dict(py, link))
            .collect()
    }

    /// What the IR element `ir` was written as in `format`.
    fn from_ir<'py>(
        &self,
        py: Python<'py>,
        ir: &str,
        format: &str,
    ) -> PyResult<Vec<Bound<'py, PyDict>>> {
        self.index
            .from_ir(ir, format)
            .map_err(runtime_error)?
            .into_iter()
            .map(|link| link_dict(py, link))
            .collect()
    }

    /// The elements of format `to` that `element` of `format` corresponds to.
    ///
    /// Each answer names the IR element it went through; `via` is set when that is a
    /// neighbour of what `element` came from, because `to` has nothing for the
    /// element itself.
    #[pyo3(signature = (format, element, to))]
    fn translate<'py>(
        &self,
        py: Python<'py>,
        format: &str,
        element: &Bound<'py, PyAny>,
        to: &str,
    ) -> PyResult<Vec<Bound<'py, PyDict>>> {
        let element = element_text(element)?;
        self.index
            .translate(format, &element, to)
            .map_err(runtime_error)?
            .iter()
            .map(|answer| translation_dict(py, answer))
            .collect()
    }

    /// The IR elements one step from `ir`, by the IR dump.
    fn neighbours(&self, ir: &str) -> Vec<String> {
        self.index.neighbours(ir)
    }

    fn __repr__(&self) -> String {
        format!("Trace(formats={:?})", self.index.formats())
    }
}
