//! The scenario file: where the vehicle drives, and what is bolted to it.
//!
//! A generated road network is one thing; driving it is another. The same map should
//! be drivable several ways, with several camera rigs, without editing the map or
//! recompiling — so the route and the sensors are read from a file rather than
//! written in code.
//!
//! ```yaml
//! clip_id: town
//! frame_rate: 30.0
//! speed: 12.0
//!
//! route:
//!   start: lane/north/0          # follow successors from here
//!   # lanes: [lane/north/0, ...] # or drive exactly these, in order
//!
//! sensors:
//!   - name: camera:front_wide_120fov
//!     position: [1.7, 0.0, 1.45]       # metres, in the rig's forward-left-up frame
//!     roll_pitch_yaw: [0.0, 0.0, 0.0]  # degrees
//!     width: 1920
//!     height: 1080
//!     fov_degrees: 120.0
//! ```
//!
//! Unknown keys are an error rather than being ignored. A scenario file is a thing
//! people edit by hand, and a silently dropped `speed_kph` that should have been
//! `speed` is a worse outcome than a message saying so.

use std::path::Path;

use serde::Deserialize;

use roadgen_core::LaneId;

use crate::error::ExportError;
use crate::{ClipConfig, Route};

/// How a camera's polynomial is to be read.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PolynomialType {
    /// Angle in, pixel distance out — the forward direction.
    AngleToPixelDistance,
    /// Pixel distance in, angle out.
    PixelDistanceToAngle,
}

impl PolynomialType {
    pub fn as_str(self) -> &'static str {
        match self {
            PolynomialType::AngleToPixelDistance => "angle-to-pixeldistance",
            PolynomialType::PixelDistanceToAngle => "pixeldistance-to-angle",
        }
    }

    fn parse(value: &str) -> Option<Self> {
        match value.to_ascii_lowercase().replace('_', "-").as_str() {
            "angle-to-pixeldistance" => Some(PolynomialType::AngleToPixelDistance),
            "pixeldistance-to-angle" => Some(PolynomialType::PixelDistanceToAngle),
            _ => None,
        }
    }
}

/// One camera on the rig.
#[derive(Debug, Clone, PartialEq)]
pub struct Sensor {
    /// The rig name, always `camera:…` so that a reader discovers it.
    pub name: String,
    /// Where it sits, metres in the rig's forward-left-up frame.
    pub position: [f64; 3],
    /// How it is aimed, degrees, applied as an `xyz` Euler triple.
    pub roll_pitch_yaw: [f64; 3],
    pub width: u32,
    pub height: u32,
    pub cx: f64,
    pub cy: f64,
    /// The f-theta coefficients, lowest order first.
    pub polynomial: Vec<f64>,
    pub polynomial_type: PolynomialType,
    /// The affine term `[c, d, e]`; `[1, 0, 0]` is the identity.
    pub linear: [f64; 3],
}

impl Sensor {
    /// The name with `:` replaced by `_`, which is the key a reader files the camera
    /// under and the stem of its timestamps file.
    pub fn canonical_name(&self) -> String {
        self.name.replace(':', "_")
    }
}

// --------------------------------------------------------------------------- //
// The document
// --------------------------------------------------------------------------- //

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Document {
    clip_id: Option<String>,
    frame_rate: Option<f64>,
    speed: Option<f64>,
    route: Option<RouteDocument>,
    #[serde(default)]
    sensors: Vec<SensorDocument>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct RouteDocument {
    start: Option<String>,
    lanes: Option<Vec<String>>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct SensorDocument {
    name: String,
    position: [f64; 3],
    roll_pitch_yaw: Option<[f64; 3]>,
    width: u32,
    height: u32,
    cx: Option<f64>,
    cy: Option<f64>,
    polynomial: Option<Vec<f64>>,
    polynomial_type: Option<String>,
    fov_degrees: Option<f64>,
    linear: Option<[f64; 3]>,
}

/// Reads a scenario from YAML text, starting from `base`.
///
/// Anything the file does not mention keeps the value it has in `base`, so a scenario
/// can say only what it wants to change.
pub fn from_yaml_str(text: &str, base: ClipConfig) -> Result<ClipConfig, ExportError> {
    let document: Document =
        serde_norway::from_str(text).map_err(|error| ExportError::Scenario(error.to_string()))?;
    apply(document, base)
}

/// Reads a scenario from a file, starting from `base`.
pub fn from_yaml_file(path: impl AsRef<Path>, base: ClipConfig) -> Result<ClipConfig, ExportError> {
    let path = path.as_ref();
    let text = std::fs::read_to_string(path)
        .map_err(|error| ExportError::Io(format!("{}: {error}", path.display())))?;
    from_yaml_str(&text, base).map_err(|error| match error {
        // A parse failure names a line and column, which is only useful with the file
        // it belongs to.
        ExportError::Scenario(detail) => {
            ExportError::Scenario(format!("{}: {detail}", path.display()))
        }
        other => other,
    })
}

fn apply(document: Document, base: ClipConfig) -> Result<ClipConfig, ExportError> {
    let mut config = base;
    if let Some(clip_id) = document.clip_id {
        config.clip_id = clip_id;
    }
    if let Some(frame_rate) = document.frame_rate {
        config.frame_rate = frame_rate;
    }
    if let Some(speed) = document.speed {
        config.speed = speed;
    }
    if let Some(route) = document.route {
        config.route = Some(route_of(route)?);
    }
    if !document.sensors.is_empty() {
        config.sensors = document
            .sensors
            .into_iter()
            .map(sensor_of)
            .collect::<Result<_, _>>()?;
    }
    Ok(config)
}

/// Reads a lane identifier the way `Map::lane_ids` prints it.
///
/// The prefix is added if the file left it off, so both `lane/north/0` and `north/0`
/// name the same lane — a scenario is written by hand, and `LaneId::new` would
/// otherwise quietly turn the printed form into `lane/lane/north/0`.
pub fn lane_id(text: &str) -> LaneId {
    match text.strip_prefix(&format!("{}/", LaneId::PREFIX)) {
        Some(_) => LaneId::from_raw(text),
        None => LaneId::new(text),
    }
}

fn route_of(document: RouteDocument) -> Result<Route, ExportError> {
    match (document.start, document.lanes) {
        (Some(start), None) => Ok(Route::From(lane_id(&start))),
        (None, Some(lanes)) => {
            if lanes.is_empty() {
                return Err(ExportError::Scenario(
                    "route.lanes is empty; leave the whole `route` out to have one \
                     found automatically"
                        .into(),
                ));
            }
            Ok(Route::Lanes(
                lanes.iter().map(String::as_str).map(lane_id).collect(),
            ))
        }
        (Some(_), Some(_)) => Err(ExportError::Scenario(
            "route has both `start` and `lanes`: either follow successors from one \
             lane or list them all, not both"
                .into(),
        )),
        (None, None) => Err(ExportError::Scenario(
            "route has neither `start` nor `lanes`".into(),
        )),
    }
}

fn sensor_of(document: SensorDocument) -> Result<Sensor, ExportError> {
    let name = if document.name.contains(':') {
        document.name.clone()
    } else {
        // A reader discovers a sensor by its `camera:` prefix and skips anything
        // else, so a bare name would produce a rig whose camera is never found.
        format!("camera:{}", document.name)
    };
    if !name.starts_with("camera:") {
        return Err(ExportError::Scenario(format!(
            "sensor {:?} is not a camera: a ClipGT reader only picks up sensors whose \
             name starts with `camera:`",
            document.name
        )));
    }
    if document.width == 0 || document.height == 0 {
        return Err(ExportError::Scenario(format!(
            "sensor {name:?} has a {} × {} frame",
            document.width, document.height
        )));
    }

    let polynomial_type = match document.polynomial_type.as_deref() {
        Some(value) => PolynomialType::parse(value).ok_or_else(|| {
            ExportError::Scenario(format!(
                "sensor {name:?}: polynomial_type {value:?} is neither \
                 `angle-to-pixeldistance` nor `pixeldistance-to-angle`"
            ))
        })?,
        None => PolynomialType::AngleToPixelDistance,
    };

    let polynomial = match (document.polynomial, document.fov_degrees) {
        (Some(coefficients), None) => {
            if coefficients.iter().all(|value| *value == 0.0) {
                return Err(ExportError::Scenario(format!(
                    "sensor {name:?}: every polynomial coefficient is zero, which is \
                     a camera that maps the whole world onto one pixel"
                )));
            }
            coefficients
        }
        (None, Some(fov)) => equidistant(&name, document.width, fov, polynomial_type)?,
        (Some(_), Some(_)) => {
            return Err(ExportError::Scenario(format!(
                "sensor {name:?} gives both `polynomial` and `fov_degrees`: the second \
                 is a shorthand for the first, so only one of them can be right"
            )))
        }
        (None, None) => {
            return Err(ExportError::Scenario(format!(
                "sensor {name:?} has no lens: give either `polynomial` or, for an \
                 ideal equidistant camera, `fov_degrees`"
            )))
        }
    };

    Ok(Sensor {
        cx: document.cx.unwrap_or(document.width as f64 / 2.0),
        cy: document.cy.unwrap_or(document.height as f64 / 2.0),
        name,
        position: document.position,
        roll_pitch_yaw: document.roll_pitch_yaw.unwrap_or([0.0; 3]),
        width: document.width,
        height: document.height,
        polynomial,
        polynomial_type,
        linear: document.linear.unwrap_or([1.0, 0.0, 0.0]),
    })
}

/// The polynomial of an ideal equidistant f-theta camera: `r = f·θ`, with `f` chosen
/// so that the horizontal half-angle lands on the edge of the frame.
///
/// This is a convenience, not a calibration. A real lens is measured; this is the
/// textbook fisheye that a caller who only knows "120 degrees across" means.
fn equidistant(
    name: &str,
    width: u32,
    fov_degrees: f64,
    polynomial_type: PolynomialType,
) -> Result<Vec<f64>, ExportError> {
    if !(fov_degrees.is_finite() && fov_degrees > 0.0 && fov_degrees < 360.0) {
        return Err(ExportError::Scenario(format!(
            "sensor {name:?}: a field of view of {fov_degrees}° is not a lens"
        )));
    }
    let half_angle = fov_degrees.to_radians() / 2.0;
    let focal = (width as f64 / 2.0) / half_angle;
    Ok(match polynomial_type {
        PolynomialType::AngleToPixelDistance => vec![0.0, focal, 0.0, 0.0, 0.0, 0.0],
        // The same lens read the other way round: pixels in, angle out.
        PolynomialType::PixelDistanceToAngle => vec![0.0, 1.0 / focal, 0.0, 0.0, 0.0, 0.0],
    })
}

// --------------------------------------------------------------------------- //
// The rig
// --------------------------------------------------------------------------- //

/// The `rig_json` a clip's calibration table carries.
///
/// Properties are written as strings because that is how a rig file writes them and
/// how the reader takes them back (`float(props["cx"])`, `poly_str.split()`).
pub fn rig_json(sensors: &[Sensor]) -> String {
    let sensors: Vec<serde_json::Value> = sensors
        .iter()
        .map(|sensor| {
            let polynomial = sensor
                .polynomial
                .iter()
                .map(|value| format!("{value}"))
                .collect::<Vec<_>>()
                .join(" ");
            serde_json::json!({
                "name": sensor.name,
                "protocol": "camera.virtual",
                "properties": {
                    "cx": sensor.cx.to_string(),
                    "cy": sensor.cy.to_string(),
                    "width": sensor.width.to_string(),
                    "height": sensor.height.to_string(),
                    "polynomial": polynomial,
                    "polynomial-type": sensor.polynomial_type.as_str(),
                    "linear-c": sensor.linear[0].to_string(),
                    "linear-d": sensor.linear[1].to_string(),
                    "linear-e": sensor.linear[2].to_string(),
                },
                "nominalSensor2Rig_FLU": {
                    "t": sensor.position,
                    "roll-pitch-yaw": sensor.roll_pitch_yaw,
                },
            })
        })
        .collect();
    serde_json::json!({ "rig": { "sensors": sensors } }).to_string()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn load(text: &str) -> Result<ClipConfig, ExportError> {
        from_yaml_str(text, ClipConfig::new("base"))
    }

    #[test]
    fn a_scenario_only_changes_what_it_mentions() {
        let config = load("speed: 25.0\n").unwrap();
        assert_eq!(config.speed, 25.0);
        assert_eq!(config.clip_id, "base", "untouched");
        assert_eq!(config.frame_rate, ClipConfig::default().frame_rate);
        assert!(config.sensors.is_empty());
    }

    #[test]
    fn a_lane_may_be_named_with_or_without_its_prefix() {
        assert_eq!(lane_id("lane/north/0"), lane_id("north/0"));
        assert_eq!(lane_id("lane/north/0").as_str(), "lane/north/0");
    }

    #[test]
    fn a_route_is_either_a_start_or_a_list() {
        let from = load("route:\n  start: lane/north/0\n").unwrap();
        assert_eq!(
            from.route,
            Some(Route::From(LaneId::from_raw("lane/north/0")))
        );

        let listed = load("route:\n  lanes: [lane/a/0, lane/b/0]\n").unwrap();
        assert_eq!(
            listed.route,
            Some(Route::Lanes(vec![
                LaneId::from_raw("lane/a/0"),
                LaneId::from_raw("lane/b/0"),
            ]))
        );

        for bad in [
            "route:\n  start: lane/a/0\n  lanes: [lane/b/0]\n",
            "route: {}\n",
            "route:\n  lanes: []\n",
        ] {
            assert!(load(bad).is_err(), "{bad:?}");
        }
    }

    #[test]
    fn a_mistyped_key_is_an_error_rather_than_a_silence() {
        let error = load("speed_kph: 40.0\n").unwrap_err();
        assert!(
            format!("{error}").contains("speed_kph"),
            "the message should name the key: {error}"
        );
    }

    #[test]
    fn a_field_of_view_becomes_an_equidistant_lens() {
        let config = load(
            "sensors:\n  - name: camera:front\n    position: [1.7, 0.0, 1.45]\n\
             \x20   width: 1920\n    height: 1080\n    fov_degrees: 120.0\n",
        )
        .unwrap();
        let sensor = &config.sensors[0];
        // r = f·θ, and the half-angle has to land on the edge of the frame.
        let focal = sensor.polynomial[1];
        assert!((focal * 60.0_f64.to_radians() - 960.0).abs() < 1e-9);
        assert_eq!(sensor.polynomial[0], 0.0);
        assert_eq!(sensor.polynomial_type, PolynomialType::AngleToPixelDistance);
        // The principal point defaults to the middle of the frame.
        assert_eq!((sensor.cx, sensor.cy), (960.0, 540.0));
        assert_eq!(sensor.roll_pitch_yaw, [0.0; 3]);
        assert_eq!(sensor.linear, [1.0, 0.0, 0.0]);
    }

    #[test]
    fn a_bare_sensor_name_is_made_discoverable() {
        let config = load(
            "sensors:\n  - name: front_wide\n    position: [0.0, 0.0, 1.0]\n\
             \x20   width: 100\n    height: 100\n    fov_degrees: 90.0\n",
        )
        .unwrap();
        assert_eq!(config.sensors[0].name, "camera:front_wide");
        assert_eq!(config.sensors[0].canonical_name(), "camera_front_wide");
    }

    #[test]
    fn a_sensor_without_a_lens_is_refused() {
        let error = load(
            "sensors:\n  - name: camera:front\n    position: [0.0, 0.0, 1.0]\n\
             \x20   width: 100\n    height: 100\n",
        )
        .unwrap_err();
        assert!(format!("{error}").contains("no lens"), "{error}");

        let both = load(
            "sensors:\n  - name: camera:front\n    position: [0.0, 0.0, 1.0]\n\
             \x20   width: 100\n    height: 100\n    fov_degrees: 90.0\n\
             \x20   polynomial: [0.0, 100.0]\n",
        )
        .unwrap_err();
        assert!(format!("{both}").contains("only one of them"), "{both}");
    }

    #[test]
    fn a_non_camera_sensor_is_refused_because_nothing_would_read_it() {
        let error = load(
            "sensors:\n  - name: lidar:top\n    position: [0.0, 0.0, 2.0]\n\
             \x20   width: 100\n    height: 100\n    fov_degrees: 90.0\n",
        )
        .unwrap_err();
        assert!(format!("{error}").contains("camera:"), "{error}");
    }

    #[test]
    fn the_rig_carries_every_field_a_reader_asks_for() {
        let config = load(
            "sensors:\n  - name: camera:front\n    position: [1.7, 0.1, 1.45]\n\
             \x20   roll_pitch_yaw: [0.0, -2.0, 1.0]\n    width: 1920\n    height: 1080\n\
             \x20   fov_degrees: 120.0\n",
        )
        .unwrap();
        let rig: serde_json::Value = serde_json::from_str(&rig_json(&config.sensors)).unwrap();
        let sensor = &rig["rig"]["sensors"][0];
        assert_eq!(sensor["name"], "camera:front");

        let properties = &sensor["properties"];
        assert_eq!(properties["width"], "1920");
        assert_eq!(properties["polynomial-type"], "angle-to-pixeldistance");
        // Space-separated, because that is how it is read back.
        let coefficients: Vec<f64> = properties["polynomial"]
            .as_str()
            .unwrap()
            .split_whitespace()
            .map(|value| value.parse().unwrap())
            .collect();
        assert_eq!(coefficients.len(), 6);
        assert!((coefficients[1] * 60.0_f64.to_radians() - 960.0).abs() < 1e-9);

        let pose = &sensor["nominalSensor2Rig_FLU"];
        assert_eq!(pose["t"][0], 1.7);
        assert_eq!(pose["roll-pitch-yaw"][1], -2.0);
    }

    #[test]
    fn an_empty_rig_is_still_a_rig() {
        let rig: serde_json::Value = serde_json::from_str(&rig_json(&[])).unwrap();
        assert!(rig["rig"]["sensors"].as_array().unwrap().is_empty());
    }
}
