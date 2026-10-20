//! The drive: where the vehicle and its cameras are at each frame, and what the
//! cameras are.
//!
//! RDS-HQ is a dataset of drives, so a clip is built around an ego track the same
//! way a ClipGT clip is — the same route, the same speed, the same frames — and the
//! track is the one [`roadgen_clipgt`] computes, not a second one that happens to
//! look alike.
//!
//! Poses are 4 × 4 matrices taking a frame's own coordinates into the map's:
//!
//! * `vehicle_pose` is the rig, forward-left-up, at the track's point on the road;
//! * `pose` is each camera, in OpenCV's convention — x right, y down, z forward —
//!   which is what the renderer projects with.

use roadgen_clipgt::ego::Pose;
use roadgen_clipgt::{ClipConfig, PolynomialType, Sensor};
use roadgen_core::ValidatedMap;

use crate::error::ExportError;
use crate::wds::{npy, npy_matrix, Sample};

/// How many coefficients an f-theta polynomial has in an RDS-HQ intrinsics array.
const FTHETA_COEFFICIENTS: usize = 6;

/// The camera a clip is given when its scenario names none: the front camera the
/// toolkit's single-view dataset configuration renders, by that name, as the
/// equidistant 120° lens the name describes, mounted where ClipGT's example rig puts
/// it.
pub fn default_sensors() -> Vec<Sensor> {
    let width = 1920;
    let focal = (width as f64 / 2.0) / 60.0_f64.to_radians();
    vec![Sensor {
        name: "camera:front_wide_120fov".into(),
        position: [1.7, 0.0, 1.45],
        roll_pitch_yaw: [0.0; 3],
        width,
        height: 1080,
        cx: 960.0,
        cy: 540.0,
        polynomial: vec![0.0, focal, 0.0, 0.0, 0.0, 0.0],
        polynomial_type: PolynomialType::AngleToPixelDistance,
        linear: [1.0, 0.0, 0.0],
    }]
}

/// The sensors a clip is written with: the scenario's, or the default rig.
pub fn sensors(config: &ClipConfig) -> Vec<Sensor> {
    if config.sensors.is_empty() {
        default_sensors()
    } else {
        config.sensors.clone()
    }
}

/// The scene's shards, as `(folder, sample)`, in the order they are written.
pub fn all(
    map: &ValidatedMap,
    config: &ClipConfig,
    clip: &str,
) -> Result<Vec<(&'static str, Sample)>, ExportError> {
    let poses = roadgen_clipgt::layers::poses(map, config)?;
    let sensors = sensors(config);

    let mut vehicle = Sample::new(clip);
    let mut cameras = Sample::new(clip);
    let mut timestamps = Sample::new(clip);
    let mut objects = Sample::new(clip);
    for (frame, pose) in poses.iter().enumerate() {
        let rig = rig_to_world(pose);
        vehicle.push(format!("{frame:06}.vehicle_pose.npy"), npy_matrix(&rig));
        for sensor in &sensors {
            let camera = opencv_from_flu(&multiply(&rig, &sensor_to_rig(sensor)));
            cameras.push(
                format!("{frame:06}.pose.{}.npy", sensor.canonical_name()),
                npy_matrix(&camera),
            );
        }
        timestamps.push(
            format!("{frame:06}.timestamp_micros.txt"),
            pose.timestamp_micros.to_string().into_bytes(),
        );
        // The IR has no traffic, so every frame has no objects in it — written out
        // rather than left missing, because a reader fills a missing frame by
        // interpolating between the frames either side of it.
        objects.push_json(
            format!("{frame:06}.all_object_info.json"),
            &serde_json::json!({}),
        );
    }

    let mut ftheta = Sample::new(clip);
    let mut pinhole = Sample::new(clip);
    for sensor in &sensors {
        let name = sensor.canonical_name();
        ftheta.push(
            format!("ftheta_intrinsic.{name}.npy"),
            npy(&[14], &ftheta_intrinsic(sensor)?),
        );
        if let Some(intrinsic) = pinhole_intrinsic(sensor) {
            pinhole.push(
                format!("pinhole_intrinsic.{name}.npy"),
                npy(&[6], &intrinsic),
            );
        }
    }

    Ok(vec![
        ("pose", cameras),
        ("vehicle_pose", vehicle),
        ("timestamp", timestamps),
        ("all_object_info", objects),
        ("ftheta_intrinsic", ftheta),
        ("pinhole_intrinsic", pinhole),
    ])
}

/// `cx, cy, width, height`, the six polynomial coefficients, whether the polynomial
/// runs backward — pixel distance to angle — and the affine term `c, d, e`.
pub fn ftheta_intrinsic(sensor: &Sensor) -> Result<Vec<f64>, ExportError> {
    if sensor.polynomial[FTHETA_COEFFICIENTS.min(sensor.polynomial.len())..]
        .iter()
        .any(|value| *value != 0.0)
    {
        return Err(ExportError::Camera(format!(
            "{}'s polynomial has {} coefficients, and an RDS-HQ intrinsics array holds \
             {FTHETA_COEFFICIENTS}: the higher terms cannot be dropped without \
             changing the lens",
            sensor.name,
            sensor.polynomial.len()
        )));
    }
    let mut values = vec![
        sensor.cx,
        sensor.cy,
        sensor.width as f64,
        sensor.height as f64,
    ];
    values
        .extend((0..FTHETA_COEFFICIENTS).map(|i| sensor.polynomial.get(i).copied().unwrap_or(0.0)));
    values.push(match sensor.polynomial_type {
        PolynomialType::AngleToPixelDistance => 0.0,
        PolynomialType::PixelDistanceToAngle => 1.0,
    });
    values.extend(sensor.linear);
    Ok(values)
}

/// `fx, fy, cx, cy, width, height` of the pinhole with the f-theta lens's focal
/// length at the centre of the frame, or `None` for a lens that has none.
///
/// Near the axis an f-theta lens and a pinhole agree — `r ≈ f·θ ≈ f·tan θ` — so this
/// is the pinhole that matches the camera in the middle of the frame. Further out
/// they part: the pinhole sees less of the world in the same frame, which is what a
/// rectified fisheye looks like too.
pub fn pinhole_intrinsic(sensor: &Sensor) -> Option<[f64; 6]> {
    let slope = sensor.polynomial.get(1).copied().unwrap_or(0.0);
    let focal = match sensor.polynomial_type {
        PolynomialType::AngleToPixelDistance => slope,
        PolynomialType::PixelDistanceToAngle => 1.0 / slope,
    };
    (focal.is_finite() && focal > 0.0).then_some([
        focal,
        focal,
        sensor.cx,
        sensor.cy,
        sensor.width as f64,
        sensor.height as f64,
    ])
}

type Matrix = [[f64; 4]; 4];

/// The rig's pose: the track's orientation, a unit quaternion `(x, y, z, w)` taking
/// forward-left-up into the map, and its position.
fn rig_to_world(pose: &Pose) -> Matrix {
    let [x, y, z, w] = pose.orientation;
    let p = pose.position;
    [
        [
            1.0 - 2.0 * (y * y + z * z),
            2.0 * (x * y - z * w),
            2.0 * (x * z + y * w),
            p.x,
        ],
        [
            2.0 * (x * y + z * w),
            1.0 - 2.0 * (x * x + z * z),
            2.0 * (y * z - x * w),
            p.y,
        ],
        [
            2.0 * (x * z - y * w),
            2.0 * (y * z + x * w),
            1.0 - 2.0 * (x * x + y * y),
            p.z,
        ],
        [0.0, 0.0, 0.0, 1.0],
    ]
}

/// Where a camera sits on the rig, forward-left-up: its position, and its roll,
/// pitch and yaw in degrees as an extrinsic `xyz` Euler triple — roll about the
/// rig's x first, yaw about its z last.
pub fn sensor_to_rig(sensor: &Sensor) -> Matrix {
    let [roll, pitch, yaw] = sensor.roll_pitch_yaw.map(f64::to_radians);
    let (sr, cr) = roll.sin_cos();
    let (sp, cp) = pitch.sin_cos();
    let (sy, cy) = yaw.sin_cos();
    let [tx, ty, tz] = sensor.position;
    // Rz(yaw) · Ry(pitch) · Rx(roll).
    [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, tx],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, ty],
        [-sp, cp * sr, cp * cr, tz],
        [0.0, 0.0, 0.0, 1.0],
    ]
}

/// The same pose with its axes renamed from forward-left-up to OpenCV's
/// right-down-forward: right is −left, down is −up, forward is forward.
fn opencv_from_flu(pose: &Matrix) -> Matrix {
    let mut out = *pose;
    for row in out.iter_mut().take(3) {
        let (forward, left, up) = (row[0], row[1], row[2]);
        row[0] = -left;
        row[1] = -up;
        row[2] = forward;
    }
    out
}

fn multiply(a: &Matrix, b: &Matrix) -> Matrix {
    let mut out = [[0.0; 4]; 4];
    for (i, row) in out.iter_mut().enumerate() {
        for (j, cell) in row.iter_mut().enumerate() {
            *cell = (0..4).map(|k| a[i][k] * b[k][j]).sum();
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use roadgen_core::geometry::Point3;

    fn column(matrix: &Matrix, j: usize) -> [f64; 3] {
        [matrix[0][j], matrix[1][j], matrix[2][j]]
    }

    fn close(a: [f64; 3], b: [f64; 3]) -> bool {
        a.iter().zip(b).all(|(x, y)| (x - y).abs() < 1e-9)
    }

    #[test]
    fn a_forward_camera_looks_down_the_rigs_x_axis() {
        let pose = Pose {
            timestamp_micros: 0,
            position: Point3::new(10.0, 20.0, 3.0),
            // Yaw of a quarter turn: the vehicle faces +y.
            orientation: [
                0.0,
                0.0,
                std::f64::consts::FRAC_1_SQRT_2,
                std::f64::consts::FRAC_1_SQRT_2,
            ],
        };
        let sensor = &default_sensors()[0];
        let camera = opencv_from_flu(&multiply(&rig_to_world(&pose), &sensor_to_rig(sensor)));
        // OpenCV z is the viewing direction, along the vehicle's heading.
        assert!(close(column(&camera, 2), [0.0, 1.0, 0.0]));
        // y is down, x is to the vehicle's right — which, facing +y, is +x.
        assert!(close(column(&camera, 1), [0.0, 0.0, -1.0]));
        assert!(close(column(&camera, 0), [1.0, 0.0, 0.0]));
        // 1.7 m ahead of the rig and 1.45 m up.
        assert!(close(column(&camera, 3), [10.0, 21.7, 4.45]));
    }

    #[test]
    fn a_yaw_of_half_a_turn_looks_backward() {
        let mut sensor = default_sensors().remove(0);
        sensor.roll_pitch_yaw = [0.0, 0.0, 180.0];
        let rotation = sensor_to_rig(&sensor);
        assert!(close(column(&rotation, 0), [-1.0, 0.0, 0.0]));
        assert!(close(column(&rotation, 2), [0.0, 0.0, 1.0]));
    }

    #[test]
    fn the_intrinsics_are_in_the_order_the_reader_takes_them() {
        let sensor = &default_sensors()[0];
        let values = ftheta_intrinsic(sensor).unwrap();
        assert_eq!(values.len(), 14);
        assert_eq!(&values[..4], &[960.0, 540.0, 1920.0, 1080.0]);
        assert_eq!(values[5], sensor.polynomial[1]);
        assert_eq!(
            values[10], 0.0,
            "angle to pixel distance is the forward polynomial"
        );
        assert_eq!(&values[11..], &[1.0, 0.0, 0.0]);

        let pinhole = pinhole_intrinsic(sensor).unwrap();
        assert_eq!(pinhole[0], sensor.polynomial[1]);
        assert_eq!(&pinhole[2..], &[960.0, 540.0, 1920.0, 1080.0]);
    }

    #[test]
    fn a_polynomial_longer_than_the_array_is_refused() {
        let mut sensor = default_sensors().remove(0);
        sensor.polynomial = vec![0.0, 1000.0, 0.0, 0.0, 0.0, 0.0, 1e-9];
        assert!(ftheta_intrinsic(&sensor).is_err());
        // Trailing zeros are not a higher term.
        sensor.polynomial = vec![0.0, 1000.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        assert!(ftheta_intrinsic(&sensor).is_ok());
    }
}
