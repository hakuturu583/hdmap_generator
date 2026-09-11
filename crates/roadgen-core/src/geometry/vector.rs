//! Points, vectors and the road local frame.
//!
//! Everything here is three-dimensional. There is deliberately no 2D point type: a
//! caller that needs a planar computation projects into a [`Frame3`] first (see
//! [`Frame3::to_local`]) and lifts the result back with [`Frame3::to_global`], so the
//! projection is always visible in the code rather than implied by a missing `z`.

use std::ops::{Add, Mul, Neg, Sub};

use crate::error::GeometryError;

/// A position in the map's metric coordinate system, metres.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Point3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Point3 {
    pub const ORIGIN: Point3 = Point3::new(0.0, 0.0, 0.0);

    pub const fn new(x: f64, y: f64, z: f64) -> Self {
        Point3 { x, y, z }
    }

    pub fn as_array(self) -> [f64; 3] {
        [self.x, self.y, self.z]
    }

    pub fn to_vector(self) -> Vector3 {
        Vector3::new(self.x, self.y, self.z)
    }

    /// Straight-line distance in three dimensions.
    pub fn distance_to(self, other: Point3) -> f64 {
        (other - self).norm()
    }

    /// Distance measured after an explicit projection onto the horizontal plane.
    ///
    /// Used where a format demands it — an OpenDRIVE `s` coordinate is defined in the
    /// xy-plane — never as a stand-in for the real distance.
    pub fn horizontal_distance_to(self, other: Point3) -> f64 {
        (other - self).horizontal_norm()
    }

    /// Whether two positions agree to within `tolerance` metres in all three axes.
    pub fn is_close(self, other: Point3, tolerance: f64) -> bool {
        self.distance_to(other) <= tolerance
    }

    pub fn lerp(self, other: Point3, t: f64) -> Point3 {
        Point3::new(
            self.x + (other.x - self.x) * t,
            self.y + (other.y - self.y) * t,
            self.z + (other.z - self.z) * t,
        )
    }
}

impl Add<Vector3> for Point3 {
    type Output = Point3;
    fn add(self, rhs: Vector3) -> Point3 {
        Point3::new(self.x + rhs.x, self.y + rhs.y, self.z + rhs.z)
    }
}

impl Sub<Vector3> for Point3 {
    type Output = Point3;
    fn sub(self, rhs: Vector3) -> Point3 {
        Point3::new(self.x - rhs.x, self.y - rhs.y, self.z - rhs.z)
    }
}

impl Sub<Point3> for Point3 {
    type Output = Vector3;
    fn sub(self, rhs: Point3) -> Vector3 {
        Vector3::new(self.x - rhs.x, self.y - rhs.y, self.z - rhs.z)
    }
}

/// A free vector, metres.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Vector3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Vector3 {
    pub const ZERO: Vector3 = Vector3::new(0.0, 0.0, 0.0);
    pub const UP: Vector3 = Vector3::new(0.0, 0.0, 1.0);

    pub const fn new(x: f64, y: f64, z: f64) -> Self {
        Vector3 { x, y, z }
    }

    pub fn norm(self) -> f64 {
        self.dot(self).sqrt()
    }

    /// Length of the vector's shadow on the horizontal plane.
    pub fn horizontal_norm(self) -> f64 {
        self.x.hypot(self.y)
    }

    pub fn dot(self, other: Vector3) -> f64 {
        self.x * other.x + self.y * other.y + self.z * other.z
    }

    pub fn cross(self, other: Vector3) -> Vector3 {
        Vector3::new(
            self.y * other.z - self.z * other.y,
            self.z * other.x - self.x * other.z,
            self.x * other.y - self.y * other.x,
        )
    }

    /// Fails rather than producing a NaN direction for a zero-length vector.
    pub fn normalize(self) -> Result<UnitVector3, GeometryError> {
        UnitVector3::try_new(self)
    }

    /// Turns the vector `angle` radians about `axis`, counter-clockwise seen from
    /// the axis's positive end.
    ///
    /// Rodrigues' formula. This is how a banked road tilts its cross-section: the
    /// lateral and vertical axes turn about the tangent and nothing else moves.
    pub fn rotated_about(self, axis: UnitVector3, angle: f64) -> Vector3 {
        let (sin, cos) = angle.sin_cos();
        let axis = axis.get();
        self * cos + axis.cross(self) * sin + axis * (axis.dot(self) * (1.0 - cos))
    }
}

impl Add for Vector3 {
    type Output = Vector3;
    fn add(self, rhs: Vector3) -> Vector3 {
        Vector3::new(self.x + rhs.x, self.y + rhs.y, self.z + rhs.z)
    }
}

impl Sub for Vector3 {
    type Output = Vector3;
    fn sub(self, rhs: Vector3) -> Vector3 {
        Vector3::new(self.x - rhs.x, self.y - rhs.y, self.z - rhs.z)
    }
}

impl Mul<f64> for Vector3 {
    type Output = Vector3;
    fn mul(self, rhs: f64) -> Vector3 {
        Vector3::new(self.x * rhs, self.y * rhs, self.z * rhs)
    }
}

impl Neg for Vector3 {
    type Output = Vector3;
    fn neg(self) -> Vector3 {
        Vector3::new(-self.x, -self.y, -self.z)
    }
}

/// A direction. The unit-length invariant is established once, by the constructor,
/// so that no downstream computation has to re-check or re-normalise.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct UnitVector3(Vector3);

impl UnitVector3 {
    /// The shortest vector that is allowed to define a direction.
    const MIN_NORM: f64 = 1e-12;

    pub fn try_new(value: Vector3) -> Result<Self, GeometryError> {
        let norm = value.norm();
        if !norm.is_finite() || norm < Self::MIN_NORM {
            return Err(GeometryError::ZeroLengthVector);
        }
        Ok(UnitVector3(value * (1.0 / norm)))
    }

    pub fn get(self) -> Vector3 {
        self.0
    }

    pub fn x(self) -> f64 {
        self.0.x
    }

    pub fn y(self) -> f64 {
        self.0.y
    }

    pub fn z(self) -> f64 {
        self.0.z
    }

    pub fn dot(self, other: UnitVector3) -> f64 {
        self.0.dot(other.0)
    }

    pub fn scaled(self, factor: f64) -> Vector3 {
        self.0 * factor
    }

    pub fn reversed(self) -> UnitVector3 {
        UnitVector3(-self.0)
    }

    /// Heading of the direction's horizontal component, radians, measured
    /// counter-clockwise from +x. This is the angle OpenDRIVE calls `hdg`.
    pub fn heading(self) -> f64 {
        self.0.y.atan2(self.0.x)
    }

    /// Rise over horizontal run — the longitudinal slope of the direction.
    pub fn grade(self) -> f64 {
        let horizontal = self.0.horizontal_norm();
        if horizontal < Self::MIN_NORM {
            0.0
        } else {
            self.0.z / horizontal
        }
    }
}

/// The road local frame at one station of a reference line.
///
/// `tangent` runs along the reference line, `left` is horizontal and points to the
/// left of it, and `up` completes a right-handed frame. Keeping `left` horizontal is
/// what makes a lane width a width *in the road plane*; banking, when it is added,
/// will rotate `left` and `up` about `tangent` and nothing else here changes.
#[derive(Debug, Clone, Copy)]
pub struct Frame3 {
    pub origin: Point3,
    pub tangent: UnitVector3,
    pub left: UnitVector3,
    pub up: UnitVector3,
}

impl Frame3 {
    /// Builds the frame whose lateral axis is horizontal.
    ///
    /// Fails for a vertical tangent, where "left" is not defined — a road that goes
    /// straight up is not a road.
    pub fn from_tangent(origin: Point3, tangent: UnitVector3) -> Result<Self, GeometryError> {
        let t = tangent.get();
        let horizontal = t.horizontal_norm();
        if horizontal < 1e-9 {
            return Err(GeometryError::VerticalTangent);
        }
        let left = UnitVector3::try_new(Vector3::new(-t.y, t.x, 0.0))?;
        let up = UnitVector3::try_new(left.get().cross(t))?;
        Ok(Frame3 {
            origin,
            tangent,
            left,
            up,
        })
    }

    /// The same frame, rolled `angle` radians about its tangent.
    ///
    /// This is superelevation: the road surface tilts, so the lateral axis no longer
    /// lies in the horizontal plane and a lane offset along it gains height. The sign
    /// follows OpenDRIVE's `<superelevation>` — a positive angle raises the left-hand
    /// side, which is a counter-clockwise turn seen along the direction of travel.
    pub fn banked(&self, angle: f64) -> Frame3 {
        if angle == 0.0 {
            return *self;
        }
        let turn = |vector: UnitVector3| {
            UnitVector3::try_new(vector.get().rotated_about(self.tangent, angle))
                // A rotation preserves length, so the only way this could fail is an
                // input that was not a unit vector to begin with.
                .unwrap_or(vector)
        };
        Frame3 {
            origin: self.origin,
            tangent: self.tangent,
            left: turn(self.left),
            up: turn(self.up),
        }
    }

    /// Projects a global position into this frame: `[along, left, up]`, metres.
    ///
    /// This is the only sanctioned way to get from 3D geometry to a planar
    /// computation: do the 2D work on the `[along, left]` pair and call
    /// [`Frame3::to_global`] to come back.
    pub fn to_local(&self, point: Point3) -> [f64; 3] {
        let d = point - self.origin;
        [
            d.dot(self.tangent.get()),
            d.dot(self.left.get()),
            d.dot(self.up.get()),
        ]
    }

    /// The inverse of [`Frame3::to_local`].
    pub fn to_global(&self, local: [f64; 3]) -> Point3 {
        self.origin
            + self.tangent.scaled(local[0])
            + self.left.scaled(local[1])
            + self.up.scaled(local[2])
    }

    /// The position `lateral` metres to the left of the frame's origin.
    ///
    /// Negative values are to the right. This is how a lane boundary is placed
    /// against its road's reference line.
    pub fn offset(&self, lateral: f64) -> Point3 {
        self.origin + self.left.scaled(lateral)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_zero_vector_has_no_direction() {
        assert!(matches!(
            Vector3::ZERO.normalize(),
            Err(GeometryError::ZeroLengthVector)
        ));
    }

    #[test]
    fn the_lateral_axis_is_horizontal_even_on_a_slope() {
        let tangent = Vector3::new(1.0, 0.0, 0.5).normalize().unwrap();
        let frame = Frame3::from_tangent(Point3::new(0.0, 0.0, 10.0), tangent).unwrap();
        assert!(frame.left.z().abs() < 1e-12);
        assert!((frame.left.y() - 1.0).abs() < 1e-12);
        // The frame stays orthonormal, which is what `to_local` round-tripping needs.
        assert!(frame.tangent.dot(frame.left).abs() < 1e-12);
        assert!(frame.tangent.dot(frame.up).abs() < 1e-12);
    }

    #[test]
    fn a_vertical_tangent_has_no_frame() {
        let tangent = Vector3::new(0.0, 0.0, 1.0).normalize().unwrap();
        assert!(matches!(
            Frame3::from_tangent(Point3::ORIGIN, tangent),
            Err(GeometryError::VerticalTangent)
        ));
    }

    #[test]
    fn projecting_into_the_frame_and_back_is_the_identity() {
        let tangent = Vector3::new(1.0, 1.0, 0.2).normalize().unwrap();
        let frame = Frame3::from_tangent(Point3::new(3.0, -4.0, 5.0), tangent).unwrap();
        let point = Point3::new(11.0, 2.0, 9.0);
        let back = frame.to_global(frame.to_local(point));
        assert!(point.is_close(back, 1e-9));
    }

    #[test]
    fn banking_raises_the_left_hand_side() {
        let tangent = Vector3::new(1.0, 0.0, 0.0).normalize().unwrap();
        let frame = Frame3::from_tangent(Point3::ORIGIN, tangent).unwrap();
        let banked = frame.banked(0.1);

        // A positive superelevation lifts the left, the way OpenDRIVE defines it.
        assert!(banked.left.z() > 0.0);
        assert!((banked.left.z() - 0.1_f64.sin()).abs() < 1e-12);
        // The tangent is the axis of the roll, so it does not move.
        assert_eq!(banked.tangent, frame.tangent);
        // And the frame is still orthonormal.
        assert!(banked.left.dot(banked.up).abs() < 1e-12);
        assert!(banked.left.dot(banked.tangent).abs() < 1e-12);

        // A lane 5 m to the left of a road banked by 0.1 rad sits 5·sin(0.1) higher.
        let edge = banked.offset(5.0);
        assert!((edge.z - 5.0 * 0.1_f64.sin()).abs() < 1e-12);
    }

    #[test]
    fn rotating_about_an_axis_leaves_the_axis_alone() {
        let axis = Vector3::new(1.0, 2.0, 3.0).normalize().unwrap();
        let turned = axis.get().rotated_about(axis, 1.2);
        assert!((turned - axis.get()).norm() < 1e-12);

        // A quarter turn about +z takes +x to +y.
        let z = Vector3::UP.normalize().unwrap();
        let turned = Vector3::new(1.0, 0.0, 0.0).rotated_about(z, std::f64::consts::FRAC_PI_2);
        assert!((turned - Vector3::new(0.0, 1.0, 0.0)).norm() < 1e-12);
    }

    #[test]
    fn grade_is_rise_over_horizontal_run() {
        let tangent = Vector3::new(100.0, 0.0, 2.0).normalize().unwrap();
        assert!((tangent.grade() - 0.02).abs() < 1e-12);
        assert!(tangent.heading().abs() < 1e-12);
    }
}
