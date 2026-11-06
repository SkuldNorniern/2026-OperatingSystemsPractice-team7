use nalgebra::Isometry3;

#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Pose2 {
    pub x: f64,
    pub y: f64,
    pub yaw: f64,
}

impl Pose2 {
    pub const fn new(x: f64, y: f64, yaw: f64) -> Self {
        Self { x, y, yaw }
    }

    pub fn from_isometry(iso: &Isometry3<f64>) -> Self {
        let (_, _, yaw) = iso.rotation.euler_angles();
        Self::new(iso.translation.x, iso.translation.y, yaw)
    }

    pub fn distance_to(&self, x: f64, y: f64) -> f64 {
        (self.x - x).hypot(self.y - y)
    }
}

pub fn yaw_to_quaternion(yaw: f64) -> [f64; 4] {
    let half = yaw * 0.5;
    [0.0, 0.0, half.sin(), half.cos()]
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::{Translation3, UnitQuaternion};

    #[test]
    fn isometry_projection_keeps_yaw() {
        let iso = Isometry3::from_parts(
            Translation3::new(1.0, 2.0, 0.3),
            UnitQuaternion::from_euler_angles(0.0, 0.0, 0.7),
        );
        let p = Pose2::from_isometry(&iso);
        assert!((p.x - 1.0).abs() < 1e-12 && (p.y - 2.0).abs() < 1e-12);
        assert!((p.yaw - 0.7).abs() < 1e-12);
    }

    #[test]
    fn yaw_quaternion_is_unit() {
        let [x, y, z, w] = yaw_to_quaternion(1.234);
        assert!(((x * x + y * y + z * z + w * w) - 1.0).abs() < 1e-12);
    }
}
