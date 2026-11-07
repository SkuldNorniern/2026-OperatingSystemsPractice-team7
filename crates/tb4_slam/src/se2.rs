use std::f64::consts::{PI, TAU};

use tb4_core::Pose2;

pub fn wrap(a: f64) -> f64 {
    let a = (a + PI).rem_euclid(TAU) - PI;
    if a < -PI { a + TAU } else { a }
}

pub fn compose(a: Pose2, b: Pose2) -> Pose2 {
    let (s, c) = a.yaw.sin_cos();
    Pose2::new(a.x + c * b.x - s * b.y, a.y + s * b.x + c * b.y, wrap(a.yaw + b.yaw))
}

pub fn inverse(a: Pose2) -> Pose2 {
    let (s, c) = a.yaw.sin_cos();
    Pose2::new(-(c * a.x + s * a.y), s * a.x - c * a.y, wrap(-a.yaw))
}

/// a^-1 * b
pub fn between(a: Pose2, b: Pose2) -> Pose2 {
    compose(inverse(a), b)
}

pub fn apply(a: Pose2, p: (f64, f64)) -> (f64, f64) {
    let (s, c) = a.yaw.sin_cos();
    (a.x + c * p.0 - s * p.1, a.y + s * p.0 + c * p.1)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn inverse_roundtrip() {
        let a = Pose2::new(1.0, -2.0, 2.5);
        let b = Pose2::new(-0.3, 0.7, -3.0);
        let r = compose(a, between(a, b));
        assert!((r.x - b.x).abs() < 1e-12 && (r.y - b.y).abs() < 1e-12);
        assert!(wrap(r.yaw - b.yaw).abs() < 1e-12);
        let i = compose(a, inverse(a));
        assert!(i.x.abs() < 1e-12 && i.y.abs() < 1e-12 && i.yaw.abs() < 1e-12);
        assert!((wrap(3.0 * PI) - PI).abs() < 1e-12 || (wrap(3.0 * PI) + PI).abs() < 1e-12);
    }
}
