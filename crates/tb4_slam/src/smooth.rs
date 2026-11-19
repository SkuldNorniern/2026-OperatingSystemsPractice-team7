use tb4_core::Pose2;

use crate::se2::wrap;

/// Blends a published map->odom towards the estimator's latest one, so the robot's map pose glides
/// instead of hopping on every scan. Corrections larger than the jump limits are applied at once.
#[derive(Debug, Clone)]
pub struct PoseSmoother {
    /// time constant (s), 0 = no smoothing
    pub tau: f64,
    pub jump_lin: f64,
    pub jump_ang: f64,
    cur: Option<Pose2>,
}

impl PoseSmoother {
    pub fn new(tau: f64, jump_lin: f64, jump_ang: f64) -> Self {
        Self { tau, jump_lin, jump_ang, cur: None }
    }

    /// advance by `dt` seconds towards `target`, returns the pose to publish
    pub fn step(&mut self, target: Pose2, dt: f64) -> Pose2 {
        let cur = match self.cur {
            Some(c)
                if self.tau > 0.0
                    && c.distance_to(target.x, target.y) <= self.jump_lin
                    && wrap(target.yaw - c.yaw).abs() <= self.jump_ang =>
            {
                let k = 1.0 - (-dt.max(0.0) / self.tau).exp();
                Pose2::new(
                    c.x + k * (target.x - c.x),
                    c.y + k * (target.y - c.y),
                    wrap(c.yaw + k * wrap(target.yaw - c.yaw)),
                )
            }
            _ => target,
        };
        self.cur = Some(cur);
        cur
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn blends_small_and_jumps_large() {
        let mut s = PoseSmoother::new(0.3, 0.2, 0.1);
        assert_eq!(s.step(Pose2::new(1.0, 0.0, 0.0), 0.05).x, 1.0);
        let t = Pose2::new(1.05, 0.0, 0.02);
        let p = s.step(t, 0.05);
        assert!(p.x > 1.0 && p.x < 1.02 && p.yaw > 0.0 && p.yaw < 0.01, "{p:?}");
        let mut p = p;
        for _ in 0..40 {
            p = s.step(t, 0.05);
        }
        assert!((p.x - 1.05).abs() < 1e-3 && (p.yaw - 0.02).abs() < 1e-3);
        let far = Pose2::new(2.0, 0.0, 0.02);
        assert_eq!(s.step(far, 0.05).x, 2.0);
        let mut off = PoseSmoother::new(0.0, 0.2, 0.1);
        off.step(Pose2::default(), 0.05);
        assert_eq!(off.step(t, 0.05).x, 1.05);
    }
}
