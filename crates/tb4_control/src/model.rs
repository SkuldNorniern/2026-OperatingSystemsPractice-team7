use std::f64::consts::{PI, TAU};

use tb4_core::Pose2;

use crate::params::MppiParams;

#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Control {
    pub v: f64,
    pub w: f64,
}

impl Control {
    pub const ZERO: Control = Control { v: 0.0, w: 0.0 };

    pub const fn new(v: f64, w: f64) -> Self {
        Self { v, w }
    }
}

#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct RobotState {
    pub pose: Pose2,
    pub v: f64,
    pub w: f64,
}

impl RobotState {
    pub fn control(&self) -> Control {
        Control::new(self.v, self.w)
    }
}

#[inline]
pub fn wrap_angle(a: f64) -> f64 {
    // rollouts call this every step with angles already in range, skip fmod then
    if (-PI..PI).contains(&a) {
        return a;
    }
    let a = (a + PI).rem_euclid(TAU) - PI;
    if a < -PI { a + TAU } else { a }
}

/// unicycle step with midpoint heading
#[inline]
pub fn step(pose: &Pose2, v: f64, w: f64, dt: f64) -> Pose2 {
    let (s, c) = (pose.yaw + 0.5 * w * dt).sin_cos();
    Pose2::new(pose.x + v * c * dt, pose.y + v * s * dt, wrap_angle(pose.yaw + w * dt))
}

/// clamp `want` to [v_lo, v_hi] and accel limits from `prev`; hard bounds win if `prev` is out of range
#[inline]
pub fn limit_control(prev: Control, want: Control, p: &MppiParams, v_lo: f64, v_hi: f64) -> Control {
    let dv = p.ax_max * p.dt;
    let dw = p.aw_max * p.dt;
    let v = want.v.clamp(v_lo, v_hi).clamp(prev.v - dv, prev.v + dv).clamp(p.vx_min, p.vx_max);
    let w = want.w.clamp(-p.wz_max, p.wz_max).clamp(prev.w - dw, prev.w + dw).clamp(-p.wz_max, p.wz_max);
    Control::new(v, w)
}

/// applies limits to `controls` and integrates; returns the states after each step
pub fn rollout(state: &RobotState, controls: &[Control], p: &MppiParams) -> Vec<RobotState> {
    let mut prev = state.control();
    let mut pose = state.pose;
    controls
        .iter()
        .map(|&u| {
            let c = limit_control(prev, u, p, p.vx_min, p.vx_max);
            pose = step(&pose, c.v, c.w, p.dt);
            prev = c;
            RobotState { pose, v: c.v, w: c.w }
        })
        .collect()
}

/// ideal velocity tracking for one control period
pub fn simulate(state: &RobotState, cmd: Control, dt: f64) -> RobotState {
    RobotState { pose: step(&state.pose, cmd.v, cmd.w, dt), v: cmd.v, w: cmd.w }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn wrap() {
        for a in [-10.0, -PI, -1.0, 0.0, 1.0, PI, 7.0] {
            let w = wrap_angle(a);
            assert!((-PI..=PI).contains(&w));
            assert!(((a - w) / TAU - ((a - w) / TAU).round()).abs() < 1e-9);
        }
    }

    #[test]
    fn rollout_respects_limits() {
        let p = MppiParams::default();
        let s = RobotState::default();
        let traj = rollout(&s, &[Control::new(5.0, -5.0); 30], &p);
        let mut prev = s.control();
        for t in &traj {
            assert!(t.v >= 0.0 && t.v <= p.vx_max + 1e-12 && t.w.abs() <= p.wz_max + 1e-12);
            assert!((t.v - prev.v).abs() <= p.ax_max * p.dt + 1e-12);
            assert!((t.w - prev.w).abs() <= p.aw_max * p.dt + 1e-12);
            prev = t.control();
        }
        assert!((traj.last().unwrap().v - p.vx_max).abs() < 1e-12);
    }

    #[test]
    fn straight_and_arc() {
        let p = step(&Pose2::default(), 1.0, 0.0, 0.5);
        assert!((p.x - 0.5).abs() < 1e-12 && p.y.abs() < 1e-12);
        let mut q = Pose2::default();
        for _ in 0..1000 {
            q = step(&q, 1.0, 1.0, PI / 1000.0);
        }
        assert!(q.x.abs() < 1e-6 && (q.y - 2.0).abs() < 1e-6);
    }
}
