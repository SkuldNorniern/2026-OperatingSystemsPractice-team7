//! The command chain after the navigator in team7 mode:
//! cmd_vel_nav -> velocity_smoother -> collision_monitor (approach) -> velocity_guard -> cmd_vel.

use crate::robot::integrate;
use tb4_core::Pose2;

/// nav2 velocity_smoother, OPEN_LOOP, nav2.yaml values
#[derive(Debug, Clone)]
pub struct SmootherParams {
    pub max_v: f64,
    pub min_v: f64,
    pub max_w: f64,
    pub accel_v: f64,
    pub decel_v: f64,
    pub accel_w: f64,
    pub decel_w: f64,
    /// s without input after which the output target is zero
    pub velocity_timeout: f64,
}

impl Default for SmootherParams {
    fn default() -> Self {
        Self { max_v: 0.26, min_v: 0.0, max_w: 0.7, accel_v: 0.3, decel_v: 0.5, accel_w: 0.6, decel_w: 0.8, velocity_timeout: 1.0 }
    }
}

/// collision_monitor_team7.yaml: one circle, action "approach"
#[derive(Debug, Clone)]
pub struct MonitorParams {
    pub radius: f64,
    pub time_before_collision: f64,
    pub step: f64,
    pub min_points: usize,
}

impl Default for MonitorParams {
    fn default() -> Self {
        Self { radius: 0.20, time_before_collision: 1.2, step: 0.1, min_points: 6 }
    }
}

/// velocity_guard: forward only, Create 3 limits
#[derive(Debug, Clone)]
pub struct GuardParams {
    pub max_forward: f64,
    pub max_turn: f64,
    pub command_timeout: f64,
}

impl Default for GuardParams {
    fn default() -> Self {
        Self { max_forward: 0.26, max_turn: 0.7, command_timeout: 0.5 }
    }
}

#[derive(Debug, Clone, Default)]
pub struct ChainParams {
    pub smoother: SmootherParams,
    pub monitor: MonitorParams,
    pub guard: GuardParams,
}

#[derive(Debug, Clone)]
pub struct Chain {
    pub p: ChainParams,
    input: (f64, f64),
    input_at: f64,
    out: (f64, f64),
    /// last collision monitor scale (1 = no change)
    pub monitor_ratio: f64,
}

fn ramp(last: f64, target: f64, accel: f64, decel: f64, dt: f64) -> f64 {
    let speeding_up = target.abs() > last.abs() && target * last >= 0.0;
    let lim = if speeding_up { accel } else { decel } * dt;
    last + (target - last).clamp(-lim, lim)
}

impl Chain {
    pub fn new(p: ChainParams) -> Self {
        Self { p, input: (0.0, 0.0), input_at: f64::NEG_INFINITY, out: (0.0, 0.0), monitor_ratio: 1.0 }
    }

    /// navigator published a command
    pub fn command(&mut self, v: f64, w: f64, now: f64) {
        self.input = (v, w);
        self.input_at = now;
    }

    /// one smoother period (20 Hz): returns the cmd_vel the base gets; `scan` are lidar points in the base frame
    pub fn update(&mut self, now: f64, dt: f64, scan: &[(f64, f64)]) -> (f64, f64) {
        let s = &self.p.smoother;
        let target = if now - self.input_at > s.velocity_timeout { (0.0, 0.0) } else { self.input };
        let tv = target.0.clamp(s.min_v, s.max_v);
        let tw = target.1.clamp(-s.max_w, s.max_w);
        self.out = (ramp(self.out.0, tv, s.accel_v, s.decel_v, dt), ramp(self.out.1, tw, s.accel_w, s.decel_w, dt));
        let ratio = approach_ratio(self.out, scan, &self.p.monitor);
        self.monitor_ratio = ratio;
        let (v, w) = (self.out.0 * ratio, self.out.1 * ratio);
        let g = &self.p.guard;
        // the smoother publishes every period, so the guard never sees a stale command here
        (v.clamp(0.0, g.max_forward), w.clamp(-g.max_turn, g.max_turn))
    }
}

/// nav2 collision_monitor "approach": simulate the command forward, if `min_points` scan points enter the
/// circle at time t <= time_before_collision, scale the command by t / time_before_collision
pub fn approach_ratio(cmd: (f64, f64), scan: &[(f64, f64)], p: &MonitorParams) -> f64 {
    if cmd.0 == 0.0 && cmd.1 == 0.0 {
        return 1.0;
    }
    let r2 = p.radius * p.radius;
    let mut t = p.step;
    while t <= p.time_before_collision + 1e-9 {
        let q = integrate(Pose2::default(), cmd.0, cmd.1, t);
        let n = scan.iter().filter(|s| (s.0 - q.x).powi(2) + (s.1 - q.y).powi(2) <= r2).count();
        if n >= p.min_points {
            return t / p.time_before_collision;
        }
        t += p.step;
    }
    1.0
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn smoother_ramps_with_accel_limits() {
        let mut c = Chain::new(ChainParams::default());
        c.command(0.18, 0.5, 0.0);
        let (v, w) = c.update(0.0, 0.05, &[]);
        assert!((v - 0.015).abs() < 1e-9 && (w - 0.03).abs() < 1e-9);
        let mut t = 0.0;
        for _ in 0..40 {
            t += 0.05;
            c.command(0.18, 0.5, t);
            c.update(t, 0.05, &[]);
        }
        let (v, w) = c.update(t, 0.05, &[]);
        assert!((v - 0.18).abs() < 1e-9 && (w - 0.5).abs() < 1e-9);
        // no input for > 1 s: decelerates to zero
        for _ in 0..40 {
            t += 0.05;
            c.update(t, 0.05, &[]);
        }
        assert_eq!(c.update(t, 0.05, &[]), (0.0, 0.0));
    }

    #[test]
    fn approach_slows_before_a_wall() {
        let p = MonitorParams::default();
        let wall: Vec<(f64, f64)> = (0..20).map(|i| (0.5, -0.2 + i as f64 * 0.02)).collect();
        // 0.18 m/s: circle of 0.2 m reaches x = 0.5 after 0.3 m = 1.67 s, beyond 1.2 s
        assert_eq!(approach_ratio((0.18, 0.0), &wall, &p), 1.0);
        let near: Vec<(f64, f64)> = wall.iter().map(|q| (q.0 - 0.25, q.1)).collect();
        let r = approach_ratio((0.18, 0.0), &near, &p);
        assert!(r > 0.0 && r < 0.6, "{r}");
        // turning in place with a far wall: unchanged
        assert_eq!(approach_ratio((0.0, 0.5), &wall, &p), 1.0);
    }

    #[test]
    fn guard_drops_reverse() {
        let mut c = Chain::new(ChainParams::default());
        c.p.smoother.min_v = -0.2;
        c.command(-0.1, 0.0, 0.0);
        assert_eq!(c.update(0.0, 0.05, &[]).0, 0.0);
    }
}
