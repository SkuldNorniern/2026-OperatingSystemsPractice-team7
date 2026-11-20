//! Driving quality: speed while moving, time near the speed cap, stop-to-turn events, command jitter
//! (navigator output before the smoother) and jitter of the predicted MPPI trajectory.

use crate::metrics::{Motion, STILL_V};

/// share of the speed cap that counts as "at the cap"
pub const AT_CAP: f64 = 0.8;

#[derive(Debug, Clone, Default)]
pub struct DriveTracker {
    /// distance and time with |v| >= STILL_V (m, s)
    pub moving_distance: f64,
    pub moving_time: f64,
    /// time with |v| >= AT_CAP * cap
    pub time_at_cap: f64,
    /// in-place turns that follow driving (a stop, then turning, before driving again)
    pub stop_turns: u64,
    /// navigator entries into MPPI rotate-in-place mode
    pub rotate_entries: u64,
    /// navigator ticks in rotate-in-place mode (s)
    pub time_rotate_mode: f64,
    last_moving: Option<Motion>,
    rotating: bool,
    prev_cmd: Option<(f64, f64)>,
    /// |dv|, |dw| between consecutive navigator commands (20 Hz) while a goal is active
    pub dv: Vec<f64>,
    pub dw: Vec<f64>,
    prev_traj: Option<(Vec<(f64, f64)>, u64)>,
    /// endpoint movement between consecutive solves with the same path and goal (m)
    pub end_jump: Vec<f64>,
    /// same, time-aligned: this solve's pose at t+(H-1)dt vs the previous solve's at the same time (m)
    pub end_jitter: Vec<f64>,
}

impl DriveTracker {
    /// one physics step of `dt` with the measured base velocity and the distance it moved
    pub fn on_physics(&mut self, dt: f64, v: f64, motion: Motion, ds: f64, cap: f64) {
        if v.abs() >= STILL_V {
            self.moving_distance += ds;
            self.moving_time += dt;
            if v.abs() >= AT_CAP * cap {
                self.time_at_cap += dt;
            }
        }
        if motion != Motion::Still {
            if motion == Motion::Turning && self.last_moving == Some(Motion::Driving) {
                self.stop_turns += 1;
            }
            self.last_moving = Some(motion);
        }
    }

    /// one navigator tick: published command (None = nothing published or no goal), MPPI rotate mode,
    /// predicted trajectory and a key that changes whenever the goal or the path changes
    pub fn on_nav(&mut self, cmd: Option<(f64, f64)>, rotating: bool, dt: f64, traj: Vec<(f64, f64)>, path_key: u64) {
        if rotating {
            self.time_rotate_mode += dt;
            if !self.rotating {
                self.rotate_entries += 1;
            }
        }
        self.rotating = rotating;
        if let (Some(a), Some(b)) = (self.prev_cmd, cmd) {
            self.dv.push((b.0 - a.0).abs());
            self.dw.push((b.1 - a.1).abs());
        }
        self.prev_cmd = cmd;
        if let Some((prev, key)) = self.prev_traj.as_ref()
            && *key == path_key
            && prev.len() == traj.len()
            && traj.len() >= 2
        {
            let n = traj.len();
            let d = |a: (f64, f64), b: (f64, f64)| (a.0 - b.0).hypot(a.1 - b.1);
            self.end_jump.push(d(traj[n - 1], prev[n - 1]));
            self.end_jitter.push(d(traj[n - 2], prev[n - 1]));
        }
        self.prev_traj = (!traj.is_empty()).then_some((traj, path_key));
    }
}

/// what the metrics report
#[derive(Debug, Clone, Default, PartialEq)]
pub struct DriveSummary {
    /// distance / time with |v| >= STILL_V (m/s)
    pub moving_speed: f64,
    /// share of moving time with |v| >= AT_CAP * cap
    pub at_cap_share: f64,
    pub stop_turns: u64,
    pub rotate_entries: u64,
    pub time_rotate_mode: f64,
    /// p95 of |dw| (rad/s) and |dv| (m/s) per navigator tick
    pub cmd_dw_p95: f64,
    pub cmd_dv_p95: f64,
    /// p95 of the trajectory endpoint movement per solve, raw and time-aligned (m)
    pub end_jump_p95: f64,
    pub end_jitter_p95: f64,
}

impl DriveTracker {
    pub fn summary(&self) -> DriveSummary {
        DriveSummary {
            moving_speed: self.moving_distance / self.moving_time.max(1e-9),
            at_cap_share: self.time_at_cap / self.moving_time.max(1e-9),
            stop_turns: self.stop_turns,
            rotate_entries: self.rotate_entries,
            time_rotate_mode: self.time_rotate_mode,
            cmd_dw_p95: quantile(&self.dw, 0.95),
            cmd_dv_p95: quantile(&self.dv, 0.95),
            end_jump_p95: quantile(&self.end_jump, 0.95),
            end_jitter_p95: quantile(&self.end_jitter, 0.95),
        }
    }
}

/// changes whenever the path or the goal changes
pub fn path_key(path: &[(f64, f64)], goal: Option<(f64, f64)>) -> u64 {
    use std::hash::{Hash, Hasher};
    let mut h = std::collections::hash_map::DefaultHasher::new();
    for &(x, y) in path.iter().chain(goal.as_slice()) {
        (x.to_bits(), y.to_bits()).hash(&mut h);
    }
    h.finish()
}

/// `q` quantile (0..1, nearest rank) of `v`; 0 for no samples
pub fn quantile(v: &[f64], q: f64) -> f64 {
    if v.is_empty() {
        return 0.0;
    }
    let mut s = v.to_vec();
    s.sort_by(f64::total_cmp);
    s[((q * s.len() as f64).ceil() as usize).clamp(1, s.len()) - 1]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn quantiles() {
        assert_eq!(quantile(&[], 0.95), 0.0);
        let v: Vec<f64> = (1..=100).map(f64::from).collect();
        assert_eq!(quantile(&v, 0.95), 95.0);
        assert_eq!(quantile(&v, 1.0), 100.0);
    }

    #[test]
    fn stop_turns_and_cap() {
        let mut d = DriveTracker::default();
        let seq = [Motion::Driving, Motion::Still, Motion::Turning, Motion::Turning, Motion::Driving, Motion::Turning, Motion::Still, Motion::Turning];
        for m in seq {
            d.on_physics(0.1, if m == Motion::Driving { 0.25 } else { 0.0 }, m, 0.025, 0.26);
        }
        assert_eq!(d.stop_turns, 2);
        assert!((d.moving_time - 0.2).abs() < 1e-12 && (d.time_at_cap - 0.2).abs() < 1e-12);
    }

    #[test]
    fn trajectory_jitter_only_on_the_same_path() {
        let mut d = DriveTracker::default();
        let tr = |x0: f64| (0..4).map(|i| (x0 + i as f64 * 0.1, 0.0)).collect::<Vec<_>>();
        d.on_nav(Some((0.2, 0.0)), false, 0.05, tr(0.0), 1);
        d.on_nav(Some((0.2, 0.1)), false, 0.05, tr(0.1), 1);
        d.on_nav(Some((0.2, 0.1)), true, 0.05, tr(0.5), 2);
        assert_eq!(d.end_jump.len(), 1);
        assert!((d.end_jump[0] - 0.1).abs() < 1e-12 && d.end_jitter[0].abs() < 1e-12);
        assert_eq!(d.dw.len(), 2);
        assert_eq!(d.rotate_entries, 1);
    }
}
