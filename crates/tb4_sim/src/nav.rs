//! The team7 navigator (host_ws/.../bin/navigator.rs, mode mppi) without ROS: same 20 Hz tick, obstacle
//! field from map + scan points every obstacle_period, CSpace rebuilt on map change, Lazy Theta* with
//! smoothing and the route hysteresis, MPPI, goal timeout. Keep this in step with the node.

use tb4_control::{Mppi, MppiParams, RobotState};
use tb4_core::{DistanceField, Grid, Pose2};
use tb4_planning::{CSpace, CSpaceParams, Planner, PlannerParams, remaining_free};

#[derive(Debug, Clone)]
pub struct NavParams {
    pub replan_period: f64,
    pub obstacle_period: f64,
    pub goal_timeout: f64,
    pub cs: CSpaceParams,
    pub pp: PlannerParams,
    pub smooth_iterations: usize,
    pub use_scan_obstacles: bool,
    pub scan_obstacle_range: f64,
    pub path_switch_ratio: f64,
    /// plan on the SLAM map plus the latest scan points (rebuilds the cspace at every replan).
    /// Off: no measurable gain in the studio over 3 seeds (markers readable 6/9 both ways)
    pub plan_with_scan: bool,
    pub mppi: MppiParams,
}

impl NavParams {
    /// team7.yaml navigator section on top of the node defaults
    pub fn team7() -> Self {
        Self {
            replan_period: 2.0,
            obstacle_period: 0.2,
            goal_timeout: 90.0,
            cs: CSpaceParams {
                robot_radius: 0.18,
                safety_margin: 0.05,
                occupied_threshold: 65,
                unknown_is_obstacle: false,
                cost_scaling_dist: 0.5,
                cost_weight: 2.0,
            },
            pp: PlannerParams { allow_unknown: true, unknown_cost: 1.0, goal_tolerance: 0.3, start_tolerance: 0.4, ..PlannerParams::default() },
            smooth_iterations: 2,
            use_scan_obstacles: true,
            scan_obstacle_range: 2.5,
            path_switch_ratio: 0.8,
            plan_with_scan: false,
            mppi: MppiParams {
                samples: 1000,
                horizon: 40,
                threads: 4,
                vx_max: 0.26,
                wz_max: 0.7,
                goal_tolerance_xy: 0.15,
                goal_tolerance_yaw: 0.3,
                ..MppiParams::default()
            },
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum NavStatus {
    Succeeded,
    Canceled,
    Aborted,
}

/// why the navigator did what it did on the last tick (for stall causes)
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum NavState {
    Idle,
    /// goal active but no CSpace yet
    NoMap,
    /// planner failed, goal aborted
    NoPath,
    /// MPPI found no collision-free sample: zero command, replan
    AllColliding,
    /// MPPI rotating in place toward the path
    Rotating,
    Tracking,
}

impl NavState {
    pub fn name(self) -> &'static str {
        match self {
            Self::Idle => "idle",
            Self::NoMap => "no map",
            Self::NoPath => "no path",
            Self::AllColliding => "all samples colliding",
            Self::Rotating => "rotating",
            Self::Tracking => "tracking",
        }
    }
}

#[derive(Debug, Clone)]
pub struct Goal {
    pub x: f64,
    pub y: f64,
    /// None: xy_only goal, no final heading
    pub yaw: Option<f64>,
    pub started: f64,
    pub cancel: bool,
}

/// terminal result of a goal
#[derive(Debug, Clone, PartialEq)]
pub struct Finished {
    pub goal: (f64, f64),
    pub status: NavStatus,
    pub why: String,
}

#[derive(Debug, Clone, Default)]
pub struct NavOutput {
    /// a command was published (Some), or nothing this tick
    pub cmd: Option<(f64, f64)>,
    pub finished: Option<Finished>,
}

pub struct Navigator {
    pub p: NavParams,
    map: Option<Grid>,
    map_dirty: bool,
    cspace: Option<CSpace>,
    obstacles: Option<DistanceField>,
    last_obstacles: f64,
    scan_points: Vec<(f64, f64)>,
    goal: Option<Goal>,
    path: Vec<(f64, f64)>,
    last_plan: f64,
    planner: Planner,
    mppi: Mppi,
    stopped: bool,
    /// from /speed_limit: (percentage?, value), value 0 = no limit
    pub speed_limit: (bool, f64),
    pub state: NavState,
    pub plans: u64,
    pub plan_failures: u64,
    pub route_kept: u64,
}

const EPS: f64 = 1e-9;

impl Navigator {
    pub fn new(p: NavParams, now: f64) -> Self {
        Self {
            mppi: Mppi::new(p.mppi.clone()),
            p,
            map: None,
            map_dirty: false,
            cspace: None,
            obstacles: None,
            last_obstacles: now - 10.0,
            scan_points: Vec::new(),
            goal: None,
            path: Vec::new(),
            last_plan: now,
            planner: Planner::new(),
            stopped: true,
            speed_limit: (true, 0.0),
            state: NavState::Idle,
            plans: 0,
            plan_failures: 0,
            route_kept: 0,
        }
    }

    /// /map callback: rebuild the CSpace only when the grid changed
    pub fn set_map(&mut self, g: &Grid) {
        if !self.map.as_ref().is_some_and(|o| o.meta == g.meta && o.data == g.data) {
            self.map = Some(g.clone());
            self.map_dirty = true;
        }
    }

    /// /scan callback: scan endpoints already in the map frame
    pub fn set_scan(&mut self, map_points: impl Iterator<Item = (f64, f64)>, robot: (f64, f64)) {
        if self.p.use_scan_obstacles {
            let r = self.p.scan_obstacle_range;
            self.scan_points = map_points.filter(|p| (p.0 - robot.0).hypot(p.1 - robot.1) <= r).collect();
        }
    }

    /// new NavigateToPose goal; the old one is aborted (returned) and the MPPI warm start kept
    pub fn set_goal(&mut self, x: f64, y: f64, yaw: Option<f64>, now: f64) -> Option<Finished> {
        let old = self.goal.take().map(|g| Finished { goal: (g.x, g.y), status: NavStatus::Aborted, why: "replaced".into() });
        self.goal = Some(Goal { x, y, yaw, started: now, cancel: false });
        self.path.clear();
        old
    }

    pub fn cancel(&mut self) {
        if let Some(g) = self.goal.as_mut() {
            g.cancel = true;
        }
    }

    pub fn goal(&self) -> Option<&Goal> {
        self.goal.as_ref()
    }

    pub fn path(&self) -> &[(f64, f64)] {
        &self.path
    }

    pub fn trajectory(&self) -> Vec<(f64, f64)> {
        self.mppi.optimal_trajectory().iter().map(|p| (p.x, p.y)).collect()
    }

    pub fn obstacles(&self) -> Option<&DistanceField> {
        self.obstacles.as_ref()
    }

    fn finish(&mut self, ok: bool, why: &str) -> Option<Finished> {
        let g = self.goal.take()?;
        self.path.clear();
        let status = if g.cancel {
            NavStatus::Canceled
        } else if ok {
            NavStatus::Succeeded
        } else {
            NavStatus::Aborted
        };
        Some(Finished { goal: (g.x, g.y), status, why: why.into() })
    }

    fn stop(&mut self, out: &mut NavOutput) {
        if !self.stopped {
            out.cmd = Some((0.0, 0.0));
            self.stopped = true;
            self.mppi.reset();
        }
    }

    /// 20 Hz timer; `robot` is the map -> base_link pose, (v, w) the odometry twist
    pub fn tick(&mut self, now: f64, robot: Option<Pose2>, v: f64, w: f64) -> NavOutput {
        let mut out = NavOutput::default();
        if self.map_dirty
            && let Some(m) = self.map.as_ref()
        {
            self.cspace = Some(CSpace::new(m, &self.p.cs));
            self.map_dirty = false;
        }
        if now - self.last_obstacles >= self.p.obstacle_period - EPS {
            self.last_obstacles = now;
            if let Some(m) = self.map.as_ref() {
                let mut g = m.clone();
                for &(x, y) in &self.scan_points {
                    if let Some(c) = g.world_to_cell(x, y) {
                        g.set(c, 100);
                    }
                }
                self.obstacles = Some(DistanceField::from_grid(&g, self.p.cs.occupied_threshold, false));
            }
        }

        let Some(g) = self.goal.as_ref() else {
            self.state = NavState::Idle;
            self.stop(&mut out);
            return out;
        };
        let (gx, gy, gyaw, cancel, started) = (g.x, g.y, g.yaw, g.cancel, g.started);
        if cancel {
            out.finished = self.finish(false, "canceled");
            self.stop(&mut out);
            return out;
        }
        if now - started > self.p.goal_timeout {
            out.finished = self.finish(false, "aborted: timeout");
            self.stop(&mut out);
            return out;
        }
        let Some(robot) = robot else {
            self.stop(&mut out);
            return out;
        };

        let need_plan = self.path.is_empty() || now - self.last_plan >= self.p.replan_period - EPS;
        // plan on the map plus the latest scan, like MPPI: thin chair legs barely reach the SLAM map
        if need_plan
            && self.p.plan_with_scan
            && let Some(m) = self.map.as_ref()
        {
            let mut g = m.clone();
            for &(x, y) in &self.scan_points {
                if let Some(c) = g.world_to_cell(x, y) {
                    g.set(c, 100);
                }
            }
            self.cspace = Some(CSpace::new(&g, &self.p.cs));
        }
        if need_plan {
            let Some(cs) = self.cspace.take() else {
                self.state = NavState::NoMap;
                self.stop(&mut out);
                return out;
            };
            let res = self.planner.lazy_theta_star(&cs, (robot.x, robot.y), (gx, gy), &self.p.pp);
            self.last_plan = now;
            self.plans += 1;
            match res {
                Ok(p) => {
                    let pts = if self.p.smooth_iterations > 0 {
                        tb4_planning::smooth(&cs, &p.points, self.p.smooth_iterations, 0.3)
                    } else {
                        p.points
                    };
                    // keep the route it is on while that is still free, unless the new one is clearly shorter
                    let pts = match remaining_free(&cs, &self.path, (robot.x, robot.y)) {
                        Some(rest) if tb4_planning::path_length(&pts) >= self.p.path_switch_ratio * tb4_planning::path_length(&rest) => {
                            self.route_kept += 1;
                            self.path.clone()
                        }
                        _ => pts,
                    };
                    let changed = pts != self.path;
                    self.path = pts;
                    if changed {
                        self.mppi.set_path(&self.path, gyaw);
                    }
                }
                Err(e) => {
                    self.cspace = Some(cs);
                    self.plan_failures += 1;
                    self.state = NavState::NoPath;
                    out.finished = self.finish(false, &format!("aborted: {e}"));
                    self.stop(&mut out);
                    return out;
                }
            }
            self.cspace = Some(cs);
        }

        let st = RobotState { pose: robot, v, w };
        let cmd = self.mppi.compute(&st, self.obstacles.as_ref());
        if cmd.goal_reached {
            out.finished = self.finish(true, "reached");
            self.state = NavState::Idle;
            self.stop(&mut out);
            return out;
        }
        self.state = if !cmd.collision_free {
            NavState::AllColliding
        } else if self.mppi.is_rotating() {
            NavState::Rotating
        } else {
            NavState::Tracking
        };
        if !cmd.collision_free {
            self.path.clear(); // force replan next tick
        }
        let (v, w) = apply_speed_limit(self.speed_limit, self.mppi.params().vx_max, cmd.v, cmd.w);
        out.cmd = Some((v, w));
        self.stopped = false;
        out
    }
}

/// same meaning as nav2 controller: percentage of max speed, or absolute m/s; angular scaled alike
pub fn apply_speed_limit((pct, lim): (bool, f64), vx_max: f64, v: f64, w: f64) -> (f64, f64) {
    if lim <= 0.0 || vx_max <= 0.0 {
        return (v, w);
    }
    let k = if pct { (lim / 100.0).min(1.0) } else { (lim / vx_max).min(1.0) };
    (v.clamp(-vx_max * k, vx_max * k), w * k)
}
