use std::time::Instant;

use tb4_core::{DistanceField, Pose2};

use crate::model::{Control, RobotState, limit_control, step, wrap_angle};
use crate::params::MppiParams;
use crate::path::RefPath;
use crate::rng::Rng;

#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Command {
    pub v: f64,
    pub w: f64,
    pub goal_reached: bool,
    pub best_cost: f64,
    pub collision_free: bool,
}

impl Command {
    fn stop(goal_reached: bool, collision_free: bool, best_cost: f64) -> Self {
        Self { v: 0.0, w: 0.0, goal_reached, best_cost, collision_free }
    }

    pub fn control(&self) -> Control {
        Control::new(self.v, self.w)
    }
}

#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Timing {
    pub calls: u64,
    pub last_ms: f64,
    pub mean_ms: f64,
    pub max_ms: f64,
}

impl Timing {
    fn record(&mut self, ms: f64) {
        self.calls += 1;
        self.last_ms = ms;
        self.mean_ms += (ms - self.mean_ms) / self.calls as f64;
        self.max_ms = self.max_ms.max(ms);
    }
}

#[derive(Debug, Clone)]
pub struct Mppi {
    params: MppiParams,
    path: RefPath,
    goal_yaw: Option<f64>,
    path_idx: Option<usize>,
    xy_reached: bool,
    rotating: bool,
    step: u64,
    u: Vec<Control>,
    ctrl: Vec<Control>,
    costs: Vec<f64>,
    collided: Vec<bool>,
    traj: Vec<Pose2>,
    timing: Timing,
}

#[derive(Clone, Copy)]
struct Ctx<'a> {
    p: &'a MppiParams,
    path: &'a RefPath,
    u: &'a [Control],
    start: RobotState,
    v_lo: f64,
    v_hi: f64,
    step: u64,
    iter: u64,
    lo: usize,
    hi: usize,
    follow: (f64, f64),
    path_active: bool,
    s_follow: f64,
    goal: (f64, f64),
    goal_yaw: f64,
    goal_active: bool,
    heading_active: bool,
    angle_active: bool,
    df: Option<&'a DistanceField>,
}

impl Ctx<'_> {
    fn sample(&self, k: usize, out: &mut [Control]) -> (f64, bool) {
        let p = self.p;
        let mut rng = Rng::stream(p.seed, self.step, self.iter, k as u64);
        let inv_dv = 1.0 / (p.ax_max * p.dt);
        let inv_dw = 1.0 / (p.aw_max * p.dt);
        let r_infl = p.robot_radius + p.inflation;
        let mut prev = self.start.control();
        let mut pose = self.start.pose;
        let (mut pd, mut ang, mut gd, mut gh, mut obs, mut smooth, mut twirl, mut back) =
            (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        let mut collided = false;
        for t in 0..out.len() {
            let mut want = self.u[t];
            // sample 0 evaluates the warm-start sequence unperturbed
            if k != 0 {
                want.v += p.noise_v * rng.normal();
                want.w += p.noise_w * rng.normal();
            }
            let c = limit_control(prev, want, p, self.v_lo, self.v_hi);
            // store the unclamped perturbation so clamping at the bounds does not bias the mean
            out[t] = want;
            pose = step(&pose, c.v, c.w, p.dt);
            let (dv, dw) = ((c.v - prev.v) * inv_dv, (c.w - prev.w) * inv_dw);
            smooth += dv * dv + dw * dw;
            twirl += c.w.abs();
            back += (-c.v).max(0.0);
            prev = c;

            if let Some(df) = self.df {
                let clearance = df.at(pose.x, pose.y) as f64;
                if clearance < p.robot_radius {
                    collided = true;
                    out[t + 1..].copy_from_slice(&self.u[t + 1..]);
                    break;
                }
                if clearance < r_infl {
                    let e = (r_infl - clearance) / p.inflation;
                    obs += e * e;
                }
            }
            let mut aim = self.follow;
            if self.path_active {
                let (j, d2) = self.path.nearest(pose.x, pose.y, self.lo, self.hi);
                pd += d2.sqrt();
                aim = self.path.pts[self.path.ahead[j]];
            }
            if self.angle_active {
                let (dx, dy) = (aim.0 - pose.x, aim.1 - pose.y);
                if dx * dx + dy * dy > 0.0025 {
                    ang += wrap_angle(dy.atan2(dx) - pose.yaw).abs();
                }
            }
            if self.goal_active {
                gd += (pose.x - self.goal.0).hypot(pose.y - self.goal.1);
            }
            if self.heading_active {
                gh += wrap_angle(pose.yaw - self.goal_yaw).abs();
            }
        }
        if collided {
            return (p.collision_cost, true);
        }
        let follow = if self.path_active {
            (pose.x - self.follow.0).hypot(pose.y - self.follow.1)
        } else {
            let (j, _) = self.path.nearest(pose.x, pose.y, self.lo, self.hi);
            (self.s_follow - self.path.s[j]).max(0.0)
        };
        let stage = p.path_distance_weight * pd
            + p.path_angle_weight * ang
            + p.goal_weight * gd
            + p.goal_heading_weight * gh
            + p.obstacle_weight * obs
            + p.smoothness_weight * smooth
            + p.twirl_weight * twirl
            + p.prefer_forward_weight * back;
        (stage / out.len() as f64 + p.path_follow_weight * follow, false)
    }

    fn run(&self, first: usize, ctrl: &mut [Control], costs: &mut [f64], collided: &mut [bool]) {
        let h = self.u.len();
        for (i, (out, (c, col))) in ctrl.chunks_mut(h).zip(costs.iter_mut().zip(collided.iter_mut())).enumerate() {
            (*c, *col) = self.sample(first + i, out);
        }
    }
}

impl Mppi {
    pub fn new(params: MppiParams) -> Self {
        let mut m = Self {
            params,
            path: RefPath::default(),
            goal_yaw: None,
            path_idx: None,
            xy_reached: false,
            rotating: false,
            step: 0,
            u: Vec::new(),
            ctrl: Vec::new(),
            costs: Vec::new(),
            collided: Vec::new(),
            traj: Vec::new(),
            timing: Timing::default(),
        };
        m.reset();
        m
    }

    pub fn params(&self) -> &MppiParams {
        &self.params
    }

    /// keeps the path; clears warm start and state
    pub fn set_params(&mut self, params: MppiParams) {
        self.params = params;
        let pts = std::mem::take(&mut self.path.pts);
        self.path = RefPath::new(&pts, self.params.path_resolution, self.params.heading_lookahead);
        self.reset();
    }

    pub fn set_path(&mut self, path: &[(f64, f64)], goal_yaw: Option<f64>) {
        self.path = RefPath::new(path, self.params.path_resolution, self.params.heading_lookahead);
        self.goal_yaw = goal_yaw;
        self.path_idx = None;
        self.xy_reached = false;
        self.rotating = false;
    }

    pub fn reset(&mut self) {
        let p = &self.params;
        let h = p.horizon.max(1);
        let k = p.samples.max(1);
        self.u = vec![Control::ZERO; h];
        self.ctrl = vec![Control::ZERO; h * k];
        self.costs = vec![0.0; k];
        self.collided = vec![false; k];
        self.traj.clear();
        self.path_idx = None;
        self.xy_reached = false;
        self.rotating = false;
        self.step = 0;
        self.timing = Timing::default();
    }

    pub fn timing(&self) -> &Timing {
        &self.timing
    }

    pub fn control_sequence(&self) -> &[Control] {
        &self.u
    }

    /// predicted poses for the last returned control sequence
    pub fn optimal_trajectory(&self) -> &[Pose2] {
        &self.traj
    }

    pub fn is_rotating(&self) -> bool {
        self.rotating
    }

    pub fn path_points(&self) -> &[(f64, f64)] {
        &self.path.pts
    }

    pub fn compute(&mut self, state: &RobotState, obstacles: Option<&DistanceField>) -> Command {
        let t0 = Instant::now();
        let cmd = self.compute_inner(state, obstacles);
        self.step += 1;
        self.timing.record(t0.elapsed().as_secs_f64() * 1e3);
        cmd
    }

    fn track(&mut self, x: f64, y: f64) -> usize {
        let n = self.path.pts.len();
        if let Some(i) = self.path_idx {
            let hi = (self.path.index_at(self.path.s[i] + 1.0) + 1).min(n);
            let (j, d2) = self.path.nearest(x, y, i, hi);
            if d2 < 0.25 {
                self.path_idx = Some(j);
                return j;
            }
        }
        let (j, _) = self.path.nearest(x, y, 0, n);
        self.path_idx = Some(j);
        j
    }

    fn halt(&mut self, cmd: Command) -> Command {
        self.u.fill(Control::ZERO);
        self.traj.clear();
        cmd
    }

    fn compute_inner(&mut self, state: &RobotState, df: Option<&DistanceField>) -> Command {
        if self.path.is_empty() {
            return self.halt(Command::stop(false, true, 0.0));
        }
        let pose = state.pose;
        let goal = *self.path.pts.last().unwrap();
        let dgoal = pose.distance_to(goal.0, goal.1);
        if dgoal < self.params.goal_tolerance_xy {
            self.xy_reached = true;
        }
        if self.xy_reached {
            let yaw_ok =
                self.goal_yaw.is_none_or(|gy| wrap_angle(gy - pose.yaw).abs() < self.params.goal_tolerance_yaw);
            if yaw_ok {
                return self.halt(Command::stop(true, true, 0.0));
            }
        }

        let idx = self.track(pose.x, pose.y);
        let p = &self.params;
        let path = &self.path;
        let s_r = path.s[idx];
        let h = self.u.len();
        let k = self.costs.len();
        let reach = p.vx_max.max(-p.vx_min) * h as f64 * p.dt + 0.3;
        let lo = path.index_at(s_r - 0.2);
        let hi = (path.index_at(s_r + reach) + 1).min(path.pts.len());
        let clear = |i: usize, margin: f64| df.is_none_or(|df| df.at(path.pts[i].0, path.pts[i].1) as f64 >= margin);
        // blocked local path: drop path-shape critics, score arc-length progress to a free point past it
        let blocked = (idx..hi).any(|i| !clear(i, p.robot_radius));
        let mut fi = path.index_at(s_r + p.follow_lookahead);
        if blocked {
            while fi + 1 < path.pts.len() && !clear(fi, p.robot_radius + 0.5 * p.inflation) {
                fi += 1;
            }
        }
        let follow = path.pts[fi];

        let align = self.xy_reached;
        if align {
            self.rotating = false;
        } else {
            let a = if blocked { follow } else { path.pts[path.ahead[idx]] };
            let (dx, dy) = (a.0 - pose.x, a.1 - pose.y);
            let err = if dx * dx + dy * dy > 0.0025 { wrap_angle(dy.atan2(dx) - pose.yaw).abs() } else { 0.0 };
            if err > p.rotate_in_place_angle && !blocked {
                self.rotating = true;
            } else if err < p.rotate_exit_angle {
                self.rotating = false;
            }
        }
        let (v_lo, v_hi) = if align || self.rotating { (0.0, 0.0) } else { (p.vx_min, p.vx_max) };
        let heading_active = self.goal_yaw.is_some() && (align || dgoal < p.goal_heading_activate);
        let start = *state;

        let base = Ctx {
            p,
            path,
            u: &[],
            start,
            v_lo,
            v_hi,
            step: self.step,
            iter: 0,
            lo,
            hi,
            follow,
            goal,
            goal_yaw: self.goal_yaw.unwrap_or(0.0),
            path_active: !blocked,
            s_follow: path.s[fi],
            goal_active: path.length() - s_r < p.goal_activate_dist || dgoal < p.goal_activate_dist,
            heading_active,
            angle_active: !heading_active && !align && (!blocked || self.rotating),
            df,
        };

        let threads = match p.threads {
            0 => std::thread::available_parallelism().map_or(1, |n| n.get()),
            n => n,
        }
        .min(k);
        let per = k.div_ceil(threads);
        let mut u_new = vec![Control::ZERO; h];
        let mut best_cost = 0.0;
        for it in 0..p.iterations.max(1) {
            let ctx = Ctx { u: &self.u, iter: it as u64, ..base };
            if threads <= 1 {
                ctx.run(0, &mut self.ctrl, &mut self.costs, &mut self.collided);
            } else {
                let ctx = &ctx;
                std::thread::scope(|s| {
                    let mut chunks = self
                        .ctrl
                        .chunks_mut(per * h)
                        .zip(self.costs.chunks_mut(per).zip(self.collided.chunks_mut(per)))
                        .enumerate();
                    let (_, (c0, (k0, l0))) = chunks.next().unwrap();
                    for (i, (c, (kc, lc))) in chunks {
                        s.spawn(move || ctx.run(i * per, c, kc, lc));
                    }
                    ctx.run(0, c0, k0, l0);
                });
            }

            let (kbest, cmin) = self
                .costs
                .iter()
                .copied()
                .enumerate()
                .fold((0, f64::INFINITY), |b, (i, c)| if c < b.1 { (i, c) } else { b });
            best_cost = cmin;
            if self.collided.iter().all(|&c| c) {
                self.u.fill(Control::ZERO);
                self.traj.clear();
                return Command::stop(false, false, cmin);
            }
            u_new.fill(Control::ZERO);
            let mut wsum = 0.0;
            for (c, seq) in self.costs.iter().zip(self.ctrl.chunks(h)) {
                let w = (-(c - cmin) / p.temperature).exp();
                if w < 1e-12 {
                    continue;
                }
                wsum += w;
                for (u, s) in u_new.iter_mut().zip(seq) {
                    u.v += w * s.v;
                    u.w += w * s.w;
                }
            }
            for u in u_new.iter_mut() {
                u.v /= wsum;
                u.w /= wsum;
            }
            if p.smoothing {
                savgol(&mut u_new, start.control());
            }
            relimit(&mut u_new, start.control(), p, v_lo, v_hi);
            if df.is_some_and(|df| collides(&start.pose, &u_new, p, df)) {
                u_new.copy_from_slice(&self.ctrl[kbest * h..(kbest + 1) * h]);
                relimit(&mut u_new, start.control(), p, v_lo, v_hi);
            }
            self.u.copy_from_slice(&u_new);
        }

        self.traj.clear();
        let mut pp = start.pose;
        for u in &self.u {
            pp = step(&pp, u.v, u.w, p.dt);
            self.traj.push(pp);
        }
        let mut c = self.u[0];
        if c.v.abs() < p.v_deadband {
            c.v = 0.0;
        }
        if c.w.abs() < p.w_deadband {
            c.w = 0.0;
        }
        self.u.rotate_left(1);
        if h > 1 {
            self.u[h - 1] = self.u[h - 2];
        }
        Command { v: c.v, w: c.w, goal_reached: false, best_cost, collision_free: true }
    }
}

fn relimit(u: &mut [Control], mut prev: Control, p: &MppiParams, v_lo: f64, v_hi: f64) {
    for c in u.iter_mut() {
        *c = limit_control(prev, *c, p, v_lo, v_hi);
        prev = *c;
    }
}

fn collides(start: &Pose2, u: &[Control], p: &MppiParams, df: &DistanceField) -> bool {
    let mut pose = *start;
    u.iter().any(|c| {
        pose = step(&pose, c.v, c.w, p.dt);
        (df.at(pose.x, pose.y) as f64) < p.robot_radius
    })
}

/// 5-point quadratic Savitzky-Golay; front padded with the current control, back with the last
fn savgol(u: &mut [Control], first: Control) {
    const C: [f64; 5] = [-3.0 / 35.0, 12.0 / 35.0, 17.0 / 35.0, 12.0 / 35.0, -3.0 / 35.0];
    let n = u.len();
    if n < 3 {
        return;
    }
    let last = u[n - 1];
    let mut ext = Vec::with_capacity(n + 4);
    ext.extend([first, first]);
    ext.extend_from_slice(u);
    ext.extend([last, last]);
    for (i, out) in u.iter_mut().enumerate() {
        let (mut v, mut w) = (0.0, 0.0);
        for (c, e) in C.iter().zip(&ext[i..i + 5]) {
            v += c * e.v;
            w += c * e.w;
        }
        *out = Control::new(v, w);
    }
}
