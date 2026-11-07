use std::f64::consts::{FRAC_PI_2, PI};

use tb4_control::{Command, Control, Mppi, MppiParams, RobotState, simulate, wrap_angle};
use tb4_core::{Cell, DistanceField, Grid, GridMeta, Pose2};

fn params(threads: usize) -> MppiParams {
    MppiParams { samples: 200, threads, ..MppiParams::default() }
}

struct Run {
    states: Vec<RobotState>,
    cmds: Vec<Command>,
    reached: bool,
}

fn run(m: &mut Mppi, start: RobotState, df: Option<&DistanceField>, max_steps: usize) -> Run {
    let dt = m.params().dt;
    let mut s = start;
    let mut r = Run { states: vec![s], cmds: Vec::new(), reached: false };
    for _ in 0..max_steps {
        let c = m.compute(&s, df);
        r.cmds.push(c);
        if c.goal_reached {
            r.reached = true;
            break;
        }
        s = simulate(&s, c.control(), dt);
        r.states.push(s);
    }
    r
}

fn check_limits(p: &MppiParams, start: &RobotState, cmds: &[Command]) {
    let mut prev = start.control();
    for c in cmds.iter().filter(|c| !c.goal_reached && c.collision_free) {
        assert!(c.v >= 0.0 && c.v <= p.vx_max + 1e-12, "v {}", c.v);
        assert!(c.w.abs() <= p.wz_max + 1e-12, "w {}", c.w);
        assert!((c.v - prev.v).abs() <= p.ax_max * p.dt + 1e-9, "dv {} -> {}", prev.v, c.v);
        assert!((c.w - prev.w).abs() <= p.aw_max * p.dt + 1e-9, "dw {} -> {}", prev.w, c.w);
        prev = c.control();
    }
}

fn seg_dist(p: (f64, f64), a: (f64, f64), b: (f64, f64)) -> f64 {
    let (dx, dy) = (b.0 - a.0, b.1 - a.1);
    let t = (((p.0 - a.0) * dx + (p.1 - a.1) * dy) / (dx * dx + dy * dy)).clamp(0.0, 1.0);
    (p.0 - a.0 - t * dx).hypot(p.1 - a.1 - t * dy)
}

fn path_dist(p: (f64, f64), path: &[(f64, f64)]) -> f64 {
    path.windows(2).map(|w| seg_dist(p, w[0], w[1])).fold(f64::INFINITY, f64::min)
}

fn state(x: f64, y: f64, yaw: f64) -> RobotState {
    RobotState { pose: Pose2::new(x, y, yaw), v: 0.0, w: 0.0 }
}

#[test]
fn straight_path_reaches_goal() {
    let p = params(2);
    let mut m = Mppi::new(p.clone());
    let path = [(0.0, 0.0), (3.0, 0.0)];
    m.set_path(&path, None);
    let start = state(0.0, 0.0, 0.0);
    let r = run(&mut m, start, None, 400);
    assert!(r.reached);
    let end = r.states.last().unwrap().pose;
    assert!(end.distance_to(3.0, 0.0) < p.goal_tolerance_xy);
    // ~10 s at full speed plus accel/decel
    assert!(r.cmds.len() < 260, "took {} steps", r.cmds.len());
    let max_dev = r.states.iter().map(|s| s.pose.y.abs()).fold(0.0, f64::max);
    assert!(max_dev < 0.05, "deviation {max_dev}");
    check_limits(&p, &start, &r.cmds);
    assert!(m.timing().calls as usize == r.cmds.len() && m.timing().max_ms >= m.timing().mean_ms);
}

#[test]
fn l_shaped_path_with_goal_heading() {
    let p = params(2);
    let mut m = Mppi::new(p.clone());
    let path = [(0.0, 0.0), (2.0, 0.0), (2.0, 2.0)];
    m.set_path(&path, Some(FRAC_PI_2));
    let start = state(0.0, 0.0, 0.0);
    let r = run(&mut m, start, None, 600);
    assert!(r.reached);
    let end = r.states.last().unwrap().pose;
    assert!(end.distance_to(2.0, 2.0) < p.goal_tolerance_xy);
    assert!(wrap_angle(end.yaw - FRAC_PI_2).abs() < p.goal_tolerance_yaw);
    let max_dev = r.states.iter().map(|s| path_dist((s.pose.x, s.pose.y), &path)).fold(0.0, f64::max);
    assert!(max_dev < 0.25, "deviation {max_dev}");
    check_limits(&p, &start, &r.cmds);
}

#[test]
fn facing_backwards_turns_in_place() {
    let p = params(2);
    let mut m = Mppi::new(p.clone());
    m.set_path(&[(0.0, 0.0), (2.0, 0.0)], None);
    let start = state(0.0, 0.0, PI);
    let r = run(&mut m, start, None, 400);
    assert!(r.reached);
    assert!(r.cmds.iter().all(|c| c.v >= 0.0));
    let turn_end = r.states.iter().position(|s| s.pose.yaw.abs() < 1.0).expect("never turned");
    assert!(turn_end > 0);
    for s in &r.states[..turn_end] {
        assert!(s.pose.distance_to(0.0, 0.0) < 0.03, "moved while turning: {:?}", s.pose);
    }
    assert!(r.cmds[..turn_end].iter().all(|c| c.v < 1e-9));
    assert!(r.states.iter().all(|s| s.pose.x > -0.05));
    check_limits(&p, &start, &r.cmds);
}

fn blocked_world() -> DistanceField {
    let meta = GridMeta { width: 120, height: 60, resolution: 0.05, origin_x: -1.0, origin_y: -1.5 };
    let mut g = Grid::filled(meta, 0);
    // box x in [1.4, 1.8], y in [-0.6, 0.1] crossing the path at y = 0
    for cy in 18..32 {
        for cx in 48..56 {
            g.set(Cell { x: cx, y: cy }, 100);
        }
    }
    DistanceField::from_grid(&g, 65, false)
}

#[test]
fn avoids_obstacle_on_path() {
    let p = params(2);
    let df = blocked_world();
    assert!(df.at(1.5, 0.0) == 0.0);
    let mut m = Mppi::new(p.clone());
    let path = [(0.0, 0.0), (4.0, 0.0)];
    m.set_path(&path, None);
    let start = state(0.0, 0.0, 0.0);
    let r = run(&mut m, start, Some(&df), 600);
    assert!(r.reached);
    assert!(r.cmds.iter().all(|c| c.collision_free));
    let min_clear = r.states.iter().map(|s| df.at(s.pose.x, s.pose.y) as f64).fold(f64::INFINITY, f64::min);
    assert!(min_clear > p.robot_radius, "min clearance {min_clear}");
    check_limits(&p, &start, &r.cmds);
}

#[test]
fn deterministic_across_threads() {
    let df = blocked_world();
    let path = [(0.0, 0.0), (2.0, 0.0), (2.0, 1.0)];
    let cmds = |threads| {
        let mut m = Mppi::new(MppiParams { iterations: 2, ..params(threads) });
        m.set_path(&path, Some(FRAC_PI_2));
        run(&mut m, state(0.0, 0.0, 0.3), Some(&df), 80).cmds
    };
    let a = cmds(1);
    assert_eq!(a, cmds(4));
    assert_eq!(a, cmds(3));
    let mut m = Mppi::new(MppiParams { seed: 7, ..params(1) });
    m.set_path(&path, None);
    assert_ne!(a, run(&mut m, state(0.0, 0.0, 0.3), Some(&df), 80).cmds);
}

#[test]
fn empty_path_and_reset() {
    let mut m = Mppi::new(params(1));
    let s = RobotState { pose: Pose2::new(1.0, 1.0, 0.0), v: 0.2, w: 0.1 };
    let c = m.compute(&s, None);
    assert_eq!(c.control(), Control::ZERO);
    assert!(!c.goal_reached && c.collision_free);
    m.set_path(&[], None);
    assert_eq!(m.compute(&s, None).control(), Control::ZERO);
    m.set_path(&[(1.0, 1.0)], None);
    assert!(m.compute(&s, None).goal_reached);
    m.reset();
    assert_eq!(m.timing().calls, 0);
    assert!(m.control_sequence().iter().all(|u| *u == Control::ZERO));
}

#[test]
fn all_samples_colliding_stops() {
    let meta = GridMeta { width: 40, height: 40, resolution: 0.05, origin_x: -1.0, origin_y: -1.0 };
    let mut g = Grid::filled(meta, 0);
    // ring of obstacles 0.15 m around the robot
    for cy in 0..40u32 {
        for cx in 0..40u32 {
            let (x, y) = (cx as f64 * 0.05 - 0.975, cy as f64 * 0.05 - 0.975);
            if (x.hypot(y) - 0.15).abs() < 0.04 {
                g.set(Cell { x: cx, y: cy }, 100);
            }
        }
    }
    let df = DistanceField::from_grid(&g, 65, false);
    let mut m = Mppi::new(params(2));
    m.set_path(&[(0.0, 0.0), (0.8, 0.0)], None);
    let c = m.compute(&RobotState { pose: Pose2::new(0.0, 0.0, 0.0), v: 0.2, w: 0.0 }, Some(&df));
    assert!(!c.collision_free);
    assert_eq!(c.control(), Control::ZERO);
}

#[test]
fn rollout_helper_matches_simulate() {
    let p = MppiParams::default();
    let s = state(0.5, -0.2, 0.4);
    let u = vec![Control::new(0.3, 0.5); 20];
    let traj = tb4_control::rollout(&s, &u, &p);
    let mut q = s;
    for (t, &c) in traj.iter().zip(&u) {
        let lim = tb4_control::limit_control(q.control(), c, &p, p.vx_min, p.vx_max);
        q = simulate(&q, lim, p.dt);
        assert_eq!(*t, q);
    }
}
