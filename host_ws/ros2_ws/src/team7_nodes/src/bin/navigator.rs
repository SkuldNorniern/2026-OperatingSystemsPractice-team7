use std::cell::RefCell;
use std::rc::Rc;
use std::time::{Duration, Instant};

use futures::StreamExt;
use futures::executor::{LocalPool, LocalSpawner};
use futures::task::LocalSpawnExt;
use r2r::geometry_msgs::msg::{PoseStamped, TwistStamped};
use r2r::nav_msgs::msg::{OccupancyGrid, Odometry, Path as PathMsg};
use r2r::nav2_msgs::action::{FollowPath, NavigateToPose};
use r2r::sensor_msgs::msg::LaserScan;
use r2r::{ActionClient, ActionClientGoal, ActionServerGoal, GoalStatus, QosProfile, log_info, log_warn};
use tb4_control::{Mppi, MppiParams, RobotState};
use tb4_core::{DistanceField, Grid, Pose2, Stamp, TfBuffer};
use tb4_planning::{CSpace, CSpaceParams, Planner, PlannerParams};
use team7_nodes::{Params, grid_from_msg, pose_msg, spawn_tf_listener};

type Nav = NavigateToPose::Action;
type Follow = FollowPath::Action;

#[derive(Clone, Copy, PartialEq, Debug)]
enum Mode {
    Nav2Controller,
    Mppi,
}

struct Goal {
    handle: ActionServerGoal<Nav>,
    x: f64,
    y: f64,
    yaw: f64,
    started: Instant,
    cancel: bool,
}

struct Cfg {
    mode: Mode,
    map_frame: String,
    base_frame: String,
    replan_period: Duration,
    obstacle_period: Duration,
    goal_timeout: Duration,
    cs: CSpaceParams,
    pp: PlannerParams,
    smooth_iterations: usize,
    use_scan_obstacles: bool,
    scan_obstacle_range: f64,
    controller_id: String,
}

struct State {
    map: Option<Grid>,
    map_dirty: bool,
    cspace: Option<CSpace>,
    obstacles: Option<DistanceField>,
    last_obstacles: Instant,
    last_global: Instant,
    scan_points: Vec<(f64, f64)>,
    v: f64,
    w: f64,
    goal: Option<Goal>,
    path: Vec<(f64, f64)>,
    last_plan: Instant,
    planner: Planner,
    mppi: Mppi,
    follow: Option<ActionClientGoal<Follow>>,
    follow_gen: u64,
    follow_result: Option<(u64, GoalStatus)>,
    done: bool,
    stopped: bool,
    /// from /speed_limit: (percentage?, value), value 0 = no limit
    speed_limit: (bool, f64),
}

struct Io {
    cmd: r2r::Publisher<TwistStamped>,
    path: r2r::Publisher<PathMsg>,
    traj: r2r::Publisher<PathMsg>,
    local: r2r::Publisher<OccupancyGrid>,
    global: r2r::Publisher<OccupancyGrid>,
    follow: ActionClient<Follow>,
}

/// team7 global costmap: 100 within GLOBAL_LETHAL_M of an obstacle, falling to 0 over GLOBAL_INFLATE_M,
/// -1 where the map is unknown. Whole mapped area, same frame as /map.
const GLOBAL_LETHAL_M: f64 = 0.2;
const GLOBAL_INFLATE_M: f64 = 0.5;

fn global_cost_msg(frame: &str, map: &Grid, df: &DistanceField, now: &r2r::builtin_interfaces::msg::Time) -> OccupancyGrid {
    let m = &df.meta;
    let data: Vec<i8> = df
        .dist
        .iter()
        .enumerate()
        .map(|(k, &d)| {
            if map.data[k] < 0 {
                return -1;
            }
            let d = d as f64;
            // obstacle core and inflation band sit in the upper part of the 0..100 range so the
            // RViz costmap palette draws them clearly; free space stays 0
            if d <= GLOBAL_LETHAL_M {
                100
            } else if d <= GLOBAL_LETHAL_M + GLOBAL_INFLATE_M {
                (60.0 + 40.0 * (1.0 - (d - GLOBAL_LETHAL_M) / GLOBAL_INFLATE_M)).round() as i8
            } else {
                0
            }
        })
        .collect();
    let mut g = OccupancyGrid::default();
    g.header.frame_id = frame.to_owned();
    g.header.stamp = now.clone();
    g.info.resolution = m.resolution as f32;
    g.info.width = m.width;
    g.info.height = m.height;
    g.info.origin = pose_msg(m.origin_x, m.origin_y, 0.0);
    g.data = data;
    g
}

/// Local MPPI cost layer: a LOCAL_WINDOW_M square around the robot, from the obstacle
/// distance field MPPI scores against. 100 = touching an obstacle, falls to 0 at
/// LOCAL_COST_RANGE_M, -1 (unknown) outside the mapped area.
const LOCAL_WINDOW_M: f64 = 4.0;
const LOCAL_COST_RANGE_M: f64 = 1.0;

fn local_cost_msg(frame: &str, df: &DistanceField, cx: f64, cy: f64, now: &r2r::builtin_interfaces::msg::Time) -> OccupancyGrid {
    let res = df.meta.resolution;
    let n = (LOCAL_WINDOW_M / res).round() as u32;
    let ox = cx - LOCAL_WINDOW_M / 2.0;
    let oy = cy - LOCAL_WINDOW_M / 2.0;
    let m = &df.meta;
    let mut data = Vec::with_capacity((n * n) as usize);
    for j in 0..n {
        for i in 0..n {
            let x = ox + (i as f64 + 0.5) * res;
            let y = oy + (j as f64 + 0.5) * res;
            let inside = x >= m.origin_x && y >= m.origin_y
                && x < m.origin_x + m.width as f64 * res
                && y < m.origin_y + m.height as f64 * res;
            // circular window: cells beyond half the window (from the robot) stay unknown
            let (dx, dy) = (x - cx, y - cy);
            let in_circle = dx * dx + dy * dy <= (LOCAL_WINDOW_M / 2.0).powi(2);
            let v = if !inside || !in_circle {
                -1
            } else {
                let d = df.at(x, y) as f64;
                (100.0 * (1.0 - d / LOCAL_COST_RANGE_M)).clamp(0.0, 100.0).round() as i8
            };
            data.push(v);
        }
    }
    let mut g = OccupancyGrid::default();
    g.header.frame_id = frame.to_owned();
    g.header.stamp = now.clone();
    g.info.resolution = res as f32;
    g.info.width = n;
    g.info.height = n;
    g.info.origin = pose_msg(ox, oy, 0.0);
    g.data = data;
    g
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "navigator", "")?;
    let log = node.logger().to_owned();
    let p = Params(&node);

    let mode = match p.string("mode", "mppi").as_str() {
        "nav2" | "nav2_controller" => Mode::Nav2Controller,
        _ => Mode::Mppi,
    };
    let cfg = Rc::new(Cfg {
        mode,
        map_frame: p.string("map_frame", "map"),
        base_frame: p.string("base_frame", "base_link"),
        replan_period: Duration::from_secs_f64(p.f64("replan_period", 2.0)),
        obstacle_period: Duration::from_secs_f64(p.f64("obstacle_period", 0.2)),
        goal_timeout: Duration::from_secs_f64(p.f64("goal_timeout", 90.0)),
        cs: CSpaceParams {
            robot_radius: p.f64("robot_radius", 0.18),
            safety_margin: p.f64("safety_margin", 0.05),
            occupied_threshold: p.i64("occupied_threshold", 65) as i8,
            unknown_is_obstacle: p.bool("unknown_is_obstacle", false),
            cost_scaling_dist: p.f64("cost_scaling_dist", 0.5),
            cost_weight: p.f64("cost_weight", 2.0),
        },
        pp: PlannerParams {
            allow_unknown: p.bool("allow_unknown", true),
            unknown_cost: p.f64("unknown_cost", 1.0),
            goal_tolerance: p.f64("goal_tolerance", 0.3),
            start_tolerance: p.f64("start_tolerance", 0.4),
            ..PlannerParams::default()
        },
        smooth_iterations: p.i64("smooth_iterations", 2) as usize,
        use_scan_obstacles: p.bool("use_scan_obstacles", true),
        scan_obstacle_range: p.f64("scan_obstacle_range", 2.5),
        controller_id: p.string("controller_id", "FollowPath"),
    });
    let mp = MppiParams {
        samples: p.i64("mppi_samples", 1000) as usize,
        horizon: p.i64("mppi_horizon", 40) as usize,
        threads: p.i64("mppi_threads", 4) as usize,
        vx_max: p.f64("vx_max", 0.30),
        wz_max: p.f64("wz_max", 1.9),
        goal_tolerance_xy: p.f64("goal_tolerance_xy", 0.15),
        goal_tolerance_yaw: p.f64("goal_tolerance_yaw", 0.3),
        ..MppiParams::default()
    };
    let action_name = p.string("action_name", "/team7/navigate_to_pose");
    let follow_action = p.string("follow_action", "/follow_path");
    let map_topic = p.string("map_topic", "/map");
    let scan_topic = p.string("scan_topic", "/scan");
    let odom_topic = p.string("odom_topic", "/odom");
    let cmd_topic = p.string("cmd_vel_topic", "/cmd_vel");
    log_info!(&log, "mode {:?}, serving {action_name}", cfg.mode);

    let pool = LocalPool::new();
    let spawner = pool.spawner();
    team7_nodes::spawn_param_services(&mut node, &spawner)?;
    let tf = Rc::new(RefCell::new(TfBuffer::new(10.0, 0.2)));
    spawn_tf_listener(&mut node, &spawner, tf.clone(), "/tf", "/tf_static")?;

    let state = Rc::new(RefCell::new(State {
        map: None,
        map_dirty: false,
        cspace: None,
        obstacles: None,
        last_obstacles: Instant::now() - Duration::from_secs(10),
        last_global: Instant::now() - Duration::from_secs(10),
        scan_points: Vec::new(),
        v: 0.0,
        w: 0.0,
        goal: None,
        path: Vec::new(),
        last_plan: Instant::now(),
        planner: Planner::new(),
        mppi: Mppi::new(mp),
        follow: None,
        follow_gen: 0,
        follow_result: None,
        done: false,
        stopped: true,
        speed_limit: (true, 0.0),
    }));

    let st = state.clone();
    let maps = node.subscribe::<OccupancyGrid>(&map_topic, QosProfile::default().reliable().transient_local())?;
    spawner.spawn_local(maps.for_each(move |m| {
        let mut s = st.borrow_mut();
        // slam republishes same grid every period, rebuild cspace only on change
        if let Some(g) = grid_from_msg(&m)
            && !s.map.as_ref().is_some_and(|o| o.meta == g.meta && o.data == g.data)
        {
            s.map = Some(g);
            s.map_dirty = true;
        }
        futures::future::ready(())
    }))?;

    let st = state.clone();
    let odom = node.subscribe::<Odometry>(&odom_topic, QosProfile::default())?;
    spawner.spawn_local(odom.for_each(move |m| {
        let mut s = st.borrow_mut();
        s.v = m.twist.twist.linear.x;
        s.w = m.twist.twist.angular.z;
        futures::future::ready(())
    }))?;

    // latest scan endpoints in map frame, merged into the MPPI obstacle field
    let st = state.clone();
    let tf_scan = tf.clone();
    let c = cfg.clone();
    let scans = node.subscribe::<LaserScan>(&scan_topic, QosProfile::sensor_data())?;
    spawner.spawn_local(scans.for_each(move |m| {
        if c.use_scan_obstacles
            && let Ok(t) = tf_scan.borrow().lookup(&c.map_frame, &m.header.frame_id, team7_nodes::stamp(&m.header.stamp))
        {
            let mut pts = Vec::with_capacity(m.ranges.len());
            for (i, r) in m.ranges.iter().enumerate() {
                let r = *r as f64;
                if !r.is_finite() || r < m.range_min as f64 || r > c.scan_obstacle_range {
                    continue;
                }
                let a = m.angle_min as f64 + i as f64 * m.angle_increment as f64;
                let pt = t * tb4_core::nalgebra::Point3::new(r * a.cos(), r * a.sin(), 0.0);
                pts.push((pt.x, pt.y));
            }
            st.borrow_mut().scan_points = pts;
        }
        futures::future::ready(())
    }))?;

    let st = state.clone();
    let mission = node.subscribe::<r2r::std_msgs::msg::String>(
        "/team7/mission_state",
        QosProfile::default().reliable().transient_local(),
    )?;
    spawner.spawn_local(mission.for_each(move |m| {
        if m.data == "DONE" {
            st.borrow_mut().done = true;
        }
        futures::future::ready(())
    }))?;

    let st = state.clone();
    let limits = node.subscribe::<r2r::nav2_msgs::msg::SpeedLimit>("/speed_limit", QosProfile::default())?;
    spawner.spawn_local(limits.for_each(move |m| {
        st.borrow_mut().speed_limit = (m.percentage, m.speed_limit);
        futures::future::ready(())
    }))?;

    let requests = node.create_action_server::<Nav>(&action_name)?;
    let st = state.clone();
    let log2 = log.clone();
    let sp = spawner.clone();
    spawner.spawn_local(async move {
        let mut requests = requests;
        while let Some(req) = requests.next().await {
            if st.borrow().done {
                let _ = req.reject();
                continue;
            }
            let (handle, mut cancels) = match req.accept() {
                Ok(x) => x,
                Err(e) => {
                    log_warn!(&log2, "accept failed: {e}");
                    continue;
                }
            };
            let gp = &handle.goal.pose.pose;
            let q = &gp.orientation;
            let yaw = (2.0 * (q.w * q.z + q.x * q.y)).atan2(1.0 - 2.0 * (q.y * q.y + q.z * q.z));
            let g = Goal { x: gp.position.x, y: gp.position.y, yaw, handle: handle.clone(), started: Instant::now(), cancel: false };
            log_info!(&log2, "goal ({:.2}, {:.2})", g.x, g.y);
            {
                let mut s = st.borrow_mut();
                if let Some(mut old) = s.goal.take() {
                    let _ = old.handle.abort(NavigateToPose::Result::default());
                }
                s.goal = Some(g);
                // keep MPPI warm start across goal switches, reset only from standstill (stop())
                s.path.clear();
            }
            let st2 = st.clone();
            let uuid = handle.uuid;
            let _ = sp.spawn_local(async move {
                while let Some(c) = cancels.next().await {
                    let mut s = st2.borrow_mut();
                    if let Some(g) = s.goal.as_mut().filter(|g| g.handle.uuid == uuid) {
                        g.cancel = true;
                        c.accept();
                    } else {
                        c.reject();
                    }
                }
            });
        }
    })?;

    let io = Rc::new(Io {
        cmd: node.create_publisher::<TwistStamped>(&cmd_topic, QosProfile::default())?,
        path: node.create_publisher::<PathMsg>("/team7/path", QosProfile::default())?,
        traj: node.create_publisher::<PathMsg>("/team7/mppi/trajectory", QosProfile::default())?,
        local: node.create_publisher::<OccupancyGrid>("/team7/mppi/local_cost", QosProfile::default())?,
        global: node.create_publisher::<OccupancyGrid>("/global_costmap/costmap", QosProfile::default().reliable().transient_local())?,
        follow: node.create_action_client::<Follow>(&follow_action)?,
    });

    let clock = node.get_ros_clock();
    let mut timer = node.create_wall_timer(Duration::from_millis(50))?;
    let st = state.clone();
    let sp = spawner.clone();
    spawner.spawn_local(async move {
        loop {
            if timer.tick().await.is_err() {
                return;
            }
            let now = clock.lock().unwrap().get_now().map(|d| r2r::Clock::to_builtin_time(&d)).unwrap_or_default();
            tick(&log, &cfg, &st, &tf.borrow(), &io, &sp, now);
        }
    })?;

    team7_nodes::run(node, pool, || false);
    Ok(())
}

fn tick(
    log: &str,
    cfg: &Cfg,
    state: &Rc<RefCell<State>>,
    tf: &TfBuffer,
    io: &Rc<Io>,
    spawner: &LocalSpawner,
    now: r2r::builtin_interfaces::msg::Time,
) {
    let mut s = state.borrow_mut();

    if s.map_dirty
        && let Some(m) = s.map.as_ref()
    {
        let cs = CSpace::new(m, &cfg.cs);
        s.cspace = Some(cs);
        s.map_dirty = false;
    }
    // built even without a goal, so the local MPPI layer is up from the start
    if cfg.mode == Mode::Mppi && s.last_obstacles.elapsed() >= cfg.obstacle_period {
        s.last_obstacles = Instant::now();
        let global_due = s.last_global.elapsed() >= Duration::from_secs(1);
        let mut global_sent = false;
        if let Some(m) = s.map.as_ref() {
            let mut g = m.clone();
            for &(x, y) in &s.scan_points {
                if let Some(c) = g.world_to_cell(x, y) {
                    g.set(c, 100);
                }
            }
            let df = DistanceField::from_grid(&g, cfg.cs.occupied_threshold, false);            // team7 global costmap: team7 mode runs no Nav2, so publish ours on the usual topic
            // the costmap is big and only changes with the map: publish about once a second
            if global_due {
                global_sent = true;
                let _ = io.global.publish(&global_cost_msg(&cfg.map_frame, m, &df, &now));
            }
            s.obstacles = Some(df);
            // publish here, not in the MPPI branch: that branch needs a valid path
            if let (Some(df), Ok(t)) = (s.obstacles.as_ref(), tf.lookup(&cfg.map_frame, &cfg.base_frame, Stamp::LATEST)) {
                let r = Pose2::from_isometry(&t);
                let _ = io.local.publish(&local_cost_msg(&cfg.map_frame, df, r.x, r.y, &now));
            }
        }
        if global_sent {
            s.last_global = Instant::now();
        }
    }

    let finish = |s: &mut State, ok: bool, why: &str| {
        if let Some(mut g) = s.goal.take() {
            let r = NavigateToPose::Result::default();
            let _ = if g.cancel {
                g.handle.cancel(r)
            } else if ok {
                g.handle.succeed(r)
            } else {
                g.handle.abort(r)
            };
            log_info!(log, "goal ({:.2}, {:.2}) {why}", g.x, g.y);
        }
        if let Some(f) = s.follow.take() {
            let _ = f.cancel();
        }
        s.path.clear();
    };

    if s.done && s.goal.is_some() {
        finish(&mut s, false, "dropped: mission done");
    }
    let Some(g) = s.goal.as_ref() else {
        // no goal: still show the local MPPI layer around the robot
        if let (Some(df), Ok(t)) = (s.obstacles.as_ref(), tf.lookup(&cfg.map_frame, &cfg.base_frame, Stamp::LATEST)) {
            let r = Pose2::from_isometry(&t);
            let _ = io.local.publish(&local_cost_msg(&cfg.map_frame, df, r.x, r.y, &now));
        }
        stop(&mut s, io, cfg, &now);
        return;
    };
    let (gx, gy, gyaw, cancel, started) = (g.x, g.y, g.yaw, g.cancel, g.started);
    if cancel {
        finish(&mut s, false, "canceled");
        stop(&mut s, io, cfg, &now);
        return;
    }
    if started.elapsed() > cfg.goal_timeout {
        finish(&mut s, false, "aborted: timeout");
        stop(&mut s, io, cfg, &now);
        return;
    }
    let robot = match tf.lookup(&cfg.map_frame, &cfg.base_frame, Stamp::LATEST) {
        Ok(t) => Pose2::from_isometry(&t),
        Err(e) => {
            log_warn!(log, "no robot pose: {e}");
            stop(&mut s, io, cfg, &now);
            return;
        }
    };

    let need_plan = s.path.is_empty() || s.last_plan.elapsed() >= cfg.replan_period;
    if need_plan {
        let Some(cs) = s.cspace.take() else {
            stop(&mut s, io, cfg, &now);
            return;
        };
        let res = s.planner.lazy_theta_star(&cs, (robot.x, robot.y), (gx, gy), &cfg.pp);
        s.last_plan = Instant::now();
        match res {
            Ok(p) => {
                let pts = if cfg.smooth_iterations > 0 {
                    tb4_planning::smooth(&cs, &p.points, cfg.smooth_iterations, 0.3)
                } else {
                    p.points
                };
                let changed = pts != s.path;
                s.path = pts;
                let msg = path_msg(&cfg.map_frame, &s.path, &now);
                let _ = io.path.publish(&msg);
                match cfg.mode {
                    Mode::Mppi => {
                        let st = &mut *s;
                        st.mppi.set_path(&st.path, Some(gyaw));
                    }
                    Mode::Nav2Controller if changed || s.follow.is_none() => {
                        // nav2 critics index path points, any-angle corners alone give them nothing to follow
                        let dense = tb4_planning::densify(&s.path, 0.05);
                        send_follow(&mut s, state, io, cfg, spawner, path_msg(&cfg.map_frame, &dense, &now));
                    }
                    Mode::Nav2Controller => {}
                }
            }
            Err(e) => {
                s.cspace = Some(cs);
                finish(&mut s, false, &format!("aborted: {e}"));
                stop(&mut s, io, cfg, &now);
                return;
            }
        }
        s.cspace = Some(cs);
    }

    match cfg.mode {
        Mode::Mppi => {
            let st = RobotState { pose: robot, v: s.v, w: s.w };
            let obstacles = s.obstacles.take();
            let cmd = s.mppi.compute(&st, obstacles.as_ref());
            s.obstacles = obstacles;
            let traj: Vec<(f64, f64)> = s.mppi.optimal_trajectory().iter().map(|p| (p.x, p.y)).collect();
            let _ = io.traj.publish(&path_msg(&cfg.map_frame, &traj, &now));
            if let Some(df) = s.obstacles.as_ref() {
                let _ = io.local.publish(&local_cost_msg(&cfg.map_frame, df, robot.x, robot.y, &now));
            }
            if cmd.goal_reached {
                finish(&mut s, true, "reached");
                stop(&mut s, io, cfg, &now);
                return;
            }
            if !cmd.collision_free {
                s.path.clear(); // force replan next tick
            }
            let (v, w) = apply_speed_limit(s.speed_limit, s.mppi.params().vx_max, cmd.v, cmd.w);
            publish_cmd(io, cfg, &now, v, w);
            s.stopped = false;
        }
        Mode::Nav2Controller => {
            if let Some((gen_, status)) = s.follow_result.take()
                && gen_ == s.follow_gen
            {
                match status {
                    GoalStatus::Succeeded => finish(&mut s, true, "reached"),
                    GoalStatus::Canceled => {}
                    other => {
                        log_warn!(log, "FollowPath ended {other:?}, replanning");
                        s.follow = None;
                        s.path.clear();
                    }
                }
            }
        }
    }
}

fn send_follow(s: &mut State, state: &Rc<RefCell<State>>, io: &Rc<Io>, cfg: &Cfg, spawner: &LocalSpawner, path: PathMsg) {
    let goal = FollowPath::Goal { path, controller_id: cfg.controller_id.clone(), ..Default::default() };
    let Ok(req) = io.follow.send_goal_request(goal) else { return };
    s.follow_gen += 1;
    let gen_ = s.follow_gen;
    let st = state.clone();
    // runs after tick() released the borrow
    let _ = spawner.spawn_local(async move {
        let status = match req.await {
            Ok((handle, result, _)) => {
                if st.borrow().follow_gen == gen_ {
                    st.borrow_mut().follow = Some(handle);
                }
                result.await.map(|(status, _)| status).unwrap_or(GoalStatus::Aborted)
            }
            Err(_) => GoalStatus::Aborted,
        };
        st.borrow_mut().follow_result = Some((gen_, status));
    });
}

// same meaning as nav2 controller: percentage of max speed, or absolute m/s; angular scaled alike
fn apply_speed_limit((pct, lim): (bool, f64), vx_max: f64, v: f64, w: f64) -> (f64, f64) {
    if lim <= 0.0 || vx_max <= 0.0 {
        return (v, w);
    }
    let k = if pct { (lim / 100.0).min(1.0) } else { (lim / vx_max).min(1.0) };
    (v.clamp(-vx_max * k, vx_max * k), w * k)
}

fn stop(s: &mut State, io: &Rc<Io>, cfg: &Cfg, now: &r2r::builtin_interfaces::msg::Time) {
    if cfg.mode == Mode::Mppi && !s.stopped {
        publish_cmd(io, cfg, now, 0.0, 0.0);
        s.stopped = true;
        s.mppi.reset();
    }
}

fn publish_cmd(io: &Rc<Io>, cfg: &Cfg, now: &r2r::builtin_interfaces::msg::Time, v: f64, w: f64) {
    let mut m = TwistStamped::default();
    m.header.stamp = now.clone();
    m.header.frame_id = cfg.base_frame.clone();
    m.twist.linear.x = v;
    m.twist.angular.z = w;
    let _ = io.cmd.publish(&m);
}

fn path_msg(frame: &str, pts: &[(f64, f64)], now: &r2r::builtin_interfaces::msg::Time) -> PathMsg {
    let mut m = PathMsg::default();
    m.header.frame_id = frame.to_owned();
    m.header.stamp = now.clone();
    m.poses = pts
        .iter()
        .enumerate()
        .map(|(i, &(x, y))| {
            let yaw = pts.get(i + 1).map_or_else(
                || if i > 0 { (y - pts[i - 1].1).atan2(x - pts[i - 1].0) } else { 0.0 },
                |n| (n.1 - y).atan2(n.0 - x),
            );
            PoseStamped { header: m.header.clone(), pose: pose_msg(x, y, yaw) }
        })
        .collect();
    m
}
