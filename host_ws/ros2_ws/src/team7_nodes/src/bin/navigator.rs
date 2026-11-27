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
use tb4_planning::{CSpace, CSpaceParams, IrParams, Planner, PlannerParams, VirtualObstacles, VirtualParams, bump_zone_angle, ir_points, remaining_free};
use team7_nodes::{Params, grid_from_msg, pose_msg, spawn_tf_listener};

type Nav = NavigateToPose::Action;
/// NavigateToPose behavior_tree value for a goal without a final heading
const XY_ONLY: &str = "xy_only";
type Follow = FollowPath::Action;
/// irobot_create_msgs HazardDetection.type value for a bumper contact (BACKUP_LIMIT=0, BUMP=1, CLIFF=2, ...)
const BUMP_TYPE: u8 = 1;
/// irobot_create_msgs HazardDetection.type for a cliff sensor reading
const CLIFF_TYPE: u8 = 2;

/// Direction of a cliff sensor from the robot's forward axis (rad). Cliff sensors are at the front only
/// (Create 3 docs); the frame names are cliff_front_left, cliff_front_right, cliff_side_left, cliff_side_right.
fn cliff_zone_angle(frame_id: &str) -> Option<f64> {
    let deg = match frame_id.trim_start_matches('/') {
        "cliff_front_left" => 30.0,
        "cliff_front_right" => -30.0,
        "cliff_side_left" => 75.0,
        "cliff_side_right" => -75.0,
        _ => return None,
    };
    Some(f64::to_radians(deg))
}

#[derive(Clone, Copy, PartialEq, Debug)]
enum Mode {
    Nav2Controller,
    Mppi,
}

struct Goal {
    handle: ActionServerGoal<Nav>,
    x: f64,
    y: f64,
    /// None: goal with behavior_tree "xy_only", the final heading does not matter
    yaw: Option<f64>,
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
    /// keep the current route while it is free unless a new one is shorter than this fraction of it
    path_switch_ratio: f64,
}

struct State {
    map: Option<Grid>,
    map_dirty: bool,
    cspace: Option<CSpace>,
    obstacles: Option<DistanceField>,
    last_obstacles: Instant,
    last_global: Instant,
    scan_points: Vec<(f64, f64)>,
    /// bump marks the lidar cannot see (glass, low objects), stamped into the obstacle grid
    bumps: VirtualObstacles,
    /// IR proximity marks: short life, so a hand that leaves the sensor's view does not leave a ghost wall
    ir_marks: VirtualObstacles,
    /// count of marks last used for the planner grid; a change rebuilds the CSpace
    marks_seen: u64,
    /// cliff marks are off by default: a cliff is a drop and its arc sits inside the footprint
    cliff_on: bool,
    /// latest scan stamp (s): the clock for mark ageing, so replays and the robot agree
    now: f64,
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
    /// stage timings (obstacle field, cspace, planner, mppi), logged every TIMING_LOG_PERIOD
    timing: [StageTime; 4],
    timing_logged: Instant,
}

/// mean / max of one navigator stage since the last timing log line
#[derive(Default, Clone, Copy)]
struct StageTime {
    n: u32,
    sum_ms: f64,
    max_ms: f64,
}

impl StageTime {
    fn add(&mut self, since: Instant) {
        self.add_dur(since.elapsed());
    }

    fn add_dur(&mut self, d: Duration) {
        let ms = d.as_secs_f64() * 1e3;
        self.n += 1;
        self.sum_ms += ms;
        self.max_ms = self.max_ms.max(ms);
    }
}

const TIMING_LOG_PERIOD: Duration = Duration::from_secs(10);
const STAGES: [&str; 4] = ["obstacle field", "cspace", "planner", "mppi"];

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
    // read before the node is borrowed mutably below (Params borrows it)
    let ir_on = p.bool("ir_enabled", false);
    let cliff_marks = p.bool("cliff_marks", false);
    let ir_threshold = p.i64("ir_threshold", 1000) as i32;
    let ir_range = p.f64("ir_range", 0.10);

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
        path_switch_ratio: p.f64("path_switch_ratio", 0.8),
    });
    let mp = MppiParams {
        samples: p.i64("mppi_samples", 1000) as usize,
        horizon: p.i64("mppi_horizon", 40) as usize,
        threads: p.i64("mppi_threads", 4) as usize,
        vx_max: p.f64("vx_max", 0.30),
        wz_max: p.f64("wz_max", 1.9),
        goal_tolerance_xy: p.f64("goal_tolerance_xy", 0.15),
        goal_tolerance_yaw: p.f64("goal_tolerance_yaw", 0.3),
        // goal clearance was fixed at 0.33 m (more than the centre distance of a 0.6 m gap): a goal in a gap was
        // cut back to its mouth and counted as reached. 0.20 m is the robot radius plus 2 cm.
        goal_clearance: p.f64("goal_clearance", 0.20),
        goal_backoff: p.f64("goal_backoff", 0.4),
        // heading error that starts a rotate-in-place, and the error that ends it (was fixed at 1.2 and 0.35 rad)
        rotate_in_place_angle: p.f64("rotate_in_place_angle", 1.2),
        rotate_exit_angle: p.f64("rotate_exit_angle", 0.35),
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
        bumps: VirtualObstacles::new(VirtualParams::default()),
        ir_marks: VirtualObstacles::new(VirtualParams { ttl: 2.0, ..VirtualParams::default() }),
        marks_seen: 0,
        cliff_on: cliff_marks,
        now: 0.0,
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
        timing: [StageTime::default(); 4],
        timing_logged: Instant::now(),
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
        st.borrow_mut().now = m.header.stamp.sec as f64 + m.header.stamp.nanosec as f64 * 1e-9;
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

    // Create 3 IR proximity sensors: a reading above the threshold puts an obstacle point just in front
    // of that sensor (ir_points). Off by default: the intensity-to-distance numbers are not calibrated yet.
    if ir_on {
        let st = state.clone();
        let tf_ir = tf.clone();
        let (map_f, base_f) = (cfg.map_frame.clone(), cfg.base_frame.clone());
        let ir_params = IrParams { threshold: ir_threshold, range: ir_range };
        let ir = node.subscribe::<r2r::irobot_create_msgs::msg::IrIntensityVector>("/ir_intensity", QosProfile::sensor_data())?;
        spawner.spawn_local(ir.for_each(move |m| {
            let mut s = st.borrow_mut();
            let now = s.now;
            let readings: Vec<(String, i32)> = m.readings.iter().map(|r| (r.header.frame_id.clone(), r.value as i32)).collect();
            let refs: Vec<(&str, i32)> = readings.iter().map(|(f, v)| (f.as_str(), *v)).collect();
            if refs.is_empty() {
                return futures::future::ready(());
            }
            if let Ok(t) = tf_ir.borrow().lookup(&map_f, &base_f, Stamp::LATEST) {
                let pose = Pose2::from_isometry(&t);
                for (x, y) in ir_points(pose, &refs, &ir_params) {
                    s.ir_marks.add_point(x, y, now);
                }
            }
            futures::future::ready(())
        }))?;
    }

    // Create 3 bumper contacts: each BUMP detection marks a short arc where the bumper touched, at the
    // robot's pose at that moment. Stamped into the obstacle grid below; ages out after the TTL.
    let st = state.clone();
    let tf_bump = tf.clone();
    let (map_f, base_f) = (cfg.map_frame.clone(), cfg.base_frame.clone());
    let log_bump = log.clone();
    let hazards = node.subscribe::<r2r::irobot_create_msgs::msg::HazardDetectionVector>("/hazard_detection", QosProfile::default())?;
    spawner.spawn_local(hazards.for_each(move |m| {
        let mut s = st.borrow_mut();
        let bump_now = s.now;
        // no scan stamp yet means no clock: the mark would be seen at time 0 and dropped at once
        if bump_now <= 0.0 {
            return futures::future::ready(());
        }

        for d in &m.detections {
            // bumper contact or cliff sensor: both mark a no-go arc in the grid the planner uses
            let angle = match d.type_ {
                BUMP_TYPE => bump_zone_angle(&d.header.frame_id),
                CLIFF_TYPE if s.cliff_on => cliff_zone_angle(&d.header.frame_id),
                _ => None,
            };
            let Some(angle) = angle else { continue };
            if let Ok(t) = tf_bump.borrow().lookup(&map_f, &base_f, Stamp::LATEST) {
                let pose = Pose2::from_isometry(&t);
                let added = s.bumps.add_bump(pose, angle, bump_now);
                log_info!(&log_bump, "bump {} at ({:.2}, {:.2}), {} mark points", d.header.frame_id, pose.x, pose.y, added);
            }
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
            // frontier goals: the lidar sees all round, so stopping to turn at every goal only costs
            // time (the explorer marks them "xy_only"); camera views keep their heading
            let yaw = (handle.goal.behavior_tree != XY_ONLY)
                .then(|| (2.0 * (q.w * q.z + q.x * q.y)).atan2(1.0 - 2.0 * (q.y * q.y + q.z * q.z)));
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

    if s.timing_logged.elapsed() >= TIMING_LOG_PERIOD {
        let parts: Vec<String> = STAGES
            .iter()
            .zip(s.timing)
            .filter(|(_, t)| t.n > 0)
            .map(|(name, t)| format!("{name} {}x mean {:.1} max {:.1} ms", t.n, t.sum_ms / t.n as f64, t.max_ms))
            .collect();
        if !parts.is_empty() {
            log_info!(log, "timing: {}", parts.join(", "));
        }
        s.timing = [StageTime::default(); 4];
        s.timing_logged = Instant::now();
    }

    // marks age out in every mode, not only MPPI
    let mark_now = s.now;
    s.bumps.expire(mark_now);
    s.ir_marks.expire(mark_now);
    // the planner must see the same marks as MPPI, or the global path runs through a bumped zone
    let marks = s.bumps.revision() + s.ir_marks.revision();
    if (s.map_dirty || marks != s.marks_seen)
        && let Some(m) = s.map.as_ref()
    {
        let t = Instant::now();
        let mut planner_grid = m.clone();
        s.bumps.stamp(&mut planner_grid, 100);
        s.ir_marks.stamp(&mut planner_grid, 100);
        let cs = CSpace::new(&planner_grid, &cfg.cs);
        s.timing[1].add(t);
        s.cspace = Some(cs);
        s.map_dirty = false;
        s.marks_seen = marks;
    }
    // built even without a goal, so the local MPPI layer is up from the start
    if cfg.mode == Mode::Mppi && s.last_obstacles.elapsed() >= cfg.obstacle_period {
        s.last_obstacles = Instant::now();
        let global_due = s.last_global.elapsed() >= Duration::from_secs(1);
        let mut global_sent = false;
        let mut obs_time = None;
        // bumper marks age out first (needs &mut), then go into the grid below
        if let Some(m) = s.map.as_ref() {
            let t_obs = Instant::now();
            let mut g = m.clone();
            for &(x, y) in &s.scan_points {
                if let Some(c) = g.world_to_cell(x, y) {
                    g.set(c, 100);
                }
            }
            // bumper marks: age out, then write into the grid the planner and MPPI use
            s.bumps.stamp(&mut g, 100);
            s.ir_marks.stamp(&mut g, 100);
            let df = DistanceField::from_grid(&g, cfg.cs.occupied_threshold, false);
            obs_time = Some(t_obs.elapsed());            // team7 global costmap: team7 mode runs no Nav2, so publish ours on the usual topic
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
        if let Some(d) = obs_time {
            s.timing[0].add_dur(d);
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
        let t = Instant::now();
        let res = s.planner.lazy_theta_star(&cs, (robot.x, robot.y), (gx, gy), &cfg.pp);
        s.timing[2].add(t);
        s.last_plan = Instant::now();
        match res {
            Ok(p) => {
                let pts = if cfg.smooth_iterations > 0 {
                    tb4_planning::smooth(&cs, &p.points, cfg.smooth_iterations, 0.3)
                } else {
                    p.points
                };
                // Field run 20261118_184838: among chair legs every 2 s replan picked another way
                // round, and the robot circled for 45 s until the goal timed out. Keep the route
                // it is on while that is still free, unless the new one is clearly shorter.
                let pts = match remaining_free(&cs, &s.path, (robot.x, robot.y)) {
                    Some(rest) if tb4_planning::path_length(&pts) >= cfg.path_switch_ratio * tb4_planning::path_length(&rest) => s.path.clone(),
                    _ => pts,
                };
                let changed = pts != s.path;
                s.path = pts;
                let msg = path_msg(&cfg.map_frame, &s.path, &now);
                let _ = io.path.publish(&msg);
                match cfg.mode {
                    // same route kept: leave MPPI's tracking state (rotate / goal reached) alone
                    Mode::Mppi if changed => {
                        let st = &mut *s;
                        st.mppi.set_path(&st.path, gyaw);
                    }
                    Mode::Mppi => {}
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
            let t = Instant::now();
            let cmd = s.mppi.compute(&st, obstacles.as_ref());
            s.timing[3].add(t);
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
