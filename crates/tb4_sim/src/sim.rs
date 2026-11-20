//! The closed loop on a simulated clock: physics 100 Hz, lidar 5.5 Hz into SLAM, /map every 1 s,
//! navigator + velocity chain 20 Hz, explorer 1 Hz (from 10 s, like the launch file), coverage 5 Hz,
//! camera 15 Hz (detections into the marker tracker). Nothing reads the wall clock except the wall-time
//! figure in the metrics.

use std::collections::BTreeMap;
use std::time::Instant;

use tb4_control::Rng;
use tb4_core::{Grid, Pose2};
use tb4_explore::{Action, ApproachParams, CoverageParams, ExploreParams, ExplorerParams, ExplorerPolicy, MarkerHint, TickInput};
use tb4_markers::{LocateParams, Tracker};
use tb4_slam::{Slam, SlamParams};

use crate::chain::{Chain, ChainParams};
use crate::metrics::{AREA_TIMES, MarkerRecord, Metrics, Motion, STILL_V, STILL_W, StallTracker, classify, free_area};
use crate::nav::{NavParams, NavState, NavStatus, Navigator};
use crate::robot::{Robot, RobotParams, wrap};
use crate::sensors::{CameraParams, LidarParams, Scan, camera_pose, scan, view_marker};
use crate::vision::{VisionParams, detect, team7_locate_params, team7_track_params};
use crate::world::World;

/// wheel odometry errors (Create 3 odom drifts slowly)
#[derive(Debug, Clone)]
pub struct OdomParams {
    pub trans_scale: f64,
    pub yaw_scale: f64,
    /// rad/s
    pub yaw_bias: f64,
    /// per-step noise: sigma * sqrt(|step|)
    pub trans_noise: f64,
    pub yaw_noise: f64,
}

impl Default for OdomParams {
    fn default() -> Self {
        Self { trans_scale: 1.01, yaw_scale: 1.005, yaw_bias: 0.0003, trans_noise: 0.002, yaw_noise: 0.002 }
    }
}

/// marker_mapper slow_on_tentative: /speed_limit while a marker that is not yet confirmed is in view
#[derive(Debug, Clone)]
pub struct SlowParams {
    pub enabled: bool,
    pub percent: f64,
    pub hold: f64,
    /// without the tracker (`vision.enabled` false): views (camera moved 0.05 m or turned 0.05 rad)
    /// after which a marker counts as confirmed
    pub min_views: u64,
}

impl Default for SlowParams {
    fn default() -> Self {
        Self { enabled: true, percent: 60.0, hold: 2.5, min_views: 5 }
    }
}

#[derive(Debug, Clone)]
pub struct SimConfig {
    pub world: World,
    pub seed: u64,
    /// map with the ground-truth pose instead of scan matching (SLAM trusts noise-free odometry)
    pub perfect_pose: bool,
    pub robot: RobotParams,
    pub lidar: LidarParams,
    pub camera: CameraParams,
    pub odom: OdomParams,
    pub slam: SlamParams,
    pub nav: NavParams,
    pub chain: ChainParams,
    pub explorer: ExplorerParams,
    pub slow: SlowParams,
    /// detections of markers and decoys into tb4_markers::Tracker
    pub vision: VisionParams,
    pub locate: LocateParams,
    pub track: tb4_markers::TrackParams,
    /// the launch file starts the explorer 10 s after the rest
    pub explorer_start: f64,
    pub explorer_period: f64,
    pub coverage_period: f64,
    pub map_period: f64,
    pub nav_period: f64,
    pub physics_dt: f64,
}

/// explorer section of team7.yaml (working tree values of 2026-11-19)
pub fn team7_explorer_params() -> ExplorerParams {
    let cov = CoverageParams { half_fov: 0.52, range: 1.5, max_incidence: 1.3, standoff: 1.0, max_cluster_cells: 32, ..Default::default() };
    ExplorerParams {
        approach_budget: 0.0,
        idle_fallback: false,
        goal_timeout: 120.0,
        progress_timeout: 15.0,
        progress_epsilon: 0.1,
        retry_delay: 5.0,
        blacklist_duration: 45.0,
        blacklist_max_duration: 300.0,
        turn_weight: 0.5,
        costmap_timeout: 5.0,
        explore: ExploreParams {
            free_threshold: 50,
            min_frontier_cells: 12,
            size_weight: 1.0,
            distance_weight: 1.5,
            blacklist_radius: 0.7,
            min_goal_distance: 0.65,
            goal_standoff: 0.3,
            traversable_threshold: 99,
            frontier_segment_length: 1.2,
            min_unknown_area: 1.0,
            goal_max_unknown: 0.15,
            revisit_radius: 0.0,
            revisit_penalty: 0.0,
            gain_weight: 0.0,
            gain_radius: 2.0,
                min_free_fraction: 0.5,
        },
        coverage: Some(cov),
        view_detour: 1.0,
        look_around_turns: 3,
        preempt_distance: 0.5,
        switch_cooldown: 20.0,
        approach: Some(ApproachParams::default()),
    }
}

/// SLAM that keeps the odometry pose: no scan matching, no ICP, no loop closures
pub fn perfect_pose_slam(mut p: SlamParams) -> SlamParams {
    p.search_lin = 0.0;
    p.search_ang = 0.0;
    p.min_match_score = f64::INFINITY;
    p.icp = false;
    p.loop_search_radius = 0.0;
    p.loop_min_score = f64::INFINITY;
    p
}

impl SimConfig {
    /// team7.yaml parameters in `world`
    pub fn team7(world: World) -> Self {
        Self {
            world,
            seed: 1,
            perfect_pose: false,
            robot: RobotParams::default(),
            lidar: LidarParams::default(),
            camera: CameraParams::default(),
            odom: OdomParams::default(),
            slam: SlamParams { resolution: 0.05, min_travel_dist: 0.1, min_travel_yaw: 0.1, max_range: 8.0, loop_prefer_oldest: false, ..SlamParams::default() },
            nav: NavParams::team7(),
            chain: ChainParams::default(),
            explorer: team7_explorer_params(),
            slow: SlowParams::default(),
            vision: VisionParams::default(),
            locate: team7_locate_params(),
            track: team7_track_params(),
            explorer_start: 10.0,
            explorer_period: 1.0,
            coverage_period: 0.2,
            map_period: 1.0,
            nav_period: 0.05,
            physics_dt: 0.01,
        }
    }
}

/// names `SimConfig::set` accepts
pub const SET_NAMES: [&str; 45] = [
    "min_triangulation_spread",
    "min_free_fraction",
    "require_scale_all",
    "max_consistency", "scale_tolerance", "scale_min_samples",
    "goal_timeout", "progress_timeout", "retry_delay", "blacklist_duration", "blacklist_max_duration", "turn_weight", "distance_weight",
    "size_weight", "min_goal_distance", "goal_standoff", "blacklist_radius", "camera_range", "view_standoff",
    "view_detour", "preempt_distance", "switch_cooldown", "min_unknown_area", "revisit_radius", "revisit_penalty", "approach_budget", "gain_weight", "gain_radius", "idle_fallback", "min_frontier_cells", "plan_with_scan", "look_around_turns", "camera_coverage", "vx_max", "wz_max",
    "path_switch_ratio", "slow_on_tentative", "decoys", "vision", "pnp_sigma", "bearing_sigma_deg",
    "approach_markers", "approach_standoff", "approach_spread_deg", "approach_timeout",
];

impl SimConfig {
    /// one `name=value` override, as `--set` takes it on the command line
    pub fn set(&mut self, kv: &str) -> Result<(), String> {
        let (k, v) = kv.split_once('=').ok_or(format!("--set {kv}: want name=value"))?;
        let x: f64 = v.parse().map_err(|_| format!("--set {kv}: not a number"))?;
        // any MppiParams field, e.g. mppi.temperature=0.3 (vx_max / wz_max below also move the chain)
        if let Some(f) = k.strip_prefix("mppi.") {
            return self.nav.mppi.set(f, x).map_err(|e| format!("--set {kv}: {e}"));
        }
        let e = &mut self.explorer;
        match k {
            "goal_timeout" => e.goal_timeout = x,
            "progress_timeout" => e.progress_timeout = x,
            "retry_delay" => e.retry_delay = x,
            "blacklist_duration" => e.blacklist_duration = x,
            "blacklist_max_duration" => e.blacklist_max_duration = x,
            "turn_weight" => e.turn_weight = x,
            "distance_weight" => e.explore.distance_weight = x,
            "size_weight" => e.explore.size_weight = x,
            "min_goal_distance" => e.explore.min_goal_distance = x,
            "goal_standoff" => e.explore.goal_standoff = x,
            "blacklist_radius" => e.explore.blacklist_radius = x,
            "camera_range" => e.coverage.iter_mut().for_each(|c| c.range = x),
            "view_standoff" => e.coverage.iter_mut().for_each(|c| c.standoff = x),
            "view_detour" => e.view_detour = x,
            "preempt_distance" => e.preempt_distance = x,
            "switch_cooldown" => e.switch_cooldown = x,
            "min_unknown_area" => e.explore.min_unknown_area = x,
            "revisit_radius" => e.explore.revisit_radius = x,
            "approach_budget" => e.approach_budget = x,
            "gain_weight" => e.explore.gain_weight = x,
            "gain_radius" => e.explore.gain_radius = x,
            "idle_fallback" => e.idle_fallback = x != 0.0,
            "revisit_penalty" => e.explore.revisit_penalty = x,
            "min_frontier_cells" => e.explore.min_frontier_cells = x as usize,
            "look_around_turns" => e.look_around_turns = x as u32,
            "camera_coverage" if x == 0.0 => e.coverage = None,
            "camera_coverage" => {}
            // like the real chain: navigator, velocity_smoother and velocity_guard move together
            "vx_max" => {
                self.nav.mppi.vx_max = x;
                self.chain.smoother.max_v = x;
                self.chain.guard.max_forward = x;
            }
            "wz_max" => {
                self.nav.mppi.wz_max = x;
                self.chain.smoother.max_w = x;
                self.chain.guard.max_turn = x;
            }
            "path_switch_ratio" => self.nav.path_switch_ratio = x,
            "plan_with_scan" => self.nav.plan_with_scan = x != 0.0,
            "slow_on_tentative" => self.slow.enabled = x != 0.0,
            "decoys" if x == 0.0 => self.world.decoys.clear(),
            "decoys" => {}
            "vision" => self.vision.enabled = x != 0.0,
            "pnp_sigma" => self.vision.pnp_sigma = x,
            "bearing_sigma_deg" => self.vision.bearing_sigma = x.to_radians(),
            "require_scale_all" => self.track.require_scale_all = x != 0.0,
            "max_consistency" => self.track.max_consistency = x,
            "scale_tolerance" => self.track.scale_tolerance = x,
            "scale_min_samples" => self.track.scale_min_samples = x as usize,
            "min_triangulation_spread" => self.track.min_triangulation_spread = x,
            "min_free_fraction" => e.explore.min_free_fraction = x,
            "approach_markers" if x == 0.0 => e.approach = None,
            "approach_markers" => {
                e.approach.get_or_insert_with(ApproachParams::default);
            }
            "approach_standoff" => e.approach.iter_mut().for_each(|a| a.standoff = x),
            "approach_spread_deg" => e.approach.iter_mut().for_each(|a| a.spread = x.to_radians()),
            "approach_timeout" => e.approach.iter_mut().for_each(|a| a.timeout = x),
            _ => return Err(format!("--set: unknown name {k}")),
        }
        Ok(())
    }
}

/// everything a viewer needs for one frame; map-frame items come with `map_origin` (world pose of the map frame)
pub struct SimState<'a> {
    pub t: f64,
    pub world: &'a World,
    pub map_origin: Pose2,
    /// ground truth, world frame
    pub truth: Pose2,
    /// map -> base_link estimate, map frame
    pub estimate: Pose2,
    pub map: Option<&'a Grid>,
    pub path: &'a [(f64, f64)],
    pub goal: Option<(f64, f64)>,
    pub scan: &'a Scan,
    pub cmd: (f64, f64),
    pub markers: &'a [MarkerRecord],
    pub collisions: u64,
    /// world-frame contact points of the bumps (front bumper zones, one per bump)
    pub bumps: &'a [(f64, f64)],
}

impl SimState<'_> {
    /// text view of the SLAM map in world coordinates, `cell` metres per character, north up:
    /// '#' occupied, '.' free, ' ' unknown, 'R' robot (truth), 'M' marker, 'G' goal
    pub fn ascii_map(&self, cell: f64) -> String {
        let [x0, y0, x1, y1] = self.world.bounds;
        let (nx, ny) = (((x1 - x0) / cell).ceil() as usize, ((y1 - y0) / cell).ceil() as usize);
        let o = self.map_origin;
        let mut rows = vec![vec![b' '; nx]; ny];
        for (j, row) in rows.iter_mut().enumerate() {
            for (i, ch) in row.iter_mut().enumerate() {
                let (wx, wy) = (x0 + (i as f64 + 0.5) * cell, y1 - (j as f64 + 0.5) * cell);
                let m = relative(o, Pose2::new(wx, wy, 0.0));
                let v = self.map.and_then(|g| g.world_to_cell(m.x, m.y).map(|c| g.get(c)));
                *ch = match v {
                    Some(v) if v >= 65 => b'#',
                    Some(v) if v >= 0 => b'.',
                    _ => b' ',
                };
            }
        }
        let mut put = |x: f64, y: f64, c: u8| {
            let (i, j) = (((x - x0) / cell).floor(), ((y1 - y) / cell).floor());
            if i >= 0.0 && j >= 0.0 && (i as usize) < nx && (j as usize) < ny {
                rows[j as usize][i as usize] = c;
            }
        };
        for m in &self.world.markers {
            put(m.pos.0, m.pos.1, b'M');
        }
        if let Some(g) = self.goal {
            let w = compose(o, Pose2::new(g.0, g.1, 0.0));
            put(w.x, w.y, b'G');
        }
        put(self.truth.x, self.truth.y, b'R');
        rows.into_iter().map(|r| String::from_utf8(r).unwrap_or_default() + "\n").collect()
    }
}

pub struct Sim {
    pub cfg: SimConfig,
    k: u64,
    t: f64,
    robot: Robot,
    /// odom -> base_link
    odom: Pose2,
    odom_rng: Rng,
    lidar_rng: Rng,
    slam: Slam,
    slam_dirty: bool,
    map: Option<Grid>,
    map_stamp: f64,
    nav: Navigator,
    nav_cmd: (f64, f64),
    chain: Chain,
    cmd: (f64, f64),
    explorer: ExplorerPolicy,
    nav_result: Option<((f64, f64), tb4_explore::NavStatus)>,
    scan: Scan,
    scan_base: Vec<(f64, f64)>,
    next_scan: f64,
    next_map: f64,
    next_nav: f64,
    next_explore: f64,
    next_cov: f64,
    next_cam: f64,
    slow_until: f64,
    last_view: Vec<Option<((f64, f64), f64)>>,
    bump_points: Vec<(f64, f64)>,
    tracker: Tracker,
    vision_rng: Rng,
    // metrics
    m: Metrics,
    stalls: StallTracker,
    area_next: usize,
    drive: crate::drive::DriveTracker,
    log: Vec<(f64, String)>,
    pub keep_log: bool,
    wall: Instant,
}

impl Sim {
    pub fn new(cfg: SimConfig) -> Self {
        let slam_p = if cfg.perfect_pose { perfect_pose_slam(cfg.slam.clone()) } else { cfg.slam.clone() };
        let record = |mk: &crate::world::Marker| MarkerRecord { id: mk.id, size: mk.size, closest: f64::INFINITY, ..Default::default() };
        let m = Metrics {
            world: cfg.world.name.clone(),
            seed: cfg.seed,
            perfect_pose: cfg.perfect_pose,
            reference_area: reference_area(&cfg.world, 0.05),
            markers: cfg.world.markers.iter().map(record).collect(),
            decoys: cfg.world.decoys.iter().map(record).collect(),
            ..Default::default()
        };
        let mut mppi = cfg.nav.clone();
        mppi.mppi.seed ^= cfg.seed;
        Self {
            robot: Robot::new(cfg.robot.clone(), cfg.world.start),
            odom: Pose2::default(),
            odom_rng: Rng::stream(cfg.seed, 1, 0, 0),
            lidar_rng: Rng::stream(cfg.seed, 2, 0, 0),
            slam: Slam::new(slam_p),
            slam_dirty: false,
            map: None,
            map_stamp: 0.0,
            nav: Navigator::new(mppi, 0.0),
            nav_cmd: (0.0, 0.0),
            chain: Chain::new(cfg.chain.clone()),
            cmd: (0.0, 0.0),
            explorer: ExplorerPolicy::new(cfg.explorer.clone(), 0.0),
            nav_result: None,
            scan: Scan::default(),
            scan_base: Vec::new(),
            next_scan: 0.0,
            next_map: 0.0,
            next_nav: 0.0,
            next_explore: cfg.explorer_start + cfg.explorer_period,
            next_cov: cfg.explorer_start + cfg.coverage_period,
            next_cam: 0.0,
            slow_until: f64::NEG_INFINITY,
            last_view: vec![None; cfg.world.markers.len()],
            bump_points: Vec::new(),
            tracker: Tracker::new(cfg.track.clone()),
            vision_rng: Rng::stream(cfg.seed, 3, 0, 0),
            m,
            stalls: StallTracker::default(),
            area_next: 0,
            drive: Default::default(),
            log: Vec::new(),
            keep_log: false,
            wall: Instant::now(),
            k: 0,
            t: 0.0,
            cfg,
        }
    }

    pub fn time(&self) -> f64 {
        self.t
    }

    /// world pose of the map / odom frame origin (the start pose)
    pub fn map_origin(&self) -> Pose2 {
        self.cfg.world.start
    }

    /// ground truth expressed in the map frame
    pub fn truth_in_map(&self) -> Pose2 {
        relative(self.cfg.world.start, self.robot.pose)
    }

    /// map -> base_link as the nodes see it (SLAM map->odom times odometry)
    pub fn estimate(&self) -> Pose2 {
        compose(self.slam.map_to_odom(), self.odom)
    }

    /// the explorer policy (frontiers, views, coverage, blacklist of the last tick)
    pub fn explorer(&self) -> &ExplorerPolicy {
        &self.explorer
    }

    /// the navigator (planned path, MPPI optimal trajectory, state)
    pub fn navigator(&self) -> &Navigator {
        &self.nav
    }

    /// the marker tracker fed by the simulated camera (map frame)
    pub fn tracker(&self) -> &Tracker {
        &self.tracker
    }

    pub fn log(&self) -> &[(f64, String)] {
        &self.log
    }

    pub fn state(&self) -> SimState<'_> {
        SimState {
            t: self.t,
            world: &self.cfg.world,
            map_origin: self.cfg.world.start,
            truth: self.robot.pose,
            estimate: self.estimate(),
            map: self.map.as_ref(),
            path: self.nav.path(),
            goal: self.nav.goal().map(|g| (g.x, g.y)),
            scan: &self.scan,
            cmd: self.cmd,
            markers: &self.m.markers,
            collisions: self.robot.collisions,
            bumps: &self.bump_points,
        }
    }

    fn note(&mut self, msg: String) {
        if self.keep_log {
            self.log.push((self.t, msg));
        }
    }

    /// run until simulated time `t_end` (s)
    pub fn run_until(&mut self, t_end: f64) {
        while self.t + 1e-9 < t_end {
            self.step();
        }
    }

    fn due(next: &mut f64, t: f64, period: f64) -> bool {
        if t + 1e-9 >= *next {
            *next += period;
            true
        } else {
            false
        }
    }

    /// one physics step, with every node whose timer is due
    pub fn step(&mut self) {
        let t = self.t;
        if Self::due(&mut self.next_scan, t, 1.0 / self.cfg.lidar.rate_hz) {
            self.do_scan();
        }
        if Self::due(&mut self.next_map, t, self.cfg.map_period) {
            self.publish_map();
        }
        if Self::due(&mut self.next_cam, t, 1.0 / self.cfg.camera.rate_hz) {
            self.do_camera();
        }
        if t + 1e-9 >= self.cfg.explorer_start && Self::due(&mut self.next_cov, t, self.cfg.coverage_period) {
            let est = self.estimate();
            if let Some(m) = self.map.as_ref() {
                self.explorer.update_coverage(m, (est.x, est.y, est.yaw));
                // marker face cell (just behind the face) in the map frame: did coverage count it?
                for (i, mk) in self.cfg.world.markers.iter().enumerate() {
                    if self.m.markers[i].coverage_seen.is_some() {
                        continue;
                    }
                    // the SLAM wall can sit a cell off the true face: any seen cell within 0.1 m counts
                    let p = relative(self.cfg.world.start, Pose2::new(mk.pos.0, mk.pos.1, 0.0));
                    let Some(c) = m.world_to_cell(p.x, p.y) else { continue };
                    let near_seen = (-2i64..=2).any(|dy| {
                        (-2i64..=2).any(|dx| {
                            m.cell_checked(c.x as i64 + dx, c.y as i64 + dy)
                                .is_some_and(|n| self.explorer.coverage().is_seen(m, n))
                        })
                    });
                    if near_seen {
                        self.m.markers[i].coverage_seen = Some(t);
                    }
                }
            }
        }
        if Self::due(&mut self.next_explore, t, self.cfg.explorer_period) {
            self.do_explore();
        }
        if Self::due(&mut self.next_nav, t, self.cfg.nav_period) {
            self.do_nav();
        }
        self.do_physics();
    }

    fn do_scan(&mut self) {
        self.scan = scan(&self.cfg.world, self.robot.pose, &self.cfg.lidar, &mut self.lidar_rng);
        self.scan_base = self.scan.points_base(&self.cfg.lidar);
        if self.slam.update(self.odom, &self.scan_base).is_some() {
            self.slam_dirty = true;
        }
        let est = self.estimate();
        let (s, c) = est.yaw.sin_cos();
        let pts = self.scan_base.iter().map(|&(x, y)| (est.x + c * x - s * y, est.y + s * x + c * y));
        self.nav.set_scan(pts, (est.x, est.y));
    }

    fn publish_map(&mut self) {
        if self.slam_dirty || self.map.is_none() {
            self.slam_dirty = false;
            self.map = Some(self.slam.occupancy());
        }
        self.map_stamp = self.t;
        if let Some(m) = self.map.as_ref() {
            self.nav.set_map(m);
        }
    }

    fn do_camera(&mut self) {
        let t = self.t;
        let pose = self.robot.pose;
        let cam = camera_pose(pose, &self.cfg.camera);
        let mut tentative = false;
        for (i, mk) in self.cfg.world.markers.iter().enumerate() {
            let rec = &mut self.m.markers[i];
            let d = (pose.x - mk.pos.0).hypot(pose.y - mk.pos.1);
            rec.closest = rec.closest.min(d);
            if d <= 2.0 && rec.first_within_2m.is_none() {
                rec.first_within_2m = Some(t);
            }
            let Ok(v) = view_marker(&self.cfg.world, pose, mk, &self.cfg.camera) else { continue };
            rec.readable_frames += 1;
            rec.max_side_px = rec.max_side_px.max(v.side_px);
            if rec.first_readable.is_none() {
                rec.first_readable = Some(t);
            }
            let new_view = self.last_view[i].is_none_or(|(p, yaw)| {
                (p.0 - cam.0).hypot(p.1 - cam.1) >= 0.05 || wrap(pose.yaw - yaw).abs() >= 0.05
            });
            if new_view {
                self.last_view[i] = Some((cam, pose.yaw));
                rec.views += 1;
                if rec.views >= self.cfg.slow.min_views && rec.views5_at.is_none() {
                    rec.views5_at = Some(t);
                }
            }
            if rec.views < self.cfg.slow.min_views {
                tentative = true;
            }
        }
        let all = |f: fn(&MarkerRecord) -> Option<f64>| {
            let v: Option<Vec<f64>> = self.m.markers.iter().map(f).collect();
            v.filter(|v| !v.is_empty()).map(|v| v.into_iter().fold(0.0, f64::max))
        };
        self.m.all_readable_at = all(|r| r.first_readable);
        self.m.all_within_2m_at = all(|r| r.first_within_2m);
        if self.cfg.vision.enabled {
            // like marker_mapper: slow down while a detected id has an unconfirmed estimate
            tentative = self.detect_markers();
        }
        if self.cfg.slow.enabled {
            if tentative {
                self.slow_until = t + self.cfg.slow.hold;
            }
            self.nav.speed_limit = (true, if t < self.slow_until { self.cfg.slow.percent } else { 0.0 });
        }
    }

    /// one camera frame of detections (markers and decoys) into the tracker, as marker_mapper does it;
    /// true if a detected id is tentative
    fn detect_markers(&mut self) -> bool {
        let t = self.t;
        let (truth, est) = (self.robot.pose, self.estimate());
        let n = self.cfg.world.markers.len();
        let mut detected = Vec::new();
        for (i, mk) in self.cfg.world.markers.iter().chain(&self.cfg.world.decoys).enumerate() {
            let Some((_, obs)) = detect(&self.cfg.world, truth, est, mk, &self.cfg.camera, &self.cfg.vision, &mut self.vision_rng)
            else {
                continue;
            };
            let rec = if i < n { &mut self.m.markers[i] } else { &mut self.m.decoys[i - n] };
            rec.detections += 1;
            self.tracker.observe(self.map.as_ref(), &obs, &self.cfg.locate);
            detected.push(obs.id);
        }
        if detected.is_empty() {
            return false;
        }
        let targets = self.tracker.targets();
        let origin = self.cfg.world.start;
        let mut notes = Vec::new();
        for (i, mk) in self.cfg.world.markers.iter().chain(&self.cfg.world.decoys).enumerate() {
            let Some(tg) = targets.iter().find(|tg| tg.id == mk.id as i64) else { continue };
            let rec = if i < n { &mut self.m.markers[i] } else { &mut self.m.decoys[i - n] };
            if rec.first_tentative.is_none() {
                rec.first_tentative = Some(t);
            }
            if tg.confirmed && rec.first_confirmed.is_none() {
                let truth = relative(origin, Pose2::new(mk.pos.0, mk.pos.1, 0.0));
                let err = (tg.x - truth.x).hypot(tg.y - truth.y);
                rec.first_confirmed = Some(t);
                rec.err_confirmed = Some(err);
                let kind = if i < n { "target" } else { "DECOY" };
                let [ray, pnp, scaled] = self.tracker.sources(tg.id);
                notes.push(format!(
                    "tracker: CONFIRMED {kind} id={} from {} views (ray {ray} pnp {pnp} scaled {scaled}, scale {:.2}), error {:.0} cm",
                    mk.id,
                    tg.views,
                    self.tracker.scale(tg.id).unwrap_or(f64::NAN),
                    100.0 * err
                ));
                if i >= n {
                    self.m.decoys_confirmed += 1;
                }
            }
        }
        for s in notes {
            self.note(s);
        }
        let all: Option<Vec<f64>> = self.m.markers.iter().map(|r| r.first_confirmed).collect();
        self.m.all_confirmed_at = all.filter(|v| !v.is_empty()).map(|v| v.into_iter().fold(0.0, f64::max));
        detected.iter().any(|id| targets.iter().any(|tg| tg.id == *id && !tg.confirmed))
    }

    /// what marker_mapper publishes on /team7/marker_approach: tentative targets with 2+ views, at
    /// their fused position, facing the side they were seen from
    pub fn marker_hints(&self) -> Vec<MarkerHint> {
        if !self.cfg.vision.enabled {
            return Vec::new();
        }
        self.tracker
            .targets()
            .into_iter()
            .filter(|t| !t.confirmed && t.views >= 2)
            .filter_map(|t| {
                let facing = self.tracker.facing(t.id, self.map.as_ref(), self.cfg.locate.occupied_threshold)?;
                Some(MarkerHint { id: Some(t.id), x: t.x, y: t.y, facing })
            })
            .collect()
    }

    fn do_explore(&mut self) {
        let t = self.t;
        let hints = self.marker_hints();
        let input = TickInput {
            costmap: self.map.as_ref(),
            costmap_stamp: self.map_stamp,
            map: self.map.as_ref(),
            robot: Ok(self.estimate()),
            result: self.nav_result.take(),
            done: false,
            markers: &hints,
        };
        let actions = self.explorer.tick(t, input);
        for a in actions {
            match a {
                Action::Info(m) => self.note(format!("explorer: {m}")),
                Action::Warn(m) => self.note(format!("explorer WARN: {m}")),
                Action::Cancel(_) => self.nav.cancel(),
                Action::SendGoal(r) => {
                    let yaw = (!r.xy_only).then_some(r.yaw);
                    if let Some(old) = self.nav.set_goal(r.x, r.y, yaw, t) {
                        self.nav_finished(old);
                    }
                    self.explorer.goal_accepted((r.x, r.y));
                }
            }
        }
    }

    fn nav_finished(&mut self, f: crate::nav::Finished) {
        let key = match f.status {
            NavStatus::Succeeded => "succeeded".to_owned(),
            NavStatus::Canceled => "canceled".to_owned(),
            NavStatus::Aborted => f.why.split(':').next_back().unwrap_or("").trim().to_owned(),
        };
        let key = if f.status == NavStatus::Aborted { format!("aborted {key}") } else { key };
        *self.m.nav_results.entry(key).or_default() += 1;
        self.note(format!("navigator: goal ({:.2}, {:.2}) {}", f.goal.0, f.goal.1, f.why));
        let status = match f.status {
            NavStatus::Succeeded => tb4_explore::NavStatus::Succeeded,
            NavStatus::Canceled => tb4_explore::NavStatus::Canceled,
            NavStatus::Aborted => tb4_explore::NavStatus::Aborted,
        };
        self.nav_result = Some((f.goal, status));
    }

    fn do_nav(&mut self) {
        let t = self.t;
        let est = self.estimate();
        let out = self.nav.tick(t, Some(est), self.robot.v, self.robot.w);
        if let Some((v, w)) = out.cmd {
            self.nav_cmd = (v, w);
            self.chain.command(v, w, t);
        }
        let goal = self.nav.goal().map(|g| (g.x, g.y));
        let key = crate::drive::path_key(self.nav.path(), goal);
        self.drive.on_nav(out.cmd.filter(|_| goal.is_some()), self.nav.state == NavState::Rotating, self.cfg.nav_period, self.nav.trajectory(), key);
        if let Some(g) = goal {
            let mp = &self.nav.p.mppi;
            let slow = crate::drive::SlowInput {
                v: self.robot.v,
                nav_cmd: self.nav_cmd,
                cap: mp.vx_max,
                rotating: self.nav.state == NavState::Rotating,
                limited: self.nav.speed_limit.1 > 0.0,
                monitor: self.chain.monitor_ratio,
                goal_dist: (g.0 - est.x).hypot(g.1 - est.y),
                clearance: self.nav.obstacles().map_or(f64::INFINITY, |df| df.at(est.x, est.y) as f64),
                obstacle_band: mp.robot_radius + mp.inflation,
            };
            self.drive.on_slow(self.cfg.nav_period, &slow);
        }
        if let Some(f) = out.finished {
            self.nav_finished(f);
        }
        self.cmd = self.chain.update(t, self.cfg.nav_period, &self.scan_base);
        let truth = self.truth_in_map();
        let err = (est.x - truth.x).hypot(est.y - truth.y);
        self.m.max_pose_error = self.m.max_pose_error.max(err);
    }

    /// why the robot is not moving while a goal is active
    fn stall_cause(&self) -> String {
        let t = self.t;
        if self.robot.reflex_active(t) || self.robot.in_contact(t) {
            return "bump / contact".into();
        }
        if self.nav.goal().is_some_and(|g| g.cancel) {
            return "cancelling".into();
        }
        match self.nav.state {
            NavState::NoMap | NavState::NoPath | NavState::AllColliding => return self.nav.state.name().into(),
            _ => {}
        }
        if self.chain.monitor_ratio < 0.3 {
            return "collision monitor".into();
        }
        if self.nav_cmd.0.abs() < STILL_V && self.nav_cmd.1.abs() < STILL_W {
            return format!("mppi zero output ({})", self.nav.state.name());
        }
        format!("other ({})", self.nav.state.name())
    }

    fn do_physics(&mut self) {
        let dt = self.cfg.physics_dt;
        let t = self.t;
        let before = self.robot.pose;
        let bumps_before = self.robot.bumps;
        if let Some(c) = self.robot.step(&self.cfg.world, self.cmd, t, dt) {
            if c.zone.is_none() && c.new_episode {
                self.m.rear_contacts += 1;
            }
            if c.new_episode {
                let zone = c.zone.map_or("rear", |z| z.frame_id());
                self.note(format!("collision at ({:.2}, {:.2}), {zone}", c.point.0, c.point.1));
            }
            if let Some(z) = c.zone.filter(|_| self.robot.bumps > bumps_before) {
                *self.m.bumps_by_zone.entry(z.frame_id().into()).or_default() += 1;
                self.bump_points.push(c.point);
            }
        }
        let after = self.robot.pose;
        // odometry from the wheel motion actually made
        if self.cfg.perfect_pose {
            self.odom = self.truth_in_map();
        } else {
            let o = &self.cfg.odom;
            let ds = (after.x - before.x).hypot(after.y - before.y) * self.robot.v.signum();
            let dyaw = wrap(after.yaw - before.yaw);
            let ds = ds * o.trans_scale + o.trans_noise * ds.abs().sqrt() * self.odom_rng.normal();
            let dyaw = dyaw * o.yaw_scale + o.yaw_bias * dt + o.yaw_noise * dyaw.abs().sqrt() * self.odom_rng.normal();
            let mid = self.odom.yaw + 0.5 * dyaw;
            self.odom = Pose2::new(self.odom.x + ds * mid.cos(), self.odom.y + ds * mid.sin(), wrap(self.odom.yaw + dyaw));
        }
        self.k += 1;
        self.t = self.k as f64 * dt;
        let t = self.t;

        // bookkeeping
        let (v, w) = (self.robot.v, self.robot.w);
        self.m.distance += (after.x - before.x).hypot(after.y - before.y);
        let motion = classify(v, w);
        self.drive.on_physics(dt, v, motion, (after.x - before.x).hypot(after.y - before.y), self.nav.p.mppi.vx_max);
        match motion {
            Motion::Driving => self.m.time_driving += dt,
            Motion::Turning => self.m.time_turning += dt,
            Motion::Still => self.m.time_still += dt,
        }
        let goal = self.nav.goal().map(|g| (g.x, g.y));
        if goal.is_none() {
            self.m.time_no_goal += dt;
        }
        let stalled = goal.is_some() && motion == Motion::Still;
        if stalled {
            self.m.time_still_with_goal += dt;
        }
        let cause = if stalled { self.stall_cause() } else { String::new() };
        self.stalls.update(t, dt, stalled, &cause, goal);
        while self.area_next < AREA_TIMES.len() && t + 1e-9 >= AREA_TIMES[self.area_next] {
            let a = self.map.as_ref().map_or(0.0, |g| free_area(g, 50));
            self.m.area.push((AREA_TIMES[self.area_next], a));
            self.area_next += 1;
        }
    }

    /// metrics so far (closes an open stall episode in the copy)
    pub fn metrics(&self) -> Metrics {
        let mut m = self.m.clone();
        let mut stalls = self.stalls.clone();
        stalls.close(self.t);
        m.stalls = stalls.episodes;
        m.duration = self.t;
        m.drive = self.drive.summary();
        m.wall_time = self.wall.elapsed().as_secs_f64();
        m.final_area = self.map.as_ref().map_or(0.0, |g| free_area(g, 50));
        m.explorer = self.explorer.stats().clone();
        m.nav_plans = self.nav.plans;
        m.nav_plan_failures = self.nav.plan_failures;
        m.nav_route_kept = self.nav.route_kept;
        m.collisions = self.robot.collisions;
        m.bumps = self.robot.bumps;
        let s = self.slam.stats();
        m.slam_keyframes = s.keyframes;
        m.slam_loop_closures = s.loop_closures;
        m.slam_fallbacks = s.match_fallbacks;
        let (est, truth) = (self.estimate(), self.truth_in_map());
        m.final_pose_error = ((est.x - truth.x).hypot(est.y - truth.y), wrap(est.yaw - truth.yaw).abs());
        let origin = self.cfg.world.start;
        let world = &self.cfg.world;
        for (mk, rec) in world.markers.iter().zip(&mut m.markers).chain(world.decoys.iter().zip(&mut m.decoys)) {
            if let Some(tg) = self.tracker.target(mk.id as i64) {
                let truth = relative(origin, Pose2::new(mk.pos.0, mk.pos.1, 0.0));
                rec.err_end = Some((tg.x - truth.x).hypot(tg.y - truth.y));
                rec.confirmed_end = tg.confirmed;
                rec.tracker_views = tg.views;
            }
        }
        m
    }
}

/// `b` expressed in the frame of `a`
pub fn relative(a: Pose2, b: Pose2) -> Pose2 {
    let (s, c) = a.yaw.sin_cos();
    let (dx, dy) = (b.x - a.x, b.y - a.y);
    Pose2::new(c * dx + s * dy, -s * dx + c * dy, wrap(b.yaw - a.yaw))
}

pub fn compose(a: Pose2, b: Pose2) -> Pose2 {
    let (s, c) = a.yaw.sin_cos();
    Pose2::new(a.x + c * b.x - s * b.y, a.y + s * b.x + c * b.y, wrap(a.yaw + b.yaw))
}

/// floor area (m^2) reachable from the start without crossing a solid obstacle, on a `res` grid
pub fn reference_area(w: &World, res: f64) -> f64 {
    let [x0, y0, x1, y1] = w.bounds;
    let (nx, ny) = (((x1 - x0) / res).ceil() as i64, ((y1 - y0) / res).ceil() as i64);
    let centre = |i: i64, j: i64| (x0 + (i as f64 + 0.5) * res, y0 + (j as f64 + 0.5) * res);
    let start = (((w.start.x - x0) / res).floor() as i64, ((w.start.y - y0) / res).floor() as i64);
    let mut seen: BTreeMap<(i64, i64), ()> = BTreeMap::new();
    let mut stack = vec![start];
    let mut n = 0u64;
    while let Some((i, j)) = stack.pop() {
        if i < 0 || j < 0 || i >= nx || j >= ny || seen.insert((i, j), ()).is_some() {
            continue;
        }
        // a bit more than half a cell, so a thin glass or wall line between two cell centres blocks
        if !w.disc_free(centre(i, j), 0.75 * res) {
            continue;
        }
        n += 1;
        stack.extend([(i + 1, j), (i - 1, j), (i, j + 1), (i, j - 1)]);
    }
    n as f64 * res * res
}
