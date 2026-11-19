//! What the viewer knows about the run, all in the map frame. Filled by the ROS thread (`ros`),
//! read by the window (`app`, `gui`). No ROS types here, so it is tested and drawn without a ROS graph.

use std::collections::VecDeque;
use std::time::{Duration, Instant};

use tb4_core::GridMeta;

use crate::Rgba;

/// TurtleBot 4 footprint radius, metres
pub const ROBOT_RADIUS: f64 = 0.171;
/// mission time limit, the countdown starts at the first RUNNING
pub const MISSION_LIMIT: Duration = Duration::from_secs(600);
/// trail points are at least this far apart, and at most this many are kept
const TRAIL_SPACING: f64 = 0.05;
const TRAIL_MAX: usize = 20_000;

/// message rate over a sliding window
#[derive(Debug, Clone, Default)]
pub struct RateMeter {
    stamps: VecDeque<Instant>,
    pub total: u64,
    last: Option<Instant>,
}

const RATE_WINDOW: Duration = Duration::from_secs(5);

impl RateMeter {
    pub fn tick(&mut self, now: Instant) {
        self.total += 1;
        self.last = Some(now);
        self.stamps.push_back(now);
        while self.stamps.front().is_some_and(|&t| now.saturating_duration_since(t) > RATE_WINDOW) {
            self.stamps.pop_front();
        }
    }

    /// when the last message came
    pub fn last(&self) -> Option<Instant> {
        self.last
    }

    /// messages per second over the last few seconds; 0 when the topic went quiet
    pub fn hz(&self, now: Instant) -> f64 {
        let recent: Vec<_> = self.stamps.iter().filter(|&&t| now.saturating_duration_since(t) <= RATE_WINDOW).collect();
        match (recent.first(), recent.last()) {
            (Some(&&a), Some(&&b)) if recent.len() >= 2 && b > a => {
                let n = (recent.len() - 1) as f64;
                let span = (b - a).as_secs_f64();
                // a topic that stopped (silent for over two periods) decays instead of freezing at its old rate
                let since = now.saturating_duration_since(b).as_secs_f64();
                if since > 2.0 * span / n { n / (span + since) } else { n / span }
            }
            _ => 0.0,
        }
    }
}

/// topics whose rate the health panel watches
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Topic {
    Scan,
    Odom,
    Tf,
    Map,
    Camera,
}

impl Topic {
    pub const ALL: [Topic; 5] = [Topic::Scan, Topic::Odom, Topic::Tf, Topic::Map, Topic::Camera];

    pub fn name(self) -> &'static str {
        match self {
            Topic::Scan => "/scan",
            Topic::Odom => "/odom",
            Topic::Tf => "/tf",
            Topic::Map => "/map",
            Topic::Camera => "/team7/aruco/debug",
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Target {
    pub id: i32,
    pub x: f64,
    pub y: f64,
    pub confirmed: bool,
    /// views so far, from the tentative label ("ID 3? 2v")
    pub views: Option<u32>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct MapData {
    pub meta: GridMeta,
    pub data: Vec<i8>,
    /// yaw of `info.origin` (cell (0, 0) corner); 0 for slam_toolbox and our SLAM
    pub origin_yaw: f64,
}

/// a grid and a counter bumped on every new message, so the window rebuilds its image only then
#[derive(Debug, Clone, Default)]
pub struct GridLayer {
    pub grid: Option<MapData>,
    pub seq: u64,
}

impl GridLayer {
    pub fn set(&mut self, g: MapData) {
        self.grid = Some(g);
        self.seq += 1;
    }
}

/// how a marker's points are joined
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ShapeKind {
    /// separate points (POINTS, SPHERE_LIST, CUBE_LIST, one SPHERE or CUBE)
    Points,
    /// one polyline (LINE_STRIP)
    Strip,
    /// pairs of points (LINE_LIST)
    Segments,
}

/// a marker reduced to what a top-down view draws
#[derive(Debug, Clone, PartialEq)]
pub struct Shape {
    pub kind: ShapeKind,
    pub pts: Vec<(f64, f64)>,
    pub color: Rgba,
}

/// the latest camera frame, RGBA8
#[derive(Debug, Clone)]
pub struct CameraFrame {
    pub width: u32,
    pub height: u32,
    pub rgba: Vec<u8>,
    pub seq: u64,
    pub at: Instant,
}

/// everything drawn, in the map frame
#[derive(Debug, Clone, Default)]
pub struct Model {
    pub map: GridLayer,
    pub global_cost: GridLayer,
    pub local_cost: GridLayer,
    /// map -> base_link (x, y, yaw)
    pub robot: Option<(f64, f64, f64)>,
    pub trail: Vec<(f64, f64)>,
    pub scan: Vec<(f64, f64)>,
    pub path: Vec<(f64, f64)>,
    pub trajectory: Vec<(f64, f64)>,
    pub frontiers: Vec<(f64, f64)>,
    pub views: Vec<(f64, f64)>,
    pub goal: Option<(f64, f64)>,
    pub targets: Vec<Target>,
    pub slam_graph: Vec<Shape>,
    pub camera: Option<CameraFrame>,
    pub mission: Option<String>,
    /// first RUNNING, and when the state left RUNNING after that
    pub mission_started: Option<Instant>,
    pub mission_ended: Option<Instant>,
    /// last /clock (seconds) and when it came
    pub clock: Option<(f64, Instant)>,
    /// ROS time of the newest transform on the map -> base_link chain that is the oldest
    /// (the chain is only as fresh as its slowest edge), seconds
    pub tf_stamp: Option<f64>,
    /// ROS now minus the stamp of the last scan, seconds (smoothed)
    pub scan_delay: Option<f64>,
    pub rates: [RateMeter; 5],
    /// messages received per subscription, for the debug line
    pub counts: Vec<(&'static str, u64)>,
    /// the last frame that could not be put in the map frame (no TF yet), and when
    pub tf_problem: Option<(String, Instant)>,
    /// what happened to the last goal sent with the 2D Goal tool
    pub goal_status: Option<String>,
}

impl Model {
    /// time went back (a replay restarted): forget the poses of the old time line, keep the rest
    pub fn reset_run(&mut self) {
        self.robot = None;
        self.trail.clear();
        self.scan.clear();
        self.path.clear();
        self.trajectory.clear();
        self.goal = None;
        self.tf_stamp = None;
        self.scan_delay = None;
    }

    pub fn rate(&self, t: Topic) -> &RateMeter {
        &self.rates[t as usize]
    }

    pub fn tick(&mut self, t: Topic, now: Instant) {
        self.rates[t as usize].tick(now);
    }

    pub fn count(&mut self, name: &'static str) {
        match self.counts.iter_mut().find(|(n, _)| *n == name) {
            Some((_, c)) => *c += 1,
            None => self.counts.push((name, 1)),
        }
    }

    pub fn counts_line(&self) -> String {
        self.counts.iter().map(|(n, c)| format!("{n}={c}")).collect::<Vec<_>>().join(" ")
    }

    /// a new robot pose: also grows the trail
    pub fn set_robot(&mut self, x: f64, y: f64, yaw: f64) {
        self.robot = Some((x, y, yaw));
        if self.trail.last().is_none_or(|&(px, py)| (px - x).hypot(py - y) >= TRAIL_SPACING) {
            if self.trail.len() >= TRAIL_MAX {
                self.trail.drain(..TRAIL_MAX / 10);
            }
            self.trail.push((x, y));
        }
    }

    /// a mission state message: the countdown starts at the first RUNNING
    pub fn set_mission(&mut self, state: String, now: Instant) {
        if state == "RUNNING" {
            self.mission_started.get_or_insert(now);
            self.mission_ended = None;
        } else if self.mission_started.is_some() && self.mission_ended.is_none() {
            self.mission_ended = Some(now);
        }
        self.mission = Some(state);
    }

    /// ROS time now in seconds and whether it is sim time (/clock seen in the last 2 s);
    /// `wall` is the system time in seconds since the epoch
    pub fn ros_now(&self, now: Instant, wall: f64) -> (f64, bool) {
        match self.clock {
            Some((t, at)) if now.saturating_duration_since(at) < Duration::from_secs(2) => (t + now.saturating_duration_since(at).as_secs_f64(), true),
            _ => (wall, false),
        }
    }

    /// world box to fit the view to: the map, else around the robot, else 10 x 10 m at the origin
    pub fn bounds(&self) -> [f64; 4] {
        if let Some(m) = &self.map.grid {
            let g = &m.meta;
            return [g.origin_x, g.origin_y, g.origin_x + f64::from(g.width) * g.resolution, g.origin_y + f64::from(g.height) * g.resolution];
        }
        let (x, y) = self.robot.map_or((0.0, 0.0), |r| (r.0, r.1));
        [x - 5.0, y - 5.0, x + 5.0, y + 5.0]
    }

    pub fn targets(&self, confirmed: bool) -> impl Iterator<Item = &Target> {
        self.targets.iter().filter(move |t| t.confirmed == confirmed)
    }
}

/// "mm:ss" left of the 10 minutes, "-mm:ss" past them; frozen once the mission ended
pub fn countdown(started: Option<Instant>, ended: Option<Instant>, now: Instant) -> String {
    let Some(s) = started else { return "10:00".into() };
    let used = ended.unwrap_or(now).saturating_duration_since(s);
    let (sign, left) = match MISSION_LIMIT.checked_sub(used) {
        Some(l) => ("", l.as_secs()),
        None => ("-", (used - MISSION_LIMIT).as_secs()),
    };
    format!("{sign}{:02}:{:02}", left / 60, left % 60)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rate_meter_measures_and_decays() {
        let t0 = Instant::now();
        let mut r = RateMeter::default();
        assert_eq!(r.hz(t0), 0.0);
        for i in 0..=20 {
            r.tick(t0 + Duration::from_millis(100 * i));
        }
        let end = t0 + Duration::from_secs(2);
        assert!((r.hz(end) - 10.0).abs() < 0.01, "{}", r.hz(end));
        assert!(r.hz(end + Duration::from_secs(2)) < 6.0);
        assert_eq!(r.hz(end + Duration::from_secs(10)), 0.0);
        assert_eq!(r.total, 21);
        assert_eq!(r.last(), Some(end));
    }

    #[test]
    fn bounds_follow_map_then_robot() {
        let mut m = Model::default();
        assert_eq!(m.bounds(), [-5.0, -5.0, 5.0, 5.0]);
        m.robot = Some((1.0, 2.0, 0.0));
        assert_eq!(m.bounds(), [-4.0, -3.0, 6.0, 7.0]);
        m.map.set(MapData {
            meta: GridMeta { width: 20, height: 10, resolution: 0.05, origin_x: -0.5, origin_y: 0.0 },
            data: vec![-1; 200],
            origin_yaw: 0.0,
        });
        assert_eq!(m.bounds(), [-0.5, 0.0, 0.5, 0.5]);
        assert_eq!(m.map.seq, 1);
    }

    #[test]
    fn counts_accumulate_per_name() {
        let mut m = Model::default();
        m.count("scan");
        m.count("map");
        m.count("scan");
        assert_eq!(m.counts_line(), "scan=2 map=1");
    }

    #[test]
    fn countdown_starts_at_running_and_freezes_at_the_end() {
        let t0 = Instant::now();
        let mut m = Model::default();
        assert_eq!(countdown(m.mission_started, m.mission_ended, t0), "10:00");
        m.set_mission("RUNNING".into(), t0);
        m.set_mission("RUNNING".into(), t0 + Duration::from_secs(30));
        let at = t0 + Duration::from_secs(83);
        assert_eq!(countdown(m.mission_started, m.mission_ended, at), "08:37");
        assert_eq!(countdown(m.mission_started, m.mission_ended, t0 + Duration::from_secs(612)), "-00:12");
        m.set_mission("DONE".into(), at);
        assert_eq!(countdown(m.mission_started, m.mission_ended, t0 + Duration::from_secs(900)), "08:37");
    }

    #[test]
    fn trail_keeps_spacing_and_ros_now_prefers_a_fresh_clock() {
        let mut m = Model::default();
        m.set_robot(0.0, 0.0, 0.0);
        m.set_robot(0.01, 0.0, 0.0);
        m.set_robot(0.1, 0.0, 0.0);
        assert_eq!(m.trail, vec![(0.0, 0.0), (0.1, 0.0)]);
        let t0 = Instant::now();
        assert_eq!(m.ros_now(t0, 5.0), (5.0, false));
        m.clock = Some((100.0, t0));
        let (t, sim) = m.ros_now(t0 + Duration::from_millis(500), 5.0);
        assert!(sim && (t - 100.5).abs() < 1e-6);
        assert_eq!(m.ros_now(t0 + Duration::from_secs(3), 5.0), (5.0, false));
    }
}
