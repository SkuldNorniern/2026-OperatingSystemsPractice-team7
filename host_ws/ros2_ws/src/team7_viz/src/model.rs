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
/// no scan, odom or tf for this long: the replay ended or the robot is off
pub const STALE_AFTER: Duration = Duration::from_secs(3);
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

/// a kind of data that can come from the team7 stack or the stock stack (Nav2, slam_toolbox,
/// explore_lite, aruco_opencv); its source is picked from the topics that exist
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Slot {
    Map,
    GlobalCost,
    LocalCost,
    Plan,
    Trajectory,
    Frontiers,
    Views,
    Goal,
    Targets,
    SlamGraph,
    Camera,
    /// the NavigateToPose action server for the 2D Goal tool
    NavAction,
}

impl Slot {
    /// candidate topics, team7 first
    pub fn candidates(self) -> &'static [&'static str] {
        match self {
            Slot::Map => &["/map"],
            Slot::GlobalCost => &["/global_costmap/costmap"],
            Slot::LocalCost => &["/team7/mppi/local_cost", "/local_costmap/costmap"],
            Slot::Plan => &["/team7/path", "/plan"],
            Slot::Trajectory => &["/team7/mppi/trajectory", "/local_plan"],
            Slot::Frontiers => &["/team7/frontiers", "/explore/frontiers"],
            Slot::Views => &["/team7/frontiers"],
            Slot::Goal => &["/team7/goal", "/goal_pose"],
            Slot::Targets => &["/team7/targets"],
            Slot::SlamGraph => &["/team7/slam/graph", "/slam_toolbox/graph_visualization"],
            Slot::Camera => &["/team7/aruco/debug", "/aruco_tracker/debug"],
            Slot::NavAction => &["/team7/navigate_to_pose", "/navigate_to_pose"],
        }
    }

    /// the graph name that shows a candidate exists (actions show up as their status topic)
    fn probe(self, topic: &str) -> String {
        if self == Slot::NavAction { format!("{topic}/_action/status") } else { topic.to_owned() }
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
    /// topic names in the ROS graph, polled every few seconds
    pub graph: Vec<String>,
    /// last message per topic
    pub seen: Vec<(&'static str, Instant)>,
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
        // a new run on a new time line: its mission and targets start over
        self.targets.clear();
        self.mission_started = None;
        self.mission_ended = None;
    }

    /// the topic a slot takes its data from: the first candidate in the graph, else the first
    /// one that sent something; None when no candidate exists (the layer is hidden)
    pub fn bound(&self, slot: Slot) -> Option<&'static str> {
        let c = slot.candidates();
        c.iter()
            .find(|t| self.graph.iter().any(|g| *g == slot.probe(t)))
            .or_else(|| c.iter().find(|t| self.seen.iter().any(|(s, _)| s == *t)))
            .copied()
    }

    /// a message on `topic` for `slot`: noted, and true when it is the slot's source
    pub fn accept(&mut self, slot: Slot, topic: &'static str, now: Instant) -> bool {
        match self.seen.iter_mut().find(|(s, _)| *s == topic) {
            Some(e) => e.1 = now,
            None => self.seen.push((topic, now)),
        }
        self.bound(slot) == Some(topic)
    }

    /// which stack the sources look like: "team7", "stock", "mixed" or "-"
    pub fn stack(&self) -> &'static str {
        let slots = [Slot::Plan, Slot::LocalCost, Slot::Frontiers];
        let team7 = slots.iter().filter(|&&s| self.bound(s).is_some_and(|t| t.starts_with("/team7/"))).count();
        let stock = slots.iter().filter(|&&s| self.bound(s).is_some_and(|t| !t.starts_with("/team7/"))).count();
        match (team7, stock) {
            (0, 0) => "-",
            (_, 0) => "team7",
            (0, _) => "stock",
            _ => "mixed",
        }
    }

    /// when the robot data (scan, odom, tf) last came
    pub fn last_data(&self) -> Option<Instant> {
        [Topic::Scan, Topic::Odom, Topic::Tf].iter().filter_map(|&t| self.rate(t).last()).max()
    }

    /// how long no robot data has come, when that is over `STALE_AFTER`
    pub fn stale_for(&self, now: Instant) -> Option<Duration> {
        let d = now.saturating_duration_since(self.last_data()?);
        (d > STALE_AFTER).then_some(d)
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

    /// a mission state message: the countdown starts at RUNNING, and starts again at a RUNNING
    /// after the last run ended (DONE, or its data stopped)
    pub fn set_mission(&mut self, state: String, now: Instant) {
        if state == "RUNNING" {
            if self.mission_ended.is_some() || self.stale_for(now).is_some() {
                self.mission_started = None;
            }
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

fn mmss(d: Duration) -> String {
    let s = d.as_secs();
    format!("{:02}:{:02}", s / 60, s % 60)
}

/// the mission clock: (big text, small text). Counts down from 10:00 while running; once the run
/// is DONE, or its data stopped (replay over), it shows the time used. Never negative.
pub fn mission_clock(m: &Model, now: Instant) -> (String, String) {
    let Some(start) = m.mission_started else { return ("--:--".into(), "not started".into()) };
    let (end, why) = match (m.mission_ended, m.stale_for(now)) {
        (Some(e), _) => (Some(e), format!("done ({})", m.mission.as_deref().unwrap_or("ended"))),
        (None, Some(_)) => (m.last_data(), "stopped: no data".to_owned()),
        (None, None) => (None, String::new()),
    };
    if end.is_some_and(|e| e < start) {
        // the RUNNING came after the data had already stopped (a latched message from an old run)
        return ("--:--".into(), why);
    }
    let used = end.unwrap_or(now).saturating_duration_since(start);
    if end.is_some() {
        return (mmss(used), format!("{why}, time used"));
    }
    match MISSION_LIMIT.checked_sub(used) {
        Some(left) => (mmss(left), "left of 10:00".into()),
        None => ("00:00".into(), format!("time up, +{}", mmss(used - MISSION_LIMIT))),
    }
}

/// a short human age: "0.4 s", "12 s", "3 min", "2 h"
pub fn human_age(d: Duration) -> String {
    let s = d.as_secs_f64();
    if s < 10.0 {
        format!("{s:.1} s")
    } else if s < 60.0 {
        format!("{s:.0} s")
    } else if s < 3600.0 {
        format!("{:.0} min", (s / 60.0).floor())
    } else {
        format!("{:.0} h", (s / 3600.0).floor())
    }
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

    /// robot data every 100 ms from `from` for `secs`
    fn feed(m: &mut Model, from: Instant, secs: u64) {
        for i in 0..=secs * 10 {
            m.tick(Topic::Scan, from + Duration::from_millis(100 * i));
        }
    }

    #[test]
    fn mission_clock_counts_down_stops_and_never_goes_negative() {
        let t0 = Instant::now();
        let s = Duration::from_secs;
        let mut m = Model::default();
        assert_eq!(mission_clock(&m, t0), ("--:--".into(), "not started".into()));
        m.set_mission("RUNNING".into(), t0);
        feed(&mut m, t0, 700);
        m.set_mission("RUNNING".into(), t0 + s(30));
        assert_eq!(mission_clock(&m, t0 + s(83)).0, "08:37");
        let (big, small) = mission_clock(&m, t0 + s(612));
        assert_eq!((big.as_str(), small.as_str()), ("00:00", "time up, +00:12"));
        // the replay ended at 700 s: the clock stops at the last data, 5 s later and 20 min later
        let (big, small) = mission_clock(&m, t0 + s(705));
        assert_eq!((big.as_str(), small.as_str()), ("11:40", "stopped: no data, time used"));
        assert_eq!(mission_clock(&m, t0 + s(1900)).0, "11:40");
        // DONE freezes the time used
        let mut m = Model::default();
        m.set_mission("RUNNING".into(), t0);
        feed(&mut m, t0, 100);
        m.set_mission("DONE".into(), t0 + s(83));
        assert_eq!(mission_clock(&m, t0 + s(90)), ("01:23".into(), "done (DONE), time used".into()));
        // a new RUNNING after the end starts over
        m.set_mission("RUNNING".into(), t0 + s(95));
        assert_eq!(mission_clock(&m, t0 + s(95)).0, "10:00");
    }

    #[test]
    fn slots_bind_to_what_is_published() {
        let t0 = Instant::now();
        let mut m = Model::default();
        assert_eq!(m.bound(Slot::Plan), None);
        // stock graph: Nav2 /plan, explore_lite frontiers, the Nav2 action
        m.graph = ["/plan", "/explore/frontiers", "/local_costmap/costmap", "/navigate_to_pose/_action/status"].map(String::from).to_vec();
        assert_eq!(m.bound(Slot::Plan), Some("/plan"));
        assert_eq!(m.bound(Slot::Frontiers), Some("/explore/frontiers"));
        assert_eq!(m.bound(Slot::NavAction), Some("/navigate_to_pose"));
        assert_eq!(m.bound(Slot::Views), None);
        assert_eq!(m.stack(), "stock");
        assert!(m.accept(Slot::Plan, "/plan", t0));
        assert!(!m.accept(Slot::Plan, "/team7/path", t0));
        // team7 topics win once they are in the graph
        m.graph.push("/team7/path".into());
        assert!(m.accept(Slot::Plan, "/team7/path", t0));
        assert_eq!(m.stack(), "mixed");
        // before the first graph poll, data decides
        let mut m = Model::default();
        assert!(m.accept(Slot::Camera, "/aruco_tracker/debug", t0));
        assert_eq!(m.bound(Slot::Camera), Some("/aruco_tracker/debug"));
    }

    #[test]
    fn stale_after_three_seconds_and_human_ages() {
        let t0 = Instant::now();
        let mut m = Model::default();
        assert!(m.stale_for(t0).is_none());
        feed(&mut m, t0, 1);
        assert!(m.stale_for(t0 + Duration::from_secs(3)).is_none());
        assert_eq!(m.stale_for(t0 + Duration::from_secs(10)), Some(Duration::from_secs(9)));
        assert_eq!(human_age(Duration::from_millis(400)), "0.4 s");
        assert_eq!(human_age(Duration::from_secs(82)), "1 min");
        assert_eq!(human_age(Duration::from_secs(42)), "42 s");
        assert_eq!(human_age(Duration::from_secs(82125)), "22 h");
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
