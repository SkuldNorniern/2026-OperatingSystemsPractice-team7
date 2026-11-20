//! What the viewer knows about the run, all in the map frame. Filled by the ROS thread (`ros`),
//! read by the window (`app`, `gui`). No ROS types here, so it is tested and drawn without a ROS graph.

use std::collections::{BTreeMap, HashMap, VecDeque};
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
    Scan,
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
    /// the ArUco detector's raw hits (the current camera frame)
    Detections,
    /// the NavigateToPose action server for the 2D Goal tool
    NavAction,
}

impl Slot {
    /// candidate topics, team7 first
    pub fn candidates(self) -> &'static [&'static str] {
        match self {
            Slot::Map => &["/map"],
            Slot::Scan => &["/scan"],
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
            Slot::Detections => &["/aruco_detections"],
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

#[derive(Debug, Clone, PartialEq)]
pub struct Target {
    pub id: i32,
    pub x: f64,
    pub y: f64,
    pub confirmed: bool,
    /// views so far, from the tentative label ("ID 3? 2v")
    pub views: Option<u32>,
    /// rules a candidate still fails, from its label after " | " ("wall 1/3", "angle 4/11 deg")
    pub reasons: Vec<String>,
}

impl Target {
    /// one rule left: about to confirm
    pub fn close(&self) -> bool {
        !self.confirmed && self.reasons.len() == 1
    }
}

/// what the viewer saw of a target id over the run: the most views it had as a candidate, and
/// when it was first seen confirmed
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Seen {
    pub views: Option<u32>,
    pub confirmed_at: Option<Instant>,
}

/// one detection from the camera, in the map frame: the camera position and where the marker is
/// estimated, for a short bearing tick
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Ray {
    pub id: i32,
    pub from: (f64, f64),
    pub to: (f64, f64),
    pub at: Instant,
}

/// a detection tick is drawn this long, fading
pub const RAY_FOR: Duration = Duration::from_millis(1500);

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

/// the message types a layer can show, and so what a topic is subscribed as
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum MsgKind {
    Grid,
    Scan,
    Path,
    Markers,
    Pose,
    Image,
    Detections,
}

impl MsgKind {
    /// the ROS type name, as the graph lists it
    pub fn ros_type(self) -> &'static str {
        match self {
            MsgKind::Grid => "nav_msgs/msg/OccupancyGrid",
            MsgKind::Scan => "sensor_msgs/msg/LaserScan",
            MsgKind::Path => "nav_msgs/msg/Path",
            MsgKind::Markers => "visualization_msgs/msg/MarkerArray",
            MsgKind::Pose => "geometry_msgs/msg/PoseStamped",
            MsgKind::Image => "sensor_msgs/msg/Image",
            MsgKind::Detections => "aruco_opencv_msgs/msg/ArucoDetection",
        }
    }
}

/// one topic in the ROS graph
#[derive(Debug, Clone, PartialEq, Default)]
pub struct GraphTopic {
    pub name: String,
    pub types: Vec<String>,
    pub publishers: usize,
}

/// a marker in the map frame, with what the layers need from it
#[derive(Debug, Clone, PartialEq)]
pub struct Mk {
    pub ns: String,
    pub id: i32,
    pub kind: ShapeKind,
    pub pts: Vec<(f64, f64)>,
    /// the marker's own position (a sphere's centre)
    pub pos: (f64, f64),
    pub color: Rgba,
    pub text: String,
}

/// the latest data of one topic, in the map frame
#[derive(Debug, Clone)]
pub enum Feed {
    Grid(GridLayer),
    /// scan points or a path
    Points(Vec<(f64, f64)>),
    Markers(Vec<Mk>),
    Pose(Option<(f64, f64)>),
    Image(CameraFrame),
    /// detection ticks of the last `RAY_FOR`
    Rays(Vec<Ray>),
}

/// everything drawn, in the map frame. Data is kept per topic, so a layer can switch topics
/// without losing what the other one had.
#[derive(Debug, Clone, Default)]
pub struct Model {
    pub feeds: HashMap<String, Feed>,
    /// message rate per topic
    pub topic_rates: HashMap<String, RateMeter>,
    /// map -> base_link (x, y, yaw)
    pub robot: Option<(f64, f64, f64)>,
    pub trail: Vec<(f64, f64)>,
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
    pub counts: Vec<(String, u64)>,
    /// the last frame that could not be put in the map frame (no TF yet), and when
    pub tf_problem: Option<(String, Instant)>,
    /// what happened to the last goal sent with the 2D Goal tool
    pub goal_status: Option<String>,
    /// topics in the ROS graph with their types and publisher counts, polled every few seconds
    pub graph: Vec<GraphTopic>,
    /// what the window wants subscribed: set by the window, read by the ROS thread
    pub wanted: Vec<(String, MsgKind)>,
    /// the camera image topic in use (the health panel rates it)
    pub camera_topic: Option<String>,
    /// the camera topic's last frames, oldest first, so a burst is still shown frame by frame
    pub camera_queue: VecDeque<CameraFrame>,
    /// per target id: most views as a candidate, when it was confirmed (`note_targets`)
    pub target_log: BTreeMap<i32, Seen>,
}

impl Model {
    /// time went back (a replay restarted): forget the poses of the old time line, keep the rest
    pub fn reset_run(&mut self) {
        self.robot = None;
        self.trail.clear();
        for f in self.feeds.values_mut() {
            match f {
                Feed::Points(p) => p.clear(),
                Feed::Pose(p) => *p = None,
                _ => {}
            }
        }
        self.tf_stamp = None;
        self.scan_delay = None;
        // the mission clock and the targets stay: the mission state is latched and sent once, so
        // clearing it here would lose it; a new run's RUNNING restarts the clock (`set_mission`) and
        // its target array replaces the targets
    }

    /// new data on `topic`
    pub fn put(&mut self, topic: &str, feed: Feed, now: Instant) {
        self.feeds.insert(topic.to_owned(), feed);
        self.topic_rates.entry(topic.to_owned()).or_default().tick(now);
    }

    /// change the data of `topic` in place (markers accumulate, grids count their updates)
    pub fn feed_mut(&mut self, topic: &str, now: Instant, empty: impl FnOnce() -> Feed) -> &mut Feed {
        self.topic_rates.entry(topic.to_owned()).or_default().tick(now);
        self.feeds.entry(topic.to_owned()).or_insert_with(empty)
    }

    pub fn grid(&self, topic: &str) -> Option<&GridLayer> {
        match self.feeds.get(topic) {
            Some(Feed::Grid(g)) => Some(g),
            _ => None,
        }
    }

    pub fn points(&self, topic: &str) -> &[(f64, f64)] {
        match self.feeds.get(topic) {
            Some(Feed::Points(p)) => p,
            _ => &[],
        }
    }

    pub fn markers(&self, topic: &str) -> &[Mk] {
        match self.feeds.get(topic) {
            Some(Feed::Markers(m)) => m,
            _ => &[],
        }
    }

    pub fn pose(&self, topic: &str) -> Option<(f64, f64)> {
        match self.feeds.get(topic) {
            Some(Feed::Pose(p)) => *p,
            _ => None,
        }
    }

    pub fn rays(&self, topic: &str) -> &[Ray] {
        match self.feeds.get(topic) {
            Some(Feed::Rays(r)) => r,
            _ => &[],
        }
    }

    /// add the detections of one camera frame, dropping ticks older than `RAY_FOR`
    pub fn add_rays(&mut self, topic: &str, rays: Vec<Ray>, now: Instant) {
        if let Feed::Rays(v) = self.feed_mut(topic, now, || Feed::Rays(Vec::new())) {
            v.retain(|r| now.saturating_duration_since(r.at) < RAY_FOR);
            v.extend(rays);
        }
    }

    /// remember the targets as they are now: a candidate's views, and the moment an id is first
    /// seen confirmed. Ids no longer published are forgotten (a replay restarted, a target lost).
    pub fn note_targets(&mut self, targets: &[Target], now: Instant) {
        self.target_log.retain(|id, _| targets.iter().any(|t| t.id == *id));
        for t in targets {
            let s = self.target_log.entry(t.id).or_insert(Seen { views: None, confirmed_at: None });
            if t.confirmed {
                s.confirmed_at.get_or_insert(now);
            } else {
                s.confirmed_at = None;
                if t.views.is_some() {
                    s.views = s.views.max(t.views);
                }
            }
        }
    }

    pub fn image(&self, topic: &str) -> Option<&CameraFrame> {
        match self.feeds.get(topic) {
            Some(Feed::Image(f)) => Some(f),
            _ => None,
        }
    }

    /// the camera frame of the topic in use
    pub fn camera(&self) -> Option<&CameraFrame> {
        self.image(self.camera_topic.as_deref()?)
    }

    /// keep a camera frame for showing; the queue holds the last few, older ones drop
    pub fn queue_camera(&mut self, f: CameraFrame) {
        const KEEP: usize = 4;
        if self.camera_queue.back().is_some_and(|b| f.seq <= b.seq) {
            // a new topic or a restart: start over
            self.camera_queue.clear();
        }
        self.camera_queue.push_back(f);
        while self.camera_queue.len() > KEEP {
            self.camera_queue.pop_front();
        }
    }

    /// the frame to show after `shown`: the oldest queued one newer than it, else the latest of
    /// the camera topic; and whether more are waiting after that one
    pub fn next_camera(&self, shown: u64) -> (Option<&CameraFrame>, bool) {
        let mut newer = self.camera_queue.iter().filter(|f| f.seq > shown);
        match newer.next() {
            Some(f) => (Some(f), newer.next().is_some()),
            None => (self.camera(), false),
        }
    }

    /// publishers of a topic in the last graph poll; None when the topic is not in the graph
    pub fn publishers(&self, topic: &str) -> Option<usize> {
        self.graph.iter().find(|g| g.name == topic).map(|g| g.publishers)
    }

    pub fn topic_hz(&self, topic: &str, now: Instant) -> Option<f64> {
        self.topic_rates.get(topic).map(|r| r.hz(now))
    }

    /// graph topics of a message type, publishers first, then by name
    pub fn topics_of(&self, kind: MsgKind) -> Vec<&GraphTopic> {
        let mut v: Vec<&GraphTopic> = self.graph.iter().filter(|g| g.types.iter().any(|t| t == kind.ros_type())).collect();
        v.sort_by(|a, b| b.publishers.min(1).cmp(&a.publishers.min(1)).then_with(|| a.name.cmp(&b.name)));
        v
    }

    /// the topic a slot takes its data from: the first candidate someone publishes, else the
    /// first one that sent something; None when no candidate exists (the layer is hidden)
    pub fn bound(&self, slot: Slot) -> Option<&'static str> {
        let c = slot.candidates();
        c.iter()
            .find(|t| self.publishers(&slot.probe(t)).is_some_and(|n| n > 0))
            .or_else(|| c.iter().find(|t| self.topic_rates.contains_key(**t)))
            .copied()
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

    /// the health rate of a watched topic (the camera is the image topic in use)
    pub fn rate(&self, t: Topic) -> &RateMeter {
        static NONE: std::sync::LazyLock<RateMeter> = std::sync::LazyLock::new(RateMeter::default);
        if t == Topic::Camera {
            return self.camera_topic.as_deref().and_then(|c| self.topic_rates.get(c)).unwrap_or(&NONE);
        }
        &self.rates[t as usize]
    }

    pub fn tick(&mut self, t: Topic, now: Instant) {
        self.rates[t as usize].tick(now);
    }

    pub fn count(&mut self, name: &str) {
        match self.counts.iter_mut().find(|(n, _)| n == name) {
            Some((_, c)) => *c += 1,
            None => self.counts.push((name.to_owned(), 1)),
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

    /// world box to fit the view to: the map of `map_topic`, else around the robot, else
    /// 10 x 10 m at the origin
    pub fn bounds(&self, map_topic: Option<&str>) -> [f64; 4] {
        if let Some(m) = map_topic.and_then(|t| self.grid(t)).and_then(|g| g.grid.as_ref()) {
            let g = &m.meta;
            return [g.origin_x, g.origin_y, g.origin_x + f64::from(g.width) * g.resolution, g.origin_y + f64::from(g.height) * g.resolution];
        }
        let (x, y) = self.robot.map_or((0.0, 0.0), |r| (r.0, r.1));
        [x - 5.0, y - 5.0, x + 5.0, y + 5.0]
    }
}

/// a marker array's frontier goals: team7's `frontier_goals` namespace, or for explore_lite every
/// point but our camera views and labels
pub fn frontier_points(mks: &[Mk]) -> Vec<(f64, f64)> {
    let team7 = mks.iter().any(|m| m.ns == "frontier_goals");
    mks.iter()
        .filter(|m| if team7 { m.ns == "frontier_goals" } else { m.ns != "camera_views" && !m.ns.ends_with("_label") })
        .flat_map(|m| m.pts.iter().copied())
        .collect()
}

/// team7's planned camera view poses (`camera_views` namespace)
pub fn view_points(mks: &[Mk]) -> Vec<(f64, f64)> {
    mks.iter().filter(|m| m.ns == "camera_views").flat_map(|m| m.pts.iter().copied()).collect()
}

/// views and failing rules from a tentative label: "ID 3? 2v", or with the rules after " | "
/// ("ID 3? 5v | wall 1/3, angle 4/11 deg")
pub fn parse_label(text: &str) -> (Option<u32>, Vec<String>) {
    let (head, rules) = text.split_once(" | ").unwrap_or((text, ""));
    let views = head.split_whitespace().last().and_then(|w| w.strip_suffix('v')).and_then(|v| v.parse().ok());
    let reasons = rules.split(", ").map(str::trim).filter(|r| !r.is_empty()).map(str::to_owned).collect();
    (views, reasons)
}

/// marker_mapper's targets: spheres give the position, tentative labels the view count
pub fn targets_of(mks: &[Mk]) -> Vec<Target> {
    let mut out: Vec<Target> = mks
        .iter()
        .filter_map(|m| {
            let confirmed = match m.ns.as_str() {
                "target_confirmed" => true,
                "target_tentative" => false,
                _ => return None,
            };
            Some(Target { id: m.id, x: m.pos.0, y: m.pos.1, confirmed, views: None, reasons: Vec::new() })
        })
        .collect();
    for m in mks.iter().filter(|m| m.ns == "target_tentative_label") {
        if let Some(t) = out.iter_mut().find(|t| t.id == m.id && !t.confirmed) {
            (t.views, t.reasons) = parse_label(&m.text);
        }
    }
    out.sort_by_key(|t| t.id);
    out
}

/// the candidates (seen, not confirmed), closest to confirming first: fewest rules left (a label
/// without rules, from an older mapper, last), then most views, then id
pub fn candidates(targets: &[Target]) -> Vec<Target> {
    let mut v: Vec<Target> = targets.iter().filter(|t| !t.confirmed).cloned().collect();
    v.sort_by_key(|t| (if t.reasons.is_empty() { usize::MAX } else { t.reasons.len() }, std::cmp::Reverse(t.views), t.id));
    v
}

/// the confirmed targets, by id
pub fn confirmed(targets: &[Target]) -> Vec<Target> {
    targets.iter().filter(|t| t.confirmed).cloned().collect()
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
pub(crate) mod tests {
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
        assert_eq!(m.bounds(Some("/map")), [-5.0, -5.0, 5.0, 5.0]);
        m.robot = Some((1.0, 2.0, 0.0));
        assert_eq!(m.bounds(Some("/map")), [-4.0, -3.0, 6.0, 7.0]);
        let mut g = GridLayer::default();
        g.set(MapData {
            meta: GridMeta { width: 20, height: 10, resolution: 0.05, origin_x: -0.5, origin_y: 0.0 },
            data: vec![-1; 200],
            origin_yaw: 0.0,
        });
        m.put("/map", Feed::Grid(g), Instant::now());
        assert_eq!(m.bounds(Some("/map")), [-0.5, 0.0, 0.5, 0.5]);
        assert_eq!(m.grid("/map").unwrap().seq, 1);
        assert_eq!(m.bounds(None), [-4.0, -3.0, 6.0, 7.0]);
    }

    fn mk(ns: &str, id: i32, pos: (f64, f64), pts: Vec<(f64, f64)>, text: &str) -> Mk {
        Mk { ns: ns.into(), id, kind: ShapeKind::Points, pts, pos, color: [0, 0, 0, 255], text: text.into() }
    }

    #[test]
    fn markers_read_as_frontiers_views_and_targets() {
        let team7 = vec![
            mk("frontier_goals", 0, (0.0, 0.0), vec![(1.0, 1.0)], ""),
            mk("camera_views", 0, (0.0, 0.0), vec![(2.0, 2.0)], ""),
        ];
        assert_eq!(frontier_points(&team7), vec![(1.0, 1.0)]);
        assert_eq!(view_points(&team7), vec![(2.0, 2.0)]);
        // explore_lite: any namespace
        let stock = vec![mk("frontiers", 0, (0.0, 0.0), vec![(3.0, 3.0), (4.0, 4.0)], "")];
        assert_eq!(frontier_points(&stock), vec![(3.0, 3.0), (4.0, 4.0)]);
        let t = targets_of(&[
            mk("target_confirmed", 7, (1.0, 2.0), vec![], ""),
            mk("target_tentative", 3, (-1.0, 0.5), vec![], ""),
            mk("target_tentative_label", 3, (-1.0, 0.5), vec![], "ID 3? 2v"),
        ]);
        assert_eq!(
            t,
            vec![
                Target { id: 3, x: -1.0, y: 0.5, confirmed: false, views: Some(2), reasons: vec![] },
                Target { id: 7, x: 1.0, y: 2.0, confirmed: true, views: None, reasons: vec![] }
            ]
        );
    }

    #[test]
    fn labels_parse_with_and_without_reasons() {
        assert_eq!(parse_label("ID 3? 2v"), (Some(2), vec![]));
        assert_eq!(parse_label("ID 3? 5v | wall 1/3, angle 4/11 deg"), (Some(5), vec!["wall 1/3".to_owned(), "angle 4/11 deg".to_owned()]));
        let all = "ID 12? 1v | views 1/5, wall 0/3, size unknown, angle 0/11 deg, rays disagree, spread 31 cm, views disagree";
        let (v, r) = parse_label(all);
        assert_eq!(v, Some(1));
        assert_eq!(r.len(), 7);
        assert_eq!(r[5], "spread 31 cm");
        // a confirmed label has no view count
        assert_eq!(parse_label("ID 3 (1.20, -0.50)"), (None, vec![]));
        assert_eq!(parse_label("ID 3? 4v | "), (Some(4), vec![]));
    }

    fn tentative(id: i32, at: (f64, f64), label: &str) -> [Mk; 2] {
        [mk("target_tentative", id, at, vec![], ""), mk("target_tentative_label", id, at, vec![], label)]
    }

    #[test]
    fn candidates_and_confirmed_split_and_an_id_moves_over() {
        let t0 = Instant::now();
        let mut mks: Vec<Mk> = Vec::new();
        mks.extend(tentative(3, (1.0, 1.0), "ID 3? 5v | angle 4/11 deg"));
        mks.extend(tentative(4, (2.0, 1.0), "ID 4? 2v | views 2/5, wall 0/3"));
        mks.extend(tentative(5, (3.0, 1.0), "ID 5? 7v | wall 1/3, spread 20 cm"));
        mks.extend(tentative(6, (4.0, 1.0), "ID 6? 9v"));
        mks.push(mk("target_confirmed", 7, (0.5, 2.0), vec![], ""));
        mks.push(mk("target_confirmed_label", 7, (0.5, 2.0), vec![], "ID 7 (0.50, 2.00)"));
        let all = targets_of(&mks);
        let c = candidates(&all);
        // one rule left first, then fewer rules / more views, rule-less (old mapper) last
        assert_eq!(c.iter().map(|t| t.id).collect::<Vec<_>>(), vec![3, 5, 4, 6]);
        assert!(c[0].close() && !c[1].close());
        assert_eq!(confirmed(&all).iter().map(|t| t.id).collect::<Vec<_>>(), vec![7]);
        let mut m = Model::default();
        m.note_targets(&all, t0);
        assert_eq!(m.target_log[&3], Seen { views: Some(5), confirmed_at: None });
        assert!(m.target_log[&7].confirmed_at == Some(t0));

        // id 3 gets its last view: it leaves the candidates and joins the confirmed, with its time
        // and the views it had
        let t1 = t0 + Duration::from_secs(4);
        mks.retain(|k| k.id != 3);
        mks.push(mk("target_confirmed", 3, (1.02, 0.98), vec![], ""));
        let all = targets_of(&mks);
        assert_eq!(candidates(&all).iter().map(|t| t.id).collect::<Vec<_>>(), vec![5, 4, 6]);
        assert_eq!(confirmed(&all).iter().map(|t| t.id).collect::<Vec<_>>(), vec![3, 7]);
        m.note_targets(&all, t1);
        assert_eq!(m.target_log[&3], Seen { views: Some(5), confirmed_at: Some(t1) });
        // a later frame keeps the first time
        m.note_targets(&all, t1 + Duration::from_secs(1));
        assert_eq!(m.target_log[&3].confirmed_at, Some(t1));
        assert_eq!(m.target_log[&7].confirmed_at, Some(t0));
        // a lost target goes back to candidate; an id no longer published is forgotten
        mks.retain(|k| k.id != 7 && k.id != 6);
        mks.extend(tentative(7, (0.5, 2.0), "ID 7? 8v | spread 12 cm"));
        let all = targets_of(&mks);
        m.note_targets(&all, t1 + Duration::from_secs(2));
        assert_eq!(m.target_log[&7].confirmed_at, None);
        assert!(!m.target_log.contains_key(&6));
    }

    #[test]
    fn detection_ticks_fade_out() {
        let t0 = Instant::now();
        let mut m = Model::default();
        let ray = |id, at| Ray { id, from: (0.0, 0.0), to: (1.0, 0.0), at };
        m.add_rays("/aruco_detections", vec![ray(3, t0)], t0);
        m.add_rays("/aruco_detections", vec![ray(4, t0 + Duration::from_millis(100))], t0 + Duration::from_millis(100));
        assert_eq!(m.rays("/aruco_detections").len(), 2);
        let later = t0 + RAY_FOR + Duration::from_millis(50);
        m.add_rays("/aruco_detections", vec![], later);
        assert_eq!(m.rays("/aruco_detections").iter().map(|r| r.id).collect::<Vec<_>>(), vec![4]);
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
        m.graph = graph(&[("/plan", "nav_msgs/msg/Path", 1), ("/explore/frontiers", "visualization_msgs/msg/MarkerArray", 1), ("/local_costmap/costmap", "nav_msgs/msg/OccupancyGrid", 1), ("/navigate_to_pose/_action/status", "action_msgs/msg/GoalStatusArray", 1)]);
        assert_eq!(m.bound(Slot::Plan), Some("/plan"));
        assert_eq!(m.bound(Slot::Frontiers), Some("/explore/frontiers"));
        assert_eq!(m.bound(Slot::NavAction), Some("/navigate_to_pose"));
        assert_eq!(m.bound(Slot::Views), None);
        assert_eq!(m.stack(), "stock");
        // a team7 topic with only our own subscription does not count; with a publisher it wins
        m.graph.extend(graph(&[("/team7/path", "nav_msgs/msg/Path", 0)]));
        assert_eq!(m.bound(Slot::Plan), Some("/plan"));
        m.graph.last_mut().unwrap().publishers = 1;
        assert_eq!(m.bound(Slot::Plan), Some("/team7/path"));
        assert_eq!(m.stack(), "mixed");
        // before the first graph poll, data decides
        let mut m = Model::default();
        m.put("/aruco_tracker/debug", Feed::Points(vec![]), t0);
        assert_eq!(m.bound(Slot::Camera), Some("/aruco_tracker/debug"));
    }

    pub(crate) fn graph(list: &[(&str, &str, usize)]) -> Vec<GraphTopic> {
        list.iter().map(|&(n, t, p)| GraphTopic { name: n.into(), types: vec![t.into()], publishers: p }).collect()
    }

    #[test]
    fn topics_of_a_type_list_publishers_first() {
        let m = Model {
            graph: graph(&[("/b_path", "nav_msgs/msg/Path", 0), ("/a_path", "nav_msgs/msg/Path", 2), ("/scan", "sensor_msgs/msg/LaserScan", 1), ("/c_path", "nav_msgs/msg/Path", 1)]),
            ..Model::default()
        };
        let names: Vec<&str> = m.topics_of(MsgKind::Path).iter().map(|g| g.name.as_str()).collect();
        assert_eq!(names, ["/a_path", "/c_path", "/b_path"]);
    }

    #[test]
    fn camera_frames_that_come_together_are_shown_one_by_one() {
        let now = Instant::now();
        let frame = |seq: u64| CameraFrame { width: 1, height: 1, rgba: vec![0; 4], seq, at: now };
        let mut m = Model { camera_topic: Some("/cam".into()), ..Model::default() };
        assert!(m.next_camera(0).0.is_none());
        // three frames in one burst, between two paints
        for s in 1..=3 {
            m.queue_camera(frame(s));
            m.put("/cam", Feed::Image(frame(s)), now);
        }
        let (f, more) = m.next_camera(0);
        assert_eq!((f.unwrap().seq, more), (1, true));
        let (f, more) = m.next_camera(1);
        assert_eq!((f.unwrap().seq, more), (2, true));
        let (f, more) = m.next_camera(2);
        assert_eq!((f.unwrap().seq, more), (3, false));
        // all shown: the latest stays
        assert_eq!(m.next_camera(3).0.unwrap().seq, 3);
        // the queue keeps only the last few; a restart (lower seq) starts over
        for s in 4..=10 {
            m.queue_camera(frame(s));
        }
        assert_eq!(m.camera_queue.len(), 4);
        assert_eq!(m.next_camera(3).0.unwrap().seq, 7);
        m.queue_camera(frame(1));
        assert_eq!(m.camera_queue.len(), 1);
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
