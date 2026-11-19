//! The ROS side: a node on its own thread. It subscribes, puts everything in the map frame with TF,
//! and writes into the shared `Model`. It publishes nothing; with `--allow-goals` it also has an
//! action client for NavigateToPose and sends the goals the 2D Goal tool hands it.

use std::cell::RefCell;
use std::rc::Rc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::Receiver;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use futures::executor::LocalPool;
use futures::stream::{Stream, StreamExt};
use futures::task::LocalSpawnExt;
use r2r::QosProfile;
use r2r::geometry_msgs::msg::{PoseStamped, Transform};
use r2r::nav_msgs::msg::{OccupancyGrid, Odometry, Path};
use r2r::sensor_msgs::msg::{Image, LaserScan};
use r2r::std_msgs::msg::Header;
use r2r::tf2_msgs::msg::TFMessage;
use r2r::visualization_msgs::msg::{Marker, MarkerArray};
use tb4_core::nalgebra::{Point3, Quaternion, Translation3, UnitQuaternion};
use tb4_core::{GridMeta, Isometry3, Pose2, Stamp, TfBuffer};

use crate::model::{CameraFrame, MapData, Model, Shape, ShapeKind, Target, Topic};

pub const MAP_FRAME: &str = "map";
pub const BASE_FRAME: &str = "base_link";
pub const GOAL_ACTION: &str = "/team7/navigate_to_pose";

pub type Shared = Arc<Mutex<Model>>;

fn iso(t: &Transform) -> Isometry3<f64> {
    let q = &t.rotation;
    Isometry3::from_parts(
        Translation3::new(t.translation.x, t.translation.y, t.translation.z),
        UnitQuaternion::from_quaternion(Quaternion::new(q.w, q.x, q.y, q.z)),
    )
}

fn secs(t: &r2r::builtin_interfaces::msg::Time) -> f64 {
    f64::from(t.sec) + f64::from(t.nanosec) * 1e-9
}

/// system time, seconds since the epoch
pub fn wall_secs() -> f64 {
    SystemTime::now().duration_since(UNIX_EPOCH).map_or(0.0, |d| d.as_secs_f64())
}

const DELETEALL: i32 = 3;
const LATCHED: QosProfile = QosProfile::default().keep_last(1).reliable().transient_local();

/// laser ranges -> points in the laser frame, invalid ranges dropped
pub fn scan_points(s: &LaserScan) -> impl Iterator<Item = (f64, f64)> + '_ {
    s.ranges.iter().enumerate().filter_map(move |(i, &r)| {
        let ok = r.is_finite() && r >= s.range_min && r <= s.range_max && r > 0.0;
        ok.then(|| {
            let a = f64::from(s.angle_min) + i as f64 * f64::from(s.angle_increment);
            (f64::from(r) * a.cos(), f64::from(r) * a.sin())
        })
    })
}

/// views from a tentative label "ID 3? 2v"
fn label_views(text: &str) -> Option<u32> {
    text.split_whitespace().last()?.strip_suffix('v')?.parse().ok()
}

/// the latest /team7/targets array -> targets (spheres give the position, labels the view count)
pub fn targets_from(m: &MarkerArray, prev: &[Target]) -> Vec<Target> {
    let mut out = if m.markers.iter().any(|mk| mk.action == DELETEALL) { Vec::new() } else { prev.to_vec() };
    for mk in &m.markers {
        let confirmed = match mk.ns.as_str() {
            "target_confirmed" => true,
            "target_tentative" => false,
            _ => continue,
        };
        out.retain(|t| t.id != mk.id);
        if mk.action == 0 {
            out.push(Target { id: mk.id, x: mk.pose.position.x, y: mk.pose.position.y, confirmed, views: None });
        }
    }
    for mk in m.markers.iter().filter(|mk| mk.ns == "target_tentative_label") {
        if let Some(t) = out.iter_mut().find(|t| t.id == mk.id) {
            t.views = label_views(&mk.text);
        }
    }
    out.sort_by_key(|t| t.id);
    out
}

/// a marker as a top-down shape, points still in the marker's frame
pub fn shape_of(mk: &Marker) -> Option<Shape> {
    const ARROW_ETC: [i32; 2] = [0, 9];
    let kind = match mk.type_ {
        8 | 6 | 7 | 1 | 2 => ShapeKind::Points,
        4 => ShapeKind::Strip,
        5 => ShapeKind::Segments,
        t if ARROW_ETC.contains(&t) => return None,
        _ => return None,
    };
    let pts: Vec<_> = if mk.points.is_empty() {
        vec![(mk.pose.position.x, mk.pose.position.y)]
    } else {
        // the marker pose places its points (identity for all our markers)
        let p = iso_pose(&mk.pose);
        mk.points.iter().map(|q| p * Point3::new(q.x, q.y, q.z)).map(|q| (q.x, q.y)).collect()
    };
    let c = &mk.color;
    let to8 = |v: f32| (v.clamp(0.0, 1.0) * 255.0).round() as u8;
    Some(Shape { kind, pts, color: [to8(c.r), to8(c.g), to8(c.b), to8(c.a).max(160)] })
}

fn iso_pose(p: &r2r::geometry_msgs::msg::Pose) -> Isometry3<f64> {
    let q = &p.orientation;
    let q = if q.x == 0.0 && q.y == 0.0 && q.z == 0.0 && q.w == 0.0 { Quaternion::new(1.0, 0.0, 0.0, 0.0) } else { Quaternion::new(q.w, q.x, q.y, q.z) };
    Isometry3::from_parts(Translation3::new(p.position.x, p.position.y, p.position.z), UnitQuaternion::from_quaternion(q))
}

/// sensor_msgs/Image -> RGBA8 (rgb8, bgr8, rgba8, bgra8, mono8)
pub fn image_rgba(m: &Image) -> Option<(u32, u32, Vec<u8>)> {
    let (w, h, step) = (m.width as usize, m.height as usize, m.step as usize);
    let bpp = match m.encoding.as_str() {
        "rgb8" | "bgr8" => 3,
        "rgba8" | "bgra8" => 4,
        "mono8" => 1,
        _ => return None,
    };
    if w == 0 || h == 0 || step < w * bpp || m.data.len() < step * (h - 1) + w * bpp {
        return None;
    }
    let bgr = m.encoding.starts_with("bgr");
    let mut out = Vec::with_capacity(w * h * 4);
    for row in 0..h {
        for px in m.data[row * step..row * step + w * bpp].chunks_exact(bpp) {
            out.extend_from_slice(&match (bpp, bgr) {
                (1, _) => [px[0], px[0], px[0], 255],
                (_, false) => [px[0], px[1], px[2], 255],
                (_, true) => [px[2], px[1], px[0], 255],
            });
        }
    }
    Some((m.width, m.height, out))
}

/// a dynamic TF stamp this much older than the newest one means the clock went back (a replay restarted)
const JUMP_BACK_NS: i64 = 1_000_000_000;

/// frame -> map through TF, as a 3D isometry (the laser may be mounted rolled)
struct Tf {
    buf: TfBuffer,
    /// newest dynamic stamp seen
    newest: Stamp,
    /// static transforms, kept across a reset: (child, parent, stamp, transform)
    statics: Vec<(String, String, Stamp, Isometry3<f64>)>,
    /// per child frame: parent and newest dynamic stamp, for the age of a chain
    edges: Vec<(String, String, Stamp)>,
}

impl Tf {
    fn new() -> Self {
        Self { buf: TfBuffer::new(10.0, 0.1), newest: Stamp(0), statics: Vec::new(), edges: Vec::new() }
    }

    /// adds one transform; true when time jumped back and the dynamic transforms were dropped.
    /// Without the reset, `Stamp::LATEST` would keep returning the previous run's last pose,
    /// because the buffer keeps samples sorted by stamp.
    fn insert(&mut self, parent: &str, child: &str, stamp: Stamp, t: Isometry3<f64>, is_static: bool) -> bool {
        let (parent, child) = (parent.trim_start_matches('/'), child.trim_start_matches('/'));
        if is_static {
            self.statics.retain(|(c, ..)| c != child);
            self.statics.push((child.to_owned(), parent.to_owned(), stamp, t));
            self.buf.insert(parent, child, stamp, t, true);
            return false;
        }
        let jumped = self.newest.0 > 0 && stamp.0 + JUMP_BACK_NS < self.newest.0;
        if jumped {
            self.buf = TfBuffer::new(10.0, 0.1);
            for (c, p, s, t) in &self.statics {
                self.buf.insert(p, c, *s, *t, true);
            }
            self.edges.clear();
            self.newest = stamp;
        }
        self.newest = self.newest.max(stamp);
        match self.edges.iter_mut().find(|(c, ..)| c == child) {
            Some(e) => {
                e.1 = parent.to_owned();
                e.2 = e.2.max(stamp);
            }
            None => self.edges.push((child.to_owned(), parent.to_owned(), stamp)),
        }
        self.buf.insert(parent, child, stamp, t, false);
        jumped
    }

    /// the stamp of the stalest dynamic edge from `frame` up to the map, seconds
    fn chain_stamp(&self, frame: &str) -> Option<f64> {
        let mut cur = frame;
        let mut oldest: Option<Stamp> = None;
        for _ in 0..32 {
            if cur == MAP_FRAME {
                return oldest.map(Stamp::secs_f64);
            }
            if let Some((_, p, s)) = self.edges.iter().find(|(c, ..)| c == cur) {
                oldest = Some(oldest.map_or(*s, |o| o.min(*s)));
                cur = p;
            } else if let Some((_, p, ..)) = self.statics.iter().find(|(c, ..)| c == cur) {
                cur = p;
            } else {
                return None;
            }
        }
        None
    }

    /// at `stamp` (interpolated), or the latest transform when the buffer does not cover it
    fn to_map_at(&self, frame: &str, stamp: Stamp) -> Result<Isometry3<f64>, String> {
        let frame = if frame.is_empty() { MAP_FRAME } else { frame };
        self.buf
            .lookup(MAP_FRAME, frame, stamp)
            .or_else(|_| self.buf.lookup(MAP_FRAME, frame, Stamp::LATEST))
            .map_err(|e| format!("TF {frame} -> map: {e}"))
    }

    fn to_map(&self, frame: &str) -> Result<Isometry3<f64>, String> {
        self.to_map_at(frame, Stamp::LATEST)
    }

    fn points_at(&self, frame: &str, stamp: Stamp, pts: impl Iterator<Item = (f64, f64)>) -> Result<Vec<(f64, f64)>, String> {
        let t = self.to_map_at(frame, stamp)?;
        Ok(pts.map(|(x, y)| t * Point3::new(x, y, 0.0)).map(|p| (p.x, p.y)).collect())
    }

    #[cfg(test)]
    fn points(&self, frame: &str, pts: impl Iterator<Item = (f64, f64)>) -> Result<Vec<(f64, f64)>, String> {
        self.points_at(frame, Stamp::LATEST, pts)
    }
}

struct Ctx {
    model: Shared,
    tf: Rc<RefCell<Tf>>,
    dirty: Arc<AtomicBool>,
}

impl Ctx {
    fn update(&self, f: impl FnOnce(&mut Model, &Tf)) {
        let tf = self.tf.borrow();
        if let Ok(mut m) = self.model.lock() {
            f(&mut m, &tf);
        }
        self.dirty.store(true, Ordering::Relaxed);
    }
}

fn spawn<T: 'static>(pool: &LocalPool, stream: impl Stream<Item = T> + 'static, ctx: &Rc<Ctx>, mut f: impl FnMut(&Ctx, T) + 'static) {
    let ctx = ctx.clone();
    pool.spawner()
        .spawn_local(stream.for_each(move |msg| {
            f(&ctx, msg);
            futures::future::ready(())
        }))
        .expect("spawn subscriber");
}

fn in_map(m: &mut Model, r: Result<Vec<(f64, f64)>, String>) -> Option<Vec<(f64, f64)>> {
    r.map_err(|e| m.tf_problem = Some((e, Instant::now()))).ok()
}

/// points in the header's frame -> map, with TF at the header stamp (a zero stamp means latest)
fn header_points<'a>(tf: &Tf, h: &Header, pts: impl Iterator<Item = (f64, f64)> + 'a) -> Result<Vec<(f64, f64)>, String> {
    tf.points_at(&h.frame_id, Stamp::from_ros(h.stamp.sec, h.stamp.nanosec), pts)
}

fn grid_of(msg: OccupancyGrid) -> Option<MapData> {
    let i = &msg.info;
    if i.width == 0 || i.height == 0 || msg.data.len() != i.width as usize * i.height as usize {
        return None;
    }
    let meta = GridMeta { width: i.width, height: i.height, resolution: f64::from(i.resolution), origin_x: i.origin.position.x, origin_y: i.origin.position.y };
    let q = &i.origin.orientation;
    let origin_yaw = (2.0 * (q.w * q.z + q.x * q.y)).atan2(1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    Some(MapData { meta, data: msg.data, origin_yaw })
}

/// options for the node
pub struct Options {
    /// goals from the 2D Goal tool; None: read-only, no action client at all
    pub goals: Option<Receiver<(f64, f64, f64)>>,
}

/// subscribe everything and spin until `stop`; sets `dirty` whenever the model changed
pub fn run(model: Shared, stop: Arc<AtomicBool>, dirty: Arc<AtomicBool>, opts: Options) -> r2r::Result<()> {
    let ctx_r2r = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx_r2r, "team7_viz", "")?;
    let pool = LocalPool::new();
    let tf = Rc::new(RefCell::new(Tf::new()));
    let ctx = Rc::new(Ctx { model, tf: tf.clone(), dirty });

    // TF first: everything else is put in the map frame with it
    for (topic, qos, is_static) in [("/tf", QosProfile::default().keep_last(100), false), ("/tf_static", LATCHED.keep_last(100), true)] {
        let s = node.subscribe::<TFMessage>(topic, qos)?;
        let tf = tf.clone();
        spawn(&pool, s, &ctx, move |c, msg: TFMessage| {
            let mut jumped = false;
            {
                let mut t = tf.borrow_mut();
                for x in &msg.transforms {
                    let st = Stamp::from_ros(x.header.stamp.sec, x.header.stamp.nanosec);
                    jumped |= t.insert(&x.header.frame_id, &x.child_frame_id, st, iso(&x.transform), is_static);
                }
            }
            c.update(|m, tf| {
                if jumped {
                    // a new replay: drop what belongs to the old time line
                    m.reset_run();
                    m.tf_problem = Some(("time jumped back: TF reset (replay restarted?)".into(), Instant::now()));
                }
                m.count(if is_static { "tf_static" } else { "tf" });
                if !is_static {
                    m.tick(Topic::Tf, Instant::now());
                }
                if let Ok(t) = tf.to_map(BASE_FRAME) {
                    let p = Pose2::from_isometry(&t);
                    m.set_robot(p.x, p.y, p.yaw);
                    m.tf_stamp = tf.chain_stamp(BASE_FRAME);
                }
            });
        });
    }

    let s = node.subscribe::<r2r::rosgraph_msgs::msg::Clock>("/clock", QosProfile::sensor_data())?;
    spawn(&pool, s, &ctx, |c, msg: r2r::rosgraph_msgs::msg::Clock| {
        c.update(|m, _| m.clock = Some((secs(&msg.clock), Instant::now())));
    });

    let s = node.subscribe::<OccupancyGrid>("/map", QosProfile::default().reliable().transient_local())?;
    spawn(&pool, s, &ctx, |c, msg: OccupancyGrid| {
        c.update(|m, _| {
            m.count("map");
            m.tick(Topic::Map, Instant::now());
            if let Some(g) = grid_of(msg) {
                m.map.set(g);
            }
        });
    });

    // costmaps: global is latched, the MPPI local cost is not; both in the map frame
    let s = node.subscribe::<OccupancyGrid>("/global_costmap/costmap", QosProfile::default().reliable().transient_local())?;
    spawn(&pool, s, &ctx, |c, msg: OccupancyGrid| {
        c.update(|m, _| {
            m.count("global_costmap");
            if let Some(g) = grid_of(msg) {
                m.global_cost.set(g);
            }
        });
    });
    let s = node.subscribe::<OccupancyGrid>("/team7/mppi/local_cost", QosProfile::default())?;
    spawn(&pool, s, &ctx, |c, msg: OccupancyGrid| {
        c.update(|m, _| {
            m.count("local_cost");
            if let Some(g) = grid_of(msg) {
                m.local_cost.set(g);
            }
        });
    });

    let s = node.subscribe::<LaserScan>("/scan", QosProfile::sensor_data())?;
    spawn(&pool, s, &ctx, |c, msg: LaserScan| {
        c.update(|m, tf| {
            let now = Instant::now();
            m.count("scan");
            m.tick(Topic::Scan, now);
            let (ros_now, _) = m.ros_now(now, wall_secs());
            let d = ros_now - secs(&msg.header.stamp);
            m.scan_delay = Some(m.scan_delay.map_or(d, |old| 0.8 * old + 0.2 * d));
            if let Some(p) = in_map(m, header_points(tf, &msg.header, scan_points(&msg))) {
                m.scan = p;
            }
        });
    });

    let s = node.subscribe::<Odometry>("/odom", QosProfile::sensor_data())?;
    spawn(&pool, s, &ctx, |c, _msg: Odometry| {
        c.update(|m, _| {
            m.count("odom");
            m.tick(Topic::Odom, Instant::now());
        });
    });

    for (topic, is_traj) in [("/team7/path", false), ("/team7/mppi/trajectory", true)] {
        let s = node.subscribe::<Path>(topic, QosProfile::default())?;
        spawn(&pool, s, &ctx, move |c, msg: Path| {
            c.update(|m, tf| {
                m.count(if is_traj { "trajectory" } else { "path" });
                let pts = msg.poses.iter().map(|p| (p.pose.position.x, p.pose.position.y));
                if let Some(p) = in_map(m, header_points(tf, &msg.header, pts)) {
                    if is_traj { m.trajectory = p } else { m.path = p }
                }
            });
        });
    }

    let s = node.subscribe::<MarkerArray>("/team7/frontiers", QosProfile::default())?;
    spawn(&pool, s, &ctx, |c, msg: MarkerArray| {
        c.update(|m, tf| {
            m.count("frontiers");
            if msg.markers.iter().any(|mk| mk.action == DELETEALL) {
                m.frontiers.clear();
                m.views.clear();
            }
            for mk in &msg.markers {
                let pts = mk.points.iter().map(|p| (p.x, p.y));
                let Some(p) = in_map(m, header_points(tf, &mk.header, pts)) else { continue };
                match mk.ns.as_str() {
                    "frontier_goals" => m.frontiers = p,
                    "camera_views" => m.views = p,
                    _ => {}
                }
            }
        });
    });

    let s = node.subscribe::<MarkerArray>("/team7/slam/graph", QosProfile::default())?;
    spawn(&pool, s, &ctx, |c, msg: MarkerArray| {
        c.update(|m, tf| {
            m.count("slam_graph");
            if msg.markers.iter().any(|mk| mk.action == DELETEALL) {
                m.slam_graph.clear();
            }
            for mk in msg.markers.iter().filter(|mk| mk.action == 0) {
                let Some(mut s) = shape_of(mk) else { continue };
                let Some(p) = in_map(m, header_points(tf, &mk.header, s.pts.iter().copied())) else { continue };
                s.pts = p;
                m.slam_graph.push(s);
            }
        });
    });

    let s = node.subscribe::<PoseStamped>("/team7/goal", QosProfile::default())?;
    spawn(&pool, s, &ctx, |c, msg: PoseStamped| {
        c.update(|m, tf| {
            m.count("goal");
            let p = msg.pose.position;
            if let Some(g) = in_map(m, header_points(tf, &msg.header, std::iter::once((p.x, p.y)))) {
                m.goal = g.first().copied();
            }
        });
    });

    let s = node.subscribe::<MarkerArray>("/team7/targets", QosProfile::default())?;
    spawn(&pool, s, &ctx, |c, msg: MarkerArray| {
        c.update(|m, _| {
            m.count("targets");
            // targets are published in the map frame (marker_mapper `frame` = map)
            m.targets = targets_from(&msg, &m.targets);
        });
    });

    let s = node.subscribe::<r2r::std_msgs::msg::String>("/team7/mission_state", LATCHED)?;
    spawn(&pool, s, &ctx, |c, msg: r2r::std_msgs::msg::String| {
        c.update(|m, _| {
            m.count("mission_state");
            m.set_mission(msg.data, Instant::now());
        });
    });

    let s = node.subscribe::<Image>("/team7/aruco/debug", QosProfile::sensor_data())?;
    spawn(&pool, s, &ctx, |c, msg: Image| {
        let frame = image_rgba(&msg);
        c.update(|m, _| {
            let now = Instant::now();
            m.count("camera");
            m.tick(Topic::Camera, now);
            if let Some((width, height, rgba)) = frame {
                let seq = m.camera.as_ref().map_or(1, |f| f.seq + 1);
                m.camera = Some(CameraFrame { width, height, rgba, seq, at: now });
            }
        });
    });

    // goals: only with --allow-goals, and only to our navigator's action server
    type Nav = r2r::nav2_msgs::action::NavigateToPose::Action;
    let client = match &opts.goals {
        Some(_) => Some(node.create_action_client::<Nav>(GOAL_ACTION)?),
        None => None,
    };

    let mut pool = pool;
    while !stop.load(Ordering::Relaxed) {
        node.spin_once(Duration::from_millis(10));
        if let (Some(rx), Some(client)) = (&opts.goals, &client) {
            while let Ok((x, y, yaw)) = rx.try_recv() {
                send_goal(client, &pool, &ctx, (x, y, yaw));
            }
        }
        pool.run_until_stalled();
    }
    Ok(())
}

fn send_goal(client: &r2r::ActionClient<r2r::nav2_msgs::action::NavigateToPose::Action>, pool: &LocalPool, ctx: &Rc<Ctx>, (x, y, yaw): (f64, f64, f64)) {
    let mut goal = r2r::nav2_msgs::action::NavigateToPose::Goal::default();
    goal.pose.header.frame_id = MAP_FRAME.into();
    goal.pose.pose.position.x = x;
    goal.pose.pose.position.y = y;
    goal.pose.pose.orientation.z = (yaw / 2.0).sin();
    goal.pose.pose.orientation.w = (yaw / 2.0).cos();
    let say = |c: &Ctx, s: String| c.update(|m, _| m.goal_status = Some(s));
    match client.send_goal_request(goal) {
        Err(e) => say(ctx, format!("goal not sent: {e}")),
        Ok(request) => {
            say(ctx, format!("goal ({x:.2}, {y:.2}) sent to {GOAL_ACTION}, waiting"));
            let c = ctx.clone();
            let _ = pool.spawner().spawn_local(async move {
                match request.await {
                    Ok((_handle, result, _feedback)) => {
                        say(&c, format!("goal ({x:.2}, {y:.2}) accepted"));
                        match result.await {
                            Ok((status, _)) => say(&c, format!("goal ({x:.2}, {y:.2}) finished: {status:?}")),
                            Err(e) => say(&c, format!("goal ({x:.2}, {y:.2}) result: {e}")),
                        }
                    }
                    Err(e) => say(&c, format!("goal ({x:.2}, {y:.2}) rejected: {e}")),
                }
            });
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn scan_points_drop_invalid_ranges() {
        let s = LaserScan {
            angle_min: 0.0,
            angle_increment: std::f32::consts::FRAC_PI_2,
            range_min: 0.1,
            range_max: 10.0,
            ranges: vec![1.0, f32::INFINITY, 0.05, 2.0, f32::NAN, 20.0],
            ..LaserScan::default()
        };
        let p: Vec<_> = scan_points(&s).collect();
        assert_eq!(p.len(), 2);
        assert!((p[0].0 - 1.0).abs() < 1e-9 && p[0].1.abs() < 1e-9);
        assert!(p[1].0.abs() < 1e-6 && (p[1].1 + 2.0).abs() < 1e-6, "{:?}", p[1]);
    }

    fn marker(ns: &str, id: i32, action: i32, x: f64, y: f64) -> Marker {
        let mut m = Marker { ns: ns.into(), id, action, ..Marker::default() };
        m.pose.position.x = x;
        m.pose.position.y = y;
        m
    }

    #[test]
    fn targets_follow_the_marker_mapper_array() {
        let mut label = marker("target_tentative_label", 3, 0, -1.0, 0.5);
        label.text = "ID 3? 2v".into();
        let msg = MarkerArray {
            markers: vec![
                marker("", 0, DELETEALL, 0.0, 0.0),
                marker("target_confirmed", 7, 0, 1.0, 2.0),
                marker("target_confirmed_label", 7, 0, 1.0, 2.0),
                marker("target_tentative", 3, 0, -1.0, 0.5),
                label,
            ],
        };
        let old = [Target { id: 9, x: 0.0, y: 0.0, confirmed: true, views: None }];
        let t = targets_from(&msg, &old);
        assert_eq!(
            t,
            vec![
                Target { id: 3, x: -1.0, y: 0.5, confirmed: false, views: Some(2) },
                Target { id: 7, x: 1.0, y: 2.0, confirmed: true, views: None }
            ]
        );
        let up = MarkerArray { markers: vec![marker("target_confirmed", 3, 0, -1.1, 0.5)] };
        let t = targets_from(&up, &t);
        assert_eq!(t.len(), 2);
        assert!(t.iter().all(|t| t.confirmed));
    }

    #[test]
    fn slam_graph_markers_become_shapes() {
        let mut mk = marker("slam_loop_edges", 0, 0, 0.0, 0.0);
        mk.type_ = 5;
        mk.pose.orientation.w = 1.0;
        mk.points = vec![r2r::geometry_msgs::msg::Point { x: 1.0, y: 2.0, z: 0.0 }, r2r::geometry_msgs::msg::Point { x: 3.0, y: 4.0, z: 0.0 }];
        mk.color = r2r::std_msgs::msg::ColorRGBA { r: 1.0, g: 0.1, b: 0.8, a: 0.9 };
        let s = shape_of(&mk).unwrap();
        assert_eq!((s.kind, s.pts.clone()), (ShapeKind::Segments, vec![(1.0, 2.0), (3.0, 4.0)]));
        assert_eq!(s.color, [255, 26, 204, 230]);
        mk.type_ = 9; // text
        assert!(shape_of(&mk).is_none());
    }

    #[test]
    fn images_convert_to_rgba() {
        let img = Image { width: 2, height: 1, step: 8, encoding: "bgr8".into(), data: vec![1, 2, 3, 4, 5, 6, 0, 0], ..Image::default() };
        assert_eq!(image_rgba(&img).unwrap(), (2, 1, vec![3, 2, 1, 255, 6, 5, 4, 255]));
        let mono = Image { width: 1, height: 1, step: 1, encoding: "mono8".into(), data: vec![9], ..Image::default() };
        assert_eq!(image_rgba(&mono).unwrap().2, vec![9, 9, 9, 255]);
        let short = Image { width: 4, height: 4, step: 12, encoding: "rgb8".into(), data: vec![0; 10], ..Image::default() };
        assert!(image_rgba(&short).is_none());
    }

    #[test]
    fn points_go_through_tf_into_the_map_frame() {
        let mut tf = Tf::new();
        tf.insert("map", "odom", Stamp(1), t(1.0, 0.0), false);
        tf.insert("odom", "base_link", Stamp(1), t(0.5, std::f64::consts::FRAC_PI_2), false);
        let rolled = Isometry3::from_parts(Translation3::new(0.0, 0.0, 0.2), UnitQuaternion::from_euler_angles(std::f64::consts::PI, 0.0, 0.0));
        tf.insert("base_link", "rplidar_link", Stamp(0), rolled, true);
        let p = tf.points("rplidar_link", [(1.0, 0.0), (0.0, 1.0)].into_iter()).unwrap();
        assert!((p[0].0 - 1.5).abs() < 1e-9 && (p[0].1 - 1.0).abs() < 1e-9, "{:?}", p[0]);
        assert!((p[1].0 - 2.5).abs() < 1e-9 && p[1].1.abs() < 1e-9, "{:?}", p[1]);
        assert!(tf.points("camera", std::iter::empty()).is_err());
        assert_eq!(tf.points("", [(2.0, 3.0)].into_iter()).unwrap(), vec![(2.0, 3.0)]);
    }

    fn t(x: f64, yaw: f64) -> Isometry3<f64> {
        Isometry3::from_parts(Translation3::new(x, 0.0, 0.0), UnitQuaternion::from_euler_angles(0.0, 0.0, yaw))
    }

    const S: i64 = 1_000_000_000;

    #[test]
    fn scans_use_tf_at_their_stamp() {
        let mut tf = Tf::new();
        tf.insert("map", "base_link", Stamp(10 * S), t(0.0, 0.0), false);
        tf.insert("map", "base_link", Stamp(11 * S), t(1.0, 0.0), false);
        let h = Header { stamp: r2r::builtin_interfaces::msg::Time { sec: 10, nanosec: 500_000_000 }, frame_id: "base_link".into() };
        let p = header_points(&tf, &h, std::iter::once((0.0, 0.0))).unwrap();
        assert!((p[0].0 - 0.5).abs() < 1e-9, "interpolated at the stamp {:?}", p[0]);
        let h = Header { stamp: r2r::builtin_interfaces::msg::Time { sec: 50, nanosec: 0 }, frame_id: "base_link".into() };
        assert!((header_points(&tf, &h, std::iter::once((0.0, 0.0))).unwrap()[0].0 - 1.0).abs() < 1e-9);
    }

    #[test]
    fn time_jumping_back_resets_tf_but_keeps_statics() {
        let mut tf = Tf::new();
        tf.insert("base_link", "laser", Stamp(0), t(0.1, 0.0), true);
        assert!(!tf.insert("map", "base_link", Stamp(100 * S), t(5.0, 0.0), false));
        assert!(!tf.insert("map", "base_link", Stamp(101 * S), t(6.0, 0.0), false));
        assert!(tf.insert("map", "base_link", Stamp(10 * S), t(1.0, 0.0), false));
        assert!((tf.to_map("base_link").unwrap().translation.x - 1.0).abs() < 1e-9);
        assert!((tf.to_map("laser").unwrap().translation.x - 1.1).abs() < 1e-9);
        assert!(!tf.insert("map", "base_link", Stamp(10 * S - S / 2), t(0.9, 0.0), false));
    }

    #[test]
    fn chain_age_is_the_stalest_edge() {
        let mut tf = Tf::new();
        tf.insert("map", "odom", Stamp(5 * S), t(0.0, 0.0), false);
        tf.insert("odom", "base_link", Stamp(9 * S), t(0.0, 0.0), false);
        tf.insert("base_link", "laser", Stamp(0), t(0.0, 0.0), true);
        assert_eq!(tf.chain_stamp("base_link"), Some(5.0));
        assert_eq!(tf.chain_stamp("laser"), Some(5.0));
        assert_eq!(tf.chain_stamp("nowhere"), None);
    }
}
