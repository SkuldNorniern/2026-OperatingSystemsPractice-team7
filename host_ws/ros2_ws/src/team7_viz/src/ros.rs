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
use futures::future::AbortHandle;
use futures::stream::{Stream, StreamExt};
use futures::task::LocalSpawnExt;
use r2r::QosProfile;
use r2r::aruco_opencv_msgs::msg::ArucoDetection;
use r2r::qos::{DurabilityPolicy, ReliabilityPolicy};
use r2r::geometry_msgs::msg::{PoseStamped, Transform};
use r2r::nav_msgs::msg::{OccupancyGrid, Odometry, Path};
use r2r::sensor_msgs::msg::{Image, LaserScan};
use r2r::std_msgs::msg::Header;
use r2r::tf2_msgs::msg::TFMessage;
use r2r::visualization_msgs::msg::{Marker, MarkerArray};
use tb4_core::nalgebra::{Point3, Quaternion, Translation3, UnitQuaternion};
use tb4_core::{GridMeta, Isometry3, Pose2, Stamp, TfBuffer};

use crate::model::{CameraFrame, Feed, GraphTopic, GridLayer, MapData, Mk, Model, MsgKind, Ray, ShapeKind, Slot, Topic};

pub const MAP_FRAME: &str = "map";
pub const BASE_FRAME: &str = "base_link";

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

/// a marker as the layers read it, points and position still in the marker's frame. Lists and
/// lines keep their points; a single sphere, cube or cylinder is its position; text and arrows
/// keep only id, namespace, position and text (the targets' labels)
pub fn mk_of(mk: &Marker) -> Option<Mk> {
    let kind = match mk.type_ {
        4 => ShapeKind::Strip,
        5 => ShapeKind::Segments,
        _ => ShapeKind::Points,
    };
    let pts: Vec<_> = match mk.type_ {
        4..=8 => {
            // the marker pose places its points (identity for all our markers)
            let p = iso_pose(&mk.pose);
            mk.points.iter().map(|q| p * Point3::new(q.x, q.y, q.z)).map(|q| (q.x, q.y)).collect()
        }
        1..=3 => vec![(mk.pose.position.x, mk.pose.position.y)],
        _ => Vec::new(),
    };
    let c = &mk.color;
    let to8 = |v: f32| (v.clamp(0.0, 1.0) * 255.0).round() as u8;
    Some(Mk {
        ns: mk.ns.clone(),
        id: mk.id,
        kind,
        pts,
        pos: (mk.pose.position.x, mk.pose.position.y),
        color: [to8(c.r), to8(c.g), to8(c.b), to8(c.a).max(160)],
        text: mk.text.clone(),
    })
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

    /// detections in a camera frame (optical: z ahead) -> map rays: camera origin to the marker
    fn rays_at(&self, frame: &str, stamp: Stamp, hits: impl Iterator<Item = (i32, [f64; 3])>, at: Instant) -> Result<Vec<Ray>, String> {
        let t = self.to_map_at(frame, stamp)?;
        let o = t * Point3::origin();
        Ok(hits
            .map(|(id, p)| {
                let q = t * Point3::new(p[0], p[1], p[2]);
                Ray { id, from: (o.x, o.y), to: (q.x, q.y), at }
            })
            .collect())
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
                    eprintln!("team7_viz: TF time jumped back, TF reset (replay restarted?)");
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

    let s = node.subscribe::<Odometry>("/odom", QosProfile::sensor_data())?;
    spawn(&pool, s, &ctx, |c, _msg: Odometry| {
        c.update(|m, _| {
            m.count("odom");
            m.tick(Topic::Odom, Instant::now());
        });
    });

    let s = node.subscribe::<r2r::std_msgs::msg::String>("/team7/mission_state", LATCHED)?;
    spawn(&pool, s, &ctx, |c, msg: r2r::std_msgs::msg::String| {
        c.update(|m, _| {
            m.count("mission_state");
            m.set_mission(msg.data, Instant::now());
        });
    });

    // goals: only with --allow-goals, to whichever NavigateToPose server is in the graph
    type Nav = r2r::nav2_msgs::action::NavigateToPose::Action;
    let mut clients = Vec::new();
    if opts.goals.is_some() {
        for &name in Slot::NavAction.candidates() {
            clients.push((name, node.create_action_client::<Nav>(name)?));
        }
    }

    let mut pool = pool;
    let mut subs = Subs::default();
    let mut last_graph = Instant::now() - GRAPH_EVERY;
    let mut last_wanted = Instant::now() - WANTED_EVERY;
    while !stop.load(Ordering::Relaxed) {
        node.spin_once(Duration::from_millis(5));
        if last_graph.elapsed() >= GRAPH_EVERY {
            last_graph = Instant::now();
            if let Ok(names) = node.get_topic_names_and_types() {
                let graph: Vec<GraphTopic> = names
                    .into_iter()
                    .map(|(name, types)| {
                        // our own subscriptions put topics in the graph too: count publishers
                        let publishers = node.get_publishers_info_by_topic(&name, false).map_or(0, |p| p.len());
                        GraphTopic { name, types, publishers }
                    })
                    .collect();
                ctx.update(|m, _| {
                    let mut g = graph;
                    g.sort_by(|a, b| a.name.cmp(&b.name));
                    m.graph = g;
                });
            }
        }
        // subscribe and drop topics as the window's layers ask
        if last_wanted.elapsed() >= WANTED_EVERY {
            last_wanted = Instant::now();
            let wanted = ctx.model.lock().map(|m| m.wanted.clone()).unwrap_or_default();
            subs.sync(&mut node, &pool, &ctx, &wanted);
        }
        if let Some(rx) = &opts.goals {
            while let Ok(goal) = rx.try_recv() {
                let bound = ctx.model.lock().ok().and_then(|m| m.bound(Slot::NavAction));
                match clients.iter().find(|(n, _)| Some(*n) == bound) {
                    Some((name, client)) => send_goal(client, name, &pool, &ctx, goal),
                    None => ctx.update(|m, _| m.goal_status = Some("no NavigateToPose server in the graph: goal not sent".into())),
                }
            }
        }
        pool.run_until_stalled();
    }
    Ok(())
}

/// how often the wanted subscriptions are compared with the live ones
const WANTED_EVERY: Duration = Duration::from_millis(100);

/// a viewer's QoS: take what the publisher offers (latched maps arrive at once, best-effort
/// sensor data still matches)
fn viewer_qos(depth: usize) -> QosProfile {
    let mut q = QosProfile::default().keep_last(depth);
    q.reliability = ReliabilityPolicy::BestAvailable;
    q.durability = DurabilityPolicy::BestAvailable;
    q
}

/// the live subscriptions of the layers, by topic and message kind
#[derive(Default)]
struct Subs {
    live: Vec<((String, MsgKind), AbortHandle)>,
}

impl Subs {
    /// subscribe what is wanted and not live, drop what is live and not wanted. A dropped stream
    /// makes r2r destroy the subscription at its next message.
    fn sync(&mut self, node: &mut r2r::Node, pool: &LocalPool, ctx: &Rc<Ctx>, wanted: &[(String, MsgKind)]) {
        self.live.retain(|(key, h)| {
            let keep = wanted.contains(key);
            if !keep {
                h.abort();
            }
            keep
        });
        for key in wanted {
            if self.live.iter().any(|(k, _)| k == key) {
                continue;
            }
            match subscribe_feed(node, pool, ctx, &key.0, key.1) {
                Ok(h) => self.live.push((key.clone(), h)),
                Err(e) => ctx.update(|m, _| m.tf_problem = Some((format!("cannot subscribe {}: {e}", key.0), Instant::now()))),
            }
        }
    }
}

/// spawn `f` on every message of `stream`, abortable
fn spawn_abortable<T: 'static>(pool: &LocalPool, stream: impl Stream<Item = T> + 'static, ctx: &Rc<Ctx>, mut f: impl FnMut(&Ctx, T) + 'static) -> AbortHandle {
    let ctx = ctx.clone();
    let (task, handle) = futures::future::abortable(stream.for_each(move |msg| {
        f(&ctx, msg);
        futures::future::ready(())
    }));
    pool.spawner().spawn_local(async move {
        let _ = task.await;
    }).expect("spawn subscriber");
    handle
}

/// one layer topic: decode its messages into the model's feed for that topic
fn subscribe_feed(node: &mut r2r::Node, pool: &LocalPool, ctx: &Rc<Ctx>, topic: &str, kind: MsgKind) -> r2r::Result<AbortHandle> {
    let t = topic.to_owned();
    Ok(match kind {
        MsgKind::Grid => {
            let s = node.subscribe::<OccupancyGrid>(topic, viewer_qos(2))?;
            spawn_abortable(pool, s, ctx, move |c, msg: OccupancyGrid| {
                c.update(|m, _| {
                    let now = Instant::now();
                    m.count(&t);
                    if t == "/map" {
                        m.tick(Topic::Map, now);
                    }
                    if let Some(g) = grid_of(msg)
                        && let Feed::Grid(layer) = m.feed_mut(&t, now, || Feed::Grid(GridLayer::default()))
                    {
                        layer.set(g);
                    }
                });
            })
        }
        MsgKind::Scan => {
            let s = node.subscribe::<LaserScan>(topic, viewer_qos(5))?;
            spawn_abortable(pool, s, ctx, move |c, msg: LaserScan| {
                c.update(|m, tf| {
                    let now = Instant::now();
                    m.count(&t);
                    if t == "/scan" {
                        m.tick(Topic::Scan, now);
                        let (ros_now, _) = m.ros_now(now, wall_secs());
                        let d = ros_now - secs(&msg.header.stamp);
                        m.scan_delay = Some(m.scan_delay.map_or(d, |old| 0.8 * old + 0.2 * d));
                    }
                    if let Some(p) = in_map(m, header_points(tf, &msg.header, scan_points(&msg))) {
                        m.put(&t, Feed::Points(p), now);
                    }
                });
            })
        }
        MsgKind::Path => {
            let s = node.subscribe::<Path>(topic, viewer_qos(5))?;
            spawn_abortable(pool, s, ctx, move |c, msg: Path| {
                c.update(|m, tf| {
                    m.count(&t);
                    let pts = msg.poses.iter().map(|p| (p.pose.position.x, p.pose.position.y));
                    if let Some(p) = in_map(m, header_points(tf, &msg.header, pts)) {
                        m.put(&t, Feed::Points(p), Instant::now());
                    }
                });
            })
        }
        MsgKind::Pose => {
            let s = node.subscribe::<PoseStamped>(topic, viewer_qos(5))?;
            spawn_abortable(pool, s, ctx, move |c, msg: PoseStamped| {
                c.update(|m, tf| {
                    m.count(&t);
                    let p = msg.pose.position;
                    if let Some(g) = in_map(m, header_points(tf, &msg.header, std::iter::once((p.x, p.y)))) {
                        m.put(&t, Feed::Pose(g.first().copied()), Instant::now());
                    }
                });
            })
        }
        MsgKind::Markers => {
            let s = node.subscribe::<MarkerArray>(topic, viewer_qos(10))?;
            spawn_abortable(pool, s, ctx, move |c, msg: MarkerArray| {
                c.update(|m, tf| {
                    m.count(&t);
                    // put each marker in the map frame first, then fold the array into the store
                    let mut parts = Vec::new();
                    for mk in &msg.markers {
                        let placed = mk_of(mk).and_then(|mut x| {
                            let pts = in_map(m, header_points(tf, &mk.header, x.pts.iter().copied().chain(std::iter::once(x.pos))))?;
                            let (pos, rest) = pts.split_last()?;
                            x.pos = *pos;
                            x.pts = rest.to_vec();
                            Some(x)
                        });
                        parts.push((mk.action, mk.ns.clone(), mk.id, placed));
                    }
                    if let Feed::Markers(store) = m.feed_mut(&t, Instant::now(), || Feed::Markers(Vec::new())) {
                        apply_markers(store, parts);
                    }
                });
            })
        }
        MsgKind::Image => {
            let s = node.subscribe::<Image>(topic, viewer_qos(2))?;
            spawn_abortable(pool, s, ctx, move |c, msg: Image| {
                let frame = image_rgba(&msg);
                c.update(|m, _| {
                    let now = Instant::now();
                    m.count(&t);
                    if let Some((width, height, rgba)) = frame {
                        let seq = m.image(&t).map_or(1, |f| f.seq + 1);
                        let frame = CameraFrame { width, height, rgba, seq, at: now };
                        if m.camera_topic.as_deref() == Some(t.as_str()) {
                            m.queue_camera(frame.clone());
                        }
                        m.put(&t, Feed::Image(frame), now);
                    }
                });
            })
        }
        MsgKind::Detections => {
            let s = node.subscribe::<ArucoDetection>(topic, viewer_qos(5))?;
            spawn_abortable(pool, s, ctx, move |c, msg: ArucoDetection| {
                c.update(|m, tf| {
                    let now = Instant::now();
                    m.count(&t);
                    let h = &msg.header;
                    let hits = msg.markers.iter().map(|k| (i32::from(k.marker_id), [k.pose.position.x, k.pose.position.y, k.pose.position.z]));
                    match tf.rays_at(&h.frame_id, Stamp::from_ros(h.stamp.sec, h.stamp.nanosec), hits, now) {
                        Ok(r) => m.add_rays(&t, r, now),
                        Err(e) => m.tf_problem = Some((e, now)),
                    }
                });
            })
        }
    })
}

/// fold one MarkerArray into the stored markers: DELETEALL clears, ADD replaces by (ns, id),
/// DELETE removes
pub fn apply_markers(store: &mut Vec<Mk>, parts: Vec<(i32, String, i32, Option<Mk>)>) {
    for (action, ns, id, mk) in parts {
        match action {
            DELETEALL => store.clear(),
            2 => store.retain(|s| !(s.ns == ns && s.id == id)),
            _ => {
                store.retain(|s| !(s.ns == ns && s.id == id));
                if let Some(mk) = mk {
                    store.push(mk);
                }
            }
        }
    }
}

/// how often the ROS graph is read to bind layers to topics
const GRAPH_EVERY: Duration = Duration::from_secs(2);

fn send_goal(client: &r2r::ActionClient<r2r::nav2_msgs::action::NavigateToPose::Action>, name: &str, pool: &LocalPool, ctx: &Rc<Ctx>, (x, y, yaw): (f64, f64, f64)) {
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
            say(ctx, format!("goal ({x:.2}, {y:.2}) sent to {name}"));
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
    fn marker_arrays_fold_into_the_store() {
        let mut label = marker("target_tentative_label", 3, 0, -1.0, 0.5);
        label.type_ = 9;
        label.text = "ID 3? 2v".into();
        let mut sphere = marker("target_confirmed", 7, 0, 1.0, 2.0);
        sphere.type_ = 2;
        let mut tent = marker("target_tentative", 3, 0, -1.0, 0.5);
        tent.type_ = 2;
        let parts = |ms: Vec<Marker>| ms.iter().map(|m| (m.action, m.ns.clone(), m.id, mk_of(m))).collect::<Vec<_>>();
        let mut store = vec![mk_of(&marker("old", 9, 0, 0.0, 0.0)).unwrap()];
        apply_markers(&mut store, parts(vec![marker("", 0, DELETEALL, 0.0, 0.0), sphere.clone(), tent, label]));
        assert_eq!(store.len(), 3);
        let t = crate::model::targets_of(&store);
        assert_eq!(t.iter().map(|t| (t.id, t.confirmed, t.views)).collect::<Vec<_>>(), vec![(3, false, Some(2)), (7, true, None)]);
        assert_eq!(store.iter().find(|m| m.id == 7).unwrap().pts, vec![(1.0, 2.0)]);
        // ADD replaces by (ns, id), DELETE removes
        let mut moved = sphere.clone();
        moved.pose.position.x = 5.0;
        apply_markers(&mut store, parts(vec![moved]));
        assert_eq!(store.iter().filter(|m| m.ns == "target_confirmed").count(), 1);
        assert_eq!(store.iter().find(|m| m.id == 7).unwrap().pos.0, 5.0);
        let mut del = sphere;
        del.action = 2;
        apply_markers(&mut store, parts(vec![del]));
        assert!(store.iter().all(|m| m.ns != "target_confirmed"));
    }

    #[test]
    fn slam_graph_markers_become_shapes() {
        let mut mk = marker("slam_loop_edges", 0, 0, 0.0, 0.0);
        mk.type_ = 5;
        mk.pose.orientation.w = 1.0;
        mk.points = vec![r2r::geometry_msgs::msg::Point { x: 1.0, y: 2.0, z: 0.0 }, r2r::geometry_msgs::msg::Point { x: 3.0, y: 4.0, z: 0.0 }];
        mk.color = r2r::std_msgs::msg::ColorRGBA { r: 1.0, g: 0.1, b: 0.8, a: 0.9 };
        let s = mk_of(&mk).unwrap();
        assert_eq!((s.kind, s.pts.clone()), (ShapeKind::Segments, vec![(1.0, 2.0), (3.0, 4.0)]));
        assert_eq!(s.color, [255, 26, 204, 230]);
        mk.type_ = 9; // text: no points, the label is kept
        assert!(mk_of(&mk).unwrap().pts.is_empty());
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
    fn detections_in_the_optical_frame_become_map_rays() {
        let mut tf = Tf::new();
        tf.insert("map", "base_link", Stamp(1), t(1.0, std::f64::consts::FRAC_PI_2), false);
        // the usual optical frame: z ahead (base x), x right (base -y), y down (base -z)
        let optical = UnitQuaternion::from_euler_angles(-std::f64::consts::FRAC_PI_2, 0.0, -std::f64::consts::FRAC_PI_2);
        tf.insert("base_link", "oakd_rgb_camera_optical_frame", Stamp(0), Isometry3::from_parts(Translation3::new(0.1, 0.0, 0.2), optical), true);
        let at = Instant::now();
        let r = tf.rays_at("oakd_rgb_camera_optical_frame", Stamp::LATEST, [(5, [0.0, 0.0, 2.0])].into_iter(), at).unwrap();
        // the robot at (1, 0) faces +y: the camera is 0.1 m ahead, a marker 2 m ahead of it
        assert_eq!(r[0].id, 5);
        assert!((r[0].from.0 - 1.0).abs() < 1e-9 && (r[0].from.1 - 0.1).abs() < 1e-9, "{:?}", r[0].from);
        assert!((r[0].to.0 - 1.0).abs() < 1e-9 && (r[0].to.1 - 2.1).abs() < 1e-9, "{:?}", r[0].to);
        assert!(tf.rays_at("nowhere", Stamp::LATEST, std::iter::empty(), at).is_err());
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
