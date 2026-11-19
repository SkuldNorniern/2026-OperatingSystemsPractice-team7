//! The ROS side: a read-only node on its own thread. It subscribes, puts everything in the map frame
//! with TF, and writes into the shared `Model`. It never publishes on the team's topics.

use std::cell::RefCell;
use std::rc::Rc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use futures::executor::LocalPool;
use futures::stream::{Stream, StreamExt};
use futures::task::LocalSpawnExt;
use r2r::QosProfile;
use r2r::geometry_msgs::msg::{PoseStamped, Transform};
use r2r::nav_msgs::msg::{OccupancyGrid, Odometry, Path};
use r2r::sensor_msgs::msg::LaserScan;
use r2r::std_msgs::msg::Header;
use r2r::tf2_msgs::msg::TFMessage;
use r2r::visualization_msgs::msg::MarkerArray;
use tb4_core::nalgebra::{Point3, Quaternion, Translation3, UnitQuaternion};
use tb4_core::{GridMeta, Isometry3, Pose2, Stamp, TfBuffer};

use crate::model::{MapData, Model, Target, Topic};

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

/// the latest /team7/targets array -> targets (sphere markers; labels and other namespaces ignored)
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
            out.push(Target { id: mk.id, x: mk.pose.position.x, y: mk.pose.position.y, confirmed });
        }
    }
    out.sort_by_key(|t| t.id);
    out
}

/// frame -> map with the latest TF, as a 3D isometry (the laser may be mounted rolled)
struct Tf {
    buf: TfBuffer,
}

impl Tf {
    fn to_map(&self, frame: &str) -> Result<Isometry3<f64>, String> {
        let frame = if frame.is_empty() { MAP_FRAME } else { frame };
        self.buf.lookup(MAP_FRAME, frame, Stamp::LATEST).map_err(|e| format!("TF {frame} -> map: {e}"))
    }

    fn points(&self, frame: &str, pts: impl Iterator<Item = (f64, f64)>) -> Result<Vec<(f64, f64)>, String> {
        let t = self.to_map(frame)?;
        Ok(pts.map(|(x, y)| t * Point3::new(x, y, 0.0)).map(|p| (p.x, p.y)).collect())
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

fn header_points<'a>(tf: &Tf, h: &Header, pts: impl Iterator<Item = (f64, f64)> + 'a) -> Result<Vec<(f64, f64)>, String> {
    tf.points(&h.frame_id, pts)
}

/// subscribe everything and spin until `stop`; sets `dirty` whenever the model changed
pub fn run(model: Shared, stop: Arc<AtomicBool>, dirty: Arc<AtomicBool>) -> r2r::Result<()> {
    let ctx_r2r = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx_r2r, "team7_viz", "")?;
    let pool = LocalPool::new();
    let tf = Rc::new(RefCell::new(Tf { buf: TfBuffer::new(10.0, 0.1) }));
    let ctx = Rc::new(Ctx { model, tf: tf.clone(), dirty });

    // TF first: everything else is put in the map frame with it
    for (topic, qos, is_static) in [("/tf", QosProfile::default().keep_last(100), false), ("/tf_static", LATCHED.keep_last(100), true)] {
        let s = node.subscribe::<TFMessage>(topic, qos)?;
        let tf = tf.clone();
        spawn(&pool, s, &ctx, move |c, msg: TFMessage| {
            {
                let mut t = tf.borrow_mut();
                for x in &msg.transforms {
                    let st = Stamp::from_ros(x.header.stamp.sec, x.header.stamp.nanosec);
                    t.buf.insert(&x.header.frame_id, &x.child_frame_id, st, iso(&x.transform), is_static);
                }
            }
            c.update(|m, tf| {
                m.count(if is_static { "tf_static" } else { "tf" });
                if !is_static {
                    m.tick(Topic::Tf, Instant::now());
                }
                if let Ok(t) = tf.to_map(BASE_FRAME) {
                    let p = Pose2::from_isometry(&t);
                    m.robot = Some((p.x, p.y, p.yaw));
                }
            });
        });
    }

    let s = node.subscribe::<OccupancyGrid>("/map", QosProfile::default().reliable().transient_local())?;
    spawn(&pool, s, &ctx, |c, msg: OccupancyGrid| {
        c.update(|m, _| {
            m.count("map");
            m.tick(Topic::Map, Instant::now());
            let i = &msg.info;
            // the map frame is the drawn frame; a rotated grid origin is not supported (slam publishes none)
            if i.width > 0 && i.height > 0 && msg.data.len() == i.width as usize * i.height as usize {
                let meta = GridMeta {
                    width: i.width,
                    height: i.height,
                    resolution: f64::from(i.resolution),
                    origin_x: i.origin.position.x,
                    origin_y: i.origin.position.y,
                };
                m.map = Some(MapData { meta, data: msg.data });
                m.map_seq += 1;
            }
        });
    });

    let s = node.subscribe::<LaserScan>("/scan", QosProfile::sensor_data())?;
    spawn(&pool, s, &ctx, |c, msg: LaserScan| {
        c.update(|m, tf| {
            m.count("scan");
            m.tick(Topic::Scan, Instant::now());
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
            m.mission = Some(msg.data);
        });
    });

    let mut pool = pool;
    while !stop.load(Ordering::Relaxed) {
        node.spin_once(Duration::from_millis(10));
        pool.run_until_stalled();
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use r2r::visualization_msgs::msg::Marker;

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
        // index 3: 270 degrees
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
        let msg = MarkerArray {
            markers: vec![
                marker("", 0, DELETEALL, 0.0, 0.0),
                marker("target_confirmed", 7, 0, 1.0, 2.0),
                marker("target_confirmed_label", 7, 0, 1.0, 2.0),
                marker("target_tentative", 3, 0, -1.0, 0.5),
            ],
        };
        let old = [Target { id: 9, x: 0.0, y: 0.0, confirmed: true }];
        let t = targets_from(&msg, &old);
        assert_eq!(t, vec![Target { id: 3, x: -1.0, y: 0.5, confirmed: false }, Target { id: 7, x: 1.0, y: 2.0, confirmed: true }]);
        // without DELETEALL the array updates in place
        let up = MarkerArray { markers: vec![marker("target_confirmed", 3, 0, -1.1, 0.5)] };
        let t = targets_from(&up, &t);
        assert_eq!(t.len(), 2);
        assert!(t.iter().all(|t| t.confirmed));
    }

    #[test]
    fn points_go_through_tf_into_the_map_frame() {
        let mut buf = TfBuffer::new(10.0, 0.1);
        let t = |x: f64, yaw: f64| Isometry3::from_parts(Translation3::new(x, 0.0, 0.0), UnitQuaternion::from_euler_angles(0.0, 0.0, yaw));
        buf.insert("map", "odom", Stamp(1), t(1.0, 0.0), false);
        buf.insert("odom", "base_link", Stamp(1), t(0.5, std::f64::consts::FRAC_PI_2), false);
        // laser upside down: rolled by pi
        let rolled = Isometry3::from_parts(Translation3::new(0.0, 0.0, 0.2), UnitQuaternion::from_euler_angles(std::f64::consts::PI, 0.0, 0.0));
        buf.insert("base_link", "rplidar_link", Stamp(0), rolled, true);
        let tf = Tf { buf };
        let p = tf.points("rplidar_link", [(1.0, 0.0), (0.0, 1.0)].into_iter()).unwrap();
        // base at (1.5, 0) facing +y; laser +x is base +x (map +y), laser +y is base -y (map +x)
        assert!((p[0].0 - 1.5).abs() < 1e-9 && (p[0].1 - 1.0).abs() < 1e-9, "{:?}", p[0]);
        assert!((p[1].0 - 2.5).abs() < 1e-9 && p[1].1.abs() < 1e-9, "{:?}", p[1]);
        assert!(tf.points("camera", std::iter::empty()).is_err());
        assert_eq!(tf.points("", [(2.0, 3.0)].into_iter()).unwrap(), vec![(2.0, 3.0)]);
    }
}
