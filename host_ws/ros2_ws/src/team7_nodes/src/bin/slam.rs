use std::cell::RefCell;
use std::rc::Rc;
use std::time::{Duration, Instant};

use futures::StreamExt;
use futures::executor::LocalPool;
use futures::task::LocalSpawnExt;
use r2r::geometry_msgs::msg::{Point, TransformStamped};
use r2r::nav_msgs::msg::{MapMetaData, OccupancyGrid};
use r2r::sensor_msgs::msg::LaserScan;
use r2r::tf2_msgs::msg::TFMessage;
use r2r::visualization_msgs::msg::MarkerArray;
use r2r::{QosProfile, log_info, log_warn};
use tb4_core::nalgebra::Point3;
use tb4_core::{Grid, Pose2, TfBuffer};
use tb4_slam::{Slam, SlamParams};
use team7_nodes::{Params, pose_msg, spawn_tf_listener, stamp, viz};

struct State {
    slam: Slam,
    map_dirty: bool,
    last_map: Instant,
    last_stamp: r2r::builtin_interfaces::msg::Time,
    tf_failures: u64,
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "slam", "")?;
    let log = node.logger().to_owned();
    let p = Params(&node);

    let d = SlamParams::default();
    let sp = SlamParams {
        resolution: p.f64("resolution", d.resolution),
        min_travel_dist: p.f64("min_travel_dist", d.min_travel_dist),
        min_travel_yaw: p.f64("min_travel_yaw", d.min_travel_yaw),
        max_range: p.f64("max_range", d.max_range),
        min_match_score: p.f64("min_match_score", d.min_match_score),
        loop_search_radius: p.f64("loop_search_radius", d.loop_search_radius),
        loop_min_score: p.f64("loop_min_score", d.loop_min_score),
        ..d
    };
    let scan_topic = p.string("scan_topic", "/scan");
    let map_frame = p.string("map_frame", "map");
    let odom_frame = p.string("odom_frame", "odom");
    let base_frame = p.string("base_frame", "base_link");
    let map_period = Duration::from_secs_f64(p.f64("map_publish_period", 1.0));
    let tf_period = Duration::from_secs_f64(p.f64("transform_publish_period", 0.05));
    log_info!(&log, "own slam on {scan_topic}, map every {:.1}s", map_period.as_secs_f64());

    let pool = LocalPool::new();
    let spawner = pool.spawner();
    let tf = Rc::new(RefCell::new(TfBuffer::new(10.0, 0.1)));
    spawn_tf_listener(&mut node, &spawner, tf.clone(), "/tf", "/tf_static")?;

    let state = Rc::new(RefCell::new(State {
        slam: Slam::new(sp),
        map_dirty: false,
        last_map: Instant::now(),
        last_stamp: Default::default(),
        tf_failures: 0,
    }));

    let scans = node.subscribe::<LaserScan>(&scan_topic, QosProfile::sensor_data())?;
    let st = state.clone();
    let tfb = tf.clone();
    let (log2, of, bf) = (log.clone(), odom_frame.clone(), base_frame.clone());
    spawner.spawn_local(scans.for_each(move |m| {
        let t = stamp(&m.header.stamp);
        let b = tfb.borrow();
        let (odom, base_t_laser) = match (b.lookup(&of, &bf, t), b.lookup(&bf, &m.header.frame_id, t)) {
            (Ok(o), Ok(l)) => (Pose2::from_isometry(&o), l),
            (Err(e), _) | (_, Err(e)) => {
                let mut s = st.borrow_mut();
                s.tf_failures += 1;
                if s.tf_failures % 50 == 1 {
                    log_warn!(&log2, "tf for scan: {e} ({} failures)", s.tf_failures);
                }
                return futures::future::ready(());
            }
        };
        drop(b);
        let pts: Vec<(f64, f64)> = m
            .ranges
            .iter()
            .enumerate()
            .filter(|(_, r)| r.is_finite() && **r >= m.range_min)
            .map(|(i, r)| {
                let a = m.angle_min as f64 + i as f64 * m.angle_increment as f64;
                let q = base_t_laser * Point3::new(*r as f64 * a.cos(), *r as f64 * a.sin(), 0.0);
                (q.x, q.y)
            })
            .collect();
        let mut s = st.borrow_mut();
        let before = s.slam.stats().loop_closures;
        if s.slam.update(odom, &pts).is_some() {
            s.map_dirty = true;
            s.last_stamp = m.header.stamp.clone();
            let stats = s.slam.stats();
            if stats.loop_closures > before {
                log_info!(&log2, "loop closure #{} ({} keyframes)", stats.loop_closures, stats.keyframes);
            }
            if stats.scans_processed % 200 == 0 {
                log_info!(&log2, "{} scans, mean {:.1} ms, max {:.1} ms, {} fallbacks",
                    stats.scans_processed, stats.mean_update_ms, stats.max_update_ms, stats.match_fallbacks);
            }
        }
        futures::future::ready(())
    }))?;

    let map_pub = node.create_publisher::<OccupancyGrid>("/map", QosProfile::default().keep_last(1).reliable().transient_local())?;
    let graph_pub = node.create_publisher::<MarkerArray>("/team7/slam/graph", QosProfile::default())?;
    let tf_pub = node.create_publisher::<TFMessage>("/tf", QosProfile::default().keep_last(100))?;
    let clock = node.get_ros_clock();
    let mut timer = node.create_wall_timer(tf_period)?;
    let st = state.clone();
    spawner.spawn_local(async move {
        loop {
            if timer.tick().await.is_err() {
                return;
            }
            let now = clock.lock().unwrap().get_now().map(|d| r2r::Clock::to_builtin_time(&d)).unwrap_or_default();
            let mut s = st.borrow_mut();
            let m2o = s.slam.map_to_odom();
            // stamped slightly ahead like slam_toolbox so lookups at scan time succeed
            let mut ahead = now.clone();
            ahead.nanosec += 100_000_000;
            if ahead.nanosec >= 1_000_000_000 {
                ahead.nanosec -= 1_000_000_000;
                ahead.sec += 1;
            }
            let mut t = TransformStamped::default();
            t.header.stamp = ahead;
            t.header.frame_id = map_frame.clone();
            t.child_frame_id = odom_frame.clone();
            let pm = pose_msg(m2o.x, m2o.y, m2o.yaw);
            t.transform.translation.x = pm.position.x;
            t.transform.translation.y = pm.position.y;
            t.transform.rotation = pm.orientation;
            let _ = tf_pub.publish(&TFMessage { transforms: vec![t] });

            if s.map_dirty && s.last_map.elapsed() >= map_period {
                s.map_dirty = false;
                s.last_map = Instant::now();
                let _ = map_pub.publish(&grid_msg(&s.slam.occupancy(), &map_frame, &now));
                let _ = graph_pub.publish(&graph_markers(&s.slam, &map_frame));
            }
        }
    })?;

    team7_nodes::run(node, pool, || false);
    Ok(())
}

fn grid_msg(g: &Grid, frame: &str, now: &r2r::builtin_interfaces::msg::Time) -> OccupancyGrid {
    let mut m = OccupancyGrid::default();
    m.header.frame_id = frame.to_owned();
    m.header.stamp = now.clone();
    m.info = MapMetaData {
        map_load_time: now.clone(),
        resolution: g.meta.resolution as f32,
        width: g.meta.width,
        height: g.meta.height,
        origin: pose_msg(g.meta.origin_x, g.meta.origin_y, 0.0),
    };
    m.data = g.data.clone();
    m
}

fn graph_markers(slam: &Slam, frame: &str) -> MarkerArray {
    let kf = slam.keyframes();
    let pt = |p: &Pose2| Point { x: p.x, y: p.y, z: 0.05 };
    let mut nodes = viz::base(frame, "slam_keyframes", 0, viz::POINTS);
    nodes.scale.x = 0.06;
    nodes.scale.y = 0.06;
    nodes.color = viz::color(0.9, 0.5, 0.1, 0.9);
    nodes.points = kf.iter().map(pt).collect();
    let mut chain = viz::base(frame, "slam_odom_edges", 0, LINE_STRIP);
    chain.scale.x = 0.015;
    chain.color = viz::color(0.9, 0.5, 0.1, 0.5);
    chain.points = nodes.points.clone();
    let mut loops = viz::base(frame, "slam_loop_edges", 0, LINE_LIST);
    loops.scale.x = 0.03;
    loops.color = viz::color(1.0, 0.1, 0.8, 0.9);
    loops.points = slam
        .loop_edges()
        .iter()
        .filter(|(a, b)| *a < kf.len() && *b < kf.len())
        .flat_map(|(a, b)| [pt(&kf[*a]), pt(&kf[*b])])
        .collect();
    MarkerArray { markers: vec![viz::delete_all(frame), nodes, chain, loops] }
}

const LINE_STRIP: i32 = 4;
const LINE_LIST: i32 = 5;
