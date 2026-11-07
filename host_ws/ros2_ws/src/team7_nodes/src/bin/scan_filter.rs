use std::cell::RefCell;
use std::rc::Rc;

use futures::StreamExt;
use futures::executor::LocalPool;
use futures::task::LocalSpawnExt;
use r2r::QosProfile;
use r2r::nav_msgs::msg::Odometry;
use r2r::sensor_msgs::msg::LaserScan;
use r2r::{log_info, log_warn};
use tb4_scan::{Scan, ScanParams};
use team7_nodes::Params;

#[derive(Default)]
struct State {
    v: f64,
    w: f64,
    have_odom: bool,
    scans: u64,
    removed: u64,
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "scan_filter", "")?;
    let log = node.logger().to_owned();

    let p = Params(&node);
    let sp = ScanParams {
        min_range: p.f64("min_range", 0.18) as f32,
        max_range: p.f64("max_range", 8.0) as f32,
        median_window: p.i64("median_window", 3) as usize,
        isolated_dist: p.f64("isolated_dist", 0.15) as f32,
        deskew: p.bool("deskew", true),
        stamp_at_end: p.bool("stamp_at_end", false),
    };
    let in_topic = p.string("scan_topic", "/scan");
    let out_topic = p.string("output_topic", "/scan_filtered");
    let odom_topic = p.string("odom_topic", "/odom");
    log_info!(&log, "{in_topic} -> {out_topic} (deskew {})", sp.deskew);

    let pool = LocalPool::new();
    let spawner = pool.spawner();
    let state = Rc::new(RefCell::new(State::default()));

    let odom = node.subscribe::<Odometry>(&odom_topic, QosProfile::default())?;
    let st = state.clone();
    spawner.spawn_local(odom.for_each(move |m| {
        let mut s = st.borrow_mut();
        s.v = m.twist.twist.linear.x;
        s.w = m.twist.twist.angular.z;
        s.have_odom = true;
        futures::future::ready(())
    }))?;

    let publisher = node.create_publisher::<LaserScan>(&out_topic, QosProfile::sensor_data())?;
    let scans = node.subscribe::<LaserScan>(&in_topic, QosProfile::sensor_data())?;
    let st = state.clone();
    spawner.spawn_local(scans.for_each(move |mut m| {
        let mut s = st.borrow_mut();
        let scan = Scan {
            angle_min: m.angle_min as f64,
            angle_increment: m.angle_increment as f64,
            time_increment: m.time_increment as f64,
            ranges: std::mem::take(&mut m.ranges),
        };
        let scan = if sp.deskew && s.have_odom { tb4_scan::deskew(&scan, s.v, s.w, sp.stamp_at_end) } else { scan };
        let (out, stats) = tb4_scan::filter(&scan, &sp);
        if m.intensities.len() == out.ranges.len() {
            for (i, r) in m.intensities.iter_mut().zip(&out.ranges) {
                if !r.is_finite() {
                    *i = 0.0;
                }
            }
        } else {
            m.intensities.clear();
        }
        m.ranges = out.ranges;
        m.range_min = sp.min_range;
        m.range_max = sp.max_range;
        s.scans += 1;
        s.removed += (stats.out_of_range + stats.isolated) as u64;
        if s.scans % 500 == 0 {
            log_info!(&log, "{} scans, {:.1} returns removed per scan", s.scans, s.removed as f64 / s.scans as f64);
        }
        if let Err(e) = publisher.publish(&m) {
            log_warn!(&log, "publish failed: {e}");
        }
        futures::future::ready(())
    }))?;

    team7_nodes::run(node, pool, || false);
    Ok(())
}
