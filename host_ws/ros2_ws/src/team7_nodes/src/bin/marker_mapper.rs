use std::cell::RefCell;
use std::path::PathBuf;
use std::rc::Rc;

use futures::StreamExt;
use futures::executor::LocalPool;
use futures::task::LocalSpawnExt;
use r2r::QosProfile;
use r2r::aruco_opencv_msgs::msg::ArucoDetection;
use r2r::nav_msgs::msg::OccupancyGrid;
use r2r::visualization_msgs::msg::MarkerArray;
use r2r::{log_info, log_warn};
use team7_nodes::{Params, SharedTf, grid_from_msg, spawn_tf_listener, stamp, viz};
use tb4_core::nalgebra::Point3;
use tb4_core::{Grid, TfBuffer};
use tb4_markers::{LocateParams, Observation, Source, Target, TrackParams, Tracker};

struct State {
    grid: Option<Grid>,
    tracker: Tracker,
    locate: LocateParams,
    map_frame: String,
    ignore_ids: Vec<i64>,
    csv_path: PathBuf,
    last_csv: String,
    tf_failures: u64,
    stats: [u64; 2],
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "marker_mapper", "")?;
    let log = node.logger().to_owned();

    let (state, detections_topic, map_topic, tf_topic, tf_static_topic) = {
        let p = Params(&node);
        let locate = LocateParams {
            max_range: p.f64("max_range", 4.0),
            occupied_threshold: p.i64("occupied_threshold", 65) as i8,
            range_ratio_gate: p.f64("range_ratio_gate", 3.0),
            allow_pnp_fallback: p.bool("allow_pnp_fallback", true),
            max_abs_bearing_z: p.f64("max_abs_bearing_z", 0.8),
        };
        let track = TrackParams {
            min_views: p.i64("min_views", 5) as usize,
            max_spread: p.f64("max_spread", 0.25),
            max_history: p.i64("max_history", 200) as usize,
        };
        let out_dir = team7_nodes::expand_home(&p.string("output_dir", "~/team7_run"));
        let state = State {
            grid: None,
            tracker: Tracker::new(track),
            locate,
            map_frame: p.string("map_frame", "map"),
            ignore_ids: p.i64_list("ignore_ids"),
            csv_path: out_dir.join(p.string("csv_name", "targets.csv")),
            last_csv: String::new(),
            tf_failures: 0,
            stats: [0, 0],
        };
        (
            Rc::new(RefCell::new(state)),
            p.string("detections_topic", "/aruco_detections"),
            p.string("map_topic", "/map"),
            p.string("tf_topic", "/tf"),
            p.string("tf_static_topic", "/tf_static"),
        )
    };
    log_info!(&log, "writing confirmed targets to {}", state.borrow().csv_path.display());

    let pool = LocalPool::new();
    let spawner = pool.spawner();
    let tf: SharedTf = Rc::new(RefCell::new(TfBuffer::new(10.0, 0.1)));
    spawn_tf_listener(&mut node, &spawner, tf.clone(), &tf_topic, &tf_static_topic)?;

    let map_sub = node.subscribe::<OccupancyGrid>(&map_topic, QosProfile::default().reliable().transient_local())?;
    let st = state.clone();
    spawner.spawn_local(map_sub.for_each(move |m| {
        st.borrow_mut().grid = grid_from_msg(&m);
        futures::future::ready(())
    }))?;

    let pub_targets = node.create_publisher::<MarkerArray>("/team7/targets", QosProfile::default())?;
    let det_sub = node.subscribe::<ArucoDetection>(&detections_topic, QosProfile::sensor_data())?;
    let st = state.clone();
    let log2 = log.clone();
    spawner.spawn_local(det_sub.for_each(move |det| {
        let mut s = st.borrow_mut();
        if det.markers.is_empty() {
            return futures::future::ready(());
        }
        let cam = match tf.borrow().lookup(&s.map_frame, &det.header.frame_id, stamp(&det.header.stamp)) {
            Ok(t) => t,
            Err(e) => {
                s.tf_failures += 1;
                if s.tf_failures % 50 == 1 {
                    log_warn!(&log2, "tf {} <- {}: {e} ({} failures)", s.map_frame, det.header.frame_id, s.tf_failures);
                }
                return futures::future::ready(());
            }
        };
        let origin = cam * Point3::origin();
        let before: Vec<i64> = s.tracker.confirmed().iter().map(|t| t.id).collect();
        for m in &det.markers {
            let id = m.marker_id as i64;
            if s.ignore_ids.contains(&id) {
                continue;
            }
            let p = &m.pose.position;
            let t_cam = tb4_core::Vector3::new(p.x, p.y, p.z);
            let range = t_cam.norm();
            if range < 1e-6 {
                continue;
            }
            let b = cam.rotation * (t_cam / range);
            let obs = Observation { id, camera: (origin.x, origin.y, origin.z), bearing: (b.x, b.y, b.z), pnp_range: range };
            let est = tb4_markers::locate(s.grid.as_ref(), &obs, &s.locate);
            if let Some(e) = est {
                s.stats[(e.source == Source::PnpRange) as usize] += 1;
                s.tracker.add(id, &e);
            }
        }
        let targets = s.tracker.targets();
        for t in targets.iter().filter(|t| t.confirmed && !before.contains(&t.id)) {
            log_info!(&log2, "CONFIRMED target id={} at ({:.2}, {:.2}) from {} views", t.id, t.x, t.y, t.views);
        }
        let confirmed: Vec<Target> = targets.iter().filter(|t| t.confirmed).cloned().collect();
        let csv = tb4_markers::to_csv(&confirmed);
        if csv != s.last_csv {
            match team7_nodes::write_atomic(&s.csv_path, &csv) {
                Ok(()) => s.last_csv = csv,
                Err(e) => log_warn!(&log2, "cannot write {}: {e}", s.csv_path.display()),
            }
        }
        let _ = pub_targets.publish(&markers(&s.map_frame, &targets));
        futures::future::ready(())
    }))?;

    team7_nodes::run(node, pool, || false);
    Ok(())
}

fn markers(frame: &str, targets: &[Target]) -> MarkerArray {
    let mut out = vec![viz::delete_all(frame)];
    for t in targets {
        let (ns, c) = if t.confirmed {
            ("target_confirmed", viz::color(0.1, 0.9, 0.2, 0.9))
        } else {
            ("target_tentative", viz::color(1.0, 0.8, 0.0, 0.6))
        };
        let mut s = viz::base(frame, ns, t.id as i32, viz::SPHERE);
        s.pose.position.x = t.x;
        s.pose.position.y = t.y;
        s.pose.position.z = 0.15;
        s.scale.x = 0.2;
        s.scale.y = 0.2;
        s.scale.z = 0.2;
        s.color = c.clone();
        let mut txt = viz::base(frame, &format!("{ns}_label"), t.id as i32, viz::TEXT_VIEW_FACING);
        txt.pose.position.x = t.x;
        txt.pose.position.y = t.y;
        txt.pose.position.z = 0.45;
        txt.scale.z = 0.25;
        txt.color = viz::color(1.0, 1.0, 1.0, 1.0);
        txt.text = if t.confirmed {
            format!("ID {} ({:.2}, {:.2})", t.id, t.x, t.y)
        } else {
            format!("ID {}? {}v", t.id, t.views)
        };
        out.push(s);
        out.push(txt);
    }
    MarkerArray { markers: out }
}
