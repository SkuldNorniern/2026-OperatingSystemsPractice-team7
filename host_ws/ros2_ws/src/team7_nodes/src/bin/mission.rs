use std::cell::RefCell;
use std::collections::BTreeSet;
use std::path::{Path, PathBuf};
use std::process::{Child, Command, Stdio};
use std::rc::Rc;
use std::time::{Duration, Instant};

use futures::StreamExt;
use futures::executor::LocalPool;
use futures::task::LocalSpawnExt;
use r2r::action_msgs::srv::CancelGoal;
use r2r::visualization_msgs::msg::MarkerArray;
use r2r::{QosProfile, log_error, log_info, log_warn};
use team7_nodes::Params;

const LATCHED: QosProfile = QosProfile::default().keep_last(1).reliable().transient_local();

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "mission", "")?;
    let log = node.logger().to_owned();

    let p = Params(&node);
    let expected = p.i64("expected_targets", 3) as usize;
    let time_limit = Duration::from_secs_f64(p.f64("time_limit", 590.0));
    let autosave_period = Duration::from_secs_f64(p.f64("autosave_period", 60.0));
    let out_dir = team7_nodes::expand_home(&p.string("output_dir", "~/team7_run"));
    let map_name = p.string("map_name", "map");
    let map_topic = p.string("map_topic", "/map");
    let nav_action = p.string("nav_action", "/navigate_to_pose");
    std::fs::create_dir_all(&out_dir)?;

    let pub_state = node.create_publisher::<r2r::std_msgs::msg::String>("/team7/mission_state", LATCHED)?;
    let pub_resume = node.create_publisher::<r2r::std_msgs::msg::Bool>("/explore/resume", QosProfile::default())?;
    let cancel = node.create_client::<CancelGoal::Service>(
        &format!("{nav_action}/_action/cancel_goal"),
        QosProfile::services_default(),
    )?;

    let found: Rc<RefCell<BTreeSet<i32>>> = Rc::default();
    let targets = node.subscribe::<MarkerArray>("/team7/targets", QosProfile::default())?;
    let f = found.clone();
    let pool = LocalPool::new();
    let spawner = pool.spawner();
    team7_nodes::spawn_param_services(&mut node, &spawner)?;
    spawner.spawn_local(targets.for_each(move |m| {
        let mut f = f.borrow_mut();
        for mk in m.markers.iter().filter(|mk| mk.ns == "target_confirmed") {
            f.insert(mk.id);
        }
        futures::future::ready(())
    }))?;

    let mut timer = node.create_wall_timer(Duration::from_millis(500))?;
    spawner.spawn_local(async move {
        let start = Instant::now();
        let mut last_save = start;
        let mut saver: Option<Child> = None;
        let _ = pub_state.publish(&r2r::std_msgs::msg::String { data: "RUNNING".into() });
        log_info!(&log, "mission started: {expected} targets, {:.0} s limit, output {}", time_limit.as_secs_f64(), out_dir.display());
        let mut last_count = 0;
        loop {
            if timer.tick().await.is_err() {
                return;
            }
            let elapsed = start.elapsed();
            let n = found.borrow().len();
            if n != last_count {
                log_info!(&log, "{n}/{expected} targets confirmed at {:.1} s: {:?}", elapsed.as_secs_f64(), found.borrow());
                last_count = n;
            }
            if let Some(c) = saver.as_mut()
                && let Ok(Some(_)) = c.try_wait()
            {
                saver = None;
            }
            if n >= expected || elapsed >= time_limit {
                let why = if n >= expected { "all targets found" } else { "time limit" };
                log_info!(&log, "DONE ({why}) after {:.1} s with {n}/{expected} targets", elapsed.as_secs_f64());
                let _ = pub_state.publish(&r2r::std_msgs::msg::String { data: "DONE".into() });
                let _ = pub_resume.publish(&r2r::std_msgs::msg::Bool { data: false });
                // empty goal_info cancels all goals. no await, it hangs if nav2 is down
                if let Err(e) = cancel.request(&CancelGoal::Request::default()) {
                    log_warn!(&log, "cancel_goal request failed: {e}");
                }
                if let Some(mut c) = saver.take() {
                    let _ = c.wait();
                }
                match save_map(&out_dir, &map_name, &map_topic).and_then(|mut c| c.wait()) {
                    Ok(s) if s.success() => log_info!(&log, "map saved to {}", out_dir.join(&map_name).display()),
                    Ok(s) => log_error!(&log, "map_saver_cli exited with {s}"),
                    Err(e) => log_error!(&log, "map_saver_cli failed to start: {e}"),
                }
                log_info!(&log, "targets: {}", out_dir.join("targets.csv").display());
                return;
            }
            if saver.is_none() && last_save.elapsed() >= autosave_period {
                last_save = Instant::now();
                match save_map(&out_dir, &format!("{map_name}_autosave"), &map_topic) {
                    Ok(c) => saver = Some(c),
                    Err(e) => log_warn!(&log, "autosave failed to start: {e}"),
                }
            }
        }
    })?;

    team7_nodes::run(node, pool, || false);
    Ok(())
}

fn save_map(dir: &Path, name: &str, map_topic: &str) -> std::io::Result<Child> {
    let path: PathBuf = dir.join(name);
    Command::new("ros2")
        .args(["run", "nav2_map_server", "map_saver_cli", "-f"])
        .arg(&path)
        .args(["-t", map_topic, "--ros-args", "-p", "save_map_timeout:=10.0"])
        .args(["-p", "map_subscribe_transient_local:=true", "--log-level", "warn"])
        .stdin(Stdio::null())
        .spawn()
}
