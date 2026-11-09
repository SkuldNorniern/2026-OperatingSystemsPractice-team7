use std::cell::RefCell;
use std::rc::Rc;
use std::time::{Duration, Instant};

use futures::StreamExt;
use futures::executor::LocalPool;
use futures::task::LocalSpawnExt;
use r2r::nav2_msgs::action::NavigateToPose;
use r2r::nav_msgs::msg::OccupancyGrid;
use r2r::visualization_msgs::msg::MarkerArray;
use r2r::{ActionClient, ActionClientGoal, GoalStatus, QosProfile, log_info, log_warn};
use team7_nodes::{Params, grid_from_msg, pose_msg, spawn_tf_listener, viz};
use tb4_core::{Grid, Pose2, Stamp, TfBuffer};
use tb4_explore::{ExploreParams, Frontier, find_frontiers, select_goal_with_heading};

type Nav = NavigateToPose::Action;

struct Active {
    goal: (f64, f64),
    handle: Option<ActionClientGoal<Nav>>,
    started: Instant,
    progress_pose: (f64, f64),
    last_progress: Instant,
    cancelling: bool,
}

struct State {
    grid: Option<Grid>,
    active: Option<Active>,
    blacklist: Vec<((f64, f64), Instant)>,
    next_goal_after: Instant,
    grid_received: Instant,
    done: bool,
    finished_exploring: bool,
    result: Option<((f64, f64), GoalStatus)>,
}

struct Cfg {
    map_frame: String,
    base_frame: String,
    goal_timeout: Duration,
    progress_timeout: Duration,
    progress_epsilon: f64,
    retry_delay: Duration,
    blacklist_duration: Duration,
    turn_weight: f64,
    explore: ExploreParams,
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "explorer", "")?;
    let log = node.logger().to_owned();

    let p = Params(&node);
    let cfg = Rc::new(Cfg {
        map_frame: p.string("map_frame", "map"),
        base_frame: p.string("base_frame", "base_link"),
        goal_timeout: Duration::from_secs_f64(p.f64("goal_timeout", 60.0)),
        progress_timeout: Duration::from_secs_f64(p.f64("progress_timeout", 15.0)),
        progress_epsilon: p.f64("progress_epsilon", 0.1),
        retry_delay: Duration::from_secs_f64(p.f64("retry_delay", 5.0)),
        blacklist_duration: Duration::from_secs_f64(p.f64("blacklist_duration", 45.0)),
        turn_weight: p.f64("turn_weight", 0.5),
        explore: ExploreParams {
            free_threshold: p.i64("free_threshold", 50) as i8,
            min_frontier_cells: p.i64("min_frontier_cells", 8) as usize,
            size_weight: p.f64("size_weight", 1.0),
            distance_weight: p.f64("distance_weight", 1.0),
            blacklist_radius: p.f64("blacklist_radius", 0.5),
            min_goal_distance: p.f64("min_goal_distance", 0.3),
            goal_standoff: p.f64("goal_standoff", 0.25),
            traversable_threshold: p.i64("traversable_threshold", 99) as i8,
            frontier_segment_length: p.f64("frontier_segment_length", 1.2),
        },
    });
    let period = Duration::from_secs_f64(p.f64("period", 1.0));
    let costmap_topic = p.string("costmap_topic", "/global_costmap/costmap");
    let nav_action = p.string("nav_action", "/navigate_to_pose");
    let tf_topic = p.string("tf_topic", "/tf");
    let tf_static_topic = p.string("tf_static_topic", "/tf_static");

    let pool = LocalPool::new();
    let spawner = pool.spawner();
    let tf = Rc::new(RefCell::new(TfBuffer::new(10.0, 0.2)));
    spawn_tf_listener(&mut node, &spawner, tf.clone(), &tf_topic, &tf_static_topic)?;

    let state = Rc::new(RefCell::new(State {
        grid: None,
        active: None,
        blacklist: Vec::new(),
        next_goal_after: Instant::now(),
        grid_received: Instant::now(),
        done: false,
        finished_exploring: false,
        result: None,
    }));

    let costmap = node.subscribe::<OccupancyGrid>(&costmap_topic, QosProfile::default().reliable().transient_local())?;
    let st = state.clone();
    spawner.spawn_local(costmap.for_each(move |m| {
        let mut s = st.borrow_mut();
        if let Some(grid) = grid_from_msg(&m) {
            s.grid = Some(grid);
            s.grid_received = Instant::now();
        }
        futures::future::ready(())
    }))?;

    let mission = node.subscribe::<r2r::std_msgs::msg::String>(
        "/team7/mission_state",
        QosProfile::default().reliable().transient_local(),
    )?;
    let st = state.clone();
    spawner.spawn_local(mission.for_each(move |m| {
        if m.data == "DONE" {
            st.borrow_mut().done = true;
        }
        futures::future::ready(())
    }))?;

    let client: ActionClient<Nav> = node.create_action_client::<Nav>(&nav_action)?;
    let available = r2r::Node::is_available(&client)?;
    let pub_frontiers = node.create_publisher::<MarkerArray>("/team7/frontiers", QosProfile::default())?;
    let pub_goal = node.create_publisher::<r2r::geometry_msgs::msg::PoseStamped>("/team7/goal", QosProfile::default())?;
    let mut timer = node.create_wall_timer(period)?;

    let sp = spawner.clone();
    spawner.spawn_local(async move {
        log_info!(&log, "waiting for {nav_action} ...");
        if available.await.is_err() {
            log_warn!(&log, "action server never became available");
            return;
        }
        log_info!(&log, "Nav2 is up, exploring");
        loop {
            if timer.tick().await.is_err() {
                return;
            }
            tick(&log, &cfg, &state, &tf.borrow(), &client, &sp, &pub_frontiers, &pub_goal);
        }
    })?;

    team7_nodes::run(node, pool, || false);
    Ok(())
}

fn tick(
    log: &str,
    cfg: &Cfg,
    state: &Rc<RefCell<State>>,
    tf: &TfBuffer,
    client: &ActionClient<Nav>,
    spawner: &futures::executor::LocalSpawner,
    pub_frontiers: &r2r::Publisher<MarkerArray>,
    pub_goal: &r2r::Publisher<r2r::geometry_msgs::msg::PoseStamped>,
) {
    let mut s = state.borrow_mut();

    if s.done {
        if let Some(a) = s.active.as_mut().filter(|a| !a.cancelling) {
            log_info!(log, "mission done, cancelling navigation");
            if let Some(h) = &a.handle {
                let _ = h.cancel();
                a.cancelling = true;
            }
        }
        return;
    }

    if let Some((goal, status)) = s.result.take() {
        if s.active.as_ref().is_some_and(|a| a.goal == goal) {
            s.active = None;
            s.next_goal_after = Instant::now() + cfg.retry_delay;
            log_info!(log, "goal ({:.2}, {:.2}) finished: {status:?}", goal.0, goal.1);
            if !matches!(status, GoalStatus::Succeeded | GoalStatus::Canceled) {
                s.blacklist.push((goal, Instant::now()));
            }
        }
    }

    let now = Instant::now();
    s.blacklist.retain(|(_, at)| now.duration_since(*at) < cfg.blacklist_duration);
    // Do not navigate using a costmap that stopped updating.
    if s.grid_received.elapsed() > Duration::from_secs(5) {
        if let Some(a) = s.active.as_mut().filter(|a| !a.cancelling) {
            if let Some(h) = &a.handle {
                let _ = h.cancel();
                a.cancelling = true;
                log_warn!(log, "costmap stale: cancelling goal and waiting for fresh data");
            }
        }
        return;
    }

    let Some(grid) = s.grid.as_ref() else {
        log_warn!(log, "waiting for costmap");
        return;
    };
    let robot = match tf.lookup(&cfg.map_frame, &cfg.base_frame, Stamp::LATEST) {
        Ok(t) => Pose2::from_isometry(&t),
        Err(e) => {
            log_warn!(log, "no robot pose yet: {e}");
            return;
        }
    };
    let frontiers = find_frontiers(grid, (robot.x, robot.y), &cfg.explore);
    let _ = pub_frontiers.publish(&frontier_markers(&cfg.map_frame, &frontiers));

    let mut abandon = None;
    if let Some(a) = s.active.as_mut() {
        // Moving around a wall can increase distance to the goal. Measure real
        // displacement instead of treating a valid detour as lack of progress.
        if robot.distance_to(a.progress_pose.0, a.progress_pose.1) > cfg.progress_epsilon {
            a.progress_pose = (robot.x, robot.y);
            a.last_progress = now;
        }
        if !a.cancelling && a.handle.is_some() {
            if now - a.started > cfg.goal_timeout {
                abandon = Some((a.goal, "timeout"));
            } else if now - a.last_progress > cfg.progress_timeout {
                abandon = Some((a.goal, "no progress"));
            }
        }
    }
    if let Some((goal, why)) = abandon {
        log_info!(log, "cancelling goal ({:.2}, {:.2}): {why}; waiting for terminal result", goal.0, goal.1);
        s.blacklist.push((goal, now));
        if let Some(a) = s.active.as_mut() {
            if let Some(h) = &a.handle {
                if h.cancel().is_ok() {
                    a.cancelling = true;
                }
            }
        }
    }
    if s.active.is_some() || now < s.next_goal_after {
        return;
    }

    let blacklist: Vec<_> = s.blacklist.iter().map(|(goal, _)| *goal).collect();
    let Some(f) = select_goal_with_heading(&frontiers, &blacklist, &cfg.explore,
        (robot.x, robot.y), robot.yaw, cfg.turn_weight) else {
        if !s.finished_exploring {
            log_info!(log, "no eligible reachable frontiers; waiting for map updates / blacklist expiry");
            s.finished_exploring = true;
        }
        return;
    };
    s.finished_exploring = false;
    let goal = f.goal;
    let yaw = (f.centroid.1 - goal.1).atan2(f.centroid.0 - goal.0);
    log_info!(log, "new goal ({:.2}, {:.2}) yaw={yaw:.2}, frontier {:.1} m, free-space path {:.1} m, score {:.2}", goal.0, goal.1, f.size_m, f.distance, f.score);

    let mut msg = NavigateToPose::Goal::default();
    msg.pose.header.frame_id = cfg.map_frame.clone();
    msg.pose.pose = pose_msg(goal.0, goal.1, yaw);
    let _ = pub_goal.publish(&msg.pose);
    s.active = Some(Active {
        goal,
        handle: None,
        started: now,
        progress_pose: (robot.x, robot.y),
        last_progress: now,
        cancelling: false,
    });
    let request = match client.send_goal_request(msg) {
        Ok(r) => r,
        Err(e) => {
            log_warn!(log, "send_goal failed: {e}");
            s.active = None;
            s.next_goal_after = now + cfg.retry_delay;
            return;
        }
    };
    drop(s);

    let st = state.clone();
    let log = log.to_owned();
    let _ = spawner.spawn_local(async move {
        match request.await {
            Ok((handle, result, _feedback)) => {
                if st.borrow().done {
                    let _ = handle.cancel();
                }
                if let Some(a) = st.borrow_mut().active.as_mut().filter(|a| a.goal == goal) {
                    a.handle = Some(handle);
                }
                let status = match result.await {
                    Ok((status, _)) => status,
                    Err(_) => GoalStatus::Aborted,
                };
                st.borrow_mut().result = Some((goal, status));
            }
            Err(e) => {
                log_warn!(&log, "goal rejected: {e}");
                st.borrow_mut().result = Some((goal, GoalStatus::Aborted));
            }
        }
    });
}

fn frontier_markers(frame: &str, frontiers: &[Frontier]) -> MarkerArray {
    let mut pts = viz::base(frame, "frontier_goals", 0, viz::POINTS);
    pts.scale.x = 0.12;
    pts.scale.y = 0.12;
    pts.color = viz::color(0.2, 0.6, 1.0, 0.9);
    pts.points = frontiers
        .iter()
        .map(|f| r2r::geometry_msgs::msg::Point { x: f.goal.0, y: f.goal.1, z: 0.05 })
        .collect();
    MarkerArray { markers: vec![viz::delete_all(frame), pts] }
}
