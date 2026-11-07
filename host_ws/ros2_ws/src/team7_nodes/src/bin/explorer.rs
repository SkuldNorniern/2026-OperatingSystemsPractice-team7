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
use tb4_explore::{Coverage, CoverageParams, ExploreParams, Frontier, View, find_frontiers, select_goal};

type Nav = NavigateToPose::Action;

struct Active {
    goal: (f64, f64),
    /// camera view goal: wall point it looks at
    view_target: Option<(f64, f64)>,
    handle: Option<ActionClientGoal<Nav>>,
    started: Instant,
    best_dist: f64,
    last_progress: Instant,
}

struct State {
    grid: Option<Grid>,
    map: Option<Grid>,
    coverage: Coverage,
    visited_views: Vec<(f64, f64)>,
    active: Option<Active>,
    blacklist: Vec<(f64, f64)>,
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
    gone_radius: f64,
    heading_weight: f64,
    explore: ExploreParams,
    coverage: Option<CoverageParams>,
    /// take a camera view during frontier exploration if it is this close (m), 0 = only after frontiers
    view_detour: f64,
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
        gone_radius: p.f64("frontier_gone_radius", 0.5),
        heading_weight: p.f64("heading_weight", 0.0),
        explore: ExploreParams {
            free_threshold: p.i64("free_threshold", 50) as i8,
            min_frontier_cells: p.i64("min_frontier_cells", 8) as usize,
            size_weight: p.f64("size_weight", 1.0),
            distance_weight: p.f64("distance_weight", 1.0),
            blacklist_radius: p.f64("blacklist_radius", 0.5),
            min_goal_distance: p.f64("min_goal_distance", 0.3),
        },
        // after frontiers: go look at walls the camera has not seen, markers can be on any of them
        coverage: p.bool("camera_coverage", true).then(|| CoverageParams {
            half_fov: p.f64("camera_half_fov", 0.52),
            range: p.f64("camera_range", 2.5),
            standoff: p.f64("view_standoff", 1.6),
            max_cluster_cells: p.i64("view_max_cells", 32) as usize,
            ..CoverageParams::default()
        }),
        view_detour: p.f64("view_detour", 0.0),
    });
    let map_topic = p.string("coverage_map_topic", "/map");
    let period = Duration::from_secs_f64(p.f64("period", 1.0));
    let costmap_topic = p.string("costmap_topic", "/global_costmap/costmap");
    let nav_action = p.string("nav_action", "/navigate_to_pose");
    let tf_topic = p.string("tf_topic", "/tf");
    let tf_static_topic = p.string("tf_static_topic", "/tf_static");

    let pool = LocalPool::new();
    let spawner = pool.spawner();
    team7_nodes::spawn_param_services(&mut node, &spawner)?;
    let tf = Rc::new(RefCell::new(TfBuffer::new(10.0, 0.2)));
    spawn_tf_listener(&mut node, &spawner, tf.clone(), &tf_topic, &tf_static_topic)?;

    let state = Rc::new(RefCell::new(State {
        grid: None,
        map: None,
        coverage: Coverage::new(),
        visited_views: Vec::new(),
        active: None,
        blacklist: Vec::new(),
        done: false,
        finished_exploring: false,
        result: None,
    }));

    let costmap = node.subscribe::<OccupancyGrid>(&costmap_topic, QosProfile::default().reliable().transient_local())?;
    let st = state.clone();
    spawner.spawn_local(costmap.for_each(move |m| {
        st.borrow_mut().grid = grid_from_msg(&m);
        futures::future::ready(())
    }))?;

    let maps = node.subscribe::<OccupancyGrid>(&map_topic, QosProfile::default().reliable().transient_local())?;
    let st = state.clone();
    spawner.spawn_local(maps.for_each(move |m| {
        st.borrow_mut().map = grid_from_msg(&m);
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
    let mut timer = node.create_wall_timer(period)?;

    // what the camera saw, 5 Hz
    let mut cov_timer = node.create_wall_timer(Duration::from_millis(200))?;
    let (st, tfc, c) = (state.clone(), tf.clone(), cfg.clone());
    spawner.spawn_local(async move {
        while cov_timer.tick().await.is_ok() {
            let Some(cp) = c.coverage.as_ref() else { return };
            let Ok(t) = tfc.borrow().lookup(&c.map_frame, &c.base_frame, Stamp::LATEST) else { continue };
            let r = Pose2::from_isometry(&t);
            let mut s = st.borrow_mut();
            let s = &mut *s;
            if let Some(m) = s.map.as_ref() {
                s.coverage.update(m, (r.x, r.y, r.yaw), cp);
            }
        }
    })?;

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
            tick(&log, &cfg, &state, &tf.borrow(), &client, &sp, &pub_frontiers);
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
) {
    let mut s = state.borrow_mut();

    if s.done {
        if let Some(a) = s.active.take() {
            log_info!(log, "mission done, cancelling navigation");
            if let Some(h) = a.handle {
                let _ = h.cancel();
            }
        }
        return;
    }

    if let Some((goal, status)) = s.result.take() {
        let was_active = s.active.as_ref().is_some_and(|a| a.goal == goal);
        if was_active {
            // a reached view whose wall still counts as unseen must not be picked again
            if let Some(t) = s.active.take().and_then(|a| a.view_target) {
                s.visited_views.push(t);
            }
        }
        // results of goals we already dropped (preempted -> aborted) dont count
        if was_active && !matches!(status, GoalStatus::Succeeded | GoalStatus::Canceled) {
            log_warn!(log, "goal ({:.2}, {:.2}) ended with {status:?}, blacklisting", goal.0, goal.1);
            s.blacklist.push(goal);
        }
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
    let mut frontiers = find_frontiers(grid, (robot.x, robot.y), &cfg.explore);
    tb4_explore::rescore_heading(&mut frontiers, (robot.x, robot.y, robot.yaw), cfg.heading_weight);
    let need_views = cfg.view_detour > 0.0
        || s.active.as_ref().is_some_and(|a| a.view_target.is_some())
        || select_goal(&frontiers, &s.blacklist, &cfg.explore).is_none();
    let views: Vec<View> = match (cfg.coverage.as_ref(), s.map.as_ref()) {
        (Some(cp), Some(m)) if need_views => s
            .coverage
            .views(m, (robot.x, robot.y), cp)
            .into_iter()
            .filter(|v| s.visited_views.iter().all(|t| dist(*t, v.target) > 0.6))
            .filter(|v| s.blacklist.iter().all(|b| dist(*b, (v.x, v.y)) > cfg.explore.blacklist_radius))
            .collect(),
        _ => Vec::new(),
    };
    let _ = pub_frontiers.publish(&frontier_markers(&cfg.map_frame, &frontiers, &views));

    let now = Instant::now();
    let mut abandon = None;
    if let Some(a) = s.active.as_mut() {
        let d = robot.distance_to(a.goal.0, a.goal.1);
        if d < a.best_dist - cfg.progress_epsilon {
            a.best_dist = d;
            a.last_progress = now;
        }
        let gone = match a.view_target {
            Some(t) => !views.iter().any(|v| dist(v.target, t) < 0.6),
            None => !frontiers.iter().any(|f| dist(f.goal, a.goal) < cfg.gone_radius),
        };
        if now - a.started > cfg.goal_timeout {
            abandon = Some((a.goal, "timeout", true));
        } else if now - a.last_progress > cfg.progress_timeout {
            abandon = Some((a.goal, "no progress", true));
        } else if gone {
            abandon = Some((a.goal, if a.view_target.is_some() { "wall seen" } else { "frontier explored" }, false));
        }
    }
    if let Some((goal, why, blacklist)) = abandon {
        log_info!(log, "dropping goal ({:.2}, {:.2}): {why}", goal.0, goal.1);
        if blacklist {
            s.blacklist.push(goal);
        }
        if let Some(h) = s.active.take().and_then(|a| a.handle) {
            let _ = h.cancel();
        }
    }
    if s.active.is_some() {
        return;
    }

    let detour = views.first().filter(|v| v.distance <= cfg.view_detour);
    let (goal, yaw, view_target) = if let Some(v) = detour {
        log_info!(log, "new goal ({:.2}, {:.2}) camera view detour, {} unseen wall cells, {:.1} m away", v.x, v.y, v.cells, v.distance);
        ((v.x, v.y), v.yaw, Some(v.target))
    } else if let Some(f) = select_goal(&frontiers, &s.blacklist, &cfg.explore) {
        log_info!(log, "new goal ({:.2}, {:.2}) frontier {:.1} m, {:.1} m away", f.goal.0, f.goal.1, f.size_m, f.distance);
        (f.goal, (f.goal.1 - robot.y).atan2(f.goal.0 - robot.x), None)
    } else if let Some(v) = views.first() {
        log_info!(log, "new goal ({:.2}, {:.2}) camera view of {} unseen wall cells, {:.1} m away", v.x, v.y, v.cells, v.distance);
        ((v.x, v.y), v.yaw, Some(v.target))
    } else {
        if !s.finished_exploring {
            log_info!(log, "no frontiers or unseen walls left: exploration finished ({} wall cells seen)", s.coverage.seen_count());
            s.finished_exploring = true;
        }
        return;
    };
    s.finished_exploring = false;

    let mut msg = NavigateToPose::Goal::default();
    msg.pose.header.frame_id = cfg.map_frame.clone();
    msg.pose.pose = pose_msg(goal.0, goal.1, yaw);
    s.active = Some(Active {
        goal,
        view_target,
        handle: None,
        started: now,
        best_dist: robot.distance_to(goal.0, goal.1),
        last_progress: now,
    });
    let request = match client.send_goal_request(msg) {
        Ok(r) => r,
        Err(e) => {
            log_warn!(log, "send_goal failed: {e}");
            s.active = None;
            return;
        }
    };
    drop(s);

    let st = state.clone();
    let log = log.to_owned();
    let _ = spawner.spawn_local(async move {
        match request.await {
            Ok((handle, result, _feedback)) => {
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

fn dist(a: (f64, f64), b: (f64, f64)) -> f64 {
    (a.0 - b.0).hypot(a.1 - b.1)
}

fn frontier_markers(frame: &str, frontiers: &[Frontier], views: &[View]) -> MarkerArray {
    let mut pts = viz::base(frame, "frontier_goals", 0, viz::POINTS);
    pts.scale.x = 0.12;
    pts.scale.y = 0.12;
    pts.color = viz::color(0.2, 0.6, 1.0, 0.9);
    pts.points = frontiers
        .iter()
        .map(|f| r2r::geometry_msgs::msg::Point { x: f.goal.0, y: f.goal.1, z: 0.05 })
        .collect();
    let mut cams = viz::base(frame, "camera_views", 0, viz::POINTS);
    cams.scale.x = 0.12;
    cams.scale.y = 0.12;
    cams.color = viz::color(1.0, 0.5, 0.0, 0.9);
    cams.points = views.iter().map(|v| r2r::geometry_msgs::msg::Point { x: v.x, y: v.y, z: 0.05 }).collect();
    MarkerArray { markers: vec![viz::delete_all(frame), pts, cams] }
}
