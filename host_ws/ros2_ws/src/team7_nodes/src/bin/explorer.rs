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
use tb4_explore::{Action, CoverageParams, ExploreParams, ExplorerParams, ExplorerPolicy, Frontier, NavStatus, TickInput, View};

type Nav = NavigateToPose::Action;

struct State {
    /// all decisions; this node only feeds it and carries out its actions
    policy: ExplorerPolicy,
    /// policy time base: seconds since start on the monotonic clock
    t0: Instant,
    grid: Option<Grid>,
    map: Option<Grid>,
    grid_received: f64,
    done: bool,
    /// accepted goal and its handle, for cancelling
    handle: Option<((f64, f64), ActionClientGoal<Nav>)>,
    result: Option<((f64, f64), NavStatus)>,
    viz_seq: u64,
}

impl State {
    fn now(&self) -> f64 {
        self.t0.elapsed().as_secs_f64()
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "explorer", "")?;
    let log = node.logger().to_owned();

    let p = Params(&node);
    let params = ExplorerParams {
        goal_timeout: p.f64("goal_timeout", 60.0),
        progress_timeout: p.f64("progress_timeout", 15.0),
        progress_epsilon: p.f64("progress_epsilon", 0.1),
        retry_delay: p.f64("retry_delay", 5.0),
        blacklist_duration: p.f64("blacklist_duration", 45.0),
        turn_weight: p.f64("turn_weight", 0.5),
        costmap_timeout: 5.0,
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
            min_unknown_area: p.f64("min_unknown_area", 1.0),
            goal_max_unknown: p.f64("goal_max_unknown", 0.15),
        },
        // after frontiers: go look at walls the camera has not seen, markers can be on any of them
        coverage: p.bool("camera_coverage", true).then(|| CoverageParams {
            half_fov: p.f64("camera_half_fov", 0.52),
            range: p.f64("camera_range", 2.5),
            standoff: p.f64("view_standoff", 1.6),
            max_cluster_cells: p.i64("view_max_cells", 32) as usize,
            max_incidence: p.f64("camera_max_incidence", 1.3),
            ..CoverageParams::default()
        }),
        view_detour: p.f64("view_detour", 0.0),
        look_around_turns: p.i64("look_around_turns", 3).max(0) as u32,
        preempt_distance: p.f64("preempt_distance", 0.5),
        switch_cooldown: p.f64("switch_cooldown", 20.0),
    };
    let map_frame = Rc::new(p.string("map_frame", "map"));
    let base_frame = Rc::new(p.string("base_frame", "base_link"));
    let map_topic = p.string("coverage_map_topic", "/map");
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
        policy: ExplorerPolicy::new(params, 0.0),
        t0: Instant::now(),
        grid: None,
        map: None,
        grid_received: 0.0,
        done: false,
        handle: None,
        result: None,
        viz_seq: 0,
    }));

    let costmap = node.subscribe::<OccupancyGrid>(&costmap_topic, QosProfile::default().reliable().transient_local())?;
    let st = state.clone();
    spawner.spawn_local(costmap.for_each(move |m| {
        let mut s = st.borrow_mut();
        if let Some(grid) = grid_from_msg(&m) {
            s.grid = Some(grid);
            s.grid_received = s.now();
        }
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
    let pubs = Pubs {
        frontiers: node.create_publisher::<MarkerArray>("/team7/frontiers", QosProfile::default())?,
        goal: node.create_publisher::<r2r::geometry_msgs::msg::PoseStamped>("/team7/goal", QosProfile::default())?,
    };
    let mut timer = node.create_wall_timer(period)?;

    // what the camera saw, 5 Hz
    let mut cov_timer = node.create_wall_timer(Duration::from_millis(200))?;
    let (st, tfc, mf, bf) = (state.clone(), tf.clone(), map_frame.clone(), base_frame.clone());
    spawner.spawn_local(async move {
        while cov_timer.tick().await.is_ok() {
            if st.borrow().policy.params().coverage.is_none() {
                return;
            }
            let Ok(t) = tfc.borrow().lookup(&mf, &bf, Stamp::LATEST) else { continue };
            let r = Pose2::from_isometry(&t);
            let mut s = st.borrow_mut();
            let s = &mut *s;
            if let Some(m) = s.map.as_ref() {
                s.policy.update_coverage(m, (r.x, r.y, r.yaw));
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
        let io = Io { log: &log, map_frame: &map_frame, base_frame: &base_frame, client: &client, spawner: &sp, pubs: &pubs };
        loop {
            if timer.tick().await.is_err() {
                return;
            }
            tick(&io, &state, &tf.borrow());
        }
    })?;

    team7_nodes::run(node, pool, || false);
    Ok(())
}

struct Pubs {
    frontiers: r2r::Publisher<MarkerArray>,
    goal: r2r::Publisher<r2r::geometry_msgs::msg::PoseStamped>,
}

struct Io<'a> {
    log: &'a str,
    map_frame: &'a str,
    base_frame: &'a str,
    client: &'a ActionClient<Nav>,
    spawner: &'a futures::executor::LocalSpawner,
    pubs: &'a Pubs,
}

/// one ExplorerPolicy tick: inputs from the subscriptions and tf in, actions out to the action client
fn tick(io: &Io, state: &Rc<RefCell<State>>, tf: &TfBuffer) {
    let log = io.log;
    let mut s = state.borrow_mut();
    let s = &mut *s;
    let now = s.now();
    let robot = tf
        .lookup(io.map_frame, io.base_frame, Stamp::LATEST)
        .map(|t| Pose2::from_isometry(&t))
        .map_err(|e| e.to_string());
    let input = TickInput {
        costmap: s.grid.as_ref(),
        costmap_stamp: s.grid_received,
        map: s.map.as_ref(),
        robot,
        result: s.result.take(),
        done: s.done,
    };
    let actions = s.policy.tick(now, input);
    let (frontiers, views, seq) = s.policy.viz();
    if seq != s.viz_seq {
        s.viz_seq = seq;
        let _ = io.pubs.frontiers.publish(&frontier_markers(io.map_frame, frontiers, views));
    }
    let mut send = None;
    for a in actions {
        match a {
            Action::Info(m) => log_info!(log, "{m}"),
            Action::Warn(m) => log_warn!(log, "{m}"),
            Action::Cancel(_) => {
                let active = s.policy.active().map(|a| a.goal);
                if let Some((_, h)) = s.handle.as_ref().filter(|(g, _)| Some(*g) == active) {
                    let _ = h.cancel();
                }
            }
            Action::SendGoal(r) => send = Some(r),
        }
    }
    let Some(r) = send else { return };

    let goal = (r.x, r.y);
    let mut msg = NavigateToPose::Goal::default();
    msg.pose.header.frame_id = io.map_frame.to_owned();
    msg.pose.pose = pose_msg(r.x, r.y, r.yaw);
    if r.xy_only {
        // the lidar sees all round: no stop to turn to a heading at a frontier
        msg.behavior_tree = "xy_only".to_owned();
    }
    let _ = io.pubs.goal.publish(&msg.pose);
    let request = match io.client.send_goal_request(msg) {
        Ok(r) => r,
        Err(e) => {
            log_warn!(log, "send_goal failed: {e}");
            s.policy.goal_send_failed(now);
            return;
        }
    };

    let st = state.clone();
    let log = log.to_owned();
    let _ = io.spawner.spawn_local(async move {
        match request.await {
            Ok((handle, result, _feedback)) => {
                if st.borrow().done {
                    let _ = handle.cancel();
                }
                {
                    let mut s = st.borrow_mut();
                    if s.policy.active().is_some_and(|a| a.goal == goal) {
                        s.policy.goal_accepted(goal);
                        s.handle = Some((goal, handle));
                    }
                }
                let status = match result.await {
                    Ok((GoalStatus::Succeeded, _)) => NavStatus::Succeeded,
                    Ok((GoalStatus::Canceled, _)) => NavStatus::Canceled,
                    _ => NavStatus::Aborted,
                };
                st.borrow_mut().result = Some((goal, status));
            }
            Err(e) => {
                log_warn!(&log, "goal rejected: {e}");
                st.borrow_mut().result = Some((goal, NavStatus::Aborted));
            }
        }
    });
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
