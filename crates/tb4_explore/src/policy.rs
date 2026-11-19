//! Explorer decision logic without ROS: the explorer node and the simulator both drive this.
//! Time is passed in as seconds on any monotonic clock, nothing here reads the wall clock.

use tb4_core::{Grid, Pose2};

use crate::{Coverage, CoverageParams, ExploreParams, Frontier, View, find_frontiers, select_goal_with_heading};

#[derive(Debug, Clone)]
pub struct ExplorerParams {
    /// s, a goal running longer than this is cancelled and blacklisted
    pub goal_timeout: f64,
    /// s without progress (progress_epsilon of movement or a 0.3 rad turn) before a goal is given up
    pub progress_timeout: f64,
    pub progress_epsilon: f64,
    /// s to wait after a failed or cancelled goal; a success chains on at once
    pub retry_delay: f64,
    pub blacklist_duration: f64,
    pub turn_weight: f64,
    /// s, exploration pauses when the costmap is older than this
    pub costmap_timeout: f64,
    pub explore: ExploreParams,
    /// after frontiers: go look at walls the camera has not seen, markers can be on any of them
    pub coverage: Option<CoverageParams>,
    /// take a camera view during frontier exploration if it is this close (m), 0 = only after frontiers
    pub view_detour: f64,
    /// in-place turns (120 deg each) when nothing is left to explore, so SLAM gets new keyframes
    pub look_around_turns: u32,
    /// a frontier goal this close whose frontier has started to disappear is replaced by the next
    /// one, so the robot does not stop at it. With the frontier still open the goal is finished:
    /// switching then made two goals swap for 8 min in the sim studio.
    pub preempt_distance: f64,
    /// s a goal switched away from cannot be picked again
    pub switch_cooldown: f64,
}

impl Default for ExplorerParams {
    /// the explorer node's parameter defaults
    fn default() -> Self {
        Self {
            goal_timeout: 60.0,
            progress_timeout: 15.0,
            progress_epsilon: 0.1,
            retry_delay: 5.0,
            blacklist_duration: 45.0,
            turn_weight: 0.5,
            costmap_timeout: 5.0,
            explore: ExploreParams {
                free_threshold: 50,
                min_frontier_cells: 8,
                size_weight: 1.0,
                distance_weight: 1.0,
                blacklist_radius: 0.5,
                min_goal_distance: 0.3,
                goal_standoff: 0.25,
                traversable_threshold: 99,
                frontier_segment_length: 1.2,
                min_unknown_area: 1.0,
            },
            coverage: Some({
                let mut c = CoverageParams::default();
                (c.half_fov, c.range, c.standoff, c.max_cluster_cells) = (0.52, 2.5, 1.6, 32);
                c
            }),
            view_detour: 0.0,
            look_around_turns: 3,
            preempt_distance: 0.5,
            switch_cooldown: 20.0,
        }
    }
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum GoalKind {
    Frontier,
    /// camera view of a wall piece the camera has not seen
    View,
    /// in-place look-around turn
    Turn,
}

/// terminal state of a navigation goal; everything that is neither success nor cancel counts as aborted
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum NavStatus {
    Succeeded,
    Canceled,
    Aborted,
}

#[derive(Clone, Copy, PartialEq, Debug)]
pub struct GoalRequest {
    pub x: f64,
    pub y: f64,
    pub yaw: f64,
    /// no final heading: the navigator may finish facing any way
    pub xy_only: bool,
    pub kind: GoalKind,
    /// replaces the active frontier goal without a cancel (the navigator aborts the old one)
    pub preempts: bool,
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum CancelReason {
    MissionDone,
    CostmapStale,
    Timeout,
    NoProgress,
    WallSeen,
}

#[derive(Clone, PartialEq, Debug)]
pub enum Action {
    SendGoal(GoalRequest),
    /// cancel the active goal; its terminal result still has to come back through `TickInput::result`
    Cancel(CancelReason),
    Info(String),
    Warn(String),
}

/// what the explorer sees at one tick
pub struct TickInput<'a> {
    /// grid frontiers are searched in (team7 mode: the SLAM map)
    pub costmap: Option<&'a Grid>,
    /// when `costmap` was last received (s)
    pub costmap_stamp: f64,
    /// grid the camera coverage works on
    pub map: Option<&'a Grid>,
    /// robot in the map frame, or why there is none
    pub robot: Result<Pose2, String>,
    /// latest terminal result from the navigator: (goal, status)
    pub result: Option<((f64, f64), NavStatus)>,
    /// mission finished: cancel and stop
    pub done: bool,
}

#[derive(Debug, Clone, Default, PartialEq)]
pub struct ExplorerStats {
    pub sent: u64,
    pub sent_frontier: u64,
    pub sent_view: u64,
    pub sent_turn: u64,
    /// frontier goals replaced by a new goal without stopping
    pub switched: u64,
    pub succeeded: u64,
    /// results of the active goal that came back cancelled
    pub canceled: u64,
    pub aborted: u64,
    pub cancel_timeout: u64,
    pub cancel_no_progress: u64,
    pub cancel_wall_seen: u64,
    pub cancel_other: u64,
}

#[derive(Debug, Clone)]
struct Active {
    goal: (f64, f64),
    kind: GoalKind,
    /// ticks in a row without a frontier at this goal (frontier goals)
    frontier_missing: u32,
    /// camera view goal: wall point it looks at
    view_target: Option<(f64, f64)>,
    /// the navigator accepted it (cancel is possible from then on)
    accepted: bool,
    started: f64,
    /// x, y, yaw where progress was last seen
    progress_pose: (f64, f64, f64),
    last_progress: f64,
    cancelling: bool,
    /// cancelled for a reason that is not a failure (its wall got seen on the way)
    cancel_ok: bool,
}

/// active goal as seen from outside
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ActiveGoal {
    pub goal: (f64, f64),
    pub kind: GoalKind,
    pub view_target: Option<(f64, f64)>,
    pub accepted: bool,
    pub cancelling: bool,
    pub started: f64,
}

pub struct ExplorerPolicy {
    p: ExplorerParams,
    coverage: Coverage,
    visited_views: Vec<(f64, f64)>,
    active: Option<Active>,
    blacklist: Vec<((f64, f64), f64)>,
    /// goals switched away from, with the time
    recent: Vec<((f64, f64), f64)>,
    next_goal_after: f64,
    costmap_stale: bool,
    finished_exploring: bool,
    /// look-around turns left before waiting for the map (reset when a real goal is found)
    spins_left: u32,
    frontiers: Vec<Frontier>,
    views: Vec<View>,
    viz_seq: u64,
    stats: ExplorerStats,
}

impl ExplorerPolicy {
    pub fn new(p: ExplorerParams, now: f64) -> Self {
        Self {
            coverage: Coverage::new(),
            visited_views: Vec::new(),
            active: None,
            blacklist: Vec::new(),
            recent: Vec::new(),
            next_goal_after: now,
            costmap_stale: false,
            finished_exploring: false,
            spins_left: p.look_around_turns,
            frontiers: Vec::new(),
            views: Vec::new(),
            viz_seq: 0,
            stats: ExplorerStats::default(),
            p,
        }
    }

    pub fn params(&self) -> &ExplorerParams {
        &self.p
    }

    pub fn stats(&self) -> &ExplorerStats {
        &self.stats
    }

    pub fn coverage(&self) -> &Coverage {
        &self.coverage
    }

    /// frontiers and camera views of the last tick that got that far, and a counter that changes with them
    pub fn viz(&self) -> (&[Frontier], &[View], u64) {
        (&self.frontiers, &self.views, self.viz_seq)
    }

    pub fn blacklist(&self) -> impl Iterator<Item = (f64, f64)> + '_ {
        self.blacklist.iter().map(|(g, _)| *g)
    }

    pub fn active(&self) -> Option<ActiveGoal> {
        self.active.as_ref().map(|a| ActiveGoal {
            goal: a.goal,
            kind: a.kind,
            view_target: a.view_target,
            accepted: a.accepted,
            cancelling: a.cancelling,
            started: a.started,
        })
    }

    /// mark wall cells the camera sees from `pose` (x, y, yaw); the node calls this at 5 Hz
    pub fn update_coverage(&mut self, map: &Grid, pose: (f64, f64, f64)) {
        if let Some(cp) = self.p.coverage.as_ref() {
            self.coverage.update(map, pose, cp);
        }
    }

    /// the navigator accepted the goal sent for `goal`
    pub fn goal_accepted(&mut self, goal: (f64, f64)) {
        if let Some(a) = self.active.as_mut().filter(|a| a.goal == goal) {
            a.accepted = true;
        }
    }

    /// sending the last goal failed outright: forget it and wait retry_delay
    pub fn goal_send_failed(&mut self, now: f64) {
        self.active = None;
        self.next_goal_after = now + self.p.retry_delay;
    }

    pub fn tick(&mut self, now: f64, input: TickInput) -> Vec<Action> {
        let mut out = Vec::new();
        self.tick_inner(now, input, &mut out);
        out
    }

    fn cancel_active(&mut self, why: CancelReason, out: &mut Vec<Action>) {
        if let Some(a) = self.active.as_mut()
            && a.accepted
        {
            a.cancelling = true;
            a.cancel_ok = why == CancelReason::WallSeen;
            out.push(Action::Cancel(why));
            let s = &mut self.stats;
            match why {
                CancelReason::Timeout => s.cancel_timeout += 1,
                CancelReason::NoProgress => s.cancel_no_progress += 1,
                CancelReason::WallSeen => s.cancel_wall_seen += 1,
                _ => s.cancel_other += 1,
            }
        }
    }

    fn tick_inner(&mut self, now: f64, input: TickInput, out: &mut Vec<Action>) {
        let p = &self.p;
        if input.done {
            if self.active.as_ref().is_some_and(|a| !a.cancelling) {
                out.push(Action::Info("mission done, cancelling navigation".into()));
                self.cancel_active(CancelReason::MissionDone, out);
            }
            return;
        }

        if let Some((goal, status)) = input.result
            && self.active.as_ref().is_some_and(|a| a.goal == goal)
        {
            let ok = status == NavStatus::Succeeded || (status == NavStatus::Canceled && self.active.as_ref().is_some_and(|a| a.cancel_ok));
            // a reached view whose wall still counts as unseen must not be picked again
            if let Some(t) = self.active.take().and_then(|a| a.view_target) {
                self.visited_views.push(t);
            }
            // field runs lost about 5 s at every goal: chain straight on after a success,
            // wait only after a failure so the map / blacklist can change first
            // a view cancelled because its wall got seen on the way is done, not failed (sim studio:
            // 23 such cancels cost 5 s each)
            self.next_goal_after = if ok { now } else { now + p.retry_delay };
            out.push(Action::Info(format!("goal ({:.2}, {:.2}) finished: {status:?}", goal.0, goal.1)));
            match status {
                NavStatus::Succeeded => self.stats.succeeded += 1,
                NavStatus::Canceled => self.stats.canceled += 1,
                NavStatus::Aborted => {
                    self.stats.aborted += 1;
                    self.blacklist.push((goal, now));
                }
            }
        }

        self.blacklist.retain(|(_, at)| now - *at < p.blacklist_duration);
        // Do not navigate using a costmap that stopped updating.
        if now - input.costmap_stamp > p.costmap_timeout {
            if !self.costmap_stale {
                self.costmap_stale = true;
                out.push(Action::Warn(format!("no costmap for {:.0} s, exploration paused", p.costmap_timeout)));
            }
            if self.active.as_ref().is_some_and(|a| !a.cancelling && a.accepted) {
                self.cancel_active(CancelReason::CostmapStale, out);
                out.push(Action::Warn("costmap stale: cancelling goal and waiting for fresh data".into()));
            }
            return;
        }
        if self.costmap_stale {
            self.costmap_stale = false;
            out.push(Action::Info("costmap back, exploration resumed".into()));
        }

        let Some(grid) = input.costmap else {
            out.push(Action::Warn("waiting for costmap".into()));
            return;
        };
        let robot = match input.robot {
            Ok(r) => r,
            Err(e) => {
                out.push(Action::Warn(format!("no robot pose yet: {e}")));
                return;
            }
        };
        let p = self.p.clone();
        let frontiers = find_frontiers(grid, (robot.x, robot.y), &p.explore);
        self.recent.retain(|(_, at)| now - at < p.switch_cooldown);
        // goals switched away from lately are skipped like blacklisted ones (no ping-pong)
        let blacklist: Vec<_> = self.blacklist.iter().chain(&self.recent).map(|(goal, _)| *goal).collect();
        let pick = || select_goal_with_heading(&frontiers, &blacklist, &p.explore, (robot.x, robot.y), robot.yaw, p.turn_weight);
        let need_views = p.view_detour > 0.0
            || self.active.as_ref().is_some_and(|a| a.view_target.is_some())
            || pick().is_none();
        let views: Vec<View> = match (p.coverage.as_ref(), input.map) {
            (Some(cp), Some(m)) if need_views => self
                .coverage
                .views(m, (robot.x, robot.y), cp)
                .into_iter()
                .filter(|v| self.visited_views.iter().all(|t| dist(*t, v.target) > 0.6))
                .filter(|v| blacklist.iter().all(|b| dist(*b, (v.x, v.y)) > p.explore.blacklist_radius))
                .collect(),
            _ => Vec::new(),
        };
        // nearest first doubled back along the walls (sim studio); like frontiers, prefer views
        // ahead: cost = distance + turn_weight * |turn to the view| (m-equivalent)
        let mut views = views;
        views.sort_by(|a, b| {
            let cost = |v: &View| {
                let ang = (v.y - robot.y).atan2(v.x - robot.x) - robot.yaw;
                v.distance + p.turn_weight * ang.sin().atan2(ang.cos()).abs()
            };
            cost(a).total_cmp(&cost(b))
        });

        let mut abandon = None;
        if let Some(a) = self.active.as_mut() {
            // Moving around a wall can increase distance to the goal. Measure real
            // displacement instead of treating a valid detour as lack of progress.
            // turning counts as progress too: a look-around turn, or the turn toward a goal behind
            let turned = (robot.yaw - a.progress_pose.2).sin().atan2((robot.yaw - a.progress_pose.2).cos()).abs() > 0.3;
            if robot.distance_to(a.progress_pose.0, a.progress_pose.1) > p.progress_epsilon || turned {
                a.progress_pose = (robot.x, robot.y, robot.yaw);
                a.last_progress = now;
            }
            if !a.cancelling && a.accepted {
                if now - a.started > p.goal_timeout {
                    abandon = Some((a.goal, CancelReason::Timeout, true));
                } else if now - a.last_progress > p.progress_timeout {
                    abandon = Some((a.goal, CancelReason::NoProgress, true));
                } else if a.view_target.is_some_and(|t| !views.iter().any(|v| dist(v.target, t) < 0.6)) {
                    abandon = Some((a.goal, CancelReason::WallSeen, false));
                }
            }
        }
        if let Some((goal, why, bad)) = abandon {
            let text = match why {
                CancelReason::Timeout => "timeout",
                CancelReason::NoProgress => "no progress",
                _ => "wall seen",
            };
            out.push(Action::Info(format!("cancelling goal ({:.2}, {:.2}): {text}; waiting for terminal result", goal.0, goal.1)));
            if bad {
                self.blacklist.push((goal, now));
            }
            self.cancel_active(why, out);
        }
        // Keep moving: a frontier goal whose frontier was already seen on the way, or one that is almost
        // reached, is replaced by the next goal at once. The navigator takes the new goal without
        // stopping (it keeps the MPPI warm start), so the robot does not halt at every frontier.
        let mut preempt = None;
        if abandon.is_none()
            && let Some(a) = self.active.as_mut().filter(|a| a.kind == GoalKind::Frontier && !a.cancelling && a.accepted)
        {
            if frontiers.iter().any(|f| dist(f.goal, a.goal) < p.explore.blacklist_radius) {
                a.frontier_missing = 0;
            } else {
                a.frontier_missing += 1;
            }
            if a.frontier_missing >= 2 {
                preempt = Some((a.goal, "its frontier is explored"));
            } else if a.frontier_missing >= 1 && robot.distance_to(a.goal.0, a.goal.1) < p.preempt_distance {
                preempt = Some((a.goal, "almost there"));
            }
        }
        let choice = if let Some((current, why)) = preempt {
            let mut exclude = blacklist.clone();
            exclude.push(current);
            let next = select_goal_with_heading(&frontiers, &exclude, &p.explore, (robot.x, robot.y), robot.yaw, p.turn_weight);
            let Some(f) = next else {
                self.set_viz(frontiers, views);
                return; // nothing else to go to: finish this one
            };
            let yaw = (f.centroid.1 - f.goal.1).atan2(f.centroid.0 - f.goal.0);
            out.push(Action::Info(format!(
                "switching goal ({:.2}, {:.2}) -> ({:.2}, {:.2}): {why}; frontier {:.1} m, free-space path {:.1} m, score {:.2}",
                current.0, current.1, f.goal.0, f.goal.1, f.size_m, f.distance, f.score
            )));
            (f.goal, yaw, None, GoalKind::Frontier)
        } else {
            if self.active.is_some() || now < self.next_goal_after {
                self.set_viz(frontiers, views);
                return;
            }
            let detour = views.first().filter(|v| v.distance <= p.view_detour);
            if let Some(v) = detour {
                out.push(Action::Info(format!(
                    "new goal ({:.2}, {:.2}) camera view detour, {} unseen wall cells, {:.1} m away",
                    v.x, v.y, v.cells, v.distance
                )));
                ((v.x, v.y), v.yaw, Some(v.target), GoalKind::View)
            } else if let Some(f) = pick() {
                let yaw = (f.centroid.1 - f.goal.1).atan2(f.centroid.0 - f.goal.0);
                out.push(Action::Info(format!(
                    "new goal ({:.2}, {:.2}) yaw={yaw:.2}, frontier {:.1} m, free-space path {:.1} m, score {:.2}",
                    f.goal.0, f.goal.1, f.size_m, f.distance, f.score
                )));
                (f.goal, yaw, None, GoalKind::Frontier)
            } else if let Some(v) = views.first() {
                out.push(Action::Info(format!(
                    "new goal ({:.2}, {:.2}) camera view of {} unseen wall cells, {:.1} m away",
                    v.x, v.y, v.cells, v.distance
                )));
                ((v.x, v.y), v.yaw, Some(v.target), GoalKind::View)
            } else if self.spins_left > 0 {
                // Standing still, SLAM adds no keyframes and the map never grows, so the explorer would wait
                // forever (field run 20261118_184838 sat 75 s at the start). Turn in place to see more.
                self.spins_left -= 1;
                let yaw = robot.yaw + 2.0 * std::f64::consts::FRAC_PI_3;
                out.push(Action::Info(format!(
                    "nothing to explore yet: turning in place to look around ({} turns left)",
                    self.spins_left
                )));
                ((robot.x, robot.y), yaw, None, GoalKind::Turn)
            } else {
                if !self.finished_exploring {
                    out.push(Action::Info(format!(
                        "no reachable frontiers or unseen walls ({} wall cells seen); waiting for map updates / blacklist expiry",
                        self.coverage.seen_count()
                    )));
                    self.finished_exploring = true;
                }
                self.set_viz(frontiers, views);
                return;
            }
        };
        self.set_viz(frontiers, views);
        let (goal, yaw, view_target, kind) = choice;
        self.finished_exploring = false;
        if kind != GoalKind::Turn {
            self.spins_left = p.look_around_turns;
        }
        // a preempted goal is simply replaced: the navigator aborts it when it takes the new one,
        // and its result no longer matches the active goal, so it is not blacklisted
        self.active = Some(Active {
            goal,
            kind,
            frontier_missing: 0,
            view_target,
            accepted: false,
            started: now,
            progress_pose: (robot.x, robot.y, robot.yaw),
            last_progress: now,
            cancelling: false,
            cancel_ok: false,
        });
        let s = &mut self.stats;
        s.sent += 1;
        match kind {
            GoalKind::Frontier => s.sent_frontier += 1,
            GoalKind::View => s.sent_view += 1,
            GoalKind::Turn => s.sent_turn += 1,
        }
        if let Some((current, _)) = preempt {
            s.switched += 1;
            self.recent.push((current, now));
        }
        out.push(Action::SendGoal(GoalRequest {
            x: goal.0,
            y: goal.1,
            yaw,
            // the lidar sees all round: no stop to turn to a heading at a frontier
            xy_only: kind == GoalKind::Frontier,
            kind,
            preempts: preempt.is_some(),
        }));
    }

    fn set_viz(&mut self, frontiers: Vec<Frontier>, views: Vec<View>) {
        self.frontiers = frontiers;
        self.views = views;
        self.viz_seq += 1;
    }
}

fn dist(a: (f64, f64), b: (f64, f64)) -> f64 {
    (a.0 - b.0).hypot(a.1 - b.1)
}

#[cfg(test)]
mod tests {
    use super::*;
    use tb4_core::grid::{Cell, GridMeta, UNKNOWN};

    /// 4 x 4 m known free square in an 8 x 8 m unknown map, walls on the left and bottom
    fn map() -> Grid {
        let meta = GridMeta { width: 80, height: 80, resolution: 0.05, origin_x: 0.0, origin_y: 0.0 };
        let mut g = Grid::filled(meta, UNKNOWN);
        for y in 0..40 {
            for x in 0..40 {
                g.set(Cell { x, y }, 0);
            }
        }
        for i in 0..40 {
            g.set(Cell { x: 0, y: i }, 100);
            g.set(Cell { x: i, y: 0 }, 100);
        }
        g
    }

    fn params() -> ExplorerParams {
        ExplorerParams { coverage: None, ..ExplorerParams::default() }
    }

    fn input(g: &Grid, robot: Pose2, result: Option<((f64, f64), NavStatus)>, now: f64) -> TickInput<'_> {
        TickInput { costmap: Some(g), costmap_stamp: now, map: Some(g), robot: Ok(robot), result, done: false }
    }

    fn sent(actions: &[Action]) -> Option<GoalRequest> {
        actions.iter().find_map(|a| match a {
            Action::SendGoal(r) => Some(*r),
            _ => None,
        })
    }

    #[test]
    fn sends_xy_only_frontier_goal_and_chains_after_success() {
        let g = map();
        let mut e = ExplorerPolicy::new(params(), 0.0);
        let r = Pose2::new(1.0, 1.0, 0.0);
        let goal = sent(&e.tick(0.0, input(&g, r, None, 0.0))).expect("first goal");
        assert!(goal.xy_only && goal.kind == GoalKind::Frontier);
        // not accepted yet: no second goal while one is active
        assert!(sent(&e.tick(1.0, input(&g, r, None, 1.0))).is_none());
        e.goal_accepted((goal.x, goal.y));
        // success: the next goal goes out in the same tick, no retry delay
        let next = e.tick(2.0, input(&g, r, Some(((goal.x, goal.y), NavStatus::Succeeded)), 2.0));
        assert!(sent(&next).is_some());
        assert_eq!(e.stats().succeeded, 1);
    }

    #[test]
    fn failure_waits_retry_delay_and_blacklists() {
        let g = map();
        let mut e = ExplorerPolicy::new(params(), 0.0);
        let r = Pose2::new(1.0, 1.0, 0.0);
        let goal = sent(&e.tick(0.0, input(&g, r, None, 0.0))).unwrap();
        e.goal_accepted((goal.x, goal.y));
        let a = e.tick(1.0, input(&g, r, Some(((goal.x, goal.y), NavStatus::Aborted)), 1.0));
        assert!(sent(&a).is_none());
        assert_eq!(e.blacklist().count(), 1);
        assert!(sent(&e.tick(5.5, input(&g, r, None, 5.5))).is_none());
        let again = sent(&e.tick(6.5, input(&g, r, None, 6.5))).expect("goal after retry_delay");
        assert!((again.x - goal.x).hypot(again.y - goal.y) > e.params().explore.blacklist_radius);
    }

    #[test]
    fn no_progress_cancels_but_a_turn_counts_as_progress() {
        let g = map();
        let mut e = ExplorerPolicy::new(params(), 0.0);
        let goal = sent(&e.tick(0.0, input(&g, Pose2::new(1.0, 1.0, 0.0), None, 0.0))).unwrap();
        e.goal_accepted((goal.x, goal.y));
        let mut t = 0.0;
        let mut yaw = 0.0;
        // turning 0.4 rad every 10 s: never stalls
        while t < 40.0 {
            t += 1.0;
            if t % 10.0 == 0.0 {
                yaw += 0.4;
            }
            let a = e.tick(t, input(&g, Pose2::new(1.0, 1.0, yaw), None, t));
            assert!(!a.iter().any(|a| matches!(a, Action::Cancel(_))), "cancelled at {t}");
        }
        // standing still: cancelled after progress_timeout
        let mut cancelled = None;
        for k in 1..20 {
            let a = e.tick(t + k as f64, input(&g, Pose2::new(1.0, 1.0, yaw), None, t + k as f64));
            if a.contains(&Action::Cancel(CancelReason::NoProgress)) {
                cancelled = Some(k);
                break;
            }
        }
        assert_eq!(cancelled, Some(16));
    }

    #[test]
    fn stale_costmap_pauses() {
        let g = map();
        let mut e = ExplorerPolicy::new(params(), 0.0);
        let mut inp = input(&g, Pose2::new(1.0, 1.0, 0.0), None, 0.0);
        inp.costmap_stamp = -10.0;
        let a = e.tick(0.0, inp);
        assert!(sent(&a).is_none());
        assert!(a.iter().any(|a| matches!(a, Action::Warn(_))));
    }

    #[test]
    fn look_around_turn_when_nothing_to_explore() {
        let meta = GridMeta { width: 40, height: 40, resolution: 0.05, origin_x: 0.0, origin_y: 0.0 };
        let g = Grid::filled(meta, 0);
        let mut e = ExplorerPolicy::new(params(), 0.0);
        let r = Pose2::new(1.0, 1.0, 0.0);
        let turn = sent(&e.tick(0.0, input(&g, r, None, 0.0))).unwrap();
        assert_eq!(turn.kind, GoalKind::Turn);
        assert!(!turn.xy_only);
        assert!((turn.yaw - 2.0 * std::f64::consts::FRAC_PI_3).abs() < 1e-9);
    }

    #[test]
    fn frontier_goal_is_finished_while_its_frontier_is_open_then_switched_once_it_goes() {
        let mut g = map();
        let mut e = ExplorerPolicy::new(params(), 0.0);
        let goal = sent(&e.tick(0.0, input(&g, Pose2::new(1.0, 1.0, 0.0), None, 0.0))).unwrap();
        e.goal_accepted((goal.x, goal.y));
        // almost there but the frontier is still open: keep the goal (sim studio ping-pong)
        let near = Pose2::new(goal.x - 0.1, goal.y, 0.0);
        assert!(sent(&e.tick(1.0, input(&g, near, None, 1.0))).is_none(), "no switch while the frontier is open");
        assert_eq!(e.stats().switched, 0);
        // the frontier at the goal gets mapped (free all round it): switch on, and not back for a while
        let (cx, cy) = ((goal.x / 0.05) as i64, (goal.y / 0.05) as i64);
        for y in (cy - 30).max(1)..(cy + 30).min(80) {
            for x in (cx - 30).max(1)..(cx + 30).min(80) {
                g.set(Cell { x: x as u32, y: y as u32 }, 0);
            }
        }
        let mut next = None;
        for k in 2..5 {
            next = next.or(sent(&e.tick(k as f64, input(&g, near, None, k as f64))));
        }
        let next = next.expect("switch once the frontier is explored");
        assert!(next.preempts);
        assert_eq!(e.stats().switched, 1);
        assert!((next.x - goal.x).hypot(next.y - goal.y) > 0.5, "a different goal");
    }
}
