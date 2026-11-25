//! Explorer decision logic without ROS: the explorer node and the simulator both drive this.
//! Time is passed in as seconds on any monotonic clock, nothing here reads the wall clock.

use tb4_core::{DistanceField, Grid, Pose2};

use crate::approach::{self, Approach, ApproachParams, MarkerHint, Viewpoint};
use crate::{Coverage, CoverageParams, ExploreParams, Frontier, View, find_frontiers, select_goal_with_heading};

#[derive(Debug, Clone)]
pub struct ExplorerParams {
    /// s of approach-on-sight time allowed for the whole mission (0 = no cap). Once spent, no new
    /// approach goals start; frontier exploration carries on. Stops decoy markers eating the run.
    pub approach_budget: f64,
    /// s, a goal running longer than this is cancelled and blacklisted
    pub goal_timeout: f64,
    /// s without progress (progress_epsilon of movement or a 0.3 rad turn) before a goal is given up
    pub progress_timeout: f64,
    pub progress_epsilon: f64,
    /// s without getting `closing_epsilon` closer to the goal (best distance so far) before it is given
    /// up; 0 = off. Wiggling in front of a gap the robot cannot pass counts as movement, not as closing
    /// in (live 2026-11-25: three goals behind a half wall ran into the 60 s timeout)
    pub closing_timeout: f64,
    pub closing_epsilon: f64,
    /// s to wait after a failed or cancelled goal; a success chains on at once
    pub retry_delay: f64,
    pub blacklist_duration: f64,
    /// s, cap for a spot that keeps failing: its ban doubles with every failure up to this
    pub blacklist_max_duration: f64,
    /// m and s: a goal the navigator aborts (no path, goal blocked) bans every frontier and view goal
    /// this close for this long. One spot was not enough: behind glass the next frontier sat right
    /// beside the aborted one (live 2026-11-25, five goals outside the glass); 0 = off
    pub abort_ban_radius: f64,
    pub abort_ban_duration: f64,
    /// m, cap of the abort ban radius: an abort within 2 radii of active zones gets radius x (1 + their
    /// number), so a run of aborts in one area (outside the glass) covers it instead of leaving gaps
    pub abort_ban_max_radius: f64,
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
    /// approach on sight: markers the mapper holds but has not confirmed get close views from a few
    /// angles, before frontiers and camera views. None = off
    pub approach: Option<ApproachParams>,
    /// when no goal can be picked, try once more ignoring the bans on spots that failed only once
    /// and the recent switches (bans on spots with two or more failures still hold). Only changes
    /// the no-goal case, before the explorer waits for the map
    pub idle_fallback: bool,
}

impl Default for ExplorerParams {
    /// the explorer node's parameter defaults
    fn default() -> Self {
        Self {
            goal_timeout: 60.0,
            progress_timeout: 15.0,
            progress_epsilon: 0.1,
            closing_timeout: 30.0,
            closing_epsilon: 0.25,
            retry_delay: 5.0,
            approach_budget: 0.0,
            blacklist_duration: 45.0,
            blacklist_max_duration: 300.0,
            abort_ban_radius: 2.0,
            abort_ban_duration: 120.0,
            abort_ban_max_radius: 6.0,
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
                goal_max_unknown: 0.15,
                revisit_radius: 0.0,
                revisit_penalty: 0.0,
                min_free_fraction: 0.5,
                gain_weight: 0.0,
                gain_radius: 2.0,
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
            approach: Some(ApproachParams::default()),
            idle_fallback: false,
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
    /// close view of a marker that is not confirmed yet
    Approach,
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
    /// the marker of an approach goal got confirmed
    MarkerConfirmed,
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
    /// markers seen but not confirmed (empty when none, or approach is off)
    pub markers: &'a [MarkerHint],
}

#[derive(Debug, Clone, Default, PartialEq)]
pub struct ExplorerStats {
    pub sent: u64,
    pub sent_frontier: u64,
    pub sent_view: u64,
    pub sent_turn: u64,
    pub sent_approach: u64,
    /// approach goals cancelled because their marker got confirmed
    pub approach_confirmed: u64,
    /// markers given up (timeout, failed viewpoints, all viewpoints done)
    pub approach_given_up: u64,
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
    /// approach goal: the marker's serial and the viewpoint slot
    approach: Option<(u64, usize)>,
    /// the navigator accepted it (cancel is possible from then on)
    accepted: bool,
    started: f64,
    /// x, y, yaw where progress was last seen
    progress_pose: (f64, f64, f64),
    last_progress: f64,
    /// best distance to the goal so far and when it last improved by closing_epsilon
    closest: f64,
    last_closer: f64,
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
    /// goals that were reached (for the revisit penalty)
    reached: Vec<(f64, f64)>,
    /// approach-on-sight seconds spent this mission (see ExplorerParams::approach_budget)
    approach_spent: f64,
    last_tick: Option<f64>,
    active: Option<Active>,
    /// banned goals with the time the ban ends
    blacklist: Vec<((f64, f64), f64)>,
    /// areas around goals the navigator aborted, with the time the ban ends
    zones: Vec<((f64, f64), f64, f64)>,
    /// failures per spot over the run, so a spot that keeps failing is banned longer
    strikes: Vec<((f64, f64), u32)>,
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
    /// markers seen but not confirmed, and the ones given up
    approaches: Vec<Approach>,
    next_serial: u64,
}

impl ExplorerPolicy {
    pub fn new(p: ExplorerParams, now: f64) -> Self {
        Self {
            coverage: Coverage::new(),
            visited_views: Vec::new(),
            reached: Vec::new(),
            approach_spent: 0.0,
            last_tick: None,
            active: None,
            blacklist: Vec::new(),
            strikes: Vec::new(),
            zones: Vec::new(),
            recent: Vec::new(),
            next_goal_after: now,
            costmap_stale: false,
            finished_exploring: false,
            spins_left: p.look_around_turns,
            frontiers: Vec::new(),
            views: Vec::new(),
            viz_seq: 0,
            stats: ExplorerStats::default(),
            approaches: Vec::new(),
            next_serial: 0,
            p,
        }
    }

    /// markers the approach tracks: (position, given up)
    pub fn approach_markers(&self, now: f64) -> impl Iterator<Item = ((f64, f64), bool)> + '_ {
        self.approaches.iter().map(move |a| (a.pos, now < a.ignore_until))
    }

    /// camera reading distance for the coverage (the node scales it with the camera image width)
    pub fn set_camera_range(&mut self, range: f64) {
        if let Some(c) = self.p.coverage.as_mut() {
            c.range = range;
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

    /// Bans a failed goal. Three goals failing in turn outlasted a fixed 45 s ban and cycled
    /// to the end of the 183647 replay, so each new failure at the same spot doubles the ban.
    fn ban(&mut self, goal: (f64, f64), now: f64) {
        let r = self.p.explore.blacklist_radius;
        let n = match self.strikes.iter_mut().find(|(g, _)| dist(*g, goal) < r) {
            Some((_, n)) => {
                *n += 1;
                *n
            }
            None => {
                self.strikes.push((goal, 1));
                1
            }
        };
        let cap = self.p.blacklist_max_duration.max(self.p.blacklist_duration);
        let secs = (self.p.blacklist_duration * 2f64.powi(n as i32 - 1)).min(cap);
        self.blacklist.push((goal, now + secs));
    }

    /// areas no goal is sent to now: (centre, radius m, true = navigator abort zone, false = failed spot)
    pub fn banned(&self) -> Vec<((f64, f64), f64, bool)> {
        let spot = self.p.explore.blacklist_radius;
        self.zones
            .iter()
            .map(|(c, r, _)| (*c, *r, true))
            .chain(self.blacklist.iter().map(|(c, _)| (*c, spot, false)))
            .collect()
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
            a.cancel_ok = matches!(why, CancelReason::WallSeen | CancelReason::MarkerConfirmed);
            out.push(Action::Cancel(why));
            let s = &mut self.stats;
            match why {
                CancelReason::Timeout => s.cancel_timeout += 1,
                CancelReason::NoProgress => s.cancel_no_progress += 1,
                CancelReason::WallSeen => s.cancel_wall_seen += 1,
                CancelReason::MarkerConfirmed => s.approach_confirmed += 1,
                _ => s.cancel_other += 1,
            }
        }
    }

    fn approach_entry(&mut self, serial: u64) -> Option<&mut Approach> {
        self.approaches.iter_mut().find(|a| a.serial == serial)
    }

    fn give_up(&mut self, serial: u64, now: f64, why: &str, out: &mut Vec<Action>) {
        let Some(ap) = self.p.approach.clone() else { return };
        if let Some(a) = self.approach_entry(serial) {
            a.ignore_until = now + ap.blacklist_duration;
            a.started = None;
            a.tried = [false; 3];
            a.failures = 0;
            let label = a.label();
            self.stats.approach_given_up += 1;
            out.push(Action::Info(format!("approach: giving up on {label} for {:.0} s: {why}", ap.blacklist_duration)));
        }
    }

    /// outcome of an approach goal for its marker
    fn approach_result(&mut self, serial: u64, slot: usize, status: NavStatus, cancel_ok: bool, now: f64, out: &mut Vec<Action>) {
        let max_failures = self.p.approach.as_ref().map_or(0, |a| a.max_failures);
        let Some(a) = self.approach_entry(serial) else { return };
        if status == NavStatus::Succeeded {
            a.tried[slot] = true;
        } else if !cancel_ok {
            a.tried[slot] = true;
            a.failures += 1;
            if a.failures >= max_failures {
                self.give_up(serial, now, "viewpoints failed", out);
            }
        }
    }

    /// approach on sight: update the markers from the hints, cancel an approach whose marker got
    /// confirmed or timed out, and return the best viewpoint to go to next with its marker
    fn approach_step(
        &mut self,
        now: f64,
        markers: &[MarkerHint],
        grid: Option<&Grid>,
        robot: Pose2,
        out: &mut Vec<Action>,
    ) -> Option<(Viewpoint, u64)> {
        let ap = self.p.approach.clone()?;
        // the budget only stops new approach trips: an approach already running is still cancelled when its
        // marker is confirmed or times out (returning here used to leave a confirmed marker's goal running)
        let budget_spent = self.p.approach_budget > 0.0 && self.approach_spent >= self.p.approach_budget;
        approach::sync(&mut self.approaches, markers, &ap, now, &mut self.next_serial);
        let active = self.active.as_ref().filter(|a| a.accepted && !a.cancelling).and_then(|a| a.approach);
        if let Some((serial, _)) = active
            && self.approach_entry(serial).is_none()
        {
            out.push(Action::Info("approach: marker confirmed, cancelling its view".into()));
            self.cancel_active(CancelReason::MarkerConfirmed, out);
        }
        // time out, or every viewpoint done without a confirmation
        let expired: Vec<(u64, &str)> = self
            .approaches
            .iter()
            .filter(|a| a.present && now >= a.ignore_until)
            .filter_map(|a| {
                if a.started.is_some_and(|s| now - s > ap.timeout) {
                    Some((a.serial, "timeout"))
                } else if a.tried.iter().all(|t| *t) {
                    Some((a.serial, "all viewpoints done, still not confirmed"))
                } else {
                    None
                }
            })
            .collect();
        for (serial, why) in expired {
            if active.is_some_and(|(s, _)| s == serial) {
                self.cancel_active(CancelReason::Timeout, out);
            }
            self.give_up(serial, now, why, out);
        }
        let grid = grid?;
        if budget_spent {
            return None;
        }
        let open: Vec<usize> = (0..self.approaches.len())
            .filter(|&i| self.approaches[i].present && now >= self.approaches[i].ignore_until)
            .collect();
        if open.is_empty() {
            return None;
        }
        let clearance = DistanceField::from_grid(grid, ap.occupied_threshold, true);
        let mut best: Option<(Viewpoint, u64)> = None;
        let cost = |v: &Viewpoint| robot.distance_to(v.x, v.y);
        for i in open {
            let a = &mut self.approaches[i];
            let mut vps = approach::viewpoints(a, grid, &clearance, &ap);
            // standing at one already: it counts as done
            for v in vps.iter().filter(|v| robot.distance_to(v.x, v.y) < 0.25 && (robot.yaw - v.yaw).sin().atan2((robot.yaw - v.yaw).cos()).abs() < 0.35) {
                a.tried[v.slot] = true;
            }
            vps.retain(|v| !a.tried[v.slot]);
            if let Some(v) = approach::nearest(&vps, robot, self.p.turn_weight)
                && best.is_none_or(|(b, _)| cost(&v) < cost(&b))
            {
                best = Some((v, a.serial));
            }
        }
        best
    }

    fn tick_inner(&mut self, now: f64, input: TickInput, out: &mut Vec<Action>) {
        // mission-wide approach time: count the time since the last tick while an approach goal is accepted
        if let Some(prev) = self.last_tick
            && self.active.as_ref().is_some_and(|a| a.kind == GoalKind::Approach && a.accepted && !a.cancelling)
        {
            self.approach_spent += (now - prev).clamp(0.0, 1.0);
        }
        self.last_tick = Some(now);
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
            let cancel_ok = self.active.as_ref().is_some_and(|a| a.cancel_ok);
            let ok = status == NavStatus::Succeeded || (status == NavStatus::Canceled && cancel_ok);
            let finished = self.active.take();
            // a reached view whose wall still counts as unseen must not be picked again
            if ok {
                self.reached.push(goal);
            }
            if let Some(t) = finished.as_ref().and_then(|a| a.view_target) {
                self.visited_views.push(t);
            }
            if let Some((key, slot)) = finished.and_then(|a| a.approach) {
                self.approach_result(key, slot, status, cancel_ok, now, out);
            }
            let p = &self.p;
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
                    let (base, secs) = (p.abort_ban_radius, p.abort_ban_duration);
                    let near = self.zones.iter().filter(|(c, _, _)| dist(*c, goal) <= 2.0 * base).count();
                    let radius = (base * (1 + near) as f64).min(p.abort_ban_max_radius.max(base));
                    self.ban(goal, now);
                    if base > 0.0 {
                        self.zones.push((goal, radius, now + secs));
                        out.push(Action::Info(format!(
                            "navigator aborted ({:.2}, {:.2}): no goals within {radius:.1} m for {secs:.0} s",
                            goal.0, goal.1
                        )));
                    }
                }
            }
        }

        let p = &self.p;
        self.blacklist.retain(|(_, until)| now < *until);
        self.zones.retain(|(_, _, until)| now < *until);
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
        let mut frontiers = find_frontiers(grid, (robot.x, robot.y), &p.explore);
        // revisit penalty: frontiers near a goal already reached score lower, so new places come first
        if p.explore.revisit_radius > 0.0 && p.explore.revisit_penalty > 0.0 && !self.reached.is_empty() {
            for f in frontiers.iter_mut() {
                if self.reached.iter().any(|r| dist(*r, f.goal) < p.explore.revisit_radius) {
                    f.score -= p.explore.revisit_penalty;
                }
            }
            frontiers.sort_by(|a, b| b.score.total_cmp(&a.score));
        }
        self.recent.retain(|(_, at)| now - at < p.switch_cooldown);
        // goals switched away from lately are skipped like blacklisted ones (no ping-pong)
        let blacklist: Vec<_> = self.blacklist.iter().chain(&self.recent).map(|(goal, _)| *goal).collect();
        let in_zone = |pt: (f64, f64)| self.zones.iter().any(|(z, r, _)| dist(*z, pt) <= *r);
        let pickable: Vec<Frontier> = frontiers.iter().filter(|f| !in_zone(f.goal)).cloned().collect();
        let pick = || select_goal_with_heading(&pickable, &blacklist, &p.explore, (robot.x, robot.y), robot.yaw, p.turn_weight);
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
                .filter(|v| !in_zone((v.x, v.y)))
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
            let d = robot.distance_to(a.goal.0, a.goal.1);
            if d < a.closest - p.closing_epsilon {
                a.closest = d;
                a.last_closer = now;
            }
            // a turn in place has its goal under the robot: only goals that need driving
            let closing_stalled = p.closing_timeout > 0.0
                && a.kind != GoalKind::Turn
                && a.closest > p.closing_epsilon
                && now - a.last_closer > p.closing_timeout;
            if !a.cancelling && a.accepted {
                if now - a.started > p.goal_timeout {
                    abandon = Some((a.goal, CancelReason::Timeout, true));
                } else if now - a.last_progress > p.progress_timeout || closing_stalled {
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
                self.ban(goal, now);
            }
            self.cancel_active(why, out);
        }
        // Approach on sight comes first: a marker the mapper holds but has not confirmed is looked at
        // from close by and from a few angles. It replaces a frontier, view or turn goal at once.
        let approach_choice = self.approach_step(now, input.markers, input.map.or(Some(grid)), robot, out);
        let mut approach_goal = None;
        if abandon.is_none()
            && let Some((vp, key)) = approach_choice
        {
            let replace = self.active.as_ref().is_some_and(|a| a.kind != GoalKind::Approach && a.accepted && !a.cancelling);
            if replace || (self.active.is_none() && now >= self.next_goal_after) {
                approach_goal = Some((vp, key, replace));
            }
        }
        // Keep moving: a frontier goal whose frontier was already seen on the way, or one that is almost
        // reached, is replaced by the next goal at once. The navigator takes the new goal without
        // stopping (it keeps the MPPI warm start), so the robot does not halt at every frontier.
        let mut preempt = None;
        if abandon.is_none()
            && approach_goal.is_none()
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
        let choice = if let Some((vp, key, replace)) = approach_goal {
            let label = self.approach_entry(key).map_or_else(String::new, |a| {
                a.started.get_or_insert(now);
                a.label()
            });
            let offset = approach::OFFSETS[vp.slot] * p.approach.as_ref().map_or(0.0, |a| a.spread);
            out.push(Action::Info(format!(
                "new goal ({:.2}, {:.2}) yaw={:.2}: approach {label}, view at {:+.0} deg{}",
                vp.x,
                vp.y,
                vp.yaw,
                offset.to_degrees(),
                if replace { ", replacing the current goal" } else { "" }
            )));
            ((vp.x, vp.y), vp.yaw, None, GoalKind::Approach)
        } else if let Some((current, why)) = preempt {
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
            } else if p.idle_fallback
                && let Some(f) = self.idle_fallback_pick(&frontiers, robot, &p)
            {
                let yaw = (f.centroid.1 - f.goal.1).atan2(f.centroid.0 - f.goal.0);
                out.push(Action::Info(format!(
                    "idle fallback: new goal ({:.2}, {:.2}) yaw={yaw:.2}, frontier {:.1} m, free-space path {:.1} m; \
                     ignored bans with one failure and recent switches",
                    f.goal.0, f.goal.1, f.size_m, f.distance
                )));
                (f.goal, yaw, None, GoalKind::Frontier)
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
            approach: approach_goal.map(|(vp, key, _)| (key, vp.slot)),
            accepted: false,
            started: now,
            progress_pose: (robot.x, robot.y, robot.yaw),
            last_progress: now,
            closest: robot.distance_to(goal.0, goal.1),
            last_closer: now,
            cancelling: false,
            cancel_ok: false,
        });
        let s = &mut self.stats;
        s.sent += 1;
        match kind {
            GoalKind::Frontier => s.sent_frontier += 1,
            GoalKind::View => s.sent_view += 1,
            GoalKind::Turn => s.sent_turn += 1,
            GoalKind::Approach => s.sent_approach += 1,
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
            preempts: preempt.is_some() || approach_goal.is_some_and(|(_, _, replace)| replace),
        }));
    }

    /// the no-goal fallback: frontiers that pass the bans on spots with two or more failures, ignoring
    /// the other bans and the recent switches
    fn idle_fallback_pick<'a>(&self, frontiers: &'a [Frontier], robot: Pose2, p: &ExplorerParams) -> Option<&'a Frontier> {
        let r = p.explore.blacklist_radius;
        let kept: Vec<(f64, f64)> = self
            .blacklist
            .iter()
            .map(|(goal, _)| *goal)
            .filter(|goal| self.strikes.iter().find(|(g, _)| dist(*g, *goal) < r).is_some_and(|(_, n)| *n >= 2))
            .collect();
        select_goal_with_heading(frontiers, &kept, &p.explore, (robot.x, robot.y), robot.yaw, p.turn_weight)
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
        TickInput { costmap: Some(g), costmap_stamp: now, map: Some(g), robot: Ok(robot), result, done: false, markers: &[] }
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
    fn aborts_in_one_area_grow_the_ban() {
        let mut e = ExplorerPolicy::new(params(), 0.0);
        let base = e.params().abort_ban_radius;
        let g = map();
        let r = Pose2::new(1.0, 1.0, 0.0);
        let mut abort = |e: &mut ExplorerPolicy, at: (f64, f64), t: f64| {
            e.active = Some(Active {
                goal: at, kind: GoalKind::Frontier, frontier_missing: 0, view_target: None, approach: None,
                accepted: true, started: t, progress_pose: (r.x, r.y, r.yaw), last_progress: t, closest: 9.0,
                last_closer: t, cancelling: false, cancel_ok: false,
            });
            e.tick(t, input(&g, r, Some((at, NavStatus::Aborted)), t));
        };
        abort(&mut e, (5.0, 5.0), 1.0);
        abort(&mut e, (6.0, 5.5), 2.0);
        abort(&mut e, (7.0, 6.0), 3.0);
        let radii: Vec<f64> = e.banned().iter().filter(|b| b.2).map(|b| b.1).collect();
        assert_eq!(radii, vec![base, 2.0 * base, 3.0 * base]);
        // far away: the base radius again
        abort(&mut e, (-20.0, -20.0), 4.0);
        assert_eq!(e.banned().iter().filter(|b| b.2).map(|b| b.1).last(), Some(base));
    }

    #[test]
    fn an_aborted_goal_bans_the_area_around_it() {
        let g = map();
        let r = Pose2::new(1.0, 1.0, 0.0);
        // all goals the policy offers on this map, without any ban
        let mut free = ExplorerPolicy::new(params(), 0.0);
        let first = sent(&free.tick(0.0, input(&g, r, None, 0.0))).unwrap();
        let mut e = ExplorerPolicy::new(params(), 0.0);
        let goal = sent(&e.tick(0.0, input(&g, r, None, 0.0))).unwrap();
        assert_eq!((goal.x, goal.y), (first.x, first.y));
        e.goal_accepted((goal.x, goal.y));
        e.tick(1.0, input(&g, r, None, 1.0));
        let acts = e.tick(2.0, input(&g, r, Some(((goal.x, goal.y), NavStatus::Aborted)), 2.0));
        assert!(acts.iter().any(|a| matches!(a, Action::Info(t) if t.contains("no goals within"))));
        let radius = e.params().abort_ban_radius;
        for k in 3..200 {
            // a look-around turn in place is fine; a frontier or view goal must be outside the zone
            if let Some(next) = sent(&e.tick(k as f64, input(&g, r, None, k as f64)))
                && next.kind != GoalKind::Turn
            {
                assert!((next.x - goal.x).hypot(next.y - goal.y) > radius, "{next:?} too close to {goal:?}");
            }
        }
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
    fn repeated_failures_at_one_spot_ban_longer() {
        let mut e = ExplorerPolicy::new(params(), 0.0);
        let d = e.params().blacklist_duration;
        e.ban((1.0, 1.0), 0.0);
        assert!((e.blacklist[0].1 - d).abs() < 1e-9);
        e.blacklist.clear();
        e.ban((1.1, 1.0), 100.0);
        assert!((e.blacklist[0].1 - (100.0 + 2.0 * d)).abs() < 1e-9);
        for t in 0..5 {
            e.ban((1.0, 1.1), 200.0 + t as f64);
        }
        let cap = e.params().blacklist_max_duration;
        assert!(e.blacklist.iter().all(|(_, until)| *until <= 204.0 + cap + 1e-9));
        // another spot starts at the base ban again
        e.ban((5.0, 5.0), 300.0);
        assert!((e.blacklist.last().unwrap().1 - (300.0 + d)).abs() < 1e-9);
    }

    #[test]
    fn wiggling_without_closing_in_is_given_up() {
        let g = map();
        let mut e = ExplorerPolicy::new(params(), 0.0);
        let start = Pose2::new(1.0, 1.0, 0.0);
        let goal = sent(&e.tick(0.0, input(&g, start, None, 0.0))).unwrap();
        e.goal_accepted((goal.x, goal.y));
        // back and forth 0.3 m across the line to the goal: moving, never closer
        let (dx, dy) = (goal.x - start.x, goal.y - start.y);
        let n = dx.hypot(dy);
        let (px, py) = (-dy / n, dx / n);
        let mut cancelled = None;
        for k in 1..60 {
            let s = if k % 2 == 0 { 0.15 } else { -0.15 };
            let r = Pose2::new(start.x + px * s, start.y + py * s, (k as f64 * 0.5).sin());
            if e.tick(k as f64, input(&g, r, None, k as f64)).contains(&Action::Cancel(CancelReason::NoProgress)) {
                cancelled = Some(k);
                break;
            }
        }
        assert!(cancelled.is_some_and(|k| (30..=32).contains(&k)), "{cancelled:?}");
        // driving at the goal steadily is never cut off
        let mut e = ExplorerPolicy::new(params(), 0.0);
        let goal = sent(&e.tick(0.0, input(&g, start, None, 0.0))).unwrap();
        e.goal_accepted((goal.x, goal.y));
        let n = (goal.x - start.x).hypot(goal.y - start.y);
        for k in 1..(n / 0.02) as i32 {
            let f = 0.02 * k as f64 / n;
            let r = Pose2::new(start.x + (goal.x - start.x) * f, start.y + (goal.y - start.y) * f, 0.0);
            let a = e.tick(k as f64, input(&g, r, None, k as f64));
            assert!(!a.iter().any(|a| matches!(a, Action::Cancel(_))), "cancelled at {k}");
        }
    }

    #[test]
    fn no_progress_cancels_but_a_turn_counts_as_progress() {
        let g = map();
        // the closing check would end this turn-in-place goal at 30 s; this test is about movement
        let mut e = ExplorerPolicy::new(ExplorerParams { closing_timeout: 0.0, ..params() }, 0.0);
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

    /// fully known free map with one unknown column on the east side: a single frontier
    fn one_frontier_map() -> Grid {
        let meta = GridMeta { width: 80, height: 80, resolution: 0.05, origin_x: 0.0, origin_y: 0.0 };
        let mut g = Grid::filled(meta, 0);
        for y in 0..80 {
            g.set(Cell { x: 79, y }, UNKNOWN);
        }
        g
    }

    #[test]
    fn idle_fallback_sends_goal_when_only_candidate_has_one_strike() {
        let g = one_frontier_map();
        let robot = Pose2::new(0.5, 0.5, 0.0);
        let p = ExplorerParams { look_around_turns: 0, idle_fallback: true, ..params() };
        let goal = find_frontiers(&g, (robot.x, robot.y), &p.explore)[0].goal;
        let mut e = ExplorerPolicy::new(p.clone(), 0.0);
        e.ban(goal, 0.0);
        assert_eq!(e.strikes.iter().map(|(_, n)| *n).collect::<Vec<_>>(), vec![1]);
        // fallback off: the banned spot is the only candidate and the explorer waits
        let mut off = ExplorerPolicy::new(ExplorerParams { idle_fallback: false, ..p.clone() }, 0.0);
        off.ban(goal, 0.0);
        let acts = off.tick(10.0, input(&g, robot, None, 10.0));
        assert!(sent(&acts).is_none());
        assert!(acts.iter().any(|a| matches!(a, Action::Info(t) if t.contains("waiting for map updates"))));
        // fallback on: one strike is ignored and the goal goes out, logged as an idle fallback
        let acts = e.tick(10.0, input(&g, robot, None, 10.0));
        let req = sent(&acts).expect("idle fallback goal");
        assert_eq!(req.kind, GoalKind::Frontier);
        assert!((req.x - goal.0).hypot(req.y - goal.1) < 1e-9);
        assert!(acts.iter().any(|a| matches!(a, Action::Info(t) if t.contains("idle fallback"))));
    }

    #[test]
    fn idle_fallback_keeps_spots_with_two_strikes_banned() {
        let g = one_frontier_map();
        let robot = Pose2::new(0.5, 0.5, 0.0);
        let p = ExplorerParams { look_around_turns: 0, ..params() };
        let goal = find_frontiers(&g, (robot.x, robot.y), &p.explore)[0].goal;
        let mut e = ExplorerPolicy::new(p, 0.0);
        e.ban(goal, 0.0);
        e.ban(goal, 0.0);
        let acts = e.tick(10.0, input(&g, robot, None, 10.0));
        assert!(sent(&acts).is_none(), "two failures keep the spot banned: {acts:?}");
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

    /// 4 x 4 m known free square in a 6 x 6 m unknown map, walls on the left and bottom
    fn room() -> Grid {
        let meta = GridMeta { width: 120, height: 120, resolution: 0.05, origin_x: 0.0, origin_y: 0.0 };
        let mut g = Grid::filled(meta, UNKNOWN);
        for y in 0..80 {
            for x in 0..80 {
                g.set(Cell { x, y }, 0);
            }
        }
        for i in 0..80 {
            g.set(Cell { x: 0, y: i }, 100);
            g.set(Cell { x: i, y: 0 }, 100);
        }
        g
    }

    fn approach_params() -> ExplorerParams {
        ExplorerParams { approach: Some(ApproachParams::default()), ..params() }
    }

    /// marker on the left wall at y = 2, facing east (into the room)
    const HINT: MarkerHint = MarkerHint { id: Some(7), x: 0.05, y: 2.0, facing: 0.0 };

    fn with_markers<'a>(mut i: TickInput<'a>, m: &'a [MarkerHint]) -> TickInput<'a> {
        i.markers = m;
        i
    }

    fn check_view(g: &GoalRequest, h: &MarkerHint) {
        assert_eq!(g.kind, GoalKind::Approach);
        assert!(!g.xy_only, "approach views keep their heading");
        let d = (g.x - h.x).hypot(g.y - h.y);
        assert!(d > 0.9 && d < 2.0, "{d}");
        let to_marker = (h.y - g.y).atan2(h.x - g.x);
        assert!((g.yaw - to_marker).sin().abs() < 1e-6 && (g.yaw - to_marker).cos() > 0.0, "faces the marker");
    }

    #[test]
    fn approach_replaces_a_frontier_goal_and_looks_from_three_angles() {
        let g = room();
        let mut e = ExplorerPolicy::new(approach_params(), 0.0);
        let r = Pose2::new(2.5, 1.0, 0.0);
        let f = sent(&e.tick(0.0, input(&g, r, None, 0.0))).unwrap();
        assert_eq!(f.kind, GoalKind::Frontier);
        e.goal_accepted((f.x, f.y));
        let hints = [HINT];
        let mut views = Vec::new();
        let a = sent(&e.tick(1.0, with_markers(input(&g, r, None, 1.0), &hints))).expect("approach goal");
        check_view(&a, &HINT);
        assert!(a.preempts, "replaces the frontier goal without a cancel");
        views.push(a);
        // each success chains on to the next angle at once
        for k in 0..2 {
            let last = *views.last().unwrap();
            e.goal_accepted((last.x, last.y));
            let at = Pose2::new(last.x, last.y, last.yaw);
            let t = 2.0 + k as f64;
            let n = sent(&e.tick(t, with_markers(input(&g, at, Some(((last.x, last.y), NavStatus::Succeeded)), t), &hints)))
                .expect("next view");
            check_view(&n, &HINT);
            views.push(n);
        }
        // three different angles around the facing direction
        let mut angles: Vec<f64> = views.iter().map(|v| (v.y - HINT.y).atan2(v.x - HINT.x)).collect();
        angles.sort_by(f64::total_cmp);
        assert!(angles[0] < -0.4 && angles[1].abs() < 0.1 && angles[2] > 0.4, "{angles:?}");
        assert_eq!(e.stats().sent_approach, 3);
        // all three done and still not confirmed: given up, exploration goes on
        let last = views[2];
        e.goal_accepted((last.x, last.y));
        let next = sent(&e.tick(5.0, with_markers(input(&g, r, Some(((last.x, last.y), NavStatus::Succeeded)), 5.0), &hints)));
        assert_ne!(next.map(|n| n.kind), Some(GoalKind::Approach));
        assert_eq!(e.stats().approach_given_up, 1);
    }

    #[test]
    fn confirmation_cancels_the_approach_and_exploration_goes_on_at_once() {
        let g = room();
        let mut e = ExplorerPolicy::new(approach_params(), 0.0);
        let r = Pose2::new(2.5, 1.0, 0.0);
        let hints = [HINT];
        let a = sent(&e.tick(0.0, with_markers(input(&g, r, None, 0.0), &hints))).unwrap();
        assert_eq!(a.kind, GoalKind::Approach);
        e.goal_accepted((a.x, a.y));
        // the mapper confirmed it: the marker leaves the list
        let c = e.tick(1.0, input(&g, r, None, 1.0));
        assert!(c.contains(&Action::Cancel(CancelReason::MarkerConfirmed)), "{c:?}");
        // the cancel is not a failure: no retry delay, no blacklist
        let next = sent(&e.tick(2.0, input(&g, r, Some(((a.x, a.y), NavStatus::Canceled)), 2.0))).expect("next goal at once");
        assert_ne!(next.kind, GoalKind::Approach);
        assert_eq!(e.blacklist().count(), 0);
        assert_eq!(e.stats().approach_confirmed, 1);
    }

    #[test]
    fn a_jumping_estimate_with_the_same_id_keeps_its_approach() {
        let g = room();
        let mut e = ExplorerPolicy::new(approach_params(), 0.0);
        let r = Pose2::new(2.5, 1.0, 0.0);
        let a = sent(&e.tick(0.0, with_markers(input(&g, r, None, 0.0), &[HINT]))).unwrap();
        e.goal_accepted((a.x, a.y));
        let moved = [MarkerHint { y: 3.0, ..HINT }];
        let acts = e.tick(1.0, with_markers(input(&g, r, None, 1.0), &moved));
        assert!(!acts.iter().any(|a| matches!(a, Action::Cancel(_))), "{acts:?}");
        // without ids (the ROS hints) a jump past match_radius reads as a confirmation
        let no_id = [MarkerHint { id: None, ..HINT }];
        let mut e = ExplorerPolicy::new(approach_params(), 0.0);
        let a = sent(&e.tick(0.0, with_markers(input(&g, r, None, 0.0), &no_id))).unwrap();
        e.goal_accepted((a.x, a.y));
        let moved = [MarkerHint { id: None, y: 3.0, ..HINT }];
        let acts = e.tick(1.0, with_markers(input(&g, r, None, 1.0), &moved));
        assert!(acts.contains(&Action::Cancel(CancelReason::MarkerConfirmed)));
    }

    #[test]
    fn a_decoy_cannot_trap_the_robot() {
        let g = room();
        let mut e = ExplorerPolicy::new(approach_params(), 0.0);
        let r = Pose2::new(2.5, 1.0, 0.0);
        // a pattern in mid-air that never confirms, and whose views keep failing
        let hints = [MarkerHint { id: Some(1023), x: 1.0, y: 3.0, facing: 0.0 }];
        let mut t = 0.0;
        for _ in 0..2 {
            let a = loop {
                t += 1.0;
                if let Some(a) = sent(&e.tick(t, with_markers(input(&g, r, None, t), &hints))) {
                    break a;
                }
                assert!(t < 30.0, "no approach goal");
            };
            assert_eq!(a.kind, GoalKind::Approach);
            e.goal_accepted((a.x, a.y));
            t += 1.0;
            e.tick(t, with_markers(input(&g, r, Some(((a.x, a.y), NavStatus::Aborted)), t), &hints));
        }
        assert_eq!(e.stats().approach_given_up, 1);
        // ignored from now on while it stays tentative
        for _ in 0..20 {
            t += 1.0;
            let a = e.tick(t, with_markers(input(&g, r, None, t), &hints));
            assert!(sent(&a).is_none_or(|g| g.kind != GoalKind::Approach), "{a:?}");
            if let Some(g) = sent(&a) {
                e.goal_accepted((g.x, g.y));
            }
        }
    }

    #[test]
    fn approach_times_out() {
        let g = room();
        // a goal timeout longer than the approach timeout
        // closing check off: this test is about the approach timeout
        let mut e = ExplorerPolicy::new(ExplorerParams { goal_timeout: 200.0, closing_timeout: 0.0, ..approach_params() }, 0.0);
        let r = Pose2::new(2.5, 1.0, 0.0);
        let hints = [HINT];
        let a = sent(&e.tick(0.0, with_markers(input(&g, r, None, 0.0), &hints))).unwrap();
        e.goal_accepted((a.x, a.y));
        // keeps turning (progress) but never gets there
        let mut cancelled = None;
        for k in 1..120 {
            let yaw = 0.4 * k as f64;
            let acts = e.tick(k as f64, with_markers(input(&g, Pose2::new(2.5, 1.0, yaw), None, k as f64), &hints));
            if acts.contains(&Action::Cancel(CancelReason::Timeout)) {
                cancelled = Some(k);
                break;
            }
        }
        let k = cancelled.expect("approach timeout");
        assert!((90..=92).contains(&k), "{k}");
        assert_eq!(e.stats().approach_given_up, 1);
    }

    #[test]
    fn viewpoints_skip_obstacles_and_hints_are_ignored_when_off() {
        let mut g = room();
        // a box where the head-on view at 1.3 m (and 1.04 m, 1.625 m) would stand
        for y in 36..44 {
            for x in 16..38 {
                g.set(Cell { x, y }, 100);
            }
        }
        let mut e = ExplorerPolicy::new(approach_params(), 0.0);
        let r = Pose2::new(2.5, 1.0, 0.0);
        let hints = [HINT];
        let a = sent(&e.tick(0.0, with_markers(input(&g, r, None, 0.0), &hints))).unwrap();
        check_view(&a, &HINT);
        assert!((a.y - HINT.y).abs() > 0.4, "not head-on through the box: ({:.2}, {:.2})", a.x, a.y);
        let mut off = ExplorerPolicy::new(ExplorerParams { approach: None, ..params() }, 0.0);
        let f = sent(&off.tick(0.0, with_markers(input(&g, r, None, 0.0), &hints))).unwrap();
        assert_ne!(f.kind, GoalKind::Approach);
    }
}
