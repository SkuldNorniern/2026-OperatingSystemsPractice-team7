//! Run metrics: mapped area over time, motion time split, stall episodes, goals, collisions, markers.

use std::collections::BTreeMap;
use std::fmt::Write as _;

use tb4_explore::ExplorerStats;

/// below these the robot counts as not moving / not turning (m/s, rad/s)
pub const STILL_V: f64 = 0.02;
pub const STILL_W: f64 = 0.08;
/// a goal is active and the robot neither drives nor turns for longer than this: a stall (s)
pub const STALL_MIN: f64 = 2.0;
/// mapped area is sampled at these times (s)
pub const AREA_TIMES: [f64; 5] = [60.0, 120.0, 180.0, 300.0, 600.0];

#[derive(Debug, Clone, PartialEq)]
pub struct StallEpisode {
    pub start: f64,
    pub end: f64,
    /// most frequent cause during the episode
    pub cause: String,
    pub goal: Option<(f64, f64)>,
}

impl StallEpisode {
    pub fn duration(&self) -> f64 {
        self.end - self.start
    }
}

#[derive(Debug, Clone, Default)]
pub struct StallTracker {
    start: Option<f64>,
    goal: Option<(f64, f64)>,
    causes: BTreeMap<String, f64>,
    pub episodes: Vec<StallEpisode>,
}

impl StallTracker {
    /// one step of `dt` ending at `t`; `stalled`: goal active and robot still
    pub fn update(&mut self, t: f64, dt: f64, stalled: bool, cause: &str, goal: Option<(f64, f64)>) {
        if stalled {
            if self.start.is_none() {
                self.start = Some(t - dt);
                self.goal = goal;
                self.causes.clear();
            }
            *self.causes.entry(cause.to_owned()).or_default() += dt;
        } else {
            self.close(t - dt);
        }
    }

    /// end an open episode at `t` (also at the end of the run)
    pub fn close(&mut self, t: f64) {
        if let Some(start) = self.start.take()
            && t - start > STALL_MIN
        {
            let cause = self.causes.iter().max_by(|a, b| a.1.total_cmp(b.1)).map(|(c, _)| c.clone()).unwrap_or_default();
            self.episodes.push(StallEpisode { start, end: t, cause, goal: self.goal });
        }
    }

    pub fn total(&self) -> f64 {
        self.episodes.iter().map(StallEpisode::duration).fold(0.0, |a, b| a + b)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Motion {
    Driving,
    Turning,
    Still,
}

pub fn classify(v: f64, w: f64) -> Motion {
    if v.abs() >= STILL_V {
        Motion::Driving
    } else if w.abs() >= STILL_W {
        Motion::Turning
    } else {
        Motion::Still
    }
}

#[derive(Debug, Clone, Default, PartialEq)]
pub struct MarkerRecord {
    pub id: u32,
    pub size: f64,
    pub first_readable: Option<f64>,
    /// first time the explorer's camera coverage counted the marker's wall cell as seen; before
    /// `first_readable` (or with it never read) the coverage claimed a wall the camera never read
    pub coverage_seen: Option<f64>,
    pub first_within_2m: Option<f64>,
    /// camera frames the marker was readable in
    pub readable_frames: u64,
    /// readable frames from a new viewpoint (camera moved 0.05 m or turned 0.05 rad), like the mapper's views
    pub views: u64,
    /// time of the 5th view (marker_mapper min_views), a rough stand-in for confirmation
    pub views5_at: Option<f64>,
    pub max_side_px: f64,
    pub closest: f64,
    /// marker pipeline (simulated detections into tb4_markers::Tracker): frames with a detection
    pub detections: u64,
    /// first time the tracker held an estimate for this id
    pub first_tentative: Option<f64>,
    pub first_confirmed: Option<f64>,
    /// fused position error vs truth in the map frame (m), when first confirmed and at the end
    pub err_confirmed: Option<f64>,
    pub err_end: Option<f64>,
    /// confirmed at the end of the run (a confirmation can be lost again)
    pub confirmed_end: bool,
    /// tracker views at the end
    pub tracker_views: usize,
}

#[derive(Debug, Clone, Default)]
pub struct Metrics {
    pub world: String,
    pub seed: u64,
    pub perfect_pose: bool,
    pub duration: f64,
    pub wall_time: f64,
    pub distance: f64,
    pub time_driving: f64,
    pub time_turning: f64,
    pub time_still: f64,
    /// no navigation goal active
    pub time_no_goal: f64,
    /// goal active and robot still, stops of any length (stalls are the ones over STALL_MIN)
    pub time_still_with_goal: f64,
    /// (time, known free area m^2) at AREA_TIMES that were reached, and at the end
    pub area: Vec<(f64, f64)>,
    pub final_area: f64,
    /// free floor reachable from the start in the world (m^2), for scale
    pub reference_area: f64,
    pub stalls: Vec<StallEpisode>,
    pub explorer: ExplorerStats,
    pub nav_plans: u64,
    pub nav_plan_failures: u64,
    pub nav_route_kept: u64,
    pub nav_results: BTreeMap<String, u64>,
    pub collisions: u64,
    pub bumps: u64,
    pub bumps_by_zone: BTreeMap<String, u64>,
    /// contacts behind the bumper
    pub rear_contacts: u64,
    pub markers: Vec<MarkerRecord>,
    pub all_readable_at: Option<f64>,
    pub all_within_2m_at: Option<f64>,
    /// decoy labels, same records (first_confirmed set = a false target)
    pub decoys: Vec<MarkerRecord>,
    /// every marker confirmed by the tracker (the mission would end here)
    pub all_confirmed_at: Option<f64>,
    /// decoys ever confirmed (must be 0)
    pub decoys_confirmed: u64,
    pub slam_keyframes: usize,
    pub slam_loop_closures: u64,
    pub slam_fallbacks: u64,
    /// pose estimate error at the end: (m, rad)
    pub final_pose_error: (f64, f64),
    pub max_pose_error: f64,
    pub drive: crate::drive::DriveSummary,
}

fn opt(t: Option<f64>) -> String {
    t.map_or("-".into(), |t| format!("{t:.1}"))
}

fn json_opt(t: Option<f64>) -> String {
    t.map_or("null".into(), |t| format!("{t:.3}"))
}

fn json_str(s: &str) -> String {
    let mut o = String::from("\"");
    for c in s.chars() {
        match c {
            '"' => o.push_str("\\\""),
            '\\' => o.push_str("\\\\"),
            c if (c as u32) < 0x20 => {
                let _ = write!(o, "\\u{:04x}", c as u32);
            }
            c => o.push(c),
        }
    }
    o.push('"');
    o
}

impl Metrics {
    pub fn stall_time(&self) -> f64 {
        self.stalls.iter().map(StallEpisode::duration).fold(0.0, |a, b| a + b)
    }

    /// stall time per cause
    pub fn stall_causes(&self) -> BTreeMap<String, (u64, f64)> {
        let mut m: BTreeMap<String, (u64, f64)> = BTreeMap::new();
        for s in &self.stalls {
            let e = m.entry(s.cause.clone()).or_default();
            e.0 += 1;
            e.1 += s.duration();
        }
        m
    }

    pub fn table(&self) -> String {
        let mut o = String::new();
        let w = &mut o;
        let _ = writeln!(w, "world {}  seed {}  {}  {:.0} s simulated in {:.1} s wall ({:.0}x)",
            self.world, self.seed, if self.perfect_pose { "perfect pose" } else { "team7 slam" },
            self.duration, self.wall_time, self.duration / self.wall_time.max(1e-9));
        let _ = writeln!(w, "mapped free area (m^2), reference {:.1}:", self.reference_area);
        for (t, a) in &self.area {
            let _ = writeln!(w, "  {t:>5.0} s  {a:6.1}  ({:.0} %)", 100.0 * a / self.reference_area.max(1e-9));
        }
        let _ = writeln!(w, "  final   {:6.1}", self.final_area);
        let _ = writeln!(w, "distance {:.1} m  driving {:.0} s  turning in place {:.0} s  still {:.0} s  no goal {:.0} s",
            self.distance, self.time_driving, self.time_turning, self.time_still, self.time_no_goal);
        let d = &self.drive;
        let _ = writeln!(w, "moving speed {:.3} m/s, at >= 80 % of cap {:.0} %  stop-to-turn {}  rotate mode {} x {:.0} s  cmd |dw| p95 {:.3}  |dv| p95 {:.3}  traj end jump p95 {:.1} cm, jitter p95 {:.1} cm",
            d.moving_speed, 100.0 * d.at_cap_share, d.stop_turns, d.rotate_entries, d.time_rotate_mode, d.cmd_dw_p95, d.cmd_dv_p95,
            100.0 * d.end_jump_p95, 100.0 * d.end_jitter_p95);
        let slow: Vec<String> = d.slow.iter().map(|(k, v)| format!("{k} {v:.0} s")).collect();
        let _ = writeln!(w, "moving below 80 % of cap: {}", slow.join(", "));
        let _ = writeln!(w, "still with a goal {:.0} s, of that stalls (> {STALL_MIN:.0} s) {} episodes, {:.0} s",
            self.time_still_with_goal, self.stalls.len(), self.stall_time());
        for (c, (n, t)) in self.stall_causes() {
            let _ = writeln!(w, "  {c:<28} {n:>3} x {t:6.1} s");
        }
        let e = &self.explorer;
        let _ = writeln!(w, "goals sent {} (frontier {}, view {}, turn {}, approach {})  succeeded {}  switched {}  cancelled {} (timeout {}, no progress {}, wall seen {}, marker confirmed {})  aborted {}  markers given up {}",
            e.sent, e.sent_frontier, e.sent_view, e.sent_turn, e.sent_approach, e.succeeded, e.switched,
            e.cancel_timeout + e.cancel_no_progress + e.cancel_wall_seen + e.cancel_other + e.approach_confirmed,
            e.cancel_timeout, e.cancel_no_progress, e.cancel_wall_seen, e.approach_confirmed, e.aborted, e.approach_given_up);
        let results: Vec<String> = self.nav_results.iter().map(|(k, v)| format!("{k} {v}")).collect();
        let _ = writeln!(w, "navigator plans {} (failed {}, route kept {})  results: {}",
            self.nav_plans, self.nav_plan_failures, self.nav_route_kept, results.join(", "));
        let zones: Vec<String> = self.bumps_by_zone.iter().map(|(k, v)| format!("{k} {v}")).collect();
        let _ = writeln!(w, "collisions {}  bumps {} [{}]  rear contacts {}",
            self.collisions, self.bumps, zones.join(", "), self.rear_contacts);
        let _ = writeln!(w, "markers:  id   size  readable  cov seen  within 2 m  views  5 views  max px  closest");
        for m in &self.markers {
            let _ = writeln!(w, "        {:>4} {:5.2} {:>9} {:>9} {:>11} {:>6} {:>8} {:7.1} {:8.2}",
                m.id, m.size, opt(m.first_readable), opt(m.coverage_seen), opt(m.first_within_2m), m.views, opt(m.views5_at), m.max_side_px, m.closest);
        }
        let _ = writeln!(w, "all markers readable at {} s, all within 2 m at {} s", opt(self.all_readable_at), opt(self.all_within_2m_at));
        let cm = |e: Option<f64>| e.map_or("-".into(), |e| format!("{:.1}", 100.0 * e));
        let _ = writeln!(w, "tracker:  id  kind    detections  tentative  confirmed  err@conf cm  err end cm  views  confirmed at end");
        for (kind, m) in self.markers.iter().map(|m| ("target", m)).chain(self.decoys.iter().map(|m| ("decoy", m))) {
            let _ = writeln!(w, "        {:>4}  {kind:<6} {:>11} {:>10} {:>10} {:>12} {:>11} {:>6}  {}",
                m.id, m.detections, opt(m.first_tentative), opt(m.first_confirmed), cm(m.err_confirmed), cm(m.err_end),
                m.tracker_views, if m.confirmed_end { "yes" } else { "no" });
        }
        let _ = writeln!(w, "all markers confirmed at {} s, decoys confirmed {}", opt(self.all_confirmed_at), self.decoys_confirmed);
        let _ = writeln!(w, "slam keyframes {}  loop closures {}  match fallbacks {}  pose error final {:.3} m / {:.3} rad, max {:.3} m",
            self.slam_keyframes, self.slam_loop_closures, self.slam_fallbacks,
            self.final_pose_error.0, self.final_pose_error.1, self.max_pose_error);
        o
    }

    pub fn to_json(&self) -> String {
        let mut o = String::from("{\n");
        let w = &mut o;
        let _ = writeln!(w, "  \"world\": {},", json_str(&self.world));
        let _ = writeln!(w, "  \"seed\": {},", self.seed);
        let _ = writeln!(w, "  \"perfect_pose\": {},", self.perfect_pose);
        let _ = writeln!(w, "  \"duration_s\": {:.3},", self.duration);
        let _ = writeln!(w, "  \"wall_time_s\": {:.3},", self.wall_time);
        let _ = writeln!(w, "  \"distance_m\": {:.3},", self.distance);
        let _ = writeln!(w, "  \"time_driving_s\": {:.3},", self.time_driving);
        let _ = writeln!(w, "  \"time_turning_s\": {:.3},", self.time_turning);
        let _ = writeln!(w, "  \"time_still_s\": {:.3},", self.time_still);
        let _ = writeln!(w, "  \"time_no_goal_s\": {:.3},", self.time_no_goal);
        let _ = writeln!(w, "  \"time_still_with_goal_s\": {:.3},", self.time_still_with_goal);
        let d = &self.drive;
        let _ = writeln!(w, "  \"drive\": {{\"moving_speed\": {:.4}, \"at_cap_share\": {:.4}, \"stop_turns\": {}, \"rotate_entries\": {}, \"time_rotate_mode_s\": {:.3}, \"cmd_dw_p95\": {:.4}, \"cmd_dv_p95\": {:.4}, \"traj_end_jump_p95_m\": {:.4}, \"traj_end_jitter_p95_m\": {:.4}, \"slow_s\": {{{}}}}},",
            d.moving_speed, d.at_cap_share, d.stop_turns, d.rotate_entries, d.time_rotate_mode, d.cmd_dw_p95, d.cmd_dv_p95, d.end_jump_p95, d.end_jitter_p95,
            d.slow.iter().map(|(k, v)| format!("{}: {v:.3}", json_str(k))).collect::<Vec<_>>().join(", "));
        let area: Vec<String> = self.area.iter().map(|(t, a)| format!("{{\"t\": {t:.1}, \"m2\": {a:.3}}}")).collect();
        let _ = writeln!(w, "  \"mapped_area\": [{}],", area.join(", "));
        let _ = writeln!(w, "  \"final_area_m2\": {:.3},", self.final_area);
        let _ = writeln!(w, "  \"reference_area_m2\": {:.3},", self.reference_area);
        let stalls: Vec<String> = self
            .stalls
            .iter()
            .map(|s| {
                let g = s.goal.map_or("null".into(), |g| format!("[{:.3}, {:.3}]", g.0, g.1));
                format!("{{\"start\": {:.2}, \"end\": {:.2}, \"cause\": {}, \"goal\": {g}}}", s.start, s.end, json_str(&s.cause))
            })
            .collect();
        let _ = writeln!(w, "  \"stall_time_s\": {:.3},", self.stall_time());
        let _ = writeln!(w, "  \"stalls\": [{}],", stalls.join(", "));
        let e = &self.explorer;
        let _ = writeln!(w, "  \"goals\": {{\"sent\": {}, \"frontier\": {}, \"view\": {}, \"turn\": {}, \"approach\": {}, \"succeeded\": {}, \"switched\": {}, \"canceled\": {}, \"aborted\": {}, \"cancel_timeout\": {}, \"cancel_no_progress\": {}, \"cancel_wall_seen\": {}, \"cancel_marker_confirmed\": {}, \"cancel_other\": {}, \"markers_given_up\": {}}},",
            e.sent, e.sent_frontier, e.sent_view, e.sent_turn, e.sent_approach, e.succeeded, e.switched, e.canceled, e.aborted,
            e.cancel_timeout, e.cancel_no_progress, e.cancel_wall_seen, e.approach_confirmed, e.cancel_other, e.approach_given_up);
        let results: Vec<String> = self.nav_results.iter().map(|(k, v)| format!("{}: {v}", json_str(k))).collect();
        let _ = writeln!(w, "  \"navigator\": {{\"plans\": {}, \"plan_failures\": {}, \"route_kept\": {}, \"results\": {{{}}}}},",
            self.nav_plans, self.nav_plan_failures, self.nav_route_kept, results.join(", "));
        let zones: Vec<String> = self.bumps_by_zone.iter().map(|(k, v)| format!("{}: {v}", json_str(k))).collect();
        let _ = writeln!(w, "  \"collisions\": {},", self.collisions);
        let _ = writeln!(w, "  \"bumps\": {},", self.bumps);
        let _ = writeln!(w, "  \"bumps_by_zone\": {{{}}},", zones.join(", "));
        let _ = writeln!(w, "  \"rear_contacts\": {},", self.rear_contacts);
        let record = |m: &MarkerRecord| {
            let closest = if m.closest.is_finite() { format!("{:.3}", m.closest) } else { "null".into() };
            format!("{{\"id\": {}, \"size\": {:.3}, \"first_readable\": {}, \"coverage_seen\": {}, \"first_within_2m\": {}, \"readable_frames\": {}, \"views\": {}, \"views5_at\": {}, \"max_side_px\": {:.2}, \"closest_m\": {closest}, \"detections\": {}, \"first_tentative\": {}, \"first_confirmed\": {}, \"err_confirmed_m\": {}, \"err_end_m\": {}, \"confirmed_end\": {}, \"tracker_views\": {}}}",
                m.id, m.size, json_opt(m.first_readable), json_opt(m.coverage_seen), json_opt(m.first_within_2m), m.readable_frames, m.views,
                json_opt(m.views5_at), m.max_side_px, m.detections, json_opt(m.first_tentative), json_opt(m.first_confirmed),
                json_opt(m.err_confirmed), json_opt(m.err_end), m.confirmed_end, m.tracker_views)
        };
        let markers: Vec<String> = self.markers.iter().map(record).collect();
        let decoys: Vec<String> = self.decoys.iter().map(record).collect();
        let _ = writeln!(w, "  \"markers\": [{}],", markers.join(", "));
        let _ = writeln!(w, "  \"decoys\": [{}],", decoys.join(", "));
        let _ = writeln!(w, "  \"all_readable_at\": {},", json_opt(self.all_readable_at));
        let _ = writeln!(w, "  \"all_within_2m_at\": {},", json_opt(self.all_within_2m_at));
        let _ = writeln!(w, "  \"all_confirmed_at\": {},", json_opt(self.all_confirmed_at));
        let _ = writeln!(w, "  \"decoys_confirmed\": {},", self.decoys_confirmed);
        let _ = writeln!(w, "  \"slam\": {{\"keyframes\": {}, \"loop_closures\": {}, \"match_fallbacks\": {}}},",
            self.slam_keyframes, self.slam_loop_closures, self.slam_fallbacks);
        let _ = writeln!(w, "  \"final_pose_error\": {{\"m\": {:.4}, \"rad\": {:.4}}},", self.final_pose_error.0, self.final_pose_error.1);
        let _ = writeln!(w, "  \"max_pose_error_m\": {:.4}", self.max_pose_error);
        o.push_str("}\n");
        o
    }
}

/// free area (m^2) of an occupancy grid: cells in 0..free_below
pub fn free_area(g: &tb4_core::Grid, free_below: i8) -> f64 {
    let n = g.data.iter().filter(|&&v| (0..free_below).contains(&v)).count();
    n as f64 * g.meta.resolution * g.meta.resolution
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn short_stops_are_not_stalls() {
        let mut s = StallTracker::default();
        let dt = 0.01;
        let mut t = 0.0;
        // 1.5 s still: no stall
        for _ in 0..150 {
            t += dt;
            s.update(t, dt, true, "a", None);
        }
        t += dt;
        s.update(t, dt, false, "", None);
        assert!(s.episodes.is_empty());
        // 3 s: 2 s of cause b, 1 s of cause a
        for k in 0..300 {
            t += dt;
            s.update(t, dt, true, if k < 100 { "a" } else { "b" }, Some((1.0, 2.0)));
        }
        let end = t;
        t += dt;
        s.update(t, dt, false, "", None);
        assert_eq!(s.episodes.len(), 1);
        let e = &s.episodes[0];
        assert_eq!(e.cause, "b");
        assert_eq!(e.goal, Some((1.0, 2.0)));
        assert!((e.duration() - 3.0).abs() < 1e-6, "{}", e.duration());
        assert!((e.end - end).abs() < 1e-9);
        // still open at the end of the run
        for _ in 0..250 {
            t += dt;
            s.update(t, dt, true, "c", None);
        }
        s.close(t);
        assert_eq!(s.episodes.len(), 2);
        assert!((s.total() - 5.5).abs() < 1e-6);
    }

    #[test]
    fn motion_classes() {
        assert_eq!(classify(0.05, 0.0), Motion::Driving);
        assert_eq!(classify(0.01, 0.3), Motion::Turning);
        assert_eq!(classify(0.01, 0.05), Motion::Still);
    }

    #[test]
    fn json_and_table_hold_the_numbers() {
        let m = Metrics {
            world: "w\"x".into(),
            duration: 10.0,
            area: vec![(60.0, 12.5)],
            reference_area: 50.0,
            stalls: vec![StallEpisode { start: 1.0, end: 4.0, cause: "no path".into(), goal: None }],
            markers: vec![MarkerRecord { id: 7, first_readable: Some(3.25), ..Default::default() }],
            ..Default::default()
        };
        let j = m.to_json();
        assert!(j.contains("\"world\": \"w\\\"x\""));
        assert!(j.contains("{\"t\": 60.0, \"m2\": 12.500}"));
        assert!(j.contains("\"stall_time_s\": 3.000"));
        assert!(j.contains("\"first_readable\": 3.250"));
        assert!(j.contains("\"first_within_2m\": null"));
        let t = m.table();
        assert!(t.contains("no path"));
        assert!(t.contains("25 %"));
        assert_eq!(m.stall_causes()["no path"], (1, 3.0));
    }

    #[test]
    fn free_area_counts_free_cells() {
        let meta = tb4_core::GridMeta { width: 10, height: 10, resolution: 0.1, origin_x: 0.0, origin_y: 0.0 };
        let mut g = tb4_core::Grid::filled(meta, -1);
        for i in 0..20 {
            g.data[i] = 0;
        }
        g.data[20] = 100;
        assert!((free_area(&g, 50) - 0.2).abs() < 1e-9);
    }
}
