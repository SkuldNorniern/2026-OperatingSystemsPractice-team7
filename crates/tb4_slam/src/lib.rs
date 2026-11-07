mod graph;
mod map;
mod matcher;
pub mod se2;
pub mod sim;

use std::collections::HashSet;
use std::time::Instant;

use tb4_core::{Grid, GridMeta, Pose2};

use crate::graph::Edge;
use crate::map::LogOddsMap;
use crate::matcher::{MatchField, SearchWindow, match_scan};
use crate::se2::{apply, between, compose, inverse, wrap};

#[derive(Debug, Clone)]
pub struct SlamParams {
    pub resolution: f64,
    pub min_travel_dist: f64,
    pub min_travel_yaw: f64,
    pub min_range: f64,
    /// hits beyond this become free-only rays truncated here
    pub max_range: f64,
    pub search_lin: f64,
    pub search_ang: f64,
    /// None: derived from the scan's max range and the map resolution
    pub search_ang_res: Option<f64>,
    /// score *= exp(-(w_lin * d^2 + w_ang * dyaw^2)) relative to the odometry prediction
    pub prior_weight_lin: f64,
    pub prior_weight_ang: f64,
    pub likelihood_sigma: f64,
    pub min_match_score: f64,
    pub log_odds_hit: f32,
    pub log_odds_miss: f32,
    pub log_odds_clamp: f32,
    pub occupied_threshold: f32,
    pub free_threshold: f32,
    pub map_margin: i32,
    pub keyframe_dist: f64,
    pub keyframe_yaw: f64,
    pub seq_sigma_lin: f64,
    pub seq_sigma_ang: f64,
    pub loop_search_radius: f64,
    /// keyframes between the current one and any loop candidate
    pub loop_min_gap: usize,
    pub loop_search_lin: f64,
    pub loop_search_ang: f64,
    pub loop_min_score: f64,
    /// two consecutive loop matches must imply the same correction within these bounds
    pub loop_consistency_lin: f64,
    pub loop_consistency_ang: f64,
    pub loop_cooldown: usize,
    pub loop_sigma_lin: f64,
    pub loop_sigma_ang: f64,
    pub loop_max_chi2: f64,
    pub opt_iterations: usize,
}

impl Default for SlamParams {
    fn default() -> Self {
        Self {
            resolution: 0.05,
            min_travel_dist: 0.1,
            min_travel_yaw: 0.1,
            min_range: 0.15,
            max_range: 8.0,
            search_lin: 0.3,
            search_ang: 0.35,
            search_ang_res: None,
            prior_weight_lin: 2.0,
            prior_weight_ang: 1.0,
            likelihood_sigma: 0.08,
            min_match_score: 0.35,
            log_odds_hit: 0.85,
            log_odds_miss: -0.25,
            log_odds_clamp: 3.5,
            occupied_threshold: 0.6,
            free_threshold: -0.2,
            map_margin: 10,
            keyframe_dist: 0.3,
            keyframe_yaw: 0.35,
            seq_sigma_lin: 0.02,
            seq_sigma_ang: 0.01,
            loop_search_radius: 2.5,
            loop_min_gap: 20,
            loop_search_lin: 0.8,
            loop_search_ang: 0.4,
            loop_min_score: 0.55,
            loop_consistency_lin: 0.15,
            loop_consistency_ang: 0.05,
            loop_cooldown: 5,
            loop_sigma_lin: 0.05,
            loop_sigma_ang: 0.02,
            loop_max_chi2: 30.0,
            opt_iterations: 15,
        }
    }
}

#[derive(Debug, Clone, Default)]
pub struct SlamStats {
    pub scans_processed: u64,
    pub scans_skipped: u64,
    pub match_fallbacks: u64,
    pub keyframes: usize,
    pub loop_closures: u64,
    pub loop_rejections: u64,
    pub last_score: f64,
    pub last_match_ms: f64,
    pub last_update_ms: f64,
    pub mean_update_ms: f64,
    pub max_update_ms: f64,
    pub last_optimisation_ms: f64,
    pub last_rebuild_ms: f64,
}

#[derive(Debug, Clone)]
struct StoredScan {
    rel: Pose2,
    points: Vec<(f32, f32)>,
}

#[derive(Debug, Clone)]
struct Keyframe {
    pose: Pose2,
    scans: Vec<StoredScan>,
}

#[derive(Debug, Clone, Copy)]
struct LoopCand {
    kf: usize,
    anchor: usize,
    z: Pose2,
    est: Pose2,
    matched: Pose2,
}

pub struct Slam {
    p: SlamParams,
    map: LogOddsMap,
    kfs: Vec<Keyframe>,
    edges: Vec<Edge>,
    pose: Pose2,
    map_to_odom: Pose2,
    last_odom: Option<Pose2>,
    pending: Option<LoopCand>,
    last_loop_kf: Option<usize>,
    stats: SlamStats,
    update_ms_sum: f64,
}

fn ms(t: Instant) -> f64 {
    t.elapsed().as_secs_f64() * 1e3
}

fn downsample(pts: impl Iterator<Item = (f64, f64)>, res: f64, max_r: f64) -> Vec<(f64, f64)> {
    let mut seen = HashSet::new();
    pts.filter(|p| p.0.hypot(p.1) <= max_r)
        .filter(|p| seen.insert(((p.0 / res).floor() as i32, (p.1 / res).floor() as i32)))
        .collect()
}

impl Slam {
    pub fn new(params: SlamParams) -> Self {
        let map = LogOddsMap::new(
            params.resolution,
            params.log_odds_hit,
            params.log_odds_miss,
            params.log_odds_clamp,
        );
        Self {
            p: params,
            map,
            kfs: Vec::new(),
            edges: Vec::new(),
            pose: Pose2::default(),
            map_to_odom: Pose2::default(),
            last_odom: None,
            pending: None,
            last_loop_kf: None,
            stats: SlamStats::default(),
            update_ms_sum: 0.0,
        }
    }

    pub fn params(&self) -> &SlamParams {
        &self.p
    }

    pub fn pose(&self) -> Pose2 {
        self.pose
    }

    pub fn map_to_odom(&self) -> Pose2 {
        self.map_to_odom
    }

    pub fn stats(&self) -> &SlamStats {
        &self.stats
    }

    pub fn keyframes(&self) -> Vec<Pose2> {
        self.kfs.iter().map(|k| k.pose).collect()
    }

    pub fn loop_edges(&self) -> Vec<(usize, usize)> {
        self.edges.iter().filter(|e| e.is_loop).map(|e| (e.i, e.j)).collect()
    }

    pub fn occupancy(&self) -> Grid {
        self.map
            .to_grid(self.p.occupied_threshold, self.p.free_threshold, self.p.map_margin)
    }

    pub fn update(&mut self, odom: Pose2, points: &[(f64, f64)]) -> Option<Pose2> {
        let t0 = Instant::now();
        let pose = match self.last_odom {
            None => odom,
            Some(last) => {
                let d = between(last, odom);
                if d.x.hypot(d.y) < self.p.min_travel_dist && d.yaw.abs() < self.p.min_travel_yaw {
                    self.stats.scans_skipped += 1;
                    return None;
                }
                let pred = compose(self.pose, d);
                self.match_front(pred, points)
            }
        };
        let pts: Vec<(f32, f32)> = points
            .iter()
            .filter(|p| p.0.is_finite() && p.1.is_finite() && p.0.hypot(p.1) >= self.p.min_range)
            .map(|&(x, y)| (x as f32, y as f32))
            .collect();
        self.pose = pose;
        self.last_odom = Some(odom);
        self.map.insert_scan(pose, &pts, self.p.min_range, self.p.max_range);
        self.add_to_keyframes(pose, pts);
        self.map_to_odom = compose(self.pose, inverse(odom));

        let s = &mut self.stats;
        s.scans_processed += 1;
        s.keyframes = self.kfs.len();
        s.last_update_ms = ms(t0);
        s.max_update_ms = s.max_update_ms.max(s.last_update_ms);
        self.update_ms_sum += s.last_update_ms;
        s.mean_update_ms = self.update_ms_sum / s.scans_processed as f64;
        Some(self.pose)
    }

    fn match_front(&mut self, pred: Pose2, points: &[(f64, f64)]) -> Pose2 {
        let t = Instant::now();
        let p = &self.p;
        let q = downsample(
            points
                .iter()
                .copied()
                .filter(|q| q.0.is_finite() && q.1.is_finite() && q.0.hypot(q.1) >= p.min_range),
            p.resolution,
            p.max_range,
        );
        let result = self.map.bbox().and_then(|bb| {
            let reach = p.max_range + p.search_lin;
            let pad = (p.search_lin / p.resolution).ceil() as i32 + 2;
            let c = |v: f64| self.map.cell(v);
            let b = [
                c(pred.x - reach).max(bb[0] - pad),
                c(pred.y - reach).max(bb[1] - pad),
                c(pred.x + reach).min(bb[2] + pad),
                c(pred.y + reach).min(bb[3] + pad),
            ];
            if b[0] > b[2] || b[1] > b[3] {
                return None;
            }
            let field = MatchField::new(
                &self.map.window_grid(b, p.occupied_threshold),
                p.likelihood_sigma,
                p.search_lin,
            );
            let win = SearchWindow {
                lin: p.search_lin,
                ang: p.search_ang,
                ang_res: p.search_ang_res,
                w_lin: p.prior_weight_lin,
                w_ang: p.prior_weight_ang,
            };
            match_scan(&field, &q, pred, &win, p.min_match_score)
        });
        self.stats.last_match_ms = ms(t);
        match result {
            Some(m) => {
                self.stats.last_score = m.score;
                m.pose
            }
            None => {
                self.stats.last_score = 0.0;
                self.stats.match_fallbacks += 1;
                pred
            }
        }
    }

    fn add_to_keyframes(&mut self, pose: Pose2, points: Vec<(f32, f32)>) {
        if let Some(k) = self.kfs.last_mut() {
            let d = between(k.pose, pose);
            if d.x.hypot(d.y) < self.p.keyframe_dist && d.yaw.abs() < self.p.keyframe_yaw {
                k.scans.push(StoredScan { rel: d, points });
                return;
            }
            let n = self.kfs.len();
            let (sl, sa) = (self.p.seq_sigma_lin, self.p.seq_sigma_ang);
            self.edges.push(Edge {
                i: n - 1,
                j: n,
                z: d,
                info: [1.0 / (sl * sl), 1.0 / (sl * sl), 1.0 / (sa * sa)],
                is_loop: false,
            });
        }
        self.kfs.push(Keyframe {
            pose,
            scans: vec![StoredScan {
                rel: Pose2::default(),
                points,
            }],
        });
        self.try_loop_closure();
    }

    fn try_loop_closure(&mut self) {
        let p = &self.p;
        let c = self.kfs.len() - 1;
        if c < p.loop_min_gap || self.last_loop_kf.is_some_and(|l| c < l + p.loop_cooldown) {
            return;
        }
        let cur = self.kfs[c].pose;
        let old = &self.kfs[..=c - p.loop_min_gap];
        let dist = |k: &Keyframe| k.pose.distance_to(cur.x, cur.y);
        let Some(anchor) = (0..old.len())
            .filter(|&j| dist(&old[j]) <= p.loop_search_radius)
            .min_by(|&a, &b| dist(&old[a]).total_cmp(&dist(&old[b])))
        else {
            self.pending = None;
            return;
        };

        let res = p.resolution;
        let reach = p.max_range + p.loop_search_lin;
        let pad = (p.loop_search_lin / res).ceil() as i32 + 2;
        let cell = |v: f64| (v / res).floor() as i32;
        let lim = [
            cell(cur.x - reach),
            cell(cur.y - reach),
            cell(cur.x + reach),
            cell(cur.y + reach),
        ];
        let mut hits = Vec::new();
        let mut bb = [i32::MAX, i32::MAX, i32::MIN, i32::MIN];
        for k in old.iter().filter(|k| dist(k) <= 1.5 * p.loop_search_radius) {
            for s in &k.scans {
                let t = compose(k.pose, s.rel);
                for &(x, y) in &s.points {
                    let (x, y) = (x as f64, y as f64);
                    if x.hypot(y) > p.max_range {
                        continue;
                    }
                    let (wx, wy) = apply(t, (x, y));
                    let (cx, cy) = (cell(wx), cell(wy));
                    if cx < lim[0] || cy < lim[1] || cx > lim[2] || cy > lim[3] {
                        continue;
                    }
                    bb = [bb[0].min(cx), bb[1].min(cy), bb[2].max(cx), bb[3].max(cy)];
                    hits.push((cx, cy));
                }
            }
        }
        if hits.is_empty() {
            return;
        }
        let b = [bb[0] - pad, bb[1] - pad, bb[2] + pad, bb[3] + pad];
        let meta = GridMeta {
            width: (b[2] - b[0] + 1) as u32,
            height: (b[3] - b[1] + 1) as u32,
            resolution: res,
            origin_x: b[0] as f64 * res,
            origin_y: b[1] as f64 * res,
        };
        let mut grid = Grid::filled(meta, 0);
        for (cx, cy) in hits {
            let i = ((cy - b[1]) * meta.width as i32 + (cx - b[0])) as usize;
            grid.data[i] = 100;
        }
        let field = MatchField::new(&grid, p.likelihood_sigma, p.loop_search_lin);

        let mut q = Vec::new();
        for k in &self.kfs[c.saturating_sub(2)..=c] {
            let rel = between(cur, k.pose);
            for s in &k.scans {
                let t = compose(rel, s.rel);
                q.extend(s.points.iter().map(|&(x, y)| apply(t, (x as f64, y as f64))));
            }
        }
        let q = downsample(q.into_iter(), res, p.max_range);
        let win = SearchWindow {
            lin: p.loop_search_lin,
            ang: p.loop_search_ang,
            ang_res: p.search_ang_res,
            w_lin: 0.0,
            w_ang: 0.0,
        };
        let Some(m) = match_scan(&field, &q, cur, &win, p.loop_min_score) else {
            self.pending = None;
            return;
        };
        let cand = LoopCand {
            kf: c,
            anchor,
            z: between(self.kfs[anchor].pose, m.pose),
            est: cur,
            matched: m.pose,
        };
        let (cl, ca) = (p.loop_consistency_lin, p.loop_consistency_ang);
        let confirmed = self.pending.take().filter(|prev| {
            let pred = compose(compose(prev.matched, inverse(prev.est)), cur);
            c - prev.kf <= 3 && pred.distance_to(m.pose.x, m.pose.y) <= cl && wrap(pred.yaw - m.pose.yaw).abs() <= ca
        });
        match confirmed {
            Some(prev) => self.close_loop(&[prev, cand]),
            None => self.pending = Some(cand),
        }
    }

    fn close_loop(&mut self, cands: &[LoopCand]) {
        let t = Instant::now();
        let n_before = self.edges.len();
        let (ll, la) = (self.p.loop_sigma_lin, self.p.loop_sigma_ang);
        for cd in cands {
            self.edges.push(Edge {
                i: cd.anchor,
                j: cd.kf,
                z: cd.z,
                info: [1.0 / (ll * ll), 1.0 / (ll * ll), 1.0 / (la * la)],
                is_loop: true,
            });
        }
        let mut poses: Vec<Pose2> = self.kfs.iter().map(|k| k.pose).collect();
        graph::optimize(&mut poses, &self.edges, self.p.opt_iterations);
        self.stats.last_optimisation_ms = ms(t);
        if self.edges[n_before..]
            .iter()
            .any(|e| graph::edge_chi2(&poses, e) > self.p.loop_max_chi2)
        {
            self.edges.truncate(n_before);
            self.stats.loop_rejections += 1;
            return;
        }
        let (mut moved, mut turned) = (0.0f64, 0.0f64);
        for (k, &np) in self.kfs.iter_mut().zip(&poses) {
            moved = moved.max(k.pose.distance_to(np.x, np.y));
            turned = turned.max(wrap(k.pose.yaw - np.yaw).abs());
            k.pose = np;
        }
        let last = self.kfs.last().expect("keyframes exist");
        self.pose = compose(last.pose, last.scans.last().expect("keyframe has a scan").rel);
        if moved > 0.5 * self.p.resolution || turned > 0.005 {
            let t = Instant::now();
            self.rebuild_map();
            self.stats.last_rebuild_ms = ms(t);
        }
        self.stats.loop_closures += 1;
        self.last_loop_kf = Some(self.kfs.len() - 1);
    }

    fn rebuild_map(&mut self) {
        let p = &self.p;
        let jobs: Vec<(Pose2, &[(f32, f32)])> = self
            .kfs
            .iter()
            .flat_map(|k| k.scans.iter().map(|s| (compose(k.pose, s.rel), s.points.as_slice())))
            .collect();
        let threads = std::thread::available_parallelism().map_or(1, |n| n.get()).min(8);
        let chunk = jobs.len().div_ceil(threads).max(64);
        let empty = || LogOddsMap::new(p.resolution, p.log_odds_hit, p.log_odds_miss, p.log_odds_clamp);
        let insert = |m: &mut LogOddsMap, part: &[(Pose2, &[(f32, f32)])]| {
            for (pose, pts) in part {
                m.insert_scan(*pose, pts, p.min_range, p.max_range);
            }
        };
        let parts: Vec<LogOddsMap> = std::thread::scope(|sc| {
            let handles: Vec<_> = jobs
                .chunks(chunk)
                .map(|part| {
                    sc.spawn(move || {
                        let mut m = empty();
                        insert(&mut m, part);
                        m
                    })
                })
                .collect();
            handles.into_iter().map(|h| h.join().expect("rebuild worker")).collect()
        });
        self.map.clear();
        for part in &parts {
            self.map.merge_add(part);
        }
    }
}
