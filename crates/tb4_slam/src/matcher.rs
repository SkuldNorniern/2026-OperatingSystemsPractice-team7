use tb4_core::{DistanceField, Grid, Pose2};

use crate::se2::wrap;

const DIST_CAP: f32 = 1.0;
const REFINE_TRIM: f64 = 0.2;

/// Likelihood field over a window plus max-pooled levels for branch and bound.
pub struct MatchField {
    ox: f64,
    oy: f64,
    res: f64,
    w: i32,
    h: i32,
    dist: Vec<f32>,
    levels: Vec<Vec<f32>>,
    inv2s2: f64,
}

#[derive(Debug, Clone, Copy)]
pub struct SearchWindow {
    pub lin: f64,
    pub ang: f64,
    pub ang_res: Option<f64>,
    pub w_lin: f64,
    pub w_ang: f64,
}

#[derive(Debug, Clone, Copy)]
pub struct MatchResult {
    pub pose: Pose2,
    pub score: f64,
}

impl MatchField {
    /// `grid`: 100 = occupied. Builds enough levels for a +-`lin` metre search.
    pub fn new(grid: &Grid, sigma: f64, lin: f64) -> Self {
        let m = grid.meta;
        let df = DistanceField::from_grid(grid, 50, false);
        let dist: Vec<f32> = df.dist.iter().map(|d| d.min(DIST_CAP)).collect();
        let inv2s2 = 1.0 / (2.0 * sigma * sigma);
        let l0: Vec<f32> = dist
            .iter()
            .map(|&d| (-(d as f64).powi(2) * inv2s2).exp() as f32)
            .collect();
        let n = 2 * (lin / m.resolution).ceil() as usize + 1;
        let mut depth = 1;
        while (1usize << (depth - 1)) < n {
            depth += 1;
        }
        let (w, h) = (m.width as usize, m.height as usize);
        let mut levels = vec![l0];
        for k in 1..depth {
            let s = 1usize << (k - 1);
            let prev = &levels[k - 1];
            let mut next = vec![0.0f32; w * h];
            for y in 0..h {
                for x in 0..w {
                    let mut v = prev[y * w + x];
                    if x + s < w {
                        v = v.max(prev[y * w + x + s]);
                    }
                    if y + s < h {
                        v = v.max(prev[(y + s) * w + x]);
                        if x + s < w {
                            v = v.max(prev[(y + s) * w + x + s]);
                        }
                    }
                    next[y * w + x] = v;
                }
            }
            levels.push(next);
        }
        Self {
            ox: m.origin_x,
            oy: m.origin_y,
            res: m.resolution,
            w: w as i32,
            h: h as i32,
            dist,
            levels,
            inv2s2,
        }
    }

    /// Bilinear distance (m) and its gradient at a world point.
    fn interp(&self, wx: f64, wy: f64) -> Option<(f64, f64, f64)> {
        let u = (wx - self.ox) / self.res - 0.5;
        let v = (wy - self.oy) / self.res - 0.5;
        let (fi, fj) = (u.floor(), v.floor());
        let (i, j) = (fi as i32, fj as i32);
        if i < 0 || j < 0 || i + 1 >= self.w || j + 1 >= self.h {
            return None;
        }
        let (fx, fy) = (u - fi, v - fj);
        let k = (j * self.w + i) as usize;
        let wu = self.w as usize;
        let d00 = self.dist[k] as f64;
        let d10 = self.dist[k + 1] as f64;
        let d01 = self.dist[k + wu] as f64;
        let d11 = self.dist[k + wu + 1] as f64;
        let d = d00 * (1.0 - fx) * (1.0 - fy) + d10 * fx * (1.0 - fy) + d01 * (1.0 - fx) * fy + d11 * fx * fy;
        let gx = ((d10 - d00) * (1.0 - fy) + (d11 - d01) * fy) / self.res;
        let gy = ((d01 - d00) * (1.0 - fx) + (d11 - d10) * fx) / self.res;
        Some((d, gx, gy))
    }

    /// Mean likelihood of the points at `pose`, in [0, 1].
    pub fn score(&self, pts: &[(f64, f64)], pose: Pose2) -> f64 {
        if pts.is_empty() {
            return 0.0;
        }
        let (s, c) = pose.yaw.sin_cos();
        let sum: f64 = pts
            .iter()
            .filter_map(|&(px, py)| self.interp(pose.x + c * px - s * py, pose.y + s * px + c * py))
            .map(|(d, _, _)| (-d * d * self.inv2s2).exp())
            .sum();
        sum / pts.len() as f64
    }

    /// Gauss-Newton on the distance field with trimmed residuals.
    pub fn refine(&self, pts: &[(f64, f64)], pose: Pose2) -> Pose2 {
        let mut p = pose;
        for _ in 0..10 {
            let (s, c) = p.yaw.sin_cos();
            let mut h = [[0.0f64; 3]; 3];
            let mut g = [0.0f64; 3];
            let mut n = 0;
            for &(px, py) in pts {
                let Some((d, gx, gy)) = self.interp(p.x + c * px - s * py, p.y + s * px + c * py) else {
                    continue;
                };
                if d > REFINE_TRIM {
                    continue;
                }
                let j = [gx, gy, gx * (-s * px - c * py) + gy * (c * px - s * py)];
                for a in 0..3 {
                    g[a] += j[a] * d;
                    for b in 0..3 {
                        h[a][b] += j[a] * j[b];
                    }
                }
                n += 1;
            }
            if n < 10 {
                break;
            }
            for (a, row) in h.iter_mut().enumerate() {
                row[a] += 1e-3 * row[a] + 1e-9;
            }
            let Some(dx) = solve3(h, [-g[0], -g[1], -g[2]]) else {
                break;
            };
            p = Pose2::new(p.x + dx[0], p.y + dx[1], wrap(p.yaw + dx[2]));
            if dx[0].abs() + dx[1].abs() < 1e-4 && dx[2].abs() < 1e-4 {
                break;
            }
        }
        p
    }
}

pub(crate) fn solve3(a: [[f64; 3]; 3], b: [f64; 3]) -> Option<[f64; 3]> {
    let det = |m: &[[f64; 3]; 3]| {
        m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
            + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])
    };
    let d = det(&a);
    if d.abs() < 1e-18 || !d.is_finite() {
        return None;
    }
    let mut x = [0.0; 3];
    for (k, xk) in x.iter_mut().enumerate() {
        let mut m = a;
        for r in 0..3 {
            m[r][k] = b[r];
        }
        *xk = det(&m) / d;
    }
    Some(x)
}

#[derive(Clone, Copy)]
struct Cand {
    rot: usize,
    ox: i32,
    oy: i32,
    score: f32,
}

struct Search<'a> {
    f: &'a MatchField,
    rots: Vec<Vec<(i32, i32)>>,
    n_lin: i32,
    n_rot: i32,
    dth: f64,
    win: SearchWindow,
}

impl Search<'_> {
    fn cand(&self, rot: usize, ox: i32, oy: i32, level: usize) -> Cand {
        let f = self.f;
        let lvl = &f.levels[level];
        let cells = &self.rots[rot];
        let mut sum = 0.0f32;
        for &(cx, cy) in cells {
            let (x, y) = (cx + ox, cy + oy);
            if x >= 0 && y >= 0 && x < f.w && y < f.h {
                sum += lvl[(y * f.w + x) as usize];
            }
        }
        // penalty uses the block's closest offset to the prior so the bound stays admissible
        let span = (1i32 << level) - 1;
        let nearest = |o: i32| {
            let hi = (o + span).min(self.n_lin);
            if o > 0 {
                o
            } else if hi < 0 {
                -hi
            } else {
                0
            }
        };
        let (dx, dy) = (nearest(ox) as f64 * f.res, nearest(oy) as f64 * f.res);
        let th = (rot as i32 - self.n_rot) as f64 * self.dth;
        let pen = (-(self.win.w_lin * (dx * dx + dy * dy) + self.win.w_ang * th * th)).exp();
        Cand {
            rot,
            ox,
            oy,
            score: sum / cells.len() as f32 * pen as f32,
        }
    }

    fn search(&self, mut cands: Vec<Cand>, level: usize, best: &mut Cand) {
        cands.sort_unstable_by(|a, b| b.score.total_cmp(&a.score));
        for c in cands {
            if c.score <= best.score {
                break;
            }
            if level == 0 {
                *best = c;
                break;
            }
            let s = 1i32 << (level - 1);
            let kids: Vec<Cand> = [(0, 0), (s, 0), (0, s), (s, s)]
                .into_iter()
                .filter(|&(dx, dy)| c.ox + dx <= self.n_lin && c.oy + dy <= self.n_lin)
                .map(|(dx, dy)| self.cand(c.rot, c.ox + dx, c.oy + dy, level - 1))
                .collect();
            self.search(kids, level - 1, best);
        }
    }
}

/// Branch-and-bound correlative search around `prior`, then sub-cell refinement.
pub fn match_scan(
    f: &MatchField,
    pts: &[(f64, f64)],
    prior: Pose2,
    win: &SearchWindow,
    min_score: f64,
) -> Option<MatchResult> {
    if pts.len() < 10 {
        return None;
    }
    let res = f.res;
    let n_lin = (win.lin / res).ceil() as i32;
    let r_max = pts.iter().map(|p| p.0.hypot(p.1)).fold(0.0, f64::max).max(4.0 * res);
    let dth = win
        .ang_res
        .unwrap_or_else(|| (1.0 - res * res / (2.0 * r_max * r_max)).acos());
    let n_rot = (win.ang / dth).ceil() as i32;
    let rots: Vec<Vec<(i32, i32)>> = (-n_rot..=n_rot)
        .map(|k| {
            let (s, c) = (prior.yaw + k as f64 * dth).sin_cos();
            pts.iter()
                .map(|&(px, py)| {
                    let wx = prior.x + c * px - s * py;
                    let wy = prior.y + s * px + c * py;
                    (((wx - f.ox) / res).floor() as i32, ((wy - f.oy) / res).floor() as i32)
                })
                .collect()
        })
        .collect();
    let mut top = 0;
    while (1i32 << top) < 2 * n_lin + 1 && top + 1 < f.levels.len() {
        top += 1;
    }
    let search = Search {
        f,
        rots,
        n_lin,
        n_rot,
        dth,
        win: *win,
    };
    let step = 1usize << top;
    let mut cands = Vec::new();
    for r in 0..search.rots.len() {
        for ox in (-n_lin..=n_lin).step_by(step) {
            for oy in (-n_lin..=n_lin).step_by(step) {
                cands.push(search.cand(r, ox, oy, top));
            }
        }
    }
    let mut best = Cand {
        rot: usize::MAX,
        ox: 0,
        oy: 0,
        score: min_score as f32,
    };
    search.search(cands, top, &mut best);
    if best.rot == usize::MAX {
        return None;
    }
    let k = best.rot as i32 - n_rot;
    let disc = Pose2::new(
        prior.x + best.ox as f64 * res,
        prior.y + best.oy as f64 * res,
        wrap(prior.yaw + k as f64 * dth),
    );
    let refined = f.refine(pts, disc);
    let (sd, sr) = (f.score(pts, disc), f.score(pts, refined));
    let close =
        refined.distance_to(disc.x, disc.y) < 2.0 * res && wrap(refined.yaw - disc.yaw).abs() < 2.0 * dth + 0.01;
    let (pose, score) = if close && sr >= sd { (refined, sr) } else { (disc, sd) };
    (score >= min_score).then_some(MatchResult { pose, score })
}

#[cfg(test)]
mod tests {
    use super::*;
    use tb4_core::GridMeta;

    #[test]
    fn recovers_offset_in_box() {
        let meta = GridMeta {
            width: 120,
            height: 100,
            resolution: 0.05,
            origin_x: -3.0,
            origin_y: -2.5,
        };
        let mut g = Grid::filled(meta, 0);
        let mut pts = Vec::new();
        let truth = Pose2::new(0.1, -0.05, 0.08);
        for i in 0..120 {
            let x = -2.5 + i as f64 * 0.04;
            for (wx, wy) in [(x, 2.0), (x, -2.0)] {
                pts.push((wx, wy));
            }
        }
        for i in 0..100 {
            let y = -2.0 + i as f64 * 0.04;
            pts.push((2.3, y));
            pts.push((-2.5, y * 0.5));
        }
        for &(wx, wy) in &pts {
            if let Some(c) = g.world_to_cell(wx, wy) {
                g.set(c, 100);
            }
        }
        let inv = crate::se2::inverse(truth);
        let local: Vec<(f64, f64)> = pts.iter().map(|&p| crate::se2::apply(inv, p)).collect();
        let f = MatchField::new(&g, 0.08, 0.3);
        let win = SearchWindow {
            lin: 0.3,
            ang: 0.3,
            ang_res: None,
            w_lin: 0.0,
            w_ang: 0.0,
        };
        let m = match_scan(&f, &local, Pose2::default(), &win, 0.2).unwrap();
        assert!(m.pose.distance_to(truth.x, truth.y) < 0.03, "{:?}", m.pose);
        assert!(wrap(m.pose.yaw - truth.yaw).abs() < 0.01, "{:?}", m.pose);
        assert!(m.score > 0.8);
    }
}
