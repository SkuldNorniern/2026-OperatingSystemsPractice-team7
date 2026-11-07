use tb4_core::Pose2;

use crate::se2::{between, wrap};

#[derive(Debug, Clone, Copy)]
pub struct Edge {
    pub i: usize,
    pub j: usize,
    /// measured pose of j in the frame of i
    pub z: Pose2,
    /// diagonal information (x, y, yaw)
    pub info: [f64; 3],
    pub is_loop: bool,
}

pub fn edge_error(poses: &[Pose2], e: &Edge) -> [f64; 3] {
    let d = between(e.z, between(poses[e.i], poses[e.j]));
    [d.x, d.y, d.yaw]
}

pub fn edge_chi2(poses: &[Pose2], e: &Edge) -> f64 {
    let r = edge_error(poses, e);
    (0..3).map(|k| e.info[k] * r[k] * r[k]).sum()
}

fn total_chi2(poses: &[Pose2], edges: &[Edge]) -> f64 {
    edges.iter().map(|e| edge_chi2(poses, e)).sum()
}

type J3 = [[f64; 3]; 3];

fn jacobians(poses: &[Pose2], e: &Edge) -> (J3, J3, [f64; 3]) {
    let (xi, xj) = (poses[e.i], poses[e.j]);
    let (si, ci) = xi.yaw.sin_cos();
    let (sz, cz) = e.z.yaw.sin_cos();
    let (dx, dy) = (xj.x - xi.x, xj.y - xi.y);
    // M = Rz^T Ri^T
    let m = [
        [cz * ci - sz * si, cz * si + sz * ci],
        [-sz * ci - cz * si, -sz * si + cz * ci],
    ];
    // dRi^T/dtheta * (tj - ti), then rotated by Rz^T
    let (u, v) = (-si * dx + ci * dy, -ci * dx - si * dy);
    let (au, av) = (cz * u + sz * v, -sz * u + cz * v);
    let a = [[-m[0][0], -m[0][1], au], [-m[1][0], -m[1][1], av], [0.0, 0.0, -1.0]];
    let b = [[m[0][0], m[0][1], 0.0], [m[1][0], m[1][1], 0.0], [0.0, 0.0, 1.0]];
    (a, b, edge_error(poses, e))
}

/// Lower-triangular envelope (profile) storage for symmetric positive definite matrices.
#[derive(Clone)]
struct Skyline {
    first: Vec<usize>,
    off: Vec<usize>,
    val: Vec<f64>,
}

impl Skyline {
    fn new(first: Vec<usize>) -> Self {
        let mut off = Vec::with_capacity(first.len());
        let mut n = 0;
        for (r, &f) in first.iter().enumerate() {
            off.push(n);
            n += r - f + 1;
        }
        Self {
            first,
            off,
            val: vec![0.0; n],
        }
    }

    #[inline]
    fn pos(&self, r: usize, c: usize) -> usize {
        debug_assert!(c <= r && c >= self.first[r]);
        self.off[r] + c - self.first[r]
    }

    fn factor(&mut self) -> bool {
        for i in 0..self.first.len() {
            let fi = self.first[i];
            for j in fi..=i {
                let fj = self.first[j];
                let k0 = fi.max(fj);
                let dot: f64 = {
                    let ri = &self.val[self.pos(i, k0)..self.pos(i, k0) + (j - k0)];
                    let rj = &self.val[self.off[j] + k0 - fj..self.off[j] + j - fj];
                    ri.iter().zip(rj).map(|(a, b)| a * b).sum()
                };
                let p = self.pos(i, j);
                let s = self.val[p] - dot;
                if j < i {
                    self.val[p] = s / self.val[self.pos(j, j)];
                } else {
                    if s <= 0.0 || !s.is_finite() {
                        return false;
                    }
                    self.val[p] = s.sqrt();
                }
            }
        }
        true
    }

    fn solve(&self, b: &mut [f64]) {
        let n = b.len();
        for i in 0..n {
            let fi = self.first[i];
            let row = &self.val[self.off[i]..self.off[i] + i - fi];
            let s: f64 = row.iter().zip(&b[fi..i]).map(|(l, y)| l * y).sum();
            b[i] = (b[i] - s) / self.val[self.pos(i, i)];
        }
        for i in (0..n).rev() {
            let fi = self.first[i];
            b[i] /= self.val[self.pos(i, i)];
            let xi = b[i];
            let row = &self.val[self.off[i]..self.off[i] + i - fi];
            for (bk, l) in b[fi..i].iter_mut().zip(row) {
                *bk -= l * xi;
            }
        }
    }
}

fn add_block(h: &mut Skyline, bi: usize, ji: &J3, bj: usize, jj: &J3, info: &[f64; 3]) {
    for r in 0..3 {
        let cols = if bi == bj { r + 1 } else { 3 };
        for c in 0..cols {
            let v: f64 = ji.iter().zip(jj).zip(info).map(|((a, b), w)| a[r] * w * b[c]).sum();
            let p = h.pos(3 * bi + r, 3 * bj + c);
            h.val[p] += v;
        }
    }
}

fn bfs_levels(start: usize, adj: &[Vec<usize>], done: &[bool]) -> Vec<Vec<usize>> {
    let mut seen = done.to_vec();
    seen[start] = true;
    let mut levels = vec![vec![start]];
    loop {
        let mut next = Vec::new();
        for &v in levels.last().unwrap() {
            for &u in &adj[v] {
                if !seen[u] {
                    seen[u] = true;
                    next.push(u);
                }
            }
        }
        if next.is_empty() {
            return levels;
        }
        levels.push(next);
    }
}

/// Reverse Cuthill-McKee ordering (George-Liu start node); returns position of each node.
fn rcm(adj: &[Vec<usize>]) -> Vec<usize> {
    let n = adj.len();
    let mut done = vec![false; n];
    let mut order = Vec::with_capacity(n);
    while order.len() < n {
        let mut start = (0..n).filter(|&v| !done[v]).min_by_key(|&v| adj[v].len()).unwrap();
        let mut depth = bfs_levels(start, adj, &done).len();
        for _ in 0..4 {
            let levels = bfs_levels(start, adj, &done);
            let cand = *levels.last().unwrap().iter().min_by_key(|&&v| adj[v].len()).unwrap();
            let d = bfs_levels(cand, adj, &done).len();
            if d <= depth {
                break;
            }
            (start, depth) = (cand, d);
        }
        let base = order.len();
        order.push(start);
        done[start] = true;
        let mut head = base;
        while head < order.len() {
            let v = order[head];
            head += 1;
            let mut nb: Vec<usize> = adj[v].iter().copied().filter(|&u| !done[u]).collect();
            nb.sort_by_key(|&u| adj[u].len());
            for u in nb {
                done[u] = true;
                order.push(u);
            }
        }
    }
    let mut pos = vec![0; n];
    for (k, &v) in order.iter().rev().enumerate() {
        pos[v] = k;
    }
    pos
}

/// Levenberg-Marquardt over SE(2) poses, node 0 fixed. Returns final chi2.
pub fn optimize(poses: &mut [Pose2], edges: &[Edge], iters: usize) -> f64 {
    let n = poses.len();
    let mut cur = total_chi2(poses, edges);
    if n < 2 {
        return cur;
    }
    // unknowns are nodes 1..n, block = pos[node - 1]
    let nb = n - 1;
    let mut adj = vec![Vec::new(); nb];
    for e in edges {
        if e.i >= 1 && e.j >= 1 && e.i != e.j {
            adj[e.i - 1].push(e.j - 1);
            adj[e.j - 1].push(e.i - 1);
        }
    }
    for a in &mut adj {
        a.sort_unstable();
        a.dedup();
    }
    let pos = rcm(&adj);
    let block = |node: usize| pos[node - 1];
    let mut fb: Vec<usize> = (0..nb).collect();
    for (u, a) in adj.iter().enumerate() {
        for &v in a {
            let (lo, hi) = (pos[u].min(pos[v]), pos[u].max(pos[v]));
            fb[hi] = fb[hi].min(lo);
        }
    }
    let template = Skyline::new((0..3 * nb).map(|r| 3 * fb[r / 3]).collect());
    let mut lambda = 1e-4;
    for _ in 0..iters {
        let mut h = template.clone();
        let mut g = vec![0.0; 3 * nb];
        for e in edges {
            if e.i == e.j {
                continue;
            }
            let (a, b, err) = jacobians(poses, e);
            for (node, jac) in [(e.i, &a), (e.j, &b)] {
                if node == 0 {
                    continue;
                }
                let bi = block(node);
                for r in 0..3 {
                    g[3 * bi + r] += (0..3).map(|k| jac[k][r] * e.info[k] * err[k]).sum::<f64>();
                }
                add_block(&mut h, bi, jac, bi, jac, &e.info);
            }
            if e.i >= 1 && e.j >= 1 {
                let (bi, bj) = (block(e.i), block(e.j));
                if bi > bj {
                    add_block(&mut h, bi, &a, bj, &b, &e.info);
                } else {
                    add_block(&mut h, bj, &b, bi, &a, &e.info);
                }
            }
        }
        let before = cur;
        let mut accepted = false;
        for _ in 0..8 {
            let mut hd = h.clone();
            for r in 0..3 * nb {
                let p = hd.pos(r, r);
                hd.val[p] += lambda * h.val[p] + 1e-9;
            }
            if !hd.factor() {
                lambda *= 10.0;
                continue;
            }
            let mut dx: Vec<f64> = g.iter().map(|v| -v).collect();
            hd.solve(&mut dx);
            let mut trial = poses.to_vec();
            for (k, t) in trial.iter_mut().enumerate().skip(1) {
                let d = &dx[3 * block(k)..3 * block(k) + 3];
                *t = Pose2::new(t.x + d[0], t.y + d[1], wrap(t.yaw + d[2]));
            }
            let c = total_chi2(&trial, edges);
            if c < cur {
                poses.copy_from_slice(&trial);
                cur = c;
                lambda = (lambda * 0.1).max(1e-9);
                accepted = true;
                break;
            }
            lambda *= 10.0;
        }
        if !accepted || before - cur < 1e-6 * before.max(1e-12) {
            break;
        }
    }
    cur
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::se2::compose;

    #[test]
    fn closes_square_loop() {
        let step = Pose2::new(1.0, 0.0, std::f64::consts::FRAC_PI_2 / 5.0);
        let n = 20;
        let mut truth = vec![Pose2::default()];
        for k in 1..n {
            truth.push(compose(truth[k - 1], step));
        }
        let info = [100.0, 100.0, 400.0];
        let mut edges: Vec<Edge> = (1..n)
            .map(|k| Edge {
                i: k - 1,
                j: k,
                z: between(truth[k - 1], truth[k]),
                info,
                is_loop: false,
            })
            .collect();
        edges.push(Edge {
            i: 0,
            j: n - 1,
            z: between(truth[0], truth[n - 1]),
            info,
            is_loop: true,
        });
        edges.push(Edge {
            i: 3,
            j: 13,
            z: between(truth[3], truth[13]),
            info,
            is_loop: true,
        });
        // drifted initial guess
        let mut poses = vec![Pose2::default()];
        for k in 1..n {
            let bad = Pose2::new(step.x * 1.03, 0.02, step.yaw + 0.03);
            poses.push(compose(poses[k - 1], bad));
        }
        let chi = optimize(&mut poses, &edges, 20);
        assert!(chi < 1e-8, "{chi}");
        for (p, t) in poses.iter().zip(&truth) {
            assert!(p.distance_to(t.x, t.y) < 1e-4 && wrap(p.yaw - t.yaw).abs() < 1e-5);
        }
    }

    #[test]
    fn skyline_matches_dense() {
        // 4x4 SPD with envelope first = [0, 0, 1, 0]
        let a = [
            [4.0, 1.0, 0.0, 0.5],
            [1.0, 3.0, 0.2, 0.0],
            [0.0, 0.2, 2.0, 0.3],
            [0.5, 0.0, 0.3, 5.0],
        ];
        let mut s = Skyline::new(vec![0, 0, 1, 0]);
        for (r, row) in a.iter().enumerate() {
            for (c, &v) in row.iter().enumerate().take(r + 1).skip(s.first[r]) {
                let p = s.pos(r, c);
                s.val[p] = v;
            }
        }
        assert!(s.factor());
        let x = [1.0, -2.0, 0.5, 3.0];
        let mut b: Vec<f64> = (0..4).map(|r| (0..4).map(|c| a[r][c] * x[c]).sum()).collect();
        s.solve(&mut b);
        for (u, v) in b.iter().zip(x) {
            assert!((u - v).abs() < 1e-12);
        }
    }
}
