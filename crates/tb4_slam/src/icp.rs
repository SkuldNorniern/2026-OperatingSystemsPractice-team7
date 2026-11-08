use std::collections::HashMap;

use tb4_core::Pose2;

use crate::se2::wrap;

/// points with normals, hashed for nearest neighbour. built from raw keyframe scans so it keeps
/// sub-cell detail the occupancy grid throws away
pub struct PointMap {
    cell: f64,
    pts: Vec<(f64, f64, f64, f64)>,
    index: HashMap<(i32, i32), Vec<u32>>,
}

impl PointMap {
    /// `voxel`: one point kept per voxel (averaged). normals from neighbours within `normal_r`
    pub fn new(points: impl Iterator<Item = (f64, f64)>, voxel: f64, normal_r: f64) -> Self {
        let mut acc: HashMap<(i32, i32), (f64, f64, u32)> = HashMap::new();
        for (x, y) in points {
            let e = acc.entry(((x / voxel).floor() as i32, (y / voxel).floor() as i32)).or_insert((0.0, 0.0, 0));
            e.0 += x;
            e.1 += y;
            e.2 += 1;
        }
        let centers: Vec<(f64, f64)> = acc.values().map(|&(x, y, n)| (x / n as f64, y / n as f64)).collect();
        let cell = normal_r;
        let key = |x: f64, y: f64| ((x / cell).floor() as i32, (y / cell).floor() as i32);
        let mut grid: HashMap<(i32, i32), Vec<u32>> = HashMap::new();
        for (i, &(x, y)) in centers.iter().enumerate() {
            grid.entry(key(x, y)).or_default().push(i as u32);
        }
        let mut pts = Vec::new();
        for &(x, y) in &centers {
            let (kx, ky) = key(x, y);
            let (mut n, mut sx, mut sy, mut sxx, mut syy, mut sxy) = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
            for dy in -1..=1 {
                for dx in -1..=1 {
                    for &j in grid.get(&(kx + dx, ky + dy)).into_iter().flatten() {
                        let (qx, qy) = centers[j as usize];
                        if (qx - x).hypot(qy - y) > normal_r {
                            continue;
                        }
                        n += 1.0;
                        sx += qx;
                        sy += qy;
                        sxx += qx * qx;
                        syy += qy * qy;
                        sxy += qx * qy;
                    }
                }
            }
            if n < 4.0 {
                continue;
            }
            let (mx, my) = (sx / n, sy / n);
            let (cxx, cyy, cxy) = (sxx / n - mx * mx, syy / n - my * my, sxy / n - mx * my);
            let tr = cxx + cyy;
            let disc = ((cxx - cyy).powi(2) / 4.0 + cxy * cxy).sqrt();
            // only line-like neighbourhoods give a usable normal
            if tr <= 0.0 || (tr / 2.0 - disc) / (tr / 2.0 + disc) > 0.1 {
                continue;
            }
            let ang = 0.5 * (2.0 * cxy).atan2(cxx - cyy);
            pts.push((x, y, -ang.sin(), ang.cos()));
        }
        let mut index: HashMap<(i32, i32), Vec<u32>> = HashMap::new();
        for (i, p) in pts.iter().enumerate() {
            index.entry(key(p.0, p.1)).or_default().push(i as u32);
        }
        Self { cell, pts, index }
    }

    pub fn len(&self) -> usize {
        self.pts.len()
    }

    pub fn is_empty(&self) -> bool {
        self.pts.is_empty()
    }

    fn nearest(&self, x: f64, y: f64, max_d: f64) -> Option<(f64, f64, f64, f64)> {
        let (kx, ky) = ((x / self.cell).floor() as i32, (y / self.cell).floor() as i32);
        let mut best = None;
        let mut bd = max_d * max_d;
        for dy in -1..=1 {
            for dx in -1..=1 {
                for &j in self.index.get(&(kx + dx, ky + dy)).into_iter().flatten() {
                    let p = self.pts[j as usize];
                    let d = (p.0 - x).powi(2) + (p.1 - y).powi(2);
                    if d < bd {
                        bd = d;
                        best = Some(p);
                    }
                }
            }
        }
        best
    }
}

#[derive(Debug, Clone, Copy)]
pub struct IcpParams {
    pub max_corr: f64,
    pub huber: f64,
    pub iterations: usize,
    pub min_inlier_frac: f64,
}

impl Default for IcpParams {
    fn default() -> Self {
        Self { max_corr: 0.15, huber: 0.03, iterations: 12, min_inlier_frac: 0.5 }
    }
}

/// point-to-line gauss-newton from `start`. None when too few points match
pub fn icp(map: &PointMap, pts: &[(f64, f64)], start: Pose2, p: &IcpParams) -> Option<Pose2> {
    let mut pose = start;
    let mut inliers = 0;
    for it in 0..p.iterations {
        // shrink correspondence gate once close
        let gate = if it < 3 { p.max_corr } else { p.max_corr * 0.5 };
        let (s, c) = pose.yaw.sin_cos();
        let mut h = [[0.0f64; 3]; 3];
        let mut g = [0.0f64; 3];
        inliers = 0;
        for &(px, py) in pts {
            let (wx, wy) = (pose.x + c * px - s * py, pose.y + s * px + c * py);
            let Some((qx, qy, nx, ny)) = map.nearest(wx, wy, gate) else { continue };
            let r = nx * (wx - qx) + ny * (wy - qy);
            let w = if r.abs() <= p.huber { 1.0 } else { p.huber / r.abs() };
            let j = [nx, ny, nx * (-s * px - c * py) + ny * (c * px - s * py)];
            for a in 0..3 {
                g[a] += w * j[a] * r;
                for b in 0..3 {
                    h[a][b] += w * j[a] * j[b];
                }
            }
            inliers += 1;
        }
        if (inliers as f64) < p.min_inlier_frac * pts.len() as f64 || inliers < 20 {
            return None;
        }
        for (a, row) in h.iter_mut().enumerate() {
            row[a] += 1e-6;
        }
        let d = crate::matcher::solve3(h, [-g[0], -g[1], -g[2]])?;
        pose = Pose2::new(pose.x + d[0], pose.y + d[1], wrap(pose.yaw + d[2]));
        if d[0].abs() + d[1].abs() < 1e-5 && d[2].abs() < 1e-5 {
            break;
        }
    }
    (inliers > 0).then_some(pose)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn room_pts(pose: Pose2) -> Vec<(f64, f64)> {
        // 6 x 4 box seen from pose, in robot frame
        let mut out = Vec::new();
        for k in 0..360 {
            let a = (k as f64).to_radians() + pose.yaw;
            let (dx, dy) = (a.cos(), a.sin());
            let mut t = f64::INFINITY;
            for (wx, wy, horiz) in [(0.0, 0.0, false), (6.0, 0.0, false), (0.0, 0.0, true), (0.0, 4.0, true)] {
                let tt = if horiz { (wy - pose.y) / dy } else { (wx - pose.x) / dx };
                if tt > 0.0 && tt < t {
                    t = tt;
                }
            }
            let (wx, wy) = (pose.x + t * dx, pose.y + t * dy);
            let (s, c) = pose.yaw.sin_cos();
            let (rx, ry) = (wx - pose.x, wy - pose.y);
            out.push((c * rx + s * ry, -s * rx + c * ry));
        }
        out
    }

    #[test]
    fn recovers_small_offset() {
        let truth = Pose2::new(2.0, 1.5, 0.3);
        let map = PointMap::new(room_pts(Pose2::new(3.0, 2.0, 0.0)).into_iter().map(|(x, y)| (x + 3.0, y + 2.0)), 0.025, 0.12);
        assert!(map.len() > 200);
        let start = Pose2::new(2.04, 1.47, 0.32);
        let p = icp(&map, &room_pts(truth), start, &IcpParams::default()).unwrap();
        assert!((p.x - 2.0).abs() < 0.005 && (p.y - 1.5).abs() < 0.005 && (p.yaw - 0.3).abs() < 0.002, "{p:?}");
    }
}
