use std::collections::VecDeque;

use tb4_core::grid::{Cell, Grid, GridMeta};

#[derive(Debug, Clone)]
pub struct CoverageParams {
    pub half_fov: f64,
    pub range: f64,
    /// camera ahead of base along heading (m)
    pub cam_offset: f64,
    pub ray_step: f64,
    pub occupied_threshold: i8,
    pub min_cluster_cells: usize,
    /// long walls are split into pieces of at most this many cells, one view each
    pub max_cluster_cells: usize,
    /// distance from wall to stand when looking at it (m)
    pub standoff: f64,
    /// viewpoint must be at least this far from any obstacle (m)
    pub min_clearance: f64,
}

impl Default for CoverageParams {
    fn default() -> Self {
        Self {
            half_fov: 0.52,
            range: 2.5,
            cam_offset: 0.1,
            ray_step: 0.02,
            occupied_threshold: 65,
            min_cluster_cells: 6,
            max_cluster_cells: 24,
            standoff: 1.2,
            min_clearance: 0.35,
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct View {
    pub x: f64,
    pub y: f64,
    pub yaw: f64,
    /// wall point this view looks at
    pub target: (f64, f64),
    /// unseen wall cells this view targets
    pub cells: usize,
    pub distance: f64,
}

/// which occupied cells have been inside the camera view, kept in world coordinates so the map can grow
#[derive(Debug, Default, Clone)]
pub struct Coverage {
    seen: std::collections::HashSet<(i64, i64)>,
    res: f64,
}

impl Coverage {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn seen_count(&self) -> usize {
        self.seen.len()
    }

    fn key(&self, x: f64, y: f64) -> (i64, i64) {
        ((x / self.res).floor() as i64, (y / self.res).floor() as i64)
    }

    pub fn is_seen(&self, g: &Grid, c: Cell) -> bool {
        if self.res <= 0.0 {
            return false;
        }
        let (x, y) = g.cell_to_world(c);
        self.seen.contains(&self.key(x, y))
    }

    /// mark wall cells the camera sees from pose (x, y, yaw)
    pub fn update(&mut self, g: &Grid, pose: (f64, f64, f64), p: &CoverageParams) {
        if self.res <= 0.0 || (self.res - g.meta.resolution).abs() > 1e-9 {
            self.res = g.meta.resolution;
        }
        let (cx, cy) = (pose.0 + p.cam_offset * pose.2.cos(), pose.1 + p.cam_offset * pose.2.sin());
        let n = ((2.0 * p.half_fov) / (0.5 * g.meta.resolution / p.range)).ceil().max(2.0) as usize;
        for i in 0..=n {
            let a = pose.2 - p.half_fov + 2.0 * p.half_fov * i as f64 / n as f64;
            if let Some(hit) = g.raycast(cx, cy, a.cos(), a.sin(), p.range, p.occupied_threshold) {
                // step just inside the wall cell
                let (hx, hy) = (hit.x + a.cos() * 1e-3, hit.y + a.sin() * 1e-3);
                let k = self.key(hx, hy);
                self.seen.insert(k);
            }
        }
    }

    /// clusters of occupied cells that touch free space but were never seen, with a viewpoint for each
    pub fn views(&self, g: &Grid, robot: (f64, f64), p: &CoverageParams) -> Vec<View> {
        let (w, h) = (g.meta.width as usize, g.meta.height as usize);
        let free = |c: Cell| (0..p.occupied_threshold).contains(&g.get(c));
        let occ = |c: Cell| g.get(c) >= p.occupied_threshold;
        let mut want = vec![false; w * h];
        for y in 0..h {
            for x in 0..w {
                let c = Cell { x: x as u32, y: y as u32 };
                if !occ(c) || self.is_seen(g, c) {
                    continue;
                }
                let touches_free = [(1i64, 0i64), (-1, 0), (0, 1), (0, -1)]
                    .iter()
                    .any(|(dx, dy)| g.cell_checked(x as i64 + dx, y as i64 + dy).is_some_and(free));
                want[g.index(c)] = touches_free;
            }
        }
        let clearance = tb4_core::DistanceField::from_grid(g, p.occupied_threshold, true);
        let mut seen = vec![false; w * h];
        let mut out = Vec::new();
        let mut q = VecDeque::new();
        for start in 0..w * h {
            if !want[start] || seen[start] {
                continue;
            }
            let mut members = Vec::new();
            seen[start] = true;
            q.push_back(start);
            while let Some(i) = q.pop_front() {
                members.push(i);
                let (x, y) = ((i % w) as i64, (i / w) as i64);
                for dy in -1..=1 {
                    for dx in -1..=1 {
                        if let Some(n) = g.cell_checked(x + dx, y + dy) {
                            let j = g.index(n);
                            if want[j] && !seen[j] {
                                seen[j] = true;
                                q.push_back(j);
                            }
                        }
                    }
                }
            }
            if members.len() < p.min_cluster_cells {
                continue;
            }
            // bfs order keeps each chunk contiguous along the wall
            for chunk in members.chunks(p.max_cluster_cells.max(1)) {
                if chunk.len() < p.min_cluster_cells.min(members.len()) {
                    continue;
                }
                let pts: Vec<(f64, f64)> =
                    chunk.iter().map(|&i| g.cell_to_world(Cell { x: (i % w) as u32, y: (i / w) as u32 })).collect();
                let n = pts.len() as f64;
                let mid = (pts.iter().map(|p| p.0).sum::<f64>() / n, pts.iter().map(|p| p.1).sum::<f64>() / n);
                // aim at the chunk cell nearest its centroid, so the target is on the wall
                let c = *pts.iter().min_by(|a, b| d2(**a, mid).total_cmp(&d2(**b, mid))).unwrap();
                if let Some(v) = viewpoint(g, &clearance, c, robot, chunk.len(), p) {
                    out.push(v);
                }
            }
        }
        out.sort_by(|a, b| (a.distance - 0.2 * a.cells as f64 * g.meta.resolution)
            .total_cmp(&(b.distance - 0.2 * b.cells as f64 * g.meta.resolution)));
        out
    }
}

fn d2(a: (f64, f64), b: (f64, f64)) -> f64 {
    (a.0 - b.0).powi(2) + (a.1 - b.1).powi(2)
}

/// free, clear point about `standoff` from target with line of sight to it, nearest to robot
fn viewpoint(
    g: &Grid,
    clearance: &tb4_core::DistanceField,
    target: (f64, f64),
    robot: (f64, f64),
    cells: usize,
    p: &CoverageParams,
) -> Option<View> {
    let meta: GridMeta = g.meta;
    let mut best: Option<View> = None;
    for ring in [p.standoff, p.standoff * 0.75, p.standoff * 1.4] {
        for k in 0..24 {
            let a = k as f64 * std::f64::consts::TAU / 24.0;
            let (x, y) = (target.0 + ring * a.cos(), target.1 + ring * a.sin());
            let Some(c) = g.world_to_cell(x, y) else { continue };
            if !(0..p.occupied_threshold).contains(&g.get(c)) || (clearance.get(c) as f64) < p.min_clearance {
                continue;
            }
            let (dx, dy) = (target.0 - x, target.1 - y);
            let d = dx.hypot(dy);
            match g.raycast(x, y, dx, dy, d + meta.resolution * 2.0, p.occupied_threshold) {
                Some(hit) if hit.range >= d - 3.0 * meta.resolution => {}
                _ => continue,
            }
            let dist = (x - robot.0).hypot(y - robot.1);
            if best.as_ref().is_none_or(|b| dist < b.distance) {
                best = Some(View { x, y, yaw: dy.atan2(dx), target, cells, distance: dist });
            }
        }
        if best.is_some() {
            break;
        }
    }
    best
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 6 x 4 m room, walls 1 cell thick, all known
    fn room() -> Grid {
        let meta = GridMeta { width: 120, height: 80, resolution: 0.05, origin_x: 0.0, origin_y: 0.0 };
        let mut g = Grid::filled(meta, 0);
        for x in 0..120 {
            g.set(Cell { x, y: 0 }, 100);
            g.set(Cell { x, y: 79 }, 100);
        }
        for y in 0..80 {
            g.set(Cell { x: 0, y }, 100);
            g.set(Cell { x: 119, y }, 100);
        }
        g
    }

    #[test]
    fn camera_marks_wall_in_front_only() {
        let g = room();
        let mut cov = Coverage::new();
        let p = CoverageParams { range: 4.0, ..Default::default() };
        cov.update(&g, (3.0, 2.0, 0.0), &p);
        let east = g.world_to_cell(5.97, 2.0).unwrap();
        let west = g.world_to_cell(0.02, 2.0).unwrap();
        assert!(cov.is_seen(&g, east));
        assert!(!cov.is_seen(&g, west));
        assert!(cov.seen_count() > 20);
    }

    #[test]
    fn views_point_at_unseen_walls() {
        let g = room();
        let mut cov = Coverage::new();
        let p = CoverageParams { range: 4.0, ..Default::default() };
        let views = cov.views(&g, (3.0, 2.0), &p);
        assert!(!views.is_empty());
        for v in &views {
            let c = g.world_to_cell(v.x, v.y).unwrap();
            assert_eq!(g.get(c), 0);
        }
        // look at every wall from the middle: nothing left
        for k in 0..16 {
            cov.update(&g, (3.0, 2.0, k as f64 * std::f64::consts::TAU / 16.0), &p);
        }
        assert!(cov.views(&g, (3.0, 2.0), &p).is_empty(), "{:?}", cov.views(&g, (3.0, 2.0), &p));
    }

    #[test]
    fn view_faces_target() {
        let g = room();
        let cov = Coverage::new();
        let p = CoverageParams { range: 4.0, min_cluster_cells: 3, ..Default::default() };
        let v = cov.views(&g, (5.0, 2.0), &p).into_iter().next().unwrap();
        // looking along yaw from the view hits a wall about `standoff` away
        let hit = g.raycast(v.x, v.y, v.yaw.cos(), v.yaw.sin(), 3.0, 65).expect("view looks at a wall");
        assert!((hit.range - p.standoff).abs() < 0.5, "{v:?} {hit:?}");
        let ahead = (v.x + 1.2 * v.yaw.cos(), v.y + 1.2 * v.yaw.sin());
        let near_wall = ahead.0 > 5.7 || ahead.0 < 0.3 || ahead.1 > 3.7 || ahead.1 < 0.3;
        assert!(near_wall, "{v:?}");
    }
}
