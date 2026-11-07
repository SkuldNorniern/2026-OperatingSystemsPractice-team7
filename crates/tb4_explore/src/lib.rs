mod coverage;
pub use coverage::{Coverage, CoverageParams, View};

use std::collections::VecDeque;

use tb4_core::grid::{Cell, Grid, UNKNOWN};

#[derive(Debug, Clone)]
pub struct ExploreParams {
    pub free_threshold: i8,
    pub min_frontier_cells: usize,
    pub size_weight: f64,
    pub distance_weight: f64,
    pub blacklist_radius: f64,
    pub min_goal_distance: f64,
}

impl Default for ExploreParams {
    fn default() -> Self {
        Self {
            free_threshold: 50,
            min_frontier_cells: 8,
            size_weight: 1.0,
            distance_weight: 1.0,
            blacklist_radius: 0.5,
            min_goal_distance: 0.3,
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct Frontier {
    pub cells: usize,
    pub size_m: f64,
    pub centroid: (f64, f64),
    pub goal: (f64, f64),
    pub distance: f64,
    pub score: f64,
}

#[inline]
fn is_free(v: i8, thr: i8) -> bool {
    (0..thr).contains(&v)
}

fn is_frontier(g: &Grid, c: Cell, thr: i8) -> bool {
    if !is_free(g.get(c), thr) {
        return false;
    }
    let (x, y) = (c.x as i64, c.y as i64);
    [(1, 0), (-1, 0), (0, 1), (0, -1)]
        .iter()
        .any(|(dx, dy)| g.cell_checked(x + dx, y + dy).is_some_and(|n| g.get(n) == UNKNOWN))
}

pub fn find_frontiers(g: &Grid, robot: (f64, f64), p: &ExploreParams) -> Vec<Frontier> {
    let (w, h) = (g.meta.width as usize, g.meta.height as usize);
    let mut flag = vec![false; w * h];
    for y in 0..h {
        for x in 0..w {
            let c = Cell { x: x as u32, y: y as u32 };
            flag[g.index(c)] = is_frontier(g, c, p.free_threshold);
        }
    }

    let mut seen = vec![false; w * h];
    let mut out = Vec::new();
    let mut queue = VecDeque::new();
    let mut members = Vec::new();
    for start in 0..w * h {
        if !flag[start] || seen[start] {
            continue;
        }
        seen[start] = true;
        queue.push_back(start);
        members.clear();
        while let Some(i) = queue.pop_front() {
            members.push(i);
            let (x, y) = ((i % w) as i64, (i / w) as i64);
            for dy in -1..=1 {
                for dx in -1..=1 {
                    if let Some(n) = g.cell_checked(x + dx, y + dy) {
                        let j = g.index(n);
                        if flag[j] && !seen[j] {
                            seen[j] = true;
                            queue.push_back(j);
                        }
                    }
                }
            }
        }
        if members.len() < p.min_frontier_cells {
            continue;
        }
        let to_world = |i: usize| g.cell_to_world(Cell { x: (i % w) as u32, y: (i / w) as u32 });
        let (sx, sy) = members.iter().map(|&i| to_world(i)).fold((0.0, 0.0), |a, b| (a.0 + b.0, a.1 + b.1));
        let n = members.len() as f64;
        let centroid = (sx / n, sy / n);
        let goal = members
            .iter()
            .map(|&i| to_world(i))
            .min_by(|a, b| dist2(*a, centroid).total_cmp(&dist2(*b, centroid)))
            .unwrap();
        let size_m = n * g.meta.resolution;
        let distance = dist2(goal, robot).sqrt();
        out.push(Frontier {
            cells: members.len(),
            size_m,
            centroid,
            goal,
            distance,
            score: p.size_weight * size_m - p.distance_weight * distance,
        });
    }
    out.sort_by(|a, b| b.score.total_cmp(&a.score));
    out
}

pub fn select_goal<'a>(frontiers: &'a [Frontier], blacklist: &[(f64, f64)], p: &ExploreParams) -> Option<&'a Frontier> {
    let r2 = p.blacklist_radius * p.blacklist_radius;
    frontiers
        .iter()
        .filter(|f| f.distance >= p.min_goal_distance)
        .find(|f| blacklist.iter().all(|b| dist2(*b, f.goal) > r2))
}

/// prefer frontiers in front of the robot: score -= weight * |turn angle| (rad), then re-sort
pub fn rescore_heading(frontiers: &mut [Frontier], robot: (f64, f64, f64), weight: f64) {
    if weight <= 0.0 {
        return;
    }
    for f in frontiers.iter_mut() {
        let a = (f.goal.1 - robot.1).atan2(f.goal.0 - robot.0) - robot.2;
        f.score -= weight * a.sin().atan2(a.cos()).abs();
    }
    frontiers.sort_by(|a, b| b.score.total_cmp(&a.score));
}

#[inline]
fn dist2(a: (f64, f64), b: (f64, f64)) -> f64 {
    (a.0 - b.0).powi(2) + (a.1 - b.1).powi(2)
}

#[cfg(test)]
mod tests {
    use super::*;
    use tb4_core::grid::GridMeta;

    fn partial_map() -> Grid {
        let meta = GridMeta { width: 40, height: 40, resolution: 0.1, origin_x: 0.0, origin_y: 0.0 };
        let mut g = Grid::filled(meta, UNKNOWN);
        for y in 0..20 {
            for x in 0..20 {
                g.set(Cell { x, y }, 0);
            }
        }
        for i in 0..20 {
            g.set(Cell { x: 0, y: i }, 100);
            g.set(Cell { x: i, y: 0 }, 100);
        }
        g
    }

    #[test]
    fn finds_l_shaped_frontier() {
        let g = partial_map();
        let f = find_frontiers(&g, (0.5, 0.5), &ExploreParams::default());
        assert_eq!(f.len(), 1, "{f:?}");
        assert_eq!(f[0].cells, 37);
        let (gx, gy) = f[0].goal;
        assert!(g.world_to_cell(gx, gy).is_some());
    }

    #[test]
    fn separate_clusters_are_scored() {
        let mut g = partial_map();
        for d in 17..20 {
            for e in 17..20 {
                g.set(Cell { x: d, y: e }, 100);
            }
        }
        let p = ExploreParams { min_frontier_cells: 3, ..Default::default() };
        let f = find_frontiers(&g, (1.8, 0.5), &p);
        assert_eq!(f.len(), 2, "{f:?}");
        assert!(f[0].goal.0 > 1.8);
        assert!(f[0].score >= f[1].score);
    }

    #[test]
    fn blacklist_and_min_distance() {
        let g = partial_map();
        let p = ExploreParams::default();
        let f = find_frontiers(&g, (0.5, 0.5), &p);
        assert!(select_goal(&f, &[], &p).is_some());
        assert!(select_goal(&f, &[f[0].goal], &p).is_none());
        let near = find_frontiers(&g, f[0].goal, &p);
        assert!(select_goal(&near, &[], &p).is_none());
    }

    #[test]
    fn heading_prefers_frontier_ahead() {
        let mk = |x: f64, y: f64| Frontier { cells: 10, size_m: 1.0, centroid: (x, y), goal: (x, y), distance: 2.0, score: 0.0 };
        let mut f = vec![mk(-2.0, 0.0), mk(2.0, 0.0)];
        rescore_heading(&mut f, (0.0, 0.0, 0.0), 0.5);
        assert_eq!(f[0].goal, (2.0, 0.0));
        assert!((f[1].score + 0.5 * std::f64::consts::PI).abs() < 1e-9);
    }

    #[test]
    fn fully_known_map_has_no_frontiers() {
        let meta = GridMeta { width: 10, height: 10, resolution: 0.1, origin_x: 0.0, origin_y: 0.0 };
        let g = Grid::filled(meta, 0);
        assert!(find_frontiers(&g, (0.0, 0.0), &ExploreParams::default()).is_empty());
    }
}
