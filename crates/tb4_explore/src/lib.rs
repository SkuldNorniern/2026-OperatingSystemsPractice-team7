mod coverage;
pub use coverage::{Coverage, CoverageParams, View};

use std::collections::{BTreeMap, VecDeque};

use tb4_core::grid::{Cell, Grid, UNKNOWN};

#[derive(Debug, Clone)]
pub struct ExploreParams {
    pub free_threshold: i8,
    pub min_frontier_cells: usize,
    pub size_weight: f64,
    pub distance_weight: f64,
    pub blacklist_radius: f64,
    pub min_goal_distance: f64,
    pub goal_standoff: f64,
    pub traversable_threshold: i8,
    pub frontier_segment_length: f64,
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
            goal_standoff: 0.25,
            traversable_threshold: 99,
            frontier_segment_length: 0.0,
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
    let Some(start) = g.world_to_cell(robot.0, robot.1).filter(|c| is_free(g.get(*c), p.traversable_threshold)) else {
        return Vec::new();
    };
    // Flood only traversable cells connected to the robot. Euclidean distance
    // alone selected frontiers behind walls and in disconnected map islands.
    let root = g.index(start);
    let mut steps = vec![usize::MAX; w * h];
    let mut parent = vec![usize::MAX; w * h];
    steps[root] = 0;
    parent[root] = root;
    let mut reachable = VecDeque::from([root]);
    while let Some(i) = reachable.pop_front() {
        let (x, y) = ((i % w) as i64, (i / w) as i64);
        for (dx, dy) in [(1, 0), (-1, 0), (0, 1), (0, -1)] {
            if let Some(c) = g.cell_checked(x + dx, y + dy) {
                let j = g.index(c);
                if steps[j] == usize::MAX && is_free(g.get(c), p.traversable_threshold) {
                    steps[j] = steps[i] + 1;
                    parent[j] = i;
                    reachable.push_back(j);
                }
            }
        }
    }
    let mut flag = vec![false; w * h];
    for y in 0..h {
        for x in 0..w {
            let c = Cell { x: x as u32, y: y as u32 };
            flag[g.index(c)] = steps[g.index(c)] != usize::MAX && is_frontier(g, c, p.free_threshold);
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
        // A long boundary can wrap around the robot: its centroid then lies
        // near the start and hides useful distant goals. Score local segments.
        let groups = if p.frontier_segment_length > 0.0 {
            let mut bins: BTreeMap<(i64, i64), Vec<usize>> = BTreeMap::new();
            for &i in &members {
                let (x, y) = g.cell_to_world(Cell { x: (i % w) as u32, y: (i / w) as u32 });
                bins.entry(((x / p.frontier_segment_length).floor() as i64,
                            (y / p.frontier_segment_length).floor() as i64)).or_default().push(i);
            }
            bins.into_values().collect::<Vec<_>>()
        } else { vec![members.clone()] };
        for members in groups {
        if members.len() < p.min_frontier_cells {
            continue;
        }
        let to_world = |i: usize| g.cell_to_world(Cell { x: (i % w) as u32, y: (i / w) as u32 });
        let (sx, sy) = members.iter().map(|&i| to_world(i)).fold((0.0, 0.0), |a, b| (a.0 + b.0, a.1 + b.1));
        let n = members.len() as f64;
        let centroid = (sx / n, sy / n);
        let mut goal_index = *members
            .iter()
            .min_by(|&&a, &&b| dist2(to_world(a), centroid).total_cmp(&dist2(to_world(b), centroid)))
            .unwrap();
        // Stop inside known free space, rather than placing the robot centre
        // directly at the unknown boundary. Follow the verified free path back.
        let radius = (p.goal_standoff / g.meta.resolution).ceil() as i64;
        let clear_of_unknown = |i: usize| {
            let (x, y) = ((i % w) as i64, (i / w) as i64);
            (-radius..=radius).all(|dy| (-radius..=radius).all(|dx| {
                dx * dx + dy * dy > radius * radius
                    || g.cell_checked(x + dx, y + dy).is_some_and(|c| g.get(c) != UNKNOWN)
            }))
        };
        while goal_index != root && (!clear_of_unknown(goal_index)
            || !is_free(g.data[goal_index], p.free_threshold)) {
            goal_index = parent[goal_index];
        }
        let goal = to_world(goal_index);
        let size_m = n * g.meta.resolution;
        let distance = steps[goal_index] as f64 * g.meta.resolution;
        out.push(Frontier {
            cells: members.len(),
            size_m,
            centroid,
            goal,
            distance,
            score: p.size_weight * size_m.sqrt() - p.distance_weight * distance,
        });
        }
    }
    out.sort_by(|a, b| b.score.total_cmp(&a.score));
    out
}

pub fn select_goal_with_heading<'a>(frontiers: &'a [Frontier], blacklist: &[(f64, f64)], p: &ExploreParams,
    robot: (f64, f64), yaw: f64, turn_weight: f64) -> Option<&'a Frontier> {
    let score = |f: &Frontier| {
        let angle = (f.goal.1 - robot.1).atan2(f.goal.0 - robot.0) - yaw;
        let turn = angle.sin().atan2(angle.cos()).abs();
        f.score - turn_weight * turn
    };
    frontiers.iter().filter(|f| f.distance >= p.min_goal_distance
        && blacklist.iter().all(|b| dist2(*b, f.goal) > p.blacklist_radius.powi(2)))
        .max_by(|a, b| score(a).total_cmp(&score(b)))
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

    #[test]
    fn ignores_frontiers_separated_from_robot_by_wall() {
        let mut g = partial_map();
        for y in 0..20 { g.set(Cell { x: 10, y }, 100); }
        let p = ExploreParams { min_frontier_cells: 3, ..Default::default() };
        let frontiers = find_frontiers(&g, (0.5, 0.5), &p);
        assert!(!frontiers.is_empty());
        assert!(frontiers.iter().all(|f| f.goal.0 < 1.0));
    }

    #[test]
    fn distance_follows_detour_and_goal_stays_inside_free_space() {
        let mut g = partial_map();
        // Expose unknown space only on the far side of the wall, so reaching
        // every frontier requires going around the wall's lower end.
        for value in &mut g.data { if *value == UNKNOWN { *value = 100; } }
        for y in 2..8 { g.set(Cell { x: 20, y }, UNKNOWN); }
        for y in 1..17 { g.set(Cell { x: 10, y }, 100); }
        let p = ExploreParams { min_frontier_cells: 3, goal_standoff: 0.3, ..Default::default() };
        let robot = (0.8, 0.4);
        let frontiers = find_frontiers(&g, robot, &p);
        assert!(frontiers.iter().any(|f| f.distance > dist2(robot, f.goal).sqrt() + 0.5));
        for f in frontiers {
            let goal = g.world_to_cell(f.goal.0, f.goal.1).unwrap();
            assert!(is_free(g.get(goal), p.free_threshold));
            assert!(!is_frontier(&g, goal, p.free_threshold));
        }
    }

    #[test]
    fn equally_valuable_frontier_ahead_wins_over_turning_back() {
        let ahead = Frontier { cells: 10, size_m: 1.0, centroid: (2.0, 0.0), goal: (1.0, 0.0), distance: 1.0, score: 0.0 };
        let behind = Frontier { centroid: (-2.0, 0.0), goal: (-1.0, 0.0), ..ahead.clone() };
        let frontiers = [behind, ahead];
        let chosen = select_goal_with_heading(&frontiers, &[], &ExploreParams::default(), (0.0, 0.0), 0.0, 0.5).unwrap();
        assert_eq!(chosen.goal, (1.0, 0.0));
    }

    #[test]
    fn segments_large_frontier_and_can_leave_inflated_start_cell() {
        let mut g = partial_map();
        g.set(g.world_to_cell(0.5, 0.5).unwrap(), 70);
        let p = ExploreParams { frontier_segment_length: 1.0, min_frontier_cells: 3, ..Default::default() };
        let frontiers = find_frontiers(&g, (0.5, 0.5), &p);
        assert!(frontiers.len() >= 2);
        assert!(frontiers.iter().all(|f| g.get(g.world_to_cell(f.goal.0, f.goal.1).unwrap()) < p.free_threshold));
        assert!(select_goal(&frontiers, &[], &p).is_some());
    }
}
