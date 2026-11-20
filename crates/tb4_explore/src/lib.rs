mod approach;
mod coverage;
mod policy;
pub use approach::{ApproachParams, MarkerHint, Viewpoint};
pub use coverage::{Coverage, CoverageParams, View};
pub use policy::{
    Action, ActiveGoal, CancelReason, ExplorerParams, ExplorerPolicy, ExplorerStats, GoalKind, GoalRequest, NavStatus, TickInput,
};

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
    /// skip a frontier whose unknown side is an enclosed pocket smaller than this (m^2): the inside
    /// of a box or the space under a desk is never mapped, and its edge stays a frontier for the
    /// whole run (sim studio: two such goals swapped for 8 min). 0 = off
    pub min_unknown_area: f64,
    /// share of unknown cells allowed within goal_standoff of a frontier goal. 0 = none (strict).
    /// A parked robot's map is one A1 scan with unknown speckles between the beams: strict
    /// placement walked every goal back onto the robot and the explorer waited until someone pushed
    /// it (field run 184838, 75 s; recorded costmap at 30 s: 0 goals strict, 11 with 15 %)
    pub goal_max_unknown: f64,
    /// frontiers within this distance (m) of a goal the explorer already reached lose `revisit_penalty`
    /// (score units), so the robot looks at new places before going back to old ones. 0 = off
    pub revisit_radius: f64,
    pub revisit_penalty: f64,
    /// skip a frontier whose free side is a thin spike: share of known-free cells within 0.3 m of
    /// its centroid below this (lidar beams through chair legs and gaps leave 1-2 cell free lines
    /// into unknown that score like big frontiers). 0 = off
    pub min_free_fraction: f64,
    /// score units per m^2 of unseen area within gain_radius of a frontier's centroid: prefers the
    /// frontier that opens more of the map. 0 = off (the gain is then not computed)
    pub gain_weight: f64,
    /// m, radius around a frontier's centroid in which unknown cells count towards its gain
    pub gain_radius: f64,
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
            min_unknown_area: 0.0,
            goal_max_unknown: 0.0,
            revisit_radius: 0.0,
            revisit_penalty: 0.0,
            min_free_fraction: 0.0,
            gain_weight: 0.0,
            gain_radius: 2.0,
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
    /// m^2 of unknown cells within gain_radius of the centroid (0 when gain_weight is 0)
    pub gain: f64,
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

/// share of known-free cells within `radius` (m) of the centroid of `members`
fn free_fraction(g: &Grid, members: &[usize], radius: f64, free_thr: i8) -> f64 {
    let w = g.meta.width as usize;
    let n = members.len() as f64;
    let (sx, sy) = members.iter().fold((0.0, 0.0), |a, &i| (a.0 + (i % w) as f64, a.1 + (i / w) as f64));
    let (cx, cy) = ((sx / n).round() as i64, (sy / n).round() as i64);
    let r = (radius / g.meta.resolution).ceil() as i64;
    let (mut all, mut free) = (0usize, 0usize);
    for dy in -r..=r {
        for dx in -r..=r {
            if dx * dx + dy * dy <= r * r {
                all += 1;
                free += g.cell_checked(cx + dx, cy + dy).is_some_and(|c| is_free(g.get(c), free_thr)) as usize;
            }
        }
    }
    free as f64 / all.max(1) as f64
}

/// true if the unknown cells next to `members` form a closed region smaller than `area` (m^2):
/// a flood fill through unknown cells, stopped as soon as it reaches that size
fn unknown_pocket(g: &Grid, members: &[usize], area: f64) -> bool {
    let w = g.meta.width as usize;
    let limit = (area / (g.meta.resolution * g.meta.resolution)).ceil() as usize;
    let mut seen = std::collections::HashSet::new();
    let mut queue = VecDeque::new();
    for &i in members {
        let (x, y) = ((i % w) as i64, (i / w) as i64);
        for (dx, dy) in [(1, 0), (-1, 0), (0, 1), (0, -1)] {
            if let Some(c) = g.cell_checked(x + dx, y + dy)
                && g.get(c) == UNKNOWN
                && seen.insert(g.index(c))
            {
                queue.push_back(c);
            }
        }
    }
    while let Some(c) = queue.pop_front() {
        if seen.len() >= limit {
            return false;
        }
        let (x, y) = (c.x as i64, c.y as i64);
        for (dx, dy) in [(1, 0), (-1, 0), (0, 1), (0, -1)] {
            match g.cell_checked(x + dx, y + dy) {
                // unknown reaching the map edge is open space, never a pocket
                None => return false,
                Some(n) if g.get(n) == UNKNOWN && seen.insert(g.index(n)) => queue.push_back(n),
                _ => {}
            }
        }
    }
    seen.len() < limit
}

/// unseen area (m^2) within `radius` of the centroid of `members`; cells off the map are not counted
fn unknown_area(g: &Grid, members: &[usize], radius: f64) -> f64 {
    let w = g.meta.width as usize;
    let n = members.len() as f64;
    let (sx, sy) = members.iter().fold((0.0, 0.0), |a, &i| (a.0 + (i % w) as f64, a.1 + (i / w) as f64));
    let (cx, cy) = ((sx / n).round() as i64, (sy / n).round() as i64);
    let r = (radius / g.meta.resolution).ceil() as i64;
    let mut unknown = 0usize;
    for dy in -r..=r {
        for dx in -r..=r {
            if dx * dx + dy * dy <= r * r && g.cell_checked(cx + dx, cy + dy).is_some_and(|c| g.get(c) == UNKNOWN) {
                unknown += 1;
            }
        }
    }
    unknown as f64 * g.meta.resolution * g.meta.resolution
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
        if p.min_unknown_area > 0.0 && unknown_pocket(g, &members, p.min_unknown_area) {
            continue;
        }
        if p.min_free_fraction > 0.0 && free_fraction(g, &members, 0.3, p.free_threshold) < p.min_free_fraction {
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
            let (mut all, mut unknown) = (0usize, 0usize);
            for dy in -radius..=radius {
                for dx in -radius..=radius {
                    if dx * dx + dy * dy <= radius * radius {
                        all += 1;
                        unknown += g.cell_checked(x + dx, y + dy).is_none_or(|c| g.get(c) == UNKNOWN) as usize;
                    }
                }
            }
            unknown as f64 <= p.goal_max_unknown * all as f64
        };
        while goal_index != root && (!clear_of_unknown(goal_index)
            || !is_free(g.data[goal_index], p.free_threshold)) {
            goal_index = parent[goal_index];
        }
        let goal = to_world(goal_index);
        let size_m = n * g.meta.resolution;
        let distance = steps[goal_index] as f64 * g.meta.resolution;
        let gain = if p.gain_weight > 0.0 { unknown_area(g, &members, p.gain_radius) } else { 0.0 };
        out.push(Frontier {
            cells: members.len(),
            size_m,
            centroid,
            goal,
            distance,
            gain,
            score: p.size_weight * size_m.sqrt() - p.distance_weight * distance + p.gain_weight * gain,
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
    fn thin_free_spikes_are_not_frontiers() {
        // 3 x 3 m known room around the robot, open (unknown) to the east through a 1.5 m doorway,
        // plus a 1-cell free line (a lidar beam through a gap) running 2 m north into unknown
        let meta = GridMeta { width: 120, height: 120, resolution: 0.05, origin_x: -1.0, origin_y: -1.0 };
        let mut g = Grid::filled(meta, UNKNOWN);
        for y in 0..60u32 {
            for x in 0..60u32 {
                let wall = x == 0 || y == 0 || y == 59 || (x == 59 && !(15..45).contains(&y));
                g.set(Cell { x, y }, if wall { 100 } else { 0 });
            }
        }
        for y in 59..100u32 {
            g.set(Cell { x: 30, y }, 0);
        }
        let p = ExploreParams { min_frontier_cells: 3, frontier_segment_length: 1.2, ..Default::default() };
        let spike = |f: &Frontier| f.centroid.1 > 2.2;
        let door = |f: &Frontier| f.centroid.0 > 1.8;
        let all = find_frontiers(&g, (0.5, 0.5), &p);
        assert!(all.iter().any(spike) && all.iter().any(door), "both are frontiers without the filter");
        let kept = find_frontiers(&g, (0.5, 0.5), &ExploreParams { min_free_fraction: 0.5, ..p });
        assert!(!kept.iter().any(spike), "the 1-cell spike is filtered");
        assert!(kept.iter().any(door), "the doorway stays");
    }

    #[test]
    fn speckled_start_map_still_gives_goals() {
        // parked robot, one scan: 3 m disc of known free space, about 1 cell in 9 unknown (gaps between
        // beams), unknown beyond. Strict goal placement walks everything back onto the robot.
        let meta = GridMeta { width: 160, height: 160, resolution: 0.05, origin_x: -4.0, origin_y: -4.0 };
        let mut g = Grid::filled(meta, UNKNOWN);
        for y in 0..160u32 {
            for x in 0..160u32 {
                let (wx, wy) = g.cell_to_world(Cell { x, y });
                if wx.hypot(wy) < 1.6 && (x * 7 + y * 13) % 9 != 0 {
                    g.set(Cell { x, y }, 0);
                }
            }
        }
        g.set(g.world_to_cell(0.0, 0.0).unwrap(), 0);
        let p = ExploreParams { goal_standoff: 0.3, min_goal_distance: 0.65, frontier_segment_length: 1.2, ..Default::default() };
        let ok = |p: &ExploreParams| find_frontiers(&g, (0.0, 0.0), p).iter().filter(|f| f.distance >= 0.65).count();
        assert_eq!(ok(&p), 0, "strict: every goal ends on the robot");
        assert!(ok(&ExploreParams { goal_max_unknown: 0.15, ..p }) > 0, "speckle-tolerant: goals away from the robot");
    }

    #[test]
    fn frontiers_onto_small_enclosed_pockets_are_skipped() {
        // 4 x 4 m mapped room, open to unknown on the east side, with a 0.5 x 0.5 m box whose
        // inside is unknown (never seen) in the middle
        let meta = GridMeta { width: 120, height: 80, resolution: 0.05, origin_x: 0.0, origin_y: 0.0 };
        let mut g = Grid::filled(meta, UNKNOWN);
        for y in 0..80 {
            for x in 0..80 {
                g.set(Cell { x, y }, 0);
            }
        }
        for y in 30..40 {
            for x in 30..40 {
                g.set(Cell { x, y }, UNKNOWN);
            }
        }
        let p = ExploreParams { min_frontier_cells: 3, ..Default::default() };
        let near_box = |f: &Frontier| (f.centroid.0 - 1.75).abs() < 0.6 && (f.centroid.1 - 1.75).abs() < 0.6;
        let all = find_frontiers(&g, (0.5, 0.5), &p);
        assert!(all.iter().any(near_box), "without the filter the box edge is a frontier");
        assert!(all.iter().any(|f| f.centroid.0 > 3.5), "and so is the open east side");
        let filtered = find_frontiers(&g, (0.5, 0.5), &ExploreParams { min_unknown_area: 1.0, ..p });
        assert!(!filtered.iter().any(near_box), "0.25 m^2 pocket skipped");
        assert!(filtered.iter().any(|f| f.centroid.0 > 3.5), "open side kept");
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
        let mk = |x: f64, y: f64| Frontier { cells: 10, size_m: 1.0, centroid: (x, y), goal: (x, y), distance: 2.0, gain: 0.0, score: 0.0 };
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
        let ahead = Frontier { cells: 10, size_m: 1.0, centroid: (2.0, 0.0), goal: (1.0, 0.0), distance: 1.0, gain: 0.0, score: 0.0 };
        let behind = Frontier { centroid: (-2.0, 0.0), goal: (-1.0, 0.0), ..ahead.clone() };
        let frontiers = [behind, ahead];
        let chosen = select_goal_with_heading(&frontiers, &[], &ExploreParams::default(), (0.0, 0.0), 0.0, 0.5).unwrap();
        assert_eq!(chosen.goal, (1.0, 0.0));
    }

    /// a 6 m known strip along y = 0 with walls on both sides, frontiers at its two ends. The left
    /// end opens onto unknown space; the right end has one unknown column, then occupied cells.
    fn two_ends_map() -> Grid {
        let meta = GridMeta { width: 120, height: 120, resolution: 0.1, origin_x: -6.0, origin_y: -6.0 };
        let mut g = Grid::filled(meta, UNKNOWN);
        for y in 56..=63 {
            for x in 30..=90 {
                g.set(Cell { x, y }, 0);
            }
        }
        for x in 30..=90 {
            g.set(Cell { x, y: 55 }, 100);
            g.set(Cell { x, y: 64 }, 100);
        }
        // the right half of the strip is walled in above and below, the end column is the only unknown
        for y in (30..55).chain(65..90) {
            for x in 60..=90 {
                g.set(Cell { x, y }, 100);
            }
        }
        for y in 30..90 {
            for x in 92..120 {
                g.set(Cell { x, y }, 100);
            }
        }
        g
    }

    #[test]
    fn unseen_area_decides_between_equal_frontiers() {
        let g = two_ends_map();
        let robot = (0.05, 0.05);
        let p = ExploreParams { min_frontier_cells: 3, ..Default::default() };
        let plain = find_frontiers(&g, robot, &p);
        assert_eq!(plain.len(), 2, "{plain:?}");
        assert!((plain[0].distance - plain[1].distance).abs() < 1e-9, "equal distance: {plain:?}");
        assert_eq!(plain[0].cells, plain[1].cells);
        // gain_weight 0 (default): the score is the size and distance terms only, and the two tie
        for f in &plain {
            assert_eq!(f.gain, 0.0);
            assert!((f.score - (f.size_m.sqrt() - f.distance)).abs() < 1e-9, "{f:?}");
        }
        assert!((plain[0].score - plain[1].score).abs() < 1e-9);
        // gain_weight > 0: the open left end has far more unseen area and wins
        let with_gain = find_frontiers(&g, robot, &ExploreParams { gain_weight: 0.5, ..p });
        assert!(with_gain[0].gain > 5.0 * with_gain[1].gain, "{with_gain:?}");
        assert!(with_gain[0].centroid.0 < 0.0, "the side with more unseen area first: {with_gain:?}");
        assert!(with_gain[0].score > with_gain[1].score);
        assert!((with_gain[0].distance - with_gain[1].distance).abs() < 1e-9);
    }

    #[test]
    fn gain_counts_only_the_unknown_cells_near_the_centroid() {
        let g = two_ends_map();
        let p = ExploreParams { min_frontier_cells: 3, gain_weight: 1.0, ..Default::default() };
        let f = find_frontiers(&g, (0.05, 0.05), &p);
        // the right end's 2 m disc holds one unknown column, 0.1 m wide and about 4 m long
        let right = f.iter().find(|f| f.centroid.0 > 0.0).unwrap();
        assert!(right.gain > 0.2 && right.gain < 0.6, "{right:?}");
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
