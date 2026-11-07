use std::cmp::Ordering;
use std::collections::BinaryHeap;
use std::f64::consts::SQRT_2;

use tb4_core::Cell;

use crate::cspace::CSpace;
use crate::walk::walk;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PlannerParams {
    pub allow_unknown: bool,
    /// added to the cost multiplier of unknown cells
    pub unknown_cost: f64,
    pub max_expansions: usize,
    pub goal_tolerance: f64,
    pub start_tolerance: f64,
    pub heuristic_weight: f64,
}

impl Default for PlannerParams {
    fn default() -> Self {
        Self {
            allow_unknown: true,
            unknown_cost: 1.0,
            max_expansions: 1_000_000,
            goal_tolerance: 0.3,
            start_tolerance: 0.4,
            heuristic_weight: 1.0,
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct Path {
    pub points: Vec<(f64, f64)>,
    pub length: f64,
    pub expansions: usize,
    /// cost-weighted length, metres
    pub cost: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PlanError {
    OutOfMap,
    StartBlocked,
    GoalBlocked,
    NoPath,
    ExpansionLimit,
}

impl std::fmt::Display for PlanError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        std::fmt::Debug::fmt(self, f)
    }
}

impl std::error::Error for PlanError {}

const NONE: u32 = u32::MAX;
const DIRS: [(i32, i32); 8] = [
    (1, 0),
    (-1, 0),
    (0, 1),
    (0, -1),
    (1, 1),
    (1, -1),
    (-1, 1),
    (-1, -1),
];

#[derive(Clone, Copy)]
struct Node {
    f: f64,
    g: f64,
    idx: u32,
}

impl PartialEq for Node {
    fn eq(&self, o: &Self) -> bool {
        self.cmp(o) == Ordering::Equal
    }
}
impl Eq for Node {}
impl PartialOrd for Node {
    fn partial_cmp(&self, o: &Self) -> Option<Ordering> {
        Some(self.cmp(o))
    }
}
impl Ord for Node {
    // min-heap on f, ties prefer larger g
    fn cmp(&self, o: &Self) -> Ordering {
        o.f.total_cmp(&self.f).then(self.g.total_cmp(&o.g))
    }
}

#[derive(Clone, Copy, PartialEq)]
enum Algo {
    AStar,
    LazyTheta,
}

/// Reusable search buffers; a generation counter avoids clearing them between plans.
#[derive(Debug, Default)]
pub struct Planner {
    w: usize,
    h: usize,
    g: Vec<f64>,
    parent: Vec<u32>,
    seen: Vec<u32>,
    closed: Vec<u32>,
    generation: u32,
    open: Vec<Node>,
}

impl std::fmt::Debug for Node {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "Node({}, {}, {})", self.idx, self.f, self.g)
    }
}

struct Search<'a> {
    cs: &'a CSpace,
    p: &'a PlannerParams,
    res: f64,
    unknown_add: f32,
}

impl Search<'_> {
    #[inline]
    fn cost_at(&self, i: usize) -> f32 {
        let c = self.cs.cost[i];
        if self.cs.unknown[i] {
            c + self.unknown_add
        } else {
            c
        }
    }

    #[inline]
    fn cost_xy(&self, x: i64, y: i64) -> f32 {
        if x < 0 || y < 0 || x >= self.cs.width() as i64 || y >= self.cs.height() as i64 {
            return f32::INFINITY;
        }
        self.cost_at(y as usize * self.cs.width() + x as usize)
    }

    // cost-weighted length of the straight segment between two cell centres, infinite if blocked
    fn los_cost(&self, a: usize, b: usize) -> f64 {
        let w = self.cs.width();
        let ca = ((a % w) as f64 + 0.5, (a / w) as f64 + 0.5);
        let cb = ((b % w) as f64 + 0.5, (b / w) as f64 + 0.5);
        let mut acc = 0.0f64;
        let ok = walk(ca, cb, |x, y, l| {
            let c = self.cost_xy(x, y);
            if c.is_finite() {
                acc += c as f64 * l;
                true
            } else {
                false
            }
        });
        if ok { acc * self.res } else { f64::INFINITY }
    }

    #[inline]
    fn dist(&self, a: usize, b: usize) -> f64 {
        let w = self.cs.width();
        let dx = (a % w) as f64 - (b % w) as f64;
        let dy = (a / w) as f64 - (b / w) as f64;
        dx.hypot(dy) * self.res
    }

    // 8-connected edges with no corner cutting; calls f(neighbour, edge_cost)
    #[inline]
    fn neighbours(&self, i: usize, mut f: impl FnMut(usize, f64)) {
        let w = self.cs.width() as i64;
        let (x, y) = ((i as i64) % w, (i as i64) / w);
        let ci = self.cost_at(i) as f64;
        for (dx, dy) in DIRS {
            let (nx, ny) = (x + dx as i64, y + dy as i64);
            let cn = self.cost_xy(nx, ny);
            if !cn.is_finite() {
                continue;
            }
            let step = if dx != 0 && dy != 0 {
                if !self.cost_xy(nx, y).is_finite() || !self.cost_xy(x, ny).is_finite() {
                    continue;
                }
                SQRT_2
            } else {
                1.0
            };
            f(
                (ny * w + nx) as usize,
                step * self.res * 0.5 * (ci + cn as f64),
            );
        }
    }
}

impl Planner {
    pub fn new() -> Self {
        Self::default()
    }

    fn prepare(&mut self, w: usize, h: usize) {
        let n = w * h;
        if self.w != w || self.h != h || self.g.len() != n || self.generation == u32::MAX {
            self.w = w;
            self.h = h;
            self.g = vec![f64::INFINITY; n];
            self.parent = vec![NONE; n];
            self.seen = vec![0; n];
            self.closed = vec![0; n];
            self.generation = 0;
        }
        self.generation += 1;
        self.open.clear();
    }

    pub fn lazy_theta_star(
        &mut self,
        cs: &CSpace,
        start: (f64, f64),
        goal: (f64, f64),
        p: &PlannerParams,
    ) -> Result<Path, PlanError> {
        self.plan(cs, start, goal, p, Algo::LazyTheta)
    }

    pub fn a_star(
        &mut self,
        cs: &CSpace,
        start: (f64, f64),
        goal: (f64, f64),
        p: &PlannerParams,
    ) -> Result<Path, PlanError> {
        self.plan(cs, start, goal, p, Algo::AStar)
    }

    fn plan(
        &mut self,
        cs: &CSpace,
        start: (f64, f64),
        goal: (f64, f64),
        p: &PlannerParams,
        algo: Algo,
    ) -> Result<Path, PlanError> {
        let sc = cs
            .world_to_cell(start.0, start.1)
            .ok_or(PlanError::OutOfMap)?;
        let gc = cs
            .world_to_cell(goal.0, goal.1)
            .ok_or(PlanError::OutOfMap)?;
        let s = Search {
            cs,
            p,
            res: cs.meta.resolution,
            unknown_add: if p.allow_unknown {
                p.unknown_cost as f32
            } else {
                f32::INFINITY
            },
        };
        let comp = cs.components(p.allow_unknown);
        let s_snap =
            nearest_free(&s, sc, p.start_tolerance, true, None).ok_or(PlanError::StartBlocked)?;
        let label = comp[cs.index(s_snap)];
        let g_snap = match nearest_free(&s, gc, p.goal_tolerance, false, Some((comp, label))) {
            Some(c) => c,
            None if nearest_free(&s, gc, p.goal_tolerance, false, None).is_some() => {
                return Err(PlanError::NoPath);
            }
            None => return Err(PlanError::GoalBlocked),
        };
        let (si, gi) = (cs.index(s_snap), cs.index(g_snap));

        self.prepare(cs.width(), cs.height());
        let expansions = self.search(&s, si, gi, algo)?;

        let mut cells = vec![gi];
        let mut c = gi;
        while c != si {
            c = self.parent[c] as usize;
            cells.push(c);
        }
        cells.reverse();
        let mut cost = self.g[gi];
        match algo {
            Algo::AStar => cells = drop_collinear(&cells, cs.width()),
            Algo::LazyTheta => (cells, cost) = shortcut(&s, &cells),
        }
        let mut points: Vec<(f64, f64)> = cells
            .iter()
            .map(|&i| cs.cell_to_world(cell_of(i, cs.width())))
            .collect();
        if s_snap != sc {
            let first = points[0];
            cost += dist2(start, first) * (1.0 + cs.params.cost_weight);
            points.insert(0, start);
        } else {
            points[0] = start;
        }
        if g_snap == gc {
            *points.last_mut().unwrap() = goal;
        }
        if points.len() == 1 {
            points.push(points[0]);
        }
        Ok(Path {
            length: crate::path::path_length(&points),
            points,
            expansions,
            cost,
        })
    }

    fn search(&mut self, s: &Search, si: usize, gi: usize, algo: Algo) -> Result<usize, PlanError> {
        let gen_ = self.generation;
        let hw = s.p.heuristic_weight;
        self.seen[si] = gen_;
        self.g[si] = 0.0;
        self.parent[si] = si as u32;
        let mut heap = BinaryHeap::from(std::mem::take(&mut self.open));
        heap.push(Node {
            f: hw * s.dist(si, gi),
            g: 0.0,
            idx: si as u32,
        });
        let mut expansions = 0usize;
        let result = loop {
            let Some(node) = heap.pop() else {
                break Err(PlanError::NoPath);
            };
            let cur = node.idx as usize;
            if self.closed[cur] == gen_ || node.g > self.g[cur] {
                continue;
            }
            expansions += 1;
            if expansions > s.p.max_expansions {
                break Err(PlanError::ExpansionLimit);
            }
            if algo == Algo::LazyTheta {
                self.set_vertex(s, cur);
            }
            if cur == gi {
                break Ok(expansions);
            }
            self.closed[cur] = gen_;
            let g_cur = self.g[cur];
            let pp = self.parent[cur] as usize;
            s.neighbours(cur, |n, edge| {
                if self.closed[n] == gen_ {
                    return;
                }
                if self.seen[n] != gen_ {
                    self.seen[n] = gen_;
                    self.g[n] = f64::INFINITY;
                    self.parent[n] = NONE;
                }
                let (mut best, mut bp) = (g_cur + edge, cur);
                if algo == Algo::LazyTheta && pp != cur {
                    // optimistic: straight from grandparent, priced by the endpoint costs; verified in set_vertex
                    let est =
                        self.g[pp] + s.dist(pp, n) * 0.5 * (s.cost_at(pp) + s.cost_at(n)) as f64;
                    if est < best {
                        best = est;
                        bp = pp;
                    }
                }
                if best < self.g[n] {
                    self.g[n] = best;
                    self.parent[n] = bp as u32;
                    heap.push(Node {
                        f: best + hw * s.dist(n, gi),
                        g: best,
                        idx: n as u32,
                    });
                }
            });
        };
        heap.clear();
        self.open = heap.into_vec();
        result
    }

    fn set_vertex(&mut self, s: &Search, cur: usize) {
        let pp = self.parent[cur] as usize;
        if pp == cur {
            return;
        }
        let gen_ = self.generation;
        let mut best = self.g[pp] + s.los_cost(pp, cur);
        let mut bp = pp;
        s.neighbours(cur, |n, edge| {
            if self.closed[n] == gen_ {
                let c = self.g[n] + edge;
                if c < best {
                    best = c;
                    bp = n;
                }
            }
        });
        self.g[cur] = best;
        self.parent[cur] = bp as u32;
    }
}

pub fn lazy_theta_star(
    cs: &CSpace,
    start: (f64, f64),
    goal: (f64, f64),
    p: &PlannerParams,
) -> Result<Path, PlanError> {
    Planner::new().lazy_theta_star(cs, start, goal, p)
}

pub fn a_star(
    cs: &CSpace,
    start: (f64, f64),
    goal: (f64, f64),
    p: &PlannerParams,
) -> Result<Path, PlanError> {
    Planner::new().a_star(cs, start, goal, p)
}

fn cell_of(i: usize, w: usize) -> Cell {
    Cell {
        x: (i % w) as u32,
        y: (i / w) as u32,
    }
}

fn dist2(a: (f64, f64), b: (f64, f64)) -> f64 {
    (a.0 - b.0).hypot(a.1 - b.1)
}

// greedy cost-aware vertex removal: the lazy cost estimate can leave redundant vertices
fn shortcut(s: &Search, cells: &[usize]) -> (Vec<usize>, f64) {
    let seg: Vec<f64> = cells.windows(2).map(|w| s.los_cost(w[0], w[1])).collect();
    let mut out = vec![cells[0]];
    let mut total = 0.0;
    let mut i = 0;
    while i + 1 < cells.len() {
        let mut best_j = i + 1;
        let mut best_c = seg[i];
        let mut chain = seg[i];
        for j in i + 2..cells.len() {
            chain += seg[j - 1];
            let c = s.los_cost(cells[i], cells[j]);
            if c.is_finite() && c <= chain * (1.0 + 1e-9) {
                best_j = j;
                best_c = c;
            }
        }
        total += best_c;
        out.push(cells[best_j]);
        i = best_j;
    }
    (out, total)
}

fn drop_collinear(cells: &[usize], w: usize) -> Vec<usize> {
    let dir = |a: usize, b: usize| {
        (
            (b % w) as i64 - (a % w) as i64,
            (b / w) as i64 - (a / w) as i64,
        )
    };
    let mut out = vec![cells[0]];
    for k in 1..cells.len().saturating_sub(1) {
        if dir(cells[k - 1], cells[k]) != dir(cells[k], cells[k + 1]) {
            out.push(cells[k]);
        }
    }
    if cells.len() > 1 {
        out.push(*cells.last().unwrap());
    }
    out
}

// Nearest traversable cell within tol, optionally restricted to one connected component.
// With through_free_space the search does not cross obstacle cells (start escaping the inflation zone).
fn nearest_free(
    s: &Search,
    c: Cell,
    tol: f64,
    through_free_space: bool,
    comp: Option<(&[u32], u32)>,
) -> Option<Cell> {
    let cs = s.cs;
    let ok = |i: usize| s.cost_at(i).is_finite() && comp.is_none_or(|(cm, l)| cm[i] == l);
    if ok(cs.index(c)) {
        return Some(c);
    }
    let r = (tol / s.res).floor() as i64;
    if r <= 0 {
        return None;
    }
    let side = (2 * r + 1) as usize;
    let (cx, cy) = (c.x as i64, c.y as i64);
    let r2 = (tol / s.res) * (tol / s.res);
    let in_range = |x: i64, y: i64| {
        let (dx, dy) = ((x - cx) as f64, (y - cy) as f64);
        dx * dx + dy * dy <= r2
            && cs.meta.width as i64 > x
            && x >= 0
            && cs.meta.height as i64 > y
            && y >= 0
    };
    let mut best: Option<(i64, Cell)> = None;
    let mut consider = |x: i64, y: i64| {
        let i = y as usize * cs.width() + x as usize;
        if ok(i) {
            let d = (x - cx).pow(2) + (y - cy).pow(2);
            if best.is_none_or(|(bd, _)| d < bd) {
                best = Some((
                    d,
                    Cell {
                        x: x as u32,
                        y: y as u32,
                    },
                ));
            }
        }
    };
    if through_free_space {
        let mut visited = vec![false; side * side];
        let local = |x: i64, y: i64| (y - cy + r) as usize * side + (x - cx + r) as usize;
        let mut queue = std::collections::VecDeque::from([(cx, cy)]);
        visited[local(cx, cy)] = true;
        while let Some((x, y)) = queue.pop_front() {
            consider(x, y);
            for (dx, dy) in DIRS {
                let (nx, ny) = (x + dx as i64, y + dy as i64);
                if !in_range(nx, ny) || visited[local(nx, ny)] {
                    continue;
                }
                visited[local(nx, ny)] = true;
                if cs.dist.dist[ny as usize * cs.width() + nx as usize] > 0.0 {
                    queue.push_back((nx, ny));
                }
            }
        }
    } else {
        for y in cy - r..=cy + r {
            for x in cx - r..=cx + r {
                if in_range(x, y) {
                    consider(x, y);
                }
            }
        }
    }
    best.map(|b| b.1)
}
