use tb4_core::grid::UNKNOWN;
use tb4_core::{Cell, Grid, GridMeta};
use tb4_planning::*;

const RES: f64 = 0.05;

fn empty(w: u32, h: u32) -> Grid {
    Grid::filled(
        GridMeta {
            width: w,
            height: h,
            resolution: RES,
            origin_x: 0.0,
            origin_y: 0.0,
        },
        0,
    )
}

fn rect(g: &mut Grid, x0: u32, y0: u32, x1: u32, y1: u32, v: i8) {
    for y in y0..y1 {
        for x in x0..x1 {
            g.set(Cell { x, y }, v);
        }
    }
}

fn centre(x: u32, y: u32) -> (f64, f64) {
    ((x as f64 + 0.5) * RES, (y as f64 + 0.5) * RES)
}

fn cs(g: &Grid) -> CSpace {
    CSpace::new(g, &CSpaceParams::default())
}

fn assert_segments_free(cs: &CSpace, path: &Path) {
    for w in path.points.windows(2) {
        assert!(
            segment_free(cs, w[0], w[1]),
            "segment {:?} -> {:?} hits lethal cell",
            w[0],
            w[1]
        );
    }
    assert!(path_clearance_min(cs, &path.points) >= cs.params.robot_radius - 1e-6);
}

struct Rng(u64);

impl Rng {
    fn next(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E3779B97F4A7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58476D1CE4E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D049BB133111EB);
        z ^ (z >> 31)
    }

    fn below(&mut self, n: u32) -> u32 {
        (self.next() % n as u64) as u32
    }
}

#[test]
fn open_space_is_straight() {
    let g = empty(100, 100);
    let c = cs(&g);
    let p = PlannerParams::default();
    let (a, b) = (centre(10, 10), centre(80, 60));
    let path = lazy_theta_star(&c, a, b, &p).unwrap();
    assert_eq!(path.points, vec![a, b]);
    let euclid = (b.0 - a.0).hypot(b.1 - a.1);
    assert!((path.length - euclid).abs() < 1e-9);
    assert!((path.cost - euclid).abs() < 1e-6);
    let astar = a_star(&c, a, b, &p).unwrap();
    assert!(path.length <= astar.length + 1e-9);
    assert!(astar.length > euclid * 1.03);
}

#[test]
fn wall_with_gap() {
    let mut g = empty(100, 100);
    rect(&mut g, 48, 0, 52, 100, 100);
    rect(&mut g, 48, 70, 52, 85, 0);
    let c = cs(&g);
    let p = PlannerParams::default();
    let (a, b) = (centre(20, 20), centre(80, 20));
    let theta = lazy_theta_star(&c, a, b, &p).unwrap();
    let astar = a_star(&c, a, b, &p).unwrap();
    assert_segments_free(&c, &theta);
    assert_segments_free(&c, &astar);
    assert!(
        theta.points.len() >= 3 && theta.points.len() <= 10,
        "{:?}",
        theta.points
    );
    let crossing = theta.points.windows(2).any(|w| {
        let (lo, hi) = (w[0].0.min(w[1].0), w[0].0.max(w[1].0));
        lo < 2.5 && hi > 2.5
    });
    assert!(crossing);
    assert!(
        theta.cost <= astar.cost + 1e-6,
        "{} vs {}",
        theta.cost,
        astar.cost
    );
    assert!(
        theta.length <= astar.length + 1e-9,
        "{} vs {}",
        theta.length,
        astar.length
    );
    assert!(theta.expansions > 0 && astar.expansions > 0);
}

fn corridor(free_cells: u32) -> Grid {
    let mut g = empty(120, 60);
    rect(&mut g, 0, 0, 120, 60, 100);
    rect(&mut g, 0, 0, 30, 60, 0);
    rect(&mut g, 90, 0, 120, 60, 0);
    rect(&mut g, 30, 30, 90, 30 + free_cells, 0);
    g
}

#[test]
fn narrow_corridor() {
    let p = PlannerParams::default();
    let (a, b) = (centre(10, 33), centre(110, 33));
    // 7 free cells: centre cell is 4 cells (0.20 m) from both walls >= 0.18
    let c = cs(&corridor(7));
    let path = lazy_theta_star(&c, a, b, &p).unwrap();
    assert_segments_free(&c, &path);
    assert!(a_star(&c, a, b, &p).is_ok());
    // 6 free cells: best is 3 cells (0.15 m) < 0.18
    let c = cs(&corridor(6));
    assert_eq!(lazy_theta_star(&c, a, b, &p), Err(PlanError::NoPath));
    assert_eq!(a_star(&c, a, b, &p), Err(PlanError::NoPath));
}

#[test]
fn blocked_and_out_of_map() {
    let mut g = empty(80, 80);
    rect(&mut g, 40, 0, 42, 80, 100);
    let c = cs(&g);
    let p = PlannerParams::default();
    assert_eq!(
        lazy_theta_star(&c, centre(10, 10), centre(70, 10), &p),
        Err(PlanError::NoPath)
    );
    assert_eq!(
        a_star(&c, centre(10, 10), centre(70, 10), &p),
        Err(PlanError::NoPath)
    );
    assert_eq!(
        lazy_theta_star(&c, (-1.0, 0.5), centre(10, 10), &p),
        Err(PlanError::OutOfMap)
    );
    assert_eq!(
        lazy_theta_star(&c, centre(10, 10), (0.5, 9.0), &p),
        Err(PlanError::OutOfMap)
    );
    let tight = PlannerParams {
        max_expansions: 10,
        ..p
    };
    assert_eq!(
        lazy_theta_star(&c, centre(10, 10), centre(30, 70), &tight),
        Err(PlanError::ExpansionLimit)
    );
}

#[test]
fn start_in_inflation_escapes() {
    let mut g = empty(100, 60);
    rect(&mut g, 0, 0, 100, 2, 100);
    let c = cs(&g);
    let start = centre(20, 3); // 0.05 m from the wall
    assert!(!c.is_free(c.world_to_cell(start.0, start.1).unwrap()));
    let p = PlannerParams::default();
    let path = lazy_theta_star(&c, start, centre(80, 30), &p).unwrap();
    assert_eq!(path.points[0], start);
    let rest = Path {
        points: path.points[1..].to_vec(),
        ..path.clone()
    };
    assert_segments_free(&c, &rest);
    let strict = PlannerParams {
        start_tolerance: 0.05,
        ..p
    };
    assert_eq!(
        lazy_theta_star(&c, start, centre(80, 30), &strict),
        Err(PlanError::StartBlocked)
    );
}

#[test]
fn goal_snapping() {
    let mut g = empty(100, 100);
    rect(&mut g, 60, 40, 70, 60, 100);
    let c = cs(&g);
    let p = PlannerParams::default();
    let start = centre(10, 50);
    // just inside the inflation zone, 0.15 m from obstacle edge
    let goal = centre(57, 50);
    let path = lazy_theta_star(&c, start, goal, &p).unwrap();
    let end = *path.points.last().unwrap();
    assert!((end.0 - goal.0).hypot(end.1 - goal.1) <= p.goal_tolerance + 1e-9);
    assert!(c.is_free(c.world_to_cell(end.0, end.1).unwrap()));
    assert_segments_free(&c, &path);
    // deep inside the obstacle
    assert_eq!(
        lazy_theta_star(&c, start, centre(65, 50), &p),
        Err(PlanError::GoalBlocked)
    );
    assert_eq!(
        a_star(&c, start, centre(65, 50), &p),
        Err(PlanError::GoalBlocked)
    );
}

#[test]
fn unknown_handling() {
    let mut g = empty(100, 100);
    rect(&mut g, 0, 0, 100, 100, UNKNOWN);
    rect(&mut g, 0, 0, 30, 100, 0);
    rect(&mut g, 70, 0, 100, 100, 0);
    rect(&mut g, 0, 0, 100, 15, 0);
    let c = cs(&g);
    let (a, b) = (centre(10, 80), centre(90, 80));
    let p = PlannerParams::default();
    let through = lazy_theta_star(&c, a, b, &p).unwrap();
    assert_eq!(through.points.len(), 2);
    assert!((through.cost - through.length * 1.0 - 40.0 * RES * p.unknown_cost).abs() < 1e-6);

    let known = PlannerParams {
        allow_unknown: false,
        ..p
    };
    let around = lazy_theta_star(&c, a, b, &known).unwrap();
    for q in densify(&around.points, 0.02) {
        assert!(
            !c.is_unknown(c.world_to_cell(q.0, q.1).unwrap()),
            "{q:?} {:?}",
            around.points
        );
    }
    assert!(around.length > through.length + 2.0);
    assert_eq!(
        lazy_theta_star(&c, a, centre(50, 50), &known),
        Err(PlanError::GoalBlocked)
    );
    // expensive unknown makes the detour preferable
    let pricey = PlannerParams {
        unknown_cost: 10.0,
        ..p
    };
    let detour = lazy_theta_star(&c, a, b, &pricey).unwrap();
    assert!(detour.length > through.length + 2.0);
}

fn random_map(rng: &mut Rng, w: u32, h: u32, n: u32) -> Grid {
    let mut g = empty(w, h);
    for _ in 0..n {
        let (x, y) = (rng.below(w), rng.below(h));
        let (rw, rh) = (1 + rng.below(w / 5), 1 + rng.below(h / 5));
        let v = if rng.below(6) == 0 { UNKNOWN } else { 100 };
        rect(&mut g, x, y, (x + rw).min(w), (y + rh).min(h), v);
    }
    g
}

fn random_free(rng: &mut Rng, c: &CSpace) -> (f64, f64) {
    loop {
        let cell = Cell {
            x: rng.below(c.meta.width),
            y: rng.below(c.meta.height),
        };
        if c.is_free(cell) && !c.is_unknown(cell) {
            return centre(cell.x, cell.y);
        }
    }
}

#[test]
fn random_maps_property() {
    let mut rng = Rng(7);
    let mut planner = Planner::new();
    let p = PlannerParams::default();
    let mut solved = 0;
    for _ in 0..60 {
        let g = random_map(&mut rng, 90, 70, 14);
        let c = cs(&g);
        for _ in 0..4 {
            let (a, b) = (random_free(&mut rng, &c), random_free(&mut rng, &c));
            let theta = planner.lazy_theta_star(&c, a, b, &p);
            let astar = planner.a_star(&c, a, b, &p);
            assert_eq!(theta.is_ok(), astar.is_ok(), "{a:?} {b:?}");
            if let (Ok(t), Ok(s)) = (theta, astar) {
                solved += 1;
                assert_segments_free(&c, &t);
                assert_segments_free(&c, &s);
                assert_eq!(t.points[0], a);
                assert_eq!(*t.points.last().unwrap(), b);
                assert!(t.cost <= s.cost * 1.05, "theta {} astar {}", t.cost, s.cost);
                let euclid = (b.0 - a.0).hypot(b.1 - a.1);
                assert!(t.length >= euclid - 1e-9);
                let sm = smooth(&c, &t.points, 3, 0.3);
                assert_eq!(sm[0], a);
                assert!(path_clearance_min(&c, &sm) >= c.params.robot_radius - 1e-6);
            }
        }
    }
    assert!(solved > 90, "only {solved} solvable queries");
}

#[test]
fn geometric_length_without_costs() {
    let mut rng = Rng(11);
    let flat = CSpaceParams {
        cost_weight: 0.0,
        ..CSpaceParams::default()
    };
    let p = PlannerParams::default();
    for _ in 0..20 {
        let g = random_map(&mut rng, 80, 80, 10);
        let c = CSpace::new(&g, &flat);
        let (a, b) = (random_free(&mut rng, &c), random_free(&mut rng, &c));
        if let (Ok(t), Ok(s)) = (lazy_theta_star(&c, a, b, &p), a_star(&c, a, b, &p)) {
            assert!(
                t.length <= s.length + 1e-6,
                "theta {} astar {}",
                t.length,
                s.length
            );
        }
    }
}

#[test]
fn densify_and_clearance() {
    let pts = densify(&[(0.0, 0.0), (1.0, 0.0), (1.0, 0.25)], 0.1);
    assert_eq!(pts.len(), 1 + 10 + 3);
    assert_eq!(*pts.last().unwrap(), (1.0, 0.25));
    assert!(
        pts.windows(2)
            .all(|w| (w[1].0 - w[0].0).hypot(w[1].1 - w[0].1) <= 0.1 + 1e-12)
    );

    let mut g = empty(40, 40);
    rect(&mut g, 0, 0, 40, 1, 100);
    let c = cs(&g);
    let cl = path_clearance_min(&c, &[centre(5, 10), centre(30, 6)]);
    assert!((cl - 6.0 * RES).abs() < 1e-6, "{cl}");
    assert!((c.clearance_at(centre(5, 10).0, centre(5, 10).1) - 10.0 * RES).abs() < 1e-6);
}

#[test]
fn planner_reuse_matches_fresh() {
    let mut g = empty(100, 100);
    rect(&mut g, 48, 0, 52, 80, 100);
    let c = cs(&g);
    let p = PlannerParams::default();
    let mut planner = Planner::new();
    let (a, b) = (centre(20, 20), centre(80, 20));
    let first = planner.lazy_theta_star(&c, a, b, &p).unwrap();
    for _ in 0..3 {
        let _ = planner.a_star(&c, b, a, &p).unwrap();
        assert_eq!(planner.lazy_theta_star(&c, a, b, &p).unwrap(), first);
    }
    assert_eq!(lazy_theta_star(&c, a, b, &p).unwrap(), first);
}

#[test]
fn goal_snaps_to_reachable_side() {
    let mut g = empty(100, 60);
    rect(&mut g, 50, 0, 52, 60, 100);
    let c = cs(&g);
    let p = PlannerParams::default();
    let (start, goal) = (centre(10, 30), centre(51, 30));
    let a = c.world_to_cell(start.0, start.1).unwrap();
    assert!(!c.connected(a, Cell { x: 56, y: 30 }, true));
    // nearest free cell is on the far side (x=55), the reachable one is x=46
    let path = lazy_theta_star(&c, start, goal, &p).unwrap();
    assert_eq!(*path.points.last().unwrap(), centre(46, 30));
    let far = PlannerParams {
        goal_tolerance: 0.22,
        ..p
    };
    assert_eq!(
        lazy_theta_star(&c, start, goal, &far),
        Err(PlanError::NoPath)
    );
    assert_eq!(
        lazy_theta_star(&c, start, centre(80, 30), &p),
        Err(PlanError::NoPath)
    );
}
