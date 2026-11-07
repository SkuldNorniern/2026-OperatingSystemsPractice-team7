use std::time::Instant;

use tb4_core::grid::UNKNOWN;
use tb4_core::{Cell, Grid, GridMeta};
use tb4_planning::*;

const N: u32 = 400;
const RES: f64 = 0.05;

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

fn rect(g: &mut Grid, x0: u32, y0: u32, x1: u32, y1: u32, v: i8) {
    for y in y0..y1.min(N) {
        for x in x0..x1.min(N) {
            g.set(Cell { x, y }, v);
        }
    }
}

// rooms from a 4x4 wall lattice with doorways, furniture rectangles, and unknown patches
fn office(seed: u64) -> Grid {
    let mut rng = Rng(seed);
    let meta = GridMeta {
        width: N,
        height: N,
        resolution: RES,
        origin_x: 0.0,
        origin_y: 0.0,
    };
    let mut g = Grid::filled(meta, 0);
    rect(&mut g, 0, 0, N, 2, 100);
    rect(&mut g, 0, N - 2, N, N, 100);
    rect(&mut g, 0, 0, 2, N, 100);
    rect(&mut g, N - 2, 0, N, N, 100);
    let step = N / 4;
    for k in 1..4 {
        let w = k * step;
        rect(&mut g, w - 1, 0, w + 1, N, 100);
        rect(&mut g, 0, w - 1, N, w + 1, 100);
    }
    for k in 1..4 {
        let w = k * step;
        for r in 0..4 {
            let lo = r * step;
            let d = lo + 10 + rng.below(step - 40);
            rect(&mut g, w - 1, d, w + 1, d + 18, 0);
            let d = lo + 10 + rng.below(step - 40);
            rect(&mut g, d, w - 1, d + 18, w + 1, 0);
        }
    }
    for _ in 0..60 {
        let (x, y) = (2 + rng.below(N - 30), 2 + rng.below(N - 30));
        let (w, h) = (3 + rng.below(18), 3 + rng.below(18));
        rect(&mut g, x, y, x + w, y + h, 100);
    }
    for _ in 0..12 {
        let (x, y) = (rng.below(N - 60), rng.below(N - 60));
        let (w, h) = (10 + rng.below(50), 10 + rng.below(50));
        for yy in y..y + h {
            for xx in x..x + w {
                let c = Cell { x: xx, y: yy };
                if g.get(c) == 0 {
                    g.set(c, UNKNOWN);
                }
            }
        }
    }
    g
}

fn main() {
    let cp = CSpaceParams::default();
    let pp = PlannerParams::default();
    let mut planner = Planner::new();
    let mut rng = Rng(42);
    let mut tot = [(0.0f64, 0usize, 0.0f64, 0.0f64); 2];
    let mut solved = 0;
    let g = office(1);
    let t = Instant::now();
    std::hint::black_box(CSpace::new(&g, &cp));
    println!(
        "cspace build {:.2} ms ({}x{})",
        t.elapsed().as_secs_f64() * 1e3,
        N,
        N
    );
    for map in 0..5 {
        let g = office(map + 1);
        let cs = CSpace::new(&g, &cp);
        for _ in 0..6 {
            let pick = |rng: &mut Rng| loop {
                let c = Cell {
                    x: rng.below(N),
                    y: rng.below(N),
                };
                if cs.is_free(c) && !cs.is_unknown(c) {
                    return cs.cell_to_world(c);
                }
            };
            let (a, b) = (pick(&mut rng), pick(&mut rng));
            if (a.0 - b.0).hypot(a.1 - b.1) < 8.0 {
                continue;
            }
            let t0 = Instant::now();
            let s = planner.a_star(&cs, a, b, &pp);
            let t1 = Instant::now();
            let th = planner.lazy_theta_star(&cs, a, b, &pp);
            let t2 = Instant::now();
            let (Ok(s), Ok(th)) = (s, th) else {
                println!("map {map}: {a:.2?} -> {b:.2?} no path");
                continue;
            };
            solved += 1;
            let (ms_a, ms_t) = ((t1 - t0).as_secs_f64() * 1e3, (t2 - t1).as_secs_f64() * 1e3);
            println!(
                "map {map}: euclid {:5.2} m | A* {:6.2} ms {:6} exp len {:5.2} cost {:5.2} pts {:3} | LazyTheta* {:6.2} ms {:6} exp len {:5.2} cost {:5.2} pts {:2} clear {:.3}",
                (a.0 - b.0).hypot(a.1 - b.1),
                ms_a,
                s.expansions,
                s.length,
                s.cost,
                s.points.len(),
                ms_t,
                th.expansions,
                th.length,
                th.cost,
                th.points.len(),
                path_clearance_min(&cs, &th.points),
            );
            for (k, (ms, p)) in [(ms_a, &s), (ms_t, &th)].into_iter().enumerate() {
                tot[k].0 += ms;
                tot[k].1 += p.expansions;
                tot[k].2 += p.length;
                tot[k].3 += p.cost;
            }
        }
    }
    let n = solved.max(1) as f64;
    for (name, t) in ["A*", "LazyTheta*"].iter().zip(tot) {
        println!(
            "{name:>10}: mean {:.2} ms, {:.0} expansions, length {:.2} m, cost {:.2} over {solved} queries",
            t.0 / n,
            t.1 as f64 / n,
            t.2 / n,
            t.3 / n
        );
    }

    // unreachable goal: the whole free space gets expanded before NoPath
    let mut g = office(1);
    rect(&mut g, 300, 300, 360, 302, 100);
    rect(&mut g, 300, 358, 360, 360, 100);
    rect(&mut g, 300, 300, 302, 360, 100);
    rect(&mut g, 358, 300, 360, 360, 100);
    rect(&mut g, 302, 302, 358, 358, 0);
    let cs = CSpace::new(&g, &cp);
    let (a, b) = (
        cs.cell_to_world(Cell { x: 20, y: 20 }),
        cs.cell_to_world(Cell { x: 330, y: 330 }),
    );
    for (name, astar) in [("A*", true), ("LazyTheta*", false)] {
        let t = Instant::now();
        let r = if astar {
            planner.a_star(&cs, a, b, &pp)
        } else {
            planner.lazy_theta_star(&cs, a, b, &pp)
        };
        println!(
            "{name:>10}: unreachable goal -> {:?} in {:.2} ms",
            r.err(),
            t.elapsed().as_secs_f64() * 1e3
        );
    }
}
