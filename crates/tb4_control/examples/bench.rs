use tb4_control::{Mppi, MppiParams, RobotState, simulate};
use tb4_core::{Cell, DistanceField, Grid, GridMeta, Pose2};

fn world() -> DistanceField {
    let meta = GridMeta { width: 160, height: 120, resolution: 0.05, origin_x: -1.0, origin_y: -3.0 };
    let mut g = Grid::filled(meta, 0);
    for cy in 0..120 {
        for cx in 0..160 {
            let wall = cx == 0 || cy == 0 || cx == 159 || cy == 119;
            let pillar = (48..56).contains(&cx) && (54..62).contains(&cy);
            if wall || pillar {
                g.set(Cell { x: cx, y: cy }, 100);
            }
        }
    }
    DistanceField::from_grid(&g, 65, false)
}

fn main() {
    let df = world();
    let path = [(0.0, 0.0), (3.0, 0.0), (3.0, 2.0), (5.5, 2.0)];
    let steps = 300;
    println!("horizon 40, {steps} closed-loop calls per row");
    println!("{:>7} {:>7} {:>9} {:>9}", "samples", "threads", "mean_ms", "max_ms");
    for samples in [500, 1000, 2000] {
        for threads in [1, 2, 4] {
            let p = MppiParams { samples, threads, horizon: 40, ..MppiParams::default() };
            let mut m = Mppi::new(p.clone());
            m.set_path(&path, None);
            let mut s = RobotState { pose: Pose2::new(0.0, 0.0, 0.0), v: 0.0, w: 0.0 };
            for _ in 0..steps {
                let c = m.compute(&s, Some(&df));
                if c.goal_reached {
                    m.set_path(&path, None);
                    s = RobotState::default();
                    continue;
                }
                s = simulate(&s, c.control(), p.dt);
            }
            let t = m.timing();
            println!("{samples:>7} {threads:>7} {:>9.3} {:>9.3}", t.mean_ms, t.max_ms);
        }
    }
}
