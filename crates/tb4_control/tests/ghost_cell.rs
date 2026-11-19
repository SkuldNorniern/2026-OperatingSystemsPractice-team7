// Field run 20261118_183647: one SLAM map cell 0.14 m behind-left of the robot, inside the 0.18 m
// footprint, while the lidar showed nothing within 0.42 m. Every MPPI sample collided, standing
// still included, so the robot published zero for 30 s per goal, five goals in a row.
// The robot must now drive away from such a cell, and never toward it.
use tb4_control::{Mppi, MppiParams, RobotState};
use tb4_core::{Cell, DistanceField, Grid, GridMeta, Pose2};

fn params() -> MppiParams {
    MppiParams { samples: 300, threads: 2, vx_max: 0.18, wz_max: 0.5, ..MppiParams::default() }
}

/// free 6 x 6 m map with one occupied cell at (x, y)
fn ghost_at(x: f64, y: f64) -> DistanceField {
    let meta = GridMeta { width: 120, height: 120, resolution: 0.05, origin_x: -3.0, origin_y: -3.0 };
    let mut g = Grid::filled(meta, 0);
    let c = g.world_to_cell(x, y).unwrap();
    g.set(Cell { x: c.x, y: c.y }, 100);
    DistanceField::from_grid(&g, 65, false)
}

fn run(goal: (f64, f64), ghost: (f64, f64)) -> (Pose2, f64) {
    let p = params();
    let df = ghost_at(ghost.0, ghost.1);
    let mut m = Mppi::new(p.clone());
    m.set_path(&[(0.0, 0.0), goal], None);
    let mut s = RobotState { pose: Pose2::new(0.0, 0.0, 0.0), v: 0.0, w: 0.0 };
    let start = df.at(0.0, 0.0) as f64;
    let mut min_clear = f64::MAX;
    for _ in 0..200 {
        let c = m.compute(&s, Some(&df));
        s = tb4_control::simulate(&s, c.control(), p.dt);
        min_clear = min_clear.min(df.at(s.pose.x, s.pose.y) as f64);
    }
    assert!(min_clear >= start - 0.011, "got closer to the ghost cell: start {start:.3}, min {min_clear:.3}");
    (s.pose, start)
}

#[test]
fn drives_away_from_cell_behind() {
    // ghost behind-left (157 deg), goal 1.5 m ahead
    let a = 157f64.to_radians();
    let (end, start) = run((1.5, 0.0), (0.14 * a.cos(), 0.14 * a.sin()));
    assert!(start < 0.18, "test setup: ghost must be inside the footprint, clearance {start}");
    assert!(end.x > 1.0, "did not drive to the goal, ended at {end:?}");
}

#[test]
fn turns_and_drives_when_goal_is_to_the_side() {
    // field case: goal off to the side, ghost behind; the robot has to turn first
    let a = 157f64.to_radians();
    let (end, _) = run((0.0, -1.5), (0.14 * a.cos(), 0.14 * a.sin()));
    assert!(end.y < -0.8, "did not get going toward the goal, ended at {end:?}");
}
