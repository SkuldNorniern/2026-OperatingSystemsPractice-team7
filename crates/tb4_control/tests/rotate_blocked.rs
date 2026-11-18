// A goal behind the robot, with an obstacle on the path behind it (so the local path is blocked).
// The robot must turn in place toward the goal, not stall and not drive forward.
use tb4_control::{Mppi, MppiParams, RobotState};
use tb4_core::{Cell, DistanceField, Grid, GridMeta, Pose2};

fn params() -> MppiParams {
    MppiParams { samples: 200, threads: 2, ..MppiParams::default() }
}

// wall at x = -0.3 (in front of nothing: it is behind the robot), spanning y in [-0.5, 0.5].
// The robot centre at (0, 0) is 0.3 m from the wall, so its footprint (radius 0.18) is clear.
fn wall_behind() -> DistanceField {
    let meta = GridMeta { width: 120, height: 60, resolution: 0.05, origin_x: -1.0, origin_y: -1.5 };
    let mut g = Grid::filled(meta, 0);
    for cy in 20..40 {
        g.set(Cell { x: 14, y: cy }, 100);
    }
    DistanceField::from_grid(&g, 65, false)
}

#[test]
fn turns_in_place_when_goal_is_behind_and_path_blocked() {
    let p = params();
    let df = wall_behind();
    let mut m = Mppi::new(p.clone());
    // goal 0.8 m behind the robot, facing +x; the path runs into the wall at x = -0.3
    m.set_path(&[(0.0, 0.0), (-0.8, 0.0)], None);
    let dt = p.dt;
    let mut s = RobotState { pose: Pose2::new(0.0, 0.0, 0.0), v: 0.0, w: 0.0 };
    let mut turned_at_start = false;
    for _ in 0..60 {
        let c = m.compute(&s, Some(&df));
        assert!(c.v.abs() < 1e-9 || !turned_at_start, "drove forward while turning, v = {}", c.v);
        if m.is_rotating() {
            turned_at_start = true;
        }
        assert!(c.collision_free, "turning in place with a clear footprint must be allowed");
        s = tb4_control::simulate(&s, c.control(), dt);
        assert!(s.pose.x.abs() < 0.02 && s.pose.y.abs() < 0.02, "moved while turning: {:?}", s.pose);
    }
    assert!(turned_at_start, "controller never entered rotate-in-place mode");
    assert!(s.pose.yaw.abs() > 1.0, "did not turn toward the goal, yaw = {}", s.pose.yaw);
}

#[test]
fn no_turn_in_place_when_footprint_itself_touches_obstacle() {
    let p = params();
    let df = wall_behind();
    let mut m = Mppi::new(p.clone());
    // robot 0.1 m from the wall: footprint (0.18) overlaps it, so rotating here is not allowed
    m.set_path(&[(-0.2, 0.0), (-0.8, 0.0)], None);
    let s = RobotState { pose: Pose2::new(-0.2, 0.0, 0.0), v: 0.0, w: 0.0 };
    let c = m.compute(&s, Some(&df));
    assert!(!m.is_rotating(), "must not rotate in place with the footprint in contact");
    let _ = c;
}
