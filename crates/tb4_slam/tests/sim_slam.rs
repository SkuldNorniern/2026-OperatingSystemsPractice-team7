use std::sync::OnceLock;

use tb4_core::{Pose2, grid::UNKNOWN};
use tb4_slam::sim::{Lidar, OdomModel, Rng, World, drive, pose_error};
use tb4_slam::{Slam, SlamParams};

const DT: f64 = 1.0 / 7.5;

const SHORT: &[(f64, f64)] = &[(1.6, 0.0), (1.6, 1.8), (3.4, 1.8), (3.4, 0.2)];
const LOOP: &[(f64, f64)] = &[
    (1.6, 0.0),
    (1.6, 1.8),
    (3.4, 1.8),
    (3.4, 0.2),
    (5.6, 0.6),
    (5.6, 3.6),
    (3.0, 3.7),
    (-0.2, 3.7),
    (-0.2, 0.8),
    (0.0, 0.0),
    (1.4, 0.0),
];

struct Outcome {
    world: World,
    slam: Slam,
    truth: Vec<Pose2>,
    max_err: f64,
    // (slam, odom) error at the last processed scan
    last_err: ((f64, f64), (f64, f64)),
}

fn run(waypoints: &[(f64, f64)], model: OdomModel, params: SlamParams, seed: u64) -> Outcome {
    let world = World::room();
    let truth = drive(Pose2::default(), waypoints, DT, 0.25, 0.8);
    let mut rng = Rng::new(seed);
    let odom = model.integrate(&truth, DT, &mut rng);
    let lidar = Lidar::default();
    let mut slam = Slam::new(params);
    let mut max_err = 0.0f64;
    let mut last_err = ((0.0, 0.0), (0.0, 0.0));
    for (t, o) in truth.iter().zip(&odom) {
        let scan = lidar.scan(&world, *t, &mut rng);
        if let Some(p) = slam.update(*o, &scan) {
            max_err = max_err.max(pose_error(p, *t).0);
            last_err = (pose_error(p, *t), pose_error(*o, *t));
        }
    }
    Outcome {
        world,
        slam,
        truth,
        max_err,
        last_err,
    }
}

fn loop_run() -> &'static Outcome {
    static R: OnceLock<Outcome> = OnceLock::new();
    R.get_or_init(|| {
        let model = OdomModel {
            trans_noise: 0.02,
            yaw_noise: 0.002,
            yaw_bias: 0.02,
            yaw_scale: 0.03,
        };
        run(LOOP, model, SlamParams::default(), 7)
    })
}

#[test]
fn short_run_beats_odometry() {
    let model = OdomModel {
        trans_noise: 0.02,
        yaw_noise: 0.002,
        yaw_bias: 0.015,
        yaw_scale: 0.03,
    };
    let r = run(SHORT, model, SlamParams::default(), 1);
    let ((es, ys), (eo, yo)) = r.last_err;
    eprintln!(
        "short: slam {es:.3} m {ys:.4} rad, odom {eo:.3} m {yo:.4} rad, max {:.3}",
        r.max_err
    );
    assert!(eo > 0.2, "odometry drift too small for a meaningful test");
    assert!(es < 0.08 && es < 0.25 * eo);
    assert!(ys < 0.02 && ys < 0.25 * yo);
    assert!(r.max_err < 0.12);
}

#[test]
fn loop_closure_corrects_drift() {
    let r = loop_run();
    let ((es, ys), (eo, yo)) = r.last_err;
    let st = r.slam.stats();
    eprintln!(
        "loop: slam {es:.3} m {ys:.4} rad, odom {eo:.3} m {yo:.4} rad, max {:.3}, kfs {}, loops {} (rejected {}), fallbacks {}",
        r.max_err, st.keyframes, st.loop_closures, st.loop_rejections, st.match_fallbacks
    );
    assert!(eo > 1.0);
    assert!(st.loop_closures >= 1);
    assert!(es < 0.08 && ys < 0.03);
    assert!(r.max_err < 0.15);
    assert!(r.slam.keyframes().len() > 30);
}

#[test]
fn back_end_alone_corrects_odometry_drift() {
    // scan matching disabled: the map follows odometry until a loop closure pulls it back
    let params = SlamParams {
        min_match_score: 2.0,
        ..SlamParams::default()
    };
    let model = OdomModel {
        trans_noise: 0.01,
        yaw_noise: 0.0005,
        yaw_bias: 0.002,
        yaw_scale: 0.005,
    };
    let r = run(LOOP, model, params, 11);
    let ((es, ys), (eo, yo)) = r.last_err;
    let st = r.slam.stats();
    eprintln!(
        "back end: slam {es:.3} m {ys:.4} rad, odom {eo:.3} m {yo:.4} rad, loops {} (rejected {}), fallbacks {}",
        st.loop_closures, st.loop_rejections, st.match_fallbacks
    );
    assert!(st.loop_closures >= 1);
    assert!(eo > 0.3);
    assert!(es < 0.3 * eo && ys < 0.5 * yo);
}

#[test]
fn occupancy_matches_world() {
    let r = loop_run();
    let g = r.slam.occupancy();
    let occupied_near = |x: f64, y: f64| {
        let (cx, cy) = g.world_to_cell_signed(x, y);
        (-1..=1).any(|dy| (-1..=1).any(|dx| g.cell_checked(cx + dx, cy + dy).is_some_and(|c| g.get(c) == 100)))
    };
    let views: Vec<Pose2> = r.truth.iter().step_by(10).copied().collect();
    let (mut seen, mut ok) = (0, 0);
    for s in &r.world.segments {
        let len = (s[2] - s[0]).hypot(s[3] - s[1]);
        let n = (len / 0.1).ceil() as usize;
        for k in 0..=n {
            let f = k as f64 / n as f64;
            let (x, y) = (s[0] + f * (s[2] - s[0]), s[1] + f * (s[3] - s[1]));
            let visible = views.iter().any(|v| {
                let d = v.distance_to(x, y);
                d < 6.0 && (r.world.ray(v.x, v.y, (y - v.y).atan2(x - v.x)) - d).abs() < 0.02
            });
            if visible {
                seen += 1;
                ok += occupied_near(x, y) as usize;
            }
        }
    }
    eprintln!("walls: {ok}/{seen} visible samples occupied");
    assert!(seen > 200 && ok as f64 > 0.9 * seen as f64);

    let (mut free_n, mut free_ok, mut unknown) = (0, 0, 0);
    for t in r.truth.iter().step_by(5) {
        for (dx, dy) in [(0.0, 0.0), (0.3, 0.0), (-0.3, 0.0), (0.0, 0.3), (0.0, -0.3)] {
            let (x, y) = (t.x + dx, t.y + dy);
            if r.world.wall_distance(x, y) < 0.35 {
                continue;
            }
            let v = g.world_to_cell(x, y).map_or(UNKNOWN, |c| g.get(c));
            free_n += 1;
            free_ok += (v == 0) as usize;
            unknown += (v == UNKNOWN) as usize;
        }
    }
    eprintln!("free: {free_ok}/{free_n} (unknown {unknown})");
    assert!(free_n > 200 && free_ok as f64 > 0.97 * free_n as f64);
}

#[test]
fn motion_gate_skips_stationary_scans() {
    let world = World::room();
    let mut rng = Rng::new(3);
    let lidar = Lidar::default();
    let mut slam = Slam::new(SlamParams::default());
    let at = Pose2::new(0.0, 0.0, 0.0);
    assert!(slam.update(at, &lidar.scan(&world, at, &mut rng)).is_some());
    for k in 0..20 {
        let jitter = Pose2::new(0.002 * k as f64, -0.001, 0.003);
        assert!(slam.update(jitter, &lidar.scan(&world, at, &mut rng)).is_none());
    }
    assert_eq!(slam.stats().scans_skipped, 20);
    assert_eq!(slam.stats().scans_processed, 1);
    let moved = Pose2::new(0.15, 0.0, 0.0);
    assert!(slam.update(moved, &lidar.scan(&world, moved, &mut rng)).is_some());
    let turned = Pose2::new(0.15, 0.0, 0.12);
    assert!(slam.update(turned, &lidar.scan(&world, turned, &mut rng)).is_some());
    assert_eq!(slam.stats().scans_processed, 3);
}

#[test]
fn first_scan_defines_origin() {
    let world = World::room();
    let mut rng = Rng::new(5);
    let mut slam = Slam::new(SlamParams::default());
    let scan = Lidar::default().scan(&world, Pose2::default(), &mut rng);
    let p = slam.update(Pose2::default(), &scan).unwrap();
    assert_eq!(p, Pose2::default());
    assert_eq!(slam.pose(), Pose2::default());
    assert_eq!(slam.map_to_odom(), Pose2::default());
    assert_eq!(slam.keyframes(), vec![Pose2::default()]);
    let g = slam.occupancy();
    let c = g.world_to_cell(0.0, 0.0).unwrap();
    assert_eq!(g.get(c), 0);
    // a non-zero first odom maps to itself
    let mut slam = Slam::new(SlamParams::default());
    let o = Pose2::new(1.0, 2.0, 0.5);
    let scan = Lidar::default().scan(&world, Pose2::default(), &mut rng);
    assert_eq!(slam.update(o, &scan), Some(o));
    let m = slam.map_to_odom();
    assert!(m.x.abs() < 1e-12 && m.y.abs() < 1e-12 && m.yaw.abs() < 1e-12);
}
