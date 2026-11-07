use std::time::Instant;

use tb4_core::Pose2;
use tb4_slam::sim::{Lidar, OdomModel, Rng, World, drive, pose_error};
use tb4_slam::{Slam, SlamParams};

const DT: f64 = 1.0 / 7.5;

// usage: bench [laps] [lidar_noise_m] [seed]
fn main() {
    let args: Vec<f64> = std::env::args().skip(1).filter_map(|a| a.parse().ok()).collect();
    let laps = args.first().map_or(1, |&v| v as usize).max(1);
    let noise = args.get(1).copied().unwrap_or(0.015);
    let seed = args.get(2).map_or(42, |&v| v as u64);
    let world = World::apartment();
    let lap = [
        (3.0, 0.5),
        (3.0, 3.0),
        (5.0, 3.0),
        (6.8, 2.5),
        (7.0, 4.5),
        (9.5, 4.5),
        (7.0, 4.5),
        (7.3, 7.0),
        (5.0, 7.0),
        (5.0, 3.0),
        (3.0, 3.0),
        (3.2, 6.5),
        (3.2, 3.0),
        (3.0, 0.5),
        (0.0, 0.0),
        (1.0, 0.0),
    ];
    let waypoints: Vec<(f64, f64)> = (0..laps).flat_map(|_| lap).collect();
    let truth = drive(Pose2::default(), &waypoints, DT, 0.3, 0.8);
    let mut rng = Rng::new(seed);
    let model = OdomModel {
        trans_noise: 0.02,
        yaw_noise: 0.002,
        yaw_bias: 0.01,
        yaw_scale: 0.02,
    };
    let odom = model.integrate(&truth, DT, &mut rng);
    let lidar = Lidar {
        noise,
        ..Lidar::default()
    };
    let scans: Vec<Vec<(f64, f64)>> = truth.iter().map(|t| lidar.scan(&world, *t, &mut rng)).collect();

    let mut slam = Slam::new(SlamParams::default());
    let mut times = Vec::new();
    let (mut last_err, mut max_err, mut last_k) = ((0.0, 0.0), 0.0f64, 0);
    for (k, (o, s)) in odom.iter().zip(&scans).enumerate() {
        let t = Instant::now();
        let r = slam.update(*o, s);
        let dt = t.elapsed().as_secs_f64() * 1e3;
        if let Some(p) = r {
            times.push(dt);
            last_err = pose_error(p, truth[k]);
            max_err = max_err.max(last_err.0);
            last_k = k;
        }
    }
    let t = Instant::now();
    let grid = slam.occupancy();
    let grid_ms = t.elapsed().as_secs_f64() * 1e3;

    let st = slam.stats();
    let mean = times.iter().sum::<f64>() / times.len() as f64;
    let max = times.iter().copied().fold(0.0, f64::max);
    let mut sorted = times.clone();
    sorted.sort_by(f64::total_cmp);
    let p95 = sorted[(sorted.len() as f64 * 0.95) as usize];
    let odom_err = pose_error(odom[last_k], truth[last_k]);
    println!(
        "duration       {:.1} s simulated, {} scans",
        truth.len() as f64 * DT,
        truth.len()
    );
    println!(
        "processed      {} (skipped {}), fallbacks {}",
        st.scans_processed, st.scans_skipped, st.match_fallbacks
    );
    println!(
        "update time    mean {mean:.2} ms, p95 {p95:.2} ms, max {max:.2} ms (scan period {:.0} ms)",
        DT * 1e3
    );
    println!(
        "last opt       {:.2} ms, last map rebuild {:.2} ms",
        st.last_optimisation_ms, st.last_rebuild_ms
    );
    println!("keyframes      {}", st.keyframes);
    println!("loop closures  {} (rejected {})", st.loop_closures, st.loop_rejections);
    println!(
        "final error    slam {:.3} m / {:.4} rad, odom {:.3} m / {:.4} rad",
        last_err.0, last_err.1, odom_err.0, odom_err.1
    );
    println!("max slam error {max_err:.3} m");
    println!(
        "grid           {}x{} cells, export {grid_ms:.2} ms",
        grid.meta.width, grid.meta.height
    );
}
