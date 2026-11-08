use tb4_core::Pose2;
use tb4_slam::sim::{Lidar, OdomModel, Rng, World, drive, pose_error};
use tb4_slam::{Slam, SlamParams};

const DT: f64 = 1.0 / 7.5;

// online pose error over many seeds with realistic odometry: what marker views see
// usage: drift [seeds] [yaw_bias rad/s]
fn main() {
    let args: Vec<f64> = std::env::args().skip(1).filter_map(|a| a.parse().ok()).collect();
    let seeds = args.first().map_or(8, |&v| v as u64);
    let bias = args.get(1).copied().unwrap_or(0.002);
    let world = World::room();
    let wps = [(1.6, 0.0), (1.6, 1.8), (3.4, 1.8), (3.4, 0.2), (5.6, 0.6), (5.6, 3.6), (3.0, 3.7), (-0.2, 3.7), (-0.2, 0.8), (0.0, 0.0), (1.4, 0.0)];
    let truth = drive(Pose2::default(), &wps, DT, 0.3, 0.8);
    let (mut ys, mut ts, mut ymax, mut n) = (0.0, 0.0, 0.0f64, 0);
    for seed in 1..=seeds {
        let mut rng = Rng::new(seed);
        let odom = OdomModel { trans_noise: 0.02, yaw_noise: 0.001, yaw_bias: bias, yaw_scale: 0.005 }.integrate(&truth, DT, &mut rng);
        let lidar = Lidar { noise: 0.01, ..Lidar::default() };
        let mut slam = Slam::new(SlamParams::default());
        let (mut sy, mut st, mut k) = (0.0, 0.0, 0);
        for (t, o) in truth.iter().zip(&odom) {
            let scan = lidar.scan(&world, *t, &mut rng);
            if let Some(p) = slam.update(*o, &scan) {
                let (e, y) = pose_error(p, *t);
                sy += y;
                st += e;
                ymax = ymax.max(y);
                k += 1;
            }
        }
        println!("seed {seed}: mean yaw {:.3} deg, mean pos {:.3} m, loops {}", (sy / k as f64).to_degrees(), st / k as f64, slam.stats().loop_closures);
        ys += sy;
        ts += st;
        n += k;
    }
    println!("ALL: mean yaw {:.3} deg, max yaw {:.3} deg, mean pos {:.4} m", (ys / n as f64).to_degrees(), ymax.to_degrees(), ts / n as f64);
}
