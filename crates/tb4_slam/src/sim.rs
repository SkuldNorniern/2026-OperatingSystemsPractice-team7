//! Synthetic segment world, lidar and drifting odometry for tests and benchmarks.

use std::f64::consts::TAU;

use tb4_core::Pose2;

use crate::se2::{between, compose, wrap};

/// SplitMix64 with Box-Muller normals.
pub struct Rng {
    state: u64,
    spare: Option<f64>,
}

impl Rng {
    pub fn new(seed: u64) -> Self {
        Self {
            state: seed,
            spare: None,
        }
    }

    pub fn next_u64(&mut self) -> u64 {
        self.state = self.state.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.state;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }

    pub fn uniform(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 / (1u64 << 53) as f64
    }

    pub fn gauss(&mut self) -> f64 {
        if let Some(s) = self.spare.take() {
            return s;
        }
        let u1 = self.uniform().max(1e-300);
        let r = (-2.0 * u1.ln()).sqrt();
        let t = TAU * self.uniform();
        self.spare = Some(r * t.sin());
        r * t.cos()
    }
}

#[derive(Debug, Clone, Default)]
pub struct World {
    pub segments: Vec<[f64; 4]>,
}

impl World {
    pub fn wall(&mut self, a: (f64, f64), b: (f64, f64)) {
        self.segments.push([a.0, a.1, b.0, b.1]);
    }

    pub fn rect(&mut self, x0: f64, y0: f64, x1: f64, y1: f64) {
        self.wall((x0, y0), (x1, y0));
        self.wall((x1, y0), (x1, y1));
        self.wall((x1, y1), (x0, y1));
        self.wall((x0, y1), (x0, y0));
    }

    /// 8 x 6 m room with a partial wall and boxes; start pose (0, 0) is in free space.
    pub fn room() -> Self {
        let mut w = World::default();
        w.rect(-1.5, -1.5, 6.5, 4.5);
        w.wall((2.5, -1.5), (2.5, 1.2));
        w.wall((-1.5, 2.0), (-0.6, 2.0));
        w.rect(4.2, 2.2, 4.8, 2.8);
        w.rect(0.3, 2.4, 0.7, 3.2);
        w.rect(4.65, -0.45, 4.95, -0.15);
        w
    }

    /// 12 x 9 m, three rooms connected by doorways.
    pub fn apartment() -> Self {
        let mut w = World::default();
        w.rect(-1.0, -1.0, 11.0, 8.0);
        w.wall((4.0, -1.0), (4.0, 2.5));
        w.wall((4.0, 3.5), (4.0, 8.0));
        w.wall((8.0, -1.0), (8.0, 4.0));
        w.wall((8.0, 5.0), (8.0, 8.0));
        w.wall((-1.0, 5.0), (2.5, 5.0));
        w.rect(1.5, 1.5, 2.2, 2.0);
        w.rect(5.5, 0.5, 6.2, 1.2);
        w.rect(6.0, 5.5, 6.8, 6.3);
        w.rect(9.5, 1.0, 10.2, 1.6);
        w.rect(9.2, 6.0, 10.5, 6.6);
        w.rect(0.5, 6.3, 1.3, 7.0);
        w
    }

    pub fn ray(&self, x: f64, y: f64, ang: f64) -> f64 {
        let (dx, dy) = (ang.cos(), ang.sin());
        let mut best = f64::INFINITY;
        for s in &self.segments {
            let (ex, ey) = (s[2] - s[0], s[3] - s[1]);
            let den = dx * ey - dy * ex;
            if den.abs() < 1e-12 {
                continue;
            }
            let (wx, wy) = (s[0] - x, s[1] - y);
            let t = (wx * ey - wy * ex) / den;
            let u = (wx * dy - wy * dx) / den;
            if t > 0.0 && (0.0..=1.0).contains(&u) && t < best {
                best = t;
            }
        }
        best
    }

    pub fn wall_distance(&self, x: f64, y: f64) -> f64 {
        self.segments
            .iter()
            .map(|s| {
                let (ex, ey) = (s[2] - s[0], s[3] - s[1]);
                let t = (((x - s[0]) * ex + (y - s[1]) * ey) / (ex * ex + ey * ey)).clamp(0.0, 1.0);
                (x - s[0] - t * ex).hypot(y - s[1] - t * ey)
            })
            .fold(f64::INFINITY, f64::min)
    }
}

#[derive(Debug, Clone, Copy)]
pub struct Lidar {
    pub beams: usize,
    pub min_range: f64,
    pub max_range: f64,
    pub noise: f64,
    pub dropout: f64,
}

impl Default for Lidar {
    fn default() -> Self {
        Self {
            beams: 360,
            min_range: 0.15,
            max_range: 12.0,
            noise: 0.015,
            dropout: 0.03,
        }
    }
}

impl Lidar {
    /// Endpoints in the robot frame; beams without a return are omitted.
    pub fn scan(&self, world: &World, pose: Pose2, rng: &mut Rng) -> Vec<(f64, f64)> {
        let mut out = Vec::with_capacity(self.beams);
        for k in 0..self.beams {
            let a = k as f64 * TAU / self.beams as f64;
            let r = world.ray(pose.x, pose.y, pose.yaw + a);
            let n = rng.gauss();
            if rng.uniform() < self.dropout || !(self.min_range..=self.max_range).contains(&r) {
                continue;
            }
            let r = r + self.noise * n;
            out.push((r * a.cos(), r * a.sin()));
        }
        out
    }
}

#[derive(Debug, Clone, Copy)]
pub struct OdomModel {
    /// relative std of translation increments
    pub trans_noise: f64,
    /// std of yaw noise per step (rad)
    pub yaw_noise: f64,
    /// gyro-like bias (rad/s)
    pub yaw_bias: f64,
    /// relative yaw scale error
    pub yaw_scale: f64,
}

impl OdomModel {
    pub fn integrate(&self, truth: &[Pose2], dt: f64, rng: &mut Rng) -> Vec<Pose2> {
        let mut out = Vec::with_capacity(truth.len());
        let Some(&first) = truth.first() else { return out };
        out.push(first);
        for w in truth.windows(2) {
            let d = between(w[0], w[1]);
            let noisy = Pose2::new(
                d.x * (1.0 + self.trans_noise * rng.gauss()),
                d.y * (1.0 + self.trans_noise * rng.gauss()),
                d.yaw * (1.0 + self.yaw_scale) + self.yaw_bias * dt + self.yaw_noise * rng.gauss(),
            );
            let last = *out.last().unwrap();
            out.push(compose(last, noisy));
        }
        out
    }
}

/// Unicycle path through waypoints (turn in place when heading error is large).
pub fn drive(start: Pose2, waypoints: &[(f64, f64)], dt: f64, v_max: f64, w_max: f64) -> Vec<Pose2> {
    let mut p = start;
    let mut out = vec![p];
    for &(wx, wy) in waypoints {
        for _ in 0..100_000 {
            let dist = p.distance_to(wx, wy);
            if dist < 0.05 {
                break;
            }
            let err = wrap((wy - p.y).atan2(wx - p.x) - p.yaw);
            let w = (2.0 * err).clamp(-w_max, w_max);
            let v = if err.abs() > 0.4 {
                0.0
            } else {
                v_max.min(0.8 * dist + 0.05)
            };
            let sub = 4;
            for _ in 0..sub {
                let h = dt / sub as f64;
                p.x += v * h * (p.yaw + 0.5 * w * h).cos();
                p.y += v * h * (p.yaw + 0.5 * w * h).sin();
                p.yaw = wrap(p.yaw + w * h);
            }
            out.push(p);
        }
    }
    out
}

pub fn pose_error(a: Pose2, b: Pose2) -> (f64, f64) {
    (a.distance_to(b.x, b.y), wrap(a.yaw - b.yaw).abs())
}
