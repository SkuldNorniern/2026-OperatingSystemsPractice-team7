use std::f64::consts::TAU;

pub fn splitmix64(x: &mut u64) -> u64 {
    *x = x.wrapping_add(0x9E37_79B9_7F4A_7C15);
    let mut z = *x;
    z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
    z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
    z ^ (z >> 31)
}

/// xoshiro256++ with Box-Muller normals
#[derive(Debug, Clone)]
pub struct Rng {
    s: [u64; 4],
    spare: Option<f64>,
}

impl Rng {
    pub fn new(seed: u64) -> Self {
        let mut x = seed;
        let s = [splitmix64(&mut x), splitmix64(&mut x), splitmix64(&mut x), splitmix64(&mut x)];
        Self { s, spare: None }
    }

    /// independent stream keyed by (seed, a, b, c)
    pub fn stream(seed: u64, a: u64, b: u64, c: u64) -> Self {
        let mut x = seed;
        let mut h = splitmix64(&mut x);
        for v in [a, b, c] {
            let mut y = h ^ v;
            h = splitmix64(&mut y);
        }
        Self::new(h)
    }

    pub fn next_u64(&mut self) -> u64 {
        let s = &mut self.s;
        let out = s[0].wrapping_add(s[3]).rotate_left(23).wrapping_add(s[0]);
        let t = s[1] << 17;
        s[2] ^= s[0];
        s[3] ^= s[1];
        s[1] ^= s[2];
        s[0] ^= s[3];
        s[2] ^= t;
        s[3] = s[3].rotate_left(45);
        out
    }

    /// uniform in [0, 1)
    pub fn uniform(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 * (1.0 / (1u64 << 53) as f64)
    }

    pub fn normal(&mut self) -> f64 {
        if let Some(z) = self.spare.take() {
            return z;
        }
        let u1 = 1.0 - self.uniform();
        let u2 = self.uniform();
        let r = (-2.0 * u1.ln()).sqrt();
        let (s, c) = (TAU * u2).sin_cos();
        self.spare = Some(r * s);
        r * c
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn normal_moments() {
        let mut r = Rng::new(7);
        let n = 200_000;
        let xs: Vec<f64> = (0..n).map(|_| r.normal()).collect();
        let mean = xs.iter().sum::<f64>() / n as f64;
        let var = xs.iter().map(|x| (x - mean).powi(2)).sum::<f64>() / n as f64;
        assert!(mean.abs() < 0.01 && (var - 1.0).abs() < 0.02, "{mean} {var}");
    }

    #[test]
    fn streams_differ_and_repeat() {
        let a = Rng::stream(1, 2, 3, 4).next_u64();
        assert_eq!(a, Rng::stream(1, 2, 3, 4).next_u64());
        assert_ne!(a, Rng::stream(1, 2, 3, 5).next_u64());
        assert_ne!(a, Rng::stream(1, 3, 2, 4).next_u64());
    }
}
