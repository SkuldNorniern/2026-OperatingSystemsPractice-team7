use crate::grid::{Cell, Grid, GridMeta, UNKNOWN};

#[derive(Debug, Clone)]
pub struct DistanceField {
    pub meta: GridMeta,
    /// metres to nearest obstacle cell centre, per cell, row-major like Grid
    pub dist: Vec<f32>,
}

impl DistanceField {
    /// exact euclidean distance transform (Felzenszwalb-Huttenlocher), O(cells)
    pub fn from_grid(g: &Grid, occupied_threshold: i8, unknown_is_obstacle: bool) -> Self {
        let (w, h) = (g.meta.width as usize, g.meta.height as usize);
        let inf = 1e20f64;
        let mut d2: Vec<f64> = g
            .data
            .iter()
            .map(|&v| {
                let obstacle = v >= occupied_threshold || (unknown_is_obstacle && v == UNKNOWN);
                if obstacle { 0.0 } else { inf }
            })
            .collect();
        let n = w.max(h);
        let mut f = vec![0.0; n];
        let mut out = vec![0.0; n];
        let mut v = vec![0usize; n];
        let mut z = vec![0.0; n + 1];
        for x in 0..w {
            for y in 0..h {
                f[y] = d2[y * w + x];
            }
            edt_1d(&f[..h], &mut out[..h], &mut v, &mut z);
            for y in 0..h {
                d2[y * w + x] = out[y];
            }
        }
        for y in 0..h {
            f[..w].copy_from_slice(&d2[y * w..(y + 1) * w]);
            edt_1d(&f[..w], &mut out[..w], &mut v, &mut z);
            d2[y * w..(y + 1) * w].copy_from_slice(&out[..w]);
        }
        let res = g.meta.resolution;
        let dist = d2.iter().map(|&s| if s >= inf { f32::INFINITY } else { (s.sqrt() * res) as f32 }).collect();
        Self { meta: g.meta, dist }
    }

    pub fn get(&self, c: Cell) -> f32 {
        self.dist[c.y as usize * self.meta.width as usize + c.x as usize]
    }

    /// distance at world point, 0 outside the grid
    pub fn at(&self, x: f64, y: f64) -> f32 {
        let m = &self.meta;
        let cx = ((x - m.origin_x) / m.resolution).floor() as i64;
        let cy = ((y - m.origin_y) / m.resolution).floor() as i64;
        if cx < 0 || cy < 0 || cx >= m.width as i64 || cy >= m.height as i64 {
            return 0.0;
        }
        self.dist[cy as usize * m.width as usize + cx as usize]
    }
}

fn edt_1d(f: &[f64], d: &mut [f64], v: &mut [usize], z: &mut [f64]) {
    let n = f.len();
    if n == 0 {
        return;
    }
    let inf = f64::INFINITY;
    let mut k = 0usize;
    v[0] = 0;
    z[0] = -inf;
    z[1] = inf;
    for q in 1..n {
        if f[q] >= 1e20 && f[v[k]] >= 1e20 {
            continue;
        }
        loop {
            let p = v[k];
            let s = ((f[q] + (q * q) as f64) - (f[p] + (p * p) as f64)) / (2.0 * q as f64 - 2.0 * p as f64);
            if s <= z[k] && k > 0 {
                k -= 1;
                continue;
            }
            if s <= z[k] {
                v[0] = q;
                z[0] = -inf;
                z[1] = inf;
                break;
            }
            k += 1;
            v[k] = q;
            z[k] = s;
            z[k + 1] = inf;
            break;
        }
    }
    let mut k = 0usize;
    for (q, dq) in d.iter_mut().enumerate() {
        while z[k + 1] < q as f64 {
            k += 1;
        }
        let p = v[k];
        let dx = q as f64 - p as f64;
        *dq = if f[p] >= 1e20 { 1e20 } else { dx * dx + f[p] };
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn grid(w: u32, h: u32) -> Grid {
        Grid::filled(GridMeta { width: w, height: h, resolution: 0.1, origin_x: 0.0, origin_y: 0.0 }, 0)
    }

    fn brute(g: &Grid, thr: i8) -> Vec<f32> {
        let (w, h) = (g.meta.width as i64, g.meta.height as i64);
        let obs: Vec<(i64, i64)> = (0..h)
            .flat_map(|y| (0..w).map(move |x| (x, y)))
            .filter(|&(x, y)| g.data[(y * w + x) as usize] >= thr)
            .collect();
        (0..h)
            .flat_map(|y| (0..w).map(move |x| (x, y)))
            .map(|(x, y)| {
                obs.iter()
                    .map(|&(ox, oy)| (((x - ox).pow(2) + (y - oy).pow(2)) as f64).sqrt() * 0.1)
                    .fold(f64::INFINITY, f64::min) as f32
            })
            .collect()
    }

    #[test]
    fn matches_brute_force() {
        let mut g = grid(23, 17);
        for (x, y) in [(3, 4), (10, 10), (22, 0), (0, 16), (11, 10), (15, 2)] {
            g.set(Cell { x, y }, 100);
        }
        let df = DistanceField::from_grid(&g, 65, false);
        let b = brute(&g, 65);
        for (a, e) in df.dist.iter().zip(&b) {
            assert!((a - e).abs() < 1e-4, "{a} vs {e}");
        }
    }

    #[test]
    fn unknown_and_empty() {
        let g = grid(5, 5);
        assert!(DistanceField::from_grid(&g, 65, false).dist.iter().all(|d| d.is_infinite()));
        let mut g = grid(5, 1);
        g.set(Cell { x: 0, y: 0 }, UNKNOWN);
        let df = DistanceField::from_grid(&g, 65, true);
        assert!((df.get(Cell { x: 4, y: 0 }) - 0.4).abs() < 1e-6);
        assert!(df.get(Cell { x: 0, y: 0 }) == 0.0);
        assert_eq!(df.at(-1.0, 0.0), 0.0);
    }
}
