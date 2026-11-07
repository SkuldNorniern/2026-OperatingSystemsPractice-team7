use tb4_core::{Grid, GridMeta, Pose2, grid::UNKNOWN};

const GROW_PAD: i32 = 64;

/// Growable dense log-odds grid indexed by global cell (floor(x / res)).
#[derive(Debug, Clone)]
pub struct LogOddsMap {
    res: f64,
    x0: i32,
    y0: i32,
    w: i32,
    h: i32,
    data: Vec<f32>,
    stamp: Vec<u32>,
    cur: u32,
    bbox: Option<[i32; 4]>,
    hit: f32,
    miss: f32,
    clamp: f32,
}

impl LogOddsMap {
    pub fn new(res: f64, hit: f32, miss: f32, clamp: f32) -> Self {
        Self {
            res,
            x0: 0,
            y0: 0,
            w: 0,
            h: 0,
            data: Vec::new(),
            stamp: Vec::new(),
            cur: 0,
            bbox: None,
            hit,
            miss: -miss.abs(),
            clamp,
        }
    }

    #[inline]
    pub fn cell(&self, v: f64) -> i32 {
        (v / self.res).floor() as i32
    }

    pub fn bbox(&self) -> Option<[i32; 4]> {
        self.bbox
    }

    pub fn clear(&mut self) {
        self.data.fill(0.0);
        self.bbox = None;
    }

    #[inline]
    fn idx(&self, cx: i32, cy: i32) -> Option<usize> {
        let (x, y) = (cx - self.x0, cy - self.y0);
        (x >= 0 && y >= 0 && x < self.w && y < self.h).then(|| (y * self.w + x) as usize)
    }

    #[inline]
    pub fn get(&self, cx: i32, cy: i32) -> f32 {
        self.idx(cx, cy).map_or(0.0, |i| self.data[i])
    }

    fn ensure(&mut self, b: [i32; 4]) {
        let (x1, y1) = (self.x0 + self.w - 1, self.y0 + self.h - 1);
        if self.w > 0 && b[0] >= self.x0 && b[1] >= self.y0 && b[2] <= x1 && b[3] <= y1 {
            return;
        }
        let n = if self.w == 0 {
            [b[0] - GROW_PAD, b[1] - GROW_PAD, b[2] + GROW_PAD, b[3] + GROW_PAD]
        } else {
            [
                if b[0] < self.x0 { b[0] - GROW_PAD } else { self.x0 },
                if b[1] < self.y0 { b[1] - GROW_PAD } else { self.y0 },
                if b[2] > x1 { b[2] + GROW_PAD } else { x1 },
                if b[3] > y1 { b[3] + GROW_PAD } else { y1 },
            ]
        };
        let (nw, nh) = (n[2] - n[0] + 1, n[3] - n[1] + 1);
        let mut data = vec![0.0f32; (nw * nh) as usize];
        let mut stamp = vec![0u32; (nw * nh) as usize];
        for y in 0..self.h {
            let src = (y * self.w) as usize;
            let dst = ((y + self.y0 - n[1]) * nw + (self.x0 - n[0])) as usize;
            let w = self.w as usize;
            data[dst..dst + w].copy_from_slice(&self.data[src..src + w]);
            stamp[dst..dst + w].copy_from_slice(&self.stamp[src..src + w]);
        }
        (self.x0, self.y0, self.w, self.h) = (n[0], n[1], nw, nh);
        self.data = data;
        self.stamp = stamp;
    }

    /// Points are in the sensor/base frame at `pose`; returns beyond `max_r` become free-only rays.
    pub fn insert_scan(&mut self, pose: Pose2, pts: &[(f32, f32)], min_r: f64, max_r: f64) {
        let (s, c) = pose.yaw.sin_cos();
        let (sx, sy) = (self.cell(pose.x), self.cell(pose.y));
        let mut b = [sx, sy, sx, sy];
        let mut rays = Vec::with_capacity(pts.len());
        for &(px, py) in pts {
            let (px, py) = (px as f64, py as f64);
            let r = px.hypot(py);
            if !r.is_finite() || r < min_r {
                continue;
            }
            let (k, hit) = if r > max_r { (max_r / r, false) } else { (1.0, true) };
            let ex = pose.x + k * (c * px - s * py);
            let ey = pose.y + k * (s * px + c * py);
            let (cx, cy) = (self.cell(ex), self.cell(ey));
            b = [b[0].min(cx), b[1].min(cy), b[2].max(cx), b[3].max(cy)];
            rays.push((cx, cy, hit));
        }
        self.ensure(b);
        self.grow_bbox(b);
        self.cur = self.cur.wrapping_add(1);
        if self.cur == 0 {
            self.stamp.fill(0);
            self.cur = 1;
        }
        let cur = self.cur;
        // each cell gets at most one update per scan, hits take precedence over misses
        for &(cx, cy, hit) in &rays {
            if hit {
                let i = self.idx(cx, cy).expect("map covers scan");
                if self.stamp[i] != cur {
                    self.stamp[i] = cur;
                    self.data[i] = (self.data[i] + self.hit).min(self.clamp);
                }
            }
        }
        let w = self.w;
        let (x0, y0) = (self.x0, self.y0);
        for &(ex, ey, hit) in &rays {
            let (dx, dy) = ((ex - sx).abs(), -(ey - sy).abs());
            let (stx, sty) = ((ex - sx).signum(), (ey - sy).signum());
            let (mut x, mut y, mut err) = (sx, sy, dx + dy);
            loop {
                let end = x == ex && y == ey;
                // no misses next to a hit endpoint, else slightly misaligned rays erase thin walls
                let near_end = hit && (x - ex).abs() <= 1 && (y - ey).abs() <= 1;
                if !near_end {
                    let i = ((y - y0) * w + (x - x0)) as usize;
                    if self.stamp[i] != cur {
                        self.stamp[i] = cur;
                        self.data[i] = (self.data[i] + self.miss).max(-self.clamp);
                    }
                }
                if end {
                    break;
                }
                let e2 = 2 * err;
                if e2 >= dy {
                    err += dy;
                    x += stx;
                }
                if e2 <= dx {
                    err += dx;
                    y += sty;
                }
            }
        }
    }

    fn grow_bbox(&mut self, b: [i32; 4]) {
        self.bbox = Some(match self.bbox {
            None => b,
            Some(o) => [o[0].min(b[0]), o[1].min(b[1]), o[2].max(b[2]), o[3].max(b[3])],
        });
    }

    /// Adds another map's log-odds (same resolution), clamping the sum.
    pub fn merge_add(&mut self, o: &LogOddsMap) {
        let Some(b) = o.bbox else { return };
        self.ensure(b);
        self.grow_bbox(b);
        for y in b[1]..=b[3] {
            for x in b[0]..=b[2] {
                let v = o.get(x, y);
                if v != 0.0 {
                    let i = self.idx(x, y).expect("map covers merged bbox");
                    self.data[i] = (self.data[i] + v).clamp(-self.clamp, self.clamp);
                }
            }
        }
    }

    fn meta(&self, b: [i32; 4]) -> GridMeta {
        GridMeta {
            width: (b[2] - b[0] + 1) as u32,
            height: (b[3] - b[1] + 1) as u32,
            resolution: self.res,
            origin_x: b[0] as f64 * self.res,
            origin_y: b[1] as f64 * self.res,
        }
    }

    fn render(&self, b: [i32; 4], f: impl Fn(f32) -> i8) -> Grid {
        let mut data = Vec::with_capacity(((b[2] - b[0] + 1) * (b[3] - b[1] + 1)) as usize);
        for y in b[1]..=b[3] {
            for x in b[0]..=b[2] {
                data.push(f(self.get(x, y)));
            }
        }
        Grid::new(self.meta(b), data)
    }

    /// Binary grid (100 occupied, 0 otherwise) over inclusive cell bounds.
    pub fn window_grid(&self, b: [i32; 4], occ: f32) -> Grid {
        self.render(b, |v| if v > occ { 100 } else { 0 })
    }

    pub fn to_grid(&self, occ: f32, free: f32, margin: i32) -> Grid {
        let Some(b) = self.bbox else {
            return Grid::filled(self.meta([0, 0, 0, 0]), UNKNOWN);
        };
        let b = [b[0] - margin, b[1] - margin, b[2] + margin, b[3] + margin];
        self.render(b, |v| {
            if v > occ {
                100
            } else if v < free {
                0
            } else {
                UNKNOWN
            }
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ray_marks_free_and_hit() {
        let mut m = LogOddsMap::new(0.05, 0.9, 0.4, 3.5);
        m.insert_scan(Pose2::new(0.0, 0.0, 0.0), &[(1.0, 0.0), (0.0, -2.0)], 0.1, 5.0);
        assert!(m.get(m.cell(1.0), m.cell(0.0)) > 0.5);
        assert!(m.get(m.cell(0.5), m.cell(0.0)) < -0.3);
        assert!(m.get(m.cell(0.0), m.cell(-1.0)) < -0.3);
        // grows in the other direction and keeps old content
        m.insert_scan(Pose2::new(0.0, 0.0, 0.0), &[(-6.0, 3.0)], 0.1, 10.0);
        assert!(m.get(m.cell(1.0), m.cell(0.0)) > 0.5);
        assert!(m.get(m.cell(-6.0), m.cell(3.0)) > 0.5);
        let g = m.to_grid(0.6, -0.3, 2);
        let c = g.world_to_cell(1.01, 0.01).unwrap();
        assert_eq!(g.get(c), 100);
        let c = g.world_to_cell(0.5, 0.01).unwrap();
        assert_eq!(g.get(c), 0);
    }

    #[test]
    fn max_range_is_free_only() {
        let mut m = LogOddsMap::new(0.05, 0.9, 0.4, 3.5);
        m.insert_scan(Pose2::new(0.0, 0.0, 0.0), &[(12.0, 0.0)], 0.1, 5.0);
        assert!(m.get(m.cell(4.9), 0) < 0.0);
        assert_eq!(m.get(m.cell(5.5), 0), 0.0);
        assert!(m.get(m.cell(12.0), 0) == 0.0);
    }
}
