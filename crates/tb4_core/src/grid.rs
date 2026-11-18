#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GridMeta {
    pub width: u32,
    pub height: u32,
    pub resolution: f64,
    pub origin_x: f64,
    pub origin_y: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Cell {
    pub x: u32,
    pub y: u32,
}

#[derive(Debug, Clone)]
pub struct Grid {
    pub meta: GridMeta,
    pub data: Vec<i8>,
}

pub const UNKNOWN: i8 = -1;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RayHit {
    pub x: f64,
    pub y: f64,
    pub range: f64,
}

impl Grid {
    pub fn new(meta: GridMeta, data: Vec<i8>) -> Self {
        assert_eq!(
            data.len(),
            meta.width as usize * meta.height as usize,
            "grid data length does not match width * height"
        );
        Self { meta, data }
    }

    pub fn filled(meta: GridMeta, value: i8) -> Self {
        let n = meta.width as usize * meta.height as usize;
        Self::new(meta, vec![value; n])
    }

    #[inline]
    pub fn index(&self, c: Cell) -> usize {
        c.y as usize * self.meta.width as usize + c.x as usize
    }

    #[inline]
    pub fn get(&self, c: Cell) -> i8 {
        self.data[self.index(c)]
    }

    #[inline]
    pub fn set(&mut self, c: Cell, v: i8) {
        let i = self.index(c);
        self.data[i] = v;
    }

    #[inline]
    pub fn world_to_cell_signed(&self, x: f64, y: f64) -> (i64, i64) {
        let m = &self.meta;
        (
            ((x - m.origin_x) / m.resolution).floor() as i64,
            ((y - m.origin_y) / m.resolution).floor() as i64,
        )
    }

    pub fn world_to_cell(&self, x: f64, y: f64) -> Option<Cell> {
        let (cx, cy) = self.world_to_cell_signed(x, y);
        self.cell_checked(cx, cy)
    }

    #[inline]
    pub fn cell_checked(&self, cx: i64, cy: i64) -> Option<Cell> {
        let m = &self.meta;
        (cx >= 0 && cy >= 0 && cx < m.width as i64 && cy < m.height as i64).then_some(Cell {
            x: cx as u32,
            y: cy as u32,
        })
    }

    pub fn cell_to_world(&self, c: Cell) -> (f64, f64) {
        let m = &self.meta;
        (
            m.origin_x + (c.x as f64 + 0.5) * m.resolution,
            m.origin_y + (c.y as f64 + 0.5) * m.resolution,
        )
    }

    pub fn raycast(
        &self,
        x: f64,
        y: f64,
        dx: f64,
        dy: f64,
        max_range: f64,
        occupied_threshold: i8,
    ) -> Option<RayHit> {
        let norm = dx.hypot(dy);
        if norm < 1e-12 {
            return None;
        }
        let (dx, dy) = (dx / norm, dy / norm);
        let res = self.meta.resolution;
        let (mut cx, mut cy) = self.world_to_cell_signed(x, y);

        let step_x: i64 = if dx > 0.0 { 1 } else { -1 };
        let step_y: i64 = if dy > 0.0 { 1 } else { -1 };
        let boundary = |c: i64, step: i64, origin: f64| origin + (c + (step > 0) as i64) as f64 * res;
        let mut t_max_x = if dx.abs() > 1e-12 {
            (boundary(cx, step_x, self.meta.origin_x) - x) / dx
        } else {
            f64::INFINITY
        };
        let mut t_max_y = if dy.abs() > 1e-12 {
            (boundary(cy, step_y, self.meta.origin_y) - y) / dy
        } else {
            f64::INFINITY
        };
        let t_delta_x = if dx.abs() > 1e-12 { res / dx.abs() } else { f64::INFINITY };
        let t_delta_y = if dy.abs() > 1e-12 { res / dy.abs() } else { f64::INFINITY };

        let mut t = 0.0;
        loop {
            if t > max_range {
                return None;
            }
            let cell = self.cell_checked(cx, cy)?;
            if self.get(cell) >= occupied_threshold {
                return Some(RayHit {
                    x: x + dx * t,
                    y: y + dy * t,
                    range: t,
                });
            }
            if t_max_x < t_max_y {
                t = t_max_x;
                t_max_x += t_delta_x;
                cx += step_x;
            } else {
                t = t_max_y;
                t_max_y += t_delta_y;
                cy += step_y;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn room() -> Grid {
        let meta = GridMeta {
            width: 200,
            height: 200,
            resolution: 0.05,
            origin_x: -5.0,
            origin_y: -5.0,
        };
        let mut g = Grid::filled(meta, 0);
        for cy in 0..200 {
            for x in [2.0, 2.05] {
                let c = g.world_to_cell(x + 0.01, -5.0 + cy as f64 * 0.05 + 0.01).unwrap();
                g.set(c, 100);
            }
        }
        g
    }

    #[test]
    fn world_cell_roundtrip() {
        let g = room();
        let c = g.world_to_cell(0.12, -0.33).unwrap();
        let (x, y) = g.cell_to_world(c);
        assert!((x - 0.125).abs() < 1e-9 && (y + 0.325).abs() < 1e-9);
        assert!(g.world_to_cell(5.01, 0.0).is_none());
        assert!(g.world_to_cell(-5.01, 0.0).is_none());
    }

    #[test]
    fn raycast_hits_wall_surface() {
        let g = room();
        let hit = g.raycast(0.0, 0.0, 1.0, 0.0, 8.0, 65).unwrap();
        assert!((hit.x - 2.0).abs() < 1e-9, "{hit:?}");
        assert!((hit.range - 2.0).abs() < 1e-9);
    }

    #[test]
    fn raycast_diagonal() {
        let g = room();
        let hit = g.raycast(0.0, 0.0, 1.0, 1.0, 8.0, 65).unwrap();
        assert!((hit.x - 2.0).abs() < 1e-9 && (hit.y - 2.0).abs() < 1e-9, "{hit:?}");
    }

    #[test]
    fn raycast_misses_backwards_and_out_of_range() {
        let g = room();
        assert!(g.raycast(0.0, 0.0, -1.0, 0.0, 8.0, 65).is_none());
        assert!(g.raycast(0.0, 0.0, 1.0, 0.0, 1.5, 65).is_none());
        assert!(g.raycast(0.0, 0.0, 0.0, 0.0, 8.0, 65).is_none());
    }

    #[test]
    fn raycast_passes_unknown() {
        let mut g = room();
        for cx in 10..20 {
            let c = g.world_to_cell(cx as f64 * 0.05 + 0.01, 0.01).unwrap();
            g.set(c, UNKNOWN);
        }
        assert!(g.raycast(0.0, 0.0, 1.0, 0.0, 8.0, 65).is_some());
    }
}
