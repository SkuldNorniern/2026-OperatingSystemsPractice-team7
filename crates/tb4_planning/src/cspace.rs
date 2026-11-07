use tb4_core::grid::UNKNOWN;
use tb4_core::{Cell, DistanceField, Grid, GridMeta};

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CSpaceParams {
    pub robot_radius: f64,
    pub safety_margin: f64,
    pub occupied_threshold: i8,
    pub unknown_is_obstacle: bool,
    pub cost_scaling_dist: f64,
    pub cost_weight: f64,
}

impl Default for CSpaceParams {
    fn default() -> Self {
        Self {
            robot_radius: 0.18,
            safety_margin: 0.05,
            occupied_threshold: 65,
            unknown_is_obstacle: false,
            cost_scaling_dist: 0.5,
            cost_weight: 2.0,
        }
    }
}

#[derive(Debug, Clone)]
pub struct CSpace {
    pub meta: GridMeta,
    pub params: CSpaceParams,
    pub dist: DistanceField,
    pub(crate) cost: Vec<f32>,
    pub(crate) unknown: Vec<bool>,
    // 8-connected (no corner cutting) component labels, 0 = not traversable: [known only, unknown allowed]
    comp: [Vec<u32>; 2],
}

impl CSpace {
    pub fn new(grid: &Grid, p: &CSpaceParams) -> Self {
        let dist = DistanceField::from_grid(grid, p.occupied_threshold, p.unknown_is_obstacle);
        let cost = dist
            .dist
            .iter()
            .map(|&d| cost_multiplier(d as f64, p))
            .collect();
        let unknown = grid.data.iter().map(|&v| v == UNKNOWN).collect();
        let mut cs = Self {
            meta: grid.meta,
            params: *p,
            dist,
            cost,
            unknown,
            comp: [Vec::new(), Vec::new()],
        };
        cs.comp = [cs.label(false), cs.label(true)];
        cs
    }

    pub fn components(&self, allow_unknown: bool) -> &[u32] {
        &self.comp[allow_unknown as usize]
    }

    /// true if a plan between the two cells can exist
    pub fn connected(&self, a: Cell, b: Cell, allow_unknown: bool) -> bool {
        let c = self.components(allow_unknown);
        let (la, lb) = (c[self.index(a)], c[self.index(b)]);
        la != 0 && la == lb
    }

    fn label(&self, allow_unknown: bool) -> Vec<u32> {
        let (w, h) = (self.width() as i64, self.height() as i64);
        let pass = |x: i64, y: i64| {
            x >= 0 && y >= 0 && x < w && y < h && {
                let i = (y * w + x) as usize;
                self.cost[i].is_finite() && (allow_unknown || !self.unknown[i])
            }
        };
        let mut lab = vec![0u32; self.cost.len()];
        let mut next = 0u32;
        let mut stack = Vec::new();
        for i0 in 0..lab.len() {
            let (x0, y0) = (i0 as i64 % w, i0 as i64 / w);
            if lab[i0] != 0 || !pass(x0, y0) {
                continue;
            }
            next += 1;
            lab[i0] = next;
            stack.push((x0, y0));
            while let Some((x, y)) = stack.pop() {
                for (dx, dy) in [
                    (1, 0),
                    (-1, 0),
                    (0, 1),
                    (0, -1),
                    (1, 1),
                    (1, -1),
                    (-1, 1),
                    (-1, -1),
                ] {
                    let (nx, ny) = (x + dx, y + dy);
                    if !pass(nx, ny) || (dx != 0 && dy != 0 && (!pass(nx, y) || !pass(x, ny))) {
                        continue;
                    }
                    let j = (ny * w + nx) as usize;
                    if lab[j] == 0 {
                        lab[j] = next;
                        stack.push((nx, ny));
                    }
                }
            }
        }
        lab
    }

    #[inline]
    pub fn width(&self) -> usize {
        self.meta.width as usize
    }

    #[inline]
    pub fn height(&self) -> usize {
        self.meta.height as usize
    }

    #[inline]
    pub fn index(&self, c: Cell) -> usize {
        c.y as usize * self.width() + c.x as usize
    }

    pub fn is_free(&self, c: Cell) -> bool {
        self.cost[self.index(c)].is_finite()
    }

    pub fn is_unknown(&self, c: Cell) -> bool {
        self.unknown[self.index(c)]
    }

    /// traversal cost multiplier (>= 1), infinite for lethal cells; unknown handling is up to the planner
    pub fn cost(&self, c: Cell) -> f32 {
        self.cost[self.index(c)]
    }

    pub fn clearance_at(&self, x: f64, y: f64) -> f64 {
        self.dist.at(x, y) as f64
    }

    pub fn world_to_cell(&self, x: f64, y: f64) -> Option<Cell> {
        let (gx, gy) = self.to_grid(x, y);
        let (cx, cy) = (gx.floor() as i64, gy.floor() as i64);
        (cx >= 0 && cy >= 0 && cx < self.meta.width as i64 && cy < self.meta.height as i64)
            .then_some(Cell {
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

    #[inline]
    pub(crate) fn to_grid(&self, x: f64, y: f64) -> (f64, f64) {
        let m = &self.meta;
        (
            (x - m.origin_x) / m.resolution,
            (y - m.origin_y) / m.resolution,
        )
    }
}

fn cost_multiplier(d: f64, p: &CSpaceParams) -> f32 {
    if d < p.robot_radius {
        return f32::INFINITY;
    }
    let inner = p.robot_radius + p.safety_margin;
    let s = if d <= inner {
        1.0
    } else if d >= p.cost_scaling_dist || p.cost_scaling_dist <= inner {
        0.0
    } else {
        let u = (p.cost_scaling_dist - d) / (p.cost_scaling_dist - inner);
        u * u
    };
    (1.0 + p.cost_weight * s) as f32
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn multiplier_profile() {
        let p = CSpaceParams::default();
        assert!(cost_multiplier(0.1, &p).is_infinite());
        assert_eq!(cost_multiplier(0.2, &p), 3.0);
        let a = cost_multiplier(0.3, &p);
        let b = cost_multiplier(0.4, &p);
        assert!(a > b && b > 1.0);
        assert_eq!(cost_multiplier(0.6, &p), 1.0);
        assert_eq!(cost_multiplier(f64::INFINITY, &p), 1.0);
    }
}
