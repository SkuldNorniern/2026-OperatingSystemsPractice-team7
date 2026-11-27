//! Forbidden polygons (map frame, m): areas the planner and MPPI must never enter, e.g. a glass
//! wall that the lidar sees through. Cells whose centre lies inside a polygon are stamped occupied
//! into the grid the planner and MPPI use, the same way the bump and IR marks are.

use tb4_core::{Cell, Grid};

#[derive(Debug, Clone, Default, PartialEq)]
pub struct ForbiddenZones {
    polys: Vec<Vec<(f64, f64)>>,
}

impl ForbiddenZones {
    pub fn new(polys: Vec<Vec<(f64, f64)>>) -> Self {
        Self { polys }
    }

    /// Parse config entries, one polygon per entry: "x1,y1 x2,y2 x3,y3 ..." (at least 3 vertices).
    pub fn parse(entries: &[String]) -> Result<Self, String> {
        let mut polys = Vec::with_capacity(entries.len());
        for e in entries {
            let mut poly = Vec::new();
            for tok in e.split_whitespace() {
                let (x, y) = tok.split_once(',').ok_or_else(|| format!("bad vertex '{tok}' in '{e}'"))?;
                let x: f64 = x.parse().map_err(|_| format!("bad number '{x}' in '{e}'"))?;
                let y: f64 = y.parse().map_err(|_| format!("bad number '{y}' in '{e}'"))?;
                poly.push((x, y));
            }
            if poly.len() < 3 {
                return Err(format!("polygon needs at least 3 vertices: '{e}'"));
            }
            polys.push(poly);
        }
        Ok(Self { polys })
    }

    pub fn is_empty(&self) -> bool {
        self.polys.is_empty()
    }

    pub fn contains(&self, x: f64, y: f64) -> bool {
        self.polys.iter().any(|p| inside(p, x, y))
    }

    /// Write `value` into every cell of `grid` whose centre is inside a polygon. Returns cells set.
    pub fn stamp(&self, grid: &mut Grid, value: i8) -> usize {
        let mut n = 0;
        for poly in &self.polys {
            let (mut x0, mut y0, mut x1, mut y1) = (f64::MAX, f64::MAX, f64::MIN, f64::MIN);
            for &(x, y) in poly {
                x0 = x0.min(x);
                y0 = y0.min(y);
                x1 = x1.max(x);
                y1 = y1.max(y);
            }
            let (cx0, cy0) = grid.world_to_cell_signed(x0, y0);
            let (cx1, cy1) = grid.world_to_cell_signed(x1, y1);
            let w = grid.meta.width as i64;
            let h = grid.meta.height as i64;
            for cy in cy0.max(0)..=cy1.min(h - 1) {
                for cx in cx0.max(0)..=cx1.min(w - 1) {
                    let c = Cell { x: cx as u32, y: cy as u32 };
                    let (wx, wy) = grid.cell_to_world(c);
                    if inside(poly, wx, wy) {
                        grid.set(c, value);
                        n += 1;
                    }
                }
            }
        }
        n
    }
}

/// Even-odd ray casting.
fn inside(poly: &[(f64, f64)], x: f64, y: f64) -> bool {
    let mut hit = false;
    let mut j = poly.len() - 1;
    for i in 0..poly.len() {
        let (xi, yi) = poly[i];
        let (xj, yj) = poly[j];
        if (yi > y) != (yj > y) && x < (xj - xi) * (y - yi) / (yj - yi) + xi {
            hit = !hit;
        }
        j = i;
    }
    hit
}

#[cfg(test)]
mod tests {
    use super::*;
    use tb4_core::grid::GridMeta;

    fn meta() -> GridMeta {
        GridMeta { width: 40, height: 40, resolution: 0.05, origin_x: -1.0, origin_y: -1.0 }
    }

    #[test]
    fn parses_polygons_and_rejects_bad_ones() {
        let z = ForbiddenZones::parse(&["0,0 1,0 1,1 0,1".to_string()]).unwrap();
        assert!(z.contains(0.5, 0.5));
        assert!(!z.contains(1.5, 0.5));
        assert!(ForbiddenZones::parse(&["0,0 1,0".to_string()]).is_err(), "two vertices");
        assert!(ForbiddenZones::parse(&["0,0 1 1,1".to_string()]).is_err(), "no comma");
        assert!(ForbiddenZones::parse(&["0,0 x,0 1,1".to_string()]).is_err(), "not a number");
        assert!(ForbiddenZones::parse(&[]).unwrap().is_empty());
    }

    #[test]
    fn stamp_marks_only_cells_inside_the_polygon() {
        // triangle with its vertices outside the grid corner: cells inside are marked, others untouched
        let z = ForbiddenZones::new(vec![vec![(-0.5, -0.5), (0.5, -0.5), (-0.5, 0.5)]]);
        let mut g = Grid::filled(meta(), 0);
        let n = z.stamp(&mut g, 100);
        assert!(n > 50, "about half of a 1 m square: {n}");
        assert_eq!(g.get(g.world_to_cell(-0.4, -0.4).unwrap()), 100, "inside");
        assert_eq!(g.get(g.world_to_cell(0.4, 0.4).unwrap()), 0, "outside the hypotenuse");
        assert_eq!(g.get(g.world_to_cell(0.8, 0.0).unwrap()), 0, "far outside");
    }

    #[test]
    fn polygon_partly_off_the_grid_is_clipped() {
        let z = ForbiddenZones::new(vec![vec![(0.5, 0.5), (5.0, 0.5), (5.0, 5.0), (0.5, 5.0)]]);
        let mut g = Grid::filled(meta(), 0);
        assert!(z.stamp(&mut g, 100) > 0);
        assert_eq!(g.get(g.world_to_cell(0.9, 0.9).unwrap()), 100);
    }
}
