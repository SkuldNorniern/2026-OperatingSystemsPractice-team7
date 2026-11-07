// Visits every cell the segment a->b (grid units, cell (i,j) spans [i,i+1)x[j,j+1)) passes through,
// with the length (grid units) inside each. Cells only touched at a corner or along an edge get len 0.
// Returns false as soon as `f` does.
pub(crate) fn walk(a: (f64, f64), b: (f64, f64), mut f: impl FnMut(i64, i64, f64) -> bool) -> bool {
    const EPS: f64 = 1e-9;
    let (dx, dy) = (b.0 - a.0, b.1 - a.1);
    let len = dx.hypot(dy);
    let (mut cx, mut cy) = (a.0.floor() as i64, a.1.floor() as i64);
    let (ex, ey) = (b.0.floor() as i64, b.1.floor() as i64);
    let sx: i64 = if dx > 0.0 { 1 } else { -1 };
    let sy: i64 = if dy > 0.0 { 1 } else { -1 };
    let first_t = |p: f64, c: i64, d: f64| {
        if d > 0.0 {
            (c as f64 + 1.0 - p) / d
        } else if d < 0.0 {
            (p - c as f64) / -d
        } else {
            f64::INFINITY
        }
    };
    let mut tmx = first_t(a.0, cx, dx);
    let mut tmy = first_t(a.1, cy, dy);
    let tdx = if dx != 0.0 {
        1.0 / dx.abs()
    } else {
        f64::INFINITY
    };
    let tdy = if dy != 0.0 {
        1.0 / dy.abs()
    } else {
        f64::INFINITY
    };
    // segment lying exactly on a grid line also touches the cells on the other side
    let on_hline = dy == 0.0 && a.1.fract() == 0.0;
    let on_vline = dx == 0.0 && a.0.fract() == 0.0;
    let mut steps = (ex - cx).abs() + (ey - cy).abs() + 2;
    let mut t = 0.0f64;
    loop {
        if (on_hline && !f(cx, cy - 1, 0.0)) || (on_vline && !f(cx - 1, cy, 0.0)) {
            return false;
        }
        let tn = tmx.min(tmy);
        if (cx == ex && cy == ey) || tn >= 1.0 - EPS || steps <= 0 {
            return f(cx, cy, (1.0 - t).max(0.0) * len);
        }
        steps -= 1;
        if !f(cx, cy, (tn - t) * len) {
            return false;
        }
        if (tmx - tmy).abs() < EPS {
            if !f(cx + sx, cy, 0.0) || !f(cx, cy + sy, 0.0) {
                return false;
            }
            cx += sx;
            cy += sy;
            tmx += tdx;
            tmy += tdy;
            steps -= 1;
        } else if tmx < tmy {
            cx += sx;
            tmx += tdx;
        } else {
            cy += sy;
            tmy += tdy;
        }
        t = tn;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cells(a: (f64, f64), b: (f64, f64)) -> (Vec<(i64, i64, f64)>, f64) {
        let mut v = Vec::new();
        walk(a, b, |x, y, l| {
            v.push((x, y, l));
            true
        });
        let total = v.iter().map(|c| c.2).sum();
        (v, total)
    }

    #[test]
    fn straight_and_lengths() {
        let (v, total) = cells((0.5, 0.5), (3.5, 0.5));
        assert_eq!(
            v.iter().map(|c| (c.0, c.1)).collect::<Vec<_>>(),
            [(0, 0), (1, 0), (2, 0), (3, 0)]
        );
        assert!((total - 3.0).abs() < 1e-9);
    }

    #[test]
    fn diagonal_touches_corner_cells() {
        let (v, total) = cells((0.5, 0.5), (2.5, 2.5));
        let set: Vec<(i64, i64)> = v.iter().map(|c| (c.0, c.1)).collect();
        for c in [(0, 0), (1, 0), (0, 1), (1, 1), (2, 1), (1, 2), (2, 2)] {
            assert!(set.contains(&c), "{c:?} missing in {set:?}");
        }
        assert!((total - 8f64.sqrt()).abs() < 1e-9);
    }

    #[test]
    fn general_supercover_is_connected() {
        let (v, total) = cells((0.5, 0.5), (7.5, 3.5));
        assert!((total - 58f64.sqrt()).abs() < 1e-9);
        for w in v.windows(2) {
            assert!((w[0].0 - w[1].0).abs() + (w[0].1 - w[1].1).abs() <= 2);
        }
        let (v, _) = cells((5.5, 4.5), (0.5, 0.5));
        assert_eq!((v[0].0, v[0].1), (5, 4));
        assert_eq!((v.last().unwrap().0, v.last().unwrap().1), (0, 0));
    }
}
