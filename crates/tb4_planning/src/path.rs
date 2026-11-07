use crate::cspace::CSpace;
use crate::walk::walk;

pub fn path_length(points: &[(f64, f64)]) -> f64 {
    points
        .windows(2)
        .map(|w| (w[1].0 - w[0].0).hypot(w[1].1 - w[0].1))
        .sum()
}

/// points spaced at most `step_m` apart, keeping the original vertices
pub fn densify(points: &[(f64, f64)], step_m: f64) -> Vec<(f64, f64)> {
    let Some(&first) = points.first() else {
        return Vec::new();
    };
    let mut out = vec![first];
    for w in points.windows(2) {
        let (a, b) = (w[0], w[1]);
        let d = (b.0 - a.0).hypot(b.1 - a.1);
        let n = if step_m > 0.0 {
            (d / step_m).ceil().max(1.0) as usize
        } else {
            1
        };
        for k in 1..=n {
            let t = k as f64 / n as f64;
            out.push((a.0 + (b.0 - a.0) * t, a.1 + (b.1 - a.1) * t));
        }
    }
    out
}

/// minimum obstacle distance over every grid cell the polyline passes through (0 if it leaves the map)
pub fn path_clearance_min(cs: &CSpace, points: &[(f64, f64)]) -> f64 {
    let mut min = f64::INFINITY;
    let mut visit = |x: i64, y: i64, _l: f64| {
        min = min.min(cell_dist(cs, x, y));
        true
    };
    if points.len() == 1 {
        let g = cs.to_grid(points[0].0, points[0].1);
        visit(g.0.floor() as i64, g.1.floor() as i64, 0.0);
    }
    for w in points.windows(2) {
        walk(
            cs.to_grid(w[0].0, w[0].1),
            cs.to_grid(w[1].0, w[1].1),
            &mut visit,
        );
    }
    min
}

/// true if no cell the segment passes through (including corner-touched cells) is lethal or outside the map
pub fn segment_free(cs: &CSpace, a: (f64, f64), b: (f64, f64)) -> bool {
    walk(cs.to_grid(a.0, a.1), cs.to_grid(b.0, b.1), |x, y, _| {
        x >= 0
            && y >= 0
            && x < cs.width() as i64
            && y < cs.height() as i64
            && cs.cost[y as usize * cs.width() + x as usize].is_finite()
    })
}

/// Chaikin-style corner cutting; a cut is kept only if the new chord stays out of lethal cells.
/// Endpoints are preserved. `max_cut` caps how far (m) a corner is cut back along each segment.
pub fn smooth(
    cs: &CSpace,
    points: &[(f64, f64)],
    iterations: usize,
    max_cut: f64,
) -> Vec<(f64, f64)> {
    let mut pts = points.to_vec();
    for _ in 0..iterations {
        if pts.len() < 3 {
            break;
        }
        let mut out = vec![pts[0]];
        for k in 1..pts.len() - 1 {
            let (a, b, c) = (pts[k - 1], pts[k], pts[k + 1]);
            let cut = |from: (f64, f64)| {
                let d = (from.0 - b.0).hypot(from.1 - b.1);
                let t = if d > 0.0 {
                    (0.25 * d).min(max_cut) / d
                } else {
                    0.0
                };
                (b.0 + (from.0 - b.0) * t, b.1 + (from.1 - b.1) * t)
            };
            let (p, q) = (cut(a), cut(c));
            if segment_free(cs, p, q) {
                out.push(p);
                out.push(q);
            } else {
                out.push(b);
            }
        }
        out.push(*pts.last().unwrap());
        out.dedup_by(|x, y| (x.0 - y.0).hypot(x.1 - y.1) < 1e-9);
        pts = out;
    }
    pts
}

fn cell_dist(cs: &CSpace, x: i64, y: i64) -> f64 {
    if x < 0 || y < 0 || x >= cs.width() as i64 || y >= cs.height() as i64 {
        return 0.0;
    }
    cs.dist.dist[y as usize * cs.width() + x as usize] as f64
}
