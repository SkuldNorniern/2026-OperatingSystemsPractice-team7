use crate::geom::{P2, corners_from_lines, dist, sub};
use crate::image::GrayImage;

const STEP: f64 = 0.25;
const MAX_PROFILE: usize = 64;

// Fits each edge line to the strongest dark-to-light gradient along its outward normal and
// re-intersects the lines. `q` must have positive orientation.
pub(crate) fn refine_quad(
    img: &GrayImage,
    q: &[P2; 4],
    cell: f64,
    pts: &mut Vec<(P2, f64)>,
) -> Option<[P2; 4]> {
    let radius = (0.6 * cell).clamp(1.0, 3.0);
    let mut lines = [([0.0; 2], [0.0; 2]); 4];
    for (i, line) in lines.iter_mut().enumerate() {
        let a = q[i];
        let b = q[(i + 1) % 4];
        let e = sub(b, a);
        let len = e[0].hypot(e[1]);
        if len < 2.0 {
            return None;
        }
        let d = [e[0] / len, e[1] / len];
        let n = [d[1], -d[0]];
        let samples = (len as usize).clamp(4, 48);
        pts.clear();
        for s in 0..samples {
            let t = 0.15 + 0.7 * (s as f64 + 0.5) / samples as f64;
            let p = [a[0] + t * e[0], a[1] + t * e[1]];
            if let Some((off, w)) = edge_offset(img, p, n, radius) {
                pts.push(([p[0] + off * n[0], p[1] + off * n[1]], w));
            }
        }
        *line = if pts.len() >= 3 {
            fit_line(pts).unwrap_or((a, d))
        } else {
            (a, d)
        };
    }
    let out = corners_from_lines(&lines)?;
    let limit = 0.5 * cell + 1.5;
    out.iter()
        .zip(q)
        .all(|(o, p)| dist(*o, *p) <= limit)
        .then_some(out)
}

fn edge_offset(img: &GrayImage, p: P2, n: P2, radius: f64) -> Option<(f64, f64)> {
    let m = ((2.0 * radius / STEP) as usize + 1).min(MAX_PROFILE);
    let mut prof = [0.0f64; MAX_PROFILE];
    for (k, v) in prof.iter_mut().enumerate().take(m) {
        let s = -radius + k as f64 * STEP;
        *v = img.sample(p[0] + s * n[0], p[1] + s * n[1]);
    }
    let mut best = (0usize, 0.0f64);
    for k in 1..m - 1 {
        let g = prof[k + 1] - prof[k - 1];
        if g > best.1 {
            best = (k, g);
        }
    }
    let (k, g) = best;
    if k == 0 || g < 4.0 {
        return None;
    }
    // Gradient-weighted centroid around the peak copes with flat-topped peaks on sharp edges.
    let half = (1.0 / STEP) as usize;
    let (mut sw, mut sj) = (0.0, 0.0);
    for j in k.saturating_sub(half).max(1)..(k + half + 1).min(m - 1) {
        let gj = prof[j + 1] - prof[j - 1];
        if gj > 0.5 * g {
            sw += gj;
            sj += gj * j as f64;
        }
    }
    let off = sj / sw;
    Some((-radius + off * STEP, g))
}

// Weighted total least squares line fit, returns (centroid, unit direction).
fn fit_line(pts: &[(P2, f64)]) -> Option<(P2, P2)> {
    let wsum: f64 = pts.iter().map(|p| p.1).sum();
    if wsum <= 0.0 {
        return None;
    }
    let mx = pts.iter().map(|(p, w)| p[0] * w).sum::<f64>() / wsum;
    let my = pts.iter().map(|(p, w)| p[1] * w).sum::<f64>() / wsum;
    let (mut sxx, mut sxy, mut syy) = (0.0, 0.0, 0.0);
    for (p, w) in pts {
        let dx = p[0] - mx;
        let dy = p[1] - my;
        sxx += w * dx * dx;
        sxy += w * dx * dy;
        syy += w * dy * dy;
    }
    let theta = 0.5 * (2.0 * sxy).atan2(sxx - syy);
    Some(([mx, my], [theta.cos(), theta.sin()]))
}
