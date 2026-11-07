pub(crate) type P2 = [f64; 2];

#[derive(Clone, Copy, Debug)]
pub(crate) struct Homography(pub [f64; 9]);

impl Homography {
    // Maps the unit square (0,0),(1,0),(1,1),(0,1) onto q[0..4].
    pub fn from_unit_square(q: &[P2; 4]) -> Option<Self> {
        let [[x0, y0], [x1, y1], [x2, y2], [x3, y3]] = *q;
        let sx = x0 - x1 + x2 - x3;
        let sy = y0 - y1 + y2 - y3;
        let (dx1, dx2, dy1, dy2) = (x1 - x2, x3 - x2, y1 - y2, y3 - y2);
        let den = dx1 * dy2 - dx2 * dy1;
        if den.abs() < 1e-12 {
            return None;
        }
        let g = (sx * dy2 - dx2 * sy) / den;
        let h = (dx1 * sy - sx * dy1) / den;
        Some(Self([
            x1 - x0 + g * x1,
            x3 - x0 + h * x3,
            x0,
            y1 - y0 + g * y1,
            y3 - y0 + h * y3,
            y0,
            g,
            h,
            1.0,
        ]))
    }

    #[inline]
    pub fn map(&self, u: f64, v: f64) -> P2 {
        let m = &self.0;
        let w = m[6] * u + m[7] * v + m[8];
        [
            (m[0] * u + m[1] * v + m[2]) / w,
            (m[3] * u + m[4] * v + m[5]) / w,
        ]
    }

    pub fn inverse(&self) -> Option<Self> {
        let m = &self.0;
        let c0 = m[4] * m[8] - m[5] * m[7];
        let c1 = m[5] * m[6] - m[3] * m[8];
        let c2 = m[3] * m[7] - m[4] * m[6];
        let det = m[0] * c0 + m[1] * c1 + m[2] * c2;
        if det.abs() < 1e-15 {
            return None;
        }
        let i = 1.0 / det;
        Some(Self([
            c0 * i,
            (m[2] * m[7] - m[1] * m[8]) * i,
            (m[1] * m[5] - m[2] * m[4]) * i,
            c1 * i,
            (m[0] * m[8] - m[2] * m[6]) * i,
            (m[2] * m[3] - m[0] * m[5]) * i,
            c2 * i,
            (m[1] * m[6] - m[0] * m[7]) * i,
            (m[0] * m[4] - m[1] * m[3]) * i,
        ]))
    }
}

#[inline]
pub(crate) fn sub(a: P2, b: P2) -> P2 {
    [a[0] - b[0], a[1] - b[1]]
}

#[inline]
pub(crate) fn cross(a: P2, b: P2) -> f64 {
    a[0] * b[1] - a[1] * b[0]
}

#[inline]
pub(crate) fn dist(a: P2, b: P2) -> f64 {
    let d = sub(a, b);
    d[0].hypot(d[1])
}

// Positive when the quad runs TL -> TR -> BR -> BL in image coordinates (y down).
pub(crate) fn signed_area(q: &[P2; 4]) -> f64 {
    let mut s = 0.0;
    for i in 0..4 {
        s += cross(q[i], q[(i + 1) % 4]);
    }
    0.5 * s
}

pub(crate) fn is_convex(q: &[P2; 4]) -> bool {
    (0..4).all(|i| {
        cross(
            sub(q[(i + 1) % 4], q[i]),
            sub(q[(i + 2) % 4], q[(i + 1) % 4]),
        ) > 0.0
    })
}

// Line through two points as (point, unit direction).
pub(crate) fn intersect(p: P2, d: P2, q: P2, e: P2) -> Option<P2> {
    let den = cross(d, e);
    if den.abs() < 1e-9 {
        return None;
    }
    let t = cross(sub(q, p), e) / den;
    Some([p[0] + t * d[0], p[1] + t * d[1]])
}

pub(crate) fn diag_intersection(q: &[P2; 4]) -> P2 {
    intersect(q[0], sub(q[2], q[0]), q[1], sub(q[3], q[1])).unwrap_or([
        (q[0][0] + q[1][0] + q[2][0] + q[3][0]) / 4.0,
        (q[0][1] + q[1][1] + q[2][1] + q[3][1]) / 4.0,
    ])
}

pub(crate) fn point_segment_dist(p: P2, a: P2, b: P2) -> f64 {
    let ab = sub(b, a);
    let l2 = ab[0] * ab[0] + ab[1] * ab[1];
    let t = if l2 > 0.0 {
        ((p[0] - a[0]) * ab[0] + (p[1] - a[1]) * ab[1]) / l2
    } else {
        0.0
    };
    let t = t.clamp(0.0, 1.0);
    dist(p, [a[0] + t * ab[0], a[1] + t * ab[1]])
}

// Requires positive orientation (see signed_area).
pub(crate) fn inside_quad(p: P2, q: &[P2; 4]) -> bool {
    (0..4).all(|i| cross(sub(q[(i + 1) % 4], q[i]), sub(p, q[i])) >= 0.0)
}

// Shifts every edge outward by `d` pixels; quad must have positive orientation.
pub(crate) fn offset_quad(q: &[P2; 4], d: f64) -> Option<[P2; 4]> {
    let mut lines = [([0.0; 2], [0.0; 2]); 4];
    for (i, line) in lines.iter_mut().enumerate() {
        let a = q[i];
        let e = sub(q[(i + 1) % 4], a);
        let l = e[0].hypot(e[1]);
        if l < 1e-9 {
            return None;
        }
        let n = [e[1] / l, -e[0] / l];
        *line = ([a[0] + d * n[0], a[1] + d * n[1]], e);
    }
    corners_from_lines(&lines)
}

pub(crate) fn corners_from_lines(lines: &[(P2, P2); 4]) -> Option<[P2; 4]> {
    let mut out = [[0.0; 2]; 4];
    for (i, c) in out.iter_mut().enumerate() {
        let (p, d) = lines[(i + 3) % 4];
        let (q, e) = lines[i];
        *c = intersect(p, d, q, e)?;
    }
    Some(out)
}

// Andrew's monotone chain; `pts` is sorted in place. Output has no repeated end point.
pub(crate) fn convex_hull(pts: &mut [(i32, i32)], out: &mut Vec<P2>) {
    out.clear();
    pts.sort_unstable();
    let n = pts.len();
    if n < 3 {
        out.extend(pts.iter().map(|&(x, y)| [x as f64, y as f64]));
        return;
    }
    let turn = |o: P2, a: P2, b: P2| cross(sub(a, o), sub(b, o));
    for &(x, y) in pts.iter() {
        let p = [x as f64, y as f64];
        while out.len() >= 2 && turn(out[out.len() - 2], out[out.len() - 1], p) <= 0.0 {
            out.pop();
        }
        out.push(p);
    }
    let lower = out.len() + 1;
    for &(x, y) in pts.iter().rev().skip(1) {
        let p = [x as f64, y as f64];
        while out.len() >= lower && turn(out[out.len() - 2], out[out.len() - 1], p) <= 0.0 {
            out.pop();
        }
        out.push(p);
    }
    out.pop();
}

fn tri2(a: P2, b: P2, c: P2) -> f64 {
    cross(sub(b, a), sub(c, a)).abs()
}

// Largest-area quadrilateral with vertices on a convex polygon, O(n^2).
pub(crate) fn max_area_quad(h: &[P2]) -> Option<[usize; 4]> {
    let n = h.len();
    if n < 4 {
        return None;
    }
    let at = |i: usize| h[i % n];
    let mut best = (0.0, [0usize; 4]);
    for i in 0..n {
        let mut j = i + 1;
        let mut l = i + 3;
        for k in i + 2..i + n - 1 {
            if j >= k {
                j = k - 1;
            }
            while j + 1 < k && tri2(at(i), at(j + 1), at(k)) >= tri2(at(i), at(j), at(k)) {
                j += 1;
            }
            if l <= k {
                l = k + 1;
            }
            while l + 1 < i + n && tri2(at(k), at(l + 1), at(i)) >= tri2(at(k), at(l), at(i)) {
                l += 1;
            }
            let a = tri2(at(i), at(j), at(k)) + tri2(at(k), at(l), at(i));
            if a > best.0 {
                best = (a, [i % n, j % n, k % n, l % n]);
            }
        }
    }
    (best.0 > 0.0).then_some(best.1)
}
