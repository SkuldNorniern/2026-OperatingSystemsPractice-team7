#[derive(Debug, Clone, Default)]
pub(crate) struct RefPath {
    pub pts: Vec<(f64, f64)>,
    /// arc length at each point
    pub s: Vec<f64>,
    /// index of the point `heading_lookahead` further along
    pub ahead: Vec<usize>,
}

impl RefPath {
    pub fn new(path: &[(f64, f64)], resolution: f64, heading_lookahead: f64) -> Self {
        let mut pts: Vec<(f64, f64)> = Vec::with_capacity(path.len());
        for &p in path {
            let Some(&last) = pts.last() else {
                pts.push(p);
                continue;
            };
            let len = (p.0 - last.0).hypot(p.1 - last.1);
            if len < 1e-9 {
                continue;
            }
            let n = (len / resolution).ceil().max(1.0) as usize;
            for i in 1..=n {
                let f = i as f64 / n as f64;
                pts.push((last.0 + (p.0 - last.0) * f, last.1 + (p.1 - last.1) * f));
            }
        }
        let mut s = Vec::with_capacity(pts.len());
        let mut acc = 0.0;
        for (i, p) in pts.iter().enumerate() {
            if i > 0 {
                acc += (p.0 - pts[i - 1].0).hypot(p.1 - pts[i - 1].1);
            }
            s.push(acc);
        }
        let mut r = Self { pts, s, ahead: Vec::new() };
        r.ahead = r.s.iter().map(|&si| r.index_at(si + heading_lookahead)).collect();
        r
    }

    pub fn is_empty(&self) -> bool {
        self.pts.is_empty()
    }

    pub fn length(&self) -> f64 {
        self.s.last().copied().unwrap_or(0.0)
    }

    /// first index with arc length >= s
    pub fn index_at(&self, s: f64) -> usize {
        self.s.partition_point(|&x| x < s).min(self.pts.len().saturating_sub(1))
    }

    /// nearest point in [lo, hi): (index, squared distance)
    #[inline]
    pub fn nearest(&self, x: f64, y: f64, lo: usize, hi: usize) -> (usize, f64) {
        let mut best = (lo, f64::INFINITY);
        for (i, p) in self.pts[lo..hi].iter().enumerate() {
            let d2 = (p.0 - x) * (p.0 - x) + (p.1 - y) * (p.1 - y);
            if d2 < best.1 {
                best = (lo + i, d2);
            }
        }
        best
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn densify() {
        let p = RefPath::new(&[(0.0, 0.0), (1.0, 0.0), (1.0, 0.0), (1.0, 0.5)], 0.1, 0.25);
        assert_eq!(p.pts.len(), 16);
        assert!((p.length() - 1.5).abs() < 1e-9);
        assert_eq!(p.index_at(1.05), 11);
        assert_eq!(p.ahead[0], 3);
        assert_eq!(*p.ahead.last().unwrap(), 15);
        assert_eq!(p.nearest(0.52, 0.3, 0, 16).0, 5);
    }
}
