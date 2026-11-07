use crate::Marker;
use crate::dict::Dictionary;
use crate::geom::{
    Homography, P2, convex_hull, diag_intersection, dist, inside_quad, is_convex, max_area_quad,
    offset_quad, point_segment_dist, signed_area,
};
use crate::image::GrayImage;
use crate::refine::refine_quad;

#[derive(Clone, Debug, PartialEq)]
pub struct DetectorParams {
    // Odd adaptive threshold window sizes; each one is a separate candidate pass.
    pub thresh_windows: Vec<usize>,
    // A pixel is dark when it is below the local window mean minus this constant.
    pub thresh_constant: i32,
    pub min_side_px: f64,
    // Perimeter limits relative to max(width, height).
    pub min_perimeter_rate: f64,
    pub max_perimeter_rate: f64,
    // Max contour-to-quad distance: max(rate * perimeter, min_px).
    pub poly_eps_rate: f64,
    pub poly_eps_min_px: f64,
    pub min_corner_dist_rate: f64,
    pub max_aspect: f64,
    pub border_bits: usize,
    // Fraction of each cell ignored at its edges when sampling.
    pub cell_margin: f64,
    pub min_cell_contrast: f64,
    pub max_border_errors: u32,
    pub max_hamming: u32,
    pub refine_corners: bool,
    // Laplacian sharpening strengths tried on the cell grid to undo blur on small markers.
    pub sharpen_levels: Vec<f64>,
    // Border cells read as white must lie within this fraction of the contrast from the threshold.
    pub uncertain_rate: f64,
    // A corrected match must beat every other id's soft cost by this much (in contrast units).
    pub min_decode_margin: f64,
}

impl Default for DetectorParams {
    fn default() -> Self {
        Self {
            thresh_windows: vec![7, 23],
            thresh_constant: 7,
            min_side_px: 8.0,
            min_perimeter_rate: 0.03,
            max_perimeter_rate: 4.0,
            poly_eps_rate: 0.04,
            poly_eps_min_px: 1.2,
            min_corner_dist_rate: 0.1,
            max_aspect: 4.0,
            border_bits: 1,
            cell_margin: 0.35,
            min_cell_contrast: 20.0,
            max_border_errors: 2,
            max_hamming: 1,
            refine_corners: true,
            sharpen_levels: vec![0.0, 0.25, 0.5],
            uncertain_rate: 0.3,
            min_decode_margin: 0.5,
        }
    }
}

#[derive(Clone, Copy, Debug)]
struct Run {
    y: u32,
    x0: u32,
    x1: u32,
    parent: u32,
}

#[derive(Clone, Copy, Debug)]
struct Candidate {
    marker: Marker,
    quad: [P2; 4],
    border_errors: u32,
    contrast: f64,
}

impl Candidate {
    fn better_than(&self, o: &Candidate) -> bool {
        (self.marker.hamming, self.border_errors) < (o.marker.hamming, o.border_errors)
            || ((self.marker.hamming, self.border_errors) == (o.marker.hamming, o.border_errors)
                && self.contrast > o.contrast)
    }

    fn perfect(&self) -> bool {
        self.marker.hamming == 0 && self.border_errors == 0
    }
}

#[derive(Debug, Default)]
pub struct Detector {
    pub params: DetectorParams,
    integral: Vec<u32>,
    dark: Vec<u8>,
    runs: Vec<Run>,
    bbox: Vec<[u32; 4]>,
    contour: Vec<(i32, i32)>,
    hull: Vec<P2>,
    edge_pts: Vec<(P2, f64)>,
    cells: Vec<f64>,
    sharp: Vec<f64>,
    weights: Vec<f64>,
    rot_codes: Vec<[u64; 4]>,
    rot_src: Vec<u64>,
    rot_bits: usize,
    sorted: Vec<f64>,
    found: Vec<Candidate>,
}

const DX: [i32; 8] = [1, 1, 0, -1, -1, -1, 0, 1];
const DY: [i32; 8] = [0, 1, 1, 1, 0, -1, -1, -1];

impl Detector {
    pub fn new(params: DetectorParams) -> Self {
        Self {
            params,
            ..Default::default()
        }
    }

    pub fn detect(&mut self, img: &GrayImage, dict: &Dictionary) -> Vec<Marker> {
        let mut out = Vec::new();
        self.detect_into(img, dict, &mut out);
        out
    }

    pub fn detect_into(&mut self, img: &GrayImage, dict: &Dictionary, out: &mut Vec<Marker>) {
        out.clear();
        self.found.clear();
        let (w, h) = (img.width, img.height);
        if w < 3 || h < 3 || img.data.len() < w * h {
            return;
        }
        if self.rot_bits != dict.bits || self.rot_src != dict.codes {
            self.rot_bits = dict.bits;
            self.rot_src.clone_from(&dict.codes);
            self.rot_codes.clear();
            self.rot_codes.extend(dict.codes.iter().map(|&c| {
                let r1 = dict.rotate_cw(c);
                let r2 = dict.rotate_cw(r1);
                [c, r1, r2, dict.rotate_cw(r2)]
            }));
        }
        self.build_integral(img);
        for wi in 0..self.params.thresh_windows.len() {
            let win = self.params.thresh_windows[wi];
            self.threshold(img, win);
            self.label_runs(w, h);
            self.process_components(img, dict);
        }
        out.extend(self.found.iter().map(|c| c.marker));
    }

    fn build_integral(&mut self, img: &GrayImage) {
        let (w, h) = (img.width, img.height);
        let s = w + 1;
        self.integral.clear();
        self.integral.resize(s * (h + 1), 0);
        for y in 0..h {
            let mut acc = 0u32;
            let row = &img.data[y * w..(y + 1) * w];
            for (x, &v) in row.iter().enumerate() {
                acc += v as u32;
                self.integral[(y + 1) * s + x + 1] = self.integral[y * s + x + 1] + acc;
            }
        }
    }

    fn threshold(&mut self, img: &GrayImage, win: usize) {
        let (w, h) = (img.width, img.height);
        let s = w + 1;
        let r = win / 2;
        let c = self.params.thresh_constant;
        self.dark.clear();
        self.dark.resize(w * h, 0);
        let ii = &self.integral;
        for y in 0..h {
            let y0 = y.saturating_sub(r);
            let y1 = (y + r + 1).min(h);
            for x in 0..w {
                let x0 = x.saturating_sub(r);
                let x1 = (x + r + 1).min(w);
                let area = ((y1 - y0) * (x1 - x0)) as i64;
                let sum = (ii[y1 * s + x1] + ii[y0 * s + x0]) as i64
                    - (ii[y0 * s + x1] + ii[y1 * s + x0]) as i64;
                let v = img.data[y * w + x] as i64;
                self.dark[y * w + x] = ((v + c as i64) * area < sum) as u8;
            }
        }
    }

    fn find(runs: &mut [Run], mut i: u32) -> u32 {
        while runs[i as usize].parent != i {
            let p = runs[i as usize].parent;
            runs[i as usize].parent = runs[p as usize].parent;
            i = p;
        }
        i
    }

    // 8-connected components over horizontal runs; each root is the component's first run in raster order.
    fn label_runs(&mut self, w: usize, h: usize) {
        self.runs.clear();
        let mut prev = 0..0;
        for y in 0..h {
            let row = &self.dark[y * w..(y + 1) * w];
            let start = self.runs.len();
            let mut x = 0;
            while x < w {
                if row[x] == 0 {
                    x += 1;
                    continue;
                }
                let x0 = x;
                while x < w && row[x] != 0 {
                    x += 1;
                }
                let id = self.runs.len() as u32;
                self.runs.push(Run {
                    y: y as u32,
                    x0: x0 as u32,
                    x1: x as u32,
                    parent: id,
                });
            }
            let cur = start..self.runs.len();
            let mut j = prev.start;
            for i in cur.clone() {
                let (cx0, cx1) = (self.runs[i].x0, self.runs[i].x1);
                while j < prev.end && self.runs[j].x1 < cx0 {
                    j += 1;
                }
                let mut k = j;
                while k < prev.end && self.runs[k].x0 <= cx1 {
                    let a = Self::find(&mut self.runs, i as u32);
                    let b = Self::find(&mut self.runs, k as u32);
                    if a != b {
                        let (lo, hi) = if a < b { (a, b) } else { (b, a) };
                        self.runs[hi as usize].parent = lo;
                    }
                    k += 1;
                }
            }
            prev = cur;
        }
        let n = self.runs.len();
        self.bbox.clear();
        self.bbox.resize(n, [u32::MAX, u32::MAX, 0, 0]);
        for i in 0..n {
            let root = Self::find(&mut self.runs, i as u32) as usize;
            let r = self.runs[i];
            let b = &mut self.bbox[root];
            b[0] = b[0].min(r.x0);
            b[1] = b[1].min(r.y);
            b[2] = b[2].max(r.x1 - 1);
            b[3] = b[3].max(r.y);
        }
    }

    fn process_components(&mut self, img: &GrayImage, dict: &Dictionary) {
        let (w, h) = (img.width, img.height);
        let p = &self.params;
        let dim = w.max(h) as f64;
        let min_perim = (p.min_perimeter_rate * dim).max(4.0 * (p.min_side_px - 1.0));
        let max_perim = p.max_perimeter_rate * dim;
        let min_ext = (p.min_side_px - 1.0).max(2.0) as u32;
        for i in 0..self.runs.len() {
            if self.runs[i].parent != i as u32 {
                continue;
            }
            let b = self.bbox[i];
            let (bw, bh) = (b[2] - b[0] + 1, b[3] - b[1] + 1);
            if bw.max(bh) < min_ext || ((bw + bh) * 2) as f64 > max_perim * 1.5 {
                continue;
            }
            if b[0] == 0 || b[1] == 0 || b[2] as usize == w - 1 || b[3] as usize == h - 1 {
                continue;
            }
            let start = (self.runs[i].x0 as i32, self.runs[i].y as i32);
            if !self.trace(start, w, h, max_perim as usize + 8) {
                continue;
            }
            let perim = self.contour.len() as f64;
            if perim < min_perim || perim > max_perim {
                continue;
            }
            let Some(quad) = self.fit_quad(perim) else {
                continue;
            };
            let c = diag_intersection(&quad);
            if self
                .found
                .iter()
                .any(|f| f.perfect() && inside_quad(c, &f.quad))
            {
                continue;
            }
            if let Some(cand) = self.decode(img, dict, quad) {
                self.insert(cand);
            }
        }
    }

    // Moore-neighbour tracing of the outer border; false if it exceeds `max_len`.
    fn trace(&mut self, start: (i32, i32), w: usize, h: usize, max_len: usize) -> bool {
        let dark = &self.dark;
        let is_dark = |x: i32, y: i32| {
            x >= 0
                && y >= 0
                && (x as usize) < w
                && (y as usize) < h
                && dark[y as usize * w + x as usize] != 0
        };
        let next = |p: (i32, i32), s: usize| {
            (0..8)
                .map(|k| (s + k) % 8)
                .find(|&d| is_dark(p.0 + DX[d], p.1 + DY[d]))
        };
        self.contour.clear();
        self.contour.push(start);
        let Some(first) = next(start, 4) else {
            return true;
        };
        let mut d = first;
        let mut cur = start;
        loop {
            cur = (cur.0 + DX[d], cur.1 + DY[d]);
            let s = if d % 2 == 0 { (d + 6) % 8 } else { (d + 5) % 8 };
            let nd = next(cur, s).unwrap_or(d);
            if cur == start && nd == first {
                return true;
            }
            self.contour.push(cur);
            if self.contour.len() > max_len {
                return false;
            }
            d = nd;
        }
    }

    fn fit_quad(&mut self, perim: f64) -> Option<[P2; 4]> {
        let p = &self.params;
        let mut pts = std::mem::take(&mut self.contour);
        let sorted_len = pts.len();
        pts.extend_from_within(..);
        convex_hull(&mut pts[sorted_len..], &mut self.hull);
        pts.truncate(sorted_len);
        self.contour = pts;
        let idx = max_area_quad(&self.hull)?;
        let mut q = idx.map(|i| self.hull[i]);
        if signed_area(&q) < 0.0 {
            q.reverse();
        }
        let eps = (p.poly_eps_rate * perim).max(p.poly_eps_min_px);
        let fits = self.contour.iter().all(|&(x, y)| {
            let pt = [x as f64, y as f64];
            (0..4).any(|k| point_segment_dist(pt, q[k], q[(k + 1) % 4]) <= eps)
        });
        if !fits {
            return None;
        }
        let q = offset_quad(&q, 0.5)?;
        if !is_convex(&q) {
            return None;
        }
        let sides = [0, 1, 2, 3].map(|k| dist(q[k], q[(k + 1) % 4]));
        let (smin, smax) = sides
            .iter()
            .fold((f64::MAX, 0.0f64), |(a, b), &s| (a.min(s), b.max(s)));
        let min_corner = p.min_corner_dist_rate * perim;
        if smin < p.min_side_px || smax > p.max_aspect * smin || smin < min_corner {
            return None;
        }
        Some(q)
    }

    fn decode(&mut self, img: &GrayImage, dict: &Dictionary, quad: [P2; 4]) -> Option<Candidate> {
        let p = &self.params;
        let n = dict.bits + 2 * p.border_bits;
        let side = (0..4)
            .map(|k| dist(quad[k], quad[(k + 1) % 4]))
            .sum::<f64>()
            / 4.0;
        let cell = side / n as f64;
        let quad = if p.refine_corners {
            match refine_quad(img, &quad, cell, &mut self.edge_pts) {
                Some(r) if is_convex(&r) => refine_quad(img, &r, cell, &mut self.edge_pts)
                    .filter(is_convex)
                    .unwrap_or(r),
                _ => quad,
            }
        } else {
            quad
        };
        let hmg = Homography::from_unit_square(&quad)?;
        let m = p.cell_margin.clamp(0.0, 0.45);
        let offs = [m, 0.5, 1.0 - m];
        // Cell means on an (n+2)^2 grid that includes a one-cell ring of quiet zone around the marker.
        let g = n + 2;
        self.cells.clear();
        for r in 0..g {
            for c in 0..g {
                let mut acc = 0.0;
                for oy in offs {
                    for ox in offs {
                        let u = (c as f64 - 1.0 + ox) / n as f64;
                        let v = (r as f64 - 1.0 + oy) / n as f64;
                        let [x, y] = hmg.map(u, v);
                        acc += img.sample(x, y);
                    }
                }
                self.cells.push(acc / 9.0);
            }
        }
        let mut best: Option<Candidate> = None;
        for li in 0..self.params.sharpen_levels.len() {
            let beta = self.params.sharpen_levels[li];
            self.sharp.clear();
            for r in 1..=n {
                for c in 1..=n {
                    let v = self.cells[r * g + c];
                    let nb = self.cells[(r - 1) * g + c]
                        + self.cells[(r + 1) * g + c]
                        + self.cells[r * g + c - 1]
                        + self.cells[r * g + c + 1];
                    self.sharp.push(v + beta * (4.0 * v - nb));
                }
            }
            if let Some(cand) = self.classify(dict, &quad, n) {
                let done = cand.perfect();
                if best.as_ref().is_none_or(|b| cand.better_than(b)) {
                    best = Some(cand);
                }
                if done {
                    break;
                }
            }
        }
        best
    }

    fn classify(&mut self, dict: &Dictionary, quad: &[P2; 4], n: usize) -> Option<Candidate> {
        let p = &self.params;
        let (thr, contrast) = otsu(&self.sharp, &mut self.sorted)?;
        if contrast < p.min_cell_contrast {
            return None;
        }
        let unsure = |v: f64| (v - thr).abs() < p.uncertain_rate * contrast;
        let bb = p.border_bits;
        let mut border_errors = 0;
        let mut code = 0u64;
        for r in 0..n {
            for c in 0..n {
                let v = self.sharp[r * n + c];
                let white = v > thr;
                if r < bb || c < bb || r >= n - bb || c >= n - bb {
                    if white && !unsure(v) {
                        return None;
                    }
                    border_errors += white as u32;
                } else {
                    code = (code << 1) | white as u64;
                }
            }
        }
        if border_errors > p.max_border_errors {
            return None;
        }
        let nb = dict.bits;
        self.weights.clear();
        for i in (0..nb * nb).rev() {
            let v = self.sharp[(i / nb + bb) * n + i % nb + bb];
            self.weights.push(((v - thr).abs() / contrast).min(1.0));
        }
        let (id, s, hamming) = soft_identify(&self.rot_codes, code, &self.weights, p)?;
        let corners = [0, 1, 2, 3].map(|k| quad[(k + s) % 4]);
        Some(Candidate {
            marker: Marker {
                id,
                corners: corners.map(|c| [c[0] as f32, c[1] as f32]),
                hamming,
            },
            quad: *quad,
            border_errors,
            contrast,
        })
    }

    fn insert(&mut self, cand: Candidate) {
        let c = diag_intersection(&cand.quad);
        let overlap = |f: &Candidate| {
            inside_quad(c, &f.quad) || inside_quad(diag_intersection(&f.quad), &cand.quad)
        };
        if let Some(i) = self.found.iter().position(overlap) {
            if cand.better_than(&self.found[i]) {
                self.found[i] = cand;
            }
        } else {
            self.found.push(cand);
        }
    }
}

// Soft-decision match against all rotations. `weights[b]` is the confidence of bit b (LSB first).
// Returns (id, printed top-left quad index, hamming). Exact matches resolve to the lowest id as in OpenCV;
// corrected matches need a confidence margin over every other id.
fn soft_identify(
    rot_codes: &[[u64; 4]],
    code: u64,
    weights: &[f64],
    p: &DetectorParams,
) -> Option<(u32, usize, u32)> {
    let mut best = (f64::MAX, u32::MAX, 0usize, u32::MAX);
    let mut second = f64::MAX;
    let reach = p.max_hamming + 2;
    for (id, rots) in rot_codes.iter().enumerate() {
        let mut idbest = (f64::MAX, 0usize, u32::MAX);
        // Visit rotations in OpenCV's order so 180-degree-symmetric codes get the same orientation.
        for j in [0, 3, 2, 1] {
            let diff = rots[j] ^ code;
            let h = diff.count_ones();
            if h > reach {
                continue;
            }
            let mut cost = 0.0;
            let mut d = diff;
            while d != 0 {
                cost += weights[d.trailing_zeros() as usize];
                d &= d - 1;
            }
            if cost < idbest.0 {
                idbest = (cost, j, h);
            }
        }
        if idbest.2 == u32::MAX {
            continue;
        }
        if best.3 != 0 && (idbest.2 == 0 || idbest.0 < best.0) {
            second = second.min(best.0);
            best = (idbest.0, id as u32, idbest.1, idbest.2);
        } else {
            second = second.min(idbest.0);
        }
    }
    let (cost, id, j, h) = best;
    if h > p.max_hamming {
        return None;
    }
    if h > 0 && second - cost < p.min_decode_margin {
        return None;
    }
    Some((id, j, h))
}

// Returns (threshold, mean gap between the two classes).
fn otsu(vals: &[f64], sorted: &mut Vec<f64>) -> Option<(f64, f64)> {
    sorted.clear();
    sorted.extend_from_slice(vals);
    sorted.sort_unstable_by(|a, b| a.total_cmp(b));
    let n = sorted.len();
    let total: f64 = sorted.iter().sum();
    let mut left = 0.0;
    let mut best = (f64::MIN, 0usize);
    for i in 1..n {
        left += sorted[i - 1];
        let (n0, n1) = (i as f64, (n - i) as f64);
        let gap = (total - left) / n1 - left / n0;
        let score = n0 * n1 * gap * gap;
        if score > best.0 {
            best = (score, i);
        }
    }
    let i = best.1;
    if i == 0 {
        return None;
    }
    let lo = sorted[..i].iter().sum::<f64>() / i as f64;
    let hi = sorted[i..].iter().sum::<f64>() / (n - i) as f64;
    Some((0.5 * (sorted[i - 1] + sorted[i]), hi - lo))
}
