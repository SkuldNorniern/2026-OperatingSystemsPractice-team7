use std::collections::BTreeMap;
use std::fmt::Write as _;

use tb4_core::grid::Grid;

#[derive(Debug, Clone, Copy)]
pub struct Observation {
    pub id: i64,
    pub camera: (f64, f64, f64),
    pub bearing: (f64, f64, f64),
    pub pnp_range: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Source {
    Raycast,
    PnpRange,
    /// pnp range times this marker's learned ray/pnp scale
    ScaledPnp,
}

#[derive(Debug, Clone, Copy)]
pub struct Estimate {
    pub x: f64,
    pub y: f64,
    pub source: Source,
    /// unit ray from camera, in map xy
    pub dir: (f64, f64),
    /// 1 sigma along ray and across it (m)
    pub sigma_along: f64,
    pub sigma_across: f64,
}

impl Estimate {
    /// round, equally trusted point
    pub fn point(x: f64, y: f64, source: Source) -> Self {
        Self { x, y, source, dir: (1.0, 0.0), sigma_along: 0.05, sigma_across: 0.05 }
    }
}

#[derive(Debug, Clone)]
pub struct LocateParams {
    pub max_range: f64,
    pub occupied_threshold: i8,
    pub range_ratio_gate: f64,
    pub allow_pnp_fallback: bool,
    pub max_abs_bearing_z: f64,
    /// wall cells within this radius of a ray hit get a line fit, 0 = off (m)
    pub wall_fit_radius: f64,
}

impl Default for LocateParams {
    fn default() -> Self {
        Self {
            max_range: 4.0,
            occupied_threshold: 65,
            range_ratio_gate: 3.0,
            allow_pnp_fallback: true,
            max_abs_bearing_z: 0.8,
            wall_fit_radius: 0.2,
        }
    }
}

// raycast bearing through map, so marker size doesnt matter. pnp range is fallback
pub fn locate(grid: Option<&Grid>, o: &Observation, p: &LocateParams) -> Option<Estimate> {
    let (bx, by, bz) = o.bearing;
    if bz.abs() > p.max_abs_bearing_z {
        return None;
    }
    let horiz = bx.hypot(by);
    if horiz < 1e-6 {
        return None;
    }
    let (dx, dy) = (bx / horiz, by / horiz);
    let pnp_ground = o.pnp_range * horiz;

    if let Some(g) = grid {
        if let Some(hit) = g.raycast(o.camera.0, o.camera.1, dx, dy, p.max_range, p.occupied_threshold) {
            let ratio = if hit.range > 1e-6 { pnp_ground / hit.range } else { f64::INFINITY };
            if ratio.is_finite() && ratio <= p.range_ratio_gate && ratio >= 1.0 / p.range_ratio_gate {
                return Some(Estimate::point(hit.x, hit.y, Source::Raycast));
            }
        }
    }
    if p.allow_pnp_fallback && pnp_ground > 0.0 && pnp_ground <= p.max_range {
        return Some(Estimate::point(o.camera.0 + dx * pnp_ground, o.camera.1 + dy * pnp_ground, Source::PnpRange));
    }
    None
}

/// where ray meets wall line fitted through occupied cells around grid hit. raw hit sits on the
/// cell face toward the camera, so up to one cell short. returns range and cos of incidence
pub fn wall_hit(g: &Grid, origin: (f64, f64), dir: (f64, f64), hit_range: f64, p: &LocateParams) -> Option<(f64, f64)> {
    let res = g.meta.resolution;
    let r = p.wall_fit_radius;
    if r < res {
        return None;
    }
    // probe half a cell inside so the seed is the occupied cell itself
    let (hx, hy) = (origin.0 + dir.0 * (hit_range + 0.5 * res), origin.1 + dir.1 * (hit_range + 0.5 * res));
    let (cx, cy) = g.world_to_cell_signed(hx, hy);
    let n = (r / res).ceil() as i64;
    let occ = |x: i64, y: i64| g.cell_checked(x, y).is_some_and(|c| g.get(c) >= p.occupied_threshold);
    let free = |x: i64, y: i64| g.cell_checked(x, y).is_some_and(|c| (0..p.occupied_threshold).contains(&g.get(c)));
    // (x, y, has free 4-neighbour on camera side)
    let mut cells = Vec::new();
    for y in cy - n..=cy + n {
        for x in cx - n..=cx + n {
            if !occ(x, y) {
                continue;
            }
            let (wx, wy) = g.cell_to_world(g.cell_checked(x, y).unwrap());
            if (wx - hx).hypot(wy - hy) <= r {
                let surface = [(1i64, 0i64), (-1, 0), (0, 1), (0, -1)]
                    .iter()
                    .any(|&(ex, ey)| (ex as f64 * dir.0 + ey as f64 * dir.1) < 0.0 && free(x + ex, y + ey));
                cells.push((wx, wy, surface));
            }
        }
    }
    // orientation from surface cells only, so far face of a thick wall cant tilt it
    let front: Vec<(f64, f64)> = cells.iter().filter(|c| c.2).map(|c| (c.0, c.1)).collect();
    if front.len() < 4 {
        return None;
    }
    let k = front.len() as f64;
    let (mx, my) = (front.iter().map(|c| c.0).sum::<f64>() / k, front.iter().map(|c| c.1).sum::<f64>() / k);
    let (mut cxx, mut cyy, mut cxy) = (0.0, 0.0, 0.0);
    for (x, y) in &front {
        cxx += (x - mx).powi(2) / k;
        cyy += (y - my).powi(2) / k;
        cxy += (x - mx) * (y - my) / k;
    }
    let tr = cxx + cyy;
    let disc = ((cxx - cyy).powi(2) / 4.0 + cxy * cxy).sqrt();
    let (l1, l2) = (tr / 2.0 + disc, tr / 2.0 - disc);
    // corner or blob: no single line
    if l1 <= 0.0 || l2 / l1 > 0.15 {
        return None;
    }
    let ang = 0.5 * (2.0 * cxy).atan2(cxx - cyy);
    // normal pointing back at camera
    let (mut nx, mut ny) = (-ang.sin(), ang.cos());
    if dir.0 * nx + dir.1 * ny > 0.0 {
        (nx, ny) = (-nx, -ny);
    }
    let cos_inc = -(dir.0 * nx + dir.1 * ny);
    if cos_inc < 0.2 {
        return None;
    }
    // offset from every occupied cell in the band just behind front line: lidar noise spreads one
    // surface over ~2 cells, front cells alone sit half a cell short
    let d0 = mx * nx + my * ny;
    let band: Vec<f64> = cells.iter().map(|c| c.0 * nx + c.1 * ny).filter(|d| (-1.5 * res..=0.5 * res).contains(&(d - d0))).collect();
    let d = band.iter().sum::<f64>() / band.len() as f64;
    let t = (d - (origin.0 * nx + origin.1 * ny)) / (dir.0 * nx + dir.1 * ny);
    ((t - hit_range).abs() <= 1.5 * res).then_some((t, cos_inc))
}

#[derive(Debug, Clone)]
pub struct TrackParams {
    pub min_views: usize,
    pub max_spread: f64,
    pub max_history: usize,
    /// raycast is trusted only if its ray/pnp ratio is within this fraction of the marker's median ratio
    pub scale_tolerance: f64,
    pub scale_min_samples: usize,
    /// a new view of an id only counts after the camera moved or turned this much (m, rad)
    pub min_view_move: f64,
    pub min_view_turn: f64,
    /// bearing error incl. map yaw error (rad), sets trust across the ray
    pub bearing_sigma: f64,
    /// wall position error from map (m), along ray it grows with 1/cos(incidence)
    pub wall_sigma: f64,
    /// pnp range error, fraction of range
    pub pnp_sigma: f64,
}

impl Default for TrackParams {
    fn default() -> Self {
        Self {
            min_views: 5,
            max_spread: 0.25,
            max_history: 200,
            scale_tolerance: 0.25,
            scale_min_samples: 3,
            min_view_move: 0.05,
            min_view_turn: 0.05,
            bearing_sigma: 0.012,
            wall_sigma: 0.03,
            pnp_sigma: 0.08,
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct Target {
    pub id: i64,
    pub x: f64,
    pub y: f64,
    pub views: usize,
    pub spread: f64,
    pub confirmed: bool,
}

#[derive(Debug, Default)]
pub struct Tracker {
    pub params: TrackParams,
    estimates: BTreeMap<i64, Vec<Estimate>>,
    /// per id: ray range / pnp range of raycasts that passed the coarse gate
    scales: BTreeMap<i64, Vec<f64>>,
    /// per id: camera xy and bearing angle of the last view used
    last_view: BTreeMap<i64, (f64, f64, f64)>,
}

impl Tracker {
    pub fn new(params: TrackParams) -> Self {
        Self { params, estimates: BTreeMap::new(), scales: BTreeMap::new(), last_view: BTreeMap::new() }
    }

    /// marker size is unknown but fixed, so ray/pnp ratio is constant per id. learn it, then reject
    /// raycasts that hit something in front of the marker and use the scaled pnp range instead
    pub fn observe(&mut self, grid: Option<&Grid>, o: &Observation, p: &LocateParams) -> Option<Estimate> {
        let (bx, by, bz) = o.bearing;
        let horiz = bx.hypot(by);
        if bz.abs() > p.max_abs_bearing_z || horiz < 1e-6 || o.pnp_range <= 0.0 {
            return None;
        }
        let (dx, dy) = (bx / horiz, by / horiz);
        // staring from one spot adds no information but lets one biased pose dominate the median
        let ang = dy.atan2(dx);
        if let Some(&(lx, ly, la)) = self.last_view.get(&o.id) {
            let turn = (ang - la).sin().atan2((ang - la).cos()).abs();
            if (o.camera.0 - lx).hypot(o.camera.1 - ly) < self.params.min_view_move && turn < self.params.min_view_turn {
                return None;
            }
        }
        self.last_view.insert(o.id, (o.camera.0, o.camera.1, ang));
        let pnp = o.pnp_range * horiz;
        let tp = &self.params;
        let across = |r: f64| (r * tp.bearing_sigma).max(0.005);
        let at = |r: f64, source, along: f64| Estimate {
            x: o.camera.0 + dx * r,
            y: o.camera.1 + dy * r,
            source,
            dir: (dx, dy),
            sigma_along: along,
            sigma_across: across(r),
        };
        let hit = grid.and_then(|g| {
            let h = g.raycast(o.camera.0, o.camera.1, dx, dy, p.max_range, p.occupied_threshold)?;
            Some(match wall_hit(g, (o.camera.0, o.camera.1), (dx, dy), h.range, p) {
                // bearing error slides the hit along a slanted wall by tan(incidence)
                Some((range, c)) => (range, tp.wall_sigma / c + across(range) * (1.0 - c * c).sqrt() / c),
                // no clean wall line: cell face, unknown incidence
                None => (h.range, 2.0 * (tp.wall_sigma + g.meta.resolution)),
            })
        });
        let ratio = hit.map(|h| h.0 / pnp).filter(|r| *r <= p.range_ratio_gate && *r >= 1.0 / p.range_ratio_gate);
        if let Some(r) = ratio {
            let v = self.scales.entry(o.id).or_default();
            v.push(r);
            if v.len() > self.params.max_history {
                v.remove(0);
            }
        }
        let learned = self
            .scales
            .get(&o.id)
            .filter(|v| v.len() >= self.params.scale_min_samples)
            .map(|v| median(v.clone()));
        let tp = &self.params;
        let est = match (hit, ratio, learned) {
            (Some((r, along)), Some(q), Some(k)) if (q / k - 1.0).abs() <= tp.scale_tolerance => Some(at(r, Source::Raycast, along)),
            (_, _, Some(k)) => (pnp * k <= p.max_range).then(|| at(pnp * k, Source::ScaledPnp, pnp * k * tp.pnp_sigma)),
            (Some((r, along)), Some(_), None) => Some(at(r, Source::Raycast, along)),
            // unknown marker size: range barely constrained, bearing still counts
            _ => (p.allow_pnp_fallback && pnp <= p.max_range).then(|| at(pnp, Source::PnpRange, pnp * 0.5)),
        };
        if let Some(e) = &est {
            self.add(o.id, e);
        }
        est
    }

    /// learned ray/pnp scale of a marker, if enough samples
    pub fn scale(&self, id: i64) -> Option<f64> {
        self.scales.get(&id).filter(|v| v.len() >= self.params.scale_min_samples).map(|v| median(v.clone()))
    }

    pub fn add(&mut self, id: i64, e: &Estimate) {
        let v = self.estimates.entry(id).or_default();
        v.push(*e);
        if v.len() > self.params.max_history {
            let excess = v.len() - self.params.max_history;
            v.drain(..excess);
        }
    }

    pub fn target(&self, id: i64) -> Option<Target> {
        let v = self.estimates.get(&id)?;
        let (mx, my) = fuse(v);
        let spread = median(v.iter().map(|p| (p.x - mx).hypot(p.y - my)).collect());
        Some(Target {
            id,
            x: mx,
            y: my,
            views: v.len(),
            spread,
            confirmed: v.len() >= self.params.min_views && spread <= self.params.max_spread,
        })
    }

    pub fn targets(&self) -> Vec<Target> {
        self.estimates.keys().filter_map(|&id| self.target(id)).collect()
    }

    pub fn confirmed(&self) -> Vec<Target> {
        self.targets().into_iter().filter(|t| t.confirmed).collect()
    }
}

/// robust weighted least squares over views, each a point with its own along/across ray sigma.
/// starts from coordinate median, huber reweighting drops bad views
pub fn fuse(v: &[Estimate]) -> (f64, f64) {
    let mut x = (median(v.iter().map(|p| p.x).collect()), median(v.iter().map(|p| p.y).collect()));
    if v.len() < 2 {
        return x;
    }
    let info = |e: &Estimate| {
        let (ux, uy) = e.dir;
        let (a, c) = (1.0 / e.sigma_along.max(1e-3).powi(2), 1.0 / e.sigma_across.max(1e-3).powi(2));
        // a u u^T + c n n^T with n = (-uy, ux)
        (a * ux * ux + c * uy * uy, (a - c) * ux * uy, a * uy * uy + c * ux * ux)
    };
    const K: f64 = 2.0;
    // huber first, then drop views past 3K so far outliers stop pulling at all
    for it in 0..25 {
        let hard = it >= 15;
        let (mut a11, mut a12, mut a22, mut b1, mut b2) = (0.0, 0.0, 0.0, 0.0, 0.0);
        for e in v {
            let (j11, j12, j22) = info(e);
            let (ex, ey) = (x.0 - e.x, x.1 - e.y);
            let d = (ex * (j11 * ex + j12 * ey) + ey * (j12 * ex + j22 * ey)).max(0.0).sqrt();
            let w = if d <= K { 1.0 } else if hard && d > 3.0 * K { 0.0 } else { K / d };
            a11 += w * j11;
            a12 += w * j12;
            a22 += w * j22;
            b1 += w * (j11 * e.x + j12 * e.y);
            b2 += w * (j12 * e.x + j22 * e.y);
        }
        let det = a11 * a22 - a12 * a12;
        if det.abs() < 1e-12 {
            break;
        }
        let nx = ((a22 * b1 - a12 * b2) / det, (a11 * b2 - a12 * b1) / det);
        let step = (nx.0 - x.0).hypot(nx.1 - x.1);
        x = nx;
        if step < 1e-5 && hard {
            break;
        }
    }
    x
}

fn median(mut v: Vec<f64>) -> f64 {
    if v.is_empty() {
        return f64::NAN;
    }
    v.sort_by(f64::total_cmp);
    let n = v.len();
    if n % 2 == 1 { v[n / 2] } else { 0.5 * (v[n / 2 - 1] + v[n / 2]) }
}

pub fn to_csv(targets: &[Target]) -> String {
    let mut s = String::from("id,x,y\n");
    for t in targets {
        let _ = writeln!(s, "{},{:.3},{:.3}", t.id, t.x, t.y);
    }
    s
}

#[cfg(test)]
mod tests {
    use super::*;
    use tb4_core::grid::{Grid, GridMeta};

    fn wall_map() -> Grid {
        let meta = GridMeta { width: 140, height: 140, resolution: 0.05, origin_x: -3.0, origin_y: -3.0 };
        let mut g = Grid::filled(meta, 0);
        for cy in 0..140 {
            let c = g.world_to_cell(3.01, -3.0 + cy as f64 * 0.05 + 0.01).unwrap();
            g.set(c, 100);
        }
        g
    }

    fn obs(pnp_range: f64) -> Observation {
        Observation { id: 7, camera: (0.0, 0.0, 0.25), bearing: (1.0, 0.0, 0.0), pnp_range }
    }

    #[test]
    fn raycast_ignores_marker_size() {
        let g = wall_map();
        let p = LocateParams::default();
        let e = locate(Some(&g), &obs(2.0), &p).unwrap();
        assert_eq!(e.source, Source::Raycast);
        assert!((e.x - 3.0).abs() < 1e-9 && e.y.abs() < 1e-9);
    }

    #[test]
    fn gate_rejects_inconsistent_hit() {
        let g = wall_map();
        let p = LocateParams { range_ratio_gate: 1.2, allow_pnp_fallback: false, ..Default::default() };
        assert!(locate(Some(&g), &obs(1.0), &p).is_none());
    }

    #[test]
    fn pnp_fallback_without_map() {
        let p = LocateParams::default();
        let o = Observation { bearing: (0.6, 0.0, 0.8 - 1e-9), ..obs(2.0) };
        let e = locate(None, &o, &p).unwrap();
        assert_eq!(e.source, Source::PnpRange);
        assert!((e.x - 1.2).abs() < 1e-6);
        let steep = Observation { bearing: (0.0, 0.0, 1.0), ..obs(2.0) };
        assert!(locate(None, &steep, &p).is_none());
    }

    #[test]
    fn tracker_confirms_consistent_views_and_rejects_outliers() {
        let mut t = Tracker::new(TrackParams { min_views: 3, ..Default::default() });
        let e = |x, y| Estimate::point(x, y, Source::Raycast);
        t.add(1, &e(1.0, 1.0));
        t.add(1, &e(1.02, 0.98));
        assert!(!t.target(1).unwrap().confirmed);
        t.add(1, &e(5.0, 5.0));
        t.add(1, &e(0.99, 1.01));
        let tg = t.target(1).unwrap();
        assert!(tg.confirmed, "{tg:?}");
        assert!((tg.x - 1.01).abs() < 0.02 && (tg.y - 1.005).abs() < 0.02);
        for i in 0..10 {
            t.add(2, &e(i as f64, 0.0));
        }
        assert!(!t.target(2).unwrap().confirmed);
        assert_eq!(t.confirmed().len(), 1);
        assert_eq!(to_csv(&t.confirmed()), "id,x,y\n1,1.003,0.997\n");
    }

    #[test]
    fn learned_scale_rejects_hits_in_front_of_marker() {
        // wall with marker at x = 3, thin obstacle at x = 1.5 for y in [0.3, 0.6]
        let mut g = wall_map();
        for cy in 0..6 {
            let c = g.world_to_cell(1.51, 0.31 + cy as f64 * 0.05).unwrap();
            g.set(c, 100);
        }
        let mut t = Tracker::new(TrackParams::default());
        let p = LocateParams::default();
        // marker is 2x the configured size: pnp says half the true range
        let view = |cy: f64| {
            let (dx, dy): (f64, f64) = (3.0, 0.45 - cy);
            let r = dx.hypot(dy);
            Observation { id: 9, camera: (0.0, cy, 0.25), bearing: (dx / r, dy / r, 0.0), pnp_range: r * 0.5 }
        };
        for cy in [-1.0, -0.8, -0.6, -0.4, 1.2, 1.4] {
            assert_eq!(t.observe(Some(&g), &view(cy), &p).unwrap().source, Source::Raycast);
        }
        assert!((t.scale(9).unwrap() - 2.0).abs() < 0.05);
        // straight on, the ray hits the obstacle at 1.5 m: rejected, scaled pnp gives 3 m
        let e = t.observe(Some(&g), &view(0.45), &p).unwrap();
        assert_eq!(e.source, Source::ScaledPnp);
        assert!((e.x - 3.0).abs() < 0.05 && (e.y - 0.45).abs() < 0.05, "{e:?}");
        let tg = t.target(9).unwrap();
        assert!((tg.x - 3.0).abs() < 0.06 && (tg.y - 0.45).abs() < 0.06, "{tg:?}");
    }

    #[test]
    fn staring_does_not_add_views() {
        let g = wall_map();
        let mut t = Tracker::new(TrackParams::default());
        let p = LocateParams::default();
        for _ in 0..50 {
            t.observe(Some(&g), &obs(3.0), &p);
        }
        assert_eq!(t.target(7).unwrap().views, 1);
        let moved = Observation { camera: (0.0, 0.1, 0.25), ..obs(3.0) };
        assert!(t.observe(Some(&g), &moved, &p).is_some());
        assert_eq!(t.target(7).unwrap().views, 2);
    }

    /// wall along y = 0.3 x + 1.0 drawn into grid like slam does: cell holding surface point is occupied
    fn slanted_wall() -> Grid {
        let meta = GridMeta { width: 160, height: 160, resolution: 0.05, origin_x: -4.0, origin_y: -4.0 };
        let mut g = Grid::filled(meta, 0);
        let mut x = -4.0;
        while x < 4.0 {
            if let Some(c) = g.world_to_cell(x, 0.3 * x + 1.0) {
                g.set(c, 100);
            }
            x += 0.005;
        }
        g
    }

    #[test]
    fn wall_fit_lands_on_surface_not_cell_face() {
        let g = slanted_wall();
        let p = LocateParams::default();
        let (mut raw, mut fit) = (0.0f64, 0.0f64);
        for k in 0..40 {
            let a = 1.2 + k as f64 * 0.02;
            let (dx, dy) = (a.cos(), a.sin());
            let o = (0.013 * k as f64, -0.5);
            // true range: ray meets y = 0.3x + 1
            let t = (1.0 + 0.3 * o.0 - o.1) / (dy - 0.3 * dx);
            let h = g.raycast(o.0, o.1, dx, dy, 5.0, 65).unwrap();
            let (r, cos_inc) = wall_hit(&g, o, (dx, dy), h.range, &p).expect("fit");
            assert!(cos_inc > 0.5);
            raw = raw.max((h.range - t).abs());
            fit = fit.max((r - t).abs());
        }
        assert!(fit < 0.02 && fit < raw * 0.6, "fit {fit} raw {raw}");
    }

    #[test]
    fn wall_fit_uses_near_face_of_thick_wall() {
        // both faces of a 15 cm partition mapped, x = 3.0 and x = 3.15, inside never seen
        let mut g = wall_map();
        for cy in 0..140 {
            let y = -3.0 + cy as f64 * 0.05 + 0.01;
            g.set(g.world_to_cell(3.16, y).unwrap(), 100);
            g.set(g.world_to_cell(3.06, y).unwrap(), -1);
            g.set(g.world_to_cell(3.11, y).unwrap(), -1);
        }
        let p = LocateParams::default();
        let (dx, dy) = (3.0f64, 0.4f64);
        let n = dx.hypot(dy);
        let u = (dx / n, dy / n);
        let h = g.raycast(0.0, 0.0, u.0, u.1, 5.0, 65).unwrap();
        let (t, _) = wall_hit(&g, (0.0, 0.0), u, h.range, &p).unwrap();
        // cell holding x = 3.01 spans 3.00..3.05, its center is the surface estimate
        assert!((t * u.0 - 3.025).abs() < 0.01, "{}", t * u.0);
    }

    #[test]
    fn wall_fit_skips_corner() {
        let mut g = wall_map();
        for cx in 0..140 {
            let c = g.world_to_cell(-3.0 + cx as f64 * 0.05 + 0.01, 1.01).unwrap();
            g.set(c, 100);
        }
        let p = LocateParams::default();
        let (dx, dy) = (3.0f64, 1.0f64);
        let n = dx.hypot(dy);
        let h = g.raycast(0.0, 0.0, dx / n, dy / n, 5.0, 65).unwrap();
        assert!(wall_hit(&g, (0.0, 0.0), (dx / n, dy / n), h.range, &p).is_none());
    }

    #[test]
    fn fusion_trusts_each_view_in_its_good_direction() {
        // truth (2, 0). view A looks along x: range poor, across fine. view B looks along y: same
        let a = Estimate { x: 2.3, y: 0.01, source: Source::PnpRange, dir: (1.0, 0.0), sigma_along: 0.3, sigma_across: 0.02 };
        let b = Estimate { x: 2.01, y: -0.3, source: Source::PnpRange, dir: (0.0, 1.0), sigma_along: 0.3, sigma_across: 0.02 };
        let (x, y) = fuse(&[a, b, a, b]);
        assert!((x - 2.0).abs() < 0.03 && y.abs() < 0.03, "{x} {y}");
    }

    #[test]
    fn history_is_bounded() {
        let mut t = Tracker::new(TrackParams { max_history: 4, ..Default::default() });
        for i in 0..10 {
            t.add(3, &Estimate::point(i as f64, 0.0, Source::Raycast));
        }
        assert_eq!(t.target(3).unwrap().views, 4);
        assert!((t.target(3).unwrap().x - 7.5).abs() < 1e-9);
    }
}
