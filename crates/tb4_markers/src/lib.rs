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
}

#[derive(Debug, Clone)]
pub struct LocateParams {
    pub max_range: f64,
    pub occupied_threshold: i8,
    pub range_ratio_gate: f64,
    pub allow_pnp_fallback: bool,
    pub max_abs_bearing_z: f64,
}

impl Default for LocateParams {
    fn default() -> Self {
        Self {
            max_range: 4.0,
            occupied_threshold: 65,
            range_ratio_gate: 3.0,
            allow_pnp_fallback: true,
            max_abs_bearing_z: 0.8,
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
                return Some(Estimate { x: hit.x, y: hit.y, source: Source::Raycast });
            }
        }
    }
    if p.allow_pnp_fallback && pnp_ground > 0.0 && pnp_ground <= p.max_range {
        return Some(Estimate {
            x: o.camera.0 + dx * pnp_ground,
            y: o.camera.1 + dy * pnp_ground,
            source: Source::PnpRange,
        });
    }
    None
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
}

impl Default for TrackParams {
    fn default() -> Self {
        Self { min_views: 5, max_spread: 0.25, max_history: 200, scale_tolerance: 0.25, scale_min_samples: 3, min_view_move: 0.05, min_view_turn: 0.05 }
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
    estimates: BTreeMap<i64, Vec<(f64, f64)>>,
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
        let at = |r: f64, source| Estimate { x: o.camera.0 + dx * r, y: o.camera.1 + dy * r, source };
        let hit = grid.and_then(|g| g.raycast(o.camera.0, o.camera.1, dx, dy, p.max_range, p.occupied_threshold));
        let ratio = hit.map(|h| h.range / pnp).filter(|r| *r <= p.range_ratio_gate && *r >= 1.0 / p.range_ratio_gate);
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
        let est = match (hit, ratio, learned) {
            (Some(h), Some(r), Some(k)) if (r / k - 1.0).abs() <= self.params.scale_tolerance => {
                Some(Estimate { x: h.x, y: h.y, source: Source::Raycast })
            }
            (_, _, Some(k)) => (pnp * k <= p.max_range).then(|| at(pnp * k, Source::ScaledPnp)),
            (Some(h), Some(_), None) => Some(Estimate { x: h.x, y: h.y, source: Source::Raycast }),
            _ => (p.allow_pnp_fallback && pnp <= p.max_range).then(|| at(pnp, Source::PnpRange)),
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
        v.push((e.x, e.y));
        if v.len() > self.params.max_history {
            let excess = v.len() - self.params.max_history;
            v.drain(..excess);
        }
    }

    pub fn target(&self, id: i64) -> Option<Target> {
        let v = self.estimates.get(&id)?;
        let mx = median(v.iter().map(|p| p.0).collect());
        let my = median(v.iter().map(|p| p.1).collect());
        let spread = median(v.iter().map(|p| (p.0 - mx).hypot(p.1 - my)).collect());
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
        let e = |x, y| Estimate { x, y, source: Source::Raycast };
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
        assert_eq!(to_csv(&t.confirmed()), "id,x,y\n1,1.010,1.005\n");
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

    #[test]
    fn history_is_bounded() {
        let mut t = Tracker::new(TrackParams { max_history: 4, ..Default::default() });
        for i in 0..10 {
            t.add(3, &Estimate { x: i as f64, y: 0.0, source: Source::Raycast });
        }
        assert_eq!(t.target(3).unwrap().views, 4);
        assert!((t.target(3).unwrap().x - 7.5).abs() < 1e-9);
    }
}
