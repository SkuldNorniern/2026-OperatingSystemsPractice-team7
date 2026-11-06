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
}

impl Default for TrackParams {
    fn default() -> Self {
        Self { min_views: 5, max_spread: 0.25, max_history: 200 }
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
}

impl Tracker {
    pub fn new(params: TrackParams) -> Self {
        Self { params, estimates: BTreeMap::new() }
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
    fn history_is_bounded() {
        let mut t = Tracker::new(TrackParams { max_history: 4, ..Default::default() });
        for i in 0..10 {
            t.add(3, &Estimate { x: i as f64, y: 0.0, source: Source::Raycast });
        }
        assert_eq!(t.target(3).unwrap().views, 4);
        assert!((t.target(3).unwrap().x - 7.5).abs() < 1e-9);
    }
}
