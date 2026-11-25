use std::collections::{BTreeMap, BTreeSet};
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
    /// hits and pnp ranges closer than this to the camera are ignored (m): nothing real sits inside
    /// the robot (bench run 183350 put every marker at the robot centre from a ray hitting its own cell)
    pub min_range: f64,
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
            min_range: 0.3,
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
    let mut cells = Vec::new();
    for y in cy - n..=cy + n {
        for x in cx - n..=cx + n {
            if !occ(x, y) {
                continue;
            }
            let (wx, wy) = g.cell_to_world(g.cell_checked(x, y).unwrap());
            if (wx - hx).hypot(wy - hy) <= r {
                cells.push((wx, wy));
            }
        }
    }
    // visible face: first occupied cell of a fan of rays from camera. works whether space in
    // front of wall is mapped free or still unknown, and never picks far face of thick wall
    let range = hit_range.max(0.3);
    let half = (r / range).atan();
    let step = 0.5 * res / range;
    let base = dir.1.atan2(dir.0);
    let mut front: Vec<(f64, f64)> = Vec::new();
    let mut a = -half;
    while a <= half {
        let (s, c) = (base + a).sin_cos();
        if let Some(h) = g.raycast(origin.0, origin.1, c, s, range + r, p.occupied_threshold) {
            if let Some(cell) = g.world_to_cell(h.x + c * 0.5 * res, h.y + s * 0.5 * res) {
                let w = g.cell_to_world(cell);
                if (w.0 - hx).hypot(w.1 - hy) <= r && !front.contains(&w) {
                    front.push(w);
                }
            }
        }
        a += step;
    }
    // 3 cells is a box face ~15 cm wide, still a line
    if front.len() < 3 {
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
    // raw hit can be a stray speck a cell or two in front of the wall line
    ((t - hit_range).abs() <= 2.5 * res).then_some((t, cos_inc))
}

/// what the rays a little left and right of the bearing see
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Surface {
    /// both land on same surface as centre ray
    Flat,
    /// one side agrees, other runs past: centre is near end of a surface (box face, wall end)
    Edge,
    /// a side is nearer, or none agree: ray clips something in front of marker
    Clip,
}

/// rays a little left and right of bearing should land on same surface. a jump means the hit is
/// at an edge: marker on a small box face, or ray clipping something in front of the marker
pub fn surface_around(g: &Grid, origin: (f64, f64), ang: f64, range: f64, p: &LocateParams) -> Surface {
    let res = g.meta.resolution;
    // about one marker half width at this range, at least a couple of cells
    let half = (0.06f64).max(2.0 * res);
    let sides = |half: f64| {
        let da = (half / range.max(0.3)).atan();
        [-1.0, 1.0].map(|side: f64| {
            let a = ang + side * da;
            g.raycast(origin.0, origin.1, a.cos(), a.sin(), p.max_range, p.occupied_threshold).map(|h| h.range)
        })
    };
    let rs = sides(half);
    if let [Some(a), Some(b)] = rs {
        // on a plane the two side ranges straddle the centre one, steeper incidence = bigger spread
        let (mid, spread) = (0.5 * (a + b), (a - b).abs());
        if (mid - range).abs() <= 2.0 * res + 0.1 * spread && spread <= 4.0 * half + 2.0 * res {
            return Surface::Flat;
        }
    }
    let agrees = |r: Option<f64>| r.is_some_and(|r| (r - range).abs() <= 2.0 * res + 2.0 * half);
    let past = |r: Option<f64>| r.is_none_or(|r| r > range + 2.0 * res);
    if (agrees(rs[0]) && past(rs[1])) || (agrees(rs[1]) && past(rs[0])) {
        return Surface::Edge;
    }
    // narrow face (small box): falls off both ends at marker width but holds within a cell
    let inner = sides(res);
    let close = |r: Option<f64>| r.is_some_and(|r| (r - range).abs() <= 1.5 * res);
    if close(inner[0]) && close(inner[1]) && past(rs[0]) && past(rs[1]) {
        Surface::Edge
    } else {
        Surface::Clip
    }
}

pub fn flat_around(g: &Grid, origin: (f64, f64), ang: f64, range: f64, p: &LocateParams) -> bool {
    surface_around(g, origin, ang, range, p) == Surface::Flat
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Hit {
    Clean,
    Unfit,
    Edge,
}

#[derive(Debug, Clone)]
pub struct TrackParams {
    pub min_views: usize,
    /// views whose camera ray hit a lidar surface (Source::Raycast) needed to confirm. Targets sit on
    /// faces that reach the floor, so a real marker always has one; a chair label or a pattern in
    /// mid-air only gets pnp views (field run 20261118_184838 confirmed a chair label from 1 ray, 4 pnp)
    pub min_wall_views: usize,
    /// and at least this share of all views must be lidar-backed. A real marker has its wall behind
    /// it in most views; a chair label only now and then (field 184838: 3 of 39 views, 8 %)
    pub min_wall_fraction: f64,
    /// the views' bearings must spread at least this much (rad) before a target is confirmed, and
    /// their line intersection (independent of range) must lie within `max_triangulation_gap` of
    /// the fused position. A label whose rays land on the wall behind it intersects in front of that
    /// wall and fails; views from one spot never confirm (replay 184838: chair label confirmed from
    /// 7 views at almost one bearing). 0 = off.
    pub min_triangulation_spread: f64,
    pub max_triangulation_gap: f64,
    /// ids that also need a learned physical size (ray/pnp scale from clean wall hits) to confirm.
    /// For ARUCO_ORIGINAL: 0, 341, 682, 1023, the codes whose rows are all equal, which barcode
    /// labels on chairs decode as. Field 184838: label 1023 was confirmed from 7 views next to a
    /// wall with no learned scale (NaN). A real one printed on a wall learns its scale.
    pub strict_ids: Vec<i64>,
    /// every id needs a consistent learned size to confirm (as strict_ids, plus the samples must
    /// agree within scale_tolerance). A sticker in mid-air seen from several angles has its rays
    /// land on different surfaces, so its ray/pnp ratio drifts; a printed marker's stays put.
    pub require_scale_all: bool,
    /// fused position 1 sigma must be below this (m)
    pub max_spread: f64,
    /// median mahalanobis distance of views must be below this, else views disagree
    pub max_consistency: f64,
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
    /// edge hit before any scale is learned: ray/pnp must be within this factor
    pub edge_ratio_gate: f64,
    /// m, an edge hit this close to where earlier bearings cross is on the marker's face whatever
    /// the pnp range says, and teaches the scale (spec: marker sizes differ, boxes have only edges)
    pub edge_triangulation_gap: f64,
    /// camera-only confirmation, for a marker no lidar wall hit or learned size supports: views whose
    /// bearings spread at least this (rad) and cross within max_triangulation_gap confirm it. Not for
    /// strict ids (they need a learned size). 0 = off.
    pub pnp_only_spread: f64,
}

impl Default for TrackParams {
    fn default() -> Self {
        Self {
            min_views: 5,
            min_wall_views: 3,
            min_wall_fraction: 0.25,
            min_triangulation_spread: 0.2,
            max_triangulation_gap: 0.3,
            pnp_only_spread: 0.0,
            strict_ids: Vec::new(),
            require_scale_all: false,
            max_spread: 0.08,
            max_consistency: 3.0,
            max_history: 200,
            scale_tolerance: 0.25,
            scale_min_samples: 3,
            min_view_move: 0.05,
            min_view_turn: 0.05,
            bearing_sigma: 0.012,
            wall_sigma: 0.03,
            pnp_sigma: 0.08,
            edge_ratio_gate: 1.5,
            edge_triangulation_gap: 0.15,
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

/// RViz marker namespaces on /team7/targets
pub const NS_CONFIRMED: &str = "target_confirmed";
pub const NS_TENTATIVE: &str = "target_tentative";

impl Target {
    pub fn ns(&self) -> &'static str {
        if self.confirmed { NS_CONFIRMED } else { NS_TENTATIVE }
    }
}

/// ids mission counts as found, from /team7/targets. marker_mapper sends every target on each
/// message, so the latest one replaces the set: an id whose confirmation was withdrawn (LOST)
/// drops out and the count stays equal to the rows in targets.csv
#[derive(Debug, Default)]
pub struct Found(BTreeSet<i32>);

impl Found {
    /// (namespace, id) of every marker in one /team7/targets message
    pub fn update<'a>(&mut self, markers: impl IntoIterator<Item = (&'a str, i32)>) {
        self.0 = markers.into_iter().filter(|m| m.0 == NS_CONFIRMED).map(|m| m.1).collect();
    }

    pub fn ids(&self) -> &BTreeSet<i32> {
        &self.0
    }

    pub fn len(&self) -> usize {
        self.0.len()
    }

    pub fn is_empty(&self) -> bool {
        self.0.is_empty()
    }
}

#[derive(Debug, Default)]
pub struct Tracker {
    pub params: TrackParams,
    estimates: BTreeMap<i64, Vec<Estimate>>,
    /// per id: ray range / pnp range of raycasts that passed the coarse gate
    scales: BTreeMap<i64, Vec<f64>>,
    /// per id: camera xy and bearing angle of the last view used
    last_view: BTreeMap<i64, (f64, f64, f64)>,
    sticky: std::collections::BTreeSet<i64>,
}

impl Tracker {
    pub fn new(params: TrackParams) -> Self {
        Self { params, ..Default::default() }
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
        // (range, sigma along, kind)
        if pnp < p.min_range {
            return None;
        }
        let hit = grid.and_then(|g| {
            let h = g.raycast(o.camera.0, o.camera.1, dx, dy, p.max_range, p.occupied_threshold)?;
            if h.range < p.min_range {
                return None;
            }
            let surface = surface_around(g, (o.camera.0, o.camera.1), ang, h.range, p);
            let fit = wall_hit(g, (o.camera.0, o.camera.1), (dx, dy), h.range, p);
            // bearing error slides the hit along a slanted wall by tan(incidence)
            let along = |c: f64, r: f64| tp.wall_sigma / c + across(r) * (1.0 - c * c).sqrt() / c;
            match (surface, fit) {
                (Surface::Flat, Some((r, c))) => Some((r, along(c, r), Hit::Clean)),
                // no clean wall line: cell face, unknown incidence
                (Surface::Flat, None) => Some((h.range, 2.0 * (tp.wall_sigma + g.meta.resolution), Hit::Unfit)),
                // near end of a surface: fine if marker is on it, wrong if ray clips past an
                // obstacle in front. needs a straight face and pnp range that agrees (below)
                (Surface::Edge, Some((r, c))) => Some((r, along(c, r) + g.meta.resolution, Hit::Edge)),
                // box corner in view: no single line, cell face
                (Surface::Edge, None) => Some((h.range, 2.0 * (tp.wall_sigma + g.meta.resolution), Hit::Edge)),
                (Surface::Clip, _) => None,
            }
        });
        let ratio = hit.map(|h| h.0 / pnp).filter(|r| *r <= p.range_ratio_gate && *r >= 1.0 / p.range_ratio_gate);
        // edge hit where earlier bearings from other places cross: the marker is on this face, so a
        // marker of another size on a box can still learn its scale (else it never gets a wall view)
        let tri_agrees = hit.is_some_and(|h| {
            h.2 == Hit::Edge
                && self.estimates.get(&o.id).and_then(|v| triangulate(v)).is_some_and(|t| {
                    t.spread >= tp.min_triangulation_spread.max(0.1)
                        && (t.x - o.camera.0 - dx * h.0).hypot(t.y - o.camera.1 - dy * h.0) <= tp.edge_triangulation_gap
                })
        });
        // clean wall hits and triangulated edge hits teach the scale
        if let Some(r) = ratio.filter(|_| hit.is_some_and(|h| h.2 == Hit::Clean) || tri_agrees) {
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
        // edge hit: check against this marker's scale, else any marker's (same print size), else
        // marker size close to configured one
        let hit = hit.filter(|h| {
            h.2 != Hit::Edge
                || (tri_agrees && ratio.is_some())
                || ratio.is_some_and(|q| match learned.or_else(|| self.any_scale()) {
                    Some(k) => (q / k - 1.0).abs() <= tp.scale_tolerance,
                    None => (1.0 / tp.edge_ratio_gate..=tp.edge_ratio_gate).contains(&q),
                })
        });
        let ratio = ratio.filter(|_| hit.is_some());
        let est = match (hit, ratio, learned) {
            (Some((r, along, _)), Some(q), Some(k)) if (q / k - 1.0).abs() <= tp.scale_tolerance => Some(at(r, Source::Raycast, along)),
            (_, _, Some(k)) => (pnp * k <= p.max_range).then(|| at(pnp * k, Source::ScaledPnp, pnp * k * tp.pnp_sigma)),
            (Some((r, along, _)), Some(_), None) => Some(at(r, Source::Raycast, along)),
            // unknown marker size: range barely constrained, bearing still counts
            _ => (p.allow_pnp_fallback && pnp <= p.max_range).then(|| at(pnp, Source::PnpRange, pnp * 0.5)),
        };
        if let Some(e) = &est {
            self.add(o.id, e);
        }
        est
    }

    /// views per source, in Source order: raycast, pnp, scaled pnp
    pub fn sources(&self, id: i64) -> [usize; 3] {
        let mut n = [0; 3];
        for e in self.estimates.get(&id).into_iter().flatten() {
            n[e.source as usize] += 1;
        }
        n
    }

    /// views kept for one id, oldest first
    pub fn estimates(&self, id: i64) -> &[Estimate] {
        self.estimates.get(&id).map_or(&[], |v| v.as_slice())
    }

    /// median ray/pnp scale over all markers, they are printed same size
    pub fn any_scale(&self) -> Option<f64> {
        let all: Vec<f64> = self.scales.values().flatten().copied().collect();
        (all.len() >= self.params.scale_min_samples).then(|| median(all))
    }

    /// learned scale whose samples agree: median absolute deviation within scale_tolerance of the median
    pub fn scale_consistent(&self, id: i64) -> bool {
        let Some(v) = self.scales.get(&id).filter(|v| v.len() >= self.params.scale_min_samples) else { return false };
        let m = median(v.clone());
        let mad = median(v.iter().map(|s| (s - m).abs()).collect());
        m > 0.0 && mad <= self.params.scale_tolerance * m
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
        let ok = self.target(id).is_some_and(|t| t.confirmed);
        if ok {
            self.sticky.insert(id);
        } else {
            self.sticky.remove(&id);
        }
    }

    pub fn target(&self, id: i64) -> Option<Target> {
        let v = self.estimates.get(&id)?;
        let f = fuse(v);
        // once confirmed, keep it unless views clearly fall apart (mission already counted it)
        let loose = if self.sticky.contains(&id) { 2.0 } else { 1.0 };
        let p = &self.params;
        let wall = v.iter().filter(|e| e.source == Source::Raycast).count();
        let tri = triangulate(v).filter(|t| (t.x - f.x).hypot(t.y - f.y) <= loose * p.max_triangulation_gap);
        // lidar-backed: enough wall views, and every id that needs it has a consistent learned size
        let wall_ok = wall >= p.min_wall_views && wall as f64 >= p.min_wall_fraction * v.len() as f64
            && (!p.strict_ids.contains(&id) || self.scale(id).is_some())
            && (!p.require_scale_all || self.scale_consistent(id));
        // camera-only: no wall support, but the bearings of many views cross wide apart (not for strict ids)
        let cam_ok = p.pnp_only_spread > 0.0 && !p.strict_ids.contains(&id)
            && tri.is_some_and(|t| t.spread >= p.pnp_only_spread);
        Some(Target {
            id,
            x: f.x,
            y: f.y,
            views: v.len(),
            spread: f.sigma,
            confirmed: v.len() >= p.min_views
                && (wall_ok || cam_ok)
                && (p.min_triangulation_spread <= 0.0 || tri.is_some_and(|t| t.spread >= p.min_triangulation_spread))
                && f.sigma <= loose * p.max_spread
                && f.consistency <= loose * p.max_consistency,
        })
    }

    /// Rules target `id` still fails, short and in rule order ("views 3/5", "wall 1/3", ...).
    /// Empty when confirmed or unknown. Shown with candidates so a real marker that never
    /// confirms can be told apart from a rejected decoy.
    pub fn pending(&self, id: i64) -> Vec<String> {
        let Some(v) = self.estimates.get(&id) else { return Vec::new() };
        if self.target(id).is_some_and(|t| t.confirmed) {
            return Vec::new();
        }
        let p = &self.params;
        let f = fuse(v);
        let n = v.len();
        let wall = v.iter().filter(|e| e.source == Source::Raycast).count();
        let need_wall = p.min_wall_views.max((p.min_wall_fraction * n as f64).ceil() as usize);
        let mut out = Vec::new();
        if n < p.min_views {
            out.push(format!("views {n}/{}", p.min_views));
        }
        if wall < need_wall {
            out.push(format!("wall {wall}/{need_wall}"));
        }
        if p.strict_ids.contains(&id) && self.scale(id).is_none() {
            out.push("size unknown".to_owned());
        } else if p.require_scale_all && !self.scale_consistent(id) {
            out.push(if self.scale(id).is_some() { "size mismatch" } else { "size unknown" }.to_owned());
        }
        if p.min_triangulation_spread > 0.0 {
            match triangulate(v) {
                Some(t) if t.spread < p.min_triangulation_spread => out.push(format!(
                    "angle {:.0}/{:.0} deg",
                    t.spread.to_degrees(),
                    p.min_triangulation_spread.to_degrees()
                )),
                Some(t) if (t.x - f.x).hypot(t.y - f.y) > p.max_triangulation_gap => out.push("rays disagree".to_owned()),
                None => out.push("one angle".to_owned()),
                _ => {}
            }
        }
        if f.sigma > p.max_spread {
            out.push(format!("spread {:.0} cm", f.sigma * 100.0));
        }
        if f.consistency > p.max_consistency {
            out.push("views disagree".to_owned());
        }
        out
    }

    pub fn targets(&self) -> Vec<Target> {
        self.estimates.keys().filter_map(|&id| self.target(id)).collect()
    }

    pub fn confirmed(&self) -> Vec<Target> {
        self.targets().into_iter().filter(|t| t.confirmed).collect()
    }

    /// Where to stand to look at target `id` from `standoff` m: in front of it, on the side its views
    /// came from, facing it. (x, y, yaw). None without views.
    pub fn approach(&self, id: i64, standoff: f64) -> Option<(f64, f64, f64)> {
        let v = self.estimates.get(&id).filter(|v| !v.is_empty())?;
        let t = self.target(id)?;
        let (sx, sy) = v.iter().fold((0.0, 0.0), |a, e| (a.0 + e.dir.0, a.1 + e.dir.1));
        let n = sx.hypot(sy);
        if n < 1e-9 {
            return None;
        }
        let (ux, uy) = (sx / n, sy / n);
        Some((t.x - standoff * ux, t.y - standoff * uy, uy.atan2(ux)))
    }

    /// Direction target `id` faces (yaw of its face normal, pointing to the side it was seen from).
    /// With a map: normal of the wall line through the occupied cells within 0.25 m of the fused
    /// position, if there is one and it is within 75 deg of the viewing side. Else (no wall, a
    /// corner, a label in mid-air): the reverse of the mean view direction. None without views.
    pub fn facing(&self, id: i64, grid: Option<&Grid>, occupied_threshold: i8) -> Option<f64> {
        let v = self.estimates.get(&id).filter(|v| !v.is_empty())?;
        let t = self.target(id)?;
        let (sx, sy) = v.iter().fold((0.0, 0.0), |a, e| (a.0 - e.dir.0, a.1 - e.dir.1));
        let n = sx.hypot(sy);
        if n < 1e-9 {
            return None;
        }
        let back = (sx / n, sy / n);
        let fallback = back.1.atan2(back.0);
        let Some(g) = grid else { return Some(fallback) };
        let r = 0.25;
        let res = g.meta.resolution;
        let (cx, cy) = g.world_to_cell_signed(t.x, t.y);
        let k = (r / res).ceil() as i64;
        let mut pts = Vec::new();
        for y in cy - k..=cy + k {
            for x in cx - k..=cx + k {
                if let Some(c) = g.cell_checked(x, y).filter(|c| g.get(*c) >= occupied_threshold) {
                    let w = g.cell_to_world(c);
                    if (w.0 - t.x).hypot(w.1 - t.y) <= r {
                        pts.push(w);
                    }
                }
            }
        }
        if pts.len() < 4 {
            return Some(fallback);
        }
        let m = pts.len() as f64;
        let (mx, my) = (pts.iter().map(|p| p.0).sum::<f64>() / m, pts.iter().map(|p| p.1).sum::<f64>() / m);
        let (mut cxx, mut cyy, mut cxy) = (0.0, 0.0, 0.0);
        for (x, y) in &pts {
            cxx += (x - mx).powi(2);
            cyy += (y - my).powi(2);
            cxy += (x - mx) * (y - my);
        }
        let disc = ((cxx - cyy).powi(2) / 4.0 + cxy * cxy).sqrt();
        let (l1, l2) = ((cxx + cyy) / 2.0 + disc, (cxx + cyy) / 2.0 - disc);
        if l1 <= 0.0 || l2 / l1 > 0.15 {
            return Some(fallback);
        }
        let ang = 0.5 * (2.0 * cxy).atan2(cxx - cyy);
        let (mut nx, mut ny) = (-ang.sin(), ang.cos());
        if nx * back.0 + ny * back.1 < 0.0 {
            (nx, ny) = (-nx, -ny);
        }
        if nx * back.0 + ny * back.1 < 75f64.to_radians().cos() {
            return Some(fallback);
        }
        Some(ny.atan2(nx))
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Triangulated {
    pub x: f64,
    pub y: f64,
    /// widest angle between view bearings (rad)
    pub spread: f64,
}

/// Bearing-only position: least-squares intersection of the views' bearing lines (each passes
/// through its estimate along `dir`, so the range of the view does not matter). None if the lines
/// are (nearly) parallel.
pub fn triangulate(v: &[Estimate]) -> Option<Triangulated> {
    if v.len() < 2 {
        return None;
    }
    let (mut a11, mut a12, mut a22, mut b1, mut b2) = (0.0, 0.0, 0.0, 0.0, 0.0);
    for e in v {
        let (ux, uy) = e.dir;
        // projector onto the line normal: I - u u^T
        let (p11, p12, p22) = (1.0 - ux * ux, -ux * uy, 1.0 - uy * uy);
        a11 += p11;
        a12 += p12;
        a22 += p22;
        b1 += p11 * e.x + p12 * e.y;
        b2 += p12 * e.x + p22 * e.y;
    }
    let det = a11 * a22 - a12 * a12;
    if det.abs() < 1e-6 * (a11 + a22).powi(2) {
        return None;
    }
    let angles: Vec<f64> = v.iter().map(|e| e.dir.1.atan2(e.dir.0)).collect();
    let mut spread: f64 = 0.0;
    for (i, a) in angles.iter().enumerate() {
        for b in &angles[i + 1..] {
            spread = spread.max((a - b).sin().atan2((a - b).cos()).abs());
        }
    }
    Some(Triangulated { x: (a22 * b1 - a12 * b2) / det, y: (a11 * b2 - a12 * b1) / det, spread })
}

/// robust weighted least squares over views, each a point with its own along/across ray sigma.
/// starts from coordinate median, huber reweighting drops bad views
#[derive(Debug, Clone, Copy)]
pub struct Fused {
    pub x: f64,
    pub y: f64,
    /// 1 sigma of fused position along its worst axis (m)
    pub sigma: f64,
    /// median mahalanobis distance of views to fused position, ~1 when views agree
    pub consistency: f64,
}

pub fn fuse(v: &[Estimate]) -> Fused {
    let med = (median(v.iter().map(|p| p.x).collect()), median(v.iter().map(|p| p.y).collect()));
    if v.len() < 2 {
        let s = v.first().map_or(f64::INFINITY, |e| e.sigma_along.max(e.sigma_across));
        return Fused { x: med.0, y: med.1, sigma: s, consistency: 0.0 };
    }
    // along info divided by `shared`
    let info = |e: &Estimate, shared: f64| {
        let (ux, uy) = e.dir;
        let (a, c) = (1.0 / (shared * e.sigma_along.max(1e-3).powi(2)), 1.0 / e.sigma_across.max(1e-3).powi(2));
        // a u u^T + c n n^T with n = (-uy, ux)
        (a * ux * ux + c * uy * uy, (a - c) * ux * uy, a * uy * uy + c * ux * ux)
    };
    const K: f64 = 2.0;
    let dist = |x: (f64, f64), e: &Estimate| {
        let (j11, j12, j22) = info(e, 1.0);
        let (ex, ey) = (x.0 - e.x, x.1 - e.y);
        (ex * (j11 * ex + j12 * ey) + ey * (j12 * ex + j22 * ey)).max(0.0).sqrt()
    };
    // pnp range is off by the unknown marker size, the same factor in every view. more pnp views
    // do not average that out, so for the spread all of them together count as one view along the
    // ray: pnp alone never gets tight, bearings from different places (or a wall hit) have to.
    // position still uses each view as before
    let pnp_views = v.iter().filter(|e| e.source == Source::PnpRange).count().max(1) as f64;
    // weighted least squares step, weight per view from its distance (None = all 1).
    // returns new position and its covariance
    let step = |x: Option<(f64, f64)>, hard: bool| {
        let (mut a11, mut a12, mut a22, mut b1, mut b2) = (0.0, 0.0, 0.0, 0.0, 0.0);
        let (mut s11, mut s12, mut s22) = (0.0, 0.0, 0.0);
        for e in v {
            let (j11, j12, j22) = info(e, 1.0);
            let w = match x.map(|x| dist(x, e)) {
                None => 1.0,
                Some(d) if d <= K => 1.0,
                Some(d) if hard && d > 3.0 * K => 0.0,
                Some(d) => K / d,
            };
            a11 += w * j11;
            a12 += w * j12;
            a22 += w * j22;
            b1 += w * (j11 * e.x + j12 * e.y);
            b2 += w * (j12 * e.x + j22 * e.y);
            let (k11, k12, k22) = info(e, if e.source == Source::PnpRange { pnp_views } else { 1.0 });
            s11 += w * k11;
            s12 += w * k12;
            s22 += w * k22;
        }
        let det = a11 * a22 - a12 * a12;
        let sdet = s11 * s22 - s12 * s12;
        let cov = if sdet.abs() >= 1e-12 { (s22 / sdet, -s12 / sdet, s11 / sdet) } else { (f64::INFINITY, 0.0, f64::INFINITY) };
        (det.abs() >= 1e-12).then(|| (((a22 * b1 - a12 * b2) / det, (a11 * b2 - a12 * b1) / det), cov))
    };
    // huber first, then drop views past 3K so far outliers stop pulling at all
    let solve = |mut x: (f64, f64)| {
        let mut cov = (f64::INFINITY, 0.0, f64::INFINITY);
        for it in 0..25 {
            let hard = it >= 15;
            let Some((nx, c)) = step(Some(x), hard) else { break };
            cov = c;
            let moved = (nx.0 - x.0).hypot(nx.1 - x.1);
            x = nx;
            if moved < 1e-5 && hard {
                break;
            }
        }
        // robust cost: huber, flat past 3K
        let cost: f64 = v
            .iter()
            .map(|e| match dist(x, e) {
                d if d <= K => 0.5 * d * d,
                d => K * (d.min(3.0 * K) - 0.5 * K),
            })
            .sum();
        (x, cov, cost)
    };
    // median start lets many weak views outvote few precise ones, plain weighted start the reverse.
    // try both, keep the better fit
    let mut best = solve(med);
    if let Some((x, _)) = step(None, false) {
        let other = solve(x);
        if other.2 < best.2 - 1e-9 {
            best = other;
        }
    }
    let (x, cov, _) = best;
    let tr = cov.0 + cov.2;
    let l = tr / 2.0 + ((cov.0 - cov.2).powi(2) / 4.0 + cov.1 * cov.1).sqrt();
    // same for agreement: a pnp view short or long along its ray is the marker size, not a view
    // that disagrees. across the ray it has to fit like any other
    let ds: Vec<f64> = v
        .iter()
        .map(|e| {
            let (j11, j12, j22) = info(e, if e.source == Source::PnpRange { pnp_views } else { 1.0 });
            let (ex, ey) = (x.0 - e.x, x.1 - e.y);
            (ex * (j11 * ex + j12 * ey) + ey * (j12 * ex + j22 * ey)).max(0.0).sqrt()
        })
        .collect();
    Fused { x: x.0, y: x.1, sigma: l.max(0.0).sqrt(), consistency: median(ds) }
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
        // all views share one bearing here: the spread rule has its own test
        let mut t = Tracker::new(TrackParams { min_views: 3, min_triangulation_spread: 0.0, ..Default::default() });
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

    /// view of a marker at `m` from camera `c`, placed at `m + err` along the ray (wrong range)
    fn view_at(c: (f64, f64), m: (f64, f64), err: f64, source: Source) -> Estimate {
        let (dx, dy) = (m.0 - c.0, m.1 - c.1);
        let r = dx.hypot(dy);
        let u = (dx / r, dy / r);
        Estimate { x: m.0 + u.0 * err, y: m.1 + u.1 * err, source, dir: u, sigma_along: 0.05, sigma_across: 0.02 }
    }

    #[test]
    fn triangulation_finds_the_marker_from_bearings_alone() {
        let m = (3.0, 1.0);
        // ranges wrong by up to 0.8 m, bearings right
        let v: Vec<_> = [((0.0, 0.0), 0.5), ((0.0, 2.0), -0.8), ((1.0, -1.0), 0.3)]
            .iter()
            .map(|&(c, e)| view_at(c, m, e, Source::PnpRange))
            .collect();
        let t = triangulate(&v).unwrap();
        assert!((t.x - m.0).abs() < 1e-9 && (t.y - m.1).abs() < 1e-9, "{t:?}");
        assert!(t.spread > 0.8, "views from 3 sides");
        // all from one spot: no intersection
        let same: Vec<_> = (0..3).map(|k| view_at((0.0, 0.0), m, 0.1 * k as f64, Source::PnpRange)).collect();
        assert!(triangulate(&same).is_none());
    }

    #[test]
    fn camera_only_confirms_from_wide_bearings_without_a_wall() {
        let m = (3.0, 1.0);
        // five camera spots spread around the marker, ranges wrong by up to 0.3 m, no lidar wall hit
        let spots = [((0.0, 0.0), 0.3), ((0.0, 2.0), -0.3), ((1.0, -1.0), 0.2), ((-0.5, 1.5), -0.1), ((0.5, -0.5), 0.1)];
        let views: Vec<_> = spots.iter().map(|&(c, e)| view_at(c, m, e, Source::PnpRange)).collect();
        let mut plain = Tracker::new(TrackParams::default());
        let mut cam = Tracker::new(TrackParams { pnp_only_spread: 0.35, ..TrackParams::default() });
        for e in &views {
            plain.add(7, e);
            cam.add(7, e);
        }
        assert!(!plain.target(7).unwrap().confirmed, "off by default: no wall views");
        assert!(cam.target(7).unwrap().confirmed, "wide bearings confirm when enabled");
        // the same views from one bearing: no triangulation, no confirmation
        let mut one = Tracker::new(TrackParams { pnp_only_spread: 0.35, ..TrackParams::default() });
        for k in 0..5 {
            one.add(7, &view_at((0.0, 0.0), m, 0.02 * k as f64, Source::PnpRange));
        }
        assert!(!one.target(7).unwrap().confirmed, "one bearing never confirms");
    }

    #[test]
    fn rays_landing_behind_the_pattern_fail_the_triangulation_check() {
        // a label at (2, 0); every "wall hit" lands on a wall 1 m behind it, along each ray
        let label: (f64, f64) = (2.0, 0.0);
        let mut t = Tracker::new(TrackParams { min_views: 3, min_wall_views: 3, max_spread: 1.0, max_consistency: 100.0, ..Default::default() });
        for c in [(0.0, -1.0), (0.0, -0.5), (0.0, 0.0), (0.0, 0.5), (0.0, 1.0)] {
            let r = (label.0 - c.0).hypot(label.1 - c.1);
            t.add(1, &view_at(c, label, 1.0 * r / 2.0, Source::Raycast));
        }
        assert!(!t.target(1).unwrap().confirmed, "{:?} vs {:?}", t.target(1), triangulate(t.estimates(1)));
        // the same views placed on the marker itself pass
        let mut t = Tracker::new(TrackParams { min_views: 3, min_wall_views: 3, min_triangulation_spread: 0.0, ..Default::default() });
        for c in [(0.0, -1.0), (0.0, -0.5), (0.0, 0.0), (0.0, 0.5), (0.0, 1.0)] {
            t.add(2, &view_at(c, label, 0.0, Source::Raycast));
        }
        assert!(t.target(2).unwrap().confirmed);
    }

    #[test]
    fn strict_ids_need_a_learned_marker_size() {
        let m = (2.4, -1.3);
        let views = |t: &mut Tracker, id: i64| {
            for k in 0..8 {
                t.add(id, &view_at((0.0, -1.0 + 0.3 * k as f64), m, 0.0, Source::Raycast));
            }
        };
        let mut t = Tracker::new(TrackParams { strict_ids: vec![1023], ..Default::default() });
        views(&mut t, 1023);
        views(&mut t, 7);
        assert!(!t.target(1023).unwrap().confirmed, "stripe id without a learned size");
        assert!(t.target(7).unwrap().confirmed, "same views, ordinary id");
    }

    #[test]
    fn strict_id_on_a_wall_confirms_once_its_size_is_learned() {
        // a real id 0 printed on the wall at x = 3, seen while the robot drives past
        let g = wall_map();
        let mut t = Tracker::new(TrackParams { strict_ids: vec![0], ..Default::default() });
        let p = LocateParams::default();
        let m = (3.0, 0.45);
        for cy in [-1.0, -0.6, -0.2, 0.2, 0.6, 1.0, 1.4] {
            let (dx, dy): (f64, f64) = (m.0, m.1 - cy);
            let r = dx.hypot(dy);
            // marker is 2x the configured size: pnp says half the range, consistently
            t.observe(Some(&g), &Observation { id: 0, camera: (0.0, cy, 0.25), bearing: (dx / r, dy / r, 0.0), pnp_range: r * 0.5 }, &p);
        }
        assert!(t.scale(0).is_some(), "size learned from clean wall hits");
        let tg = t.target(0).unwrap();
        assert!(tg.confirmed, "{tg:?}");
        assert!((tg.x - 3.0).abs() < 0.1 && (tg.y - 0.45).abs() < 0.1);
    }

    #[test]
    fn views_from_one_spot_never_confirm() {
        let m = (2.4, -1.3);
        let mut t = Tracker::new(TrackParams::default());
        // 7 perfect wall-backed views, robot nearly still (bearings within a few degrees)
        for k in 0..7 {
            t.add(1, &view_at((0.0, 0.02 * k as f64), m, 0.0, Source::Raycast));
        }
        assert!(!t.target(1).unwrap().confirmed, "{:?}", triangulate(t.estimates(1)));
        // the robot drives on 1 m: bearings now spread, same marker confirms
        for k in 0..4 {
            t.add(1, &view_at((0.25 * k as f64, 0.6), m, 0.0, Source::Raycast));
        }
        assert!(t.target(1).unwrap().confirmed);
    }

    #[test]
    fn pending_names_the_rules_a_candidate_still_fails() {
        let m = (2.4, -1.3);
        let mut t = Tracker::new(TrackParams::default());
        assert!(t.pending(1).is_empty(), "unknown id");
        t.add(1, &Estimate::point(2.4, -1.3, Source::PnpRange));
        let p = t.pending(1);
        assert!(p.iter().any(|r| r.starts_with("views 1/")), "{p:?}");
        assert!(p.iter().any(|r| r.starts_with("wall 0/")), "{p:?}");
        // wall-backed views from one spot: only the angle is missing
        let mut t = Tracker::new(TrackParams::default());
        for k in 0..7 {
            t.add(1, &view_at((0.0, 0.02 * k as f64), m, 0.0, Source::Raycast));
        }
        let p = t.pending(1);
        assert!(p.iter().any(|r| r.starts_with("angle ")), "{p:?}");
        assert!(!p.iter().any(|r| r.starts_with("views") || r.starts_with("wall")), "{p:?}");
        for k in 0..4 {
            t.add(1, &view_at((0.25 * k as f64, 0.6), m, 0.0, Source::Raycast));
        }
        assert!(t.target(1).unwrap().confirmed);
        assert!(t.pending(1).is_empty());
    }

    #[test]
    fn approach_point_stands_in_front_on_the_viewing_side() {
        let mut t = Tracker::new(TrackParams::default());
        let m = (3.0, 1.0);
        for c in [(0.0, 0.5), (0.0, 1.5)] {
            t.add(7, &view_at(c, m, 0.0, Source::Raycast));
        }
        let (x, y, yaw) = t.approach(7, 1.3).unwrap();
        assert!(((x - m.0).hypot(y - m.1) - 1.3).abs() < 0.05);
        assert!(x < m.0, "on the side the robot looked from");
        assert!(yaw.abs() < 0.05, "facing +x toward the marker");
    }

    #[test]
    fn facing_is_the_wall_normal_not_the_view_direction() {
        let g = wall_map();
        let mut t = Tracker::new(TrackParams::default());
        // marker on the wall at x = 3, seen only from the lower left at about 30 deg off its normal
        let m = (3.0, 1.0);
        for c in [(0.4, -0.5), (0.5, -0.4), (0.3, -0.45)] {
            t.add(7, &view_at(c, m, 0.0, Source::Raycast));
        }
        let wall = t.facing(7, Some(&g), 65).unwrap();
        assert!((wall.cos() + 1.0).abs() < 0.02, "faces -x: {wall}");
        // without a map: back along the views
        let views = t.facing(7, None, 65).unwrap();
        assert!((views - (-1.5f64).atan2(-2.6)).abs() < 0.05, "{views}");
    }

    #[test]
    fn hits_inside_the_robot_are_ignored() {
        // occupied cell right in front of the camera (robot footprint / ghost), marker on a wall at 3 m
        let mut g = wall_map();
        let c = g.world_to_cell(0.15, 0.45).unwrap();
        g.set(c, 100);
        let mut t = Tracker::new(TrackParams::default());
        let p = LocateParams::default();
        let o = Observation { id: 3, camera: (0.0, 0.45, 0.25), bearing: (1.0, 0.0, 0.0), pnp_range: 3.0 };
        let e = t.observe(Some(&g), &o, &p).unwrap();
        assert!(e.x > 1.0, "not placed at the robot: {e:?}");
        // a pattern held 0.2 m from the camera gives nothing at all
        let near = Observation { id: 4, pnp_range: 0.2, ..o };
        assert!(t.observe(Some(&g), &near, &p).is_none());
    }

    #[test]
    fn confirmation_needs_wall_supported_views() {
        let mut t = Tracker::new(TrackParams { min_views: 3, min_wall_views: 3, min_triangulation_spread: 0.0, ..Default::default() });
        let pnp = |x, y| Estimate::point(x, y, Source::PnpRange);
        let ray = |x, y| Estimate::point(x, y, Source::Raycast);
        // a tight cluster of pnp-only views (pattern with nothing behind it in the scan): never a target
        for k in 0..8 {
            t.add(4, &pnp(2.0 + 0.002 * k as f64, 1.0));
        }
        assert!(!t.target(4).unwrap().confirmed);
        t.add(4, &ray(2.0, 1.0));
        t.add(4, &ray(2.01, 1.0));
        assert!(!t.target(4).unwrap().confirmed, "two wall views are not enough");
        t.add(4, &ray(2.0, 1.01));
        assert!(t.target(4).unwrap().confirmed, "3 of 11 views lidar-backed (27 %)");
        // same count, smaller share: 3 lidar-backed of 15 (20 %) is not enough
        let mut t = Tracker::new(TrackParams { min_views: 3, min_wall_views: 3, min_triangulation_spread: 0.0, ..Default::default() });
        for k in 0..12 {
            t.add(6, &pnp(2.0 + 0.002 * k as f64, 1.0));
        }
        for _ in 0..3 {
            t.add(6, &ray(2.0, 1.0));
        }
        assert!(!t.target(6).unwrap().confirmed, "20 % lidar-backed");
        // the chair label from field run 184838: 3 lidar-backed views out of 39
        let mut t = Tracker::new(TrackParams { min_triangulation_spread: 0.0, ..Default::default() });
        for k in 0..36 {
            t.add(5, &pnp(3.4 + 0.001 * (k % 5) as f64, -2.7));
        }
        for _ in 0..3 {
            t.add(5, &ray(3.4, -2.7));
        }
        assert!(!t.target(5).unwrap().confirmed);
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
    fn wall_fit_works_with_unknown_in_front() {
        // live slam map: cell right in front of a wall hit is never cleared
        let mut g = wall_map();
        for cy in 0..140 {
            g.set(g.world_to_cell(2.96, -3.0 + cy as f64 * 0.05 + 0.01).unwrap(), -1);
        }
        let p = LocateParams::default();
        let (dx, dy) = (3.0f64, 0.7f64);
        let n = dx.hypot(dy);
        let u = (dx / n, dy / n);
        let h = g.raycast(0.0, 0.0, u.0, u.1, 5.0, 65).unwrap();
        let (t, _) = wall_hit(&g, (0.0, 0.0), u, h.range, &p).expect("fit with unknown in front");
        assert!((t * u.0 - 3.025).abs() < 0.01, "{}", t * u.0);
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
    fn ray_clipping_obstacle_end_is_not_trusted() {
        // wall at x = 3, short obstacle at x = 1.5 for y in [0.3, 0.6)
        let mut g = wall_map();
        for cy in 0..6 {
            g.set(g.world_to_cell(1.51, 0.31 + cy as f64 * 0.05).unwrap(), 100);
        }
        let p = LocateParams::default();
        let at = |y: f64| {
            let a = y.atan2(1.5);
            let h = g.raycast(0.0, 0.0, a.cos(), a.sin(), 5.0, 65).unwrap();
            flat_around(&g, (0.0, 0.0), a, h.range, &p)
        };
        assert!(at(-0.5), "open wall");
        assert!(!at(0.58), "end of obstacle");
        assert!(!at(0.62), "just past obstacle end");
        let kind = |y: f64| {
            let a = y.atan2(1.5);
            let h = g.raycast(0.0, 0.0, a.cos(), a.sin(), 5.0, 65).unwrap();
            surface_around(&g, (0.0, 0.0), a, h.range, &p)
        };
        assert_eq!(kind(0.55), Surface::Edge, "near obstacle end, rest of face beside");
        assert_eq!(kind(0.64), Surface::Clip, "wall seen past obstacle end");
    }

    /// box face 15 cm wide at x = 1.8, wall at x = 3 behind. markers sit on face near its ends
    fn box_map() -> Grid {
        let mut g = wall_map();
        for cy in 0..3 {
            g.set(g.world_to_cell(1.81, 0.41 + cy as f64 * 0.05).unwrap(), 100);
            g.set(g.world_to_cell(1.86, 0.41 + cy as f64 * 0.05).unwrap(), 100);
        }
        g
    }

    fn view_of(y: f64, pnp_scale: f64, cam_y: f64) -> Observation {
        let (dx, dy) = (1.8f64, y - cam_y);
        let r = dx.hypot(dy);
        Observation { id: 3, camera: (0.0, cam_y, 0.25), bearing: (dx / r, dy / r, 0.0), pnp_range: r * pnp_scale }
    }

    #[test]
    fn marker_on_small_box_face_uses_raycast() {
        let g = box_map();
        let p = LocateParams::default();
        let mut t = Tracker::new(TrackParams::default());
        // camera moves sideways, marker at y = 0.45 near low end of face, marker printed 20% bigger
        for k in 0..8 {
            let e = t.observe(Some(&g), &view_of(0.45, 0.8, -0.2 + 0.06 * k as f64), &p).unwrap();
            assert_eq!(e.source, Source::Raycast, "view {k}");
        }
        let tg = t.target(3).unwrap();
        assert!(tg.confirmed, "{tg:?}");
        assert!((tg.x - 1.825).abs() < 0.03 && (tg.y - 0.45).abs() < 0.03, "{tg:?}");
    }

    #[test]
    fn bigger_marker_on_a_box_face_confirms_once_bearings_cross() {
        // marker printed 2x marker_size on a box face: pnp says half the range, so the edge gate
        // alone drops every lidar hit and the scale is never learned. Driving sideways, the bearings
        // cross at the face, the hits count and the size is learned
        let g = box_map();
        let p = LocateParams::default();
        let mut t = Tracker::new(TrackParams { require_scale_all: true, ..Default::default() });
        for k in 0..20 {
            t.observe(Some(&g), &view_of(0.45, 0.5, -0.6 + 0.06 * k as f64), &p);
        }
        let tg = t.target(3).unwrap();
        assert!(tg.confirmed, "{tg:?} {:?}", t.pending(3));
        assert!((tg.x - 1.8).abs() < 0.05 && (tg.y - 0.45).abs() < 0.05, "{tg:?}");
        assert!(t.scale(3).is_some_and(|k| (k - 2.0).abs() < 0.2), "{:?}", t.scale(3));
    }

    #[test]
    fn edge_hit_with_wrong_pnp_range_is_dropped() {
        // ray ends at box edge but pnp says marker is 2x farther: wall behind, ray clips box
        let g = box_map();
        let p = LocateParams::default();
        let mut t = Tracker::new(TrackParams::default());
        for k in 0..8 {
            let e = t.observe(Some(&g), &view_of(0.42, 2.0, -0.2 + 0.06 * k as f64), &p);
            assert!(e.is_none_or(|e| e.source != Source::Raycast), "view {k}: {e:?}");
        }
    }

    #[test]
    fn few_precise_views_beat_more_weak_ones() {
        let ray = Estimate { x: 2.0, y: 0.0, source: Source::Raycast, dir: (1.0, 0.0), sigma_along: 0.04, sigma_across: 0.02 };
        let weak = |x: f64| Estimate { x, source: Source::PnpRange, sigma_along: 0.8, ..ray };
        let f = fuse(&[ray, weak(1.3), weak(1.4), ray, weak(1.2)]);
        assert!((f.x - 2.0).abs() < 0.03, "{f:?}");
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
        let Fused { x, y, sigma, .. } = fuse(&[a, b, a, b]);
        assert!(sigma < 0.03, "{sigma}");
        assert!((x - 2.0).abs() < 0.03 && y.abs() < 0.03, "{x} {y}");
    }

    #[test]
    fn weak_views_do_not_block_confirmation() {
        let mut t = Tracker::new(TrackParams::default());
        let good = |y: f64| {
            let r: f64 = 3.0f64.hypot(y);
            Estimate { x: 3.0, y: 0.0, source: Source::Raycast, dir: (3.0 / r, -y / r), sigma_along: 0.03, sigma_across: 0.03 }
        };
        // pnp with wrong marker size: 25% short, but straight along the ray
        let weak = |y: f64| {
            let r: f64 = 3.0f64.hypot(y);
            let u = (3.0 / r, -y / r);
            Estimate { x: 0.0 + u.0 * 0.75 * r, y: y + u.1 * 0.75 * r, source: Source::PnpRange, dir: u, sigma_along: 0.5 * r, sigma_across: 0.03 }
        };
        for y in [-0.6, -0.3, 0.0, 0.3, 0.6, 0.9] {
            t.add(1, &weak(y));
        }
        for y in [-0.5, 0.2, 0.7] {
            t.add(1, &good(y));
        }
        let tg = t.target(1).unwrap();
        assert!(tg.confirmed, "{tg:?}");
        assert!((tg.x - 3.0).abs() < 0.03 && tg.y.abs() < 0.03, "{tg:?}");
    }

    /// what mission gets from one /team7/targets message of marker_mapper
    fn publish(t: &Tracker, found: &mut Found) {
        found.update(t.targets().iter().map(|t| (t.ns(), t.id as i32)));
    }

    #[test]
    fn withdrawn_confirmation_no_longer_counts_as_found() {
        let mut t = Tracker::new(TrackParams { min_views: 3, min_triangulation_spread: 0.0, ..Default::default() });
        let e = |x, y| Estimate::point(x, y, Source::Raycast);
        let mut found = Found::default();
        for id in [1, 2] {
            for d in [-0.01, 0.0, 0.01] {
                t.add(id, &e(id as f64 + d, 0.0));
            }
        }
        publish(&t, &mut found);
        assert_eq!(found.len(), 2);
        // later views of id 1 land all over the place: marker_mapper logs LOST and drops its csv row
        for k in 0..10 {
            t.add(1, &e(1.0 + k as f64, 3.0));
        }
        assert!(!t.target(1).unwrap().confirmed);
        for d in [-0.01, 0.0, 0.01] {
            t.add(3, &e(3.0 + d, 0.0));
        }
        publish(&t, &mut found);
        // targets.csv has only 2 and 3, so mission must not stop with "3 targets found"
        assert_eq!(found.ids().iter().copied().collect::<Vec<_>>(), [2, 3]);
        assert_eq!(found.len(), to_csv(&t.confirmed()).lines().count() - 1);
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
