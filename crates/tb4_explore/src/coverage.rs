use std::collections::VecDeque;

use tb4_core::grid::{Cell, Grid, GridMeta};

#[derive(Debug, Clone)]
pub struct CoverageParams {
    pub half_fov: f64,
    pub range: f64,
    /// camera ahead of base along heading (m)
    pub cam_offset: f64,
    pub ray_step: f64,
    pub occupied_threshold: i8,
    pub min_cluster_cells: usize,
    /// long walls are split into pieces of at most this many cells, one view each
    pub max_cluster_cells: usize,
    /// distance from wall to stand when looking at it (m)
    pub standoff: f64,
    /// viewpoint must be at least this far from any obstacle (m)
    pub min_clearance: f64,
    /// a wall cell only counts as seen if the camera met it at most this far off its normal (rad);
    /// a marker seen edge-on does not decode
    pub max_incidence: f64,
    #[cfg(test)]
    plain_range: bool,
}

impl CoverageParams {
    /// only for comparison in tests: count a hit as seen by plain range, ignoring incidence
    #[cfg(test)]
    fn with_plain_range(self) -> Self {
        Self { plain_range: true, ..self }
    }
}

impl Default for CoverageParams {
    fn default() -> Self {
        Self {
            half_fov: 0.52,
            range: 2.5,
            cam_offset: 0.1,
            ray_step: 0.02,
            occupied_threshold: 65,
            min_cluster_cells: 6,
            max_cluster_cells: 24,
            standoff: 1.2,
            min_clearance: 0.35,
            max_incidence: 1.3,
            #[cfg(test)]
            plain_range: false,
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct View {
    pub x: f64,
    pub y: f64,
    pub yaw: f64,
    /// wall point this view looks at
    pub target: (f64, f64),
    /// unseen wall cells this view targets
    pub cells: usize,
    pub distance: f64,
}

/// world xy points
pub type Points = Vec<(f64, f64)>;

/// which occupied cells have been inside the camera view, kept in world coordinates so the map can grow
#[derive(Debug, Default, Clone)]
pub struct Coverage {
    seen: std::collections::HashSet<(i64, i64)>,
    /// per seen cell: bit k set when the camera has seen it from direction bin k (see `bin_of`)
    bins: std::collections::HashMap<(i64, i64), u8>,
    /// floor cells inside the camera cone, up to the wall or the reading distance (fog cleared)
    floor: std::collections::HashSet<(i64, i64)>,
    res: f64,
}

/// Direction bin of a camera seen from a wall cell. `a` is the ray heading (the direction the camera
/// looks, from camera to wall), so the direction from the cell to the camera is `a + PI`.
/// Bin k covers [k * 45, (k + 1) * 45) degrees, counted counter-clockwise from +x (east, +y north):
/// bin 0 = camera to the east of the cell, bin 2 = north, bin 4 = west, bin 6 = south.
/// Bin edges sit on multiples of 45 degrees, so a camera exactly due east straddles bins 7 and 0.
fn bin_of(a: f64) -> usize {
    let to_cam = (a + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU);
    ((to_cam / (std::f64::consts::PI / 4.0)) as usize) % 8
}

impl Coverage {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn seen_count(&self) -> usize {
        self.seen.len()
    }

    fn key(&self, x: f64, y: f64) -> (i64, i64) {
        ((x / self.res).floor() as i64, (y / self.res).floor() as i64)
    }

    pub fn is_seen(&self, g: &Grid, c: Cell) -> bool {
        if self.res <= 0.0 {
            return false;
        }
        let (x, y) = g.cell_to_world(c);
        self.seen.contains(&self.key(x, y))
    }

    /// bit mask of direction bins this cell has been seen from (bit k = bin k, see `bin_of`)
    fn bin_mask(&self, g: &Grid, c: Cell) -> u8 {
        if self.res <= 0.0 {
            return 0;
        }
        let (x, y) = g.cell_to_world(c);
        self.bins.get(&self.key(x, y)).copied().unwrap_or(0)
    }

    /// number of distinct direction bins (0 to 8) this cell has been seen from
    pub fn bins_seen(&self, g: &Grid, c: Cell) -> u8 {
        self.bin_mask(g, c).count_ones() as u8
    }

    /// direction bins (0 to 7, 45 degrees each) this cell has not been seen from yet
    pub fn unseen_bins(&self, g: &Grid, c: Cell) -> Vec<usize> {
        let m = self.bin_mask(g, c);
        (0..8usize).filter(|&k| (m & (1u8 << k)) == 0).collect()
    }

    /// mark wall cells the camera sees from pose (x, y, yaw)
    pub fn update(&mut self, g: &Grid, pose: (f64, f64, f64), p: &CoverageParams) {
        if self.res <= 0.0 || (self.res - g.meta.resolution).abs() > 1e-9 {
            self.res = g.meta.resolution;
        }
        let (cx, cy) = (pose.0 + p.cam_offset * pose.2.cos(), pose.1 + p.cam_offset * pose.2.sin());
        let n = ((2.0 * p.half_fov) / (0.5 * g.meta.resolution / p.range)).ceil().max(2.0) as usize;
        for i in 0..=n {
            let a = pose.2 - p.half_fov + 2.0 * p.half_fov * i as f64 / n as f64;
            let hit = g.raycast(cx, cy, a.cos(), a.sin(), p.range, p.occupied_threshold);
            // the floor this ray crossed, to the wall or as far as a marker could be read
            let reach = hit.as_ref().map_or(p.range, |h| h.range.min(p.range));
            let step = 0.5 * g.meta.resolution;
            let mut d = 0.0;
            while d < reach {
                let k = self.key(cx + d * a.cos(), cy + d * a.sin());
                self.floor.insert(k);
                d += step;
            }
            if let Some(hit) = hit {
                // step just inside the wall cell
                let (hx, hy) = (hit.x + a.cos() * 1e-3, hit.y + a.sin() * 1e-3);
                // A marker on the wall looks cos(incidence) narrower, like one range / cos further
                // away; `range` is how far a marker can be read head-on, so compare that.
                // no wall direction yet (a few cells of a box face in an early map): do not assume
                // head-on, a grazing pass would write the face off before the camera can read it
                let Some(cos_inc) = wall_direction(g, (hx, hy), p.occupied_threshold)
                    .map(|(wx, wy)| (a.cos() * wy - a.sin() * wx).abs())
                else {
                    continue;
                };
                #[cfg(test)]
                let cos_inc = if p.plain_range { 1.0 } else { cos_inc };
                if cos_inc < p.max_incidence.cos() || hit.range > p.range * cos_inc {
                    continue;
                }
                let k = self.key(hx, hy);
                self.seen.insert(k);
                *self.bins.entry(k).or_insert(0) |= 1u8 << bin_of(a);
            }
        }
    }

    /// fog of war over `g` (same size and origin): -1 where the map is unknown, 0 where the camera has
    /// seen (floor in its cone up to the reading distance, walls by the incidence rule), 100 elsewhere
    pub fn fog(&self, g: &Grid, occupied_threshold: i8) -> Vec<i8> {
        let (w, h) = (g.meta.width, g.meta.height);
        let mut out = Vec::with_capacity((w * h) as usize);
        for y in 0..h {
            for x in 0..w {
                let c = Cell { x, y };
                let v = g.get(c);
                out.push(if v < 0 {
                    -1
                } else if self.res <= 0.0 {
                    100
                } else {
                    let (px, py) = g.cell_to_world(c);
                    let k = self.key(px, py);
                    let seen = if v >= occupied_threshold { self.seen.contains(&k) } else { self.floor.contains(&k) };
                    if seen { 0 } else { 100 }
                });
            }
        }
        out
    }

    /// wall cells of `g` (occupied, next to free space), as (seen, not seen) by the camera, world xy
    pub fn wall_split(&self, g: &Grid, occupied_threshold: i8) -> (Points, Points) {
        let (w, h) = (g.meta.width as i64, g.meta.height as i64);
        let free = |c: Cell| (0..occupied_threshold).contains(&g.get(c));
        let (mut seen, mut unseen) = (Vec::new(), Vec::new());
        for y in 0..h {
            for x in 0..w {
                let c = Cell { x: x as u32, y: y as u32 };
                if g.get(c) < occupied_threshold {
                    continue;
                }
                let wall = [(1i64, 0i64), (-1, 0), (0, 1), (0, -1)]
                    .iter()
                    .any(|(dx, dy)| g.cell_checked(x + dx, y + dy).is_some_and(free));
                if wall {
                    let pt = g.cell_to_world(c);
                    if self.is_seen(g, c) { seen.push(pt) } else { unseen.push(pt) }
                }
            }
        }
        (seen, unseen)
    }

    /// clusters of occupied cells that touch free space but were never seen, with a viewpoint for each
    pub fn views(&self, g: &Grid, robot: (f64, f64), p: &CoverageParams) -> Vec<View> {
        let (w, h) = (g.meta.width as usize, g.meta.height as usize);
        let free = |c: Cell| (0..p.occupied_threshold).contains(&g.get(c));
        let occ = |c: Cell| g.get(c) >= p.occupied_threshold;
        let mut want = vec![false; w * h];
        for y in 0..h {
            for x in 0..w {
                let c = Cell { x: x as u32, y: y as u32 };
                if !occ(c) || self.is_seen(g, c) {
                    continue;
                }
                let touches_free = [(1i64, 0i64), (-1, 0), (0, 1), (0, -1)]
                    .iter()
                    .any(|(dx, dy)| g.cell_checked(x as i64 + dx, y as i64 + dy).is_some_and(free));
                want[g.index(c)] = touches_free;
            }
        }
        let clearance = tb4_core::DistanceField::from_grid(g, p.occupied_threshold, true);
        let mut seen = vec![false; w * h];
        let mut out = Vec::new();
        let mut q = VecDeque::new();
        for start in 0..w * h {
            if !want[start] || seen[start] {
                continue;
            }
            let mut members = Vec::new();
            seen[start] = true;
            q.push_back(start);
            while let Some(i) = q.pop_front() {
                members.push(i);
                let (x, y) = ((i % w) as i64, (i / w) as i64);
                for dy in -1..=1 {
                    for dx in -1..=1 {
                        if let Some(n) = g.cell_checked(x + dx, y + dy) {
                            let j = g.index(n);
                            if want[j] && !seen[j] {
                                seen[j] = true;
                                q.push_back(j);
                            }
                        }
                    }
                }
            }
            if members.len() < p.min_cluster_cells {
                continue;
            }
            // bfs order keeps each chunk contiguous along the wall
            for chunk in members.chunks(p.max_cluster_cells.max(1)) {
                if chunk.len() < p.min_cluster_cells.min(members.len()) {
                    continue;
                }
                let pts: Vec<(f64, f64)> =
                    chunk.iter().map(|&i| g.cell_to_world(Cell { x: (i % w) as u32, y: (i / w) as u32 })).collect();
                let n = pts.len() as f64;
                let mid = (pts.iter().map(|p| p.0).sum::<f64>() / n, pts.iter().map(|p| p.1).sum::<f64>() / n);
                // aim at the chunk cell nearest its centroid, so the target is on the wall
                let c = *pts.iter().min_by(|a, b| d2(**a, mid).total_cmp(&d2(**b, mid))).unwrap();
                if let Some(v) = viewpoint(g, &clearance, c, robot, chunk.len(), p) {
                    out.push(v);
                }
            }
        }
        out.sort_by(|a, b| (a.distance - 0.2 * a.cells as f64 * g.meta.resolution)
            .total_cmp(&(b.distance - 0.2 * b.cells as f64 * g.meta.resolution)));
        out
    }
}

/// unit direction along the wall at `pt`: principal axis of the occupied cells within 2 cells.
/// None if too few cells to tell (a single post).
fn wall_direction(g: &Grid, pt: (f64, f64), occupied: i8) -> Option<(f64, f64)> {
    let c = g.world_to_cell(pt.0, pt.1)?;
    let mut pts = Vec::with_capacity(25);
    for dy in -2..=2i64 {
        for dx in -2..=2i64 {
            if let Some(n) = g.cell_checked(c.x as i64 + dx, c.y as i64 + dy)
                && g.get(n) >= occupied
            {
                pts.push((dx as f64, dy as f64));
            }
        }
    }
    if pts.len() < 3 {
        return None;
    }
    let n = pts.len() as f64;
    let (mx, my) = (pts.iter().map(|p| p.0).sum::<f64>() / n, pts.iter().map(|p| p.1).sum::<f64>() / n);
    let (mut sxx, mut syy, mut sxy) = (0.0, 0.0, 0.0);
    for (x, y) in pts {
        sxx += (x - mx) * (x - mx);
        syy += (y - my) * (y - my);
        sxy += (x - mx) * (y - my);
    }
    // major axis angle of the 2x2 covariance
    let th = 0.5 * (2.0 * sxy).atan2(sxx - syy);
    Some((th.cos(), th.sin()))
}

fn d2(a: (f64, f64), b: (f64, f64)) -> f64 {
    (a.0 - b.0).powi(2) + (a.1 - b.1).powi(2)
}

/// free, clear point about `standoff` from target with line of sight to it, nearest to robot
fn viewpoint(
    g: &Grid,
    clearance: &tb4_core::DistanceField,
    target: (f64, f64),
    robot: (f64, f64),
    cells: usize,
    p: &CoverageParams,
) -> Option<View> {
    let meta: GridMeta = g.meta;
    let mut best: Option<View> = None;
    for ring in [p.standoff, p.standoff * 0.75, p.standoff * 1.4] {
        for k in 0..24 {
            let a = k as f64 * std::f64::consts::TAU / 24.0;
            let (x, y) = (target.0 + ring * a.cos(), target.1 + ring * a.sin());
            let Some(c) = g.world_to_cell(x, y) else { continue };
            if !(0..p.occupied_threshold).contains(&g.get(c)) || (clearance.get(c) as f64) < p.min_clearance {
                continue;
            }
            let (dx, dy) = (target.0 - x, target.1 - y);
            let d = dx.hypot(dy);
            match g.raycast(x, y, dx, dy, d + meta.resolution * 2.0, p.occupied_threshold) {
                Some(hit) if hit.range >= d - 3.0 * meta.resolution => {}
                _ => continue,
            }
            let dist = (x - robot.0).hypot(y - robot.1);
            if best.as_ref().is_none_or(|b| dist < b.distance) {
                best = Some(View { x, y, yaw: dy.atan2(dx), target, cells, distance: dist });
            }
        }
        if best.is_some() {
            break;
        }
    }
    best
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 6 x 4 m room, walls 1 cell thick, all known
    fn room() -> Grid {
        let meta = GridMeta { width: 120, height: 80, resolution: 0.05, origin_x: 0.0, origin_y: 0.0 };
        let mut g = Grid::filled(meta, 0);
        for x in 0..120 {
            g.set(Cell { x, y: 0 }, 100);
            g.set(Cell { x, y: 79 }, 100);
        }
        for y in 0..80 {
            g.set(Cell { x: 0, y }, 100);
            g.set(Cell { x: 119, y }, 100);
        }
        g
    }

    #[test]
    fn fog_clears_in_the_camera_cone_only() {
        let g = room();
        let mut cov = Coverage::new();
        let p = CoverageParams { range: 1.5, ..Default::default() };
        cov.update(&g, (3.0, 2.0, 0.0), &p);
        let fog = cov.fog(&g, 65);
        let at = |x: f64, y: f64| fog[g.index(g.world_to_cell(x, y).unwrap())];
        assert_eq!(at(3.8, 2.0), 0, "ahead, within range");
        assert_eq!(at(5.5, 2.0), 100, "ahead, past the reading distance");
        assert_eq!(at(2.0, 2.0), 100, "behind the camera");
    }

    #[test]
    fn wall_split_follows_what_the_camera_saw() {
        let g = room();
        let mut cov = Coverage::new();
        let (seen, unseen) = cov.wall_split(&g, 65);
        assert!(seen.is_empty() && unseen.len() > 300);
        cov.update(&g, (3.0, 2.0, 0.0), &CoverageParams { range: 4.0, ..Default::default() });
        let (seen, unseen2) = cov.wall_split(&g, 65);
        assert!(!seen.is_empty() && seen.iter().all(|p| p.0 > 5.5));
        assert_eq!(seen.len() + unseen2.len(), unseen.len());
    }

    #[test]
    fn camera_marks_wall_in_front_only() {
        let g = room();
        let mut cov = Coverage::new();
        let p = CoverageParams { range: 4.0, ..Default::default() };
        cov.update(&g, (3.0, 2.0, 0.0), &p);
        let east = g.world_to_cell(5.97, 2.0).unwrap();
        let west = g.world_to_cell(0.02, 2.0).unwrap();
        assert!(cov.is_seen(&g, east));
        assert!(!cov.is_seen(&g, west));
        assert!(cov.seen_count() > 20);
    }

    #[test]
    fn a_hit_without_a_wall_direction_is_not_seen() {
        // two lone cells of a box face in an early map: no direction, so no incidence check. Counting
        // them as head-on wrote a box face off before the camera could read its marker (sim, id 42)
        let meta = GridMeta { width: 120, height: 80, resolution: 0.05, origin_x: 0.0, origin_y: 0.0 };
        let mut g = Grid::filled(meta, 0);
        let (a, b) = (g.world_to_cell(4.0, 2.0).unwrap(), g.world_to_cell(4.0, 2.05).unwrap());
        g.set(a, 100);
        g.set(b, 100);
        let mut cov = Coverage::new();
        cov.update(&g, (3.0, 2.0, 0.0), &CoverageParams { range: 4.0, ..Default::default() });
        assert!(!cov.is_seen(&g, a) && !cov.is_seen(&g, b));
        // once the face is mapped, the same view counts
        for k in -6..=6 {
            g.set(g.world_to_cell(4.0, 2.0 + 0.05 * k as f64).unwrap(), 100);
        }
        cov.update(&g, (3.0, 2.0, 0.0), &CoverageParams { range: 4.0, ..Default::default() });
        assert!(cov.is_seen(&g, a));
    }

    #[test]
    fn oblique_or_far_walls_are_not_seen() {
        let g = room();
        let p = CoverageParams { range: 1.5, ..Default::default() };
        // 1 m from the south wall, looking straight at it: seen
        let mut cov = Coverage::new();
        cov.update(&g, (3.0, 1.1, -std::f64::consts::FRAC_PI_2), &p);
        assert!(cov.is_seen(&g, g.world_to_cell(3.0, 0.02).unwrap()));
        // grazing view: wall point about 1.4 m away (inside range) but 70 deg off its normal, so a
        // marker there looks like one ~4 m away: not seen. Without the incidence rule it would be.
        let mut cov = Coverage::new();
        let pose = (3.0, 0.5, -0.349);
        cov.update(&g, pose, &p);
        let grazing = g.world_to_cell(4.3, 0.02).unwrap();
        assert!(!cov.is_seen(&g, grazing));
        let mut cov = Coverage::new();
        cov.update(&g, pose, &CoverageParams { range: 1.5, max_incidence: std::f64::consts::FRAC_PI_2, ..Default::default() }
            .with_plain_range());
        assert!(cov.is_seen(&g, grazing), "plain range check sees it");
        // head-on but 2.5 m away: a marker would be too small, not seen
        let mut cov = Coverage::new();
        cov.update(&g, (3.0, 2.6, -std::f64::consts::FRAC_PI_2), &p);
        assert!(!cov.is_seen(&g, g.world_to_cell(3.0, 0.02).unwrap()));
    }

    #[test]
    fn views_point_at_unseen_walls() {
        let g = room();
        let mut cov = Coverage::new();
        let p = CoverageParams { range: 4.0, ..Default::default() };
        let views = cov.views(&g, (3.0, 2.0), &p);
        assert!(!views.is_empty());
        for v in &views {
            let c = g.world_to_cell(v.x, v.y).unwrap();
            assert_eq!(g.get(c), 0);
        }
        // look at every wall from the middle (range long enough for the corners at an angle): nothing left
        let far = CoverageParams { range: 10.0, ..p.clone() };
        for k in 0..16 {
            cov.update(&g, (3.0, 2.0, k as f64 * std::f64::consts::TAU / 16.0), &far);
        }
        assert!(cov.views(&g, (3.0, 2.0), &p).is_empty(), "{:?}", cov.views(&g, (3.0, 2.0), &p));
    }

    #[test]
    fn view_faces_target() {
        let g = room();
        let cov = Coverage::new();
        let p = CoverageParams { range: 4.0, min_cluster_cells: 3, ..Default::default() };
        let v = cov.views(&g, (5.0, 2.0), &p).into_iter().next().unwrap();
        // looking along yaw from the view hits a wall about `standoff` away
        let hit = g.raycast(v.x, v.y, v.yaw.cos(), v.yaw.sin(), 3.0, 65).expect("view looks at a wall");
        assert!((hit.range - p.standoff).abs() < 0.5, "{v:?} {hit:?}");
        let ahead = (v.x + 1.2 * v.yaw.cos(), v.y + 1.2 * v.yaw.sin());
        let near_wall = ahead.0 > 5.7 || ahead.0 < 0.3 || ahead.1 > 3.7 || ahead.1 < 0.3;
        assert!(near_wall, "{v:?}");
    }

    #[test]
    fn bin_sign_convention() {
        use std::f64::consts::{FRAC_PI_2, PI};
        // ray heading west (PI): the camera is east of the cell -> bin 0
        assert_eq!(bin_of(PI), 0);
        assert_eq!(bin_of(PI + 0.1), 0);
        // ray heading south (-PI/2): the camera is north of the cell -> bin 2
        assert_eq!(bin_of(-FRAC_PI_2), 2);
        // ray heading east (0): the camera is west -> bin 4
        assert_eq!(bin_of(0.0), 4);
        // just below the 0-degree edge (direction to camera about -6 deg) is bin 7, not bin 0
        assert_eq!(bin_of(PI - 0.1), 7);
    }

    /// solid 11 x 11 cell block at cells 10..=20 in a 6 x 4 m free grid; cell (20, 20) is its NE corner
    fn corner_block() -> Grid {
        let meta = GridMeta { width: 120, height: 80, resolution: 0.05, origin_x: 0.0, origin_y: 0.0 };
        let mut g = Grid::filled(meta, 0);
        for y in 10..=20 {
            for x in 10..=20 {
                g.set(Cell { x, y }, 100);
            }
        }
        g
    }

    /// camera 1 m away from the NE corner, heading at it: `yaw` is the heading, the camera sits behind
    /// the corner on the line through it. Headings of 202.5 deg (camera east-north-east of the corner, so
    /// the ray direction from the wall is bin 0) and -67.5 deg (camera north-north-west, bin 2) sit in
    /// the middle of their bins, so the small fan used here stays in one bin.
    fn pose_towards_corner(g: &Grid, yaw: f64) -> (f64, f64, f64) {
        let t = g.cell_to_world(Cell { x: 20, y: 20 });
        (t.0 - yaw.cos(), t.1 - yaw.sin(), yaw)
    }

    #[test]
    fn wall_cell_records_the_bins_it_is_seen_from() {
        let g = corner_block();
        let corner = Cell { x: 20, y: 20 };
        let p = CoverageParams { range: 4.0, half_fov: 0.02, cam_offset: 0.0, ..Default::default() };
        let east = pose_towards_corner(&g, 202.5f64.to_radians());
        let north = pose_towards_corner(&g, -67.5f64.to_radians());

        let mut cov = Coverage::new();
        assert_eq!(cov.bins_seen(&g, corner), 0);
        cov.update(&g, east, &p);
        assert!(cov.is_seen(&g, corner), "the corner is seen from the east");
        assert_eq!(cov.bins_seen(&g, corner), 1);
        assert!(!cov.unseen_bins(&g, corner).contains(&0), "bit 0 (east) is set");

        cov.update(&g, north, &p);
        assert_eq!(cov.bins_seen(&g, corner), 2);
        assert_eq!(cov.unseen_bins(&g, corner), vec![1, 3, 4, 5, 6, 7], "bits 0 and 2 set");

        // the same side again adds no bin
        cov.update(&g, north, &p);
        cov.update(&g, east, &p);
        assert_eq!(cov.bins_seen(&g, corner), 2);
        assert_eq!(cov.unseen_bins(&g, corner), vec![1, 3, 4, 5, 6, 7]);
    }

    #[test]
    fn cells_never_seen_have_no_bins() {
        let g = corner_block();
        let p = CoverageParams { range: 4.0, half_fov: 0.02, cam_offset: 0.0, ..Default::default() };
        let mut cov = Coverage::new();
        cov.update(&g, pose_towards_corner(&g, 202.5f64.to_radians()), &p);
        cov.update(&g, pose_towards_corner(&g, -67.5f64.to_radians()), &p);
        // the far (south-west) corner of the block faces away from both cameras
        let back = Cell { x: 10, y: 10 };
        assert!(!cov.is_seen(&g, back));
        assert_eq!(cov.bins_seen(&g, back), 0);
        assert_eq!(cov.unseen_bins(&g, back), (0..8).collect::<Vec<_>>());
        // free space never has bins
        let free = Cell { x: 60, y: 40 };
        assert_eq!(cov.bins_seen(&g, free), 0);
        assert_eq!(cov.unseen_bins(&g, free).len(), 8);
        // and nothing is recorded before any update
        assert_eq!(Coverage::new().bins_seen(&g, Cell { x: 20, y: 20 }), 0);
    }
}
