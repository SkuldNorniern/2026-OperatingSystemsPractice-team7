//! Obstacles the lidar cannot see (glass, low objects), learned from the Create 3 bumper and,
//! later, its IR proximity sensors. Kept apart from the SLAM map, so a scan through glass cannot
//! erase them, and stamped into the grid the planner and MPPI use.
//!
//! Times are plain seconds from the caller (ROS time), so replays and sims behave like the robot.

use tb4_core::{Grid, Pose2};

/// Create 3 bumper zones, as `/hazard_detection` frame ids. Angle of the zone centre from the
/// robot's forward axis (rad), from the robot's TF: bump_front_center 0, front_left/right +-30 deg,
/// left/right +-60 deg, all on a 0.175 m radius.
pub fn bump_zone_angle(frame_id: &str) -> Option<f64> {
    let deg = match frame_id.trim_start_matches('/') {
        "bump_front_center" => 0.0,
        "bump_front_left" => 30.0,
        "bump_front_right" => -30.0,
        "bump_left" => 60.0,
        "bump_right" => -60.0,
        _ => return None,
    };
    Some(f64::to_radians(deg))
}

/// Create 3 IR proximity sensors (`/ir_intensity` frame ids): position (m) and facing (rad) in
/// base_link, from the robot's TF (bag 20261118_184838).
pub const IR_SENSORS: [(&str, f64, f64, f64); 7] = [
    ("ir_intensity_side_left", 0.095, 0.147, 1.0001),
    ("ir_intensity_left", 0.141, 0.103, 0.6283),
    ("ir_intensity_front_left", 0.165, 0.057, 0.3334),
    ("ir_intensity_front_center_left", 0.175, 0.008, 0.0454),
    ("ir_intensity_front_center_right", 0.170, -0.042, -0.2409),
    ("ir_intensity_front_right", 0.148, -0.094, -0.5637),
    ("ir_intensity_right", 0.081, -0.155, -1.0926),
];

/// IR reading -> obstacle point. The Create 3 reports raw intensity (higher = closer), not range,
/// so both numbers below are placeholders until recorded readings at known distances set them.
#[derive(Debug, Clone)]
pub struct IrParams {
    /// raw intensity at or above which a sensor counts as "something close"
    pub threshold: i32,
    /// distance in front of the sensor where that something is placed (m)
    pub range: f64,
}

/// Obstacle points (map frame) for the IR readings `(frame_id, value)` at robot `pose`. Unknown
/// frame ids and readings below the threshold give nothing.
pub fn ir_points(pose: Pose2, readings: &[(&str, i32)], p: &IrParams) -> Vec<(f64, f64)> {
    let (s, c) = pose.yaw.sin_cos();
    readings
        .iter()
        .filter(|(_, v)| *v >= p.threshold)
        .filter_map(|(id, _)| IR_SENSORS.iter().find(|q| q.0 == id.trim_start_matches('/')))
        .map(|&(_, sx, sy, sa)| {
            // sensor position plus `range` along its facing, in base_link, then into the map
            let (bx, by) = (sx + p.range * sa.cos(), sy + p.range * sa.sin());
            (pose.x + c * bx - s * by, pose.y + s * bx + c * by)
        })
        .collect()
}

#[derive(Debug, Clone)]
pub struct VirtualParams {
    /// distance from the robot centre where a bump contact is placed (m): bumper radius + a little
    pub bump_radius: f64,
    /// half width of the arc marked for one bump zone (rad)
    pub bump_half_arc: f64,
    /// spacing of points along that arc (m)
    pub point_spacing: f64,
    /// a mark is dropped after this long (s); the scene may change (a person, a moved chair)
    pub ttl: f64,
    /// a new point closer than this to an existing one only refreshes it (m)
    pub merge_dist: f64,
    /// hard cap on stored points; the oldest go first
    pub max_points: usize,
}

impl Default for VirtualParams {
    fn default() -> Self {
        Self { bump_radius: 0.20, bump_half_arc: 0.30, point_spacing: 0.04, ttl: 180.0, merge_dist: 0.03, max_points: 2000 }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct VirtualPoint {
    pub x: f64,
    pub y: f64,
    /// last time this point was confirmed (s)
    pub seen: f64,
}

#[derive(Debug, Clone, Default)]
pub struct VirtualObstacles {
    pub params: VirtualParams,
    points: Vec<VirtualPoint>,
}

impl VirtualObstacles {
    pub fn new(params: VirtualParams) -> Self {
        Self { params, points: Vec::new() }
    }

    pub fn points(&self) -> &[VirtualPoint] {
        &self.points
    }

    /// Bump in zone `angle` (see [`bump_zone_angle`]) while the robot was at `pose` (map frame):
    /// marks a short arc at `bump_radius` around that direction. Returns the points added or refreshed.
    pub fn add_bump(&mut self, pose: Pose2, angle: f64, now: f64) -> usize {
        let p = &self.params;
        let arc = 2.0 * p.bump_half_arc * p.bump_radius;
        let n = ((arc / p.point_spacing).ceil() as usize).max(1);
        let pts: Vec<(f64, f64)> = (0..=n)
            .map(|k| {
                let a = pose.yaw + angle - p.bump_half_arc + 2.0 * p.bump_half_arc * k as f64 / n as f64;
                (pose.x + p.bump_radius * a.cos(), pose.y + p.bump_radius * a.sin())
            })
            .collect();
        pts.into_iter().map(|(x, y)| self.add_point(x, y, now)).count()
    }

    /// One obstacle point in the map frame (also the entry for IR hits once their range is known).
    pub fn add_point(&mut self, x: f64, y: f64, now: f64) {
        let m2 = self.params.merge_dist * self.params.merge_dist;
        if let Some(q) = self.points.iter_mut().find(|q| (q.x - x).powi(2) + (q.y - y).powi(2) <= m2) {
            q.seen = now;
            return;
        }
        self.points.push(VirtualPoint { x, y, seen: now });
        if self.points.len() > self.params.max_points {
            // oldest first
            let oldest = (0..self.points.len()).min_by(|&a, &b| self.points[a].seen.total_cmp(&self.points[b].seen)).unwrap();
            self.points.swap_remove(oldest);
        }
    }

    /// Drop points older than `ttl`.
    pub fn expire(&mut self, now: f64) {
        let ttl = self.params.ttl;
        self.points.retain(|q| now - q.seen <= ttl);
    }

    /// Write the points into `grid` as occupied (`value`), e.g. a copy of the SLAM map before the
    /// cspace / distance field is built. Points outside the grid are skipped. Returns cells set.
    pub fn stamp(&self, grid: &mut Grid, value: i8) -> usize {
        let mut n = 0;
        for q in &self.points {
            if let Some(c) = grid.world_to_cell(q.x, q.y) {
                grid.set(c, value);
                n += 1;
            }
        }
        n
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use tb4_core::grid::GridMeta;

    #[test]
    fn zones_match_the_robot_tf() {
        assert_eq!(bump_zone_angle("bump_front_center"), Some(0.0));
        assert!((bump_zone_angle("/bump_left").unwrap() - 60f64.to_radians()).abs() < 1e-12);
        assert!((bump_zone_angle("bump_front_right").unwrap() + 30f64.to_radians()).abs() < 1e-12);
        assert_eq!(bump_zone_angle("cliff_front_left"), None);
    }

    #[test]
    fn bump_marks_an_arc_in_front_of_the_hit_zone() {
        let mut v = VirtualObstacles::default();
        // robot at (1, 2) facing +y, front-left bump: contact around 90 + 30 = 120 deg
        v.add_bump(Pose2::new(1.0, 2.0, std::f64::consts::FRAC_PI_2), bump_zone_angle("bump_front_left").unwrap(), 10.0);
        assert!(v.points().len() >= 3);
        for q in v.points() {
            let (dx, dy) = (q.x - 1.0, q.y - 2.0);
            assert!((dx.hypot(dy) - 0.20).abs() < 1e-9, "on the bump radius");
            let a = dy.atan2(dx).to_degrees();
            assert!((a - 120.0).abs() <= 0.30f64.to_degrees() + 1e-6, "within the zone arc: {a}");
        }
    }

    #[test]
    fn repeated_bumps_refresh_instead_of_piling_up_and_old_marks_expire() {
        let mut v = VirtualObstacles::new(VirtualParams { ttl: 60.0, ..VirtualParams::default() });
        let pose = Pose2::new(0.0, 0.0, 0.0);
        v.add_bump(pose, 0.0, 0.0);
        let n = v.points().len();
        v.add_bump(pose, 0.0, 30.0);
        assert_eq!(v.points().len(), n, "same contact refreshes");
        assert!(v.points().iter().all(|q| q.seen == 30.0));
        v.expire(80.0);
        assert_eq!(v.points().len(), n, "refreshed at 30 s, ttl 60 s");
        v.expire(91.0);
        assert!(v.points().is_empty());
    }

    #[test]
    fn ir_points_in_front_of_the_firing_sensor() {
        let p = IrParams { threshold: 100, range: 0.10 };
        // robot at (2, 0) facing -x (yaw pi): front-centre sensors point toward x < 2
        let pose = Pose2::new(2.0, 0.0, std::f64::consts::PI);
        let pts = ir_points(pose, &[("ir_intensity_front_center_left", 400), ("ir_intensity_right", 20), ("bogus", 999)], &p);
        assert_eq!(pts.len(), 1, "below threshold and unknown ids give nothing");
        let (x, y) = pts[0];
        // base_link (0.175 + 0.1*cos(2.6 deg), 0.008 + 0.1*sin(2.6 deg)) rotated by 180 deg
        assert!((x - (2.0 - 0.2749)).abs() < 1e-3 && (y - (-0.0125)).abs() < 1e-3, "({x}, {y})");
        // and they go into the layer like bump points
        let mut v = VirtualObstacles::default();
        for (x, y) in pts {
            v.add_point(x, y, 1.0);
        }
        assert_eq!(v.points().len(), 1);
    }

    #[test]
    fn cap_drops_the_oldest_point() {
        let mut v = VirtualObstacles::new(VirtualParams { max_points: 3, ..VirtualParams::default() });
        for k in 0..4 {
            v.add_point(k as f64, 0.0, k as f64);
        }
        assert_eq!(v.points().len(), 3);
        assert!(v.points().iter().all(|q| q.x != 0.0), "point from t=0 was dropped");
    }

    #[test]
    fn stamp_marks_cells_occupied() {
        let meta = GridMeta { width: 40, height: 40, resolution: 0.05, origin_x: -1.0, origin_y: -1.0 };
        let mut g = Grid::filled(meta, 0);
        let mut v = VirtualObstacles::default();
        v.add_point(0.30, 0.0, 0.0);
        v.add_point(5.0, 5.0, 0.0); // outside the grid
        assert_eq!(v.stamp(&mut g, 100), 1);
        assert_eq!(g.get(g.world_to_cell(0.30, 0.0).unwrap()), 100);
    }
}
