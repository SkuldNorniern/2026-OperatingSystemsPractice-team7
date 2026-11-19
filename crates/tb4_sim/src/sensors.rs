//! RPLIDAR A1 style 2D lidar and an OAK-D camera model for marker readability (geometry only, no images).

use tb4_control::Rng;
use tb4_core::Pose2;

use crate::world::{Marker, World};

#[derive(Debug, Clone)]
pub struct LidarParams {
    pub rate_hz: f64,
    pub beams: usize,
    pub min_range: f64,
    pub max_range: f64,
    /// range noise sigma: constant part (m) plus a fraction of the range
    pub noise_sigma: f64,
    pub noise_frac: f64,
    /// probability a beam returns nothing
    pub dropout: f64,
    /// sensor position on the base (m, forward); tf rplidar_link x = -0.04
    pub offset_x: f64,
}

impl Default for LidarParams {
    fn default() -> Self {
        Self {
            rate_hz: 5.5,
            beams: 360,
            min_range: 0.15,
            max_range: 8.0,
            noise_sigma: 0.005,
            noise_frac: 0.005,
            dropout: 0.01,
            offset_x: -0.04,
        }
    }
}

/// one scan: beam i points at base-frame angle `i * 2 pi / beams` from the sensor; inf = no return
#[derive(Debug, Clone, Default)]
pub struct Scan {
    pub ranges: Vec<f64>,
}

impl Scan {
    pub fn angle(&self, i: usize) -> f64 {
        i as f64 * std::f64::consts::TAU / self.ranges.len() as f64
    }

    /// returns as points in the base frame (sensor offset applied), only finite ranges >= min_range
    pub fn points_base(&self, p: &LidarParams) -> Vec<(f64, f64)> {
        self.ranges
            .iter()
            .enumerate()
            .filter(|(_, r)| r.is_finite() && **r >= p.min_range)
            .map(|(i, r)| {
                let a = self.angle(i);
                (p.offset_x + r * a.cos(), r * a.sin())
            })
            .collect()
    }
}

/// glass does not reflect; posts and walls do
pub fn scan(world: &World, pose: Pose2, p: &LidarParams, rng: &mut Rng) -> Scan {
    let (s, c) = pose.yaw.sin_cos();
    let o = (pose.x + c * p.offset_x, pose.y + s * p.offset_x);
    let ranges = (0..p.beams)
        .map(|i| {
            let a = pose.yaw + i as f64 * std::f64::consts::TAU / p.beams as f64;
            let r = world.raycast(o, (a.cos(), a.sin()), p.max_range, false);
            let drop = rng.uniform() < p.dropout;
            let noise = rng.normal() * (p.noise_sigma + p.noise_frac * r.unwrap_or(0.0));
            match r {
                Some(r) if !drop => {
                    let r = r + noise;
                    if r < p.min_range || r > p.max_range { f64::INFINITY } else { r }
                }
                _ => f64::INFINITY,
            }
        })
        .collect();
    Scan { ranges }
}

#[derive(Debug, Clone)]
pub struct CameraParams {
    pub rate_hz: f64,
    /// focal length (px) of the 250 x 250 preview
    pub fx: f64,
    pub hfov: f64,
    pub vfov: f64,
    /// camera ahead of the base centre (m)
    pub offset_x: f64,
    /// lens height above the floor (m)
    pub height: f64,
    /// a marker seen further off its normal than this does not decode (rad)
    pub max_incidence: f64,
    /// apparent side (px) a marker needs to be read
    pub min_side_px: f64,
}

impl Default for CameraParams {
    fn default() -> Self {
        Self {
            rate_hz: 15.0,
            fx: 201.0,
            hfov: 63.8f64.to_radians(),
            vfov: 63.8f64.to_radians(),
            offset_x: 0.1,
            height: 0.25,
            max_incidence: 75f64.to_radians(),
            min_side_px: 12.0,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MarkerView {
    pub range: f64,
    /// angle between the marker normal and the ray back to the camera (rad)
    pub incidence: f64,
    /// horizontal angle from the optical axis (rad, left positive)
    pub bearing: f64,
    /// apparent side length fx * size * cos(incidence) / range (px)
    pub side_px: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum NotReadable {
    OutOfFov,
    Occluded,
    BackFacing,
    TooOblique,
    TooSmall,
}

pub fn camera_pose(robot: Pose2, p: &CameraParams) -> (f64, f64) {
    (robot.x + p.offset_x * robot.yaw.cos(), robot.y + p.offset_x * robot.yaw.sin())
}

/// whether the marker can be read from the camera on `robot`
pub fn view_marker(world: &World, robot: Pose2, m: &Marker, p: &CameraParams) -> Result<MarkerView, NotReadable> {
    let cam = camera_pose(robot, p);
    let (dx, dy) = (m.pos.0 - cam.0, m.pos.1 - cam.1);
    let range = dx.hypot(dy);
    if range < 1e-6 {
        return Err(NotReadable::OutOfFov);
    }
    let bearing = wrap(dy.atan2(dx) - robot.yaw);
    let elevation = (m.z - p.height).atan2(range);
    if bearing.abs() > p.hfov / 2.0 || elevation.abs() > p.vfov / 2.0 {
        return Err(NotReadable::OutOfFov);
    }
    // the ray back from the marker to the camera against the face normal
    let cos_inc = -(dx * m.normal.0 + dy * m.normal.1) / range;
    if cos_inc <= 0.0 {
        return Err(NotReadable::BackFacing);
    }
    let incidence = cos_inc.clamp(-1.0, 1.0).acos();
    // stop just short of the face the marker is on
    if world.raycast(cam, (dx / range, dy / range), range - 0.02, false).is_some() {
        return Err(NotReadable::Occluded);
    }
    if incidence > p.max_incidence {
        return Err(NotReadable::TooOblique);
    }
    let side_px = p.fx * m.size * cos_inc / range;
    if side_px < p.min_side_px {
        return Err(NotReadable::TooSmall);
    }
    Ok(MarkerView { range, incidence, bearing, side_px })
}

fn wrap(a: f64) -> f64 {
    a.sin().atan2(a.cos())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn room() -> World {
        let mut w = World::new("t", [0.0, 0.0, 6.0, 4.0], Pose2::new(1.0, 2.0, 0.0));
        w.rect(0.0, 0.0, 6.0, 4.0);
        // marker on the east wall at x = 6, facing west
        w.marker(7, (6.0, 2.0), 0.3, (-1.0, 0.0), 0.1);
        w
    }

    #[test]
    fn lidar_ranges_to_walls() {
        let w = room();
        let p = LidarParams { noise_sigma: 0.0, noise_frac: 0.0, dropout: 0.0, ..LidarParams::default() };
        let s = scan(&w, Pose2::new(1.0, 2.0, 0.0), &p, &mut Rng::new(1));
        assert_eq!(s.ranges.len(), 360);
        // sensor sits at x = 0.96: 5.04 m to the east wall, 0.96 m to the west wall
        assert!((s.ranges[0] - 5.04).abs() < 1e-9);
        assert!((s.ranges[180] - 0.96).abs() < 1e-9);
        assert!((s.ranges[90] - 2.0).abs() < 1e-9);
        let pts = s.points_base(&p);
        assert!((pts[0].0 - 5.0).abs() < 1e-9 && pts[0].1.abs() < 1e-9);
    }

    #[test]
    fn lidar_out_of_range_and_glass_give_inf() {
        let mut w = room();
        w.glass((3.0, 0.0), (3.0, 4.0));
        let p = LidarParams { max_range: 4.0, noise_sigma: 0.0, noise_frac: 0.0, dropout: 0.0, ..LidarParams::default() };
        let s = scan(&w, Pose2::new(1.0, 2.0, 0.0), &p, &mut Rng::new(1));
        assert!(s.ranges[0].is_infinite());
        assert!(s.points_base(&p).len() < 360);
    }

    #[test]
    fn marker_readable_head_on_within_range_only() {
        let w = room();
        let p = CameraParams::default();
        // 10 cm marker: 201 * 0.1 / 12 = 1.675 m head-on limit
        let near = view_marker(&w, Pose2::new(4.5, 2.0, 0.0), &w.markers[0], &p).unwrap();
        assert!((near.side_px - 201.0 * 0.1 / 1.4).abs() < 1e-6);
        assert_eq!(view_marker(&w, Pose2::new(3.5, 2.0, 0.0), &w.markers[0], &p), Err(NotReadable::TooSmall));
        assert_eq!(view_marker(&w, Pose2::new(4.5, 2.0, std::f64::consts::PI), &w.markers[0], &p), Err(NotReadable::OutOfFov));
    }

    #[test]
    fn marker_oblique_occluded_or_too_low() {
        let mut w = room();
        let p = CameraParams::default();
        // 80 deg off the normal
        let a = 80f64.to_radians();
        let at = (6.0 - 1.0 * a.cos(), 2.0 + 1.0 * a.sin());
        let yaw = (2.0 - at.1).atan2(6.0 - at.0);
        let robot = Pose2::new(at.0 - 0.1 * yaw.cos(), at.1 - 0.1 * yaw.sin(), yaw);
        assert_eq!(view_marker(&w, robot, &w.markers[0], &p), Err(NotReadable::TooOblique));
        // a post in between
        w.post(5.2, 2.0, 0.05);
        assert_eq!(view_marker(&w, Pose2::new(4.5, 2.0, 0.0), &w.markers[0], &p), Err(NotReadable::Occluded));
        // a marker near the floor right in front of the camera is below the image
        let mut w = room();
        w.markers[0].z = 0.05;
        assert_eq!(view_marker(&w, Pose2::new(5.75, 2.0, 0.0), &w.markers[0], &p), Err(NotReadable::OutOfFov));
    }
}
