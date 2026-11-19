//! Camera detections of markers and decoys as the marker_mapper node sees them: `tb4_markers::Observation`
//! from the robot's SLAM pose (not the truth), with bearing and PnP range noise. The tracker settings are
//! the `marker_mapper` section of team7.yaml.

use tb4_control::Rng;
use tb4_core::Pose2;
use tb4_markers::{LocateParams, Observation, TrackParams};

use crate::sensors::{CameraParams, MarkerView, view_marker_px};
use crate::world::{Marker, World};

#[derive(Debug, Clone)]
pub struct VisionParams {
    /// feed detections to a real tb4_markers::Tracker
    pub enabled: bool,
    /// 1 sigma of the horizontal and vertical bearing (rad)
    pub bearing_sigma: f64,
    /// 1 sigma of the PnP range, fraction of the range
    pub pnp_sigma: f64,
    /// marker side the detector assumes for PnP (aruco.yaml marker_size, m): a real marker of another
    /// size gets its range scaled by assumed / real
    pub assumed_size: f64,
    /// detection probability at or above `CameraParams::min_side_px`
    pub p_detect: f64,
    /// detection probability at `CameraParams::detect_min_side_px`, rising linearly to `p_detect`
    pub p_detect_small: f64,
}

impl Default for VisionParams {
    fn default() -> Self {
        Self { enabled: true, bearing_sigma: 1f64.to_radians(), pnp_sigma: 0.06, assumed_size: 0.10, p_detect: 0.95, p_detect_small: 0.2 }
    }
}

/// marker_mapper locate parameters from team7.yaml
pub fn team7_locate_params() -> LocateParams {
    LocateParams {
        max_range: 4.0,
        occupied_threshold: 65,
        range_ratio_gate: 3.0,
        allow_pnp_fallback: true,
        max_abs_bearing_z: 0.8,
        wall_fit_radius: 0.2,
        min_range: 0.3,
    }
}

/// marker_mapper tracker parameters from team7.yaml
pub fn team7_track_params() -> TrackParams {
    TrackParams {
        min_views: 5,
        min_wall_views: 3,
        min_wall_fraction: 0.25,
        min_triangulation_spread: 0.2,
        max_triangulation_gap: 0.3,
        strict_ids: vec![0, 341, 682, 1023],
        require_scale_all: true,
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
    }
}

/// approach poses sit this far in front of a tentative marker (team7.yaml approach_standoff)
pub const APPROACH_STANDOFF: f64 = 1.3;

/// Detection of `mk` by the camera on the robot at `truth` (world frame), turned into the observation
/// the mapper would build from the map -> camera transform of the pose estimate `est` (map frame).
/// None if not in view, too small, or missed this frame.
pub fn detect(
    world: &World,
    truth: Pose2,
    est: Pose2,
    mk: &Marker,
    cam: &CameraParams,
    vp: &VisionParams,
    rng: &mut Rng,
) -> Option<(MarkerView, Observation)> {
    let v = view_marker_px(world, truth, mk, cam, cam.detect_min_side_px).ok()?;
    let p = if v.side_px >= cam.min_side_px {
        vp.p_detect
    } else {
        let f = ((v.side_px - cam.detect_min_side_px) / (cam.min_side_px - cam.detect_min_side_px)).clamp(0.0, 1.0);
        vp.p_detect_small + f * (vp.p_detect - vp.p_detect_small)
    };
    if rng.uniform() >= p {
        return None;
    }
    // horizontal range from the camera, height difference to the marker centre
    let dz = mk.z - cam.height;
    let elevation = dz.atan2(v.range) + vp.bearing_sigma * rng.normal();
    let azimuth = est.yaw + v.bearing + vp.bearing_sigma * rng.normal();
    let (ce, se) = (elevation.cos(), elevation.sin());
    let bearing = (ce * azimuth.cos(), ce * azimuth.sin(), se);
    let range3 = v.range.hypot(dz);
    let pnp_range = range3 * vp.assumed_size / mk.size * (1.0 + vp.pnp_sigma * rng.normal());
    let camera = (est.x + cam.offset_x * est.yaw.cos(), est.y + cam.offset_x * est.yaw.sin(), cam.height);
    Some((v, Observation { id: mk.id as i64, camera, bearing, pnp_range: pnp_range.max(1e-3) }))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn observation_points_at_the_marker_from_the_estimate() {
        let mut w = World::new("t", [0.0, 0.0, 6.0, 4.0], Pose2::new(1.0, 2.0, 0.0));
        w.rect(0.0, 0.0, 6.0, 4.0);
        w.marker(7, (6.0, 2.0), 0.25, (-1.0, 0.0), 0.2);
        let cam = CameraParams::default();
        let vp = VisionParams { bearing_sigma: 0.0, pnp_sigma: 0.0, p_detect: 1.0, ..Default::default() };
        let truth = Pose2::new(4.0, 2.0, 0.0);
        // estimate 0.5 m off in y and turned by 0.1 rad: the observation carries the estimate's error
        let est = Pose2::new(4.0, 2.5, 0.1);
        let (v, o) = detect(&w, truth, est, &w.markers[0], &cam, &vp, &mut Rng::new(1)).unwrap();
        assert!((v.range - 1.9).abs() < 1e-9);
        assert!((o.camera.0 - (4.0 + 0.1 * 0.1f64.cos())).abs() < 1e-9 && (o.camera.1 - (2.5 + 0.1 * 0.1f64.sin())).abs() < 1e-9);
        assert!((o.bearing.1.atan2(o.bearing.0) - 0.1).abs() < 1e-9 && o.bearing.2.abs() < 1e-9);
        // 20 cm marker, detector assumes 10 cm: PnP says half the distance
        assert!((o.pnp_range - 0.95).abs() < 1e-9);
        // 0.2 m at 3.9 m is 10.3 px: found sometimes, not always
        let far = Pose2::new(2.0, 2.0, 0.0);
        let vp = VisionParams { bearing_sigma: 0.0, pnp_sigma: 0.0, ..Default::default() };
        let mut rng = Rng::new(3);
        let n = (0..1000).filter(|_| detect(&w, far, far, &w.markers[0], &cam, &vp, &mut rng).is_some()).count();
        assert!((300..800).contains(&n), "{n}");
    }

    #[test]
    fn studio_decoys_are_detectable_from_in_front() {
        let w = crate::worlds::studio();
        let cam = CameraParams::default();
        let vp = VisionParams { p_detect: 1.0, ..Default::default() };
        for d in &w.decoys {
            let at = (d.pos.0 + 0.7 * d.normal.0, d.pos.1 + 0.7 * d.normal.1);
            let robot = Pose2::new(at.0, at.1, (-d.normal.1).atan2(-d.normal.0));
            let r = view_marker_px(&w, robot, d, &cam, cam.detect_min_side_px);
            assert!(r.is_ok(), "decoy {}: {r:?}", d.id);
            assert!(detect(&w, robot, robot, d, &cam, &vp, &mut Rng::new(1)).is_some());
        }
    }
}
