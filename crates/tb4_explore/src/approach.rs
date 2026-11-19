//! Approach on sight: a marker the mapper holds but has not confirmed gets a few close views from
//! different angles, so its views spread enough to triangulate and confirm.

use tb4_core::grid::Grid;
use tb4_core::{DistanceField, Pose2};

/// a marker seen but not yet confirmed, in the map frame
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MarkerHint {
    /// marker id when known (the sim knows it, the ROS PoseArray does not carry it)
    pub id: Option<i64>,
    /// fused position (m)
    pub x: f64,
    pub y: f64,
    /// direction the marker faces: yaw of its face normal, pointing to the free side (rad)
    pub facing: f64,
}

#[derive(Debug, Clone)]
pub struct ApproachParams {
    /// distance from the marker to look at it (m); the spec scores an approach within 2 m
    pub standoff: f64,
    /// never stand further than this (m)
    pub max_standoff: f64,
    /// view angles: the facing direction and +- this (rad)
    pub spread: f64,
    /// s from the first approach goal to giving the marker up
    pub timeout: f64,
    /// failed viewpoints (aborted, no progress, timed out) before giving the marker up
    pub max_failures: u32,
    /// s a given-up marker is ignored (a decoy must not trap the robot)
    pub blacklist_duration: f64,
    /// hints this close are the same marker (m)
    pub match_radius: f64,
    /// viewpoint clearance from obstacles and unknown (m)
    pub min_clearance: f64,
    pub occupied_threshold: i8,
}

impl Default for ApproachParams {
    fn default() -> Self {
        Self {
            standoff: 1.3,
            max_standoff: 1.9,
            spread: 30f64.to_radians(),
            timeout: 90.0,
            max_failures: 2,
            blacklist_duration: 300.0,
            match_radius: 0.5,
            min_clearance: 0.3,
            occupied_threshold: 65,
        }
    }
}

/// what the explorer remembers per marker
#[derive(Debug, Clone)]
pub(crate) struct Approach {
    /// unique per marker, so a goal finds its marker after the estimate moved
    pub serial: u64,
    pub pos: (f64, f64),
    pub facing: f64,
    pub id: Option<i64>,
    /// in the latest hint list (not confirmed)
    pub present: bool,
    /// time of the first approach goal
    pub started: Option<f64>,
    /// viewpoints done or failed, per offset (0, +spread, -spread)
    pub tried: [bool; 3],
    pub failures: u32,
    /// given up until this time
    pub ignore_until: f64,
}

impl Approach {
    pub fn label(&self) -> String {
        match self.id {
            Some(id) => format!("marker {id} at ({:.2}, {:.2})", self.pos.0, self.pos.1),
            None => format!("marker at ({:.2}, {:.2})", self.pos.0, self.pos.1),
        }
    }
}

/// a place to look at a marker from
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Viewpoint {
    pub x: f64,
    pub y: f64,
    /// heading facing the marker
    pub yaw: f64,
    /// which offset of the facing direction (0, +spread, -spread)
    pub slot: usize,
}

pub(crate) const OFFSETS: [f64; 3] = [0.0, 1.0, -1.0];

/// update the remembered markers from this tick's hints. A marker that left the hints was confirmed
/// by the mapper (or its estimate jumped): it is dropped unless it is being ignored.
pub(crate) fn sync(list: &mut Vec<Approach>, hints: &[MarkerHint], p: &ApproachParams, now: f64, serial: &mut u64) {
    for a in list.iter_mut() {
        a.present = false;
    }
    for h in hints {
        // same id: same marker however far its estimate moved; without ids: the nearest one close by
        let near = list
            .iter_mut()
            .filter(|a| !a.present && (h.id.is_none() || a.id.is_none() || a.id == h.id))
            .map(|a| {
                let same_id = h.id.is_some() && a.id == h.id;
                (if same_id { 0.0 } else { (a.pos.0 - h.x).hypot(a.pos.1 - h.y) }, a)
            })
            .filter(|(d, _)| *d <= p.match_radius)
            .min_by(|a, b| a.0.total_cmp(&b.0))
            .map(|(_, a)| a);
        match near {
            Some(a) => {
                a.pos = (h.x, h.y);
                a.facing = h.facing;
                a.id = a.id.or(h.id);
                a.present = true;
            }
            None => list.push(Approach {
                serial: {
                    *serial += 1;
                    *serial
                },
                pos: (h.x, h.y),
                facing: h.facing,
                id: h.id,
                present: true,
                started: None,
                tried: [false; 3],
                failures: 0,
                ignore_until: f64::NEG_INFINITY,
            }),
        }
    }
    list.retain(|a| a.present || now < a.ignore_until);
}

/// free, clear viewpoints of `a` for the slots not tried yet, at the standoff or a little nearer / further
pub(crate) fn viewpoints(a: &Approach, g: &Grid, clearance: &DistanceField, p: &ApproachParams) -> Vec<Viewpoint> {
    let mut out = Vec::new();
    let res = g.meta.resolution;
    for (slot, k) in OFFSETS.iter().enumerate() {
        if a.tried[slot] {
            continue;
        }
        let dir = a.facing + k * p.spread;
        let rings = [p.standoff, 0.8 * p.standoff, (1.25 * p.standoff).min(p.max_standoff)];
        for r in rings.into_iter().filter(|r| *r <= p.max_standoff) {
            let (x, y) = (a.pos.0 + r * dir.cos(), a.pos.1 + r * dir.sin());
            let Some(c) = g.world_to_cell(x, y) else { continue };
            if !(0..p.occupied_threshold).contains(&g.get(c)) || (clearance.get(c) as f64) < p.min_clearance {
                continue;
            }
            // line of sight: nothing until just short of the marker (it may sit on a wall)
            let (dx, dy) = (a.pos.0 - x, a.pos.1 - y);
            let d = dx.hypot(dy);
            if g.raycast(x, y, dx, dy, d, p.occupied_threshold).is_some_and(|h| h.range < d - 3.0 * res) {
                continue;
            }
            out.push(Viewpoint { x, y, yaw: dy.atan2(dx), slot });
            break;
        }
    }
    out
}

/// nearest viewpoint to the robot, with travel plus turn cost
pub(crate) fn nearest(vps: &[Viewpoint], robot: Pose2, turn_weight: f64) -> Option<Viewpoint> {
    let cost = |v: &Viewpoint| {
        let ang = (v.y - robot.y).atan2(v.x - robot.x) - robot.yaw;
        robot.distance_to(v.x, v.y) + turn_weight * ang.sin().atan2(ang.cos()).abs()
    };
    vps.iter().copied().min_by(|a, b| cost(a).total_cmp(&cost(b)))
}
