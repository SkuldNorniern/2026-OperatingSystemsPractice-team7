//! Headless, deterministic, faster-than-real-time closed loop for the team7 TurtleBot 4 stack.
//!
//! The robot, lidar, camera and world are simulated in 2D; the decision code is the real one:
//! `tb4_slam` (scan matching map), the navigator logic (`nav`, a copy of the node's tick on
//! `tb4_planning` + `tb4_control`), the velocity chain, `tb4_explore::ExplorerPolicy` and the marker
//! fusion `tb4_markers::Tracker` fed with simulated detections (`vision`).
//! Everything runs on a simulated clock. `Sim::state` gives a snapshot for a later viewer.

pub mod chain;
pub mod metrics;
pub mod nav;
pub mod robot;
pub mod sensors;
pub mod sim;
pub mod vision;
pub mod world;
pub mod worlds;

pub use metrics::Metrics;
pub use sim::{SET_NAMES, Sim, SimConfig, SimState};
pub use world::World;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn small_world_closed_loop_maps_and_moves_without_collision() {
        let mut cfg = SimConfig::team7(worlds::small());
        cfg.nav.mppi.threads = 1;
        let mut sim = Sim::new(cfg);
        sim.run_until(60.0);
        let m = sim.metrics();
        assert!(m.distance > 1.0, "{}", m.table());
        assert!(m.final_area > 4.0, "{}", m.table());
        assert_eq!(m.collisions, 0, "{}", m.table());
        assert!(m.explorer.sent > 0);
        assert!(m.max_pose_error < 0.3, "{}", m.table());
    }

    #[test]
    fn same_seed_same_run() {
        let run = || {
            let mut cfg = SimConfig::team7(worlds::small());
            cfg.perfect_pose = true;
            let mut sim = Sim::new(cfg);
            sim.run_until(25.0);
            let s = sim.state();
            (s.truth, sim.metrics().final_area)
        };
        assert_eq!(run(), run());
    }
}
