//! Live viewer for the `tb4_sim` closed loop: the simulator runs in-process and is drawn top-down
//! with aurea. `scene` is the pure part (transform and draw list), `render` paints it, `app` holds
//! the sim, its options and the stepping against the wall clock.

pub mod app;
pub mod render;
pub mod scene;
