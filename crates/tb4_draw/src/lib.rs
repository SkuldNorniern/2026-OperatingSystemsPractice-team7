//! Top-down drawing shared by the sim viewer (`tb4_viewer`) and the ROS viewer (`team7_viz`).
//!
//! Everything here takes plain data (grid meta and cells, screen points, colours) and turns it into
//! aurea drawing calls, so it works the same on a window canvas and on an offscreen `CpuRasterizer`.
//!
//! - [`palette`]: every colour the viewers use, in one place
//! - [`view`]: world -> screen fit, plus the zoom / pan / follow camera
//! - [`map`]: occupancy grid -> RGBA image and the affine that puts it on screen
//! - [`prim`]: drawing primitives, shape builders (robot, goal, cross, dots) and their painter

pub mod map;
pub mod palette;
pub mod prim;
pub mod view;

pub use map::{MapLayer, cell_color, cost_color, costmap_image, tint_image, draw_map, map_image, map_layer, map_rgba, occupied_image};
pub use prim::{Prim, paint_prims};
pub use view::{Camera, Viewport};

/// 8-bit RGBA
pub type Rgba = [u8; 4];
