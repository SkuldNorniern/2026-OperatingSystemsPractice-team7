//! The one place where viewer colours live. Both viewers draw on a dark background; a thing keeps
//! the same colour in the sim viewer, the ROS viewer and their legends.
//!
//! | colour | used for |
//! |---|---|
//! | `BACKGROUND`, `PANEL`, `TEXT`, `TEXT_DIM` | window, side panel, panel text |
//! | `UNKNOWN`, `FREE`, `OCCUPIED` | occupancy map cells (-1, 0, >= 65; in between blends) |
//! | `ROBOT`, `ROBOT_EDGE` | robot footprint and heading line |
//! | `PATH` green | global plan (`/team7/path`) |
//! | `TRAJECTORY` magenta | MPPI optimal trajectory (`/team7/mppi/trajectory`) |
//! | `FRONTIER` blue, `VIEW` orange | frontier goals and camera views (`/team7/frontiers`) |
//! | `GOAL` yellow | the active goal (`/team7/goal`), drawn as a ring with a cross |
//! | `TARGET_TENTATIVE` amber, `TARGET_CONFIRMED` green | ArUco targets (`/team7/targets`) |
//! | `SCAN` red | latest `/scan` points |
//! | `WALL`, `GLASS`, `ESTIMATE`, `TRAIL`, `BUMP`, `MARKER_*` | sim only: ground truth and metrics |

use crate::Rgba;

pub const BACKGROUND: Rgba = [28, 28, 32, 255];
pub const PANEL: Rgba = [38, 38, 44, 255];
pub const TEXT: Rgba = [235, 235, 235, 255];
pub const TEXT_DIM: Rgba = [160, 160, 160, 255];

pub const UNKNOWN: Rgba = [64, 66, 74, 255];
pub const FREE: Rgba = [206, 206, 200, 255];
pub const OCCUPIED: Rgba = [20, 20, 20, 255];

pub const ROBOT: Rgba = [255, 255, 255, 200];
pub const ROBOT_EDGE: Rgba = [20, 20, 20, 255];
pub const PATH: Rgba = [40, 200, 60, 255];
pub const TRAJECTORY: Rgba = [255, 0, 200, 255];
pub const FRONTIER: Rgba = [50, 110, 255, 255];
pub const VIEW: Rgba = [255, 150, 0, 255];
pub const GOAL: Rgba = [255, 230, 0, 255];
pub const TARGET_TENTATIVE: Rgba = [255, 200, 0, 255];
pub const TARGET_CONFIRMED: Rgba = [0, 210, 90, 255];
pub const SCAN: Rgba = [255, 70, 70, 255];

// sim only
pub const WALL: Rgba = [170, 110, 255, 255];
pub const GLASS: Rgba = [0, 220, 255, 255];
pub const ESTIMATE: Rgba = [255, 120, 200, 255];
pub const TRAIL: Rgba = [40, 170, 160, 255];
pub const BUMP: Rgba = [255, 40, 40, 255];
pub const MARKER_UNSEEN: Rgba = [150, 150, 150, 255];
pub const MARKER_NEAR: Rgba = TARGET_TENTATIVE;
pub const MARKER_READABLE: Rgba = TARGET_CONFIRMED;
