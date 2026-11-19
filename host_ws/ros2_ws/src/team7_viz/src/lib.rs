//! team7_viz: a map view of a team7 or stock run, live or replayed, drawn with aurea. Read-only
//! unless started with `--allow-goals`.
//!
//! - `ros` subscribes on its own thread and fills the shared `model` (all in the map frame); each
//!   layer is bound to the team7 or the stock topic that exists (`model::Slot`)
//! - `scene` draws the map layers, `app` holds the view, tools and input of the map
//! - `chrome` draws the rest of the window: toolbar, header strip, dock, status bar
//! - `health`, `outputs` and `shots` are the checks and files behind the panels
//!
//! Colours, the viewport and the shapes come from `tb4_draw`, shared with the sim viewer.

pub mod app;
pub mod chrome;
pub mod health;
pub mod model;
pub mod outputs;
pub mod profile;
pub mod ros;
pub mod scene;
pub mod shots;

pub use tb4_draw::Rgba;
