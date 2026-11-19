//! team7_viz: a map view of a team7 run, live or replayed, drawn with aurea. Read-only unless
//! started with `--allow-goals`.
//!
//! - `ros` subscribes on its own thread and fills the shared `model` (all in the map frame)
//! - `scene` draws the map layers, `app` holds the view, tools and input around the map canvas
//! - `gui` is the native window: menus, toolbar, dock (mission, camera, health, layers), status bar
//! - `health` and `outputs` are the checks behind the health and mission panels
//!
//! Colours, the viewport and the shapes come from `tb4_draw`, shared with the sim viewer.

pub mod app;
pub mod gui;
pub mod health;
pub mod model;
pub mod outputs;
pub mod ros;
pub mod scene;

pub use tb4_draw::Rgba;
