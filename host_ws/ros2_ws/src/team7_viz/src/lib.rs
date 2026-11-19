//! team7_viz: a read-only map view of a team7 run, live or replayed, drawn with aurea.
//!
//! `ros` subscribes on its own thread and fills the shared `model` (all in the map frame); `view`
//! draws a model and handles keys and the mouse. Colours, the viewport and the shapes come from
//! `tb4_draw`, shared with the sim viewer.

pub mod model;
pub mod ros;
pub mod view;
