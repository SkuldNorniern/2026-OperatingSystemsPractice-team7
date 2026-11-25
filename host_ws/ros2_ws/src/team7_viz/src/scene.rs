//! The map view: layers, their legend symbols, and drawing one frame of them on any
//! `DrawingContext`. Draw order keeps walls readable: map, costmaps, grid, then the occupied cells
//! again on top, then everything else, the scan and the robot last.
//!
//! A layer is a kind (what it draws and how) plus a binding: Auto (the team7 or the stock topic,
//! whichever is published) or a topic the user picked. More than one layer of a kind can be shown,
//! e.g. two paths, or `/scan` and a filtered scan.

use std::collections::HashMap;
use std::time::Instant;

use aurea::AureaResult;
use aurea::render::{DrawingContext, Font, Image, Point};
use tb4_draw::palette as pal;
use tb4_draw::prim::{self, Prim, fill, stroke};
use tb4_draw::{Rgba, Viewport};

use crate::model::{self, GridLayer, MapData, Model, MsgKind, ROBOT_RADIUS, ShapeKind, Slot};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Layer {
    Map,
    GlobalCost,
    LocalCost,
    Scan,
    Path,
    Trajectory,
    Frontiers,
    Goal,
    /// ArUco ids seen but not confirmed
    Candidates,
    /// confirmed targets only
    Confirmed,
    /// raw detections of the current camera frame
    Detections,
    Views,
    /// camera views that saw a marker (`/team7/views_seen`), as wedges
    SeenViews,
    /// walls the camera has seen (green) and not seen yet (orange), `/team7/camera_coverage`
    Coverage,
    /// areas the explorer will not send goals to (`/team7/banned`): abort zones and failed spots
    Banned,
    SlamGraph,
    Robot,
    Grid,
    Trail,
}

pub struct LayerInfo {
    pub layer: Layer,
    pub name: &'static str,
    /// a few words: what it shows
    pub legend: &'static str,
    pub on: bool,
    /// None: no opacity control
    pub opacity: Option<f32>,
    /// where its data comes from; None: always there (TF, or drawn by the viewer)
    pub slot: Option<Slot>,
    /// the message type of its topic; None for the layers without one
    pub msg: Option<MsgKind>,
    /// the profile name of the kind
    pub key: &'static str,
}

#[allow(clippy::too_many_arguments)]
const fn info(layer: Layer, key: &'static str, name: &'static str, legend: &'static str, on: bool, opacity: Option<f32>, slot: Option<Slot>, msg: Option<MsgKind>) -> LayerInfo {
    LayerInfo { layer, name, legend, on, opacity, slot, msg, key }
}

/// default costmap opacity: readable over the map, walls stay on top
pub const COST_OPACITY: f32 = 0.35;

/// all base layers in panel order; the first nine toggle with keys 1-9. Costmaps and the raw
/// detections start off.
pub const LAYERS: [LayerInfo; 19] = [
    info(Layer::Map, "map", "Map", "free, wall, unknown", true, None, Some(Slot::Map), Some(MsgKind::Grid)),
    info(Layer::Scan, "scan", "Scan", "laser points", true, None, Some(Slot::Scan), Some(MsgKind::Scan)),
    info(Layer::Path, "plan", "Plan", "global path", true, None, Some(Slot::Plan), Some(MsgKind::Path)),
    info(Layer::Trajectory, "local_plan", "Local plan", "MPPI / controller path", true, None, Some(Slot::Trajectory), Some(MsgKind::Path)),
    info(Layer::Frontiers, "frontiers", "Frontiers", "exploration goals", true, None, Some(Slot::Frontiers), Some(MsgKind::Markers)),
    info(Layer::Goal, "goal", "Goal", "active goal", true, None, Some(Slot::Goal), Some(MsgKind::Pose)),
    info(Layer::Confirmed, "confirmed", "Confirmed", "confirmed targets", true, Some(1.0), Some(Slot::Targets), Some(MsgKind::Markers)),
    info(Layer::Candidates, "candidates", "Candidates", "ArUco ids seen, not confirmed", true, Some(1.0), Some(Slot::Targets), Some(MsgKind::Markers)),
    info(Layer::GlobalCost, "global_cost", "Global costmap", "low .. high cost", false, Some(COST_OPACITY), Some(Slot::GlobalCost), Some(MsgKind::Grid)),
    info(Layer::LocalCost, "local_cost", "Local cost", "around the robot", false, Some(COST_OPACITY), Some(Slot::LocalCost), Some(MsgKind::Grid)),
    info(Layer::Detections, "detections", "ArUco detections", "camera hits now", false, None, Some(Slot::Detections), Some(MsgKind::Detections)),
    info(Layer::Robot, "robot", "Robot", "pose, x/y axes", true, None, None, None),
    info(Layer::Views, "camera_views", "Camera views", "planned view poses", true, None, Some(Slot::Views), Some(MsgKind::Markers)),
    info(Layer::SeenViews, "seen_views", "Seen views", "where the camera saw a marker", true, None, Some(Slot::SeenViews), Some(MsgKind::Markers)),
    info(Layer::Coverage, "camera_coverage", "Camera coverage", "walls seen / not seen", true, Some(1.0), Some(Slot::Coverage), Some(MsgKind::Markers)),
    info(Layer::Banned, "banned", "Banned areas", "no goals sent here", true, None, Some(Slot::Banned), Some(MsgKind::Markers)),
    info(Layer::SlamGraph, "slam_graph", "SLAM graph", "keyframes, edges", true, None, Some(Slot::SlamGraph), Some(MsgKind::Markers)),
    info(Layer::Grid, "grid", "Grid", "1 m", true, None, None, None),
    info(Layer::Trail, "trail", "Trail", "where it drove", true, None, None, None),
];

/// kinds a second layer can be added for (one message type each)
pub const EXTRA_KINDS: [Layer; 5] = [Layer::Scan, Layer::Path, Layer::Trajectory, Layer::Map, Layer::LocalCost];

/// colours of added layers, so two paths or two scans are told apart
const EXTRA_COLORS: [Rgba; 4] = [[0, 200, 230, 255], [255, 150, 40, 255], [180, 230, 60, 255], [190, 120, 255, 255]];

pub fn info_of(l: Layer) -> &'static LayerInfo {
    LAYERS.iter().find(|i| i.layer == l).expect("every layer is listed")
}

/// costmaps get an opacity slider row; the other layers with an opacity a small button that
/// steps through `OPACITY_STEPS`, so the list stays short
pub fn has_slider(l: Layer) -> bool {
    matches!(l, Layer::GlobalCost | Layer::LocalCost)
}

pub const OPACITY_STEPS: [f32; 3] = [1.0, 0.6, 0.3];

/// the step after `o`
pub fn next_opacity(o: f32) -> f32 {
    let i = OPACITY_STEPS.iter().position(|s| (s - o).abs() < 0.05).map_or(0, |i| (i + 1) % OPACITY_STEPS.len());
    OPACITY_STEPS[i]
}

pub fn index(l: Layer) -> usize {
    LAYERS.iter().position(|i| i.layer == l).expect("every layer is listed")
}

/// where a layer takes its topic from
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub enum Binding {
    /// the team7 or the stock topic, whichever is published
    #[default]
    Auto,
    /// a topic the user picked
    Pinned(String),
}

/// one layer row: kind, binding, visibility, opacity
#[derive(Debug, Clone, PartialEq)]
pub struct LayerSpec {
    pub kind: Layer,
    pub on: bool,
    pub opacity: f32,
    pub binding: Binding,
    /// added by the user (can be removed; drawn in its own colour)
    pub extra: bool,
}

impl LayerSpec {
    pub fn base(i: &LayerInfo) -> Self {
        Self { kind: i.layer, on: i.on, opacity: i.opacity.unwrap_or(1.0), binding: Binding::Auto, extra: false }
    }

    pub fn info(&self) -> &'static LayerInfo {
        info_of(self.kind)
    }

    /// the topic it draws from now; None when Auto found nothing or the kind has no topic
    pub fn topic(&self, model: &Model) -> Option<String> {
        match &self.binding {
            Binding::Pinned(t) => Some(t.clone()),
            Binding::Auto => self.info().slot.and_then(|s| model.bound(s)).map(str::to_owned),
        }
    }
}

/// the layer list: the base layers in panel order, then added ones
#[derive(Debug, Clone, PartialEq)]
pub struct Layers {
    pub list: Vec<LayerSpec>,
    /// the camera panel's image topic
    pub camera: Binding,
}

impl Default for Layers {
    fn default() -> Self {
        Self { list: LAYERS.iter().map(LayerSpec::base).collect(), camera: Binding::Auto }
    }
}

impl Layers {
    fn first(&self, l: Layer) -> Option<&LayerSpec> {
        self.list.iter().find(|s| s.kind == l && !s.extra)
    }

    pub fn shown(&self, l: Layer) -> bool {
        self.first(l).is_some_and(|s| s.on)
    }

    pub fn toggle(&mut self, l: Layer) {
        if let Some(s) = self.list.iter_mut().find(|s| s.kind == l && !s.extra) {
            s.on = !s.on;
        }
    }

    pub fn opacity(&self, l: Layer) -> f32 {
        self.first(l).map_or(1.0, |s| s.opacity)
    }

    pub fn set_opacity(&mut self, l: Layer, o: f32) {
        if let Some(s) = self.list.iter_mut().find(|s| s.kind == l && !s.extra) {
            s.opacity = o;
        }
    }

    /// the topic of the base layer of a kind (the header strip's targets, the map to fit to)
    pub fn topic(&self, l: Layer, model: &Model) -> Option<String> {
        self.first(l).and_then(|s| s.topic(model))
    }

    pub fn camera_topic(&self, model: &Model) -> Option<String> {
        match &self.camera {
            Binding::Pinned(t) => Some(t.clone()),
            Binding::Auto => model.bound(Slot::Camera).map(str::to_owned),
        }
    }

    /// add a layer of `kind`, bound to `topic`
    pub fn add(&mut self, kind: Layer, topic: Option<String>) {
        let o = info_of(kind).opacity.unwrap_or(1.0);
        self.list.push(LayerSpec { kind, on: true, opacity: o, binding: topic.map_or(Binding::Auto, Binding::Pinned), extra: true });
    }

    /// colour of an added layer, by its place among the added ones
    pub fn extra_color(&self, idx: usize) -> Rgba {
        let n = self.list[..idx].iter().filter(|s| s.extra).count();
        EXTRA_COLORS[n % EXTRA_COLORS.len()]
    }

    /// what has to be subscribed: Auto layers take every candidate (so they can switch as the
    /// graph changes), pinned ones their topic, plus the camera and the health topics
    pub fn wanted(&self) -> Vec<(String, MsgKind)> {
        let mut out: Vec<(String, MsgKind)> = vec![("/scan".into(), MsgKind::Scan), ("/map".into(), MsgKind::Grid)];
        let mut add = |t: &str, k: MsgKind| {
            if !out.iter().any(|(o, ok)| o == t && *ok == k) {
                out.push((t.to_owned(), k));
            }
        };
        for s in &self.list {
            let (Some(k), slot) = (s.info().msg, s.info().slot) else { continue };
            match (&s.binding, slot) {
                (Binding::Pinned(t), _) => add(t, k),
                (Binding::Auto, Some(slot)) => slot.candidates().iter().for_each(|t| add(t, k)),
                (Binding::Auto, None) => {}
            }
        }
        match &self.camera {
            Binding::Pinned(t) => add(t, MsgKind::Image),
            Binding::Auto => Slot::Camera.candidates().iter().for_each(|t| add(t, MsgKind::Image)),
        }
        out.sort();
        out
    }
}

/// images rebuilt only when their grid (or opacity) changed, per topic
#[derive(Default)]
pub struct Cache {
    maps: HashMap<String, (u64, Image, Image)>,
    costs: HashMap<String, (u64, f32, Image)>,
}

/// where a grid image lands: the grid frame is `info.origin` (position and yaw) in the map frame
pub fn grid_layer(m: &MapData, vp: &Viewport) -> tb4_draw::MapLayer {
    let at_origin = tb4_core::GridMeta { origin_x: 0.0, origin_y: 0.0, ..m.meta };
    tb4_draw::map_layer(&at_origin, (m.meta.origin_x, m.meta.origin_y, m.origin_yaw), vp)
}

impl Cache {
    fn cost(&mut self, topic: &str, g: &GridLayer, opacity: f32) -> Option<Image> {
        let m = g.grid.as_ref()?;
        let slot = self.costs.get(topic);
        if slot.is_none_or(|(seq, o, _)| *seq != g.seq || *o != opacity) {
            let img = tb4_draw::costmap_image(&m.meta, &m.data, opacity)?;
            self.costs.insert(topic.to_owned(), (g.seq, opacity, img));
        }
        self.costs.get(topic).map(|(.., i)| i.clone())
    }

    /// the map image and its occupied-only overlay
    fn map(&mut self, topic: &str, g: &GridLayer) -> Option<(Image, Image)> {
        let m = g.grid.as_ref()?;
        if self.maps.get(topic).is_none_or(|(seq, ..)| *seq != g.seq) {
            let (a, b) = tb4_draw::map_image(&m.meta, &m.data).zip(tb4_draw::occupied_image(&m.meta, &m.data))?;
            self.maps.insert(topic.to_owned(), (g.seq, a, b));
        }
        self.maps.get(topic).map(|(_, a, b)| (a.clone(), b.clone()))
    }
}

/// grid lines every metre over `bounds`, faint
fn grid_lines(bounds: [f64; 4], vp: &Viewport) -> Vec<Prim> {
    let [x0, y0, x1, y1] = bounds.map(f64::floor);
    let (x1, y1) = (x1 + 1.0, y1 + 1.0);
    let mut out = Vec::new();
    let line = |a: (f64, f64), b: (f64, f64)| Prim::Line { a: vp.to_screen(a.0, a.1), b: vp.to_screen(b.0, b.1), color: pal::GRID, width: 1.0, dashed: false };
    let mut x = x0;
    while x <= x1 && out.len() < 400 {
        out.push(line((x, y0), (x, y1)));
        x += 1.0;
    }
    let mut y = y0;
    while y <= y1 && out.len() < 800 {
        out.push(line((x0, y), (x1, y)));
        y += 1.0;
    }
    out
}

/// the drawing order of the kinds over the map images
const ORDER: [Layer; 16] = [
    Layer::Coverage,
    Layer::Banned,
    Layer::Trail,
    Layer::SlamGraph,
    Layer::Path,
    Layer::Trajectory,
    Layer::Frontiers,
    Layer::SeenViews,
    Layer::Views,
    Layer::Goal,
    Layer::Detections,
    Layer::Candidates,
    Layer::Confirmed,
    Layer::Scan,
    Layer::Robot,
    Layer::Map,
];

/// camera coverage: wall seen / not seen yet
const COVERAGE_SEEN: Rgba = [60, 210, 110, 230];
const COVERAGE_UNSEEN: Rgba = [255, 140, 30, 230];
/// banned areas: navigator abort zone / failed goal spot
const BANNED_ZONE: Rgba = [235, 50, 50, 230];
const BANNED_SPOT: Rgba = [235, 120, 120, 180];

/// `c` with its alpha scaled by `o`
fn faded(c: Rgba, o: f32) -> Rgba {
    [c[0], c[1], c[2], (f32::from(c[3]) * o.clamp(0.0, 1.0)).round() as u8]
}

/// a candidate: hollow amber ring and "id?"; the selected one larger, with its failing rules
fn candidate_prims(p: (f32, f32), tg: &model::Target, selected: bool, o: f32) -> Vec<Prim> {
    let c = faded(pal::TARGET_TENTATIVE, o);
    let r = if selected { 10.0 } else { 7.0 };
    let mut out = vec![
        Prim::Circle { c: p, r: r + 1.5, fill: None, stroke: Some((faded(pal::ROBOT_EDGE, o), 1.5)) },
        Prim::Circle { c: p, r, fill: None, stroke: Some((c, 2.5)) },
    ];
    out.extend(prim::outlined_text((p.0 + r + 3.0, p.1 - r + 2.0), &format!("{}?", tg.id), c, 12.0));
    if selected && !tg.reasons.is_empty() {
        out.extend(prim::outlined_text((p.0 + r + 3.0, p.1 + r + 10.0), &tg.reasons.join(", "), faded(pal::TEXT, o), 11.0));
    }
    out
}

/// a confirmed target: solid green, id and position
fn confirmed_prims(p: (f32, f32), tg: &model::Target, selected: bool, o: f32) -> Vec<Prim> {
    let c = faded(pal::TARGET_CONFIRMED, o);
    let r = if selected { 9.0 } else { 7.0 };
    let mut out = vec![Prim::Circle { c: p, r, fill: Some(c), stroke: Some((faded(pal::ROBOT_EDGE, o), 1.5)) }];
    out.extend(prim::outlined_text((p.0 + r + 3.0, p.1 - r - 2.0), &format!("ID {} ({:.2}, {:.2})", tg.id, tg.x, tg.y), c, 13.0));
    out
}

/// everything over the map images, in drawing order. `selected`: the target id picked in the dock
pub fn prims(model: &Model, layers: &Layers, vp: &Viewport, now: Instant, selected: Option<i32>) -> Vec<Prim> {
    let at = |&(x, y): &(f64, f64)| vp.to_screen(x, y);
    let mut out = Vec::new();
    for kind in ORDER {
        for (idx, s) in layers.list.iter().enumerate().filter(|(_, s)| s.kind == kind && s.on) {
            let topic = s.topic(model);
            let t = topic.as_deref().unwrap_or("");
            let color = |base: Rgba| if s.extra { layers.extra_color(idx) } else { base };
            match kind {
                Layer::Trail if model.trail.len() >= 2 => out.push(Prim::Polyline { pts: model.trail.iter().map(at).collect(), color: pal::TRAIL, width: 1.5 }),
                Layer::SlamGraph => {
                    for m in model.markers(t) {
                        let pts: Vec<_> = m.pts.iter().map(at).collect();
                        match m.kind {
                            ShapeKind::Points => out.extend(pts.iter().map(|&p| prim::dot(p, 2.0, m.color))),
                            ShapeKind::Strip if pts.len() >= 2 => out.push(Prim::Polyline { pts, color: m.color, width: 1.0 }),
                            ShapeKind::Segments => {
                                out.extend(pts.as_chunks::<2>().0.iter().map(|&[a, b]| Prim::Line { a, b, color: m.color, width: 1.0, dashed: false }));
                            }
                            ShapeKind::Strip => {}
                        }
                    }
                }
                Layer::Path | Layer::Trajectory => {
                    let p = model.points(t);
                    if p.len() >= 2 {
                        let base = if kind == Layer::Path { pal::PATH } else { pal::TRAJECTORY };
                        out.push(Prim::Polyline { pts: p.iter().map(at).collect(), color: color(base), width: 2.0 });
                    }
                }
                Layer::Frontiers => out.extend(model::frontier_points(model.markers(t)).iter().map(|p| prim::dot(at(p), 4.0, pal::FRONTIER))),
                Layer::Views => out.extend(model::view_points(model.markers(t)).iter().map(|p| Prim::Points { pts: vec![at(p)], size: 7.0, color: pal::VIEW })),
                Layer::Coverage => {
                    for m in model.markers(t) {
                        let c = if m.ns == "coverage_seen" { COVERAGE_SEEN } else { COVERAGE_UNSEEN };
                        out.push(Prim::Points { pts: m.pts.iter().map(at).collect(), size: 3.0, color: color(c) });
                    }
                }
                Layer::Banned => {
                    // LINE_LIST circles; abort zones stronger than failed spots
                    for m in model.markers(t) {
                        let c = if m.ns == "banned_zone" { BANNED_ZONE } else { BANNED_SPOT };
                        let pts: Vec<_> = m.pts.iter().map(at).collect();
                        out.extend(pts.as_chunks::<2>().0.iter().map(|&[a, b]| Prim::Line { a, b, color: color(c), width: 2.0, dashed: true }));
                    }
                }
                Layer::SeenViews => {
                    // LINE_LIST wedges: each pair of points is one edge
                    for m in model.markers(t) {
                        let pts: Vec<_> = m.pts.iter().map(at).collect();
                        out.extend(pts.as_chunks::<2>().0.iter().map(|&[a, b]| Prim::Line { a, b, color: color(pal::SEEN_VIEW), width: 1.0, dashed: false }));
                    }
                }
                Layer::Goal => {
                    if let Some(g) = model.pose(t) {
                        out.push(Prim::Circle { c: at(&g), r: 7.0, fill: None, stroke: Some((pal::ROBOT_EDGE, 4.0)) });
                        out.extend(prim::goal(at(&g)));
                    }
                }
                Layer::Candidates => {
                    for tg in model::candidates(&model::targets_of(model.markers(t))).iter().rev() {
                        out.extend(candidate_prims(at(&(tg.x, tg.y)), tg, selected == Some(tg.id), s.opacity));
                    }
                }
                Layer::Confirmed => {
                    for tg in model::confirmed(&model::targets_of(model.markers(t))) {
                        out.extend(confirmed_prims(at(&(tg.x, tg.y)), &tg, selected == Some(tg.id), s.opacity));
                    }
                }
                Layer::Detections => {
                    let rays = model.rays(t);
                    for (k, r) in rays.iter().enumerate() {
                        // the line back to the camera only for the newest hit of an id
                        let newest = !rays[k + 1..].iter().any(|o| o.id == r.id);
                        let age = now.saturating_duration_since(r.at).as_secs_f32() / model::RAY_FOR.as_secs_f32();
                        if age >= 1.0 {
                            continue;
                        }
                        let c = faded(pal::DETECTION, 1.0 - age * 0.8);
                        // a tick along the bearing, ending at the estimated marker position
                        let (a, b) = (at(&r.from), at(&r.to));
                        let (dx, dy) = (b.0 - a.0, b.1 - a.1);
                        let l = dx.hypot(dy).max(1e-3);
                        let tick = 18.0f32.min(l);
                        out.push(Prim::Line { a: (b.0 - dx / l * tick, b.1 - dy / l * tick), b, color: c, width: 2.0, dashed: false });
                        if newest {
                            out.push(Prim::Line { a, b, color: faded(c, 0.5), width: 1.0, dashed: true });
                            out.extend(prim::outlined_text((b.0 + 4.0, b.1 + 12.0), &r.id.to_string(), c, 11.0));
                        }
                    }
                }
                Layer::Scan => {
                    let p = model.points(t);
                    if !p.is_empty() {
                        out.push(Prim::Points { pts: p.iter().map(at).collect(), size: 3.0, color: color(pal::SCAN) });
                    }
                }
                Layer::Robot => {
                    if let Some((x, y, yaw)) = model.robot {
                        let c = vp.to_screen(x, y);
                        // true size when zoomed in, never smaller than an easy-to-see icon
                        let r = vp.len(ROBOT_RADIUS).max(9.0);
                        out.extend(prim::robot(c, r, vp.heading(yaw)));
                        // x (red) and y (green) axes, 0.3 m, like RViz's robot axes
                        let axis = |a: f64, color| {
                            let h = vp.heading(yaw + a);
                            let l = vp.len(0.3).max(r + 8.0);
                            Prim::Line { a: c, b: (c.0 + l * h.cos() as f32, c.1 - l * h.sin() as f32), color, width: 2.0, dashed: false }
                        };
                        out.push(axis(0.0, pal::AXIS_X));
                        out.push(axis(std::f64::consts::FRAC_PI_2, pal::AXIS_Y));
                    }
                }
                _ => {}
            }
        }
    }
    out
}

/// one frame of the map view in `ctx` (the whole context is the view)
#[allow(clippy::too_many_arguments)]
pub fn draw(ctx: &mut dyn DrawingContext, model: &Model, layers: &Layers, cache: &mut Cache, vp: &Viewport, grid_bounds: [f64; 4], now: Instant, selected: Option<i32>) -> AureaResult<()> {
    let place = |g: &GridLayer| g.grid.as_ref().map(|m| grid_layer(m, vp));
    // maps: images now, their walls again after the costmaps and the grid
    let mut walls = Vec::new();
    for s in layers.list.iter().filter(|s| s.kind == Layer::Map && s.on) {
        let Some(t) = s.topic(model) else { continue };
        let Some(g) = model.grid(&t) else { continue };
        if let (Some((img, w)), Some(l)) = (cache.map(&t, g), place(g)) {
            tb4_draw::draw_map(ctx, &img, &l)?;
            walls.push((w, l));
        }
    }
    for s in layers.list.iter().filter(|s| matches!(s.kind, Layer::GlobalCost | Layer::LocalCost) && s.on) {
        let Some(t) = s.topic(model) else { continue };
        let Some(g) = model.grid(&t) else { continue };
        if let (Some(img), Some(l)) = (cache.cost(&t, g, s.opacity), place(g)) {
            tb4_draw::draw_map(ctx, &img, &l)?;
        }
    }
    if layers.shown(Layer::Grid) {
        tb4_draw::paint_prims(ctx, &grid_lines(grid_bounds, vp))?;
    }
    for (w, l) in &walls {
        tb4_draw::draw_map(ctx, w, l)?;
    }
    tb4_draw::paint_prims(ctx, &prims(model, layers, vp, now, selected))
}

/// scale bar bottom left and, when the view is turned, a north arrow top right
pub fn chrome(ctx: &mut dyn DrawingContext, vp: &Viewport, w: f32, h: f32) -> AureaResult<()> {
    let font = Font::monospace(13.0);
    let metres = [10.0, 5.0, 2.0, 1.0, 0.5, 0.1].into_iter().find(|&m| vp.len(m) <= 160.0).unwrap_or(0.1);
    let len = vp.len(metres);
    let (x, y) = (16.0, h - 18.0);
    ctx.draw_path(&prim::polyline(&[(x, y - 5.0), (x, y), (x + len, y), (x + len, y - 5.0)]), &stroke(pal::TEXT, 2.0))?;
    ctx.draw_text_with_font(&format!("{metres} m"), Point::new(x + len + 8.0, y + 1.0), &font, &fill(pal::TEXT))?;
    if vp.rot != 0.0 {
        let c = (w - 30.0, 34.0);
        let n = vp.heading(std::f64::consts::FRAC_PI_2);
        let tip = (c.0 + 16.0 * n.cos() as f32, c.1 - 16.0 * n.sin() as f32);
        tb4_draw::paint_prims(ctx, &[Prim::Circle { c, r: 20.0, fill: Some([0, 0, 0, 120]), stroke: Some((pal::TEXT_DIM, 1.0)) }, Prim::Line { a: c, b: tip, color: pal::AXIS_Y, width: 3.0, dashed: false }])?;
        ctx.draw_text_with_font("N", Point::new(tip.0 - 4.0, tip.1 - 4.0), &font, &fill(pal::TEXT))?;
    }
    Ok(())
}

/// the legend symbol of a layer, drawn as on the map, centred at `c` (for the layer list swatches)
pub fn symbol(l: Layer, c: (f32, f32)) -> Vec<Prim> {
    let (x, y) = c;
    let square = |dx: f32, color: Rgba| Prim::Points { pts: vec![(x + dx, y)], size: 7.0, color };
    let line = |color: Rgba, width: f32| Prim::Line { a: (x - 10.0, y + 3.0), b: (x + 10.0, y - 3.0), color, width, dashed: false };
    match l {
        Layer::Map => vec![square(-8.0, pal::FREE), square(0.0, pal::OCCUPIED), square(8.0, pal::UNKNOWN)],
        Layer::GlobalCost | Layer::LocalCost => vec![square(-8.0, pal::COST_LOW), square(0.0, pal::COST_MID), square(8.0, pal::COST_HIGH)],
        Layer::Scan => vec![Prim::Points { pts: vec![(x - 7.0, y + 2.0), (x - 2.0, y), (x + 3.0, y - 1.0), (x + 8.0, y - 1.0)], size: 3.0, color: pal::SCAN }],
        Layer::Path => vec![line(pal::PATH, 2.0)],
        Layer::Trajectory => vec![line(pal::TRAJECTORY, 2.0)],
        Layer::Trail => vec![line(pal::TRAIL, 1.5)],
        Layer::Frontiers => vec![prim::dot(c, 4.0, pal::FRONTIER)],
        Layer::Views => vec![square(0.0, pal::VIEW)],
        Layer::SeenViews => vec![Prim::Line { a: (x - 8.0, y + 4.0), b: (x + 8.0, y - 4.0), color: pal::SEEN_VIEW, width: 2.0, dashed: false }],
        Layer::Coverage => vec![square(-4.0, COVERAGE_SEEN), square(4.0, COVERAGE_UNSEEN)],
        Layer::Banned => vec![Prim::Circle { c, r: 6.0, fill: None, stroke: Some((BANNED_ZONE, 2.0)) }],
        Layer::Goal => prim::goal(c).to_vec(),
        Layer::Candidates => vec![Prim::Circle { c, r: 5.5, fill: None, stroke: Some((pal::TARGET_TENTATIVE, 2.0)) }],
        Layer::Confirmed => vec![Prim::Circle { c, r: 5.5, fill: Some(pal::TARGET_CONFIRMED), stroke: Some((pal::ROBOT_EDGE, 1.0)) }],
        Layer::Detections => vec![
            Prim::Line { a: (x - 10.0, y + 4.0), b: (x + 6.0, y - 2.0), color: pal::DETECTION, width: 1.0, dashed: true },
            Prim::Line { a: (x + 1.0, y), b: (x + 9.0, y - 3.0), color: pal::DETECTION, width: 2.0, dashed: false },
        ],
        Layer::SlamGraph => vec![
            Prim::Line { a: (x - 9.0, y + 3.0), b: (x + 9.0, y - 3.0), color: pal::SLAM_GRAPH, width: 1.0, dashed: false },
            prim::dot((x - 9.0, y + 3.0), 2.0, pal::SLAM_GRAPH),
            prim::dot((x + 9.0, y - 3.0), 2.0, pal::SLAM_GRAPH),
        ],
        Layer::Robot => prim::robot(c, 6.0, 0.6).to_vec(),
        Layer::Grid => vec![
            Prim::Line { a: (x - 10.0, y - 4.0), b: (x + 10.0, y - 4.0), color: [140, 150, 160, 255], width: 1.0, dashed: false },
            Prim::Line { a: (x - 10.0, y + 4.0), b: (x + 10.0, y + 4.0), color: [140, 150, 160, 255], width: 1.0, dashed: false },
            Prim::Line { a: (x, y - 7.0), b: (x, y + 7.0), color: [140, 150, 160, 255], width: 1.0, dashed: false },
        ],
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::app::App;
    use crate::app::tests::{near, render, rgb, scene};
    use crate::chrome::{binding_of, topic_choices};
    use crate::model::Feed;
    use crate::model::tests::graph;
    use std::time::Instant;

    fn fixed() -> App {
        let mut a = App::new(false);
        a.camera.follow = false;
        a
    }

    #[test]
    fn picking_a_topic_rebinds_the_layer() {
        let now = Instant::now();
        let mut m = scene();
        // a second path topic, south of the room
        m.put("/team7/path_alt", Feed::Points(vec![(0.0, -0.6), (1.5, -0.6)]), now);
        let mut app = fixed();
        let (w, h) = (900u32, 600u32);
        render(&mut app, &m, w, h);
        let (vp, _) = app.viewport().unwrap();
        let at = |px: &[u32], x: f64, y: f64| {
            let (sx, sy) = vp.to_screen(x, y);
            px[sy as usize * w as usize + sx as usize]
        };
        let px = render(&mut app, &m, w, h);
        assert!(near(at(&px, 2.0, 1.0), pal::PATH, 20), "auto path {:?}", rgb(at(&px, 2.0, 1.0)));
        assert!(!near(at(&px, 0.8, -0.6), pal::PATH, 20));

        // pin the Plan layer to the other topic: drawn there, subscribed, old topic gone from the layer
        let plan = index(Layer::Path);
        app.layers.list[plan].binding = Binding::Pinned("/team7/path_alt".into());
        let px = render(&mut app, &m, w, h);
        assert!(near(at(&px, 0.8, -0.6), pal::PATH, 20), "pinned path {:?}", rgb(at(&px, 0.8, -0.6)));
        assert!(!near(at(&px, 2.0, 1.0), pal::PATH, 20));
        let wanted = app.layers.wanted();
        assert!(wanted.contains(&("/team7/path_alt".into(), MsgKind::Path)));
        assert!(!wanted.contains(&("/plan".into(), MsgKind::Path)), "the pinned layer no longer asks for its auto candidates");

        // back to Auto through the picker's choices
        m.graph = graph(&[("/team7/path", "nav_msgs/msg/Path", 1), ("/team7/path_alt", "nav_msgs/msg/Path", 1), ("/plan", "nav_msgs/msg/Path", 0)]);
        let c = topic_choices(&m, MsgKind::Path, &app.layers.list[plan].binding, m.bound(Slot::Plan), now);
        assert_eq!(c.topics, vec![None, Some("/team7/path".into()), Some("/team7/path_alt".into()), Some("/plan".into())]);
        assert_eq!(c.selected, 2);
        assert!(c.labels[0].starts_with("Auto") && c.labels[0].contains("/team7/path"));
        assert_eq!(binding_of(&c, 0), Binding::Auto);
        assert_eq!(binding_of(&c, 3), Binding::Pinned("/plan".into()));
        // a pinned topic missing from the graph is still offered, marked
        let gone = topic_choices(&m, MsgKind::Path, &Binding::Pinned("/old_path".into()), None, now);
        assert!(gone.labels[gone.selected].contains("no publisher"));
    }

    #[test]
    fn candidates_and_confirmed_are_separate_layers() {
        let mut m = scene();
        let mut app = fixed();
        let (w, h) = (900u32, 600u32);
        render(&mut app, &m, w, h);
        let (vp, _) = app.viewport().unwrap();
        let px_at = |px: &[u32], x: f64, y: f64, dx: f32| {
            let (sx, sy) = vp.to_screen(x, y);
            px[sy as usize * w as usize + (sx + dx) as usize]
        };
        // scene(): 7 confirmed at (0, 2), 3 a candidate at (2.5, -0.5)
        let px = render(&mut app, &m, w, h);
        assert!(near(px_at(&px, 0.0, 2.0, 0.0), pal::TARGET_CONFIRMED, 20));
        assert!(near(px_at(&px, 2.5, -0.5, 7.0), pal::TARGET_TENTATIVE, 30));
        // candidates off: the confirmed one stays
        app.layers.toggle(Layer::Candidates);
        let px = render(&mut app, &m, w, h);
        assert!(!near(px_at(&px, 2.5, -0.5, 7.0), pal::TARGET_TENTATIVE, 60));
        assert!(near(px_at(&px, 0.0, 2.0, 0.0), pal::TARGET_CONFIRMED, 20));
        // confirmed off, candidates on at 30 %: a fainter ring, no green
        app.layers.toggle(Layer::Candidates);
        app.layers.toggle(Layer::Confirmed);
        let ci = index(Layer::Candidates);
        app.layers.list[ci].opacity = next_opacity(next_opacity(1.0));
        assert_eq!(app.layers.list[ci].opacity, 0.3);
        let px = render(&mut app, &m, w, h);
        assert!(!near(px_at(&px, 0.0, 2.0, 0.0), pal::TARGET_CONFIRMED, 60));
        let ring = px_at(&px, 2.5, -0.5, 7.0);
        assert!(!near(ring, pal::TARGET_TENTATIVE, 30) && rgb(ring)[0] > rgb(ring)[2], "faint ring {:?}", rgb(ring));
        assert_eq!(next_opacity(0.3), 1.0);

        // raw detections: off by default; on, a fresh tick shows at the marker, an old one not
        assert!(!app.layers.shown(Layer::Detections));
        app.layers.toggle(Layer::Detections);
        let now = Instant::now();
        m.graph = graph(&[("/aruco_detections", "aruco_opencv_msgs/msg/ArucoDetection", 1)]);
        m.add_rays("/aruco_detections", vec![model::Ray { id: 9, from: (0.5, 0.5), to: (2.0, 0.5), at: now }], now);
        let px = render(&mut app, &m, w, h);
        assert!(near(px_at(&px, 2.0, 0.5, -3.0), pal::DETECTION, 40), "tick {:?}", rgb(px_at(&px, 2.0, 0.5, -3.0)));
        let later = now + model::RAY_FOR + std::time::Duration::from_millis(10);
        m.add_rays("/aruco_detections", vec![], later);
        let px = render(&mut app, &m, w, h);
        assert!(!near(px_at(&px, 2.0, 0.5, -3.0), pal::DETECTION, 40));
    }

    #[test]
    fn auto_rebinds_when_the_stock_topic_appears() {
        let now = Instant::now();
        let mut m = Model::default();
        let layers = Layers::default();
        let plan = &layers.list[index(Layer::Path)];
        assert_eq!(plan.topic(&m), None);
        // both candidates are subscribed while Auto, so either can start publishing
        let wanted = layers.wanted();
        assert!(wanted.contains(&("/team7/path".into(), MsgKind::Path)) && wanted.contains(&("/plan".into(), MsgKind::Path)));
        // Nav2's planner comes up
        m.graph = graph(&[("/plan", "nav_msgs/msg/Path", 1)]);
        m.put("/plan", Feed::Points(vec![(0.0, 0.0), (1.0, 0.0)]), now);
        assert_eq!(plan.topic(&m).as_deref(), Some("/plan"));
        // our planner comes up too: team7 wins
        m.graph.extend(graph(&[("/team7/path", "nav_msgs/msg/Path", 1)]));
        assert_eq!(plan.topic(&m).as_deref(), Some("/team7/path"));
    }

    #[test]
    fn an_added_layer_draws_its_topic_in_its_own_colour() {
        let now = Instant::now();
        let mut m = scene();
        m.put("/scan_filtered", Feed::Points((0..40).map(|i| (-0.5 + 0.05 * f64::from(i), -0.5)).collect()), now);
        let mut app = fixed();
        app.layers.add(Layer::Scan, Some("/scan_filtered".into()));
        let idx = app.layers.list.len() - 1;
        let c = app.layers.extra_color(idx);
        assert_ne!(c, pal::SCAN);
        let (w, h) = (900u32, 600u32);
        render(&mut app, &m, w, h);
        let px = render(&mut app, &m, w, h);
        let (vp, _) = app.viewport().unwrap();
        let (sx, sy) = vp.to_screen(0.5, -0.5);
        let p = px[sy as usize * w as usize + sx as usize];
        assert!(near(p, c, 30), "added scan {:?}", rgb(p));
        // the base scan still draws in red
        let (sx, sy) = vp.to_screen(1.0, 2.8);
        assert!(near(px[sy as usize * w as usize + sx as usize], pal::SCAN, 20));
        assert!(app.layers.wanted().contains(&("/scan_filtered".into(), MsgKind::Scan)));
    }
}
