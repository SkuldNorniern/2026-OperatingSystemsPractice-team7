//! The map view: layers, their legend symbols, and drawing one frame of them on any
//! `DrawingContext`. Draw order keeps walls readable: map, costmaps, grid, then the occupied cells
//! again on top, then everything else, the scan and the robot last.

use aurea::AureaResult;
use aurea::render::{DrawingContext, Font, Image, Point};
use tb4_draw::palette as pal;
use tb4_draw::prim::{self, Prim, fill, stroke};
use tb4_draw::{Rgba, Viewport};

use crate::model::{GridLayer, MapData, Model, ROBOT_RADIUS, ShapeKind, Slot};

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
    Targets,
    Views,
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
}

const fn info(layer: Layer, name: &'static str, legend: &'static str, on: bool, opacity: Option<f32>, slot: Option<Slot>) -> LayerInfo {
    LayerInfo { layer, name, legend, on, opacity, slot }
}

/// default costmap opacity: readable over the map, walls stay on top
pub const COST_OPACITY: f32 = 0.35;

/// all layers in panel order; the first nine toggle with keys 1-9. Costmaps start off.
pub const LAYERS: [LayerInfo; 14] = [
    info(Layer::Map, "Map", "free, wall, unknown", true, None, Some(Slot::Map)),
    info(Layer::Scan, "Scan", "laser points", true, None, None),
    info(Layer::Path, "Plan", "global path", true, None, Some(Slot::Plan)),
    info(Layer::Trajectory, "Local plan", "MPPI / controller path", true, None, Some(Slot::Trajectory)),
    info(Layer::Frontiers, "Frontiers", "exploration goals", true, None, Some(Slot::Frontiers)),
    info(Layer::Goal, "Goal", "active goal", true, None, Some(Slot::Goal)),
    info(Layer::Targets, "Targets", "tentative / confirmed", true, None, Some(Slot::Targets)),
    info(Layer::GlobalCost, "Global costmap", "low .. high cost", false, Some(COST_OPACITY), Some(Slot::GlobalCost)),
    info(Layer::LocalCost, "Local cost", "around the robot", false, Some(COST_OPACITY), Some(Slot::LocalCost)),
    info(Layer::Robot, "Robot", "pose, x/y axes", true, None, None),
    info(Layer::Views, "Camera views", "planned view poses", true, None, Some(Slot::Views)),
    info(Layer::SlamGraph, "SLAM graph", "keyframes, edges", true, None, Some(Slot::SlamGraph)),
    info(Layer::Grid, "Grid", "1 m", true, None, None),
    info(Layer::Trail, "Trail", "where it drove", true, None, None),
];

pub fn index(l: Layer) -> usize {
    LAYERS.iter().position(|i| i.layer == l).expect("every layer is listed")
}

/// which layers are shown and their opacity
#[derive(Debug, Clone, PartialEq)]
pub struct Layers {
    pub on: [bool; 14],
    pub opacity: [f32; 14],
}

impl Default for Layers {
    fn default() -> Self {
        Self { on: LAYERS.map(|i| i.on), opacity: LAYERS.map(|i| i.opacity.unwrap_or(1.0)) }
    }
}

impl Layers {
    pub fn shown(&self, l: Layer) -> bool {
        self.on[index(l)]
    }

    pub fn toggle(&mut self, l: Layer) {
        let i = index(l);
        self.on[i] = !self.on[i];
    }

    pub fn opacity(&self, l: Layer) -> f32 {
        self.opacity[index(l)]
    }
}

/// images rebuilt only when their grid (or opacity) changed
#[derive(Default)]
pub struct Cache {
    map: Option<(u64, Image, Image)>,
    global: Option<(u64, f32, Image)>,
    local: Option<(u64, f32, Image)>,
}

/// where a grid image lands: the grid frame is `info.origin` (position and yaw) in the map frame
pub fn grid_layer(m: &MapData, vp: &Viewport) -> tb4_draw::MapLayer {
    let at_origin = tb4_core::GridMeta { origin_x: 0.0, origin_y: 0.0, ..m.meta };
    tb4_draw::map_layer(&at_origin, (m.meta.origin_x, m.meta.origin_y, m.origin_yaw), vp)
}

fn cost_image(slot: &mut Option<(u64, f32, Image)>, g: &GridLayer, opacity: f32) -> Option<Image> {
    let m = g.grid.as_ref()?;
    if slot.as_ref().is_none_or(|(seq, o, _)| *seq != g.seq || *o != opacity) {
        *slot = tb4_draw::costmap_image(&m.meta, &m.data, opacity).map(|i| (g.seq, opacity, i));
    }
    slot.as_ref().map(|(.., i)| i.clone())
}

impl Cache {
    /// the map image and its occupied-only overlay
    fn map(&mut self, g: &GridLayer) -> Option<(Image, Image)> {
        let m = g.grid.as_ref()?;
        if self.map.as_ref().is_none_or(|(seq, ..)| *seq != g.seq) {
            self.map = tb4_draw::map_image(&m.meta, &m.data).zip(tb4_draw::occupied_image(&m.meta, &m.data)).map(|(a, b)| (g.seq, a, b));
        }
        self.map.as_ref().map(|(_, a, b)| (a.clone(), b.clone()))
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

/// everything over the map images, in drawing order
pub fn prims(model: &Model, layers: &Layers, vp: &Viewport) -> Vec<Prim> {
    let at = |&(x, y): &(f64, f64)| vp.to_screen(x, y);
    let mut out = Vec::new();
    if layers.shown(Layer::Trail) && model.trail.len() >= 2 {
        out.push(Prim::Polyline { pts: model.trail.iter().map(at).collect(), color: pal::TRAIL, width: 1.5 });
    }
    if layers.shown(Layer::SlamGraph) {
        for s in &model.slam_graph {
            let pts: Vec<_> = s.pts.iter().map(at).collect();
            match s.kind {
                ShapeKind::Points => out.extend(pts.iter().map(|&p| prim::dot(p, 2.0, s.color))),
                ShapeKind::Strip if pts.len() >= 2 => out.push(Prim::Polyline { pts, color: s.color, width: 1.0 }),
                ShapeKind::Segments => {
                    out.extend(pts.as_chunks::<2>().0.iter().map(|&[a, b]| Prim::Line { a, b, color: s.color, width: 1.0, dashed: false }));
                }
                ShapeKind::Strip => {}
            }
        }
    }
    if layers.shown(Layer::Path) && model.path.len() >= 2 {
        out.push(Prim::Polyline { pts: model.path.iter().map(at).collect(), color: pal::PATH, width: 2.0 });
    }
    if layers.shown(Layer::Trajectory) && model.trajectory.len() >= 2 {
        out.push(Prim::Polyline { pts: model.trajectory.iter().map(at).collect(), color: pal::TRAJECTORY, width: 2.0 });
    }
    if layers.shown(Layer::Frontiers) {
        out.extend(model.frontiers.iter().map(|p| prim::dot(at(p), 4.0, pal::FRONTIER)));
    }
    if layers.shown(Layer::Views) {
        out.extend(model.views.iter().map(|p| Prim::Points { pts: vec![at(p)], size: 7.0, color: pal::VIEW }));
    }
    if layers.shown(Layer::Goal)
        && let Some(g) = model.goal
    {
        out.push(Prim::Circle { c: at(&g), r: 7.0, fill: None, stroke: Some((pal::ROBOT_EDGE, 4.0)) });
        out.extend(prim::goal(at(&g)));
    }
    if layers.shown(Layer::Targets) {
        for t in &model.targets {
            let (color, label) = if t.confirmed {
                (pal::TARGET_CONFIRMED, format!("ID {}", t.id))
            } else {
                (pal::TARGET_TENTATIVE, format!("ID {}?", t.id))
            };
            out.extend(prim::target(at(&(t.x, t.y)), 6.0, &label, color));
        }
    }
    if layers.shown(Layer::Scan) && !model.scan.is_empty() {
        out.push(Prim::Points { pts: model.scan.iter().map(at).collect(), size: 3.0, color: pal::SCAN });
    }
    if layers.shown(Layer::Robot)
        && let Some((x, y, yaw)) = model.robot
    {
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
    out
}

/// one frame of the map view in `ctx` (the whole context is the view)
pub fn draw(ctx: &mut dyn DrawingContext, model: &Model, layers: &Layers, cache: &mut Cache, vp: &Viewport, grid_bounds: [f64; 4]) -> AureaResult<()> {
    let map = if layers.shown(Layer::Map) { cache.map(&model.map) } else { None };
    let place = |g: &GridLayer| g.grid.as_ref().map(|m| grid_layer(m, vp));
    if let (Some((img, _)), Some(l)) = (&map, place(&model.map)) {
        tb4_draw::draw_map(ctx, img, &l)?;
    }
    for (layer, g, slot) in [(Layer::GlobalCost, &model.global_cost, &mut cache.global), (Layer::LocalCost, &model.local_cost, &mut cache.local)] {
        if layers.shown(layer)
            && let (Some(img), Some(l)) = (cost_image(slot, g, layers.opacity(layer)), place(g))
        {
            tb4_draw::draw_map(ctx, &img, &l)?;
        }
    }
    if layers.shown(Layer::Grid) {
        tb4_draw::paint_prims(ctx, &grid_lines(grid_bounds, vp))?;
    }
    // walls again, over costmaps and grid
    if let (Some((_, walls)), Some(l)) = (&map, place(&model.map)) {
        tb4_draw::draw_map(ctx, walls, &l)?;
    }
    tb4_draw::paint_prims(ctx, &prims(model, layers, vp))
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
        Layer::Goal => prim::goal(c).to_vec(),
        Layer::Targets => vec![
            Prim::Circle { c: (x - 6.0, y), r: 4.5, fill: Some(pal::TARGET_TENTATIVE), stroke: Some((pal::ROBOT_EDGE, 1.0)) },
            Prim::Circle { c: (x + 6.0, y), r: 4.5, fill: Some(pal::TARGET_CONFIRMED), stroke: Some((pal::ROBOT_EDGE, 1.0)) },
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
