//! The viewer's state around the map canvas: camera (zoom, pan, rotation, follow), layers, the
//! current tool (interact, measure, 2D goal), mouse and key handling, and drawing one frame.
//! Pointer coordinates are canvas pixels. No native widgets here, so it is tested offscreen.

use std::time::{Duration, Instant};

use aurea::AureaResult;
use aurea::prelude::{KeyCode, Modifiers};
use aurea::render::{DrawingContext, Font, Image, Point, Rect};
use tb4_draw::palette as pal;
use tb4_draw::prim::{self, Prim, fill};
use tb4_draw::view::ScreenRect;
use tb4_draw::{Camera, Viewport};

use crate::model::Model;
use crate::scene::{self, Cache, LAYERS, Layer, Layers};

const MARGIN: f64 = 12.0;
const ZOOM_STEP: f64 = 1.2;
const ROTATE_STEP: f64 = std::f64::consts::PI / 12.0;
/// radians per pixel of a right-button drag
const ROTATE_DRAG: f64 = 0.01;
const MESSAGE_FOR: Duration = Duration::from_secs(6);

/// what the status bar says about the mouse and keys
pub const HINTS: &str = "Left-drag: pan or tool   Middle-drag: pan   Right-drag: rotate   Wheel: zoom   F follow   0 fit   [ ] rotate   M measure   G goal   Esc";

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Tool {
    Interact,
    Measure,
    Goal,
}

impl Tool {
    pub fn name(self) -> &'static str {
        match self {
            Tool::Interact => "Interact",
            Tool::Measure => "Measure",
            Tool::Goal => "2D Goal",
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Button {
    Left,
    Middle,
    Right,
}

/// what the window has to do after a key
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Action {
    None,
    Redraw,
    Quit,
    Screenshot,
}

#[derive(Debug, Clone, Copy)]
enum Drag {
    Pan((f64, f64)),
    Rotate(f64),
    Measure((f64, f64)),
    Goal((f64, f64)),
}

/// what the last frame was fitted to, so mouse events use the same viewport
#[derive(Debug, Clone, Copy)]
struct Frame {
    bounds: [f64; 4],
    rect: ScreenRect,
    robot: Option<(f64, f64)>,
}

pub struct App {
    pub camera: Camera,
    pub layers: Layers,
    pub tool: Tool,
    /// the 2D Goal tool may send goals (`--allow-goals`); otherwise it only shows them
    pub allow_goals: bool,
    pub cache: Cache,
    /// show the camera image big over the map
    pub camera_big: bool,
    /// dock sections open: mission, camera, health, layers
    pub sections: [bool; 4],
    /// a goal to send (x, y, yaw), taken by the window
    pub pending_goal: Option<(f64, f64, f64)>,
    /// measured segment, world
    pub measure: Option<((f64, f64), (f64, f64))>,
    /// the goal being dragged or the last one set: position, yaw
    pub goal_pose: Option<((f64, f64), f64)>,
    /// pointer over the map, world
    pub cursor: Option<(f64, f64)>,
    message: Option<(String, Instant)>,
    fit_bounds: Option<[f64; 4]>,
    frame: Option<Frame>,
    drag: Option<Drag>,
    frames: (Instant, u32, f64),
    draw_time: f64,
}

impl App {
    pub fn new(allow_goals: bool) -> Self {
        Self {
            camera: Camera { follow: true, ..Camera::default() },
            layers: Layers::default(),
            tool: Tool::Interact,
            allow_goals,
            cache: Cache::default(),
            camera_big: false,
            sections: [true; 4],
            pending_goal: None,
            measure: None,
            goal_pose: None,
            cursor: None,
            message: None,
            fit_bounds: None,
            frame: None,
            drag: None,
            frames: (Instant::now(), 0, 0.0),
            draw_time: 0.0,
        }
    }

    pub fn say(&mut self, text: impl Into<String>) {
        self.message = Some((text.into(), Instant::now()));
    }

    pub fn message(&self, now: Instant) -> Option<&str> {
        self.message.as_ref().filter(|(_, at)| now.saturating_duration_since(*at) < MESSAGE_FOR).map(|(m, _)| m.as_str())
    }

    pub fn fps(&self) -> f64 {
        self.frames.2
    }

    pub fn viewport(&self) -> Option<(Viewport, ScreenRect)> {
        self.frame.map(|f| (self.camera.viewport(f.bounds, f.rect, MARGIN, f.robot), f.rect))
    }

    /// fit to the first map and keep it, so the view does not jump each time SLAM grows the map
    fn fit_bounds(&mut self, model: &Model) -> [f64; 4] {
        match (self.fit_bounds, &model.map.grid) {
            (Some(b), _) => b,
            (None, Some(_)) => *self.fit_bounds.insert(model.bounds()),
            (None, None) => model.bounds(),
        }
    }

    /// back to the whole map, north up
    pub fn fit(&mut self) {
        self.camera.reset();
        self.fit_bounds = None;
    }

    pub fn set_tool(&mut self, tool: Tool) {
        if tool == Tool::Goal && !self.allow_goals {
            self.say("2D Goal shows the pose only: read-only (start with --allow-goals to send)");
        }
        self.tool = tool;
        self.drag = None;
    }

    pub fn zoom(&mut self, factor: f64) {
        if let Some((vp, rect)) = self.viewport() {
            self.camera.zoom_at(&vp, rect, (rect.0 + rect.2 / 2.0, rect.1 + rect.3 / 2.0), factor);
        }
    }

    pub fn rotate(&mut self, by: f64) {
        if let Some((vp, rect)) = self.viewport() {
            self.camera.rotate(&vp, rect, by);
        }
    }

    pub fn key(&mut self, key: KeyCode, mods: Modifiers) -> Action {
        let layer_keys = [KeyCode::Key1, KeyCode::Key2, KeyCode::Key3, KeyCode::Key4, KeyCode::Key5, KeyCode::Key6, KeyCode::Key7, KeyCode::Key8, KeyCode::Key9];
        if let Some(i) = layer_keys.iter().position(|&k| k == key) {
            self.layers.toggle(LAYERS[i].layer);
            return Action::Redraw;
        }
        match key {
            KeyCode::Q if mods.ctrl => return Action::Quit,
            KeyCode::S if mods.ctrl => return Action::Screenshot,
            KeyCode::P => return Action::Screenshot,
            KeyCode::F => self.camera.follow = !self.camera.follow,
            KeyCode::Key0 | KeyCode::R => self.fit(),
            KeyCode::Equals => self.zoom(ZOOM_STEP),
            KeyCode::Minus => self.zoom(1.0 / ZOOM_STEP),
            KeyCode::LeftBracket => self.rotate(ROTATE_STEP),
            KeyCode::RightBracket => self.rotate(-ROTATE_STEP),
            KeyCode::M => self.set_tool(Tool::Measure),
            KeyCode::G => self.set_tool(Tool::Goal),
            KeyCode::I => self.set_tool(Tool::Interact),
            KeyCode::C => self.camera_big = !self.camera_big,
            KeyCode::Escape => {
                self.set_tool(Tool::Interact);
                self.measure = None;
                self.camera_big = false;
            }
            _ => return Action::None,
        }
        Action::Redraw
    }

    fn world(&self, x: f64, y: f64) -> Option<(f64, f64)> {
        self.viewport().map(|(vp, _)| vp.to_world(x, y))
    }

    /// a button went down at canvas (x, y); true when the view changed
    pub fn press(&mut self, button: Button, x: f64, y: f64) -> bool {
        if self.camera_big {
            self.camera_big = false;
            return true;
        }
        let Some(w) = self.world(x, y) else { return false };
        self.drag = Some(match (button, self.tool) {
            (Button::Right, _) => Drag::Rotate(x),
            (Button::Middle, _) | (Button::Left, Tool::Interact) => Drag::Pan((x, y)),
            (Button::Left, Tool::Measure) => {
                self.measure = Some((w, w));
                Drag::Measure(w)
            }
            (Button::Left, Tool::Goal) => {
                self.goal_pose = Some((w, 0.0));
                Drag::Goal(w)
            }
        });
        true
    }

    /// the pointer moved to canvas (x, y); true when the view changed
    pub fn moved(&mut self, x: f64, y: f64) -> bool {
        self.cursor = self.world(x, y);
        let (Some(d), Some((vp, rect))) = (self.drag, self.viewport()) else { return false };
        let w = vp.to_world(x, y);
        match d {
            Drag::Pan((px, py)) => {
                self.camera.pan(&vp, rect, x - px, y - py);
                self.drag = Some(Drag::Pan((x, y)));
            }
            Drag::Rotate(px) => {
                self.camera.rotate(&vp, rect, (x - px) * ROTATE_DRAG);
                self.drag = Some(Drag::Rotate(x));
            }
            Drag::Measure(from) => self.measure = Some((from, w)),
            Drag::Goal(from) => self.goal_pose = Some((from, (w.1 - from.1).atan2(w.0 - from.0))),
        }
        true
    }

    /// the button came up; a finished 2D goal becomes `pending_goal` when goals are allowed
    pub fn release(&mut self, x: f64, y: f64) -> bool {
        let Some(d) = self.drag else { return false };
        // the end point first, while the drag is still on
        self.moved(x, y);
        self.drag = None;
        if let (Drag::Goal(_), Some(((gx, gy), yaw))) = (d, self.goal_pose) {
            if self.allow_goals {
                self.pending_goal = Some((gx, gy, yaw));
                self.say(format!("goal sent: ({gx:.2}, {gy:.2}) {:.0} deg", yaw.to_degrees()));
            } else {
                self.say("read-only: goal not sent (start with --allow-goals)");
            }
        }
        true
    }

    /// wheel at canvas (x, y), positive `dy` = away from the user = zoom in
    pub fn wheel(&mut self, dy: f64, x: f64, y: f64) -> bool {
        let Some((vp, rect)) = self.viewport() else { return false };
        if dy == 0.0 {
            return false;
        }
        self.camera.zoom_at(&vp, rect, (x, y), ZOOM_STEP.powf(dy.clamp(-5.0, 5.0)));
        true
    }

    /// the whole canvas: map view, tool overlays, camera overlay, scale and hints
    pub fn draw(&mut self, ctx: &mut dyn DrawingContext, model: &Model, now: Instant) -> AureaResult<()> {
        let (w, h) = (ctx.width() as f32, ctx.height() as f32);
        self.draw_in(ctx, model, now, Rect::new(0.0, 0.0, w, h))
    }

    /// the map view in `area` of a larger canvas; pointer positions given to this `App` are then
    /// relative to the area's top left
    pub fn draw_in(&mut self, ctx: &mut dyn DrawingContext, model: &Model, now: Instant, area: Rect) -> AureaResult<()> {
        let started = Instant::now();
        ctx.save()?;
        ctx.clip_rect(area)?;
        ctx.translate(area.x, area.y)?;
        let out = self.draw_local(ctx, model, now, area.width, area.height);
        ctx.restore()?;
        self.draw_time = 0.8 * self.draw_time + 0.2 * started.elapsed().as_secs_f64();
        out
    }

    /// seconds a map frame takes to draw (smoothed)
    pub fn draw_time(&self) -> f64 {
        self.draw_time
    }

    fn draw_local(&mut self, ctx: &mut dyn DrawingContext, model: &Model, now: Instant, w: f32, h: f32) -> AureaResult<()> {
        let rect = (0.0, 0.0, f64::from(w), f64::from(h));
        ctx.draw_rect(Rect::new(0.0, 0.0, w, h), &fill(pal::BACKGROUND))?;
        let bounds = self.fit_bounds(model);
        self.frame = Some(Frame { bounds, rect, robot: model.robot.map(|r| (r.0, r.1)) });
        let (vp, _) = self.viewport().expect("frame just set");
        scene::draw(ctx, model, &self.layers, &mut self.cache, &vp, model.bounds())?;
        tb4_draw::paint_prims(ctx, &self.tool_prims(&vp))?;
        if let Some((a, b)) = self.measure {
            let (d, ang) = ((b.0 - a.0).hypot(b.1 - a.1), (b.1 - a.1).atan2(b.0 - a.0).to_degrees());
            let m = vp.to_screen((a.0 + b.0) / 2.0, (a.1 + b.1) / 2.0);
            tb4_draw::paint_prims(ctx, &prim::outlined_text((m.0 + 8.0, m.1 - 8.0), &format!("{d:.2} m  {ang:.0} deg"), pal::MEASURE, 15.0))?;
        }
        scene::chrome(ctx, &vp, w, h)?;
        self.hints(ctx, model, now)?;
        if self.camera_big {
            self.camera_overlay(ctx, model, w, h)?;
        }
        self.count_frame(now);
        Ok(())
    }

    fn tool_prims(&self, vp: &Viewport) -> Vec<Prim> {
        let mut out = Vec::new();
        if let Some((a, b)) = self.measure {
            let (sa, sb) = (vp.to_screen(a.0, a.1), vp.to_screen(b.0, b.1));
            out.push(Prim::Line { a: sa, b: sb, color: pal::MEASURE, width: 2.0, dashed: true });
            out.push(prim::dot(sa, 3.5, pal::MEASURE));
            out.push(prim::dot(sb, 3.5, pal::MEASURE));
        }
        if let Some(((x, y), yaw)) = self.goal_pose {
            let c = vp.to_screen(x, y);
            let h = vp.heading(yaw);
            let l = 34.0f32;
            let tip = (c.0 + l * h.cos() as f32, c.1 - l * h.sin() as f32);
            let wing = |a: f64| (tip.0 - 10.0 * (h + a).cos() as f32, tip.1 + 10.0 * (h + a).sin() as f32);
            let color = if self.allow_goals { pal::GOAL } else { pal::TEXT_DIM };
            out.push(Prim::Circle { c, r: 5.0, fill: None, stroke: Some((color, 2.0)) });
            out.push(Prim::Polyline { pts: vec![c, tip], color, width: 3.0 });
            out.push(Prim::Polyline { pts: vec![wing(0.45), tip, wing(-0.45)], color, width: 3.0 });
        }
        out
    }

    fn hints(&self, ctx: &mut dyn DrawingContext, model: &Model, now: Instant) -> AureaResult<()> {
        let font = Font::monospace(13.0);
        let mut lines: Vec<(String, [u8; 4])> = Vec::new();
        if model.map.grid.is_none() {
            lines.push(("waiting for /map".into(), pal::TEXT_DIM));
        }
        if model.robot.is_none() {
            lines.push(("no TF map -> base_link yet".into(), pal::TEXT_DIM));
        }
        if let Some((p, at)) = &model.tf_problem
            && now.saturating_duration_since(*at) < Duration::from_secs(3)
        {
            lines.push((p.clone(), pal::TEXT_DIM));
        }
        if let Some(m) = self.message(now) {
            lines.push((m.to_owned(), pal::TEXT));
        }
        for (i, (t, c)) in lines.iter().enumerate() {
            tb4_draw::paint_prims(ctx, &prim::outlined_text((14.0, 22.0 + 18.0 * i as f32), t, *c, 13.0))?;
        }
        let _ = font;
        Ok(())
    }

    fn camera_overlay(&self, ctx: &mut dyn DrawingContext, model: &Model, w: f32, h: f32) -> AureaResult<()> {
        ctx.draw_rect(Rect::new(0.0, 0.0, w, h), &fill([0, 0, 0, 170]))?;
        let font = Font::monospace(14.0);
        let Some(f) = &model.camera else {
            return ctx.draw_text_with_font("no camera image yet (/team7/aruco/debug)", Point::new(24.0, 40.0), &font, &fill(pal::TEXT));
        };
        let s = ((w - 48.0) / f.width as f32).min((h - 72.0) / f.height as f32).max(0.1);
        let (iw, ih) = (f.width as f32 * s, f.height as f32 * s);
        let (x, y) = ((w - iw) / 2.0, (h - ih) / 2.0);
        ctx.draw_image_rect(&Image::new(f.width, f.height, f.rgba.clone()).with_opaque(true), Rect::new(x, y, iw, ih))?;
        ctx.draw_text_with_font("ArUco debug (click or Esc to close)", Point::new(x, y - 10.0), &font, &fill(pal::TEXT))
    }

    fn count_frame(&mut self, now: Instant) {
        let (since, n, _) = &mut self.frames;
        *n += 1;
        let dt = now.saturating_duration_since(*since).as_secs_f64();
        if dt >= 1.0 {
            self.frames.2 = f64::from(self.frames.1) / dt;
            self.frames = (now, 0, self.frames.2);
        }
    }

    /// the status bar line (the mouse and key hints are `HINTS`, on a line of their own)
    pub fn status(&self, model: &Model, now: Instant, wall: f64, domain: &str) -> String {
        let (t, sim) = model.ros_now(now, wall);
        let time = if sim { format!("sim {t:.1} s") } else { "wall clock".to_owned() };
        let cursor = self.cursor.map_or_else(String::new, |(x, y)| format!("   x {x:.2} y {y:.2}"));
        let goals = if self.allow_goals { "goals ON" } else { "read-only" };
        format!("{time}   {:.0} fps   ROS_DOMAIN_ID {domain}   frame map{cursor}   {}   {goals}", self.fps(), self.tool.name())
    }

    pub fn shown(&self, l: Layer) -> bool {
        self.layers.shown(l)
    }
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;
    use crate::model::{CameraFrame, GridLayer, MapData, Shape, ShapeKind, Target, Topic};
    use aurea::render::{CpuRasterizer, Renderer};
    use tb4_core::GridMeta;
    use tb4_draw::Rgba;

    pub(crate) fn rgb(p: u32) -> [u8; 3] {
        [(p >> 16) as u8, (p >> 8) as u8, p as u8]
    }

    pub(crate) fn near(p: u32, c: Rgba, tol: i32) -> bool {
        rgb(p).iter().zip(c).all(|(&a, b)| (i32::from(a) - i32::from(b)).abs() <= tol)
    }

    fn grid(ox: f64, oy: f64, w: u32, h: u32, cell: impl Fn(f64, f64) -> i8) -> MapData {
        let meta = GridMeta { width: w, height: h, resolution: 0.05, origin_x: ox, origin_y: oy };
        let mut data = Vec::with_capacity((w * h) as usize);
        for r in 0..h {
            for c in 0..w {
                data.push(cell(ox + (f64::from(c) + 0.5) * 0.05, oy + (f64::from(r) + 0.5) * 0.05));
            }
        }
        MapData { meta, data, origin_yaw: 0.0 }
    }

    /// a 6 x 4 m room with walls, an inner wall at x 2.0..2.1, unknown east strip, costmaps
    pub(crate) fn scene() -> Model {
        let wall = |x: f64, y: f64| !(-0.9..=4.9).contains(&x) || !(-0.9..=2.9).contains(&y) || ((2.0..=2.1).contains(&x) && y < 1.0);
        let mut m = Model::default();
        m.map.set(grid(-1.0, -1.0, 120, 80, |x, y| if wall(x, y) { 100 } else if x > 4.0 { -1 } else { 0 }));
        // global costmap: cost falls off with distance to the inner wall
        m.global_cost.set(grid(-1.0, -1.0, 120, 80, |x, y| {
            let d = (x - 2.05).abs();
            if wall(x, y) { 100 } else if d < 0.6 && y < 1.2 { (98.0 * (1.0 - d / 0.6)) as i8 } else { 0 }
        }));
        m.local_cost.set(grid(0.0, 0.0, 40, 40, |x, y| if (x - 1.0).hypot(y - 1.0) < 0.9 { 40 } else { -1 }));
        m.set_robot(1.0, 1.0, std::f64::consts::FRAC_PI_2);
        m.path = vec![(1.0, 1.0), (3.0, 1.0), (3.0, 2.0)];
        m.trajectory = vec![(1.0, 1.2), (1.0, 2.0)];
        m.scan = (0..90).map(|i| (-0.8 + 0.06 * f64::from(i), 2.8)).collect();
        m.frontiers = vec![(3.9, 0.5)];
        m.views = vec![(2.6, 2.4)];
        m.goal = Some((3.0, 2.0));
        m.targets = vec![
            Target { id: 7, x: 0.0, y: 2.0, confirmed: true, views: None },
            Target { id: 3, x: 2.5, y: -0.5, confirmed: false, views: Some(2) },
        ];
        m.slam_graph = vec![Shape { kind: ShapeKind::Strip, pts: vec![(-0.5, -0.5), (-0.5, 0.5)], color: pal::SLAM_GRAPH }];
        m.set_mission("RUNNING".into(), Instant::now());
        m.tick(Topic::Scan, Instant::now());
        m
    }

    pub(crate) fn render(app: &mut App, m: &Model, w: u32, h: u32) -> Vec<u32> {
        let mut r = CpuRasterizer::new(w, h);
        {
            let mut ctx = r.begin_frame().unwrap();
            app.draw(ctx.as_mut(), m, Instant::now()).unwrap();
        }
        r.end_frame().unwrap();
        r.pixels().to_vec()
    }

    /// TEAM7_VIZ_SNAPSHOT=dir cargo test: writes the test frames for a look
    pub(crate) fn snapshot(px: &[u32], w: u32, h: u32, name: &str) {
        let Some(dir) = std::env::var_os("TEAM7_VIZ_SNAPSHOT") else { return };
        let rgba: Vec<u8> = px.iter().flat_map(|p| [(p >> 16) as u8, (p >> 8) as u8, *p as u8, 255]).collect();
        let file = std::fs::File::create(std::path::Path::new(&dir).join(name)).unwrap();
        let mut enc = png::Encoder::new(std::io::BufWriter::new(file), w, h);
        enc.set_color(png::ColorType::Rgba);
        enc.set_depth(png::BitDepth::Eight);
        enc.write_header().unwrap().write_image_data(&rgba).unwrap();
    }

    fn fixed() -> App {
        let mut a = App::new(false);
        a.camera.follow = false;
        a
    }

    #[test]
    fn layers_land_where_they_belong_and_walls_stay_on_top() {
        let mut m = scene();
        let mut app = fixed();
        app.layers.toggle(Layer::GlobalCost); // off by default
        app.layers.opacity[scene::index(Layer::GlobalCost)] = 1.0;
        let (w, h) = (900u32, 600u32);
        let px = render(&mut app, &m, w, h);
        snapshot(&px, w, h, "frame_layers.png");
        let (vp, _) = app.viewport().unwrap();
        let at = |px: &[u32], x: f64, y: f64| {
            let (sx, sy) = vp.to_screen(x, y);
            px[sy as usize * w as usize + sx as usize]
        };
        // the inner wall stays black under a full costmap; next to it the costmap tints the floor
        assert!(near(at(&px, 2.05, 0.4), pal::OCCUPIED, 2), "wall {:?}", rgb(at(&px, 2.05, 0.4)));
        let tinted = at(&px, 1.75, 0.4);
        assert!(!near(tinted, pal::FREE, 20), "costmap tint {:?}", rgb(tinted));
        assert!(near(at(&px, 4.4, 1.5), pal::UNKNOWN, 2));
        assert!(near(at(&px, 2.0, 1.0), pal::PATH, 12), "path {:?}", rgb(at(&px, 2.0, 1.0)));
        assert!(near(at(&px, 1.0, 1.6), pal::TRAJECTORY, 12));
        assert!(near(at(&px, 1.0, 2.8), pal::SCAN, 12), "scan {:?}", rgb(at(&px, 1.0, 2.8)));
        assert!(near(at(&px, 3.9, 0.5), pal::FRONTIER, 12));
        assert!(near(at(&px, 2.6, 2.4), pal::VIEW, 12));
        assert!(near(at(&px, 0.0, 2.0), pal::TARGET_CONFIRMED, 12));
        assert!(near(at(&px, 2.5, -0.5), pal::TARGET_TENTATIVE, 12));
        let g = rgb(at(&px, -0.5, 0.0));
        assert!(g[0] > 200 && g[2] > 150 && g[1] < 190, "graph {g:?}");
        // robot faces north: its x axis (red) points up, the y axis (green) to the left
        assert!(near(at(&px, 1.0, 1.25), pal::AXIS_X, 40), "x axis {:?}", rgb(at(&px, 1.0, 1.25)));
        assert!(near(at(&px, 0.75, 1.0), pal::AXIS_Y, 40), "y axis {:?}", rgb(at(&px, 0.75, 1.0)));

        // both costmaps start off; the local cost at its default opacity tints, but faintly
        app.layers.toggle(Layer::GlobalCost);
        assert!(!App::new(false).shown(Layer::GlobalCost) && !App::new(false).shown(Layer::LocalCost));
        app.layers.toggle(Layer::LocalCost);
        let px = render(&mut app, &m, w, h);
        let disc = at(&px, 1.4, 0.6);
        assert!(!near(disc, pal::FREE, 8) && near(disc, pal::FREE, 120), "faint disc {:?}", rgb(disc));
        // key 9 (local cost) hides it
        app.key(KeyCode::Key9, Modifiers::default());
        let px = render(&mut app, &m, w, h);
        assert!(near(at(&px, 1.4, 0.6), pal::FREE, 2));

        // camera overlay shows the image over the map
        m.camera = Some(CameraFrame { width: 4, height: 3, rgba: [10, 200, 30, 255].repeat(12), seq: 1, at: Instant::now() });
        app.key(KeyCode::C, Modifiers::default());
        let px = render(&mut app, &m, w, h);
        snapshot(&px, w, h, "frame_camera.png");
        assert!(near(px[(h / 2 * w + w / 2) as usize], [10, 200, 30, 255], 3));
        assert!(app.press(Button::Left, 10.0, 10.0));
        assert!(!app.camera_big);
        let _ = GridLayer::default();
    }

    #[test]
    fn rotated_view_keeps_layers_together() {
        let m = scene();
        let mut app = fixed();
        let (w, h) = (700u32, 700u32);
        render(&mut app, &m, w, h);
        app.rotate(0.6);
        let px = render(&mut app, &m, w, h);
        snapshot(&px, w, h, "frame_rotated.png");
        let (vp, _) = app.viewport().unwrap();
        assert!((vp.rot - 0.6).abs() < 1e-9);
        let at = |x: f64, y: f64| {
            let (sx, sy) = vp.to_screen(x, y);
            px[sy as usize * w as usize + sx as usize]
        };
        assert!(near(at(2.05, 0.3), pal::OCCUPIED, 2), "wall {:?}", rgb(at(2.05, 0.3)));
        assert!(near(at(1.0, 2.8), pal::SCAN, 12), "scan {:?}", rgb(at(1.0, 2.8)));
        assert!(near(at(2.0, 1.0), pal::PATH, 20), "path {:?}", rgb(at(2.0, 1.0)));
        app.key(KeyCode::Key0, Modifiers::default());
        render(&mut app, &m, w, h);
        assert_eq!(app.viewport().unwrap().0.rot, 0.0);
    }

    #[test]
    fn measure_and_goal_tools() {
        let m = scene();
        let mut app = fixed();
        let (w, h) = (800u32, 600u32);
        render(&mut app, &m, w, h);
        let (vp, _) = app.viewport().unwrap();
        let s = |x: f64, y: f64| {
            let p = vp.to_screen(x, y);
            (f64::from(p.0), f64::from(p.1))
        };

        app.key(KeyCode::M, Modifiers::default());
        let (a, b) = (s(0.0, 0.0), s(3.0, 4.0));
        app.press(Button::Left, a.0, a.1);
        // no motion events in between: the release point still ends the segment
        app.release(b.0, b.1);
        let (p, q) = app.measure.unwrap();
        assert!(((q.0 - p.0).hypot(q.1 - p.1) - 5.0).abs() < 0.02);
        let px = render(&mut app, &m, w, h);
        snapshot(&px, w, h, "frame_measure.png");
        let mid = s(1.5, 2.0);
        // the dashed line or its label are drawn around the middle
        let cyan = (-6..=6).flat_map(|dy| (-6..=6).map(move |dx| (dx, dy))).any(|(dx, dy)| near(px[((mid.1 as i32 + dy) * w as i32 + mid.0 as i32 + dx) as usize], pal::MEASURE, 40));
        assert!(cyan);

        // 2D goal, read-only: shown, not sent
        app.key(KeyCode::G, Modifiers::default());
        let (g, t) = (s(1.0, 1.0), s(1.0, 2.0));
        app.press(Button::Left, g.0, g.1);
        app.moved(t.0, t.1);
        app.release(t.0, t.1);
        assert!(app.pending_goal.is_none());
        assert!(app.message(Instant::now()).unwrap().contains("read-only"));
        let ((gx, gy), yaw) = app.goal_pose.unwrap();
        assert!((gx - 1.0).abs() < 0.02 && (gy - 1.0).abs() < 0.02 && (yaw - std::f64::consts::FRAC_PI_2).abs() < 0.05);

        // allowed: the goal is handed to the window to send
        app.allow_goals = true;
        app.press(Button::Left, g.0, g.1);
        app.moved(t.0, t.1);
        app.release(t.0, t.1);
        let (x, y, yaw) = app.pending_goal.unwrap();
        assert!((x - 1.0).abs() < 0.02 && (y - 1.0).abs() < 0.02 && (yaw - std::f64::consts::FRAC_PI_2).abs() < 0.05);
    }

    #[test]
    fn mouse_and_keys_drive_the_camera() {
        let m = scene();
        let mut app = App::new(false);
        let (w, h) = (800u32, 600u32);
        render(&mut app, &m, w, h);
        // following: the robot is in the middle
        let (vp, _) = app.viewport().unwrap();
        assert_eq!(vp.to_screen(1.0, 1.0), (400.0, 300.0));
        // right drag rotates
        app.press(Button::Right, 400.0, 300.0);
        app.moved(450.0, 300.0);
        app.release(450.0, 300.0);
        assert!((app.camera.rotation - 0.5).abs() < 1e-9);
        // middle drag pans and stops following
        app.press(Button::Middle, 100.0, 100.0);
        app.moved(130.0, 100.0);
        app.release(130.0, 100.0);
        assert!(!app.camera.follow);
        assert!(app.wheel(1.0, 200.0, 200.0));
        assert!((app.camera.zoom - ZOOM_STEP).abs() < 1e-9);
        assert_eq!(app.key(KeyCode::F, Modifiers::default()), Action::Redraw);
        assert!(app.camera.follow);
        assert_eq!(app.key(KeyCode::P, Modifiers::default()), Action::Screenshot);
        assert_eq!(app.key(KeyCode::Q, Modifiers::default()), Action::None);
        assert_eq!(app.key(KeyCode::Q, Modifiers { ctrl: true, ..Modifiers::default() }), Action::Quit);
        let status = app.status(&m, Instant::now(), 0.0, "79");
        assert!(status.contains("ROS_DOMAIN_ID 79") && status.contains("read-only") && HINTS.contains("Right-drag: rotate"));
    }
}
