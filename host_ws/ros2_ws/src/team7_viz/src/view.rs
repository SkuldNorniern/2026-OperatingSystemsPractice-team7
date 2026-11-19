//! One frame of the viewer: the map view on the left, the side panel on the right, plus keyboard and
//! mouse handling. Works on any `DrawingContext` (the window canvas or an offscreen `CpuRasterizer`).

use std::time::Instant;

use aurea::AureaResult;
use aurea::prelude::{KeyCode, MouseButton, WindowEvent};
use aurea::render::{DrawingContext, Font, Image, Paint, Point, Rect};
use tb4_draw::palette as pal;
use tb4_draw::prim::{self, Prim, fill, stroke};
use tb4_draw::view::ScreenRect;
use tb4_draw::{Camera, Rgba, Viewport};

use crate::model::{Model, ROBOT_RADIUS, Topic};

pub const PANEL_WIDTH: f32 = 320.0;
const MARGIN: f64 = 12.0;
const ZOOM_STEP: f64 = 1.2;
/// at most this many targets listed per state in the panel
const LIST_MAX: usize = 6;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Layer {
    Map,
    Scan,
    Path,
    Trajectory,
    Frontiers,
    Goal,
    Targets,
}

/// layer, toggle key label, legend text
pub const LAYERS: [(Layer, &str, &str); 7] = [
    (Layer::Map, "1", "map free / wall / unknown"),
    (Layer::Scan, "2", "laser scan"),
    (Layer::Path, "3", "global path"),
    (Layer::Trajectory, "4", "MPPI trajectory"),
    (Layer::Frontiers, "5", "frontier / camera view"),
    (Layer::Goal, "6", "active goal"),
    (Layer::Targets, "7", "target tentative / confirmed"),
];

/// the legend symbol of a layer (None: the robot row), drawn as on the map, centred at `c`
pub fn legend_symbol(layer: Option<Layer>, c: (f32, f32)) -> Vec<Prim> {
    let (x, y) = c;
    let square = |dx: f32, color: Rgba| Prim::Points { pts: vec![(x + dx, y)], size: 9.0, color };
    let line = |color: Rgba| Prim::Line { a: (x - 12.0, y + 3.0), b: (x + 12.0, y - 3.0), color, width: 2.0, dashed: false };
    match layer {
        None => prim::robot(c, 7.0, 0.5).to_vec(),
        Some(Layer::Map) => vec![square(-11.0, pal::FREE), square(0.0, pal::OCCUPIED), square(11.0, pal::UNKNOWN)],
        Some(Layer::Scan) => vec![Prim::Points { pts: vec![(x - 8.0, y + 2.0), (x - 3.0, y), (x + 2.0, y - 1.0), (x + 7.0, y - 1.0)], size: 3.0, color: pal::SCAN }],
        Some(Layer::Path) => vec![line(pal::PATH)],
        Some(Layer::Trajectory) => vec![line(pal::TRAJECTORY)],
        Some(Layer::Frontiers) => vec![prim::dot((x - 6.0, y), 4.0, pal::FRONTIER), prim::dot((x + 6.0, y), 4.0, pal::VIEW)],
        Some(Layer::Goal) => prim::goal(c).to_vec(),
        Some(Layer::Targets) => vec![
            Prim::Circle { c: (x - 7.0, y), r: 5.0, fill: Some(pal::TARGET_TENTATIVE), stroke: Some((pal::ROBOT_EDGE, 1.5)) },
            Prim::Circle { c: (x + 7.0, y), r: 5.0, fill: Some(pal::TARGET_CONFIRMED), stroke: Some((pal::ROBOT_EDGE, 1.5)) },
        ],
    }
}

/// what the last frame was fitted to, so mouse events use the same viewport
#[derive(Debug, Clone, Copy)]
struct Frame {
    bounds: [f64; 4],
    rect: ScreenRect,
    robot: Option<(f64, f64)>,
}

pub struct Viewer {
    pub camera: Camera,
    pub layers: [bool; 7],
    map_image: Option<(u64, Image)>,
    frame: Option<Frame>,
    drag: Option<(f64, f64)>,
}

impl Default for Viewer {
    fn default() -> Self {
        Self::new()
    }
}

impl Viewer {
    pub fn new() -> Self {
        Self { camera: Camera { follow: true, ..Camera::default() }, layers: [true; 7], map_image: None, frame: None, drag: None }
    }

    pub fn shown(&self, l: Layer) -> bool {
        self.layers[l as usize]
    }

    fn viewport(&self) -> Option<(Viewport, ScreenRect)> {
        self.frame.map(|f| (self.camera.viewport(f.bounds, f.rect, MARGIN, f.robot), f.rect))
    }

    /// keyboard: true when the window should close
    pub fn key(&mut self, key: KeyCode) -> bool {
        let layer = match key {
            KeyCode::Key1 => Some(0),
            KeyCode::Key2 => Some(1),
            KeyCode::Key3 => Some(2),
            KeyCode::Key4 => Some(3),
            KeyCode::Key5 => Some(4),
            KeyCode::Key6 => Some(5),
            KeyCode::Key7 => Some(6),
            _ => None,
        };
        if let Some(i) = layer {
            self.layers[i] = !self.layers[i];
            return false;
        }
        match key {
            KeyCode::F => self.camera.follow = !self.camera.follow,
            KeyCode::Key0 | KeyCode::R => self.camera.reset(),
            KeyCode::Equals | KeyCode::Minus => {
                let f = if key == KeyCode::Equals { ZOOM_STEP } else { 1.0 / ZOOM_STEP };
                if let Some((vp, rect)) = self.viewport() {
                    self.camera.zoom_at(&vp, rect, (rect.0 + rect.2 / 2.0, rect.1 + rect.3 / 2.0), f);
                }
            }
            KeyCode::Q | KeyCode::Escape => return true,
            _ => {}
        }
        false
    }

    /// mouse: wheel zooms at the pointer, left drag pans; true when the view changed
    pub fn pointer(&mut self, event: &WindowEvent) -> bool {
        let Some((vp, rect)) = self.viewport() else { return false };
        let inside = |x: f64, y: f64| x >= rect.0 && x < rect.0 + rect.2 && y >= rect.1 && y < rect.1 + rect.3;
        match *event {
            WindowEvent::MouseWheel { delta_y, x, y, .. } if inside(x, y) && delta_y != 0.0 => {
                // positive delta_y is the wheel turned away from the user: zoom in
                self.camera.zoom_at(&vp, rect, (x, y), ZOOM_STEP.powf(delta_y.clamp(-5.0, 5.0)));
                true
            }
            WindowEvent::MouseButton { button: MouseButton::Left, pressed, x, y, .. } => {
                self.drag = (pressed && inside(x, y)).then_some((x, y));
                false
            }
            WindowEvent::MouseMove { x, y, .. } => {
                let Some((px, py)) = self.drag else { return false };
                self.camera.pan(&vp, rect, x - px, y - py);
                self.drag = Some((x, y));
                true
            }
            _ => false,
        }
    }

    fn map_image(&mut self, model: &Model) -> Option<Image> {
        let m = model.map.as_ref()?;
        if self.map_image.as_ref().is_none_or(|(seq, _)| *seq != model.map_seq) {
            self.map_image = tb4_draw::map_image(&m.meta, &m.data).map(|img| (model.map_seq, img));
        }
        self.map_image.as_ref().map(|(_, i)| i.clone())
    }

    pub fn draw(&mut self, ctx: &mut dyn DrawingContext, model: &Model, now: Instant) -> AureaResult<()> {
        let (w, h) = (ctx.width() as f32, ctx.height() as f32);
        ctx.clear(prim::color(pal::BACKGROUND))?;
        let map_w = (w - PANEL_WIDTH).max(100.0);
        let rect = (0.0, 0.0, f64::from(map_w), f64::from(h));
        let robot = model.robot.map(|r| (r.0, r.1));
        self.frame = Some(Frame { bounds: model.bounds(), rect, robot });
        let (vp, _) = self.viewport().expect("frame just set");

        ctx.save()?;
        ctx.clip_rect(Rect::new(0.0, 0.0, map_w, h))?;
        if self.shown(Layer::Map)
            && let (Some(m), Some(img)) = (model.map.as_ref(), self.map_image(model))
        {
            tb4_draw::draw_map(ctx, &img, &tb4_draw::map_layer(&m.meta, (0.0, 0.0, 0.0), &vp))?;
        }
        tb4_draw::paint_prims(ctx, &self.scene(model, &vp))?;
        self.overlay(ctx, model, &vp, h, now)?;
        ctx.restore()?;

        self.panel(ctx, model, now, map_w, w, h)
    }

    /// everything over the map, in drawing order
    pub fn scene(&self, model: &Model, vp: &Viewport) -> Vec<Prim> {
        let at = |&(x, y): &(f64, f64)| vp.to_screen(x, y);
        let mut out = Vec::new();
        if self.shown(Layer::Scan) && !model.scan.is_empty() {
            out.push(Prim::Points { pts: model.scan.iter().map(at).collect(), size: 3.0, color: pal::SCAN });
        }
        if self.shown(Layer::Path) && model.path.len() >= 2 {
            out.push(Prim::Polyline { pts: model.path.iter().map(at).collect(), color: pal::PATH, width: 2.0 });
        }
        if self.shown(Layer::Trajectory) && model.trajectory.len() >= 2 {
            out.push(Prim::Polyline { pts: model.trajectory.iter().map(at).collect(), color: pal::TRAJECTORY, width: 2.0 });
        }
        if self.shown(Layer::Frontiers) {
            out.extend(model.frontiers.iter().map(|p| prim::dot(at(p), 4.0, pal::FRONTIER)));
            out.extend(model.views.iter().map(|p| prim::dot(at(p), 4.0, pal::VIEW)));
        }
        if self.shown(Layer::Goal)
            && let Some(g) = model.goal
        {
            // dark under-ring: the yellow ring alone is faint on the free (light) map
            out.push(Prim::Circle { c: at(&g), r: 7.0, fill: None, stroke: Some((pal::ROBOT_EDGE, 4.0)) });
            out.extend(prim::goal(at(&g)));
        }
        if self.shown(Layer::Targets) {
            for t in &model.targets {
                let (color, label) = if t.confirmed {
                    (pal::TARGET_CONFIRMED, format!("ID {}", t.id))
                } else {
                    (pal::TARGET_TENTATIVE, format!("ID {}?", t.id))
                };
                out.extend(prim::target(at(&(t.x, t.y)), 6.0, &label, color));
            }
        }
        if let Some((x, y, yaw)) = model.robot {
            out.extend(prim::robot(vp.to_screen(x, y), vp.len(ROBOT_RADIUS).max(3.0), yaw));
        }
        out
    }

    /// scale bar and hints in the map view
    fn overlay(&self, ctx: &mut dyn DrawingContext, model: &Model, vp: &Viewport, h: f32, now: Instant) -> AureaResult<()> {
        let font = Font::monospace(13.0);
        let dim = fill(pal::TEXT_DIM);
        // scale bar: the largest of 0.1 / 0.5 / 1 / 2 / 5 m that is under 160 px
        let metres = [5.0, 2.0, 1.0, 0.5, 0.1].into_iter().find(|&m| vp.len(m) <= 160.0).unwrap_or(0.1);
        let len = vp.len(metres);
        let (x, y) = (16.0, h - 18.0);
        ctx.draw_path(&prim::polyline(&[(x, y - 5.0), (x, y), (x + len, y), (x + len, y - 5.0)]), &stroke(pal::TEXT, 2.0))?;
        ctx.draw_text_with_font(&format!("{metres} m"), Point::new(x + len + 8.0, y + 1.0), &font, &fill(pal::TEXT))?;

        let mut hints = Vec::new();
        if model.map.is_none() {
            hints.push("waiting for /map".to_owned());
        }
        if model.robot.is_none() {
            hints.push("no TF map -> base_link yet".to_owned());
        }
        // a TF miss is shown for a few seconds (the first scans often arrive before /tf)
        if let Some((p, at)) = &model.tf_problem
            && now.saturating_duration_since(*at) < std::time::Duration::from_secs(3)
        {
            hints.push(p.clone());
        }
        for (i, t) in hints.iter().enumerate() {
            ctx.draw_text_with_font(t, Point::new(16.0, 24.0 + 18.0 * i as f32), &font, &dim)?;
        }
        Ok(())
    }

    fn panel(&self, ctx: &mut dyn DrawingContext, model: &Model, now: Instant, px: f32, w: f32, h: f32) -> AureaResult<()> {
        ctx.draw_rect(Rect::new(px, 0.0, w - px, h), &fill(pal::PANEL))?;
        let font = Font::monospace(13.0);
        let title = Font::monospace(16.0);
        let text = fill(pal::TEXT);
        let dim = fill(pal::TEXT_DIM);
        let x = px + 16.0;
        let mut y = 30.0;

        line(ctx, x, &mut y, "team7 viewer", &text, &title, 28.0)?;
        let mission = model.mission.as_deref().unwrap_or("-");
        line(ctx, x, &mut y, &format!("mission  {mission}"), &text, &font, 18.0)?;
        let follow = if self.camera.follow { "on" } else { "off" };
        line(ctx, x, &mut y, &format!("follow   {follow}   zoom {:.1}x", self.camera.zoom), &text, &font, 26.0)?;

        for confirmed in [true, false] {
            let n = model.targets(confirmed).count();
            let head = if confirmed { "confirmed" } else { "tentative" };
            line(ctx, x, &mut y, &format!("targets {head} ({n})"), &dim, &font, 18.0)?;
            for t in model.targets(confirmed).take(LIST_MAX) {
                let id = format!("{}{}", t.id, if confirmed { "" } else { "?" });
                line(ctx, x, &mut y, &format!("  ID {id:<5} ({:6.2}, {:6.2})", t.x, t.y), &text, &font, 18.0)?;
            }
            if n > LIST_MAX {
                line(ctx, x, &mut y, &format!("  +{} more", n - LIST_MAX), &dim, &font, 18.0)?;
            }
            y += 8.0;
        }

        line(ctx, x, &mut y, "topic rates", &dim, &font, 18.0)?;
        for t in Topic::ALL {
            let hz = model.rate(t).hz(now);
            let paint = if hz > 0.0 { &text } else { &dim };
            line(ctx, x, &mut y, &format!("  {:<6} {:6.1} Hz", t.name(), hz), paint, &font, 18.0)?;
        }
        y += 8.0;

        // legend and layer switches in one list: key, the symbol as drawn on the map, name (dim when off)
        line(ctx, x, &mut y, "legend   (key toggles layer)", &dim, &font, 22.0)?;
        let rows = std::iter::once((None, "", "robot (TF base_link)")).chain(LAYERS.iter().map(|&(l, k, n)| (Some(l), k, n)));
        for (layer, key, name) in rows {
            if y > h - 52.0 {
                break;
            }
            let on = layer.is_none_or(|l| self.shown(l));
            if on {
                tb4_draw::paint_prims(ctx, &legend_symbol(layer, (x + 40.0, y - 5.0)))?;
            } else {
                ctx.draw_rect(Rect::new(x + 35.0, y - 10.0, 10.0, 10.0), &stroke(pal::TEXT_DIM, 1.0))?;
            }
            ctx.draw_text_with_font(key, Point::new(x + 6.0, y), &font, &dim)?;
            let label = if on { name.to_owned() } else { format!("{name} (off)") };
            ctx.draw_text_with_font(&label, Point::new(x + 64.0, y), &font, if on { &text } else { &dim })?;
            y += 22.0;
        }
        ctx.draw_text_with_font("wheel zoom  drag pan  f follow", Point::new(x, h - 30.0), &font, &dim)?;
        ctx.draw_text_with_font("+ - zoom  0 fit  q quit", Point::new(x, h - 12.0), &font, &dim)?;
        Ok(())
    }
}

/// one panel line at (x, *y), then move *y down by `dy`
fn line(ctx: &mut dyn DrawingContext, x: f32, y: &mut f32, s: &str, paint: &Paint, font: &Font, dy: f32) -> AureaResult<()> {
    ctx.draw_text_with_font(s, Point::new(x, *y), font, paint)?;
    *y += dy;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model::{MapData, Target};
    use aurea::render::{CpuRasterizer, Renderer};
    use tb4_core::GridMeta;

    fn rgb(p: u32) -> [u8; 3] {
        [(p >> 16) as u8, (p >> 8) as u8, p as u8]
    }

    fn near(p: u32, c: Rgba, tol: i32) -> bool {
        rgb(p).iter().zip(c).all(|(&a, b)| (i32::from(a) - i32::from(b)).abs() <= tol)
    }

    /// 6 x 4 m room, free inside, walls on the border, robot at (1, 1) facing north
    fn scene() -> Model {
        let meta = GridMeta { width: 120, height: 80, resolution: 0.05, origin_x: -1.0, origin_y: -1.0 };
        let mut data = vec![0i8; 120 * 80];
        for r in 0..80 {
            for c in 0..120 {
                if !(2..78).contains(&r) || !(2..118).contains(&c) {
                    data[r * 120 + c] = 100;
                }
            }
        }
        // unknown east strip
        for r in 2..78 {
            for c in 100..118 {
                data[r * 120 + c] = -1;
            }
        }
        let now = Instant::now();
        let mut m = Model { map: Some(MapData { meta, data }), map_seq: 1, robot: Some((1.0, 1.0, std::f64::consts::FRAC_PI_2)), ..Model::default() };
        m.path = vec![(1.0, 1.0), (3.0, 1.0), (3.0, 2.0)];
        m.trajectory = vec![(1.0, 1.2), (1.0, 2.0)];
        m.scan = (0..90).map(|i| (-0.8 + 0.06 * f64::from(i), 2.8)).collect();
        m.frontiers = vec![(3.9, 0.5)];
        m.views = vec![(2.0, 0.0)];
        m.goal = Some((3.0, 2.0));
        m.targets = vec![Target { id: 7, x: 0.0, y: 2.0, confirmed: true }, Target { id: 3, x: 2.5, y: -0.5, confirmed: false }];
        m.mission = Some("RUNNING".into());
        for i in 0..11 {
            m.tick(Topic::Scan, now - std::time::Duration::from_millis(100 * (10 - i)));
        }
        m
    }

    /// TEAM7_VIZ_SNAPSHOT=frame.png cargo test: writes the test frame for a look
    fn snapshot(px: &[u32], w: u32, h: u32, path: &std::ffi::OsStr) {
        let rgba: Vec<u8> = px.iter().flat_map(|p| [(p >> 16) as u8, (p >> 8) as u8, *p as u8, 255]).collect();
        let file = std::fs::File::create(path).unwrap();
        let mut enc = png::Encoder::new(std::io::BufWriter::new(file), w, h);
        enc.set_color(png::ColorType::Rgba);
        enc.set_depth(png::BitDepth::Eight);
        enc.write_header().unwrap().write_image_data(&rgba).unwrap();
    }

    fn render(v: &mut Viewer, m: &Model, w: u32, h: u32) -> Vec<u32> {
        let mut r = CpuRasterizer::new(w, h);
        {
            let mut ctx = r.begin_frame().unwrap();
            v.draw(ctx.as_mut(), m, Instant::now()).unwrap();
        }
        r.end_frame().unwrap();
        r.pixels().to_vec()
    }

    #[test]
    fn offscreen_frame_draws_the_layers_where_they_belong() {
        let m = scene();
        let mut v = Viewer::new();
        v.camera.follow = false;
        let (w, h) = (1000u32, 560u32);
        let px = render(&mut v, &m, w, h);
        if let Some(path) = std::env::var_os("TEAM7_VIZ_SNAPSHOT") {
            snapshot(&px, w, h, &path);
        }
        let vp = v.camera.viewport(m.bounds(), (0.0, 0.0, f64::from(w) - f64::from(PANEL_WIDTH), f64::from(h)), MARGIN, None);
        let at = |x: f64, y: f64| {
            let (sx, sy) = vp.to_screen(x, y);
            px[sy as usize * w as usize + sx as usize]
        };
        assert!(near(at(2.0, 2.4), pal::FREE, 2), "free {:?}", rgb(at(2.0, 2.4)));
        assert!(near(at(4.2, 1.5), pal::UNKNOWN, 2), "unknown {:?}", rgb(at(4.2, 1.5)));
        assert!(near(at(-0.97, 1.5), pal::OCCUPIED, 2), "wall {:?}", rgb(at(-0.97, 1.5)));
        assert!(near(at(2.0, 1.0), pal::PATH, 12), "path {:?}", rgb(at(2.0, 1.0)));
        assert!(near(at(1.0, 1.6), pal::TRAJECTORY, 12), "trajectory {:?}", rgb(at(1.0, 1.6)));
        assert!(near(at(1.0, 2.8), pal::SCAN, 12), "scan {:?}", rgb(at(1.0, 2.8)));
        assert!(near(at(3.9, 0.5), pal::FRONTIER, 12), "frontier {:?}", rgb(at(3.9, 0.5)));
        assert!(near(at(2.0, 0.0), pal::VIEW, 12), "view {:?}", rgb(at(2.0, 0.0)));
        assert!(near(at(0.0, 2.0), pal::TARGET_CONFIRMED, 12), "confirmed {:?}", rgb(at(0.0, 2.0)));
        assert!(near(at(2.5, -0.5), pal::TARGET_TENTATIVE, 12), "tentative {:?}", rgb(at(2.5, -0.5)));
        // robot body east of its heading line (it faces north)
        assert!(rgb(at(1.08, 1.0)).iter().all(|&c| c > 215), "robot {:?}", rgb(at(1.08, 1.0)));
        // panel background and text
        assert!(near(px[(h / 2 * w + w - 4) as usize], pal::PANEL, 1));
        let bright = (0..h).flat_map(|y| (w - 300..w).map(move |x| (x, y))).filter(|&(x, y)| near(px[(y * w + x) as usize], pal::TEXT, 30)).count();
        assert!(bright > 300, "panel text pixels {bright}");

        // layer 2 off: the scan pixel shows the map again
        v.key(KeyCode::Key2);
        let px = render(&mut v, &m, w, h);
        let (sx, sy) = vp.to_screen(1.0, 2.8);
        assert!(near(px[sy as usize * w as usize + sx as usize], pal::FREE, 2));
    }

    #[test]
    fn follow_centres_the_robot_and_keys_drive_the_camera() {
        let m = scene();
        let mut v = Viewer::new();
        let (w, h) = (900u32, 500u32);
        let _ = render(&mut v, &m, w, h);
        let (vp, rect) = v.viewport().unwrap();
        let (cx, cy) = vp.to_screen(1.0, 1.0);
        assert!((f64::from(cx) - rect.2 / 2.0).abs() < 1e-3 && (f64::from(cy) - rect.3 / 2.0).abs() < 1e-3);

        v.key(KeyCode::Equals);
        assert!((v.camera.zoom - ZOOM_STEP).abs() < 1e-9);
        // a drag stops following
        assert!(!v.pointer(&press(true, 100.0, 100.0)));
        assert!(v.pointer(&moved(140.0, 120.0)));
        assert!(!v.camera.follow);
        v.pointer(&press(false, 140.0, 120.0));
        assert!(!v.pointer(&moved(200.0, 200.0)));
        v.key(KeyCode::F);
        assert!(v.camera.follow);
        v.key(KeyCode::Key0);
        assert_eq!(v.camera.zoom, 1.0);
        assert!(v.key(KeyCode::Q));
    }

    fn press(pressed: bool, x: f64, y: f64) -> WindowEvent {
        WindowEvent::MouseButton {
            button: MouseButton::Left,
            pressed,
            modifiers: Default::default(),
            x,
            y,
            click_count: 1,
            pointer: Default::default(),
            time: Default::default(),
        }
    }

    fn moved(x: f64, y: f64) -> WindowEvent {
        WindowEvent::MouseMove { x, y, buttons: Default::default(), modifiers: Default::default(), pointer: Default::default(), time: Default::default() }
    }
}
