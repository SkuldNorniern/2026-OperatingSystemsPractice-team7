//! Painting a frame with aurea: the map image, the scene primitives, the status panel and its controls.
//! Works on any `DrawingContext`, the window's canvas or an offscreen `CpuRasterizer`.

use std::time::Instant;

use aurea::AureaResult;
use aurea::prelude::KeyCode;
use aurea::render::{Color, DrawingContext, Font, Image, Paint, PaintStyle, Path, PathCommand, Point, Rect, Sampling, Transform};
use aurea::ui::{Button, Segmented, Theme, UiState};
use tb4_core::GridMeta;
use tb4_sim::worlds;

use crate::app::{App, SPEEDS, STEP_BUDGET, status_lines};
use crate::scene::{self, Prim, Rgba, SceneInput, Viewport, colors};

pub const PANEL_WIDTH: f32 = 340.0;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Wid {
    Pause,
    Speed,
    World,
    Reset,
    SeedDown,
    SeedUp,
}

pub struct Viewer {
    pub app: App,
    pub ui: UiState<Wid>,
    pub theme: Theme,
    map: Option<(GridMeta, Vec<i8>, Image)>,
}

fn color(c: Rgba) -> Color {
    Color::rgba(c[0], c[1], c[2], c[3])
}

fn stroke(c: Rgba, width: f32) -> Paint {
    Paint::new().color(color(c)).style(PaintStyle::Stroke).stroke_width(width)
}

fn fill(c: Rgba) -> Paint {
    Paint::new().color(color(c)).style(PaintStyle::Fill)
}

fn polyline(pts: &[(f32, f32)]) -> Path {
    let mut p = Path::new();
    for (i, &(x, y)) in pts.iter().enumerate() {
        let pt = Point::new(x, y);
        p.commands.push(if i == 0 { PathCommand::MoveTo(pt) } else { PathCommand::LineTo(pt) });
    }
    p
}

pub fn paint_prims(ctx: &mut dyn DrawingContext, prims: &[Prim]) -> AureaResult<()> {
    for p in prims {
        match p {
            Prim::Line { a, b, color, width, dashed } => {
                let mut paint = stroke(*color, *width);
                if *dashed {
                    paint = paint.dash(&[6.0, 4.0], 0.0);
                }
                ctx.draw_path(&polyline(&[*a, *b]), &paint)?;
            }
            Prim::Polyline { pts, color, width } => ctx.draw_path(&polyline(pts), &stroke(*color, *width))?,
            Prim::Circle { c, r, fill: f, stroke: s } => {
                let at = Point::new(c.0, c.1);
                if let Some(f) = f {
                    ctx.draw_circle(at, *r, &fill(*f))?;
                }
                if let Some((sc, w)) = s {
                    ctx.draw_circle(at, *r, &stroke(*sc, *w))?;
                }
            }
            Prim::Text { at, text, color: c, size } => {
                ctx.draw_text_with_font(text, Point::new(at.0, at.1), &Font::new("", *size), &fill(*c))?;
            }
        }
    }
    Ok(())
}

impl Viewer {
    pub fn new(app: App) -> Self {
        Self { app, ui: UiState::new(), theme: Theme::dark(), map: None }
    }

    /// the map image, rebuilt only when the SLAM map changed
    fn map_image(&mut self) -> Option<Image> {
        let state = self.app.sim.state();
        let g = state.map?;
        let same = self.map.as_ref().is_some_and(|(m, d, _)| *m == g.meta && *d == g.data);
        if !same {
            let img = Image::new(g.meta.width, g.meta.height, scene::map_rgba(g)).with_opaque(true);
            self.map = Some((g.meta, g.data.clone(), img));
        }
        self.map.as_ref().map(|(_, _, i)| i.clone())
    }

    /// keyboard shortcuts; true when the viewer should close
    pub fn key(&mut self, key: KeyCode) -> bool {
        let a = &mut self.app;
        match key {
            KeyCode::Space | KeyCode::P => a.opts.paused = !a.opts.paused,
            KeyCode::Key1 => a.opts.speed = 0,
            KeyCode::Key2 => a.opts.speed = 1,
            KeyCode::Key3 => a.opts.speed = 2,
            KeyCode::Key4 => a.opts.speed = 3,
            KeyCode::R => a.reset(),
            KeyCode::W => a.set_world((a.world_index() + 1) % worlds::NAMES.len()),
            KeyCode::LeftBracket | KeyCode::Minus => a.set_seed(a.opts.seed.saturating_sub(1).max(1)),
            KeyCode::RightBracket | KeyCode::Equals => a.set_seed(a.opts.seed + 1),
            KeyCode::S if a.opts.paused => a.step_for(0.1),
            KeyCode::Q | KeyCode::Escape => return true,
            _ => {}
        }
        false
    }

    /// advance the sim to now and draw one frame
    pub fn frame(&mut self, ctx: &mut dyn DrawingContext) -> AureaResult<()> {
        self.app.advance(Instant::now(), STEP_BUDGET);
        self.draw(ctx)
    }

    pub fn draw(&mut self, ctx: &mut dyn DrawingContext) -> AureaResult<()> {
        let (w, h) = (ctx.width() as f32, ctx.height() as f32);
        ctx.clear(color(colors::BACKGROUND))?;
        let map_w = (w - PANEL_WIDTH).max(100.0);
        let image = self.map_image();
        let sim = &self.app.sim;
        let state = sim.state();
        let vp = Viewport::fit(state.world.bounds, (0.0, 0.0, f64::from(map_w), f64::from(h)), 12.0);

        if let (Some(g), Some(img)) = (state.map, image.as_ref()) {
            let layer = scene::map_layer(g, state.map_origin, &vp);
            let [a, b, c, d, e, f] = layer.affine;
            ctx.save()?;
            ctx.transform(Transform { m11: a, m12: b, m13: 0.0, m21: c, m22: d, m23: 0.0, m31: e, m32: f, m33: 1.0 })?;
            ctx.set_sampling(Sampling::Nearest)?;
            ctx.draw_image_rect(img, Rect::new(0.0, 0.0, layer.width as f32, layer.height as f32))?;
            ctx.set_sampling(Sampling::Linear)?;
            ctx.restore()?;
        }

        let trajectory = sim.navigator().trajectory();
        let (frontiers, views, _) = sim.explorer().viz();
        let input = SceneInput {
            state: &state,
            robot_radius: sim.cfg.robot.radius,
            trajectory: &trajectory,
            frontiers,
            views,
            trail: &self.app.trail,
        };
        paint_prims(ctx, &scene::build(&input, &vp))?;

        // status panel
        let px = map_w;
        ctx.draw_rect(Rect::new(px, 0.0, w - px, h), &fill([38, 38, 44, 255]))?;
        let lines = status_lines(&sim.metrics(), &self.app.opts, self.app.rate);
        let font = Font::monospace(13.0);
        let text = fill(colors::TEXT);
        let mut y = 196.0;
        for l in &lines {
            ctx.draw_text_with_font(l, Point::new(px + 14.0, y), &font, &text)?;
            y += 18.0;
        }
        y += 8.0;
        for (label, c) in [
            ("map free / occupied / unknown", colors::FREE),
            ("walls (truth)", colors::WALL),
            ("glass (truth)", colors::GLASS),
            ("planned path", colors::PATH),
            ("MPPI trajectory", colors::TRAJECTORY),
            ("trail", colors::TRAIL),
            ("SLAM pose estimate", colors::ESTIMATE),
            ("frontier goals", colors::FRONTIER),
            ("camera views", colors::VIEW),
            ("active goal", colors::GOAL),
            ("bumps", colors::BUMP),
            ("marker readable / <2 m / not seen", colors::MARKER_READABLE),
        ] {
            // the legend gives way to the key help in a short window
            if y > h - 48.0 {
                break;
            }
            ctx.draw_rect(Rect::new(px + 14.0, y - 9.0, 10.0, 10.0), &fill(c))?;
            ctx.draw_text_with_font(label, Point::new(px + 30.0, y), &font, &text)?;
            y += 16.0;
        }
        ctx.draw_text_with_font(
            "keys: space pause  1-4 speed  r reset",
            Point::new(px + 14.0, h - 30.0),
            &font,
            &fill([160, 160, 160, 255]),
        )?;
        ctx.draw_text_with_font(
            "w world  [ ] seed  s step  q quit",
            Point::new(px + 14.0, h - 12.0),
            &font,
            &fill([160, 160, 160, 255]),
        )?;
        self.controls(ctx, px)
    }

    fn controls(&mut self, ctx: &mut dyn DrawingContext, px: f32) -> AureaResult<()> {
        let app = &mut self.app;
        let mut speed = app.opts.speed;
        let mut world = app.world_index();
        let (reset, pause, down, up);
        {
            let mut ui = self.ui.frame(ctx, &self.theme);
            let x = px + 12.0;
            let label = if app.opts.paused { "Resume" } else { "Pause" };
            pause = Button::new(label).show(&mut ui, Wid::Pause, Rect::new(x, 12.0, 100.0, 32.0))?.clicked;
            reset = Button::new("Reset").show(&mut ui, Wid::Reset, Rect::new(x + 108.0, 12.0, 100.0, 32.0))?.clicked;
            Segmented::new(&SPEEDS, &mut speed).show(&mut ui, Wid::Speed, Rect::new(x, 54.0, 316.0, 32.0))?;
            Segmented::new(&worlds::NAMES, &mut world).show(&mut ui, Wid::World, Rect::new(x, 96.0, 316.0, 32.0))?;
            down = Button::new("seed -").show(&mut ui, Wid::SeedDown, Rect::new(x, 138.0, 100.0, 32.0))?.clicked;
            up = Button::new("seed +").show(&mut ui, Wid::SeedUp, Rect::new(x + 108.0, 138.0, 100.0, 32.0))?.clicked;
        }
        app.opts.speed = speed;
        if pause {
            app.opts.paused = !app.opts.paused;
        }
        if world != app.world_index() {
            app.set_world(world);
        } else if reset {
            app.reset();
        } else if down {
            app.set_seed(app.opts.seed.saturating_sub(1).max(1));
        } else if up {
            app.set_seed(app.opts.seed + 1);
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::app::Options;
    use aurea::render::{CpuRasterizer, Renderer};

    /// draws a frame offscreen and returns 0xAARRGGBB pixels
    fn render(viewer: &mut Viewer, w: u32, h: u32) -> Vec<u32> {
        let mut r = CpuRasterizer::new(w, h);
        {
            let mut ctx = r.begin_frame().unwrap();
            viewer.draw(ctx.as_mut()).unwrap();
        }
        r.end_frame().unwrap();
        r.pixels().to_vec()
    }

    fn rgb(p: u32) -> [u8; 3] {
        [(p >> 16) as u8, (p >> 8) as u8, p as u8]
    }

    fn near(p: u32, c: Rgba, tol: i32) -> bool {
        rgb(p).iter().zip(c).all(|(&a, b)| (i32::from(a) - i32::from(b)).abs() <= tol)
    }

    #[test]
    fn offscreen_frame_draws_map_world_robot_and_panel() {
        let opts = Options { world: "small".into(), perfect_pose: true, ..Options::default() };
        let mut viewer = Viewer::new(App::new(opts).unwrap());
        while viewer.app.sim.time() < 15.0 {
            viewer.app.step();
        }
        let (w, h) = (1000u32, 600u32);
        let px = render(&mut viewer, w, h);
        let count = |c: Rgba| px.iter().filter(|&&p| near(p, c, 2)).count();
        assert!(count(colors::FREE) > 2000, "free {}", count(colors::FREE));
        assert!(count(colors::UNKNOWN) > 2000, "unknown {}", count(colors::UNKNOWN));
        assert!(count(colors::WALL) > 100, "walls {}", count(colors::WALL));

        // the robot body sits where the viewport puts the true pose
        let state = viewer.app.sim.state();
        let vp = Viewport::fit(state.world.bounds, (0.0, 0.0, f64::from(w) - f64::from(PANEL_WIDTH), f64::from(h)), 12.0);
        let r = vp.len(viewer.app.sim.cfg.robot.radius);
        let (cx, cy) = vp.to_screen(state.truth.x, state.truth.y);
        // a point inside the disc, away from the heading line
        let a = state.truth.yaw + std::f64::consts::FRAC_PI_2;
        let (sx, sy) = (cx + 0.5 * r * a.cos() as f32, cy - 0.5 * r * a.sin() as f32);
        let p = px[sy as usize * w as usize + sx as usize];
        assert!(rgb(p).iter().all(|&v| v > 215), "robot pixel {:?}", rgb(p));

        // panel background and some text in it
        let panel = |x: u32, y: u32| px[(y * w + x) as usize];
        assert!(near(panel(w - 5, h / 2), [38, 38, 44, 255], 1));
        let bright = (200..h).flat_map(|y| (700..w).map(move |x| (x, y))).filter(|&(x, y)| near(panel(x, y), colors::TEXT, 30)).count();
        assert!(bright > 200, "panel text pixels {bright}");

        if let Some(path) = std::env::var_os("TB4_VIEWER_SNAPSHOT") {
            let mut rgba = Vec::with_capacity(px.len() * 4);
            for p in &px {
                rgba.extend_from_slice(&[(p >> 16) as u8, (p >> 8) as u8, *p as u8, 255]);
            }
            let file = std::fs::File::create(path).unwrap();
            let mut enc = png::Encoder::new(std::io::BufWriter::new(file), w, h);
            enc.set_color(png::ColorType::Rgba);
            enc.set_depth(png::BitDepth::Eight);
            enc.write_header().unwrap().write_image_data(&rgba).unwrap();
        }
    }

    #[test]
    fn keys_change_pause_speed_and_world() {
        let opts = Options { world: "small".into(), ..Options::default() };
        let mut v = Viewer::new(App::new(opts).unwrap());
        assert!(!v.key(KeyCode::Space));
        assert!(v.app.opts.paused);
        v.key(KeyCode::Key4);
        assert_eq!(v.app.opts.speed, 3);
        v.key(KeyCode::W);
        assert_eq!(v.app.opts.world, "studio");
        v.key(KeyCode::RightBracket);
        assert_eq!(v.app.opts.seed, 2);
        assert!(v.key(KeyCode::Q));
    }
}
