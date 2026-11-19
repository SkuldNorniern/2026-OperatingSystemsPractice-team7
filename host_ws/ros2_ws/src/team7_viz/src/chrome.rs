//! Everything around the map, drawn on the same canvas with aurea's UI widgets: a toolbar, a
//! header strip with what matters at a glance (mission clock, the 3 targets, one status light),
//! the dock (layers, targets, camera, health) beside the map with a splitter, and a status bar.
//! Drawn, not native, so it gets real typography, colour, wrapping and hover states, and every
//! pixel of it is tested offscreen.

use std::path::PathBuf;
use std::time::{Duration, Instant, SystemTime};

use aurea::AureaResult;
use aurea::prelude::{KeyCode, Modifiers, WindowEvent};
use aurea::render::{Color, DrawingContext, Font, FontWeight, Image, Rect};
use aurea::ui::{Align, Button, Checkbox, Disclosure, ScrollArea, Sense, Slider, Splitter, Theme, UiState};
use tb4_draw::palette as pal;
use tb4_draw::prim;

use crate::app::{App, Tool};
use crate::health::{self, Level};
use crate::model::{Model, Slot, human_age, mission_clock};
use crate::outputs;
use crate::ros::wall_secs;
use crate::scene::{self, LAYERS};

pub const TOOLBAR_H: f32 = 40.0;
pub const STRIP_H: f32 = 58.0;
pub const STATUS_H: f32 = 24.0;
const ROW_H: f32 = 25.0;
const HEADER_H: f32 = 28.0;
const PAD: f32 = 12.0;
/// a camera image older than this is shown grey, marked stale
const CAMERA_STALE: Duration = Duration::from_secs(3);
const OUT_EVERY: Duration = Duration::from_secs(5);

/// dock sections, in order
pub const SECTIONS: [&str; 4] = ["Layers", "Camera", "Targets", "Health"];

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Wid {
    Tool(u8),
    ZoomIn,
    ZoomOut,
    Fit,
    Follow,
    North,
    Shot,
    Dock,
    Split,
    SectionScroll(u8),
    SectionSplit(u8),
    Section(u8),
    Check(u8),
    Row(u8),
    Opacity(u8),
    Camera,
}

/// what the window must do after a frame
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Requests {
    pub screenshot: bool,
}

pub struct Chrome {
    pub ui: UiState<Wid>,
    pub theme: Theme,
    pub dock_open: bool,
    pub dock_w: f32,
    pub sections: [bool; 4],
    /// the map's rectangle in the last frame, canvas pixels
    pub map_area: Rect,
    pub domain: String,
    pub layout: DockLayout,
    out_dir: Option<PathBuf>,
    fixed_out_dir: bool,
    out_checked: Option<Instant>,
    camera_image: Option<(u64, bool, Image)>,
}

fn rgba(c: [u8; 4]) -> Color {
    prim::color(c)
}

fn font(size: f32) -> Font {
    Font::new("", size)
}

fn bold(size: f32) -> Font {
    Font::new("", size).with_weight(FontWeight::Bold)
}

fn level_color(t: &Theme, l: Level) -> Color {
    match l {
        Level::Ok => t.palette.success,
        Level::Warn => t.palette.warning,
        Level::Bad => t.palette.danger,
        Level::Off => t.palette.faint,
    }
}

impl Chrome {
    pub fn new(domain: String, out_dir: Option<PathBuf>) -> Self {
        Self {
            ui: UiState::new(),
            theme: Theme::dark(),
            dock_open: true,
            dock_w: 330.0,
            sections: [true, true, true, false],
            map_area: Rect::new(0.0, 0.0, 1.0, 1.0),
            domain,
            layout: DockLayout::default(),
            fixed_out_dir: out_dir.is_some(),
            out_dir,
            out_checked: None,
            camera_image: None,
        }
    }

    /// keys for the chrome itself; true when used
    pub fn key(&mut self, key: KeyCode, mods: Modifiers) -> bool {
        match key {
            KeyCode::D if !mods.ctrl => self.dock_open = !self.dock_open,
            KeyCode::H if !mods.ctrl => self.sections[3] = !self.sections[3],
            _ => return false,
        }
        true
    }

    /// the run folder: re-checked every few seconds so a newer run is picked up
    fn run_dir(&mut self, now: Instant) -> Option<PathBuf> {
        if !self.fixed_out_dir && self.out_checked.is_none_or(|t| now.saturating_duration_since(t) >= OUT_EVERY) {
            self.out_dir = outputs::newest_run_dir(&outputs::runs_root());
            self.out_checked = Some(now);
        }
        self.out_dir.clone()
    }

    /// set the run folder (tests, `--output-dir`)
    pub fn set_run_dir(&mut self, dir: Option<PathBuf>) {
        self.fixed_out_dir = true;
        self.out_dir = dir;
    }

    /// one frame of the whole canvas. One UI frame only: a press is resolved within the frame that
    /// follows it, so the splitter and the dock must share it.
    pub fn frame(&mut self, ctx: &mut dyn DrawingContext, app: &mut App, model: &Model, now: Instant) -> AureaResult<Requests> {
        let (w, h) = (ctx.width() as f32, ctx.height() as f32);
        let mut req = Requests::default();
        let run_dir = self.run_dir(now);
        let theme = self.theme.clone();
        ctx.clear(theme.palette.background)?;
        let Self { ui: state, dock_open, dock_w, sections, map_area, domain, layout, camera_image, .. } = self;
        let mut ui = state.frame(ctx, &theme);

        let body = Rect::new(0.0, TOOLBAR_H + STRIP_H, w, (h - TOOLBAR_H - STRIP_H - STATUS_H).max(10.0));
        let (dock, map) = if *dock_open {
            // the preferred width changes only when dragged: a small first frame must not shrink it
            let mut at = *dock_w;
            let (a, b, r) = Splitter::new(&mut at).min(240.0, 240.0).show(&mut ui, Wid::Split, body)?;
            if r.held {
                *dock_w = at;
            }
            (Some(a), b)
        } else {
            (None, body)
        };
        *map_area = map;
        // the map first, so the chrome draws over its edges
        app.draw_in(&mut *ui.ctx, model, now, map)?;

        toolbar(&mut ui, app, dock_open, &mut req, model, w)?;
        strip(&mut ui, model, now, w)?;
        if let Some(d) = dock {
            ui.fill(d, theme.palette.surface)?;
            dock_sections(&mut ui, d, layout, sections, camera_image, app, model, now, run_dir.as_deref())?;
            ui.line(aurea::render::Point::new(d.right(), d.y), aurea::render::Point::new(d.right(), d.bottom()), 1.0, theme.palette.border)?;
        }
        status_bar(&mut ui, app, model, now, domain, Rect::new(0.0, h - STATUS_H, w, STATUS_H))?;
        Ok(req)
    }

    /// pass a window event (already in canvas pixels) to the widgets; true when they want a redraw
    pub fn handle(&mut self, event: &WindowEvent) -> bool {
        self.ui.handle(event)
    }
}

fn toolbar(ui: &mut aurea::ui::Ui<'_, Wid>, app: &mut App, dock_open: &mut bool, req: &mut Requests, model: &Model, w: f32) -> AureaResult<()> {
    let t = ui.theme.clone();
    ui.fill(Rect::new(0.0, 0.0, w, TOOLBAR_H), t.palette.surface)?;
    let (y, bh) = (6.0, 28.0);
    let mut x = 8.0;
    // tools
    for (i, (tool, label)) in [(Tool::Interact, "Interact"), (Tool::Measure, "Measure"), (Tool::Goal, "2D Goal")].iter().enumerate() {
        let bw = 76.0;
        if Button::new(label).selected(app.tool == *tool).show(ui, Wid::Tool(i as u8), Rect::new(x, y, bw, bh))?.clicked {
            app.set_tool(*tool);
        }
        x += bw + 4.0;
    }
    x += 10.0;
    ui.line(aurea::render::Point::new(x - 7.0, 9.0), aurea::render::Point::new(x - 7.0, TOOLBAR_H - 9.0), 1.0, t.palette.border)?;
    // view
    for (id, label, bw) in [(Wid::ZoomOut, "-", 30.0), (Wid::ZoomIn, "+", 30.0), (Wid::Fit, "Fit", 44.0)] {
        if Button::new(label).show(ui, id, Rect::new(x, y, bw, bh))?.clicked {
            match id {
                Wid::ZoomOut => app.zoom(1.0 / 1.2),
                Wid::ZoomIn => app.zoom(1.2),
                _ => app.fit(),
            }
        }
        x += bw + 4.0;
    }
    if Button::new("Follow").selected(app.camera.follow).show(ui, Wid::Follow, Rect::new(x, y, 62.0, bh))?.clicked {
        app.camera.follow = !app.camera.follow;
    }
    x += 66.0;
    if Button::new("North up").selected(app.camera.rotation == 0.0).show(ui, Wid::North, Rect::new(x, y, 72.0, bh))?.clicked {
        let r = app.camera.rotation;
        app.rotate(-r);
    }
    x += 76.0 + 10.0;
    ui.line(aurea::render::Point::new(x - 7.0, 9.0), aurea::render::Point::new(x - 7.0, TOOLBAR_H - 9.0), 1.0, t.palette.border)?;
    // capture
    if Button::new("Screenshot").show(ui, Wid::Shot, Rect::new(x, y, 92.0, bh))?.clicked {
        req.screenshot = true;
    }

    // right side: mode badge, source stack, dock toggle
    let mut rx = w - 8.0;
    let dw = 64.0;
    rx -= dw;
    if Button::new("Panel").selected(*dock_open).show(ui, Wid::Dock, Rect::new(rx, y, dw, bh))?.clicked {
        *dock_open = !*dock_open;
    }
    let (badge, color) = if app.allow_goals { ("GOALS ON", t.palette.warning) } else { ("READ-ONLY", t.palette.dim) };
    let f = bold(11.0);
    let bw = ui.text_width(badge, &f) + 16.0;
    rx -= bw + 10.0;
    let r = Rect::new(rx, 11.0, bw, 18.0);
    ui.border(r, 9.0, 1.0, color)?;
    ui.text(badge, r, &f, color, Align::Center)?;
    let stack = format!("{} stack", model.stack());
    let sw = ui.text_width(&stack, &font(11.0));
    rx -= sw + 12.0;
    if model.stack() != "-" {
        ui.text(&stack, Rect::new(rx, 11.0, sw, 18.0), &font(11.0), t.palette.dim, Align::Start)?;
    }
    Ok(())
}

/// the header strip: mission clock, 3 target slots, one status light (or the stale banner)
fn strip(ui: &mut aurea::ui::Ui<'_, Wid>, model: &Model, now: Instant, w: f32) -> AureaResult<()> {
    let t = ui.theme.clone();
    let r = Rect::new(0.0, TOOLBAR_H, w, STRIP_H);
    let stale = model.stale_for(now);
    ui.fill(r, t.palette.raised)?;
    ui.line(aurea::render::Point::new(0.0, r.bottom() - 0.5), aurea::render::Point::new(w, r.bottom() - 0.5), 1.0, t.palette.border)?;

    // clock
    let (big, small) = mission_clock(model, now);
    let x = PAD;
    ui.text("MISSION", Rect::new(x, r.y + 6.0, 140.0, 12.0), &bold(10.0), t.palette.dim, Align::Start)?;
    ui.text(&big, Rect::new(x, r.y + 18.0, 140.0, 24.0), &bold(22.0), t.palette.text, Align::Start)?;
    ui.text(&small, Rect::new(x + 74.0, r.y + 26.0, 160.0, 14.0), &font(11.0), t.palette.dim, Align::Start)?;

    // targets: three slots
    let mut x = 250.0f32.min(w * 0.3);
    ui.text("TARGETS", Rect::new(x, r.y + 6.0, 140.0, 12.0), &bold(10.0), t.palette.dim, Align::Start)?;
    let mut targets: Vec<_> = model.targets.iter().collect();
    targets.sort_by_key(|t| (!t.confirmed, t.id));
    let slot_w = 150.0;
    for i in 0..3 {
        let sr = Rect::new(x, r.y + 20.0, slot_w, 30.0);
        ui.round(sr, 6.0, t.palette.surface)?;
        match targets.get(i) {
            Some(tg) => {
                let c = rgba(if tg.confirmed { pal::TARGET_CONFIRMED } else { pal::TARGET_TENTATIVE });
                ui.circle(aurea::render::Point::new(sr.x + 12.0, sr.y + 15.0), 5.0, c)?;
                let id = if tg.confirmed { format!("ID {}", tg.id) } else { format!("ID {}?", tg.id) };
                ui.text(&id, Rect::new(sr.x + 22.0, sr.y + 2.0, 120.0, 14.0), &bold(12.0), t.palette.text, Align::Start)?;
                let sub = match (tg.confirmed, tg.views) {
                    (true, _) => format!("({:.2}, {:.2})", tg.x, tg.y),
                    (false, Some(v)) => format!("({:.2}, {:.2}) {v}v", tg.x, tg.y),
                    (false, None) => format!("({:.2}, {:.2})", tg.x, tg.y),
                };
                ui.text(&sub, Rect::new(sr.x + 22.0, sr.y + 15.0, 126.0, 13.0), &font(11.0), t.palette.dim, Align::Start)?;
            }
            None => {
                ui.text(&format!("target {}", i + 1), Rect::new(sr.x + 12.0, sr.y + 8.0, 120.0, 14.0), &font(12.0), t.palette.faint, Align::Start)?;
            }
        }
        x += slot_w + 8.0;
    }
    if targets.len() > 3 {
        ui.text(&format!("+{}", targets.len() - 3), Rect::new(x, r.y + 28.0, 40.0, 14.0), &font(11.0), t.palette.dim, Align::Start)?;
        x += 30.0;
    }

    // status light, or the banner when nothing comes
    let x = (x + 16.0).max(w - 360.0).min(w - 200.0).max(x + 8.0);
    let sr = Rect::new(x, r.y + 14.0, (w - x - PAD).max(80.0), 30.0);
    let (wall, _) = (wall_secs(), ());
    let (ros_now, _) = model.ros_now(now, wall);
    let (level, text) = health::summary(model, now, ros_now);
    if stale.is_some() {
        ui.round(sr, 6.0, t.palette.danger.with_alpha(60))?;
        ui.border(sr, 6.0, 1.0, t.palette.danger)?;
    }
    let c = level_color(&t, level);
    ui.circle(aurea::render::Point::new(sr.x + 14.0, sr.y + 15.0), 6.0, c)?;
    ui.text_clipped(&text, Rect::new(sr.x + 26.0, sr.y + 7.0, sr.width - 32.0, 16.0), &if stale.is_some() { bold(12.0) } else { font(12.0) }, t.palette.text, Align::Start)?;
    Ok(())
}

/// height of the drag handle between two open sections
pub const SPLIT_H: f32 = 7.0;
/// a section's body is never smaller than about two rows
pub const MIN_BODY: f32 = 2.0 * ROW_H + 8.0;

/// sizes of the dock sections: what the user dragged them to, how far each is scrolled, and where
/// the last frame put them (for hit tests)
#[derive(Debug, Clone)]
pub struct DockLayout {
    /// preferred body heights (without the header), pixels
    pub heights: [f32; 4],
    pub scrolls: [f32; 4],
    content: [f32; 4],
    /// a handle being dragged: (section above, section below, pointer y at the press, their heights then)
    drag: Option<(usize, usize, f32, f32, f32)>,
    /// last frame: body rectangles of the open sections, and the handles (above, below, rect)
    pub bodies: [Option<Rect>; 4],
    pub handles: Vec<(usize, usize, Rect)>,
}

impl Default for DockLayout {
    fn default() -> Self {
        Self { heights: [380.0, 250.0, 170.0, 170.0], scrolls: [0.0; 4], content: [0.0; 4], drag: None, bodies: [None; 4], handles: Vec::new() }
    }
}

/// open sections' preferred heights scaled to fill `avail`, each at least `MIN_BODY`
fn fit(heights: &[f32; 4], open: &[usize], avail: f32) -> [f32; 4] {
    let mut out = [0.0; 4];
    let sum: f32 = open.iter().map(|&i| heights[i].max(1.0)).sum();
    if open.is_empty() || sum <= 0.0 {
        return out;
    }
    let k = avail / sum;
    for &i in open {
        out[i] = (heights[i].max(1.0) * k).max(MIN_BODY);
    }
    // what scaling and the minimum left over goes to the largest section
    let used: f32 = open.iter().map(|&i| out[i]).sum();
    if let Some(&big) = open.iter().max_by(|&&a, &&b| out[a].total_cmp(&out[b])) {
        out[big] = (out[big] + avail - used).max(MIN_BODY);
    }
    out
}

/// the dock: a header per section; open sections share the height, each scrolls on its own, and a
/// handle between two open sections moves their boundary
#[allow(clippy::too_many_arguments)]
fn dock_sections(
    ui: &mut aurea::ui::Ui<'_, Wid>,
    d: Rect,
    layout: &mut DockLayout,
    sections: &mut [bool; 4],
    camera_image: &mut Option<(u64, bool, Image)>,
    app: &mut App,
    model: &Model,
    now: Instant,
    run_dir: Option<&std::path::Path>,
) -> AureaResult<()> {
    let t = ui.theme.clone();
    let open: Vec<usize> = (0..4).filter(|&i| sections[i]).collect();
    let headers = 4.0 * (HEADER_H + 2.0) + 8.0;
    let avail = (d.height - headers - SPLIT_H * open.len().saturating_sub(1) as f32).max(0.0);
    let shown = fit(&layout.heights, &open, avail);
    layout.bodies = [None; 4];
    layout.handles.clear();
    let (x, w) = (d.x + 8.0, d.width - 16.0);
    let mut y = d.y + 4.0;
    ui.clipped(d, |ui| -> AureaResult<()> {
        for (i, title) in SECTIONS.iter().enumerate() {
            Disclosure::new(title, &mut sections[i]).show(ui, Wid::Section(i as u8), Rect::new(x, y, w, HEADER_H))?;
            y += HEADER_H + 2.0;
            if !open.contains(&i) {
                continue;
            }
            let body = Rect::new(d.x, y, d.width, shown[i]);
            layout.bodies[i] = Some(body);
            // the thin bar shows whenever the section has more than fits
            let (_, used) = ScrollArea::new(&mut layout.scrolls[i], layout.content[i]).always_show_bar(true).show(ui, Wid::SectionScroll(i as u8), body, |ui, c, _view| {
                let (cx, cw) = (c.x + 8.0, c.width - 16.0);
                let end = match i {
                    0 => layers(ui, app, model, cx, c.y, cw)?,
                    // image, 4 px gap, 16 px info line, 6 px bottom margin: fits without a bar
                    1 => camera(ui, camera_image, app, model, now, cx, c.y, cw, body.height - 28.0)?,
                    2 => targets(ui, model, cx, c.y, cw, run_dir)?,
                    _ => health_rows(ui, model, now, cx, c.y, cw)?,
                };
                Ok(end - c.y + 6.0)
            })?;
            layout.content[i] = used;
            y += shown[i];
            // a handle to the next open section, if any
            if let Some(&below) = open.iter().find(|&&j| j > i) {
                let r = Rect::new(d.x, y, d.width, SPLIT_H);
                layout.handles.push((i, below, r));
                let input = ui.interact(Wid::SectionSplit(i as u8), 0, r, Sense::Adjust, true);
                if input.hovered || input.held {
                    ui.set_cursor(aurea::CursorIcon::ResizeVertical);
                }
                if let Some(p) = ui.press(Wid::SectionSplit(i as u8), 0) {
                    layout.drag = Some((i, below, p.at.y, shown[i], shown[below]));
                }
                match layout.drag {
                    Some((a, b, y0, ha, hb)) if a == i && input.held => {
                        if let Some(p) = ui.pointer() {
                            let dy = (p.y - y0).clamp(MIN_BODY - ha, hb - MIN_BODY);
                            // the others keep what they show, so the scaling does not undo the drag
                            for &k in &open {
                                layout.heights[k] = shown[k];
                            }
                            layout.heights[a] = ha + dy;
                            layout.heights[b] = hb - dy;
                        }
                    }
                    Some((a, ..)) if a == i => layout.drag = None,
                    _ => {}
                }
                let c = if input.hovered || input.held { t.palette.accent } else { t.palette.border };
                ui.line(aurea::render::Point::new(d.x + 8.0, r.y + SPLIT_H / 2.0), aurea::render::Point::new(d.right() - 8.0, r.y + SPLIT_H / 2.0), 1.0, c)?;
                y += SPLIT_H;
            }
        }
        Ok(())
    })
}

fn layers(ui: &mut aurea::ui::Ui<'_, Wid>, app: &mut App, model: &Model, x: f32, mut y: f32, w: f32) -> AureaResult<f32> {
    let t = ui.theme.clone();
    let mut missing = Vec::new();
    for (i, l) in LAYERS.iter().enumerate() {
        let topic = match l.slot {
            Some(s) => match model.bound(s) {
                Some(topic) => Some(topic),
                None => {
                    missing.push(l.name);
                    continue;
                }
            },
            None => None,
        };
        let row = Rect::new(x, y, w, ROW_H);
        // the whole row toggles; the checkbox takes the keyboard
        let input = ui.interact(Wid::Row(i as u8), 0, row, Sense::Click, true);
        if input.hovered {
            ui.round(row, 5.0, t.palette.hover)?;
        }
        let mut on = app.layers.on[i];
        let check = Checkbox::new(&mut on).show(ui, Wid::Check(i as u8), Rect::new(x + 6.0, y + 3.5, 18.0, 18.0))?;
        if input.clicked && !check.clicked {
            on = !on;
        }
        app.layers.on[i] = on;
        tb4_draw::paint_prims(ui.ctx, &scene::symbol(l.layer, (x + 46.0, y + ROW_H / 2.0)))?;
        let key = if i < 9 { format!("{}", i + 1) } else { String::new() };
        let name_c = if on { t.palette.text } else { t.palette.dim };
        let name_w = ui.text_width(l.name, &font(13.0));
        ui.text(l.name, Rect::new(x + 66.0, y + 4.0, name_w + 2.0, 16.0), &font(13.0), name_c, Align::Start)?;
        let sub = topic.unwrap_or(l.legend);
        // the bound topic, dim, after the name on the same line
        let sx = x + 66.0 + name_w + 8.0;
        ui.text_clipped(sub, Rect::new(sx, y + 6.0, (x + w - 24.0 - sx).max(0.0), 13.0), &font(10.0), t.palette.faint, Align::End)?;
        ui.text(&key, Rect::new(x + w - 20.0, y + 5.0, 14.0, 14.0), &font(11.0), t.palette.faint, Align::End)?;
        y += ROW_H + 2.0;
        if let (Some(_), true) = (l.opacity, on) {
            let mut o = app.layers.opacity[i];
            let pct = format!("{:.0}%", o * 100.0);
            ui.text("opacity", Rect::new(x + 66.0, y + 2.0, 60.0, 14.0), &font(11.0), t.palette.dim, Align::Start)?;
            Slider::new(&mut o, 0.0..=1.0).readout(&pct).show(ui, Wid::Opacity(i as u8), Rect::new(x + 118.0, y, w - 126.0, 20.0))?;
            app.layers.opacity[i] = o;
            y += 24.0;
        }
    }
    if !missing.is_empty() {
        let text = format!("Not published: {}", missing.join(", "));
        let used = ui.text_wrapped(&text, Rect::new(x + 6.0, y + 2.0, w - 12.0, 200.0), &font(11.0), t.palette.faint, 14.0)?;
        y += used + 4.0;
    }
    Ok(y)
}

fn targets(ui: &mut aurea::ui::Ui<'_, Wid>, model: &Model, x: f32, mut y: f32, w: f32, run_dir: Option<&std::path::Path>) -> AureaResult<f32> {
    let t = ui.theme.clone();
    // columns scale with the dock; state carries the view count ("tentative, 2 views")
    let col = |f: f32| x + 6.0 + (w - 12.0) * f;
    let cols = [(0.0, "ID"), (0.16, "x [m]"), (0.36, "y [m]"), (0.56, "state")];
    for (f, head) in cols {
        ui.text(head, Rect::new(col(f), y, 70.0, 14.0), &bold(10.0), t.palette.dim, Align::Start)?;
    }
    y += 18.0;
    if model.targets.is_empty() {
        ui.text("no targets yet", Rect::new(x + 6.0, y, w, 16.0), &font(12.0), t.palette.faint, Align::Start)?;
        y += 20.0;
    }
    for tg in &model.targets {
        let c = rgba(if tg.confirmed { pal::TARGET_CONFIRMED } else { pal::TARGET_TENTATIVE });
        let state = match (tg.confirmed, tg.views) {
            (true, _) => "confirmed".to_owned(),
            (false, Some(v)) => format!("tentative, {v}v"),
            (false, None) => "tentative".to_owned(),
        };
        ui.text(&tg.id.to_string(), Rect::new(col(0.0), y, 60.0, 16.0), &font(12.0), t.palette.text, Align::Start)?;
        ui.text(&format!("{:.2}", tg.x), Rect::new(col(0.16), y, 60.0, 16.0), &font(12.0), t.palette.text, Align::Start)?;
        ui.text(&format!("{:.2}", tg.y), Rect::new(col(0.36), y, 60.0, 16.0), &font(12.0), t.palette.text, Align::Start)?;
        ui.text_clipped(&state, Rect::new(col(0.56), y, (w - 12.0) * 0.44, 16.0), &font(12.0), c, Align::Start)?;
        y += 19.0;
    }
    if let Some(g) = &model.goal_status {
        ui.text_clipped(g, Rect::new(x + 6.0, y + 2.0, w - 12.0, 15.0), &font(11.0), t.palette.dim, Align::Start)?;
        y += 18.0;
    }
    y += 6.0;
    match run_dir {
        None => {
            ui.text_clipped("no run folder in ~/team7_runs", Rect::new(x + 6.0, y, w - 12.0, 15.0), &font(11.0), t.palette.faint, Align::Start)?;
            y += 18.0;
        }
        Some(d) => {
            ui.text("Run", Rect::new(x + 6.0, y, 40.0, 15.0), &bold(11.0), t.palette.dim, Align::Start)?;
            // cut at the start: the end of a run folder name (its time) tells runs apart
            let fits = ((w - 46.0) / 7.0).max(8.0) as usize;
            ui.text(&outputs::short_name(d, fits), Rect::new(x + 40.0, y, w - 46.0, 15.0), &font(11.0), t.palette.text, Align::Start)?;
            y += 18.0;
            for f in outputs::status(d) {
                let (mark, c, what) = match f.modified {
                    Some(m) => ("ok", t.palette.success, format!("{} ago", human_age(SystemTime::now().duration_since(m).unwrap_or_default()))),
                    None => ("--", t.palette.faint, "not yet".to_owned()),
                };
                ui.text(mark, Rect::new(x + 6.0, y, 24.0, 15.0), &bold(11.0), c, Align::Start)?;
                ui.text(f.name, Rect::new(x + 40.0, y, 90.0, 15.0), &font(11.0), t.palette.text, Align::Start)?;
                ui.text_clipped(&what, Rect::new(x + 130.0, y, w - 136.0, 15.0), &font(11.0), t.palette.dim, Align::Start)?;
                y += 17.0;
            }
        }
    }
    Ok(y)
}

#[allow(clippy::too_many_arguments)]
fn camera(ui: &mut aurea::ui::Ui<'_, Wid>, cache: &mut Option<(u64, bool, Image)>, app: &mut App, model: &Model, now: Instant, x: f32, mut y: f32, w: f32, max_h: f32) -> AureaResult<f32> {
    let t = ui.theme.clone();
    let topic = model.bound(Slot::Camera);
    let Some(f) = &model.camera else {
        let text = match topic {
            Some(tp) => format!("waiting for {tp}"),
            None => "no camera debug image published".into(),
        };
        ui.text(&text, Rect::new(x + 6.0, y, w, 15.0), &font(11.0), t.palette.faint, Align::Start)?;
        return Ok(y + 18.0);
    };
    let age = now.saturating_duration_since(f.at);
    let stale = age > CAMERA_STALE;
    if cache.as_ref().is_none_or(|(seq, s, _)| *seq != f.seq || *s != stale) {
        let rgba: Vec<u8> = if stale {
            // grey and dim: an old picture must not look live
            f.rgba.as_chunks::<4>().0.iter().flat_map(|p| {
                let g = ((u32::from(p[0]) * 30 + u32::from(p[1]) * 59 + u32::from(p[2]) * 11) / 250) as u8;
                [g, g, g, 255]
            }).collect()
        } else {
            f.rgba.clone()
        };
        *cache = Some((f.seq, stale, Image::new(f.width, f.height, rgba).with_opaque(true)));
    }
    let img = cache.as_ref().map(|(.., i)| i.clone()).expect("just set");
    // fits the section both ways, aspect ratio kept
    let s = (w / f.width as f32).min(max_h.max(40.0) / f.height as f32);
    let (iw, ih) = (f.width as f32 * s, f.height as f32 * s);
    let r = Rect::new(x + (w - iw) / 2.0, y, iw, ih);
    let input = ui.interact(Wid::Camera, 0, r, Sense::Click, true);
    ui.ctx.draw_image_rect(&img, r)?;
    if input.hovered {
        ui.border(r, 0.0, 2.0, t.palette.accent)?;
        ui.tooltip_at(aurea::render::Point::new(r.x + 10.0, r.bottom() - 10.0), "click: big view (C)");
    }
    if input.clicked {
        app.camera_big = !app.camera_big;
    }
    if stale {
        let b = Rect::new(r.x + 8.0, r.y + 8.0, 110.0, 20.0);
        ui.round(b, 4.0, Color::rgba(0, 0, 0, 170))?;
        ui.text(&format!("stale {}", human_age(age)), b, &bold(11.0), t.palette.warning, Align::Center)?;
    }
    y += ih + 4.0;
    let info = format!("{}  {}x{}  {} ago", topic.unwrap_or(""), f.width, f.height, human_age(age));
    ui.text_clipped(&info, Rect::new(x + 6.0, y, w - 12.0, 14.0), &font(10.0), t.palette.faint, Align::Start)?;
    Ok(y + 16.0)
}

fn health_rows(ui: &mut aurea::ui::Ui<'_, Wid>, model: &Model, now: Instant, x: f32, mut y: f32, w: f32) -> AureaResult<f32> {
    let t = ui.theme.clone();
    let (ros_now, _) = model.ros_now(now, wall_secs());
    for c in health::all(model, now, ros_now) {
        ui.circle(aurea::render::Point::new(x + 12.0, y + 8.0), 4.5, level_color(&t, c.level))?;
        ui.text(c.name, Rect::new(x + 24.0, y, 90.0, 16.0), &font(12.0), t.palette.text, Align::Start)?;
        ui.text(&c.value, Rect::new(x + 110.0, y, 100.0, 16.0), &font(12.0), t.palette.text, Align::Start)?;
        ui.text(c.expect, Rect::new(x + 210.0, y, w - 210.0, 16.0), &font(11.0), t.palette.faint, Align::Start)?;
        y += 19.0;
    }
    Ok(y)
}

fn status_bar(ui: &mut aurea::ui::Ui<'_, Wid>, app: &App, model: &Model, now: Instant, domain: &str, r: Rect) -> AureaResult<()> {
    let t = ui.theme.clone();
    ui.fill(r, t.palette.surface)?;
    let (ros, sim) = model.ros_now(now, wall_secs());
    let time = if sim { format!("sim {ros:.1} s") } else { "wall clock".to_owned() };
    let cursor = app.cursor.map_or_else(String::new, |(x, y)| format!("   x {x:.2}  y {y:.2}"));
    let slow = if app.draw_time() > 0.04 { format!("   map draw {:.0} ms", app.draw_time() * 1000.0) } else { String::new() };
    let left = format!("{time}   ROS_DOMAIN_ID {domain}   frame map{cursor}{slow}");
    let lw = ui.text_width(&left, &font(11.0));
    ui.text(&left, Rect::new(PAD, r.y + 5.0, lw, 14.0), &font(11.0), t.palette.dim, Align::Start)?;
    let msg = app.message(now).map_or_else(|| crate::app::HINTS.to_owned(), str::to_owned);
    let mx = PAD + lw + 24.0;
    ui.text_clipped(&msg, Rect::new(mx, r.y + 5.0, (r.width - mx - PAD).max(0.0), 14.0), &font(11.0), if app.message(now).is_some() { t.palette.text } else { t.palette.faint }, Align::End)?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::app::tests::{near, rgb, scene as team7_scene, snapshot};
    use crate::model::Topic;
    use aurea::PointerKind;
    use aurea::prelude::{MouseButton, MouseButtons};
    use aurea::render::{CpuRasterizer, Renderer};

    fn draw(chrome: &mut Chrome, app: &mut App, m: &Model, w: u32, h: u32, now: Instant) -> Vec<u32> {
        let mut r = CpuRasterizer::new(w, h);
        {
            let mut ctx = r.begin_frame().unwrap();
            chrome.frame(ctx.as_mut(), app, m, now).unwrap();
        }
        r.end_frame().unwrap();
        r.pixels().to_vec()
    }

    /// a click as the window gives it: move, press, a frame, release, a frame
    fn click(chrome: &mut Chrome, app: &mut App, m: &Model, now: Instant, x: f64, y: f64) {
        chrome.handle(&WindowEvent::MouseMove { x, y, buttons: MouseButtons::default(), modifiers: Modifiers::default(), pointer: PointerKind::Mouse, time: Duration::ZERO });
        draw(chrome, app, m, 1280, 800, now);
        for pressed in [true, false] {
            chrome.handle(&WindowEvent::MouseButton {
                button: MouseButton::Left,
                pressed,
                modifiers: Modifiers::default(),
                x,
                y,
                click_count: 1,
                pointer: PointerKind::Mouse,
                time: Duration::ZERO,
            });
            draw(chrome, app, m, 1280, 800, now);
        }
    }

    fn team7_model() -> Model {
        let mut m = team7_scene();
        m.graph = ["/map", "/team7/path", "/team7/mppi/trajectory", "/team7/frontiers", "/team7/goal", "/team7/targets", "/team7/mppi/local_cost", "/global_costmap/costmap"].map(String::from).to_vec();
        m
    }

    #[test]
    fn whole_window_frame_and_clicks_on_the_dock() {
        let now = Instant::now();
        let mut m = team7_model();
        for i in 0..20 {
            m.tick(Topic::Odom, now - Duration::from_millis(50 * i));
            m.tick(Topic::Tf, now - Duration::from_millis(50 * i));
        }
        let mut chrome = Chrome::new("91".into(), None);
        chrome.set_run_dir(None);
        let mut app = App::new(false);
        let (w, h) = (1280u32, 800u32);
        let px = draw(&mut chrome, &mut app, &m, w, h, now);
        snapshot(&px, w, h, "window_team7.png");
        // the map sits right of the dock, below the strip
        let a = chrome.map_area;
        assert!(a.x > 300.0 && (a.y - (TOOLBAR_H + STRIP_H)).abs() < 1.0, "{a:?}");
        // the costmaps start off: the floor beside the inner wall is plain free grey
        let (vp, _) = app.viewport().unwrap();
        let (sx, sy) = vp.to_screen(1.75, 0.4);
        let p = px[(sy + a.y) as usize * w as usize + (sx + a.x) as usize];
        assert!(near(p, pal::FREE, 2), "free {:?}", rgb(p));
        // a confirmed target fills the first header slot (green dot)
        let dot = px[(TOOLBAR_H + 20.0 + 15.0) as usize * w as usize + 250 + 12];
        assert!(near(dot, pal::TARGET_CONFIRMED, 30), "slot dot {:?}", rgb(dot));

        // click the "Global costmap" row: it turns on and the floor gets tinted
        let gi = scene::index(scene::Layer::GlobalCost);
        assert!(!app.layers.on[gi]);
        let row_y = row_center(&chrome, &mut app, &m, now, gi);
        click(&mut chrome, &mut app, &m, now, 23.0, f64::from(row_y));
        assert!(app.layers.on[gi], "checkbox click turned it on");
        click(&mut chrome, &mut app, &m, now, 200.0, f64::from(row_y));
        assert!(!app.layers.on[gi], "a click on the row turned it off again");
        click(&mut chrome, &mut app, &m, now, 200.0, f64::from(row_y));
        let px = draw(&mut chrome, &mut app, &m, w, h, now);
        assert!(app.layers.on[gi], "row click turned the layer on");
        snapshot(&px, w, h, "window_team7_costmap.png");
        let p = px[(sy + a.y) as usize * w as usize + (sx + a.x) as usize];
        assert!(!near(p, pal::FREE, 10), "tinted {:?}", rgb(p));

        // collapse the dock with D: the map takes the full width
        chrome.key(KeyCode::D, Modifiers::default());
        draw(&mut chrome, &mut app, &m, w, h, now);
        assert_eq!(chrome.map_area.x, 0.0);
    }

    /// y of a layer row's centre in the dock, found from the layout the same way the dock does
    fn row_center(chrome: &Chrome, _app: &mut App, m: &Model, _now: Instant, idx: usize) -> f32 {
        let mut y = chrome.layout.bodies[0].expect("layers open").y;
        for (i, l) in LAYERS.iter().enumerate() {
            if l.slot.is_some_and(|s| m.bound(s).is_none()) {
                continue;
            }
            if i == idx {
                return y + ROW_H / 2.0;
            }
            y += ROW_H + 2.0;
            if l.opacity.is_some() && chrome.sections[0] && _app.layers.on[i] {
                y += 24.0;
            }
        }
        panic!("layer not shown")
    }

    fn pointer(chrome: &mut Chrome, x: f64, y: f64) {
        chrome.handle(&WindowEvent::MouseMove { x, y, buttons: MouseButtons::default(), modifiers: Modifiers::default(), pointer: PointerKind::Mouse, time: Duration::ZERO });
    }

    fn button(chrome: &mut Chrome, pressed: bool, x: f64, y: f64) {
        chrome.handle(&WindowEvent::MouseButton {
            button: MouseButton::Left,
            pressed,
            modifiers: Modifiers::default(),
            x,
            y,
            click_count: 1,
            pointer: PointerKind::Mouse,
            time: Duration::ZERO,
        });
    }

    #[test]
    fn dragging_a_handle_moves_the_boundary_between_two_sections() {
        let now = Instant::now();
        let m = team7_model();
        let mut chrome = Chrome::new("91".into(), None);
        chrome.set_run_dir(None);
        chrome.sections = [true; 4];
        let mut app = App::new(false);
        let (w, h) = (1280u32, 900u32);
        draw(&mut chrome, &mut app, &m, w, h, now);
        let (a, b, r) = chrome.layout.handles[0];
        assert_eq!((a, b), (0, 1));
        let (ha, hb) = (chrome.layout.bodies[0].unwrap().height, chrome.layout.bodies[1].unwrap().height);
        let total: f32 = chrome.layout.bodies.iter().flatten().map(|b| b.height).sum();
        let (x, y) = (f64::from(r.x + 100.0), f64::from(r.y + SPLIT_H / 2.0));
        pointer(&mut chrome, x, y);
        draw(&mut chrome, &mut app, &m, w, h, now);
        button(&mut chrome, true, x, y);
        draw(&mut chrome, &mut app, &m, w, h, now);
        pointer(&mut chrome, x, y - 60.0);
        draw(&mut chrome, &mut app, &m, w, h, now);
        button(&mut chrome, false, x, y - 60.0);
        let px = draw(&mut chrome, &mut app, &m, w, h, now);
        snapshot(&px, w, h, "dock_dragged.png");
        let (na, nb) = (chrome.layout.bodies[0].unwrap().height, chrome.layout.bodies[1].unwrap().height);
        assert!((na - (ha - 60.0)).abs() < 1.5 && (nb - (hb + 60.0)).abs() < 1.5, "{ha} {hb} -> {na} {nb}");
        let total2: f32 = chrome.layout.bodies.iter().flatten().map(|b| b.height).sum();
        assert!((total - total2).abs() < 1.0);
        // dragged far: stops at the minimum
        let (_, _, r) = chrome.layout.handles[0];
        let (x, y) = (f64::from(r.x + 100.0), f64::from(r.y + SPLIT_H / 2.0));
        pointer(&mut chrome, x, y);
        draw(&mut chrome, &mut app, &m, w, h, now);
        button(&mut chrome, true, x, y);
        draw(&mut chrome, &mut app, &m, w, h, now);
        pointer(&mut chrome, x, y - 900.0);
        draw(&mut chrome, &mut app, &m, w, h, now);
        button(&mut chrome, false, x, y - 900.0);
        draw(&mut chrome, &mut app, &m, w, h, now);
        assert!((chrome.layout.bodies[0].unwrap().height - MIN_BODY).abs() < 1.0);
        // a folded section has no body and no handle
        chrome.sections[1] = false;
        draw(&mut chrome, &mut app, &m, w, h, now);
        assert!(chrome.layout.bodies[1].is_none());
        assert!(chrome.layout.handles.iter().all(|&(a, b, _)| a != 1 && b != 1));
    }

    #[test]
    fn the_wheel_scrolls_only_the_section_under_the_pointer() {
        let now = Instant::now();
        let mut m = team7_model();
        m.targets = (0..40).map(|i| crate::model::Target { id: i, x: f64::from(i), y: 1.0, confirmed: i % 2 == 0, views: None }).collect();
        let mut chrome = Chrome::new("91".into(), None);
        chrome.set_run_dir(None);
        let mut app = App::new(false);
        let (w, h) = (1280u32, 800u32);
        draw(&mut chrome, &mut app, &m, w, h, now);
        let body = chrome.layout.bodies[2].expect("targets open");
        let (x, y) = (f64::from(body.x + body.width / 2.0), f64::from(body.y + body.height / 2.0));
        pointer(&mut chrome, x, y);
        draw(&mut chrome, &mut app, &m, w, h, now);
        let mut ev = aurea::ScrollEvent::wheel(0.0, -3.0, 40.0, Modifiers::default());
        (ev.x, ev.y) = (x, y);
        for _ in 0..4 {
            chrome.handle(&WindowEvent::Scroll(ev));
            draw(&mut chrome, &mut app, &m, w, h, now);
        }
        let px = draw(&mut chrome, &mut app, &m, w, h, now);
        snapshot(&px, w, h, "dock_scrolled.png");
        assert!(chrome.layout.scrolls[2] > 20.0, "targets scrolled {:?}", chrome.layout.scrolls);
        assert_eq!(chrome.layout.scrolls[0], 0.0);
        assert_eq!(chrome.layout.scrolls[1], 0.0);
        // the sections below the long list keep their place
        assert!(chrome.layout.bodies[2].unwrap().bottom() <= h as f32 - STATUS_H + 1.0);

        // a scroll bar along the right edge of an overflowing section, none on one that fits
        let bar = |px: &[u32], b: Rect| {
            let (x0, x1) = ((b.right() - 10.0) as usize, (b.right() - 1.0) as usize);
            let mid = (b.y + b.height / 2.0) as usize;
            (mid.saturating_sub(10)..mid + 10).any(|y| (x0..x1).any(|x| rgb(px[y * w as usize + x]).iter().all(|&c| c > 60)))
        };
        assert!(bar(&px, chrome.layout.bodies[2].unwrap()), "bar on the long target list");
        m.targets.truncate(2);
        let mut chrome2 = Chrome::new("91".into(), None);
        chrome2.set_run_dir(None);
        // the first frame measures the content, the second draws with it
        draw(&mut chrome2, &mut app, &m, w, h, now);
        let px = draw(&mut chrome2, &mut app, &m, w, h, now);
        assert!(!bar(&px, chrome2.layout.bodies[2].unwrap()), "no bar when two targets fit");
    }

    /// cargo test --release frame_cost -- --ignored --nocapture: full window frames, recorded and
    /// rasterized, with a 500x500 camera image changing every frame
    #[test]
    #[ignore]
    fn frame_cost() {
        let mut m = team7_model();
        let mut chrome = Chrome::new("91".into(), None);
        chrome.set_run_dir(None);
        let mut app = App::new(false);
        let (w, h) = (1440u32, 870u32);
        let mut r = CpuRasterizer::new(w, h);
        let mut times = Vec::new();
        for i in 0..60u64 {
            let rgba = [(i * 4) as u8, 90, 160, 255].repeat(500 * 500);
            m.camera = Some(crate::model::CameraFrame { width: 500, height: 500, rgba, seq: i + 1, at: Instant::now() });
            let t = Instant::now();
            {
                let mut ctx = r.begin_frame().unwrap();
                chrome.frame(ctx.as_mut(), &mut app, &m, Instant::now()).unwrap();
            }
            r.end_frame().unwrap();
            times.push(t.elapsed().as_secs_f64() * 1000.0);
        }
        times.sort_by(f64::total_cmp);
        eprintln!("full frame {w}x{h}: p50 {:.1} ms p95 {:.1} ms", times[30], times[57]);
    }

    #[test]
    fn stock_sources_and_stale_banner() {
        let now = Instant::now();
        let mut m = team7_scene();
        m.graph = ["/map", "/plan", "/local_costmap/costmap", "/explore/frontiers", "/goal_pose", "/aruco_tracker/debug", "/global_costmap/costmap"].map(String::from).to_vec();
        // data stopped 90 s ago, mission was running
        for i in 0..20 {
            m.tick(Topic::Scan, now - Duration::from_secs(90) - Duration::from_millis(100 * i));
        }
        let mut chrome = Chrome::new("79".into(), None);
        chrome.set_run_dir(None);
        let mut app = App::new(true);
        let (w, h) = (1280u32, 800u32);
        let px = draw(&mut chrome, &mut app, &m, w, h, now);
        snapshot(&px, w, h, "window_stock_stale.png");
        assert_eq!(m.stack(), "stock");
        let (ros_now, _) = m.ros_now(now, 0.0);
        let (level, text) = health::summary(&m, now, ros_now);
        assert_eq!(level, Level::Bad);
        assert!(text.starts_with("no data for 1 min"), "{text}");
        // the clock stopped instead of going negative
        assert!(mission_clock(&m, now).1.starts_with("stopped"));
        // the banner tints the right side of the strip red
        let p = px[(TOOLBAR_H + 30.0) as usize * w as usize + (w as usize - 40)];
        let [r, g, b] = rgb(p);
        assert!(r > g + 20 && r > b + 20, "banner {:?}", rgb(p));
    }
}
