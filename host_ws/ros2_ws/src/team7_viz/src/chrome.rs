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
use aurea::ui::{Align, Button, Checkbox, Disclosure, Dropdown, ScrollArea, Sense, Slider, Splitter, Theme, UiState};
use tb4_draw::palette as pal;
use tb4_draw::prim;

use crate::app::{App, Tool};
use crate::health::{self, Level};
use crate::model::{Model, MsgKind, Slot, Target, human_age, mission_clock};
use crate::outputs;
use crate::ros::wall_secs;
use crate::scene::{self, Binding};

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
    Topic(u8),
    Remove(u8),
    AddLayer,
    CameraTopic,
    Overflow,
    Camera,
}

/// what the window must do after a frame
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Requests {
    pub screenshot: bool,
    /// camera frames are waiting to be shown: draw again soon
    pub more: bool,
}

pub struct Chrome {
    pub ui: UiState<Wid>,
    pub theme: Theme,
    /// the dock beside the map (wide windows); saved in the profile
    pub dock_open: bool,
    pub dock_w: f32,
    /// the dock as a drawer over the map (narrow windows)
    pub drawer_open: bool,
    pub sections: [bool; 4],
    /// the map's rectangle in the last frame, canvas pixels
    pub map_area: Rect,
    /// what covers part of the map (the drawer), so presses there are not the map's
    pub overlay: Option<Rect>,
    pub domain: String,
    /// the active profile's name, shown in the toolbar
    pub profile: Option<String>,
    pub layout: DockLayout,
    /// canvas width of the last frame (the dock key acts on the drawer when narrow)
    last_w: f32,
    out_dir: Option<PathBuf>,
    fixed_out_dir: bool,
    out_checked: Option<Instant>,
    camera_image: Option<(u64, bool, Image)>,
    /// the last camera frame drawn (its seq)
    camera_shown: u64,
}

/// under this width the dock becomes a drawer over the map
pub const NARROW: f32 = 900.0;
/// under this width the drawer takes the whole width
pub const SHEET: f32 = 520.0;
/// the map keeps at least this much beside the dock
const MAP_MIN: f32 = 360.0;
const DOCK_MIN: f32 = 240.0;
const DOCK_MAX: f32 = 560.0;

thread_local! {
    /// every layout rectangle of the last frame, named, for the size tests
    static LAYOUT: std::cell::RefCell<Vec<(&'static str, Rect)>> = const { std::cell::RefCell::new(Vec::new()) };
    /// full text of what the pointer is over, shown in the status bar
    static HOVER: std::cell::RefCell<Option<String>> = const { std::cell::RefCell::new(None) };
}

/// a layout rectangle: never negative, and noted for the size tests
fn note(name: &'static str, r: Rect) -> Rect {
    let r = Rect::new(r.x, r.y, r.width.max(0.0), r.height.max(0.0));
    LAYOUT.with(|l| l.borrow_mut().push((name, r)));
    r
}

/// the layout rectangles of the last frame
pub fn layout_rects() -> Vec<(&'static str, Rect)> {
    LAYOUT.with(|l| l.borrow().clone())
}

fn hover(text: impl Into<String>) {
    HOVER.with(|h| *h.borrow_mut() = Some(text.into()));
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
            dock_w: 340.0,
            drawer_open: false,
            sections: [true, true, true, false],
            map_area: Rect::new(0.0, 0.0, 1.0, 1.0),
            overlay: None,
            domain,
            profile: None,
            layout: DockLayout::default(),
            last_w: 1440.0,
            fixed_out_dir: out_dir.is_some(),
            out_dir,
            out_checked: None,
            camera_image: None,
            camera_shown: 0,
        }
    }

    /// the panel toggle: the dock when wide, the drawer when narrow
    pub fn toggle_panel(&mut self) {
        if self.last_w < NARROW {
            self.drawer_open = !self.drawer_open;
        } else {
            self.dock_open = !self.dock_open;
        }
    }

    /// whether the panel shows, as the window is now
    pub fn panel_shown(&self) -> bool {
        if self.last_w < NARROW { self.drawer_open } else { self.dock_open }
    }

    /// keys for the chrome itself; true when used
    pub fn key(&mut self, key: KeyCode, mods: Modifiers) -> bool {
        match key {
            KeyCode::D if !mods.ctrl => self.toggle_panel(),
            KeyCode::H if !mods.ctrl => self.sections[3] = !self.sections[3],
            _ => return false,
        }
        true
    }

    /// a canvas point the map takes (inside the map, not under the drawer)
    pub fn map_hit(&self, x: f32, y: f32) -> bool {
        let p = aurea::render::Point::new(x, y);
        self.map_area.contains(p) && self.overlay.is_none_or(|o| !o.contains(p))
    }

    /// the run folder: re-checked every few seconds so a newer run is picked up
    fn run_dir(&mut self, now: Instant) -> Option<PathBuf> {
        if !self.fixed_out_dir && self.out_checked.is_none_or(|t| now.saturating_duration_since(t) >= OUT_EVERY) {
            self.out_dir = outputs::newest_run_dir(&outputs::runs_root());
            self.out_checked = Some(now);
        }
        self.out_dir.clone()
    }

    /// the seq of the last camera frame drawn
    pub fn camera_shown(&self) -> u64 {
        self.camera_shown
    }

    /// the run folder found last
    pub fn known_run_dir(&self) -> Option<&std::path::Path> {
        self.out_dir.as_deref()
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
        LAYOUT.with(|l| l.borrow_mut().clear());
        HOVER.with(|v| *v.borrow_mut() = None);
        self.last_w = w;
        let mut req = Requests::default();
        let run_dir = self.run_dir(now);
        let theme = self.theme.clone();
        ctx.clear(theme.palette.background)?;
        let narrow = w < NARROW;
        let Self { ui: state, dock_open, dock_w, drawer_open, sections, map_area, overlay, domain, layout, camera_image, camera_shown, profile, .. } = self;
        let mut ui = state.frame(ctx, &theme);

        let body = note("body", Rect::new(0.0, TOOLBAR_H + STRIP_H, w, (h - TOOLBAR_H - STRIP_H - STATUS_H).max(0.0)));
        // wide: the dock beside the map, its width dragged between limits; narrow: the map takes
        // the whole body and the dock is a drawer over it
        let (dock, map) = if !narrow && *dock_open {
            let mut at = dock_w.clamp(DOCK_MIN, DOCK_MAX);
            let (a, b, r) = Splitter::new(&mut at).min(DOCK_MIN, MAP_MIN).show(&mut ui, Wid::Split, body)?;
            if r.held {
                *dock_w = at.clamp(DOCK_MIN, DOCK_MAX);
            }
            (Some(note("dock", a)), note("map", b))
        } else {
            (None, note("map", body))
        };
        *map_area = map;
        *overlay = None;
        // the map first, so the chrome draws over its edges
        app.draw_in(&mut *ui.ctx, model, now, map)?;

        let panel_on = if narrow { *drawer_open } else { *dock_open };
        let mut toggle = false;
        toolbar(&mut ui, app, panel_on, &mut toggle, &mut req, profile.as_deref(), w)?;
        if toggle {
            if narrow {
                *drawer_open = !*drawer_open;
            } else {
                *dock_open = !*dock_open;
            }
        }
        let found = app.targets(model);
        strip(&mut ui, model, &found, now, w)?;
        let drawer = (narrow && *drawer_open).then(|| {
            let dw = if w < SHEET { w } else { dock_w.clamp(DOCK_MIN, DOCK_MAX).min(w * 0.85) };
            note("drawer", Rect::new(0.0, body.y, dw, body.height))
        });
        if let Some(d) = dock.or(drawer) {
            if drawer.is_some() {
                *overlay = Some(d);
                ui.shadow(d, 0.0, 18.0, theme.palette.shadow)?;
            }
            ui.fill(d, theme.palette.surface)?;
            // the next camera frame: each one is drawn once even when several came at once
            let (cam, more) = model.next_camera(*camera_shown);
            dock_sections(&mut ui, d, layout, sections, camera_image, cam, app, model, now, run_dir.as_deref())?;
            if sections[1]
                && let Some(f) = cam
            {
                *camera_shown = f.seq;
                req.more = more;
            }
            ui.line(aurea::render::Point::new(d.right(), d.y), aurea::render::Point::new(d.right(), d.bottom()), 1.0, theme.palette.border)?;
        }
        status_bar(&mut ui, app, model, now, domain, note("status", Rect::new(0.0, h - STATUS_H, w, STATUS_H)))?;
        Ok(req)
    }

    /// pass a window event (already in canvas pixels) to the widgets; true when they want a redraw
    pub fn handle(&mut self, event: &WindowEvent) -> bool {
        self.ui.handle(event)
    }
}

/// how a toolbar group shows
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Look {
    Text,
    Icon,
    /// in the overflow menu
    Menu,
}

/// what fits in the toolbar at a width: groups collapse to icons, then into the overflow menu,
/// then the profile label goes, then the badge and the panel button shrink
#[derive(Debug, Clone, Copy, PartialEq)]
struct ToolbarPlan {
    tools: Look,
    view: Look,
    profile: bool,
    short_badge: bool,
    panel_icon: bool,
}

const BTN_H: f32 = 28.0;
const ICON_W: f32 = 32.0;
const GAP: f32 = 4.0;
const SEP: f32 = 16.0;

fn plan_toolbar(w: f32, profile_w: f32, badge_w: f32) -> ToolbarPlan {
    let tools = |l: Look| if l == Look::Text { 3.0 * (78.0 + GAP) } else { 3.0 * (ICON_W + GAP) };
    let view = |l: Look| match l {
        Look::Text => 30.0 + 30.0 + 44.0 + 62.0 + 72.0 + 92.0 + 6.0 * GAP + SEP,
        Look::Icon => 6.0 * (ICON_W + GAP) + SEP,
        Look::Menu => 40.0 + GAP,
    };
    let right = |p: &ToolbarPlan| {
        (if p.panel_icon { ICON_W } else { 64.0 }) + 12.0 + (if p.short_badge { 34.0 } else { badge_w }) + if p.profile { profile_w + 12.0 } else { 0.0 }
    };
    let plans = [
        ToolbarPlan { tools: Look::Text, view: Look::Text, profile: true, short_badge: false, panel_icon: false },
        ToolbarPlan { tools: Look::Text, view: Look::Icon, profile: true, short_badge: false, panel_icon: false },
        ToolbarPlan { tools: Look::Icon, view: Look::Icon, profile: true, short_badge: false, panel_icon: false },
        ToolbarPlan { tools: Look::Icon, view: Look::Icon, profile: false, short_badge: false, panel_icon: false },
        ToolbarPlan { tools: Look::Icon, view: Look::Menu, profile: false, short_badge: false, panel_icon: false },
        ToolbarPlan { tools: Look::Icon, view: Look::Menu, profile: false, short_badge: true, panel_icon: true },
    ];
    plans.into_iter().find(|p| 8.0 + tools(p.tools) + SEP + view(p.view) + right(p) + 8.0 <= w).unwrap_or(plans[5])
}

#[allow(clippy::too_many_arguments)]
fn toolbar(ui: &mut aurea::ui::Ui<'_, Wid>, app: &mut App, panel_on: bool, toggle: &mut bool, req: &mut Requests, profile: Option<&str>, w: f32) -> AureaResult<()> {
    use aurea::render::Icon;
    let t = ui.theme.clone();
    ui.fill(note("toolbar", Rect::new(0.0, 0.0, w, TOOLBAR_H)), t.palette.surface)?;
    let (badge, color) = if app.allow_goals { ("GOALS ON", t.palette.warning) } else { ("READ-ONLY", t.palette.dim) };
    let label = profile.map_or_else(|| "profile: detecting".to_owned(), |p| format!("profile {p}"));
    let profile_w = ui.text_width(&label, &font(11.0));
    let badge_w = ui.text_width(badge, &bold(11.0)) + 16.0;
    let plan = plan_toolbar(w, profile_w, badge_w);
    let y = (TOOLBAR_H - BTN_H) / 2.0;
    let mut x = 8.0;
    let sep = |ui: &mut aurea::ui::Ui<'_, Wid>, x: f32| ui.line(aurea::render::Point::new(x, 10.0), aurea::render::Point::new(x, TOOLBAR_H - 10.0), 1.0, t.palette.border);

    // tools: the active one is highlighted
    for (i, (tool, text, icon, tip)) in [
        (Tool::Interact, "Interact", Icon::Hand, "Interact: drag pans the map (I)"),
        (Tool::Measure, "Measure", Icon::Crop, "Measure: drag for distance and angle (M)"),
        (Tool::Goal, "2D Goal", Icon::Arrow, "2D Goal: drag position, then heading (G)"),
    ]
    .iter()
    .enumerate()
    {
        let (b, bw) = if plan.tools == Look::Text { (Button::new(text), 78.0) } else { (Button::icon_only(*icon), ICON_W) };
        let r = b.selected(app.tool == *tool).show(ui, Wid::Tool(i as u8), note("tool", Rect::new(x, y, bw, BTN_H)))?;
        if r.hovered {
            hover(*tip);
        }
        if r.clicked {
            app.set_tool(*tool);
        }
        x += bw + GAP;
    }
    x += SEP / 2.0;
    sep(ui, x - GAP / 2.0)?;
    x += SEP / 2.0 - GAP;

    // view and capture: buttons, icons, or one overflow menu
    let follow_label = if app.camera.follow { "Follow robot: on" } else { "Follow robot: off" };
    let actions: [(Wid, &str, Icon, f32, &str, bool); 6] = [
        (Wid::ZoomOut, "-", Icon::ZoomOut, 30.0, "Zoom out (-)", false),
        (Wid::ZoomIn, "+", Icon::ZoomIn, 30.0, "Zoom in (+)", false),
        (Wid::Fit, "Fit", Icon::Zoom, 44.0, "Fit the map, north up (0)", false),
        (Wid::Follow, "Follow", Icon::Eye, 62.0, follow_label, app.camera.follow),
        (Wid::North, "North up", Icon::ChevronUp, 72.0, "North up", app.camera.rotation == 0.0),
        (Wid::Shot, "Screenshot", Icon::File, 92.0, "Screenshot of the map (P)", false),
    ];
    let mut picked = None;
    match plan.view {
        Look::Menu => {
            let labels: Vec<&str> = actions.iter().map(|a| if a.0 == Wid::Follow { follow_label } else { a.4 }).collect();
            let mut sel = labels.len();
            Dropdown::new(&labels, &mut sel).placeholder("...").show(ui, Wid::Overflow, note("overflow", Rect::new(x, y, 40.0, BTN_H)))?;
            picked = actions.get(sel).map(|a| a.0);
        }
        look => {
            for (i, (id, text, icon, tw, tip, sel)) in actions.iter().enumerate() {
                if *id == Wid::Shot {
                    x += SEP / 2.0;
                    sep(ui, x - GAP / 2.0)?;
                    x += SEP / 2.0 - GAP;
                }
                let (b, bw) = if look == Look::Text { (Button::new(text), *tw) } else { (Button::icon_only(*icon), ICON_W) };
                let r = b.selected(*sel).show(ui, *id, note("view", Rect::new(x, y, bw, BTN_H)))?;
                if r.hovered {
                    hover(*tip);
                }
                if r.clicked {
                    picked = Some(*id);
                }
                x += bw + GAP;
                let _ = i;
            }
        }
    }
    match picked {
        Some(Wid::ZoomOut) => app.zoom(1.0 / 1.2),
        Some(Wid::ZoomIn) => app.zoom(1.2),
        Some(Wid::Fit) => app.fit(),
        Some(Wid::Follow) => app.camera.follow = !app.camera.follow,
        Some(Wid::North) => {
            let r = app.camera.rotation;
            app.rotate(-r);
        }
        Some(Wid::Shot) => req.screenshot = true,
        _ => {}
    }

    // right side, from the edge: panel toggle, mode badge, profile
    let mut rx = w - 8.0;
    let pw = if plan.panel_icon { ICON_W } else { 64.0 };
    rx -= pw;
    let b = if plan.panel_icon { Button::icon_only(Icon::Layers) } else { Button::new("Panel") };
    let r = b.selected(panel_on).show(ui, Wid::Dock, note("panel", Rect::new(rx, y, pw, BTN_H)))?;
    if r.hovered {
        hover("Panel: layers, camera, targets, health (D)");
    }
    if r.clicked {
        *toggle = true;
    }
    let shown = if plan.short_badge { if app.allow_goals { "GO" } else { "RO" } } else { badge };
    let f = bold(11.0);
    let bw = if plan.short_badge { 34.0 } else { badge_w };
    rx -= bw + 12.0;
    let br = note("badge", Rect::new(rx, (TOOLBAR_H - 20.0) / 2.0, bw, 20.0));
    ui.border(br, 10.0, 1.0, color)?;
    ui.text(shown, br, &f, color, Align::Center)?;
    if ui.pointer().is_some_and(|p| br.contains(p)) {
        hover(if app.allow_goals { "GOALS ON: the 2D Goal tool sends NavigateToPose" } else { "READ-ONLY: the viewer sends nothing (start with --allow-goals)" });
    }
    if plan.profile && rx - profile_w - 12.0 > x {
        rx -= profile_w + 12.0;
        ui.text(&label, note("profile", Rect::new(rx, (TOOLBAR_H - 16.0) / 2.0, profile_w, 16.0)), &font(11.0), t.palette.dim, Align::Start)?;
    }
    Ok(())
}

/// how the targets show in the strip at a width
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum SlotLook {
    Full,
    Compact,
    Chip,
    Hidden,
}

/// the header strip: mission clock (always), the 3 targets as slots, small chips or one count, and
/// one status light (or the stale banner), shortened to the room left
fn strip(ui: &mut aurea::ui::Ui<'_, Wid>, model: &Model, found: &[Target], now: Instant, w: f32) -> AureaResult<()> {
    let t = ui.theme.clone();
    let r = note("strip", Rect::new(0.0, TOOLBAR_H, w, STRIP_H));
    let stale = model.stale_for(now);
    ui.fill(r, t.palette.raised)?;
    ui.line(aurea::render::Point::new(0.0, r.bottom() - 0.5), aurea::render::Point::new(w, r.bottom() - 0.5), 1.0, t.palette.border)?;

    // clock: the big time always; its note beside it when there is room
    let (big, small) = mission_clock(model, now);
    let big_w = ui.text_width(&big, &bold(22.0));
    let small_w = ui.text_width(&small, &font(11.0));
    let with_note = w >= 700.0;
    let clock_w = big_w + if with_note { 8.0 + small_w } else { 0.0 };
    let x = PAD;
    ui.text("MISSION", note("clock", Rect::new(x, r.y + 8.0, clock_w.max(60.0), 12.0)), &bold(10.0), t.palette.dim, Align::Start)?;
    ui.text(&big, Rect::new(x, r.y + 20.0, big_w + 4.0, 24.0), &bold(22.0), t.palette.text, Align::Start)?;
    if with_note {
        ui.text(&small, Rect::new(x + big_w + 8.0, r.y + 28.0, small_w + 4.0, 14.0), &font(11.0), t.palette.dim, Align::Start)?;
    } else if ui.pointer().is_some_and(|p| Rect::new(x, r.y, big_w, STRIP_H).contains(p)) {
        hover(format!("mission {big} {small}"));
    }

    // targets: what fits between the clock and a minimal status
    let mut targets: Vec<_> = found.iter().collect();
    targets.sort_by_key(|t| (!t.confirmed, t.id));
    let status_min = if w >= SHEET { 140.0 } else { 100.0 };
    let tx = x + clock_w + 24.0;
    let room = w - tx - status_min - 2.0 * PAD;
    let look = if room >= 3.0 * 158.0 {
        SlotLook::Full
    } else if room >= 3.0 * 72.0 {
        SlotLook::Compact
    } else if room >= 92.0 {
        SlotLook::Chip
    } else {
        SlotLook::Hidden
    };
    let mut x = tx;
    if look != SlotLook::Hidden {
        ui.text("TARGETS", note("targets", Rect::new(x, r.y + 8.0, 80.0, 12.0)), &bold(10.0), t.palette.dim, Align::Start)?;
    }
    let full_text = |tg: &Target| {
        let state = if tg.confirmed { "confirmed".to_owned() } else { tg.views.map_or("tentative".to_owned(), |v| format!("tentative, {v} views")) };
        format!("ID {} at ({:.2}, {:.2}) {state}", tg.id, tg.x, tg.y)
    };
    let dot = |tg: &Target| rgba(if tg.confirmed { pal::TARGET_CONFIRMED } else { pal::TARGET_TENTATIVE });
    match look {
        SlotLook::Full | SlotLook::Compact => {
            let slot_w = if look == SlotLook::Full { 150.0 } else { 64.0 };
            for i in 0..3 {
                let sr = note("slot", Rect::new(x, r.y + 22.0, slot_w, 28.0));
                ui.round(sr, 6.0, t.palette.surface)?;
                match targets.get(i) {
                    Some(tg) => {
                        ui.circle(aurea::render::Point::new(sr.x + 12.0, sr.y + 14.0), 5.0, dot(tg))?;
                        let id = if tg.confirmed { format!("ID {}", tg.id) } else { format!("ID {}?", tg.id) };
                        if look == SlotLook::Full {
                            ui.text_clipped(&id, Rect::new(sr.x + 24.0, sr.y + 1.0, slot_w - 28.0, 14.0), &bold(12.0), t.palette.text, Align::Start)?;
                            let sub = match (tg.confirmed, tg.views) {
                                (false, Some(v)) => format!("({:.2}, {:.2}) {v}v", tg.x, tg.y),
                                _ => format!("({:.2}, {:.2})", tg.x, tg.y),
                            };
                            ui.text_clipped(&sub, Rect::new(sr.x + 24.0, sr.y + 14.0, slot_w - 28.0, 13.0), &font(11.0), t.palette.dim, Align::Start)?;
                        } else {
                            ui.text_clipped(&id, Rect::new(sr.x + 22.0, sr.y + 7.0, slot_w - 24.0, 14.0), &bold(12.0), t.palette.text, Align::Start)?;
                        }
                        if ui.pointer().is_some_and(|p| sr.contains(p)) {
                            hover(full_text(tg));
                        }
                    }
                    None => {
                        let empty = if look == SlotLook::Full { format!("target {}", i + 1) } else { "-".to_owned() };
                        ui.text_clipped(&empty, Rect::new(sr.x + 12.0, sr.y + 7.0, slot_w - 16.0, 14.0), &font(12.0), t.palette.faint, Align::Start)?;
                    }
                }
                x += slot_w + 8.0;
            }
            if targets.len() > 3 {
                ui.text(&format!("+{}", targets.len() - 3), note("more", Rect::new(x, r.y + 29.0, 28.0, 14.0)), &font(11.0), t.palette.dim, Align::Start)?;
                x += 32.0;
            }
        }
        SlotLook::Chip => {
            let confirmed = targets.iter().filter(|t| t.confirmed).count();
            let chip = note("chip", Rect::new(x, r.y + 22.0, 92.0, 28.0));
            ui.round(chip, 14.0, t.palette.surface)?;
            let c = if confirmed > 0 { rgba(pal::TARGET_CONFIRMED) } else if targets.is_empty() { t.palette.faint } else { rgba(pal::TARGET_TENTATIVE) };
            ui.circle(aurea::render::Point::new(chip.x + 14.0, chip.y + 14.0), 5.0, c)?;
            ui.text(&format!("{confirmed}/3 found"), Rect::new(chip.x + 24.0, chip.y + 7.0, 64.0, 14.0), &bold(12.0), t.palette.text, Align::Start)?;
            if ui.pointer().is_some_and(|p| chip.contains(p)) {
                hover(if targets.is_empty() { "no targets yet".to_owned() } else { targets.iter().map(|t| full_text(t)).collect::<Vec<_>>().join("; ") });
            }
            x += 92.0 + 8.0;
        }
        SlotLook::Hidden => {}
    }

    // status light, or the banner when nothing comes; shortened to the room
    let sx = (x + 8.0).max(w - 360.0).min(w - PAD - status_min).max(x);
    let sr = note("status_light", Rect::new(sx, r.y + 14.0, (w - sx - PAD).max(0.0), 30.0));
    let (ros_now, _) = model.ros_now(now, wall_secs());
    let (level, text) = health::summary(model, now, ros_now);
    let short = match stale {
        Some(d) => format!("no data {}", human_age(d)),
        None => text.split(',').next().unwrap_or(&text).to_owned(),
    };
    if stale.is_some() && sr.width > 0.0 {
        ui.round(sr, 6.0, t.palette.danger.with_alpha(60))?;
        ui.border(sr, 6.0, 1.0, t.palette.danger)?;
    }
    if sr.width >= 20.0 {
        ui.circle(aurea::render::Point::new(sr.x + 14.0, sr.y + 15.0), 6.0, level_color(&t, level))?;
    }
    let f = if stale.is_some() { bold(12.0) } else { font(12.0) };
    let tr = Rect::new(sr.x + 26.0, sr.y + 7.0, (sr.width - 32.0).max(0.0), 16.0);
    let shown = if ui.text_width(&text, &f) <= tr.width { &text } else { &short };
    if tr.width > 8.0 {
        ui.text_clipped(shown, tr, &f, t.palette.text, Align::Start)?;
    }
    if ui.pointer().is_some_and(|p| sr.contains(p)) {
        hover(text.clone());
    }
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
    cam: Option<&crate::model::CameraFrame>,
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
                    1 => camera(ui, camera_image, cam, app, model, now, cx, c.y, cw, body.height - 28.0)?,
                    2 => targets(ui, model, &app.targets(model), cx, c.y, cw, run_dir)?,
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

/// the choices of a topic picker: "Auto" first, then the graph's topics of the message type
/// (publishers first), and a pinned topic that is not in the graph at the end
pub struct TopicChoices {
    pub labels: Vec<String>,
    /// topic per label; None for Auto
    pub topics: Vec<Option<String>>,
    pub selected: usize,
}

pub fn topic_choices(model: &Model, kind: MsgKind, binding: &Binding, auto_topic: Option<&str>, now: Instant) -> TopicChoices {
    let auto = match auto_topic {
        Some(t) => format!("Auto  {t}"),
        None => "Auto  (none)".to_owned(),
    };
    let mut labels = vec![auto];
    let mut topics = vec![None];
    for g in model.topics_of(kind) {
        let hz = model.topic_hz(&g.name, now).filter(|h| *h > 0.0).map_or(String::new(), |h| format!(" {h:.0}Hz"));
        labels.push(format!("{}  {}p{hz}", g.name, g.publishers));
        topics.push(Some(g.name.clone()));
    }
    let selected = match binding {
        Binding::Auto => 0,
        Binding::Pinned(t) => match topics.iter().position(|o| o.as_deref() == Some(t)) {
            Some(i) => i,
            None => {
                labels.push(format!("{t}  no publisher"));
                topics.push(Some(t.clone()));
                topics.len() - 1
            }
        },
    };
    TopicChoices { labels, topics, selected }
}

/// the binding a picked choice means
pub fn binding_of(choices: &TopicChoices, picked: usize) -> Binding {
    match choices.topics.get(picked).cloned().flatten() {
        Some(t) => Binding::Pinned(t),
        None => Binding::Auto,
    }
}

fn layers(ui: &mut aurea::ui::Ui<'_, Wid>, app: &mut App, model: &Model, x: f32, mut y: f32, w: f32) -> AureaResult<f32> {
    let t = ui.theme.clone();
    let now = Instant::now();
    let mut missing = Vec::new();
    let mut remove = None;
    for i in 0..app.layers.list.len() {
        let spec = app.layers.list[i].clone();
        let info = spec.info();
        let topic = spec.topic(model);
        // a base layer with no source and nothing of its type to pick is only listed by name
        if let (Some(_), Some(kind)) = (info.slot, info.msg)
            && topic.is_none()
            && !spec.extra
            && model.topics_of(kind).is_empty()
        {
            missing.push(info.name);
            continue;
        }
        let row = Rect::new(x, y, w, ROW_H);
        // the whole row toggles; the checkbox takes the keyboard
        let input = ui.interact(Wid::Row(i as u8), 0, Rect::new(x, y, 150.0f32.min(w * 0.45), ROW_H), Sense::Click, true);
        if input.hovered {
            ui.round(row, 5.0, t.palette.hover)?;
        }
        // the full name and topic in the status bar, whatever the row had to cut
        if ui.pointer().is_some_and(|p| row.contains(p)) {
            let bound = match (&spec.binding, topic.as_deref()) {
                (Binding::Pinned(p), _) => format!("pinned {p}"),
                (Binding::Auto, Some(t)) => format!("auto: {t}"),
                (Binding::Auto, None) => "auto: nothing published".to_owned(),
            };
            hover(format!("{}: {bound}", info.name));
        }
        let mut on = spec.on;
        let check = Checkbox::new(&mut on).show(ui, Wid::Check(i as u8), Rect::new(x + 6.0, y + 3.5, 18.0, 18.0))?;
        if input.clicked && !check.clicked {
            on = !on;
        }
        app.layers.list[i].on = on;
        let mut symbol = scene::symbol(info.layer, (x + 44.0, y + ROW_H / 2.0));
        if spec.extra {
            let c = app.layers.extra_color(i);
            for p in &mut symbol {
                recolor(p, c);
            }
        }
        tb4_draw::paint_prims(ui.ctx, &symbol)?;
        let name_c = if on { t.palette.text } else { t.palette.dim };
        let name_x = x + 62.0;
        // names get half the row; the topic picker the rest
        let name_w = (w * 0.5 - 62.0).clamp(40.0, 130.0);
        ui.text_clipped(info.name, Rect::new(name_x, y + 4.0, name_w, 16.0), &font(13.0), name_c, Align::Start)?;

        // the topic picker, or the legend for layers without a topic
        let right = x + w - 22.0;
        let fx = name_x + name_w + 4.0;
        if let Some(kind) = info.msg {
            let auto = info.slot.and_then(|s| model.bound(s));
            let choices = topic_choices(model, kind, &spec.binding, auto, now);
            let labels: Vec<&str> = choices.labels.iter().map(String::as_str).collect();
            let mut sel = choices.selected;
            let field = Rect::new(fx + 10.0, y + 2.5, (right - fx - 10.0).max(30.0), ROW_H - 5.0);
            Dropdown::new(&labels, &mut sel).show(ui, Wid::Topic(i as u8), field)?;
            if sel != choices.selected {
                app.layers.list[i].binding = binding_of(&choices, sel);
            }
            // a pinned topic nobody publishes gets a small amber dot
            if let Binding::Pinned(p) = &spec.binding
                && model.publishers(p).unwrap_or(0) == 0
            {
                ui.circle(aurea::render::Point::new(fx + 4.0, y + ROW_H / 2.0), 3.5, t.palette.warning)?;
            }
        } else {
            ui.text_clipped(info.legend, Rect::new(fx, y + 6.0, (right - fx).max(0.0), 13.0), &font(10.0), t.palette.faint, Align::End)?;
        }
        if spec.extra {
            if Button::new("x").quiet().show(ui, Wid::Remove(i as u8), Rect::new(x + w - 20.0, y + 4.0, 18.0, 17.0))?.clicked {
                remove = Some(i);
            }
        } else if i < 9 {
            ui.text(&format!("{}", i + 1), Rect::new(x + w - 16.0, y + 5.0, 12.0, 14.0), &font(11.0), t.palette.faint, Align::End)?;
        }
        y += ROW_H + 2.0;
        if let (Some(_), true) = (info.opacity, on) {
            let mut o = spec.opacity;
            let pct = format!("{:.0}%", o * 100.0);
            ui.text("opacity", Rect::new(x + 62.0, y + 2.0, 60.0, 14.0), &font(11.0), t.palette.dim, Align::Start)?;
            Slider::new(&mut o, 0.0..=1.0).readout(&pct).show(ui, Wid::Opacity(i as u8), Rect::new(x + 118.0, y, (w - 126.0).max(40.0), 20.0))?;
            app.layers.list[i].opacity = o;
            y += 24.0;
        }
    }
    if let Some(i) = remove {
        app.layers.list.remove(i);
    }
    // a second layer of a kind: two paths, a filtered scan, another map
    let kinds: Vec<&str> = scene::EXTRA_KINDS.iter().map(|&k| scene::info_of(k).name).collect();
    let mut pick = kinds.len();
    Dropdown::new(&kinds, &mut pick).placeholder("+ add layer").show(ui, Wid::AddLayer, Rect::new(x + 6.0, y + 3.0, 140.0, ROW_H - 4.0))?;
    if let Some(&kind) = scene::EXTRA_KINDS.get(pick) {
        // bound to the first topic of its type that is not already shown
        let msg = scene::info_of(kind).msg.expect("extra kinds have a topic");
        let used: Vec<String> = app.layers.list.iter().filter(|s| s.info().msg == Some(msg)).filter_map(|s| s.topic(model)).collect();
        let topic = model.topics_of(msg).iter().find(|g| !used.contains(&g.name)).map(|g| g.name.clone());
        app.layers.add(kind, topic);
    }
    y += ROW_H + 2.0;
    if !missing.is_empty() {
        let text = format!("Not published: {}", missing.join(", "));
        let used = ui.text_wrapped(&text, Rect::new(x + 6.0, y + 2.0, w - 12.0, 200.0), &font(11.0), t.palette.faint, 14.0)?;
        y += used + 4.0;
    }
    Ok(y)
}

/// give a legend symbol an added layer's colour
fn recolor(p: &mut prim::Prim, c: tb4_draw::Rgba) {
    match p {
        prim::Prim::Line { color, .. } | prim::Prim::Polyline { color, .. } | prim::Prim::Points { color, .. } => *color = c,
        prim::Prim::Circle { fill: Some(f), .. } => *f = c,
        _ => {}
    }
}

#[allow(clippy::too_many_arguments)]
fn targets(ui: &mut aurea::ui::Ui<'_, Wid>, model: &Model, found: &[Target], x: f32, mut y: f32, w: f32, run_dir: Option<&std::path::Path>) -> AureaResult<f32> {
    let t = ui.theme.clone();
    // columns scale with the dock; state carries the view count ("tentative, 2 views")
    let col = |f: f32| x + 6.0 + (w - 12.0) * f;
    let cols = [(0.0, "ID"), (0.16, "x [m]"), (0.36, "y [m]"), (0.56, "state")];
    for (f, head) in cols {
        ui.text(head, Rect::new(col(f), y, 70.0, 14.0), &bold(10.0), t.palette.dim, Align::Start)?;
    }
    y += 18.0;
    if found.is_empty() {
        ui.text("no targets yet", Rect::new(x + 6.0, y, w, 16.0), &font(12.0), t.palette.faint, Align::Start)?;
        y += 20.0;
    }
    for tg in found {
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
fn camera(ui: &mut aurea::ui::Ui<'_, Wid>, cache: &mut Option<(u64, bool, Image)>, cam: Option<&crate::model::CameraFrame>, app: &mut App, model: &Model, now: Instant, x: f32, mut y: f32, w: f32, max_h: f32) -> AureaResult<f32> {
    let t = ui.theme.clone();
    // the image topic, picked like a layer's
    let choices = topic_choices(model, MsgKind::Image, &app.layers.camera, model.bound(Slot::Camera), now);
    let labels: Vec<&str> = choices.labels.iter().map(String::as_str).collect();
    let mut sel = choices.selected;
    Dropdown::new(&labels, &mut sel).show(ui, Wid::CameraTopic, Rect::new(x + 6.0, y, (w - 12.0).max(40.0), ROW_H - 4.0))?;
    if sel != choices.selected {
        app.layers.camera = binding_of(&choices, sel);
    }
    y += ROW_H;
    let max_h = max_h - ROW_H;
    let topic = app.layers.camera_topic(model);
    // the queued frame of the camera topic; right after a topic change, that topic's latest
    let queued = cam.filter(|_| topic == model.camera_topic);
    let Some(f) = queued.or_else(|| topic.as_deref().and_then(|tp| model.image(tp))) else {
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
    let info = format!("{}  {}x{}  {} ago", topic.as_deref().unwrap_or(""), f.width, f.height, human_age(age));
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

/// the status bar: fields by priority (time, cursor, domain, frame, slow draws), each kept only if
/// it fits; on the right the full text of what the pointer is over, a message, or the hints
fn status_bar(ui: &mut aurea::ui::Ui<'_, Wid>, app: &App, model: &Model, now: Instant, domain: &str, r: Rect) -> AureaResult<()> {
    let t = ui.theme.clone();
    ui.fill(r, t.palette.surface)?;
    let f = font(11.0);
    let (ros, sim) = model.ros_now(now, wall_secs());
    // (priority, text): lower numbers stay longest; shown in this order
    let mut fields: Vec<(u8, String)> = vec![(0, if sim { format!("sim {ros:.1} s") } else { "wall clock".to_owned() }), (2, format!("ROS_DOMAIN_ID {domain}")), (3, "frame map".into())];
    if let Some((x, y)) = app.cursor {
        fields.push((1, format!("x {x:.2}  y {y:.2}")));
    }
    if app.draw_time() > 0.04 {
        fields.push((4, format!("map draw {:.0} ms", app.draw_time() * 1000.0)));
    }
    let gap = 16.0;
    let room = r.width - 2.0 * PAD;
    let mut keep: Vec<(u8, String, f32)> = Vec::new();
    let mut used = 0.0;
    let mut by_priority = fields.clone();
    by_priority.sort_by_key(|(p, _)| *p);
    for (p, s) in by_priority {
        let fw = ui.text_width(&s, &f);
        if used + fw + gap <= room * 0.6 || keep.is_empty() {
            used += fw + gap;
            keep.push((p, s, fw));
        }
    }
    let mut x = PAD;
    for (_, s) in &fields {
        if let Some((_, _, fw)) = keep.iter().find(|(_, k, _)| k == s) {
            let tr = Rect::new(x, r.y + 5.0, fw.min(r.right() - x - PAD).max(0.0), 14.0);
            ui.text_clipped(s, tr, &f, t.palette.dim, Align::Start)?;
            x += fw + gap;
        }
    }
    let over = HOVER.with(|h| h.borrow().clone());
    let (msg, c) = match (over, app.message(now)) {
        (Some(o), _) => (o, t.palette.text),
        (None, Some(m)) => (m.to_owned(), t.palette.text),
        (None, None) => (crate::app::HINTS.to_owned(), t.palette.faint),
    };
    let mx = x + 8.0;
    let mw = (r.right() - PAD - mx).max(0.0);
    if mw > 60.0 {
        ui.text_clipped(&msg, Rect::new(mx, r.y + 5.0, mw, 14.0), &f, c, Align::End)?;
    }
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

    /// marker_mapper's array for `n` targets, every other one confirmed
    fn many_targets(n: i32) -> Vec<crate::model::Mk> {
        (0..n)
            .map(|i| crate::model::Mk {
                ns: if i % 2 == 0 { "target_confirmed".into() } else { "target_tentative".into() },
                id: i,
                kind: crate::model::ShapeKind::Points,
                pts: vec![],
                pos: (f64::from(i), 1.0),
                color: [0, 0, 0, 255],
                text: String::new(),
            })
            .collect()
    }

    fn team7_model() -> Model {
        let mut m = team7_scene();
        m.graph = team7_graph();
        m
    }

    pub(crate) fn team7_graph() -> Vec<crate::model::GraphTopic> {
        crate::model::tests::graph(&[
            ("/map", "nav_msgs/msg/OccupancyGrid", 1),
            ("/scan", "sensor_msgs/msg/LaserScan", 1),
            ("/team7/path", "nav_msgs/msg/Path", 1),
            ("/team7/mppi/trajectory", "nav_msgs/msg/Path", 1),
            ("/team7/frontiers", "visualization_msgs/msg/MarkerArray", 1),
            ("/team7/goal", "geometry_msgs/msg/PoseStamped", 1),
            ("/team7/targets", "visualization_msgs/msg/MarkerArray", 1),
            ("/team7/mppi/local_cost", "nav_msgs/msg/OccupancyGrid", 1),
            ("/global_costmap/costmap", "nav_msgs/msg/OccupancyGrid", 1),
        ])
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
        let slot = layout_rects().into_iter().find(|(n, _)| *n == "slot").expect("target slots").1;
        let dot = px[(slot.y + 14.0) as usize * w as usize + (slot.x + 12.0) as usize];
        assert!(near(dot, pal::TARGET_CONFIRMED, 30), "slot dot {:?}", rgb(dot));

        // click the "Global costmap" row: it turns on and the floor gets tinted
        let gi = scene::index(scene::Layer::GlobalCost);
        assert!(!app.layers.list[gi].on);
        let row_y = row_center(&chrome, &mut app, &m, now, gi);
        click(&mut chrome, &mut app, &m, now, 23.0, f64::from(row_y));
        assert!(app.layers.list[gi].on, "checkbox click turned it on");
        click(&mut chrome, &mut app, &m, now, 100.0, f64::from(row_y));
        assert!(!app.layers.list[gi].on, "a click on the row turned it off again");
        click(&mut chrome, &mut app, &m, now, 100.0, f64::from(row_y));
        let px = draw(&mut chrome, &mut app, &m, w, h, now);
        assert!(app.layers.list[gi].on, "row click turned the layer on");
        snapshot(&px, w, h, "window_team7_costmap.png");
        let p = px[(sy + a.y) as usize * w as usize + (sx + a.x) as usize];
        assert!(!near(p, pal::FREE, 10), "tinted {:?}", rgb(p));

        // collapse the dock with D: the map takes the full width
        chrome.key(KeyCode::D, Modifiers::default());
        draw(&mut chrome, &mut app, &m, w, h, now);
        assert_eq!(chrome.map_area.x, 0.0);
    }

    /// y of a layer row's centre in the dock, found from the layout the same way the dock does
    fn row_center(chrome: &Chrome, app: &mut App, m: &Model, _now: Instant, idx: usize) -> f32 {
        let mut y = chrome.layout.bodies[0].expect("layers open").y;
        for (i, s) in app.layers.list.iter().enumerate() {
            let info = s.info();
            if info.slot.is_some() && s.topic(m).is_none() && info.msg.is_some_and(|k| m.topics_of(k).is_empty()) {
                continue;
            }
            if i == idx {
                return y + ROW_H / 2.0;
            }
            y += ROW_H + 2.0;
            if info.opacity.is_some() && s.on {
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
        m.put("/team7/targets", crate::model::Feed::Markers(many_targets(40)), now);
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
        m.put("/team7/targets", crate::model::Feed::Markers(many_targets(2)), now);
        let mut chrome2 = Chrome::new("91".into(), None);
        chrome2.set_run_dir(None);
        // the first frame measures the content, the second draws with it
        draw(&mut chrome2, &mut app, &m, w, h, now);
        let px = draw(&mut chrome2, &mut app, &m, w, h, now);
        assert!(!bar(&px, chrome2.layout.bodies[2].unwrap()), "no bar when two targets fit");
    }

    /// the window at the widths the narrow-layout work is checked at (and two short heights)
    pub(crate) const SIZES: [(u32, u32); 8] = [(1440, 800), (1100, 800), (900, 800), (700, 800), (520, 800), (400, 800), (900, 500), (700, 500)];

    /// a model with everything: targets, camera, running mission, fresh data
    fn full_model(now: Instant) -> Model {
        let mut m = team7_model();
        for i in 0..30 {
            let t = now - Duration::from_millis(50 * i);
            m.tick(Topic::Odom, t);
            m.tick(Topic::Tf, t);
            m.tick(Topic::Scan, t);
        }
        let rgba: Vec<u8> = (0..120 * 90).flat_map(|i| [(i % 120 * 2) as u8, 90, 160, 255]).collect();
        m.put("/team7/aruco/debug", crate::model::Feed::Image(crate::model::CameraFrame { width: 120, height: 90, rgba, seq: 1, at: now }), now);
        m.camera_topic = Some("/team7/aruco/debug".into());
        m
    }

    /// TEAM7_VIZ_NARROW=dir TEAM7_VIZ_TAG=before cargo test --release narrow_widths -- --ignored:
    /// writes ux_narrow_<width>[x<height>]_<tag>.png for each size
    #[test]
    #[ignore]
    fn narrow_widths() {
        let Some(dir) = std::env::var_os("TEAM7_VIZ_NARROW") else { return };
        let tag = std::env::var("TEAM7_VIZ_TAG").unwrap_or_else(|_| "after".into());
        let now = Instant::now();
        let m = full_model(now);
        let save = |px: &[u32], w: u32, h: u32, name: String| {
            let rgba = crate::shots::rgba_of(px);
            let file = std::fs::File::create(std::path::Path::new(&dir).join(name)).unwrap();
            let mut enc = png::Encoder::new(std::io::BufWriter::new(file), w, h);
            enc.set_color(png::ColorType::Rgba);
            enc.set_depth(png::BitDepth::Eight);
            enc.write_header().unwrap().write_image_data(&rgba).unwrap();
        };
        for (w, h) in SIZES {
            let mut chrome = Chrome::new("79".into(), None);
            chrome.set_run_dir(None);
            chrome.profile = Some("team7-replay".into());
            let mut app = App::new(false);
            draw(&mut chrome, &mut app, &m, w, h, now);
            let px = draw(&mut chrome, &mut app, &m, w, h, now);
            let size = if h == 800 { format!("{w}") } else { format!("{w}x{h}") };
            save(&px, w, h, format!("ux_narrow_{size}_{tag}.png"));
            // narrow windows: the drawer (or the sheet) opened over the map
            if (w as f32) < NARROW && tag == "after" {
                chrome.drawer_open = true;
                draw(&mut chrome, &mut app, &m, w, h, now);
                let px = draw(&mut chrome, &mut app, &m, w, h, now);
                save(&px, w, h, format!("ux_narrow_{size}_drawer_{tag}.png"));
            }
        }
    }

    #[test]
    fn every_size_lays_out_inside_the_window() {
        let now = Instant::now();
        let m = full_model(now);
        let sizes = SIZES.iter().copied().chain([(320, 300), (1920, 1080), (260, 200)]);
        for (w, h) in sizes {
            for drawer in [false, true] {
                let mut chrome = Chrome::new("79".into(), None);
                chrome.set_run_dir(None);
                chrome.profile = Some("team7-replay".into());
                chrome.drawer_open = drawer;
                let mut app = App::new(true);
                draw(&mut chrome, &mut app, &m, w, h, now);
                draw(&mut chrome, &mut app, &m, w, h, now);
                let rects = layout_rects();
                assert!(rects.len() > 10, "{w}x{h}: only {} rects", rects.len());
                for (name, r) in rects {
                    let ok = r.width >= 0.0 && r.height >= 0.0 && r.x >= -0.5 && r.y >= -0.5 && r.right() <= w as f32 + 0.5 && r.bottom() <= h as f32 + 0.5;
                    assert!(ok, "{w}x{h} drawer {drawer}: {name} {r:?} is negative or outside");
                }
                // the map always has room, the clock is always there
                assert!(chrome.map_area.width >= (w as f32 * 0.4).min(300.0), "{w}x{h}: map {:?}", chrome.map_area);
                assert!(layout_rects().iter().any(|(n, _)| *n == "clock"));
                // under the narrow width the dock never takes width from the map
                if (w as f32) < NARROW {
                    assert_eq!(chrome.map_area.width, w as f32);
                    assert_eq!(chrome.overlay.is_some(), drawer);
                }
            }
        }
    }

    #[test]
    fn the_toolbar_collapses_step_by_step() {
        let wide = plan_toolbar(1440.0, 140.0, 84.0);
        assert_eq!((wide.tools, wide.view, wide.profile), (Look::Text, Look::Text, true));
        let mid = plan_toolbar(900.0, 140.0, 84.0);
        assert_eq!(mid.view, Look::Icon);
        let narrow = plan_toolbar(400.0, 140.0, 84.0);
        assert_eq!((narrow.tools, narrow.view, narrow.profile), (Look::Icon, Look::Menu, false));
        let tiny = plan_toolbar(260.0, 140.0, 84.0);
        assert!(tiny.short_badge && tiny.panel_icon);
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
            m.put("/team7/aruco/debug", crate::model::Feed::Image(crate::model::CameraFrame { width: 500, height: 500, rgba, seq: i + 1, at: Instant::now() }), Instant::now());
            m.camera_topic = Some("/team7/aruco/debug".into());
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
        m.graph = crate::model::tests::graph(&[
            ("/map", "nav_msgs/msg/OccupancyGrid", 1),
            ("/plan", "nav_msgs/msg/Path", 1),
            ("/local_costmap/costmap", "nav_msgs/msg/OccupancyGrid", 1),
            ("/explore/frontiers", "visualization_msgs/msg/MarkerArray", 1),
            ("/goal_pose", "geometry_msgs/msg/PoseStamped", 1),
            ("/aruco_tracker/debug", "sensor_msgs/msg/Image", 1),
            ("/global_costmap/costmap", "nav_msgs/msg/OccupancyGrid", 1),
        ]);
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
