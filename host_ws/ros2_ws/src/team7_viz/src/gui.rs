//! The native window around the map canvas: menu bar, toolbar, a split pane with the dock
//! (mission, camera, health, layers) on the left and the canvas on the right, and a status bar.
//! Widgets are GTK elements through aurea; they are refreshed from the model a few times a second
//! on the UI thread (`refresh_soon`), never from the ROS thread.

use std::cell::RefCell;
use std::path::PathBuf;
use std::rc::{Rc, Weak};
use std::sync::{Arc, Mutex};
use std::time::{Instant, SystemTime};

use aurea::elements::{ImageScaling, ScrollView, Shared, Table, TextStyle};
use aurea::menu::MenuItem;
use aurea::prelude::*;
use aurea::render::{CanvasId, CpuRasterizer, Renderer, request_canvas_redraw};
use tb4_draw::palette as pal;

use crate::app::{App, Tool};
use crate::health::{self, Level};
use crate::model::{Model, countdown};
use crate::outputs;
use crate::ros::wall_secs;
use crate::scene::{self, LAYERS};

pub type SharedModel = Arc<Mutex<Model>>;

/// dock width at start, pixels
pub const DOCK_WIDTH: f32 = 340.0;

/// section titles, in `App::sections` order
pub const SECTIONS: [&str; 4] = ["Mission", "Camera", "Health", "Layers"];

/// a label that only touches GTK when its text changes
struct Text {
    label: Shared<Label>,
    last: String,
}

impl Text {
    fn new(text: &str, style: TextStyle) -> AureaResult<Self> {
        Ok(Self { label: Shared::new(Label::with_style(text, style)?), last: text.to_owned() })
    }

    fn set(&mut self, text: &str) {
        if self.last != text {
            let _ = self.label.borrow_mut().set_text(text);
            text.clone_into(&mut self.last);
        }
    }
}

/// a small image whose PNG is replaced only when its key changes
struct Picture {
    view: Shared<ImageView>,
    key: u64,
}

impl Picture {
    fn new(w: u16, h: u16) -> AureaResult<Self> {
        let view = Shared::new(ImageView::with_scaling(ImageScaling::AspectFit)?);
        view.borrow().set_min_size(w, h)?;
        Ok(Self { view, key: u64::MAX })
    }

    fn set(&mut self, key: u64, png: impl FnOnce() -> Vec<u8>) {
        if self.key != key {
            let _ = self.view.borrow_mut().load_from_data(&png());
            self.key = key;
        }
    }
}

pub struct Gui {
    app: Rc<RefCell<App>>,
    model: SharedModel,
    canvas: Canvas,
    window: Weak<Window>,
    domain: String,
    out_dir: Option<PathBuf>,
    screens_dir: PathBuf,
    sections: Vec<Shared<Stack>>,
    section_menu: Vec<MenuItem>,
    layer_checks: Vec<Shared<Checkbox>>,
    split: Shared<SplitView>,
    layer_menu: Vec<MenuItem>,
    follow_menu: MenuItem,
    tool_menu: Vec<MenuItem>,
    mode: Text,
    countdown: Text,
    mission: Text,
    targets: Shared<Table>,
    target_rows: Vec<Vec<String>>,
    output: Text,
    camera: Picture,
    camera_info: Text,
    health: Vec<(Picture, Text)>,
    status: Text,
}

thread_local! {
    static GUI: RefCell<Option<Gui>> = const { RefCell::new(None) };
}

/// options the window needs
pub struct GuiOptions {
    pub domain: String,
    pub out_dir: Option<PathBuf>,
    pub screens_dir: PathBuf,
}

/// refresh the widgets from the model on the UI thread, later (safe from any thread and from
/// inside widget callbacks)
pub fn refresh_soon() {
    aurea::on_ui_thread(|| {
        GUI.with(|g| {
            if let Ok(mut g) = g.try_borrow_mut()
                && let Some(g) = g.as_mut()
            {
                g.refresh();
            }
        });
    });
}

/// runs `f` on the window state, if it is built and not busy
pub fn with<R>(f: impl FnOnce(&mut Gui) -> R) -> Option<R> {
    GUI.with(|g| g.try_borrow_mut().ok().and_then(|mut g| g.as_mut().map(f)))
}

/// at most `n` characters, "..." at the end when cut (GTK labels here do not wrap)
fn clip(s: &str, n: usize) -> String {
    if s.chars().count() <= n { s.to_owned() } else { format!("{}...", s.chars().take(n - 3).collect::<String>()) }
}

/// at most `n` characters, keeping the end
fn clip_start(s: &str, n: usize) -> String {
    let c = s.chars().count();
    if c <= n { s.to_owned() } else { format!("...{}", s.chars().skip(c - (n - 3)).collect::<String>()) }
}

fn png(w: u32, h: u32, rgba: &[u8]) -> Vec<u8> {
    let mut out = Vec::new();
    let mut enc = png::Encoder::new(&mut out, w, h);
    enc.set_color(png::ColorType::Rgba);
    enc.set_depth(png::BitDepth::Eight);
    if let Ok(mut wr) = enc.write_header() {
        let _ = wr.write_image_data(rgba);
    }
    out
}

/// 0xAARRGGBB pixels -> RGBA bytes, opaque
pub fn rgba_of(px: &[u32]) -> Vec<u8> {
    px.iter().flat_map(|p| [(p >> 16) as u8, (p >> 8) as u8, *p as u8, 255]).collect()
}

/// a layer's legend symbol as a PNG, on the map background so it reads on any GTK theme
fn swatch_png(l: scene::Layer) -> Vec<u8> {
    let (w, h) = (34u32, 20u32);
    let mut r = CpuRasterizer::new(w, h);
    if let Ok(mut ctx) = r.begin_frame() {
        let _ = ctx.clear(tb4_draw::prim::color(pal::BACKGROUND));
        let _ = tb4_draw::paint_prims(ctx.as_mut(), &scene::symbol(l, (17.0, 10.0)));
    }
    let _ = r.end_frame();
    png(w, h, &rgba_of(r.pixels()))
}

fn dot_png(level: Level) -> Vec<u8> {
    let c = match level {
        Level::Ok => [40, 200, 80, 255],
        Level::Warn => [255, 180, 0, 255],
        Level::Bad => [230, 50, 50, 255],
        Level::Off => [110, 110, 110, 255],
    };
    png(12, 12, &c.repeat(144))
}

fn level_key(l: Level) -> u64 {
    l as u64
}

/// UTC "20261119_074806" for file names
pub fn stamp_name(t: SystemTime) -> String {
    let s = t.duration_since(SystemTime::UNIX_EPOCH).map_or(0, |d| d.as_secs());
    let (days, rem) = ((s / 86_400) as i64, s % 86_400);
    // civil from days (Howard Hinnant)
    let z = days + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z - era * 146_097;
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let d = doy - (153 * mp + 2) / 5 + 1;
    let m = if mp < 10 { mp + 3 } else { mp - 9 };
    let y = yoe + era * 400 + i64::from(m <= 2);
    format!("{y:04}{m:02}{d:02}_{:02}{:02}{:02}", rem / 3600, rem % 3600 / 60, rem % 60)
}

/// 0xAARRGGBB pixels to a PNG file
pub fn save_rgba(path: &std::path::Path, px: &[u32], w: u32, h: u32) -> Result<(), String> {
    std::fs::write(path, png(w, h, &rgba_of(px))).map_err(|e| e.to_string())
}

/// the canvas's last frame to a PNG file
pub fn save_canvas(canvas: &Canvas, path: &std::path::Path) -> Result<(), String> {
    if let Some(dir) = path.parent() {
        std::fs::create_dir_all(dir).map_err(|e| e.to_string())?;
    }
    canvas
        .with_frame_pixels(|px, w, h| std::fs::write(path, png(w, h, &rgba_of(px))).map_err(|e| e.to_string()))
        .unwrap_or_else(|| Err("no frame drawn yet".into()))
}

impl Gui {
    fn poke(&self) {
        request_canvas_redraw(self.canvas.id());
        refresh_soon();
    }

    /// quick screenshot of the map view into the screens folder
    pub fn screenshot(&mut self) {
        let path = self.screens_dir.join(format!("team7_viz_{}.png", stamp_name(SystemTime::now())));
        let msg = match save_canvas(&self.canvas, &path) {
            Ok(()) => format!("screenshot saved: {}", path.display()),
            Err(e) => format!("screenshot failed: {e}"),
        };
        eprintln!("team7_viz: {msg}");
        self.app.borrow_mut().say(msg);
        self.poke();
    }

    /// screenshot to a file the user picks
    pub fn screenshot_as(&mut self) {
        let Some(window) = self.window.upgrade() else { return };
        let dialog = aurea::dialog::FileDialog::new()
            .title("Save map screenshot")
            .file_name(&format!("team7_viz_{}.png", stamp_name(SystemTime::now())))
            .directory(&self.screens_dir)
            .filter("PNG image", &["png"]);
        if let Ok(Some(path)) = dialog.save(&window) {
            let msg = match save_canvas(&self.canvas, &path) {
                Ok(()) => format!("screenshot saved: {}", path.display()),
                Err(e) => format!("screenshot failed: {e}"),
            };
            self.app.borrow_mut().say(msg);
            self.poke();
        }
    }

    /// put the divider where it belongs once the window has its size (GTK resets it before that)
    pub fn place_divider(&mut self) {
        let _ = self.split.borrow().set_divider_position(0, DOCK_WIDTH);
    }

    /// widgets <- app state and model
    pub fn refresh(&mut self) {
        let now = Instant::now();
        let app = self.app.borrow();
        for (i, body) in self.sections.iter().enumerate() {
            let _ = body.borrow().set_visible(app.sections[i]);
            let _ = self.section_menu[i].set_checked(app.sections[i]);
        }
        for (i, check) in self.layer_checks.iter().enumerate() {
            let on = app.layers.on[i];
            if check.borrow().get_checked() != on {
                let _ = check.borrow_mut().set_checked(on);
            }
            let _ = self.layer_menu[i].set_checked(on);
        }
        let _ = self.follow_menu.set_checked(app.camera.follow);
        for (i, t) in [Tool::Interact, Tool::Measure, Tool::Goal].iter().enumerate() {
            let _ = self.tool_menu[i].set_checked(app.tool == *t);
        }
        let mode = if app.allow_goals { format!("{}   GOALS ON", app.tool.name()) } else { format!("{}   READ-ONLY", app.tool.name()) };
        self.mode.set(&mode);

        let Ok(m) = self.model.lock() else { return };
        let status = app.status(&m, now, wall_secs(), &self.domain);
        drop(app);
        self.status.set(&status);

        // mission
        self.countdown.set(&format!("{} left", countdown(m.mission_started, m.mission_ended, now)));
        let state = m.mission.as_deref().unwrap_or("no mission state yet");
        let (nc, nt) = (m.targets(true).count(), m.targets(false).count());
        let goal = m.goal_status.as_deref().map_or(String::new(), |g| format!("\n{}", clip(g, 40)));
        self.mission.set(&format!("state {state}\n{nc} confirmed, {nt} tentative{goal}"));
        let rows: Vec<Vec<String>> = m
            .targets
            .iter()
            .map(|t| {
                let views = t.views.map_or_else(|| "-".to_owned(), |v| v.to_string());
                vec![t.id.to_string(), format!("{:.2}", t.x), format!("{:.2}", t.y), if t.confirmed { "confirmed" } else { "tentative" }.into(), views]
            })
            .collect();
        if rows != self.target_rows {
            let mut table = self.targets.borrow_mut();
            let _ = table.clear();
            for (i, r) in rows.iter().enumerate() {
                let _ = table.add_row(i as u64, r);
            }
            self.target_rows = rows;
        }
        let out = match &self.out_dir {
            None => "no run folder (--output-dir)".to_owned(),
            Some(d) => {
                let files = outputs::status(d).iter().map(|f| outputs::describe(f, SystemTime::now())).collect::<Vec<_>>().join("\n");
                let name = d.file_name().map_or_else(|| d.display().to_string(), |n| n.to_string_lossy().into_owned());
                format!("run {}\n{files}", clip_start(&name, 34))
            }
        };
        self.output.set(&out);

        // camera
        match &m.camera {
            Some(f) => {
                let (w, h, rgba, seq) = (f.width, f.height, &f.rgba, f.seq);
                self.camera.set(seq, || png(w, h, rgba));
                self.camera_info.set(&format!("{}x{}, {:.1} s ago", f.width, f.height, now.saturating_duration_since(f.at).as_secs_f64()));
            }
            None => self.camera_info.set("no image yet"),
        }

        // health
        let (ros_now, _) = m.ros_now(now, wall_secs());
        for (check, (dot, text)) in health::all(&m, now, ros_now).iter().zip(self.health.iter_mut()) {
            dot.set(level_key(check.level), || dot_png(check.level));
            text.set(&format!("{:<10} {:>9}   {}", check.name, check.value, check.expect));
        }
    }
}

/// a dock section: a header button that opens and closes its body
fn section(dock: &mut Stack, app: &Rc<RefCell<App>>, i: usize) -> AureaResult<Shared<Stack>> {
    let a = app.clone();
    dock.add(Button::with_callback(SECTIONS[i], move || {
        let mut a = a.borrow_mut();
        a.sections[i] = !a.sections[i];
        drop(a);
        refresh_soon();
    })?)?;
    let mut body = Stack::new(Orientation::Vertical)?;
    body.set_padding(6)?;
    body.set_spacing(4)?;
    let body = Shared::new(body);
    dock.add(body.clone())?;
    Ok(body)
}

fn row() -> AureaResult<Stack> {
    let mut r = Stack::new(Orientation::Horizontal)?;
    r.set_spacing(6)?;
    Ok(r)
}

/// builds menus, toolbar, dock, canvas and status bar into `window`; returns the content root
/// the window for dialogs and Quit; set once it is shared
pub fn set_window(window: Weak<Window>) {
    with(|g| g.window = window);
}

pub fn build(menu_window: &mut Window, app: Rc<RefCell<App>>, model: SharedModel, canvas: Canvas, opts: GuiOptions) -> AureaResult<Shared<Stack>> {
    let id: CanvasId = canvas.id();
    let redraw = move || {
        request_canvas_redraw(id);
        refresh_soon();
    };

    // menus
    let bar = menu_window.create_menu_bar()?;
    let mut file = bar.add_submenu("File")?;
    file.add_item("Save screenshot as...", || {
        aurea::after_frame(|| {
            with(Gui::screenshot_as);
        });
    })?;
    file.add_item("Quick screenshot (P)", || {
        with(Gui::screenshot);
    })?;
    file.add_separator()?;
    file.add_item("Quit (Ctrl+Q)", || {
        if let Some(w) = with(|g| g.window.upgrade()).flatten() {
            w.close();
        }
    })?;

    let mut view = bar.add_submenu("View")?;
    let a = app.clone();
    let follow_menu = view.add_check_item("Follow robot (F)", true, move || {
        let mut a = a.borrow_mut();
        a.camera.follow = !a.camera.follow;
        drop(a);
        redraw();
    })?;
    let a = app.clone();
    view.add_item("Fit map, north up (0)", move || {
        a.borrow_mut().fit();
        redraw();
    })?;
    let a = app.clone();
    view.add_item("Rotate left 15 deg ([)", move || {
        a.borrow_mut().rotate(std::f64::consts::PI / 12.0);
        redraw();
    })?;
    let a = app.clone();
    view.add_item("Rotate right 15 deg (])", move || {
        a.borrow_mut().rotate(-std::f64::consts::PI / 12.0);
        redraw();
    })?;
    view.add_separator()?;
    let mut layers_menu = view.add_submenu("Layers")?;
    let mut layer_menu = Vec::new();
    for (i, l) in LAYERS.iter().enumerate() {
        let a = app.clone();
        let title = if i < 9 { format!("{} ({})", l.name, i + 1) } else { l.name.to_owned() };
        layer_menu.push(layers_menu.add_check_item(&title, l.on, move || {
            a.borrow_mut().layers.on[i] ^= true;
            redraw();
        })?);
    }
    let mut panels_menu = view.add_submenu("Panels")?;
    let mut section_menu = Vec::new();
    for (i, s) in SECTIONS.iter().enumerate() {
        let a = app.clone();
        section_menu.push(panels_menu.add_check_item(s, true, move || {
            a.borrow_mut().sections[i] ^= true;
            refresh_soon();
        })?);
    }

    let mut tools = bar.add_submenu("Tools")?;
    let mut tool_menu = Vec::new();
    for (t, title) in [(Tool::Interact, "Interact (I, Esc)"), (Tool::Measure, "Measure (M)"), (Tool::Goal, "2D Goal (G)")] {
        let a = app.clone();
        tool_menu.push(tools.add_radio_item(title, t == Tool::Interact, move || {
            a.borrow_mut().set_tool(t);
            redraw();
        })?);
    }

    // toolbar
    let mut toolbar = row()?;
    toolbar.set_padding(4)?;
    let tool_button = |title: &str, t: Tool| {
        let a = app.clone();
        Button::with_callback(title, move || {
            a.borrow_mut().set_tool(t);
            redraw();
        })
    };
    toolbar.add(tool_button("Interact", Tool::Interact)?)?;
    toolbar.add(tool_button("Measure", Tool::Measure)?)?;
    let goal_button = tool_button("2D Goal", Tool::Goal)?;
    goal_button.set_tooltip(if app.borrow().allow_goals { "drag on the map: position, then heading; sends NavigateToPose" } else { "read-only: shows the pose, sends nothing (start with --allow-goals)" })?;
    toolbar.add(goal_button)?;
    for (title, f) in [("+", 1.2), ("-", 1.0 / 1.2)] {
        let a = app.clone();
        toolbar.add(Button::with_callback(title, move || {
            a.borrow_mut().zoom(f);
            redraw();
        })?)?;
    }
    let a = app.clone();
    toolbar.add(Button::with_callback("Fit", move || {
        a.borrow_mut().fit();
        redraw();
    })?)?;
    let a = app.clone();
    toolbar.add(Button::with_callback("Follow", move || {
        let mut a = a.borrow_mut();
        a.camera.follow = !a.camera.follow;
        drop(a);
        redraw();
    })?)?;
    let a = app.clone();
    toolbar.add(Button::with_callback("North up", move || {
        let mut a = a.borrow_mut();
        let r = a.camera.rotation;
        a.rotate(-r);
        drop(a);
        redraw();
    })?)?;
    toolbar.add(Button::with_callback("Shot", || {
        with(Gui::screenshot);
    })?)?;
    let mode = Text::new("", TextStyle::Strong)?;
    toolbar.add(mode.label.clone())?;

    // dock
    let mut dock = Stack::new(Orientation::Vertical)?;
    dock.set_padding(6)?;
    dock.set_spacing(2)?;
    let mut sections = Vec::new();

    let body = section(&mut dock, &app, 0)?;
    let countdown = Text::new("10:00", TextStyle::Heading)?;
    let mission = Text::new("", TextStyle::Body)?;
    let targets = Shared::new(Table::new(["ID", "x [m]", "y [m]", "state", "views"])?);
    targets.borrow().set_min_size(260, 120)?;
    let output = Text::new("", TextStyle::Small)?;
    {
        let mut b = body.borrow_mut();
        b.add(countdown.label.clone())?;
        b.add(mission.label.clone())?;
        b.add(targets.clone())?;
        b.add(output.label.clone())?;
    }
    sections.push(body);

    let body = section(&mut dock, &app, 1)?;
    let camera = Picture::new(300, 200)?;
    let camera_info = Text::new("", TextStyle::Small)?;
    {
        let mut b = body.borrow_mut();
        b.add(camera.view.clone())?;
        b.add(camera_info.label.clone())?;
        let a = app.clone();
        b.add(Button::with_callback("Enlarge (C)", move || {
            let mut a = a.borrow_mut();
            a.camera_big = !a.camera_big;
            drop(a);
            redraw();
        })?)?;
    }
    sections.push(body);

    let body = section(&mut dock, &app, 2)?;
    let mut health = Vec::new();
    for _ in 0..7 {
        let mut r = row()?;
        let dot = Picture::new(12, 12)?;
        let text = Text::new("", TextStyle::Small)?;
        r.add(dot.view.clone())?;
        r.add(text.label.clone())?;
        body.borrow_mut().add(r)?;
        health.push((dot, text));
    }
    sections.push(body);

    let body = section(&mut dock, &app, 3)?;
    let mut layer_checks = Vec::new();
    for (i, l) in LAYERS.iter().enumerate() {
        let mut r = row()?;
        let mut sw = Picture::new(34, 20)?;
        sw.set(1, || swatch_png(l.layer));
        r.add(sw.view.clone())?;
        let title = if i < 9 { format!("{}  [{}]", l.name, i + 1) } else { l.name.to_owned() };
        let check = Shared::new(Checkbox::with_checked(&title, l.on)?);
        let a = app.clone();
        check.borrow_mut().on_change(move |on| {
            if let Ok(mut a) = a.try_borrow_mut()
                && a.layers.on[i] != on
            {
                a.layers.on[i] = on;
                drop(a);
                redraw();
            }
        })?;
        r.add(check.clone())?;
        layer_checks.push(check);
        let mut b = body.borrow_mut();
        b.add(r)?;
        b.add(Label::with_style(l.legend, TextStyle::Small)?)?;
        if let Some(o) = l.opacity {
            let mut r = row()?;
            r.add(Label::with_style("opacity", TextStyle::Small)?)?;
            let mut s = Slider::with_value(0.0, 1.0, f64::from(o))?;
            let a = app.clone();
            s.on_change(move |v| {
                if let Ok(mut a) = a.try_borrow_mut() {
                    a.layers.opacity[i] = v as f32;
                    drop(a);
                    request_canvas_redraw(id);
                }
            })?;
            s.set_min_size(160, 0)?;
            r.add_weighted(s, 1.0)?;
            b.add(r)?;
        }
    }
    sections.push(body);

    let scroll = ScrollView::with_content(dock)?;
    scroll.set_min_size(300, 200)?;
    let split = Shared::new(SplitView::with_children(SplitOrientation::Horizontal, scroll, canvas.clone())?);
    split.borrow().set_divider_position(0, DOCK_WIDTH)?;

    let status = Text::new("", TextStyle::Small)?;
    let mut bottom = Stack::new(Orientation::Vertical)?;
    bottom.set_padding(4)?;
    bottom.set_spacing(0)?;
    bottom.add(status.label.clone())?;
    bottom.add(Label::with_style(crate::app::HINTS, TextStyle::Small)?)?;
    let mut root = Stack::new(Orientation::Vertical)?;
    root.set_padding(0)?;
    root.set_spacing(0)?;
    root.add(toolbar)?;
    root.add_weighted(split.clone(), 1.0)?;
    root.add(bottom)?;
    let root = Shared::new(root);

    let gui = Gui {
        app,
        model,
        canvas,
        window: Weak::new(),
        domain: opts.domain,
        out_dir: opts.out_dir,
        screens_dir: opts.screens_dir,
        sections,
        section_menu,
        layer_checks,
        split,
        layer_menu,
        follow_menu,
        tool_menu,
        mode,
        countdown,
        mission,
        targets,
        target_rows: Vec::new(),
        output,
        camera,
        camera_info,
        health,
        status,
    };
    GUI.with(|g| *g.borrow_mut() = Some(gui));
    Ok(root)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn stamp_names_are_utc_dates() {
        let t = SystemTime::UNIX_EPOCH + std::time::Duration::from_secs(1_795_074_486);
        assert_eq!(stamp_name(t), "20261119_074806");
        assert_eq!(stamp_name(SystemTime::UNIX_EPOCH), "19700101_000000");
    }
}
