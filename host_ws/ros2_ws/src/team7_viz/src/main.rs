//! team7_viz: map view of a team7 or stock run (see USAGE). Read-only unless `--allow-goals`.

use std::cell::{Cell, RefCell};
use std::path::PathBuf;
use std::rc::{Rc, Weak};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, mpsc};
use std::time::{Duration, Instant, SystemTime};

use aurea::menu::MenuItem;
use aurea::prelude::*;
use aurea::render::{CpuRasterizer, Renderer, request_canvas_redraw};
use team7_viz::app::{Action, App, Button as MouseKey, Tool};
use team7_viz::chrome::Chrome;
use team7_viz::model::Model;
use team7_viz::{outputs, ros, shots};

const W: u32 = 1440;
const H: u32 = 900;
/// the canvas is made small: its first size is still a minimum when the window starts tiled
/// (aurea ed32453 lets it shrink after the first layout, but a tiled start then overflows)
const MIN_W: u32 = 320;
const MIN_H: u32 = 240;
/// redraw at most this often (60 Hz) while data streams in, and at least every `IDLE_REDRAW` (clocks, ages)
const FRAME: Duration = Duration::from_millis(16);
/// how often the pump looks for new data: short, so a camera frame is shown within a few ms
const POLL: Duration = Duration::from_millis(4);
const IDLE_REDRAW: Duration = Duration::from_millis(500);

const USAGE: &str = "usage: team7_viz [--allow-goals] [--output-dir DIR] [--screens-dir DIR]
                 [--exit-after SECONDS] [--verbose] [--dump-frame PNG]
  map view of a team7 or stock (Nav2, slam_toolbox, explore_lite) run; layers follow the topics
  that exist. Set ROS_DOMAIN_ID=79 to watch a replay from run_demo.sh --replay
  --allow-goals   the 2D Goal tool sends NavigateToPose to /team7/navigate_to_pose or /navigate_to_pose
  --output-dir    run folder whose targets.csv and map.pgm the Targets panel checks
                  (default: the most recently active folder in ~/team7_runs)
  --screens-dir   where screenshots go (default ~/team7_runs/screens)
  --verbose       print received message counts every 2 s
  --dump-frame    on exit save the canvas (and PNG.ref.png drawn offscreen) for checks";

struct Options {
    exit_after: Option<f64>,
    verbose: bool,
    dump_frame: Option<String>,
    allow_goals: bool,
    output_dir: Option<PathBuf>,
    screens_dir: Option<PathBuf>,
}

fn parse() -> Result<Options, String> {
    let mut o = Options { exit_after: None, verbose: false, dump_frame: None, allow_goals: false, output_dir: None, screens_dir: None };
    let mut args = std::env::args().skip(1);
    while let Some(a) = args.next() {
        let mut val = || args.next().ok_or(format!("{a} needs a value"));
        match a.as_str() {
            "--exit-after" => {
                let v = val()?;
                o.exit_after = Some(v.parse().map_err(|_| format!("bad --exit-after {v}"))?);
            }
            "--verbose" | "-v" => o.verbose = true,
            "--dump-frame" => o.dump_frame = Some(val()?),
            "--allow-goals" => o.allow_goals = true,
            "--output-dir" => o.output_dir = Some(val()?.into()),
            "--screens-dir" => o.screens_dir = Some(val()?.into()),
            "-h" | "--help" => return Err(String::new()),
            other => return Err(format!("unknown argument {other}")),
        }
    }
    Ok(o)
}

/// where the canvas sits in the window content: aurea reports mouse events in window-content
/// coordinates only
#[cfg(target_os = "linux")]
fn canvas_origin(canvas: &Canvas, content: *mut std::ffi::c_void) -> (f64, f64) {
    use std::ffi::{c_int, c_void};
    unsafe extern "C" {
        fn gtk_widget_translate_coordinates(src: *mut c_void, dest: *mut c_void, x: c_int, y: c_int, dx: *mut c_int, dy: *mut c_int) -> c_int;
    }
    let (mut x, mut y) = (0, 0);
    // SAFETY: both are live GtkWidgets owned by the window for as long as it runs
    let ok = unsafe { gtk_widget_translate_coordinates(Element::handle(canvas), content, 0, 0, &mut x, &mut y) };
    if ok == 0 { (0.0, 0.0) } else { (f64::from(x), f64::from(y)) }
}

#[cfg(not(target_os = "linux"))]
fn canvas_origin(_: &Canvas, _: *mut std::ffi::c_void) -> (f64, f64) {
    (0.0, 0.0)
}

/// a window event moved into canvas pixels
fn shifted(e: &WindowEvent, ox: f64, oy: f64) -> WindowEvent {
    let mut e = e.clone();
    match &mut e {
        WindowEvent::MouseMove { x, y, .. } | WindowEvent::MouseButton { x, y, .. } => {
            *x -= ox;
            *y -= oy;
        }
        WindowEvent::Scroll(s) => {
            s.x -= ox;
            s.y -= oy;
        }
        _ => {}
    }
    e
}

thread_local! {
    /// true while the menus are being set from the app state: GTK runs a check item's callback
    /// when its state is set from code, and that must not toggle anything
    static SYNCING: Cell<bool> = const { Cell::new(false) };
    /// saves the canvas before the window closes (`--dump-frame`)
    static DUMP: RefCell<Option<Box<dyn FnOnce()>>> = RefCell::new(None);
    static STATS: RefCell<Stats> = RefCell::new(Stats::default());
}

/// paint times and camera frames shown, for the line printed at exit
#[derive(Default)]
struct Stats {
    paints: Vec<f64>,
    last_seq: Option<u64>,
    shown: u64,
}

impl Stats {
    fn frame(&mut self, paint: Duration, camera_seq: Option<u64>) {
        self.paints.push(paint.as_secs_f64() * 1000.0);
        if camera_seq.is_some() && camera_seq != self.last_seq {
            self.shown += 1;
            self.last_seq = camera_seq;
        }
    }

    fn line(&self, received: u64, secs: f64) -> String {
        let mut p = self.paints.clone();
        p.sort_by(f64::total_cmp);
        let q = |f: f64| p.get(((p.len() as f64 - 1.0) * f).round() as usize).copied().unwrap_or(0.0);
        format!(
            "{} frames ({:.1} fps), paint p50 {:.1} ms p95 {:.1} ms max {:.1} ms; camera received {received} ({:.1} Hz), shown {} ({:.1} Hz)",
            p.len(),
            p.len() as f64 / secs,
            q(0.5),
            q(0.95),
            q(1.0),
            received as f64 / secs,
            self.shown,
            self.shown as f64 / secs
        )
    }
}

fn syncing() -> bool {
    SYNCING.with(Cell::get)
}

struct Menus {
    panel: MenuItem,
    follow: MenuItem,
    tools: Vec<(Tool, MenuItem)>,
    last: Option<(bool, bool, Tool)>,
}

impl Menus {
    fn sync(&mut self, chrome: &Chrome, app: &App) {
        let now = (chrome.dock_open, app.camera.follow, app.tool);
        if self.last == Some(now) {
            return;
        }
        self.last = Some(now);
        SYNCING.with(|s| s.set(true));
        let _ = self.panel.set_checked(now.0);
        let _ = self.follow.set_checked(now.1);
        for (t, item) in &self.tools {
            let _ = item.set_checked(*t == now.2);
        }
        SYNCING.with(|s| s.set(false));
    }
}

fn main() -> AureaResult<()> {
    let opts = parse().unwrap_or_else(|e| {
        if !e.is_empty() {
            eprintln!("{e}");
        }
        eprintln!("{USAGE}");
        std::process::exit(2)
    });
    let domain = std::env::var("ROS_DOMAIN_ID").unwrap_or_else(|_| "0".into());
    eprintln!("team7_viz: ROS_DOMAIN_ID={domain}, {}", if opts.allow_goals { "goals ENABLED" } else { "read-only" });

    let model = Arc::new(Mutex::new(Model::default()));
    let stop = Arc::new(AtomicBool::new(false));
    let dirty = Arc::new(AtomicBool::new(true));
    let interrupted = Arc::new(AtomicBool::new(false));
    {
        let flag = interrupted.clone();
        if let Err(e) = ctrlc::set_handler(move || flag.store(true, Ordering::Relaxed)) {
            eprintln!("team7_viz: no signal handler: {e}");
        }
    }
    let (goal_tx, goal_rx) = mpsc::channel();
    let ros_thread = {
        let (model, stop, dirty) = (model.clone(), stop.clone(), dirty.clone());
        let goals = opts.allow_goals.then_some(goal_rx);
        std::thread::Builder::new()
            .name("team7_viz_ros".into())
            .spawn(move || {
                if let Err(e) = ros::run(model, stop, dirty, ros::Options { goals }) {
                    eprintln!("team7_viz: ROS node failed: {e} (is ROS sourced?)");
                }
            })
            .expect("spawn ROS thread")
    };

    let app = Rc::new(RefCell::new(App::new(opts.allow_goals)));
    let chrome = Rc::new(RefCell::new(Chrome::new(domain.clone(), opts.output_dir.clone())));
    let screens_dir = opts.screens_dir.clone().unwrap_or_else(|| outputs::runs_root().join("screens"));
    let mut window = Window::new("team7 viewer", W as i32, H as i32)?;
    let canvas = Canvas::new(MIN_W, MIN_H, RendererBackend::Cpu)?;
    let id = canvas.id();
    let redraw = move || request_canvas_redraw(id);

    // native menu bar; everything else is drawn on the canvas
    let shot: Rc<dyn Fn()> = {
        let (canvas, app, dir) = (canvas.clone(), app.clone(), screens_dir.clone());
        Rc::new(move || {
            let path = dir.join(format!("team7_viz_{}.png", shots::stamp_name(SystemTime::now())));
            let msg = match shots::save_canvas(&canvas, &path) {
                Ok(()) => format!("screenshot saved: {}", path.display()),
                Err(e) => format!("screenshot failed: {e}"),
            };
            eprintln!("team7_viz: {msg}");
            app.borrow_mut().say(msg);
            redraw();
        })
    };
    let win_slot: Rc<RefCell<Weak<Window>>> = Rc::new(RefCell::new(Weak::new()));
    let bar = window.create_menu_bar()?;
    let mut file = bar.add_submenu("File")?;
    let s = shot.clone();
    file.add_item("Screenshot (P)", move || {
        let s = s.clone();
        aurea::after_frame(move || s());
    })?;
    {
        let (canvas, app, dir, win) = (canvas.clone(), app.clone(), screens_dir.clone(), win_slot.clone());
        file.add_item("Save screenshot as...", move || {
            let (canvas, app, dir, win) = (canvas.clone(), app.clone(), dir.clone(), win.clone());
            aurea::after_frame(move || {
                let Some(w) = win.borrow().upgrade() else { return };
                let dialog = aurea::dialog::FileDialog::new()
                    .title("Save map screenshot")
                    .file_name(&format!("team7_viz_{}.png", shots::stamp_name(SystemTime::now())))
                    .directory(&dir)
                    .filter("PNG image", &["png"]);
                if let Ok(Some(path)) = dialog.save(&w) {
                    let msg = match shots::save_canvas(&canvas, &path) {
                        Ok(()) => format!("screenshot saved: {}", path.display()),
                        Err(e) => format!("screenshot failed: {e}"),
                    };
                    app.borrow_mut().say(msg);
                    redraw();
                }
            });
        })?;
    }
    file.add_separator()?;
    let win = win_slot.clone();
    file.add_item("Quit (Ctrl+Q)", move || close_later(win.borrow().clone()))?;

    let mut view = bar.add_submenu("View")?;
    let c = chrome.clone();
    let panel = view.add_check_item("Panel (D)", true, move || {
        if !syncing() {
            let mut c = c.borrow_mut();
            c.dock_open = !c.dock_open;
            redraw();
        }
    })?;
    let a = app.clone();
    let follow = view.add_check_item("Follow robot (F)", true, move || {
        if !syncing() {
            let mut a = a.borrow_mut();
            a.camera.follow = !a.camera.follow;
            redraw();
        }
    })?;
    let a = app.clone();
    view.add_item("Fit map, north up (0)", move || {
        a.borrow_mut().fit();
        redraw();
    })?;
    let c = chrome.clone();
    view.add_item("Health details (H)", move || {
        let mut c = c.borrow_mut();
        c.sections[3] = !c.sections[3];
        redraw();
    })?;
    let mut tools_menu = bar.add_submenu("Tools")?;
    let mut tools = Vec::new();
    for (t, title) in [(Tool::Interact, "Interact (I, Esc)"), (Tool::Measure, "Measure (M)"), (Tool::Goal, "2D Goal (G)")] {
        let a = app.clone();
        tools.push((
            t,
            tools_menu.add_radio_item(title, t == Tool::Interact, move || {
                if !syncing() {
                    a.borrow_mut().set_tool(t);
                    redraw();
                }
            })?,
        ));
    }
    let menus = Rc::new(RefCell::new(Menus { panel, follow, tools, last: None }));

    let mut root = Stack::new(Orientation::Vertical)?;
    root.set_padding(0)?;
    // weight 1: the canvas takes all the room the window gets
    root.add_weighted(canvas.clone(), 1.0)?;
    let content = Element::handle(&root);
    window.set_content(root)?;
    let window = Rc::new(window);
    *win_slot.borrow_mut() = Rc::downgrade(&window);
    let started = Instant::now();

    let (dc, da, shared, menus_d) = (chrome.clone(), app.clone(), model.clone(), menus.clone());
    let weak = Rc::downgrade(&window);
    let quit = interrupted.clone();
    let mut closing = false;
    let exit_after = opts.exit_after;
    let reference = opts.dump_frame.as_ref().map(|p| format!("{}.ref.png", p.trim_end_matches(".png")));
    let shot_d = shot.clone();
    canvas.set_draw_callback(move |ctx| {
        let timed_out = exit_after.is_some_and(|s| started.elapsed().as_secs_f64() >= s);
        let last = (timed_out || quit.load(Ordering::Relaxed)) && !closing;
        let now = Instant::now();
        let Ok(mut m) = shared.lock() else { return Ok(()) };
        let mut c = dc.borrow_mut();
        let mut a = da.borrow_mut();
        // tell the ROS thread what the layers want subscribed, and which image the camera shows
        a.publish_wants(&mut m);
        let paint = Instant::now();
        let req = c.frame(ctx, &mut a, &m, now)?;
        STATS.with(|s| s.borrow_mut().frame(paint.elapsed(), m.camera().map(|f| f.seq)));
        if c.ui.wants_frame() {
            redraw();
        }
        // the same frame drawn offscreen, to compare the window path with the reference path
        if let (true, Some(path)) = (last, reference.as_ref()) {
            let (w, h) = (ctx.width(), ctx.height());
            let mut r = CpuRasterizer::new(w, h);
            if let Ok(mut off) = r.begin_frame() {
                let _ = c.frame(off.as_mut(), &mut a, &m, now);
            }
            let _ = r.end_frame();
            report(path, shots::save_rgba(path.as_ref(), r.pixels(), w, h));
        }
        let (cc, aa, mm) = (dc.clone(), da.clone(), menus_d.clone());
        aurea::after_frame(move || {
            if let (Ok(c), Ok(a), Ok(mut menus)) = (cc.try_borrow(), aa.try_borrow(), mm.try_borrow_mut()) {
                menus.sync(&c, &a);
            }
        });
        if req.screenshot {
            let s = shot_d.clone();
            aurea::after_frame(move || s());
        }
        if last {
            closing = true;
            close_later(weak.clone());
        }
        Ok(())
    })?;
    if let Some(path) = opts.dump_frame.clone() {
        let c = canvas.clone();
        DUMP.with(|d| *d.borrow_mut() = Some(Box::new(move || report(&path, shots::save_canvas(&c, path.as_ref())))));
    }

    // redraw pump: the ROS thread marks the model dirty, this asks for a canvas frame
    let pump = {
        let (stop, dirty, model) = (stop.clone(), dirty.clone(), model.clone());
        let verbose = opts.verbose;
        std::thread::spawn(move || {
            let (mut last_draw, mut last_log) = (Instant::now(), Instant::now());
            while !stop.load(Ordering::Relaxed) {
                std::thread::sleep(POLL);
                let due = last_draw.elapsed() >= FRAME;
                if (due && dirty.swap(false, Ordering::Relaxed)) || last_draw.elapsed() >= IDLE_REDRAW {
                    request_canvas_redraw(id);
                    last_draw = Instant::now();
                }
                if verbose && last_log.elapsed() >= Duration::from_secs(2) {
                    if let Ok(m) = model.lock() {
                        eprintln!("team7_viz: received {}", m.counts_line());
                    }
                    last_log = Instant::now();
                }
            }
        })
    };

    let weak = Rc::downgrade(&window);
    let events_canvas = canvas.clone();
    let debug_input = std::env::var_os("TEAM7_VIZ_DEBUG_INPUT").is_some();
    window.on_event(move |event| {
        let (ox, oy) = canvas_origin(&events_canvas, content);
        let e = shifted(event, ox, oy);
        if debug_input && matches!(e, WindowEvent::MouseButton { .. }) {
            eprintln!("team7_viz: input {e:?}");
        }
        let (Ok(mut c), Ok(mut a)) = (chrome.try_borrow_mut(), app.try_borrow_mut()) else { return };
        let area = c.map_area;
        let in_map = |x: f64, y: f64| area.contains(aurea::render::Point::new(x as f32, y as f32));
        let local = |x: f64, y: f64| (x - f64::from(area.x), y - f64::from(area.y));
        let mut changed = match e {
            WindowEvent::KeyInput { key, pressed: true, modifiers } => {
                c.handle(&e);
                if c.key(key, modifiers) {
                    true
                } else {
                    match a.key(key, modifiers) {
                        Action::Quit => {
                            close_later(weak.clone());
                            false
                        }
                        Action::Screenshot => {
                            let s = shot.clone();
                            aurea::after_frame(move || s());
                            false
                        }
                        Action::Redraw => true,
                        Action::None => false,
                    }
                }
            }
            // the map takes presses and the wheel over it; the widgets take the rest
            WindowEvent::MouseButton { button, pressed: true, x, y, .. } if in_map(x, y) => {
                let b = match button {
                    MouseButton::Left => Some(MouseKey::Left),
                    MouseButton::Middle => Some(MouseKey::Middle),
                    MouseButton::Right => Some(MouseKey::Right),
                    MouseButton::Other(_) => None,
                };
                let (lx, ly) = local(x, y);
                b.is_some_and(|b| a.press(b, lx, ly))
            }
            WindowEvent::MouseButton { pressed: false, x, y, .. } => {
                let (lx, ly) = local(x, y);
                let used = a.release(lx, ly);
                c.handle(&e) || used
            }
            WindowEvent::MouseMove { x, y, .. } => {
                let (lx, ly) = local(x, y);
                let used = a.moved(lx, ly);
                c.handle(&e) || used || in_map(x, y)
            }
            WindowEvent::Scroll(s) if in_map(s.x, s.y) => {
                let (lx, ly) = local(s.x, s.y);
                // a wheel detent is one zoom step; trackpads give pixels, about 40 per step
                a.wheel(s.clicks.map_or(s.delta_y / 40.0, |c| c.1), lx, ly)
            }
            _ => c.handle(&e),
        };
        if let Some(g) = a.pending_goal.take() {
            let _ = goal_tx.send(g);
            changed = true;
        }
        drop((c, a));
        if changed {
            redraw();
        }
    });

    window.show();
    let result = window.run();

    stop.store(true, Ordering::Relaxed);
    let _ = pump.join();
    let _ = ros_thread.join();
    if let Ok(m) = model.lock() {
        let line = m.counts_line();
        let secs = started.elapsed().as_secs_f64();
        eprintln!("team7_viz: exit after {secs:.1} s, received {}", if line.is_empty() { "nothing" } else { &line });
        let received = m.rate(team7_viz::model::Topic::Camera).total;
        eprintln!("team7_viz: {}", STATS.with(|s| s.borrow().line(received, secs)));
    }
    result
}

/// close once the current frame or event is done (`WindowProxy::dispatch` is not drained under `run`)
fn close_later(window: Weak<Window>) {
    aurea::after_frame(move || {
        if let Some(dump) = DUMP.with(|d| d.borrow_mut().take()) {
            dump();
        }
        if let Some(w) = window.upgrade() {
            w.close();
        }
    });
}

fn report(path: &str, r: Result<(), String>) {
    match r {
        Ok(()) => eprintln!("team7_viz: frame saved to {path}"),
        Err(e) => eprintln!("team7_viz: cannot save {path}: {e}"),
    }
}
