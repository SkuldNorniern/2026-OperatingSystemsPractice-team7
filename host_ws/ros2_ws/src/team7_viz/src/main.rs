//! team7_viz: map view of a team7 run (see USAGE). Read-only unless `--allow-goals`.

use std::cell::RefCell;
use std::path::PathBuf;
use std::rc::{Rc, Weak};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, mpsc};
use std::time::{Duration, Instant};

use aurea::prelude::*;
use aurea::render::{CpuRasterizer, Renderer, request_canvas_redraw};
use team7_viz::app::{Action, App, Button as MouseKey};
use team7_viz::gui::{self, GuiOptions};
use team7_viz::model::Model;
use team7_viz::{outputs, ros};
use tb4_draw::palette;

const W: u32 = 1440;
const H: u32 = 900;
/// the canvas size request is a minimum in GTK: keep it small so a tiling compositor can shrink the window
const MIN_W: u32 = 320;
const MIN_H: u32 = 240;
/// redraw at most this often while data streams in, and at least every `IDLE_REDRAW` (rates decay)
const FRAME: Duration = Duration::from_millis(50);
const IDLE_REDRAW: Duration = Duration::from_millis(500);
/// dock widgets follow the model this often
const PANEL_REFRESH: Duration = Duration::from_millis(250);

const USAGE: &str = "usage: team7_viz [--allow-goals] [--output-dir DIR] [--screens-dir DIR] [--light]
                 [--exit-after SECONDS] [--verbose] [--dump-frame PNG]
  read-only map view; set ROS_DOMAIN_ID=79 to watch a replay from run_demo.sh --replay
  --allow-goals   the 2D Goal tool sends NavigateToPose goals to /team7/navigate_to_pose
  --output-dir    run folder whose targets.csv and map.pgm the mission panel checks
                  (default: the newest folder in ~/team7_runs)
  --screens-dir   where quick screenshots go (default ~/team7_runs/screens)
  --light         keep the desktop GTK theme (default: dark)
  --verbose       print received message counts every 2 s
  --dump-frame    on exit save the map canvas (and PNG.ref.png drawn offscreen) for checks";

struct Options {
    exit_after: Option<f64>,
    verbose: bool,
    dump_frame: Option<String>,
    allow_goals: bool,
    output_dir: Option<PathBuf>,
    screens_dir: Option<PathBuf>,
    light: bool,
}

fn parse() -> Result<Options, String> {
    let mut o = Options { exit_after: None, verbose: false, dump_frame: None, allow_goals: false, output_dir: None, screens_dir: None, light: false };
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
            "--light" => o.light = true,
            "-h" | "--help" => return Err(String::new()),
            other => return Err(format!("unknown argument {other}")),
        }
    }
    Ok(o)
}

/// where the canvas sits in the window content, for turning window mouse positions into canvas
/// pixels (aurea reports mouse events in window-content coordinates only)
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

fn main() -> AureaResult<()> {
    let opts = parse().unwrap_or_else(|e| {
        if !e.is_empty() {
            eprintln!("{e}");
        }
        eprintln!("{USAGE}");
        std::process::exit(2)
    });
    if !opts.light && std::env::var_os("GTK_THEME").is_none() {
        // SAFETY: before any thread is started and before GTK reads its settings
        unsafe { std::env::set_var("GTK_THEME", "Adwaita:dark") };
    }
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
    let mut window = Window::new("team7 viewer", W as i32, H as i32)?;
    let canvas = Canvas::new(MIN_W, MIN_H, RendererBackend::Cpu)?;
    let [r, g, b, _] = palette::BACKGROUND;
    canvas.set_background_color(Color::rgb(r, g, b));
    let out_dir = opts.output_dir.clone().or_else(|| outputs::newest_run_dir(&outputs::runs_root()));
    let screens_dir = opts.screens_dir.clone().unwrap_or_else(|| outputs::runs_root().join("screens"));
    let root = gui::build(&mut window, app.clone(), model.clone(), canvas.clone(), GuiOptions { domain: domain.clone(), out_dir, screens_dir })?;
    let content = root.handle();
    window.set_content(root)?;
    let window = Rc::new(window);
    gui::set_window(Rc::downgrade(&window));
    let started = Instant::now();

    let drawn = app.clone();
    let shared = model.clone();
    let weak = Rc::downgrade(&window);
    let quit = interrupted.clone();
    let mut closing = false;
    let exit_after = opts.exit_after;
    let reference = opts.dump_frame.as_ref().map(|p| format!("{}.ref.png", p.trim_end_matches(".png")));
    canvas.set_draw_callback(move |ctx| {
        let timed_out = exit_after.is_some_and(|s| started.elapsed().as_secs_f64() >= s);
        let last = (timed_out || quit.load(Ordering::Relaxed)) && !closing;
        let now = Instant::now();
        let out = match shared.lock() {
            Ok(m) => {
                let out = drawn.borrow_mut().draw(ctx, &m, now);
                // the same frame drawn offscreen, to compare the window path with the reference path
                if let (true, Some(path)) = (last, reference.as_ref()) {
                    let (w, h) = (ctx.width(), ctx.height());
                    let mut r = CpuRasterizer::new(w, h);
                    if let Ok(mut off) = r.begin_frame() {
                        let _ = drawn.borrow_mut().draw(off.as_mut(), &m, now);
                    }
                    let _ = r.end_frame();
                    report(path, gui::save_rgba(path.as_ref(), r.pixels(), w, h));
                }
                out
            }
            Err(_) => Ok(()),
        };
        if last {
            closing = true;
            close_later(weak.clone());
        }
        out
    })?;
    if let Some(path) = opts.dump_frame.clone() {
        let c = canvas.clone();
        DUMP.with(|d| *d.borrow_mut() = Some(Box::new(move || report(&path, gui::save_canvas(&c, path.as_ref())))));
    }

    // redraw pump: the ROS thread marks the model dirty, this asks for a canvas frame and a panel refresh
    let id = canvas.id();
    let pump = {
        let (stop, dirty, model) = (stop.clone(), dirty.clone(), model.clone());
        let verbose = opts.verbose;
        std::thread::spawn(move || {
            let (mut last_draw, mut last_panel, mut last_log) = (Instant::now(), Instant::now(), Instant::now());
            while !stop.load(Ordering::Relaxed) {
                std::thread::sleep(FRAME);
                if dirty.swap(false, Ordering::Relaxed) || last_draw.elapsed() >= IDLE_REDRAW {
                    request_canvas_redraw(id);
                    last_draw = Instant::now();
                }
                if last_panel.elapsed() >= PANEL_REFRESH {
                    gui::refresh_soon();
                    last_panel = Instant::now();
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
        if debug_input && matches!(event, WindowEvent::MouseButton { .. } | WindowEvent::MouseMove { .. }) {
            eprintln!("team7_viz: input {event:?} canvas origin ({ox}, {oy}) size {:?}", events_canvas.size());
        }
        let (cw, ch) = events_canvas.size();
        let inside = |x: f64, y: f64| x >= ox && y >= oy && x < ox + f64::from(cw) && y < oy + f64::from(ch);
        let mut a = app.borrow_mut();
        let changed = match *event {
            WindowEvent::KeyInput { key, pressed: true, modifiers } => match a.key(key, modifiers) {
                Action::Quit => {
                    close_later(weak.clone());
                    false
                }
                Action::Screenshot => {
                    drop(a);
                    aurea::after_frame(|| {
                        gui::with(gui::Gui::screenshot);
                    });
                    return;
                }
                Action::Redraw => true,
                Action::None => false,
            },
            WindowEvent::MouseButton { button, pressed, x, y, .. } => {
                let b = match button {
                    MouseButton::Left => Some(MouseKey::Left),
                    MouseButton::Middle => Some(MouseKey::Middle),
                    MouseButton::Right => Some(MouseKey::Right),
                    MouseButton::Other(_) => None,
                };
                match (b, pressed) {
                    (Some(b), true) if inside(x, y) => a.press(b, x - ox, y - oy),
                    (Some(_), false) => a.release(x - ox, y - oy),
                    _ => false,
                }
            }
            WindowEvent::MouseMove { x, y, .. } => a.moved(x - ox, y - oy),
            WindowEvent::MouseWheel { delta_y, x, y, .. } if inside(x, y) => a.wheel(delta_y, x - ox, y - oy),
            _ => false,
        };
        if let Some(g) = a.pending_goal.take() {
            let _ = goal_tx.send(g);
        }
        drop(a);
        if changed {
            request_canvas_redraw(id);
            gui::refresh_soon();
        }
    });

    window.show();
    // GTK places the divider by its own rule until the window has a size; place it again then
    aurea::run_after(Duration::from_millis(300), || {
        gui::with(gui::Gui::place_divider);
    });
    let result = window.run();

    stop.store(true, Ordering::Relaxed);
    let _ = pump.join();
    let _ = ros_thread.join();
    if let Ok(m) = model.lock() {
        let line = m.counts_line();
        eprintln!("team7_viz: exit after {:.1} s, received {}", started.elapsed().as_secs_f64(), if line.is_empty() { "nothing" } else { &line });
    }
    result
}

thread_local! {
    /// saves the canvas before the window closes (`--dump-frame`)
    static DUMP: RefCell<Option<Box<dyn FnOnce()>>> = RefCell::new(None);
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
