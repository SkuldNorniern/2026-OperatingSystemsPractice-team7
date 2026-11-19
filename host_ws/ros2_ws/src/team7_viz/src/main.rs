//! team7_viz [--exit-after SECONDS] [--verbose]
//!
//! Read-only: subscribes on the current ROS_DOMAIN_ID (79 for replays) and never publishes.

use std::cell::RefCell;
use std::rc::{Rc, Weak};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use aurea::prelude::*;
use aurea::render::{CpuRasterizer, Renderer, request_canvas_redraw};
use team7_viz::model::Model;
use team7_viz::ros;
use team7_viz::view::Viewer;
use tb4_draw::palette;

const W: u32 = 1280;
const H: u32 = 820;
/// the canvas size request is a minimum in GTK: keep it small so a tiling compositor can shrink the window
const MIN_W: u32 = 320;
const MIN_H: u32 = 240;
/// redraw at most this often while data streams in, and at least every `IDLE_REDRAW` (rates decay)
const FRAME: Duration = Duration::from_millis(50);
const IDLE_REDRAW: Duration = Duration::from_millis(500);

const USAGE: &str = "usage: team7_viz [--exit-after SECONDS] [--verbose] [--dump-frame PNG]\n  \
read-only viewer; set ROS_DOMAIN_ID=79 to watch a replay from run_demo.sh --replay\n  \
--verbose prints received message counts every 2 s; --dump-frame saves the last frame on exit";

struct Options {
    exit_after: Option<f64>,
    verbose: bool,
    dump_frame: Option<String>,
}

fn parse() -> Result<Options, String> {
    let mut o = Options { exit_after: None, verbose: false, dump_frame: None };
    let mut args = std::env::args().skip(1);
    while let Some(a) = args.next() {
        match a.as_str() {
            "--exit-after" => {
                let v = args.next().ok_or("--exit-after needs a value")?;
                o.exit_after = Some(v.parse().map_err(|_| format!("bad --exit-after {v}"))?);
            }
            "--verbose" | "-v" => o.verbose = true,
            "--dump-frame" => o.dump_frame = Some(args.next().ok_or("--dump-frame needs a path")?),
            "-h" | "--help" => return Err(String::new()),
            other => return Err(format!("unknown argument {other}")),
        }
    }
    Ok(o)
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
    eprintln!("team7_viz: ROS_DOMAIN_ID={domain}, read-only");

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

    let ros_thread = {
        let (model, stop, dirty) = (model.clone(), stop.clone(), dirty.clone());
        std::thread::Builder::new()
            .name("team7_viz_ros".into())
            .spawn(move || {
                if let Err(e) = ros::run(model, stop, dirty) {
                    eprintln!("team7_viz: ROS node failed: {e} (is ROS sourced?)");
                }
            })
            .expect("spawn ROS thread")
    };

    let viewer = Rc::new(RefCell::new(Viewer::new()));
    let mut window = Window::new("team7 viewer", W as i32, H as i32)?;
    let canvas = Canvas::new(MIN_W, MIN_H, RendererBackend::Cpu)?;
    let [r, g, b, _] = palette::BACKGROUND;
    canvas.set_background_color(Color::rgb(r, g, b));
    let mut layout = Stack::new(Orientation::Vertical)?;
    layout.set_padding(0)?;
    // weight 1: the canvas takes all the room the window gets (weight 0 packs it at its minimum height)
    layout.add_weighted(canvas.clone(), 1.0)?;
    window.set_content(layout)?;
    let window = Rc::new(window);
    let started = Instant::now();

    let drawn = Rc::clone(&viewer);
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
                    report_png(path, write_png(path, r.pixels(), w, h));
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
    // `close_later` runs after the frame is presented, so the dump below sees the final frame
    if let Some(path) = opts.dump_frame.clone() {
        let c = canvas.clone();
        DUMP.with(|d| *d.borrow_mut() = Some(Box::new(move || dump_frame(&c, &path))));
    }

    // redraw pump: the ROS thread marks the model dirty, this asks the canvas for a frame
    let id = canvas.id();
    let pump = {
        let (stop, dirty, model) = (stop.clone(), dirty.clone(), model.clone());
        let verbose = opts.verbose;
        std::thread::spawn(move || {
            let (mut last_draw, mut last_log) = (Instant::now(), Instant::now());
            while !stop.load(Ordering::Relaxed) {
                std::thread::sleep(FRAME);
                if dirty.swap(false, Ordering::Relaxed) || last_draw.elapsed() >= IDLE_REDRAW {
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
    window.on_event(move |event| {
        let mut v = viewer.borrow_mut();
        let changed = match event {
            WindowEvent::KeyInput { key, pressed: true, .. } => {
                if v.key(*key) {
                    close_later(weak.clone());
                }
                true
            }
            e => v.pointer(e),
        };
        if changed {
            request_canvas_redraw(id);
        }
    });

    window.show();
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

/// the canvas's last presented frame as a PNG (debug: shows what the window really drew)
fn dump_frame(canvas: &Canvas, path: &str) {
    match canvas.with_frame_pixels(|px, w, h| write_png(path, px, w, h)) {
        Some(r) => report_png(path, r),
        None => eprintln!("team7_viz: no frame to save"),
    }
}

/// 0xAARRGGBB pixels to an RGBA PNG
fn write_png(path: &str, px: &[u32], w: u32, h: u32) -> Result<(), String> {
    let rgba: Vec<u8> = px.iter().flat_map(|p| [(p >> 16) as u8, (p >> 8) as u8, *p as u8, 255]).collect();
    let file = std::fs::File::create(path).map_err(|e| e.to_string())?;
    let mut enc = png::Encoder::new(std::io::BufWriter::new(file), w, h);
    enc.set_color(png::ColorType::Rgba);
    enc.set_depth(png::BitDepth::Eight);
    enc.write_header().and_then(|mut wr| wr.write_image_data(&rgba)).map_err(|e| e.to_string())
}

fn report_png(path: &str, r: Result<(), String>) {
    match r {
        Ok(()) => eprintln!("team7_viz: frame saved to {path}"),
        Err(e) => eprintln!("team7_viz: cannot save {path}: {e}"),
    }
}
