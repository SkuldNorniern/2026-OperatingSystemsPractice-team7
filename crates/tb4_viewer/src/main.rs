//! cargo run --release -p tb4_viewer -- [--world studio|studio_glass|small] [--seed 1] [--perfect-pose]
//!     [--set name=value]... [--speed 1x|4x|16x|max] [--paused] [--exit-after SECONDS]

use std::cell::RefCell;
use std::rc::{Rc, Weak};
use std::time::Instant;

use aurea::prelude::*;
use aurea::render::request_canvas_redraw;
use tb4_sim::SET_NAMES;
use tb4_viewer::app::{App, Options, USAGE};
use tb4_viewer::render::Viewer;
use tb4_viewer::scene::colors;

const W: u32 = 1280;
const H: u32 = 820;
/// the canvas is made small: its first size is still a minimum when the window starts tiled
/// (aurea ed32453 lets it shrink after the first layout, but a tiled start then overflows)
const MIN_W: u32 = 320;
const MIN_H: u32 = 240;

fn main() -> AureaResult<()> {
    let opts = Options::parse(std::env::args().skip(1)).unwrap_or_else(|e| {
        if !e.is_empty() {
            eprintln!("{e}");
        }
        eprintln!("{USAGE}");
        eprintln!("--set names: {}", SET_NAMES.join(" "));
        std::process::exit(2)
    });
    let exit_after = opts.exit_after;
    let app = App::new(opts).unwrap_or_else(|e| {
        eprintln!("{e}");
        std::process::exit(2)
    });
    let viewer = Rc::new(RefCell::new(Viewer::new(app)));

    let mut window = Window::new("tb4 sim viewer", W as i32, H as i32)?;
    let canvas = Canvas::new(MIN_W, MIN_H, RendererBackend::Cpu)?;
    let [r, g, b, _] = colors::BACKGROUND;
    canvas.set_background_color(Color::rgb(r, g, b));
    let mut layout = Stack::new(Orientation::Vertical)?;
    layout.set_padding(0)?;
    // weight 1: the canvas takes all the room the window gets (weight 0 packs it at its minimum height)
    layout.add_weighted(canvas.clone(), 1.0)?;
    window.set_content(layout)?;
    let window = Rc::new(window);
    let started = Instant::now();

    let drawn = Rc::clone(&viewer);
    let weak = Rc::downgrade(&window);
    let mut closing = false;
    canvas.set_draw_callback(move |ctx| {
        let mut v = drawn.borrow_mut();
        let out = v.frame(ctx);
        if let Some(limit) = exit_after
            && started.elapsed().as_secs_f64() >= limit
            && !closing
        {
            closing = true;
            let s = v.app.sim.state();
            eprintln!("tb4_viewer: exit after {limit} s wall, sim time {:.1} s, robot at ({:.2}, {:.2})", s.t, s.truth.x, s.truth.y);
            close_later(weak.clone());
        }
        out
    })?;
    // redraw every frame: the draw callback advances the sim
    canvas.on_frame(|_| true);

    let id = canvas.id();
    let weak = Rc::downgrade(&window);
    window.on_event(move |event| {
        let mut v = viewer.borrow_mut();
        if let WindowEvent::KeyInput { key, pressed: true, .. } = event
            && v.key(*key)
        {
            close_later(weak.clone());
        }
        if v.ui.handle(event) {
            request_canvas_redraw(id);
        }
    });

    window.show();
    window.run()
}

/// close once the current frame or event is done (`WindowProxy::dispatch` is not drained under `run`)
fn close_later(window: Weak<Window>) {
    aurea::after_frame(move || {
        if let Some(w) = window.upgrade() {
            w.close();
        }
    });
}
