//! Viewer state around one `Sim`: options, pause and speed, the trail, stepping against the wall clock.

use std::time::{Duration, Instant};

use tb4_sim::{Metrics, Sim, SimConfig, worlds};

pub const SPEEDS: [&str; 4] = ["1x", "4x", "16x", "max"];
const SPEED_FACTORS: [f64; 3] = [1.0, 4.0, 16.0];
/// sim time per frame is capped to this much wall time, so the window stays responsive
pub const STEP_BUDGET: Duration = Duration::from_millis(30);
const TRAIL_SPACING: f64 = 0.03;

#[derive(Debug, Clone, PartialEq)]
pub struct Options {
    pub world: String,
    pub seed: u64,
    pub perfect_pose: bool,
    pub sets: Vec<String>,
    /// index into `SPEEDS`
    pub speed: usize,
    pub paused: bool,
    /// close the window after this many wall seconds (for scripted checks)
    pub exit_after: Option<f64>,
}

impl Default for Options {
    fn default() -> Self {
        Self { world: "studio".into(), seed: 1, perfect_pose: false, sets: Vec::new(), speed: 0, paused: false, exit_after: None }
    }
}

pub const USAGE: &str = "usage: tb4_viewer [--world studio|studio_glass|small] [--seed 1] [--perfect-pose] [--set name=value]... \
[--speed 1x|4x|16x|max] [--paused] [--exit-after SECONDS]";

impl Options {
    pub fn parse(args: impl IntoIterator<Item = String>) -> Result<Self, String> {
        let mut o = Self::default();
        let mut args = args.into_iter();
        while let Some(a) = args.next() {
            let mut val = || args.next().ok_or_else(|| format!("{a} needs a value"));
            match a.as_str() {
                "--world" => o.world = val()?,
                "--seed" => o.seed = val()?.parse().map_err(|_| "bad --seed".to_owned())?,
                "--perfect-pose" => o.perfect_pose = true,
                "--set" => o.sets.push(val()?),
                "--speed" => {
                    let v = val()?;
                    o.speed = SPEEDS.iter().position(|s| *s == v).ok_or(format!("bad --speed {v}, have {SPEEDS:?}"))?;
                }
                "--paused" => o.paused = true,
                "--exit-after" => o.exit_after = Some(val()?.parse().map_err(|_| "bad --exit-after".to_owned())?),
                "-h" | "--help" => return Err(String::new()),
                other => return Err(format!("unknown argument {other}")),
            }
        }
        o.config()?;
        Ok(o)
    }

    pub fn config(&self) -> Result<SimConfig, String> {
        let w = worlds::by_name(&self.world).ok_or(format!("unknown world {}, have {:?}", self.world, worlds::NAMES))?;
        let mut cfg = SimConfig::team7(w);
        cfg.seed = self.seed;
        cfg.perfect_pose = self.perfect_pose;
        for kv in &self.sets {
            cfg.set(kv)?;
        }
        Ok(cfg)
    }
}

pub struct App {
    pub opts: Options,
    pub sim: Sim,
    /// true robot positions (world frame), one every few centimetres
    pub trail: Vec<(f64, f64)>,
    /// measured sim seconds per wall second
    pub rate: f64,
    last: Option<Instant>,
    /// sim time the wall clock asks for; steps are 10 ms, so the overshoot carries over
    target: f64,
}

impl App {
    pub fn new(opts: Options) -> Result<Self, String> {
        let sim = Sim::new(opts.config()?);
        let mut app = Self { opts, sim, trail: Vec::new(), rate: 0.0, last: None, target: 0.0 };
        app.push_trail();
        Ok(app)
    }

    /// rebuild the sim from the current options (world, seed); keeps pause and speed
    pub fn reset(&mut self) {
        match self.opts.config() {
            Ok(cfg) => {
                self.sim = Sim::new(cfg);
                self.trail.clear();
                self.push_trail();
                self.rate = 0.0;
                self.target = 0.0;
            }
            Err(e) => eprintln!("tb4_viewer: {e}"),
        }
    }

    pub fn set_world(&mut self, i: usize) {
        if let Some(name) = worlds::NAMES.get(i)
            && self.opts.world != *name
        {
            self.opts.world = (*name).to_owned();
            self.reset();
        }
    }

    pub fn world_index(&self) -> usize {
        worlds::NAMES.iter().position(|n| *n == self.opts.world).unwrap_or(0)
    }

    pub fn set_seed(&mut self, seed: u64) {
        self.opts.seed = seed;
        self.reset();
    }

    fn push_trail(&mut self) {
        let p = self.sim.state().truth;
        if self.trail.last().is_none_or(|&(x, y)| (p.x - x).hypot(p.y - y) >= TRAIL_SPACING) {
            self.trail.push((p.x, p.y));
        }
    }

    pub fn step(&mut self) {
        self.sim.step();
        self.push_trail();
    }

    /// step `dt` seconds of sim time at once (single-stepping while paused)
    pub fn step_for(&mut self, dt: f64) {
        let end = self.sim.time() + dt;
        while self.sim.time() + 1e-9 < end {
            self.step();
        }
    }

    /// advance the sim by the wall time since the last call times the speed; at most `budget` of
    /// wall time is spent stepping. Call once per frame.
    pub fn advance(&mut self, now: Instant, budget: Duration) {
        let dt = self.last.map_or(0.0, |l| now.duration_since(l).as_secs_f64()).min(0.25);
        self.last = Some(now);
        let t0 = self.sim.time();
        if self.opts.paused || dt <= 0.0 {
            self.rate = 0.0;
            self.target = t0;
            return;
        }
        let factor = SPEED_FACTORS.get(self.opts.speed).copied();
        // never ask for more than one frame behind, so a slow stretch is dropped, not caught up
        self.target = self.target.max(t0 - 0.01) + factor.map_or(0.0, |f| f * dt);
        let start = Instant::now();
        loop {
            if factor.is_some() && self.sim.time() + 1e-9 >= self.target {
                break;
            }
            if start.elapsed() >= budget {
                self.target = self.sim.time();
                break;
            }
            self.step();
        }
        let r = (self.sim.time() - t0) / dt;
        self.rate = if self.rate == 0.0 { r } else { 0.8 * self.rate + 0.2 * r };
    }
}

fn opt_t(t: Option<f64>) -> String {
    t.map_or_else(|| "-".to_owned(), |t| format!("{t:.1} s"))
}

/// the status panel text
pub fn status_lines(m: &Metrics, opts: &Options, rate: f64) -> Vec<String> {
    let pct = if m.reference_area > 0.0 { 100.0 * m.final_area / m.reference_area } else { 0.0 };
    let e = &m.explorer;
    let mut l = vec![
        format!("world {}  seed {}{}", m.world, m.seed, if m.perfect_pose { "  perfect pose" } else { "" }),
        format!("sim time   {:8.1} s", m.duration),
        format!(
            "speed      {:>8}  ({:.1}x){}",
            SPEEDS.get(opts.speed).copied().unwrap_or("?"),
            rate,
            if opts.paused { "  PAUSED" } else { "" }
        ),
        format!("mapped     {:6.1} %  ({:.1} / {:.1} m2)", pct, m.final_area, m.reference_area),
        format!("distance   {:8.2} m", m.distance),
        format!("driving    {:8.1} s", m.time_driving),
        format!("turning    {:8.1} s", m.time_turning),
        format!("still      {:8.1} s", m.time_still),
        format!("goals sent {:4}  ok {:4}", e.sent, e.succeeded),
        format!("switched   {:4}  cancel {:4}", e.switched, e.canceled),
        format!("bumps      {:4}  contacts {:4}", m.bumps, m.collisions),
        "markers (first readable / within 2 m)".to_owned(),
    ];
    for r in &m.markers {
        l.push(format!("  id {:3}  {:>9} / {:>9}", r.id, opt_t(r.first_readable), opt_t(r.first_within_2m)));
    }
    l
}

#[cfg(test)]
mod tests {
    use super::*;

    fn args(s: &str) -> Vec<String> {
        s.split_whitespace().map(str::to_owned).collect()
    }

    #[test]
    fn parses_flags_like_the_run_example() {
        let o = Options::parse(args("--world small --seed 7 --perfect-pose --set vx_max=0.2 --speed max")).unwrap();
        assert_eq!((o.world.as_str(), o.seed, o.perfect_pose, o.speed), ("small", 7, true, 3));
        assert_eq!(o.config().unwrap().nav.mppi.vx_max, 0.2);
        assert!(Options::parse(args("--set nope=1")).is_err());
        assert!(Options::parse(args("--world moon")).is_err());
    }

    #[test]
    fn advance_follows_speed_and_pause() {
        let mut o = Options::parse(args("--world small --perfect-pose")).unwrap();
        o.speed = 1; // 4x
        let mut app = App::new(o).unwrap();
        let t0 = Instant::now();
        app.advance(t0, STEP_BUDGET);
        app.advance(t0 + Duration::from_millis(100), Duration::from_secs(10));
        assert!((app.sim.time() - 0.4).abs() < 0.011, "{}", app.sim.time());
        app.opts.paused = true;
        app.advance(t0 + Duration::from_millis(200), Duration::from_secs(10));
        assert!((app.sim.time() - 0.4).abs() < 0.011);
        app.reset();
        assert_eq!(app.sim.time(), 0.0);
        assert_eq!(app.trail.len(), 1);
    }

    #[test]
    fn many_short_frames_do_not_drift() {
        let mut app = App::new(Options::parse(args("--world small --perfect-pose")).unwrap()).unwrap();
        let t0 = Instant::now();
        // 60 frames of 1/60 s at 1x: 10 ms steps must not overshoot every frame
        for i in 0..=60u64 {
            app.advance(t0 + Duration::from_micros(i * 16_667), Duration::from_secs(10));
        }
        assert!((app.sim.time() - 1.0).abs() < 0.011, "{}", app.sim.time());
    }
}
