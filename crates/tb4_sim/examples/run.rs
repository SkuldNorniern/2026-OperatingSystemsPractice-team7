//! One closed-loop run: prints the metrics table and writes them as JSON.
//!
//! cargo run --release -p tb4_sim --example run -- --world studio --seconds 600 --seed 1 [--perfect-pose]
//!     [--json out.json] [--log]

use tb4_sim::{Sim, SimConfig, worlds};

fn main() {
    let mut world = "studio".to_owned();
    let mut seconds = 600.0;
    let mut seed = 1u64;
    let mut perfect = false;
    let mut json = None;
    let mut log = false;
    let mut ascii = false;
    let mut sets = Vec::new();
    let mut args = std::env::args().skip(1);
    while let Some(a) = args.next() {
        let mut val = || args.next().unwrap_or_else(|| usage(&format!("{a} needs a value")));
        match a.as_str() {
            "--world" => world = val(),
            "--seconds" => seconds = val().parse().unwrap_or_else(|_| usage("bad --seconds")),
            "--seed" => seed = val().parse().unwrap_or_else(|_| usage("bad --seed")),
            "--perfect-pose" => perfect = true,
            "--json" => json = Some(val()),
            "--log" => log = true,
            "--ascii" => ascii = true,
            "--set" => sets.push(val()),
            "-h" | "--help" => usage(""),
            other => usage(&format!("unknown argument {other}")),
        }
    }
    let w = worlds::by_name(&world).unwrap_or_else(|| usage(&format!("unknown world {world}, have {:?}", worlds::NAMES)));
    let mut cfg = SimConfig::team7(w);
    cfg.seed = seed;
    cfg.perfect_pose = perfect;
    for kv in &sets {
        set(&mut cfg, kv).unwrap_or_else(|e| usage(&e));
    }
    let mut sim = Sim::new(cfg);
    sim.keep_log = log;
    sim.run_until(seconds);
    if log {
        for (t, m) in sim.log() {
            println!("[{t:7.2}] {m}");
        }
    }
    if ascii {
        print!("{}", sim.state().ascii_map(0.1));
    }
    let m = sim.metrics();
    print!("{}", m.table());
    let path = json.unwrap_or_else(|| format!("tb4_sim_{world}_seed{seed}{}.json", if perfect { "_perfect" } else { "" }));
    match std::fs::write(&path, m.to_json()) {
        Ok(()) => println!("metrics written to {path}"),
        Err(e) => eprintln!("cannot write {path}: {e}"),
    }
}

fn usage(msg: &str) -> ! {
    if !msg.is_empty() {
        eprintln!("{msg}");
    }
    eprintln!("usage: run [--world studio|studio_glass|small] [--seconds 600] [--seed 1] [--perfect-pose] [--json FILE] [--log] [--ascii] [--set name=value]");
    eprintln!("--set names: {}", SET_NAMES.join(" "));
    std::process::exit(2)
}

const SET_NAMES: [&str; 24] = [
    "goal_timeout", "progress_timeout", "retry_delay", "blacklist_duration", "turn_weight", "distance_weight",
    "size_weight", "min_goal_distance", "goal_standoff", "blacklist_radius", "camera_range", "view_standoff",
    "view_detour", "preempt_distance", "switch_cooldown", "min_unknown_area", "min_frontier_cells", "plan_with_scan", "look_around_turns", "camera_coverage", "vx_max", "wz_max",
    "path_switch_ratio", "slow_on_tentative",
];

fn set(cfg: &mut SimConfig, kv: &str) -> Result<(), String> {
    let (k, v) = kv.split_once('=').ok_or(format!("--set {kv}: want name=value"))?;
    let x: f64 = v.parse().map_err(|_| format!("--set {kv}: not a number"))?;
    let e = &mut cfg.explorer;
    match k {
        "goal_timeout" => e.goal_timeout = x,
        "progress_timeout" => e.progress_timeout = x,
        "retry_delay" => e.retry_delay = x,
        "blacklist_duration" => e.blacklist_duration = x,
        "turn_weight" => e.turn_weight = x,
        "distance_weight" => e.explore.distance_weight = x,
        "size_weight" => e.explore.size_weight = x,
        "min_goal_distance" => e.explore.min_goal_distance = x,
        "goal_standoff" => e.explore.goal_standoff = x,
        "blacklist_radius" => e.explore.blacklist_radius = x,
        "camera_range" => e.coverage.iter_mut().for_each(|c| c.range = x),
        "view_standoff" => e.coverage.iter_mut().for_each(|c| c.standoff = x),
        "view_detour" => e.view_detour = x,
        "preempt_distance" => e.preempt_distance = x,
        "switch_cooldown" => e.switch_cooldown = x,
        "min_unknown_area" => e.explore.min_unknown_area = x,
        "min_frontier_cells" => e.explore.min_frontier_cells = x as usize,
        "look_around_turns" => e.look_around_turns = x as u32,
        "camera_coverage" if x == 0.0 => e.coverage = None,
        "camera_coverage" => {}
        "vx_max" => cfg.nav.mppi.vx_max = x,
        "wz_max" => cfg.nav.mppi.wz_max = x,
        "path_switch_ratio" => cfg.nav.path_switch_ratio = x,
        "plan_with_scan" => cfg.nav.plan_with_scan = x != 0.0,
        "slow_on_tentative" => cfg.slow.enabled = x != 0.0,
        _ => return Err(format!("--set: unknown name {k}")),
    }
    Ok(())
}
