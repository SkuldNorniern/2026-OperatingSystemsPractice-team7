//! One closed-loop run: prints the metrics table, and writes them as JSON with `--json FILE`.
//!
//! cargo run --release -p tb4_sim --example run -- --world studio --seconds 600 --seed 1 [--perfect-pose]
//!     [--json out.json] [--log]

use tb4_sim::{SET_NAMES, Sim, SimConfig, worlds};

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
        cfg.set(kv).unwrap_or_else(|e| usage(&e));
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
    if let Some(path) = json {
        match std::fs::write(&path, m.to_json()) {
            Ok(()) => println!("metrics written to {path}"),
            Err(e) => eprintln!("cannot write {path}: {e}"),
        }
    }
}

fn usage(msg: &str) -> ! {
    if !msg.is_empty() {
        eprintln!("{msg}");
    }
    eprintln!("usage: run [--world studio|studio_glass|small] [--seconds 600] [--seed 1] [--perfect-pose] [--json FILE] [--log] [--ascii] [--set name=value]");
    eprintln!("--set names: {} mppi.<MppiParams field>", SET_NAMES.join(" "));
    std::process::exit(2)
}
