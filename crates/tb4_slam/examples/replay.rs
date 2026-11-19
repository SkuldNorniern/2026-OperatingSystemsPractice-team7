// Offline replay of recorded lidar scans through the same path as the live nodes:
// scan_filter (tb4_scan deskew + filter) -> slam node (laser -> base_link points) -> Slam::update.
// stdin: host_ws/tools/bag_scans.py output. Prints per-200-scan timing, loop closure events with the
// pose-graph correction they applied, map jitter between 1 s map publishes and a final summary.
// usage: bag_scans.py BAG | cargo run --release -p tb4_slam --example replay -- [key=value ...] [map=out.pgm]
use std::collections::HashSet;
use std::io::{BufReader, Read};
use std::time::Instant;

use tb4_core::{Grid, Isometry3, Pose2, Stamp, TfBuffer, Vector3};
use tb4_scan::{Scan, ScanParams};
use tb4_slam::se2::{apply, wrap};
use tb4_slam::{Slam, SlamParams};

const MAP_PERIOD: f64 = 1.0;
const WINDOW: u64 = 200;

fn read_f64<R: Read>(r: &mut R) -> Option<f64> {
    let mut b = [0u8; 8];
    r.read_exact(&mut b).ok().map(|_| f64::from_le_bytes(b))
}

fn read_f32<R: Read>(r: &mut R) -> Option<f32> {
    let mut b = [0u8; 4];
    r.read_exact(&mut b).ok().map(|_| f32::from_le_bytes(b))
}

struct Frame {
    t: f64,
    odom: Pose2,
}

enum Event {
    Tf(f64, Pose2),
    Twist(f64, f64),
    Scan(f64, Scan),
}

/// laser pose in base_link and all events, see host_ws/tools/bag_scans.py
fn read_events<R: Read>(r: &mut R) -> (Pose2, Vec<Event>) {
    let mut tag = [0u8; 1];
    r.read_exact(&mut tag).expect("header");
    assert_eq!(tag[0], 0, "stream must start with the laser pose");
    let lx = Pose2::new(read_f64(r).unwrap(), read_f64(r).unwrap(), read_f64(r).unwrap());
    let mut ev = Vec::new();
    while r.read_exact(&mut tag).is_ok() {
        ev.push(match tag[0] {
            1 => Event::Tf(read_f64(r).unwrap(), Pose2::new(read_f64(r).unwrap(), read_f64(r).unwrap(), read_f64(r).unwrap())),
            2 => Event::Twist(read_f64(r).unwrap(), read_f64(r).unwrap()),
            3 => {
                let t = read_f64(r).unwrap();
                let angle_min = read_f32(r).unwrap() as f64;
                let angle_increment = read_f32(r).unwrap() as f64;
                let time_increment = read_f32(r).unwrap() as f64;
                let mut nb = [0u8; 4];
                r.read_exact(&mut nb).unwrap();
                let mut raw = vec![0u8; 4 * u32::from_le_bytes(nb) as usize];
                r.read_exact(&mut raw).unwrap();
                let ranges = raw.chunks_exact(4).map(|c| f32::from_le_bytes(c.try_into().unwrap())).collect();
                Event::Scan(t, Scan { angle_min, angle_increment, time_increment, ranges })
            }
            t => panic!("bad event tag {t}"),
        });
    }
    (lx, ev)
}

/// index of the event after the last odometry reset (odom jumps back onto the origin), else 0
fn after_reset(ev: &[Event]) -> usize {
    let mut last: Option<Pose2> = None;
    let mut start = 0;
    for (k, e) in ev.iter().enumerate() {
        if let Event::Tf(_, p) = e {
            if let Some(a) = last {
                let jump = a.distance_to(p.x, p.y).max(wrap(a.yaw - p.yaw).abs());
                if jump > 0.05 && p.x.hypot(p.y) < 0.005 && p.yaw.abs() < 0.005 {
                    start = k;
                }
            }
            last = Some(*p);
        }
    }
    start
}

fn iso(p: Pose2) -> Isometry3<f64> {
    Isometry3::new(Vector3::new(p.x, p.y, 0.0), Vector3::new(0.0, 0.0, p.yaw))
}

fn set_param(p: &mut SlamParams, key: &str, val: &str) -> bool {
    let f = || val.parse::<f64>().ok();
    let b = || matches!(val, "1" | "true");
    match key {
        "resolution" => p.resolution = f().unwrap(),
        "min_travel_dist" => p.min_travel_dist = f().unwrap(),
        "min_travel_yaw" => p.min_travel_yaw = f().unwrap(),
        "max_range" => p.max_range = f().unwrap(),
        "min_match_score" => p.min_match_score = f().unwrap(),
        "loop_search_radius" => p.loop_search_radius = f().unwrap(),
        "loop_prefer_oldest" => p.loop_prefer_oldest = b(),
        "loop_min_gap" => p.loop_min_gap = f().unwrap() as usize,
        "loop_cooldown" => p.loop_cooldown = f().unwrap() as usize,
        "loop_min_score" => p.loop_min_score = f().unwrap(),
        "icp" => p.icp = b(),
        "icp_max_keyframes" => p.icp_max_keyframes = f().unwrap() as usize,
        "redraw_min_shift" => p.redraw_min_shift = f().unwrap(),
        "redraw_min_turn" => p.redraw_min_turn = f().unwrap(),
        "opt_iterations" => p.opt_iterations = f().unwrap() as usize,
        _ => return false,
    }
    true
}

fn occupied(g: &Grid) -> HashSet<(i32, i32)> {
    let res = g.meta.resolution;
    let (x0, y0) = ((g.meta.origin_x / res).round() as i32, (g.meta.origin_y / res).round() as i32);
    let w = g.meta.width as usize;
    g.data
        .iter()
        .enumerate()
        .filter(|(_, v)| **v == 100)
        .map(|(i, _)| (x0 + (i % w) as i32, y0 + (i / w) as i32))
        .collect()
}

fn write_pgm(path: &str, g: &Grid) {
    let (w, h) = (g.meta.width as usize, g.meta.height as usize);
    let mut out = format!("P5\n{w} {h}\n255\n").into_bytes();
    for y in (0..h).rev() {
        out.extend(g.data[y * w..(y + 1) * w].iter().map(|&v| match v {
            100 => 0u8,
            0 => 254,
            _ => 205,
        }));
    }
    std::fs::write(path, out).expect("write map");
}

#[derive(Default, Clone, Copy)]
struct Acc {
    n: u64,
    sum: f64,
    max: f64,
}

impl Acc {
    fn add(&mut self, v: f64) {
        self.n += 1;
        self.sum += v;
        self.max = self.max.max(v);
    }

    fn mean(&self) -> f64 {
        self.sum / self.n.max(1) as f64
    }
}

impl std::fmt::Display for Acc {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "mean {:.4} max {:.4} (n {})", self.mean(), self.max, self.n)
    }
}

fn main() {
    let mut params = SlamParams::default();
    let (mut map_out, mut time_offset) = (None, 0.0);
    let mut sp = ScanParams::default();
    for a in std::env::args().skip(1) {
        let (k, v) = a.split_once('=').expect("key=value");
        if k == "map" {
            map_out = Some(v.to_owned());
        } else if k == "time_offset" {
            // look odometry up at scan stamp + this (s)
            time_offset = v.parse().expect("seconds");
        } else if k == "stamp_at_end" {
            // scan_filter parameter: deskew to the last beam instead of the first
            sp.stamp_at_end = matches!(v, "1" | "true");
        } else if !set_param(&mut params, k, v) {
            panic!("unknown parameter {k}");
        }
    }
    let (lx, events) = read_events(&mut BufReader::new(std::io::stdin().lock()));
    let start = after_reset(&events);
    let mut slam = Slam::new(params);
    // same buffer as the slam node: 10 s, odometry held up to 0.1 s past the newest sample
    let mut tf = TfBuffer::new(10.0, 0.1);
    let (mut v, mut w) = (0.0, 0.0);
    let (mut tf_failures, mut t0) = (0u64, None);

    let (mut frames, mut last_t) = (0u64, 0.0);
    let (mut win, mut all) = (Acc::default(), Acc::default());
    // per window: scan match, icp, loop search incl. closing (ms summed)
    let mut win_parts = [0.0f64; 3];
    let mut score = Acc::default();
    let mut prev_kfs: Vec<Pose2> = Vec::new();
    // pose-graph correction per optimisation: mean / max keyframe shift
    let (mut kf_shift_mean, mut kf_shift_max, mut kf_turn_max) = (Acc::default(), Acc::default(), Acc::default());
    // map->odom change per processed scan, on scans that closed a loop and on the others
    let (mut m2o_loop, mut m2o_other, mut yaw_loop, mut yaw_other) = (Acc::default(), Acc::default(), Acc::default(), Acc::default());
    let (mut dirty, mut closed_since_pub, mut next_map) = (false, false, MAP_PERIOD);
    let mut prev_occ: Option<HashSet<(i32, i32)>> = None;
    // occupied cells of the previous published map that are gone in the next one
    let (mut lost_loop, mut lost_other) = (Acc::default(), Acc::default());
    let mut last_closures = 0;

    for e in &events[start..] {
        let (stamp, raw) = match e {
            Event::Tf(t, p) => {
                tf.insert("odom", "base_link", Stamp((t * 1e9).round() as i64), iso(*p), false);
                continue;
            }
            Event::Twist(a, b) => {
                (v, w) = (*a, *b);
                continue;
            }
            Event::Scan(t, s) => (*t, s),
        };
        let odom = match tf.lookup("odom", "base_link", Stamp(((stamp + time_offset) * 1e9).round() as i64)) {
            Ok(o) => Pose2::from_isometry(&o),
            Err(_) => {
                tf_failures += 1;
                continue;
            }
        };
        let f = Frame { t: stamp - *t0.get_or_insert(stamp), odom };
        frames += 1;
        last_t = f.t;
        // scan_filter: deskew with the latest odom twist, then range / median / isolated filter
        let scan = if sp.deskew { tb4_scan::deskew(raw, v, w, sp.stamp_at_end) } else { raw.clone() };
        let (scan, _) = tb4_scan::filter(&scan, &sp);
        // slam node: finite returns >= range_min, into base_link
        let pts: Vec<(f64, f64)> = tb4_scan::to_points(&scan)
            .into_iter()
            .filter(|p| p.0.hypot(p.1) >= sp.min_range as f64)
            .map(|p| apply(lx, p))
            .collect();

        let m2o_before = slam.map_to_odom();
        let t0 = Instant::now();
        let res = slam.update(f.odom, &pts);
        let dt = t0.elapsed().as_secs_f64() * 1e3;
        if res.is_some() {
            dirty = true;
            let st = slam.stats();
            let n = st.scans_processed;
            score.add(st.last_score);
            win.add(dt);
            all.add(dt);
            win_parts[0] += st.last_match_ms;
            win_parts[1] += st.last_icp_ms;
            win_parts[2] += st.last_loop_search_ms;
            let m2o = slam.map_to_odom();
            let jump = m2o.distance_to(m2o_before.x, m2o_before.y);
            let jump_yaw = wrap(m2o.yaw - m2o_before.yaw).abs();
            let kfs = slam.keyframes();
            if st.loop_closures > last_closures {
                last_closures = st.loop_closures;
                closed_since_pub = true;
                m2o_loop.add(jump);
                yaw_loop.add(jump_yaw);
                let shifts: Vec<f64> = prev_kfs.iter().zip(&kfs).map(|(a, b)| a.distance_to(b.x, b.y)).collect();
                let mean = shifts.iter().sum::<f64>() / shifts.len().max(1) as f64;
                let max = shifts.iter().copied().fold(0.0, f64::max);
                let turn = prev_kfs.iter().zip(&kfs).map(|(a, b)| wrap(a.yaw - b.yaw).abs()).fold(0.0, f64::max);
                kf_turn_max.add(turn);
                kf_shift_mean.add(mean);
                kf_shift_max.add(max);
                let (i, j) = slam.loop_edges().last().copied().unwrap_or_default();
                let path: f64 = kfs[i..=j].windows(2).map(|w| w[0].distance_to(w[1].x, w[1].y)).sum();
                println!(
                    "t {:6.1} loop #{:3} edge {:4}->{:4} gap {:3} kf {:5.2} m path | match moved kf {:.3} m {:.4} rad | kf shift mean {:.4} max {:.4} m turn max {:.4} rad | m2o jump {:.4} m {:.4} rad | opt {:.1} rebuild {:.1} update {:.1} ms",
                    f.t,
                    st.loop_closures,
                    i,
                    j,
                    j - i,
                    path,
                    st.last_loop_correction.0,
                    st.last_loop_correction.1,
                    mean,
                    max,
                    turn,
                    jump,
                    jump_yaw,
                    st.last_optimisation_ms,
                    st.last_rebuild_ms,
                    dt
                );
            } else {
                m2o_other.add(jump);
                yaw_other.add(jump_yaw);
            }
            prev_kfs = kfs;
            if n % WINDOW == 0 {
                let w = WINDOW as f64;
                println!(
                    "t {:6.1} {:5} scans: window mean {:6.1} max {:6.1} ms (match {:5.1} icp {:5.1} loop {:5.1}) | run mean {:5.1} max {:6.1} | kf {} loops {} rejected {} fallbacks {} icp {}/{}",
                    f.t,
                    n,
                    win.mean(),
                    win.max,
                    win_parts[0] / w,
                    win_parts[1] / w,
                    win_parts[2] / w,
                    all.mean(),
                    all.max,
                    st.keyframes,
                    st.loop_closures,
                    st.loop_rejections,
                    st.match_fallbacks,
                    st.icp_used,
                    st.icp_used + st.icp_rejected
                );
                (win, win_parts) = (Acc::default(), [0.0; 3]);
            }
        }
        if f.t >= next_map {
            next_map = f.t + MAP_PERIOD;
            if dirty {
                dirty = false;
                let occ = occupied(&slam.occupancy());
                if let Some(p) = &prev_occ {
                    let lost = p.difference(&occ).count() as f64;
                    if closed_since_pub {
                        lost_loop.add(lost);
                    } else {
                        lost_other.add(lost);
                    }
                }
                closed_since_pub = false;
                prev_occ = Some(occ);
            }
        }
    }

    let st = slam.stats();
    if win.n > 0 {
        println!("last {} scans: window mean {:.1} max {:.1} ms", win.n, win.mean(), win.max);
    }
    let g = slam.occupancy();
    let occ = g.data.iter().filter(|&&v| v == 100).count();
    let known = g.data.iter().filter(|&&v| v != -1).count();
    let mut lost_all = lost_loop;
    lost_all.n += lost_other.n;
    lost_all.sum += lost_other.sum;
    lost_all.max = lost_all.max.max(lost_other.max);
    println!(
        "SUMMARY {frames} frames, {last_t:.0} s, {} processed, {} keyframes, {tf_failures} tf failures",
        st.scans_processed, st.keyframes
    );
    println!("  update ms       {all}");
    println!(
        "  loops {} rejected {} map redraws {} | fallbacks {} | icp used {}/{}",
        st.loop_closures,
        st.loop_rejections,
        st.map_redraws,
        st.match_fallbacks,
        st.icp_used,
        st.icp_used + st.icp_rejected
    );
    println!(
        "  kf shift / opt  mean of mean {:.4} m, largest {:.4} m | largest turn per opt {kf_turn_max} rad",
        kf_shift_mean.mean(),
        kf_shift_max.max
    );
    println!("  m2o jump loop   {m2o_loop} m | yaw {yaw_loop} rad");
    println!("  m2o jump other  {m2o_other} m | yaw {yaw_other} rad");
    println!("  map lost occ    all {lost_all} | after loop {lost_loop} | other {lost_other}");
    println!(
        "  final map       occ {occ} known {known} occ/known {:.4} | front-end match score {:.3}",
        occ as f64 / known.max(1) as f64,
        score.mean()
    );
    if let Some(p) = map_out {
        write_pgm(&p, &g);
    }
}

