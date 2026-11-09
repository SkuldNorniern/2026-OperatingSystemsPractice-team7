// rerun tracker on a recorded run: map + observations.csv written by marker_mapper
// cargo run --release -p tb4_markers --example replay -- <run dir> [map name] [origin shift]
use std::collections::BTreeMap;

use tb4_core::{Grid, GridMeta};
use tb4_markers::{LocateParams, Observation, Source, TrackParams, Tracker, fuse};

fn load_map(dir: &str, name: &str, shift: f64) -> Grid {
    let yaml = std::fs::read_to_string(format!("{dir}/{name}.yaml")).expect("map yaml");
    let field = |k: &str| yaml.lines().find_map(|l| l.trim().strip_prefix(k)).map(|v| v.trim().to_owned()).unwrap();
    let res: f64 = field("resolution:").parse().unwrap();
    let o: Vec<f64> = field("origin:").trim_matches(|c| c == '[' || c == ']').split(',').map(|v| v.trim().parse().unwrap()).collect();
    let pgm = std::fs::read(format!("{dir}/{name}.pgm")).expect("map pgm");
    let mut fields = Vec::new();
    let mut i = 0;
    while fields.len() < 4 {
        while pgm[i].is_ascii_whitespace() {
            i += 1;
        }
        if pgm[i] == b'#' {
            while pgm[i] != b'\n' {
                i += 1;
            }
            continue;
        }
        let s = i;
        while !pgm[i].is_ascii_whitespace() {
            i += 1;
        }
        fields.push(String::from_utf8_lossy(&pgm[s..i]).to_string());
    }
    let (w, h): (usize, usize) = (fields[1].parse().unwrap(), fields[2].parse().unwrap());
    let px = &pgm[i + 1..];
    // pgm rows go top down, grid rows bottom up. map_saver trinary: 0 occupied, 254 free, else unknown
    let mut data = vec![-1i8; w * h];
    for y in 0..h {
        for x in 0..w {
            data[(h - 1 - y) * w + x] = match px[y * w + x] {
                0..=50 => 100,
                250..=255 => 0,
                _ => -1,
            };
        }
    }
    Grid::new(GridMeta { width: w as u32, height: h as u32, resolution: res, origin_x: o[0] + shift, origin_y: o[1] + shift }, data)
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let dir = args.first().expect("run dir");
    let name = args.get(1).map_or("map", |s| s.as_str());
    let shift: f64 = args.get(2).map_or(0.0, |s| s.parse().unwrap());
    let grid = load_map(dir, name, shift);
    let csv = std::fs::read_to_string(format!("{dir}/observations.csv")).expect("observations.csv");
    let obs: Vec<Observation> = csv
        .lines()
        .skip(1)
        .filter_map(|l| {
            let v: Vec<f64> = l.split(',').map(|x| x.parse().ok()).collect::<Option<_>>()?;
            Some(Observation { id: v[1] as i64, camera: (v[2], v[3], v[4]), bearing: (v[5], v[6], v[7]), pnp_range: v[8] })
        })
        .collect();
    let lp = LocateParams::default();
    // DEBUG=n: raycast details of first n observations per id
    let dbg: usize = std::env::var("DEBUG").ok().and_then(|v| v.parse().ok()).unwrap_or(0);
    let mut shown: BTreeMap<i64, usize> = BTreeMap::new();
    for o in &obs {
        let n = shown.entry(o.id).or_default();
        if *n >= dbg {
            continue;
        }
        *n += 1;
        let (bx, by, _) = o.bearing;
        let h = bx.hypot(by);
        let (dx, dy) = (bx / h, by / h);
        let ang = dy.atan2(dx);
        let hit = grid.raycast(o.camera.0, o.camera.1, dx, dy, lp.max_range, lp.occupied_threshold);
        let flat = hit.as_ref().map(|h| tb4_markers::surface_around(&grid, (o.camera.0, o.camera.1), ang, h.range, &lp));
        let wall = hit.as_ref().and_then(|h| tb4_markers::wall_hit(&grid, (o.camera.0, o.camera.1), (dx, dy), h.range, &lp));
        println!("dbg id {} ang {:.1} pnp {:.2} hit {:?} flat {:?} wall {:?}", o.id, ang.to_degrees(), o.pnp_range * h,
            hit.map(|h| (h.range * 1000.0).round() / 1000.0), flat, wall);
    }
    let mut tr = Tracker::new(TrackParams::default());
    let mut first_confirm: BTreeMap<i64, usize> = BTreeMap::new();
    let mut per_id: BTreeMap<i64, usize> = BTreeMap::new();
    for (k, o) in obs.iter().enumerate() {
        *per_id.entry(o.id).or_default() += 1;
        tr.observe(Some(&grid), o, &lp);
        if tr.target(o.id).is_some_and(|t| t.confirmed) {
            first_confirm.entry(o.id).or_insert(k);
        }
    }
    println!("{} observations, map {}x{} res {}", obs.len(), grid.meta.width, grid.meta.height, grid.meta.resolution);
    for t in tr.targets() {
        let [r, n, k] = tr.sources(t.id);
        let est = tr.estimates(t.id);
        let f = fuse(est);
        println!("id {:3}: ({:.3}, {:.3}) {} views of {} obs (ray {r} pnp {n} scaled {k}) scale {:?} sigma {:.3} consistency {:.2} confirmed {} first at obs {:?}",
            t.id, t.x, t.y, t.views, per_id[&t.id], tr.scale(t.id).map(|s| (s * 1000.0).round() / 1000.0), f.sigma, f.consistency, t.confirmed, first_confirm.get(&t.id));
        if std::env::var("VIEWS").is_ok() {
            for e in est {
                println!("         view {:?} ({:.3}, {:.3}) dir ({:.3}, {:.3}) along {:.3} across {:.3}", e.source, e.x, e.y, e.dir.0, e.dir.1, e.sigma_along, e.sigma_across);
            }
        }
        let ray: Vec<_> = est.iter().filter(|e| e.source == Source::Raycast).collect();
        if !ray.is_empty() {
            let sx = ray.iter().map(|e| e.x).sum::<f64>() / ray.len() as f64;
            let sy = ray.iter().map(|e| e.y).sum::<f64>() / ray.len() as f64;
            let sd = (ray.iter().map(|e| (e.x - sx).powi(2) + (e.y - sy).powi(2)).sum::<f64>() / ray.len() as f64).sqrt();
            println!("         raycast mean ({sx:.3}, {sy:.3}) rms spread {sd:.3}");
        }
    }
}
