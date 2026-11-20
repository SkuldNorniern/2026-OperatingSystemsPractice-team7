// markers are not all printed at the configured size (and not all at the same size).
// pnp range = true range / factor, factor = real side / configured side
use tb4_core::grid::{Grid, GridMeta};
use tb4_markers::{LocateParams, Observation, Target, TrackParams, Tracker};

/// wall at x = 3 (surface estimate 3.025, cell centre), free in front
fn wall_map() -> Grid {
    let meta = GridMeta { width: 140, height: 140, resolution: 0.05, origin_x: -3.0, origin_y: -3.0 };
    let mut g = Grid::filled(meta, 0);
    for cy in 0..140 {
        g.set(g.world_to_cell(3.01, -3.0 + cy as f64 * 0.05 + 0.01).unwrap(), 100);
    }
    g
}

/// same room with a box in front of the wall: face 15 cm wide at x = 1.8, y 0.40..0.55
fn box_map() -> Grid {
    let mut g = wall_map();
    for cy in 0..3 {
        g.set(g.world_to_cell(1.81, 0.41 + cy as f64 * 0.05).unwrap(), 100);
        g.set(g.world_to_cell(1.86, 0.41 + cy as f64 * 0.05).unwrap(), 100);
    }
    g
}

#[derive(Clone, Copy)]
struct Marker {
    id: i64,
    x: f64,
    y: f64,
    factor: f64,
}

const ON_BOX: Marker = Marker { id: 9, x: 1.825, y: 0.475, factor: 2.0 };

fn view(m: Marker, cam: (f64, f64)) -> Observation {
    let (dx, dy) = (m.x - cam.0, m.y - cam.1);
    let r = dx.hypot(dy);
    Observation { id: m.id, camera: (cam.0, cam.1, 0.25), bearing: (dx / r, dy / r, 0.0), pnp_range: r / m.factor }
}

/// robot drives straight at the marker, from 3 m to 0.7 m in front of it
fn drive_at(t: &mut Tracker, g: &Grid, m: Marker) {
    let mut d = 3.0;
    while d >= 0.7 {
        t.observe(Some(g), &view(m, (m.x - d, m.y)), &LocateParams::default());
        d -= 0.06;
    }
}

/// robot passes the marker sideways at `dist` in front of it, 1 m either side
fn drive_past(t: &mut Tracker, g: &Grid, m: Marker, dist: f64) {
    for k in 0..21 {
        t.observe(Some(g), &view(m, (m.x - dist, m.y - 1.0 + 0.1 * k as f64)), &LocateParams::default());
    }
}

fn error(t: &Target, m: Marker) -> f64 {
    (t.x - m.x).hypot(t.y - m.y)
}

#[test]
fn wall_markers_of_different_sizes_land_on_the_wall() {
    let g = wall_map();
    let mut t = Tracker::new(TrackParams::default());
    let markers = [(1, -1.5, 0.5), (2, 0.0, 1.0), (3, 1.5, 2.0)].map(|(id, y, factor)| Marker { id, x: 3.025, y, factor });
    for m in markers {
        drive_past(&mut t, &g, m, 2.0);
    }
    for m in markers {
        let tg = t.target(m.id).unwrap();
        assert!(tg.confirmed && error(&tg, m) < 0.02, "{}x: {tg:?}", m.factor);
    }
}

#[test]
fn oversized_marker_seen_head_on_is_not_confirmed_short_of_the_box() {
    // 2x marker on the box face: pnp says half the range, so the edge hit is not trusted and
    // every view falls back to pnp range. all of them are short by the same factor
    let g = box_map();
    let mut t = Tracker::new(TrackParams::default());
    drive_at(&mut t, &g, ON_BOX);
    let tg = t.target(ON_BOX.id).unwrap();
    assert!(!tg.confirmed || error(&tg, ON_BOX) < 0.1, "confirmed {:.2} m off: {tg:?}", error(&tg, ON_BOX));
}

#[test]
fn head_on_views_keep_the_pnp_guess_while_unconfirmed() {
    // marker at the configured size on a wall that is not in the map yet. bearings jitter a bit,
    // that is not parallax: best guess for targets_tentative.csv stays at pnp range
    let m = Marker { id: 4, x: 3.025, y: 0.0, factor: 1.0 };
    let mut t = Tracker::new(TrackParams::default());
    for k in 0..39 {
        let d = 3.0 - 0.06 * k as f64;
        let jitter = 0.008 * ((k * 37 % 11) as f64 / 5.0 - 1.0);
        let o = Observation { bearing: (jitter.cos(), jitter.sin(), 0.0), ..view(m, (m.x - d, m.y)) };
        t.observe(None, &o, &LocateParams::default());
    }
    let tg = t.target(m.id).unwrap();
    assert!(!tg.confirmed && error(&tg, m) < 0.15, "{:.2} m off: {tg:?}", error(&tg, m));
}

#[test]
fn oversized_marker_is_found_once_the_camera_moves_sideways() {
    // bearings from different places triangulate it, whatever the pnp ranges say
    let g = box_map();
    for factor in [2.0, 2.8] {
        let m = Marker { factor, ..ON_BOX };
        let mut t = Tracker::new(TrackParams::default());
        drive_at(&mut t, &g, m);
        drive_past(&mut t, &g, m, 1.8);
        let tg = t.target(m.id).unwrap();
        assert!(tg.confirmed && error(&tg, m) < 0.05, "{factor}x {:.2} m off: {tg:?}", error(&tg, m));
    }
}
