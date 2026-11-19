//! Pure part of the viewer: world -> screen transform and the list of things to draw for one frame.
//! Nothing here touches a window, so it is unit tested.

use tb4_core::{Grid, Pose2};
use tb4_explore::{Frontier, View as CameraView};
use tb4_sim::SimState;
use tb4_sim::metrics::MarkerRecord;
use tb4_sim::sim::compose;
use tb4_sim::world::SegmentKind;

pub type Rgba = [u8; 4];

pub mod colors {
    use super::Rgba;
    pub const BACKGROUND: Rgba = [28, 28, 32, 255];
    pub const UNKNOWN: Rgba = [64, 66, 74, 255];
    pub const FREE: Rgba = [206, 206, 200, 255];
    pub const OCCUPIED: Rgba = [20, 20, 20, 255];
    pub const WALL: Rgba = [170, 110, 255, 255];
    pub const GLASS: Rgba = [0, 220, 255, 255];
    pub const ROBOT: Rgba = [255, 255, 255, 200];
    pub const ROBOT_EDGE: Rgba = [20, 20, 20, 255];
    pub const ESTIMATE: Rgba = [255, 120, 200, 255];
    pub const TRAIL: Rgba = [40, 170, 160, 255];
    pub const PATH: Rgba = [40, 200, 60, 255];
    pub const TRAJECTORY: Rgba = [255, 0, 200, 255];
    pub const FRONTIER: Rgba = [50, 110, 255, 255];
    pub const VIEW: Rgba = [255, 150, 0, 255];
    pub const GOAL: Rgba = [255, 230, 0, 255];
    pub const BUMP: Rgba = [255, 40, 40, 255];
    pub const MARKER_UNSEEN: Rgba = [150, 150, 150, 255];
    pub const MARKER_NEAR: Rgba = [255, 200, 0, 255];
    pub const MARKER_READABLE: Rgba = [0, 210, 90, 255];
    pub const TEXT: Rgba = [235, 235, 235, 255];
}

/// metres -> pixels, north up: world `bounds` fitted and centred in a screen rectangle
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Viewport {
    pub x0: f64,
    pub y1: f64,
    /// pixels per metre
    pub scale: f64,
    pub left: f64,
    pub top: f64,
}

impl Viewport {
    /// fit `bounds` [x0, y0, x1, y1] into the screen rect (x, y, w, h) with `margin` pixels around
    pub fn fit(bounds: [f64; 4], rect: (f64, f64, f64, f64), margin: f64) -> Self {
        let [x0, y0, x1, y1] = bounds;
        let (bw, bh) = ((x1 - x0).max(1e-6), (y1 - y0).max(1e-6));
        let (aw, ah) = ((rect.2 - 2.0 * margin).max(1.0), (rect.3 - 2.0 * margin).max(1.0));
        let scale = (aw / bw).min(ah / bh);
        Self { x0, y1, scale, left: rect.0 + (rect.2 - bw * scale) / 2.0, top: rect.1 + (rect.3 - bh * scale) / 2.0 }
    }

    pub fn to_screen(&self, x: f64, y: f64) -> (f32, f32) {
        ((self.left + (x - self.x0) * self.scale) as f32, (self.top + (self.y1 - y) * self.scale) as f32)
    }

    pub fn len(&self, metres: f64) -> f32 {
        (metres * self.scale) as f32
    }
}

#[derive(Debug, Clone, PartialEq)]
pub enum Prim {
    Line { a: (f32, f32), b: (f32, f32), color: Rgba, width: f32, dashed: bool },
    Polyline { pts: Vec<(f32, f32)>, color: Rgba, width: f32 },
    Circle { c: (f32, f32), r: f32, fill: Option<Rgba>, stroke: Option<(Rgba, f32)> },
    Text { at: (f32, f32), text: String, color: Rgba, size: f32 },
}

/// the SLAM map as an RGBA image (row 0 = north edge) and the affine map from image pixels to the
/// screen: sx = a*u + c*v + e, sy = b*u + d*v + f
#[derive(Debug, Clone, PartialEq)]
pub struct MapLayer {
    pub width: u32,
    pub height: u32,
    pub affine: [f32; 6],
}

pub fn cell_color(v: i8) -> Rgba {
    use colors::*;
    if v < 0 {
        UNKNOWN
    } else if v >= 65 {
        OCCUPIED
    } else {
        let t = f32::from(v) / 65.0;
        let mix = |a: u8, b: u8| (f32::from(a) + (f32::from(b) - f32::from(a)) * t * 0.5).round() as u8;
        [mix(FREE[0], OCCUPIED[0]), mix(FREE[1], OCCUPIED[1]), mix(FREE[2], OCCUPIED[2]), 255]
    }
}

/// RGBA8 pixels of the grid, north row first
pub fn map_rgba(g: &Grid) -> Vec<u8> {
    let (w, h) = (g.meta.width as usize, g.meta.height as usize);
    let mut out = Vec::with_capacity(w * h * 4);
    for v in 0..h {
        let row = &g.data[(h - 1 - v) * w..(h - v) * w];
        for &c in row {
            out.extend_from_slice(&cell_color(c));
        }
    }
    out
}

pub fn map_layer(g: &Grid, map_origin: Pose2, vp: &Viewport) -> MapLayer {
    let m = &g.meta;
    let screen = |u: f64, v: f64| {
        let p = Pose2::new(m.origin_x + u * m.resolution, m.origin_y + (f64::from(m.height) - v) * m.resolution, 0.0);
        let w = compose(map_origin, p);
        vp.to_screen(w.x, w.y)
    };
    let o = screen(0.0, 0.0);
    let pu = screen(1.0, 0.0);
    let pv = screen(0.0, 1.0);
    MapLayer { width: m.width, height: m.height, affine: [pu.0 - o.0, pu.1 - o.1, pv.0 - o.0, pv.1 - o.1, o.0, o.1] }
}

/// everything drawn over the map for one frame
pub struct SceneInput<'a> {
    pub state: &'a SimState<'a>,
    pub robot_radius: f64,
    /// MPPI optimal trajectory, map frame
    pub trajectory: &'a [(f64, f64)],
    pub frontiers: &'a [Frontier],
    pub views: &'a [CameraView],
    /// true robot positions, world frame
    pub trail: &'a [(f64, f64)],
}

pub fn marker_color(r: &MarkerRecord) -> Rgba {
    if r.first_readable.is_some() {
        colors::MARKER_READABLE
    } else if r.first_within_2m.is_some() {
        colors::MARKER_NEAR
    } else {
        colors::MARKER_UNSEEN
    }
}

pub fn build(input: &SceneInput, vp: &Viewport) -> Vec<Prim> {
    let s = input.state;
    let o = s.map_origin;
    let world_of = |(x, y): (f64, f64)| {
        let w = compose(o, Pose2::new(x, y, 0.0));
        vp.to_screen(w.x, w.y)
    };
    let px = |m: f64| vp.len(m).max(1.0);
    let mut out = Vec::new();

    // true world on top of the map, thin
    for seg in &s.world.segments {
        let glass = seg.kind == SegmentKind::Glass;
        out.push(Prim::Line {
            a: vp.to_screen(seg.a.0, seg.a.1),
            b: vp.to_screen(seg.b.0, seg.b.1),
            color: if glass { colors::GLASS } else { colors::WALL },
            width: if glass { 2.5 } else { 1.0 },
            dashed: glass,
        });
    }
    for p in &s.world.posts {
        out.push(Prim::Circle { c: vp.to_screen(p.c.0, p.c.1), r: px(p.r), fill: None, stroke: Some((colors::WALL, 1.0)) });
    }

    if input.trail.len() >= 2 {
        let pts = input.trail.iter().map(|&(x, y)| vp.to_screen(x, y)).collect();
        out.push(Prim::Polyline { pts, color: colors::TRAIL, width: 1.5 });
    }
    if s.path.len() >= 2 {
        out.push(Prim::Polyline { pts: s.path.iter().map(|&p| world_of(p)).collect(), color: colors::PATH, width: 2.0 });
    }
    if input.trajectory.len() >= 2 {
        let pts = input.trajectory.iter().map(|&p| world_of(p)).collect();
        out.push(Prim::Polyline { pts, color: colors::TRAJECTORY, width: 2.0 });
    }

    for f in input.frontiers {
        out.push(Prim::Circle { c: world_of(f.goal), r: 4.0, fill: Some(colors::FRONTIER), stroke: None });
    }
    for v in input.views {
        let c = world_of((v.x, v.y));
        out.push(Prim::Circle { c, r: 4.0, fill: Some(colors::VIEW), stroke: None });
        let yaw = o.yaw + v.yaw;
        let tip = (c.0 + 10.0 * yaw.cos() as f32, c.1 - 10.0 * yaw.sin() as f32);
        out.push(Prim::Line { a: c, b: tip, color: colors::VIEW, width: 1.5, dashed: false });
    }
    if let Some(g) = s.goal {
        let c = world_of(g);
        out.push(Prim::Circle { c, r: 7.0, fill: None, stroke: Some((colors::GOAL, 2.0)) });
        out.push(Prim::Line { a: (c.0 - 10.0, c.1), b: (c.0 + 10.0, c.1), color: colors::GOAL, width: 1.5, dashed: false });
        out.push(Prim::Line { a: (c.0, c.1 - 10.0), b: (c.0, c.1 + 10.0), color: colors::GOAL, width: 1.5, dashed: false });
    }

    for (mk, rec) in s.world.markers.iter().zip(s.markers) {
        let color = marker_color(rec);
        let half = mk.size.max(0.1) / 2.0 + 0.03;
        let (tx, ty) = (-mk.normal.1, mk.normal.0);
        let a = vp.to_screen(mk.pos.0 - tx * half, mk.pos.1 - ty * half);
        let b = vp.to_screen(mk.pos.0 + tx * half, mk.pos.1 + ty * half);
        out.push(Prim::Line { a, b, color, width: 5.0, dashed: false });
        let l = vp.to_screen(mk.pos.0 + mk.normal.0 * 0.3, mk.pos.1 + mk.normal.1 * 0.3);
        out.push(Prim::Text { at: (l.0 - 6.0, l.1 + 5.0), text: mk.id.to_string(), color, size: 14.0 });
    }

    for &(x, y) in s.bumps {
        let (cx, cy) = vp.to_screen(x, y);
        let d = 5.0;
        out.push(Prim::Line { a: (cx - d, cy - d), b: (cx + d, cy + d), color: colors::BUMP, width: 2.5, dashed: false });
        out.push(Prim::Line { a: (cx - d, cy + d), b: (cx + d, cy - d), color: colors::BUMP, width: 2.5, dashed: false });
    }

    // estimate (where SLAM thinks the robot is), then the true robot
    let est = compose(o, s.estimate);
    out.push(Prim::Circle {
        c: vp.to_screen(est.x, est.y),
        r: px(input.robot_radius),
        fill: None,
        stroke: Some((colors::ESTIMATE, 1.5)),
    });
    let t = s.truth;
    let c = vp.to_screen(t.x, t.y);
    let r = px(input.robot_radius);
    out.push(Prim::Circle { c, r, fill: Some(colors::ROBOT), stroke: Some((colors::ROBOT_EDGE, 1.5)) });
    let tip = (c.0 + r * t.yaw.cos() as f32, c.1 - r * t.yaw.sin() as f32);
    out.push(Prim::Line { a: c, b: tip, color: colors::ROBOT_EDGE, width: 2.0, dashed: false });
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use tb4_core::GridMeta;

    #[test]
    fn fit_keeps_aspect_north_up_and_centres() {
        // 10 x 5 m into 1000 x 1000 px: 100 px/m, centred vertically
        let vp = Viewport::fit([0.0, 0.0, 10.0, 5.0], (0.0, 0.0, 1000.0, 1000.0), 0.0);
        assert!((vp.scale - 100.0).abs() < 1e-9);
        assert_eq!(vp.to_screen(0.0, 5.0), (0.0, 250.0));
        assert_eq!(vp.to_screen(10.0, 0.0), (1000.0, 750.0));
        // north is up: larger y, smaller screen y
        assert!(vp.to_screen(1.0, 4.0).1 < vp.to_screen(1.0, 1.0).1);
        assert_eq!(vp.len(0.171), 17.1);
    }

    #[test]
    fn fit_with_offset_rect_and_margin() {
        let vp = Viewport::fit([-1.0, -1.0, 1.0, 1.0], (100.0, 50.0, 400.0, 200.0), 10.0);
        // limited by height: (200 - 20) / 2 m = 90 px/m
        assert!((vp.scale - 90.0).abs() < 1e-9);
        assert_eq!(vp.to_screen(0.0, 0.0), (300.0, 150.0));
    }

    #[test]
    fn map_layer_puts_cells_where_the_world_is() {
        let g = Grid::filled(GridMeta { width: 4, height: 2, resolution: 0.5, origin_x: -1.0, origin_y: -0.5 }, -1);
        let vp = Viewport::fit([0.0, 0.0, 10.0, 10.0], (0.0, 0.0, 100.0, 100.0), 0.0);
        let layer = map_layer(&g, Pose2::new(2.0, 3.0, 0.0), &vp);
        let [a, b, c, d, e, f] = layer.affine;
        // image top-left = map (-1, 0.5) = world (1, 3.5) = screen (10, 65)
        assert!((e - 10.0).abs() < 1e-4 && (f - 65.0).abs() < 1e-4, "{e} {f}");
        // one pixel = 0.5 m = 5 px, u east, v south
        assert!((a - 5.0).abs() < 1e-4 && b.abs() < 1e-4 && c.abs() < 1e-4 && (d - 5.0).abs() < 1e-4);
    }

    #[test]
    fn map_rgba_flips_rows() {
        let mut g = Grid::filled(GridMeta { width: 1, height: 2, resolution: 1.0, origin_x: 0.0, origin_y: 0.0 }, 0);
        g.data[1] = 100; // north cell occupied
        let px = map_rgba(&g);
        assert_eq!(&px[0..4], &colors::OCCUPIED);
        assert_eq!(&px[4..8], &colors::FREE);
        assert_eq!(cell_color(-1), colors::UNKNOWN);
    }
}
