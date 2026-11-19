//! Pure part of the viewer: the list of things to draw for one frame. The transform, colours, map
//! image and primitives come from `tb4_draw`, shared with the ROS viewer. Nothing here touches a
//! window, so it is unit tested.

use tb4_core::{Grid, Pose2};
use tb4_draw::prim;
use tb4_explore::{Frontier, View as CameraView};
use tb4_sim::SimState;
use tb4_sim::metrics::MarkerRecord;
use tb4_sim::sim::compose;
use tb4_sim::world::SegmentKind;

pub use tb4_draw::palette as colors;
pub use tb4_draw::{MapLayer, Prim, Rgba, Viewport, cell_color};

/// RGBA8 pixels of the grid, north row first
pub fn map_rgba(g: &Grid) -> Vec<u8> {
    tb4_draw::map_rgba(&g.meta, &g.data)
}

/// where the SLAM map lands on screen; `map_origin` is the map frame in the world
pub fn map_layer(g: &Grid, map_origin: Pose2, vp: &Viewport) -> MapLayer {
    tb4_draw::map_layer(&g.meta, (map_origin.x, map_origin.y, map_origin.yaw), vp)
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
        out.push(prim::dot(world_of(f.goal), 4.0, colors::FRONTIER));
    }
    for v in input.views {
        out.extend(prim::view(world_of((v.x, v.y)), o.yaw + v.yaw, colors::VIEW));
    }
    if let Some(g) = s.goal {
        out.extend(prim::goal(world_of(g)));
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
        out.extend(prim::cross(vp.to_screen(x, y), 5.0, colors::BUMP, 2.5));
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
    out.extend(prim::robot(vp.to_screen(t.x, t.y), px(input.robot_radius), t.yaw));
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
