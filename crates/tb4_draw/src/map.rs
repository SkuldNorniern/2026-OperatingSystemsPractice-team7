//! Occupancy grid -> RGBA image (row 0 = north edge) and the affine that puts it on screen.

use aurea_render::{DrawingContext, Image, Rect, Sampling, Transform};
use tb4_core::GridMeta;

use crate::Rgba;
use crate::palette::{FREE, OCCUPIED, UNKNOWN};
use crate::view::Viewport;

/// the map image size and the affine from image pixels to the screen:
/// sx = a*u + c*v + e, sy = b*u + d*v + f
#[derive(Debug, Clone, PartialEq)]
pub struct MapLayer {
    pub width: u32,
    pub height: u32,
    pub affine: [f32; 6],
}

/// -1 unknown, >= 65 occupied, 0..65 blends from free halfway to occupied
pub fn cell_color(v: i8) -> Rgba {
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

/// RGBA8 pixels of the grid (cells row-major from the origin, as in OccupancyGrid), north row first
pub fn map_rgba(meta: &GridMeta, data: &[i8]) -> Vec<u8> {
    let (w, h) = (meta.width as usize, meta.height as usize);
    let mut out = Vec::with_capacity(w * h * 4);
    for v in 0..h {
        let row = &data[(h - 1 - v) * w..(h - v) * w];
        for &c in row {
            out.extend_from_slice(&cell_color(c));
        }
    }
    out
}

/// the grid as an opaque aurea image; None when `data` does not match the meta
pub fn map_image(meta: &GridMeta, data: &[i8]) -> Option<Image> {
    let n = meta.width as usize * meta.height as usize;
    (n > 0 && data.len() == n).then(|| Image::new(meta.width, meta.height, map_rgba(meta, data)).with_opaque(true))
}

/// costmap cell colour, straight alpha scaled by `opacity` (0..1): unknown and free cells are clear,
/// 1..98 run blue -> magenta -> red and get more opaque with cost, 99 (inscribed) is cyan, 100
/// (lethal) purple
pub fn cost_color(v: i8, opacity: f32) -> Rgba {
    use crate::palette::{COST_HIGH, COST_INSCRIBED, COST_LETHAL, COST_LOW, COST_MID};
    let a = |alpha: f32| (alpha * opacity.clamp(0.0, 1.0)).round() as u8;
    let lerp = |p: Rgba, q: Rgba, t: f32| {
        let m = |i: usize| (f32::from(p[i]) + (f32::from(q[i]) - f32::from(p[i])) * t).round() as u8;
        [m(0), m(1), m(2)]
    };
    match v {
        i8::MIN..=0 => [0, 0, 0, 0],
        100.. => [COST_LETHAL[0], COST_LETHAL[1], COST_LETHAL[2], a(235.0)],
        99 => [COST_INSCRIBED[0], COST_INSCRIBED[1], COST_INSCRIBED[2], a(210.0)],
        _ => {
            let t = f32::from(v - 1) / 97.0;
            let [r, g, b] = if t < 0.5 { lerp(COST_LOW, COST_MID, t * 2.0) } else { lerp(COST_MID, COST_HIGH, t * 2.0 - 1.0) };
            [r, g, b, a(70.0 + 140.0 * t)]
        }
    }
}

/// a costmap as a see-through image (north row first), `opacity` 0..1
pub fn costmap_image(meta: &GridMeta, data: &[i8], opacity: f32) -> Option<Image> {
    let (w, h) = (meta.width as usize, meta.height as usize);
    if w == 0 || h == 0 || data.len() != w * h {
        return None;
    }
    let mut out = Vec::with_capacity(w * h * 4);
    for v in 0..h {
        for &c in &data[(h - 1 - v) * w..(h - v) * w] {
            out.extend_from_slice(&cost_color(c, opacity));
        }
    }
    Some(Image::new(meta.width, meta.height, out).with_opaque(false))
}

/// only the occupied cells of a map (>= 65), opaque, the rest clear: drawn over the costmaps so
/// walls stay readable
pub fn occupied_image(meta: &GridMeta, data: &[i8]) -> Option<Image> {
    let (w, h) = (meta.width as usize, meta.height as usize);
    if w == 0 || h == 0 || data.len() != w * h {
        return None;
    }
    let mut out = Vec::with_capacity(w * h * 4);
    for v in 0..h {
        for &c in &data[(h - 1 - v) * w..(h - v) * w] {
            out.extend_from_slice(&if c >= 65 { OCCUPIED } else { [0, 0, 0, 0] });
        }
    }
    Some(Image::new(meta.width, meta.height, out).with_opaque(false))
}

/// where the grid lands on screen; `frame` = (x, y, yaw) of the grid's frame in the drawn frame
/// (identity when the map frame is the drawn frame)
pub fn map_layer(meta: &GridMeta, frame: (f64, f64, f64), vp: &Viewport) -> MapLayer {
    let (s, c) = frame.2.sin_cos();
    let screen = |u: f64, v: f64| {
        let (px, py) = (meta.origin_x + u * meta.resolution, meta.origin_y + (f64::from(meta.height) - v) * meta.resolution);
        vp.to_screen(frame.0 + c * px - s * py, frame.1 + s * px + c * py)
    };
    let o = screen(0.0, 0.0);
    let pu = screen(1.0, 0.0);
    let pv = screen(0.0, 1.0);
    MapLayer { width: meta.width, height: meta.height, affine: [pu.0 - o.0, pu.1 - o.1, pv.0 - o.0, pv.1 - o.1, o.0, o.1] }
}

/// paint the map image through its layer affine, one image pixel per cell, no smoothing
pub fn draw_map(ctx: &mut dyn DrawingContext, image: &Image, layer: &MapLayer) -> aurea_foundation::AureaResult<()> {
    let [a, b, c, d, e, f] = layer.affine;
    ctx.save()?;
    ctx.transform(Transform { m11: a, m12: b, m13: 0.0, m21: c, m22: d, m23: 0.0, m31: e, m32: f, m33: 1.0 })?;
    ctx.set_sampling(Sampling::Nearest)?;
    ctx.draw_image_rect(image, Rect::new(0.0, 0.0, layer.width as f32, layer.height as f32))?;
    ctx.set_sampling(Sampling::Linear)?;
    ctx.restore()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn meta(width: u32, height: u32, resolution: f64, origin_x: f64, origin_y: f64) -> GridMeta {
        GridMeta { width, height, resolution, origin_x, origin_y }
    }

    #[test]
    fn layer_puts_cells_where_the_world_is() {
        let m = meta(4, 2, 0.5, -1.0, -0.5);
        let vp = Viewport::fit([0.0, 0.0, 10.0, 10.0], (0.0, 0.0, 100.0, 100.0), 0.0);
        let [a, b, c, d, e, f] = map_layer(&m, (2.0, 3.0, 0.0), &vp).affine;
        // image top-left = map (-1, 0.5) = world (1, 3.5) = screen (10, 65)
        assert!((e - 10.0).abs() < 1e-4 && (f - 65.0).abs() < 1e-4, "{e} {f}");
        assert!((a - 5.0).abs() < 1e-4 && b.abs() < 1e-4 && c.abs() < 1e-4 && (d - 5.0).abs() < 1e-4);
    }

    #[test]
    fn layer_rotates_with_the_frame() {
        let m = meta(1, 1, 1.0, 0.0, 0.0);
        let vp = Viewport::centred((0.0, 0.0), 10.0, (0.0, 0.0, 100.0, 100.0));
        // a quarter turn: image u (east in the grid) points north on screen
        let [a, b, ..] = map_layer(&m, (0.0, 0.0, std::f64::consts::FRAC_PI_2), &vp).affine;
        assert!(a.abs() < 1e-4 && (b + 10.0).abs() < 1e-4, "{a} {b}");
    }

    #[test]
    fn rgba_flips_rows_and_rejects_bad_sizes() {
        let m = meta(1, 2, 1.0, 0.0, 0.0);
        let px = map_rgba(&m, &[0, 100]);
        assert_eq!(&px[0..4], &OCCUPIED);
        assert_eq!(&px[4..8], &FREE);
        assert_eq!(cell_color(-1), UNKNOWN);
        assert!(map_image(&m, &[0]).is_none());
        assert!(map_image(&m, &[0, 0]).is_some());
    }

    #[test]
    fn cost_ramp_is_clear_for_free_and_rises_with_cost() {
        assert_eq!(cost_color(0, 1.0)[3], 0);
        assert_eq!(cost_color(-1, 1.0)[3], 0);
        let (lo, hi) = (cost_color(5, 1.0), cost_color(95, 1.0));
        assert!(lo[3] < hi[3] && lo[2] > lo[0] && hi[0] > hi[2], "{lo:?} {hi:?}");
        assert_eq!(cost_color(100, 0.5)[3], 118);
        assert_eq!(cost_color(100, 0.0)[3], 0);
    }

    fn rgb(p: u32) -> [u8; 3] {
        [(p >> 16) as u8, (p >> 8) as u8, p as u8]
    }

    #[test]
    fn walls_stay_on_top_of_a_full_costmap_in_a_turned_view() {
        use aurea_render::{CpuRasterizer, Renderer};
        // 2 x 2 m, free, one occupied column at x in [1.0, 1.1); a costmap of lethal-ish cost everywhere
        let m = meta(20, 20, 0.1, 0.0, 0.0);
        let mut data = vec![0i8; 400];
        for r in 0..20 {
            data[r * 20 + 10] = 100;
        }
        let cost = vec![90i8; 400];
        let (w, h) = (200u32, 200u32);
        let vp = Viewport::centred_rotated((1.0, 1.0), 80.0, (0.0, 0.0, 200.0, 200.0), 0.5);
        let layer = map_layer(&m, (0.0, 0.0, 0.0), &vp);
        let mut r = CpuRasterizer::new(w, h);
        {
            let mut ctx = r.begin_frame().unwrap();
            draw_map(ctx.as_mut(), &map_image(&m, &data).unwrap(), &layer).unwrap();
            draw_map(ctx.as_mut(), &costmap_image(&m, &cost, 1.0).unwrap(), &layer).unwrap();
            draw_map(ctx.as_mut(), &occupied_image(&m, &data).unwrap(), &layer).unwrap();
        }
        r.end_frame().unwrap();
        let px = r.pixels();
        let at = |x: f64, y: f64| {
            let (sx, sy) = vp.to_screen(x, y);
            rgb(px[sy as usize * w as usize + sx as usize])
        };
        // the wall is drawn black over the costmap, on its rotated place
        assert!(at(1.05, 1.3).iter().all(|&c| c < 30), "wall {:?}", at(1.05, 1.3));
        // free cells are tinted by the costmap: no longer the free grey
        let free = at(0.5, 0.5);
        assert!(free.iter().zip(FREE).any(|(&a, b)| (i32::from(a) - i32::from(b)).abs() > 20), "tint {free:?}");
    }
}
