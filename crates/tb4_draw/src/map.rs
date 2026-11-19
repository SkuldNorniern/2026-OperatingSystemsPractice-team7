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
}
