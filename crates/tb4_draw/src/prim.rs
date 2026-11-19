//! Screen-space drawing primitives, shape builders and the painter that turns them into aurea calls.

use aurea_foundation::AureaResult;
use aurea_render::{Color, DrawingContext, Font, Paint, PaintStyle, Path, PathCommand, Point, Rect};

use crate::Rgba;
use crate::palette;

pub type Pt = (f32, f32);

#[derive(Debug, Clone, PartialEq)]
pub enum Prim {
    Line { a: Pt, b: Pt, color: Rgba, width: f32, dashed: bool },
    Polyline { pts: Vec<Pt>, color: Rgba, width: f32 },
    Circle { c: Pt, r: f32, fill: Option<Rgba>, stroke: Option<(Rgba, f32)> },
    /// many small squares of side `size` centred on `pts` (laser points), cheaper than circles
    Points { pts: Vec<Pt>, size: f32, color: Rgba },
    /// `at` is the baseline start
    Text { at: Pt, text: String, color: Rgba, size: f32 },
}

pub fn color(c: Rgba) -> Color {
    Color::rgba(c[0], c[1], c[2], c[3])
}

pub fn stroke(c: Rgba, width: f32) -> Paint {
    Paint::new().color(color(c)).style(PaintStyle::Stroke).stroke_width(width)
}

pub fn fill(c: Rgba) -> Paint {
    Paint::new().color(color(c)).style(PaintStyle::Fill)
}

pub fn polyline(pts: &[Pt]) -> Path {
    let mut p = Path::new();
    for (i, &(x, y)) in pts.iter().enumerate() {
        let pt = Point::new(x, y);
        p.commands.push(if i == 0 { PathCommand::MoveTo(pt) } else { PathCommand::LineTo(pt) });
    }
    p
}

pub fn paint_prims(ctx: &mut dyn DrawingContext, prims: &[Prim]) -> AureaResult<()> {
    for p in prims {
        match p {
            Prim::Line { a, b, color, width, dashed } => {
                let mut paint = stroke(*color, *width);
                if *dashed {
                    paint = paint.dash(&[6.0, 4.0], 0.0);
                }
                ctx.draw_path(&polyline(&[*a, *b]), &paint)?;
            }
            Prim::Polyline { pts, color, width } => ctx.draw_path(&polyline(pts), &stroke(*color, *width))?,
            Prim::Circle { c, r, fill: f, stroke: s } => {
                let at = Point::new(c.0, c.1);
                if let Some(f) = f {
                    ctx.draw_circle(at, *r, &fill(*f))?;
                }
                if let Some((sc, w)) = s {
                    ctx.draw_circle(at, *r, &stroke(*sc, *w))?;
                }
            }
            Prim::Points { pts, size, color } => {
                let paint = fill(*color);
                let h = size / 2.0;
                for &(x, y) in pts {
                    ctx.draw_rect(Rect::new(x - h, y - h, *size, *size), &paint)?;
                }
            }
            Prim::Text { at, text, color: c, size } => {
                ctx.draw_text_with_font(text, Point::new(at.0, at.1), &Font::new("", *size), &fill(*c))?;
            }
        }
    }
    Ok(())
}

/// the robot: footprint disc of `r` pixels at `c`, heading line along `yaw` (world radians, CCW)
pub fn robot(c: Pt, r: f32, yaw: f64) -> [Prim; 2] {
    let tip = (c.0 + r * yaw.cos() as f32, c.1 - r * yaw.sin() as f32);
    [
        Prim::Circle { c, r, fill: Some(palette::ROBOT), stroke: Some((palette::ROBOT_EDGE, 1.5)) },
        Prim::Line { a: c, b: tip, color: palette::ROBOT_EDGE, width: 2.0, dashed: false },
    ]
}

/// the active goal: a ring with a cross
pub fn goal(c: Pt) -> [Prim; 3] {
    let g = palette::GOAL;
    [
        Prim::Circle { c, r: 7.0, fill: None, stroke: Some((g, 2.0)) },
        Prim::Line { a: (c.0 - 10.0, c.1), b: (c.0 + 10.0, c.1), color: g, width: 1.5, dashed: false },
        Prim::Line { a: (c.0, c.1 - 10.0), b: (c.0, c.1 + 10.0), color: g, width: 1.5, dashed: false },
    ]
}

/// an X of half-size `d`
pub fn cross(c: Pt, d: f32, color: Rgba, width: f32) -> [Prim; 2] {
    [
        Prim::Line { a: (c.0 - d, c.1 - d), b: (c.0 + d, c.1 + d), color, width, dashed: false },
        Prim::Line { a: (c.0 - d, c.1 + d), b: (c.0 + d, c.1 - d), color, width, dashed: false },
    ]
}

/// a filled dot
pub fn dot(c: Pt, r: f32, color: Rgba) -> Prim {
    Prim::Circle { c, r, fill: Some(color), stroke: None }
}

/// a dot with a 10 px tick along `yaw` (camera view poses)
pub fn view(c: Pt, yaw: f64, color: Rgba) -> [Prim; 2] {
    let tip = (c.0 + 10.0 * yaw.cos() as f32, c.1 - 10.0 * yaw.sin() as f32);
    [dot(c, 4.0, color), Prim::Line { a: c, b: tip, color, width: 1.5, dashed: false }]
}

/// a labelled target: dot with a dark rim and the label to its upper right
pub fn target(c: Pt, r: f32, label: &str, color: Rgba) -> [Prim; 2] {
    [
        Prim::Circle { c, r, fill: Some(color), stroke: Some((palette::ROBOT_EDGE, 1.5)) },
        Prim::Text { at: (c.0 + r + 3.0, c.1 - r - 2.0), text: label.to_owned(), color, size: 14.0 },
    ]
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::map::{draw_map, map_image, map_layer};
    use crate::view::Viewport;
    use aurea_render::{CpuRasterizer, Renderer};
    use tb4_core::GridMeta;

    fn rgb(p: u32) -> [u8; 3] {
        [(p >> 16) as u8, (p >> 8) as u8, p as u8]
    }

    fn near(p: u32, c: Rgba, tol: i32) -> bool {
        rgb(p).iter().zip(c).all(|(&a, b)| (i32::from(a) - i32::from(b)).abs() <= tol)
    }

    #[test]
    fn offscreen_scene_lands_on_the_expected_pixels() {
        // 4 x 2 m map at 0.1 m: west half free, east half unknown, one occupied column at x = 1.0 m
        let meta = GridMeta { width: 40, height: 20, resolution: 0.1, origin_x: -2.0, origin_y: -1.0 };
        let mut data = vec![-1i8; 800];
        for row in 0..20 {
            for col in 0..20 {
                data[row * 40 + col] = 0;
            }
            data[row * 40 + 30] = 100;
        }
        let (w, h) = (400u32, 200u32);
        let vp = Viewport::fit([-2.0, -1.0, 2.0, 1.0], (0.0, 0.0, f64::from(w), f64::from(h)), 0.0);
        assert!((vp.scale - 100.0).abs() < 1e-9);

        let mut prims = Vec::new();
        prims.push(Prim::Polyline { pts: vec![vp.to_screen(-1.8, -0.8), vp.to_screen(-0.2, -0.8)], color: palette::PATH, width: 3.0 });
        prims.push(dot(vp.to_screen(0.5, 0.5), 4.0, palette::FRONTIER));
        prims.extend(robot(vp.to_screen(-1.0, 0.3), vp.len(0.171), 0.0));
        prims.extend(target(vp.to_screen(1.5, -0.5), 6.0, "7", palette::TARGET_CONFIRMED));

        let mut r = CpuRasterizer::new(w, h);
        {
            let mut ctx = r.begin_frame().unwrap();
            ctx.clear(color(palette::BACKGROUND)).unwrap();
            let img = map_image(&meta, &data).unwrap();
            draw_map(ctx.as_mut(), &img, &map_layer(&meta, (0.0, 0.0, 0.0), &vp)).unwrap();
            paint_prims(ctx.as_mut(), &prims).unwrap();
        }
        r.end_frame().unwrap();
        let px = r.pixels().to_vec();
        let at = |x: f64, y: f64| {
            let (sx, sy) = vp.to_screen(x, y);
            px[sy as usize * w as usize + sx as usize]
        };
        assert!(near(at(-1.5, 0.0), palette::FREE, 2), "free {:?}", rgb(at(-1.5, 0.0)));
        assert!(near(at(0.5, -0.5), palette::UNKNOWN, 2), "unknown {:?}", rgb(at(0.5, -0.5)));
        assert!(near(at(1.05, 0.0), palette::OCCUPIED, 2), "occupied {:?}", rgb(at(1.05, 0.0)));
        assert!(near(at(-1.0, -0.8), palette::PATH, 10), "path {:?}", rgb(at(-1.0, -0.8)));
        assert!(near(at(0.5, 0.5), palette::FRONTIER, 10), "frontier {:?}", rgb(at(0.5, 0.5)));
        assert!(near(at(1.5, -0.5), palette::TARGET_CONFIRMED, 10), "target {:?}", rgb(at(1.5, -0.5)));
        // robot body, below its heading line (ROBOT is white at alpha 200 over the free map)
        assert!(rgb(at(-1.0, 0.22)).iter().all(|&v| v > 215), "robot {:?}", rgb(at(-1.0, 0.22)));
        // heading line drawn east of the centre
        assert!(rgb(at(-0.92, 0.3)).iter().all(|&v| v < 80), "heading {:?}", rgb(at(-0.92, 0.3)));
        // label pixels to the upper right of the target
        let (tx, ty) = vp.to_screen(1.5, -0.5);
        let label = (ty as u32 - 22..ty as u32 - 6)
            .flat_map(|y| (tx as u32 + 8..tx as u32 + 22).map(move |x| (x, y)))
            .filter(|&(x, y)| near(px[(y * w + x) as usize], palette::TARGET_CONFIRMED, 40))
            .count();
        assert!(label > 5, "label pixels {label}");
    }
}
