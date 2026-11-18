// Synthetic marker rendering for tests and benchmarks.
use crate::dict::Dictionary;
use crate::geom::Homography;
use crate::image::GrayImage;

pub const DARK: u8 = 30;
pub const LIGHT: u8 = 220;

// Square marker corners (TL, TR, BR, BL as printed) rotated by `angle` radians about its centre.
pub fn square_corners(cx: f64, cy: f64, side: f64, angle: f64) -> [[f64; 2]; 4] {
    let (s, c) = angle.sin_cos();
    let h = side / 2.0;
    [[-h, -h], [h, -h], [h, h], [-h, h]].map(|[x, y]| [cx + c * x - s * y, cy + s * x + c * y])
}

// Renders markers (outer black border corners) on a LIGHT background with `ss`x`ss` supersampling.
pub fn render(
    width: usize,
    height: usize,
    dict: &Dictionary,
    markers: &[(u32, [[f64; 2]; 4])],
    ss: usize,
) -> GrayImage {
    let mut img = GrayImage::filled(width, height, LIGHT);
    let n = dict.bits + 2;
    let ss = ss.max(1);
    for (id, q) in markers {
        let Some(inv) = Homography::from_unit_square(q).and_then(|h| h.inverse()) else {
            continue;
        };
        let xs = q.iter().map(|p| p[0]);
        let ys = q.iter().map(|p| p[1]);
        let x0 = (xs.clone().fold(f64::MAX, f64::min).floor() - 1.0).max(0.0) as usize;
        let x1 = ((xs.fold(f64::MIN, f64::max).ceil() + 1.0).max(0.0) as usize).min(width);
        let y0 = (ys.clone().fold(f64::MAX, f64::min).floor() - 1.0).max(0.0) as usize;
        let y1 = ((ys.fold(f64::MIN, f64::max).ceil() + 1.0).max(0.0) as usize).min(height);
        for y in y0..y1 {
            for x in x0..x1 {
                let mut acc = 0u32;
                let mut hit = false;
                for sy in 0..ss {
                    for sx in 0..ss {
                        let px = x as f64 - 0.5 + (sx as f64 + 0.5) / ss as f64;
                        let py = y as f64 - 0.5 + (sy as f64 + 0.5) / ss as f64;
                        let [u, v] = inv.map(px, py);
                        let val = if (0.0..1.0).contains(&u) && (0.0..1.0).contains(&v) {
                            hit = true;
                            let c = (u * n as f64) as usize;
                            let r = (v * n as f64) as usize;
                            let inner = r >= 1 && c >= 1 && r <= n - 2 && c <= n - 2;
                            if inner && dict.is_white(*id, r - 1, c - 1) {
                                LIGHT
                            } else {
                                DARK
                            }
                        } else {
                            img.get(x, y)
                        };
                        acc += val as u32;
                    }
                }
                if hit {
                    img.data[y * width + x] =
                        ((acc + (ss * ss / 2) as u32) / (ss * ss) as u32) as u8;
                }
            }
        }
    }
    img
}

pub fn gaussian_blur(img: &GrayImage, sigma: f64) -> GrayImage {
    let r = (3.0 * sigma).ceil() as i64;
    let k: Vec<f64> = (-r..=r)
        .map(|i| (-(i * i) as f64 / (2.0 * sigma * sigma)).exp())
        .collect();
    let ks: f64 = k.iter().sum();
    let (w, h) = (img.width as i64, img.height as i64);
    let mut tmp = vec![0.0; img.data.len()];
    for y in 0..h {
        for x in 0..w {
            let mut a = 0.0;
            for (i, kv) in k.iter().enumerate() {
                let xx = (x + i as i64 - r).clamp(0, w - 1);
                a += kv * img.data[(y * w + xx) as usize] as f64;
            }
            tmp[(y * w + x) as usize] = a / ks;
        }
    }
    let mut out = img.clone();
    for y in 0..h {
        for x in 0..w {
            let mut a = 0.0;
            for (i, kv) in k.iter().enumerate() {
                let yy = (y + i as i64 - r).clamp(0, h - 1);
                a += kv * tmp[(yy * w + x) as usize];
            }
            out.data[(y * w + x) as usize] = (a / ks).round().clamp(0.0, 255.0) as u8;
        }
    }
    out
}

// Deterministic uniform noise in [-amp, amp].
pub fn add_noise(img: &mut GrayImage, amp: i32, seed: u64) {
    let mut s = seed
        .wrapping_mul(6364136223846793005)
        .wrapping_add(1442695040888963407);
    for v in img.data.iter_mut() {
        s = s
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        let r = ((s >> 33) % (2 * amp as u64 + 1)) as i32 - amp;
        *v = (*v as i32 + r).clamp(0, 255) as u8;
    }
}
