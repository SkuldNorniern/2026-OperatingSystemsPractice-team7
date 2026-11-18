use tb4_aruco::synth::{add_noise, render, square_corners};
use tb4_aruco::{Detector, DetectorParams, Dictionary, GrayImage};

// horizontal box blur of `len` px, like a camera panning while the robot turns
fn motion_blur(img: &GrayImage, len: usize) -> GrayImage {
    if len <= 1 {
        return img.clone();
    }
    let mut out = img.clone();
    let half = len as i64 / 2;
    for y in 0..img.height {
        for x in 0..img.width {
            let mut s = 0u32;
            let mut n = 0u32;
            for k in -half..=(len as i64 - 1 - half) {
                let xx = x as i64 + k;
                if xx >= 0 && (xx as usize) < img.width {
                    s += img.get(xx as usize, y) as u32;
                    n += 1;
                }
            }
            out.data[y * img.width + x] = (s / n) as u8;
        }
    }
    out
}

fn main() {
    let dict = Dictionary::aruco_original();
    let params = DetectorParams::default();
    let trials = 60;
    println!("side_px  blur_px  detected  wrong_id");
    for side in [16.0, 24.0, 36.0] {
        for blur in [0usize, 2, 4, 6, 8, 10, 14] {
            let mut det = Detector::new(params.clone());
            let (mut ok, mut wrong) = (0, 0);
            for t in 0..trials {
                let id = (t * 37 + 11) % 1024;
                let angle = (t as f64 * 0.37).sin() * 0.6;
                let c = square_corners(125.0 + (t % 7) as f64 * 3.0, 125.0 - (t % 5) as f64 * 4.0, side, angle);
                let mut img = render(250, 250, &dict, &[(id as u32, c)], 4);
                img = motion_blur(&img, blur);
                add_noise(&mut img, 6, t as u64);
                let found = det.detect(&img, &dict);
                if found.iter().any(|m| m.id == id as u32) {
                    ok += 1;
                }
                wrong += found.iter().filter(|m| m.id != id as u32).count();
            }
            println!("{side:7.0}  {blur:7}  {:7.0}%  {wrong:8}", 100.0 * ok as f64 / trials as f64);
        }
    }
}
