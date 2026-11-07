use std::time::Instant;

use tb4_aruco::synth::{add_noise, gaussian_blur, render, square_corners};
use tb4_aruco::{Detector, DetectorParams, Dictionary};

fn main() {
    let dict = Dictionary::aruco_original();
    let runs = 200;
    for size in [250usize, 480] {
        let s = size as f64 / 250.0;
        let markers = [
            (3u32, square_corners(60.0 * s, 70.0 * s, 40.0 * s, 0.3)),
            (42, square_corners(180.0 * s, 80.0 * s, 25.0 * s, -0.7)),
            (700, square_corners(120.0 * s, 190.0 * s, 15.0 * s, 1.9)),
        ];
        let img = render(size, size, &dict, &markers, 4);
        let mut img = gaussian_blur(&img, 0.8);
        add_noise(&mut img, 6, 1);
        let mut det = Detector::new(DetectorParams::default());
        let mut out = Vec::new();
        det.detect_into(&img, &dict, &mut out);
        let t = Instant::now();
        for _ in 0..runs {
            det.detect_into(&img, &dict, &mut out);
        }
        let ms = t.elapsed().as_secs_f64() * 1e3 / runs as f64;
        let ids: Vec<u32> = out.iter().map(|m| m.id).collect();
        println!("{size}x{size}: {ms:.3} ms/frame over {runs} runs, ids {ids:?}");
    }
}
