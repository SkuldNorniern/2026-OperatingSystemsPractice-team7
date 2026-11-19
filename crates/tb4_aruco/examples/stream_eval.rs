// Detector evaluation on recorded camera frames, several parameter sets side by side.
// stdin: frames as [u32 width][u32 height][f64 time][width*height gray bytes], little endian
//   (host_ws/tools/bag_frames.py writes this from a rosbag)
// stdout: one CSV line per detection: time,variant,id,side_px,hamming; a summary goes to stderr
// cargo run --release -p tb4_aruco --example stream_eval < frames.bin
use std::io::{BufWriter, Read, Write};

use tb4_aruco::{Detector, DetectorParams, Dictionary, GrayImage};

fn variants() -> Vec<(&'static str, DetectorParams)> {
    let d = DetectorParams::default();
    vec![
        // "config" = team7.yaml (hamming 0, barcode filter off); keep it in step with the yaml
        ("config_ham0", DetectorParams { max_hamming: 0, max_stripe_flips: None, ..d.clone() }),
        ("config_ham1", DetectorParams { max_hamming: 1, max_stripe_flips: None, ..d.clone() }),
        ("border0", DetectorParams { max_hamming: 1, max_stripe_flips: None, max_border_errors: 0, ..d.clone() }),
        ("aspect2", DetectorParams { max_hamming: 1, max_stripe_flips: None, max_aspect: 2.0, ..d.clone() }),
        ("aspect1.5", DetectorParams { max_hamming: 1, max_stripe_flips: None, max_aspect: 1.5, ..d.clone() }),
        ("border0_aspect2", DetectorParams { max_hamming: 1, max_stripe_flips: None, max_border_errors: 0, max_aspect: 2.0, ..d.clone() }),
        ("no_stripe", DetectorParams { max_stripe_flips: None, ..d.clone() }),
        ("default", d.clone()),
        ("ham0", DetectorParams { max_hamming: 0, ..d.clone() }),
        ("ham0_side12", DetectorParams { max_hamming: 0, min_side_px: 12.0, ..d.clone() }),
        ("ham0_stripe2", DetectorParams { max_hamming: 0, max_stripe_flips: Some(2), ..d }),
    ]
}

fn side(c: &[[f32; 2]; 4]) -> f32 {
    let e = |a: [f32; 2], b: [f32; 2]| (a[0] - b[0]).hypot(a[1] - b[1]);
    (e(c[0], c[1]) + e(c[1], c[2]) + e(c[2], c[3]) + e(c[3], c[0])) / 4.0
}

fn main() {
    let dict = Dictionary::by_name(&std::env::args().nth(1).unwrap_or("ARUCO_ORIGINAL".into())).expect("dictionary");
    let mut dets: Vec<_> = variants().into_iter().map(|(n, p)| (n, Detector::new(p))).collect();
    let mut input = std::io::stdin().lock();
    let mut out = BufWriter::new(std::io::stdout().lock());
    let mut head = [0u8; 16];
    let (mut frames, mut counts) = (0usize, vec![0usize; dets.len()]);
    while input.read_exact(&mut head).is_ok() {
        let w = u32::from_le_bytes(head[0..4].try_into().unwrap()) as usize;
        let h = u32::from_le_bytes(head[4..8].try_into().unwrap()) as usize;
        let t = f64::from_le_bytes(head[8..16].try_into().unwrap());
        let mut px = vec![0u8; w * h];
        if input.read_exact(&mut px).is_err() {
            break;
        }
        let img = GrayImage::from_mono8(w, h, &px).expect("frame");
        frames += 1;
        for (k, (name, det)) in dets.iter_mut().enumerate() {
            for m in det.detect(&img, &dict) {
                counts[k] += 1;
                writeln!(out, "{t:.3},{name},{},{:.1},{}", m.id, side(&m.corners), m.hamming).unwrap();
            }
        }
    }
    eprintln!("{frames} frames");
    for ((name, _), n) in dets.iter().zip(counts) {
        eprintln!("{name:>14}: {n} detections");
    }
}
