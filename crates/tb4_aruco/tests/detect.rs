use std::f64::consts::FRAC_PI_2;

use tb4_aruco::synth::{add_noise, gaussian_blur, render, square_corners};
use tb4_aruco::{Detector, DetectorParams, Dictionary, GrayImage, Marker};

fn load_pgm(bytes: &[u8]) -> GrayImage {
    let mut fields = Vec::new();
    let mut i = 0;
    while fields.len() < 4 {
        while bytes[i].is_ascii_whitespace() {
            i += 1;
        }
        if bytes[i] == b'#' {
            while bytes[i] != b'\n' {
                i += 1;
            }
            continue;
        }
        let s = i;
        while !bytes[i].is_ascii_whitespace() {
            i += 1;
        }
        fields.push(std::str::from_utf8(&bytes[s..i]).unwrap().to_string());
    }
    assert_eq!(fields[0], "P5");
    let w: usize = fields[1].parse().unwrap();
    let h: usize = fields[2].parse().unwrap();
    GrayImage::from_mono8(w, h, &bytes[i + 1..i + 1 + w * h]).unwrap()
}

fn corner_err(m: &Marker, truth: &[[f64; 2]; 4]) -> f64 {
    m.corners
        .iter()
        .zip(truth)
        .map(|(c, t)| (c[0] as f64 - t[0]).hypot(c[1] as f64 - t[1]))
        .fold(0.0, f64::max)
}

fn detect(img: &GrayImage) -> Vec<Marker> {
    Detector::new(DetectorParams::default()).detect(img, &Dictionary::aruco_original())
}

fn check(found: &[Marker], expected: &[(u32, [[f64; 2]; 4])], tol: f64) {
    assert_eq!(found.len(), expected.len(), "found {found:?}");
    for (id, q) in expected {
        let m = found
            .iter()
            .find(|m| m.id == *id)
            .unwrap_or_else(|| panic!("id {id} missing in {found:?}"));
        let e = corner_err(m, q);
        assert!(
            e <= tol,
            "id {id} corner error {e:.3} > {tol}: {:?} vs {q:?}",
            m.corners
        );
    }
}

fn scene(
    w: usize,
    h: usize,
    markers: &[(u32, [[f64; 2]; 4])],
    sigma: f64,
    noise: i32,
    seed: u64,
) -> GrayImage {
    let dict = Dictionary::aruco_original();
    let img = render(w, h, &dict, markers, 4);
    let mut img = if sigma > 0.0 {
        gaussian_blur(&img, sigma)
    } else {
        img
    };
    if noise > 0 {
        add_noise(&mut img, noise, seed);
    }
    img
}

#[test]
fn golden_opencv_fixture() {
    let img = load_pgm(include_bytes!("fixtures/opencv_multi.pgm"));
    // (id, side px, x, y, np.rot90 k) as placed by the generator script.
    let placed = [
        (0u32, 28.0, 10.0, 10.0, 0usize),
        (7, 21.0, 60.0, 15.0, 2),
        (42, 49.0, 100.0, 10.0, 0),
        (1023, 35.0, 20.0, 70.0, 1),
        (5, 14.0, 80.0, 85.0, 3),
    ];
    let expected: Vec<_> = placed
        .iter()
        .map(|&(id, s, x, y, k)| {
            let (x0, y0, x1, y1) = (x - 0.5, y - 0.5, x + s - 0.5, y + s - 0.5);
            let sq = [[x0, y0], [x1, y0], [x1, y1], [x0, y1]];
            (id, [0, 1, 2, 3].map(|j| sq[(j + 4 - k) % 4]))
        })
        .collect();
    check(&detect(&img), &expected, 0.6);
}

#[test]
fn in_plane_rotations() {
    for k in 0..4 {
        let a = k as f64 * FRAC_PI_2 + 0.2;
        let q = square_corners(60.0, 58.0, 40.0, a);
        let img = scene(120, 120, &[(42, q)], 0.8, 6, k);
        check(&detect(&img), &[(42, q)], 1.0);
    }
}

#[test]
fn perspective() {
    let q = [[40.0, 30.0], [120.0, 42.0], [116.0, 98.0], [44.0, 112.0]];
    let img = scene(160, 140, &[(777, q)], 0.7, 5, 3);
    check(&detect(&img), &[(777, q)], 1.0);
    let q = [[30.0, 50.0], [95.0, 40.0], [100.0, 75.0], [28.0, 70.0]];
    let img = scene(130, 110, &[(300, q)], 0.7, 5, 4);
    check(&detect(&img), &[(300, q)], 1.0);
}

#[test]
fn small_markers() {
    for (i, (side, angle)) in [
        (15.0, 0.0),
        (15.0, 0.35),
        (14.0, 2.0),
        (13.0, -0.6),
        (16.0, 4.0),
    ]
    .iter()
    .enumerate()
    {
        let id = 100 + 37 * i as u32;
        let q = square_corners(30.3, 29.7, *side, *angle);
        let img = scene(60, 60, &[(id, q)], 0.6, 4, i as u64);
        check(&detect(&img), &[(id, q)], 1.0);
    }
}

#[test]
fn multiple_markers_250() {
    let ms = [
        (0u32, square_corners(50.0, 50.0, 40.0, 0.1)),
        (7, square_corners(180.0, 60.0, 30.0, 2.5)),
        (512, square_corners(70.0, 180.0, 18.0, -0.4)),
        (1000, square_corners(170.0, 180.0, 60.0, 1.0)),
    ];
    let img = scene(250, 250, &ms, 0.8, 6, 9);
    check(&detect(&img), &ms, 1.0);
}

#[test]
fn all_ids_decode() {
    let dict = Dictionary::aruco_original();
    let mut det = Detector::new(DetectorParams::default());
    for chunk in 0..4u32 {
        let ms: Vec<_> = (0..256u32)
            .map(|i| {
                let (gx, gy) = ((i % 16) as f64, (i / 16) as f64);
                (
                    chunk * 256 + i,
                    square_corners(20.0 + 36.0 * gx, 20.0 + 36.0 * gy, 24.0, 0.0),
                )
            })
            .collect();
        let img = render(16 * 36 + 4, 16 * 36 + 4, &dict, &ms, 2);
        let found = det.detect(&img, &dict);
        assert_eq!(found.len(), 256);
        for (id, q) in &ms {
            let c = [(q[0][0] + q[2][0]) / 2.0, (q[0][1] + q[2][1]) / 2.0];
            let m = found
                .iter()
                .find(|m| (m.center()[0] as f64 - c[0]).hypot(m.center()[1] as f64 - c[1]) < 2.0)
                .unwrap();
            let expect = dict.identify(dict.codes[*id as usize], 0).unwrap().id;
            assert_eq!(m.id, expect);
            let code = dict.codes[*id as usize];
            let symmetric = dict.rotate_cw(dict.rotate_cw(code)) == code;
            if m.id == *id && !symmetric {
                assert!(
                    corner_err(m, q) < 0.6,
                    "id {id} err {} {:?} {q:?}",
                    corner_err(m, q),
                    m.corners
                );
            }
        }
    }
}

#[test]
fn no_false_positives() {
    let blank = GrayImage::filled(250, 250, 128);
    assert!(detect(&blank).is_empty());
    let mut black = GrayImage::filled(250, 250, 0);
    assert!(detect(&black).is_empty());
    add_noise(&mut black, 127, 1);
    assert!(detect(&black).is_empty());
    for seed in 0..4 {
        let mut img = GrayImage::filled(250, 250, 128);
        add_noise(&mut img, 127, seed);
        assert!(detect(&img).is_empty());
        let blurred = gaussian_blur(&img, 1.0 + seed as f64 * 0.5);
        assert!(detect(&blurred).is_empty());
    }
}

#[test]
fn detector_reuse_and_sizes() {
    let dict = Dictionary::aruco_original();
    let mut det = Detector::new(DetectorParams::default());
    let q = square_corners(60.0, 60.0, 30.0, 0.5);
    let img = scene(120, 120, &[(9, q)], 0.7, 4, 1);
    let a = det.detect(&img, &dict);
    let b = det.detect(&img, &dict);
    assert_eq!(a, b);
    let q2 = square_corners(100.0, 80.0, 30.0, 0.5);
    let img2 = scene(200, 160, &[(10, q2)], 0.7, 4, 2);
    check(&det.detect(&img2, &dict), &[(10, q2)], 1.0);
    assert!(det.detect(&GrayImage::new(2, 2), &dict).is_empty());
}

#[test]
fn no_refinement_still_detects() {
    let q = square_corners(60.0, 60.0, 36.0, 0.3);
    let img = scene(120, 120, &[(55, q)], 0.7, 4, 5);
    let params = DetectorParams {
        refine_corners: false,
        ..Default::default()
    };
    let found = Detector::new(params).detect(&img, &Dictionary::aruco_original());
    check(&found, &[(55, q)], 1.5);
}

#[test]
fn image_conversions() {
    let rgb = [255u8, 0, 0, 0, 255, 0, 0, 0, 255, 9, 9, 9];
    let g = GrayImage::from_rgb8(2, 1, &rgb[..6]).unwrap();
    assert_eq!(g.data, vec![77, 149]);
    let b = GrayImage::from_bgr8(2, 1, &rgb[..6]).unwrap();
    assert_eq!(b.data, vec![29, 149]);
    let s = GrayImage::from_rgb8_step(1, 2, 6, &rgb[..9]).unwrap();
    assert_eq!(s.data, vec![77, 29]);
    let m = GrayImage::from_mono8_step(2, 2, 3, &[1, 2, 0, 3, 4]).unwrap();
    assert_eq!(m.data, vec![1, 2, 3, 4]);
    assert!(GrayImage::from_mono8(3, 3, &[0; 8]).is_none());
}

#[test]
fn single_bit_error_is_corrected() {
    let dict = Dictionary::aruco_original();
    let mut corrupted = dict.clone();
    corrupted.codes[321] ^= 1 << 12;
    let q = square_corners(50.0, 50.0, 49.0, 0.25);
    let img = render(100, 100, &corrupted, &[(321, q)], 4);
    let mut img = gaussian_blur(&img, 0.6);
    add_noise(&mut img, 4, 2);
    let found = detect(&img);
    check(&found, &[(321, q)], 1.0);
    assert_eq!(found[0].hamming, 1);
    let strict = DetectorParams {
        max_hamming: 0,
        ..Default::default()
    };
    assert!(Detector::new(strict).detect(&img, &dict).is_empty());
}

#[test]
fn every_dictionary_detects_its_own_markers() {
    for name in Dictionary::names() {
        let dict = Dictionary::by_name(name).unwrap();
        // first, middle and last id, rotated so orientation decoding is exercised too
        let ids = [0, dict.len() as u32 / 2, dict.len() as u32 - 1];
        let ms: Vec<_> = ids
            .iter()
            .enumerate()
            .map(|(k, &id)| (id, square_corners(60.0 + 110.0 * k as f64, 70.0, 70.0, 0.3 * k as f64)))
            .collect();
        let img = render(340, 140, &dict, &ms, 4);
        let found = Detector::new(DetectorParams::default()).detect(&img, &dict);
        let mut got: Vec<u32> = found.iter().map(|m| m.id).collect();
        got.sort();
        assert_eq!(got, ids.to_vec(), "{name}");
    }
}
