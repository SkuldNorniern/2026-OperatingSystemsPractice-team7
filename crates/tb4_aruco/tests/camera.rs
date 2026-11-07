use tb4_aruco::{CameraModel, Marker, estimate_range, marker_bearing};

fn cam() -> CameraModel {
    let k = [200.0, 0.0, 124.5, 0.0, 201.0, 125.5, 0.0, 0.0, 1.0];
    CameraModel::from_k_d(&k, &[-0.12, 0.05, 0.001, -0.002, -0.01])
}

fn close(a: [f64; 3], b: [f64; 3], tol: f64) -> bool {
    a.iter().zip(&b).all(|(x, y)| (x - y).abs() < tol)
}

#[test]
fn principal_point_ray() {
    let c = cam();
    assert!(close(c.pixel_to_ray(c.cx, c.cy), [0.0, 0.0, 1.0], 1e-12));
    let r = c.pixel_to_ray(240.0, 10.0);
    assert!((r[0] * r[0] + r[1] * r[1] + r[2] * r[2] - 1.0).abs() < 1e-12);
    assert!(r[0] > 0.0 && r[1] < 0.0 && r[2] > 0.0);
}

#[test]
fn undistort_round_trip() {
    let c = cam();
    for &(u, v) in &[
        (0.0, 0.0),
        (249.0, 249.0),
        (30.0, 200.0),
        (124.5, 10.0),
        (180.0, 140.0),
    ] {
        let (du, dv) = c.distort_pixel(u, v);
        let (uu, uv) = c.undistort_pixel(du, dv);
        assert!(
            (uu - u).abs() < 1e-6 && (uv - v).abs() < 1e-6,
            "{u},{v} -> {du},{dv} -> {uu},{uv}"
        );
    }
    let plain = CameraModel::from_k_d(&[100.0, 0.0, 50.0, 0.0, 100.0, 50.0, 0.0, 0.0, 1.0], &[]);
    assert_eq!(plain.undistort_pixel(12.0, 34.0), (12.0, 34.0));
}

fn project(c: &CameraModel, p: [f64; 3]) -> [f32; 2] {
    let (u, v) = c.normalized_to_pixel(p[0] / p[2], p[1] / p[2]);
    let (u, v) = c.distort_pixel(u, v);
    [u as f32, v as f32]
}

#[test]
fn bearing_and_range() {
    let c = cam();
    let side = 0.1;
    let centre = [0.3, -0.1, 1.5];
    // Marker tilted 40 degrees about the camera y axis.
    let (s, co) = 40f64.to_radians().sin_cos();
    let local = [[-0.5, -0.5], [0.5, -0.5], [0.5, 0.5], [-0.5, 0.5]];
    let corners = local.map(|[x, y]| {
        let (px, py) = (x * side, y * side);
        project(
            &c,
            [centre[0] + co * px, centre[1] + py, centre[2] - s * px],
        )
    });
    let m = Marker {
        id: 1,
        corners,
        hamming: 0,
    };
    let n = (centre[0] * centre[0] + centre[1] * centre[1] + centre[2] * centre[2]).sqrt();
    let truth = centre.map(|v| v / n);
    assert!(close(marker_bearing(&m, &c), truth, 1e-5));
    let r = estimate_range(&m, &c, side);
    assert!((r - n).abs() / n < 0.01, "range {r} vs {n}");
}
