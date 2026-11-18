use crate::Marker;
use crate::geom::{Homography, diag_intersection};

// Pinhole + distortion as in sensor_msgs/CameraInfo: plumb_bob (k1, k2, p1, p2, k3) or
// rational_polynomial (k1, k2, p1, p2, k3, k4, k5, k6), OAK-D reports the second.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CameraModel {
    pub fx: f64,
    pub fy: f64,
    pub cx: f64,
    pub cy: f64,
    pub dist: [f64; 8],
}

impl CameraModel {
    pub fn new(fx: f64, fy: f64, cx: f64, cy: f64) -> Self {
        Self {
            fx,
            fy,
            cx,
            cy,
            dist: [0.0; 8],
        }
    }

    // Up to eight coefficients of `d` are used, missing ones are zero.
    pub fn from_k_d(k: &[f64; 9], d: &[f64]) -> Self {
        let mut dist = [0.0; 8];
        for (o, v) in dist.iter_mut().zip(d) {
            *o = *v;
        }
        Self {
            fx: k[0],
            fy: k[4],
            cx: k[2],
            cy: k[5],
            dist,
        }
    }

    fn radial(&self, r2: f64) -> f64 {
        let [k1, k2, _, _, k3, k4, k5, k6] = self.dist;
        (1.0 + r2 * (k1 + r2 * (k2 + r2 * k3))) / (1.0 + r2 * (k4 + r2 * (k5 + r2 * k6)))
    }

    pub fn distort_normalized(&self, x: f64, y: f64) -> (f64, f64) {
        let [_, _, p1, p2, ..] = self.dist;
        let r2 = x * x + y * y;
        let rad = self.radial(r2);
        (
            x * rad + 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x),
            y * rad + p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y,
        )
    }

    // Newton with numeric jacobian, starts from the distorted point.
    pub fn undistort_normalized(&self, xd: f64, yd: f64) -> (f64, f64) {
        if self.dist.iter().all(|&v| v == 0.0) {
            return (xd, yd);
        }
        let (mut x, mut y) = (xd, yd);
        let h = 1e-7;
        for _ in 0..30 {
            let (fx, fy) = self.distort_normalized(x, y);
            let (ex, ey) = (fx - xd, fy - yd);
            if ex.abs() < 1e-14 && ey.abs() < 1e-14 {
                break;
            }
            let (ax, ay) = self.distort_normalized(x + h, y);
            let (bx, by) = self.distort_normalized(x, y + h);
            let (a, c) = ((ax - fx) / h, (ay - fy) / h);
            let (b, d) = ((bx - fx) / h, (by - fy) / h);
            let det = a * d - b * c;
            if det.abs() < 1e-12 {
                break;
            }
            x -= (d * ex - b * ey) / det;
            y -= (a * ey - c * ex) / det;
        }
        (x, y)
    }

    pub fn pixel_to_normalized(&self, u: f64, v: f64) -> (f64, f64) {
        self.undistort_normalized((u - self.cx) / self.fx, (v - self.cy) / self.fy)
    }

    pub fn normalized_to_pixel(&self, x: f64, y: f64) -> (f64, f64) {
        (self.fx * x + self.cx, self.fy * y + self.cy)
    }

    pub fn undistort_pixel(&self, u: f64, v: f64) -> (f64, f64) {
        let (x, y) = self.pixel_to_normalized(u, v);
        self.normalized_to_pixel(x, y)
    }

    pub fn distort_pixel(&self, u: f64, v: f64) -> (f64, f64) {
        let (x, y) = self.distort_normalized((u - self.cx) / self.fx, (v - self.cy) / self.fy);
        self.normalized_to_pixel(x, y)
    }

    // Unit ray in the optical frame (x right, y down, z forward).
    pub fn pixel_to_ray(&self, u: f64, v: f64) -> [f64; 3] {
        let (x, y) = self.pixel_to_normalized(u, v);
        normalize([x, y, 1.0])
    }
}

fn normalize(v: [f64; 3]) -> [f64; 3] {
    let n = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
    [v[0] / n, v[1] / n, v[2] / n]
}

fn undistorted_corners(marker: &Marker, cam: &CameraModel) -> [[f64; 2]; 4] {
    marker.corners.map(|c| {
        let (x, y) = cam.pixel_to_normalized(c[0] as f64, c[1] as f64);
        [x, y]
    })
}

pub fn marker_bearing(marker: &Marker, cam: &CameraModel) -> [f64; 3] {
    let c = diag_intersection(&undistorted_corners(marker, cam));
    normalize([c[0], c[1], 1.0])
}

// Distance from the camera centre to the marker centre, from the plane homography; NaN if degenerate.
pub fn estimate_range(marker: &Marker, cam: &CameraModel, side_m: f64) -> f64 {
    let q = undistorted_corners(marker, cam);
    let Some(h) = Homography::from_unit_square(&q) else {
        return f64::NAN;
    };
    let m = &h.0;
    let n1 = (m[0] * m[0] + m[3] * m[3] + m[6] * m[6]).sqrt();
    let n2 = (m[1] * m[1] + m[4] * m[4] + m[7] * m[7]).sqrt();
    let scale = 2.0 * side_m / (n1 + n2);
    let cx = 0.5 * (m[0] + m[1]) + m[2];
    let cy = 0.5 * (m[3] + m[4]) + m[5];
    let cz = 0.5 * (m[6] + m[7]) + m[8];
    scale * (cx * cx + cy * cy + cz * cz).sqrt()
}
