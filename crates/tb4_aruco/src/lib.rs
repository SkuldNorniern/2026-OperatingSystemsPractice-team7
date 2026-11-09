mod camera;
mod detect;
mod dict;
mod dict_tables;
mod geom;
mod image;
mod refine;
pub mod synth;

pub use camera::{CameraModel, estimate_range, marker_bearing};
pub use detect::{Detector, DetectorParams};
pub use dict::{Dictionary, Match};
pub use image::GrayImage;

// Corners: top-left, top-right, bottom-right, bottom-left of the printed marker, pixel centres at integers.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Marker {
    pub id: u32,
    pub corners: [[f32; 2]; 4],
    pub hamming: u32,
}

impl Marker {
    pub fn center(&self) -> [f32; 2] {
        let c = geom::diag_intersection(&self.corners_f64());
        [c[0] as f32, c[1] as f32]
    }

    pub fn corners_f64(&self) -> [[f64; 2]; 4] {
        self.corners.map(|c| [c[0] as f64, c[1] as f64])
    }
}
