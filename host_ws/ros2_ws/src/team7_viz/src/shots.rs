//! Screenshots: the canvas to a PNG file, with a UTC time stamp in the name.

use std::path::Path;
use std::time::SystemTime;

use aurea::render::Canvas;

fn png(w: u32, h: u32, rgba: &[u8]) -> Vec<u8> {
    let mut out = Vec::new();
    let mut enc = png::Encoder::new(&mut out, w, h);
    enc.set_color(png::ColorType::Rgba);
    enc.set_depth(png::BitDepth::Eight);
    if let Ok(mut wr) = enc.write_header() {
        let _ = wr.write_image_data(rgba);
    }
    out
}

/// 0xAARRGGBB pixels -> RGBA bytes, opaque
pub fn rgba_of(px: &[u32]) -> Vec<u8> {
    px.iter().flat_map(|p| [(p >> 16) as u8, (p >> 8) as u8, *p as u8, 255]).collect()
}

/// UTC "20261119_074806" for file names
pub fn stamp_name(t: SystemTime) -> String {
    let s = t.duration_since(SystemTime::UNIX_EPOCH).map_or(0, |d| d.as_secs());
    let (days, rem) = ((s / 86_400) as i64, s % 86_400);
    // civil from days (Howard Hinnant)
    let z = days + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z - era * 146_097;
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let d = doy - (153 * mp + 2) / 5 + 1;
    let m = if mp < 10 { mp + 3 } else { mp - 9 };
    let y = yoe + era * 400 + i64::from(m <= 2);
    format!("{y:04}{m:02}{d:02}_{:02}{:02}{:02}", rem / 3600, rem % 3600 / 60, rem % 60)
}

/// 0xAARRGGBB pixels to a PNG file
pub fn save_rgba(path: &Path, px: &[u32], w: u32, h: u32) -> Result<(), String> {
    if let Some(dir) = path.parent() {
        std::fs::create_dir_all(dir).map_err(|e| e.to_string())?;
    }
    std::fs::write(path, png(w, h, &rgba_of(px))).map_err(|e| e.to_string())
}

/// the canvas's last frame to a PNG file
pub fn save_canvas(canvas: &Canvas, path: &Path) -> Result<(), String> {
    canvas.with_frame_pixels(|px, w, h| save_rgba(path, px, w, h)).unwrap_or_else(|| Err("no frame drawn yet".into()))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn stamp_names_are_utc_dates() {
        let t = SystemTime::UNIX_EPOCH + std::time::Duration::from_secs(1_795_074_486);
        assert_eq!(stamp_name(t), "20261119_074806");
        assert_eq!(stamp_name(SystemTime::UNIX_EPOCH), "19700101_000000");
    }
}
