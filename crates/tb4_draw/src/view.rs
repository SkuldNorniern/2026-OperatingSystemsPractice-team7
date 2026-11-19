//! World (metres, x east, y north) to screen (pixels, y down) and the interactive camera on top.

/// metres -> pixels, north up: world point (`x0`, `y1`) lands on screen (`left`, `top`)
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Viewport {
    pub x0: f64,
    pub y1: f64,
    /// pixels per metre
    pub scale: f64,
    pub left: f64,
    pub top: f64,
}

/// a screen rectangle (x, y, w, h)
pub type ScreenRect = (f64, f64, f64, f64);

impl Viewport {
    /// fit `bounds` [x0, y0, x1, y1] into the screen rect (x, y, w, h) with `margin` pixels around
    pub fn fit(bounds: [f64; 4], rect: ScreenRect, margin: f64) -> Self {
        let [x0, y0, x1, y1] = bounds;
        let (bw, bh) = ((x1 - x0).max(1e-6), (y1 - y0).max(1e-6));
        let (aw, ah) = ((rect.2 - 2.0 * margin).max(1.0), (rect.3 - 2.0 * margin).max(1.0));
        let scale = (aw / bw).min(ah / bh);
        Self { x0, y1, scale, left: rect.0 + (rect.2 - bw * scale) / 2.0, top: rect.1 + (rect.3 - bh * scale) / 2.0 }
    }

    /// world point `centre` in the middle of `rect`, `scale` pixels per metre
    pub fn centred(centre: (f64, f64), scale: f64, rect: ScreenRect) -> Self {
        Self { x0: centre.0, y1: centre.1, scale, left: rect.0 + rect.2 / 2.0, top: rect.1 + rect.3 / 2.0 }
    }

    pub fn to_screen(&self, x: f64, y: f64) -> (f32, f32) {
        ((self.left + (x - self.x0) * self.scale) as f32, (self.top + (self.y1 - y) * self.scale) as f32)
    }

    pub fn to_world(&self, sx: f64, sy: f64) -> (f64, f64) {
        (self.x0 + (sx - self.left) / self.scale, self.y1 - (sy - self.top) / self.scale)
    }

    pub fn len(&self, metres: f64) -> f32 {
        (metres * self.scale) as f32
    }
}

pub const MIN_ZOOM: f64 = 0.1;
pub const MAX_ZOOM: f64 = 80.0;

/// zoom, pan and follow on top of a fitted view. `zoom` multiplies the fit scale; `centre` is the
/// world point in the middle of the view (None: the fitted centre); `follow` keeps the robot there.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Camera {
    pub zoom: f64,
    pub centre: Option<(f64, f64)>,
    pub follow: bool,
}

impl Default for Camera {
    fn default() -> Self {
        Self { zoom: 1.0, centre: None, follow: false }
    }
}

fn mid(rect: ScreenRect) -> (f64, f64) {
    (rect.0 + rect.2 / 2.0, rect.1 + rect.3 / 2.0)
}

impl Camera {
    /// the viewport for this frame: `bounds` fitted into `rect`, then zoomed and centred
    pub fn viewport(&self, bounds: [f64; 4], rect: ScreenRect, margin: f64, robot: Option<(f64, f64)>) -> Viewport {
        let fit = Viewport::fit(bounds, rect, margin);
        let centre = if self.follow { robot.or(self.centre) } else { self.centre };
        if centre.is_none() && self.zoom == 1.0 {
            return fit;
        }
        let (mx, my) = mid(rect);
        let c = centre.unwrap_or_else(|| fit.to_world(mx, my));
        Viewport::centred(c, fit.scale * self.zoom, rect)
    }

    /// zoom by `factor` keeping the world point under `cursor` still (around the robot when following)
    pub fn zoom_at(&mut self, vp: &Viewport, rect: ScreenRect, cursor: (f64, f64), factor: f64) {
        let zoom = (self.zoom * factor).clamp(MIN_ZOOM, MAX_ZOOM);
        let applied = zoom / self.zoom;
        self.zoom = zoom;
        if self.follow {
            return;
        }
        let (wx, wy) = vp.to_world(cursor.0, cursor.1);
        let scale = vp.scale * applied;
        let (mx, my) = mid(rect);
        self.centre = Some((wx - (cursor.0 - mx) / scale, wy + (cursor.1 - my) / scale));
    }

    /// drag by (dx, dy) screen pixels: the map moves with the pointer; stops following
    pub fn pan(&mut self, vp: &Viewport, rect: ScreenRect, dx: f64, dy: f64) {
        let (mx, my) = mid(rect);
        let (cx, cy) = vp.to_world(mx - dx, my - dy);
        self.centre = Some((cx, cy));
        self.follow = false;
    }

    /// back to the fitted view (keeps follow)
    pub fn reset(&mut self) {
        *self = Self { follow: self.follow, ..Self::default() };
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn close(a: (f64, f64), b: (f64, f64)) -> bool {
        (a.0 - b.0).abs() < 1e-6 && (a.1 - b.1).abs() < 1e-6
    }

    #[test]
    fn fit_keeps_aspect_north_up_and_centres() {
        // 10 x 5 m into 1000 x 1000 px: 100 px/m, centred vertically
        let vp = Viewport::fit([0.0, 0.0, 10.0, 5.0], (0.0, 0.0, 1000.0, 1000.0), 0.0);
        assert!((vp.scale - 100.0).abs() < 1e-9);
        assert_eq!(vp.to_screen(0.0, 5.0), (0.0, 250.0));
        assert_eq!(vp.to_screen(10.0, 0.0), (1000.0, 750.0));
        assert!(vp.to_screen(1.0, 4.0).1 < vp.to_screen(1.0, 1.0).1);
        assert_eq!(vp.len(0.171), 17.1);
    }

    #[test]
    fn to_world_inverts_to_screen() {
        let vp = Viewport::fit([-3.0, -1.0, 7.0, 4.0], (20.0, 10.0, 640.0, 480.0), 12.0);
        let (sx, sy) = vp.to_screen(1.25, -0.5);
        assert!(close(vp.to_world(f64::from(sx), f64::from(sy)), (1.25, -0.5)));
    }

    #[test]
    fn default_camera_is_the_fit() {
        let rect = (0.0, 0.0, 800.0, 600.0);
        let b = [0.0, 0.0, 10.0, 5.0];
        assert_eq!(Camera::default().viewport(b, rect, 10.0, Some((3.0, 3.0))), Viewport::fit(b, rect, 10.0));
    }

    #[test]
    fn zoom_keeps_the_point_under_the_cursor() {
        let rect = (0.0, 0.0, 800.0, 600.0);
        let b = [0.0, 0.0, 10.0, 5.0];
        let mut cam = Camera::default();
        let vp = cam.viewport(b, rect, 0.0, None);
        let cursor = (200.0, 150.0);
        let before = vp.to_world(cursor.0, cursor.1);
        cam.zoom_at(&vp, rect, cursor, 2.0);
        let vp2 = cam.viewport(b, rect, 0.0, None);
        assert!((vp2.scale - 2.0 * vp.scale).abs() < 1e-9);
        assert!(close(vp2.to_world(cursor.0, cursor.1), before));
        // zoom is clamped
        cam.zoom_at(&vp2, rect, cursor, 1e9);
        assert_eq!(cam.zoom, MAX_ZOOM);
    }

    #[test]
    fn pan_moves_the_map_with_the_pointer_and_stops_following() {
        let rect = (0.0, 0.0, 800.0, 600.0);
        let b = [0.0, 0.0, 10.0, 5.0];
        let mut cam = Camera { follow: true, ..Camera::default() };
        let vp = cam.viewport(b, rect, 0.0, Some((2.0, 2.0)));
        // following: robot in the middle
        assert_eq!(vp.to_screen(2.0, 2.0), (400.0, 300.0));
        let (sx, sy) = vp.to_screen(5.0, 1.0);
        cam.pan(&vp, rect, 30.0, -20.0);
        assert!(!cam.follow);
        let vp2 = cam.viewport(b, rect, 0.0, Some((2.0, 2.0)));
        let (sx2, sy2) = vp2.to_screen(5.0, 1.0);
        assert!((sx2 - sx - 30.0).abs() < 1e-3 && (sy2 - sy + 20.0).abs() < 1e-3);
        cam.reset();
        assert_eq!(cam, Camera::default());
    }

    #[test]
    fn follow_zoom_stays_on_the_robot() {
        let rect = (0.0, 0.0, 800.0, 600.0);
        let mut cam = Camera { follow: true, ..Camera::default() };
        let vp = cam.viewport([0.0, 0.0, 10.0, 5.0], rect, 0.0, Some((7.0, 1.0)));
        cam.zoom_at(&vp, rect, (10.0, 10.0), 3.0);
        let vp2 = cam.viewport([0.0, 0.0, 10.0, 5.0], rect, 0.0, Some((7.0, 1.0)));
        assert_eq!(vp2.to_screen(7.0, 1.0), (400.0, 300.0));
    }
}
