use std::f64::consts::PI;

#[derive(Debug, Clone, PartialEq)]
pub struct Scan {
    pub angle_min: f64,
    pub angle_increment: f64,
    /// seconds between beams, 0 if the driver does not fill it
    pub time_increment: f64,
    pub ranges: Vec<f32>,
}

#[derive(Debug, Clone)]
pub struct ScanParams {
    pub min_range: f32,
    pub max_range: f32,
    /// odd window, 1 = off
    pub median_window: usize,
    /// drop a return when both neighbours are further than this from it (m)
    pub isolated_dist: f32,
    pub deskew: bool,
    /// true if header stamp is the last beam (rplidar_ros), false if first
    pub stamp_at_end: bool,
    /// keep a return that is clearly in front of both neighbours (a chair or desk leg is one or two
    /// beams wide for the A1): no median, no isolated removal. Off by default: on the three field
    /// bags the SLAM maps came out the same with it on (pole-like clusters 38/39/33 -> 36/39/38), the
    /// occupancy update clears one-beam legs anyway, so it is not what keeps legs out of the map.
    pub keep_foreground_spikes: bool,
    /// in front of both neighbours by at least this much (m); missing neighbours count as far
    pub spike_margin: f32,
    /// only this close (m): further out a lone return is more often noise
    pub spike_max_range: f32,
}

impl Default for ScanParams {
    fn default() -> Self {
        Self {
            min_range: 0.18,
            max_range: 8.0,
            median_window: 3,
            isolated_dist: 0.15,
            deskew: true,
            stamp_at_end: false,
            keep_foreground_spikes: false,
            spike_margin: 0.15,
            spike_max_range: 3.0,
        }
    }
}

#[derive(Debug, Default, Clone, Copy, PartialEq)]
pub struct FilterStats {
    pub out_of_range: usize,
    pub isolated: usize,
}

/// invalid returns come back as +inf, same length as input
pub fn filter(scan: &Scan, p: &ScanParams) -> (Scan, FilterStats) {
    let mut st = FilterStats::default();
    let n = scan.ranges.len();
    let mut r: Vec<f32> = scan
        .ranges
        .iter()
        .map(|&v| {
            if v.is_finite() && v >= p.min_range && v <= p.max_range {
                v
            } else {
                if v.is_finite() {
                    st.out_of_range += 1;
                }
                f32::INFINITY
            }
        })
        .collect();

    // thin objects in front of the background, judged on the range-limited input
    let spike: Vec<bool> = (0..n)
        .map(|i| {
            let v = r[i];
            if !p.keep_foreground_spikes || !v.is_finite() || v > p.spike_max_range || n < 3 {
                return false;
            }
            let ahead = |j: usize| !r[j].is_finite() || r[j] - v >= p.spike_margin;
            ahead(if i == 0 { n - 1 } else { i - 1 }) && ahead((i + 1) % n)
        })
        .collect();

    if p.median_window > 1 {
        let half = p.median_window / 2;
        let src = r.clone();
        let mut win = Vec::with_capacity(p.median_window);
        for i in 0..n {
            if !src[i].is_finite() || spike[i] {
                continue;
            }
            win.clear();
            win.extend(src[i.saturating_sub(half)..=(i + half).min(n - 1)].iter().filter(|v| v.is_finite()));
            win.sort_by(f32::total_cmp);
            r[i] = win[win.len() / 2];
        }
    }

    if p.isolated_dist > 0.0 && n >= 3 {
        let src = r.clone();
        let pts: Vec<Option<(f64, f64)>> = (0..n).map(|i| point(scan, &src, i)).collect();
        for i in 0..n {
            let Some(a) = pts[i] else { continue };
            if spike[i] {
                continue;
            }
            let near = |j: usize| pts[j].is_some_and(|b| dist(a, b) <= p.isolated_dist as f64);
            let prev = if i == 0 { n - 1 } else { i - 1 };
            let next = (i + 1) % n;
            if !near(prev) && !near(next) {
                r[i] = f32::INFINITY;
                st.isolated += 1;
            }
        }
    }
    (Scan { ranges: r, ..scan.clone() }, st)
}

/// remove robot motion during the sweep. v, w = base velocity (m/s, rad/s), sensor at base origin.
/// result is re-binned on the same angles, as seen from the pose at stamp time
pub fn deskew(scan: &Scan, v: f64, w: f64, stamp_at_end: bool) -> Scan {
    let n = scan.ranges.len();
    if scan.time_increment <= 0.0 || n == 0 || (v.abs() < 1e-6 && w.abs() < 1e-6) {
        return scan.clone();
    }
    let t_ref = if stamp_at_end { (n - 1) as f64 * scan.time_increment } else { 0.0 };
    let mut out = vec![f32::INFINITY; n];
    for i in 0..n {
        let Some((px, py)) = point(scan, &scan.ranges, i) else { continue };
        let dt = i as f64 * scan.time_increment - t_ref;
        // pose of sensor at beam time relative to reference pose (unicycle, constant v w)
        let th = w * dt;
        let (tx, ty) = if w.abs() < 1e-9 {
            (v * dt, 0.0)
        } else {
            (v / w * th.sin(), v / w * (1.0 - th.cos()))
        };
        let (c, s) = (th.cos(), th.sin());
        let x = c * px - s * py + tx;
        let y = s * px + c * py + ty;
        let ang = y.atan2(x);
        let k = bin(scan, ang);
        if let Some(k) = k {
            let rr = x.hypot(y) as f32;
            if rr < out[k] {
                out[k] = rr;
            }
        }
    }
    Scan { ranges: out, ..scan.clone() }
}

pub fn to_points(scan: &Scan) -> Vec<(f64, f64)> {
    (0..scan.ranges.len()).filter_map(|i| point(scan, &scan.ranges, i)).collect()
}

fn point(scan: &Scan, r: &[f32], i: usize) -> Option<(f64, f64)> {
    let v = r[i];
    if !v.is_finite() {
        return None;
    }
    let a = scan.angle_min + i as f64 * scan.angle_increment;
    Some((v as f64 * a.cos(), v as f64 * a.sin()))
}

fn bin(scan: &Scan, ang: f64) -> Option<usize> {
    let n = scan.ranges.len() as i64;
    let mut rel = ang - scan.angle_min;
    let span = scan.angle_increment * n as f64;
    if span >= 2.0 * PI - 1e-6 {
        rel = rel.rem_euclid(2.0 * PI);
    }
    let k = (rel / scan.angle_increment).round() as i64;
    if span >= 2.0 * PI - 1e-6 {
        Some(k.rem_euclid(n) as usize)
    } else {
        (0..n).contains(&k).then_some(k as usize)
    }
}

fn dist(a: (f64, f64), b: (f64, f64)) -> f64 {
    (a.0 - b.0).hypot(a.1 - b.1)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 360 beams of a square room centered on the sensor, half size `h`
    fn room(h: f64) -> Scan {
        let n = 360;
        let inc = 2.0 * PI / n as f64;
        let ranges = (0..n)
            .map(|i| {
                let a = -PI + i as f64 * inc;
                (h / a.cos().abs().max(a.sin().abs())) as f32
            })
            .collect();
        Scan { angle_min: -PI, angle_increment: inc, time_increment: 1.0 / (7.0 * n as f64), ranges }
    }

    #[test]
    fn range_limits_and_inf() {
        let mut s = room(2.0);
        s.ranges[10] = 0.05;
        s.ranges[20] = 20.0;
        s.ranges[30] = f32::NAN;
        let p = ScanParams { median_window: 1, isolated_dist: 0.0, ..Default::default() };
        let (f, st) = filter(&s, &p);
        assert!(f.ranges[10].is_infinite() && f.ranges[20].is_infinite() && f.ranges[30].is_infinite());
        assert_eq!(st.out_of_range, 2);
        assert_eq!(f.ranges.len(), 360);
        assert_eq!(f.ranges[100], s.ranges[100]);
    }

    #[test]
    fn median_and_isolated_remove_spikes() {
        // default (spike keeping off): a one-beam return in front of the wall is smoothed away
        let old = ScanParams::default();
        let mut s = room(2.0);
        s.ranges[180] = 0.6;
        let (f, st) = filter(&s, &old);
        assert!((f.ranges[180] - room(2.0).ranges[180]).abs() < 0.01, "{}", f.ranges[180]);
        let mut s2 = room(2.0);
        for i in 0..360 {
            if i != 200 {
                s2.ranges[i] = f32::INFINITY;
            }
        }
        let (f2, st2) = filter(&s2, &ScanParams { median_window: 1, ..old.clone() });
        assert!(f2.ranges[200].is_infinite());
        assert_eq!(st2.isolated, 1);
        assert_eq!(st.isolated, 0);
    }

    #[test]
    fn thin_poles_in_front_of_the_wall_are_kept() {
        let p = ScanParams { keep_foreground_spikes: true, ..Default::default() };
        // chair leg: one beam at 0.6 m and a two-beam leg at 1.2 m, wall at 2 m
        let mut s = room(2.0);
        s.ranges[180] = 0.6;
        s.ranges[90] = 1.2;
        s.ranges[91] = 1.21;
        let (f, st) = filter(&s, &p);
        assert!((f.ranges[180] - 0.6).abs() < 1e-6, "one-beam leg kept: {}", f.ranges[180]);
        assert!((f.ranges[90] - 1.2).abs() < 0.02 && (f.ranges[91] - 1.21).abs() < 0.02, "two-beam leg kept");
        assert_eq!(st.isolated, 0);
        // a lone return in open space (no neighbours at all) within range is kept too
        let mut s2 = room(2.0);
        for i in 0..360 {
            if i != 200 {
                s2.ranges[i] = f32::INFINITY;
            }
        }
        let (f2, _) = filter(&s2, &p);
        assert!(f2.ranges[200].is_finite());
    }

    #[test]
    fn edge_noise_and_far_spikes_are_still_cleaned() {
        let p = ScanParams { median_window: 1, keep_foreground_spikes: true, ..Default::default() };
        // mixed pixel between a near edge (1.0) and the wall behind (2.0): not in front of both, removed
        let mut s = room(2.0);
        for r in &mut s.ranges[100..120] {
            *r = 1.0;
        }
        s.ranges[120] = 1.5;
        let (f, st) = filter(&s, &p);
        assert!(f.ranges[120].is_infinite(), "mixed pixel {}", f.ranges[120]);
        assert!(st.isolated >= 1);
        // lone return beyond spike_max_range: treated as noise as before
        let mut s3 = room(6.0);
        for i in 0..360 {
            if i != 50 {
                s3.ranges[i] = f32::INFINITY;
            }
        }
        s3.ranges[50] = 4.0;
        let (f3, _) = filter(&s3, &p);
        assert!(f3.ranges[50].is_infinite());
    }

    #[test]
    fn deskew_undoes_rotation() {
        // robot spins at w during sweep: beam i was taken after rotating w*t_i, so it appears at a smaller angle
        let w = 1.0;
        let truth = room(2.0);
        let n = truth.ranges.len();
        let mut skewed = truth.clone();
        for i in 0..n {
            let t = i as f64 * truth.time_increment;
            let world_ang = truth.angle_min + i as f64 * truth.angle_increment + w * t;
            let h = 2.0;
            skewed.ranges[i] = (h / world_ang.cos().abs().max(world_ang.sin().abs())) as f32;
        }
        let fixed = deskew(&skewed, 0.0, w, false);
        let err = |s: &Scan| -> f32 {
            (0..n).filter(|&i| s.ranges[i].is_finite()).map(|i| (s.ranges[i] - truth.ranges[i]).abs()).fold(0.0, f32::max)
        };
        assert!(err(&fixed) < err(&skewed) * 0.3, "{} vs {}", err(&fixed), err(&skewed));
        assert!(fixed.ranges.iter().filter(|r| r.is_finite()).count() > 340);
    }

    #[test]
    fn deskew_noop_when_still() {
        let s = room(1.5);
        assert_eq!(deskew(&s, 0.0, 0.0, false), s);
        let mut no_t = s.clone();
        no_t.time_increment = 0.0;
        assert_eq!(deskew(&no_t, 0.3, 1.0, false), no_t);
    }

    #[test]
    fn points_count() {
        let mut s = room(1.0);
        s.ranges[0] = f32::INFINITY;
        assert_eq!(to_points(&s).len(), 359);
    }
}
