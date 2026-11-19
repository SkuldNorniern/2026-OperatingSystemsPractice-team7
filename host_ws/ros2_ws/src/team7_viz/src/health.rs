//! Health checks for the panel: topic rates against what the robot normally gives, TF age and the
//! scan delay, and a one-line summary for the header strip. Pure functions of the model.

use std::time::{Duration, Instant};

use crate::model::{Model, RateMeter, Topic, human_age};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Level {
    /// within the expected range
    Ok,
    /// low or late
    Warn,
    /// stopped or far off
    Bad,
    /// never seen, or nothing is coming at all (then the banner says so once)
    Off,
}

impl Level {
    /// CSS class of the level
    pub fn class(self) -> &'static str {
        match self {
            Level::Ok => "ok",
            Level::Warn => "warn",
            Level::Bad => "bad",
            Level::Off => "off",
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct Check {
    pub name: &'static str,
    /// the measured value, e.g. "7.7 Hz"
    pub value: String,
    /// what it should be, e.g. "5-10 Hz"
    pub expect: &'static str,
    pub level: Level,
}

/// (ok from, warn from, stale after seconds, expected text)
fn expected(t: Topic) -> (f64, f64, f64, &'static str) {
    match t {
        Topic::Scan => (5.0, 2.0, 1.5, "5-10 Hz"),
        Topic::Odom => (10.0, 3.0, 1.0, "~20 Hz"),
        Topic::Tf => (10.0, 3.0, 1.0, ">10 Hz"),
        Topic::Map => (0.3, 0.1, 6.0, "~1 Hz"),
        Topic::Camera => (3.0, 1.0, 3.0, ">3 Hz"),
    }
}

fn short(t: Topic) -> &'static str {
    match t {
        Topic::Scan => "scan",
        Topic::Odom => "odom",
        Topic::Tf => "tf",
        Topic::Map => "map",
        Topic::Camera => "camera",
    }
}

pub fn topic(t: Topic, r: &RateMeter, now: Instant) -> Check {
    let (ok, warn, stale, expect) = expected(t);
    let hz = r.hz(now);
    let (level, value) = match r.last() {
        None => (Level::Off, "no data".to_owned()),
        Some(last) if now.saturating_duration_since(last).as_secs_f64() > stale => {
            (Level::Bad, format!("silent {}", human_age(now.saturating_duration_since(last))))
        }
        _ if hz >= ok => (Level::Ok, format!("{hz:.1} Hz")),
        _ if hz >= warn => (Level::Warn, format!("{hz:.1} Hz")),
        _ => (Level::Bad, format!("{hz:.1} Hz")),
    };
    Check { name: short(t), value, expect, level }
}

fn ms_or_age(secs: f64) -> String {
    if secs.abs() < 10.0 { format!("{:.0} ms", secs * 1000.0) } else if secs.abs() < 60.0 { format!("{secs:.0} s") } else { "stale >1 min".into() }
}

/// how old the map -> base_link transform is against ROS now
pub fn tf_age(m: &Model, ros_now: f64) -> Check {
    let (value, level) = match m.tf_stamp {
        None => ("no data".into(), Level::Off),
        Some(s) => {
            let age = ros_now - s;
            let level = if age.abs() < 0.3 {
                Level::Ok
            } else if age.abs() < 1.0 {
                Level::Warn
            } else {
                Level::Bad
            };
            (ms_or_age(age), level)
        }
    };
    Check { name: "TF age", value, expect: "<300 ms", level }
}

/// ROS now minus the scan stamp when it arrived: transport delay plus clock skew between machines
pub fn scan_delay(m: &Model) -> Check {
    let (value, level) = match m.scan_delay {
        None => ("no data".into(), Level::Off),
        Some(d) => {
            let level = if d.abs() < 0.2 {
                Level::Ok
            } else if d.abs() < 1.0 {
                Level::Warn
            } else {
                Level::Bad
            };
            let v = ms_or_age(d);
            (if d > 0.0 && d.abs() < 10.0 { format!("+{v}") } else { v }, level)
        }
    };
    Check { name: "scan delay", value, expect: "<200 ms", level }
}

/// every check the panel shows, in order. When nothing comes at all the rows go grey: the
/// banner says it once instead of every row turning red.
pub fn all(m: &Model, now: Instant, ros_now: f64) -> Vec<Check> {
    let mut out: Vec<Check> = Topic::ALL.iter().map(|&t| topic(t, m.rate(t), now)).collect();
    out.push(tf_age(m, ros_now));
    out.push(scan_delay(m));
    if m.stale_for(now).is_some() {
        for c in &mut out {
            c.level = Level::Off;
            if c.name == "TF age" || c.name == "scan delay" {
                c.value = "-".into();
            }
        }
    }
    out
}

/// the header strip: the worst level and what to say about it
pub fn summary(m: &Model, now: Instant, ros_now: f64) -> (Level, String) {
    if let Some(d) = m.stale_for(now) {
        return (Level::Bad, format!("no data for {} - replay ended or robot off", human_age(d)));
    }
    if m.last_data().is_none() {
        return (Level::Off, "waiting for data".into());
    }
    let checks = all(m, now, ros_now);
    let worst = |l: Level| checks.iter().filter(|c| c.level == l).map(|c| format!("{} {}", c.name, c.value)).collect::<Vec<_>>();
    let bad = worst(Level::Bad);
    if !bad.is_empty() {
        return (Level::Bad, bad.join(", "));
    }
    let warn = worst(Level::Warn);
    if !warn.is_empty() {
        return (Level::Warn, warn.join(", "));
    }
    let scan = m.rate(Topic::Scan).hz(now);
    (Level::Ok, format!("all OK, scan {scan:.1} Hz"))
}

/// for tests and callers that build a stale model
pub const STALE: Duration = crate::model::STALE_AFTER;

#[cfg(test)]
mod tests {
    use super::*;

    fn ticked(hz: f64, secs: f64, t0: Instant) -> RateMeter {
        let mut r = RateMeter::default();
        let n = (hz * secs) as u64;
        for i in 0..=n {
            r.tick(t0 + Duration::from_secs_f64(i as f64 / hz));
        }
        r
    }

    #[test]
    fn rates_are_judged_against_the_expected_range() {
        let t0 = Instant::now();
        let end = t0 + Duration::from_secs(3);
        assert_eq!(topic(Topic::Scan, &RateMeter::default(), end).level, Level::Off);
        assert_eq!(topic(Topic::Scan, &ticked(7.0, 3.0, t0), end).level, Level::Ok);
        assert_eq!(topic(Topic::Scan, &ticked(3.0, 3.0, t0), end).level, Level::Warn);
        assert_eq!(topic(Topic::Map, &ticked(1.0, 3.0, t0), end).level, Level::Ok);
        let late = end + Duration::from_secs(2);
        let c = topic(Topic::Scan, &ticked(7.0, 3.0, t0), late);
        assert_eq!((c.level, c.value.as_str()), (Level::Bad, "silent 2.0 s"));
        assert_eq!(topic(Topic::Map, &ticked(1.0, 3.0, t0), late).level, Level::Ok);
    }

    #[test]
    fn tf_age_and_scan_delay_read_well() {
        let mut m = Model::default();
        assert_eq!(tf_age(&m, 10.0).level, Level::Off);
        m.tf_stamp = Some(9.9);
        let c = tf_age(&m, 10.0);
        assert_eq!((c.level, c.value.as_str()), (Level::Ok, "100 ms"));
        m.tf_stamp = Some(-82_115.0);
        let c = tf_age(&m, 10.0);
        assert_eq!((c.level, c.value.as_str()), (Level::Bad, "stale >1 min"));
        m.scan_delay = Some(-0.5);
        assert_eq!(scan_delay(&m).value, "-500 ms");
        m.scan_delay = Some(0.03);
        assert_eq!(scan_delay(&m).value, "+30 ms");
    }

    #[test]
    fn stale_data_gives_one_banner_and_grey_rows() {
        let t0 = Instant::now();
        let mut m = Model::default();
        assert_eq!(summary(&m, t0, 0.0).0, Level::Off);
        m.rates[Topic::Scan as usize] = ticked(7.0, 3.0, t0);
        m.rates[Topic::Odom as usize] = ticked(20.0, 3.0, t0);
        m.rates[Topic::Tf as usize] = ticked(20.0, 3.0, t0);
        m.rates[Topic::Map as usize] = ticked(1.0, 3.0, t0);
        let fresh = t0 + Duration::from_secs(3);
        assert_eq!(summary(&m, fresh, 0.0).0, Level::Ok, "{:?}", summary(&m, fresh, 0.0));
        let late = t0 + Duration::from_secs(85);
        let (level, text) = summary(&m, late, 0.0);
        assert_eq!(level, Level::Bad);
        assert_eq!(text, "no data for 1 min - replay ended or robot off");
        assert!(all(&m, late, 0.0).iter().all(|c| c.level == Level::Off));
        assert!(STALE < Duration::from_secs(85));
    }
}
