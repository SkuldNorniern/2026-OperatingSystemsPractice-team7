//! Health checks for the panel: topic rates against what the robot normally gives, TF age and the
//! scan delay. Pure functions of the model, so they are tested without ROS.

use std::time::Instant;

use crate::model::{Model, RateMeter, Topic};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Level {
    /// within the expected range
    Ok,
    /// low or late
    Warn,
    /// stopped or far off
    Bad,
    /// never seen
    Off,
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

pub fn topic(t: Topic, r: &RateMeter, now: Instant) -> Check {
    let (ok, warn, stale, expect) = expected(t);
    let hz = r.hz(now);
    let level = match r.last() {
        None => Level::Off,
        Some(last) if now.saturating_duration_since(last).as_secs_f64() > stale => Level::Bad,
        _ if hz >= ok => Level::Ok,
        _ if hz >= warn => Level::Warn,
        _ => Level::Bad,
    };
    let value = match level {
        Level::Off => "no data".into(),
        Level::Bad if hz == 0.0 => "stopped".into(),
        _ => format!("{hz:.1} Hz"),
    };
    let name = match t {
        Topic::Scan => "scan",
        Topic::Odom => "odom",
        Topic::Tf => "tf",
        Topic::Map => "map",
        Topic::Camera => "camera",
    };
    Check { name, value, expect, level }
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
            (format!("{:.0} ms", age * 1000.0), level)
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
            (format!("{:+.0} ms", d * 1000.0), level)
        }
    };
    Check { name: "scan delay", value, expect: "<200 ms", level }
}

/// every check the panel shows, in order
pub fn all(m: &Model, now: Instant, ros_now: f64) -> Vec<Check> {
    let mut out: Vec<Check> = Topic::ALL.iter().map(|&t| topic(t, m.rate(t), now)).collect();
    out.push(tf_age(m, ros_now));
    out.push(scan_delay(m));
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Duration;

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
        // stopped 2 s ago: stale for the scan, still fine for the 1 Hz map
        let late = end + Duration::from_secs(2);
        let c = topic(Topic::Scan, &ticked(7.0, 3.0, t0), late);
        assert_eq!(c.level, Level::Bad);
        assert_eq!(topic(Topic::Map, &ticked(1.0, 3.0, t0), late).level, Level::Ok);
    }

    #[test]
    fn tf_age_and_scan_delay_levels() {
        let mut m = Model::default();
        assert_eq!(tf_age(&m, 10.0).level, Level::Off);
        m.tf_stamp = Some(9.9);
        let c = tf_age(&m, 10.0);
        assert_eq!((c.level, c.value.as_str()), (Level::Ok, "100 ms"));
        m.tf_stamp = Some(8.0);
        assert_eq!(tf_age(&m, 10.0).level, Level::Bad);
        m.scan_delay = Some(-0.5);
        let c = scan_delay(&m);
        assert_eq!((c.level, c.value.as_str()), (Level::Warn, "-500 ms"));
        assert_eq!(all(&m, Instant::now(), 10.0).len(), 7);
    }
}
