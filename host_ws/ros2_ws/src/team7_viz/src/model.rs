//! What the viewer knows about the run, all in the map frame. Filled by the ROS thread (`ros`),
//! read by the window (`view`). No ROS types here, so it is tested and drawn without a ROS graph.

use std::collections::VecDeque;
use std::time::{Duration, Instant};

use tb4_core::GridMeta;

/// TurtleBot 4 footprint radius, metres
pub const ROBOT_RADIUS: f64 = 0.171;

/// message rate over a sliding window
#[derive(Debug, Clone, Default)]
pub struct RateMeter {
    stamps: VecDeque<Instant>,
    pub total: u64,
}

const RATE_WINDOW: Duration = Duration::from_secs(5);

impl RateMeter {
    pub fn tick(&mut self, now: Instant) {
        self.total += 1;
        self.stamps.push_back(now);
        self.prune(now);
    }

    fn prune(&mut self, now: Instant) {
        while self.stamps.front().is_some_and(|&t| now.saturating_duration_since(t) > RATE_WINDOW) {
            self.stamps.pop_front();
        }
    }

    /// messages per second over the last few seconds; 0 when the topic went quiet
    pub fn hz(&self, now: Instant) -> f64 {
        let recent: Vec<_> = self.stamps.iter().filter(|&&t| now.saturating_duration_since(t) <= RATE_WINDOW).collect();
        match (recent.first(), recent.last()) {
            (Some(&&a), Some(&&b)) if recent.len() >= 2 && b > a => {
                let n = (recent.len() - 1) as f64;
                let span = (b - a).as_secs_f64();
                // a topic that stopped (silent for over two periods) decays instead of freezing at its old rate
                let since = now.saturating_duration_since(b).as_secs_f64();
                if since > 2.0 * span / n { n / (span + since) } else { n / span }
            }
            _ => 0.0,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Topic {
    Scan,
    Map,
    Tf,
    Odom,
}

impl Topic {
    pub const ALL: [Topic; 4] = [Topic::Scan, Topic::Map, Topic::Tf, Topic::Odom];

    pub fn name(self) -> &'static str {
        match self {
            Topic::Scan => "/scan",
            Topic::Map => "/map",
            Topic::Tf => "/tf",
            Topic::Odom => "/odom",
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Target {
    pub id: i32,
    pub x: f64,
    pub y: f64,
    pub confirmed: bool,
}

#[derive(Debug, Clone, PartialEq)]
pub struct MapData {
    pub meta: GridMeta,
    pub data: Vec<i8>,
}

/// everything drawn, in the map frame
#[derive(Debug, Clone, Default)]
pub struct Model {
    pub map: Option<MapData>,
    /// bumped on every new map, so the window rebuilds its image only then
    pub map_seq: u64,
    /// map -> base_link (x, y, yaw)
    pub robot: Option<(f64, f64, f64)>,
    pub scan: Vec<(f64, f64)>,
    pub path: Vec<(f64, f64)>,
    pub trajectory: Vec<(f64, f64)>,
    pub frontiers: Vec<(f64, f64)>,
    pub views: Vec<(f64, f64)>,
    pub goal: Option<(f64, f64)>,
    pub targets: Vec<Target>,
    pub mission: Option<String>,
    pub rates: [RateMeter; 4],
    /// messages received per subscription, for the debug line
    pub counts: Vec<(&'static str, u64)>,
    /// the last frame that could not be put in the map frame (no TF yet), and when
    pub tf_problem: Option<(String, Instant)>,
}

impl Model {
    pub fn rate(&self, t: Topic) -> &RateMeter {
        &self.rates[t as usize]
    }

    pub fn tick(&mut self, t: Topic, now: Instant) {
        self.rates[t as usize].tick(now);
    }

    pub fn count(&mut self, name: &'static str) {
        match self.counts.iter_mut().find(|(n, _)| *n == name) {
            Some((_, c)) => *c += 1,
            None => self.counts.push((name, 1)),
        }
    }

    pub fn counts_line(&self) -> String {
        self.counts.iter().map(|(n, c)| format!("{n}={c}")).collect::<Vec<_>>().join(" ")
    }

    /// world box to fit the view to: the map, else around the robot, else 10 x 10 m at the origin
    pub fn bounds(&self) -> [f64; 4] {
        if let Some(m) = &self.map {
            let g = &m.meta;
            return [g.origin_x, g.origin_y, g.origin_x + f64::from(g.width) * g.resolution, g.origin_y + f64::from(g.height) * g.resolution];
        }
        let (x, y) = self.robot.map_or((0.0, 0.0), |r| (r.0, r.1));
        [x - 5.0, y - 5.0, x + 5.0, y + 5.0]
    }

    pub fn targets(&self, confirmed: bool) -> impl Iterator<Item = &Target> {
        self.targets.iter().filter(move |t| t.confirmed == confirmed)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rate_meter_measures_and_decays() {
        let t0 = Instant::now();
        let mut r = RateMeter::default();
        assert_eq!(r.hz(t0), 0.0);
        for i in 0..=20 {
            r.tick(t0 + Duration::from_millis(100 * i));
        }
        let end = t0 + Duration::from_secs(2);
        assert!((r.hz(end) - 10.0).abs() < 0.01, "{}", r.hz(end));
        // quiet for 2 s: the rate falls
        assert!(r.hz(end + Duration::from_secs(2)) < 6.0);
        // quiet for longer than the window: zero
        assert_eq!(r.hz(end + Duration::from_secs(10)), 0.0);
        assert_eq!(r.total, 21);
    }

    #[test]
    fn bounds_follow_map_then_robot() {
        let mut m = Model::default();
        assert_eq!(m.bounds(), [-5.0, -5.0, 5.0, 5.0]);
        m.robot = Some((1.0, 2.0, 0.0));
        assert_eq!(m.bounds(), [-4.0, -3.0, 6.0, 7.0]);
        m.map = Some(MapData {
            meta: GridMeta { width: 20, height: 10, resolution: 0.05, origin_x: -0.5, origin_y: 0.0 },
            data: vec![-1; 200],
        });
        assert_eq!(m.bounds(), [-0.5, 0.0, 0.5, 0.5]);
    }

    #[test]
    fn counts_accumulate_per_name() {
        let mut m = Model::default();
        m.count("scan");
        m.count("map");
        m.count("scan");
        assert_eq!(m.counts_line(), "scan=2 map=1");
    }
}
