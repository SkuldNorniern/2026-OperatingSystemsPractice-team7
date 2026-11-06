use std::collections::{HashMap, VecDeque};
use std::fmt;

use nalgebra::{Isometry3, Translation3, UnitQuaternion};

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub struct Stamp(pub i64);

impl Stamp {
    pub const LATEST: Stamp = Stamp(0);

    pub fn from_ros(sec: i32, nanosec: u32) -> Self {
        Stamp(sec as i64 * 1_000_000_000 + nanosec as i64)
    }

    pub fn secs_f64(self) -> f64 {
        self.0 as f64 * 1e-9
    }
}

#[derive(Debug, Clone, PartialEq)]
pub enum TfError {
    UnknownFrame(String),
    NotConnected { target: String, source: String },
    Extrapolation { child: String, requested: Stamp, oldest: Stamp, newest: Stamp },
}

impl fmt::Display for TfError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            TfError::UnknownFrame(fr) => write!(f, "unknown frame '{fr}'"),
            TfError::NotConnected { target, source } => {
                write!(f, "frames '{target}' and '{source}' are not in the same tree")
            }
            TfError::Extrapolation { child, requested, oldest, newest } => write!(
                f,
                "extrapolation on edge to '{child}': requested {:.3}s, buffer [{:.3}, {:.3}]",
                requested.secs_f64(),
                oldest.secs_f64(),
                newest.secs_f64()
            ),
        }
    }
}

impl std::error::Error for TfError {}

#[derive(Debug)]
struct Edge {
    parent: String,
    is_static: bool,
    samples: VecDeque<(Stamp, Isometry3<f64>)>,
}

#[derive(Debug)]
pub struct TfBuffer {
    edges: HashMap<String, Edge>,
    pub cache_ns: i64,
    pub tolerance_ns: i64,
}

impl Default for TfBuffer {
    fn default() -> Self {
        Self::new(10.0, 0.1)
    }
}

fn strip(frame: &str) -> &str {
    frame.strip_prefix('/').unwrap_or(frame)
}

impl TfBuffer {
    pub fn new(cache_secs: f64, tolerance_secs: f64) -> Self {
        Self {
            edges: HashMap::new(),
            cache_ns: (cache_secs * 1e9) as i64,
            tolerance_ns: (tolerance_secs * 1e9) as i64,
        }
    }

    pub fn insert(&mut self, parent: &str, child: &str, stamp: Stamp, tf: Isometry3<f64>, is_static: bool) {
        let parent = strip(parent).to_owned();
        let child = strip(child).to_owned();
        let edge = self.edges.entry(child).or_insert_with(|| Edge {
            parent: parent.clone(),
            is_static,
            samples: VecDeque::new(),
        });
        if edge.parent != parent || edge.is_static != is_static {
            edge.parent = parent;
            edge.is_static = is_static;
            edge.samples.clear();
        }
        if is_static {
            edge.samples.clear();
            edge.samples.push_back((stamp, tf));
            return;
        }
        let pos = edge.samples.iter().rposition(|(s, _)| *s <= stamp).map_or(0, |p| p + 1);
        if pos > 0 && edge.samples[pos - 1].0 == stamp {
            edge.samples[pos - 1].1 = tf;
        } else {
            edge.samples.insert(pos, (stamp, tf));
        }
        if let Some(&(newest, _)) = edge.samples.back() {
            while edge.samples.front().is_some_and(|(s, _)| newest.0 - s.0 > self.cache_ns) {
                edge.samples.pop_front();
            }
        }
    }

    pub fn has_frame(&self, frame: &str) -> bool {
        let f = strip(frame);
        self.edges.contains_key(f) || self.edges.values().any(|e| e.parent == f)
    }

    fn edge_at(&self, child: &str, edge: &Edge, stamp: Stamp) -> Result<Isometry3<f64>, TfError> {
        let s = &edge.samples;
        let (oldest, newest) = (s.front().unwrap().0, s.back().unwrap().0);
        if edge.is_static || stamp == Stamp::LATEST {
            return Ok(s.back().unwrap().1);
        }
        let extrap = || TfError::Extrapolation { child: child.to_owned(), requested: stamp, oldest, newest };
        if stamp <= oldest {
            return if oldest.0 - stamp.0 <= self.tolerance_ns { Ok(s.front().unwrap().1) } else { Err(extrap()) };
        }
        if stamp >= newest {
            return if stamp.0 - newest.0 <= self.tolerance_ns { Ok(s.back().unwrap().1) } else { Err(extrap()) };
        }
        let hi = s.partition_point(|(t, _)| *t < stamp);
        let (t0, a) = s[hi - 1];
        let (t1, b) = s[hi];
        let alpha = (stamp.0 - t0.0) as f64 / (t1.0 - t0.0) as f64;
        Ok(interpolate(&a, &b, alpha))
    }

    fn chain<'a>(&'a self, frame: &'a str) -> Vec<(&'a str, &'a Edge)> {
        let mut out = Vec::new();
        let mut cur = frame;
        while let Some(edge) = self.edges.get(cur) {
            out.push((cur, edge));
            cur = &edge.parent;
            if out.len() > 128 {
                break;
            }
        }
        out
    }

    pub fn lookup(&self, target: &str, source: &str, stamp: Stamp) -> Result<Isometry3<f64>, TfError> {
        let (target, source) = (strip(target), strip(source));
        if target == source {
            return Ok(Isometry3::identity());
        }
        for f in [target, source] {
            if !self.has_frame(f) {
                return Err(TfError::UnknownFrame(f.to_owned()));
            }
        }
        let tc = self.chain(target);
        let sc = self.chain(source);
        let root_of = |c: &[(&str, &Edge)], f: &'_ str| c.last().map_or(f.to_owned(), |(_, e)| e.parent.clone());
        if root_of(&tc, target) != root_of(&sc, source) {
            return Err(TfError::NotConnected { target: target.into(), source: source.into() });
        }
        let on_target = |f: &str| f == target || tc.iter().any(|(_, e)| e.parent == f);
        let mut lca = source.to_owned();
        let mut source_up = Vec::new();
        if !on_target(source) {
            for (child, edge) in &sc {
                source_up.push((*child, *edge));
                if on_target(&edge.parent) {
                    lca = edge.parent.clone();
                    break;
                }
            }
        }
        let mut lca_t_source = Isometry3::identity();
        for (child, edge) in source_up.iter().rev() {
            lca_t_source *= self.edge_at(child, edge, stamp)?;
        }
        let mut lca_t_target = Isometry3::identity();
        for (child, edge) in tc.iter().take_while(|(c, _)| *c != lca).collect::<Vec<_>>().iter().rev() {
            lca_t_target *= self.edge_at(child, edge, stamp)?;
        }
        Ok(lca_t_target.inverse() * lca_t_source)
    }
}

fn interpolate(a: &Isometry3<f64>, b: &Isometry3<f64>, alpha: f64) -> Isometry3<f64> {
    let t = a.translation.vector.lerp(&b.translation.vector, alpha);
    let r: UnitQuaternion<f64> = a.rotation.slerp(&b.rotation, alpha);
    Isometry3::from_parts(Translation3::from(t), r)
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::{Point3, Vector3};

    fn iso(x: f64, y: f64, z: f64, yaw: f64) -> Isometry3<f64> {
        Isometry3::new(Vector3::new(x, y, z), Vector3::new(0.0, 0.0, yaw))
    }

    fn sec(s: f64) -> Stamp {
        Stamp((s * 1e9) as i64)
    }

    fn tree() -> TfBuffer {
        let mut b = TfBuffer::new(10.0, 0.05);
        b.insert("map", "odom", sec(1.0), iso(1.0, 0.0, 0.0, 0.0), false);
        b.insert("map", "odom", sec(2.0), iso(1.0, 0.0, 0.0, 0.0), false);
        b.insert("odom", "base_link", sec(1.0), iso(0.0, 0.0, 0.0, 0.0), false);
        b.insert("odom", "base_link", sec(2.0), iso(2.0, 0.0, 0.0, std::f64::consts::FRAC_PI_2), false);
        b.insert("base_link", "camera", Stamp(0), iso(0.1, 0.0, 0.2, 0.0), true);
        b.insert("/base_link", "lidar", Stamp(0), iso(-0.1, 0.0, 0.1, 0.0), true);
        b
    }

    #[test]
    fn chain_lookup_and_interpolation() {
        let b = tree();
        let t = b.lookup("map", "camera", sec(1.5)).unwrap();
        let p = t * Point3::origin();
        let c = std::f64::consts::FRAC_PI_4.cos() * 0.1;
        assert!((p.x - (2.0 + c)).abs() < 1e-9, "{p:?}");
        assert!((p.y - c).abs() < 1e-9 && (p.z - 0.2).abs() < 1e-9);
    }

    #[test]
    fn inverse_and_sibling_lookup() {
        let b = tree();
        let a = b.lookup("camera", "map", sec(2.0)).unwrap();
        let c = b.lookup("map", "camera", sec(2.0)).unwrap();
        assert!(((a * c).translation.vector.norm()) < 1e-9);
        let s = b.lookup("lidar", "camera", sec(2.0)).unwrap();
        assert!((s.translation.x - 0.2).abs() < 1e-9 && (s.translation.z - 0.1).abs() < 1e-9);
        let m = b.lookup("odom", "camera", sec(2.0)).unwrap();
        assert!((m.translation.x - 2.0).abs() < 1e-9 && (m.translation.y - 0.1).abs() < 1e-9);
    }

    #[test]
    fn extrapolation_and_latest() {
        let b = tree();
        assert!(matches!(b.lookup("map", "camera", sec(3.0)), Err(TfError::Extrapolation { .. })));
        assert!(b.lookup("map", "camera", sec(2.04)).is_ok());
        assert!(b.lookup("map", "camera", Stamp::LATEST).is_ok());
    }

    #[test]
    fn errors() {
        let mut b = tree();
        assert!(matches!(b.lookup("map", "nope", sec(1.0)), Err(TfError::UnknownFrame(_))));
        b.insert("world", "other", Stamp(0), Isometry3::identity(), true);
        assert!(matches!(b.lookup("map", "other", sec(1.0)), Err(TfError::NotConnected { .. })));
    }

    #[test]
    fn history_is_trimmed() {
        let mut b = TfBuffer::new(1.0, 0.0);
        for i in 0..100 {
            b.insert("odom", "base_link", sec(i as f64 * 0.1), Isometry3::identity(), false);
        }
        assert!(b.lookup("odom", "base_link", sec(5.0)).is_err());
        assert!(b.lookup("odom", "base_link", sec(9.5)).is_ok());
    }
}
