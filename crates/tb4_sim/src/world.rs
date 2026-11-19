//! 2D world: wall and box-face segments, glass segments, round posts (chair and desk legs), markers.

use tb4_core::Pose2;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SegmentKind {
    /// seen by lidar and camera, solid
    Wall,
    /// invisible to lidar and camera, solid for the robot (bumper)
    Glass,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Segment {
    pub a: (f64, f64),
    pub b: (f64, f64),
    pub kind: SegmentKind,
}

/// round obstacle standing on the floor: chair leg, desk leg
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Post {
    pub c: (f64, f64),
    pub r: f64,
}

/// ArUco marker on a vertical face
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Marker {
    pub id: u32,
    /// centre on the face (m)
    pub pos: (f64, f64),
    /// height of the centre above the floor (m)
    pub z: f64,
    /// unit normal pointing away from the face, into the room
    pub normal: (f64, f64),
    /// black square side (m)
    pub size: f64,
}

#[derive(Debug, Clone, Default)]
pub struct World {
    pub name: String,
    pub segments: Vec<Segment>,
    pub posts: Vec<Post>,
    pub markers: Vec<Marker>,
    /// robot start pose (world frame); the map and odom frames start here
    pub start: Pose2,
    /// bounding box [x0, y0, x1, y1] for reference grids
    pub bounds: [f64; 4],
}

/// closest obstacle to a point
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Nearest {
    pub dist: f64,
    pub point: (f64, f64),
}

impl World {
    pub fn new(name: &str, bounds: [f64; 4], start: Pose2) -> Self {
        Self { name: name.into(), bounds, start, ..Self::default() }
    }

    pub fn wall(&mut self, a: (f64, f64), b: (f64, f64)) -> &mut Self {
        self.segments.push(Segment { a, b, kind: SegmentKind::Wall });
        self
    }

    pub fn glass(&mut self, a: (f64, f64), b: (f64, f64)) -> &mut Self {
        self.segments.push(Segment { a, b, kind: SegmentKind::Glass });
        self
    }

    /// closed axis-aligned rectangle of walls (a room outline or a box)
    pub fn rect(&mut self, x0: f64, y0: f64, x1: f64, y1: f64) -> &mut Self {
        self.wall((x0, y0), (x1, y0)).wall((x1, y0), (x1, y1)).wall((x1, y1), (x0, y1)).wall((x0, y1), (x0, y0))
    }

    pub fn post(&mut self, x: f64, y: f64, r: f64) -> &mut Self {
        self.posts.push(Post { c: (x, y), r });
        self
    }

    /// four legs of a chair or desk centred at (x, y), `w` x `d` between leg centres
    pub fn legs(&mut self, x: f64, y: f64, w: f64, d: f64, r: f64) -> &mut Self {
        for (sx, sy) in [(-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)] {
            self.post(x + sx * w / 2.0, y + sy * d / 2.0, r);
        }
        self
    }

    pub fn marker(&mut self, id: u32, pos: (f64, f64), z: f64, normal: (f64, f64), size: f64) -> &mut Self {
        let n = normal.0.hypot(normal.1);
        self.markers.push(Marker { id, pos, z, normal: (normal.0 / n, normal.1 / n), size });
        self
    }

    /// distance along the ray from `o` in unit direction `d` to the first obstacle within `max`;
    /// glass is skipped when `see_glass` is false (lidar, camera)
    pub fn raycast(&self, o: (f64, f64), d: (f64, f64), max: f64, see_glass: bool) -> Option<f64> {
        let mut best = max;
        let mut hit = false;
        for s in &self.segments {
            if s.kind == SegmentKind::Glass && !see_glass {
                continue;
            }
            if let Some(t) = ray_segment(o, d, s.a, s.b)
                && t < best
            {
                best = t;
                hit = true;
            }
        }
        for p in &self.posts {
            if let Some(t) = ray_circle(o, d, p.c, p.r)
                && t < best
            {
                best = t;
                hit = true;
            }
        }
        hit.then_some(best)
    }

    /// closest solid obstacle (glass included) to `p`
    pub fn nearest(&self, p: (f64, f64)) -> Option<Nearest> {
        let mut best: Option<Nearest> = None;
        let mut take = |dist: f64, point: (f64, f64)| {
            if best.is_none_or(|b| dist < b.dist) {
                best = Some(Nearest { dist, point });
            }
        };
        for s in &self.segments {
            let q = closest_on_segment(p, s.a, s.b);
            take((q.0 - p.0).hypot(q.1 - p.1), q);
        }
        for c in &self.posts {
            let (dx, dy) = (p.0 - c.c.0, p.1 - c.c.1);
            let d = dx.hypot(dy);
            let q = if d > 1e-12 { (c.c.0 + dx / d * c.r, c.c.1 + dy / d * c.r) } else { c.c };
            take((d - c.r).max(0.0), q);
        }
        best
    }

    /// true if a disc of radius `r` at `p` overlaps no solid obstacle
    pub fn disc_free(&self, p: (f64, f64), r: f64) -> bool {
        self.nearest(p).is_none_or(|n| n.dist >= r)
    }
}

/// ray parameter t >= 0 where o + t d meets segment ab
pub fn ray_segment(o: (f64, f64), d: (f64, f64), a: (f64, f64), b: (f64, f64)) -> Option<f64> {
    let e = (b.0 - a.0, b.1 - a.1);
    let den = d.0 * e.1 - d.1 * e.0;
    if den.abs() < 1e-12 {
        return None;
    }
    let w = (a.0 - o.0, a.1 - o.1);
    let t = (w.0 * e.1 - w.1 * e.0) / den;
    let u = (w.0 * d.1 - w.1 * d.0) / den;
    (t >= 0.0 && (0.0..=1.0).contains(&u)).then_some(t)
}

/// first ray parameter t >= 0 where o + t d (unit d) enters the circle
pub fn ray_circle(o: (f64, f64), d: (f64, f64), c: (f64, f64), r: f64) -> Option<f64> {
    let f = (o.0 - c.0, o.1 - c.1);
    let b = f.0 * d.0 + f.1 * d.1;
    let cc = f.0 * f.0 + f.1 * f.1 - r * r;
    let disc = b * b - cc;
    if disc < 0.0 {
        return None;
    }
    let s = disc.sqrt();
    let t0 = -b - s;
    if t0 >= 0.0 {
        Some(t0)
    } else if -b + s >= 0.0 {
        Some(0.0) // starts inside
    } else {
        None
    }
}

pub fn closest_on_segment(p: (f64, f64), a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    let e = (b.0 - a.0, b.1 - a.1);
    let l2 = e.0 * e.0 + e.1 * e.1;
    if l2 < 1e-18 {
        return a;
    }
    let t = (((p.0 - a.0) * e.0 + (p.1 - a.1) * e.1) / l2).clamp(0.0, 1.0);
    (a.0 + t * e.0, a.1 + t * e.1)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn square() -> World {
        let mut w = World::new("t", [0.0, 0.0, 4.0, 4.0], Pose2::new(1.0, 1.0, 0.0));
        w.rect(0.0, 0.0, 4.0, 4.0);
        w
    }

    #[test]
    fn ray_hits_wall_and_post() {
        let mut w = square();
        assert!((w.raycast((1.0, 1.0), (1.0, 0.0), 8.0, false).unwrap() - 3.0).abs() < 1e-9);
        w.post(2.0, 1.0, 0.1);
        assert!((w.raycast((1.0, 1.0), (1.0, 0.0), 8.0, false).unwrap() - 0.9).abs() < 1e-9);
        assert!(w.raycast((1.0, 1.0), (1.0, 0.0), 0.5, false).is_none());
    }

    #[test]
    fn glass_is_invisible_to_rays_but_solid() {
        let mut w = square();
        w.glass((2.0, 0.0), (2.0, 4.0));
        assert!((w.raycast((1.0, 1.0), (1.0, 0.0), 8.0, false).unwrap() - 3.0).abs() < 1e-9);
        assert!((w.raycast((1.0, 1.0), (1.0, 0.0), 8.0, true).unwrap() - 1.0).abs() < 1e-9);
        assert!(!w.disc_free((1.9, 1.0), 0.17));
        assert!(w.disc_free((1.5, 1.0), 0.17));
    }

    #[test]
    fn nearest_point_on_post_and_wall() {
        let mut w = square();
        w.post(2.0, 2.0, 0.05);
        let n = w.nearest((2.2, 2.0)).unwrap();
        assert!((n.dist - 0.15).abs() < 1e-9);
        assert!((n.point.0 - 2.05).abs() < 1e-9);
        let n = w.nearest((0.1, 2.0)).unwrap();
        assert!((n.dist - 0.1).abs() < 1e-9 && n.point == (0.0, 2.0));
    }
}
