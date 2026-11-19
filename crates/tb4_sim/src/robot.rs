//! Round differential-drive base (Create 3): unicycle with acceleration limits, contact, bumper zones
//! and a rough bump reflex (short back-off that overrides the command, like REFLEX_BUMP).

use tb4_core::Pose2;

use crate::world::World;

#[derive(Debug, Clone)]
pub struct RobotParams {
    pub radius: f64,
    /// m/s^2, rad/s^2 the wheels follow a command with
    pub accel_v: f64,
    pub accel_w: f64,
    /// bump reflex: reverse at this speed (m/s) for `reflex_time` (s)
    pub reflex_speed: f64,
    pub reflex_time: f64,
    /// contact-free time (s) that ends a collision episode
    pub episode_gap: f64,
}

impl Default for RobotParams {
    fn default() -> Self {
        Self { radius: 0.171, accel_v: 1.0, accel_w: 4.0, reflex_speed: 0.1, reflex_time: 0.3, episode_gap: 1.0 }
    }
}

/// Create 3 bumper zones (frame ids of /hazard_detection)
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum BumpZone {
    FrontCenter,
    FrontLeft,
    FrontRight,
    Left,
    Right,
}

impl BumpZone {
    pub const ALL: [BumpZone; 5] = [Self::FrontCenter, Self::FrontLeft, Self::FrontRight, Self::Left, Self::Right];

    pub fn frame_id(self) -> &'static str {
        match self {
            Self::FrontCenter => "bump_front_center",
            Self::FrontLeft => "bump_front_left",
            Self::FrontRight => "bump_front_right",
            Self::Left => "bump_left",
            Self::Right => "bump_right",
        }
    }

    /// zone centre, rad from the heading (left positive)
    pub fn angle(self) -> f64 {
        match self {
            Self::FrontCenter => 0.0,
            Self::FrontLeft => 30f64.to_radians(),
            Self::FrontRight => -30f64.to_radians(),
            Self::Left => 60f64.to_radians(),
            Self::Right => -60f64.to_radians(),
        }
    }

    /// zone that a contact at `angle` (rad from the heading) presses; None behind the bumper (|angle| > 90 deg)
    pub fn from_angle(angle: f64) -> Option<Self> {
        let a = wrap(angle);
        if a.abs() > std::f64::consts::FRAC_PI_2 {
            return None;
        }
        Self::ALL.into_iter().min_by(|x, y| (x.angle() - a).abs().total_cmp(&(y.angle() - a).abs()))
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Contact {
    pub t: f64,
    /// None: contact behind the bumper
    pub zone: Option<BumpZone>,
    pub point: (f64, f64),
    /// first contact of a new collision episode
    pub new_episode: bool,
}

#[derive(Debug, Clone)]
pub struct Robot {
    pub p: RobotParams,
    /// ground truth pose in the world frame
    pub pose: Pose2,
    pub v: f64,
    pub w: f64,
    reflex_until: f64,
    last_contact: f64,
    pub collisions: u64,
    pub bumps: u64,
}

impl Robot {
    pub fn new(p: RobotParams, pose: Pose2) -> Self {
        Self { p, pose, v: 0.0, w: 0.0, reflex_until: f64::NEG_INFINITY, last_contact: f64::NEG_INFINITY, collisions: 0, bumps: 0 }
    }

    pub fn reflex_active(&self, now: f64) -> bool {
        now < self.reflex_until
    }

    /// true while the last contact is less than `episode_gap` ago
    pub fn in_contact(&self, now: f64) -> bool {
        now - self.last_contact < self.p.episode_gap
    }

    /// advance by `dt` following command (v, w); returns the contact if the base touched something
    pub fn step(&mut self, world: &World, cmd: (f64, f64), now: f64, dt: f64) -> Option<Contact> {
        let target = if self.reflex_active(now) { (-self.p.reflex_speed, 0.0) } else { cmd };
        self.v += (target.0 - self.v).clamp(-self.p.accel_v * dt, self.p.accel_v * dt);
        self.w += (target.1 - self.w).clamp(-self.p.accel_w * dt, self.p.accel_w * dt);
        let next = integrate(self.pose, self.v, self.w, dt);
        let r = self.p.radius;
        let Some(n) = world.nearest((next.x, next.y)).filter(|n| n.dist < r) else {
            self.pose = next;
            return None;
        };
        // moving away from an obstacle the base already touches is allowed
        let before = world.nearest((self.pose.x, self.pose.y)).map_or(f64::INFINITY, |b| b.dist);
        if n.dist > before {
            self.pose = next;
            return None;
        }
        // contact: the round base can still turn, it does not move into the obstacle
        self.pose.yaw = next.yaw;
        self.v = 0.0;
        let angle = (n.point.1 - self.pose.y).atan2(n.point.0 - self.pose.x) - self.pose.yaw;
        let zone = BumpZone::from_angle(angle);
        let new_episode = !self.in_contact(now);
        if new_episode {
            self.collisions += 1;
        }
        self.last_contact = now;
        if zone.is_some() && !self.reflex_active(now) {
            self.bumps += 1;
            self.reflex_until = now + self.p.reflex_time;
        }
        Some(Contact { t: now, zone, point: n.point, new_episode })
    }
}

/// exact unicycle step
pub fn integrate(p: Pose2, v: f64, w: f64, dt: f64) -> Pose2 {
    if w.abs() < 1e-9 {
        let (s, c) = p.yaw.sin_cos();
        return Pose2::new(p.x + v * dt * c, p.y + v * dt * s, p.yaw);
    }
    let yaw = p.yaw + w * dt;
    let r = v / w;
    Pose2::new(p.x + r * (yaw.sin() - p.yaw.sin()), p.y - r * (yaw.cos() - p.yaw.cos()), wrap(yaw))
}

pub fn wrap(a: f64) -> f64 {
    a.sin().atan2(a.cos())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn corridor() -> World {
        let mut w = World::new("t", [0.0, 0.0, 4.0, 2.0], Pose2::new(1.0, 1.0, 0.0));
        w.rect(0.0, 0.0, 4.0, 2.0);
        w
    }

    #[test]
    fn zones_follow_create3_layout() {
        assert_eq!(BumpZone::from_angle(0.1), Some(BumpZone::FrontCenter));
        assert_eq!(BumpZone::from_angle(0.45), Some(BumpZone::FrontLeft));
        assert_eq!(BumpZone::from_angle(-0.5), Some(BumpZone::FrontRight));
        assert_eq!(BumpZone::from_angle(1.2), Some(BumpZone::Left));
        assert_eq!(BumpZone::from_angle(-1.4), Some(BumpZone::Right));
        assert_eq!(BumpZone::from_angle(2.5), None);
        assert_eq!(BumpZone::FrontLeft.frame_id(), "bump_front_left");
    }

    #[test]
    fn drives_into_wall_bumps_front_and_backs_off() {
        let w = corridor();
        let mut r = Robot::new(RobotParams::default(), Pose2::new(3.5, 1.0, 0.0));
        let mut contact = None;
        let mut t = 0.0;
        while t < 5.0 && contact.is_none() {
            contact = r.step(&w, (0.18, 0.0), t, 0.01);
            t += 0.01;
        }
        let c = contact.expect("hits the wall at x = 4");
        assert_eq!(c.zone, Some(BumpZone::FrontCenter));
        assert!(c.new_episode);
        assert!(r.pose.x <= 4.0 - 0.171 + 1e-9);
        assert_eq!((r.collisions, r.bumps), (1, 1));
        // reflex: backs away although the command still pushes forward
        let x = r.pose.x;
        for _ in 0..20 {
            r.step(&w, (0.18, 0.0), t, 0.01);
            t += 0.01;
        }
        assert!(r.pose.x < x);
    }

    #[test]
    fn contact_on_the_left_is_left_zone() {
        let w = corridor();
        // heading 0.6 rad, wall at y = 2 meets the base about 1 rad left of the heading
        let mut r = Robot::new(RobotParams::default(), Pose2::new(2.0, 1.75, 0.6));
        let mut t = 0.0;
        let c = loop {
            if let Some(c) = r.step(&w, (0.1, 0.0), t, 0.01) {
                break c;
            }
            t += 0.01;
            assert!(t < 5.0);
        };
        assert_eq!(c.zone, Some(BumpZone::Left));
    }

    #[test]
    fn turning_in_contact_is_allowed() {
        let w = corridor();
        let mut r = Robot::new(RobotParams::default(), Pose2::new(1.0, 0.1715, 0.0));
        for k in 0..100 {
            r.step(&w, (0.0, 0.5), k as f64 * 0.01, 0.01);
        }
        assert!(r.pose.yaw > 0.3);
        assert_eq!(r.collisions, 0);
    }
}
