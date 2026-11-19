//! Built-in worlds. World frame: metres, x east, y north, origin at the room's south-west corner.

use tb4_core::Pose2;

use crate::world::World;

pub const NAMES: [&str; 3] = ["studio", "studio_glass", "small"];

pub fn by_name(name: &str) -> Option<World> {
    match name {
        "studio" => Some(studio()),
        "studio_glass" => Some(studio_glass()),
        "small" => Some(small()),
        _ => None,
    }
}

/// ~10 x 7 m studio after page 5 of OSP_final_project.pdf: four boxes in two rows, three ARUCO_ORIGINAL
/// targets (two on walls, one low on a box face), a cluster of four chairs (legs only), a desk (legs only).
/// Robot starts in the south-west corner facing east, like after undocking.
pub fn studio() -> World {
    let mut w = studio_base("studio");
    w.wall((0.0, 0.0), (10.0, 0.0));
    w
}

/// studio with a 3 m glass section in the south wall and a corridor wall 2 m behind it that the lidar sees
pub fn studio_glass() -> World {
    let mut w = studio_base("studio_glass");
    w.bounds = [-0.5, -2.5, 10.5, 7.5];
    w.wall((0.0, 0.0), (4.0, 0.0)).glass((4.0, 0.0), (7.0, 0.0)).wall((7.0, 0.0), (10.0, 0.0));
    w.wall((3.0, -2.0), (8.0, -2.0)).wall((3.0, -2.0), (3.0, 0.0)).wall((8.0, -2.0), (8.0, 0.0));
    w
}

/// room walls except the south one, boxes, furniture, markers
fn studio_base(name: &str) -> World {
    let mut w = World::new(name, [-0.5, -0.5, 10.5, 7.5], Pose2::new(1.0, 1.0, 0.0));
    w.wall((10.0, 0.0), (10.0, 7.0)).wall((10.0, 7.0), (0.0, 7.0)).wall((0.0, 7.0), (0.0, 0.0));
    // two rows of two boxes, 1.2 x 0.6 m, faces reach the floor
    for (x, y) in [(2.4, 2.0), (5.4, 2.0), (2.4, 4.4), (5.4, 4.4)] {
        w.rect(x, y, x + 1.2, y + 0.6);
    }
    // four chairs side by side, 0.42 m between leg centres, 3 cm legs
    for (x, y) in [(8.0, 1.0), (8.7, 1.0), (8.0, 1.7), (8.7, 1.7)] {
        w.legs(x, y, 0.42, 0.42, 0.015);
    }
    // desk, 1.4 x 0.7 m top, 5 cm legs; the robot fits underneath
    w.legs(1.2, 5.9, 1.3, 0.6, 0.025);
    // targets: north wall, east wall, low on the east face of the north-east box
    w.marker(7, (4.5, 7.0), 0.35, (0.0, -1.0), 0.15);
    w.marker(23, (10.0, 4.0), 0.35, (-1.0, 0.0), 0.12);
    w.marker(42, (6.6, 4.7), 0.10, (1.0, 0.0), 0.10);
    w
}

/// 4 x 3 m room, one box, one chair, one marker: for tests
pub fn small() -> World {
    let mut w = World::new("small", [-0.5, -0.5, 4.5, 3.5], Pose2::new(0.6, 0.6, 0.0));
    w.rect(0.0, 0.0, 4.0, 3.0);
    w.rect(2.2, 1.2, 2.8, 1.6);
    w.legs(1.0, 2.3, 0.42, 0.42, 0.015);
    w.marker(5, (4.0, 2.2), 0.3, (-1.0, 0.0), 0.12);
    w
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn worlds_start_free_and_markers_sit_on_faces() {
        for name in NAMES {
            let w = by_name(name).unwrap();
            assert!(w.disc_free((w.start.x, w.start.y), 0.3), "{name}");
            for m in &w.markers {
                let n = w.nearest(m.pos).unwrap();
                assert!(n.dist < 1e-9, "{name} marker {}", m.id);
                // the room side of the face is free
                assert!(w.disc_free((m.pos.0 + 0.3 * m.normal.0, m.pos.1 + 0.3 * m.normal.1), 0.2), "{name} {}", m.id);
            }
        }
        assert_eq!(studio().markers.len(), 3);
        assert!(studio_glass().segments.iter().any(|s| s.kind == crate::world::SegmentKind::Glass));
    }
}
