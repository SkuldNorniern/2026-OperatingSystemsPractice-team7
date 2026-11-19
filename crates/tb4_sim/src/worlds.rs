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

/// 10 x 7 m studio laid out like the sketch on page 5 of OSP_final_project.pdf (sketch 774 x 322 pt scaled
/// to 10 x 7 m): two rows of two tall boxes in the middle, targets in the upper-left and upper-right corners
/// and one low on the east face of the lower-right box, robot in the lower left. Added (not in the sketch):
/// a cluster of four chairs (legs only), a desk (legs only) and three decoy labels (`--set decoys=0` drops them).
pub fn studio() -> World {
    let mut w = studio_base("studio");
    w.wall((0.0, 0.0), (10.0, 0.0));
    w
}

/// studio with a 3 m glass section in the south wall and a corridor wall 2 m behind it that the lidar sees
pub fn studio_glass() -> World {
    let mut w = studio_base("studio_glass");
    w.bounds = [-0.5, -2.5, 10.5, 7.5];
    w.wall((0.0, 0.0), (6.5, 0.0)).glass((6.5, 0.0), (9.5, 0.0)).wall((9.5, 0.0), (10.0, 0.0));
    w.wall((6.0, -2.0), (10.0, -2.0)).wall((6.0, -2.0), (6.0, 0.0)).wall((10.0, -2.0), (10.0, 0.0));
    w
}

/// room walls except the south one, boxes, furniture, markers
fn studio_base(name: &str) -> World {
    let mut w = World::new(name, [-0.5, -0.5, 10.5, 7.5], Pose2::new(2.0, 2.0, 0.0));
    w.wall((10.0, 0.0), (10.0, 7.0)).wall((10.0, 7.0), (0.0, 7.0)).wall((0.0, 7.0), (0.0, 0.0));
    // boxes 0.55 x 1.8 m, faces reach the floor; upper row 0.5 m from the north wall
    for (x, y) in [(3.5, 4.7), (4.9, 4.7), (3.8, 0.8), (5.35, 0.8)] {
        w.rect(x, y, x + 0.55, y + 1.8);
    }
    // four chairs side by side, 0.42 m between leg centres, 3 cm legs
    for (x, y) in [(7.8, 2.4), (8.5, 2.4), (7.8, 3.1), (8.5, 3.1)] {
        w.legs(x, y, 0.42, 0.42, 0.015);
    }
    // desk, 1.4 x 0.7 m top, 5 cm legs; the robot fits underneath
    w.legs(1.6, 4.0, 1.3, 0.6, 0.025);
    // targets: west wall upper left, east wall upper right, low on the east face of the lower-right box
    w.marker(7, (0.0, 5.6), 0.35, (1.0, 0.0), 0.15);
    w.marker(23, (10.0, 5.7), 0.35, (-1.0, 0.0), 0.12);
    w.marker(42, (5.9, 1.2), 0.10, (1.0, 0.0), 0.10);
    // decoys, with ids the field detector returned for chair barcode labels: a label on the back of a
    // chair pushed against the north wall (the wall is 0.66 m behind it, like field run 184838), one
    // on a chair of the cluster, and a sticker on the desk's east apron. Nothing at lidar height
    // behind any of them; the camera ray runs on past the legs
    w.legs(6.0, 6.55, 0.42, 0.42, 0.015);
    w.decoy(1023, (6.0, 6.34), 0.45, (0.0, -1.0), 0.05);
    w.decoy(0, (7.59, 3.1), 0.45, (-1.0, 0.0), 0.04);
    w.decoy(703, (2.25, 4.0), 0.6, (1.0, 0.0), 0.06);
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
        // decoys hang in free space: no solid face within 10 cm
        for d in &studio().decoys {
            assert!(studio().nearest(d.pos).unwrap().dist > 0.1, "decoy {}", d.id);
        }
        assert!(studio_glass().segments.iter().any(|s| s.kind == crate::world::SegmentKind::Glass));
    }
}
