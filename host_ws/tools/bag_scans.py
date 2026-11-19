#!/usr/bin/env python3
"""Stream the lidar scans and odometry of a rosbag for tb4_slam's replay example.

usage: bag_scans.py BAG_DIR_OR_MCAP [MORE_MCAP ...] | cargo run --release -p tb4_slam --example replay
Several .mcap files of one run: pass each file, they are read one after another.

Events in recorded (arrival) order, so the replay sees tf the way the live node does.
Little endian, each event starts with a u8 tag:
  0 laser pose in base_link (from /tf_static, once, first): f64 x, y, yaw
  1 odom->base_link from /tf: f64 stamp (s), x, y, yaw
  2 /odom twist (scan_filter deskews with the latest one): f64 v, w
  3 /scan: f64 stamp (s), f32 angle_min, angle_increment, time_increment, u32 n, n * f32 ranges
"""
import math
import struct
import sys

import rosbag2_py
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage

ODOM, BASE = 'odom', 'base_link'


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def stamp(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


def compose(a, b):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], a[2] + b[2])


def laser_in_base(statics, frame):
    pose = (0.0, 0.0, 0.0)
    while frame != BASE:
        if frame not in statics:
            sys.exit(f'no static tf from {frame} to {BASE}')
        parent, p = statics[frame]
        pose = compose(p, pose)
        frame = parent
    return pose


class Out:
    """holds events back until the laser pose header is written"""

    def __init__(self):
        self.held, self.ready = [], False

    def write(self, b):
        if self.ready:
            sys.stdout.buffer.write(b)
        else:
            self.held.append(b)

    def header(self, b):
        sys.stdout.buffer.write(b + b''.join(self.held))
        self.held, self.ready = [], True


def main():
    out = Out()
    statics, header_sent, early = {}, False, 0
    counts = {'tf': 0, 'odom': 0, 'scan': 0}
    for uri in sys.argv[1:]:
        storage = 'mcap' if uri.endswith('.mcap') else ''
        r = rosbag2_py.SequentialReader()
        r.open(rosbag2_py.StorageOptions(uri=uri, storage_id=storage), rosbag2_py.ConverterOptions('', ''))
        r.set_filter(rosbag2_py.StorageFilter(topics=['/scan', '/tf', '/tf_static', '/odom']))
        while r.has_next():
            topic, data, _ = r.read_next()
            if topic == '/tf_static':
                for t in deserialize_message(data, TFMessage).transforms:
                    p = t.transform.translation
                    statics[t.child_frame_id.lstrip('/')] = (t.header.frame_id.lstrip('/'),
                                                             (p.x, p.y, yaw_of(t.transform.rotation)))
            elif topic == '/scan':
                m = deserialize_message(data, LaserScan)
                if not header_sent:
                    # the node cannot use scans before it knows the laser pose either
                    frame = m.header.frame_id.lstrip('/')
                    if frame not in statics:
                        early += 1
                        continue
                    out.header(struct.pack('<B3d', 0, *laser_in_base(statics, frame)))
                    header_sent = True
                out.write(struct.pack('<Bd3fI', 3, stamp(m.header), m.angle_min, m.angle_increment,
                                      m.time_increment, len(m.ranges)))
                out.write(struct.pack(f'<{len(m.ranges)}f', *m.ranges))
                counts['scan'] += 1
            elif topic == '/odom':
                m = deserialize_message(data, Odometry)
                out.write(struct.pack('<B2d', 2, m.twist.twist.linear.x, m.twist.twist.angular.z))
                counts['odom'] += 1
            else:
                for t in deserialize_message(data, TFMessage).transforms:
                    if t.header.frame_id.lstrip('/') == ODOM and t.child_frame_id.lstrip('/') == BASE:
                        p = t.transform.translation
                        out.write(struct.pack('<B4d', 1, stamp(t.header), p.x, p.y, yaw_of(t.transform.rotation)))
                        counts['tf'] += 1
    print(f'{counts}, {early} scans before /tf_static dropped', file=sys.stderr)


if __name__ == '__main__':
    main()
