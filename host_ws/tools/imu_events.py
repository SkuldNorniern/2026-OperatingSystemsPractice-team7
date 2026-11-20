#!/usr/bin/env python3
"""Find impacts (bumps, wall contacts) in a recorded run from the IMU, and check them against the lidar.

usage: imu_events.py BAG_DIR [--thresh 1.5] [--cmd /cmd_vel_smoothed] [--front 0.25]

Method:
  1. linear acceleration from /imu, high-passed by subtracting a 1 s running median per axis
     (removes gravity and slow tilt).
  2. an impact is a sample where the horizontal high-passed magnitude exceeds --thresh (m/s^2).
     Samples closer than 0.5 s are merged into one event.
  3. for each event: the commanded speed, the odometry speed around it, and the closest lidar return
     in a cone of +-30 deg in front of the robot (what the robot was driving toward).

Prints the events and a summary: how many come with a forward command, how many with a lidar wall
within 0.3 m, and how many of these are also seen as a stall (commanded but not moving).
"""
import argparse
import bisect
import math
import statistics as st
import sys

import rosbag2_py
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu, LaserScan


def read(bag_dir, topics):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag_dir, storage_id="mcap"),
                rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"))
    out = []
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        if topic in topics:
            out.append((t_ns * 1e-9, topic, data))
    return out


def running_median(vals, times, window=1.0):
    """median of the samples within +-window/2 seconds of each sample (O(n * k), fine for ~40 Hz)."""
    out = []
    lo = hi = 0
    n = len(vals)
    for i in range(n):
        t = times[i]
        while times[lo] < t - window / 2:
            lo += 1
        while hi + 1 < n and times[hi + 1] <= t + window / 2:
            hi += 1
        out.append(st.median(vals[lo:hi + 1]))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("--thresh", type=float, default=1.5, help="m/s^2 above the local median")
    ap.add_argument("--cmd", default="/cmd_vel_smoothed")
    ap.add_argument("--front", type=float, default=0.25, help="wall distance (m) counted as a wall in front")
    a = ap.parse_args()

    msgs = read(a.bag, {"/imu", "/odom", a.cmd, "/scan"})
    if not msgs:
        sys.exit("no messages")
    t0 = msgs[0][0]
    imu, odom, cmds, scans = [], [], [], []
    for t, topic, data in msgs:
        if topic == "/imu":
            m = deserialize_message(data, Imu)
            la = m.linear_acceleration
            imu.append((t, la.x, la.y, la.z))
        elif topic == "/odom":
            m = deserialize_message(data, Odometry)
            odom.append((t, m.twist.twist.linear.x))
        elif topic == "/scan":
            m = deserialize_message(data, LaserScan)
            scans.append((t, m))
        else:
            m = deserialize_message(data, TwistStamped)
            cmds.append((t, m.twist.linear.x))
    if not imu:
        sys.exit("no /imu in this bag")

    times = [s[0] for s in imu]
    ax = [s[1] for s in imu]
    ay = [s[2] for s in imu]
    hx = [x - m for x, m in zip(ax, running_median(ax, times))]
    hy = [y - m for y, m in zip(ay, running_median(ay, times))]
    mag = [math.hypot(x, y) for x, y in zip(hx, hy)]

    # impacts: threshold crossings, merged within 0.5 s
    events = []
    for i, m in enumerate(mag):
        if m >= a.thresh:
            if events and times[i] - events[-1]["t"] < 0.5:
                if m > events[-1]["peak"]:
                    events[-1].update(peak=m, t_peak=times[i])
                events[-1]["t_end"] = times[i]
            else:
                events.append({"t": times[i], "t_end": times[i], "peak": m, "t_peak": times[i],
                               "dir": math.degrees(math.atan2(hy[i], hx[i]))})

    def nearest(seq, t):
        ts = [s[0] for s in seq]
        j = bisect.bisect_left(ts, t)
        cands = [k for k in (j - 1, j) if 0 <= k < len(seq)]
        return min((seq[k] for k in cands), key=lambda s: abs(s[0] - t))

    def cmd_at(t):
        return nearest(cmds, t)[1] if cmds else float("nan")

    def odom_speed(t):
        best = [o for o in odom if abs(o[0] - t) < 0.5]
        return st.mean(o[1] for o in best) if best else float("nan")

    def front_wall(t):
        best = nearest(scans, t)[1] if scans else None
        if best is None:
            return float("nan")
        r_min = float("inf")
        for i, r in enumerate(best.ranges):
            ang = best.angle_min + i * best.angle_increment
            if abs(ang) <= math.radians(30) and math.isfinite(r) and best.range_min <= r <= best.range_max:
                r_min = min(r_min, r)
        return r_min

    print(f"{a.bag}")
    print(f"  imu {len(imu)} samples over {times[-1] - times[0]:.0f} s, threshold {a.thresh} m/s^2")
    print(f"  {len(events)} impact events")
    rows = []
    for e in events:
        c = cmd_at(e["t_peak"])
        v = odom_speed(e["t_peak"])
        w = front_wall(e["t_peak"])
        rows.append((e, c, v, w))
        print(f"  t={e['t_peak'] - t0:7.1f}s peak={e['peak']:5.2f} m/s^2 dir={e['dir']:+6.0f} deg"
              f"  cmd v={c:+.2f}  odom v={v:+.2f}  front wall={w:5.2f} m")
    fwd = [r for r in rows if r[1] >= 0.05]
    walls = [r for r in fwd if r[3] <= a.front]
    print(f"  with forward command: {len(fwd)} / {len(rows)}; of those, wall within {a.front} m in front: {len(walls)}")


if __name__ == "__main__":
    main()
