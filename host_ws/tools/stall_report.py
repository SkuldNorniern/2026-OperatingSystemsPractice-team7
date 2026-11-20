#!/usr/bin/env python3
"""Find stalls in a recorded run: the command says move, odometry does not.

usage: stall_report.py BAG_DIR [--cmd /cmd_vel] [--min-cmd 0.05] [--window 2.0] [--min-move 0.05]

A stall is a stretch where |cmd v| >= min-cmd (or |cmd w| >= 0.2 rad/s) and the robot neither moves
min-move metres nor turns 0.15 rad over the next window seconds. Prints each stall with its start time, length, position
and the command, plus totals. Needs ROS 2 sourced (rosbag2_py, rclpy, nav_msgs, geometry_msgs).

Note: a stall while the robot is turning in place is shown with w, not v.
"""
import argparse
import math
import sys

import rosbag2_py
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message


def read(bag_dir, topics):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag_dir, storage_id="mcap"),
                rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    msgs = []
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        if topic in topics:
            msgs.append((t_ns * 1e-9, topic, data, types.get(topic, "")))
    return msgs


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("--cmd", default="/cmd_vel")
    ap.add_argument("--min-cmd", type=float, default=0.05)
    ap.add_argument("--window", type=float, default=2.0)
    ap.add_argument("--min-move", type=float, default=0.05)
    a = ap.parse_args()

    msgs = read(a.bag, {a.cmd, "/odom"})
    t0 = msgs[0][0] if msgs else 0.0
    cmds, odom = [], []
    for t, topic, data, typ in msgs:
        if topic == "/odom":
            m = deserialize_message(data, Odometry)
            p = m.pose.pose.position
            q = m.pose.pose.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            odom.append((t, p.x, p.y, yaw))
        else:
            m = deserialize_message(data, TwistStamped)
            cmds.append((t, m.twist.linear.x, m.twist.angular.z))
    if not cmds or not odom:
        sys.exit(f"no {a.cmd} or /odom messages in {a.bag}")

    def pos_at(t):
        # latest odom at or before t
        lo, hi = 0, len(odom) - 1
        if t < odom[0][0]:
            return odom[0][1:]
        def pose(i):
            return odom[i][1:]
        while lo < hi:
            mid = (lo + hi + 1) // 2
            if odom[mid][0] <= t:
                lo = mid
            else:
                hi = mid - 1
        return pose(lo)

    stalls = []
    in_stall = None
    for t, v, w in cmds:
        commanded = abs(v) >= a.min_cmd or abs(w) >= 0.2
        if not commanded:
            if in_stall:
                stalls.append(in_stall)
                in_stall = None
            continue
        x0, y0, h0 = pos_at(t)
        x1, y1, h1 = pos_at(t + a.window)
        moved = math.hypot(x1 - x0, y1 - y0)
        turned = abs(math.atan2(math.sin(h1 - h0), math.cos(h1 - h0)))
        # a turn in place moves nothing but the heading; count it as motion
        if moved < a.min_move and turned < 0.15:
            if in_stall is None:
                in_stall = {"t0": t, "t1": t, "x": x0, "y": y0, "v": v, "w": w, "n": 1}
            else:
                in_stall["t1"] = t
                in_stall["n"] += 1
        elif in_stall:
            stalls.append(in_stall)
            in_stall = None
    if in_stall:
        stalls.append(in_stall)

    # merge stalls closer than 1 s apart
    merged = []
    for s in stalls:
        if merged and s["t0"] - merged[-1]["t1"] < 1.0:
            merged[-1]["t1"] = s["t1"]
            merged[-1]["n"] += s["n"]
        else:
            merged.append(dict(s))

    print(f"{a.bag}")
    print(f"  stalls (>= {a.window:.0f} s, commanded but < {a.min_move} m moved and < 0.15 rad turned): {len(merged)}")
    total = 0.0
    for s in merged:
        dur = s["t1"] - s["t0"] + a.window
        total += dur
        kind = "turn" if abs(s["w"]) >= 0.2 and abs(s["v"]) < a.min_cmd else "drive"
        print(f"  t={s['t0'] - t0:7.1f}s  {dur:5.1f}s  {kind:5s} at x={s['x']:6.2f} y={s['y']:6.2f}"
              f"  cmd v={s['v']:+.2f} w={s['w']:+.2f}")
    print(f"  total stalled time: {total:.0f} s of {msgs[-1][0] - t0:.0f} s run")


if __name__ == "__main__":
    main()
