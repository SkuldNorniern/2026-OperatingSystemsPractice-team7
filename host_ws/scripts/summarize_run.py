#!/usr/bin/env python3
"""After sourcing ROS, summarize a run directory or one finalized rosbag."""
import argparse
from collections import Counter, defaultdict
import json
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def summarize(bag):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    counts, ages = Counter(), defaultdict(list)
    classes, ids, states = {}, set(), Counter()
    goal_results = {}
    forward, turning = [], []
    first, last = None, None
    errors = Counter()
    while reader.has_next():
        topic, data, at = reader.read_next()
        counts[topic] += 1
        first = at if first is None else min(first, at)
        last = at if last is None else max(last, at)
        if topic not in ('/scan', '/odom', '/oakd/rgb/preview/image_raw/compressed',
                         '/aruco_detections', '/cmd_vel', '/team7/mission_state',
                         '/navigate_to_pose/_action/status'):
            continue
        try:
            if topic not in classes:
                classes[topic] = get_message(types[topic])
            msg = deserialize_message(data, classes[topic])
            if hasattr(msg, 'header'):
                stamp = msg.header.stamp
                seconds = stamp.sec + stamp.nanosec * 1e-9
                if seconds > 0:
                    ages[topic].append(at * 1e-9 - seconds)
            if topic == '/aruco_detections':
                ids.update(m.marker_id for m in msg.markers)
            elif topic == '/cmd_vel':
                twist = msg.twist if hasattr(msg, 'twist') else msg
                forward.append(twist.linear.x)
                turning.append(abs(twist.angular.z))
            elif topic == '/team7/mission_state':
                states[msg.data] += 1
            elif topic == '/navigate_to_pose/_action/status':
                for status in msg.status_list:
                    goal_results[bytes(status.goal_info.goal_id.uuid).hex()] = status.status
        except (ImportError, AttributeError, RuntimeError, ValueError) as error:
            errors[str(error)] += 1
    def stats(values):
        values = sorted(values)
        return {'median': round(values[len(values)//2], 4),
                'p95': round(values[min(len(values)-1, int(len(values)*0.95))], 4)}
    return {'bag': str(bag), 'duration_s': round((last-first)*1e-9, 2) if first else 0,
            'message_counts': dict(sorted(counts.items())),
            'sensor_age_s': {t: stats(v) for t, v in ages.items() if v},
            'detected_ids': sorted(ids), 'mission_states': dict(states),
            'navigation_goal_states': dict(Counter(
                {0: 'unknown', 1: 'accepted', 2: 'executing', 3: 'canceling',
                 4: 'succeeded', 5: 'canceled', 6: 'aborted'}.get(state, str(state))
                for state in goal_results.values())),
            'cmd_vel': {'samples': len(forward), 'reverse_samples': sum(v < -1e-6 for v in forward),
                        'max_forward': max(forward, default=0), 'max_abs_turn': max(turning, default=0)},
            'decode_errors': dict(errors)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run_dir', type=Path)
    args = parser.parse_args()
    directory = args.run_dir.expanduser().resolve()
    bags = [directory] if (directory / 'metadata.yaml').exists() else sorted(
        p.parent for p in directory.glob('rosbag_*/metadata.yaml'))
    if not bags:
        parser.error('no finalized bags found; stop the recorder cleanly first')
    results = [summarize(bag) for bag in bags]
    report = directory / 'summary.json'
    report.write_text(json.dumps(results, indent=2) + '\n')
    print(report)
    for result in results:
        print(json.dumps({k: v for k, v in result.items() if k != 'message_counts'}, indent=2))


if __name__ == '__main__':
    main()
