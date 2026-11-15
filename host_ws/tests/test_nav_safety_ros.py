"""Isolated ROS smoke test; requires a built nav_safety executable and rclpy."""

import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--executable', type=Path)
    args = parser.parse_args()
    executable = args.executable
    if executable is None:
        from ament_index_python.packages import get_package_prefix
        executable = Path(get_package_prefix('team7_nodes')) / 'lib/team7_nodes/nav_safety'
    executable = executable.resolve(strict=True)

    # Keep generated sensor/control traffic away from the live robot.
    os.environ['ROS_DOMAIN_ID'] = '177'
    os.environ['ROS_LOCALHOST_ONLY'] = '1'

    import rclpy
    from geometry_msgs.msg import TransformStamped, TwistStamped
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import Bool, String
    from tf2_msgs.msg import TFMessage
    from rclpy.qos import DurabilityPolicy, QoSProfile

    with tempfile.TemporaryDirectory(prefix='team7-nav-safety-test-') as temp:
        os.environ['ROS_LOG_DIR'] = temp
        rclpy.init()
        node = rclpy.create_node('nav_safety_test')
        pubs = {name: node.create_publisher(kind, topic, 10) for name, kind, topic in [
            ('active', Bool, '/team7/nav/active'),
            ('intent', TwistStamped, '/team7/nav/intent'),
            ('odom', Odometry, '/odom'),
            ('scan', LaserScan, '/scan'),
        ]}
        reports = []
        subscription = node.create_subscription(
            String, '/team7/nav/safety', lambda msg: reports.append(json.loads(msg.data)), 10)
        tf_publisher = node.create_publisher(
            TFMessage, '/tf_static', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        transform = TransformStamped()
        transform.header.frame_id = 'base_link'
        transform.child_frame_id = 'nav_safety_test_lidar'
        transform.transform.translation.x = -0.2
        transform.transform.translation.y = 0.02
        transform.transform.rotation.z = math.sin(math.pi / 4)
        transform.transform.rotation.w = math.cos(math.pi / 4)
        # Publish before starting the monitor: its late subscription must receive
        # this retained static TF, then apply both rotation and translation.
        tf_publisher.publish(TFMessage(transforms=[transform]))
        with open(Path(temp) / 'node.log', 'w+') as log:
            process = subprocess.Popen([
                str(executable), '--ros-args', '-p', 'progress_timeout:=0.7',
                '-p', 'blocked_timeout:=0.4', '-p', 'input_timeout:=0.4',
                '-p', 'active_timeout:=0.5',
            ], stdout=log, stderr=log)

            position = 0.0

            def feed(duration, *, obstacle=False, moving=True, sensors=True, active=True,
                     invalid_scan=False, stale_stamp=False, heartbeat=True, scan_frame='base_link'):
                nonlocal position
                until = time.monotonic() + duration
                while time.monotonic() < until:
                    if process.poll() is not None:
                        raise AssertionError(f'nav_safety exited: {process.returncode}')
                    if heartbeat:
                        pubs['active'].publish(Bool(data=active))
                    stamp = node.get_clock().now().to_msg()
                    command = TwistStamped()
                    command.header.stamp = stamp
                    command.header.frame_id = 'base_link'
                    command.twist.linear.x = 0.1
                    pubs['intent'].publish(command)
                    if sensors:
                        odom = Odometry()
                        odom.header.stamp = stamp
                        odom.header.frame_id = 'odom'
                        odom.child_frame_id = 'base_link'
                        odom.pose.pose.orientation.w = 1.0
                        if moving:
                            position += 0.02
                        odom.pose.pose.position.x = position
                        pubs['odom'].publish(odom)
                        scan = LaserScan()
                        scan.header.stamp = stamp
                        if stale_stamp:
                            scan.header.stamp.sec -= 10
                        scan.header.frame_id = scan_frame
                        transformed = scan_frame == 'nav_safety_test_lidar'
                        scan.angle_min = -math.pi / 2 - 0.2 if transformed else -0.2
                        scan.angle_increment = 0.1
                        scan.range_min = 0.05
                        scan.range_max = 8.0
                        distance = (0.5 if transformed else 0.3) if obstacle else 3.0
                        scan.ranges = [math.nan if invalid_scan else distance] * 5
                        pubs['scan'].publish(scan)
                    rclpy.spin_once(node, timeout_sec=0.02)
                    time.sleep(0.02)

            def require(state, reason):
                assert reports, 'No safety report'
                result = reports[-1]
                assert result['state'] == state and result['reason'] == reason, reports[-4:]
                assert set(result) == {
                    'version', 'sequence', 'monotonic_time', 'state', 'reason', 'observed_seconds'}
                assert result['version'] == 2 and result['observed_seconds'] >= 0
                print(f'{state}/{reason}: OK', flush=True)

            try:
                deadline = time.monotonic() + 8
                while any(pub.get_subscription_count() == 0 for pub in pubs.values()):
                    assert time.monotonic() < deadline, 'DDS discovery timeout'
                    node_period = 0.05
                    rclpy.spin_once(node, timeout_sec=node_period)
                feed(0.4)
                require('CLEAR', 'NONE')
                feed(0.3, scan_frame='missing_test_lidar')
                require('INPUT_STALE', 'SCAN_INPUT')
                feed(0.3, scan_frame='nav_safety_test_lidar')
                require('CLEAR', 'NONE')
                feed(0.8, obstacle=True, scan_frame='nav_safety_test_lidar')
                require('BLOCKED', 'OBSTACLE')  # real progress prevents STUCK, despite proximity
                feed(0.2, obstacle=True)
                require('BLOCKED', 'OBSTACLE')
                feed(0.6, obstacle=True, moving=False)
                require('STUCK', 'PERSISTENT_OBSTACLE')
                assert reports[-1]['observed_seconds'] >= 0.4
                feed(0.3)
                require('CLEAR', 'NONE')  # no reset/ack needed
                feed(1.1, moving=False)
                require('STUCK', 'NO_LINEAR_PROGRESS')
                feed(0.3)
                require('CLEAR', 'NONE')
                feed(0.3, invalid_scan=True)
                require('INPUT_STALE', 'SCAN_INPUT')
                feed(0.3, stale_stamp=True)
                require('INPUT_STALE', 'SCAN_INPUT')
                feed(0.3)
                require('CLEAR', 'NONE')
                feed(0.7, sensors=False)
                require('INPUT_STALE', 'ODOM_INPUT')
                feed(0.3, active=False)
                require('IDLE', 'INACTIVE')
                feed(0.7, heartbeat=False)
                require('INPUT_STALE', 'ACTIVE_INPUT')
                assert node.count_publishers('/cmd_vel') == 0
                assert node.count_subscribers('/team7/nav/reset') == 0
                assert node.count_subscribers('/team7/nav/replan_ack') == 0
                assert all(b['sequence'] > a['sequence'] for a, b in zip(reports, reports[1:]))
                print('ROS integration passed; no cmd_vel publisher or reset/ack subscribers')
            except BaseException:
                log.flush()
                log.seek(0)
                print(log.read())
                raise
            finally:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
                node.destroy_subscription(subscription)
                node.destroy_node()
                rclpy.shutdown()


if __name__ == '__main__':
    main()
