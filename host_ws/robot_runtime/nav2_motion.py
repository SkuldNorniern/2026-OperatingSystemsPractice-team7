"""Serialize Nav2 movement and spin actions for the V1 mission."""
import math
import threading
import time


class Nav2Motion:
    def __init__(self, node, map_frame):
        from action_msgs.msg import GoalStatus
        from nav2_msgs.action import NavigateToPose, Spin
        from rclpy.action import ActionClient

        self.node = node
        self.frame = map_frame
        self.navigation = ActionClient(node, NavigateToPose, '/navigate_to_pose')
        self.spin = ActionClient(node, Spin, '/spin')
        self.NavigateToPose = NavigateToPose
        self.Spin = Spin
        self.succeeded = GoalStatus.STATUS_SUCCEEDED
        self.lock = threading.RLock()
        self.key = None
        self.acceptance = None
        self.handle = None
        self.result = None
        self.cancelled = False
        self.cancelling = False
        self.started = 0.0

    def ready(self):
        return self.navigation.server_is_ready() and self.spin.server_is_ready()

    def navigate(self, goal, pose):
        command = self.NavigateToPose.Goal()
        command.pose.header.frame_id = self.frame
        command.pose.header.stamp = self.node.get_clock().now().to_msg()
        command.pose.pose.position.x, command.pose.pose.position.y = goal
        yaw = math.atan2(goal[1] - pose[1], goal[0] - pose[0])
        command.pose.pose.orientation.z = math.sin(yaw / 2)
        command.pose.pose.orientation.w = math.cos(yaw / 2)
        return self._request(('navigate', *goal), self.navigation, command, 180)

    def turn(self, angle, purpose):
        command = self.Spin.Goal()
        command.target_yaw = angle
        command.time_allowance.sec = 40
        return self._request(('spin', purpose), self.spin, command, 45)

    def _request(self, key, client, command, timeout):
        with self.lock:
            if self.key is not None:
                if self.key != key:
                    self.cancel()
                    return False
                if not self._poll():
                    if time.monotonic() - self.started > timeout:
                        self.cancel()
                        raise RuntimeError(f'Nav2 action timed out: {key}')
                    return False
                if not self.cancelled:
                    response = self.result.result() if self.result else None
                    self._reset()
                    if response is None or response.status != self.succeeded:
                        detail = getattr(getattr(response, 'result', None), 'error_msg', '')
                        raise RuntimeError(f'Nav2 action did not succeed: {key} {detail}')
                    return True
                self._reset()
            if client.server_is_ready():
                self.acceptance = client.send_goal_async(command)
                self.key = key
                self.started = time.monotonic()
            return False

    def _poll(self):
        if not self.acceptance.done():
            return False
        if self.handle is None:
            self.handle = self.acceptance.result()
            if self.handle is None or not self.handle.accepted:
                return True
            self.result = self.handle.get_result_async()
        return self.result.done()

    def _reset(self):
        self.key = self.acceptance = self.handle = self.result = None
        self.cancelled = self.cancelling = False

    def cancel(self):
        with self.lock:
            if self.key is None:
                return
            self.cancelled = True
            if self._poll():
                self._reset()
            elif self.handle is not None and not self.cancelling:
                self.handle.cancel_goal_async()
                self.cancelling = True

    def close(self):
        deadline = time.monotonic() + 4
        while time.monotonic() < deadline:
            self.cancel()
            with self.lock:
                if self.key is None:
                    return
            time.sleep(.05)
        self.node.get_logger().error('Nav2 did not confirm goal cancellation; stopping the managed stack')
