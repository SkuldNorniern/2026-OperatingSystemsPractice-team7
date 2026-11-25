"""Verify that an in-flight Nav2 goal cannot overlap the next V1 action."""
from concurrent.futures import Future
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

from nav2_motion import Nav2GoalError, Nav2Motion


class GoalHandle:
    accepted = True

    def __init__(self):
        self.result = Future()
        self.cancel_count = 0

    def get_result_async(self):
        return self.result

    def cancel_goal_async(self):
        self.cancel_count += 1


class Client:
    def __init__(self):
        self.requests = []

    def server_is_ready(self):
        return True

    def send_goal_async(self, command):
        acceptance = Future()
        self.requests.append((command, acceptance))
        return acceptance

    def accept(self):
        handle = GoalHandle()
        self.requests[-1][1].set_result(handle)
        return handle


class MotionTests(unittest.TestCase):
    def setUp(self):
        self.motion = Nav2Motion.__new__(Nav2Motion)
        self.motion.node = Mock()
        self.motion.lock = threading.RLock()
        self.motion.succeeded = 4
        self.motion._reset()
        self.nav, self.spin = Client(), Client()

    def test_cancel_pending_navigation_before_spin(self):
        motion = self.motion
        self.assertFalse(motion._request('waypoint', self.nav, 'go', 30))
        motion.cancel()
        self.assertFalse(motion._request('scan', self.spin, 'full turn', 30))
        self.assertEqual(self.spin.requests, [])
        handle = self.nav.accept()
        motion.cancel()
        self.assertEqual(handle.cancel_count, 1)
        self.assertFalse(motion._request('scan', self.spin, 'full turn', 30))
        self.assertEqual(self.spin.requests, [])
        self.assertEqual(handle.cancel_count, 1)
        handle.result.set_result(SimpleNamespace(status=5))
        motion.cancel()
        self.assertFalse(motion._request('scan', self.spin, 'full turn', 30))
        self.assertEqual(len(self.spin.requests), 1)

    def test_action_success_is_only_reported_after_result(self):
        self.assertFalse(self.motion._request('scan', self.spin, 'full turn', 30))
        handle = self.spin.accept()
        self.assertFalse(self.motion._request('scan', self.spin, 'full turn', 30))
        handle.result.set_result(SimpleNamespace(status=4))
        self.assertTrue(self.motion._request('scan', self.spin, 'full turn', 30))
        self.assertEqual(len(self.spin.requests), 1)

    def test_rejection_is_not_reported_as_arrival(self):
        self.motion._request('waypoint', self.nav, 'go', 30)
        self.nav.requests[-1][1].set_result(SimpleNamespace(accepted=False))
        with self.assertRaisesRegex(Nav2GoalError, 'rejected'):
            self.motion._request('waypoint', self.nav, 'go', 30)
