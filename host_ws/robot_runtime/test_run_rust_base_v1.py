import os
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from run_rust_base_v1 import live_ros_environment


class LiveRosEnvironmentTests(unittest.TestCase):
    def test_live_domain_defaults_to_zero_after_a_replay(self):
        for robot_domain in ({}, {'TB4_DOMAIN': ''}):
            with (self.subTest(robot_domain=robot_domain),
                  patch.dict(os.environ, {'ROS_DOMAIN_ID': '79', **robot_domain}, clear=True),
                  patch('run_rust_base_v1.subprocess.run', return_value=SimpleNamespace(stdout=''))):
                env = live_ros_environment(Path('/checkout'))
                self.assertEqual(env['TB4_DOMAIN'], '0')
                self.assertEqual(env['ROS_DOMAIN_ID'], '0')
                self.assertEqual(env['RMW_IMPLEMENTATION'], 'rmw_fastrtps_cpp')
                self.assertEqual(os.environ['ROS_DOMAIN_ID'], '79')

    def test_explicit_robot_domain_takes_priority(self):
        with (patch.dict(os.environ, {'ROS_DOMAIN_ID': '79', 'TB4_DOMAIN': '15'}, clear=True),
              patch('run_rust_base_v1.subprocess.run', return_value=SimpleNamespace(stdout=''))):
            env = live_ros_environment(Path('/checkout'))
        self.assertEqual(env['TB4_DOMAIN'], '15')
        self.assertEqual(env['ROS_DOMAIN_ID'], '15')
