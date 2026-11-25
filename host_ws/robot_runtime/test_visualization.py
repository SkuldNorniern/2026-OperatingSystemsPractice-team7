import os
from pathlib import Path
from tempfile import TemporaryDirectory
import tomllib
import unittest
from unittest.mock import patch

from visualization import MARKER_TOPIC, VIEWER_PROFILE, find_native_viewer, prepare_viewer_profile


class VisualizationTests(unittest.TestCase):
    def test_uses_an_executable_from_an_existing_workspace(self):
        with TemporaryDirectory() as directory, patch.dict(os.environ, AMENT_PREFIX_PATH=''):
            install = Path(directory) / 'install'
            self.assertIsNone(find_native_viewer([install]))
            viewer = install / 'team7_viz/lib/team7_viz/team7_viz'
            viewer.parent.mkdir(parents=True)
            viewer.touch()
            self.assertIsNone(find_native_viewer([install]))
            viewer.chmod(0o700)
            self.assertEqual(find_native_viewer([install]), viewer)

    def test_profile_follows_robot_and_uses_v1_topics(self):
        with TemporaryDirectory() as directory:
            config = Path(directory)
            prepare_viewer_profile(config, map_topic='/room/map', scan_topic='/robot/scan',
                                   image_topic='/robot/rgb/compressed')
            profile = tomllib.loads((config / 'team7_viz' / f'{VIEWER_PROFILE}.toml').read_text())
        self.assertTrue(profile['view']['follow'])
        self.assertEqual(profile['camera_topic'], '/robot/rgb')
        self.assertEqual({layer['kind']: layer['topic'] for layer in profile['layers']},
                         {'map': '/room/map', 'scan': '/robot/scan',
                          'confirmed': MARKER_TOPIC, 'candidates': MARKER_TOPIC})
