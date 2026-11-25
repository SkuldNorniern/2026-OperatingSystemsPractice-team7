import unittest
from types import SimpleNamespace as NS

import numpy as np

from depth_input import DepthFrames, select_depth_topic


class DepthInputTests(unittest.TestCase):
    def header(self, ns=50_000_000, frame='rgb_optical'):
        return NS(frame_id=frame, stamp=NS(sec=1, nanosec=ns))

    def calibration(self, width, height, focal):
        return NS(header=self.header(), width=width, height=height,
                  k=[focal, 0, width/2, 0, focal, height/2, 0, 0, 1],
                  d=[], distortion_model='plumb_bob')

    def image(self, pixels, encoding, *, ns=50_000_000, camera='rgb_optical'):
        height, width = pixels.shape
        step = pixels.dtype.itemsize * (width + 1)
        data = b''.join(row.tobytes() + bytes(pixels.dtype.itemsize) for row in pixels)
        return NS(header=self.header(ns, camera), width=width, height=height,
                  step=step, data=data, encoding=encoding, is_bigendian=pixels.dtype.byteorder == '>')

    def test_selects_aligned_raw_depth_instead_of_visualization_or_compressed(self):
        image = ['sensor_msgs/msg/Image']
        self.assertEqual(select_depth_topic([
            ('/oakd/aligned_depth/colorized', image),
            ('/oakd/stereo/image_raw', image),
            ('/oakd/aligned_depth/image_raw', image),
            ('/oakd/aligned_depth/compressed', ['sensor_msgs/msg/CompressedImage']),
        ], '/oakd/rgb/preview/image_raw/compressed'), '/oakd/aligned_depth/image_raw')

    def test_uses_calibrated_depth_projection_with_distinct_resolution_and_row_padding(self):
        depth = DepthFrames()
        depth.info = self.calibration(32, 24, 20)
        pixels = np.zeros((24, 32), dtype='>u2')
        pixels[10:15, 16:21] = 1800
        depth.add(self.image(pixels, '16UC1'))
        frame = depth.match(self.header(), self.calibration(64, 48, 40), 'rgb_optical')
        t = np.array([[.2], [0.], [2.]])
        np.testing.assert_allclose(frame.refine_translation(t), [[.19], [0.], [1.9]])

    def test_discards_unsynchronized_unaligned_or_inconsistent_depth(self):
        depth = DepthFrames()
        info = self.calibration(32, 24, 20)
        pixels = np.full((24, 32), 4.0, dtype='<f4')
        depth.add(self.image(pixels, '32FC1', camera='stereo_optical'))
        depth.add(self.image(pixels, '32FC1', ns=10_000_000))
        self.assertIsNone(depth.match(self.header(300_000_000), info, 'rgb_optical'))
        self.assertIsNone(depth.match(self.header(), info, 'different_optical'))
        t = np.array([[0.], [0.], [2.]])
        self.assertIsNone(depth.match(self.header(), info, 'rgb_optical').refine_translation(t))
        depth.add(self.image(np.ones((24, 32), dtype='<f4') * 2.1, '32FC1'))
        np.testing.assert_allclose(depth.match(self.header(), info, 'rgb_optical').refine_translation(t),
                                   [[0.], [0.], [2.05]])
