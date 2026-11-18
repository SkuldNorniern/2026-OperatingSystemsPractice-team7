"""Exercise the compiled Rust detector against independently generated OpenCV markers."""
import unittest
from unittest.mock import patch
import cv2
import numpy as np
from rust_detector import RustDetector

class RustDetectorTests(unittest.TestCase):
    def setUp(self):self.detector=RustDetector()
    def tearDown(self):self.detector.close()
    def image(self,mid):
        d=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_50)
        return cv2.copyMakeBorder(cv2.aruco.generateImageMarker(d,mid,210),35,35,35,35,cv2.BORDER_CONSTANT,value=255)
    def test_all_three_ids_and_four_rotations(self):
        # The actual image detector must be Rust, even if OpenCV's detector is unusable.
        with patch.object(cv2.aruco,'ArucoDetector',side_effect=AssertionError('OpenCV detection used')):
            for mid in [8,19,29]:
                for turns in range(4):
                    image=np.rot90(self.image(mid),turns)
                    corners,ids,_=self.detector.detectMarkers(image)
                    self.assertIsNotNone(ids,(mid,turns));self.assertIn(mid,ids.flatten())
                    self.assertEqual(corners[0].shape,(1,4,2))
    def test_perspective_marker(self):
        image=self.image(19)
        transform=cv2.getPerspectiveTransform(np.float32([[0,0],[279,0],[279,279],[0,279]]),np.float32([[25,5],[270,30],[240,270],[10,250]]))
        image=cv2.warpPerspective(image,transform,(300,300),borderValue=255)
        _,ids,_=self.detector.detectMarkers(image);self.assertIsNotNone(ids);self.assertIn(19,ids.flatten())
    def test_blank_frame_has_no_marker(self):
        self.assertIsNone(self.detector.detectMarkers(np.full((480,640),255,np.uint8))[1])
    def test_invalid_input_and_repeated_close(self):
        with self.assertRaises(ValueError):self.detector.detectMarkers(np.zeros((2,2,3),np.uint8))
        self.detector.close();self.detector.close()
        with self.assertRaises(RuntimeError):self.detector.detectMarkers(self.image(8))

if __name__=='__main__':unittest.main()
