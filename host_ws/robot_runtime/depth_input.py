"""Time-matched RGB-aligned depth for ArUco position estimates."""
from collections import deque
from dataclasses import dataclass

import cv2
import numpy as np

DEPTH_FORMATS = {'16UC1': ('u2', .001), '32FC1': ('f4', 1.0)}
MAX_FRAME_SKEW_NS = 80_000_000


def select_depth_topic(topics, rgb_topic):
    """Select one raw image stream, preferring aligned depth on the RGB camera."""
    camera = rgb_topic.partition('/rgb/')[0].partition('/color/')[0]
    candidates = []
    for topic, types in topics:
        name = topic.lower()
        if 'sensor_msgs/msg/Image' not in types or topic == rgb_topic:
            continue
        if any(kind in name for kind in ('disparity', 'colorized', 'visualization')):
            continue
        if not ('depth' in name or name.endswith(('/stereo/image_raw', '/stereo/image_rect_raw'))):
            continue
        rank = ('aligned' not in name and 'registered' not in name,
                not topic.startswith(camera + '/'), topic)
        candidates.append(rank)
    return min(candidates)[-1] if candidates else None


def stamp_ns(header):
    return header.stamp.sec * 1_000_000_000 + header.stamp.nanosec


@dataclass(frozen=True)
class DepthFrame:
    pixels: np.ndarray
    units: float
    intrinsics: np.ndarray
    distortion: np.ndarray | None
    rectification: np.ndarray

    def refine_translation(self, translation):
        """Blend consistent depth with PnP while keeping the RGB viewing ray."""
        point = self.rectification @ translation.reshape(3)
        z = float(point[2])
        if not np.isfinite(point).all() or z <= 0:
            return None
        origin = np.zeros(3)
        projected, _ = cv2.projectPoints(point.reshape(1, 3), origin, origin,
                                         self.intrinsics, self.distortion)
        uv = projected.reshape(2)
        if not np.isfinite(uv).all():
            return None
        u, v = map(round, uv)
        height, width = self.pixels.shape
        if not (2 <= u < width - 2 and 2 <= v < height - 2):
            return None
        patch = self.pixels[v-2:v+3, u-2:u+3].astype(float) * self.units
        valid = patch[np.isfinite(patch) & (patch > 0)]
        if valid.size < 9:
            return None
        depth = float(np.median(valid))
        inliers = np.count_nonzero(np.abs(valid - depth) <= max(.04, .04 * depth))
        if inliers < max(9, .7 * valid.size) or abs(depth - z) > max(.12, .20 * z):
            return None
        return translation * ((z + depth) / (2 * z))


class DepthFrames:
    def __init__(self):
        self.images = deque(maxlen=4)
        self.info = None

    def add(self, message):
        if message.encoding in DEPTH_FORMATS and stamp_ns(message.header) > 0:
            self.images.append(message)

    def match(self, rgb_header, rgb_info, optical_frame, *, rectified=False):
        rgb_stamp = stamp_ns(rgb_header)
        images = [image for image in self.images if image.header.frame_id == optical_frame]
        image = min(images, key=lambda image: abs(stamp_ns(image.header) - rgb_stamp), default=None)
        if image is None or abs(stamp_ns(image.header) - rgb_stamp) > MAX_FRAME_SKEW_NS:
            return None

        info = self.info
        if info is None:
            if rectified:
                return None
            info = rgb_info
        if info.header.frame_id != optical_frame or (image.width, image.height) != (info.width, info.height):
            return None

        rectification = np.eye(3)
        if rectified:
            projection = np.asarray(info.p, dtype=float).reshape(3, 4)
            if not np.allclose(projection[:, 3], 0):
                return None
            intrinsics = projection[:, :3].copy()
            rectification = np.asarray(info.r, dtype=float).reshape(3, 3)
            if not np.allclose(rectification @ rectification.T, np.eye(3), atol=.01):
                return None
            distortion = None
        else:
            intrinsics = np.asarray(info.k, dtype=float).reshape(3, 3)
            distortion = np.asarray(info.d, dtype=float)
            if (info.distortion_model not in ('', 'plumb_bob', 'rational_polynomial')
                    or distortion.size not in (0, 4, 5, 8, 12, 14) or not np.isfinite(distortion).all()):
                return None
        if not np.isfinite(intrinsics).all() or intrinsics[0, 0] <= 0 or intrinsics[1, 1] <= 0:
            return None

        element, units = DEPTH_FORMATS[image.encoding]
        dtype = np.dtype(('>' if image.is_bigendian else '<') + element)
        if min(image.width, image.height) < 5 or image.step < image.width * dtype.itemsize:
            return None
        if len(image.data) < image.step * image.height:
            return None
        pixels = np.ndarray((image.height, image.width), dtype=dtype, buffer=image.data,
                            strides=(image.step, dtype.itemsize))
        return DepthFrame(pixels, units, intrinsics, distortion, rectification)
