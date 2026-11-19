#!/usr/bin/env python3
"""Stream the camera frames of a rosbag as gray frames for tb4_aruco's stream_eval example.

usage: bag_frames.py BAG_DIR_OR_MCAP [TOPIC] | cargo run --release -p tb4_aruco --example stream_eval
Each frame: u32 width, u32 height, f64 time (s from the first frame), gray bytes; little endian.
Several .mcap files of one run: pass each file, they are read one after another.
"""
import struct
import sys

import cv2
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import CompressedImage

topic = sys.argv[2] if len(sys.argv) > 2 else '/oakd/rgb/preview/image_raw/compressed'
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=sys.argv[1], storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
r.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
out = sys.stdout.buffer
t0 = None
while r.has_next():
    _, data, ts = r.read_next()
    m = deserialize_message(data, CompressedImage)
    img = cv2.imdecode(np.frombuffer(bytes(m.data), np.uint8), cv2.IMREAD_GRAYSCALE)
    if img is None:
        continue
    t0 = t0 if t0 is not None else ts
    h, w = img.shape
    out.write(struct.pack('<IId', w, h, (ts - t0) / 1e9))
    out.write(np.ascontiguousarray(img).tobytes())
