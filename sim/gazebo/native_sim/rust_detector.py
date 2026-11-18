"""ctypes adapter for the rust_base tb4_aruco core. No OpenCV detection fallback."""
import ctypes as C
from pathlib import Path
import sys
import numpy as np

class Detection(C.Structure):
    _fields_=[('id',C.c_uint32),('corners',C.c_float*8),('hamming',C.c_uint32)]

class RustDetector:
    def __init__(self):
        repo=Path(__file__).resolve().parents[3]
        filename='libtb4_aruco_gazebo.dylib' if sys.platform=='darwin' else 'libtb4_aruco_gazebo.so'
        library=repo/'target/release'/filename
        if not library.is_file():
            raise RuntimeError('Rust detector library missing. Run ./setup_native.sh, or cargo build --release -p tb4_aruco_gazebo from repository root.')
        self.lib=C.CDLL(str(library))
        self.lib.tb4_detector_create.restype=C.c_void_p
        self.lib.tb4_detector_create.argtypes=[]
        self.lib.tb4_detector_detect.argtypes=[C.c_void_p,C.POINTER(C.c_uint8),C.c_size_t,C.c_size_t,C.c_size_t,C.POINTER(Detection),C.c_size_t]
        self.lib.tb4_detector_detect.restype=C.c_int32
        self.lib.tb4_detector_destroy.argtypes=[C.c_void_p]
        self.lib.tb4_detector_destroy.restype=None
        self.context=self.lib.tb4_detector_create()
        if not self.context:raise RuntimeError('Rust detector initialization failed')
        self.output=(Detection*128)()
    def detectMarkers(self,gray):
        if not self.context:raise RuntimeError('Rust detector is closed')
        if gray.ndim!=2 or gray.dtype!=np.uint8:raise ValueError('Expected 2D uint8 grayscale image')
        gray=np.ascontiguousarray(gray)
        count=self.lib.tb4_detector_detect(self.context,gray.ctypes.data_as(C.POINTER(C.c_uint8)),gray.size,gray.shape[1],gray.shape[0],self.output,len(self.output))
        if count<0:raise RuntimeError(f'Rust marker detection failed: {count}')
        if count==0:return [],None,[]
        corners=[np.array(list(self.output[i].corners),dtype=np.float32).reshape(1,4,2) for i in range(count)]
        ids=np.array([[self.output[i].id] for i in range(count)],dtype=np.int32)
        return corners,ids,[]
    def close(self):
        if getattr(self,'context',None):
            self.lib.tb4_detector_destroy(self.context);self.context=None
    def __del__(self):self.close()
