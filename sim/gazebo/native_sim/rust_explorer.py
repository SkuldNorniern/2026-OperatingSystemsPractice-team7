"""Native Gazebo adapter to unchanged rust_base frontier and Coverage cores."""
import ctypes as C
from pathlib import Path
import sys
import numpy as np

class Goal(C.Structure):
    _fields_=[('x',C.c_double),('y',C.c_double),('yaw',C.c_double),('kind',C.c_uint32)]

class RustExplorer:
    def __init__(self):
        repo=Path(__file__).resolve().parents[3]
        filename='libtb4_aruco_gazebo.dylib' if sys.platform=='darwin' else 'libtb4_aruco_gazebo.so'
        self.lib=C.CDLL(str(repo/'target/release'/filename))
        self.lib.tb4_explorer_create.restype=C.c_void_p;self.lib.tb4_explorer_create.argtypes=[]
        self.lib.tb4_explorer_destroy.argtypes=[C.c_void_p];self.lib.tb4_explorer_destroy.restype=None
        self.lib.tb4_explorer_goals.argtypes=[C.c_void_p,C.POINTER(C.c_int8),C.POINTER(C.c_int8),C.c_uint32,C.c_uint32,*([C.c_double]*6),C.POINTER(C.c_double),C.c_size_t,C.POINTER(Goal),C.c_size_t]
        self.lib.tb4_explorer_goals.restype=C.c_int32
        self.context=self.lib.tb4_explorer_create();self.output=(Goal*32)()
        if not self.context:raise RuntimeError('Rust explorer initialization failed')
    def goals(self,grid,pose,blacklist):
        raw=np.full(grid.log.shape,-1,np.int8);raw[grid.log<-.4]=0;raw[grid.log>1]=100
        cost=raw.copy();cost[grid.blocked()]=100
        points=np.ascontiguousarray(blacklist,dtype=np.float64).reshape(-1,2)
        n=self.lib.tb4_explorer_goals(self.context,raw.ctypes.data_as(C.POINTER(C.c_int8)),cost.ctypes.data_as(C.POINTER(C.c_int8)),raw.shape[1],raw.shape[0],grid.res,*grid.origin,*pose,points.ctypes.data_as(C.POINTER(C.c_double)),len(points),self.output,len(self.output))
        if n<0:raise RuntimeError(f'Rust exploration failed: {n}')
        return [{'x':self.output[i].x,'y':self.output[i].y,'yaw':self.output[i].yaw,'kind':'frontier' if self.output[i].kind==1 else 'coverage'} for i in range(n)]
    def close(self):
        if getattr(self,'context',None):self.lib.tb4_explorer_destroy(self.context);self.context=None
    def __del__(self):self.close()
