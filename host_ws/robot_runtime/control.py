"""ROS-independent motion gate; tested on the development Mac without ROS."""
import math,time
import numpy as np

def transform_matrix(translation,q):
    q=np.array([q.x,q.y,q.z,q.w],dtype=float);norm=np.linalg.norm(q)
    if not np.isfinite(q).all() or norm<1e-9:raise ValueError('Invalid TF quaternion')
    x,y,z,w=q/norm
    R=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
    T=np.eye(4);T[:3,:3]=R;T[:3,3]=[translation.x,translation.y,translation.z]
    if not np.isfinite(T).all():raise ValueError('Invalid TF translation')
    return T

class CommandGate:
    def __init__(self,timeout=.5,max_forward=.18,max_turn=.5,stop_distance=.45):
        self.timeout=timeout;self.max_forward=max_forward;self.max_turn=max_turn;self.stop_distance=stop_distance
        self.command=(0.,0.);self.command_time=-math.inf;self.fresh={};self.front=0.;self.clearance=0.;self.execute=False
    def propose(self,v,w,now=None):
        if not math.isfinite(v) or not math.isfinite(w):v=w=0.
        self.command=(max(0.,min(v,self.max_forward)),max(-self.max_turn,min(w,self.max_turn)))
        self.command_time=time.monotonic() if now is None else now
    def output(self,now=None):
        now=time.monotonic() if now is None else now
        if not self.execute or now-self.command_time>self.timeout:return 0.,0.
        for name,limit in [('scan',.6),('odom',.6),('tf',.6),('map',15.),('camera',2.)]:
            if now-self.fresh.get(name,-math.inf)>limit:return 0.,0.
        v,w=self.command
        if self.front<self.stop_distance:v=0.
        if self.clearance<.22:w=0.
        return v,w
