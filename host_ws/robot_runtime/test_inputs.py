"""ROS message-shaped fixtures exercise adapter conversion without ROS installed."""
import math,sys,unittest
from pathlib import Path
from types import SimpleNamespace as NS
import cv2,numpy as np
from ros_transport import ROSAdapter
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'sim/gazebo/native_sim'))

class InputTests(unittest.TestCase):
    def adapter(self):
        a=ROSAdapter(NS(execute=False,map_frame='map',base_frame='base_link',optical_frame=''))
        a.age=lambda header:0.;a.lookup=lambda *args:np.eye(4)
        return a
    def header(self,frame):return NS(frame_id=frame,stamp=NS(sec=1,nanosec=0))
    def test_scan_applies_laser_offset(self):
        a=self.adapter();T=np.eye(4);T[0,3]=.2;a.lookup=lambda *args:T
        scan=NS(header=self.header('laser'),angle_min=-.15,angle_increment=.01,range_min=.1,range_max=12,ranges=[1.]*31)
        a.scan_cb(scan)
        self.assertIn('scan',a.gate.fresh);self.assertIsNotNone(a.latest_scan)
        self.assertGreater(a.gate.front,1.19);self.assertLess(a.gate.front,1.21)
    def test_stale_scan_does_not_become_ready(self):
        a=self.adapter();a.age=lambda header:5.
        a.scan_cb(NS(header=self.header('laser')))
        self.assertNotIn('scan',a.gate.fresh);self.assertEqual(a.gate.front,0)
    def test_ros_grid_is_imported_in_map_coordinates(self):
        a=self.adapter();a.mission=NS(grid=None)
        info=NS(width=12,height=8,resolution=.05,origin=NS(position=NS(x=-2.,y=-1.),orientation=NS(x=0.,y=0.,z=0.,w=1.)))
        data=[0]*96;data[3]=100;data[4]=-1
        a.map_cb(NS(header=self.header('map'),info=info,data=data))
        self.assertEqual(a.mission.grid.origin,(-2.,-1.));self.assertEqual(a.mission.grid.res,.05)
        self.assertEqual(a.mission.grid.log[0,3],4);self.assertEqual(a.mission.grid.log[0,4],0)
        self.assertIn('map',a.gate.fresh)
    def test_wrong_map_frame_is_rejected(self):
        a=self.adapter();a.map_cb(NS(header=self.header('wrong_frame')))
        self.assertNotIn('map',a.gate.fresh)
    def test_camera_passes_real_calibration_and_tf(self):
        a=self.adapter();frame='camera_optical'
        a.info=NS(width=64,height=48,header=self.header(frame),k=[100.,0,32,0,100,24,0,0,1],d=[0.,0.,0.,0.,0.])
        a.latest_pose=(0,0,0);received=[];a.mission=NS(image_cb=received.append)
        T=np.eye(4);T[:3,3]=[2,3,1];a.lookup=lambda *args:T
        ok,data=cv2.imencode('.jpg',np.full((48,64,3),127,np.uint8));self.assertTrue(ok)
        a.image_cb(NS(header=self.header(frame),format='jpeg',data=data.tobytes()))
        self.assertEqual(len(received),1);self.assertEqual(received[0].intrinsics[0,0],100.)
        np.testing.assert_array_equal(received[0].camera_to_map,T)
        self.assertIn('camera',a.gate.fresh)

if __name__=='__main__':unittest.main()
