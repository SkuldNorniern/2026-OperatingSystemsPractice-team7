"""ROS message-shaped fixtures exercise adapter conversion without ROS installed."""
import math,sys,unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch
import cv2,numpy as np
from nav2_motion import Nav2GoalError
from ros_transport import ROSAdapter
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'sim/gazebo/native_sim'))
from mission import Grid

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
    def test_validated_candidate_applies_and_clears_speed_limit(self):
        adapter = self.adapter()
        messages = []
        adapter.SpeedLimit = NS
        adapter.speed_limit_pub = NS(publish=messages.append)
        adapter.update_marker_speed_limit(9.0)
        with patch('ros_transport.time.monotonic', return_value=10.0):
            adapter.request_marker_slowdown()
        for now in (10.0, 10.5, 11.0, 12.5, 13.0):
            adapter.update_marker_speed_limit(now)
        self.assertEqual([(m.percentage, m.speed_limit) for m in messages],
                         [(True, 65.0), (True, 65.0), (True, 0.0)])

    def test_navigation_uses_known_space_until_the_map_reaches_the_corner(self):
        adapter = self.adapter()
        adapter.ready = lambda: True
        adapter.motion = Mock()
        adapter.motion.navigate.side_effect = [False, True, True]
        grid = Grid()
        grid.origin = (0.0, 0.0)
        grid.log[0:20, 0:20] = -4
        grid.log[5:15, 47:55] = -4  # A disconnected patch around the destination.
        adapter.mission = NS(grid=grid, event=Mock())
        pose = (1.0, 1.0, 0.0)
        goal = (5.0, 1.0)

        self.assertFalse(adapter.navigate(goal, pose))
        intermediate, _ = adapter.motion.navigate.call_args.args
        self.assertTrue(1.4 < intermediate[0] < 2.0)
        grid.log[5:15, 19:48] = -4
        self.assertFalse(adapter.navigate(goal, pose))
        self.assertEqual(adapter.motion.navigate.call_args.args[0], intermediate)
        self.assertTrue(adapter.navigate(goal, (*intermediate, 0.0)))
        self.assertEqual(adapter.motion.navigate.call_args.args[0], goal)

    def test_navigation_waits_for_known_space_and_retries_rejected_goals(self):
        adapter = self.adapter()
        adapter.ready = lambda: True
        adapter.motion = Mock()
        grid = Mock()
        grid.reachable_goal.side_effect = [None, (2.0, 1.0), (2.0, 1.0)]
        adapter.mission = NS(grid=grid, event=Mock())
        adapter.motion.navigate.side_effect = [Nav2GoalError('first costmap'), True]
        goal = (2.0, 1.0)
        pose = (1.0, 1.0, 0.0)
        with patch('ros_transport.time.monotonic', side_effect=[10, 11, 12, 13]):
            self.assertFalse(adapter.navigate(goal, pose))
            adapter.motion.navigate.assert_not_called()
            self.assertFalse(adapter.navigate(goal, pose))
            self.assertFalse(adapter.navigate(goal, pose))
            self.assertTrue(adapter.navigate(goal, pose))
        self.assertEqual(adapter.motion.navigate.call_count, 2)
        self.assertEqual([call.args[0] for call in adapter.mission.event.call_args_list],
                         ['waiting_for_known_map', 'nav_goal_retry'])
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
