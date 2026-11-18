import math,threading,time,unittest
from types import SimpleNamespace
import cv2,numpy as np
from mission import Grid,Mission,wrap,LaserScan

class MissionTests(unittest.TestCase):
    def stub(self):
        m=Mission.__new__(Mission);m.cfg={'room_width':19.85,'room_height':11.05,'wall_standoff':.8,'scan_every_m':5.,'marker_sizes':{'8':.25,'19':.2,'29':.28},'marker_standoff':.8}
        m.pose=(-5.,0.,0.);m.markers={};m.visited=set();m.events=[];m.event=lambda e,**kw:m.events.append((e,kw));m.grid=Grid();m.path=[]
        return m
    def test_nearest_wall_and_four_distinct_corners(self):
        m=self.stub();s=LaserScan();s.range_min=.1;s.range_max=12;s.angle_min=-math.pi;s.angle_step=2*math.pi/360
        for i in range(360):
            a=s.angle_min+i*s.angle_step;cx,cy=math.cos(a),math.sin(a)
            rays=[(10+5)/cx if cx>0 else (-10+5)/cx if cx<0 else 1e6,5.5/cy if cy>0 else -5.5/cy if cy<0 else 1e6]
            s.ranges.append(min(12,*rays))
        self.assertTrue(m.initialize(s,m.pose));self.assertLess(m.goal[0],-9)
        self.assertEqual(len(set(m.route[:4])),4);self.assertEqual(m.route[-1],m.goal)
    def test_scan_trigger_and_full_turn_across_yaw_wrap(self):
        m=self.stub();m.distance=5.01;m.begin_spin('PATROL')
        self.assertEqual(m.state,'SCAN_360');self.assertEqual(m.distance,0)
        last=3.;total=0.
        for i in range(1,101):
            current=wrap(3.+i*2*math.pi/100);total+=wrap(current-last);last=current
        self.assertAlmostEqual(total,2*math.pi)
    def test_nearest_unvisited_marker_and_front_standoff(self):
        m=self.stub();m.pose=(0,0,0)
        for mid,x in [('8',4),('19',2),('29',6)]:m.markers[mid]={'id':int(mid),'x':x,'y':0,'normal_x':-1,'normal_y':0,'observations':3}
        m.next_marker();self.assertEqual(m.target['id'],19);self.assertAlmostEqual(m.goal[0],1.2)
        m.visited.add('19');m.next_marker();self.assertEqual(m.target['id'],8)
    def test_astar_routes_around_obstacle(self):
        g=Grid();x,y=g.cell(1,0);g.log[y-5:y+6,x-2:x+3]=4
        path=g.plan((0,0),(2,0),(-4,4,-4,4));self.assertTrue(path)
        self.assertGreater(max(abs(y) for x,y in path),.6)
    def test_real_5x5_dictionary_recognizes_all_three(self):
        d=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_50);det=cv2.aruco.ArucoDetector(d)
        for mid in [8,19,29]:
            image=cv2.copyMakeBorder(cv2.aruco.generateImageMarker(d,mid,210),30,30,30,30,cv2.BORDER_CONSTANT,value=255)
            _,ids,_=det.detectMarkers(image);self.assertIn(mid,ids.flatten())

if __name__=='__main__':unittest.main()
