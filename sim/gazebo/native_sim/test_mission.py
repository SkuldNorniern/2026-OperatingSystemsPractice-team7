import math,threading,time,unittest
from types import SimpleNamespace
import cv2,numpy as np
from mission import Grid,Mission,overlaps_qr,wrap
from rust_detector import RustDetector
from gz.msgs10.laserscan_pb2 import LaserScan

class MissionTests(unittest.TestCase):
    def stub(self):
        m=Mission.__new__(Mission);m.cfg={'room_width':19.85,'room_height':11.05,'wall_standoff':.8,'scan_every_m':5.,'marker_sizes':{'8':.25,'19':.2,'29':.28},'marker_standoff':.8}
        m.pose=(-5.,0.,0.);m.markers={};m.visited=set();m.visit_ids=None;m.expected_targets=3;m.command=lambda *a:None;m.events=[];m.event=lambda e,**kw:m.events.append((e,kw));m.grid=Grid();m.path=[]
        return m
    def rectangle_scan(self, pose):
        scan = LaserScan(range_min=.1, range_max=12, angle_min=-math.pi, angle_step=2*math.pi/360)
        x, y, yaw = pose
        for index in range(360):
            angle = yaw + scan.angle_min + index * scan.angle_step
            dx, dy = math.cos(angle), math.sin(angle)
            wall_x = 10 if dx > 0 else -10
            wall_y = 5.5 if dy > 0 else -5.5
            scan.ranges.append(min(scan.range_max, (wall_x-x)/dx if dx else math.inf,
                                   (wall_y-y)/dy if dy else math.inf))
        return scan
    def test_nearest_wall_and_four_distinct_corners(self):
        m=self.stub()
        self.assertTrue(m.initialize(self.rectangle_scan(m.pose),m.pose));self.assertLess(m.goal[0],-9)
        self.assertEqual(len(set(m.route[:4])),4);self.assertEqual(m.route[-1],m.goal)
    def test_near_visible_wall_is_used_to_estimate_room(self):
        mission = self.stub()
        mission.pose = (-9.4, 0.0, 0.0)
        self.assertTrue(mission.initialize(self.rectangle_scan(mission.pose), mission.pose))
        self.assertAlmostEqual(mission.bounds[0], -10.0)
        self.assertAlmostEqual(mission.goal[0], -9.2)
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
        m.visit_ids = list(m.markers)
        m.next_marker();self.assertEqual(m.target['id'],19);self.assertAlmostEqual(m.goal[0],1.2)
        m.visited.add('19');m.next_marker();self.assertEqual(m.target['id'],8)
    def test_astar_routes_around_obstacle(self):
        g=Grid();x,y=g.cell(1,0);g.log[y-5:y+6,x-2:x+3]=4
        path=g.plan((0,0),(2,0),(-4,4,-4,4));self.assertTrue(path)
        self.assertGreater(max(abs(y) for x,y in path),.6)
    def test_third_confirmed_id_cancels_patrol_and_starts_visits(self):
        m=self.stub();m.state='PATROL';m.pose=(0,0,0)
        stops=[];m.command=lambda *a:stops.append(a)
        for mid,x in [('8',4),('19',2),('29',6)]:
            m.markers[mid]={'id':int(mid),'x':x,'y':0,'normal_x':-1,'normal_y':0,'observations':2}
        m.markers['29']['observations']=1
        self.assertFalse(m.begin_marker_visits());self.assertEqual(stops,[])
        m.markers['29']['observations']=2
        self.assertTrue(m.begin_marker_visits());self.assertEqual(stops,[()])
        self.assertEqual(m.state,'VISIT_MARKER');self.assertEqual(m.target['id'],19)
        self.assertFalse(m.begin_marker_visits())
    def test_qr_is_not_an_aruco_and_only_overlapping_candidates_are_removed(self):
        qr = cv2.QRCodeEncoder_create().encode('TurtleBot V1 QR test')
        qr = cv2.resize(qr, None, fx=8, fy=8, interpolation=cv2.INTER_NEAREST)
        frame = cv2.copyMakeBorder(qr, 24, 24, 24, 24, cv2.BORDER_CONSTANT, value=255)
        found, decoded, points, _ = cv2.QRCodeDetector().detectAndDecodeMulti(frame)
        self.assertTrue(found)
        polygons = [cv2.convexHull(p) for value, p in zip(decoded, points) if value]
        self.assertTrue(overlaps_qr(points[0], polygons))
        self.assertFalse(overlaps_qr(points[0] + frame.shape[1], polygons))
        detector = RustDetector()
        try:
            self.assertIsNone(detector.detectMarkers(frame)[1])
        finally:
            detector.close()
    def test_real_5x5_dictionary_recognizes_all_three(self):
        d=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_50);det=cv2.aruco.ArucoDetector(d)
        for mid in [8,19,29]:
            image=cv2.copyMakeBorder(cv2.aruco.generateImageMarker(d,mid,210),30,30,30,30,cv2.BORDER_CONSTANT,value=255)
            _,ids,_=det.detectMarkers(image);self.assertIn(mid,ids.flatten())

if __name__=='__main__':unittest.main()
