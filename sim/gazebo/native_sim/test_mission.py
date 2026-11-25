import math,threading,time,unittest
from types import SimpleNamespace
import cv2,numpy as np
from mission import Grid,Mission,overlaps_qr,wrap
from rust_detector import RustDetector
from wall_survey import WallSurvey, visible_walls
from gz.msgs10.laserscan_pb2 import LaserScan

class MissionTests(unittest.TestCase):
    def stub(self):
        m=Mission.__new__(Mission);m.cfg={'room_width':19.85,'room_height':11.05,'wall_standoff':.8,'scan_every_m':5.,'marker_sizes':{'8':.25,'19':.2,'29':.28},'marker_standoff':.8}
        m.interior_exploration=False
        m.wall_survey=None
        m.pose=(-5.,0.,0.);m.markers={};m.visited=set();m.visit_ids=None;m.expected_targets=3;m.command=lambda *a:None;m.events=[];m.event=lambda e,**kw:m.events.append((e,kw));m.grid=Grid();m.path=[]
        return m
    def rectangle_scan(self, pose, *, half_width=10, half_height=5.5, reach=12):
        scan = LaserScan(range_min=.1, range_max=reach, angle_min=-math.pi, angle_step=2*math.pi/360)
        x, y, yaw = pose
        for index in range(360):
            angle = yaw + scan.angle_min + index * scan.angle_step
            dx, dy = math.cos(angle), math.sin(angle)
            wall_x = half_width if dx > 0 else -half_width
            wall_y = half_height if dy > 0 else -half_height
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
    def test_wall_survey_requires_a_visible_wall(self):
        survey = WallSurvey(.8)
        pose = (0., 0., 0.)
        self.assertIsNone(survey.step(self.rectangle_scan(pose, reach=4), pose))
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
    def test_hybrid_handoff_waits_for_active_rust_goal_to_finish_cancelling(self):
        mission = self.stub()
        mission.interior_exploration = True
        mission.pose = (0., 0., 0.)
        mission.finish_perimeter()
        self.assertEqual(mission.state, 'EXPLORE_INTERIOR')
        self.assertEqual([name for name, _ in mission.events], ['patrol_lap_complete', 'interior_exploration_begin'])
        terminal = iter([False, True])
        mission.transport = SimpleNamespace(stop_exploration=lambda: next(terminal))
        for mid, x in [(8, 4), (19, 2), (29, 6)]:
            mission.markers[str(mid)] = {'id':mid,'x':x,'y':0,'normal_x':-1,'normal_y':0,'observations':2}
        self.assertTrue(mission.begin_marker_visits())
        self.assertIsNone(mission.visit_ids)
        self.assertTrue(mission.begin_marker_visits())
        self.assertEqual(mission.state, 'VISIT_MARKER')
        self.assertEqual(mission.target['id'], 19)

    def test_hybrid_starts_from_one_wall_and_maps_a_full_lap_before_rust_explorer(self):
        mission = self.stub()
        mission.interior_exploration = mission.nav2 = mission.external_map = True
        mission.state = 'WAIT_SENSORS'
        mission.pose = (0., -7., 0.)
        mission.goal = mission.wall_step = None
        mission.next_wall_plan = 0.0
        mission.wall_survey = WallSurvey(.8)
        mission.lock = threading.Lock()
        mission.image = mission.first_sim_time = mission.last_pose = None
        mission.odom_seen = mission.scan_seen = time.monotonic()
        mission.odom_t = mission.last_save = mission.distance = mission.total_distance = mission.total_rotation = 0.0
        visited = []
        starts = []

        def scan():
            return self.rectangle_scan(mission.pose, half_width=12, half_height=9, reach=5)

        def navigate(goal, _pose):
            visited.append(goal)
            x, y, _ = mission.pose
            mission.pose = (*goal, math.atan2(goal[1] - y, goal[0] - x))
            return True

        mission.transport = SimpleNamespace(navigate=navigate, start_exploration=lambda: starts.append(True))
        mission.scan = scan()
        self.assertEqual(len(visible_walls(mission.scan, mission.pose)), 1)
        mission.wait_print = time.monotonic() - 10
        mission.tick()
        self.assertFalse(visited)
        self.assertEqual(mission.events[-1][0], 'waiting_for_wall_path')

        mission.grid.reachable_goal = lambda _start, goal: goal
        mission.next_wall_plan = 0.0
        for _ in range(100):
            mission.scan = scan()
            mission.tick()
            self.assertFalse(starts)
            if mission.state == 'EXPLORE_INTERIOR':
                break
        self.assertEqual(len(mission.wall_survey.corners), 4)
        self.assertLess(math.dist(visited[0], visited[-1]), .25)
        np.testing.assert_allclose(mission.bounds, [-12, 12, -9, 9], atol=.11)
        mission.tick()
        self.assertEqual(starts, [True])
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
