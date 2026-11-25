"""ROS-free Gazebo perimeter mapping mission. Sensor input only; no world-pose lookup."""
import argparse,hashlib,heapq,json,math,signal,threading,time
from pathlib import Path
import cv2
import numpy as np
from rust_detector import RustDetector
from wall_survey import WallSurvey

ROOT=Path(__file__).resolve().parent
TAU=2*math.pi

def wrap(a):return (a+math.pi)%TAU-math.pi

def yaw(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))

def stamp(m):return m.header.stamp.sec+m.header.stamp.nsec*1e-9

class Grid:
    def __init__(self):
        self.res=.1;self.origin=(-25.,-25.);self.n=600
        self.size = (self.n, self.n)
        self.log=np.zeros((self.n,self.n),np.float32)
    def cell(self,x,y):return (math.floor((x-self.origin[0])/self.res),math.floor((y-self.origin[1])/self.res))
    def xy(self,c):return (self.origin[0]+(c[0]+.5)*self.res,self.origin[1]+(c[1]+.5)*self.res)
    def update(self,scan,pose):
        x,y,a=pose; start=self.cell(x,y)
        for i in range(0,len(scan.ranges),2):
            r=scan.ranges[i];hit=math.isfinite(r) and scan.range_min<r<scan.range_max-.03
            if math.isnan(r) or r<scan.range_min:continue
            r=min(r,scan.range_max);theta=a+scan.angle_min+i*scan.angle_step
            end=self.cell(x+r*math.cos(theta),y+r*math.sin(theta))
            count=max(abs(end[0]-start[0]),abs(end[1]-start[1]),1)
            xs=np.rint(np.linspace(start[0],end[0],count+1)).astype(int)
            ys=np.rint(np.linspace(start[1],end[1],count+1)).astype(int)
            mask=(xs>=0)&(xs<self.n)&(ys>=0)&(ys<self.n);xs,ys=xs[mask],ys[mask]
            if len(xs):
                self.log[ys[:-1],xs[:-1]]-=.2
                if hit:self.log[ys[-1],xs[-1]]+=.9
        np.clip(self.log,-4,4,out=self.log)
    def blocked(self):
        return cv2.dilate((self.log>1).astype(np.uint8),np.ones((9,9),np.uint8)).astype(bool)
    def contains(self, cell):
        return 0 <= cell[0] < self.size[0] and 0 <= cell[1] < self.size[1]

    def cell_status(self, point):
        x, y = self.cell(*point)
        if not self.contains((x, y)):
            return 'outside_map'
        if self.log[y, x] > 1:
            return 'occupied'
        return 'free' if self.log[y, x] < -.4 else 'unknown'

    def extent(self):
        x, y = self.origin
        w, h = self.size
        return [x, y, x + self.res * w, y + self.res * h]

    def reachable_goal(self, start, goal):
        """Pick the destination or a closer waypoint in connected, observed free space."""
        sx, sy = self.cell(*start)
        gx, gy = self.cell(*goal)
        if not self.contains((sx, sy)):
            return None
        blocked = self.blocked()
        free = (self.log < -.4) & ~blocked
        if not free[sy, sx]:
            if blocked[sy, sx] or not any(self.contains((x, y)) and free[y, x]
                                          for x, y in ((sx-1, sy), (sx+1, sy), (sx, sy-1), (sx, sy+1))):
                return None
            # SLAM can leave the chassis cell unobserved. Connect that one cell
            # to adjacent measured free space without opening other unknown cells.
            free[sy, sx] = True
        reachable = free.astype(np.uint8)
        cv2.floodFill(reachable, None, (sx, sy), 2, flags=4)
        if self.contains((gx, gy)) and self.log[gy, gx] < -.4 and reachable[gy, gx] == 2:
            return goal

        # Leave space for the robot footprint at an intermediate waypoint.
        radius = math.ceil(.20 / self.res)
        footprint = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * radius + 1, 2 * radius + 1))
        safe = cv2.erode((reachable == 2).astype(np.uint8), footprint,
                         borderType=cv2.BORDER_CONSTANT, borderValue=0)
        rows, cols = np.nonzero(safe)
        if not len(rows):
            return None
        xs, ys = self.xy((cols, rows))
        distances = np.hypot(xs - goal[0], ys - goal[1])
        index = int(np.argmin(distances))
        if distances[index] > math.dist(start, goal) - .40:
            return None
        return float(xs[index]), float(ys[index])

    def plan(self,start,goal,bounds,known_only=False):
        a,b=self.cell(*start),self.cell(*goal);occ=self.blocked()
        if known_only:occ |= self.log>=-.4
        if not all(0<=v<self.n for v in (*a,*b)):return []
        if occ[b[1],b[0]]:return []
        todo=[(0,a)];cost={a:0};prev={}
        while todo:
            _,p=heapq.heappop(todo)
            if p==b:
                path=[p]
                while p!=a:p=prev[p];path.append(p)
                return [self.xy(c) for c in path[::-1]]
            for dx,dy in [(1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)]:
                q=(p[0]+dx,p[1]+dy)
                if not (0<=q[0]<self.n and 0<=q[1]<self.n):continue
                if occ[q[1],q[0]]:continue
                if dx and dy and (occ[p[1],q[0]] or occ[q[1],p[0]]):continue
                x,y=self.xy(q)
                if bounds and not(bounds[0]+.35<x<bounds[1]-.35 and bounds[2]+.35<y<bounds[3]-.35):continue
                c=cost[p]+math.hypot(dx,dy)
                if c<cost.get(q,1e20):
                    cost[q]=c;prev[q]=p;heapq.heappush(todo,(c+math.hypot(q[0]-b[0],q[1]-b[1]),q))
        return []
    def save(self,out):
        image=np.full_like(self.log,205,dtype=np.uint8);image[self.log<-.4]=254;image[self.log>1]=0
        cv2.imwrite(str(out/'map.pgm'),np.flipud(image))
        (out/'map.yaml').write_text(f'image: map.pgm\nresolution: {self.res}\norigin: [{self.origin[0]}, {self.origin[1]}, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')

def overlaps_qr(corners, qr_polygons):
    """Disregard ArUco candidates inside a verified QR symbol."""
    quad = np.ascontiguousarray(corners.reshape(4, 2), dtype=np.float32)
    area = cv2.contourArea(quad)
    if area <= 0:
        return False
    for polygon in qr_polygons:
        overlap, _ = cv2.intersectConvexConvex(quad, polygon)
        if overlap >= 0.5 * area:
            return True
    return False


class Mission:
    def __init__(self,args,transport=None):
        self.version='v1'
        self.args=args;self.cfg=getattr(args,'config',None) or json.loads((ROOT/'config.json').read_text())
        self.frame=getattr(args,'map_frame','gazebo_world')
        self.external_map=getattr(args,'external_map',False)
        self.out=Path(getattr(args,'output_dir',ROOT/'runs'))/(time.strftime('%Y%m%d-%H%M%S')+'-'+self.version);self.out.mkdir(parents=True)
        (self.out/'run_info.json').write_text(json.dumps({'version':self.version,'world_sha256':hashlib.sha256((ROOT/'track1_native.sdf').read_bytes()).hexdigest() if not self.external_map else None,'config':self.cfg,'max_sim_seconds':getattr(args,'max_sim_seconds',600)},indent=2))
        self.events=(self.out/'events.jsonl').open('w');self.lock=threading.Lock()
        self.pose=None;self.scan=None;self.image=None;self.odom_seen=0;self.scan_seen=0
        self.odom_t=0.;self.grid=Grid();self.markers={};self.visited=set();self.running=True
        self.allowed_ids = {str(i) for i in self.cfg.get('marker_ids', []) or self.cfg['marker_sizes']}
        self.detector=RustDetector(self.cfg['dictionary'], allow_correction=bool(self.allowed_ids))
        self.qr_detector = cv2.QRCodeDetector()
        self.expected_targets = self.cfg.get('expected_targets', len(self.cfg['marker_sizes']))
        self.visit_ids = None
        self.patrol_route = ()
        self.total_distance=0.;self.total_rotation=0.;self.first_sim_time=None
        if transport is None:
            from gazebo_transport import GazeboTransport
            transport=GazeboTransport()
        self.transport=transport
        self.nav2=getattr(args,'nav2',False)
        self.interior_exploration = bool(getattr(args, 'interior_explorer', None))
        self.wall_survey = WallSurvey(self.cfg['wall_standoff']) if self.interior_exploration else None
        self.wall_step = None
        self.next_wall_plan = 0.0
        self.state='WAIT_SENSORS';self.bounds=None;self.route=[];self.goal=None;self.path=[]
        self.last_pose=None;self.distance=0.;self.spin_total=0.;self.last_yaw=None;self.after_spin='PATROL'
        self.last_plan=-100;self.last_map=-100;self.last_save=-100;self.last_image=-100;self.status='running'
        self.start_wall=time.monotonic();self.last_msg=time.monotonic();self.wait_print=0;self.blocked_since=None
        self.transport.start(self)
        self.event('start',output=str(self.out),detector='rust_base/tb4_aruco',version=self.version)
    def event(self,event,**data):
        row={'event':event,'sim_time':self.odom_t,'version':self.version,**data};print(json.dumps(row),flush=True)
        self.events.write(json.dumps(row)+'\n');self.events.flush()
    def odom_cb(self,m):
        with self.lock:self.pose=(m.pose.position.x,m.pose.position.y,yaw(m.pose.orientation));self.odom_t=stamp(m);self.odom_seen=time.monotonic()
    def scan_cb(self,m):
        with self.lock:self.scan=m;self.scan_seen=time.monotonic()
    def image_cb(self,m):
        with self.lock:self.image=(m,self.pose)
    def command(self,v=0,w=0):
        self.transport.publish(v,w)
    def show_markers(self):
        publish = getattr(self.transport, 'publish_markers', None)
        if publish is not None:
            publish(self.markers, self.visited)

    def save(self):
        self.grid.save(self.out)
        (self.out/'markers.json').write_text(json.dumps({'frame':self.frame,'dictionary':self.cfg['dictionary'],'markers':self.markers,'visited':sorted(self.visited)},indent=2))
        (self.out/'status.json').write_text(json.dumps({'state':self.state,'status':self.status,'pose':self.pose,'bounds':self.bounds,'version':self.version,'distance_m':self.total_distance,'rotation_rad':self.total_rotation,'elapsed_sim_seconds':self.odom_t-(self.first_sim_time if self.first_sim_time is not None else self.odom_t),'known_map_cells':int(np.count_nonzero(np.abs(self.grid.log)>.4))},indent=2))
    def detect(self,pair):
        if not pair or pair[1] is None:return
        m,pose=pair
        if stamp(m)<=self.last_image:return
        self.last_image=stamp(m)
        if len(m.data)<m.height*m.step:return
        rgb=np.frombuffer(m.data,np.uint8).reshape(m.height,m.step)[:,:m.width*3].reshape(m.height,m.width,3)
        gray=cv2.cvtColor(rgb,cv2.COLOR_RGB2GRAY)
        corners,ids,_=self.detector.detectMarkers(gray)
        if ids is None:return
        # Finder-like squares alone can be false positives in a room with several
        # ArUco tags. Mask only verified QR polygons; their payload is discarded.
        has_qr, decoded, qr_corners, _ = self.qr_detector.detectAndDecodeMulti(gray)
        qr_polygons = [cv2.convexHull(p) for value, p in zip(decoded, qr_corners) if value] if has_qr else []
        updated = False
        frame_ids = set()
        f=m.width/(2*math.tan(1.2/2));K=getattr(m,'intrinsics',np.array([[f,0,m.width/2],[0,f,m.height/2],[0,0,1]],np.float64))
        D=getattr(m,'distortion',None)
        depth = getattr(m, 'depth', None)
        for mid,c in zip(ids.flatten(),corners):
            if (self.allowed_ids and str(mid) not in self.allowed_ids) or mid in frame_ids or overlaps_qr(c, qr_polygons):
                continue
            size=self.cfg['marker_sizes'].get(str(mid),self.cfg.get('default_marker_size'))
            if size is None:continue
            edges=np.linalg.norm(np.roll(c.reshape(4,2),-1,axis=0)-c.reshape(4,2),axis=1)
            if min(edges)<18 or max(edges)/min(edges)>3:continue
            s=size/2;obj=np.array([[-s,s,0],[s,s,0],[s,-s,0],[-s,-s,0]],np.float32)
            ok,rv,tv=cv2.solvePnP(obj,c.reshape(4,2),K,D,flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if not ok or tv[2,0]<=0:continue
            projected,_=cv2.projectPoints(obj,rv,tv,K,D)
            if np.mean(np.linalg.norm(projected.reshape(4,2)-c.reshape(4,2),axis=1))>1.5:continue
            refined = depth.refine_translation(tv) if depth is not None else None
            if refined is not None:
                tv = refined
            R,_=cv2.Rodrigues(rv);n=R[:,2]
            if np.dot(n,-tv[:,0])<0:n=-n
            if hasattr(m,'camera_to_map'):
                T=m.camera_to_map;point=T[:3,:3]@tv[:,0]+T[:3,3];normal=T[:3,:3]@n
                mx,my=map(float,point[:2]);nx,ny=float(normal[0]),float(normal[1])
            else:
                # Sim camera optical (right,down,forward) -> robot (forward,left,up).
                dx,dy=float(tv[2,0])+.13,-float(tv[0,0]);ca,sa=math.cos(pose[2]),math.sin(pose[2])
                mx,my=pose[0]+ca*dx-sa*dy,pose[1]+sa*dx+ca*dy
                nx,ny=ca*n[2]+sa*n[0],sa*n[2]-ca*n[0]
            norm=math.hypot(nx,ny)
            if norm<.5:continue
            nx,ny=nx/norm,ny/norm
            if self.bounds and not(self.bounds[0]-.2<mx<self.bounds[1]+.2 and self.bounds[2]-.2<my<self.bounds[3]+.2):continue
            old=self.markers.get(str(mid));count=1
            if old:
                if math.hypot(mx-old['x'],my-old['y'])>1:continue
                count=old['observations']+1;weight=1/min(count,20)
                mx=old['x']*(1-weight)+mx*weight;my=old['y']*(1-weight)+my*weight
            self.markers[str(mid)]={'id':int(mid),'x':mx,'y':my,'normal_x':float(nx),'normal_y':float(ny),'size_m':size,'observations':count}
            depth_count = old.get('depth_observations', 0) if old else 0
            if refined is not None:
                if depth_count == 0:
                    self.event('marker_depth_used', id=int(mid), fused_z_m=float(tv[2, 0]))
                depth_count += 1
            if depth_count:
                self.markers[str(mid)]['depth_observations'] = depth_count
            frame_ids.add(mid)
            updated = True
            if not old:
                self.event('marker_detected',id=int(mid),x=mx,y=my)
                slowdown = getattr(self.transport, 'request_marker_slowdown', None)
                if slowdown is not None:
                    slowdown()
            if count == 2:self.event('marker_confirmed',id=int(mid),x=mx,y=my)
        if updated:
            self.show_markers()
            temporary=self.out/'markers.json.tmp'
            temporary.write_text(json.dumps({'frame':self.frame,'dictionary':self.cfg['dictionary'],'markers':self.markers,'visited':sorted(self.visited)},indent=2));temporary.replace(self.out/'markers.json')
        cv2.imwrite(str(self.out/'camera_latest.jpg'),cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR))
    def initialize(self,scan,p):
        # Known rectangular room size; estimate location in odometry from aligned lidar wall clusters.
        pts=[]
        for i,r in enumerate(scan.ranges):
            if math.isfinite(r) and .2<r<scan.range_max-.1:
                a=p[2]+scan.angle_min+i*scan.angle_step;pts.append((p[0]+r*math.cos(a),p[1]+r*math.sin(a)))
        if len(pts)<40:return False
        pts=np.array(pts);walls=[]
        for axis,size in [(0,self.cfg['room_width']),(1,self.cfg['room_height'])]:
            bins=np.round(pts[:,axis]/.1).astype(int);values,counts=np.unique(bins,return_counts=True)
            order=np.argsort(counts)[::-1];usable=[]
            for j in order[:15]:
                if counts[j]<5:continue
                val=values[j]*.1
                if not .20 < abs(val-p[axis]) < size:continue
                if np.ptp(pts[bins == values[j], 1-axis]) < .75:continue
                usable.append((counts[j], val if val < p[axis] else val - size))
            if not usable:return False
            _, lo = max(usable)
            walls.extend([lo, lo + size])
        self.bounds=walls;x0,x1,y0,y1=walls;s=self.cfg['wall_standoff']
        near=[(p[0]-x0,(x0+s,p[1]),3),(x1-p[0],(x1-s,p[1]),1),(p[1]-y0,(p[0],y0+s),0),(y1-p[1],(p[0],y1-s),2)]
        _,self.goal,side=min(near)
        corners=[(x0+s,y0+s),(x1-s,y0+s),(x1-s,y1-s),(x0+s,y1-s)]
        # CCW perimeter starting with the next corner along the nearest side.
        first={0:1,1:2,2:3,3:0}[side]
        self.route=[corners[(first+i)%4] for i in range(4)]+[self.goal]
        self.patrol_route = tuple(self.route)
        self.state='APPROACH_WALL';self.event('room_estimated',bounds=walls,first_goal=self.goal)
        return True
    def begin_spin(self,after):
        self.after_spin=after;self.state='SCAN_360';self.spin_total=0;self.last_yaw=self.pose[2]
        self.event('scan_begin',distance_m=self.distance);self.distance=0
    def begin_marker_visits(self):
        if self.visit_ids is not None:
            return False
        confirmed = [key for key, marker in self.markers.items() if marker['observations'] >= 2]
        if len(confirmed) < self.expected_targets:
            return False
        if self.state == 'EXPLORE_INTERIOR' and not self.transport.stop_exploration():
            return True
        self.visit_ids = confirmed[:self.expected_targets]
        self.command()  # Cancel the current Nav2 action before the first visit.
        self.event('marker_search_complete', ids=[int(key) for key in self.visit_ids])
        self.next_marker()
        return True

    def finish_perimeter(self):
        self.event('patrol_lap_complete', confirmed=sum(m['observations'] >= 2 for m in self.markers.values()))
        if self.interior_exploration:
            self.state = 'EXPLORE_INTERIOR'
            self.goal = None
            self.event('interior_exploration_begin', algorithm='rust_base')
        else:
            self.route = list(self.patrol_route)
            self.goal = self.route.pop(0)
            self.begin_spin('PATROL')

    def follow_wall(self, scan, pose, now):
        survey = self.wall_survey
        already_there = False
        if self.goal is None:
            if now < self.next_wall_plan:
                return
            self.next_wall_plan = now + .5
            step = survey.step(scan, pose)
            if step is not None:
                goal = (step.target if math.dist(pose[:2], step.target) <= .25
                        else self.grid.reachable_goal(pose[:2], step.target))
                if goal is not None:
                    self.goal = goal
                    self.wall_step = step if goal == step.target else None
                    already_there = math.dist(pose[:2], goal) <= .25
                    self.event('wall_navigation_goal', goal=goal)
            if self.goal is None:
                if now - self.wait_print > 5:
                    self.event('waiting_for_wall_path', wall_step_found=step is not None,
                               robot_xy=pose[:2], map_extent=self.grid.extent(),
                               robot_cell=self.grid.cell_status(pose[:2]),
                               wall_target=step.target if step is not None else None)
                    self.wait_print = now
                return

        if not already_there and not self.transport.navigate(self.goal, pose):
            return
        self.command()
        if self.wall_step is not None:
            step = self.wall_step
            first = survey.home is None
            survey.reached(step)
            if first:
                self.event('wall_follow_started', start=survey.home)
            if step.corner is not None:
                self.event('wall_corner_reached', corner=step.corner, count=len(survey.corners))
        self.goal = self.wall_step = None
        self.next_wall_plan = 0.0
        if survey.complete:
            self.bounds = survey.bounds()
            self.event('room_mapped_from_perimeter', bounds=self.bounds)
            self.finish_perimeter()

    def next_marker(self):
        candidates = [self.markers[key] for key in self.visit_ids if key not in self.visited]
        if not candidates:
            self.state = 'DONE'
            self.status = 'completed'
            self.event('mission_finished', visited=sorted(self.visited), detected=list(self.markers), status=self.status)
            return
        m=min(candidates,key=lambda m:math.hypot(m['x']-self.pose[0],m['y']-self.pose[1]));self.target=m
        d = self.cfg['marker_standoff']
        front_points = [(m['x'] + (d + extra) * m['normal_x'], m['y'] + (d + extra) * m['normal_y'])
                        for extra in (0.0, 0.2, 0.4, 0.6)]
        grid = self.grid
        blocked = grid.blocked()
        self.goal = front_points[0]
        for point in front_points:
            x, y = grid.cell(*point)
            if 0 <= x < grid.n and 0 <= y < grid.n and not blocked[y, x]:
                self.goal = point
                break
        self.state='VISIT_MARKER';self.path=[];self.event('visit_begin',id=m['id'],goal=self.goal)
    def tick(self):
        now=time.monotonic()
        with self.lock:p=self.pose;scan=self.scan;pair=self.image;fresh=min(self.odom_seen,self.scan_seen);t=self.odom_t
        if p is None or scan is None or now-fresh>2:
            self.command()
            if now-self.wait_print>5:self.event('waiting_for_sensors',odom=p is not None,lidar=scan is not None);self.wait_print=now
            if now-self.start_wall>getattr(self.args,'timeout',30) and (p is None or scan is None):
                raise RuntimeError('Timed out waiting for mission inputs; check the sensor and navigation logs')
            return
        if self.first_sim_time is None:self.first_sim_time=t
        self.detect(pair)
        if not self.external_map and t-self.last_map>.15:self.grid.update(scan,p);self.last_map=t
        if t-self.last_save>5:self.save();self.last_save=t
        if self.begin_marker_visits():
            return
        if self.state == 'WAIT_SENSORS' and self.wall_survey is not None:
            self.state = 'WALL_SURVEY'
        if self.state=='WAIT_SENSORS':
            if not self.initialize(scan,p):
                self.command()
                if now-self.wait_print > 5:
                    self.event('waiting_for_room', reason='Need lidar returns from walls on both room axes')
                    self.wait_print = now
                return
        if self.last_pose:
            delta=math.hypot(p[0]-self.last_pose[0],p[1]-self.last_pose[1])
            self.total_distance+=delta;self.total_rotation+=abs(wrap(p[2]-self.last_pose[2]))
            if (self.state in ('APPROACH_WALL', 'PATROL')
                    or (self.state == 'WALL_SURVEY' and self.wall_survey.home is not None)):
                self.distance += delta
        self.last_pose=p
        if self.state == 'WALL_SURVEY':
            if self.distance >= self.cfg['scan_every_m']:
                self.begin_spin('WALL_SURVEY')
                self.command()
            else:
                self.follow_wall(scan, p, now)
            return
        if self.state == 'EXPLORE_INTERIOR':
            self.transport.start_exploration()
            return
        if self.state=='SCAN_360':
            step=wrap(p[2]-self.last_yaw);self.spin_total+=step;self.last_yaw=p[2]
            spin_finished = self.transport.turn(TAU, 'scan') if self.nav2 else self.spin_total>=TAU-.04
            if spin_finished:
                self.command();self.state=self.after_spin;self.path=[];self.event('scan_finished',radians=self.spin_total)
            elif not self.nav2:self.command(0,.55)
            return
        if self.state=='DONE':self.command();return
        if self.state=='FACE_MARKER':
            err=wrap(math.atan2(self.target['y']-p[1], self.target['x']-p[0])-p[2])
            aligned = self.transport.turn(err, 'face_marker') if self.nav2 and abs(err)>=.06 else abs(err)<.06
            if aligned:
                self.command();self.visited.add(str(self.target['id']));self.event('marker_visited',id=self.target['id']);self.show_markers();self.next_marker()
            elif not self.nav2:self.command(0,float(np.clip(1.5*err,-.5,.5)))
            return
        if self.state=='PATROL' and not self.interior_exploration and self.distance>=self.cfg['scan_every_m']:
            self.begin_spin('PATROL');self.command();return
        distance=math.hypot(self.goal[0]-p[0],self.goal[1]-p[1])
        reached = self.transport.navigate(self.goal, p) if self.nav2 else distance<.18
        if reached:
            self.command();self.path=[]
            if self.state=='APPROACH_WALL':self.state='PATROL';self.goal=self.route.pop(0);self.event('wall_reached')
            elif self.state=='PATROL':
                self.event('waypoint_reached',goal=self.goal)
                if self.route:self.goal=self.route.pop(0)
                else:self.finish_perimeter()
            elif self.state=='VISIT_MARKER':self.state='FACE_MARKER'
            return
        if self.nav2:return
        if t-self.last_plan>1 or not self.path:
            self.path=self.grid.plan(p[:2],self.goal,self.bounds);self.last_plan=t
        if not self.path:
            self.command();
            if self.blocked_since is None:self.blocked_since=now
            if now-self.blocked_since>30:
                self.status='blocked';self.running=False;self.event('mission_blocked',goal=self.goal);return
            if now-self.wait_print>5:self.event('goal_blocked',goal=self.goal);self.wait_print=now
            return
        self.blocked_since=None
        while len(self.path)>1 and math.hypot(self.path[0][0]-p[0],self.path[0][1]-p[1])<.3:self.path.pop(0)
        gx,gy=self.path[0];err=wrap(math.atan2(gy-p[1],gx-p[0])-p[2])
        front=[r for i,r in enumerate(scan.ranges) if abs(wrap(scan.angle_min+i*scan.angle_step))<.3 and math.isfinite(r)]
        v=min(.28,distance)*max(0,math.cos(err)) if abs(err)<.65 else 0
        if front and min(front)<.40:v=0
        self.command(v,float(np.clip(2*err,-.6,.6)))
    def run(self):
        try:
            while self.running and self.state!='DONE':
                self.tick();time.sleep(.05)
                if getattr(self.args,'max_sim_seconds',0) and self.first_sim_time is not None and self.odom_t-self.first_sim_time>=self.args.max_sim_seconds:
                    self.status='time_limit';self.event('sim_time_limit');break
                if self.args.max_seconds and time.monotonic()-self.start_wall>self.args.max_seconds:
                    self.status='time_limit';self.event('time_limit');break
        except Exception as error:
            self.status='error'
            self.event('mission_error', state=self.state, goal=self.goal, error=str(error))
            raise
        finally:
            if not self.running and self.status=='running':self.status='interrupted'
            for _ in range(3):self.command();time.sleep(.05)
            self.save();self.events.close()
            self.transport.close()
            time.sleep(.2)
            self.detector.close()

if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('--version',choices=['v1'],default='v1');ap.add_argument('--max-sim-seconds',type=float,default=600);ap.add_argument('--max-seconds',type=float,default=0);args=ap.parse_args()
    mission=Mission(args)
    signal.signal(signal.SIGINT,lambda *_:setattr(mission,'running',False))
    signal.signal(signal.SIGTERM,lambda *_:setattr(mission,'running',False))
    mission.run()
