"""ROS-free Gazebo perimeter mapping mission. Sensor input only; no world-pose lookup."""
import argparse,hashlib,heapq,json,math,signal,threading,time
from pathlib import Path
import cv2
import numpy as np
from rust_detector import RustDetector
from rust_explorer import RustExplorer
from gz.transport13 import Node
from gz.msgs10.twist_pb2 import Twist
from gz.msgs10.odometry_pb2 import Odometry
from gz.msgs10.laserscan_pb2 import LaserScan
from gz.msgs10.image_pb2 import Image

ROOT=Path(__file__).resolve().parent
TAU=2*math.pi

def wrap(a):return (a+math.pi)%TAU-math.pi

def yaw(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))

def stamp(m):return m.header.stamp.sec+m.header.stamp.nsec*1e-9

class Grid:
    def __init__(self):
        self.res=.1;self.origin=(-25.,-25.);self.n=600
        self.log=np.zeros((self.n,self.n),np.float32)
    def cell(self,x,y):return (int((x-self.origin[0])/self.res),int((y-self.origin[1])/self.res))
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

class Mission:
    def __init__(self,args):
        self.version=getattr(args,'version','v1')
        self.args=args;self.cfg=json.loads((ROOT/'config.json').read_text())
        self.out=ROOT/'runs'/(time.strftime('%Y%m%d-%H%M%S')+'-'+self.version);self.out.mkdir(parents=True)
        (self.out/'run_info.json').write_text(json.dumps({'version':self.version,'world_sha256':hashlib.sha256((ROOT/'track1_native.sdf').read_bytes()).hexdigest(),'config':self.cfg,'max_sim_seconds':getattr(args,'max_sim_seconds',600)},indent=2))
        self.events=(self.out/'events.jsonl').open('w');self.lock=threading.Lock()
        self.pose=None;self.scan=None;self.image=None;self.odom_seen=0;self.scan_seen=0
        self.odom_t=0.;self.grid=Grid();self.markers={};self.visited=set();self.running=True
        self.detector=RustDetector()
        self.explorer=RustExplorer() if self.version=='v2' else None
        self.v2_blacklist=[];self.v2_views=[];self.v2_candidates=[];self.v2_update=-100;self.v2_empty=0
        self.total_distance=0.;self.total_rotation=0.;self.first_sim_time=None
        self.node=Node();self.pub=self.node.advertise('/model/turtlebot4/cmd_vel',Twist)
        for cls,topic,cb in [(Odometry,'/model/turtlebot4/ideal_odometry',self.odom_cb),(LaserScan,'/turtlebot4/scan',self.scan_cb),(Image,'/turtlebot4/oakd/rgb/image',self.image_cb)]:
            if not self.node.subscribe(cls,topic,cb):raise RuntimeError('Cannot subscribe '+topic)
        self.state='WAIT_SENSORS';self.bounds=None;self.route=[];self.goal=None;self.path=[]
        self.last_pose=None;self.distance=0.;self.spin_total=0.;self.last_yaw=None;self.after_spin='PATROL'
        self.last_plan=-100;self.last_map=-100;self.last_save=-100;self.last_image=-100;self.status='running'
        self.start_wall=time.monotonic();self.last_msg=time.monotonic();self.wait_print=0;self.blocked_since=None
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
        msg=Twist();msg.linear.x=v;msg.angular.z=w;self.pub.publish(msg)
    def save(self):
        self.grid.save(self.out)
        (self.out/'markers.json').write_text(json.dumps({'frame':'gazebo_world','dictionary':self.cfg['dictionary'],'markers':self.markers,'visited':sorted(self.visited)},indent=2))
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
        f=m.width/(2*math.tan(1.2/2));K=np.array([[f,0,m.width/2],[0,f,m.height/2],[0,0,1]],np.float64)
        for mid,c in zip(ids.flatten(),corners):
            size=self.cfg['marker_sizes'].get(str(mid))
            if size is None:continue
            edges=np.linalg.norm(np.roll(c.reshape(4,2),-1,axis=0)-c.reshape(4,2),axis=1)
            if min(edges)<18 or max(edges)/min(edges)>3:continue
            s=size/2;obj=np.array([[-s,s,0],[s,s,0],[s,-s,0],[-s,-s,0]],np.float32)
            ok,rv,tv=cv2.solvePnP(obj,c.reshape(4,2),K,None,flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if not ok or tv[2,0]<=0:continue
            projected,_=cv2.projectPoints(obj,rv,tv,K,None)
            if np.mean(np.linalg.norm(projected.reshape(4,2)-c.reshape(4,2),axis=1))>1.5:continue
            R,_=cv2.Rodrigues(rv);n=R[:,2]
            if np.dot(n,-tv[:,0])<0:n=-n
            # camera optical (right,down,forward) -> robot (forward,left,up)
            dx,dy=float(tv[2,0])+.13,-float(tv[0,0]);ca,sa=math.cos(pose[2]),math.sin(pose[2])
            mx,my=pose[0]+ca*dx-sa*dy,pose[1]+sa*dx+ca*dy
            nx,ny=ca*n[2]+sa*n[0],sa*n[2]-ca*n[0];norm=math.hypot(nx,ny)
            if norm<.5:continue
            nx,ny=nx/norm,ny/norm
            if self.bounds and not(self.bounds[0]-.2<mx<self.bounds[1]+.2 and self.bounds[2]-.2<my<self.bounds[3]+.2):continue
            old=self.markers.get(str(mid));count=1
            if old:
                if math.hypot(mx-old['x'],my-old['y'])>1:continue
                count=old['observations']+1;weight=1/min(count,20)
                mx=old['x']*(1-weight)+mx*weight;my=old['y']*(1-weight)+my*weight
            self.markers[str(mid)]={'id':int(mid),'x':mx,'y':my,'normal_x':float(nx),'normal_y':float(ny),'size_m':size,'observations':count}
            if not old:self.event('marker_detected',id=int(mid),x=mx,y=my)
            temporary=self.out/'markers.json.tmp'
            temporary.write_text(json.dumps({'frame':'gazebo_world','dictionary':self.cfg['dictionary'],'markers':self.markers,'visited':sorted(self.visited)},indent=2));temporary.replace(self.out/'markers.json')
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
            order=np.argsort(counts)[::-1];pairs=[]
            for j in order[:15]:
                if counts[j]<5:continue
                val=values[j]*.1
                if abs(val-p[axis])<1:continue
                lo=val if val<p[axis] else val-size
                pairs.append((counts[j],lo,lo+size))
            if not pairs:return False
            _,lo,hi=max(pairs);walls.extend([lo,hi])
        self.bounds=walls;x0,x1,y0,y1=walls;s=self.cfg['wall_standoff']
        near=[(p[0]-x0,(x0+s,p[1]),3),(x1-p[0],(x1-s,p[1]),1),(p[1]-y0,(p[0],y0+s),0),(y1-p[1],(p[0],y1-s),2)]
        _,self.goal,side=min(near)
        corners=[(x0+s,y0+s),(x1-s,y0+s),(x1-s,y1-s),(x0+s,y1-s)]
        # CCW perimeter starting with the next corner along the nearest side.
        first={0:1,1:2,2:3,3:0}[side]
        self.route=[corners[(first+i)%4] for i in range(4)]+[self.goal]
        self.state='APPROACH_WALL';self.event('room_estimated',bounds=walls,first_goal=self.goal)
        return True
    def begin_spin(self,after):
        self.after_spin=after;self.state='SCAN_360';self.spin_total=0;self.last_yaw=self.pose[2]
        self.event('scan_begin',distance_m=self.distance);self.distance=0
    def next_marker(self):
        candidates=[m for key,m in self.markers.items() if key not in self.visited and m['observations']>=2]
        if not candidates:
            self.state='DONE';self.status='completed' if len(self.visited)==len(self.cfg['marker_sizes']) else 'completed_with_missing_markers'
            self.event('mission_finished',visited=sorted(self.visited),detected=list(self.markers),status=self.status);return
        m=min(candidates,key=lambda m:math.hypot(m['x']-self.pose[0],m['y']-self.pose[1]));self.target=m
        d=self.cfg['marker_standoff'];self.goal=(m['x']+d*m['normal_x'],m['y']+d*m['normal_y'])
        self.state='VISIT_MARKER';self.path=[];self.event('visit_begin',id=m['id'],goal=self.goal)
    def tick(self):
        now=time.monotonic()
        with self.lock:p=self.pose;scan=self.scan;pair=self.image;fresh=min(self.odom_seen,self.scan_seen);t=self.odom_t
        if p is None or scan is None or now-fresh>2:
            self.command()
            if now-self.wait_print>5:self.event('waiting_for_sensors',odom=p is not None,lidar=scan is not None);self.wait_print=now
            if now-self.start_wall>30 and (p is None or scan is None):raise RuntimeError('Sensor timeout: ensure Gazebo server is running with -r and same GZ_PARTITION')
            return
        if self.first_sim_time is None:self.first_sim_time=t
        self.detect(pair)
        if t-self.last_map>.15:self.grid.update(scan,p);self.last_map=t
        if t-self.last_save>5:self.save();self.last_save=t
        if self.state=='WAIT_SENSORS':
            if self.version=='v1':
                if not self.initialize(scan,p):self.command();return
            else:
                self.state='V2_SELECT';self.event('exploration_started',policy='rust_base_frontier_coverage')
        if self.last_pose:
            delta=math.hypot(p[0]-self.last_pose[0],p[1]-self.last_pose[1])
            self.total_distance+=delta;self.total_rotation+=abs(wrap(p[2]-self.last_pose[2]))
            if self.state in ('APPROACH_WALL','PATROL'):self.distance+=delta
        self.last_pose=p
        if self.version=='v2' and self.state.startswith('V2_'):
            if self.v2_tick(p,t):return
        if self.state=='SCAN_360':
            step=wrap(p[2]-self.last_yaw);self.spin_total+=step;self.last_yaw=p[2]
            if self.spin_total>=TAU-.04:
                self.command();self.state=self.after_spin;self.path=[];self.event('scan_finished',radians=self.spin_total)
                if self.state=='SELECT_MARKER':self.next_marker()
            else:self.command(0,.55)
            return
        if self.state=='DONE':self.command();return
        if self.state=='FACE_MARKER':
            err=wrap(math.atan2(-self.target['normal_y'],-self.target['normal_x'])-p[2])
            if abs(err)<.06:
                self.command();self.visited.add(str(self.target['id']));self.event('marker_visited',id=self.target['id']);self.next_marker()
            else:self.command(0,float(np.clip(1.5*err,-.5,.5)))
            return
        if self.state=='PATROL' and self.distance>=self.cfg['scan_every_m']:
            self.begin_spin('PATROL');self.command();return
        distance=math.hypot(self.goal[0]-p[0],self.goal[1]-p[1])
        if distance<.18:
            self.command();self.path=[]
            if self.state=='APPROACH_WALL':self.state='PATROL';self.goal=self.route.pop(0);self.event('wall_reached')
            elif self.state=='PATROL':
                self.event('waypoint_reached',goal=self.goal)
                if self.route:self.goal=self.route.pop(0)
                else:self.begin_spin('SELECT_MARKER')
            elif self.state=='VISIT_MARKER':self.state='FACE_MARKER'
            elif self.state=='V2_DRIVE':self.state='V2_FACE'
            return
        if t-self.last_plan>1 or not self.path:
            self.path=self.grid.plan(p[:2],self.goal,self.bounds,known_only=self.state=='V2_DRIVE');self.last_plan=t
        if not self.path:
            self.command();
            if self.state=='V2_DRIVE':
                self.v2_reject(t,'no_path');return
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
    def v2_reject(self,t,reason):
        self.v2_blacklist.append((self.goal[0],self.goal[1],t+45))
        self.event('v2_goal_rejected',goal=self.goal,reason=reason)
        self.state='V2_SELECT';self.path=[];self.v2_update=-100
    def v2_tick(self,p,t):
        self.v2_blacklist=[b for b in self.v2_blacklist if b[2]>t]
        updated=False
        if t-self.v2_update>=1:
            updated=True
            blocked=[b[:2] for b in self.v2_blacklist]+self.v2_views
            self.v2_candidates=self.explorer.goals(self.grid,p,blocked);self.v2_update=t
        if self.state=='V2_SELECT':
            for g in self.v2_candidates:
                goal=(g['x'],g['y'])
                path=self.grid.plan(p[:2],goal,None,known_only=True)
                if not path:
                    self.v2_blacklist.append((*goal,t+45));continue
                self.goal=goal;self.v2_goal=g;self.path=path;self.state='V2_DRIVE'
                self.v2_goal_start=t;self.v2_progress=t;self.v2_best=math.hypot(goal[0]-p[0],goal[1]-p[1])
                self.v2_empty=0;self.event('v2_goal_selected',goal=goal,kind=g['kind'],yaw=g['yaw']);return False
            self.command()
            # Do not declare completion while failed goals are waiting to be retried.
            if not self.v2_blacklist and updated:
                self.v2_empty+=1
                if self.v2_empty>=3:
                    self.event('exploration_finished',policy='rust_base_frontier_coverage');self.next_marker()
            return True
        if self.state=='V2_FACE':
            err=wrap(self.v2_goal['yaw']-p[2])
            if abs(err)<.06:
                self.command();self.event('v2_goal_reached',goal=self.goal,kind=self.v2_goal['kind'])
                if self.v2_goal['kind']=='coverage':self.v2_views.append(self.goal)
                self.state='V2_SELECT';self.path=[];self.v2_update=-100
            else:self.command(0,float(np.clip(1.5*err,-.5,.5)))
            return True
        d=math.hypot(self.goal[0]-p[0],self.goal[1]-p[1])
        if d<self.v2_best-.1:self.v2_best=d;self.v2_progress=t
        if t-self.v2_goal_start>120 or t-self.v2_progress>30:
            self.command();self.v2_reject(t,'progress_timeout');return True
        return False
    def run(self):
        try:
            while self.running and self.state!='DONE':
                self.tick();time.sleep(.05)
                if getattr(self.args,'max_sim_seconds',0) and self.first_sim_time is not None and self.odom_t-self.first_sim_time>=self.args.max_sim_seconds:
                    self.status='time_limit';self.event('sim_time_limit');break
                if self.args.max_seconds and time.monotonic()-self.start_wall>self.args.max_seconds:
                    self.status='time_limit';self.event('time_limit');break
        except Exception:
            self.status='error'
            raise
        finally:
            if not self.running and self.status=='running':self.status='interrupted'
            for _ in range(3):self.command();time.sleep(.05)
            self.save();self.events.close()
            for topic in ['/model/turtlebot4/ideal_odometry','/turtlebot4/scan','/turtlebot4/oakd/rgb/image']:self.node.unsubscribe(topic)
            time.sleep(.2)
            self.detector.close()
            if self.explorer:self.explorer.close()

if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('--version',choices=['v1','v2'],default='v1');ap.add_argument('--max-sim-seconds',type=float,default=600);ap.add_argument('--max-seconds',type=float,default=0);args=ap.parse_args()
    mission=Mission(args)
    signal.signal(signal.SIGINT,lambda *_:setattr(mission,'running',False))
    signal.signal(signal.SIGTERM,lambda *_:setattr(mission,'running',False))
    mission.run()
