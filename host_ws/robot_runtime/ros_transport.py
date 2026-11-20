"""Real TurtleBot4 ROS 2 input adapter; no Gazebo dependency."""
import math,threading,time
from types import SimpleNamespace as NS
import cv2,numpy as np
from control import CommandGate,transform_matrix

MARKER_TOPIC = '/team7/v1/targets'
MARKER_SLOW_PERCENT = 65.0
MARKER_SLOW_HOLD = 2.5

class ROSAdapter:
    def __init__(self,args):
        self.args=args;self.mission=None;self.gate=CommandGate();self.gate.execute=args.execute
        self.info=None;self.latest_scan=None;self.latest_pose=None;self.errors={};self.closed=False
        self.lock=threading.Lock();self.last_graph_check=0.;self.conflict=False;self.motion=None;self.nav2_ready=False
        self.speed_limit_pub = None
        self.slow_until = self.last_speed_limit = 0.0
        self.marker_slow_active = False
    def start(self,mission):
        import rclpy
        from rclpy.node import Node
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import qos_profile_sensor_data,QoSProfile,ReliabilityPolicy,DurabilityPolicy
        from sensor_msgs.msg import LaserScan,CameraInfo,CompressedImage,Image
        from nav_msgs.msg import Odometry,OccupancyGrid
        from geometry_msgs.msg import Twist,TwistStamped
        from visualization_msgs.msg import Marker, MarkerArray
        from tf2_ros import Buffer,TransformListener
        self.rclpy=rclpy;self.mission=mission;self.node=Node('team7_robot_mission')
        self.tf=Buffer();self.listener=TransformListener(self.tf,self.node)
        if getattr(self.args,'nav2',False):
            from nav2_motion import Nav2Motion
            self.motion=Nav2Motion(self.node,self.args.map_frame)
        self.Twist=Twist;self.TwistStamped=TwistStamped
        self.pub=self.node.create_publisher(TwistStamped if self.args.cmd_type=='stamped' else Twist,self.args.cmd_topic,10) if self.args.execute and self.motion is None else None
        self.node.create_subscription(LaserScan,self.args.scan_topic,self.scan_cb,qos_profile_sensor_data)
        self.node.create_subscription(Odometry,self.args.odom_topic,self.odom_cb,qos_profile_sensor_data)
        self.node.create_subscription(CameraInfo,self.args.camera_info_topic,self.info_cb,qos_profile_sensor_data)
        self.node.create_subscription(CompressedImage if self.args.image_topic.endswith('/compressed') else Image,self.args.image_topic,self.image_cb,qos_profile_sensor_data)
        map_qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.node.create_subscription(OccupancyGrid,self.args.map_topic,self.map_cb,map_qos)
        if mission is not None:
            self.Marker, self.MarkerArray = Marker, MarkerArray
            self.marker_pub = self.node.create_publisher(MarkerArray, MARKER_TOPIC, map_qos)
            self.publish_markers({}, set())
            if self.motion is not None and self.args.marker_slowdown:
                from nav2_msgs.msg import SpeedLimit
                self.SpeedLimit = SpeedLimit
                self.speed_limit_pub = self.node.create_publisher(SpeedLimit, '/speed_limit', 10)
        self.node.create_timer(.05,self.timer)
        self.executor=SingleThreadedExecutor();self.executor.add_node(self.node)
        self.thread=threading.Thread(target=self.executor.spin,daemon=True);self.thread.start()
    def error(self,key,exception):self.errors[key]=str(exception)
    def publish_markers(self, markers, visited):
        stamp = self.node.get_clock().now().to_msg()
        clear = self.Marker()
        clear.header.frame_id = self.args.map_frame
        clear.header.stamp = stamp
        clear.action = self.Marker.DELETEALL
        result = [clear]
        for key, target in markers.items():
            confirmed = target['observations'] >= 2
            color = (0.25, 0.65, 1.0) if key in visited else (
                (0.15, 0.95, 0.30) if confirmed else (1.0, 0.70, 0.15))
            for label in (False, True):
                marker = self.Marker()
                marker.header.frame_id = self.args.map_frame
                marker.header.stamp = stamp
                marker.ns = 'v1_labels' if label else 'v1_markers'
                marker.id = target['id']
                marker.action = self.Marker.ADD
                marker.type = self.Marker.TEXT_VIEW_FACING if label else self.Marker.SPHERE
                marker.pose.position.x = target['x']
                marker.pose.position.y = target['y']
                marker.pose.position.z = 0.32 if label else 0.12
                marker.pose.orientation.w = 1.0
                marker.scale.x = marker.scale.y = marker.scale.z = 0.20
                marker.color.r, marker.color.g, marker.color.b = color
                marker.color.a = 1.0
                if label:
                    marker.text = f"ArUco {target['id']}" + ('' if confirmed else ' ?')
                result.append(marker)
        self.marker_pub.publish(self.MarkerArray(markers=result))

    def request_marker_slowdown(self):
        if self.speed_limit_pub is not None:
            self.slow_until = time.monotonic() + MARKER_SLOW_HOLD

    def update_marker_speed_limit(self, now):
        if self.speed_limit_pub is None:
            return
        active = not self.closed and now < self.slow_until
        if active != self.marker_slow_active or (active and now - self.last_speed_limit >= 1.0):
            limit = MARKER_SLOW_PERCENT if active else 0.0
            message = self.SpeedLimit(percentage=True, speed_limit=limit)
            self.speed_limit_pub.publish(message)
            self.marker_slow_active, self.last_speed_limit = active, now

    def age(self,header):
        then=header.stamp.sec+header.stamp.nanosec*1e-9
        now=self.node.get_clock().now().nanoseconds*1e-9
        return now-then
    def lookup(self,target,source,stamp=None):
        from rclpy.time import Time
        at=Time.from_msg(stamp) if stamp is not None else Time()
        tf=self.tf.lookup_transform(target,source,at)
        if stamp is None and self.age(tf.header)>2:raise ValueError('TF is stale')
        return transform_matrix(tf.transform.translation,tf.transform.rotation)
    def info_cb(self,m):
        if m.width and m.height and len(m.k)==9 and m.k[0]>0 and np.isfinite(m.k).all() and np.isfinite(m.d).all():
            if m.distortion_model not in ('plumb_bob','rational_polynomial',''):self.error('camera_info','Unsupported distortion model');return
            self.info=m
    def odom_cb(self,m):
        if -.2<=self.age(m.header)<.6 and np.isfinite([m.pose.pose.position.x,m.pose.pose.position.y]).all():
            self.gate.fresh['odom']=time.monotonic()
    def map_cb(self,m):
        try:
            if m.header.frame_id!=self.args.map_frame:raise ValueError('Map frame differs from --map-frame')
            # A latched map may retain its last scan stamp while the robot is stationary.
            # Use receipt freshness for maps; dynamic sensor and TF stamps remain checked.
            w,h=m.info.width,m.info.height
            if not(0<w<=1600 and 0<h<=1600 and .01<=m.info.resolution<=1):raise ValueError('Invalid map geometry')
            q=m.info.origin.orientation
            if not np.isfinite([q.x,q.y,q.z,q.w]).all() or abs(q.x)+abs(q.y)+abs(q.z)>.001 or abs(abs(q.w)-1)>.001:raise ValueError('Rotated occupancy grid unsupported')
            if not np.isfinite([m.info.origin.position.x,m.info.origin.position.y,m.info.resolution]).all():raise ValueError('Invalid map origin')
            data=np.array(m.data,dtype=np.int16).reshape(h,w)
            if np.any((data < -1)|(data>100)):raise ValueError('Invalid occupancy value')
            if self.mission:
                from mission import Grid
                g=Grid();g.n=max(w,h);g.res=m.info.resolution
                g.origin=(m.info.origin.position.x,m.info.origin.position.y)
                g.log=np.zeros((g.n,g.n),np.float32)
                log=g.log[:h,:w];log[(data>=0)&(data<50)]=-4;log[data>=65]=4
                self.mission.grid=g
            self.gate.fresh['map']=time.monotonic();self.errors.pop('map',None)
        except Exception as e:self.error('map',e)
    def scan_cb(self,m):
        try:
            if not(-.2<=self.age(m.header)<.6):raise ValueError('Stale/future lidar scan')
            if len(m.ranges)<30 or not math.isfinite(m.angle_min) or not math.isfinite(m.angle_increment) or m.angle_increment<=0:raise ValueError('Invalid scan')
            T=self.lookup(self.args.base_frame,m.header.frame_id,m.header.stamp)
            ranges=np.array(m.ranges,dtype=float);a=m.angle_min+np.arange(len(ranges))*m.angle_increment
            valid=np.isfinite(ranges)&(ranges>=m.range_min)&(ranges<=m.range_max)
            if np.count_nonzero(valid)<10:raise ValueError('Insufficient valid scan returns')
            local=np.vstack((ranges[valid]*np.cos(a[valid]),ranges[valid]*np.sin(a[valid]),np.zeros(valid.sum())))
            points=T[:3,:3]@local+T[:3,3:4]
            angles=np.arctan2(points[1],points[0]);distance=np.hypot(points[0],points[1])
            n=720;bins=np.clip(((angles+math.pi)/(2*math.pi)*n).astype(int),0,n-1)
            output=np.full(n,np.inf);np.minimum.at(output,bins,distance)
            # Unknown/no-return bins stay +inf; direct motion gate only trusts actual returns.
            self.gate.clearance=float(distance.min())
            front=distance[np.abs(angles)<.55];self.gate.front=float(front.min()) if len(front) else 0.
            self.latest_scan=NS(ranges=output.tolist(),angle_min=-math.pi,angle_step=2*math.pi/n,range_min=.05,range_max=float(m.range_max))
            self.gate.fresh['scan']=time.monotonic();self.errors.pop('scan',None)
        except Exception as e:self.gate.front=0.;self.gate.clearance=0.;self.error('scan',e)
    def image_cb(self,m):
        try:
            if self.info is None:raise ValueError('Waiting for camera calibration')
            if not(-.2<=self.age(m.header)<2):raise ValueError('Stale/future camera image')
            if hasattr(m,'format'):
                bgr=cv2.imdecode(np.frombuffer(bytes(m.data),np.uint8),cv2.IMREAD_COLOR)
                if bgr is None:raise ValueError('Cannot decode camera JPEG')
                rgb=cv2.cvtColor(bgr,cv2.COLOR_BGR2RGB)
            else:
                if m.encoding not in ('rgb8','bgr8','mono8'):raise ValueError('Unsupported image encoding')
                raw=np.frombuffer(bytes(m.data),np.uint8).reshape(m.height,m.step)
                if m.encoding=='mono8':rgb=cv2.cvtColor(raw[:,:m.width],cv2.COLOR_GRAY2RGB)
                else:
                    rgb=raw[:,:m.width*3].reshape(m.height,m.width,3)
                    if m.encoding=='bgr8':rgb=cv2.cvtColor(rgb,cv2.COLOR_BGR2RGB)
            info=self.info
            if rgb.shape[:2]!=(info.height,info.width):raise ValueError('Camera image/calibration size mismatch')
            optical=self.args.optical_frame or info.header.frame_id
            if not optical or (m.header.frame_id and m.header.frame_id!=optical):raise ValueError('Image frame is not the calibrated optical frame')
            T=self.lookup(self.args.map_frame,optical,m.header.stamp)
            self.gate.fresh['camera']=time.monotonic();self.errors.pop('camera',None)
            if self.mission and self.latest_pose:
                rgb=np.ascontiguousarray(rgb);normalized=NS(height=rgb.shape[0],width=rgb.shape[1],step=rgb.shape[1]*3,data=rgb.tobytes(),
                    header=NS(stamp=NS(sec=m.header.stamp.sec,nsec=m.header.stamp.nanosec)),
                    intrinsics=np.array(info.k,dtype=float).reshape(3,3),distortion=np.array(info.d,dtype=float),camera_to_map=T)
                self.mission.image_cb(normalized)
        except Exception as e:self.error('camera',e)
    def ready(self):
        now=time.monotonic()
        return not self.conflict and (self.motion is None or self.nav2_ready) and all(now-self.gate.fresh.get(key,-math.inf)<limit for key,limit in [('scan',.6),('odom',.6),('tf',.6),('map',15),('camera',2)])
    def timer(self):
        try:
            T=self.lookup(self.args.map_frame,self.args.base_frame)
            theta=math.atan2(T[1,0],T[0,0]);self.latest_pose=(float(T[0,3]),float(T[1,3]),theta)
            self.gate.fresh['tf']=time.monotonic();self.errors.pop('tf',None)
            if self.mission and self.latest_scan and self.ready():
                ns=self.node.get_clock().now().nanoseconds
                pose=NS(position=NS(x=self.latest_pose[0],y=self.latest_pose[1]),orientation=NS(w=math.cos(theta/2),x=0.,y=0.,z=math.sin(theta/2)))
                self.mission.odom_cb(NS(pose=pose,header=NS(stamp=NS(sec=ns//10**9,nsec=ns%10**9))))
                self.mission.scan_cb(self.latest_scan)
            if time.monotonic()-self.last_graph_check>1:
                self.last_graph_check=time.monotonic()
                if self.motion is not None:
                    guards=self.node.get_publishers_info_by_topic('/cmd_vel')
                    self.nav2_ready=self.motion.ready() and any(p.node_name=='velocity_guard' for p in guards)
                    self.conflict=any(p.node_name!='velocity_guard' for p in guards)
                    if not self.nav2_ready or self.conflict:
                        self.error('nav2','Waiting for Nav2 actions and the sole velocity_guard publisher')
                    else:
                        self.errors.pop('nav2',None)
                else:
                    names=self.node.get_publishers_info_by_topic(self.args.cmd_topic)
                    self.conflict=any(n.node_name!=self.node.get_name() for n in names)
                    if self.conflict:self.error('cmd','Another publisher owns command topic')
                    types=dict(self.node.get_topic_names_and_types()).get(self.args.cmd_topic,[])
                    expected='geometry_msgs/msg/TwistStamped' if self.args.cmd_type=='stamped' else 'geometry_msgs/msg/Twist'
                    if any(t!=expected for t in types):self.conflict=True;self.error('cmd','Command message type mismatch')
        except Exception as e:self.gate.fresh.pop('tf',None);self.error('tf',e)
        if self.motion is not None and not self.ready():
            self.motion.cancel()
        if not self.closed:
            self.update_marker_speed_limit(time.monotonic())
        if self.pub:
            v,w=self.gate.output() if not self.conflict else (0.,0.)
            self.emit(v,w)
    def emit(self,v,w):
        if self.args.cmd_type=='stamped':
            msg=self.TwistStamped();msg.header.stamp=self.node.get_clock().now().to_msg();msg.header.frame_id=self.args.base_frame;twist=msg.twist
        else:msg=self.Twist();twist=msg
        twist.linear.x=v;twist.angular.z=w;self.pub.publish(msg)
    def publish(self,v,w):
        if self.motion is None:
            self.gate.propose(v,w)
        else:
            self.motion.cancel()
    def navigate(self,goal,pose):
        return self.ready() and self.motion.navigate(goal,pose)
    def turn(self,angle,purpose):
        return self.ready() and self.motion.turn(angle,purpose)
    def close(self):
        if self.closed:return
        self.closed=True;self.gate.execute=False
        if self.motion is not None:self.motion.close()
        if getattr(self,'pub',None):
            for _ in range(3):self.emit(0.,0.);time.sleep(.05)
        if hasattr(self,'executor'):
            self.executor.shutdown(timeout_sec=2);self.thread.join(timeout=2)
        self.update_marker_speed_limit(time.monotonic())
        if hasattr(self,'node'):self.node.destroy_node()
