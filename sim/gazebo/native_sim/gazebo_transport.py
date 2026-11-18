"""Gazebo-only imports live here so the mission can also run on a ROS-only host."""
from gz.transport13 import Node
from gz.msgs10.twist_pb2 import Twist
from gz.msgs10.odometry_pb2 import Odometry
from gz.msgs10.laserscan_pb2 import LaserScan
from gz.msgs10.image_pb2 import Image

class GazeboTransport:
    def start(self,mission):
        self.node=Node();self.pub=self.node.advertise('/model/turtlebot4/cmd_vel',Twist)
        self.topics=[]
        for cls,topic,cb in [(Odometry,'/model/turtlebot4/ideal_odometry',mission.odom_cb),(LaserScan,'/turtlebot4/scan',mission.scan_cb),(Image,'/turtlebot4/oakd/rgb/image',mission.image_cb)]:
            if not self.node.subscribe(cls,topic,cb):raise RuntimeError('Cannot subscribe '+topic)
            self.topics.append(topic)
    def publish(self,v,w):
        msg=Twist();msg.linear.x=v;msg.angular.z=w;self.pub.publish(msg)
    def close(self):
        for topic in self.topics:self.node.unsubscribe(topic)
