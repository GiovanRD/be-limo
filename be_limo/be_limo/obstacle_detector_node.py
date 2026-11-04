"""Lidar obstacle detection.

Inputs
  /scan      sensor_msgs/LaserScan
  /cmd_vel   geometry_msgs/Twist      current curvature of the robot (omega / v)
Outputs
  /obstacle_distance  std_msgs/Float32            free distance in the corridor
                                                  (inf when the way is clear)
  /obstacles          geometry_msgs/PoseArray     obstacles in the corridor (base_link)
  /obstacles/markers  visualization_msgs/MarkerArray   corridor and obstacles for RViz
"""

import math

from geometry_msgs.msg import Point, Pose, PoseArray, Twist
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker, MarkerArray

from be_limo.obstacles import ObstacleParams, detect, scan_to_points
from be_limo.ros_utils import SENSOR, run


class ObstacleDetector(Node):

    def __init__(self):
        super().__init__('obstacle_detector')
        self.declare_parameter('half_width', 0.16)
        self.declare_parameter('max_range', 1.5)
        self.declare_parameter('laser_x', 0.1236)
        self.declare_parameter('laser_yaw', 0.0)
        self.declare_parameter('base_frame', 'base_link')
        gp = self.get_parameter
        self.p = ObstacleParams(laser_x=gp('laser_x').value, laser_yaw=gp('laser_yaw').value,
                                half_width=gp('half_width').value,
                                max_range=gp('max_range').value)
        self.curvature = 0.0
        self.dist_pub = self.create_publisher(Float32, '/obstacle_distance', 10)
        self.obs_pub = self.create_publisher(PoseArray, '/obstacles', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/obstacles/markers', 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, SENSOR)
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 10)

    def on_cmd(self, msg):
        v = msg.linear.x
        self.curvature = msg.angular.z / v if abs(v) > 0.02 else 0.0

    def on_scan(self, msg):
        pts = scan_to_points(msg.ranges, msg.angle_min, msg.angle_increment, self.p,
                             msg.range_min, msg.range_max)
        report = detect(pts, self.curvature, self.p)
        self.dist_pub.publish(Float32(data=float(report.distance)))

        frame = self.get_parameter('base_frame').value
        arr = PoseArray()
        arr.header.stamp, arr.header.frame_id = msg.header.stamp, frame
        for ob in report.in_corridor:
            pose = Pose()
            pose.position.x, pose.position.y = ob.x, ob.y
            pose.orientation.w = 1.0
            arr.poses.append(pose)
        self.obs_pub.publish(arr)
        self.publish_markers(msg.header.stamp, frame, report)

    def publish_markers(self, stamp, frame, report):
        markers = MarkerArray()
        corridor = Marker()
        corridor.header.stamp, corridor.header.frame_id = stamp, frame
        corridor.ns, corridor.id, corridor.type = 'corridor', 0, Marker.LINE_STRIP
        corridor.scale.x = 0.008
        blocked = math.isfinite(report.distance)
        corridor.color.r, corridor.color.g, corridor.color.a = (1.0, 0.2, 0.8) if blocked else (0.2, 1.0, 0.6)
        corridor.pose.orientation.w = 1.0
        length = min(report.distance, self.p.max_range)
        k = self.curvature
        for side in (1, -1):
            n = 12
            seq = range(n + 1) if side == 1 else range(n, -1, -1)
            for i in seq:
                s = length * i / n
                if abs(k) < 1e-3:
                    x, y, th = s, 0.0, 0.0
                else:
                    th = s * k
                    x, y = math.sin(th) / k, (1 - math.cos(th)) / k
                off = side * self.p.half_width
                corridor.points.append(Point(x=x - off * math.sin(th), y=y + off * math.cos(th), z=0.02))
        corridor.points.append(corridor.points[0])
        markers.markers.append(corridor)
        for i, ob in enumerate(report.in_corridor[:10]):
            m = Marker()
            m.header = corridor.header
            m.ns, m.id, m.type = 'obstacles', i, Marker.CYLINDER
            m.pose.position.x, m.pose.position.y, m.pose.position.z = ob.x, ob.y, 0.05
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = max(0.05, ob.size)
            m.scale.z = 0.1
            m.color.r, m.color.a = 1.0, 0.8
            m.lifetime.nanosec = 300_000_000
            markers.markers.append(m)
        self.marker_pub.publish(markers)


def main(args=None):
    run(ObstacleDetector, args)
