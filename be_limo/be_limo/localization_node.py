"""Localisation node: robot pose on the track map.

/odom gives the pose in the odometry frame. In Gazebo, the LIMO plugin
publishes the pose in the world frame, which is the map frame of the track
(parameter odom_is_world = true). On the real robot, odometry starts at
(0, 0, 0) where the robot is switched on: the node composes it with the
initial pose of the robot on the track, given as a road-map node
(initial_node) or with RViz "2D Pose Estimate" (/initialpose).

Publishes /robot_pose (PoseStamped, frame "map") and the TF map -> odom.
"""

import math

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from be_limo.ros_utils import quaternion_from_yaw, run, yaw_from_quaternion
from be_limo.roadmap import node_poses


def compose(a, b):
    """Pose a (x, y, yaw) followed by pose b expressed in a."""
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1],
            math.atan2(math.sin(a[2] + b[2]), math.cos(a[2] + b[2])))


def inverse(a):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (-c * a[0] - s * a[1], s * a[0] - c * a[1], -a[2])


class Localization(Node):

    def __init__(self):
        super().__init__('localization_node')
        self.declare_parameter('odom_is_world', True)
        self.declare_parameter('initial_node', -1)
        self.declare_parameter('initial_pose', [0.0, 0.0, 0.0])
        self.declare_parameter('publish_tf', True)

        if self.get_parameter('odom_is_world').value:
            initial = (0.0, 0.0, 0.0)
            self.offset = initial               # map -> odom
            self.waiting_first_odom = False
        else:
            node = self.get_parameter('initial_node').value
            if node >= 0:
                initial = node_poses()[node]
                self.get_logger().info('Pose initiale : noeud %d %s' % (node, str(initial)))
            else:
                initial = tuple(self.get_parameter('initial_pose').value)
            self.initial = initial
            self.offset = None                  # computed with the first odom
            self.waiting_first_odom = True

        self.odom_pose = None
        self.pub = self.create_publisher(PoseStamped, '/robot_pose', 10)
        self.tf = TransformBroadcaster(self) if self.get_parameter('publish_tf').value else None
        self.create_subscription(Odometry, '/odom', self.on_odom, 20)
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.on_initialpose, 10)

    def on_initialpose(self, msg):
        p = msg.pose.pose
        target = (p.position.x, p.position.y, yaw_from_quaternion(p.orientation))
        if self.odom_pose is None:
            self.initial, self.waiting_first_odom = target, True
            return
        # map -> odom such that the current odom pose lands on the target.
        self.offset = compose(target, inverse(self.odom_pose))
        self.get_logger().info('Pose réinitialisée : %.2f %.2f %.2f' % target)

    def on_odom(self, msg):
        p = msg.pose.pose
        self.odom_pose = (p.position.x, p.position.y, yaw_from_quaternion(p.orientation))
        if self.waiting_first_odom:
            self.offset = compose(self.initial, inverse(self.odom_pose))
            self.waiting_first_odom = False
        x, y, yaw = compose(self.offset, self.odom_pose)

        out = PoseStamped()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = 'map'
        out.pose.position.x, out.pose.position.y = x, y
        q = quaternion_from_yaw(yaw)
        (out.pose.orientation.x, out.pose.orientation.y,
         out.pose.orientation.z, out.pose.orientation.w) = q
        self.pub.publish(out)

        if self.tf is not None:
            t = TransformStamped()
            t.header.stamp = msg.header.stamp
            t.header.frame_id = 'map'
            t.child_frame_id = msg.header.frame_id or 'odom'
            t.transform.translation.x, t.transform.translation.y = self.offset[0], self.offset[1]
            q = quaternion_from_yaw(self.offset[2])
            (t.transform.rotation.x, t.transform.rotation.y,
             t.transform.rotation.z, t.transform.rotation.w) = q
            self.tf.sendTransform(t)


def main(args=None):
    run(Localization, args)
