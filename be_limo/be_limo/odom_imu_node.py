"""Odométrie corrigée par le gyroscope (robot réel).

Entrées   /odom        nav_msgs/Odometry  (limo_base : bonne vitesse, cap faux)
          /limo/imu    sensor_msgs/Imu    (paramètre imu_topic)
Sorties   /odom_imu    nav_msgs/Odometry  pose corrigée, repère "odom_imu"
          TF odom_imu -> odom

limo_base publie déjà le TF odom -> base_link (avec le cap faux) et on ne
modifie pas le driver du labo. On ajoute donc un repère au-dessus :
odom_imu -> odom = pose_corrigée ∘ pose_driver⁻¹, si bien que
odom_imu -> base_link est la pose corrigée. AMCL, slam_toolbox et
localization_node utilisent "odom_imu" comme repère d'odométrie.
"""

import time

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster

from be_limo.frames import compose, inverse
from be_limo.odom_imu import GyroOdometry
from be_limo.ros_utils import SENSOR, quaternion_from_yaw, run, yaw_from_quaternion


class OdomImu(Node):

    def __init__(self):
        super().__init__('odom_imu')
        d = self.declare_parameter
        d('imu_topic', '/limo/imu')
        d('odom_topic', '/odom')
        d('frame', 'odom_imu')
        d('gyro_scale', 1.0)          # à étalonner : un tour complet en roulant, cap final = 360°
        d('publish_rate', 50.0)       # Hz max de /odom_imu et du TF (le gyroscope est intégré à chaque mesure)
        gp = lambda name: self.get_parameter(name).value
        self.frame = gp('frame')
        self.odo = GyroOdometry(gyro_scale=gp('gyro_scale'))
        self.driver_pose = None
        self.driver_frame = 'odom'
        self.tf = TransformBroadcaster(self)
        self.pub = self.create_publisher(Odometry, '/odom_imu', 20)
        self.create_subscription(Odometry, gp('odom_topic'), self.on_odom, 20)
        self.create_subscription(Imu, gp('imu_topic'), self.on_imu, SENSOR)
        self.got_imu = False
        self.last_pub = 0.0
        self.pub_period = 1.0 / gp('publish_rate')     # lu une fois (get_parameter est lent)
        self.create_timer(3.0, self.check_imu)
        self.get_logger().info('Odométrie corrigée : cap du gyroscope %s' % gp('imu_topic'))

    def check_imu(self):
        if not self.got_imu:
            self.get_logger().warn('Pas de gyroscope sur %s : /odom_imu reste immobile en rotation'
                                   % self.get_parameter('imu_topic').value, throttle_duration_sec=30.0)

    def on_odom(self, msg):
        p = msg.pose.pose
        self.driver_pose = (p.position.x, p.position.y, yaw_from_quaternion(p.orientation))
        self.driver_frame = msg.header.frame_id or 'odom'
        self.driver_stamp = msg.header.stamp
        self.odo.update_speed(time.monotonic(), msg.twist.twist.linear.x)

    def on_imu(self, msg):
        self.got_imu = True
        now = time.monotonic()
        pose = self.odo.update_gyro(now, msg.angular_velocity.z)
        if self.driver_pose is None or now - self.last_pub < self.pub_period:
            return
        self.last_pub = now
        stamp = self.driver_stamp
        out = Odometry()
        out.header.stamp = stamp
        out.header.frame_id = self.frame
        out.child_frame_id = 'base_link'
        out.pose.pose.position.x, out.pose.pose.position.y = pose[0], pose[1]
        q = quaternion_from_yaw(pose[2])
        o = out.pose.pose.orientation
        o.x, o.y, o.z, o.w = q
        out.twist.twist.linear.x = self.odo.v
        out.twist.twist.angular.z = self.odo.omega
        self.pub.publish(out)

        corr = compose(pose, inverse(self.driver_pose))
        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = self.frame
        tf.child_frame_id = self.driver_frame
        tf.transform.translation.x, tf.transform.translation.y = corr[0], corr[1]
        q = quaternion_from_yaw(corr[2])
        r = tf.transform.rotation
        r.x, r.y, r.z, r.w = q
        self.tf.sendTransform(tf)


def main(args=None):
    run(OdomImu, args)
