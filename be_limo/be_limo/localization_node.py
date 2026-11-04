"""Localisation node: robot pose on the track map.

Two modes (parameter `mode`):

* "odom" (default). /odom gives the pose in the odometry frame.
  - Gazebo (odom_is_world = true): the LIMO plugin publishes the pose in the
    world frame, which is the track frame.
  - Real robot: odometry starts at (0, 0, 0) where the robot is switched on.
    The node composes it with the starting pose, given as a road-map node
    (initial_node) or with RViz "2D Pose Estimate" (/initialpose).
  Publishes the TF map -> odom.

* "lidar". AMCL (or slam_toolbox while mapping) localises the robot on the
  lidar map of the room, frame "room_map", and publishes room_map -> odom.
  The node reads room_map -> base_link and converts it into the track frame
  with the calibration saved when the room was mapped (see frames.py).
  Publishes the static TF map -> room_map, and gives AMCL its initial pose
  from initial_node or from RViz "2D Pose Estimate" (track frame).

In both modes: /robot_pose (geometry_msgs/PoseStamped, frame "map").
"""

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node

from be_limo.frames import calibration_path, compose, inverse, load_calibration, save_calibration
from be_limo.roadmap import node_poses
from be_limo.ros_utils import quaternion_from_yaw, run, yaw_from_quaternion

ROOM_FRAME = 'room_map'


def set_pose(msg_pose, pose):
    msg_pose.position.x, msg_pose.position.y = float(pose[0]), float(pose[1])
    q = quaternion_from_yaw(pose[2])
    o = msg_pose.orientation
    o.x, o.y, o.z, o.w = q


def make_transform(stamp, parent, child, pose):
    t = TransformStamped()
    t.header.stamp = stamp
    t.header.frame_id = parent
    t.child_frame_id = child
    t.transform.translation.x, t.transform.translation.y = float(pose[0]), float(pose[1])
    q = quaternion_from_yaw(pose[2])
    r = t.transform.rotation
    r.x, r.y, r.z, r.w = q
    return t


class Localization(Node):

    def __init__(self):
        super().__init__('localization_node')
        d = self.declare_parameter
        d('mode', 'odom')                   # 'odom' or 'lidar'
        d('odom_is_world', True)
        d('initial_node', -1)
        d('initial_pose', [0.0, 0.0, 0.0])
        d('publish_tf', True)
        # lidar mode
        d('map_yaml', '')                   # lidar map, e.g. ~/maps/salle207.yaml
        d('calibration_file', '')           # default: <map>_track.yaml
        d('save_calibration', False)        # mapping session: write the calibration
        d('send_initial_pose', True)        # give AMCL its initial pose
        d('rate', 20.0)

        self.pub = self.create_publisher(PoseStamped, '/robot_pose', 10)
        gp = lambda name: self.get_parameter(name).value
        self.mode = gp('mode')
        node = gp('initial_node')
        if node >= 0:
            self.initial = node_poses()[node]
            self.get_logger().info('Pose initiale : noeud %d (%.2f, %.2f, %.2f)' % ((node,) + self.initial))
        else:
            self.initial = tuple(gp('initial_pose'))

        if self.mode == 'lidar':
            self.setup_lidar(node)
        else:
            self.setup_odom()

    # ================================================================== odom
    def setup_odom(self):
        from tf2_ros import TransformBroadcaster
        if self.get_parameter('odom_is_world').value:
            self.offset, self.waiting_first_odom = (0.0, 0.0, 0.0), False     # map -> odom
        else:
            self.offset, self.waiting_first_odom = None, True
        self.odom_pose = None
        self.tf = TransformBroadcaster(self) if self.get_parameter('publish_tf').value else None
        self.create_subscription(Odometry, '/odom', self.on_odom, 20)
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.on_initialpose_odom, 10)

    def on_initialpose_odom(self, msg):
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
        self.publish_pose(msg.header.stamp, compose(self.offset, self.odom_pose))
        if self.tf is not None:
            self.tf.sendTransform(make_transform(msg.header.stamp, 'map',
                                                 msg.header.frame_id or 'odom', self.offset))

    # ================================================================= lidar
    def setup_lidar(self, node):
        from tf2_ros.buffer import Buffer
        from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
        from tf2_ros.transform_listener import TransformListener

        gp = lambda name: self.get_parameter(name).value
        path = gp('calibration_file') or (calibration_path(gp('map_yaml')) if gp('map_yaml') else '')
        if gp('save_calibration'):
            # Mapping session: room_map starts at the robot, i.e. at the start node.
            if not path:
                raise RuntimeError('map_yaml ou calibration_file est nécessaire')
            if node < 0:
                raise RuntimeError('initial_node est nécessaire pour la cartographie')
            save_calibration(path, self.initial, node)
            self.get_logger().info('Calibration enregistrée : %s' % path)
        try:
            self.room_in_track = load_calibration(path)
        except (OSError, KeyError, ValueError) as exc:
            raise RuntimeError('Calibration introuvable (%s) : refaire la carte avec '
                               'mapping.launch.py. %s' % (path, exc))
        self.get_logger().info('Carte lidar -> piste : x=%.3f y=%.3f yaw=%.3f' % self.room_in_track)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.static_tf = StaticTransformBroadcaster(self)
        self.static_tf.sendTransform(make_transform(self.get_clock().now().to_msg(), 'map',
                                                    ROOM_FRAME, self.room_in_track))
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.on_initialpose_lidar, 10)
        self.initial_sent = 0
        self.localised = False
        self.warned = False
        self.create_timer(1.0 / gp('rate'), self.on_timer)
        if gp('send_initial_pose') and gp('initial_node') >= 0:
            self.create_timer(1.0, self.send_initial_pose)

    def to_room(self, track_pose):
        return compose(inverse(self.room_in_track), track_pose)

    def publish_initial(self, track_pose):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = ROOM_FRAME
        set_pose(msg.pose.pose, self.to_room(track_pose))
        cov = [0.0] * 36
        cov[0] = cov[7] = 0.05 ** 2          # 5 cm
        cov[35] = 0.1 ** 2                   # 6 degrees
        msg.pose.covariance = cov
        self.initial_pose_pub.publish(msg)

    def send_initial_pose(self):
        # AMCL starts after this node and only localises once it has an
        # initial pose: send it every second until the localisation is up.
        if not self.localised and self.initial_sent < 30:
            self.publish_initial(self.initial)
            self.initial_sent += 1
            if self.initial_sent == 1:
                self.get_logger().info('Pose initiale envoyée à AMCL')

    def on_initialpose_lidar(self, msg):
        if msg.header.frame_id == ROOM_FRAME:
            return                            # already for AMCL (sent by this node)
        p = msg.pose.pose
        target = (p.position.x, p.position.y, yaw_from_quaternion(p.orientation))
        self.get_logger().info('Pose donnée dans RViz : %.2f %.2f %.2f -> AMCL' % target)
        self.publish_initial(target)

    def on_timer(self):
        from rclpy.time import Time
        try:
            t = self.tf_buffer.lookup_transform(ROOM_FRAME, 'base_link', Time())
        except Exception as exc:  # TF not available yet
            if not self.warned:
                self.get_logger().info('En attente de la localisation lidar (%s)...' % type(exc).__name__)
                self.warned = True
            return
        tr, q = t.transform.translation, t.transform.rotation
        pose = compose(self.room_in_track, (tr.x, tr.y, yaw_from_quaternion(q)))
        if not self.localised:
            self.localised = True
            self.get_logger().info('Localisé par le lidar : x=%.2f y=%.2f' % pose[:2])
        self.publish_pose(t.header.stamp, pose)

    # ================================================================ common
    def publish_pose(self, stamp, pose):
        out = PoseStamped()
        out.header.stamp = stamp
        out.header.frame_id = 'map'
        set_pose(out.pose, pose)
        self.pub.publish(out)


def main(args=None):
    run(Localization, args)
