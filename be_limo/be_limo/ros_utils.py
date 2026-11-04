"""Small helpers shared by the ROS 2 nodes."""

import math

import numpy as np
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

# Latched topics (plan, road map): late subscribers get the last message.
LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)
# Sensor topics published by Gazebo and the drivers.
SENSOR = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def quaternion_from_yaw(yaw):
    """(x, y, z, w)."""
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def image_to_bgr(msg):
    """sensor_msgs/Image (rgb8, bgr8, rgba8, bgra8, mono8) -> BGR numpy array."""
    channels = {'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4, 'mono8': 1}
    enc = msg.encoding.lower()
    if enc not in channels:
        raise ValueError('unsupported image encoding: ' + msg.encoding)
    c = channels[enc]
    data = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    img = data.reshape(msg.height, msg.step)[:, :msg.width * c].reshape(msg.height, msg.width, c)
    if enc == 'mono8':
        return np.repeat(img, 3, axis=2)
    if enc.startswith('rgb'):
        img = img[..., [2, 1, 0]]
    else:
        img = img[..., :3]
    return np.ascontiguousarray(img)


def bgr_to_image(bgr, header=None):
    msg = Image()
    if header is not None:
        msg.header = header
    msg.height, msg.width = bgr.shape[:2]
    msg.encoding = 'bgr8'
    msg.is_bigendian = 0
    msg.step = msg.width * 3
    msg.data = np.ascontiguousarray(bgr, dtype=np.uint8).tobytes()
    return msg


def run(node_class, args=None):
    """Standard main(): init, spin until Ctrl+C, clean shutdown."""
    import rclpy
    rclpy.init(args=args)
    node = node_class()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
