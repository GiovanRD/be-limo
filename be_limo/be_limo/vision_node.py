"""Vision node: lane, traffic lights and stop signs from the front camera.

Inputs
  image_topic (parameter, default /rgb/image_raw)   sensor_msgs/Image
Outputs
  /lane_detection   std_msgs/Float32MultiArray  [valid, offset, heading, confidence]
                    offset: lane centre to the left of the robot at x_ref [m]
                    heading: lane direction relative to the robot [rad]
  /detected_events  std_msgs/String (JSON)
                    {"stamp": t, "detections": [{"type": "traffic_light",
                     "state": "red", "distance": 0.8, "bearing": -0.3}, ...]}
  /vision/debug     sensor_msgs/Image           camera image with the detections drawn
"""

import json

from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, String

from be_limo.lane_vision import CameraModel, LaneParams, detect_lane, draw_lane
from be_limo.ros_utils import SENSOR, bgr_to_image, image_to_bgr, run
from be_limo.sign_vision import SignParams, detect_signs, draw_signs


class Vision(Node):

    def __init__(self):
        super().__init__('vision_node')
        d = self.declare_parameter
        d('image_topic', '/rgb/image_raw')
        d('hfov', 1.0471976)
        d('camera_x', 0.10)
        d('camera_height', 0.21)
        d('camera_pitch', 0.0)
        d('process_every', 1)          # process one image out of N
        d('debug_every', 2)            # publish the debug image every N processed images
        d('detect_lane', True)
        d('detect_signs', True)
        gp = self.get_parameter
        self.cam = CameraModel(hfov=gp('hfov').value, cam_x=gp('camera_x').value,
                               cam_height=gp('camera_height').value,
                               pitch=gp('camera_pitch').value)
        self.lane_params = LaneParams()
        self.sign_params = SignParams()
        self.count = 0
        self.lane_pub = self.create_publisher(Float32MultiArray, '/lane_detection', 10)
        self.event_pub = self.create_publisher(String, '/detected_events', 10)
        self.debug_pub = self.create_publisher(Image, '/vision/debug', SENSOR)
        topic = gp('image_topic').value
        self.create_subscription(Image, topic, self.on_image, SENSOR)
        self.get_logger().info('Vision sur %s' % topic)

    def on_image(self, msg):
        self.count += 1
        gp = self.get_parameter
        if self.count % max(1, gp('process_every').value):
            return
        try:
            bgr = image_to_bgr(msg)
        except ValueError as exc:
            self.get_logger().warn(str(exc), throttle_duration_sec=5.0)
            return

        lane = None
        if gp('detect_lane').value:
            lane = detect_lane(bgr, self.cam, self.lane_params)
            self.lane_pub.publish(Float32MultiArray(data=[
                1.0 if lane.valid else 0.0, float(lane.offset),
                float(lane.heading), float(lane.confidence)]))

        signs = detect_signs(bgr, self.cam, self.sign_params) if gp('detect_signs').value else []
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.event_pub.publish(String(data=json.dumps({
            'stamp': stamp,
            'detections': [{'type': s.kind, 'state': s.state,
                            'distance': round(s.distance, 3),
                            'bearing': round(s.bearing, 3)} for s in signs]})))

        processed = self.count // max(1, gp('process_every').value)
        if self.debug_pub.get_subscription_count() and processed % max(1, gp('debug_every').value) == 0:
            out = draw_lane(bgr, lane, self.cam, self.lane_params) if lane else bgr
            out = draw_signs(out, signs)
            self.debug_pub.publish(bgr_to_image(out, msg.header))


def main(args=None):
    run(Vision, args)
