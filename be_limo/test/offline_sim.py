"""Offline simulator: the real be_limo nodes around a kinematic LIMO.

No ROS and no Gazebo are needed: the nodes run on a tiny in-process
stand-in for rclpy (test/fake_ros). The robot receives /cmd_vel like the
Gazebo plugin (same steering conversion as gazebo_twist_adapter.py); the
simulator publishes /odom, a synthetic /scan (obstacle box, poles) and
synthetic camera images (track texture, traffic light, stop sign).

Used by test_integration.py, and handy to try a change quickly:

    python3 test/offline_sim.py
"""
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [os.path.join(HERE, 'fake_ros'), os.path.join(HERE, '..'), HERE]

import cv2  # noqa: E402

from rclpy.node import BUS, Node  # noqa: E402  (fake rclpy)
from std_msgs.msg import Header, String  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from sensor_msgs.msg import LaserScan  # noqa: E402

from sim_camera import find_texture, render  # noqa: E402
from sim_signs import draw_stop_sign, draw_traffic_light  # noqa: E402

from be_limo import local_controller_node  # noqa: E402
from be_limo.global_planner_node import GlobalPlanner  # noqa: E402
from be_limo.lane_vision import CameraModel  # noqa: E402
from be_limo.local_controller_node import LocalController  # noqa: E402
from be_limo.localization_node import Localization  # noqa: E402
from be_limo.obstacle_detector_node import ObstacleDetector  # noqa: E402
from be_limo.roadmap import node_poses  # noqa: E402
from be_limo.ros_utils import bgr_to_image, quaternion_from_yaw  # noqa: E402
from be_limo.vision_node import Vision  # noqa: E402

L = 0.24
MAX_ANGLE = math.atan(0.24 / 0.55)          # gazebo_twist_adapter.py
LIGHT_WEST = (0.54, -1.21, math.pi)         # light for vehicles coming from node 9
STOP_SIGN = (2.36, 1.75, -math.pi / 2)
BOX = (-0.70, 2.795)
POLES = [(0.54, -1.21), (-0.51, -0.26), (-0.51, -1.21), (0.54, -0.26), (2.36, 1.75)]


class FakeTime:
    @staticmethod
    def monotonic():
        return BUS.now


def ray_circle(ox, oy, dx, dy, cx, cy, r):
    fx, fy = ox - cx, oy - cy
    b = fx * dx + fy * dy
    c = fx * fx + fy * fy - r * r
    disc = b * b - c
    if disc < 0:
        return math.inf
    t = -b - math.sqrt(disc)
    return t if t > 0 else math.inf


def ray_box(ox, oy, dx, dy, cx, cy, half):
    tmin, tmax = -math.inf, math.inf
    for o, d, c in ((ox, dx, cx), (oy, dy, cy)):
        if abs(d) < 1e-9:
            if not c - half <= o <= c + half:
                return math.inf
            continue
        t1, t2 = (c - half - o) / d, (c + half - o) / d
        tmin, tmax = max(tmin, min(t1, t2)), min(tmax, max(t1, t2))
    return tmin if tmax >= tmin > 0 else math.inf


def relative(pose, x, y):
    px, py, yaw = pose
    dx, dy = x - px, y - py
    return math.cos(yaw) * dx + math.sin(yaw) * dy, -math.sin(yaw) * dx + math.cos(yaw) * dy


class OfflineSim:
    """Steps the nodes and the robot; scripted events for the demo."""

    def __init__(self, start_node=2, mission='demo', autostart=True, auto_events=True):
        self.texture = cv2.imread(find_texture())
        self.cam = CameraModel()
        BUS.__init__()
        Node.ALL = []
        Node.OVERRIDES = {
            'global_planner': {'mission': mission},
            'local_controller': {'autostart': autostart},
            'localization_node': {'odom_is_world': True},
        }
        local_controller_node.time = FakeTime
        self.nodes = [Localization(), GlobalPlanner(), LocalController(), ObstacleDetector(), Vision()]
        self.x, self.y, self.yaw = node_poses()[start_node]
        self.v = 0.0
        self.cmd = {'v': 0.0, 'w': 0.0}
        self.states = []
        self.box_present, self.light_red = True, True
        self.auto_events = auto_events
        self.waited = {'obstacle': 0.0, 'red': 0.0}
        self.min_box_distance = math.inf
        self.dt = 0.05
        self.step_count = 0
        BUS.subs['/cmd_vel'].append(lambda m: self.cmd.update(v=m.linear.x, w=m.angular.z))
        BUS.subs['/controller/state'].append(lambda m: self.states.append(json.loads(m.data)))
        self.publish_lights()

    def publish_lights(self):
        colour = 'red' if self.light_red else 'green'
        other = 'green' if self.light_red else 'red'
        BUS.publish('/traffic_lights/state', String(data=json.dumps({
            'mode': 'auto', 'lights': {'west': colour, 'east': colour, 'north': other, 'south': other}})), None)

    def step(self):
        dt = self.dt
        BUS.now += dt
        self.step_count += 1
        for node in self.nodes + Node.ALL[len(self.nodes):]:
            for t in node.timers:
                if t.period and BUS.now >= t.next - 1e-9:
                    t.next += t.period
                    t.cb()

        # robot: Gazebo adapter conversion + kinematic bicycle (rear axle)
        cmd = self.cmd
        if abs(cmd['v']) > 0.001:
            angle = max(-MAX_ANGLE, min(MAX_ANGLE, math.atan(L * cmd['w'] / abs(cmd['v']))))
        else:
            angle = 0.0
        self.v += (cmd['v'] - self.v) * min(1.0, dt / 0.15)
        x, y, yaw, v = self.x, self.y, self.yaw, self.v
        xr = x - 0.12 * math.cos(yaw) + v * math.cos(yaw) * dt
        yr = y - 0.12 * math.sin(yaw) + v * math.sin(yaw) * dt
        yaw += v / L * math.tan(angle) * dt
        self.x, self.y, self.yaw = xr + 0.12 * math.cos(yaw), yr + 0.12 * math.sin(yaw), yaw
        x, y = self.x, self.y
        if self.box_present:
            self.min_box_distance = min(self.min_box_distance, math.hypot(x - BOX[0], y - BOX[1]))

        odom = Odometry(header=Header(frame_id='odom'))
        odom.pose.pose.position.x, odom.pose.pose.position.y = x, y
        o = odom.pose.pose.orientation
        o.x, o.y, o.z, o.w = quaternion_from_yaw(yaw)
        BUS.publish('/odom', odom, None)

        if self.step_count % 3 == 0:
            self.publish_scan()
        if self.step_count % 2 == 0:
            self.publish_image()
        if self.auto_events:
            self.scripted_events()

    def publish_scan(self):
        x, y, yaw = self.x, self.y, self.yaw
        lx, ly = x + 0.1236 * math.cos(yaw), y + 0.1236 * math.sin(yaw)
        n, amin, amax = 360, -2.0944, 2.0944
        ranges = []
        for i in range(n):
            a = yaw + amin + i * (amax - amin) / (n - 1)
            dx, dy = math.cos(a), math.sin(a)
            r = min([ray_circle(lx, ly, dx, dy, px, py, 0.012) for px, py in POLES] +
                    ([ray_box(lx, ly, dx, dy, BOX[0], BOX[1], 0.06)] if self.box_present else []))
            ranges.append(r if r < 8.0 else math.inf)
        BUS.publish('/scan', LaserScan(angle_min=amin, angle_max=amax,
                                       angle_increment=(amax - amin) / (n - 1),
                                       range_min=0.2, range_max=8.0, ranges=ranges), None)

    def publish_image(self):
        pose = (self.x, self.y, self.yaw)
        img = render(self.texture, pose, self.cam)
        for (ox, oy, face), kind in ((LIGHT_WEST, 'light'), (STOP_SIGN, 'stop')):
            fx, fy = relative(pose, ox, oy)
            facing = math.cos(face) * (self.x - ox) + math.sin(face) * (self.y - oy) > 0
            if fx > 0.25 and facing and abs(math.atan2(fy, fx - 0.1)) < 0.6:
                if kind == 'light':
                    draw_traffic_light(img, self.cam, fx, fy, 'red' if self.light_red else 'green')
                else:
                    draw_stop_sign(img, self.cam, fx, fy)
        BUS.publish('/rgb/image_raw', bgr_to_image(img), None)

    def scripted_events(self):
        """Remove the obstacle after 3 s of waiting; turn the light green after 3 s."""
        if not self.states:
            return
        st = self.states[-1]['state']
        if st == 'STOP_OBSTACLE':
            self.waited['obstacle'] += self.dt
            if self.waited['obstacle'] > 3.0:
                self.box_present = False
        if st == 'STOP_RED_LIGHT':
            self.waited['red'] += self.dt
            if self.waited['red'] > 3.0 and self.light_red:
                self.light_red = False
                self.publish_lights()

    def state_sequence(self):
        seen = []
        for st in self.states:
            if not seen or seen[-1] != st['state']:
                seen.append(st['state'])
        return seen


if __name__ == '__main__':
    sim = OfflineSim()
    while BUS.now < 240 and 'GOAL_REACHED' not in sim.state_sequence():
        sim.step()
    print(' > '.join(sim.state_sequence()))
    print('%.1f s' % BUS.now)
