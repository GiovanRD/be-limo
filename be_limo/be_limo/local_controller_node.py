"""Local controller: follows the planned path and obeys the detections.

Inputs
  /plan               nav_msgs/Path           path from the global planner
  /robot_pose         geometry_msgs/PoseStamped
  /obstacle_distance  std_msgs/Float32        free distance ahead (lidar)
  /detected_events    std_msgs/String (JSON)  traffic lights and stop signs
  /lane_detection     std_msgs/Float32MultiArray [valid, offset, heading, confidence]
  /supervisor/command std_msgs/String (JSON)  {"cmd": "start" | "pause" | "estop" | "reset"},
                                              {"speed": 0.25}
Outputs
  /cmd_vel            geometry_msgs/Twist     (v, omega) to the robot
  /controller/state   std_msgs/String (JSON)  state machine and tracking data

State machine (priority order):
  EMERGENCY_STOP > PAUSED > NO_PLAN / NO_POSE > STOP_OBSTACLE >
  STOP_RED_LIGHT > STOP_SIGN > DRIVING (speed reduced near obstacles) > GOAL_REACHED
"""

import json
import math
import time

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray, String

from be_limo.pure_pursuit import PurePursuit, PurePursuitParams
from be_limo.ros_utils import LATCHED, run, yaw_from_quaternion


class LocalController(Node):

    def __init__(self):
        super().__init__('local_controller')
        d = self.declare_parameter
        d('rate', 20.0)
        d('speed', 0.25)
        d('max_steering', 0.41)
        d('lookahead_min', 0.25)
        d('lookahead_gain', 0.6)
        d('autostart', False)
        # obstacles
        d('obstacle_stop', 0.35)
        d('obstacle_resume', 0.45)
        d('obstacle_slow', 0.80)
        # traffic lights and stop signs
        # The traffic lights stand on the far side of the crossing: a red light
        # is obeyed only when it is seen from the stop line, i.e. between
        # red_light_min and red_light_distance (not once inside the crossing).
        d('red_light_min', 1.0)
        d('red_light_distance', 1.4)
        d('green_light_distance', 2.0)
        d('red_light_timeout', 2.0)
        d('stop_sign_distance', 0.60)
        d('stop_sign_duration', 2.0)
        d('stop_sign_cooldown', 6.0)
        d('confirm_frames', 2)
        # lane correction (camera)
        d('use_lane_correction', False)
        d('lane_gain', 0.8)
        d('lane_max_heading', 0.25)
        d('pose_timeout', 0.5)

        gp = self.get_parameter
        self.pp = PurePursuit(PurePursuitParams(
            speed=gp('speed').value, max_steering=gp('max_steering').value,
            lookahead_min=gp('lookahead_min').value, lookahead_gain=gp('lookahead_gain').value))
        self.speed = gp('speed').value

        self.pose = None
        self.pose_time = 0.0
        self.plan_id = 0
        self.has_plan = False
        self.running = gp('autostart').value
        self.estop = False
        self.obstacle = float('inf')
        self.obstacle_blocking = False
        self.lane = None
        self.red_seen, self.green_seen, self.stop_seen = 0, 0, 0
        self.red_any_seen = 0
        self.red_until = 0.0
        self.stop_until, self.stop_ignore_until = 0.0, 0.0
        self.last_events = []
        self.state = 'NO_PLAN'
        self.state_since = time.monotonic()

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.state_pub = self.create_publisher(String, '/controller/state', 10)
        self.create_subscription(Path, '/plan', self.on_plan, LATCHED)
        self.create_subscription(PoseStamped, '/robot_pose', self.on_pose, 20)
        self.create_subscription(Float32, '/obstacle_distance', self.on_obstacle, 10)
        self.create_subscription(String, '/detected_events', self.on_events, 10)
        self.create_subscription(Float32MultiArray, '/lane_detection', self.on_lane, 10)
        self.create_subscription(String, '/supervisor/command', self.on_command, 10)
        self.create_timer(1.0 / gp('rate').value, self.step)
        self.get_logger().info('Contrôleur prêt (%s)' % ('démarré' if self.running else
                                                       'en attente de "start"'))

    # ------------------------------------------------------------------ inputs
    def on_plan(self, msg):
        pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self.pp.set_path(pts)
        self.has_plan = len(pts) >= 2
        self.plan_id += 1
        self.get_logger().info('Nouveau chemin : %d points, %.1f m' % (len(pts), self.pp.length))

    def on_pose(self, msg):
        p = msg.pose
        self.pose = (p.position.x, p.position.y, yaw_from_quaternion(p.orientation))
        self.pose_time = time.monotonic()

    def on_obstacle(self, msg):
        self.obstacle = msg.data

    def on_lane(self, msg):
        if len(msg.data) >= 4:
            self.lane = tuple(msg.data[:4])

    def on_events(self, msg):
        try:
            data = json.loads(msg.data)
        except ValueError:
            return
        dets = data.get('detections', [])
        self.last_events = dets
        gp = self.get_parameter
        red = any(d['type'] == 'traffic_light' and d['state'] == 'red'
                  and gp('red_light_min').value < d['distance'] < gp('red_light_distance').value
                  for d in dets)
        red_any = any(d['type'] == 'traffic_light' and d['state'] == 'red'
                      and d['distance'] < gp('green_light_distance').value for d in dets)
        green = any(d['type'] == 'traffic_light' and d['state'] == 'green'
                    and d['distance'] < gp('green_light_distance').value for d in dets)
        stop = any(d['type'] == 'stop_sign' and d['distance'] < gp('stop_sign_distance').value
                   for d in dets)
        # Count consecutive frames to ignore one-frame false detections.
        self.red_seen = self.red_seen + 1 if red else 0
        self.red_any_seen = self.red_any_seen + 1 if red_any else 0
        self.green_seen = self.green_seen + 1 if green else 0
        self.stop_seen = self.stop_seen + 1 if stop else 0

    def on_command(self, msg):
        try:
            data = json.loads(msg.data)
        except ValueError:
            data = {'cmd': msg.data.strip().lower()}
        cmd = data.get('cmd')
        if cmd == 'start':
            self.running, self.estop = True, False
        elif cmd == 'pause':
            self.running = False
        elif cmd == 'estop':
            self.estop, self.running = True, False
        elif cmd == 'reset':
            self.estop = False
        if 'speed' in data:
            self.speed = max(0.05, min(0.6, float(data['speed'])))
        self.get_logger().info('Commande supervision : %s' % data)

    # -------------------------------------------------------------- control
    def decide(self, now):
        """Return (state, speed_scale)."""
        gp = self.get_parameter
        n = gp('confirm_frames').value
        if self.estop:
            return 'EMERGENCY_STOP', 0.0
        if not self.running:
            return 'PAUSED', 0.0
        if not self.has_plan:
            return 'NO_PLAN', 0.0
        if self.pose is None or now - self.pose_time > gp('pose_timeout').value:
            return 'NO_POSE', 0.0

        # Obstacle with hysteresis.
        stop_d, resume_d = gp('obstacle_stop').value, gp('obstacle_resume').value
        if self.obstacle < stop_d:
            self.obstacle_blocking = True
        elif self.obstacle > resume_d:
            self.obstacle_blocking = False
        if self.obstacle_blocking:
            return 'STOP_OBSTACLE', 0.0

        # Red light: stop until green is seen or red is no longer seen.
        if self.red_seen >= n or (now < self.red_until and self.red_any_seen >= 1):
            # Triggered from the stop line, then held while the red stays visible.
            self.red_until = now + gp('red_light_timeout').value
        if self.green_seen >= n:
            self.red_until = 0.0
        if now < self.red_until:
            return 'STOP_RED_LIGHT', 0.0

        # Stop sign: full stop, then ignore the sign for a while.
        if self.stop_seen >= n and now > self.stop_ignore_until:
            self.stop_until = now + gp('stop_sign_duration').value
            self.stop_ignore_until = self.stop_until + gp('stop_sign_cooldown').value
        if now < self.stop_until:
            return 'STOP_SIGN', 0.0

        slow = gp('obstacle_slow').value
        scale = 1.0
        if self.obstacle < slow:
            scale = max(0.25, (self.obstacle - stop_d) / (slow - stop_d))
            return 'SLOW_OBSTACLE', scale
        return 'DRIVING', scale

    def step(self):
        now = time.monotonic()
        state, scale = self.decide(now)
        cmd = Twist()
        result = None
        if state in ('DRIVING', 'SLOW_OBSTACLE'):
            x, y, yaw = self.pose
            result = self.pp.compute(x, y, yaw, speed_scale=scale, speed=self.speed)
            if result.done:
                state = 'GOAL_REACHED'
            else:
                omega = result.omega
                if self.get_parameter('use_lane_correction').value and self.lane:
                    valid, offset, heading, conf = self.lane
                    if valid > 0.5 and conf > 0.5 and abs(heading) < self.get_parameter(
                            'lane_max_heading').value:
                        # Steer towards the lane centre seen by the camera.
                        omega += self.get_parameter('lane_gain').value * offset * result.v
                cmd.linear.x = result.v
                cmd.angular.z = omega
        if state != self.state:
            self.state_since = now
            self.get_logger().info('Etat : %s -> %s' % (self.state, state))
        # Idle states only send zeros for one second, so that the robot can
        # still be driven by hand (teleoperation) while the controller waits.
        idle = state in ('PAUSED', 'NO_PLAN', 'NO_POSE', 'GOAL_REACHED')
        if not idle or now - self.state_since < 1.0:
            self.cmd_pub.publish(cmd)
        if state == 'GOAL_REACHED':
            self.has_plan = False
        self.state = state
        self.publish_state(cmd, result)

    def publish_state(self, cmd, result):
        info = {
            'state': self.state, 'plan_id': self.plan_id, 'running': self.running,
            'speed_setpoint': round(self.speed, 3),
            'v': round(cmd.linear.x, 3), 'omega': round(cmd.angular.z, 3),
            'obstacle': None if not math.isfinite(self.obstacle) else round(self.obstacle, 3),
        }
        if self.pose:
            info['pose'] = [round(v, 3) for v in self.pose]
        if result:
            info.update({'cross_track_error': round(result.cross_track_error, 3),
                         'progress': round(result.progress, 3),
                         'remaining': round(result.remaining, 2),
                         'steering': round(result.steering, 3),
                         'target': [round(result.target[0], 3), round(result.target[1], 3)]})
        self.state_pub.publish(String(data=json.dumps(info)))


def main(args=None):
    run(LocalController, args)
