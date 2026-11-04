"""Pure pursuit path tracking for the AgileX LIMO (Ackermann steering).

The robot aims at a point of the path located a look-ahead distance Ld in
front of it. The circle through the rear axle and this point gives the
steering angle of the equivalent bicycle model:

    delta = atan(2 L sin(alpha) / Ld)        omega = v tan(delta) / L

where alpha is the angle between the robot heading and the target point,
L the wheelbase. The command sent to the robot is (v, omega), as on
/cmd_vel. The speed is reduced in curves and near the goal.

This module has no ROS dependency.
"""

import math
from dataclasses import dataclass


@dataclass
class PurePursuitParams:
    wheelbase: float = 0.24          # L [m]
    speed: float = 0.30              # nominal speed [m/s]
    min_speed: float = 0.08          # speed floor while moving [m/s]
    lookahead_min: float = 0.25      # Ld at standstill [m]
    lookahead_gain: float = 0.6      # Ld = lookahead_min + gain * v [s]
    max_steering: float = 0.41       # |delta| limit [rad] (23.6 deg)
    curve_slowdown: float = 1.5      # v = speed / (1 + k |kappa|)
    goal_tolerance: float = 0.06     # distance at which the goal is reached [m]
    goal_slowdown: float = 1.0       # v <= gain * distance to goal [1/s]
    search_window: int = 60          # points searched ahead of the progress index
    rear_axle_offset: float = 0.12   # base_link -> rear axle distance [m]


@dataclass
class TrackingResult:
    v: float = 0.0
    omega: float = 0.0
    steering: float = 0.0
    cross_track_error: float = 0.0
    progress: float = 0.0            # fraction of the path done, 0..1
    remaining: float = 0.0           # distance to the end of the path [m]
    target: tuple = (0.0, 0.0)
    done: bool = False


class PurePursuit:
    """Follows a polyline [(x, y), ...] given the robot pose (x, y, yaw)."""

    def __init__(self, params=None):
        self.p = params or PurePursuitParams()
        self.path = []
        self.cumulative = []
        self.index = 0

    def set_path(self, points):
        self.path = list(points)
        self.cumulative = [0.0]
        for a, b in zip(self.path, self.path[1:]):
            self.cumulative.append(
                self.cumulative[-1] + math.hypot(b[0] - a[0], b[1] - a[1]))
        self.index = 0

    @property
    def length(self):
        return self.cumulative[-1] if self.cumulative else 0.0

    def _update_index(self, x, y):
        # The path crosses itself at intersections: only search forward,
        # in a window after the current progress index.
        end = min(len(self.path), self.index + self.p.search_window)
        best, best_d = self.index, float('inf')
        for i in range(self.index, end):
            d = math.hypot(self.path[i][0] - x, self.path[i][1] - y)
            if d < best_d:
                best, best_d = i, d
        self.index = best
        return best_d

    def _cross_track(self, x, y):
        i = min(self.index, len(self.path) - 2)
        (ax, ay), (bx, by) = self.path[i], self.path[i + 1]
        dx, dy = bx - ax, by - ay
        norm = math.hypot(dx, dy) or 1.0
        # Positive when the robot is to the left of the path.
        return (dx * (y - ay) - dy * (x - ax)) / norm

    def compute(self, x, y, yaw, speed_scale=1.0, speed=None):
        """Command for the current pose. speed_scale in [0, 1] slows down."""
        res = TrackingResult()
        if len(self.path) < 2:
            res.done = True
            return res

        # The bicycle model is written at the rear axle.
        x -= self.p.rear_axle_offset * math.cos(yaw)
        y -= self.p.rear_axle_offset * math.sin(yaw)

        self._update_index(x, y)
        res.cross_track_error = self._cross_track(x, y)
        res.remaining = self.length - self.cumulative[self.index]
        gx, gy = self.path[-1]
        dist_goal = math.hypot(gx - x, gy - y)
        res.progress = self.cumulative[self.index] / self.length if self.length else 1.0

        if res.remaining < self.p.goal_tolerance or (
                self.index >= len(self.path) - 2 and dist_goal < self.p.goal_tolerance):
            res.done = True
            res.progress = 1.0
            return res

        nominal = self.p.speed if speed is None else speed
        lookahead = self.p.lookahead_min + self.p.lookahead_gain * nominal

        # Target point: first path point at least Ld away, after the index.
        target = self.path[-1]
        for i in range(self.index, len(self.path)):
            px, py = self.path[i]
            if math.hypot(px - x, py - y) >= lookahead:
                target = (px, py)
                break
        res.target = target

        alpha = math.atan2(target[1] - y, target[0] - x) - yaw
        alpha = math.atan2(math.sin(alpha), math.cos(alpha))
        ld = max(math.hypot(target[0] - x, target[1] - y), 1e-3)
        curvature = 2.0 * math.sin(alpha) / ld
        steering = math.atan(self.p.wheelbase * curvature)
        steering = max(-self.p.max_steering, min(self.p.max_steering, steering))

        v = nominal / (1.0 + self.p.curve_slowdown * abs(curvature))
        v = min(v, self.p.goal_slowdown * res.remaining + 0.02)
        v = max(v, self.p.min_speed) * max(0.0, min(1.0, speed_scale))
        if abs(alpha) > math.pi / 2:
            # Target behind the robot: turn as tightly as possible, slowly.
            v = min(v, self.p.min_speed)

        res.v = v
        res.steering = steering
        res.omega = v * math.tan(steering) / self.p.wheelbase
        return res
