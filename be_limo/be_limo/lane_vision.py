"""Lane detection with the front camera of the LIMO.

The track has a black road, a yellow line between the two lanes and white
road edges. With right-hand traffic, the robot's lane is bounded by the
yellow line on its left and a white edge on its right.

Pipeline, for each image:

1. colour masks in HSV: yellow (centre line) and white (road edge);
2. on each image row of the region of interest, the first yellow/white
   pixel to the left of the lane centre and the first white pixel to its
   right;
3. projection of these pixels on the ground (flat-ground pinhole model,
   known camera height and pitch), in the robot frame base_link;
4. robust line fit of each boundary, then lane centre and heading.

Output: lateral offset of the lane centre at a reference distance (positive
when the lane centre is to the left of the robot) and heading error.

This module only needs numpy and OpenCV, not ROS.
"""

import math
from dataclasses import dataclass, field

import cv2
import numpy as np


@dataclass
class CameraModel:
    width: int = 320
    height: int = 240
    hfov: float = 1.0471976          # horizontal field of view [rad]
    cam_x: float = 0.10              # camera position in base_link [m]
    cam_y: float = 0.0
    cam_height: float = 0.21         # camera height above the ground [m]
    pitch: float = 0.0               # downwards tilt [rad]

    @property
    def f(self):
        return (self.width / 2.0) / math.tan(self.hfov / 2.0)

    @property
    def cx(self):
        return (self.width - 1) / 2.0

    @property
    def cy(self):
        return (self.height - 1) / 2.0

    def scaled(self, width, height):
        """Same camera at another image resolution."""
        return CameraModel(width, height, self.hfov, self.cam_x, self.cam_y,
                           self.cam_height, self.pitch)

    def pixel_to_ground(self, u, v):
        """Ground point (x, y) in base_link of pixel (u, v), arrays allowed.

        Returns NaN for pixels above the horizon.
        """
        u = np.asarray(u, dtype=float)
        v = np.asarray(v, dtype=float)
        xc = (u - self.cx) / self.f
        yc = (v - self.cy) / self.f
        cp, sp = math.cos(self.pitch), math.sin(self.pitch)
        fwd = cp - yc * sp
        down = sp + yc * cp
        with np.errstate(divide='ignore', invalid='ignore'):
            t = np.where(down > 1e-6, self.cam_height / down, np.nan)
        return self.cam_x + t * fwd, self.cam_y - t * xc

    def ground_to_pixel(self, x, y):
        """Pixel (u, v) of a ground point (x, y) in base_link."""
        x = np.asarray(x, dtype=float) - self.cam_x
        y = np.asarray(y, dtype=float) - self.cam_y
        cp, sp = math.cos(self.pitch), math.sin(self.pitch)
        h = self.cam_height
        zc = x * cp + h * sp                     # along the optical axis
        yc = -x * sp + h * cp                    # downwards
        xc = -y
        with np.errstate(divide='ignore', invalid='ignore'):
            return self.cx + self.f * xc / zc, self.cy + self.f * yc / zc


@dataclass
class LaneParams:
    lane_half_width: float = 0.135   # lane centre -> line distance [m]
    x_min: float = 0.30              # region of interest on the ground [m]
    x_max: float = 1.10
    x_ref: float = 0.70              # offset is given at this distance [m]
    row_step: int = 2
    min_points: int = 6
    outlier: float = 0.03            # residual to reject a point [m]
    # HSV thresholds (OpenCV ranges: H 0-180, S and V 0-255)
    yellow_low: tuple = (15, 80, 90)
    yellow_high: tuple = (40, 255, 255)
    white_max_s: int = 60
    white_min_v: int = 170


@dataclass
class LaneResult:
    valid: bool = False
    offset: float = 0.0              # lane centre lateral position at x_ref [m]
    heading: float = 0.0             # lane direction relative to the robot [rad]
    confidence: float = 0.0          # 0..1
    left: tuple = None               # (a, b) of y = a + b x, yellow side
    right: tuple = None              # (a, b), white edge side
    left_points: list = field(default_factory=list)
    right_points: list = field(default_factory=list)
    source: str = 'none'             # 'both', 'left', 'right' or 'none'


def _fit(points, p):
    if len(points) < p.min_points:
        return None
    pts = np.asarray(points)
    keep = np.ones(len(pts), dtype=bool)
    for _ in range(2):
        if keep.sum() < p.min_points:
            return None
        b, a = np.polyfit(pts[keep, 0], pts[keep, 1], 1)
        res = np.abs(pts[:, 1] - (a + b * pts[:, 0]))
        keep = res < p.outlier
    if keep.sum() < p.min_points:
        return None
    b, a = np.polyfit(pts[keep, 0], pts[keep, 1], 1)
    return float(a), float(b)


def masks(bgr, p=None):
    p = p or LaneParams()
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    yellow = cv2.inRange(hsv, np.array(p.yellow_low), np.array(p.yellow_high)) > 0
    white = (hsv[..., 1] <= p.white_max_s) & (hsv[..., 2] >= p.white_min_v)
    return yellow, white


def detect_lane(bgr, cam=None, p=None):
    """Detect the robot's lane in a BGR image."""
    p = p or LaneParams()
    h, w = bgr.shape[:2]
    cam = (cam or CameraModel()).scaled(w, h)
    yellow, white = masks(bgr, p)

    # Image rows covering [x_min, x_max] on the ground.
    v_far = int(math.ceil(cam.ground_to_pixel(p.x_max, 0.0)[1]))
    v_near = int(math.floor(cam.ground_to_pixel(p.x_min, 0.0)[1]))
    v_far, v_near = max(v_far, 0), min(v_near, h - 1)

    res = LaneResult()
    rows = list(range(v_near, v_far - 1, -p.row_step))
    if not rows:
        return res

    centre = w // 2
    found_rows = 0
    for v in rows:
        right_cols = np.nonzero(white[v, centre:])[0]
        left_cols = np.nonzero((yellow[v, :centre] | white[v, :centre])[::-1])[0]
        hit = False
        if right_cols.size:
            u = centre + right_cols[0]
            res.right_points.append(cam.pixel_to_ground(u, v))
            hit = True
        if left_cols.size:
            u = centre - 1 - left_cols[0]
            if yellow[v, u]:
                res.left_points.append(cam.pixel_to_ground(u, v))
                hit = True
        if hit:
            found_rows += 1
        # Follow the lane: next row is searched from the middle of the lane.
        if right_cols.size and left_cols.size:
            centre = int((centre + right_cols[0] + centre - 1 - left_cols[0]) / 2)
            centre = min(max(centre, w // 4), 3 * w // 4)

    res.left_points = [(float(x), float(y)) for x, y in res.left_points]
    res.right_points = [(float(x), float(y)) for x, y in res.right_points]
    res.left = _fit(res.left_points, p)
    res.right = _fit(res.right_points, p)

    xr = p.x_ref
    candidates = []
    if res.left:
        candidates.append(('left', res.left[0] + res.left[1] * xr - p.lane_half_width,
                           res.left[1]))
    if res.right:
        candidates.append(('right', res.right[0] + res.right[1] * xr + p.lane_half_width,
                           res.right[1]))

    if len(candidates) == 2:
        width = (res.left[0] + res.left[1] * xr) - (res.right[0] + res.right[1] * xr)
        if abs(width - 2 * p.lane_half_width) < 0.08:
            res.source = 'both'
            res.offset = (candidates[0][1] + candidates[1][1]) / 2.0
            res.heading = math.atan((candidates[0][2] + candidates[1][2]) / 2.0)
        else:
            # Inconsistent width: trust the yellow centre line.
            res.source = 'left'
            res.offset, res.heading = candidates[0][1], math.atan(candidates[0][2])
    elif candidates:
        res.source = candidates[0][0]
        res.offset, res.heading = candidates[0][1], math.atan(candidates[0][2])

    res.valid = res.source != 'none'
    res.confidence = found_rows / len(rows) if res.valid else 0.0
    if res.source == 'both':
        res.confidence = min(1.0, res.confidence * 1.2)
    return res


def draw_lane(bgr, result, cam=None, p=None):
    """Debug image: detected points and estimated lane centre."""
    p = p or LaneParams()
    out = bgr.copy()
    h, w = out.shape[:2]
    cam = (cam or CameraModel()).scaled(w, h)
    for pts, colour in ((result.left_points, (0, 200, 255)),
                        (result.right_points, (255, 120, 0))):
        for x, y in pts:
            u, v = cam.ground_to_pixel(x, y)
            if np.isfinite(u) and np.isfinite(v):
                cv2.circle(out, (int(u), int(v)), 2, colour, -1)
    if result.valid:
        xs = np.linspace(p.x_min, p.x_max, 12)
        ys = result.offset + np.tan(result.heading) * (xs - p.x_ref)
        us, vs = cam.ground_to_pixel(xs, ys)
        pts = [(int(u), int(v)) for u, v in zip(us, vs) if np.isfinite(u) and np.isfinite(v)]
        for a, b in zip(pts, pts[1:]):
            cv2.line(out, a, b, (0, 255, 0), 2)
    label = 'voie: %s  dy=%+.3f m  cap=%+.1f deg' % (
        result.source, result.offset, math.degrees(result.heading))
    cv2.putText(out, label, (4, 14), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 255, 0), 1)
    return out
