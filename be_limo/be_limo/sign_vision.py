"""Traffic light and stop sign detection with the front camera.

Both are found by colour, above the horizon (they stand higher than the
camera), then told apart by what surrounds the coloured disc:

* traffic light: a bright red or green disc inside a dark housing;
* stop sign: a red disc with a white border.

The distance is estimated from the apparent diameter of the disc
(pinhole model: d = f * D / diameter_px), the bearing from its column.

This module only needs numpy and OpenCV, not ROS.
"""

import math
from dataclasses import dataclass

import cv2
import numpy as np

from be_limo.lane_vision import CameraModel


@dataclass
class SignParams:
    light_diameter: float = 0.034    # traffic light bulb [m]
    stop_diameter: float = 0.10      # red part of the stop sign [m]
    min_area: int = 6                # pixels
    max_aspect: float = 2.0          # bounding box elongation
    min_fill: float = 0.45           # blob area / bounding box area
    horizon_margin: int = 4          # blobs must be above cy + margin [px]
    dark_v: int = 80                 # housing brightness (V) below this
    bright_v: int = 150              # stop sign border brightness above this
    red_s: int = 120
    red_v: int = 120
    green_s: int = 100
    green_v: int = 100


@dataclass
class Detection:
    kind: str                        # 'traffic_light' or 'stop_sign'
    state: str                       # 'red', 'green' or '' (stop sign)
    distance: float                  # [m]
    bearing: float                   # [rad], positive to the left
    u: float
    v: float
    radius_px: float


def _colour_masks(hsv, p):
    h, s, v = hsv[..., 0], hsv[..., 1], hsv[..., 2]
    red = ((h <= 8) | (h >= 170)) & (s >= p.red_s) & (v >= p.red_v)
    green = (h >= 45) & (h <= 90) & (s >= p.green_s) & (v >= p.green_v)
    return red.astype(np.uint8), green.astype(np.uint8)


def _surround_brightness(hsv, mask, x, y, w, h):
    """Median V and S of the ring around a blob, outside the blob itself."""
    H, W = mask.shape
    pad_x, pad_y = max(2, w // 3 + 1), max(2, h // 3 + 1)
    x0, y0 = max(0, x - pad_x), max(0, y - pad_y)
    x1, y1 = min(W, x + w + pad_x), min(H, y + h + pad_y)
    ring = np.ones((y1 - y0, x1 - x0), dtype=bool)
    ring[y - y0:y - y0 + h, x - x0:x - x0 + w] = False
    ring &= mask[y0:y1, x0:x1] == 0
    if not ring.any():
        return 255.0, 0.0
    return (float(np.median(hsv[y0:y1, x0:x1, 2][ring])),
            float(np.median(hsv[y0:y1, x0:x1, 1][ring])))


def detect_signs(bgr, cam=None, p=None):
    """List of Detection objects found in a BGR image."""
    p = p or SignParams()
    H, W = bgr.shape[:2]
    cam = (cam or CameraModel()).scaled(W, H)
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    red, green = _colour_masks(hsv, p)
    found = []
    for colour, mask in (('red', red), ('green', green)):
        n, _, stats, centroids = cv2.connectedComponentsWithStats(mask, connectivity=8)
        for i in range(1, n):
            x, y, w, h, area = stats[i]
            if area < p.min_area:
                continue
            if max(w, h) > p.max_aspect * max(1, min(w, h)):
                continue
            if area < p.min_fill * w * h:
                continue
            u, v = centroids[i]
            if v > cam.cy + p.horizon_margin:
                continue                            # on the ground: not a sign
            surround_v, surround_s = _surround_brightness(hsv, mask, x, y, w, h)
            diameter = (w + h) / 2.0
            if surround_v <= p.dark_v:
                kind, size = 'traffic_light', p.light_diameter
            elif colour == 'red' and surround_v >= p.bright_v and surround_s < 80:
                kind, size = 'stop_sign', p.stop_diameter
            else:
                continue
            distance = cam.f * size / max(diameter, 1.0)
            bearing = -math.atan((u - cam.cx) / cam.f)
            found.append(Detection(kind, colour if kind == 'traffic_light' else '',
                                   distance, bearing, float(u), float(v), diameter / 2.0))
    found.sort(key=lambda d: d.distance)
    return found


def draw_signs(bgr, detections):
    out = bgr.copy()
    for d in detections:
        colour = (0, 0, 255) if d.state == 'red' or d.kind == 'stop_sign' else (0, 255, 0)
        r = int(max(4, d.radius_px * 1.8))
        cv2.circle(out, (int(d.u), int(d.v)), r, colour, 1)
        label = ('FEU ' + d.state.upper()) if d.kind == 'traffic_light' else 'STOP'
        cv2.putText(out, '%s %.2fm' % (label, d.distance), (int(d.u) - 20, int(d.v) - r - 3),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, colour, 1)
    return out
