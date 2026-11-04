"""Obstacle detection from a 2D lidar scan.

The lidar points are expressed in the robot frame (base_link). An obstacle
is a group of points inside the corridor that the robot is about to sweep:
an arc of the current curvature (kappa = omega / v), with the robot's width
plus a safety margin. The distance returned is measured along that arc.

This module has no ROS dependency.
"""

import math
from dataclasses import dataclass, field


@dataclass
class ObstacleParams:
    laser_x: float = 0.1236          # lidar position in base_link [m]
    laser_y: float = 0.0
    laser_yaw: float = 0.0
    half_width: float = 0.16         # robot half width + margin [m]
    max_range: float = 1.5           # corridor length [m]
    min_range: float = 0.05          # ignore points closer than this [m]
    cluster_gap: float = 0.08        # gap that splits two clusters [m]
    min_points: int = 2              # points needed to make an obstacle


@dataclass
class Obstacle:
    x: float
    y: float
    distance: float                  # along the corridor [m]
    size: float                      # cluster extent [m]
    points: int


@dataclass
class ObstacleReport:
    distance: float = float('inf')   # closest obstacle in the corridor [m]
    in_corridor: list = field(default_factory=list)
    all_clusters: list = field(default_factory=list)


def scan_to_points(ranges, angle_min, angle_increment, p=None,
                   range_min=0.0, range_max=float('inf')):
    """Convert a scan to (x, y) points in base_link."""
    p = p or ObstacleParams()
    pts = []
    c, s = math.cos(p.laser_yaw), math.sin(p.laser_yaw)
    for i, r in enumerate(ranges):
        if not math.isfinite(r) or r < max(range_min, p.min_range) or r > range_max:
            continue
        a = angle_min + i * angle_increment
        lx, ly = r * math.cos(a), r * math.sin(a)
        pts.append((p.laser_x + c * lx - s * ly, p.laser_y + s * lx + c * ly))
    return pts


def _cluster(points, gap):
    clusters, current = [], []
    for pt in points:
        if current and math.hypot(pt[0] - current[-1][0], pt[1] - current[-1][1]) > gap:
            clusters.append(current)
            current = []
        current.append(pt)
    if current:
        clusters.append(current)
    # The scan wraps around: merge the first and last clusters if close.
    if len(clusters) > 1:
        a, b = clusters[0][0], clusters[-1][-1]
        if math.hypot(a[0] - b[0], a[1] - b[1]) <= gap:
            clusters[0] = clusters.pop() + clusters[0]
    return clusters


def corridor_position(x, y, curvature):
    """(distance along the arc, lateral offset from the arc) of a point."""
    if abs(curvature) < 1e-3:
        return x, y
    r = 1.0 / curvature                      # signed radius, centre at (0, r)
    dx, dy = x, y - r
    angle = math.atan2(dx, -dy) if r > 0 else math.atan2(dx, dy)
    lateral = abs(r) - math.hypot(dx, dy)
    if r < 0:
        lateral = -lateral
    return angle * abs(r), lateral


def detect(points, curvature=0.0, p=None):
    """Find the obstacles in the corridor of the given curvature."""
    p = p or ObstacleParams()
    report = ObstacleReport()
    for cluster in _cluster(points, p.cluster_gap):
        if len(cluster) < p.min_points:
            continue
        cx = sum(q[0] for q in cluster) / len(cluster)
        cy = sum(q[1] for q in cluster) / len(cluster)
        size = math.hypot(cluster[0][0] - cluster[-1][0], cluster[0][1] - cluster[-1][1])
        closest = float('inf')
        for qx, qy in cluster:
            along, lateral = corridor_position(qx, qy, curvature)
            if 0.0 < along <= p.max_range and abs(lateral) <= p.half_width:
                closest = min(closest, along)
        ob = Obstacle(cx, cy, closest, size, len(cluster))
        report.all_clusters.append(ob)
        if math.isfinite(closest):
            report.in_corridor.append(ob)
            report.distance = min(report.distance, closest)
    return report
