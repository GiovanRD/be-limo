"""Unit tests of the algorithms (no ROS needed): python3 -m pytest test/"""
import math
import os
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [os.path.join(HERE, '..'), HERE]

from be_limo.obstacles import ObstacleParams, corridor_position, detect, scan_to_points  # noqa: E402
from be_limo.pure_pursuit import PurePursuit  # noqa: E402
from be_limo.roadmap import RoadMap  # noqa: E402

DEMO = [2, 14, 20, 22, 9, 0, 2]


@pytest.fixture(scope='module')
def roadmap():
    return RoadMap()


@pytest.fixture(scope='module')
def texture():
    cv2 = pytest.importorskip('cv2')
    from sim_camera import find_texture
    path = find_texture()
    if path is None:
        pytest.skip('track texture not found')
    return cv2.imread(path)


# ------------------------------------------------------------------ road map
def test_every_edge_stays_on_the_road(roadmap, texture):
    h, w = texture.shape[:2]
    for (a, b), (pts, _) in roadmap.edges.items():
        off = 0
        for x, y in pts:
            u, v = int((x / 4.88 + 0.5) * w), int((0.5 - y / 6.2) * h)
            assert 0 <= u < w and 0 <= v < h, (a, b)
            bgr = texture[v, u].astype(int)
            yellow = bgr[2] > 150 and bgr[1] > 150 and bgr[0] < 100
            if bgr.mean() > 120 and not yellow:
                off += 1
        assert off / len(pts) < 0.05, 'edge %d->%d leaves the road' % (a, b)


def test_routes(roadmap):
    assert roadmap.shortest_path(2, 2) == [2]
    route = roadmap.shortest_path(2, 16)
    assert route[0] == 2 and route[-1] == 16
    for a, b in zip(route, route[1:]):
        assert (a, b) in roadmap.edges
    route, pts = roadmap.plan(DEMO)
    assert route == [2, 4, 14, 20, 22, 9, 0, 2]
    assert 15 < sum(math.hypot(q[0] - p[0], q[1] - p[1]) for p, q in zip(pts, pts[1:])) < 20


def test_every_node_reachable(roadmap):
    n = len(roadmap.nodes)
    for start in range(n):
        for goal in range(n):
            assert roadmap.shortest_path(start, goal) is not None, (start, goal)


# -------------------------------------------------------------- pure pursuit
def test_pure_pursuit_follows_the_demo_loop(roadmap):
    _, pts = roadmap.plan(DEMO)
    pp = PurePursuit()
    pp.set_path(pts)
    x, y, yaw = roadmap.nodes[2]
    v, dt, t, worst = 0.0, 0.02, 0.0, 0.0
    max_angle = math.atan(0.24 / 0.55)
    while t < 150:
        r = pp.compute(x, y, yaw)
        if r.done:
            break
        v += (r.v - v) * min(1.0, dt / 0.15)
        angle = max(-max_angle, min(max_angle, r.steering))
        xr, yr = x - 0.12 * math.cos(yaw) + v * math.cos(yaw) * dt, y - 0.12 * math.sin(yaw) + v * math.sin(yaw) * dt
        yaw += v / 0.24 * math.tan(angle) * dt
        x, y = xr + 0.12 * math.cos(yaw), yr + 0.12 * math.sin(yaw)
        worst = max(worst, abs(r.cross_track_error))
        t += dt
    assert r.done
    assert worst < 0.06


# ----------------------------------------------------------------- obstacles
def test_corridor_geometry():
    assert corridor_position(1.0, 0.1, 0.0) == (1.0, 0.1)
    k = 1.0                                   # left turn, radius 1 m
    along, lateral = corridor_position(math.sin(0.5), 1 - math.cos(0.5), k)
    assert along == pytest.approx(0.5, abs=1e-6) and lateral == pytest.approx(0.0, abs=1e-6)


def test_obstacle_in_front_and_beside():
    p = ObstacleParams()

    def scan_with(px, py):
        ranges = []
        n = 241
        for i in range(n):
            a = -2.0944 + i * 4.1888 / (n - 1)
            # point obstacle of radius 3 cm at (px, py) in the laser frame
            dx, dy = math.cos(a), math.sin(a)
            b = -(dx * px + dy * py)
            c = px * px + py * py - 0.03 ** 2
            disc = b * b - c
            ranges.append(-b - math.sqrt(disc) if disc >= 0 else math.inf)
        return scan_to_points(ranges, -2.0944, 4.1888 / (n - 1), p)

    ahead = detect(scan_with(0.6, 0.0))
    assert ahead.distance == pytest.approx(0.6 + p.laser_x - 0.03, abs=0.03)
    beside = detect(scan_with(0.6, 0.35))
    assert math.isinf(beside.distance)
    # the same point is in the corridor when the robot turns towards it
    turning = detect(scan_with(0.6, 0.35), curvature=1.6)
    assert math.isfinite(turning.distance)


# -------------------------------------------------------------------- vision
def test_lane_offset_and_signs(roadmap, texture):
    from sim_camera import render
    from sim_signs import draw_stop_sign, draw_traffic_light
    from be_limo.lane_vision import CameraModel, detect_lane
    from be_limo.sign_vision import detect_signs
    cam = CameraModel()
    x, y, yaw = roadmap.nodes[20]                       # straight lane
    for shift in (-0.04, 0.0, 0.04):                    # robot to the left of the lane centre
        pose = (x - shift * math.sin(yaw), y + shift * math.cos(yaw), yaw)
        lane = detect_lane(render(texture, pose, cam), cam)
        assert lane.valid and lane.source == 'both'
        assert lane.offset == pytest.approx(-shift, abs=0.015)
        assert abs(lane.heading) < 0.05
    base = render(texture, roadmap.nodes[20], cam)
    assert detect_signs(base, cam) == []
    for state in ('red', 'green'):
        img = base.copy()
        draw_traffic_light(img, cam, 1.0, -0.3, state)
        dets = detect_signs(img, cam)
        assert len(dets) == 1 and dets[0].kind == 'traffic_light' and dets[0].state == state
        assert dets[0].distance == pytest.approx(0.9, rel=0.15)
    img = base.copy()
    draw_stop_sign(img, cam, 0.8, -0.25)
    dets = detect_signs(img, cam)
    assert len(dets) == 1 and dets[0].kind == 'stop_sign'


# ------------------------------------------------------------- localisation
def test_room_map_calibration(tmp_path):
    from be_limo.frames import calibration_path, compose, inverse, load_calibration, save_calibration
    from be_limo.roadmap import node_poses
    start = node_poses()[2]
    path = calibration_path(str(tmp_path / 'salle207.yaml'))
    assert path.endswith('salle207_track.yaml')
    save_calibration(path, start, 2)
    room_in_track = load_calibration(path)
    assert room_in_track == pytest.approx(start, abs=1e-6)
    # At the start of the mapping session the robot is at the room_map origin.
    assert compose(room_in_track, (0.0, 0.0, 0.0)) == pytest.approx(start, abs=1e-6)
    # Any track pose goes to room_map and back unchanged (initial pose for AMCL).
    for pose in node_poses():
        room = compose(inverse(room_in_track), pose)
        assert compose(room_in_track, room) == pytest.approx(pose, abs=1e-9)
