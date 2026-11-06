"""Modes route et suivi, sécurité obstacles, odométrie gyroscope (sans ROS)."""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [os.path.join(HERE, '..'), HERE]

from be_limo.frames import compose, inverse  # noqa: E402
from be_limo.lane_follow import LaneFollower  # noqa: E402
from be_limo.obstacles import ObstacleParams, scan_to_points  # noqa: E402
from be_limo.odom_imu import GyroOdometry  # noqa: E402
from be_limo.person_follow import PersonFollower, move_point, person_candidates  # noqa: E402
from be_limo.safety import ObstacleGuard  # noqa: E402
from be_limo.teleop import TeleopState, omega_from_steering  # noqa: E402

LASER_X = 0.1236


# ------------------------------------------------------------- lidar simulé
def ray_circle(ox, oy, dx, dy, cx, cy, r):
    fx, fy = ox - cx, oy - cy
    b = fx * dx + fy * dy
    c = fx * fx + fy * fy - r * r
    disc = b * b - c
    if disc < 0:
        return None
    t = -b - math.sqrt(disc)
    return t if t > 0 else None


def ray_segment(ox, oy, dx, dy, ax, ay, bx, by):
    ex, ey = bx - ax, by - ay
    den = dx * ey - dy * ex
    if abs(den) < 1e-12:
        return None
    t = ((ax - ox) * ey - (ay - oy) * ex) / den
    u = ((ax - ox) * dy - (ay - oy) * dx) / den
    return t if t > 0 and 0 <= u <= 1 else None


def scan(robot, circles=(), segments=(), n=450, max_range=8.0):
    """Points (repère base_link) vus par le lidar du robot (x, y, yaw), monde donné."""
    lx, ly, _ = compose(robot, (LASER_X, 0.0, 0.0))
    ranges = []
    a0, inc = -math.pi, 2 * math.pi / n
    for i in range(n):
        a = robot[2] + a0 + i * inc
        dx, dy = math.cos(a), math.sin(a)
        best = float('inf')
        for cx, cy, r in circles:
            t = ray_circle(lx, ly, dx, dy, cx, cy, r)
            if t is not None:
                best = min(best, t)
        for s in segments:
            t = ray_segment(lx, ly, dx, dy, *s)
            if t is not None:
                best = min(best, t)
        ranges.append(best if best <= max_range else float('inf'))
    return scan_to_points(ranges, a0, inc, ObstacleParams(laser_x=LASER_X), 0.05, max_range)


def legs(px, py, heading=0.0, gap=0.2, r=0.05):
    nx, ny = -math.sin(heading), math.cos(heading)
    return [(px + nx * gap / 2, py + ny * gap / 2, r), (px - nx * gap / 2, py - ny * gap / 2, r)]


def drive(pose, v, delta, dt, L=0.2):
    w = omega_from_steering(v, delta, L)
    x, y, th = pose
    return (x + v * math.cos(th + w * dt / 2) * dt, y + v * math.sin(th + w * dt / 2) * dt,
            math.atan2(math.sin(th + w * dt), math.cos(th + w * dt))), w


# ------------------------------------------------------------- odométrie
def test_gyro_odometry_closes_a_circle_and_learns_the_bias():
    o = GyroOdometry()
    t, bias = 0.0, 0.004
    for _ in range(300):                       # 3 s immobile : apprend le biais
        t += 0.01
        o.update_speed(t, 0.0)
        o.update_gyro(t, bias)
    assert abs(o.bias - bias) < 1e-3 and o.yaw == 0.0
    w, v = 0.25, 0.1
    for _ in range(int(round(2 * math.pi / w / 0.01))):
        t += 0.01
        o.update_speed(t, v)
        o.update_gyro(t, w + bias)
    assert math.hypot(o.x, o.y) < 0.03         # tour complet : revient au départ
    assert abs(o.yaw) < 0.05


def test_corrected_frame_composes_to_the_corrected_pose():
    driver = (0.8, 0.01, 0.0)                  # cap faux (toujours 0)
    good = (0.5, 0.45, 1.2)
    corr = compose(good, inverse(driver))      # odom_imu -> odom
    back = compose(corr, driver)               # odom_imu -> base_link
    assert all(abs(a - b) < 1e-9 for a, b in zip(back, good))


# ------------------------------------------------------- sécurité obstacles
def test_guard_stops_forward_but_lets_reverse():
    wall = [(0.0 + 0.30, -0.5, 0.30, 0.5)]     # mur à 30 cm devant (repère robot = monde)
    pts = scan((0, 0, 0), segments=wall)
    g = ObstacleGuard()
    v, d = g.limit(pts, 0.15, 0.0)
    assert v == 0.0 and d < 0.35
    v, _ = g.limit(pts, -0.1, 0.0)             # reculer : libre
    assert v == -0.1


def test_guard_slows_then_stops_with_hysteresis():
    g = ObstacleGuard()
    far = scan((0, 0, 0), segments=[(0.6, -0.5, 0.6, 0.5)])
    v, d = g.limit(far, 0.2, 0.0)
    assert 0 < v < 0.2 and 0.5 < d < 0.7       # ralentit
    near = scan((0, 0, 0), segments=[(0.33, -0.5, 0.33, 0.5)])
    assert g.limit(near, 0.2, 0.0)[0] == 0.0
    mid = scan((0, 0, 0), segments=[(0.40, -0.5, 0.40, 0.5)])
    assert g.limit(mid, 0.2, 0.0)[0] == 0.0    # 0,40 < resume : reste arrêté
    assert g.limit(far, 0.2, 0.0)[0] > 0.0


def test_guard_ignores_obstacle_beside_when_going_straight():
    post = [(0.5, 0.35, 0.03)]                 # poteau à 35 cm à gauche
    pts = scan((0, 0, 0), circles=post)
    g = ObstacleGuard()
    assert g.limit(pts, 0.15, 0.0)[0] == 0.15
    assert g.limit(pts, 0.15, math.radians(25))[0] < 0.15   # en tournant vers lui : ralentit


def test_teleop_applies_guard_in_every_mode():
    s = TeleopState(accel=10.0)
    s.command(0.0, armed=True, v=0.2)
    s.step(0.0)
    v, _ = s.step(0.1, guard=lambda v, d: 0.0)
    assert v == 0.0
    s.command(0.11, mode='route')
    assert not s.armed and s.mode == 'route'   # changer de mode désarme
    s.command(0.12, armed=True, v=0.15, steering=0.3)
    assert s.cruise == 0.15 and s.delta == 0.0  # en auto, la page ne braque pas
    s.set_auto(0.1, 0.2)
    s.step(0.12)
    v, w = s.step(0.2)
    assert abs(v - 0.1) < 1e-9 and w > 0


# ------------------------------------------------------------- mode route
def test_lane_follower_steers_towards_lane_centre_and_stops_without_lane():
    f = LaneFollower()
    v, d = f.update(0.0, (1.0, 0.0, 0.0, 0.9), 0.15)
    assert abs(d) < 1e-9 and abs(v - 0.15) < 1e-9
    for k in range(10):
        v, d = f.update(0.05 * k, (1.0, 0.10, 0.0, 0.9), 0.15)   # voie à gauche
    assert d > 0.05
    for k in range(10):
        v, d = f.update(1.0 + 0.05 * k, (1.0, -0.10, 0.0, 0.9), 0.15)
    assert d < -0.05
    v, _ = f.update(1.6, None, 0.15)           # perdue un instant : ralentit
    assert 0 < v < 0.15
    v, _ = f.update(2.5, None, 0.15)           # perdue : arrêt
    assert v == 0.0


def test_lane_follower_converges_on_a_straight_lane():
    """Robot décalé de 12 cm et tourné de 10° : il rejoint le centre de la voie."""
    f = LaneFollower()
    pose = (0.0, 0.12, math.radians(10))       # voie = axe x
    t = 0.0
    for _ in range(300):
        x, y, th = pose
        # Ce que voit la caméra : la ligne centrale (axe x du monde) dans le
        # repère du robot, décalage à x_ref = 0,7 m devant et angle.
        c, s = math.cos(th), math.sin(th)
        offset = (-y - 0.7 * s) / c
        v, d = f.update(t, (1.0, offset, -th, 0.9), 0.15)
        pose, _ = drive(pose, v, d, 0.05)
        t += 0.05
    assert abs(pose[1]) < 0.02 and abs(pose[2]) < math.radians(3)


# ------------------------------------------------------------- mode suivi
def test_person_candidates_keep_legs_and_drop_walls():
    world_legs = legs(1.0, 0.2)
    wall = [(1.8, -2.0, 1.8, 2.0)]
    pts = scan((0, 0, 0), circles=world_legs, segments=wall)
    people = person_candidates(pts)
    assert len(people) == 1
    x, y = people[0]
    assert abs(x - (1.0 - 0.0)) < 0.1 and abs(y - 0.2) < 0.08


def test_acquire_nearest_person_in_front_only():
    f = PersonFollower()
    behind = legs(-0.8, 0.0)
    side = legs(0.2, 1.0)
    front = legs(1.2, -0.1)
    pts = scan((0, 0, 0), circles=behind + side + front)
    v, d = f.update(0.0, pts, 0.0, 0.0, 0.2)
    assert f.target is not None and f.target[0] > 1.0


def test_move_point_matches_robot_motion():
    robot0 = (0.0, 0.0, 0.0)
    p_world = (1.0, 0.5)
    robot1, w = drive(robot0, 0.2, 0.3, 0.1)
    expect = compose(inverse(robot1), (p_world[0], p_world[1], 0.0))[:2]
    got = move_point(p_world, 0.2, w, 0.1)
    assert math.hypot(expect[0] - got[0], expect[1] - got[1]) < 1e-3


def test_follow_a_walking_person_and_keep_the_distance():
    """La personne marche à 0,12 m/s puis tourne à gauche ; le robot suit à ~0,7 m."""
    f = PersonFollower()
    guard = ObstacleGuard()
    robot = (0.0, 0.0, 0.0)
    person = [1.0, 0.0]
    heading = 0.0
    wall = [(-1.0, -1.0, 4.0, -1.0)]           # mur à droite, loin
    v_r, w_r, t, dt = 0.0, 0.0, 0.0, 0.1
    dists = []
    for k in range(200):
        if k == 80:
            heading = math.radians(60)
        person[0] += 0.12 * math.cos(heading) * dt
        person[1] += 0.12 * math.sin(heading) * dt
        pts = scan(robot, circles=legs(person[0], person[1], heading), segments=wall)
        v, d = f.update(t, pts, v_r, w_r, 0.2)
        exclude = None if f.target is None else (f.target[0], f.target[1], 0.3)
        v = guard.limit(pts, v, d, exclude)[0] if v > 0 else 0.0
        robot, w_r = drive(robot, v, d, dt)
        v_r = v
        t += dt
        if k > 40:
            dists.append(math.hypot(person[0] - robot[0], person[1] - robot[1]))
    assert f.target is not None, f.state
    assert 0.55 < sum(dists) / len(dists) < 1.0
    assert min(dists) > 0.4


def test_lost_person_stops_the_robot():
    f = PersonFollower()
    pts = scan((0, 0, 0), circles=legs(1.0, 0.0))
    f.update(0.0, pts, 0.0, 0.0, 0.2)
    assert f.target is not None
    empty = scan((0, 0, 0))
    v = None
    for k in range(1, 15):
        v, _ = f.update(0.1 * k, empty, 0.0, 0.0, 0.2)
    assert v == 0.0 and f.target is None and 'perdue' in f.state
