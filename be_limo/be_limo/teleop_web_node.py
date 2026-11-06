"""Pilote web de la LIMO : à lancer SUR LE ROBOT, à ouvrir dans le navigateur du PC.

    ros2 launch be_limo teleop.launch.py                 # driver + lidar + caméra + vision + pilote
    # sur le PC (hors conteneur) :
    ssh -N -L 8080:localhost:8080 jetson@100.75.155.201
    # puis ouvrir http://localhost:8080

Trois modes, choisis dans la page :
* manuel : vitesse et braquage donnés par la page (clavier, curseurs) ;
* route  : suit la voie vue par la caméra (lane_follow.py, /lane_detection) ;
* suivi  : suit la personne devant le robot, repérée au lidar (person_follow.py).
Dans tous les modes, le robot ralentit puis s'arrête devant un obstacle
(safety.py, couloir du lidar dans le sens de la marche).

Pourquoi une page web : le PC et le robot sont sur deux sous-réseaux différents,
la découverte DDS de ROS 2 ne passe pas. Une connexion TCP (comme SSH) passe.
Par défaut le serveur n'écoute que localhost : il faut l'accès SSH au robot pour
le piloter (paramètre host = 0.0.0.0 pour l'ouvrir au réseau).

Entrées  /odom /odom_imu /limo/imu (imu_topic) /scan /limo_status /rgb/image_raw
         /vision/debug /lane_detection /controller/state /robot_pose
Sorties  /cmd_vel (seulement quand la page est armée), /supervisor/command
         (« pause » à l'armement, pour que local_controller ne pilote pas en même temps)

HTTP     GET  /                 la page (web/teleop.html)
         GET  /api/state        état JSON (robot, capteurs, pilote), ~10 Hz côté page
         GET  /api/map          carte des routes (polylignes, repère "map")
         POST /api/cmd          {"client", "mode", "armed", "v", "steering", "steer_in_place", "reacquire"}
         POST /api/stop         arrêt immédiat
         GET  /api/camera.mjpg  flux caméra (MJPEG), ?src=debug pour l'image de la vision
"""

import json
import math
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu, LaserScan
from std_msgs.msg import Float32MultiArray, String

from be_limo.lane_follow import LaneFollower, LaneFollowParams
from be_limo.obstacles import ObstacleParams, scan_to_points
from be_limo.person_follow import FollowParams, PersonFollower, person_candidates
from be_limo.ros_utils import SENSOR, image_to_bgr, run, yaw_from_quaternion
from be_limo.safety import ObstacleGuard, SafetyParams
from be_limo.teleop import TRACK, WHEELBASE, TeleopState, steering_from_omega

try:
    from limo_msgs.msg import LimoStatus
except ImportError:          # simulation sans limo_msgs
    LimoStatus = None

MOTION_MODES = {0: 'différentiel 4 roues', 1: 'Ackermann', 2: 'Mecanum'}


def find_page():
    try:
        from ament_index_python.packages import get_package_share_directory
        path = os.path.join(get_package_share_directory('be_limo'), 'web', 'teleop.html')
        if os.path.exists(path):
            return path
    except Exception:        # paquet pas installé (tests)
        pass
    return os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'web', 'teleop.html')


def downsample_scan(msg, max_points=360):
    n = len(msg.ranges)
    step = max(1, int(math.ceil(n / float(max_points))))
    ranges = []
    for r in msg.ranges[::step]:
        ranges.append(round(r, 3) if math.isfinite(r) and msg.range_min <= r <= msg.range_max else 0)
    return {'angle_min': msg.angle_min, 'angle_increment': msg.angle_increment * step,
            'range_max': msg.range_max, 'ranges': ranges}


def road_map_json():
    """Polylignes des routes (repère de la piste), simplifiées pour la page."""
    try:
        from be_limo.roadmap import RoadMap
        rm = RoadMap()
    except Exception as exc:     # pragma: no cover
        return {'error': str(exc)}
    edges = []
    for pts, _ in rm.edges.values():
        step = max(1, len(pts) // 25)
        sel = pts[::step] + ([pts[-1]] if (len(pts) - 1) % step else [])
        edges.append([[round(p[0], 3), round(p[1], 3)] for p in sel])
    return {'nodes': [[round(n[0], 3), round(n[1], 3), round(n[2], 3)] for n in rm.nodes],
            'edges': edges, 'width': 4.88, 'height': 6.20}


class TeleopWeb(Node):

    def __init__(self):
        super().__init__('teleop_web')
        d = self.declare_parameter
        d('host', '127.0.0.1')
        d('port', 8080)
        d('max_speed', 0.30)               # m/s, borne absolue de l'interface
        d('max_steering_deg', 25.0)        # braquage central max
        d('accel', 0.5)                    # m/s²
        d('timeout', 0.5)                  # s sans commande de la page -> arrêt
        d('rate', 20.0)                    # Hz de publication de /cmd_vel
        d('image_topic', '/rgb/image_raw')
        d('imu_topic', '/limo/imu')        # limo_start remappe /imu en /limo/imu
        d('camera_fps', 8.0)
        d('camera_width', 320)
        d('laser_x', 0.1236)               # position du lidar dans base_link
        d('laser_yaw', 0.0)
        # sécurité obstacles
        d('obstacle_stop', 0.35)
        d('obstacle_resume', 0.45)
        d('obstacle_slow', 0.80)
        d('obstacle_half_width', 0.16)
        # mode route
        d('lane_x_ref', 0.70)
        d('lane_min_confidence', 0.3)
        # mode suivi
        d('follow_distance', 0.70)
        gp = lambda name: self.get_parameter(name).value

        self.lock = threading.Lock()
        max_steer = math.radians(gp('max_steering_deg'))
        self.teleop = TeleopState(max_speed=gp('max_speed'), max_steering=max_steer,
                                  accel=gp('accel'), timeout=gp('timeout'))
        self.obstacle_params = ObstacleParams(laser_x=gp('laser_x'), laser_yaw=gp('laser_yaw'),
                                              half_width=gp('obstacle_half_width'))
        self.guard = ObstacleGuard(SafetyParams(stop=gp('obstacle_stop'), resume=gp('obstacle_resume'),
                                                slow=gp('obstacle_slow'), wheelbase=WHEELBASE),
                                   self.obstacle_params)
        self.lane = LaneFollower(LaneFollowParams(x_ref=gp('lane_x_ref'), wheelbase=WHEELBASE,
                                                  max_steering=max_steer,
                                                  min_confidence=gp('lane_min_confidence')))
        self.follower = PersonFollower(FollowParams(distance=gp('follow_distance'), wheelbase=WHEELBASE,
                                                    max_steering=max_steer))
        self.points = []                   # dernier scan, repère base_link
        self.points_t = 0.0
        self.lane_msg = None
        self.lane_t = 0.0
        self.follow_cmd = (0.0, 0.0)
        self.preview = None                # mode suivi, désarmé : personne qui serait suivie
        self.owner = None                  # onglet qui pilote
        self.data = {}
        self.sent = (0.0, 0.0)
        self.images = {}                   # 'raw' / 'debug' -> dernier sensor_msgs/Image
        self.image_seq = {'raw': 0, 'debug': 0}
        self.image_cond = threading.Condition()
        self.page = find_page()
        self.map_json = json.dumps(road_map_json()).encode()

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sup_pub = self.create_publisher(String, '/supervisor/command', 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 20)
        self.create_subscription(Odometry, '/odom_imu', self.on_odom_imu, 20)
        self.create_subscription(Imu, gp('imu_topic'), self.on_imu, SENSOR)
        self.create_subscription(LaserScan, '/scan', self.on_scan, SENSOR)
        # Images : abonnement seulement quand un navigateur regarde (le Jetson
        # passe beaucoup de temps à recevoir des images 640×480 en Python).
        self.image_topics = {'raw': gp('image_topic'), 'debug': '/vision/debug'}
        self.image_subs = {}
        self.viewers = {'raw': 0, 'debug': 0}
        self.create_timer(0.5, self.update_image_subs)
        self.create_subscription(Float32MultiArray, '/lane_detection', self.on_lane, 10)
        self.create_subscription(String, '/controller/state', self.on_controller, 10)
        self.create_subscription(PoseStamped, '/robot_pose', self.on_pose, 10)
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd_vel, 10)
        if LimoStatus is not None:
            self.create_subscription(LimoStatus, '/limo_status', self.on_status, 10)
        self.create_timer(1.0 / gp('rate'), self.on_timer)

        server = ThreadingHTTPServer((gp('host'), gp('port')), self.handler_class())
        server.daemon_threads = True
        threading.Thread(target=server.serve_forever, daemon=True).start()
        self.get_logger().info('Interface de téléopération : http://%s:%d  (page %s)' % (
            gp('host'), gp('port'), self.page))
        if gp('host') in ('127.0.0.1', 'localhost'):
            self.get_logger().info('Depuis le PC : ssh -N -L %d:localhost:%d jetson@<robot>, '
                                   'puis http://localhost:%d' % ((gp('port'),) * 3))

    # ---------------------------------------------------------- abonnements
    def put(self, key, value):
        with self.lock:
            self.data[key] = value
            self.data[key + '_t'] = time.monotonic()

    @staticmethod
    def odom_dict(msg):
        p, t = msg.pose.pose, msg.twist.twist
        return {'x': p.position.x, 'y': p.position.y, 'yaw': yaw_from_quaternion(p.orientation),
                'v': t.linear.x, 'omega': t.angular.z,
                'steering': steering_from_omega(t.linear.x, t.angular.z)}

    def on_odom(self, msg):
        self.put('odom', self.odom_dict(msg))

    def on_odom_imu(self, msg):
        self.put('odom_imu', self.odom_dict(msg))

    def on_imu(self, msg):
        self.put('imu', {'omega': msg.angular_velocity.z, 'ax': msg.linear_acceleration.x})

    def on_scan(self, msg):
        now = time.monotonic()
        pts = scan_to_points(msg.ranges, msg.angle_min, msg.angle_increment, self.obstacle_params,
                             msg.range_min, msg.range_max)
        with self.lock:
            self.points, self.points_t = pts, now
            t = self.teleop
            if t.mode == 'follow' and t.armed:
                v_sent, w_sent = self.sent
                self.follow_cmd = self.follower.update(now, pts, v_sent, w_sent, t.cruise)
            elif t.mode == 'follow':
                # Pas encore armé : montre qui serait suivi.
                self.preview = self.follower.acquire(person_candidates(pts, self.follower.p))
        self.put('scan', downsample_scan(msg))

    def on_lane(self, msg):
        with self.lock:
            self.lane_msg, self.lane_t = tuple(msg.data[:4]) if len(msg.data) >= 4 else None, time.monotonic()

    def on_image(self, key, msg):
        with self.image_cond:
            self.images[key] = msg
            self.image_seq[key] += 1
            self.image_cond.notify_all()
        self.put('camera' if key == 'raw' else 'camera_debug',
                 {'width': msg.width, 'height': msg.height, 'encoding': msg.encoding})

    def update_image_subs(self):
        for key, topic in self.image_topics.items():
            with self.image_cond:
                watched = self.viewers[key] > 0
            if watched and key not in self.image_subs:
                self.image_subs[key] = self.create_subscription(
                    Image, topic, lambda m, k=key: self.on_image(k, m), SENSOR)
            elif not watched and key in self.image_subs:
                self.destroy_subscription(self.image_subs.pop(key))

    def on_controller(self, msg):
        try:
            self.put('controller', json.loads(msg.data))
        except ValueError:
            pass

    def on_pose(self, msg):
        p = msg.pose
        self.put('pose', {'x': p.position.x, 'y': p.position.y, 'yaw': yaw_from_quaternion(p.orientation)})

    def on_cmd_vel(self, msg):
        # Tout ce qui passe sur /cmd_vel (nous ou local_controller).
        self.put('cmd_vel', {'v': msg.linear.x, 'omega': msg.angular.z})

    def on_status(self, msg):
        self.put('status', {'motion_mode': msg.motion_mode,
                            'motion_mode_name': MOTION_MODES.get(msg.motion_mode, '?'),
                            'control_mode': msg.control_mode, 'battery': msg.battery_voltage,
                            'error_code': msg.error_code, 'vehicle_state': msg.vehicle_state})

    # ------------------------------------------------------------ commande
    def guard_fn(self, now):
        """Sécurité obstacles ; sans scan récent, on n'avance pas."""
        stale = now - self.points_t > 0.5
        exclude = None
        if self.teleop.mode == 'follow' and self.follower.target is not None:
            exclude = (self.follower.target[0], self.follower.target[1], 0.30)

        def fn(v, delta):
            if abs(v) < 1e-6:
                return 0.0
            if stale:
                self.guard.distance = None
                return 0.0
            return self.guard.limit(self.points, v, delta, exclude)[0]
        return fn

    def on_timer(self):
        now = time.monotonic()
        with self.lock:
            t = self.teleop
            if not t.armed:
                self.lane.reset()
                self.follower.reset()
                self.follow_cmd = (0.0, 0.0)
            elif t.mode == 'route':
                lane = self.lane_msg if now - self.lane_t < 0.5 else None
                t.set_auto(*self.lane.update(now, lane, t.cruise))
            elif t.mode == 'follow':
                t.set_auto(*(self.follow_cmd if now - self.points_t < 0.5 else (0.0, self.follow_cmd[1])))
            out = t.step(now, self.guard_fn(now))
            if out is not None:
                self.sent = out
        if out is not None:
            cmd = Twist()
            cmd.linear.x, cmd.angular.z = float(out[0]), float(out[1])
            self.cmd_pub.publish(cmd)

    def handle_cmd(self, body):
        now = time.monotonic()
        client = str(body.get('client', ''))
        with self.lock:
            t = self.teleop
            if t.armed and self.owner not in (None, client):
                return 409, {'error': 'Le robot est déjà piloté depuis un autre onglet.'}
            was_armed = t.armed
            mode = body.get('mode')
            if mode is not None and mode != t.mode:
                t.command(now, mode=mode)                # désarme
                self.owner = None
                return 200, {'ok': True, 'mode': t.mode}
            if body.get('reacquire'):
                self.follower.reset()
            steering = body.get('steering')
            t.command(now, v=body.get('v'), armed=body.get('armed'),
                      steering=None if steering is None else float(steering),
                      steer_in_place=body.get('steer_in_place'))
            if t.armed:
                self.owner = client
            elif self.owner == client:
                self.owner = None
            just_armed = t.armed and not was_armed
        if just_armed:
            # local_controller ne doit pas publier sur /cmd_vel en même temps.
            self.sup_pub.publish(String(data=json.dumps({'cmd': 'pause'})))
            self.get_logger().info('Pilote armé, mode %s (local_controller mis en pause)' % t.mode)
        return 200, {'ok': True}

    def handle_stop(self, body):
        with self.lock:
            self.teleop.stop('ARRÊT demandé par l\'interface')
            self.owner = None
        self.cmd_pub.publish(Twist())
        self.get_logger().warn('ARRÊT demandé par l\'interface')
        return 200, {'ok': True}

    def state(self, client=None):
        now = time.monotonic()
        with self.lock:
            out = {}
            for key, value in self.data.items():
                if key.endswith('_t'):
                    continue
                out[key] = value
                out[key + '_age'] = round(now - self.data[key + '_t'], 2)
            t = self.teleop
            out['teleop'] = t.status(now)
            out['teleop']['sent'] = {'v': self.sent[0], 'omega': self.sent[1]}
            out['teleop']['owner_is_me'] = self.owner is not None and self.owner == client
            out['teleop']['owner_other'] = self.owner is not None and self.owner != client
            g = self.guard
            out['safety'] = {'distance': None if g.distance is None or not math.isfinite(g.distance)
                             else round(g.distance, 3),
                             'blocked': t.armed and abs(t.target_v) > 1e-6 and abs(t.allowed_v) < 1e-6,
                             'slowed': t.armed and 1e-6 < abs(t.allowed_v) < abs(t.target_v) - 1e-6,
                             'stop': g.p.stop, 'slow': g.p.slow}
            lane = self.lane_msg if now - self.lane_t < 1.0 else None
            out['lane'] = {'state': self.lane.state, 'valid': bool(lane and lane[0] > 0.5),
                           'offset': lane[1] if lane else None, 'heading': lane[2] if lane else None,
                           'confidence': lane[3] if lane else None, 'x_ref': self.lane.p.x_ref}
            f = self.follower
            out['follow'] = {'state': f.state, 'distance': f.p.distance,
                             'target': None if f.target is None else [round(f.target[0], 3),
                                                                       round(f.target[1], 3)],
                             'preview': None if t.armed or self.preview is None else
                             [round(self.preview[0], 3), round(self.preview[1], 3)]}
        out['geometry'] = {'wheelbase': WHEELBASE, 'track': TRACK,
                           'laser_x': self.obstacle_params.laser_x,
                           'laser_yaw': self.obstacle_params.laser_yaw}
        return out

    def jpeg_frames(self, key):
        """Générateur de JPEG, au plus camera_fps images/s, seulement s'il y a des images."""
        import cv2
        period = 1.0 / max(0.5, self.get_parameter('camera_fps').value)
        width = self.get_parameter('camera_width').value
        seq = -1
        while True:
            with self.image_cond:
                self.image_cond.wait_for(lambda: self.image_seq[key] != seq, timeout=2.0)
                if self.image_seq[key] == seq:
                    yield None          # pas d'image : garde la connexion vivante
                    continue
                msg, seq = self.images[key], self.image_seq[key]
            if msg is None:             # déjà prise par un autre onglet
                yield None
                continue
            t0 = time.monotonic()
            try:
                bgr = image_to_bgr(msg)
            except ValueError:
                yield None
                continue
            h, w = bgr.shape[:2]
            if w > width:
                bgr = cv2.resize(bgr, (width, int(h * width / w)), interpolation=cv2.INTER_AREA)
            ok, buf = cv2.imencode('.jpg', bgr, [cv2.IMWRITE_JPEG_QUALITY, 70])
            if ok:
                yield buf.tobytes()
            time.sleep(max(0.0, period - (time.monotonic() - t0)))

    # ----------------------------------------------------------------- HTTP
    def handler_class(self):
        node = self

        class Handler(BaseHTTPRequestHandler):
            protocol_version = 'HTTP/1.1'

            def log_message(self, fmt, *args):     # pas de journal à chaque requête
                pass

            def send_bytes(self, code, body, ctype):
                self.send_response(code)
                self.send_header('Content-Type', ctype)
                self.send_header('Content-Length', str(len(body)))
                self.send_header('Cache-Control', 'no-store')
                self.end_headers()
                self.wfile.write(body)

            def send_json(self, code, obj):
                self.send_bytes(code, json.dumps(obj).encode(), 'application/json')

            def do_GET(self):
                path, _, query = self.path.partition('?')
                params = dict(kv.split('=', 1) for kv in query.split('&') if '=' in kv)
                if path in ('/', '/index.html'):
                    with open(node.page, 'rb') as f:
                        self.send_bytes(200, f.read(), 'text/html; charset=utf-8')
                elif path == '/api/state':
                    self.send_json(200, node.state(params.get('client')))
                elif path == '/api/map':
                    self.send_bytes(200, node.map_json, 'application/json')
                elif path == '/api/camera.mjpg':
                    self.stream_camera('debug' if params.get('src') == 'debug' else 'raw')
                else:
                    self.send_json(404, {'error': 'introuvable'})

            def do_POST(self):
                length = int(self.headers.get('Content-Length') or 0)
                try:
                    body = json.loads(self.rfile.read(length) or b'{}')
                except ValueError:
                    return self.send_json(400, {'error': 'JSON invalide'})
                if self.path == '/api/cmd':
                    try:
                        self.send_json(*node.handle_cmd(body))
                    except (TypeError, ValueError) as exc:
                        self.send_json(400, {'error': str(exc)})
                elif self.path == '/api/stop':
                    self.send_json(*node.handle_stop(body))
                else:
                    self.send_json(404, {'error': 'introuvable'})

            def stream_camera(self, key):
                self.send_response(200)
                self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
                self.send_header('Cache-Control', 'no-store')
                self.send_header('Connection', 'close')
                self.end_headers()
                with node.image_cond:
                    node.viewers[key] += 1
                try:
                    for jpg in node.jpeg_frames(key):
                        if jpg is None:
                            continue
                        self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n'
                                         % len(jpg))
                        self.wfile.write(jpg)
                        self.wfile.write(b'\r\n')
                except (BrokenPipeError, ConnectionResetError):
                    pass
                finally:
                    with node.image_cond:
                        node.viewers[key] -= 1
                self.close_connection = True

        return Handler


def main(args=None):
    run(TeleopWeb, args)
