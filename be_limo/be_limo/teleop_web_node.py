"""Interface de téléopération web : à lancer SUR LE ROBOT, à ouvrir dans le navigateur du PC.

    ros2 launch be_limo teleop.launch.py                 # driver + lidar + caméra + interface
    # sur le PC (hors conteneur) :
    ssh -N -L 8080:localhost:8080 jetson@100.75.155.201
    # puis ouvrir http://localhost:8080

Pourquoi une page web : le PC et le robot sont sur deux sous-réseaux différents,
la découverte DDS de ROS 2 ne passe pas. Une connexion TCP (comme SSH) passe.
Par défaut le serveur n'écoute que localhost : il faut l'accès SSH au robot pour
le piloter (paramètre host = 0.0.0.0 pour l'ouvrir au réseau).

Entrées  /odom /limo/imu (paramètre imu_topic) /scan /limo_status /rgb/image_raw /controller/state /robot_pose
Sorties  /cmd_vel (seulement quand l'interface est armée), /supervisor/command
         (« pause » à l'armement, pour que local_controller ne pilote pas en même temps)

HTTP     GET  /                 la page (web/teleop.html)
         GET  /api/state        état JSON (robot, capteurs, téléop), ~10 Hz côté page
         POST /api/cmd          {"client", "armed", "v", "steering", "steer_in_place"}
         POST /api/stop         arrêt immédiat
         GET  /api/camera.mjpg  flux caméra (MJPEG)
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
from std_msgs.msg import String

from be_limo.ros_utils import SENSOR, image_to_bgr, run, yaw_from_quaternion
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
        d('imu_topic', '/limo/imu')          # limo_start remappe /imu en /limo/imu
        d('camera_fps', 8.0)
        d('camera_width', 320)
        d('laser_x', 0.1236)               # position du lidar dans base_link
        d('laser_yaw', 0.0)
        gp = lambda name: self.get_parameter(name).value

        self.lock = threading.Lock()
        self.teleop = TeleopState(max_speed=gp('max_speed'),
                                  max_steering=math.radians(gp('max_steering_deg')),
                                  accel=gp('accel'), timeout=gp('timeout'))
        self.owner = None                  # onglet qui pilote
        self.data = {}
        self.sent = (0.0, 0.0)
        self.image = None                  # dernier sensor_msgs/Image
        self.image_seq = 0
        self.image_cond = threading.Condition()
        self.page = find_page()

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sup_pub = self.create_publisher(String, '/supervisor/command', 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 20)
        self.create_subscription(Imu, gp('imu_topic'), self.on_imu, SENSOR)
        self.create_subscription(LaserScan, '/scan', self.on_scan, SENSOR)
        self.create_subscription(Image, gp('image_topic'), self.on_image, SENSOR)
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

    def on_odom(self, msg):
        p, t = msg.pose.pose, msg.twist.twist
        self.put('odom', {'x': p.position.x, 'y': p.position.y, 'yaw': yaw_from_quaternion(p.orientation),
                          'v': t.linear.x, 'omega': t.angular.z,
                          'steering': steering_from_omega(t.linear.x, t.angular.z)})

    def on_imu(self, msg):
        self.put('imu', {'omega': msg.angular_velocity.z, 'ax': msg.linear_acceleration.x})

    def on_scan(self, msg):
        self.put('scan', downsample_scan(msg))

    def on_image(self, msg):
        with self.image_cond:
            self.image = msg
            self.image_seq += 1
            self.image_cond.notify_all()
        self.put('camera', {'width': msg.width, 'height': msg.height, 'encoding': msg.encoding})

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
    def on_timer(self):
        with self.lock:
            out = self.teleop.step(time.monotonic())
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
            self.get_logger().info('Téléopération armée (local_controller mis en pause)')
        return 200, {'ok': True}

    def handle_stop(self, body):
        with self.lock:
            self.teleop.stop('ARRÊT demandé par l\'interface')
            self.owner = None
        stop = Twist()
        self.cmd_pub.publish(stop)
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
            out['teleop'] = self.teleop.status(now)
            out['teleop']['sent'] = {'v': self.sent[0], 'omega': self.sent[1]}
            out['teleop']['owner_is_me'] = self.owner is not None and self.owner == client
            out['teleop']['owner_other'] = self.owner is not None and self.owner != client
        out['geometry'] = {'wheelbase': WHEELBASE, 'track': TRACK,
                           'laser_x': self.get_parameter('laser_x').value,
                           'laser_yaw': self.get_parameter('laser_yaw').value}
        return out

    def jpeg_frames(self):
        """Générateur de JPEG, au plus camera_fps images/s, seulement s'il y a des images."""
        import cv2
        period = 1.0 / max(0.5, self.get_parameter('camera_fps').value)
        width = self.get_parameter('camera_width').value
        seq = -1
        while True:
            with self.image_cond:
                self.image_cond.wait_for(lambda: self.image_seq != seq, timeout=2.0)
                if self.image_seq == seq:
                    yield None          # pas d'image : garde la connexion vivante
                    continue
                msg, seq = self.image, self.image_seq
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

            def send_json(self, code, obj):
                body = json.dumps(obj).encode()
                self.send_response(code)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Content-Length', str(len(body)))
                self.send_header('Cache-Control', 'no-store')
                self.end_headers()
                self.wfile.write(body)

            def do_GET(self):
                path, _, query = self.path.partition('?')
                if path in ('/', '/index.html'):
                    with open(node.page, 'rb') as f:
                        body = f.read()
                    self.send_response(200)
                    self.send_header('Content-Type', 'text/html; charset=utf-8')
                    self.send_header('Content-Length', str(len(body)))
                    self.send_header('Cache-Control', 'no-store')
                    self.end_headers()
                    self.wfile.write(body)
                elif path == '/api/state':
                    client = query[len('client='):] if query.startswith('client=') else None
                    self.send_json(200, node.state(client))
                elif path == '/api/camera.mjpg':
                    self.stream_camera()
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

            def stream_camera(self):
                self.send_response(200)
                self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
                self.send_header('Cache-Control', 'no-store')
                self.send_header('Connection', 'close')
                self.end_headers()
                try:
                    for jpg in node.jpeg_frames():
                        if jpg is None:
                            continue
                        self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n'
                                         % len(jpg))
                        self.wfile.write(jpg)
                        self.wfile.write(b'\r\n')
                except (BrokenPipeError, ConnectionResetError):
                    pass
                self.close_connection = True

        return Handler


def main(args=None):
    run(TeleopWeb, args)
