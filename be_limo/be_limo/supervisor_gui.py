"""Supervision interface of the LIMO (deliverable 3).

A Tkinter window, connected to ROS 2, that shows and drives the whole
system, in simulation or with the real robot:

* map of the track: road graph, planned route, robot and its trail,
  obstacles seen by the lidar, traffic lights;
* state of the controller (state machine, speed, tracking error, progress);
* camera view with the detections (/vision/debug);
* event log (state changes, traffic lights, stop signs, obstacles);
* commands: start / pause / emergency stop, speed, goal or mission
  (click a node on the map), and in simulation the traffic lights and the
  obstacle.

Topics used: see README of the be_limo package.
"""

import base64
import json
import math
import os
import time
import tkinter as tk
from tkinter import ttk

import cv2
import rclpy
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, String

from be_limo.roadmap import RoadMap
from be_limo.ros_utils import LATCHED, SENSOR, image_to_bgr, yaw_from_quaternion

TRACK_W, TRACK_H = 4.88, 6.20
LIGHT_POSITIONS = {'west': (0.54, -1.21), 'east': (-0.51, -0.26),
                   'north': (-0.51, -1.21), 'south': (0.54, -0.26)}
STOP_SIGN = (2.36, 1.75)
OBSTACLE_HOME = (-0.70, 2.795)
STATE_COLOURS = {
    'DRIVING': '#2e9d4a', 'SLOW_OBSTACLE': '#d9a400', 'GOAL_REACHED': '#2a6fdb',
    'STOP_OBSTACLE': '#d9480f', 'STOP_RED_LIGHT': '#c92a2a', 'STOP_SIGN': '#c92a2a',
    'EMERGENCY_STOP': '#a00000', 'PAUSED': '#666666', 'NO_PLAN': '#666666',
    'NO_POSE': '#a06000',
}
STATE_LABELS = {
    'DRIVING': 'En route', 'SLOW_OBSTACLE': 'Ralentit (obstacle)', 'GOAL_REACHED': 'Arrivé',
    'STOP_OBSTACLE': 'Arrêt : obstacle', 'STOP_RED_LIGHT': 'Arrêt : feu rouge',
    'STOP_SIGN': 'Arrêt : stop', 'EMERGENCY_STOP': "ARRÊT D'URGENCE", 'PAUSED': 'En pause',
    'NO_PLAN': 'Pas de chemin', 'NO_POSE': 'Pas de position',
}
OBSTACLE_SDF = """<?xml version="1.0"?>
<sdf version="1.7"><model name="obstacle_box"><static>true</static><link name="link">
<collision name="c"><geometry><box><size>0.12 0.12 0.15</size></box></geometry></collision>
<visual name="v"><geometry><box><size>0.12 0.12 0.15</size></box></geometry>
<material><ambient>0.9 0.45 0.05 1</ambient><diffuse>0.9 0.45 0.05 1</diffuse></material>
</visual></link></model></sdf>"""


def find_texture():
    try:
        from ament_index_python.packages import get_package_share_directory
        share = get_package_share_directory('qcar_large_track')
        path = os.path.join(share, 'models', 'qcar_large_track', 'materials', 'textures',
                            'SDCS_MapLayout.png')
        return path if os.path.exists(path) else None
    except Exception:  # package not installed on this computer
        return None


class SupervisorNode(Node):
    """ROS side: stores the last messages, the GUI reads them."""

    def __init__(self):
        super().__init__('supervisor')
        self.pose = None
        self.trail = []
        self.plan = []
        self.route = {}
        self.ctrl = {}
        self.obstacles = []
        self.lane = None
        self.events = []
        self.lights = {}
        self.debug_image = None
        self.new_image = False
        self.log = []
        self.cmd_pub = self.create_publisher(String, '/supervisor/command', 10)
        self.goal_pub = self.create_publisher(String, '/planner/goal', 10)
        self.light_pub = self.create_publisher(String, '/supervisor/traffic_lights', 10)
        self.create_subscription(PoseStamped, '/robot_pose', self.on_pose, 10)
        self.create_subscription(Path, '/plan', self.on_plan, LATCHED)
        self.create_subscription(String, '/plan/route', self.on_route, LATCHED)
        self.create_subscription(String, '/controller/state', self.on_ctrl, 10)
        self.create_subscription(PoseArray, '/obstacles', self.on_obstacles, 10)
        self.create_subscription(Float32MultiArray, '/lane_detection', self.on_lane, 10)
        self.create_subscription(String, '/detected_events', self.on_events, 10)
        self.create_subscription(String, '/traffic_lights/state', self.on_lights, LATCHED)
        self.create_subscription(Image, '/vision/debug', self.on_image, SENSOR)
        self.spawn_client = self.delete_client = None
        self.pending_spawn = None
        try:
            from gazebo_msgs.srv import DeleteEntity, SpawnEntity
            self.SpawnEntity, self.DeleteEntity = SpawnEntity, DeleteEntity
            self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
            self.delete_client = self.create_client(DeleteEntity, '/delete_entity')
        except ImportError:
            pass

    def add_log(self, text):
        self.log.append(time.strftime('%H:%M:%S ') + text)
        self.log = self.log[-200:]

    # callbacks
    def on_pose(self, msg):
        p = msg.pose
        self.pose = (p.position.x, p.position.y, yaw_from_quaternion(p.orientation))
        if not self.trail or math.hypot(self.trail[-1][0] - self.pose[0],
                                        self.trail[-1][1] - self.pose[1]) > 0.03:
            self.trail.append(self.pose[:2])
            self.trail = self.trail[-800:]

    def on_plan(self, msg):
        self.plan = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]

    def on_route(self, msg):
        self.route = json.loads(msg.data)
        self.add_log('Itinéraire : %s (%.1f m)' % (
            ' > '.join(map(str, self.route.get('route', []))), self.route.get('length', 0)))

    def on_ctrl(self, msg):
        new = json.loads(msg.data)
        if new.get('state') != self.ctrl.get('state'):
            self.add_log('État : %s' % STATE_LABELS.get(new.get('state'), new.get('state')))
        # Keep the progress of the route while the robot is stopped.
        for key in ('progress', 'remaining'):
            if key not in new and new.get('state') not in ('NO_PLAN', 'GOAL_REACHED'):
                new[key] = self.ctrl.get(key)
        self.ctrl = new

    def on_obstacles(self, msg):
        self.obstacles = [(p.position.x, p.position.y) for p in msg.poses]

    def on_lane(self, msg):
        self.lane = tuple(msg.data)

    def on_events(self, msg):
        dets = json.loads(msg.data).get('detections', [])
        before = {(d['type'], d['state']) for d in self.events}
        for d in dets:
            if (d['type'], d['state']) not in before:
                name = ('Feu %s' % {'red': 'rouge', 'green': 'vert'}.get(d['state'], d['state'])
                        if d['type'] == 'traffic_light' else 'Panneau stop')
                self.add_log('Détecté : %s à %.2f m' % (name, d['distance']))
        self.events = dets

    def on_lights(self, msg):
        self.lights = json.loads(msg.data)

    def on_image(self, msg):
        try:
            self.debug_image = image_to_bgr(msg)
            self.new_image = True
        except ValueError:
            pass

    # commands
    def command(self, **kw):
        self.cmd_pub.publish(String(data=json.dumps(kw)))
        self.add_log('Commande : %s' % kw)

    def goal(self, text):
        self.goal_pub.publish(String(data=text))
        self.add_log('Objectif : %s' % text)

    def traffic_lights(self, mode):
        self.light_pub.publish(String(data=mode))
        self.add_log('Feux : %s' % mode)

    def sim_available(self):
        return bool(self.spawn_client and self.spawn_client.service_is_ready())

    def remove_obstacle(self):
        if not self.delete_client:
            return
        req = self.DeleteEntity.Request()
        req.name = 'obstacle_box'
        self.delete_client.call_async(req)
        self.add_log('Obstacle retiré')

    def place_obstacle(self, x, y):
        if not self.spawn_client:
            return
        self.remove_obstacle()
        req = self.SpawnEntity.Request()
        req.name = 'obstacle_box'
        req.xml = OBSTACLE_SDF
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = x, y, 0.075
        pose.orientation.w = 1.0
        req.initial_pose = pose
        req.reference_frame = 'world'
        # Spawned half a second later, once Gazebo has deleted the previous box.
        self.pending_spawn = (time.monotonic() + 0.5, req)
        self.add_log('Obstacle placé en (%.2f, %.2f)' % (x, y))

    def process_pending(self):
        if self.pending_spawn and time.monotonic() > self.pending_spawn[0]:
            self.spawn_client.call_async(self.pending_spawn[1])
            self.pending_spawn = None


class SupervisorApp:
    """Tkinter side."""

    def __init__(self, node):
        self.node = node
        self.roadmap = RoadMap()
        self.root = tk.Tk()
        self.root.title('LIMO - Supervision')
        self.root.protocol('WM_DELETE_WINDOW', self.close)
        self.closed = False
        self.photo = None
        self.cam_photo = None
        self.build()
        self.root.after(30, self.tick)

    # ------------------------------------------------------------------ layout
    def build(self):
        main = ttk.Frame(self.root, padding=6)
        main.pack(fill='both', expand=True)

        texture = find_texture()
        if texture:
            img = tk.PhotoImage(file=texture)
            factor = max(1, int(round(img.height() / 650)))
            self.photo = img.subsample(factor)
            w, h = self.photo.width(), self.photo.height()
        else:
            w, h = 504, 640
        self.scale = w / TRACK_W
        self.canvas = tk.Canvas(main, width=w, height=h, bg='#202020', highlightthickness=0)
        self.canvas.grid(row=0, column=0, rowspan=3, sticky='n')
        if self.photo:
            self.canvas.create_image(0, 0, image=self.photo, anchor='nw')
        self.draw_static()
        self.canvas.bind('<Button-1>', self.on_click)

        right = ttk.Frame(main, padding=(8, 0))
        right.grid(row=0, column=1, sticky='nw')

        # state
        box = ttk.LabelFrame(right, text='État', padding=6)
        box.pack(fill='x')
        self.state_label = tk.Label(box, text='—', font=('TkDefaultFont', 14, 'bold'),
                                    fg='white', bg='#666666', width=24, pady=4)
        self.state_label.pack(fill='x')
        self.info = {}
        grid = ttk.Frame(box)
        grid.pack(fill='x', pady=(6, 0))
        for i, (key, label) in enumerate([
                ('v', 'Vitesse'), ('omega', 'Vitesse angulaire'), ('cte', 'Écart à la trajectoire'),
                ('obstacle', 'Obstacle devant'), ('lane', 'Voie (caméra)'), ('route', 'Itinéraire'),
                ('remaining', 'Reste à parcourir'), ('pose', 'Position')]):
            ttk.Label(grid, text=label + ' :').grid(row=i, column=0, sticky='w')
            self.info[key] = ttk.Label(grid, text='—', width=30)
            self.info[key].grid(row=i, column=1, sticky='w')
        self.progress = ttk.Progressbar(box, maximum=1.0, length=300)
        self.progress.pack(fill='x', pady=(6, 0))

        # commands
        box = ttk.LabelFrame(right, text='Commande', padding=6)
        box.pack(fill='x', pady=(6, 0))
        row = ttk.Frame(box)
        row.pack(fill='x')
        ttk.Button(row, text='▶ Démarrer', command=lambda: self.node.command(cmd='start')).pack(side='left')
        ttk.Button(row, text='⏸ Pause', command=lambda: self.node.command(cmd='pause')).pack(side='left', padx=4)
        tk.Button(row, text="ARRÊT D'URGENCE", bg='#c92a2a', fg='white', activebackground='#a00000',
                  command=lambda: self.node.command(cmd='estop')).pack(side='left', padx=4)
        ttk.Button(row, text='Réarmer', command=lambda: self.node.command(cmd='reset')).pack(side='left')
        row = ttk.Frame(box)
        row.pack(fill='x', pady=(6, 0))
        ttk.Label(row, text='Vitesse (m/s)').pack(side='left')
        self.speed = tk.DoubleVar(value=0.25)
        tk.Scale(row, from_=0.05, to=0.5, resolution=0.05, orient='horizontal', variable=self.speed,
                 length=170, command=self.on_speed).pack(side='left', padx=4)
        row = ttk.Frame(box)
        row.pack(fill='x', pady=(6, 0))
        ttk.Label(row, text='Mission (noeuds)').pack(side='left')
        self.mission = tk.StringVar(value='14,20,22,9,0,2')
        ttk.Entry(row, textvariable=self.mission, width=16).pack(side='left', padx=4)
        ttk.Button(row, text='Envoyer', command=lambda: self.node.goal(self.mission.get())).pack(side='left')
        ttk.Button(row, text='Boucle démo', command=lambda: self.node.goal('demo')).pack(side='left', padx=4)
        ttk.Label(box, text='Astuce : cliquer sur un noeud de la carte pour y aller.',
                  foreground='#555').pack(anchor='w', pady=(4, 0))

        # simulation
        self.sim_box = ttk.LabelFrame(right, text='Simulation (Gazebo)', padding=6)
        self.sim_box.pack(fill='x', pady=(6, 0))
        row = ttk.Frame(self.sim_box)
        row.pack(fill='x')
        ttk.Label(row, text='Feux :').pack(side='left')
        for text, mode in (('Auto', 'auto'), ('E-O vert', 'ew_green'), ('N-S vert', 'ns_green'),
                           ('Tous rouges', 'all_red')):
            ttk.Button(row, text=text, command=lambda m=mode: self.node.traffic_lights(m)).pack(side='left', padx=2)
        row = ttk.Frame(self.sim_box)
        row.pack(fill='x', pady=(6, 0))
        ttk.Label(row, text='Obstacle :').pack(side='left')
        ttk.Button(row, text='Retirer', command=self.node.remove_obstacle).pack(side='left', padx=2)
        ttk.Button(row, text='Remettre', command=lambda: self.node.place_obstacle(*OBSTACLE_HOME)).pack(side='left', padx=2)
        ttk.Button(row, text='Devant le robot', command=self.obstacle_ahead).pack(side='left', padx=2)

        # camera + log
        box = ttk.LabelFrame(main, text='Caméra (détections)', padding=4)
        box.grid(row=0, column=2, sticky='n')
        self.cam_label = tk.Label(box, text='Pas d\'image /vision/debug', width=40, height=12, bg='#111', fg='#aaa')
        self.cam_label.pack()
        box = ttk.LabelFrame(main, text='Journal', padding=4)
        box.grid(row=1, column=1, columnspan=2, sticky='nsew', pady=(6, 0))
        self.log = tk.Listbox(box, height=12, width=90)
        self.log.pack(fill='both', expand=True)
        self.log_len = 0

    # ---------------------------------------------------------------- drawing
    def to_px(self, x, y):
        return (x + TRACK_W / 2) * self.scale, (TRACK_H / 2 - y) * self.scale

    def to_world(self, u, v):
        return u / self.scale - TRACK_W / 2, TRACK_H / 2 - v / self.scale

    def draw_static(self):
        for pts, _ in self.roadmap.edges.values():
            coords = [c for p in pts[::4] + [pts[-1]] for c in self.to_px(*p)]
            self.canvas.create_line(*coords, fill='#3a7d44', width=1)
        for i, (x, y, yaw) in enumerate(self.roadmap.nodes):
            u, v = self.to_px(x, y)
            self.canvas.create_line(u, v, u + 12 * math.cos(yaw), v - 12 * math.sin(yaw),
                                    fill='#ff4040', width=2, arrow='last')
            self.canvas.create_oval(u - 4, v - 4, u + 4, v + 4, fill='#ff4040', outline='')
            self.canvas.create_text(u + 9, v - 9, text=str(i), fill='#00e5ff',
                                    font=('TkDefaultFont', 9, 'bold'))
        u, v = self.to_px(*STOP_SIGN)
        self.canvas.create_oval(u - 6, v - 6, u + 6, v + 6, fill='#c00', outline='white', width=2)
        self.canvas.create_text(u - 22, v, text='STOP', fill='#c00', font=('TkDefaultFont', 8, 'bold'))

    def draw_dynamic(self):
        c = self.canvas
        c.delete('dyn')
        n = self.node
        if len(n.plan) > 1:
            coords = [k for p in n.plan[::3] + [n.plan[-1]] for k in self.to_px(*p)]
            c.create_line(*coords, fill='#1e90ff', width=4, tags='dyn')
        if len(n.trail) > 1:
            coords = [k for p in n.trail for k in self.to_px(*p)]
            c.create_line(*coords, fill='#ffd43b', width=2, tags='dyn')
        for name, (x, y) in LIGHT_POSITIONS.items():
            colour = n.lights.get('lights', {}).get(name)
            u, v = self.to_px(x, y)
            fill = {'red': '#ff2020', 'green': '#20e020'}.get(colour, '#555')
            c.create_rectangle(u - 6, v - 6, u + 6, v + 6, fill=fill, outline='black', tags='dyn')
        target = n.ctrl.get('target')
        if target and n.ctrl.get('state') in ('DRIVING', 'SLOW_OBSTACLE'):
            u, v = self.to_px(*target)
            c.create_oval(u - 4, v - 4, u + 4, v + 4, outline='#ff00ff', width=2, tags='dyn')
        if n.pose:
            x, y, yaw = n.pose
            for ox, oy in n.obstacles:
                wx = x + math.cos(yaw) * ox - math.sin(yaw) * oy
                wy = y + math.sin(yaw) * ox + math.cos(yaw) * oy
                u, v = self.to_px(wx, wy)
                c.create_oval(u - 6, v - 6, u + 6, v + 6, fill='#ff7f0e', outline='black', tags='dyn')
            pts = [(0.17, 0.0), (-0.12, 0.10), (-0.12, -0.10)]
            coords = []
            for px, py in pts:
                coords += self.to_px(x + math.cos(yaw) * px - math.sin(yaw) * py,
                                     y + math.sin(yaw) * px + math.cos(yaw) * py)
            colour = STATE_COLOURS.get(n.ctrl.get('state'), '#ffffff')
            c.create_polygon(*coords, fill=colour, outline='white', width=2, tags='dyn')

    def update_panels(self):
        n = self.node
        st = n.ctrl.get('state')
        self.state_label.config(text=STATE_LABELS.get(st, st or '—'),
                                bg=STATE_COLOURS.get(st, '#666666'))
        fmt = lambda v, f: '—' if v is None else f % v
        self.info['v'].config(text=fmt(n.ctrl.get('v'), '%.2f m/s'))
        self.info['omega'].config(text=fmt(n.ctrl.get('omega'), '%.2f rad/s'))
        self.info['cte'].config(text=fmt(n.ctrl.get('cross_track_error'), '%+.3f m'))
        self.info['obstacle'].config(text=fmt(n.ctrl.get('obstacle'), '%.2f m') if n.ctrl.get('obstacle') is not None else 'aucun')
        if n.lane and len(n.lane) >= 4 and n.lane[0] > 0.5:
            self.info['lane'].config(text='décalage %+.3f m, cap %+.1f°, conf. %.0f %%' % (
                n.lane[1], math.degrees(n.lane[2]), 100 * n.lane[3]))
        else:
            self.info['lane'].config(text='non détectée')
        self.info['route'].config(text=' > '.join(map(str, n.route.get('route', []))) or '—')
        self.info['remaining'].config(text=fmt(n.ctrl.get('remaining'), '%.2f m'))
        self.info['pose'].config(text='x %.2f  y %.2f  cap %.0f°' % (
            n.pose[0], n.pose[1], math.degrees(n.pose[2])) if n.pose else '—')
        self.progress['value'] = n.ctrl.get('progress', 0.0) or 0.0
        state = 'normal' if n.sim_available() else 'disabled'
        for child in self.sim_box.winfo_children():
            for w in child.winfo_children():
                if isinstance(w, ttk.Button):
                    w.state(['!disabled'] if state == 'normal' else ['disabled'])
        if len(n.log) != self.log_len:
            self.log.delete(0, 'end')
            for line in n.log[-200:]:
                self.log.insert('end', line)
            self.log.see('end')
            self.log_len = len(n.log)
        if n.new_image and n.debug_image is not None:
            n.new_image = False
            img = n.debug_image
            if img.shape[1] > 400:
                img = cv2.resize(img, (400, int(img.shape[0] * 400 / img.shape[1])))
            ok, png = cv2.imencode('.png', img)
            if ok:
                self.cam_photo = tk.PhotoImage(data=base64.b64encode(png.tobytes()))
                self.cam_label.config(image=self.cam_photo, width=img.shape[1], height=img.shape[0], text='')

    # ----------------------------------------------------------------- events
    def on_click(self, event):
        x, y = self.to_world(event.x, event.y)
        best, best_d = None, 0.15
        for i, (nx, ny, _) in enumerate(self.roadmap.nodes):
            d = math.hypot(nx - x, ny - y)
            if d < best_d:
                best, best_d = i, d
        if best is not None:
            self.mission.set(str(best))
            self.node.goal(str(best))

    def on_speed(self, _value):
        self.node.command(speed=float(self.speed.get()))

    def obstacle_ahead(self):
        if self.node.pose:
            x, y, yaw = self.node.pose
            self.node.place_obstacle(x + 0.7 * math.cos(yaw), y + 0.7 * math.sin(yaw))

    def tick(self):
        if self.closed:
            return
        if rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.0)
            for _ in range(20):
                rclpy.spin_once(self.node, timeout_sec=0.0)
            self.node.process_pending()
        self.draw_dynamic()
        self.update_panels()
        self.root.after(50, self.tick)

    def close(self):
        self.closed = True
        self.root.destroy()


def main(args=None):
    rclpy.init(args=args)
    node = SupervisorNode()
    app = SupervisorApp(node)
    try:
        app.root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
