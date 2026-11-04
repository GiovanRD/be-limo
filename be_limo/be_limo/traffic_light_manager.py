"""Traffic lights of the simulated track (Gazebo only).

The four traffic lights of the central crossing are static housings in the
world file. This node switches them on by spawning a small emissive disc
(red or green) in front of each housing, and deleting the previous one,
through the Gazebo services /spawn_entity and /delete_entity.

Lights "west" and "east" control the east-west road, "north" and "south"
the north-south road. In automatic mode, the phases alternate every
`period` seconds.

Inputs
  /supervisor/traffic_lights  std_msgs/String   "auto", "ew_green", "ns_green",
                                                "all_red" or "all_green"
Outputs
  /traffic_lights/state       std_msgs/String (JSON)  {"mode": ..., "lights": {...}}
"""

import json
import math

from gazebo_msgs.srv import DeleteEntity, SpawnEntity
from geometry_msgs.msg import Pose
from rclpy.node import Node
from std_msgs.msg import String

from be_limo.ros_utils import LATCHED, quaternion_from_yaw, run

# name: (x, y, yaw of the face, road)  -- world frame, see worlds/be_limo_track.world
LIGHTS = {
    # Each light stands on the far right corner of the crossing, facing the
    # vehicles it controls, so that the camera sees it from the stop line.
    'west': (0.54, -1.21, math.pi, 'ew'),         # vehicles coming from the west (node 9)
    'east': (-0.51, -0.26, 0.0, 'ew'),            # from the east (node 6)
    'north': (-0.51, -1.21, math.pi / 2, 'ns'),   # from the north (node 12)
    'south': (0.54, -0.26, -math.pi / 2, 'ns'),   # from the south (node 1)
}
BULB_HEIGHT = {'red': 0.42, 'green': 0.33}
FACE_OFFSET = 0.046      # in front of the housing (half depth 0.04) and of the dim bulbs

BULB_SDF = """<?xml version="1.0"?>
<sdf version="1.7">
  <model name="{name}">
    <static>true</static>
    <link name="link">
      <visual name="bulb">
        <pose>0 0 0 0 1.5708 0</pose>
        <geometry><cylinder><radius>0.017</radius><length>0.004</length></cylinder></geometry>
        <material>
          <ambient>{r} {g} 0.05 1</ambient>
          <diffuse>{r} {g} 0.05 1</diffuse>
          <emissive>{r} {g} 0 1</emissive>
        </material>
      </visual>
    </link>
  </model>
</sdf>"""


class TrafficLightManager(Node):

    def __init__(self):
        super().__init__('traffic_light_manager')
        self.declare_parameter('period', 12.0)
        self.declare_parameter('mode', 'auto')
        self.mode = self.get_parameter('mode').value
        self.phase = 'ew'                  # road with the green light in auto mode
        self.current = {}                  # light -> colour shown in Gazebo
        self.spawn = self.create_client(SpawnEntity, '/spawn_entity')
        self.delete = self.create_client(DeleteEntity, '/delete_entity')
        self.state_pub = self.create_publisher(String, '/traffic_lights/state', LATCHED)
        self.create_subscription(String, '/supervisor/traffic_lights', self.on_mode, 10)
        self.create_timer(self.get_parameter('period').value, self.on_period)
        self.create_timer(1.0, self.apply)

    def on_mode(self, msg):
        mode = msg.data.strip().lower()
        if mode in ('auto', 'ew_green', 'ns_green', 'all_red', 'all_green'):
            self.mode = mode
            self.get_logger().info('Feux : mode %s' % mode)
            self.apply()

    def on_period(self):
        if self.mode == 'auto':
            self.phase = 'ns' if self.phase == 'ew' else 'ew'
            self.apply()

    def wanted(self):
        out = {}
        for name, (_, _, _, road) in LIGHTS.items():
            if self.mode == 'all_red':
                out[name] = 'red'
            elif self.mode == 'all_green':
                out[name] = 'green'
            else:
                green_road = {'auto': self.phase, 'ew_green': 'ew', 'ns_green': 'ns'}[self.mode]
                out[name] = 'green' if road == green_road else 'red'
        return out

    def apply(self):
        if not (self.spawn.service_is_ready() and self.delete.service_is_ready()):
            return
        wanted = self.wanted()
        for name, colour in wanted.items():
            if self.current.get(name) == colour:
                continue
            old = self.current.get(name)
            if old:
                req = DeleteEntity.Request()
                req.name = 'light_%s_%s' % (name, old)
                self.delete.call_async(req)
            self.spawn_bulb(name, colour)
            self.current[name] = colour
        self.state_pub.publish(String(data=json.dumps({'mode': self.mode, 'lights': wanted})))

    def spawn_bulb(self, name, colour):
        x, y, yaw, _ = LIGHTS[name]
        req = SpawnEntity.Request()
        req.name = 'light_%s_%s' % (name, colour)
        r, g = (1.0, 0.05) if colour == 'red' else (0.05, 1.0)
        req.xml = BULB_SDF.format(name=req.name, r=r, g=g)
        pose = Pose()
        pose.position.x = x + FACE_OFFSET * math.cos(yaw)
        pose.position.y = y + FACE_OFFSET * math.sin(yaw)
        pose.position.z = BULB_HEIGHT[colour]
        q = quaternion_from_yaw(yaw)
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = q
        req.initial_pose = pose
        req.reference_frame = 'world'
        self.spawn.call_async(req)


def main(args=None):
    run(TrafficLightManager, args)
