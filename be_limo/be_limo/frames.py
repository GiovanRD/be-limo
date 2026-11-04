"""2D poses (x, y, yaw) and the link between the room map and the track map.

Two maps are used on the real robot:

* "map": the track frame of be_limo (road map, planned path), the same as
  in Gazebo: the mat centred on the origin.
* "room_map": the lidar map of room 207 built by slam_toolbox, in which AMCL
  localises the robot.

The mapping session starts with the robot on a known road-map node. The
lidar map starts where the robot is at that moment, so the pose of the
room_map origin in the track frame is simply the pose of that node. It is
saved next to the map (<map>_track.yaml) and reused by the localisation.

No ROS dependency.
"""

import math
import os


def compose(a, b):
    """Pose a (x, y, yaw) followed by pose b expressed in a."""
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1],
            math.atan2(math.sin(a[2] + b[2]), math.cos(a[2] + b[2])))


def inverse(a):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (-c * a[0] - s * a[1], s * a[0] - c * a[1], -a[2])


def calibration_path(map_yaml):
    """~/maps/salle207.yaml -> ~/maps/salle207_track.yaml"""
    path = os.path.expanduser(map_yaml)
    root, _ = os.path.splitext(path)
    return root + '_track.yaml'


def save_calibration(path, pose, node=None):
    """Pose of the room_map origin in the track frame."""
    path = os.path.expanduser(path)
    os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
    with open(path, 'w') as f:
        f.write('# Position de l\'origine de la carte lidar (room_map) dans le repère de la piste (map).\n')
        if node is not None:
            f.write('# Cartographie commencée au noeud %d.\n' % node)
        f.write('x: %.6f\ny: %.6f\nyaw: %.6f\n' % pose)
        if node is not None:
            f.write('start_node: %d\n' % node)


def load_calibration(path):
    """(x, y, yaw) read from a file written by save_calibration."""
    values = {}
    with open(os.path.expanduser(path)) as f:
        for line in f:
            line = line.split('#', 1)[0].strip()
            if ':' in line:
                key, value = line.split(':', 1)
                values[key.strip()] = float(value)
    return values['x'], values['y'], values['yaw']
