"""Synthetic camera images of the track, for the tests (no Gazebo needed).

Each pixel below the horizon is projected on the ground (flat-ground
pinhole model) and takes the colour of the track texture at that point.
"""
import os

import numpy as np

from be_limo.lane_vision import CameraModel

TRACK_W, TRACK_H = 4.88, 6.20


def find_texture():
    here = os.path.dirname(os.path.abspath(__file__))
    for root in (os.path.join(here, '..', '..'), os.path.join(here, '..', '..', '..')):
        path = os.path.join(root, 'environment', 'qcar_large_track', 'models',
                            'qcar_large_track', 'materials', 'textures', 'SDCS_MapLayout.png')
        if os.path.exists(path):
            return path
    return None


def render(texture_bgr, pose, cam=None, sky=(170, 170, 170)):
    cam = cam or CameraModel()
    th, tw = texture_bgr.shape[:2]
    uu, vv = np.meshgrid(np.arange(cam.width), np.arange(cam.height))
    gx, gy = cam.pixel_to_ground(uu, vv)
    x, y, yaw = pose
    wx = x + np.cos(yaw) * gx - np.sin(yaw) * gy
    wy = y + np.sin(yaw) * gx + np.cos(yaw) * gy
    tu = ((wx / TRACK_W + 0.5) * tw).astype(float)
    tv = ((0.5 - wy / TRACK_H) * th).astype(float)
    ok = np.isfinite(tu) & np.isfinite(tv) & (tu >= 0) & (tu < tw) & (tv >= 0) & (tv < th)
    img = np.empty((cam.height, cam.width, 3), np.uint8)
    img[:] = sky
    img[ok] = texture_bgr[tv[ok].astype(int), tu[ok].astype(int)]
    return img
