"""Closed-loop test of the ROS nodes, without ROS and without Gazebo.

Scenario (see offline_sim.py): the demonstration loop
2 -> 14 -> 20 -> 22 -> 9 -> 0 -> 2 with
  - the stop sign before the roundabout,
  - the obstacle on the top lane (removed after 3 s of waiting),
  - the traffic light at the central crossing, red until the robot has
    waited 3 s in front of it.
"""
import os
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
pytest.importorskip('cv2')


def test_demo_loop():
    from sim_camera import find_texture
    if find_texture() is None:
        pytest.skip('track texture not found')
    from offline_sim import BUS, OfflineSim

    sim = OfflineSim()
    while BUS.now < 240.0 and 'GOAL_REACHED' not in sim.state_sequence():
        sim.step()

    seen = sim.state_sequence()
    ctes = [abs(s['cross_track_error']) for s in sim.states
            if s['state'] == 'DRIVING' and 'cross_track_error' in s]
    errors = [entry for entry in BUS.log if entry[2] == 'ERROR']
    print(' > '.join(seen))
    print('%.1f s, max cross-track %.3f m, closest to the box %.2f m' % (
        BUS.now, max(ctes), sim.min_box_distance))
    assert not errors, errors
    assert 'GOAL_REACHED' in seen, seen
    for expected in ('STOP_SIGN', 'STOP_OBSTACLE', 'STOP_RED_LIGHT'):
        assert expected in seen, seen
    assert seen.index('STOP_SIGN') < seen.index('STOP_OBSTACLE') < seen.index('STOP_RED_LIGHT')
    assert max(ctes) < 0.08
    assert sim.min_box_distance > 0.30
    assert not sim.light_red and not sim.box_present
