"""Livrable 2 - robot réel : planification, contrôle, détection sur la LIMO.

À lancer sur le robot (SSH), une fois le robot posé sur la piste au noeud de départ,
dans l'axe de la voie :

    ros2 launch be_limo real_l2.launch.py initial_node:=2
    ros2 launch be_limo real_l2.launch.py initial_node:=2 driver:=false   # si limo_start tourne déjà
    ros2 launch be_limo real_l2.launch.py localization:=lidar map:=~/maps/salle207.yaml initial_node:=2

Avec localization:=lidar, le robot est localisé par AMCL sur la carte de la salle (faite avec
mapping.launch.py) : il peut partir de n'importe quel noeud (initial_node), ou d'ailleurs en
donnant sa position avec RViz "2D Pose Estimate".

Puis démarrer depuis l'interface de supervision (sur le PC) : ros2 launch be_limo supervisor.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from be_limo.launch_helpers import driver_actions, localization_actions


def nodes(context):
    share = get_package_share_directory('be_limo')
    params = os.path.join(share, 'config', 'be_limo.yaml')
    cfg = lambda name: LaunchConfiguration(name).perform(context)
    actions = driver_actions(cfg('driver'), cfg('use_camera'), cfg('odom_imu'))
    actions += localization_actions(cfg('localization'), cfg('initial_node'), cfg('map'))
    actions += [
        Node(package='be_limo', executable='global_planner', output='screen',
             parameters=[params, {'mission': cfg('mission')}]),
        Node(package='be_limo', executable='local_controller', output='screen',
             parameters=[params, {'autostart': False, 'speed': float(cfg('speed'))}]),
        Node(package='be_limo', executable='obstacle_detector', parameters=[params]),
        Node(package='be_limo', executable='vision_node', parameters=[params],
             condition=IfCondition(LaunchConfiguration('use_camera'))),
    ]
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('initial_node', default_value='2',
                              description='noeud de la carte où le robot est posé au démarrage'),
        DeclareLaunchArgument('driver', default_value='true',
                              description='lancer aussi limo_start (driver, lidar)'),
        DeclareLaunchArgument('odom_imu', default_value='auto',
                              description='odométrie corrigée par le gyroscope : auto (avec le driver), true, false'),
        DeclareLaunchArgument('use_camera', default_value='false'),
        DeclareLaunchArgument('mission', default_value=''),
        DeclareLaunchArgument('speed', default_value='0.15', description='vitesse nominale [m/s]'),
        DeclareLaunchArgument('localization', default_value='odom',
                              description='"odom" (départ au noeud initial_node) ou "lidar" (AMCL)'),
        DeclareLaunchArgument('map', default_value='~/maps/salle207.yaml',
                              description='carte lidar de la salle (localization:=lidar)'),
        OpaqueFunction(function=nodes),
    ])
