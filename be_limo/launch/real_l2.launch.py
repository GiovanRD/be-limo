"""Livrable 2 - robot réel : planification, contrôle, détection sur la LIMO.

À lancer sur le robot (SSH), une fois le robot posé sur la piste au noeud de départ,
dans l'axe de la voie :

    ros2 launch be_limo real_l2.launch.py initial_node:=2
    ros2 launch be_limo real_l2.launch.py initial_node:=2 driver:=false   # si limo_start tourne déjà

Puis démarrer depuis l'interface de supervision (sur le PC) : ros2 launch be_limo supervisor.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def nodes(context):
    share = get_package_share_directory('be_limo')
    params = os.path.join(share, 'config', 'be_limo.yaml')
    cfg = lambda name: LaunchConfiguration(name).perform(context)
    actions = []
    if cfg('driver') == 'true':
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('limo_start'), 'launch', 'limo_real.launch.py')),
            launch_arguments={'use_camera': cfg('use_camera')}.items()))
    actions += [
        Node(package='be_limo', executable='localization_node', output='screen',
             parameters=[{'odom_is_world': False, 'initial_node': int(cfg('initial_node'))}]),
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
        DeclareLaunchArgument('use_camera', default_value='false'),
        DeclareLaunchArgument('mission', default_value=''),
        DeclareLaunchArgument('speed', default_value='0.15', description='vitesse nominale [m/s]'),
        OpaqueFunction(function=nodes),
    ])
