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
    if cfg('localization') == 'lidar':
        map_yaml = os.path.expanduser(cfg('map'))
        amcl = os.path.join(share, 'config', 'amcl.yaml')
        actions += [
            Node(package='nav2_map_server', executable='map_server', name='map_server', output='screen',
                 parameters=[amcl, {'yaml_filename': map_yaml}]),
            Node(package='nav2_amcl', executable='amcl', name='amcl', output='screen', parameters=[amcl]),
            Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                 name='lifecycle_manager_localization', output='screen',
                 parameters=[{'autostart': True, 'node_names': ['map_server', 'amcl']}]),
            Node(package='be_limo', executable='localization_node', output='screen',
                 parameters=[{'mode': 'lidar', 'map_yaml': map_yaml,
                              'initial_node': int(cfg('initial_node'))}]),
        ]
    else:
        actions.append(
            Node(package='be_limo', executable='localization_node', output='screen',
                 parameters=[{'odom_is_world': False, 'initial_node': int(cfg('initial_node'))}]))
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
        DeclareLaunchArgument('use_camera', default_value='false'),
        DeclareLaunchArgument('mission', default_value=''),
        DeclareLaunchArgument('speed', default_value='0.15', description='vitesse nominale [m/s]'),
        DeclareLaunchArgument('localization', default_value='odom',
                              description='"odom" (départ au noeud initial_node) ou "lidar" (AMCL)'),
        DeclareLaunchArgument('map', default_value='~/maps/salle207.yaml',
                              description='carte lidar de la salle (localization:=lidar)'),
        OpaqueFunction(function=nodes),
    ])
