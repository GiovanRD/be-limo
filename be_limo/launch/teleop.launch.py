"""Pilote web de la LIMO (manuel, route, suivi) : à lancer SUR LE ROBOT.

    ros2 launch be_limo teleop.launch.py                          # tout : driver, caméra, vision, position
    ros2 launch be_limo teleop.launch.py initial_node:=9          # robot posé au noeud 9
    ros2 launch be_limo teleop.launch.py localization:=lidar map:=~/maps/salle207.yaml
    ros2 launch be_limo teleop.launch.py driver:=false localization:=none   # avec mapping.launch.py

Puis sur le PC (hors conteneur) :
    ssh -N -L 8080:localhost:8080 jetson@100.75.155.201
et ouvrir http://localhost:8080 dans le navigateur.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from be_limo.launch_helpers import driver_actions, localization_actions


def nodes(context):
    cfg = lambda name: LaunchConfiguration(name).perform(context)
    params = os.path.join(get_package_share_directory('be_limo'), 'config', 'be_limo.yaml')
    actions = driver_actions(cfg('driver'), cfg('use_camera'), cfg('odom_imu'))
    actions += localization_actions(cfg('localization'), cfg('initial_node'), cfg('map'))
    if cfg('vision') == 'true' and cfg('use_camera') == 'true':
        # Détection de la voie pour le mode route (une image sur deux : Jetson Nano).
        actions.append(Node(package='be_limo', executable='vision_node', output='screen',
                            parameters=[params, {'process_every': int(cfg('vision_every')),
                                                 'image_topic': cfg('image_topic')}]))
    actions.append(Node(package='be_limo', executable='teleop_web', output='screen', parameters=[{
        'host': cfg('host'), 'port': int(cfg('port')),
        'max_speed': float(cfg('max_speed')), 'image_topic': cfg('image_topic'),
        'imu_topic': cfg('imu_topic')}]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('driver', default_value='true',
                              description='lancer aussi limo_start (driver, lidar, caméra)'),
        DeclareLaunchArgument('odom_imu', default_value='auto',
                              description='odométrie corrigée par le gyroscope : auto (avec le driver), true, false'),
        DeclareLaunchArgument('use_camera', default_value='true'),
        DeclareLaunchArgument('vision', default_value='true', description='détection de voie (mode route)'),
        DeclareLaunchArgument('vision_every', default_value='2', description='traiter une image sur N'),
        DeclareLaunchArgument('localization', default_value='odom',
                              description='odom (départ sur initial_node), lidar (AMCL) ou none'),
        DeclareLaunchArgument('initial_node', default_value='2',
                              description='noeud de la piste où le robot est posé au démarrage'),
        DeclareLaunchArgument('map', default_value='~/maps/salle207.yaml',
                              description='carte lidar de la salle (localization:=lidar)'),
        DeclareLaunchArgument('host', default_value='127.0.0.1',
                              description='127.0.0.1 = accès par tunnel SSH seulement ; 0.0.0.0 = tout le réseau'),
        DeclareLaunchArgument('port', default_value='8080'),
        DeclareLaunchArgument('max_speed', default_value='0.30', description='vitesse max de l\'interface [m/s]'),
        DeclareLaunchArgument('image_topic', default_value='/rgb/image_raw'),
        DeclareLaunchArgument('imu_topic', default_value='/limo/imu'),
        OpaqueFunction(function=nodes),
    ])
