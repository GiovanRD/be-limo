"""Téléopération web de la LIMO : à lancer SUR LE ROBOT.

    ros2 launch be_limo teleop.launch.py                    # driver + lidar + caméra + interface
    ros2 launch be_limo teleop.launch.py driver:=false      # si limo_start ou real_l2 tourne déjà
    ros2 launch be_limo teleop.launch.py max_speed:=0.2

Puis sur le PC (hors conteneur) :
    ssh -N -L 8080:localhost:8080 jetson@100.75.155.201
et ouvrir http://localhost:8080 dans le navigateur.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def nodes(context):
    cfg = lambda name: LaunchConfiguration(name).perform(context)
    actions = []
    if cfg('driver') == 'true':
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('limo_start'), 'launch', 'limo_real.launch.py')),
            launch_arguments={'use_camera': cfg('use_camera')}.items()))
    actions.append(Node(package='be_limo', executable='teleop_web', output='screen', parameters=[{
        'host': cfg('host'), 'port': int(cfg('port')),
        'max_speed': float(cfg('max_speed')), 'image_topic': cfg('image_topic'),
        'imu_topic': cfg('imu_topic')}]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('driver', default_value='true',
                              description='lancer aussi limo_start (driver, lidar, caméra)'),
        DeclareLaunchArgument('use_camera', default_value='true'),
        DeclareLaunchArgument('host', default_value='127.0.0.1',
                              description='127.0.0.1 = accès par tunnel SSH seulement ; 0.0.0.0 = tout le réseau'),
        DeclareLaunchArgument('port', default_value='8080'),
        DeclareLaunchArgument('max_speed', default_value='0.30', description='vitesse max de l\'interface [m/s]'),
        DeclareLaunchArgument('image_topic', default_value='/rgb/image_raw'),
        DeclareLaunchArgument('imu_topic', default_value='/limo/imu'),
        OpaqueFunction(function=nodes),
    ])
