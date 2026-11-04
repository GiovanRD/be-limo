"""Cartographie de la salle 207 avec le lidar (à faire une fois).

Le robot doit être posé sur un noeud connu de la piste (par défaut le 2) :
la carte lidar commence là, ce qui la relie à la carte de la piste.

    ros2 launch be_limo mapping.launch.py start_node:=2 map:=~/maps/salle207.yaml

Puis piloter le robot au clavier sur toute la piste (autre terminal) :
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
et enregistrer la carte (autre terminal) :
    ros2 run nav2_map_server map_saver_cli -f ~/maps/salle207
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def nodes(context):
    share = get_package_share_directory('be_limo')
    cfg = lambda name: LaunchConfiguration(name).perform(context)
    map_yaml = os.path.expanduser(cfg('map'))
    actions = []
    if cfg('driver') == 'true':
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('limo_start'), 'launch', 'limo_real.launch.py')),
            launch_arguments={'use_camera': 'false'}.items()))
    slam_defaults = os.path.join(get_package_share_directory('slam_toolbox'), 'config',
                                 'mapper_params_online_async.yaml')
    params = [os.path.join(share, 'config', 'slam.yaml')]
    if os.path.exists(slam_defaults):
        params.insert(0, slam_defaults)
    actions += [
        Node(package='slam_toolbox', executable='async_slam_toolbox_node', name='slam_toolbox',
             output='screen', parameters=params),
        # Writes <map>_track.yaml (link room_map -> track) and publishes /robot_pose.
        Node(package='be_limo', executable='localization_node', output='screen',
             parameters=[{'mode': 'lidar', 'map_yaml': map_yaml, 'save_calibration': True,
                          'send_initial_pose': False,
                          'initial_node': int(cfg('start_node'))}]),
    ]
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('start_node', default_value='2',
                              description='noeud de la piste où le robot est posé au départ'),
        DeclareLaunchArgument('map', default_value='~/maps/salle207.yaml',
                              description='carte à enregistrer ensuite avec map_saver_cli'),
        DeclareLaunchArgument('driver', default_value='true',
                              description='lancer aussi limo_start (driver, lidar)'),
        OpaqueFunction(function=nodes),
    ])
