"""Livrable 2 - simulation complète : Gazebo + LIMO + planification, contrôle, détection.

    ros2 launch be_limo sim_l2.launch.py                 # sans fenêtre Gazebo (PC lent)
    ros2 launch be_limo sim_l2.launch.py gui:=true       # avec la fenêtre Gazebo
    ros2 launch be_limo sim_l2.launch.py mission:=demo autostart:=true

Arguments : gui, rviz, supervisor, start_node, mission, autostart.
"""

import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
                            SetEnvironmentVariable, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def nodes(context):
    share = get_package_share_directory('be_limo')
    params = os.path.join(share, 'config', 'be_limo.yaml')
    sim = {'use_sim_time': True}
    cfg = lambda name: LaunchConfiguration(name).perform(context)

    from be_limo.roadmap import node_poses
    x, y, yaw = node_poses()[int(cfg('start_node'))]

    gazebo_ros = get_package_share_directory('gazebo_ros')
    world = os.path.join(share, 'worlds', 'be_limo_track.world')
    xacro_file = os.path.join(get_package_share_directory('limo_car'), 'gazebo',
                              'ackermann_with_sensor.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()

    actions = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo_ros, 'launch', 'gzserver.launch.py')),
            launch_arguments={'world': world}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo_ros, 'launch', 'gzclient.launch.py')),
            condition=IfCondition(LaunchConfiguration('gui'))),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description}, sim], output='screen'),
        TimerAction(period=3.0, actions=[Node(
            package='gazebo_ros', executable='spawn_entity.py', output='screen',
            arguments=['-topic', 'robot_description', '-entity', 'limo',
                       '-x', str(x), '-y', str(y), '-z', '0.15', '-Y', str(yaw)])]),
        # /cmd_vel (v, omega) -> /cmd_vel_gazebo (v, angle), as on the real robot
        Node(package='limo_car', executable='gazebo_twist_adapter.py', parameters=[sim]),
        Node(package='be_limo', executable='traffic_light_manager', parameters=[params, sim]),
        Node(package='be_limo', executable='localization_node',
             parameters=[{'odom_is_world': True}, sim]),
        Node(package='be_limo', executable='global_planner', output='screen',
             parameters=[params, {'mission': cfg('mission')}, sim]),
        Node(package='be_limo', executable='local_controller', output='screen',
             parameters=[params, {'autostart': cfg('autostart') == 'true'}, sim]),
        Node(package='be_limo', executable='obstacle_detector', parameters=[params, sim]),
        Node(package='be_limo', executable='vision_node', parameters=[params, sim]),
        Node(package='rviz2', executable='rviz2', arguments=['-d', os.path.join(share, 'rviz', 'be_limo.rviz')],
             parameters=[sim], condition=IfCondition(LaunchConfiguration('rviz'))),
        Node(package='be_limo', executable='supervisor', parameters=[sim],
             condition=IfCondition(LaunchConfiguration('supervisor'))),
    ]
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='false', description='fenêtre Gazebo'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('supervisor', default_value='true', description='interface de supervision'),
        DeclareLaunchArgument('start_node', default_value='2', description='noeud de départ du robot'),
        DeclareLaunchArgument('mission', default_value='demo',
                              description='"demo" (boucle), "14" ou "14,20,9"'),
        DeclareLaunchArgument('autostart', default_value='false',
                              description='démarrer sans attendre le bouton Démarrer'),
        # Gazebo ne cherche pas de modèles sur Internet (sinon "Preparing your world" bloque).
        SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', ''),
        OpaqueFunction(function=nodes),
    ])
