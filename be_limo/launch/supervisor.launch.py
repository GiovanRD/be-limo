"""Livrable 3 - interface de supervision (sur le PC), pour le robot réel ou la simulation.

    ros2 launch be_limo supervisor.launch.py
    ros2 launch be_limo supervisor.launch.py rviz:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('be_limo')
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='false'),
        Node(package='be_limo', executable='supervisor', output='screen'),
        Node(package='rviz2', executable='rviz2', arguments=['-d', os.path.join(share, 'rviz', 'be_limo.rviz')],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
