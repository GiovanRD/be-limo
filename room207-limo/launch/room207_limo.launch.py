import os

import xacro

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    """Launch the Room 207 simulation environment."""

    # ============================================================
    # Packages
    # ============================================================

    room207_limo_pkg = get_package_share_directory(
        'room207_limo'
    )

    qcar_large_track_pkg = get_package_share_directory(
        'qcar_large_track'
    )

    limo_car_pkg = get_package_share_directory(
        'limo_car'
    )

    gazebo_ros_pkg = get_package_share_directory(
        'gazebo_ros'
    )

    # ============================================================
    # World
    # ============================================================

    world_file = os.path.join(
        qcar_large_track_pkg,
        'worlds',
        'qcar_large_track.world'
    )

    # ============================================================
    # LIMO robot description
    # ============================================================

    xacro_file = os.path.join(
        limo_car_pkg,
        'gazebo',
        'ackermann_with_sensor.xacro'
    )

    robot_description = xacro.process_file(
        xacro_file
    ).toxml()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {
                'robot_description': robot_description,
                'use_sim_time': True,
            }
        ],
    )

    # ============================================================
    # Gazebo
    # ============================================================

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                gazebo_ros_pkg,
                'launch',
                'gazebo.launch.py'
            )
        ),
        launch_arguments={
            'world': world_file,
        }.items(),
    )

    # ============================================================
    # Spawn LIMO
    # ============================================================

    # Wait for Gazebo to be ready before spawning the robot.
    spawn_limo = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-topic', 'robot_description',
                    '-entity', 'limo',
                    '-x', '-0.725347',
                    '-y', '-0.086613',
                    '-z', '0.15',
                    '-Y', '1.5708',
                ],
                output='screen',
            )
        ],
    )

    # ============================================================
    # RViz
    # ============================================================

    rviz_config = os.path.join(
        room207_limo_pkg,
        'rviz',
        'room207_limo.rviz'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=[
            '-d',
            rviz_config,
        ],
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_limo,
        rviz,
    ])