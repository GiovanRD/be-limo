import os

import xacro

from ament_index_python.packages import (
    get_package_prefix,
    get_package_share_directory,
)

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Launch the Room 207 simulation environment."""

    # ============================================================
    # Packages
    # ============================================================

    room207_limo_pkg = get_package_share_directory(
        'room207_limo'
    )

    ensem_room207_pkg = get_package_share_directory(
        'ensem_room207'
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

    # Room 207 by default; the bare track world can be selected with
    # world:=$(ros2 pkg prefix qcar_large_track)/share/qcar_large_track/worlds/qcar_large_track.world
    world = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(
            ensem_room207_pkg,
            'worlds',
            'ensem_room207.world'
        ),
        description='Gazebo world file.',
    )

    # The Room 207 track is on a 20 cm platform; use spawn_z:=0.15 with
    # the qcar_large_track world (track on the ground).
    spawn_z = DeclareLaunchArgument(
        'spawn_z',
        default_value='0.35',
        description='Spawn height of the LIMO in metres.',
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

    # The Room 207 world loads a GUI plugin from ensem_room207 (hides the
    # green light markers); make sure gzclient finds it.
    plugin_path = SetEnvironmentVariable(
        'GAZEBO_PLUGIN_PATH',
        os.pathsep.join([
            os.path.join(get_package_prefix('ensem_room207'), 'lib'),
            os.environ.get('GAZEBO_PLUGIN_PATH', ''),
        ]),
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                gazebo_ros_pkg,
                'launch',
                'gazebo.launch.py'
            )
        ),
        launch_arguments={
            'world': LaunchConfiguration('world'),
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
                    '-z', LaunchConfiguration('spawn_z'),
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
        world,
        spawn_z,
        plugin_path,
        gazebo,
        robot_state_publisher,
        spawn_limo,
        rviz,
    ])