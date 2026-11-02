import os

from ament_index_python.packages import (
    get_package_prefix,
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    """Open the ENSEM Room 207 world in Gazebo."""
    room_pkg = get_package_share_directory('ensem_room207')
    gazebo_ros_pkg = get_package_share_directory('gazebo_ros')

    world_file = os.path.join(room_pkg, 'worlds', 'ensem_room207.world')

    # The world loads a GUI plugin (hides the green light markers); make
    # sure gzclient finds it even if the shell was sourced before it existed.
    plugin_dir = os.path.join(get_package_prefix('ensem_room207'), 'lib')
    plugin_path = SetEnvironmentVariable(
        'GAZEBO_PLUGIN_PATH',
        os.pathsep.join(
            [plugin_dir, os.environ.get('GAZEBO_PLUGIN_PATH', '')]
        ),
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_pkg, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_file}.items(),
    )

    return LaunchDescription([plugin_path, gazebo])
