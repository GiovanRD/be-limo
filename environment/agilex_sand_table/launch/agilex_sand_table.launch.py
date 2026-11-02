import os # Python standard library for operating for dependency management (os.path.join instead of /)

from ament_index_python.packages import get_package_share_directory # Get the package share directory for the agilex_sand_table and gazebo_ros packages


# Import the necessary modules from the launch package to create a launch description and include other launch files.
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


# Classical function to generate the launch description for the agilex_sand_table environment in Gazebo (called when "ros2 launch agilex_sand_table agilex_sand_table.launch.py").
def generate_launch_description():

    # Get the package share directories for the agilex_sand_table and gazebo_ros packages.
    agilex_sand_table_pkg = get_package_share_directory('agilex_sand_table')
    gazebo_ros_pkg = get_package_share_directory('gazebo_ros')


    # Define the path to the world file for the agilex_sand_table environment.
    world_file = os.path.join(
        agilex_sand_table_pkg,
        'worlds',
        'agilex_sand_table.world'
    )

    # Include the Gazebo launch file from the gazebo_ros package, passing the world file as an argument to load the agilex_sand_table environment in Gazebo.
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                gazebo_ros_pkg,
                'launch',
                'gazebo.launch.py'
            )
        ),
        launch_arguments={
            'world': world_file
        }.items()
    )

    return LaunchDescription([
        gazebo
    ])
