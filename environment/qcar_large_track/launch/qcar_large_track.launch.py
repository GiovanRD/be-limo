import os # Python standard library for operating for dependency management (os.path.join instead of /)

from ament_index_python.packages import get_package_share_directory # Get the package share directory for the qcar_large_track and gazebo_ros packages


# Import the necessary modules from the launch package to create a launch description and include other launch files.
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


# Classical function to generate the launch description for the qcar_large_track environment in Gazebo (called when "ros2 launch qcar_large_track qcar_large_track.launch.py").
def generate_launch_description():

    # Get the package share directories for the qcar_large_track and gazebo_ros packages using the get_package_share_directory function.
    qcar_large_track_pkg = get_package_share_directory('qcar_large_track')
    gazebo_ros_pkg = get_package_share_directory('gazebo_ros')


    # Define the path to the world file for the qcar_large_track environment by joining the package share directory with the relative path to the world file.
    world_file = os.path.join(
        qcar_large_track_pkg,
        'worlds',
        'qcar_large_track.world'
    )

    # Include the Gazebo launch file from the gazebo_ros package, passing the world file as an argument to load the qcar_large_track environment in Gazebo.
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