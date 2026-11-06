
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Launch the complete Room 207 demonstration."""

    # Simulation: Gazebo, LIMO and RViz
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('room207_limo'),
            '/launch/room207_limo.launch.py',
        ])
    )

    # Recording backend
    recording = Node(
        package='recording',
        executable='recorder',
        name='recorder',
        output='screen',
        parameters=[{
            'topics': [
                '/odom',
                '/scan',
                '/imu_plugin/out',
            ],
        }],
    )

    # Integrated teleoperation and recording GUI
    gui = Node(
        package='room207_limo',
        executable='room207_simu_gui',
        name='room207_simu_gui',
        remappings=[
            ('/cmd_vel', '/cmd_vel_gazebo'),
        ],
        output='screen',
    )

    return LaunchDescription([
        simulation,
        recording,
        gui,
    ])
