from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Launch the complete Room 206 demonstration."""

    # ============================================================
    # Room 206 simulation
    # ============================================================

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                FindPackageShare('room207_limo'),
                '/launch/room207_limo.launch.py'
            ]
        )
    )

    # ============================================================
    # Teleoperation
    # ============================================================

    teleoperation = Node(
        package='teleoperation',
        executable='teleop_limo',
        name='teleop_limo',
        remappings=[
            ('/cmd_vel', '/cmd_vel_gazebo'),
        ],
        output='screen',
    )

    # ============================================================
    # Recording
    # ============================================================

    recording = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                FindPackageShare('recording'),
                '/launch/recording.launch.py'
            ]
        ),
        launch_arguments={
            'topics': '[/odom, /scan, /imu_plugin/out]',
        }.items()
    )

    # ============================================================
    # Recording management
    # ============================================================

    recording_simulation = Node(
        package='room207_limo',
        executable='recording_simulation',
        name='recording_simulation',
        output='screen',
    )

    return LaunchDescription([
        simulation,
        teleoperation,
        recording,
        recording_simulation,
    ])