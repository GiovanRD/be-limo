from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Create the launch description for the recording node."""

    topics = DeclareLaunchArgument(
        'topics',
        default_value="['/odom']",
        description='List of ROS 2 topics to record.'
    )

    output_dir = DeclareLaunchArgument(
        'output_dir',
        default_value='',
        description='Directory where rosbag files are stored.'
    )

    recorder = Node(
        package='recording',
        executable='recorder',
        name='recorder',
        output='screen',
        parameters=[
            {
                'topics': LaunchConfiguration('topics'),
                'output_dir': LaunchConfiguration('output_dir'),
            }
        ],
    )

    return LaunchDescription([
        topics,
        output_dir,
        recorder,
    ])