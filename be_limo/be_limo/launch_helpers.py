"""Morceaux de lancement communs au robot réel (real_l2, teleop, mapping).

* driver_actions : driver du labo (limo_start : châssis, lidar, caméra) et
  odométrie corrigée par le gyroscope (odom_imu, repère "odom_imu").
* localization_actions : position du robot sur la carte de la piste
  (/robot_pose, repère "map"), par l'odométrie seule (départ sur un noeud) ou
  par le lidar (AMCL sur la carte de la salle, faite avec mapping.launch.py).
"""

import os


def driver_actions(driver, use_camera, odom_imu='auto'):
    from ament_index_python.packages import get_package_share_directory
    from launch.actions import IncludeLaunchDescription
    from launch.launch_description_sources import PythonLaunchDescriptionSource
    from launch_ros.actions import Node

    actions = []
    if driver == 'true':
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('limo_start'), 'launch', 'limo_real.launch.py')),
            launch_arguments={'use_camera': use_camera}.items()))
    # L'odométrie corrigée va avec le driver ; odom_imu:=true la force (driver lancé à part).
    if odom_imu == 'true' or (odom_imu == 'auto' and driver == 'true'):
        actions.append(Node(package='be_limo', executable='odom_imu', output='screen'))
    return actions


def localization_actions(mode, initial_node, map_yaml):
    """mode : 'odom', 'lidar' ou 'none'."""
    from ament_index_python.packages import get_package_share_directory
    from launch_ros.actions import Node

    share = get_package_share_directory('be_limo')
    if mode == 'none':
        return []
    if mode == 'lidar':
        map_yaml = os.path.expanduser(map_yaml)
        amcl = os.path.join(share, 'config', 'amcl.yaml')
        return [
            Node(package='nav2_map_server', executable='map_server', name='map_server', output='screen',
                 parameters=[amcl, {'yaml_filename': map_yaml}]),
            Node(package='nav2_amcl', executable='amcl', name='amcl', output='screen', parameters=[amcl]),
            Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                 name='lifecycle_manager_localization', output='screen',
                 parameters=[{'autostart': True, 'node_names': ['map_server', 'amcl']}]),
            Node(package='be_limo', executable='localization_node', output='screen',
                 parameters=[{'mode': 'lidar', 'map_yaml': map_yaml, 'initial_node': int(initial_node)}]),
        ]
    return [Node(package='be_limo', executable='localization_node', output='screen',
                 parameters=[{'odom_is_world': False, 'odom_topic': '/odom_imu',
                              'initial_node': int(initial_node)}])]
