from glob import glob

from setuptools import setup

package_name = 'be_limo'

setup(
    name=package_name,
    version='0.2.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/worlds', glob('worlds/*.world')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='BE LIMO groupe 1',
    maintainer_email='giovanrocca.duringer@gmail.com',
    description='Planification, contrôle, détection et supervision de la LIMO',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'localization_node = be_limo.localization_node:main',
            'global_planner = be_limo.global_planner_node:main',
            'local_controller = be_limo.local_controller_node:main',
            'obstacle_detector = be_limo.obstacle_detector_node:main',
            'vision_node = be_limo.vision_node:main',
            'traffic_light_manager = be_limo.traffic_light_manager:main',
            'supervisor = be_limo.supervisor_gui:main',
        ],
    },
)
