from setuptools import find_packages, setup


package_name = 'recording'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
        (
            'share/' + package_name + '/launch',
            ['launch/recording.launch.py']
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jérémie Kreiss',
    maintainer_email='jeremie.kreiss@univ-lorraine.fr',
    description='Tools for recording and processing robotics data.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'recorder = recording.recorder:main',
            'start_recording = recording.start_recording:main',
            'stop_recording = recording.stop_recording:main',
            'recording_widget = recording.recording_widget:main',
        ],
    },
)