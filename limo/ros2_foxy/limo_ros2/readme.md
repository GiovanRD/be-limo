# LIMO ROS 2 Foxy

## 1. Packages

This directory contains the ROS 2 packages for the AgileX LIMO robot.

```text
limo_ros2/
├── limo_base/
├── limo_bringup/
├── limo_car/
├── limo_description/
└── limo_msgs/
```

- **`limo_base`**: Robot hardware driver.
- **`limo_bringup`**: Launch files for starting the robot nodes.
- **`limo_car`**: Gazebo simulation package.
- **`limo_description`**: Robot description, including its URDF/Xacro model.
- **`limo_msgs`**: Custom ROS 2 message definitions.

## 2. Environment

The development environment uses:

- Ubuntu 20.04
- ROS 2 Foxy

## 3. Installation

### Dependencies

Install the required ROS 2 tools:

```bash
sudo apt update
sudo apt install \
    ros-foxy-joint-state-publisher-gui \
    ros-foxy-rqt-robot-steering \
    ros-foxy-teleop-twist-keyboard \
    python3-colcon-common-extensions \
    python3-rosdep
```

Initialize `rosdep` if it has not already been initialized:

```bash
sudo rosdep init
rosdep update --rosdistro foxy
```

### Build

Create a ROS 2 workspace and add the LIMO packages:

```bash
mkdir -p ~/ros2_ws/src
ln -s /path/to/my_limo/ros2_foxy/limo_ros2 ~/ros2_ws/src/limo_ros2
```

Replace `/path/to/my_limo` with the absolute path to your repository.

Install the package dependencies and build:

```bash
source /opt/ros/foxy/setup.bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y --rosdistro foxy
colcon build
```

Source the workspace:

```bash
source ~/ros2_ws/install/setup.bash
```

To source it automatically in new terminals, add the following line to `~/.bashrc`:

```bash
source ~/ros2_ws/install/setup.bash
```

## 4. Usage

### Start the LIMO base node

```bash
ros2 launch limo_base limo_base.launch.py
```

### Start keyboard teleoperation

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
    --ros-args -r cmd_vel:=/cmd_vel_teleop
```

The teleoperation command publishes velocity commands to `/cmd_vel_teleop`. Ensure that the base driver or controller is configured to subscribe to this topic.

## 5. Acknowledgements

This repository is based on the original LIMO ROS 2 packages developed and published by AgileX Robotics:

- [AgileX Robotics — limo_ros2](https://github.com/agilexrobotics/limo_ros2)

The Gazebo simulation package `limo_car` was developed jointly with the Institute for Intermodal Transport and Logistics Systems at TU Braunschweig, Germany. We acknowledge their contributions.