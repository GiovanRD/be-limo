# LIMO — Ubuntu 22.04 and ROS 2 Humble

This guide describes how to set up the LIMO ROS 2 packages on a native Ubuntu 22.04 (Jammy) installation with ROS 2 Humble.

## 1. Requirements

- Ubuntu 22.04 (64-bit)
- ROS 2 Humble Desktop

Install ROS 2 Humble by following the [official installation instructions](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html).

## 2. Install development tools

Install the tools required to manage dependencies and build ROS 2 packages:

```bash
sudo apt update
sudo apt install -y \
  build-essential \
  git \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool \
  ros-humble-joint-state-publisher-gui \
  ros-humble-teleop-twist-keyboard
```

Initialize `rosdep` if it has not already been initialized:

```bash
sudo rosdep init
```

If the command reports that `rosdep` is already initialized, continue with:

```bash
rosdep update
```

## 3. Clone the repository

Clone the LIMO repository into the `robotic_lab` directory:

```bash
mkdir -p ~/robotic_lab
cd ~/robotic_lab

git clone https://gitlab.univ-lorraine.fr/coll/l-inp/ensem/robotic_lab/limo.git
```

The ROS 2 Humble packages are located in:

```text
limo/ros2_humble/limo_ros2/
```

This directory contains the following packages:

- `limo_base`
- `limo_car`
- `limo_description`
- `limo_msgs`

## 4. Install package dependencies

Move to the repository directory and install the dependencies declared in the Humble packages:

```bash
cd ~/robotic_lab/limo

source /opt/ros/humble/setup.bash

rosdep install \
  --from-paths ros2_humble/limo_ros2 \
  --ignore-src \
  --rosdistro humble \
  -r -y
```

## 5. Build the workspace

Create a separate ROS 2 workspace and build the Humble packages:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

source /opt/ros/humble/setup.bash

colcon build \
  --base-paths ~/robotic_lab/limo/ros2_humble/limo_ros2 \
  --symlink-install \
  --parallel-workers 2
```

The `--base-paths` option tells `colcon` where to find the source packages. The build and installation files remain in `~/ros2_ws`.

## 6. Source the workspace

After a successful build, source the ROS 2 installation and the workspace:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

Run these commands in each new terminal before using the LIMO packages.

To source them automatically in new Bash terminals, add the following lines to `~/.bashrc`:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

## 7. Verify the installation

List the installed LIMO packages:

```bash
ros2 pkg list | grep '^limo'
```

Check the executables provided by `limo_base`:

```bash
ros2 pkg executables limo_base
```

## 8. Useful tools

### Robot model visualization

The joint state publisher GUI allows you to manipulate the joints of a robot model when used with the appropriate URDF and `robot_state_publisher` configuration.

```bash
ros2 run joint_state_publisher_gui joint_state_publisher_gui
```

This tool is useful for inspecting a robot model. It does not command the physical LIMO's wheels.

### Keyboard teleoperation

Run the keyboard teleoperation node:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

By default, the node publishes velocity commands on `/cmd_vel`.

**Important:** this command only publishes velocity commands. To move a physical LIMO, its driver must also be running and subscribed to the same command topic. The robot's configuration and safety conditions must be checked before sending motion commands.

## 9. Scope

This guide covers the ROS 2 Humble software environment and the LIMO packages included in `ros2_humble/limo_ros2`.

It does not configure the physical robot's hardware, sensors, network connection, or device permissions. Camera and LiDAR drivers are not included in this Humble source directory and must be configured separately if required.
