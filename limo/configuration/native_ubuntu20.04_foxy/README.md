# LIMO — Native Ubuntu 20.04 and ROS 2 Foxy

This guide describes how to configure the LIMO ROS 2 packages and Gazebo simulation on Ubuntu 20.04 (Focal), either natively or through WSL 2, without Docker.

## 1. Requirements

- Ubuntu 20.04 (64-bit)
- ROS 2 Foxy Desktop
- Git and internet access
- For WSL 2: WSLg for graphical applications such as Gazebo and RViz2

**Note:** ROS 2 Foxy reached end of life in June 2023. It is retained for compatibility with the Ubuntu 20.04 environment used by the physical LIMO robots.

## 2. Install ROS 2 Foxy

Follow the [official ROS 2 Foxy installation instructions](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html).

Install the Desktop variant. Verify the installation:

```bash
source /opt/ros/foxy/setup.bash
ros2 --help
```

## 3. Clone the repository

Clone the LIMO repository into `~/robotic_lab`:

```bash
mkdir -p ~/robotic_lab
cd ~/robotic_lab

git clone https://gitlab.univ-lorraine.fr/coll/l-inp/ensem/robotic_lab/limo.git

 git sparse-checkout set \
> configuration/native_ubuntu20.04_foxy/ \
> ros2_foxy/limo_ros2/ \
> ros2_foxy/ros2_astra_camera/ \
> ros2_foxy/ydlidar_ros2_driver/ \
> ros2_foxy/YDLidar-SDK/
```

The configuration and source packages are located in:

```text
limo/
├── configuration/native_ubuntu20.04_foxy/
└── ros2_foxy/
    ├── limo_ros2/
    ├── ros2_astra_camera/
    ├── YDLidar-SDK/
    └── ydlidar_ros2_driver/
```

## 4. Prepare and build the workspace

Run the setup script from the repository:

```bash
cd ~/robotic_lab/limo

bash configuration/native_ubuntu20.04_foxy/setup.sh
```

The script installs the development tools and required ROS dependencies, creates the workspace source links, builds the YDLidar SDK, and builds the selected ROS 2 packages.

The default workspace is `~/ros2_ws`. Set `LIMO_WS` to use another location.

To prepare the workspace and install dependencies without compiling the packages, run:

```bash
bash configuration/native_ubuntu20.04_foxy/setup.sh --no-build
```

## 5. Source the workspace

After a successful build, source ROS 2 Foxy and the workspace:

```bash
source /opt/ros/foxy/setup.bash
source ~/ros2_ws/install/setup.bash
```

Run these commands in each new terminal before using the LIMO packages.

To source them automatically in new Bash terminals, add the following lines to `~/.bashrc`:

```bash
source /opt/ros/foxy/setup.bash
source ~/ros2_ws/install/setup.bash
```

## 6. Launch the simulation

In a terminal where the workspace has been sourced, launch the LIMO simulation:

```bash
ros2 launch limo_car ackermann_gazebo.launch.py
```

For keyboard teleoperation, open a second terminal and source the environment:

```bash
source /opt/ros/foxy/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

The teleoperation node publishes velocity commands on `/cmd_vel`. The simulation must be running and configured to receive those commands.

For WSL 2, graphical applications require a working WSLg setup. Verify Gazebo and RViz2 rendering before using this configuration for teaching.

## 7. Verify the installation

List the installed LIMO packages:

```bash
ros2 pkg list | grep '^limo'
```

Check the executables provided by `limo_base`:

```bash
ros2 pkg executables limo_base
```

## 8. Scope

This configuration targets the Ubuntu 20.04 / ROS 2 Foxy simulation environment. It does not configure the physical robot's hardware, network connection, or device permissions.

The physical LIMO setup is documented separately under:

```text
configuration/limo_ubuntu20.04_foxy/
```

The Docker-based WSL 2 configuration is documented separately under:

```text
configuration/wsl2_ubuntu20.04_foxy/
```
