# Room 207 — LIMO

This repository contains the ROS 2 package `room207_limo`, which orchestrates the LIMO demonstration in ENSEM Room 207.

It provides the Gazebo simulation launch, RViz configuration, recording manager, and odometry plotting tool. It relies on the shared `robotic_lab` packages for the LIMO, environment, teleoperation, and recording functionalities.

## 1. Installation

### 1.1. Workspace location

Clone this repository into the `src` directory of the ROS 2 workspace:

```text
~/ros2_ws/
├── src/
│   ├── room207-limo/
│   ├── environment/
│   ├── teleoperation/
│   ├── recording/
│   └── limo_ros2/
├── build/
├── install/
└── log/
```

The directory names above illustrate the expected workspace organization. Follow the README of each repository for its installation method; some repositories may be maintained elsewhere in `robotic_lab` and linked into the workspace.

```bash
cd ~/ros2_ws/src
git clone <room207-limo-repository-url> room207-limo
```

### 1.2. Required packages

Before building `room207_limo`, install and configure the following components:

| Component | Purpose | Installation instructions |
|---|---|---|
| LIMO | ROS 2 driver, robot description, and Gazebo model | [LIMO README](https://gitlab.univ-lorraine.fr/coll/l-inp/ensem/robotic_lab/limo/-/blob/44db027fc72b962120c4b1db06d0b43faaa263aa/README.md) |
| Environment | QCar large-track world and map | [Environment README](https://gitlab.univ-lorraine.fr/coll/l-inp/ensem/robotic_lab/environment/-/blob/7a8e9488a37bd217728c5c93ace80d1eaea8be77/README.md) |
| Teleoperation | Keyboard control | [Teleoperation README](https://gitlab.univ-lorraine.fr/coll/l-inp/ensem/robotic_lab/teleoperation/-/blob/e17321d3ae30c894ec82d1604d34931bbe40d025/README.md) |
| Recording | ROS 2 bag recording and start/stop services | [Recording README](https://gitlab.univ-lorraine.fr/coll/l-inp/ensem/robotic_lab/recording/-/blob/ad42d6aacc407b2482d03756d04260d42d58a02d/README.md) |
| Room 207 LIMO | Simulation launch and demonstration tools | This README |

Follow the README for the relevant Ubuntu and ROS 2 configuration. Do not duplicate those repositories' installation instructions here.

### 1.3. Build

Once all required packages are installed, build the workspace using the instructions appropriate for your ROS 2 distribution.

For Ubuntu 20.04 / ROS 2 Foxy:

```bash
source /opt/ros/foxy/setup.bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## 2. Usage

In each new terminal, source the ROS 2 environment and workspace:

```bash
source /opt/ros/foxy/setup.bash
source ~/ros2_ws/install/setup.bash
```

### 2.1. Launch the simulation

Start Gazebo, the LIMO model, and RViz:

```bash
ros2 launch room207_limo room207_limo.launch.py
```

### 2.2. Launch the complete demonstration

Start the simulation, keyboard teleoperation, recording services, and recording manager:

```bash
ros2 launch room207_limo demo.launch.py
```

Use the keyboard controls documented in the `teleoperation` package README to drive the LIMO.

### 2.3. Record a demonstration

While the complete demonstration is running:

- Press **R** to start recording.
- Press **R** again to stop recording.

The recording manager uses the services provided by the `recording` package. The configured topics include `/odom`, `/scan`, and `/imu_plugin/out`; their availability depends on the running simulation and sensor configuration.

Recordings are stored in the workspace data directory according to the `recording` package configuration.

For details on recording options and output locations, refer to the `recording` package README.

### 2.4. Plot odometry

The demonstration manager can automatically launch the odometry plotting tool after a recording is stopped.

To plot an existing rosbag manually:

```bash
ros2 run room207_limo plot_odometry ~/ros2_ws/data/<rosbag-directory>
```

Replace `<rosbag-directory>` with the actual rosbag directory name.

The tool generates PDF plots of:

- the estimated LIMO trajectory over the QCar track map;
- the linear and angular velocities;
- the yaw angle.

The generated plots are saved in the workspace `data/` directory.

## 3. Repository structure

```text
room207-limo/
├── launch/
│   ├── demo.launch.py
│   └── room207_limo.launch.py
├── room207_limo/
│   ├── __init__.py
│   └── recording_simulation.py
├── rviz/
│   └── room207_limo.rviz
├── scripts/
│   └── plot_odometry.py
├── CMakeLists.txt
├── package.xml
└── README.md
```

## 4. Package responsibilities

This package orchestrates the Room 207 demonstration. The underlying functionalities are maintained separately:

- `limo_ros2`: LIMO driver and robot description;
- `environment`: Gazebo world and QCar track;
- `teleoperation`: keyboard control;
- `recording`: ROS 2 bag recording.

Keeping these components separate avoids duplication and allows them to be reused in other teaching sessions.
