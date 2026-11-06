# Room 207 — LIMO

This repository contains the LIMO demonstration of ENSEM Room 207. It is the reference repository that students fork for the **BE Véhicule Autonome**.

It is provided in two versions:

| Version | Folder | Content |
|---|---|---|
| ROS 2 Foxy (Ubuntu 20.04) | [`ros2_foxy/room207_limo/`](ros2_foxy/room207_limo/) | ROS 2 package `room207_limo`: Gazebo simulation launch (Room 207 world by default), RViz configuration, control window (camera, teleoperation, recording, live trajectory) and odometry plotting tool |
| MATLAB/Simulink (R2026b) | [`matlab/`](matlab/) | Simulink versions of the demonstration, standalone or connected to the Gazebo simulation through ROS 2 |

Both rely on the shared `robotic_lab` repositories: `limo` (robot software and MATLAB model), `environment` (worlds and track map), `teleoperation` and `recording`.

Sections 1 and 2 describe the ROS 2 version and section 3 the MATLAB/Simulink version.

## Quick start

Once the workspace is installed (section 1), the shortcuts of [`ros2_foxy/room207.bash`](ros2_foxy/room207.bash) start each mode with one command. Load them in every terminal by adding this line to `~/.bashrc` (on the workstation, or in the Docker container on Windows):

```bash
source ~/ros2_ws/src/room207-limo/ros2_foxy/room207.bash
```

`r207_help` lists the commands. `r207_build` builds `room207_limo` and the packages it needs.

**Simulation in Gazebo**

| Command | Starts |
|---|---|
| `r207_sim` | Gazebo (Room 207) and RViz |
| `r207_demo` | The complete demonstration: simulation, recorder and control window |

**Real robot.** Run each command in its own terminal, after selecting the robot:

| Command | Runs on | Starts |
|---|---|---|
| `r207_robot 12` | workstation | Selects LIMO number 12 (`100.75.155.12`), for this terminal |
| `r207_robot_start` | robot, over SSH | LIMO driver |
| `r207_robot_server` | robot, over SSH | TCP server of `robot_communication` (odometry on `/odom`) |
| `r207_robot_tunnel` | workstation | SSH tunnel to the TCP server |
| `r207_robot_teleop` | workstation | Teleoperation window |

`r207_robot_ssh` opens a shell on the robot. Set `R207_ROBOT` in `~/.bashrc` to keep the same robot in all terminals.

**MATLAB/Simulink** (Windows or Linux, no ROS 2 needed for the standalone model):

```matlab
cd room207-limo/matlab
room207_setup
open room207_demo.slx      % Simulink only
open room207_gazebo.slx    % connected to Gazebo: start it with r207_matlab
```

See section 3 for the MATLAB details.

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

`colcon` finds the package in `room207-limo/ros2_foxy/room207_limo/`; the `matlab/` folder is ignored by the ROS 2 build.

> **Updating from an older version:** the package used to be at the root of the repository. After pulling, delete `build/room207_limo` and `install/room207_limo` from the workspace before rebuilding, so that colcon does not keep the old copy.

### 1.2. Required packages

Before building `room207_limo`, install and configure the following components:

| Component | Purpose | Installation instructions |
|---|---|---|
| LIMO | ROS 2 driver, robot description, and Gazebo model | [LIMO README](https://gitlab.univ-lorraine.fr/coll/l-inp/ensem/robotic_lab/limo/-/blob/44db027fc72b962120c4b1db06d0b43faaa263aa/README.md) |
| Environment | Room 207 world (`ensem_room207`) and QCar large-track world and map | [Environment README](https://gitlab.univ-lorraine.fr/coll/l-inp/ensem/robotic_lab/environment/-/blob/7a8e9488a37bd217728c5c93ace80d1eaea8be77/README.md) |
| Teleoperation | Keyboard control widget used by the control window | [Teleoperation README](https://gitlab.univ-lorraine.fr/coll/l-inp/ensem/robotic_lab/teleoperation/-/blob/e17321d3ae30c894ec82d1604d34931bbe40d025/README.md) |
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
sudo apt install python3-pil.imagetk
colcon build --symlink-install
source install/setup.bash
```

`python3-pil.imagetk` (camera image in the control window) has no rosdep key, so it is installed with `apt`.

## 2. Usage

In each new terminal, source the ROS 2 environment and workspace:

```bash
source /opt/ros/foxy/setup.bash
source ~/ros2_ws/install/setup.bash
```

### 2.1. Launch the simulation

Start Gazebo with the Room 207 world, the LIMO model, and RViz:

```bash
ros2 launch room207_limo room207_limo.launch.py
```

The LIMO is spawned on the track, which lies on a 20 cm platform in Room 207. Launch arguments:

| Argument | Default | Description |
|---|---|---|
| `world` | `ensem_room207/worlds/ensem_room207.world` | Gazebo world file |
| `spawn_z` | `0.35` | Spawn height of the LIMO (m) |

To use the bare QCar track world (track on the ground):

```bash
ros2 launch room207_limo room207_limo.launch.py \
  world:=$(ros2 pkg prefix qcar_large_track)/share/qcar_large_track/worlds/qcar_large_track.world \
  spawn_z:=0.15
```

### 2.2. Launch the complete demonstration

Start the simulation, the recording backend (`recorder` node of the `recording` package), and the control window:

```bash
ros2 launch room207_limo demo_simu.launch.py
```

The **ROOM 207 | LIMO CONTROL** window shows:

- the LIVE CAMERA image of the LIMO (`/rgb/image_raw`);
- the keyboard controls;
- the RECORDING button and status;
- the live trajectory from `/odom`.

Keyboard controls (AZERTY), with the window focused:

| Key | Action |
|---|---|
| **Z** / **S** | Forward / backward |
| **Q** / **D** | Steer left / right |
| **Space** | Stop the robot (the recording continues) |
| **R** | Start / stop recording |
| **Esc** | Close the window |

### 2.3. Record a demonstration

While the complete demonstration is running, press **R** or click the RECORDING button to start recording, and again to stop it.

The control window calls the `/start_recording` and `/stop_recording` services of the `recording` package. The recorded topics are `/odom`, `/scan`, and `/imu_plugin/out` (set in `demo_simu.launch.py`); their availability depends on the running simulation and sensor configuration.

For details on recording options and output locations, refer to the `recording` package README.

### 2.4. Plot odometry

When a recording is stopped, the control window automatically runs the odometry plotting tool on the new rosbag.

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

## 3. MATLAB/Simulink demonstrations

The `matlab/` folder contains two Simulink versions of the demonstration. Both use the `LIMO (Gazebo)` block of the LIMO repository (`limo/matlab/limo_lib.slx`), which reproduces the dynamics of the Gazebo LIMO. Both start the robot at the Room 207 spawn pose and draw it live on the QCar track map.

| Model | ROS 2 needed | Content |
|---|---|---|
| `room207_demo.slx` | No | Simulink LIMO driven from the keyboard or a scenario |
| `room207_gazebo.slx` | Yes (ROS Toolbox) | Drives the Gazebo LIMO of the demonstration and runs the Simulink LIMO alongside as a twin, both drawn on the map |

### 3.1. Setup

The MATLAB code expects the `limo` and `environment` repositories next to `room207-limo`, as in `robotic_lab`. In MATLAB (R2026b, Simulink):

```matlab
cd room207-limo/matlab
room207_setup          % adds this folder and limo/matlab to the path
```

Set the environment variable `LIMO_MATLAB_PATH` if `limo/matlab` is elsewhere.

### 3.2. Simulink-only demonstration

Open `room207_demo.slx` and press **Run**. The simulation runs in real time and opens the **ROOM 207 | LIMO** view. Click the view, then drive with the keyboard (AZERTY, as in the ROS 2 control window):

| Key | Action |
|---|---|
| **Z** / **S** | Forward / backward at 0.5 m/s (held) |
| **Q** / **D** | Steer left / right at 30° (held) |
| **Space** | Stop the robot |
| **Esc** | Stop the simulation |

Double-click the **Keyboard or scenario** switch in the `Command source` block to replay the `Scenario` block instead. The default scenario leaves the parking bay, turns into the car park and reverses back into the bay. The command is `u = [v (m/s); γ (rad)]`, where γ is the steering angle, positive to the left.

The `odom` and `state` outputs of the LIMO are saved in the simulation output `out`.

### 3.3. Connection to the Gazebo demonstration

`room207_gazebo.slx` replaces the control window of `demo_simu.launch.py`. Start the Gazebo simulation only:

```bash
ros2 launch room207_limo room207_limo.launch.py
```

Then open `room207_gazebo.slx` in MATLAB and press **Run**. The model:

- publishes the keyboard or scenario command on `/cmd_vel_gazebo`, as the control window does (γ sign-flipped in reverse for the Gazebo plugin);
- reads the Gazebo pose on `/odom`;
- simulates the Simulink LIMO with the same command, from the Room 207 spawn pose.

The view shows both robots, so the Simulink model can be compared with Gazebo live. The `gazebo_pose` and `odom` signals are saved in `out`.

Requirements and limitations:

- MATLAB and the Gazebo machine must be on the same ROS 2 network, with the same `ROS_DOMAIN_ID` and no firewall blocking DDS traffic.
- Restart Gazebo before each run. The twin always starts at the spawn pose and does not know where the Gazebo LIMO was left.
- Simulink runs in real time, while Gazebo may run slower (real-time factor below 1 in Gazebo). The two robots then drift apart even with identical dynamics.
- Do not run the ROS 2 control window at the same time: both would publish on `/cmd_vel_gazebo`.

### 3.4. Rebuilding and testing

`build_room207_models.m` regenerates both models. Edit it rather than the `.slx` files. The tests check the scenario, the keyboard mapping and that the Gazebo model compiles:

```matlab
results = runtests('tests/test_room207.m')
```

## 4. Repository structure

```text
room207-limo/
├── ros2_foxy/
│   ├── room207.bash                  shortcuts (r207_*)
│   └── room207_limo/                 ROS 2 package
│       ├── launch/
│       │   ├── demo_simu.launch.py
│       │   └── room207_limo.launch.py
│       ├── room207_limo/
│       │   ├── __init__.py
│       │   └── room207_simu_gui.py
│       ├── rviz/
│       │   └── room207_limo.rviz
│       ├── scripts/
│       │   └── plot_odometry.py
│       ├── CMakeLists.txt
│       └── package.xml
├── matlab/                           MATLAB/Simulink version
│   ├── +room207/                     view, keyboard and parameters
│   ├── tests/
│   ├── build_room207_models.m
│   ├── room207_demo.slx
│   ├── room207_gazebo.slx
│   └── room207_setup.m
└── README.md
```

The layout follows the `robotic_lab` convention: one folder per version (`ros2_<distribution>/`, `matlab/`). A future ROS 2 Humble version would go in `ros2_humble/`.

## 5. Package responsibilities

This package orchestrates the Room 207 demonstration. The underlying functionalities are maintained separately:

- `limo_ros2`: LIMO driver and robot description;
- `environment`: Gazebo worlds (Room 207 in `ensem_room207`, QCar track);
- `teleoperation`: keyboard control widget;
- `recording`: ROS 2 bag recording.

Keeping these components separate avoids duplication and allows them to be reused in other teaching sessions.
