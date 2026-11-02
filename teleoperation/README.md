# Teleoperation

ROS 2 package providing keyboard teleoperation interfaces for the AgileX LIMO, using a Tkinter widget for real and simulated robots.

This repository is part of the `robotic_lab` platform developed for teaching and student projects at ENSEM.

## Features

- **Shared Tkinter interface:** common ZQSD keyboard controls for real and simulated robots.
- **LIMO simulation:** publishes velocity commands directly to ROS 2.
- **LIMO real robot:** sends velocity commands to the robot through a TCP client and an SSH tunnel.
- **Reusable component:** `teleop_widget.py` can be integrated into other graphical applications.

## Supported robots

| Robot | Executable | Communication | Status |
|---|---|---|---|
| AgileX LIMO (simulation) | `teleop_limo_simu` | ROS 2 topic `/cmd_vel` | Available |
| AgileX LIMO (real robot) | `teleop_limo_real` | TCP through SSH tunnel | Available |

## Requirements

### LIMO simulation

- Ubuntu 20.04 with ROS 2 Foxy.
- A built ROS 2 workspace containing the `teleoperation` package.
- A running LIMO simulation whose command interface subscribes to `/cmd_vel`.
- Tkinter installed on the workstation.

### LIMO real robot

- **Workstation:** Ubuntu 20.04 with ROS 2 Foxy, the `teleoperation` package built, and Tkinter installed.
- **Robot:** AgileX LIMO running Ubuntu 20.04 with ROS 2 Foxy.
- The LIMO base driver and the TCP server provided by the `robot_communication` package must be running on the robot.
- An SSH connection to the robot and an SSH tunnel forwarding TCP port `8765`.

## Installation

From the ROS 2 workspace root:

```bash
cd ~/ros2_ws
colcon build --packages-select teleoperation --symlink-install
source /opt/ros/foxy/setup.bash
source install/setup.bash
```

On a new terminal, source the ROS 2 environment and workspace again before running an executable.

## LIMO simulation

The simulated interface uses the shared Tkinter widget and publishes velocity commands directly to `/cmd_vel` using `geometry_msgs/msg/Twist`.

### Start the interface

Start the LIMO simulation first. Then, in a terminal using the same ROS 2 environment:

```bash
ros2 run teleoperation teleop_limo_simu
```

The simulation and the teleoperation interface must belong to the same ROS 2 graph. The simulator must subscribe to `/cmd_vel`.

### Keyboard controls

| Key | Action |
|---|---|
| `Z` | Forward |
| `S` | Backward |
| `Q` | Steer left |
| `D` | Steer right |
| `SPACEBAR` | Stop |
| `ESC` | Close the interface |

The interface publishes commands periodically while running. Releasing a movement key resets the corresponding command.

## LIMO real robot

The real-robot interface is a Tkinter graphical application. It sends speed and steering commands to a TCP server running on the robot. The server translates the received commands into ROS 2 velocity commands published on `/cmd_vel`.

The workstation and the robot communicate through an SSH tunnel. The graphical client does not publish directly to ROS 2 topics.

### Architecture

```text
Workstation
┌──────────────────────────┐
│ teleop_limo_real         │
│ Tkinter interface        │
│ Shared TeleopWidget      │
└────────────┬─────────────┘
             │ TCP localhost:8765
             │ SSH tunnel
             ▼
LIMO robot (Jetson)
┌──────────────────────────┐
│ robot_communication      │
│ TCP server               │
└────────────┬─────────────┘
             │ /cmd_vel
             ▼
┌──────────────────────────┐
│ LIMO base driver         │
└──────────────────────────┘
```

### Start the real-robot interface

The following components must be running:

1. The LIMO base driver on the robot.
2. The TCP server from `robot_communication` on the robot.
3. An SSH tunnel from the workstation to the robot.
4. The graphical interface on the workstation.

#### 1. Start the LIMO base driver

On the robot, launch the real-robot driver using the appropriate launch file. For the current setup:

```bash
ros2 launch limo_start limo_real.launch.py
```

#### 2. Start the TCP server

On the robot, launch the TCP server using the executable provided by `robot_communication`. Use the command: 

```bash
ros2 run robot_communication tcp_server
```

The server listens on port `8765` and publishes velocity commands on `/cmd_vel`.

#### 3. Create the SSH tunnel

On the workstation:

```bash
ssh -N -L 8765:127.0.0.1:8765 jetson@<jetson_ip>
```

Replace `<jetson_ip>` with the robot's reachable IP address. Keep the tunnel running during teleoperation.

#### 4. Start the graphical interface

On the workstation:

```bash
ros2 run teleoperation teleop_limo_real
```

The graphical client connects to `127.0.0.1:8765` through the SSH tunnel.

### Graphical interface controls

| Control | Action |
|---|---|
| `Z` | Forward |
| `S` | Backward |
| `Q` | Steer left |
| `D` | Steer right |
| `SPACEBAR` | Stop the motion command |
| `ESC` | Close the interface |

The speed and steering limits are configured separately for the real and simulated interfaces. Releasing a movement key resets the corresponding command. Losing window focus also resets the commands.

**Safety:** the software stop is not a hardware emergency stop. Keep the robot's wheels clear during initial testing and ensure that a physical emergency-stop procedure is available.

## Shared Tkinter widget

The `teleop_widget.py` module implements the shared graphical component:

- Keyboard event handling for ZQSD.
- Visual feedback for pressed keys.
- Spacebar stop command.
- Escape key to close the interface.
- Periodic command callback.

The widget is independent of the communication mechanism. Each application provides its own command callback:

- `teleop_limo_real.py`: sends commands through the TCP client.
- `teleop_limo_simu.py`: publishes commands through ROS 2.

This allows the same widget to be reused in more advanced graphical interfaces without duplicating keyboard handling.

## Dependencies

Dependencies depend on the selected executable.

| Dependency | Purpose |
|---|---|
| `rclpy` | ROS 2 Python client library |
| `geometry_msgs` | Velocity command messages |
| `python3-tk` | Tkinter graphical interface |
| `robot_communication` | TCP communication with the real robot |

On the workstation, install Tkinter if it is not already available:

```bash
sudo apt update
sudo apt install python3-tk
```

Verify the installation:

```bash
python3 -c "import tkinter; print('Tkinter OK')"
```


## Package structure

```text
teleoperation/
├── README.md
├── package.xml
├── setup.py
├── setup.cfg
├── resource/
│   └── teleoperation
└── teleoperation/
    ├── __init__.py
    ├── teleop_widget.py
    ├── teleop_limo_real.py
    └── teleop_limo_simu.py
```
