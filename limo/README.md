# LIMO

Software, models, and documentation for the AgileX LIMO platform at ENSEM. This repository maintains the ROS 2 implementations, sensor drivers, development configurations, and MATLAB tools used with the physical robots and their simulation environments.

## Repository structure

```text
limo/
├── configuration/                    one folder per environment
│   ├── limo_ubuntu20.04_foxy/        physical LIMO
│   ├── native_ubuntu20.04_foxy/      workstation, Ubuntu 20.04
│   ├── wsl2_ubuntu20.04_foxy/        Windows + WSL2 + Docker
│   └── native_ubuntu22.04_humble/    workstation, Ubuntu 22.04
├── ros2_foxy/                        ROS 2 Foxy version
│   ├── limo_ros2/
│   ├── limo_start/
│   ├── ros2_astra_camera/
│   ├── ydlidar_ros2_driver/
│   └── YDLidar-SDK/
├── ros2_humble/                      ROS 2 Humble version
│   └── limo_ros2/
├── matlab/                           MATLAB/Simulink version
├── documentation/
└── README.md
```

The repository follows the `robotic_lab` convention: one folder per version of the software (`ros2_foxy/`, `ros2_humble/`, `matlab/`) and one folder per environment in `configuration/`. It is organized around three complementary concerns:

- **Configurations** describe how to prepare a development environment and build the software.
- **ROS 2 implementations** contain the robot software, sensor drivers, and supporting libraries for specific ROS 2 distributions.
- **MATLAB and documentation** provide modelling tools and technical information about the platform and its integration at ENSEM.

ROS 2 Foxy on Ubuntu 20.04 is the reference version, for the physical robots and for the simulation. The Humble version is maintained separately and is not tested with the Room 207 simulation.

## Configurations

The `configuration/` directory contains environment-specific setup instructions and scripts for developing with the LIMO platform.

| Configuration | Environment | Documentation |
|---|---|---|
| `limo_ubuntu20.04_foxy/` | LIMO-specific Ubuntu 20.04 + ROS 2 Foxy setup | [README](configuration/limo_ubuntu20.04_foxy/README.md) |
| `native_ubuntu20.04_foxy/` | Native Ubuntu 20.04 + ROS 2 Foxy | [README](configuration/native_ubuntu20.04_foxy/README.md) |
| `native_ubuntu22.04_humble/` | Native Ubuntu 22.04 + ROS 2 Humble | [README](configuration/native_ubuntu22.04_humble/README.md) |
| `wsl2_ubuntu20.04_foxy/` | Ubuntu 20.04 + ROS 2 Foxy under WSL2 | [README](configuration/wsl2_ubuntu20.04_foxy/README.md) |

Choose the configuration matching your operating system and ROS 2 distribution. Each configuration documents its own setup procedure and dependencies.

Follow the README corresponding to your environment before building the workspace. Configuration-specific setup instructions take precedence over generic build instructions.

## ROS 2 implementations

The ROS software is maintained separately for each ROS 2 distribution to accommodate differences in operating systems, dependencies, and hardware integration.

### ROS 2 Foxy — Ubuntu 20.04

The Foxy implementation targets Ubuntu 20.04. It is the reference version, used with the physical LIMO robots and with the Gazebo simulation (Room 207).

| Component | Purpose |
|---|---|
| [`limo_ros2/`](ros2_foxy/limo_ros2/) | Core robot software: base interface, bringup, robot description, simulation-related resources, and message definitions |
| [`limo_start/`](ros2_foxy/limo_start/) | Launch and configuration files for the physical ENSEM LIMO |
| [`ros2_astra_camera/`](ros2_foxy/ros2_astra_camera/) | ROS 2 driver and messages for the Astra depth camera |
| [`ydlidar_ros2_driver/`](ros2_foxy/ydlidar_ros2_driver/) | ROS 2 driver for the YDLIDAR sensor |
| [`YDLidar-SDK/`](ros2_foxy/YDLidar-SDK/) | Underlying YDLIDAR software development kit |

The Foxy implementation is based on the work of **Giulio Cataldo**. Many thanks to Giulio for his contribution. See the [original repository](https://github.com/chullius17/repo_limo_gazebo_env).

For installation and build instructions, see the [native Ubuntu 20.04 + Foxy configuration](configuration/native_ubuntu20.04_foxy/README.md).

### ROS 2 Humble — Ubuntu 22.04

The Humble implementation is maintained separately for Ubuntu 22.04.

- [`ros2_humble/limo_ros2/`](ros2_humble/limo_ros2/)

Use the ROS 2 distribution that matches the target environment. Do not mix packages from the Foxy and Humble implementations in the same workspace unless compatibility has been explicitly established.

## MATLAB/Simulink version

The [`matlab/`](matlab/) directory contains the MATLAB/Simulink model of the LIMO (R2026b). The Simulink library `limo_lib.slx` provides a `LIMO (Gazebo)` block with the same dynamics as the Gazebo LIMO of the ROS 2 Foxy version. Its mask selects the input: speed and steering angle, `/cmd_vel`, or joint torques. The folder also contains a demo model, the `+limo` package with the equations, a script that compares the model with a Gazebo rosbag, and tests.

MATLAB-only work does not require a ROS 2 installation. See the [MATLAB README](matlab/README.md).

## Documentation

The [`documentation/`](documentation/) directory contains technical documentation about the LIMO platform, its integration at ENSEM, and the adaptations made to the software.

## Development workflow

For ROS 2 development:

1. Identify the target operating system and ROS 2 distribution.
2. Follow the corresponding configuration instructions.
3. Build the software using the workspace and build procedure specified by that configuration.
4. Keep platform-level improvements in this repository and teaching-project-specific developments in their respective repositories.

The configuration scripts prepare the workspace and handle the required build steps. **Do not copy the ROS packages manually into a workspace unless the relevant configuration explicitly instructs you to do so.**

## Project organization

This repository provides reusable platform software and supporting tools. Teaching activities, robot experiments, and project-specific developments are maintained separately where appropriate, allowing the LIMO platform to be reused across different activities at ENSEM.
