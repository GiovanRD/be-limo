# LIMO Platform — Software Origins and Adaptations

## 1. Purpose

This repository maintains the software and tools used to develop, operate, and simulate the AgileX LIMO platform at ENSEM.

The repository includes ROS 2 implementations for different software environments, sensor drivers, development configurations, and MATLAB modelling tools. Its purpose is to provide a reusable platform for teaching and research while keeping environment-specific adaptations identifiable.

## 2. Upstream software

The ROS 2 implementation of the LIMO platform originates from the official [AgileX Robotics `limo_ros2` repository](https://github.com/agilexrobotics/limo_ros2).

Additional components, including sensor drivers and supporting libraries, originate from their respective upstream projects. Their origins and applicable licences should be preserved when integrating or modifying them.

The Foxy implementation also incorporates contributions from [Giulio Cataldo's repository](https://github.com/chullius17/repo_limo_gazebo_env).

## 3. ENSEM adaptations

The software has been adapted to the environments and requirements of ENSEM, particularly for the use of physical LIMO robots and their simulation.

Adaptations may include:

- Compatibility with specific Ubuntu and ROS 2 distributions.
- Build-system and package-configuration fixes.
- Robot descriptions and simulation models.
- Sensor integration and driver configuration.
- Installation, workspace setup, and development scripts.

The exact adaptations depend on the component and its maintained version. Refer to the corresponding source files and Git history for implementation details.

## 4. ROS 2 implementations

The repository maintains separate implementations for different ROS 2 distributions:

- **ROS 2 Foxy / Ubuntu 20.04:** used for the current physical-robot development environment.
- **ROS 2 Humble / Ubuntu 22.04:** maintained as a separate implementation for its corresponding environment.

These implementations should be treated as distinct software baselines. Changes made to one implementation should not automatically be assumed to apply to the other.

Environment-specific installation and build procedures are documented under [`configuration/`](../configuration/).

## 5. Git history and traceability

Where upstream Git history has been preserved, it provides a record of the original software and subsequent changes.

The history should be used to distinguish upstream code from ENSEM adaptations whenever possible. Changes should be committed with clear messages describing their purpose, particularly when modifying upstream robot models, build files, or hardware interfaces.

Upstream licences, copyright notices, and attribution must be retained as required by the relevant projects.

## 6. Project-specific developments

Reusable platform software is maintained in this repository. Teaching activities, experiments, and application-specific developments may be maintained in separate repositories or ROS 2 packages.

This separation helps keep the platform maintainable and allows it to be reused across different teaching and research activities at ENSEM.
