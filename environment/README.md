# Environment

This repository contains simulation environments used for robotics teaching and student projects at ENSEM.

The environments are primarily intended for use with **ROS 2** and **Gazebo**.

## Requirements

The packages have a single version, which builds with both ROS 2 distributions used in `robotic_lab`:

- **Ubuntu 20.04 LTS, ROS 2 Foxy, Gazebo Classic 11** (reference): configuration of the LIMO robots and their workstations; `ensem_room207` is developed and tested with it;
- **Ubuntu 22.04 LTS, ROS 2 Humble, Gazebo Classic** (alternative).

Other configurations may work, but have not been tested. See the [`documentations` README](https://gitlab.univ-lorraine.fr/coll/l-inp/ensem/robotic_lab/documentations) for the setup guides.

## Repository structure

```text
environment/
├── agilex_sand_table/
├── ensem_room207/
├── qcar_large_track/
└── README.md
```

## Available environments

### `agilex_sand_table`

Gazebo environment of the AgileX LIMO simulation table ("sand table"): the 3.0 m × 3.0 m road mat with its 200 mm hoardings. The map texture comes from the top view published by AgileX (MIT licence).

For more information, see the [`agilex_sand_table/README.md`](agilex_sand_table/README.md).

### `ensem_room207`

Gazebo environment of ENSEM Room 207: the QCar large track on its 20 cm platform (same frame as `qcar_large_track`, with traffic lights and the UrbanLoop banner) and the room around it (walls and windows, entrance, whiteboards, workstations, chairs, cabinet, LIMO boxes).

It is the default world of the `room207_limo` simulation. For more information, see the [`ensem_room207/README.md`](ensem_room207/README.md).

### `qcar_large_track`

Gazebo environment representing the large **Quanser QCar SDCS track**.

The environment is provided as a ROS 2 package named `qcar_large_track`.

It contains:

* the Gazebo world;
* the QCar track model;
* the associated materials and textures;
* an RViz configuration;
* a ROS 2 launch file;
* a road map that can be used as a resource for applications such as planning, localization, or perception.

For more information, see the [`qcar_large_track/README.md`](qcar_large_track/README.md).

## Usage

Each environment is provided as an independent ROS 2 package and can be built with `colcon`.

For example:

```bash
colcon build --packages-select qcar_large_track
```

After building the workspace, source the environment:

```bash
source install/setup.bash
```

The large QCar track can then be launched with:

```bash
ros2 launch qcar_large_track qcar_large_track.launch.py
```

Room 207 with:

```bash
ros2 launch ensem_room207 ensem_room207.launch.py
```

and the AgileX sand table with:

```bash
ros2 launch agilex_sand_table agilex_sand_table.launch.py
```

## Organization

This repository contains reusable simulation environments and is independent of the robot-specific software and student projects.

An environment provides the simulated world and its associated resources. Robot models, controllers, navigation, perception, and student applications are provided by other packages.
