# QCar Large Track

This ROS 2 package provides a Gazebo simulation environment representing the large **Quanser QCar SDCS track**.

The package provides the simulation environment only. Robot models, controllers, navigation, perception, and student applications are handled by other packages.

## Package structure

```text
qcar_large_track/
├── CMakeLists.txt
├── env/
│   └── qcar_large_track.dsv
├── launch/
│   └── qcar_large_track.launch.py
├── models/
│   └── qcar_large_track/
│       ├── materials/
│       │   ├── scripts/
│       │   │   └── qcar_large_track.material
│       │   └── textures/
│       │       ├── SDCS_MapLayout.png
│       │       └── SDCS_RoadMap_RightHandTraffic.png
│       ├── model.config
│       └── model.sdf
├── package.xml
├── rviz/
│   └── qcar_large_track.rviz
├── worlds/
│   └── qcar_large_track.world
└── README.md
```

## Environment

The environment consists of a Gazebo world containing:

* the QCar large track;
* a sun light source.

The track is represented by a static Gazebo model with dimensions:

```text
4.88 m × 6.20 m
```

The track model is composed of:

* a collision plane used for physical interaction with the robot;
* a visual plane using the SDCS track texture.

The visual and collision geometries are defined independently in `model.sdf`.

## Model and materials

The track model is defined in:

```text
models/qcar_large_track/model.sdf
```

The model is static and contains a single link named `track`.

The visual appearance is defined by the material:

```text
QCar/SDCS_Map
```

This material is defined in:

```text
models/qcar_large_track/materials/scripts/qcar_large_track.material
```

and uses the texture:

```text
SDCS_MapLayout.png
```

The associated resources are organized as follows:

```text
model.sdf
   │
   └── QCar/SDCS_Map
            │
            ▼
qcar_large_track.material
            │
            ▼
SDCS_MapLayout.png
```

## Road map

The package also provides:

```text
SDCS_RoadMap_RightHandTraffic.png
```

This image is not used as the visual texture of the Gazebo model.

It is provided as an additional map resource that can be used by other applications, for example for:

* trajectory planning;
* navigation;
* localization;
* perception;
* road or lane interpretation.

The interpretation and use of this map are handled by the corresponding application or ROS 2 package.

## Gazebo model discovery

The package installs the track model in:

```text
share/qcar_large_track/models
```

An environment hook adds this directory to `GAZEBO_MODEL_PATH`.

This allows Gazebo to resolve:

```text
model://qcar_large_track
```

to the corresponding installed model.

The environment hook is defined in:

```text
env/qcar_large_track.dsv
```

## Launch

The package provides the launch file:

```text
launch/qcar_large_track.launch.py
```

It starts Gazebo using:

```text
worlds/qcar_large_track.world
```

The launch file does not start a robot.

Its role is limited to launching the simulation environment. A higher-level launch file can subsequently combine this environment with a robot, RViz, controllers, navigation nodes, or other components.

Conceptually:

```text
qcar_large_track.launch.py
          │
          ▼
qcar_large_track.world
          │
          ├── sun
          │
          └── qcar_large_track
                    │
                    ▼
                model.sdf
```

## RViz configuration

An RViz configuration is provided in:

```text
rviz/qcar_large_track.rviz
```

This configuration can be used by a higher-level simulation launch or loaded manually in RViz.

It is not automatically started by `qcar_large_track.launch.py`.

## Build

From the root of the ROS 2 workspace:

```bash
colcon build --packages-select qcar_large_track
```

Then source the workspace:

```bash
source install/setup.bash
```

## Launch the environment

```bash
ros2 launch qcar_large_track qcar_large_track.launch.py
```

This launches Gazebo with the QCar large track environment.

The robot and its associated software must be launched separately.
