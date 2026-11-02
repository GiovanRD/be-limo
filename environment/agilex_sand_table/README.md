# AgileX Sand Table

This ROS 2 package provides a Gazebo simulation environment representing the **AgileX LIMO simulation table** ("sand table"): the 3.0 m × 3.0 m road mat sold by AgileX for teaching with the LIMO, with its hoardings.

The package provides the simulation environment only. Robots are spawned by their own packages (`limo_car`, `qcar2_gazebo`, ...).

## Package structure

```text
agilex_sand_table/
├── CMakeLists.txt
├── env/
│   └── agilex_sand_table.dsv
├── launch/
│   └── agilex_sand_table.launch.py
├── models/
│   └── agilex_sand_table/
│       ├── materials/
│       │   ├── scripts/
│       │   │   └── agilex_sand_table.material
│       │   └── textures/
│       │       ├── agilex_sand_table.png
│       │       └── LICENSE
│       ├── model.config
│       └── model.sdf
├── package.xml
├── worlds/
│   └── agilex_sand_table.world
└── README.md
```

## Environment

The world contains:

* the sand table: 16 plates of 750 × 750 × 5 mm, i.e. **3.0 m × 3.0 m**, centred on the world origin, top surface at z = 0;
* the hoardings: 200 mm high walls around the table (16 panels of 750 × 200 × 5 mm in reality, one wall per side in the model), which the lidar sees as on the real table;
* a ground plane under the plates and a sun light source.

The dimensions come from the AgileX *Limo Simulation Table Manual*. The trees, traffic light, uphill ramp, character whiteboard and lifting lever of the real table are not modelled.

```text
          +y
           ▲
   ┌───────┼───────┐  1.5 m
   │ 13  14│ 15  16│
   │  9  10│ 11  12│
───┼───────┼───────┼──► +x
   │  5   6│  7   8│
   │  1   2│  3   4│
   └───────┴───────┘ -1.5 m
```

The plate numbers are those of the AgileX assembly guide (the numbered stickers under the plates).

## Map texture

The texture `agilex_sand_table.png` is made from the top view of the table published by AgileX in [`agilexrobotics/limo-doc`](https://github.com/agilexrobotics/limo-doc) (`sand table_image/sand table_6.png`, MIT licence, see `materials/textures/LICENSE`):

* the thin white lines of the plate joints were removed by interpolation;
* the image was resized from 550 × 550 to 1024 × 1024 px (2.9 mm per pixel).

The source image is small (5.5 mm per pixel): the markings look soft at close range. A larger texture does not add detail and is not rendered by the software OpenGL of WSL.

As for `qcar_large_track`, the top of the image is towards +y and its left side towards −x.

## Gazebo model discovery

An environment hook adds `share/agilex_sand_table/models` to `GAZEBO_MODEL_PATH`, so that Gazebo resolves `model://agilex_sand_table`.

## Build

From the root of the ROS 2 workspace:

```bash
colcon build --packages-select agilex_sand_table
source install/setup.bash
```

## Launch the environment

```bash
ros2 launch agilex_sand_table agilex_sand_table.launch.py
```

This launches Gazebo with the sand table only.

### With a robot

The robot launch files take the world as an argument. Get its path with:

```bash
WORLD=$(ros2 pkg prefix agilex_sand_table)/share/agilex_sand_table/worlds/agilex_sand_table.world
```

QCar 2 (ROS 2 Humble), on the bottom road heading west:

```bash
ros2 launch qcar2_gazebo qcar2_gazebo.launch.py world:=$WORLD x:=0.5 y:=-1.2 yaw:=3.1416
```

LIMO (ROS 2 Foxy), at the Room 207 spawn position (x = −0.73 m, y = −0.09 m, heading north), which falls on the road between the left islands, just below the parking bay:

```bash
ros2 launch room207_limo room207_limo.launch.py world:=$WORLD spawn_z:=0.15
```

## Tests carried out

With ROS 2 Humble and Gazebo 11 (headless, WSL 2): the package builds, the world loads without errors, the QCar 2 stands on the plates at z = 0 and drives 0.6 m along the bottom road. The visual appearance was not checked in a rendered image (camera rendering is not available headless under WSL).
