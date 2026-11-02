# Robotic Lab

This GitLab group contains the software resources and projects used for robotics teaching at ENSEM, built around the AgileX LIMO, the Quanser QCar 2 and LEGO SPIKE Prime robots. The common repositories are meant to serve all the robots.

The organization separates:

* common resources;
* documentation;
* reference environments maintained by the teaching staff;
* student projects.

---

## 1. Reference environment

Each robot uses the ROS 2 version of its manufacturer software, for the physical robot and for its simulation, so that software developed in simulation runs unchanged on the robot:

* **LIMO: Ubuntu 20.04 with ROS 2 Foxy** (the reference environment of the group);
* **QCar 2: Ubuntu 22.04 with ROS 2 Humble**, the version installed by Quanser.

| Use | Environment |
|---|---|
| Physical LIMO robot | Ubuntu 20.04, ROS 2 Foxy |
| LIMO simulation workstation | Ubuntu 20.04, ROS 2 Foxy, Gazebo Classic 11, RViz2, on Windows (WSL 2 + Docker) or native Ubuntu |
| Physical QCar 2 | ROS 2 Humble, Quanser software |
| QCar 2 simulation workstation | Ubuntu 22.04, ROS 2 Humble, Gazebo Classic 11, RViz2 (native or WSL 2) |
| LEGO SPIKE bike simulation | Ubuntu 22.04, ROS 2 Humble, Gazebo Classic 11 (native or WSL 2); the real robot runs SPIKE 3 Python |
| MATLAB/Simulink | MATLAB R2026b with Simulink; ROS Toolbox only for the connection to Gazebo |

---

## 2. Setup guides

| Guide | Environment | Location |
|---|---|---|
| [LIMO Windows workstation](https://gitlab.univ-lorraine.fr/coll/l-inp/ensem/robotic_lab/limo/-/blob/main/configuration/wsl2_ubuntu20.04_foxy/README.md) | Windows + WSL 2 + Docker, Ubuntu 20.04, Foxy | `limo/configuration/` |
| [LIMO Ubuntu workstation](https://gitlab.univ-lorraine.fr/coll/l-inp/ensem/robotic_lab/limo/-/blob/main/configuration/native_ubuntu20.04_foxy/README.md) | Native Ubuntu 20.04, Foxy | `limo/configuration/` |
| [LIMO robot software](https://gitlab.univ-lorraine.fr/coll/l-inp/ensem/robotic_lab/limo/-/blob/main/configuration/limo_ubuntu20.04_foxy/README.md) | LIMO, Ubuntu 20.04, Foxy | `limo/configuration/` |
| [LIMO 1 operating system](limo1_ubuntu20.04_foxy/README.md) | Flashing Ubuntu 20.04 and installing ROS 2 Foxy on a LIMO 1 | this repository |
| [Humble workstation](workstation_ubuntu22.04_humble/README.md) | Ubuntu 22.04, ROS 2 Humble (QCar 2 simulation, or alternative for the LIMO) | this repository |
| [QCar 2 simulation](https://gitlab.univ-lorraine.fr/coll/l-inp/ensem/robotic_lab/qcar/-/blob/main/README.md) | QCar 2 packages on a Humble workstation | `qcar/` |
| [LEGO SPIKE bike simulation](https://gitlab.univ-lorraine.fr/coll/l-inp/ensem/robotic_lab/lego/-/blob/main/README.md) | SPIKE bike packages on a Humble workstation | `lego/` |
| [Ubuntu 22.04 workstation in the `limo` repository](https://gitlab.univ-lorraine.fr/coll/l-inp/ensem/robotic_lab/limo/-/blob/main/configuration/native_ubuntu22.04_humble/README.md) | Native Ubuntu 22.04, Humble, LIMO packages | `limo/configuration/` |
| [Room 206 WSL image (in French)](room206_wsl_ubuntu24.04_jazzy/README.md) | Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic, for the ROS 2 course in room 206. Not the LIMO environment. | this repository |

---

## 3. Group organization

The general organization is:

```text
Robotic Lab
│
├── documentations
├── limo
├── qcar
├── lego
├── environment
├── teleoperation
├── recording
├── robot-communication
│
├── Teaching Staff
│   └── room207-limo
│
└── Students
    └── BE
        ├── BE-2026-Group-01
        ├── BE-2026-Group-02
        └── BE-2026-Group-03
```

### Common repositories

The repositories directly under `Robotic Lab` provide shared building blocks for the different projects:

| Repository | Content | Versions |
|---|---|---|
| [`documentations`](https://gitlab.univ-lorraine.fr/coll/l-inp/ensem/robotic_lab/documentations) | Group overview and setup guides | — |
| [`limo`](https://gitlab.univ-lorraine.fr/coll/l-inp/ensem/robotic_lab/limo) | LIMO robot software, Gazebo model, sensor drivers, environment configurations and MATLAB/Simulink model | ROS 2 Foxy, ROS 2 Humble, MATLAB/Simulink |
| [`qcar`](https://gitlab.univ-lorraine.fr/coll/l-inp/ensem/robotic_lab/qcar) | QCar 2 robot description (from Quanser) and Gazebo simulation | ROS 2 Humble |
| [`lego`](https://gitlab.univ-lorraine.fr/coll/l-inp/ensem/robotic_lab/lego) | LEGO SPIKE Prime self-balancing bike: Gazebo simulation that runs the SPIKE Python programs unchanged | ROS 2 Humble |
| [`environment`](https://gitlab.univ-lorraine.fr/coll/l-inp/ensem/robotic_lab/environment) | Gazebo worlds: Room 207, the QCar large track and the AgileX sand table, shared by all the robots | ROS 2 Foxy and Humble |
| [`teleoperation`](https://gitlab.univ-lorraine.fr/coll/l-inp/ensem/robotic_lab/teleoperation) | Keyboard teleoperation of the simulated and real LIMO | ROS 2 Foxy |
| [`recording`](https://gitlab.univ-lorraine.fr/coll/l-inp/ensem/robotic_lab/recording) | ROS 2 bag recording and its start/stop services | ROS 2 Foxy |
| [`robot-communication`](https://gitlab.univ-lorraine.fr/coll/l-inp/ensem/robotic_lab/robot-communication) | TCP link between a PC and the robot (commands, odometry, recording control) | ROS 2 Foxy |

Students have read access to these repositories and can use them as resources for their projects.

### Teaching Staff

The `Teaching Staff` subgroup contains resources maintained by the teaching staff.

The [`room207-limo`](https://gitlab.univ-lorraine.fr/coll/l-inp/ensem/robotic_lab/teaching-staff/room207-limo) repository is the reference environment for the **BE Véhicule Autonome** project. It is provided in two versions:

* **ROS 2 Foxy** (`ros2_foxy/`): Gazebo simulation of Room 207, RViz2 configuration, control window (camera, teleoperation, recording, live trajectory) and odometry analysis.
* **MATLAB/Simulink** (`matlab/`): the same demonstration in Simulink, standalone or connected to the Gazebo simulation through ROS 2.

This repository is maintained by the teaching staff and should not be modified by students.

### Students

The `Students/BE` subgroup contains the student projects. Each group has its own repository, which is used as its development workspace.

---

## 4. Repository layout

The repositories follow the same layout rules, so that the version of a piece of software is clear from its folder.

**Robot and demonstration repositories** (`limo`, `qcar`, `lego`, `room207-limo`) contain one folder per version, even when only one exists so far:

```text
<repository>/
├── ros2_foxy/        ROS 2 Foxy packages
├── ros2_humble/      ROS 2 Humble packages
├── matlab/           MATLAB/Simulink version, when provided
├── configuration/    one folder per environment, when provided,
│                     named <platform>_<ubuntu version>_<ROS 2 distribution>
└── README.md
```

A ROS 2 workspace must contain only one version of each package. Follow the setup guide of the repository to get the right folder: the `limo` guides use a Git sparse checkout of `ros2_foxy/`, while `room207-limo`, which has a single ROS 2 version, is cloned entirely (`colcon` finds the package in `ros2_foxy/` and ignores `matlab/`).

**Common tool and world repositories** (`environment`, `teleoperation`, `recording`, `robot-communication`) have a single version: they keep their ROS 2 package(s) at the root, and their README states the supported ROS 2 distribution.

This repository follows the same idea: one folder per guide, named after its target environment.

```text
documentations/
├── limo1_ubuntu20.04_foxy/            LIMO 1 operating system and ROS 2 Foxy
├── workstation_ubuntu22.04_humble/    Humble workstation (alternative)
├── room206_wsl_ubuntu24.04_jazzy/     Room 206 ROS 2 course image (in French)
└── README.md
```

---

## 5. Starting a student project

Each group repository is created as a fork of the reference environment:

```text
Teaching Staff/room207-limo

                │

                │ fork
                ▼

Students/BE/BE-2026-Group-XX
```

The fork provides the initial simulation environment, in its ROS 2 and MATLAB/Simulink versions. The group then develops independently in its own repository.

The reference repository remains unchanged by student work.

---

## 6. Access rights

| Resource | Student access | Purpose |
|---|---|---|
| Common repositories | `Reporter` | Use and inspect shared resources |
| Own project | `Developer` | Develop and push code |
| Teaching Staff | No access | Staff-only resources |

Students should therefore carry out their development in:

```text
Students/BE/BE-2026-Group-XX
```

The common repositories should be treated as shared resources rather than development repositories.

---

## 7. Git workflow

Students are encouraged to use Git throughout the project:

* commit changes regularly;
* use descriptive commit messages;
* create branches when appropriate;
* push their work to the group repository.

Examples:

```text
Add LIMO trajectory controller

Fix odometry recording

Add traffic sign detection

Update RViz configuration

Improve trajectory tracking
```

The Git history provides a record of the project's development.

---

## 8. Updates to common resources

The teaching staff may update the common repositories or the reference environment during the project.

Student repositories are not automatically updated when these repositories change.

When an update needs to be incorporated into a student project, the teaching staff will indicate the appropriate procedure.

---

## 9. Basic rules

### Students

> **Use the common repositories as resources.
> Develop in your group's repository.
> Do not modify the reference repositories.**

### Teaching staff

> **Maintain common resources under `Robotic Lab`, reference environments under `Teaching Staff`, and student projects under `Students`.
> Put each version of a piece of software in its own folder (`ros2_<distribution>/`, `matlab/`).**
