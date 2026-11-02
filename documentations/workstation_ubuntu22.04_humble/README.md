# Workstation setup — Ubuntu 22.04 and ROS 2 Humble

> **Alternative configuration.** The reference environment of `robotic_lab` is **Ubuntu 20.04 + ROS 2 Foxy**, for the physical LIMO robots and for the simulation. To prepare a workstation with the reference environment, use the setup guides of the `limo` repository:
>
> - [Windows + WSL 2 + Docker, Ubuntu 20.04 / Foxy](https://gitlab.univ-lorraine.fr/coll/l-inp/ensem/robotic_lab/limo/-/blob/main/configuration/wsl2_ubuntu20.04_foxy/README.md)
> - [Native Ubuntu 20.04 / Foxy](https://gitlab.univ-lorraine.fr/coll/l-inp/ensem/robotic_lab/limo/-/blob/main/configuration/native_ubuntu20.04_foxy/README.md)
>
> This guide installs a Humble workstation instead. With Humble, use the `ros2_humble/` version of the `limo` repository. The Room 207 demonstration (`room207-limo`) is developed and tested with Foxy only.

This document describes how to install a ROS 2 Humble development environment for the `robotic_lab` workspace, on Windows with WSL 2 or on a native Ubuntu 22.04.

The environment uses:

* Ubuntu 22.04
* ROS 2 Humble
* Gazebo
* RViz2
* Python 3
* Git

It can be installed either on:

1. **Windows 11 + WSL 2 + Ubuntu 22.04**
2. **Native Ubuntu 22.04**

Both configurations provide the same ROS 2 Humble environment.

The physical LIMO robots run Ubuntu 20.04 with ROS 2 Foxy. Software developed on a Humble workstation must therefore be checked with Foxy before it is used on a robot.

---

# 1. Configuration overview

## 1.1. Windows + WSL 2

```text
Windows 11
└── WSL 2
    └── Ubuntu 22.04
        ├── ROS 2 Humble
        ├── Gazebo
        ├── RViz2
        └── ~/robotic_lab
```

This configuration is used when developing from a Windows computer.

WSLg provides the graphical support required to run applications such as Gazebo and RViz2.

---

## 1.2. Native Ubuntu

```text
Ubuntu 22.04
├── ROS 2 Humble
├── Gazebo
├── RViz2
└── ~/robotic_lab
```

This configuration is used when Ubuntu 22.04 is installed directly on the computer.

The ROS 2 environment and project workflow are the same as with WSL 2.

---

## 1.3. Real robot

The physical LIMO robot uses a separate environment:

```text
LIMO robot
└── Ubuntu 20.04
    └── ROS 2 Foxy
        ├── LIMO driver
        ├── teleoperation
        └── recording
```

The real robot environment is not covered by the Ubuntu 22.04 / ROS 2 Humble installation instructions in this document.

---

# 2. Configuration A — Windows + WSL 2 + Ubuntu 22.04

## 2.1. Install WSL 2

From PowerShell:

```powershell
wsl --install
```

Restart Windows if requested.

Check the installation:

```powershell
wsl --status
```

Check the installed distributions:

```powershell
wsl --list --verbose
```

The Ubuntu distribution should use WSL 2.

If necessary:

```powershell
wsl --set-version Ubuntu 2
```

---

## 2.2. Install Ubuntu 22.04

Ubuntu 22.04 can be installed from PowerShell with:

```powershell
wsl --install -d Ubuntu-22.04
```

Launch Ubuntu from Windows and create the Linux user account.

Check the Ubuntu version:

```bash
lsb_release -a
```

The expected version is:

```text
Ubuntu 22.04
```

---

## 2.3. WSL graphical environment

Gazebo and RViz2 require graphical support.

WSLg is normally installed with current versions of WSL.

Check the WSL version from PowerShell:

```powershell
wsl --version
```

The output should include a WSLg version.

If Gazebo or RViz2 cannot start correctly, verify that WSLg and the Windows graphics drivers are correctly installed.

The ROS 2 workspace should be located in the Linux filesystem:

```text
~/robotic_lab
```

rather than under:

```text
/mnt/c/...
```

This avoids unnecessary filesystem performance issues when building ROS 2 packages under WSL.

---

# 3. Configuration B — Native Ubuntu 22.04

No additional virtualization layer is required.

Check the Ubuntu version:

```bash
lsb_release -a
```

The expected version is:

```text
Ubuntu 22.04
```

The rest of the installation and development procedure is identical to the WSL configuration.

---

# 4. Ubuntu development environment

The following setup is identical for both simulation configurations.

Whether Ubuntu 22.04 is running natively or through WSL 2, perform the following steps from the Ubuntu terminal.

---

## 4.1. Update Ubuntu

```bash
sudo apt update
sudo apt upgrade
```

Install the basic development tools:

```bash
sudo apt install -y \
    build-essential \
    curl \
    git \
    wget \
    software-properties-common
```

---

# 5. Install ROS 2 Humble

ROS 2 Humble is the ROS 2 distribution of this configuration.

It must be installed inside Ubuntu 22.04.

Follow the official ROS 2 Humble installation procedure and install the Desktop distribution.

The Desktop installation provides the ROS 2 tools as well as RViz2.

After installation, source ROS 2:

```bash
source /opt/ros/humble/setup.bash
```

Check the ROS distribution:

```bash
echo $ROS_DISTRO
```

Expected result:

```text
humble
```

To source ROS 2 automatically in every terminal:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

Then reload the configuration:

```bash
source ~/.bashrc
```

> **Note:** the real LIMO robot uses ROS 2 Foxy on Ubuntu 20.04. ROS 2 Humble is therefore specific to the simulation environment described in this document.

---

# 6. Gazebo and RViz2

Gazebo is required for the simulation environment.

Check that Gazebo is available:

```bash
gazebo --version
```

If Gazebo is not installed, install the version required by the ROS 2 Humble environment and the project dependencies.

The `room207-limo` package provides the Gazebo configuration used by the project.

RViz2 is installed with the ROS 2 Humble Desktop installation.

Launch RViz2 with:

```bash
rviz2
```

A graphical RViz2 window should appear.

Gazebo and RViz2 are only required for the simulation environment. They are not part of the software environment running on the physical LIMO robot.

---

# 7. ROS 2 workspace

`robotic_lab` is the ROS 2 workspace.

The workspace itself is **not a Git repository**.

The different project components are maintained in separate Git repositories and are cloned into the `src/` directory.

For the simulation environment, the workspace is:

```text
~/robotic_lab/
└── src/
    ├── limo/
    ├── environment/
    ├── teleoperation/
    ├── recording/
    └── teaching-staff/
        └── room207-limo/
```

The `teleoperation` and `recording` repositories can also be used in the real robot environment, with the appropriate ROS 2 distribution and robot-specific dependencies.

After building the simulation workspace, ROS 2 generates:

```text
~/robotic_lab/
├── src/
├── build/
├── install/
├── log/
└── data/
```

Only the directories under `src/` are source repositories.

The following directories are generated by ROS 2:

```text
build/
install/
log/
```

The `data/` directory contains recorded data and generated results.

---

# 8. Set up the ROS 2 workspace

## 8.1. Create the workspace

Create the workspace and its source directory:

```bash
mkdir -p ~/robotic_lab/src
```

Then enter the source directory:

```bash
cd ~/robotic_lab/src
```

---

## 8.2. Clone the common repositories

Each component is maintained in a separate Git repository.

Clone the repositories:

```bash
git clone <limo-repository-url>
git clone <environment-repository-url>
git clone <teleoperation-repository-url>
git clone <recording-repository-url>
```

Git automatically creates a directory using the repository name.

The resulting structure is:

```text
~/robotic_lab/
└── src/
    ├── limo/
    ├── environment/
    ├── teleoperation/
    └── recording/
```

Each directory is an independent Git repository.

The `teleoperation` and `recording` repositories are intended to be reusable in both the simulation and real robot environments.

---

## 8.3. Clone the simulation repository

The simulation repository is located under the `Teaching Staff` subgroup.

Create the corresponding directory:

```bash
mkdir -p ~/robotic_lab/src/teaching-staff
```

Clone the repository:

```bash
cd ~/robotic_lab/src/teaching-staff

git clone <room207-limo-repository-url>
```

Git automatically creates the `room207-limo` directory.

The complete simulation source tree is then:

```text
~/robotic_lab/
└── src/
    ├── limo/
    ├── environment/
    ├── teleoperation/
    ├── recording/
    └── teaching-staff/
        └── room207-limo/
```

`room207-limo` is specific to the simulation environment and is not required on the physical LIMO robot.

> **Note:** `room207-limo` contains only a ROS 2 Foxy version (`ros2_foxy/`) and is not tested with Humble. Students work in their fork of `room207-limo` (see the [`documentations` README](../README.md)).

---

## 8.4. Build the workspace

The workspace must be built from its root directory:

```bash
cd ~/robotic_lab
```

Source ROS 2:

```bash
source /opt/ros/humble/setup.bash
```

Build all packages:

```bash
colcon build
```

Then source the workspace:

```bash
source install/setup.bash
```

> **Note:** this build procedure applies to the Ubuntu 22.04 / ROS 2 Humble simulation environment. The real robot uses Ubuntu 20.04 / ROS 2 Foxy and must be built in its corresponding environment.

---

# 9. Git workflow

Because each component is a separate Git repository, Git commands must be executed inside the corresponding repository.

For example, for the `teleoperation` package:

```bash
cd ~/robotic_lab/src/teleoperation

git status
git add .
git commit -m "Update teleoperation"
git push
```

For the `recording` package:

```bash
cd ~/robotic_lab/src/recording

git status
git add .
git commit -m "Update recording"
git push
```

The ROS 2 workspace itself is not used as a single Git repository.

Do not modify generated files inside:

```text
build/
install/
log/
```

---

# 10. Environment verification

Before starting development, check the main components of the simulation environment.

## Ubuntu

```bash
lsb_release -a
```

Expected:

```text
Ubuntu 22.04
```

## ROS 2

```bash
echo $ROS_DISTRO
```

Expected:

```text
humble
```

## ROS 2 tools

```bash
ros2 --help
```

## Gazebo

```bash
gazebo --version
```

## RViz2

```bash
rviz2
```

A graphical RViz2 window should appear.

## Workspace

```bash
cd ~/robotic_lab
ls
```

After a successful build, the simulation workspace should contain:

```text
src
build
install
log
```

The physical LIMO robot uses a separate Ubuntu 20.04 / ROS 2 Foxy environment. The `teleoperation` and `recording` repositories are shared between the two environments, while the simulation-specific components remain in the Humble workspace.
