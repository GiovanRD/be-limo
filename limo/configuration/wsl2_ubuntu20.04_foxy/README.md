# Windows setup: Ubuntu 20.04 and ROS 2 Foxy in Docker

This profile provides an Ubuntu 20.04 x86_64 development environment with ROS 2 Foxy and the tools required to build and run the AgileX LIMO ROS 2 packages.

Docker Desktop runs the container on Windows through WSL2. The LIMO source code remains in the Git repository, while the ROS 2 workspace and build artifacts are stored in a dedicated Docker volume.

Run Windows commands in PowerShell, Linux commands in the Ubuntu WSL terminal, and ROS commands inside the container, as indicated.

## 1. Install Ubuntu 20.04 with WSL2

Use an x86_64 Windows computer that supports Docker Desktop and WSLg. Hardware virtualization must be enabled in the BIOS/UEFI.

Open **PowerShell as Administrator** and run:

```powershell
wsl --install -d Ubuntu-20.04
```

Restart Windows if prompted. Open **Ubuntu 20.04** from the Start menu and create your Linux username and password when prompted.

Back in PowerShell, update WSL and check the installed distributions:

```powershell
wsl --update
wsl --list --verbose
```

The `Ubuntu-20.04` row must show `VERSION 2`. If it shows `1`, run:

```powershell
wsl --set-version Ubuntu-20.04 2
```

If Ubuntu 20.04 is already installed in WSL2, skip the installation command.

See the [Microsoft WSL command reference](https://learn.microsoft.com/en-us/windows/wsl/basic-commands).

## 2. Install and configure Docker Desktop

Download and install [Docker Desktop for Windows](https://docs.docker.com/desktop/setup/install/windows-install/).

Start Docker Desktop and complete its initial setup. In **Settings**:

1. Under **General**, enable **Use the WSL 2 based engine**, if the option is shown.
2. Under **Resources > WSL Integration**, enable **Ubuntu-20.04**.
3. Click **Apply & restart**.
4. Make sure Docker Desktop uses Linux containers.

Keep Docker Desktop running while using this workspace.

See the [Docker Desktop WSL2 guide](https://docs.docker.com/desktop/features/wsl/).

This profile uses Docker Desktop's Docker Engine. Do not install a separate Docker Engine inside Ubuntu WSL.

## 3. Get the project

Open PowerShell and start the Ubuntu 20.04 WSL distribution:

```powershell
wsl -d Ubuntu-20.04
```

This opens a Bash terminal inside Ubuntu. Run the following actions inside the Bash terminal unless otherwise indicated.

Clone the repository from GitLab. Run:

```bash
mkdir -p ~/robotic_lab
cd ~/robotic_lab

git clone https://gitlab.univ-lorraine.fr/coll/l-inp/ensem/robotic_lab/limo.git
cd limo

git sparse-checkout set \
  configuration/wsl2_ubuntu20.04_foxy \
  ros2_foxy/limo_ros2 \
  ros2_foxy/ros2_astra_camera \
  ros2_foxy/ydlidar_ros2_driver \
  ros2_foxy/YDLidar-SDK
```

The repository is now located at `~/robotic_lab/limo` in the Ubuntu filesystem.

The sparse checkout retrieves only the directories required by this development profile:

- `configuration/wsl2_ubuntu20.04_foxy/`: Docker configuration and helper scripts.
- `ros2_foxy/limo_ros2/`: LIMO ROS 2 packages.
- `ros2_foxy/ros2_astra_camera/`: Astra camera driver.
- `ros2_foxy/ydlidar_ros2_driver/`: YDLIDAR driver.
- `ros2_foxy/YDLidar-SDK/`: YDLIDAR SDK, required to build the driver.

Verify that the configuration files are available:

```bash
ls configuration/wsl2_ubuntu20.04_foxy
git status
```

## 4. Check the setup

Navigate to the repository root if you are not already there:

```bash
cd ~/robotic_lab/limo
```

Run the setup checks:

```bash
bash configuration/wsl2_ubuntu20.04_foxy/check-setup.sh
```

The script checks the WSL2 environment, Docker Desktop integration, Docker Compose, and WSLg.

If the setup is correct, you should see output similar to the following:

```text
Environment checks passed.
Docker version 29.8.2, build 7fc2dff
Docker Compose version v5.5.1
Repository: /home/jkreiss/robotic_lab/limo
```

## 5. Configure the project

This section defines the Docker runtime options, including GPU support, host networking, and ROS 2 communication settings. These options are stored in the `.env` file and used in the following steps.

If `.env` already exists with your desired configuration, skip this section.

Navigate to the configuration directory:

```bash
cd ~/robotic_lab/limo/configuration/wsl2_ubuntu20.04_foxy
```

Create the default configuration file:

```bash
cp .env.example .env
```

The default configuration disables GPU support and host networking. Software OpenGL is enabled as a graphics fallback.

## 6. Build and start the container

From the configuration/wsl2_ubuntu20.04_foxy directory, run:
```bash id="k8b1rd"
bash dev.sh config
bash dev.sh build
bash dev.sh up
bash dev.sh check
```

These commands validate the Docker Compose configuration, build the Docker image with the required system and ROS 2 dependencies, start the container, and check the ROS 2 environment.


## 7. Build the workspace

Install dependencies and build the ROS 2 workspace:

```bash
bash dev.sh deps
bash dev.sh build-workspace
```

## 8. Run the LIMO simulation

Start the simulation:

```bash
bash dev.sh shell
ros2 launch limo_car ackermann_gazebo.launch.py
```

In a **second Ubuntu terminal**, start keyboard teleoperation:

```bash
bash ~/robotic_lab/limo/configuration/wsl2_ubuntu20.04_foxy/dev.sh shell
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Use the keys displayed in the terminal to control the robot. Keep this terminal focused while driving.

Gazebo and RViz2 require graphical support through WSLg.



### Run the physical robot

To run the physical robot instead of the Gazebo simulation, use the hardware-specific configuration and launch file. Building the workspace alone does not establish communication with the robot. The `limo_base` package also provides a driver for the physical LIMO:

```bash
ros2 launch limo_base limo_base.launch.py
```

This command requires access to the physical hardware and a suitable device configuration. A successful workspace build does not, by itself, establish communication with the robot. Do not assume that the physical robot will be reachable through the default Docker network configuration.

## 9. Everyday use

Run commands from the repository root in the Ubuntu terminal.

| Command | Purpose |
|---|---|
| `bash configuration/wsl2_ubuntu20.04_foxy/dev.sh up` | Start the container |
| `bash configuration/wsl2_ubuntu20.04_foxy/dev.sh shell` | Open a shell in the container |
| `bash configuration/wsl2_ubuntu20.04_foxy/dev.sh build-workspace` | Rebuild the ROS 2 workspace after source changes |
| `bash configuration/wsl2_ubuntu20.04_foxy/dev.sh deps` | Install dependencies after changing package manifests |
| `bash configuration/wsl2_ubuntu20.04_foxy/dev.sh build` | Rebuild the Docker image after configuration or Dockerfile changes |
| `bash configuration/wsl2_ubuntu20.04_foxy/dev.sh check` | Check the environment |
| `bash configuration/wsl2_ubuntu20.04_foxy/dev.sh gpu` | Check GPU access |
| `bash configuration/wsl2_ubuntu20.04_foxy/dev.sh down` | Stop and remove the containers |

The Git repository stores the source code, while the persistent Docker volume `foxy_workspace` stores workspace build artifacts.

## 10. Limitations

- ROS 2 Foxy is out of support. It is retained here to match the Ubuntu 20.04 environment used by the LIMO platform.
- The `astra_camera` package is currently excluded from the workspace build.
- The Docker image build, Gazebo/RViz2 graphical behavior, GPU support and communication with physical robots must be validated on the target computer.
- GPU access does not guarantee accelerated rendering in Gazebo or RViz2.
- USB and serial devices require additional configuration before they can be accessed from the container.
