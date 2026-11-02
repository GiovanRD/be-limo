# LIMO Robot Setup: Ubuntu 20.04 and ROS 2 Foxy

This guide configures the **physical AgileX LIMO robot** running Ubuntu 20.04 and ROS 2 Foxy. It supports `arm64` and `amd64` systems.

The setup covers the LIMO chassis, native odometry, IMU, YDLidar, the optional Astra/Orbbec camera, and development tools for navigation and SLAM. Navigation and SLAM are not launched automatically.

## 1. Connection between the Limo and the computer

Before configuring the robot, connect it and your computer to the appropriate network.

### 1.1 Connect the LIMO to Wi-Fi

Connect the LIMO to the **ENSEMSAMI** Wi-Fi network using the following credentials:

- **Identifier:** `ENSEMSAMI`
- **Password:** `QWERTYUI`

### 1.2 Connect your computer to the network

If your computer is connected to the ENSEM wired network, no additional configuration is required.

If you are using a laptop connected over Wi-Fi, connect it to **EDUROAM** to communicate with the LIMO.

The ENSEM SAMI and EDUROAM networks are bridged, allowing devices connected to either network to communicate without additional configuration.

### 1.3 Connect to the LIMO over SSH

The standard Linux account on the LIMO is:

- **Username:** `jetson`
- **Password:** `jetson`

From a terminal on your computer, connect using the IP address obtained in Section 1:

```bash
ssh jetson@100.75.155.<LIMO_NUMBER>
```

Replace `<LIMO_NUMBER>` with the LIMO's actual number.

Enter the robot account password when prompted. Obtain the password through the designated internal channel; do not commit it to GitLab or store it in scripts.

After connecting, verify that you are working on the robot:

```bash
whoami
hostname
```

The expected output is:

```text
jetson
```

## 2. Check the LIMO operating system and architecture

Run the following commands in the LIMO terminal:

```bash
cat /etc/os-release
dpkg --print-architecture
```

The supported configuration is Ubuntu 20.04 with `arm64`.

## 3. Clone the required repository directories

Clone the repository on the LIMO:

```bash
git clone https://gitlab.univ-lorraine.fr/coll/l-inp/ensem/robotic_lab/limo.git \
  "$HOME/robotic_lab/limo"

cd "$HOME/robotic_lab/limo"

git sparse-checkout init --cone
git sparse-checkout set \
  configuration/limo_ubuntu20.04_foxy \
  ros2_foxy/limo_ros2 \
  ros2_foxy/limo_start \
  ros2_foxy/ros2_astra_camera \
  ros2_foxy/ydlidar_ros2_driver \
  ros2_foxy/YDLidar-SDK

cd configuration/limo_ubuntu20.04_foxy
```

The sparse checkout limits the working tree to the selected directories.


## 4. Install system dependencies

From the configuration directory, run:

```bash
bash install-deps.sh
```

The installer installs the packages listed in `apt-packages.txt`, configures the ROS 2 apt repository, and resolves the declared dependencies of the selected robot packages using `rosdep`.

The ROS 2 workspace is located at `~/ros2_ws`, where the robot packages are built and installed locally.

The installation includes:

- ROS 2 Foxy and development tools, if not already there.
- LIMO chassis, TF, and sensor dependencies.
- YDLidar dependencies and tools for compiling the local C++ SDK.
- Astra/Orbbec camera dependencies, including OpenCV, Eigen, USB, and image-processing libraries.
- Navigation2, SLAM Toolbox, Cartographer, teleoperation, and optional RViz tools.
- Python libraries, including NumPy, SciPy, OpenCV, and YAML support.

Internet access and `sudo` permissions are required.

## 5. Configure hardware permissions

Run:

```bash
bash setup-devices.sh
```

The script adds the current user to the `dialout` and `video` groups, installs the supplied udev rules, and reloads the rules.

Log out and log back in for group membership changes to take effect. For an SSH session, disconnect and reconnect.


### Chassis serial-port permissions

Check the current user and device permissions:

```bash
id
ls -l /dev/ttyTHS1
ls -l /dev/ydlidar
lsusb
```

The expected chassis serial port is `/dev/ttyTHS1`. The LiDAR is expected at `/dev/ydlidar`, provided that the corresponding USB adapter matches one of the configured udev rules.

The expected output of `ls -l /dev/ttyTHS1` should show `root` as the owner, `dialout` as the group, and read/write permissions for the owner and group, for example:

```text
crw-rw---- 1 root dialout ... /dev/ttyTHS1
```

If the permissions are incorrect, run:

```bash
sudo chgrp dialout /dev/ttyTHS1
sudo chmod 660 /dev/ttyTHS1
```

These commands correct the permissions for the current device node. If the problem reappears after rebooting, check the udev configuration in `99-limo-hardware.rules`.

## 6. Prepare the ROS workspace

The robot uses the workspace `~/ros2_ws`. The repository remains in GitLab; the workspace contains symbolic links to the source packages rather than independent copies.

Create the workspace and its links:

```bash
REPO_ROOT="$HOME/robotic_lab/limo"

mkdir -p "$HOME/ros2_ws/src"

python3 "$REPO_ROOT/configuration/limo_ubuntu20.04_foxy/prepare-workspace.py" \
  "$REPO_ROOT/ros2_foxy" \
  "$HOME/ros2_ws/src"
```

The expected layout is:

```text
~/ros2_ws/
└── src/
    ├── limo_ros2 -> .../ros2_foxy/limo_ros2
    ├── limo_start -> .../ros2_foxy/limo_start
    ├── ydlidar_ros2_driver -> .../ros2_foxy/ydlidar_ros2_driver
    └── astra_camera -> .../ros2_foxy/ros2_astra_camera
```

## 7. Build the robot workspace

From the configuration directory:

```bash
cd "$HOME/robotic_lab/limo/configuration/limo_ubuntu20.04_foxy"
```

Build the SDK and ROS packages:

```bash
bash build-robot.sh
```

By default, the script checks for the Orbbec OpenNI2 runtime and downloads and validates it if necessary.

To build without the camera:

```bash
bash build-robot.sh --without-camera
```

On a low-memory system, reduce the number of build workers:

```bash
BUILD_JOBS=1 bash build-robot.sh
```

The build script:

1. Checks that ROS 2 Foxy and the expected source packages are present.
2. Downloads and validates the Orbbec OpenNI2 runtime when the camera is enabled and the required files are missing.
3. Builds the standalone YDLidar C++ SDK under `~/ros2_ws/build/ydlidar_sdk` and installs it under `~/ros2_ws/install/ydlidar_sdk`.
4. Builds the selected ROS packages using `colcon build --symlink-install`.
5. Configures the shell environment and aliases after a successful build.

The build selects the physical robot packages explicitly; simulator packages are not included in the build command. Source files are not copied into the workspace or modified by the build script, except that the OpenNI2 runtime files are downloaded into the camera source package when needed.

## 8. Shell configuration and aliases

After a successful build, `build-robot.sh` runs `setup-shell.sh` to configure `~/.bashrc`. This adds commands to load ROS 2 Foxy, the workspace environment, and the LIMO aliases.

Open a new terminal to apply the configuration automatically. To apply it in the current terminal, run:

```bash
source ~/.bashrc
```

The following aliases are available:

| Alias | Purpose |
|---|---|
| `wsp` | Navigate to the ROS 2 workspace (`~/ros2_ws`). |
| `cb_limo` | Build the YDLidar SDK and selected ROS packages. |
| `start` | Launch the physical LIMO. |

These aliases are defined in `aliases/limo.bash`. To update the shell configuration manually, run the following command from the configuration directory:

```bash
bash setup-shell.sh
```

After updating the configuration, run `source ~/.bashrc` or open a new terminal to apply the changes.

## 9. Launch the physical robot

Navigate to the ROS 2 workspace:

```bash
cd "$HOME/ros2_ws"
```

Launch the physical robot:

```bash
ros2 launch limo_start limo_real.launch.py
```

The launch profile is intended to start the chassis driver, native odometry, IMU transform, and enabled sensors. RViz is disabled by default.

If the workspace was built with `--without-camera`, the camera should remain disabled according to the launch configuration :

```bash
ros2 launch limo_start limo_real.launch.py use_camera:=false
```

The chassis serial port is configured in `limo_start/config/limo_real.yaml`. The expected default is `/dev/ttyTHS1`. Check the launch parameters and driver conventions before changing the port.

Stop the launched nodes with `Ctrl+C`.

## 10. Validate ROS topics, sensors, and teleoperation

Keep the terminal running the LIMO launch active. Use a second terminal for the following checks and commands.

### 10.1. Inspect ROS topics and sensor data

List the active topics:

```bash
ros2 topic list
```

Check that the main sensor topics are publishing data:

```bash
ros2 topic hz /odom
ros2 topic hz /scan
ros2 topic hz /limo/imu
```

If the camera is enabled, check its image topics. In the default configuration, these are typically:

- `/rgb/image_raw`: RGB images
- `/depth_camera/depth/image_raw`: depth images

```bash
ros2 topic hz /rgb/image_raw
ros2 topic hz /depth_camera/depth/image_raw
```

Stop each frequency check with `Ctrl+C` before starting the next one. Topic names and publication rates may vary with the configuration.

### 10.2. Teleoperate the LIMO

Make sure the robot is on a clear, level surface with enough room to move safely, or securely raised so that its wheels are off the ground.

Start keyboard teleoperation in a terminal:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Follow the instructions displayed in the terminal to send velocity commands. Start with small commands and verify that the robot responds as expected.

In another terminal, monitor the odometry:

```bash
ros2 topic echo /odom
```

To check whether velocity commands are being published, use:

```bash
ros2 topic hz /cmd_vel
```

Stop teleoperation with `Ctrl+C`.

### 10.3. ROS 2 network communication

For communication between ROS 2 computers on the same network, ensure that they use the same `ROS_DOMAIN_ID`. The default is `0`, unless it has been changed in the shell configuration.

## 11. Troubleshooting

### 11.1. LiDAR troubleshooting

The LiDAR connects through a USB serial interface. If `/scan` is missing or no valid measurements are received, check the device and driver.


If `/dev/ydlidar` is missing, inspect the available serial devices:

```bash
ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
```

If a serial device exists but `/dev/ydlidar` does not, check the udev rule in `99-limo-hardware.rules`. Verify that its USB vendor and product IDs match the adapter reported by `lsusb`, then reapply the rule and reconnect the device if necessary.

**Check the driver and scan data:**

```bash
ros2 topic info /scan
ros2 topic hz /scan
```

If the topic is not publishing, inspect the launch terminal for driver errors and verify the serial port and baud rate. If necessary, inspect the scan messages:

```bash
ros2 topic echo /scan
```

Stop the command with `Ctrl+C`. USB detection alone does not guarantee that valid LiDAR measurements are being received.

### 11.2. Camera troubleshooting

The Astra/Orbbec camera requires the OpenNI2 runtime and the ROS 2 camera driver. If the camera does not start or publish images, use the following checks.

**Check the OpenNI2 runtime files:**

The build script downloads the runtime when required files are missing. Verify that the expected files are present for the system architecture:

```bash
ARCH="$(dpkg --print-architecture)"
if [ "$ARCH" = "amd64" ]; then
  CAMERA_ARCH=x64
else
  CAMERA_ARCH=arm64
fi

CAMERA_DIR="$HOME/robotic_lab/limo/ros2_foxy/ros2_astra_camera/astra_camera/openni2_redist/$CAMERA_ARCH"

ls -l "$CAMERA_DIR/libOpenNI2.so"
ls -l "$CAMERA_DIR/OpenNI.ini"
ls -l "$CAMERA_DIR/OpenNI2/Drivers/"
```

The driver directory should contain the appropriate Orbbec driver library, typically `liborbbec.so`.


**Test without the camera:**

If the camera is not required for the exercise, rebuild the workspace without it:

```bash
bash build-robot.sh --without-camera
```

This allows you to test the chassis and LiDAR independently of the camera driver.

## Configuration files

| File | Purpose |
|---|---|
| `install-deps.sh` | Install system dependencies and resolve ROS dependencies |
| `apt-packages.txt` | Explicit Ubuntu and ROS package list |
| `no-gazebo.pref` | Temporary apt preference excluding matching Gazebo packages |
| `setup-devices.sh` | Configure device permissions and udev |
| `99-limo-hardware.rules` | Hardware-specific udev rules |
| `prepare-workspace.py` | Validate sources and create workspace links or temporary manifests |
| `build-robot.sh` | Build the YDLidar SDK and selected ROS packages |
| `setup-shell.sh` | Configure the Bash environment |
| `aliases/limo.bash` | Define robot shortcuts |
| `launch-robot.sh` | Launch the physical robot using `limo_start` |