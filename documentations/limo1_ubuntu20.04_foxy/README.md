# AgileX LIMO 1 — Ubuntu 20.04 and ROS 2 Foxy Installation

## 1. Overview

This procedure installs Ubuntu 20.04.6 LTS on the Jetson Nano embedded in an AgileX LIMO 1, using a preconfigured image from Qengineering. ROS 2 Foxy is then installed through APT.

**Configuration used**
- Robot: AgileX LIMO 1
- Embedded computer: NVIDIA Jetson Nano
- Host computer: Windows
- microSD card port
- Operating system: Ubuntu 20.04.6 LTS (Focal Fossa), ARM64
- ROS distribution: ROS 2 Foxy

## 2. Remove the microSD Card from the NVIDIA Jetson Nano

The microSD card is inserted into the NVIDIA Jetson Nano, located in the upper section of the LIMO. The card slot is near the front of the robot and not directly accessible from the outside.

1. Power off the limo and disconnect it from its power supply.
2. Unscrew the four screws securing the black enclosure that houses the electronic boards.
2. Carefully move the enclosure slightly to access the microSD card slot.
3. Push the microSD card inward to release it, then remove it.

**Caution:** Handle the enclosure and the microSD card carefully to avoid damaging the cables or electronic components.

## 3. Download the Ubuntu image

1. Open the Qengineering repository: https://github.com/Qengineering/Jetson-Nano-Ubuntu-20-image
2. Follow the repository instructions to obtain the `JetsonNanoUb20_3b.img.xz` image.
3. Save the downloaded image on the Windows computer.

This is a Jetson Nano image, not a Raspberry Pi operating system image. Raspberry Pi Imager is simply used as a convenient tool to flash it onto the microSD card.

## 4. Flash the microSD card with Raspberry Pi Imager

1. Insert the 64 GB microSD card into the Windows computer using a card reader.
2. Save the JetsonNanoUb20_3b.img.xz image file to the Windows computer, ensuring that the download is complete before proceeding.
2. Launch Raspberry Pi Imager. Install it first if necessary: https://www.raspberrypi.com/software/
3. Select **Choose Device** and choose the option corresponding to a generic device, if requested.
4. Select **Choose OS**, then **Use custom**.
5. Select `JetsonNanoUb20_3b.img.xz`.
6. Select **Choose Storage** and carefully select the 64 GB microSD card.
7. Start the write operation and wait for it to finish successfully.
8. Safely eject the card.


## 5. Boot the LIMO

1. Make sure that the LIMO is still Off.
2. Replace the flashed microSD card into the LIMO.
3. Power on the robot and allow Ubuntu to boot.
4. Connect the robot to the network.

If SSH is enabled and the robot is reachable, connect from Windows PowerShell:

```powershell
ssh jetson@<IP_address>
```

Replace <IP_address> with the robot's IP address, then reconnect using the ssh command above and verify the new host key fingerprint before accepting it.

If SSH reports that the remote host identification has changed after reinstalling the operating system, first verify that the IP address belongs to your robot. Then remove the obsolete host key:

```powershell
ssh-keygen -R <IP_address>
```

Reconnect and verify the new host key before accepting it.

## 6. Verify Ubuntu and the microSD card

Check the operating system and CPU architecture:

```bash
lsb_release -a
dpkg --print-architecture
```

Expected results:

- Ubuntu 20.04.6 LTS
- Codename: `focal`
- Architecture: `arm64`

Check the root filesystem:

```bash
df -h /
lsblk -o NAME,SIZE,FSTYPE,LABEL,MOUNTPOINT
```

On the tested LIMO, the root filesystem was `/dev/mmcblk1p1`, while `/dev/mmcblk1` represented the 64 GB microSD card. Device names can differ between systems, so identify the correct card before proceeding.

Initially, the root partition occupied only about 29 GB, leaving roughly 30 GB unused.

## 7. Expand the root partition

The following commands apply to the tested configuration, where the microSD card is `/dev/mmcblk1` and the root partition is `/dev/mmcblk1p1`. Verify these device names first.

### 7.1. Correct the backup GPT location

```bash
sudo sgdisk -e /dev/mmcblk1
sudo sgdisk -v /dev/mmcblk1
```

The verification command should report no problems.

### 7.2. Reboot

```bash
sudo reboot
```

Reconnect over SSH once the robot has restarted.

### 7.3. Extend the root partition

Launch the partition editor:

```bash
sudo parted /dev/mmcblk1
```

At the `parted` prompt, enter:

```text
resizepart 1 100%
```

When asked for the end position, enter `100%`. Confirm the operation if prompted, then exit:

```text
quit
```

### 7.4. Expand the ext4 filesystem

```bash
sudo resize2fs /dev/mmcblk1p1
```

Verify the result:

```bash
df -h /
```

On the tested 64 GB card, the resulting root filesystem was approximately 59 GB, with around 36 GB available.


## 8. Update APT package lists

Refresh the package lists:

```bash
sudo apt update
```

The command completed successfully using the preconfigured NVIDIA repositories.

**Do not automatically upgrade every available package.** The Jetson image contains NVIDIA-specific software, and a broad upgrade could affect driver compatibility.

## 9. Configure the ROS 2 repository

ROS 2 Foxy targets Ubuntu 20.04 and supports ARM64. It is now end-of-life, so this installation uses legacy packages and should be treated as a compatibility setup rather than a currently supported ROS distribution.

Install the required tools:

```bash
sudo apt install software-properties-common curl
```


Download the ROS repository key:

```bash
sudo curl -sSL \
  https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
```

Add the ROS 2 repository for Ubuntu Focal and ARM64:

```bash
echo "deb [arch=arm64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu focal main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list
```

Refresh the package lists:

```bash
sudo apt update
```

## 10. Install ROS 2 Foxy

Check that the package is available:

```bash
apt-cache policy ros-foxy-ros-base
```

A candidate version should be displayed. In the tested installation, the available version was:

```text
0.9.2-1focal.20230606.043331
```

Install the base ROS 2 distribution:

```bash
sudo apt install ros-foxy-ros-base
```

If APT proposes unexpected removals, especially of NVIDIA or CUDA packages, cancel the operation and investigate before proceeding.

## 11. Verify the ROS 2 installation

Load the ROS environment:

```bash
source /opt/ros/foxy/setup.bash
```

Check that the command-line interface works:

```bash
ros2 --help
```

List installed ROS packages:

```bash
ros2 pkg list | head -20
```

If `ros2 --help` displays the available ROS commands, the ROS 2 installation is functional.

To load ROS automatically in future Bash sessions, append the setup command to `~/.bashrc`:

```bash
echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Installation status

At the end of this procedure, Ubuntu 20.04.6 LTS was running successfully from the 64 GB microSD card, the root partition had been expanded, and ROS 2 Foxy was installed and responding to command-line queries. The LIMO-specific driver and peripheral tests remained to be performed.
