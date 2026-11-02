# Recording

ROS 2 package for recording robotics data using `rosbag2`.

## Overview

The `recording` package provides a generic interface for recording data published on ROS 2 topics.

It can be used with different robots, simulations, and sensors. Recorded data is stored using `rosbag2`, preserving the original ROS message types.

Examples of recordable data include:

- Odometry
- IMU measurements
- LiDAR scans
- Camera and depth images
- Point clouds
- TF transformations
- Other ROS 2 messages

## Requirements

- Ubuntu 20.04 with ROS 2 Foxy, or another compatible ROS 2 installation
- A ROS 2 workspace built with `colcon`
- `rosbag2` installed for the selected ROS 2 distribution
- Tkinter for the graphical interface

On Ubuntu, Tkinter can be installed with:

```bash
sudo apt install python3-tk
```

## Dependencies

The package uses the following ROS 2 components:

- `rclpy`
- `std_srvs`
- `ament_index_python`
- `rosbag2`

These dependencies must be available in the selected ROS 2 installation.

## Architecture

The package consists of a recorder node, two command-line service clients, and a graphical interface.

```text
                     +---------------------+
                     |       recorder      |
                     |     (ROS 2 node)    |
                     +----------+----------+
                                |
                       starts/stops process
                                |
                                v
                     +---------------------+
                     |   ros2 bag record   |
                     |    (subprocess)     |
                     +----------+----------+
                                |
                                v
                     +---------------------+
                     |       ROS bag       |
                     | metadata + data     |
                     +---------------------+

              ROS 2 services provided by recorder
                   /start_recording
                   /stop_recording
                           ^
                           |
             +-------------+-------------+
             |                           |
      +------+-------+           +-------+-------+
      | CLI clients  |           | Tkinter GUI   |
      | start/stop   |           | recording_    |
      | recording    |           | widget        |
      +--------------+           +---------------+
```

The `recorder` node provides two ROS 2 services:

- `/start_recording`
- `/stop_recording`

The `start_recording` and `stop_recording` command-line tools and the `recording_widget` graphical interface call these services to control the recording process.

The recorder starts and stops the external `ros2 bag record` process.

## Installation

The package can be included in a ROS 2 workspace built with `colcon`.

### Source ROS 2

Source the appropriate ROS 2 distribution.

For Ubuntu 20.04 with ROS 2 Foxy:

```bash
source /opt/ros/foxy/setup.bash
```

### Build the package

From the workspace root, build the package and source the workspace:

```bash
cd <workspace>
colcon build --packages-select recording --symlink-install
source install/setup.bash
```

Replace `<workspace>` with the path to your ROS 2 workspace.

## Usage

### 1. Start the recorder

In a terminal, start the recorder node and specify the topics to record:

```bash
ros2 run recording recorder --ros-args \
  -p topics:="['/odom', '/scan', '/imu']"
```

Replace the example topics with the names published by your robot or simulation.

The recorder waits for a request to start recording. If no topics are configured, recording requests will fail.

By default, recorded data is stored in the `data/` directory at the workspace root. A custom output directory can be configured using the `output_dir` parameter:

```bash
ros2 run recording recorder --ros-args \
  -p topics:="['/odom', '/scan']" \
  -p output_dir:=/path/to/data
```

### 2. Start recording

Recording can be controlled in either of two ways.

#### Graphical interface

In another terminal, run:

```bash
ros2 run recording recording_widget
```

Click **Start recording** to begin recording and **Stop recording** to finish. The interface displays the response returned by the recorder service.

#### Command-line tools

Alternatively, start recording with:

```bash
ros2 run recording start_recording
```

Stop recording with:

```bash
ros2 run recording stop_recording
```

The recorder must remain running while these commands or the graphical interface are in use.

### 3. Recorded data

Each recording is stored in a separate directory with a timestamp-based name, for example:

```text
<workspace>/
└── data/
    └── rosbag_YYYY-MM-DD_HH-MM-SS/
        ├── metadata.yaml
        └── rosbag_YYYY-MM-DD_HH-MM-SS_0.db3
```

The `.db3` file corresponds to the SQLite3 storage format used in this configuration. The exact storage files may vary with the selected `rosbag2` storage plugin.

When recording is stopped, the recorder sends an interrupt signal to `ros2 bag record` and waits for the process to finish so that the bag can be finalized.

### 4. Inspect recorded data

Use `ros2 bag info` to inspect a recording:

```bash
ros2 bag info <workspace>/data/rosbag_YYYY-MM-DD_HH-MM-SS
```

This provides information such as:

- Recording duration
- Storage format
- Number of messages
- Topic names
- ROS message types
- Message counts

The ROS bag is the primary recorded dataset. Its contents can later be converted to other formats, such as CSV, for analysis.

## Recording frequency

The package does not impose a sampling frequency or resample the data.

Each topic is recorded according to the messages published and received during the recording interval. The number of messages therefore depends on the publisher frequency, the recording duration, and the availability of messages.

For example, a particular simulation might publish:

```text
/odom             -> approximately 100 Hz
/imu              -> approximately 100 Hz
/scan             -> approximately 8 Hz
```

These frequencies are illustrative and depend on the robot, sensor, or simulation configuration.

This approach allows heterogeneous sensors to be recorded without imposing a common sampling period.

## Package structure

```text
recording/
├── recording/
│   ├── __init__.py
│   ├── recorder.py
│   ├── start_recording.py
│   ├── stop_recording.py
│   └── recording_widget.py
├── resource/
│   └── recording
├── package.xml
├── setup.py
├── setup.cfg
└── README.md
```

### Nodes and tools

#### `recorder`

Main ROS 2 node. It manages the recording lifecycle, exposes the start and stop services, and launches the `ros2 bag record` process.

#### `start_recording`

Command-line tool that requests the start of a recording through the `/start_recording` service.

#### `stop_recording`

Command-line tool that requests the end of a recording through the `/stop_recording` service.

#### `recording_widget`

Tkinter graphical interface for controlling recording. It calls the same ROS 2 services as the command-line clients.

## Design principles

- **Generic** — no robot-specific topics or message types are hard-coded.
- **ROS-native** — data is stored using `rosbag2`.
- **Non-invasive** — the recorder does not impose a sampling frequency or resample the data.
- **Reusable** — the package can be used with different robots, simulations, and sensors.
- **Extensible** — additional recording and data-processing tools can be added later.

## Future developments

Possible extensions include:

- Predefined recording profiles
- Topic throttling
- ROS bag conversion tools
- Automated dataset organization
- Data visualization and analysis tools
- A ROS 2 launch file for configurable startup

## License

Apache-2.0