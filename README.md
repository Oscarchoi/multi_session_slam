# multi_session_slam

This repository provides a ROS 2 package that supports multi-session SLAM, allowing for independent mapping sessions that can be started and stopped dynamically. The package is built upon the principles of 3D SLAM and takes inspiration from the lidarslam_ros2 library, which serves as a reference for the fundamental SLAM implementation. Details regarding licensing and attribution are provided in the LICENSE file.

## Features

- Multi-session SLAM capability
- Supports starting and stopping mapping sessions dynamically
- Built on ROS 2 for seamless integration with modern robotic applications

## Installation

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/Oscarchoi/multi_session_slam.git
colcon build
```

## Usage

### Start multi_session_slam & pointcloud_converter

```bash
ros2 launch pointcloud_to_geotiff pointcloud_converter.launch.py
```

### Start a Session

To begin a new SLAM session, use the following ROS 2 service call:

```bash
ros2 service call /session_start test_msgs/srv/BasicTypes "{string_value: 'Hello'}"
```

### End a Session

To conclude an active SLAM session, use:

```bash
ros2 service call /session_end test_msgs/srv/BasicTypes "{string_value: 'Hello'}"
```

Ensure that your ROS 2 environment is properly sourced before executing these commands.

## Dependencies

- [ndt_omp_ros2](https://github.com/rsasaki0109/ndt_omp_ros2)
