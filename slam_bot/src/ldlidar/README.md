# LiDAR Node

## Overview

This node interfaces with the FHL-LD19 LiDAR sensor to provide 2D scanning data for mapping and obstacle detection. It's a crucial component for the SLAM Bot's perception system.

## Hardware

- Sensor: FHL-LD19 LiDAR
- Interface: USB Serial

## Features

- Real-time 2D scanning
- Point cloud generation
- Dynamic obstacle detection
- Auto-calibration and filtering

## ROS2 Topics

### Published Topics

- `/scan` ([sensor_msgs/LaserScan](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/LaserScan.html))
  - 2D laser scan data
- `/pointcloud` ([sensor_msgs/PointCloud2](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/PointCloud2.html))
  - 3D point cloud representation of scan data

### Parameters

- `frame_id` (string, default: "laser_link")
  - TF frame for the LiDAR
- `serial_port` (string, default: "/dev/ttyUSB0")
  - Serial port for LiDAR communication
- `baud_rate` (int, default: 230400)
  - Serial communication speed

## Usage

1. Launch the LiDAR node:

```bash
ros2 launch ldlidar ldlidar.launch.py
```

2. View scan data:

```bash
# Raw laser scan data
ros2 topic echo /scan

# Point cloud visualization (requires RViz2)
ros2 run rviz2 rviz2 -d config/lidar_view.rviz
```

3. Run tests:

```bash
colcon test --packages-select ldlidar ldlidar_driver
```

## Implementation Details

### Data Processing Pipeline

1. Raw serial data acquisition
2. Point cloud generation
3. Filtering and outlier removal
4. Coordinate transformation
5. ROS2 message publication

### Error Handling

- Serial communication monitoring
- Data validation and integrity checks
- Automatic recovery procedures

## Dependencies

- ROS2 Jazzy
- [sensor_msgs](https://github.com/ros2/common_interfaces)
- [tf2](https://github.com/ros2/geometry2)
- [pcl_ros](https://github.com/ros-perception/perception_pcl)

## References

- [FHL-LD19 SDK Documentation](https://drive.google.com/drive/folders/1jDIx0p-0Cy9rYoSuTpTnV4MNIgaVVAof)
- [YouYeeToo Wiki](https://wiki.youyeetoo.com/en/Lidar/D300)
