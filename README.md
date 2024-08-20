# LidarMerger Node for ROS2

## Overview

The `LidarMerger` node is designed to merge scan data from two 2D LiDAR sensors (front and rear) into a single cohesive scan message in ROS2. This package is tested on ROS2 humble.

## Features

- Subscribes to two LiDAR scan topics: `front_scan` and `rear_scan`.
- Merges the scan data from both LiDAR sensors.
- Publishes a unified scan message on the `scan` topic.
- Allows customization through various parameters.

## Parameters

This node supports several parameters to adapt to different LiDAR models and installation configurations:

- `scan_data_length` (default: 6120): The length of the scan data array returned by the LiDAR.
- `front_lidar_offset` (default: 1.0/4.0 * M_PI): The heading offset of the front LiDAR in radians.
- `rear_lidar_offset` (default: 3.0/4.0 * M_PI): The heading offset of the rear LiDAR in radians.
- `lidar_installed_reversely` (default: true): Indicates if the rear LiDAR is installed in reverse.
- `lidar_scan_data_direction` (default: true): Direction of the scan data array from LiDAR. True is clockwise(top-viewed).
- `x_diff_from_rear_lidar_to_front_lidar` (default: 0.49): The distance between the rear and front LiDAR in the x-axis.
- `y_diff_from_rear_lidar_to_front_lidar` (default: -0.73): The distance between the rear and front LiDAR in the y-axis.

The default values are set for the [SLAMTEC LPX-T1] LiDAR sensors when they are installed in reverse on a robot facing a heading of 315 degrees and a heading of 135 degrees, respectively.

## Topics

- **Subscriptions:**
  - `front_scan` (*sensor_msgs/msg/LaserScan*): Receives scan data from the front LiDAR.
  - `rear_scan` (*sensor_msgs/msg/LaserScan*): Receives scan data from the rear LiDAR.

- **Publications:**
  - `scan` (*sensor_msgs/msg/LaserScan*): Publishes the merged scan data from both LiDAR sensors.

## Installation

1. Ensure you have ROS2 installed on your system.
2. Clone this repository into your ROS2 workspace.
3. Compile the workspace:
   ```sh
   colcon build
   ```

## Usage

To run the `LidarMerger` node, use the following command after sourcing your ROS2 environment:

```sh
ros2 run <your_package_name> lidar_merger
```

## Example

Here is a basic launch file example to run the `LidarMerger` node:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='your_package_name',
            executable='lidar_merger',
            name='lidar_merger',
            parameters=[
                {'scan_data_length': 6120},
                {'front_lidar_offset': 0.785},
                {'rear_lidar_offset': 2.356},
                {'lidar_installed_reversely': True},
                {'lidar_scan_data_direction': True},
                {'x_diff_from_rear_lidar_to_front_lidar': 0.49},
                {'y_diff_from_rear_lidar_to_front_lidar': -0.73}
            ],
            output='screen'
        ),
    ])
```


## License

This project is licensed under the MIT License. See the LICENSE file for more details.

## Contributing

Contributions are welcome! Please submit a pull request or create an issue to discuss improvements or fixes.