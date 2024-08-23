# ros2_lidar_merge

This ros2 package merges two LaserScan messages from two 2D LiDAR into a single LaserScan message.

The package is tested on ros2 humble.

## Requisites

You need to have two LiDARs that are installed in same z direction (Both upward or both downward).


## Installation
You can clone this repository into your ROS2 workspace:
```bash
cd /path/to/workspace/src
git clone https://github.com/lineworld-lab/ros2_lidar_merger.git
mv ros2_lidar_merger lidar_merger
```

Build the package using colcon:

```bash
cd /path/to/workspace
colcon build
```

## Usage

To run the ros2_lidar_merger node, use the following command:

```bash
ros2 run lidar_merger lidar_merger
```

Make sure to have the necessary permissions to access the lidar sensors and configure the node accordingly.

## Parameters

The lidar merger node has the following parameters:

- `first_lidar_yaw`: The yaw angle of the first LiDAR relative to the robot's heading.
- `second_lidar_yaw`: The yaw angle of the second LiDAR relative to the robot's heading.
- `x_diff`: The distance between the second LiDAR and the first LiDAR in the coordinate that +y axis is the heading of the robot.
- `y_diff`: The distance between the second LiDAR and the first LiDAR in the coordinate that +y axis is the heading of the robot.


## Contributing

Contributions are welcome! If you find any issues or have suggestions for improvements, please open an issue or submit a pull request.

## License

This project is licensed under the [MIT License](LICENSE).
