# Robot Controller - ROS2 Package (Jazzy)

This is a ROS2 Python package that implements path following and collision avoidance for TurtleBot3 robots.

## Package Structure

- `robot_controller/` - Python package directory
  - `robot_controller.py` - Main controller node
  - `path_publisher.py` - Path publishing node
  - `robot_utils.py` - Utility functions for transforms and odometry
  - `__init__.py` - Package initialization
- `launch/` - ROS2 launch files (Python format)
  - `epd3.launch.py` - Main launch file
  - `challenge0.launch.py` - Challenge scenario launch file
  - `path_publisher.launch.py` - Path publisher launch file
  - `turtlebot3_sim.launch.py` - Gazebo simulation setup
- `config/` - Configuration files
  - `path.yaml` - Default path configuration
  - `challenge0_path.yaml` - Challenge 0 path configuration
  - `epd3.rviz` - RViz configuration
- `setup.py` - Python package setup
- `package.xml` - ROS2 package manifest

## Building the Package

```bash
# Source your ROS2 environment (Jazzy)
source /opt/ros/jazzy/setup.bash

# Navigate to your workspace root
cd ~/robot_work/src

# Build the package
colcon build --packages-select robot_controller

# Source the build output
source ../install/setup.bash
```

## Running the Package

### Launch the main controller with default path:
```bash
ros2 launch robot_controller epd3.launch.py
```

### Launch with challenge path:
```bash
ros2 launch robot_controller challenge0.launch.py
```

### Run individual nodes:

#### Path Publisher:
```bash
ros2 run robot_controller path_publisher
```

#### Robot Controller:
```bash
ros2 run robot_controller robot_controller
```

## Configuration

The robot controller can be configured via ROS parameters:

- `robot_vel_topic` (str): Topic name for velocity commands (default: `cmd_vel`)
- `robot_scan_topic` (str): Topic name for laser scans (default: `scan`)
- `max_lin_vel` (float): Maximum linear velocity in m/s (default: `0.2`)
- `max_ang_vel` (float): Maximum angular velocity in rad/s (default: `0.4`)
- `control_rate` (Hz): Control loop frequency (default: `10`)
- `path_file` (str): Name of path configuration file without extension (default: `path`)
- `path_frame` (str): Coordinate frame for the path (default: `odom`)

These can be set in launch files or via command line:
```bash
ros2 run robot_controller robot_controller --ros-args -p max_lin_vel:=0.5
```

## ROS1 to ROS2 Migration Notes

Key changes from ROS1:
- **Initialization**: Changed from `rospy.init_node()` to `rclpy.init()` and node class inheritance
- **Logging**: Changed from `rospy.loginfo()` to `self.get_logger().info()`
- **Publishers/Subscribers**: Changed from `rospy.Publisher/Subscriber` to `self.create_publisher/subscription`
- **Parameters**: Changed from `rospy.get_param()` to `self.declare_parameter()` and `get_parameter()`
- **TF**: Changed from `tf.TransformListener` to `tf2_ros.Buffer` and `tf2_ros.TransformListener`
- **Control Loop**: Changed from `rospy.Rate()` to `self.create_timer()`
- **Timestamps**: Updated to use `node.get_clock().now().to_msg()`
- **Launch Files**: Changed from XML format to Python launch files
- **Method names**: Changed to follow Python naming conventions (snake_case)

## Dependencies

- rclpy
- geometry_msgs
- nav_msgs
- sensor_msgs
- tf2_ros
- tf2_py

These are specified in `package.xml` and will be installed via `colcon build`.

## TODO Items

The following methods need to be implemented to make the controller functional:

1. `goal_reached()` - Check if the final goal has been reached
2. `get_sub_goal()` - Extract the next intermediate goal from the path
3. `check_collision()` - Detect potential collisions using laser scans
4. `collision_avoidance()` - Implement collision avoidance behavior

These are marked with TODO comments in the code for easy identification.

## Troubleshooting

If you encounter issues:

1. Ensure all dependencies are installed:
   ```bash
   rosdep install --from-paths src --ignore-src -r -y
   ```

2. Check that the package is properly sourced:
   ```bash
   echo $AMENT_PREFIX_PATH
   ```

3. Verify launch file syntax:
   ```bash
   ros2 launch robot_controller epd3.launch.py --show-args
   ```

4. Check node status:
   ```bash
   ros2 node list
   ros2 node info /robot_controller
   ```

## License

TODO
