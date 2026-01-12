# Path Planning and Vehicle Control
This repository contains the modules responsible for path planning and vehicle control. These components receive perception and mapping information and generate motion commands that drive the vehicle along the intended trajectory.

### The planning subsystem handles:
Path generation based on the mapped environment
Trajectory interpolation and smoothing
Waypoint sequencing and motion constraints

### The control subsystem is responsible for:
Tracking the reference trajectory
Computing steering, throttle, and braking commands
Ensuring stable and consistent motion execution

## System Data Flow (Conceptual)
Perception → Mapping → Path Planning → Vehicle Control → Actuation

Both subsystems interface through ROS 2 topics using consistent timestamping and coordinate frames, enabling integration with upstream modules such as mapping and perception, and downstream modules for actuation.


---

# ROS Interfaces

## Topics (Example)

self.subscription = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.subscription = self.create_subscription(Track, 'track', self.track_callback, 10)
        self.subscription = self.create_subscription(GoSignal, 'go', self.go_callback, 10)
        self.publisher_ = self.create_publisher(Path, 'path', 10)
        self.publisher_concatenated = self.create_publisher(Path, 'path_concatenated',10)
        self.publisher_pointcloud = self.create_publisher(PointCloud2, 'track_pointcloud',10)
        

| Module           | Direction | Topic                     | Message Type                 | Notes |
|------------------|-----------|---------------------------|-------------------------------|-------|
| Path Planning     | Sub       | `/odom`                   | `sensor_msgs/Odom`     | Odometry input |
| Path Planning     | Sub       | `/mission/go_signal`      | `std_msgs/Bool`               | Trigger for planning |
| Path Planning     | Sub       | `/track`      | `nav_msgs/Track`               | Track input |
| Path Planning     | Pub       | `/path`  | `nav_msgs/Path`      | Reference path |
| Path Planning     | Pub       | `/path_concatenated`  | `nav_msgs/Path`      | Reference path (Control input) |
| Path Planning     | Pub       | `/track_pointcloud`  | `nav_msgs/PointCloud2`      | Track for debugging |

> Topics and messages used in Path Planning package.

---

| Module           | Direction | Topic                     | Message Type                 | Notes |
|------------------|-----------|---------------------------|-------------------------------|-------|
| Path Planning     | Sub       | `/odom`                   | `sensor_msgs/Odom`     | Odometry input |
| Path Planning     | Sub       | `/mission/go_signal`      | `std_msgs/Bool`               | Trigger for planning |
| Path Planning     | Sub       | `/track`      | `nav_msgs/Track`               | Track input |
| Path Planning     | Pub       | `/path`  | `nav_msgs/Path`      | Reference path |
| Path Planning     | Pub       | `/path_concatenated`  | `nav_msgs/Path`      | Reference path (Control input) |
| Path Planning     | Pub       | `/track_pointcloud`  | `nav_msgs/PointCloud2`      | Track for debugging |

> Topics and messages used in Vehicle Control package.

---

# Coordinate Frames

Common frames in use:

- `map` – global SLAM / mapping frame
- `/fsds/map` – fsds frame 
- `base_link` – vehicle base frame (control reference)

Frame transforms are managed through TF2.

---

# Dependencies

Core dependencies (minimum):

- ROS 2 Humble (or newer)
- `rclcpp` / `rclpy`
- `nav_msgs`, `geometry_msgs`, `sensor_msgs`, `lifecycle_msgs`
- `tf2` + `tf2_ros`
- `colcon` (build system)

---


## Commands for compiling packages 

### For compiling both, use: 
```bash
    colcon build 
   ```

### For compiling individualy, use: 
```bash
    colcon build --packages-select ros2_path_planning
   ```
```bash
    colcon build --packages-select ros2_control
   ```


## Running & Launching

### Path Planning launchs: 

```bash
    ros2 run ros2_path_planning path_node.py
   ```

```bash
    ros2 launch ros2_path_planning path_planning.launch.py
   ```

### Control launchs: 

```bash
    ros2 run ros2_control control_node.py
   ```

```bash
    ros2 launch ros2_control control.launch.py
   ```
