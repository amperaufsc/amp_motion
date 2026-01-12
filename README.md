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

| Module           | Direction | Topic                     | Message Type                 | Notes |
|------------------|-----------|---------------------------|-------------------------------|-------|
| Path Planning     | Sub       | `/map/track`              | `sensor_msgs/PointCloud2`     | Track / cones / environment |
| Path Planning     | Sub       | `/mission/go_signal`      | `std_msgs/Bool`               | Trigger for planning |
| Path Planning     | Pub       | `/motion/ref_trajectory`  | `nav_msgs/Path` or custom     | Reference path |
| Vehicle Control   | Sub       | `/motion/ref_trajectory`  | `nav_msgs/Path` or custom     | Input trajectory |
| Vehicle Control   | Pub       | `/vehicle/cmd`            | `geometry_msgs/Twist`         | Command output |

> Note: message types may vary depending on final integration.

---

# Coordinate Frames

Common frames in use:

- `map` – global SLAM / mapping frame
- `odom` – local odometry frame for short-term drift-free motion
- `base_link` – vehicle base frame (control reference)

Frame transforms are managed through TF2.

---

# Dependencies

Core dependencies (minimum):

- ROS 2 Humble (or newer)
- `rclcpp` / `rclpy`
- `nav_msgs`, `geometry_msgs`, `sensor_msgs`
- `tf2` + `tf2_ros`
- `colcon` (build system)
- Custom interfaces (if used)

---

# Repository Structure (Example)

├── ros2_path_planning/
│ ├── src/
│ ├── launch/
│ └── package.xml
├── ros2_control/
│ ├── src/
│ ├── launch/
│ └── package.xml
└── README.md

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
