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

Both subsystems interface through ROS 2 topics using consistent timestamping and coordinate frames, enabling integration with upstream modules such as mapping and perception, and downstream modules for actuation.


Comando de run e launch dos pacotes

```bash
    ros2 run ros2_path_planning path_node.py
   ```

```bash
    ros2 launch ros2_path_planning path_planning.launch.py
   ```

```bash
    ros2 run ros2_control control_node.py
   ```

```bash
    ros2 launch ros2_control control.launch.py
   ```
