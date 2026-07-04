# Lifecycle 
The LifecycleNode organizes the code into primary states (Primary States) and transition phases (Transitional States). This allows the node to follow a standardized lifecycle protocol during its activation within the pipeline, including a proper shutdown procedure in case of runtime issues. 


### Primary State:

- unconfigured
- inactive
- active
- shutdown


### Transitional State:

- configuring
- activating
- deactivating
- cleaningup
- shuttingdown



Below is a simple state diagram illustrating the Lifecycle states and their transitions:

![simple lifecycle](lifecycle1.png)





Now, a more detailed diagram:

![detailed lifecycle](lifecycle2.png)




# Changes in the node:

Within the already existing callbacks, the changes are minimal. The main difference is that, inside the node—after the constructor—three additional functions are added:


### on_configure:

Within this function, the node’s parameters, subscribers and publishers are declared.


### on_activate:

Within this function, the node’s main callback is called.

### on_shutdown

Within this function, the node enters a frozen state, where it no longer executes any logic, requiring the node to be restarted in such cases.



# Debug

To check the current state of a running node:
`ros2 lifecycle get /node_name`

To set a new state for the node:

`ros2 lifecycle set /node_name <state_number / state_name>`

If it is not possible to transition to the desired state from the current state, a corresponding warning is returned.



# Commands for compiling packages 

### For compiling both, use: 
```bash
    colcon build 
   ```

### For compiling individualy, use: 
```bash
    colcon build --packages-select path_planning
   ```
```bash
    colcon build --packages-select control
   ```


## Running & Launching

### Path Planning launchs: 

```bash
    ros2 run path_planning path_node_lifecycle.py
   ```

```bash
    ros2 launch path_planning path_lifecycle.launch.py
    ros2 run path_planning path_node.py
   ```

```bash
    ros2 launch path_planning path_planning.launch.py
   ```

### Control launchs: 

```bash
    ros2 run control control_node_lifecycle.py
   ```

```bash
    ros2 launch control control_lifecycle.launch.py
    ros2 run control control_node.py
   ```

```bash
    ros2 launch control control.launch.py
   ```
