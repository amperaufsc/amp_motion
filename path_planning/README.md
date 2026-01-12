# Path Planning

## Overview 
This package is responsible for generating a trajectory for the vehicle in accordance with Formula Student Germany (FSG) regulations. Its objective is to ensure a safe and efficient path that completes each mission in the shortest possible time. By integrating odometry and track data, the code calculates and provides optimal waypoints for the vehicle to navigate through the various dynamic events.

Each event requires a distinct approach to trajectory planning. Before detailing these strategies, a brief summary of each event held at FSG is presented below.

### Skidpad
The Skidpad event is designed to evaluate the vehicle’s lateral dynamics by making it follow a predefined figure-eight path composed of two constant-radius circles. The car must complete multiple laps around the circles, switching direction midway. The focus is on testing the lateral acceleration capabilities and the consistency of the vehicle’s path-following behavior. Trajectory planning for this event emphasizes maintaining a stable circular path and minimizing lateral slip, while ensuring smooth transitions during the direction change.

### Trackdrive and Autocross
These events test the vehicle's ability to navigate a full track with varying turns and straights, without any prior mapping of the course (in the case of Autocross). The goal is to complete the course in the shortest time possible while maintaining control and avoiding cones. Path planning must focus on identifying and optimizing the racing line, handling tight corners effectively, and maintaining speed through smooth curvature transitions. Autocross also requires robust perception and real-time decision-making due to the absence of a preloaded map

### Acceleration and Brake-test
The Acceleration event assesses the vehicle’s capability to reach maximum speed over a straight, flat course within a fixed distance (typically 75 meters). Precise trajectory planning is minimal here; instead, focus lies on launch control, traction optimization, and drivetrain efficiency.

The Brake-test, often conducted at the end of the Acceleration run, evaluates the vehicle's braking system by requiring it to come to a complete stop within a designated area. While path planning is relatively straightforward, accuracy in deceleration control and stopping distance calculation is critical to avoid disqualification.

### Bayesian Inference
This module applies Bayesian inference to estimate the most probable vehicle trajectories under uncertainty. Inspired by AMZ Racing’s approach, it leverages cone detections from the perception system to infer a probabilistic representation of the track layout. To generate a reliable centerline, the module first performs a Delaunay triangulation over the set of detected cones. From the resulting mesh of triangles, it then extracts the midpoints of edges that connect left and right cones—these midpoints serve as the estimated centerline waypoints. Each of these points is part of a tree of trajectory hypotheses, where branches represent different feasible sequences of waypoints that the car could follow.

To evaluate and choose the best path, the system assigns a weight (or cost) to each waypoint, reflecting how likely it is to belong to the true centerline. These weights are calculated using a combination of geometric and semantic cues: the standard deviation of the track width, which penalizes inconsistent paths; the color classification of cones, which assumes blue cones are on the left and yellow on the right; and the maximum allowed heading angle change between consecutive waypoints, to promote smooth steering and avoid sharp turns.

The resulting tree is explored using a variant of beam search, which efficiently narrows down the most promising hypotheses by retaining only the top-k candidates at each step, based on their cumulative weight. This method balances exploration (considering alternative paths) and efficiency, avoiding the combinatorial explosion of full-tree search while still capturing multiple plausible centerlines.

### Speed Profile
The speed profile defines the vehicle's speed along the trajectory, balancing performance and safety. The forward pass calculates the path and speed from the start to the endpoint. The backward pass ensures that the vehicle will come to a complete stop at the final waypoint, adjusting the speed profile to account for smooth deceleration. Together, these steps guarantee that the vehicle follows an optimal trajectory and safely stops at the end of the path.



