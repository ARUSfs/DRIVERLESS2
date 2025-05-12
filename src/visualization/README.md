# Visualization

The `visualization` node is part of the DRIVERLESS2 project and is responsible for displaying relevant ROS2 pipeline messages in RViz2.

## Usage
To run the `visualization` node, use the following command:
```bash
ros2 launch visualization visualization_launch.py
```

## Connections
### Subscribed Topics
- '/path_planning/triangulation' (type: `common_msgs/Triangulation`): Receives delaunay's triangulation generated in the path planning process.
- '/trajectory_optimization/trajectory' (type: `common_msgs/Trajectory`): Receives an optimized trajectory from the optimization node.
- '/arussim_interface/fixed_trajectory' (type: `common_msgs/Trajectory`): Receives a fixed trajectory from ARUSSim.
- '/path_planning/trajectory' (type: `common_msgs/Trajectory`): Receives a Delaunay-based trajectory.
- '/acc_planning/trajectory' (type: `common_msgs/Trajectory`): Receives a trajectory from acc_planning node.
- '/skidpad_planning/trajectory' (type: `common_msgs/Trajectory`): Receives a trajectory from skidpad_planning node.
- '/controller/pursuit_point' (type: `common_msgs/PointXY`): Receives the pursuit point from control node.
- '/path_planning/track_limits' (type: `common_msgs/TrackLimits`): Receives track limits generated in the path planning process .

### Published Topics
- '/visualization/triangulation' (type: `visualization_msgs/MarkerArray`): Publishes the triangulation visualization.
- '/visualization/trajectory_optimized' (type: `visualization_msgs/Marker`): Publishes the optimized trajectory visualization.
- '/visualization/arussim_fixed_trajectory' (type: `visualization_msgs/Marker`): Publishes the ARUSSim fixed trajectory visualization.
- '/visualization/trajectory' (type: `visualization_msgs/Marker`): Publishes general trajectory visualizations.
- '/visualization/pursuit_point' (type: `visualization_msgs/Marker`): Publishes the pursuit point visualization.
- '/visualization/track_limits' (type: `visualization_msgs/MarkerArray`): Publishes track limits visualizations.
- '/visualization/optimized_trajectory_3d' (type: `visualization_msgs/Marker`): Publishes 3D representations of the optimized trajectory (X, Y, speed_profile)