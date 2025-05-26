# Acceleration Planning
The `acc_planning` node is part of the DRIVERLESS2 project and is responsible of generating the trajectory, speed, and acceleration profiles for the acceleration event.

## Algorithm

### Input
Positions of detected cones (`PointCloud2`).

### Line Fitting (RANSAC)
- Iteratively sample pairs of cones.
- Fit a line and count inliers under a distance threshold.
- Select the best line with highest score.
- Repeat for a second line using remaining cones.

### Trajectory Generation
- Compute the average of the two lines to find a central trajectory.
- Apply smoothing over time using a history buffer.
- Generate a set of waypoints along the trajectory.

### Speed/Acceleration Profiles
- Apply a constant acceleration model up to target speed.
- Decelerate after reaching track midpoint.
- Compute corresponding acceleration profile analytically.


## Usage
To run the `acc_planning` node, use the following command:
```bash
ros2 launch acc_planning acc_planning_launch.py
```


## Connections
### Subscribed Topics
- `/slam/map` (type: `sensor_msgs/PointCloud2`): Receives global map cloud.

### Published Topics
- `/acc_planning/trajectory` (type: `common_msgs/Trajectory`): Publishes acceleration route.
