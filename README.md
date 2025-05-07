# Acceleration Planning Node

This ROS 2 node generates a trajectory for a vehicle performing an acceleration run, using cone detections from the perception system. It fits two lines using a RANSAC-based algorithm and publishes a trajectory between them, along with speed and acceleration profiles.

---

## Features

- Subscribes to cone detections as a `PointCloud2` from a perception module.
- Uses RANSAC to detect two lines from the detected cones.
- Computes a trajectory as the midpoint between the two fitted lines.
- Calculates a speed and acceleration profile along the trajectory.
- Publishes a trajectory message with spatial and temporal data.
- All parameters are configurable via YAML.

---

## Architecture

The node performs the following steps:

1. Waits for cone detections (as point cloud).
2. Applies RANSAC to fit two lines to the cones.
3. Validates and averages the line fits.
4. Computes the centerline trajectory.
5. Generates speed and acceleration profiles based on physical constraints.
6. Publishes the complete trajectory.

---

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

---

## Parameters

These are loaded via `acc_params.yaml`:

| Parameter              | Type    | Description                                       |
|------------------------|---------|---------------------------------------------------|
| `perception_topic`     | string  | Topic for cone detections (PointCloud2)           |
| `trajectory_topic`     | string  | Topic to publish the generated trajectory         |
| `target_speed`         | double  | Maximum speed to reach during acceleration        |
| `max_acc`              | double  | Maximum forward acceleration                      |
| `max_dec`              | double  | Maximum deceleration (used in second half)        |
| `track_length`         | double  | Length of the acceleration track (in meters)      |
| `debug`                | bool    | Enables logging for debugging                     |

---

## Launch

You can launch the node using the included launch file:

```bash
ros2 launch common_meta sim_acceleration_launch.py
