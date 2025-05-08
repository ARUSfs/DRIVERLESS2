# Skidpad Planning Node

This ROS 2 node generates a trajectory for a vehicle navigating a skidpad track (a 8-shape or double circle layout) using cone detections from perception. It detects circle centers via RANSAC and produces a smooth trajectory around the centers.

## Features

- Subscribes to cone detections from the SLAM node.
- Detects left and right circles using RANSAC.
- Publishes skidpad centerline trajectory.
- All parameters are configurable via YAML.

## Architecture

The node performs the following steps:

1. Waits for cone detections.
2. Applies RANSAC to detect a circle.
3. Computes the center points of the left and right circles.
4. Transforms and generates a centerline trajectory based on the detected centers.
5. Publishes the trajectory to a topic for downstream controllers.

## Algorithm

- **Input**: Positions of detected cones.
- **RANSAC Circle Detection**:
  - Fit both circles using a RANSAC-based method to handle noise and outliers.
  - Extract circle center and radius.
- **Trajectory Generation**:
  - Pre-computed trajectory is transformed to fit the detected centers.
  - The trajectory is aligned and smoothed for consistent vehicle control.

## Parameters

These are loaded via `skidpad_params.yaml`:

| Parameter | Type | Description |
|----------|------|-------------|
| `num_circle_points` | int | Number of points per circle arc |
| `circle_radius` | double | Radius of the skidpad circles |
| `distance_between_centers` | double | Distance between the two circles |
| `ransac_iterations` | int | Max RANSAC iterations |
| `ransac_threshold` | double | Distance threshold for RANSAC inliers |
| `min_inliers` | int | Minimum inliers for a valid circle |


## Usage

You can launch the node using the included launch file:

```bash
ros2 launch skidpad_planning skidpad_planning_launch.py 

```

## Connections
### Subscribed Topics
- `/slam/map` (type: `sensor_msgs/PointCloud2`): Receives global map cloud.

### Published Topics
- `/skidpad_planning/trajectory` (type: `common_msgs/Trajectory`): Publishes skidpad route.
