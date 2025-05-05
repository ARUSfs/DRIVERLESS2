# Skidpad Planning Node

This ROS 2 node generates a trajectory for a vehicle navigating a skidpad track (a figure-8 or double circle layout) using cone detections from perception. It detects circle centers via RANSAC and produces a smooth trajectory around the centers.

## Features

- Subscribes to cone detections from a SLAM module.
- Detects left and right circles using RANSAC.
- Publishes a centerline trajectory as a `nav_msgs::Path`.
- All parameters are configurable via YAML.

## Architecture

The node performs the following steps:

1. Waits for cone detections (left and right cones).
2. Applies RANSAC to each cone set to detect a circle.
3. Computes the center points of the left and right circles.
4. Transforms and generates a centerline trajectory based on the detected centers.
5. Publishes the trajectory to a topic for downstream controllers.

## Algorithm

- **Input**: Positions of detected left and right cones.
- **RANSAC Circle Detection**:
  - For each cone set (left/right), fit a circle using a RANSAC-based method to handle noise and outliers.
  - Extract circle center and radius.
- **Trajectory Generation**:
  - Calculate the midpoint between the detected circles.
  - Generate a trajectory as a series of waypoints forming a loop around the midpoints.
  - The trajectory is aligned and smoothed for consistent vehicle control.

## Parameters

These are loaded via `skidpad_params.yaml`:

| Parameter | Type | Description |
|----------|------|-------------|
| `frame_id` | string | Coordinate frame for the output path |
| `num_circle_points` | int | Number of points per circle arc |
| `circle_radius` | double | Radius of the skidpad circles |
| `distance_between_centers` | double | Distance between the two circles |
| `ransac_iterations` | int | Max RANSAC iterations |
| `ransac_threshold` | double | Distance threshold for RANSAC inliers |
| `min_inliers` | int | Minimum inliers for a valid circle |
| `enable_viz` | bool | Whether to publish visualization markers |

## Launch

You can launch the node using the included launch file:

```bash
ros2 launch common_meta sim_skidpad_launch.py 

```

## 📞 Contact

For questions or contributions, please reach out to the ARUS Driverless software team.
