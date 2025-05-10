# Path Planning Node

This ROS 2 node generates a trajectory for the ART-25D during autoX or trackdrive (only 1st lap) events. We assume the track is rules compliant: right cones are yellow and left ones are blue, track width is more than 3 meters and total length is 200-500 m. The main algorithm of the node is the SimplexTree generation of trajectories.

## Features

- Subscribes to cone detections from the SLAM node.
- Triangulates the cones and delete triangles with long edges or 3 cones of the same color.
- Generates a tree with all valid trajectories and calculates the cost of them. 
- Publishes smoothed centerline trajectory (the one with less cost) with acceleration and speed profiles.
- All parameters are configurable via YAML.

## Architecture of SimplexTree

SimplexTree is a header file containing all the methods and necesary functions to create the tree mentioned above. The structure is a recursive function that navigates only by neighbors triangles:

-  Start from a given triangle (the one closer to the car) and expand a subtree to each valid neighbor.
- Conditions to discard a neighbor are:
  - Both cones of the shared edge are of the same color.
  - The separation of the two cones is less than 2 meters.
  - Neighbor is already discarded by previous iteration of the same branch.
  - Neighbor contains a discarded vertex (it was in a previous triangle of the route).
- When a neighbor is selected as valid, the cost is calculated at that point and if is higher than the minimum cost of the calculated routes plus a constant ($\text{len-coeff} \cdot 500$) that path is truncated.
- Then, triangle index is stored and midpoint is calculated.

## Algorithm

- **Input**: Positions of detected cones (SLAM).
- **Triangulation**: External library (CDT) computes the triangulation.
- **Trajectory Generation**: From the best route of the tree, it is appended to the previous midpoints of the routes from the origin and then it is smoothed using Laplacian Smoothing and spline interpolation. Finally, acceleration and speed profiles are calculated using forwards and backwards loops.
- **Track Limits Generation**: In the begining of the second lap of the trackdrive we need to calculate the track limits in order to have more information for the MPC or other controls. Track limits is calculated using an auxiliary tree which is initialized without the passed vertices conditions of the previous routes, from the best triangles route we take the cones in order using its color or the position relative to the trajectory.

## Parameters

These are loaded via `path_planning_config.yaml`:

| Parameter | Type | Description |
|----------|------|-------------|
| `max_tri_len` | double | Max length of an edge in the triangulation|
| `len_coeff` | double | Coefficient of length in the cost function |
| `angle_coeff` | double | Coefficient of angle in the cost function |
| `route_back` | int | Number of routes to look back in buffer |
| `prev_rotue_bias` | double | Coefficient of the threshold to validate route |
| `smooth_coeff` | int | Laplacian smoothing coefficient |
| `use_buffer` | bool | Select to use buffer |
| `time_to_close` | double | Minimum time without unfinished routes to close the node |
| `stop_after_closing` | bool | Select to close the node before starting a new lap |
| `v_max` | double | Max speed for the profile creation |
| `ay_max` | double | Max acc for the profile creation |
| `ax_max` | double | Max acc for the profile creation |

## Usage

You can launch the node using the included launch file:

```bash
ros2 launch path_planning path_planning_launch.py 

```

## Connections
### Subscribed Topics
- `/slam/map` (type: `sensor_msgs/PointCloud2`): Receives global map cloud.
- `/slam/lap_count` (type: `std_msgs/Int16`): Receives lap count
- `/car_state/car_info` (type: `common_msgs/CarInfo`): Receives car information (speed, yaw, x, y, ...).
- `/trajectory_optimization/trajectory` (type: `common_msgs/Trajectory`): Receives optimized trajectory in 2nd lap.

### Published Topics
- `/path_planning/trajectory` (type: `common_msgs/Trajectory`): Publishes planning route.
- `/path_planning/triangulation` (type: `common_msgs/Triangulation`): Publishes triangulation with visualization purposes.
- `/path_planning/track_limits` (type: `common_msgs/TrackLimits`): Publishes track limits in second lap.