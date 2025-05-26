# Graph SLAM

The `graph_slam` node is part of the DRIVERLESS2 project and is responsible for the continuous localization and mapping of the vehicle using g2o graph optimization.

## Algorithm
- Perception data is collected into new Landmark vertices and Pose-Landmark edges are created. A Data Association algorithm is used to fit observations in the map and detect new observations.
- Odometry data is collected into new Pose vertices and Pose-Pose edges are created. Motion of the vehicle is estimated using a bicycle model.
- Periodically, vertices and edges are loaded into the g2o graph optimizer and the optimization is done.
- Global map and vehicle pose are updated after optimization.

## Usage
To run the `graph_slam` node, use the following command:
```bash
ros2 launch graph_slam graph_slam_launch.py
```

## Connections
### Subscribed Topics
- `/perception/map` (type: `sensor_msgs/PointCloud2`): Receives actual cones perceptions.
- `/car_state/state` (type: `common_msgs/State`): Receives vehicle status data.

### Published Topics
- `/slam/map` (type: `sensor_msgs/PointCloud2`): Publishes global map cloud.
- `/slam/final_map` (type: `sensor_msgs/PointCloud2`): Publishes the final global map cloud.
- `/slam/lap_count` (type: `std_msgs/Int16`): Publishes the number of laps finished.
