# Perception Node  

The `perception_node` is part of the DRIVERLESS2 project and is responsible for processing LiDAR data to detect and localize cones on the track in real time. It serves as a key component of the perception pipeline, providing a filtered, clustered, and scored point cloud that represents the surrounding cones, which is later used for SLAM and path planning.  

---

## Algorithm  

1. **Field-of-View Cropping**: Raw LiDAR data is cropped according to predefined field-of-view limits.  
2. **Ground Filtering**: Ground points are separated from obstacles using a grid-based approach.  
3. **Clustering**: Remaining points are segmented using Euclidean clustering to identify potential cones.  
4. **Scoring**: Each cluster is scored and those meeting a scoring threshold are retained as cones.  
5. **Color Estimation**: For close-range cones, a color estimation module classifies their color based on intensity.  
6. **Motion Correction**: The final cone map is motion-corrected using vehicle velocity and yaw rate.  
7. **Publishing**: The resulting point cloud is published for further use in mapping and localization.  

---

## Usage  

To run the perception node, use the following command:  

```bash  
ros2 launch perception perception_launch.py  
```  

## Connections  

### Subscribed Topics  

- `/rslidar_points` (`sensor_msgs/PointCloud2`): Receives raw LiDAR point clouds.  
- `/car_state/state` (`common_msgs/State`): Receives the current velocity and yaw rate of the vehicle.  

### Published Topics  

- `/perception/filtered_cloud` (`sensor_msgs/PointCloud2`): Publishes the ground-filtered point cloud (debug mode only).  
- `/perception/clusters` (`sensor_msgs/PointCloud2`): Publishes clustered points potentially corresponding to cones (debug mode only).  
- `/perception/map` (`sensor_msgs/PointCloud2`): Publishes the final set of detected cones (positions and colors).  

---  