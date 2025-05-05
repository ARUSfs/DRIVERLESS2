# Trajectory Optimization  

The `trajectory_optimization` node is part of the DRIVERLESS2 project and is responsible for calculating an optimized trajectory for a known track to allow higher speeds. From the midpath and track limits received from `path_planning`, it calculates a minimal curvature path and its speed and acceleration profiles.

---

## Algorithm  

1. **Track width generation**: The track widths left and right allowed at each midpoint for the optimized trajectory are calculated by ensuring `kMinDist` to the track limits.
2. **Track data processing**: The midpath and track width vectors are interpolated to increase the number of points of the optimized trajectory, and the inner and outer limits for the optimized trajectory are calculated from the midpoints and track widths allowed.
3. **Trajectory optimization**: The matrices of the quadratic optimization problem are calculated, and the optimization problem is solved using `qpmad` solver.
4. **Reiteration**: Steps `1` to `3` are repeated `kNIter` times, repeating the optimization process to converge to the optimal solution (step `3` gives a good suboptimal).
5. **Generate profiles**: the speed and acceleration profiles are calculated from the trajectory's curvature and the car's physical limits.
6. **Publishing**: The trajectory message is generated and published for `controller` node to follow it.

---

## Usage  

To run the `trajectory_optimization` node, use the following command:  

```bash  
ros2 launch trajectory_optimization trajectory_optimization_launch.py  
```  

## Connections  

### Subscribed Topics  

- `/path_planning/track_limits` (`common_msgs/TrackLimits`): Receives the midpath and track limits obtained in `path_planning` node.

### Published Topics  

- `/trajectory_optimization/trajectory` (`common_msgs/Trajectory`): Publishes the optimized trajectory (optimized path, accumulated distance, curvature, speed profile and acceleration profile).

---  