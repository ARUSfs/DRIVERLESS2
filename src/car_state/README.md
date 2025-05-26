# Car State Node

The `car_state` node is part of the **DRIVERLESS2** project and is responsible for estimating and publishing the dynamic state of the vehicle in real time. It fuses sensor data using a custom `SpeedEstimator` that combines kinematic and dynamic bicycle models via Kalman filters, while also performing safety and plausibility checks.

---

## Algorithm

- Subscribes to various vehicle sensors: IMU, wheel speeds, steering, inverter speed, etc.
- Estimates longitudinal and lateral velocity (`vx`, `vy`) using a `SpeedEstimator`.
- The estimator blends two Kalman filters (kinematic and dynamic) based on current speed.
- Performs plausibility checks on sensor inputs (timeouts, physical limits).
- Publishes:
  - Estimated vehicle state.
  - Extended diagnostics and system health information.
- Outputs flags for safe operation and mission progression.

---

## Usage

To run the `car_state` node, use the following command:

```bash
ros2 launch car_state car_state_launch.py
```

---

## Parameters

The node loads its configuration from a YAML file.

### General Parameters
- `simulation` (bool): Enables simulation mode.
- `mission` (string): Mission type, e.g. `"autocross"`, `"trackdrive"`.
- `trackdrive_laps` (int): Number of laps before mission ends.

### Safety and Estimation Parameters
- `safe_mode` (bool): Enables/disables plausibility checks.
- `use_wheelspeeds` (bool): Use wheelspeeds for speed estimation when inverter speed is low.
- `dt_threshold_*` (double): Max allowed time between sensor updates.
- `max_ax`, `max_ay`, `max_r`, `max_vx` (double): Physical limits.
- `error_weight_*` (double): Penalty weights for plausibility violations.
- `max_plausability_error` (double): Max error before the system is considered unsafe.

### Topic Configuration
- All topic names (e.g. IMU, wheelspeeds) are configurable.

---

## Connections

### Subscribed Topics

- `/controller/cmd` (`common_msgs/Cmd`): Steering command.
- `/controller/target_speed` (`std_msgs/Float32`): Target speed from controller.
- `/slam/lap_count` (`std_msgs/Int16`): Current lap number.
- `/can_interface/AMI` (`std_msgs/Float32`): AMI status.
- `/can_interface/AS_status` (`std_msgs/Float32`): Autonomous system state.
- `/can_interface/inv_speed` (`std_msgs/Float32`): Inverter-measured speed.
- `/can_interface/extensometer` (`std_msgs/Float32`): Steering position.
- `/can_interface/fl_wheel_speed` (`std_msgs/Float32`): Front-left wheel speed.
- `/can_interface/fr_wheel_speed` (`std_msgs/Float32`): Front-right wheel speed.
- `/can_interface/rl_wheel_speed` (`std_msgs/Float32`): Rear-left wheel speed.
- `/can_interface/rr_wheel_speed` (`std_msgs/Float32`): Rear-right wheel speed.
- `/can_interface/IMU/ax` (`std_msgs/Float32`): Longitudinal acceleration.
- `/can_interface/IMU/ay` (`std_msgs/Float32`): Lateral acceleration.
- `/can_interface/IMU/r` (`std_msgs/Float32`): Yaw rate.
- `/perception/map` (`sensor_msgs/PointCloud2`): Detected cones (perception).
- `/slam/map` (`sensor_msgs/PointCloud2`): Mapped cones (SLAM).
- `/arussim_interface/arussim_ground_truth` (`common_msgs/State`): Ground-truth position (if simulation enabled).

### Published Topics

- `/car_state/state` (`common_msgs/State`): Estimated vehicle state.
- `/car_state/car_info` (`common_msgs/CarInfo`): Extended vehicle and status data.
- `/car_state/run_check` (`std_msgs/Bool`): System is ready and running.
- `/car_state/steer_check` (`std_msgs/Bool`): Steering is permitted.
- `/car_state/braking_procedure` (`std_msgs/Bool`): Signal for end-of-mission braking.

---

## Internals

### Speed Estimator

- Combines kinematic and dynamic bicycle models with Kalman filters.
- Smoothly blends both filters based on speed (kinematic at low speed, dynamic at high).
- Returns robust `vx`, `vy` estimation used in control and diagnostics.

### Plausibility System

- Applies time and value thresholds to incoming sensor data.
- Accumulates a plausibility score; if it exceeds a threshold, the system is flagged unsafe.
- The plausibility score decays over time to allow recovery.

---

## TF Integration

- Uses TF2 to get the position and orientation from the transform `"arussim/world"` → `"slam/vehicle"` when available.