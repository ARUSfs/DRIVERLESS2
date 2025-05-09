# Controller node

The `controller` node is part of the DRIVERLESS2 project and is responsible for computing the decoupled speed and steering control.

## Algorithm
- Vehicle status and computed trajectory from planning are continuously updated.
- Speed command is computed with a PID + feed forward control at a fixed frequency.
- Steering command is computed with pure pursuit or MPC control at a fixed frequency.
- When optimized trajectory is received, the control type and parameters may change (see config).


## Usage
To run the `controller` node, use the following command:
```bash
ros2 launch controller controller_launch.py
```

## Connections
### Subscribed Topics
- `/car_state/state` (type: `common_msgs/State`): Receives speed data for control.
- `/can_interface/run_check` (type: `std_msgs/Bool`): Receives whether the vehicle is allowed to run or not.
- `/path_planning/trajectory` (type: `common_msgs/Trajectory`): Receives the computed central trajectory.
- `/trajectory_optimization/trajectory` (type: `common_msgs/Trajectory`): Receives the total optimized trajectory.
- `/can_interface/braking_procedure` (type: `std_msgs/Bool`): Received when the vehicle must start braking procedure.

### Published Topics
- `/controller/cmd` (type: `common_msgs/Cmd`): Publishes desired command values.
- `/controller/pursuit_point` (type: `common_msgs/PointXY`): Publishes pursuit point location.
- `/controller/target_speed` (type: `std_msgs/Float32`): Publishes current tartget speed.
