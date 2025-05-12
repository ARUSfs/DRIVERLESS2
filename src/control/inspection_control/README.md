# Inspection Control

The `inspection_control` node is part of the DRIVERLESS2 project and is responsible for managing the Inspection Mission defined as defined in FSRules.

## Algorithm
- Acceleration command is obtained by a PID speed control.
- Steering command is obtained as a sinusoidal wave with a fixed amplitude and frequency.
- It sends the command to `can_interface` node until the mission duration is over.


## Usage
To run the `inspection_control` node, use the following command:
```bash
ros2 launch inspection_control inspection_control_launch.py
```

## Connections
### Subscribed Topics
- `/car_state/state` (type: `common_msgs/State`): Receives speed data for control.
- `/can_interface/as_status` (type: `std_msgs/Float32`): Receives current AS status.

### Published Topics
- `/controller/cmd` (type: `common_msgs/Cmd`): Publishes desired command values.
- `/can_interface/as_status` (type: `std_msgs/Float32`): Publishes Finished status when mission is over.
