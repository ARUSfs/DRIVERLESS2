# EPOS interface

The `epos_interface` node is part of the DRIVERLESS2 project and is responsible for managing the commands to the EPOS4 steering motor controller.

## Algorithm
- EPOS4 controller is initalized in Position Profile mode over CAN.
- Initial steer position is obtained from an extensometer.
- Steering commands are transformed into increments and sent to EPOS4 controller.

## Usage
To run the `epos_interface` node, use the following command:
```bash
ros2 launch epos_interface epos_interface_launch.py
```

## Connections
### Subscribed Topics
- `/controller/cmd` (type: `common_msgs/Cmd`): Receives commands from controller node.
- `/car_state/steer_check` (type: `std_msgs/Bool`): Whether steering is allowed or not.
- `/can_interface/extensometer` (type: `std_msgs/Float32`): Receives current AS status.

### Published Topics
- `/epos_interface/epos_info` (type: `std_msgs/Float32`): Publishes several EPOS4 information.
