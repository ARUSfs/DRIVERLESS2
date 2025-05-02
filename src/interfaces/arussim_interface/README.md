# ARUSSim Interface

The `arussim_interface` node is part of the DRIVERLESS2 project and is responsible for translating custom messages between the simulator and the pipeline.


## Usage
To run the `arussim_interface` node, use the following command:
```bash
ros2 launch arussim_interface arussim_interface_launch.py
```

## Connections
### Subscribed Topics
- `/controller/cmd` (type: `common_msgs/Cmd`): Receives command pipeline data.
- `/arussim/cmd4wd` (type: `common_msgs/Cmd4WD`): Receives 4WD command pipeline data.
- `/arussim/wheel_speeds` (type: `arussim_msgs/FourWheelDrive`): Receives each wheel speed from ARUSSim.
- `/arussim/fixed_trajectory` (type: `arussim_msgs/Trajectory`): Receives central trajectory of the circuit from ARUSSim.
- `/arussim/state` (type: `arussim_msgs/State`): Receives ground truth of the car from ARUSSim.

### Published Topics
- `/arussim/cmd` (type: `arussim_msgs/Cmd`): Publishes command pipeline data for ARUSSim.
- `/arussim/cmd4wd` (type: `arussim_msgs/Cmd4WD`): Publishes 4WD command pipeline data for ARUSSim.
- `/arussim_interface/wheel_speeds` (type: `common_msgs/FourWheelDrive`): Publishes each wheel speed from ARUSSim.
- `/arussim_interface/fixed_trajectory` (type: `common_msgs/Trajectory`): Publishes central trajectory of the circuit from ARUSSim.
- `/arussim_interface/arussim_ground_truth` (type: `common_msgs/State`): Publishes ground truth of the car from ARUSSim.