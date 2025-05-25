# CAN Interface

The `can_interface` node is part of the **DRIVERLESS2** project and acts as the gateway between the vehicle’s CAN bus and the ROS 2 ecosystem, translating ROS commands into CAN frames and vice-versa.

## Algorithm
- **ROS → CAN**  
  - Commands from `/controller/cmd` are packed into inverter frames (0x201).  
  - Periodic frames: heart-beat 0x140 (10 Hz) and data-logger 0x500–0x502.  
- **CAN → ROS**  
  - Incoming frames are matched with two CSV look-ups (`can_aux.csv`, `can.csv`).  
  - Sub-ID filtering and dynamic-ID construction pick the parsing rule.  
  - Bytes are extracted, sign-extended (if needed) and transformed:  
    \[
      value = (raw^{Power} \times Scale) + Offset
    \]  
  - The result is published on the ROS topic named in the **Publisher** column of `can.csv`.  
- **Safety logic**  
  - Driving frames are only sent while `/car_state/run_check` is `true`.  
  - `/car_state/car_info` updates internal status and triggers emergency frame 0x202 if `as_status == 4`.

## Usage
```bash
ros2 launch can_interface can_interface_launch.py