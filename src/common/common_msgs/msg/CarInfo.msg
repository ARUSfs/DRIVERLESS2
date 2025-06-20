std_msgs/Header header

int16 as_status  # 1: Off, 2: Ready, 3: Driving, 4: Emergency, 5: Finished
int16 ami        # 1: Acceleration, 2: Skidpad, 3: Trackdrive, 4: BrakeTest, 5: Inspection, 6: Autocross
int16 ebs_status    # 1: deactivated, 2: checkup_passed, 3: activated
int16 ebs_redundancy_status # 1: deactivated, 2: engaged, 3: checkup_passed
float32 x      # m
float32 y      # m
float32 yaw    # radians
float32 vx     # m/s
float32 vy     # m/s
float32 target_speed    # m/s
float32 r      # radians/s
float32 ax     # m/s²
float32 ay     # m/s²
float32 delta  # radians   
float32 target_delta    # radians
int16 steering_state
float32 brake_hydr_pressure    # bar
int16 lap_count
int16 cones_count_actual
int16 cones_count_all