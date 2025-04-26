std_msgs/Header header

common_msgs/PointXY[] points    # points on the trajectory
float32[] s                     # distance values of the points              
float32[] k                     # curvature values of the points
float32[] speed_profile         # speed profile of the trajectory
float32[] acc_profile           # acceleration profile of the trajectory
