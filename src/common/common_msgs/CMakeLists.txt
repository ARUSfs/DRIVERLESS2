cmake_minimum_required(VERSION 3.8)
project(common_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED) # for custom messages

# generate custom messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/State.msg"
  "msg/CarInfo.msg"
  "msg/PointXY.msg"
  "msg/Trajectory.msg"
  "msg/Simplex.msg"
  "msg/Triangulation.msg"
  "msg/Cmd.msg"
  "msg/FourWheelDrive.msg"
  "msg/Cmd4WD.msg"
  "msg/TrackLimits.msg"
  DEPENDENCIES std_msgs
)


ament_package()
