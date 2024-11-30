cmake_minimum_required(VERSION 3.8)
project(icp_slam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(common_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)

add_executable(icp_slam_exec src/icp_slam_node.cpp)
ament_target_dependencies(icp_slam_exec rclcpp 
                            std_msgs 
                            geometry_msgs 
                            sensor_msgs 
                            common_msgs
                            PCL
                            pcl_conversions
                            tf2_ros
                            tf2)

# Install the node executable
install(TARGETS
  icp_slam_exec
  DESTINATION lib/${PROJECT_NAME})

# Install configuration and launch files
install(DIRECTORY config launch
DESTINATION share/${PROJECT_NAME})

ament_package()
