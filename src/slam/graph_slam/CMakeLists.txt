cmake_minimum_required(VERSION 3.8)
project(graph_slam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(common_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(g2o REQUIRED)
find_package(fmt REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${g2o_INCLUDE_DIRS}
)

add_executable(graph_slam_exec src/graph_slam_node.cpp)
ament_target_dependencies(graph_slam_exec std_msgs 
                                      sensor_msgs     
                                      geometry_msgs 
                                      visualization_msgs
                                      common_msgs
                                      rclcpp 
                                      PCL
                                      pcl_conversions
                                      tf2_ros
                                      tf2)

target_link_libraries(graph_slam_exec 
                      g2o_core 
                      g2o_stuff 
                      g2o_types_slam2d 
                      g2o_csparse_extension
                      g2o_solver_csparse
                      fmt::fmt)

install(TARGETS
  graph_slam_exec
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config
DESTINATION share/${PROJECT_NAME})

ament_package()
