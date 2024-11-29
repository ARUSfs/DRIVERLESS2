cmake_minimum_required(VERSION 3.8)
project(skidpad_planning)


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(common_msgs REQUIRED)


include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  include/${PROJECT_NAME}
)


ament_export_include_directories(include)


add_executable(skidpad_planning_exec 
  src/skidpad_planning_node.cpp  
)


ament_target_dependencies(skidpad_planning_exec 
  rclcpp 
  std_msgs 
  geometry_msgs 
  sensor_msgs 
  PCL
  pcl_conversions
  visualization_msgs
  common_msgs
)
target_link_libraries(skidpad_planning_exec ${PCL_LIBRARIES})


install(TARGETS
  skidpad_planning_exec
  DESTINATION lib/${PROJECT_NAME})


install(DIRECTORY launch
DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

ament_package()
