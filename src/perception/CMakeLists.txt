cmake_minimum_required(VERSION 3.8)
project(perception)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -funroll-loops -ffast-math")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters segmentation search)
find_package(pcl_conversions REQUIRED)
find_package(common_msgs REQUIRED)
find_package(OpenMP REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}  
)

add_executable(perception_exec src/perception_node.cpp)

ament_target_dependencies(perception_exec 
  rclcpp 
  std_msgs 
  geometry_msgs
  sensor_msgs 
  PCL 
  pcl_conversions
  common_msgs
)

target_link_libraries(perception_exec 
  ${PCL_LIBRARIES}
  OpenMP::OpenMP_CXX
)

install(TARGETS
  perception_exec
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

ament_package()