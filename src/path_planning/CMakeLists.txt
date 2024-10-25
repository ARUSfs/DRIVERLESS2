cmake_minimum_required(VERSION 3.8)
project(path_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(common_msgs REQUIRED)

add_subdirectory(include/${PROJECT_NAME}/CDT CDT)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  )
  
# find_library(fade2d fade2d)

add_executable(path_planning_exec src/path_planning_node.cpp)
ament_target_dependencies(path_planning_exec rclcpp 
                            std_msgs 
                            geometry_msgs 
                            sensor_msgs 
                            visualization_msgs 
                            tf2_ros
                            tf2
                            PCL
                            pcl_conversions
                            common_msgs)

  
install(TARGETS
  path_planning_exec
  DESTINATION lib/${PROJECT_NAME})

# share folders location
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})



ament_package()
