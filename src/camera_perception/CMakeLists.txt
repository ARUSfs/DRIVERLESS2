cmake_minimum_required(VERSION 3.8)
project(camera_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(common_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

include_directories(
  include
  ${Open_CV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(camera_perception_exec src/camera_perception_node.cpp)

ament_target_dependencies(camera_perception_exec rclcpp 
                            std_msgs 
                            geometry_msgs
                            sensor_msgs 
                            PCL
                            pcl_conversions
                            common_msgs
                            OpenCV
                            cv_bridge)

  
install(TARGETS
  camera_perception_exec
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

ament_package()

