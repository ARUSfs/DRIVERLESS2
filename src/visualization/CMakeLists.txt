cmake_minimum_required(VERSION 3.8)
project(visualization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

include_directories(
  include
)

add_executable(visualization_exec src/visualization_node.cpp)
ament_target_dependencies(visualization_exec rclcpp 
                            std_msgs 
                            geometry_msgs 
                            sensor_msgs 
                            visualization_msgs)

  
install(TARGETS
  visualization_exec
  DESTINATION lib/${PROJECT_NAME})

ament_package()
