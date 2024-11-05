cmake_minimum_required(VERSION 3.8)
project(acc_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(visualization_msgs REQUIRED)

find_package(common_msgs REQUIRED)


include_directories(
  include
 
  )
  
  add_executable(acc_planning_exec src/acc_planning_node.cpp)
  ament_target_dependencies(acc_planning_exec rclcpp 
                              std_msgs 
                              geometry_msgs 
                              sensor_msgs 
                              visualization_msgs 
                              common_msgs)
  
    
  install(TARGETS
    acc_planning_exec
    DESTINATION lib/${PROJECT_NAME})
  
  # share folders location
  install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME})
  install(DIRECTORY config
    DESTINATION share/${PROJECT_NAME})
  

ament_package()
