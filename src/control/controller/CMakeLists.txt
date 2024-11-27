cmake_minimum_required(VERSION 3.8)
project(controller)

# Add compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(common_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Include directories for header files
include_directories(include)

# Add the executable for the controller node
add_executable(controller_exec src/controller_node.cpp)

# Link the executable to the required packages
ament_target_dependencies(controller_exec 
                            rclcpp 
                            std_msgs 
                            geometry_msgs
                            common_msgs)

# Install the node executable
install(TARGETS controller_exec
  DESTINATION lib/${PROJECT_NAME})

# Install configuration and launch files
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

# Mark the package for ament
ament_package()

