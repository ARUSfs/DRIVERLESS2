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
find_package(arussim_msgs REQUIRED)

# Include directories for header files
include_directories(include)

# Add the executable for the controller node
add_executable(controller src/controller.cpp)

# Link the executable to the required packages
ament_target_dependencies(controller 
                            rclcpp 
                            std_msgs 
                            common_msgs
                            arussim_msgs
                          )

# Install the node executable
install(TARGETS controller
  DESTINATION lib/${PROJECT_NAME})

# Install configuration and launch files
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

# Mark the package for ament
ament_package()

