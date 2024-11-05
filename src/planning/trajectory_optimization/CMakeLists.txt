cmake_minimum_required(VERSION 3.8)
project(trajectory_optimization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(common_msgs REQUIRED)

include_directories(
  include
)

add_executable(trajectory_optimization_exec src/trajectory_optimization_node.cpp)
ament_target_dependencies(trajectory_optimization_exec rclcpp
                            std_msgs
                            common_msgs
)

install(TARGETS
  trajectory_optimization_exec
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
