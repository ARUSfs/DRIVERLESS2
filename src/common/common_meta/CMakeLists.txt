cmake_minimum_required(VERSION 3.8)
project(common_meta)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(common_msgs REQUIRED)

find_package(rcl_interfaces REQUIRED)


add_executable(param_dumper_exec param_dumper.cpp)
ament_target_dependencies(param_dumper_exec rclcpp 
                            std_msgs
                            rcl_interfaces 
                            common_msgs)


install(TARGETS
  param_dumper_exec
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY .
  DESTINATION share/${PROJECT_NAME})

ament_package()
