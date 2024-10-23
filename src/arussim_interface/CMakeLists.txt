cmake_minimum_required(VERSION 3.8)
project(arussim_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(common_msgs REQUIRED)
find_package(arussim_msgs REQUIRED)

include_directories(
  include
)

add_executable(arussim_interface_exec src/arussim_interface_node.cpp)
ament_target_dependencies(arussim_interface_exec rclcpp 
                            common_msgs 
                            arussim_msgs)

  
install(TARGETS
  arussim_interface_exec
  DESTINATION lib/${PROJECT_NAME})

ament_package()
