cmake_minimum_required(VERSION 3.8)
project(can_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(
  include
)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(common_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

add_executable(can_interface_exec src/can_interface_node.cpp)
ament_target_dependencies(can_interface_exec rclcpp 
                            std_msgs 
                            common_msgs)

install(TARGETS
  can_interface_exec
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/include/can_interface/can24.csv
  ${CMAKE_CURRENT_SOURCE_DIR}/include/can_interface/can_aux24.csv
  DESTINATION share/${PROJECT_NAME})

ament_package()
