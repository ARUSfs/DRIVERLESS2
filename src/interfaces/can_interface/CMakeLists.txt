cmake_minimum_required(VERSION 3.8)
project(can_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(
  include
  ${CANLIB_INCLUDE_DIR}
)

find_library(CANLIB_LIBRARY NAMES canlib HINTS /usr/lib)
find_path(CANLIB_INCLUDE_DIR NAMES canlib.h HINTS /usr/include)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(can_interface_exec src/can_interface_node.cpp)
ament_target_dependencies(can_interface_exec rclcpp)

target_link_libraries(can_interface_exec ${CANLIB_LIBRARY})

install(TARGETS
  can_interface_exec
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()