cmake_minimum_required(VERSION 3.8)
project(epos_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(
  include
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(common_msgs REQUIRED)

install(
  PROGRAMS
  src/steering_handle.py  
  src/eposhandle.py
  DESTINATION lib/${PROJECT_NAME}
)

# share folders location
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include
  DESTINATION share/${PROJECT_NAME})

ament_package()

