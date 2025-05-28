cmake_minimum_required(VERSION 3.8)
project(perception_acc)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -funroll-loops -ffast-math")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(common_msgs REQUIRED)
find_package(OpenMP REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

### To include ROBIN& KISS-Matcher ###
find_package(robin REQUIRED)
find_package(kiss_matcher REQUIRED)
# find_package(teaserpp REQUIRED)

# The best way to set proper compiler settings for using OpenMP in all platforms
include(FindOpenMP)
if (OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
else (OPENMP_FOUND)
    message("ERROR: OpenMP could not be found.")
endif (OPENMP_FOUND)  


include_directories(
  include
  include/perception_acc
  ${PCL_INCLUDE_DIRS}  
)

add_executable(perception_acc_exec src/perception_acc_node.cpp)

ament_target_dependencies(perception_acc_exec 
  rclcpp 
  std_msgs 
  geometry_msgs
  sensor_msgs 
  PCL 
  pcl_conversions
  common_msgs
  pcl_ros
  tf2_ros
  tf2
  # TBB::tbb
)

target_link_libraries(perception_acc_exec
  ${PCL_LIBRARIES}
  OpenMP::OpenMP_CXX
  kiss_matcher::kiss_matcher_core
  robin::robin
)

install(TARGETS
  perception_acc_exec
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

ament_package()