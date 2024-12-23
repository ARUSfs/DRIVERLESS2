cmake_minimum_required(VERSION 3.8)
project(epos_interface_c)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Buscar dependencias de ROS 2
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(common_msgs REQUIRED)

# Incluir directorios
include_directories(include)

# Añadir el ejecutable principal
add_executable(epos_interface_exec src/epos_interface_node.cpp)

# Añadir dependencias de ament
ament_target_dependencies(epos_interface_exec
  rclcpp
  std_msgs
  common_msgs
)

# Instalar el ejecutable
install(TARGETS epos_interface_exec
  DESTINATION lib/${PROJECT_NAME})

# Instalar archivos de configuración y lanzamiento
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

ament_package()



