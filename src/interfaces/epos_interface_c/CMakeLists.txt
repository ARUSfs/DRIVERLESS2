cmake_minimum_required(VERSION 3.8)
project(epos_interface_c)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Ruta a la biblioteca dinámica (EposCmd)
set(EposCmd_LIBRARY ${PROJECT_SOURCE_DIR}/include/epos_interface_c/lib/libEposCmd.so.6.8.1.0)

# Buscar dependencias de ROS 2
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(common_msgs REQUIRED)

# Incluir directorios
include_directories(include)

# Añadir el ejecutable principal
add_executable(epos_interface_exec src/steering_handle_node.cpp)
target_include_directories(epos_interface_exec PRIVATE include)

# Vincular la biblioteca dinámica al ejecutable
target_link_libraries(epos_interface_exec ${EposCmd_LIBRARY})

# Añadir dependencias de ament
ament_target_dependencies(epos_interface_exec
  rclcpp
  std_msgs
  common_msgs
)

# Instalar el ejecutable
install(TARGETS epos_interface_exec
  DESTINATION lib/${PROJECT_NAME})

# Instalar la biblioteca dinámica
install(FILES ${EposCmd_LIBRARY}
  DESTINATION lib/ RENAME libEposCmd.so)

# Instalar archivos de configuración y lanzamiento
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

# Exportar el directorio de inclusión y la biblioteca
ament_export_include_directories(include)
ament_export_libraries(epos_interface_exec)

ament_package()



