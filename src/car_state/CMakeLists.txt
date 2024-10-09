cmake_minimum_required(VERSION 3.8)
project(car_state)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Incluir el directorio de encabezados
include_directories(include)

# Añadir el ejecutable
add_executable(car_state_node src/car_state.cpp)

# Establecer dependencias del ejecutable
ament_target_dependencies(car_state_node rclcpp std_msgs)

# Instalar el ejecutable
install(TARGETS
  car_state_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
