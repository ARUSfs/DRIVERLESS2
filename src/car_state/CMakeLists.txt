cmake_minimum_required(VERSION 3.8)
project(car_state)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(common_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)


# Incluir el directorio de encabezados
include_directories(include)

# Añadir el ejecutable
add_executable(car_state_exec src/car_state.cpp)

# Establecer dependencias del ejecutable
ament_target_dependencies(car_state_exec 
                rclcpp 
                std_msgs 
                sensor_msgs 
                common_msgs
                tf2
                tf2_ros)

target_link_libraries(car_state_exec Eigen3::Eigen)

# Instalar el ejecutable
install(TARGETS
car_state_exec
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
