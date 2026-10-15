cmake_minimum_required(VERSION 3.8)
project(nereo_controller_node)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nereo_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# Add executable
add_executable(nereo_controller_node src/nereo_controller_node.cpp)
target_include_directories(nereo_controller_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include)
# Eigen through its imported target: EIGEN3_INCLUDE_DIR is gone in Eigen 5
target_link_libraries(nereo_controller_node Eigen3::Eigen)

# Target dependencies
ament_target_dependencies(nereo_controller_node
  rclcpp
  nereo_interfaces
  sensor_msgs
  std_msgs
  Eigen3
)

# Install executable
install(TARGETS
  nereo_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/pid_tuner_gui.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install params YAML
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

# Package configuration
ament_package()