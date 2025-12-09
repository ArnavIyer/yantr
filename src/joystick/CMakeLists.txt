cmake_minimum_required(VERSION 3.8)
project(joystick)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Create include directory
include_directories(include)

# Add joystick interface library
add_library(joystick_interface 
  src/joystick_interface.cpp
)

target_include_directories(joystick_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(joystick_interface rclcpp)

# Add executable
add_executable(joystick_node 
  src/main.cpp
  src/joystick_node.cpp
)

target_include_directories(joystick_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_link_libraries(joystick_node joystick_interface)

ament_target_dependencies(joystick_node
  rclcpp
  std_msgs
  sensor_msgs)

# Install targets
install(TARGETS
  joystick_interface
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  joystick_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()