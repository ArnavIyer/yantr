cmake_minimum_required(VERSION 3.8)
project(vesc_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find ROS2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# Find system dependencies  
find_package(Boost REQUIRED COMPONENTS system)

# Include directories  
include_directories(
  include
  src  # Temporary for current header locations
)

# Create executables
add_executable(vesc_driver_node
  src/vesc_driver_node.cpp
  src/vesc_interface.cpp
  src/vesc_packet.cpp
  src/vesc_packet_factory.cpp
  src/serial.cc
)

# Single VESC driver node
add_executable(single_vesc_driver_node
  src/single_vesc_driver_node.cpp
  src/vesc_interface.cpp
  src/vesc_packet.cpp
  src/vesc_packet_factory.cpp
  src/serial.cc
)

# Skid-steer controller node  
add_executable(skid_steer_controller_node
  src/skid_steer_controller_node.cpp
)

# Link dependencies
target_link_libraries(vesc_driver_node
  rclcpp::rclcpp
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${Boost_LIBRARIES}
)

target_link_libraries(single_vesc_driver_node
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
  ${Boost_LIBRARIES}
)

target_link_libraries(skid_steer_controller_node
  rclcpp::rclcpp
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${std_msgs_TARGETS}
)

# Install executables
install(TARGETS 
  vesc_driver_node
  single_vesc_driver_node
  skid_steer_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/
  DESTINATION include/
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
