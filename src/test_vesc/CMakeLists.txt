cmake_minimum_required(VERSION 3.8)
project(test_vesc)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find ROS2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Create executable
add_executable(test_drive_node
  src/test_drive_node.cpp
)

# Link dependencies using modern CMake approach
target_link_libraries(test_drive_node PUBLIC
  ${geometry_msgs_TARGETS}
  ${std_msgs_TARGETS}
  rclcpp::rclcpp
)

# Install executable
install(TARGETS test_drive_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()