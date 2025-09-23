#pragma once

#include <memory>
#include <string>
#include <atomic>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float64.hpp"

#include "vesc_driver/vesc_interface.h"

namespace vesc_driver {

class VescDriverNode : public rclcpp::Node {
public:
  VescDriverNode();
  ~VescDriverNode() = default;

private:
  // Core functionality
  void timerCallback();
  void velocityCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void leftVescPacketCallback(const boost::shared_ptr<VescPacket const>& packet);
  void rightVescPacketCallback(const boost::shared_ptr<VescPacket const>& packet);
  void sendDriveCommands();
  void updateOdometry();
  void calculateWheelVelocities(double linear_vel, double angular_vel, double& left_vel, double& right_vel);

  // ROS2 interfaces
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_rpm_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_rpm_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // VESC interfaces
  std::unique_ptr<VescInterface> left_vesc_;
  std::unique_ptr<VescInterface> right_vesc_;
  
  // State variables
  std::atomic<double> target_linear_velocity_{0.0};
  std::atomic<double> target_angular_velocity_{0.0};
  std::atomic<double> t_last_command_{0.0};
  bool driver_initialized_{false};
  
  // Dual motor state
  std::atomic<double> left_rpm_{0.0};
  std::atomic<double> right_rpm_{0.0};
  
  // Robot parameters
  double wheel_track_;  // Distance between left and right wheels (m)
  double position_x_;
  double position_y_;
  double orientation_yaw_;
  rclcpp::Time last_odom_time_;
};

} // namespace vesc_driver